// Palette 2 MMU support, Firmware 9.0.9 and newer supported only!
// (port of klippy/extras/palette2.py)
//
// Copyright (C) 2021 Clifford Roche <clifford.roche@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::pause_resume::PauseResume;
use crate::extras::virtual_sdcard::VirtualSD;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_float_repr, py_int, py_int_base16, py_str_repr, py_strip};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::serialhdl::Uart;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::rc::{Rc, Weak};

const COMMAND_HEARTBEAT: &str = "O99";
const COMMAND_CUT: &str = "O10 D5";
const COMMAND_CLEAR: [&str; 6] = ["O10 D5", "O10 D0 D0 D0 DFFE1", "O10 D1 D0 D0 DFFE1", "O10 D2 D0 D0 DFFE1", "O10 D3 D0 D0 DFFE1", "O10 D4 D0 D0 D0069"];
const COMMAND_FILENAME: &str = "O51";
const COMMAND_FILENAMES_DONE: &str = "O52";
const COMMAND_FIRMWARE: &str = "O50";
const COMMAND_PING: &str = "O31";
const COMMAND_SMART_LOAD_STOP: &str = "O102 D1";

const HEARTBEAT_SEND: f64 = 5.0;
const HEARTBEAT_TIMEOUT: f64 = (HEARTBEAT_SEND * 2.0) + 1.0;
const SETUP_TIMEOUT: f64 = 10.0;

const SERIAL_TIMER: f64 = 0.1;
const AUTOLOAD_TIMER: f64 = 5.0;

const INFO_NOT_CONNECTED: &str = "Palette 2 is not connected, connect first";

const CMD_CONNECT_HELP: &str = "Connect to the Palette 2";
const CMD_DISCONNECT_HELP: &str = "Disconnect from the Palette 2";
const CMD_CLEAR_HELP: &str = "Clear the input and output of the Palette 2";
const CMD_CUT_HELP: &str = "Cut the outgoing filament";
const CMD_SMART_LOAD_HELP: &str = "Automatically load filament through the extruder";
const CMD_O1_HELP: &str = "Initialize the print, and check connection with the Palette 2";
const CMD_O9_HELP: &str = "Reset print information";

/// python's AttributeError of reactor.unregister_timer(None)
fn no_timer() -> KError {
    KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'waketime' and no __dict__ for setting new attributes".into())
}

fn index_error(what: &str) -> KError {
    KError::Py("IndexError".into(), format!("{} index out of range", what))
}

fn value_error(base: u32, s: &str) -> KError {
    KError::Py("ValueError".into(), format!("invalid literal for int() with base {}: {}", base, py_str_repr(s)))
}

/// s[n:] of a python str
fn tail(s: &str, n: usize) -> &str {
    s.char_indices().nth(n).map(|(i, _)| &s[i..]).unwrap_or("")
}

/// the state reset by _reset()
#[derive(Default)]
struct Omega {
    files: Vec<String>,
    is_setup_complete: bool,
    is_splicing: bool,
    is_loading: bool,
    remaining_load_length: Option<i64>,
    algorithms: Vec<String>,
    algorithms_counter: usize,
    splices: Vec<(i64, String)>,
    splices_counter: usize,
    /// {"number": n, "percent": p}
    pings: Vec<(i64, f64)>,
    pongs: Vec<(i64, f64)>,
    current_ping: Option<String>,
    header: [Option<String>; 9],
    header_counter: usize,
    last_command: String,
}

pub struct Palette2 {
    me: Weak<Palette2>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    virtual_sdcard: Rc<VirtualSD>,
    pause_resume: Rc<PauseResume>,
    gcode: Rc<GCodeDispatch>,
    serial: RefCell<Option<Uart>>,
    serial_port: String,
    baud: i64,
    feedrate_splice: f64,
    feedrate_normal: f64,
    auto_load_speed: i64,
    auto_cancel_variation: Option<f64>,
    st: RefCell<Omega>,
    omega_drives: RefCell<Vec<String>>,
    read_timer: RefCell<Option<Timer>>,
    read_buffer: RefCell<String>,
    read_queue: RefCell<VecDeque<String>>,
    write_timer: RefCell<Option<Timer>>,
    write_queue: RefCell<VecDeque<Option<String>>>,
    heartbeat_timer: RefCell<Option<Timer>>,
    heartbeat: Cell<Option<f64>>,
    signal_disconnect: Cell<bool>,
    is_printing: Cell<bool>,
    smart_load_timer: RefCell<Option<Timer>>,
}

type P2Cmd = fn(&Palette2, &GCodeCommand) -> KResult<()>;

impl Palette2 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let virtual_sdcard = match printer.load_object(config, "virtual_sdcard", true) {
            Err(e) if matches!(e.base(), KError::Config(_)) => {
                return Err(KError::config("Palette 2 requires [virtual_sdcard] to work, please add it to your config!").with_context(e));
            }
            r => r?.and_then(downcast_rc::<VirtualSD>).ok_or_else(|| KError::Other("virtual_sdcard has an unexpected type".into()))?,
        };
        let pause_resume = match printer.load_object(config, "pause_resume", true) {
            Err(e) if matches!(e.base(), KError::Config(_)) => {
                return Err(KError::config("Palette 2 requires [pause_resume] to work, please add it to your config!").with_context(e));
            }
            r => r?.and_then(downcast_rc::<PauseResume>).ok_or_else(|| KError::Other("pause_resume has an unexpected type".into()))?,
        };
        printer.load_object(config, "gcode_move", true)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // (python registers the PALETTE_* commands here, before reading the
        // options below; a config error stops the startup either way)
        let serial_port = config.get("serial", Dflt::Required)?;
        if serial_port.is_empty() {
            return Err(KError::config("Invalid serial port specific for Palette 2"));
        }
        let baud = config.getint("baud", 115200, Lim::new())?;
        let feedrate_splice = config.getfloat("feedrate_splice", 0.8, Lim::new().min(0.0).max(1.0))?;
        let feedrate_normal = config.getfloat("feedrate_normal", 1.0, Lim::new().min(0.0).max(1.0))?;
        let auto_load_speed = config.getint("auto_load_speed", 2, Lim::new())?;
        let auto_cancel_variation = config.getfloat_opt("auto_cancel_variation", Lim::new().min(0.01).max(0.2))?;
        let p = Rc::new_cyclic(|me| Palette2 {
            me: me.clone(),
            printer: printer.clone(),
            reactor: printer.get_reactor(),
            virtual_sdcard,
            pause_resume,
            gcode: gcode.clone(),
            serial: RefCell::new(None),
            serial_port,
            baud,
            feedrate_splice,
            feedrate_normal,
            auto_load_speed,
            auto_cancel_variation,
            st: RefCell::new(Omega::default()),
            omega_drives: RefCell::new(vec![]),
            read_timer: RefCell::new(None),
            read_buffer: RefCell::new(String::new()),
            read_queue: RefCell::new(VecDeque::new()),
            write_timer: RefCell::new(None),
            write_queue: RefCell::new(VecDeque::new()),
            heartbeat_timer: RefCell::new(None),
            heartbeat: Cell::new(None),
            signal_disconnect: Cell::new(false),
            is_printing: Cell::new(false),
            smart_load_timer: RefCell::new(None),
        });
        let register = |cmd: &str, f: P2Cmd, desc: Option<&str>| -> KResult<()> {
            let w = p.me.clone();
            gcode.register_command(
                cmd,
                Some(Rc::new(move |gcmd: &GCodeCommand| match w.upgrade() {
                    Some(p) => f(&p, gcmd),
                    None => Ok(()),
                })),
                false,
                desc,
            )?;
            Ok(())
        };
        register("PALETTE_CONNECT", Self::cmd_connect, Some(CMD_CONNECT_HELP))?;
        register("PALETTE_DISCONNECT", |p, _| p.cmd_disconnect(), Some(CMD_DISCONNECT_HELP))?;
        register("PALETTE_CLEAR", Self::cmd_clear, Some(CMD_CLEAR_HELP))?;
        register("PALETTE_CUT", Self::cmd_cut, Some(CMD_CUT_HELP))?;
        register("PALETTE_SMART_LOAD", Self::cmd_smart_load, Some(CMD_SMART_LOAD_HELP))?;

        // Omega code matchers
        for i in 0..33 {
            let (f, desc): (P2Cmd, Option<&str>) = match i {
                1 => (Self::cmd_o1, Some(CMD_O1_HELP)),
                9 => (Self::cmd_o9, Some(CMD_O9_HELP)),
                21 => (Self::cmd_o21, None),
                22 => (|p, g| p.cmd_header(g, 1, "Omega printer profile: "), None),
                23 => (|p, g| p.cmd_header(g, 2, "Omega slicer profile: "), None),
                24 => (|p, g| p.cmd_header(g, 3, "Omega PPM: "), None),
                25 => (Self::cmd_o25, None),
                26 => (|p, g| p.cmd_header(g, 5, "Omega splices "), None),
                27 => (|p, g| p.cmd_header(g, 6, "Omega pings: "), None),
                28 => (|p, g| p.cmd_header(g, 7, "Omega MSF NA: "), None),
                29 => (|p, g| p.cmd_header(g, 8, "Omega MSF NH: "), None),
                30 => (Self::cmd_o30, None),
                31 => (Self::cmd_o31, None),
                32 => (Self::cmd_o32, None),
                _ => (Self::cmd_omega_default, None),
            };
            register(&format!("O{}", i), f, desc)?;
        }
        Ok(p)
    }

    fn reset(&self) {
        *self.st.borrow_mut() = Omega::default();
    }

    fn put(&self, line: &str) {
        self.write_queue.borrow_mut().push_back(Some(line.to_string()));
    }

    fn check_p2(&self, gcmd: Option<&GCodeCommand>) -> bool {
        if self.serial.borrow().is_some() {
            return true;
        }
        if let Some(gcmd) = gcmd {
            gcmd.respond_info(INFO_NOT_CONNECTED, true);
        }
        false
    }

    /// a reactor timer calling one of the _run_* methods
    fn timer(&self, f: fn(&Palette2, f64) -> KResult<f64>) -> Timer {
        let w = self.me.clone();
        self.reactor.register_timer(
            Box::new(move |eventtime| {
                let Some(p) = w.upgrade() else { return NEVER };
                match f(&p, eventtime) {
                    Ok(t) => t,
                    Err(e) => {
                        p.printer.unhandled_exception(&e);
                        NEVER
                    }
                }
            }),
            NOW,
        )
    }

    fn cmd_connect(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.serial.borrow().is_some() {
            gcmd.respond_info("Palette 2 serial port is already active, disconnect first", true);
            return Ok(());
        }

        self.signal_disconnect.set(false);
        logging::info(&format!("Connecting to Palette 2 on port ({}) at ({})", self.serial_port, self.baud));
        match Uart::open(&self.serial_port, self.baud as u32, None, Some(true)) {
            Ok(s) => *self.serial.borrow_mut() = Some(s),
            // (except SerialException: other errors propagate)
            Err(e) if e.py_class() != "serial.serialutil.SerialException" => return Err(e),
            Err(_) => {
                gcmd.respond_info("Unable to connect to the Palette 2", true);
                return Ok(());
            }
        }

        self.write_queue.borrow_mut().clear();
        self.read_queue.borrow_mut().clear();

        *self.read_timer.borrow_mut() = Some(self.timer(Self::run_read));
        *self.write_timer.borrow_mut() = Some(self.timer(Self::run_write));
        *self.heartbeat_timer.borrow_mut() = Some(self.timer(Self::run_heartbeat));

        // Tell the device we're alive
        self.put("\n");
        self.put(COMMAND_FIRMWARE);
        self.wait_for_heartbeat()
    }

    fn cmd_disconnect(&self) -> KResult<()> {
        self.gcode.respond_info("Disconnecting from Palette 2", true);
        // closed when dropped
        self.serial.borrow_mut().take();

        for t in [&self.read_timer, &self.write_timer, &self.heartbeat_timer] {
            let t = t.borrow().clone().ok_or_else(no_timer)?;
            self.reactor.unregister_timer(&t);
        }
        *self.read_timer.borrow_mut() = None;
        *self.write_timer.borrow_mut() = None;
        self.heartbeat.set(None);
        self.is_printing.set(false);
        Ok(())
    }

    fn cmd_clear(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::info("Clearing Palette 2 input and output");
        if self.check_p2(Some(gcmd)) {
            for l in COMMAND_CLEAR {
                self.put(l);
            }
        }
        Ok(())
    }

    fn cmd_cut(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::info("Cutting outgoing filament in Palette 2");
        if self.check_p2(Some(gcmd)) {
            self.put(COMMAND_CUT);
        }
        Ok(())
    }

    fn cmd_smart_load(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.check_p2(Some(gcmd)) {
            if !self.st.borrow().is_loading {
                gcmd.respond_info("Cannot auto load when the Palette 2 is not ready", true);
                return Ok(());
            }
            self.p2cmd_o102()?;
        }
        Ok(())
    }

    fn cmd_omega_default(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::debug(&format!("Omega Code: {}", gcmd.get_command()));
        if self.check_p2(Some(gcmd)) {
            self.put(gcmd.get_commandline());
        }
        Ok(())
    }

    fn wait_for_heartbeat(&self) -> KResult<()> {
        let start_ts = self.reactor.monotonic();
        let mut curr_ts = start_ts;
        loop {
            let waiting = match self.heartbeat.get() {
                None => true,
                Some(hb) => hb < (curr_ts - SETUP_TIMEOUT) && start_ts > (curr_ts - SETUP_TIMEOUT),
            };
            if !waiting {
                break;
            }
            curr_ts = self.reactor.pause(curr_ts + 1.0)?;
        }

        if self.heartbeat.get().unwrap_or(0.0) < (curr_ts - SETUP_TIMEOUT) {
            self.signal_disconnect.set(true);
            return Err(KError::command("No response from Palette 2"));
        }
        Ok(())
    }

    fn cmd_o1(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::info("Initializing print with Pallete 2");
        if !self.check_p2(Some(gcmd)) {
            return Err(KError::command("Cannot initialize print, palette 2 is not connected"));
        }

        if let Some(t) = self.heartbeat_timer.borrow().clone() {
            self.reactor.update_timer(&t, NOW);
        }
        self.wait_for_heartbeat()?;
        self.put(gcmd.get_commandline());
        self.gcode.respond_info("Palette 2 waiting on user to complete setup", true);
        self.pause_resume.send_pause_command()
    }

    fn cmd_o9(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::info("Print finished, resetting Palette 2 state");
        if self.check_p2(Some(gcmd)) {
            self.put(gcmd.get_commandline());
        }
        self.is_printing.set(false);
        Ok(())
    }

    fn cmd_o21(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::debug(&format!("Omega version: {}", gcmd.get_commandline()));
        self.reset();
        self.st.borrow_mut().header[0] = Some(gcmd.get_commandline().to_string());
        self.is_printing.set(true);
        Ok(())
    }

    /// cmd_O22 .. cmd_O29 (but O25): log and store an omega header line
    fn cmd_header(&self, gcmd: &GCodeCommand, index: usize, log_prefix: &str) -> KResult<()> {
        logging::debug(&format!("{}{}", log_prefix, gcmd.get_commandline()));
        self.st.borrow_mut().header[index] = Some(gcmd.get_commandline().to_string());
        Ok(())
    }

    fn cmd_o25(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::debug(&format!("Omega inputs: {}", gcmd.get_commandline()));
        let line = gcmd.get_commandline().to_string();
        self.st.borrow_mut().header[4] = Some(line.clone());
        let mut drives: Vec<String> = tail(&line, 4).split_whitespace().map(|s| s.to_string()).collect();
        for (idx, d) in drives.iter_mut().enumerate() {
            let state: String = d.chars().take(2).collect();
            if state == "D1" {
                *d = format!("U{}", 60 + idx);
            }
        }
        let drives: Vec<String> = drives.into_iter().filter(|d| d != "D0").collect();
        let repr = PyVal::List(drives.iter().map(|d| PyVal::Str(d.clone())).collect()).repr();
        *self.omega_drives.borrow_mut() = drives;
        logging::info(&format!("Omega drives: {}", repr));
        Ok(())
    }

    fn cmd_o30(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let line = gcmd.get_commandline();
        let param_drive: String = line.chars().skip(5).take(1).collect();
        let param_distance = tail(line, 8).to_string();
        match py_int(&param_drive) {
            Some(d) => self.st.borrow_mut().splices.push((d, param_distance.clone())),
            None => gcmd.respond_info("Incorrectly formatted splice command", true),
        }
        logging::debug(&format!("Omega splice command drive {} distance {}", param_drive, param_distance));
        Ok(())
    }

    fn cmd_o31(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.check_p2(Some(gcmd)) {
            self.st.borrow_mut().current_ping = Some(gcmd.get_commandline().to_string());
            logging::debug(&format!("Omega ping command: {}", gcmd.get_commandline()));

            self.put(COMMAND_PING);
            // python creates (but never runs) a "G4 P10" command here
        }
        Ok(())
    }

    fn cmd_o32(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::debug(&format!("Omega algorithm: {}", gcmd.get_commandline()));
        self.st.borrow_mut().algorithms.push(gcmd.get_commandline().to_string());
        Ok(())
    }

    fn p2cmd_o20(&self, params: &[&str]) -> KResult<()> {
        if !self.is_printing.get() {
            return Ok(());
        }

        let first = *params.first().ok_or_else(|| index_error("list"))?;
        // First print, we can ignore
        if first == "D5" {
            logging::info("First print on Palette");
            return Ok(());
        }

        let n = py_int(tail(first, 1)).ok_or_else(|| value_error(10, tail(first, 1)))?;

        if n == 0 {
            let counter = self.st.borrow().header_counter;
            logging::info(&format!("Sending omega header {}", counter));
            let h = self.st.borrow().header.get(counter).cloned().ok_or_else(|| index_error("list"))?;
            self.write_queue.borrow_mut().push_back(h);
            self.st.borrow_mut().header_counter = counter + 1;
        } else if n == 1 {
            let counter = self.st.borrow().splices_counter;
            logging::info(&format!("Sending splice info {}", counter));
            let splice = self.st.borrow().splices.get(counter).cloned().ok_or_else(|| index_error("list"))?;
            self.put(&format!("O30 D{} D{}", splice.0, splice.1));
            self.st.borrow_mut().splices_counter = counter + 1;
        } else if n == 2 {
            let ping = self.st.borrow().current_ping.clone();
            logging::info(&format!("Sending current ping info {}", ping.as_deref().unwrap_or("None")));
            self.write_queue.borrow_mut().push_back(ping);
        } else if n == 4 {
            let counter = self.st.borrow().algorithms_counter;
            logging::info(&format!("Sending algorithm info {}", counter));
            let a = self.st.borrow().algorithms.get(counter).cloned().ok_or_else(|| index_error("list"))?;
            self.put(&a);
            self.st.borrow_mut().algorithms_counter = counter + 1;
        } else if n == 8 {
            logging::info("Resending the last command to Palette 2");
            let last = self.st.borrow().last_command.clone();
            self.put(&last);
        }
        Ok(())
    }

    fn p2cmd_o34(&self, params: &[&str]) -> KResult<()> {
        if !self.is_printing.get() {
            return Ok(());
        }

        if params.len() > 2 {
            let p1 = tail(params[1], 1);
            let percent = py_float(p1).ok_or_else(|| KError::Py("ValueError".into(), format!("could not convert string to float: {}", py_str_repr(p1))))?;
            if params[0] == "D1" {
                let number = self.st.borrow().pings.len() as i64 + 1;
                logging::info(&format!("Ping {}, {} percent", number, percent.trunc() as i64));
                self.st.borrow_mut().pings.push((number, percent));
                // check_ping_variation
                if let Some(v) = self.auto_cancel_variation {
                    let ping_max = 100.0 + (v * 100.0);
                    let ping_min = 100.0 - (v * 100.0);
                    if percent < ping_min || percent > ping_max {
                        logging::info("Ping variation is too high, cancelling print");
                        self.gcode.run_script("CANCEL_PRINT")?;
                    }
                }
            } else if params[0] == "D2" {
                let number = self.st.borrow().pongs.len() as i64 + 1;
                logging::info(&format!("Pong {}, {} percent", number, percent.trunc() as i64));
                self.st.borrow_mut().pongs.push((number, percent));
            }
        }
        Ok(())
    }

    fn p2cmd_o50(&self, params: &[&str]) -> KResult<()> {
        if params.len() > 1 {
            let fw = tail(params[0], 1);
            logging::info(&format!("Palette 2 firmware version {} detected", fw));

            if fw < "9.0.9" {
                return Err(KError::command("Palette 2 firmware version is too old, update to at least 9.0.9"));
            }
        } else {
            let files: Vec<String> = self.virtual_sdcard.get_file_list(true)?.into_iter().map(|(f, _)| f).filter(|f| f.contains(".mcf.gcode")).collect();
            for file in &files {
                self.put(&format!("{} D{}", COMMAND_FILENAME, file));
            }
            self.st.borrow_mut().files = files;
            self.put(COMMAND_FILENAMES_DONE);
        }
        Ok(())
    }

    fn p2cmd_o53(&self, params: &[&str]) -> KResult<()> {
        if params.len() > 1 && params[0] == "D1" {
            let p1 = tail(params[1], 1);
            let idx = py_int_base16(p1).ok_or_else(|| value_error(16, p1))?;
            let files = self.st.borrow().files.clone();
            // files[::-1][idx], python indexing (negative from the end)
            let n = files.len() as i64;
            let i = if idx < 0 { idx + n } else { idx };
            if i < 0 || i >= n {
                logging::error("O53 has invalid command parameters");
                return Ok(());
            }
            let file = &files[(n - 1 - i) as usize];
            self.gcode.run_script(&format!("SDCARD_PRINT_FILE FILENAME={}", file))?;
        }
        Ok(())
    }

    fn p2cmd_o88(&self, params: &[&str]) -> KResult<()> {
        logging::error("Palette 2 error detected");
        let Some(first) = params.first() else {
            logging::error("Unable to parse Palette 2 error");
            return Ok(());
        };
        let p = tail(first, 1);
        let error = py_int_base16(p).ok_or_else(|| value_error(16, p))?;
        logging::error(&format!("Palette 2 error code {}", error));
        Ok(())
    }

    fn p2cmd_o97(&self, params: &[&str]) -> KResult<()> {
        #[derive(Clone, Copy)]
        enum M {
            PrintCancelling,
            PrintCancelled,
            LoadingOffset,
            LoadingOffsetStart,
            FeedrateStart,
            FeedrateEnd,
        }
        let mut matchers: Vec<(M, usize, &[&str])> = vec![];
        if self.is_printing.get() {
            matchers.extend([
                (M::PrintCancelling, 2, &["U0", "D2"][..]),
                (M::PrintCancelled, 2, &["U0", "D3"][..]),
                (M::LoadingOffset, 2, &["U39"][..]),
                (M::LoadingOffsetStart, 1, &["U39"][..]),
            ]);
        }

        matchers.push((M::FeedrateStart, 3, &["U25", "D0"]));
        matchers.push((M::FeedrateEnd, 3, &["U25", "D1"]));
        // _param_Matcher: the first matching handler runs
        let Some((m, _, _)) = matchers
            .iter()
            .find(|(_, count, match_params)| params.len() >= *count && match_params.iter().enumerate().all(|(i, mp)| params.get(i) == Some(mp)))
        else {
            return Ok(());
        };
        match m {
            M::PrintCancelling => {
                logging::info("Print Cancelling");
                self.gcode.run_script("CLEAR_PAUSE")?;
                self.gcode.run_script("CANCEL_PRINT")?;
            }
            M::PrintCancelled => {
                logging::info("Print Cancelled");
                self.reset();
            }
            M::LoadingOffsetStart => {
                logging::info("Waiting for user to load filament into printer");
                self.st.borrow_mut().is_loading = true;
            }
            M::LoadingOffset => {
                let p1 = tail(params[1], 1);
                let remaining = py_int(p1).ok_or_else(|| value_error(10, p1))?;
                self.st.borrow_mut().remaining_load_length = Some(remaining);
                logging::debug(&format!("Loading filamant remaining {}", remaining));
                let timer = self.smart_load_timer.borrow().clone();
                if remaining >= 0 {
                    if let Some(t) = timer {
                        logging::info("Smart load filament is complete");
                        self.reactor.unregister_timer(&t);
                        *self.smart_load_timer.borrow_mut() = None;
                        self.st.borrow_mut().is_loading = false;
                    }
                }
            }
            M::FeedrateStart => {
                logging::info(&format!("Setting feedrate to {:.6} for splice", self.feedrate_splice));
                self.st.borrow_mut().is_splicing = true;
                self.gcode.run_script(&format!("M220 S{}", (self.feedrate_splice * 100.0).trunc() as i64))?;
            }
            M::FeedrateEnd => {
                logging::info(&format!("Setting feedrate to {:.6} splice done", self.feedrate_normal));
                self.st.borrow_mut().is_splicing = false;
                self.gcode.run_script(&format!("M220 S{}", (self.feedrate_normal * 100.0).trunc() as i64))?;
            }
        }
        Ok(())
    }

    fn p2cmd_o100(&self) -> KResult<()> {
        logging::info("Pause request from Palette 2");
        self.st.borrow_mut().is_setup_complete = true;
        self.pause_resume.send_pause_command()
    }

    fn p2cmd_o102(&self) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        if !toolhead.get_extruder().get_heater()?.can_extrude.get() {
            self.put(COMMAND_SMART_LOAD_STOP);
            self.gcode.respond_info("Unable to auto load filament, extruder is below minimum temp", true);
            return Ok(());
        }

        if self.smart_load_timer.borrow().is_none() {
            logging::info("Smart load starting");
            *self.smart_load_timer.borrow_mut() = Some(self.timer(Self::run_smart_load));
        }
        Ok(())
    }

    fn p2cmd(&self, line: &str) -> KResult<()> {
        let t: Vec<&str> = line.split_whitespace().collect();
        let ocode = t[0];
        let params = &t[1..];
        // (python's parameter check filters D / U words, then all() of the
        // non-empty strings left, which always passes)
        match ocode {
            "O20" => self.p2cmd_o20(params),
            "O34" => self.p2cmd_o34(params),
            "O40" => {
                logging::info("Resume request from Palette 2");
                self.pause_resume.send_resume_command()
            }
            "O50" => self.p2cmd_o50(params),
            "O53" => self.p2cmd_o53(params),
            "O88" => self.p2cmd_o88(params),
            "O97" => self.p2cmd_o97(params),
            "O100" => self.p2cmd_o100(),
            "O102" => self.p2cmd_o102(),
            _ => Ok(()),
        }
    }

    fn run_read(&self, eventtime: f64) -> KResult<f64> {
        if self.signal_disconnect.get() {
            self.cmd_disconnect()?;
            return Ok(NEVER);
        }

        // Do non-blocking reads from serial and try to find lines
        loop {
            let res = self.serial.borrow().as_ref().map(|s| s.read_checked(1));
            let raw = match res {
                Some(Ok(raw)) => raw,
                Some(Err(_)) => {
                    logging::error("Unable to communicate with the Palette 2");
                    self.cmd_disconnect()?;
                    return Ok(NEVER);
                }
                None => return Err(KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'read'".into())),
            };
            if raw.is_empty() {
                break;
            }
            // each byte is decoded alone, errors="ignore": only ascii survives
            let new_buffer: String = raw.iter().filter(|b| b.is_ascii()).map(|b| *b as char).collect();
            let mut text_buffer = format!("{}{}", self.read_buffer.borrow(), new_buffer);
            while let Some(i) = text_buffer.find('\n') {
                let line = text_buffer[..i + 1].to_string();
                self.read_queue.borrow_mut().push_back(py_strip(&line).to_string());
                text_buffer = text_buffer[i + 1..].to_string();
            }
            *self.read_buffer.borrow_mut() = text_buffer;
        }

        // Process any decoded lines from the device
        loop {
            let Some(text_line) = self.read_queue.borrow_mut().pop_front() else { break };

            let heartbeat_strings = [COMMAND_HEARTBEAT, "Connection Okay"];
            if !heartbeat_strings.iter().any(|x| text_line.contains(x)) {
                logging::debug(&format!("{:.3} P2 -> : {}", eventtime, text_line));
            }

            // Received a heartbeat from the device
            if text_line == COMMAND_HEARTBEAT {
                self.heartbeat.set(Some(eventtime));
            } else if text_line.chars().next().ok_or_else(|| index_error("string"))? == 'O' {
                self.p2cmd(&text_line)?;
            }
        }

        Ok(eventtime + SERIAL_TIMER)
    }

    fn run_heartbeat(&self, eventtime: f64) -> KResult<f64> {
        self.put(COMMAND_HEARTBEAT);
        let eventtime = self.reactor.pause(eventtime + 5.0)?;
        if let Some(hb) = self.heartbeat.get().filter(|hb| *hb != 0.0) {
            if hb < (eventtime - HEARTBEAT_TIMEOUT) {
                logging::error("P2 has not responded to heartbeat");
                if !self.is_printing.get() || self.st.borrow().is_setup_complete {
                    self.cmd_disconnect()?;
                    return Ok(NEVER);
                }
            }
        }
        Ok(eventtime + HEARTBEAT_SEND)
    }

    fn run_write(&self, eventtime: f64) -> KResult<f64> {
        loop {
            let Some(text_line) = self.write_queue.borrow_mut().pop_front() else { break };

            let Some(text_line) = text_line.filter(|t| !t.is_empty()) else { continue };
            self.st.borrow_mut().last_command = text_line.clone();
            let l = py_strip(&text_line);
            if !l.contains(COMMAND_HEARTBEAT) {
                logging::debug(&format!("{} -> P2 : {}", py_float_repr(self.reactor.monotonic()), l));
            }
            let terminated_line = format!("{}\n", l);
            let res = match self.serial.borrow().as_ref() {
                Some(s) => s.write_checked(terminated_line.as_bytes()),
                None => return Err(KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'write'".into())),
            };
            if res.is_err() {
                logging::error("Unable to communicate with the Palette 2");
                self.signal_disconnect.set(true);
                return Ok(NEVER);
            }
            return Ok(eventtime + SERIAL_TIMER);
        }
        Ok(eventtime + SERIAL_TIMER)
    }

    fn run_smart_load(&self, eventtime: f64) -> KResult<f64> {
        let (is_splicing, remaining) = {
            let st = self.st.borrow();
            (st.is_splicing, st.remaining_load_length)
        };
        if !is_splicing {
            let remaining = remaining.ok_or_else(|| KError::Py("TypeError".into(), "'<' not supported between instances of 'NoneType' and 'int'".into()))?;
            if remaining < 0 {
                // Make sure toolhead class isn't busy
                let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
                let (print_time, est_print_time, lookahead_empty) = toolhead.check_busy(eventtime);
                let idle_time = est_print_time - print_time;
                if !lookahead_empty || idle_time < 0.5 {
                    return Ok(eventtime + 0.0f64.max(1.0f64.min(print_time - est_print_time)));
                }

                let extrude = remaining.abs() as f64;
                let mut extrude = 50.0f64.min(extrude / 2.0);
                if extrude <= 10.0 {
                    extrude = 1.0;
                }
                logging::info(&format!("Smart loading {}mm filament with {}mm remaining", extrude.trunc() as i64, remaining.abs()));

                self.gcode.run_script("G92 E0")?;
                self.gcode.run_script(&format!("G1 E{} F{}", extrude.trunc() as i64, self.auto_load_speed * 60))?;
                return Ok(NOW);
            }
        }
        Ok(eventtime + AUTOLOAD_TIMER)
    }
}

impl PrinterObject for Palette2 {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let st = self.st.borrow();
        let mut status = IndexMap::new();
        status.insert("ping".to_string(), PyVal::None);
        status.insert("remaining_load_length".to_string(), st.remaining_load_length.map(PyVal::Int).unwrap_or(PyVal::None));
        status.insert("is_splicing".to_string(), PyVal::Bool(st.is_splicing));
        if let Some((number, percent)) = st.pings.last() {
            let mut d = IndexMap::new();
            d.insert("number".to_string(), PyVal::Int(*number));
            d.insert("percent".to_string(), PyVal::Float(*percent));
            status.insert("ping".to_string(), PyVal::Dict(d));
        }
        Some(PyVal::Dict(status))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Palette2::new(config)?)
}
