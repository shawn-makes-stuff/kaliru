// Virtual sdcard support (print files directly from a host g-code file)
// (port of klippy/extras/virtual_sdcard.py)
//
// Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::error_mcu::event_args;
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::print_stats::PrintStats;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_str_repr, py_strip};
use crate::pyglob::{expanduser, join, normpath};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::io::{Read, Seek, SeekFrom};
use std::rc::{Rc, Weak};

pub const VALID_GCODE_EXTS: &[&str] = &["gcode", "g", "gco"];

pub const DEFAULT_ERROR_GCODE: &str = "
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
";

/// io.open(name, "r", newline=""): utf-8 text, no newline translation,
/// tell()/seek() in bytes, read(n) in characters
pub struct TextFile {
    pub name: String,
    file: std::fs::File,
    /// bytes read from the file but not handed out yet
    buf: Vec<u8>,
}

impl TextFile {
    fn open(name: &str) -> std::io::Result<Self> {
        Ok(TextFile { name: name.to_string(), file: std::fs::File::open(name)?, buf: vec![] })
    }

    fn seek(&mut self, pos: u64) -> std::io::Result<()> {
        self.buf.clear();
        self.file.seek(SeekFrom::Start(pos))?;
        Ok(())
    }

    fn size(&mut self) -> std::io::Result<u64> {
        let end = self.file.seek(SeekFrom::End(0))?;
        self.seek(0)?;
        Ok(end)
    }

    /// read(n): up to n characters (an Err for undecodable data)
    fn read(&mut self, n: usize) -> Result<String, String> {
        loop {
            // complete characters in the buffer
            let chars = self.buf.iter().filter(|b| (**b & 0xC0) != 0x80).count();
            if chars > n {
                break;
            }
            let mut chunk = vec![0u8; 8192];
            let got = self.file.read(&mut chunk).map_err(|e| e.to_string())?;
            if got == 0 {
                break;
            }
            self.buf.extend_from_slice(&chunk[..got]);
        }
        // take n characters
        let mut count = 0;
        let mut end = self.buf.len();
        for (i, b) in self.buf.iter().enumerate() {
            if (b & 0xC0) != 0x80 {
                if count == n {
                    end = i;
                    break;
                }
                count += 1;
            }
        }
        let taken: Vec<u8> = self.buf.drain(..end).collect();
        String::from_utf8(taken).map_err(|_| "'utf-8' codec can't decode bytes".to_string())
    }
}

const CMD_SDCARD_RESET_FILE_HELP: &str = "Clears a loaded SD File. Stops the print if necessary";
const CMD_SDCARD_PRINT_FILE_HELP: &str = "Loads a SD file and starts the print.  May include files in subdirectories.";

pub struct VirtualSD {
    me: Weak<VirtualSD>,
    printer: Rc<Printer>,
    pub with_subdirs: bool,
    pub sdcard_dirname: String,
    current_file: RefCell<Option<TextFile>>,
    file_position: Cell<u64>,
    file_size: Cell<u64>,
    print_stats: Rc<PrintStats>,
    reactor: Rc<Reactor>,
    must_pause_work: Cell<bool>,
    cmd_from_sd: Cell<bool>,
    next_file_position: Cell<u64>,
    work_timer: RefCell<Option<Timer>>,
    on_error_gcode: Rc<Template>,
    gcode: Rc<GCodeDispatch>,
}

impl VirtualSD {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // sdcard state
        let sd = config.get("path", Dflt::Required)?;
        let with_subdirs = config.getboolean("with_subdirs", false)?;
        let sdcard_dirname = normpath(&expanduser(&sd));
        // Print Stat Tracking
        let print_stats = printer
            .load_object(config, "print_stats", true)?
            .and_then(downcast_rc::<PrintStats>)
            .ok_or_else(|| KError::Other("print_stats has an unexpected type".into()))?;
        // Error handling
        let gcode_macro = load_gcode_macro(config)?;
        let on_error_gcode = gcode_macro.load_template(config, "on_error_gcode", Some(DEFAULT_ERROR_GCODE))?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let vsd = Rc::new_cyclic(|me| VirtualSD {
            me: me.clone(),
            printer: printer.clone(),
            with_subdirs,
            sdcard_dirname,
            current_file: RefCell::new(None),
            file_position: Cell::new(0),
            file_size: Cell::new(0),
            print_stats,
            reactor: printer.get_reactor(),
            must_pause_work: Cell::new(false),
            cmd_from_sd: Cell::new(false),
            next_file_position: Cell::new(0),
            work_timer: RefCell::new(None),
            on_error_gcode,
            gcode: gcode.clone(),
        });
        // Register commands
        type VCmd = fn(&VirtualSD, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, VCmd); 7] = [
            ("M20", VirtualSD::cmd_m20),
            ("M21", VirtualSD::cmd_m21),
            ("M23", VirtualSD::cmd_m23),
            ("M24", VirtualSD::cmd_m24),
            ("M25", VirtualSD::cmd_m25),
            ("M26", VirtualSD::cmd_m26),
            ("M27", VirtualSD::cmd_m27),
        ];
        for (cmd, f) in cmds {
            let v = vsd.clone();
            gcode.register_command(cmd, Some(Rc::new(move |g| f(&v, g))), false, None)?;
        }
        for cmd in ["M28", "M29", "M30"] {
            gcode.register_command(cmd, Some(Rc::new(|g: &GCodeCommand| Err(g.error("SD write not supported")))), false, None)?;
        }
        let v = vsd.clone();
        gcode.register_command(
            "SDCARD_RESET_FILE",
            Some(Rc::new(move |g| v.cmd_sdcard_reset_file(g))),
            false,
            Some(CMD_SDCARD_RESET_FILE_HELP),
        )?;
        let v = vsd.clone();
        gcode.register_command(
            "SDCARD_PRINT_FILE",
            Some(Rc::new(move |g| v.cmd_sdcard_print_file(g))),
            false,
            Some(CMD_SDCARD_PRINT_FILE_HELP),
        )?;
        let v = vsd.clone();
        printer.register_event_handler(
            "klippy:analyze_shutdown",
            Rc::new(move |args| {
                let (msg, details) = event_args(args);
                v.handle_analyze_shutdown(&msg, &details);
                Ok(())
            }),
        );
        Ok(vsd)
    }

    fn handle_analyze_shutdown(&self, _msg: &str, _details: &PyVal) {
        if self.work_timer.borrow().is_none() {
            return;
        }
        self.must_pause_work.set(true);
        let readpos = self.file_position.get().saturating_sub(1024);
        let readcount = (self.file_position.get() - readpos) as usize;
        let data = {
            let mut cf = self.current_file.borrow_mut();
            let r = match cf.as_mut() {
                None => Err("'NoneType' object has no attribute 'seek'".to_string()),
                Some(f) => f.seek(readpos).map_err(|e| e.to_string()).and_then(|_| f.read(readcount + 128)),
            };
            match r {
                Ok(d) => d,
                Err(e) => {
                    logging::exception_err("virtual_sdcard shutdown read", &KError::Other(e));
                    return;
                }
            }
        };
        let split = data.char_indices().nth(readcount).map(|(i, _)| i).unwrap_or(data.len());
        logging::info(&format!(
            "Virtual sdcard ({}): {}\nUpcoming ({}): {}",
            readpos,
            py_str_repr(&data[..split]),
            self.file_position.get(),
            py_str_repr(&data[split..])
        ));
    }

    pub fn get_file_list(&self, check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        if check_subdirs {
            let mut flist = vec![];
            walk(&self.sdcard_dirname, &mut |root, name| {
                let ext = match name.rfind('.') {
                    Some(i) => &name[i + 1..],
                    None => name,
                };
                if !VALID_GCODE_EXTS.contains(&ext) {
                    return;
                }
                let full_path = join(root, &[name]);
                let r_path = full_path.get(self.sdcard_dirname.len() + 1..).unwrap_or("").to_string();
                let size = std::fs::metadata(&full_path).map(|m| m.len()).unwrap_or(0);
                flist.push((r_path, size));
            });
            flist.sort_by_key(|f: &(String, u64)| f.0.to_lowercase());
            return Ok(flist);
        }
        let dname = &self.sdcard_dirname;
        let listed = std::fs::read_dir(dname).map(|rd| rd.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().into_owned()).collect::<Vec<_>>());
        match listed {
            Ok(mut filenames) => {
                filenames.sort_by_key(|f| f.to_lowercase());
                Ok(filenames
                    .into_iter()
                    .filter(|fname| !fname.starts_with('.') && std::path::Path::new(&join(dname, &[fname])).is_file())
                    .map(|fname| {
                        let size = std::fs::metadata(join(dname, &[&fname])).map(|m| m.len()).unwrap_or(0);
                        (fname, size)
                    })
                    .collect())
            }
            Err(e) => {
                logging::exception_err(
                    "virtual_sdcard get_file_list",
                    &KError::os(&e, dname),
                );
                Err(KError::command("Unable to get file list"))
            }
        }
    }

    fn file_path(&self) -> PyVal {
        match self.current_file.borrow().as_ref() {
            Some(f) => PyVal::Str(f.name.clone()),
            None => PyVal::None,
        }
    }

    fn progress(&self) -> f64 {
        if self.file_size.get() != 0 {
            self.file_position.get() as f64 / self.file_size.get() as f64
        } else {
            0.0
        }
    }

    pub fn is_active(&self) -> bool {
        self.work_timer.borrow().is_some()
    }

    pub fn do_pause(&self) -> KResult<()> {
        if self.work_timer.borrow().is_some() {
            self.must_pause_work.set(true);
            while self.work_timer.borrow().is_some() && !self.cmd_from_sd.get() {
                self.reactor.pause(self.reactor.monotonic() + 0.001)?;
            }
        }
        Ok(())
    }

    pub fn do_resume(&self) -> KResult<()> {
        if self.work_timer.borrow().is_some() {
            return Err(KError::command("SD busy"));
        }
        self.must_pause_work.set(false);
        let w = self.me.clone();
        let timer = self.reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(v) => v.work_handler(eventtime),
                None => NEVER,
            }),
            NOW,
        );
        *self.work_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    pub fn do_cancel(&self) -> KResult<()> {
        if self.current_file.borrow().is_some() {
            self.do_pause()?;
            *self.current_file.borrow_mut() = None;
            self.print_stats.note_cancel()?;
        }
        self.file_position.set(0);
        self.file_size.set(0);
        Ok(())
    }

    // G-Code commands
    fn reset_file(&self) -> KResult<()> {
        if self.current_file.borrow().is_some() {
            self.do_pause()?;
            *self.current_file.borrow_mut() = None;
        }
        self.file_position.set(0);
        self.file_size.set(0);
        self.print_stats.reset()?;
        self.printer.send_event("virtual_sdcard:reset_file", &[])
    }

    fn cmd_sdcard_reset_file(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.cmd_from_sd.get() {
            return Err(gcmd.error("SDCARD_RESET_FILE cannot be run from the sdcard"));
        }
        self.reset_file()
    }

    fn cmd_sdcard_print_file(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.work_timer.borrow().is_some() {
            return Err(gcmd.error("SD busy"));
        }
        self.reset_file()?;
        let mut filename = gcmd.get("FILENAME")?;
        if filename.starts_with('/') {
            filename = filename[1..].to_string();
        }
        self.load_file(gcmd, &filename, true)?;
        self.do_resume()
    }

    fn cmd_m20(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // List SD card
        let files = self.get_file_list(self.with_subdirs)?;
        gcmd.respond_raw("Begin file list");
        for (fname, fsize) in files {
            gcmd.respond_raw(&format!("{} {}", fname, fsize));
        }
        gcmd.respond_raw("End file list");
        Ok(())
    }

    fn cmd_m21(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Initialize SD card
        gcmd.respond_raw("SD card ok");
        Ok(())
    }

    fn cmd_m23(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Select SD file
        if self.work_timer.borrow().is_some() {
            return Err(gcmd.error("SD busy"));
        }
        self.reset_file()?;
        let mut filename = py_strip(&gcmd.get_raw_command_parameters()).to_string();
        if filename.starts_with('/') {
            filename = filename[1..].to_string();
        }
        self.load_file(gcmd, &filename, self.with_subdirs)
    }

    fn load_file(&self, gcmd: &GCodeCommand, filename: &str, check_subdirs: bool) -> KResult<()> {
        let files = self.get_file_list(check_subdirs)?;
        let flist: Vec<&String> = files.iter().map(|f| &f.0).collect();
        let mut files_by_lower: IndexMap<String, String> = IndexMap::new();
        for (fname, _fsize) in &files {
            files_by_lower.insert(fname.to_lowercase(), fname.clone());
        }
        let opened = (|| -> Result<(TextFile, u64), String> {
            let mut fname = filename.to_string();
            if !flist.contains(&&fname) {
                fname = files_by_lower.get(&fname.to_lowercase()).cloned().ok_or_else(|| py_str_repr(&fname.to_lowercase()))?;
            }
            let fname = join(&self.sdcard_dirname, &[&fname]);
            let mut f = TextFile::open(&fname).map_err(|e| crate::pyfmt::py_oserror(&e, &fname))?;
            let fsize = f.size().map_err(|e| e.to_string())?;
            Ok((f, fsize))
        })();
        let (f, fsize) = match opened {
            Ok(v) => v,
            Err(e) => {
                logging::exception_err("virtual_sdcard file open", &KError::Other(e));
                return Err(gcmd.error("Unable to open file"));
            }
        };
        gcmd.respond_raw(&format!("File opened: {} Size: {}", filename, fsize));
        gcmd.respond_raw("File selected");
        *self.current_file.borrow_mut() = Some(f);
        self.file_position.set(0);
        self.file_size.set(fsize);
        self.print_stats.set_current_file(filename)?;
        self.printer.send_event("virtual_sdcard:load_file", &[])
    }

    fn cmd_m24(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Start/resume SD print
        self.do_resume()
    }

    fn cmd_m25(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Pause SD print
        self.do_pause()
    }

    fn cmd_m26(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Set SD position
        if self.work_timer.borrow().is_some() {
            return Err(gcmd.error("SD busy"));
        }
        let pos = gcmd.get_int("S", None, Lim::new().min(0))?;
        self.file_position.set(pos as u64);
        Ok(())
    }

    fn cmd_m27(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Report SD print status
        if self.current_file.borrow().is_none() {
            gcmd.respond_raw("Not SD printing.");
            return Ok(());
        }
        gcmd.respond_raw(&format!("SD printing byte {}/{}", self.file_position.get(), self.file_size.get()));
        Ok(())
    }

    pub fn get_file_position(&self) -> u64 {
        self.next_file_position.get()
    }

    pub fn set_file_position(&self, pos: u64) {
        self.next_file_position.set(pos);
    }

    pub fn is_cmd_from_sd(&self) -> bool {
        self.cmd_from_sd.get()
    }

    // Background work timer
    fn work_handler(&self, _eventtime: f64) -> f64 {
        logging::info(&format!("Starting SD card print (position {})", self.file_position.get()));
        if let Some(t) = self.work_timer.borrow().as_ref() {
            self.reactor.unregister_timer(t);
        }
        let seek = match self.current_file.borrow_mut().as_mut() {
            Some(f) => f.seek(self.file_position.get()).map_err(|e| e.to_string()),
            None => Err("'NoneType' object has no attribute 'seek'".to_string()),
        };
        if let Err(e) = seek {
            logging::exception_err("virtual_sdcard seek", &KError::Other(e));
            *self.work_timer.borrow_mut() = None;
            return NEVER;
        }
        if let Err(e) = self.print_stats.note_start() {
            logging::exception_err("virtual_sdcard note_start", &e);
        }
        let gcode_mutex = self.gcode.get_mutex();
        let mut partial_input = String::new();
        let mut lines: Vec<String> = vec![];
        let mut error_message: Option<String> = None;
        while !self.must_pause_work.get() {
            if lines.is_empty() {
                // Read more data
                let data = match self.current_file.borrow_mut().as_mut().map(|f| f.read(8192)) {
                    Some(Ok(d)) => d,
                    Some(Err(e)) => {
                        logging::exception_err("virtual_sdcard read", &KError::Other(e));
                        break;
                    }
                    None => {
                        logging::exception_err("virtual_sdcard read", &KError::Other("'NoneType' object has no attribute 'read'".into()));
                        break;
                    }
                };
                if data.is_empty() {
                    // End of file
                    *self.current_file.borrow_mut() = None;
                    logging::info("Finished SD card print");
                    self.gcode.respond_raw("Done printing file");
                    break;
                }
                lines = data.split('\n').map(|s| s.to_string()).collect();
                lines[0] = format!("{}{}", partial_input, lines[0]);
                partial_input = lines.pop().unwrap_or_default();
                lines.reverse();
                if self.reactor.pause(NOW).is_err() {
                    break;
                }
                continue;
            }
            // Pause if any other request is pending in the gcode class
            if gcode_mutex.test() {
                if self.reactor.pause(self.reactor.monotonic() + 0.100).is_err() {
                    break;
                }
                continue;
            }
            // Dispatch command
            self.cmd_from_sd.set(true);
            let line = lines.pop().unwrap();
            let next_file_position = self.file_position.get() + line.len() as u64 + 1;
            self.next_file_position.set(next_file_position);
            match self.gcode.run_script(&line) {
                Ok(()) => {}
                Err(e) if e.is_command_error() => {
                    error_message = Some(e.to_string());
                    let r = self.on_error_gcode.render(None).and_then(|s| self.gcode.run_script(&s));
                    if let Err(e) = r {
                        logging::exception_err("virtual_sdcard on_error", &e);
                    }
                    break;
                }
                Err(e) => {
                    logging::exception_err("virtual_sdcard dispatch", &e);
                    break;
                }
            }
            self.cmd_from_sd.set(false);
            self.file_position.set(self.next_file_position.get());
            // Do we need to skip around?
            if self.next_file_position.get() != next_file_position {
                let seek = match self.current_file.borrow_mut().as_mut() {
                    Some(f) => f.seek(self.file_position.get()).map_err(|e| e.to_string()),
                    None => Err("'NoneType' object has no attribute 'seek'".to_string()),
                };
                if let Err(e) = seek {
                    logging::exception_err("virtual_sdcard seek", &KError::Other(e));
                    *self.work_timer.borrow_mut() = None;
                    return NEVER;
                }
                lines.clear();
                partial_input.clear();
            }
        }
        logging::info(&format!("Exiting SD card print (position {})", self.file_position.get()));
        *self.work_timer.borrow_mut() = None;
        self.cmd_from_sd.set(false);
        let r = if let Some(msg) = error_message {
            self.print_stats.note_error(&msg)
        } else if self.current_file.borrow().is_some() {
            self.print_stats.note_pause()
        } else {
            self.print_stats.note_complete()
        };
        if let Err(e) = r {
            logging::exception_err("virtual_sdcard print_stats", &e);
        }
        NEVER
    }
}

/// os.walk(top, followlinks=True): f(root, name) for every file
fn walk(top: &str, f: &mut dyn FnMut(&str, &str)) {
    let Ok(rd) = std::fs::read_dir(top) else { return };
    let mut dirs = vec![];
    let mut files = vec![];
    for e in rd.filter_map(|e| e.ok()) {
        let name = e.file_name().to_string_lossy().into_owned();
        // is_dir() follows symlinks (followlinks=True)
        if std::path::Path::new(&join(top, &[&name])).is_dir() {
            dirs.push(name);
        } else {
            files.push(name);
        }
    }
    for name in &files {
        f(top, name);
    }
    for d in dirs {
        walk(&join(top, &[&d]), f);
    }
}

impl PrinterObject for VirtualSD {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("file_path".to_string(), self.file_path());
        d.insert("progress".to_string(), PyVal::Float(self.progress()));
        d.insert("is_active".to_string(), PyVal::Bool(self.is_active()));
        d.insert("file_position".to_string(), PyVal::Int(self.file_position.get() as i64));
        d.insert("file_size".to_string(), PyVal::Int(self.file_size.get() as i64));
        Some(PyVal::Dict(d))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        if self.work_timer.borrow().is_none() {
            return Some((false, String::new()));
        }
        Some((true, format!("sd_pos={}", self.file_position.get())))
    }
    fn teardown(&self) {
        if let Some(t) = self.work_timer.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
        *self.current_file.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(VirtualSD::new(config)?)
}
