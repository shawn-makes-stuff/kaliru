// Support for reading acceleration data from an adxl345 chip
// (port of klippy/extras/adxl345.py)
//
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{strftime_local, ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, FixedFreqReader, RawSample};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_int_base0, py_min, py_round, py_sum};
use crate::pyglob::gettempdir;
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// ADXL345 registers
const REG_DEVID: u8 = 0x00;
const REG_BW_RATE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_DATA_FORMAT: u8 = 0x31;
const REG_FIFO_CTL: u8 = 0x38;
const REG_MOD_READ: u8 = 0x80;

const QUERY_RATES: [(i64, u8); 8] = [(25, 0x8), (50, 0x9), (100, 0xA), (200, 0xB), (400, 0xC), (800, 0xD), (1600, 0xE), (3200, 0xF)];

const ADXL345_DEV_ID: u8 = 0xE5;
const SET_FIFO_CTL: u8 = 0x90;

const FREEFALL_ACCEL: f64 = 9.80665 * 1000.0;
const SCALE_XY: f64 = 0.003774 * FREEFALL_ACCEL; // 1 / 265 (at 3.3V) mg/LSB
const SCALE_Z: f64 = 0.003906 * FREEFALL_ACCEL; // 1 / 256 (at 3.3V) mg/LSB

/// Accel_Measurement(time, accel_x, accel_y, accel_z)
#[derive(Clone, Copy, Debug)]
pub struct AccelMeasurement {
    pub time: f64,
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
}

/// the samples of one batch message: msg["data"] as (time, x, y, z)
fn msg_data(msg: &PyVal) -> Vec<[f64; 4]> {
    let PyVal::Dict(d) = msg else { return vec![] };
    let Some(PyVal::List(data)) = d.get("data") else { return vec![] };
    data.iter()
        .map(|s| match s {
            PyVal::Tuple(v) | PyVal::List(v) => {
                let f = |i: usize| v.get(i).and_then(|x| x.as_f64()).unwrap_or(0.0);
                [f(0), f(1), f(2), f(3)]
            }
            _ => [0.0; 4],
        })
        .collect()
}

/// the batch message of converted samples (python's {} when there are none)
pub fn batch_msg(samples: &[[f64; 4]], errors: i64, overflows: i64) -> PyVal {
    if samples.is_empty() {
        return PyVal::dict();
    }
    let data = samples.iter().map(|s| PyVal::Tuple(s.iter().map(|v| PyVal::Float(*v)).collect())).collect();
    let mut d = IndexMap::new();
    d.insert("data".to_string(), PyVal::List(data));
    d.insert("errors".to_string(), PyVal::Int(errors));
    d.insert("overflows".to_string(), PyVal::Int(overflows));
    PyVal::Dict(d)
}

// Helper class to obtain measurements
pub struct AccelQueryHelper {
    printer: Rc<Printer>,
    is_finished: Cell<bool>,
    request_start_time: f64,
    request_end_time: Cell<f64>,
    msgs: RefCell<Vec<PyVal>>,
    samples: RefCell<Vec<AccelMeasurement>>,
}

impl AccelQueryHelper {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let print_time = printer.lookup::<ToolHead>("toolhead")?.get_last_move_time()?;
        Ok(Rc::new(AccelQueryHelper {
            printer: printer.clone(),
            is_finished: Cell::new(false),
            request_start_time: print_time,
            request_end_time: Cell::new(print_time),
            msgs: RefCell::new(vec![]),
            samples: RefCell::new(vec![]),
        }))
    }

    pub fn finish_measurements(&self) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        self.request_end_time.set(toolhead.get_last_move_time()?);
        toolhead.wait_moves()?;
        self.is_finished.set(true);
        Ok(())
    }

    pub fn handle_batch(&self, msg: &PyVal) -> bool {
        if self.is_finished.get() {
            return false;
        }
        if self.msgs.borrow().len() >= 10000 {
            // Avoid filling up memory with too many samples
            return false;
        }
        self.msgs.borrow_mut().push(msg.clone());
        true
    }

    pub fn has_valid_samples(&self) -> bool {
        for msg in self.msgs.borrow().iter() {
            let data = msg_data(msg);
            let (Some(first), Some(last)) = (data.first(), data.last()) else { continue };
            let (first_sample_time, last_sample_time) = (first[0], last[0]);
            if first_sample_time > self.request_end_time.get() || last_sample_time < self.request_start_time {
                continue;
            }
            // The time intervals [first_sample_time, last_sample_time]
            // and [request_start_time, request_end_time] have non-zero
            // intersection. It is still theoretically possible that none
            // of the samples from msgs fall into the time interval
            // [request_start_time, request_end_time] if it is too narrow
            // or on very heavy data losses. In practice, that interval
            // is at least 1 second, so this possibility is negligible.
            return true;
        }
        false
    }

    pub fn get_samples(&self) -> Vec<AccelMeasurement> {
        if self.msgs.borrow().is_empty() {
            return self.samples.borrow().clone();
        }
        let mut samples = vec![];
        for msg in self.msgs.borrow().iter() {
            for [samp_time, x, y, z] in msg_data(msg) {
                if samp_time < self.request_start_time {
                    continue;
                }
                if samp_time > self.request_end_time.get() {
                    break;
                }
                samples.push(AccelMeasurement { time: samp_time, accel_x: x, accel_y: y, accel_z: z });
            }
        }
        *self.samples.borrow_mut() = samples.clone();
        samples
    }

    pub fn write_to_file(&self, filename: &str) {
        let samples = {
            let s = self.samples.borrow().clone();
            if s.is_empty() {
                self.get_samples()
            } else {
                s
            }
        };
        let filename = filename.to_string();
        // python writes from a separate (re-niced) process
        std::thread::spawn(move || {
            let mut out = String::from("#time,accel_x,accel_y,accel_z\n");
            for s in samples {
                out += &format!("{:.6},{:.6},{:.6},{:.6}\n", s.time, s.accel_x, s.accel_y, s.accel_z);
            }
            let _ = std::fs::write(&filename, out);
        });
    }
}

/// what AccelCommandHelper and resonance_tester use of an accelerometer
pub trait AccelChip {
    fn start_internal_client(&self) -> KResult<Rc<AccelQueryHelper>>;
    fn read_reg(&self, reg: i64) -> KResult<i64>;
    fn set_reg(&self, reg: i64, val: i64) -> KResult<()>;
    /// chip.name
    fn chip_name(&self) -> String;
}

/// a printer object as an accelerometer chip
pub fn as_accel_chip(obj: &PObj) -> Option<Rc<dyn AccelChip>> {
    if let Some(c) = downcast_rc::<ADXL345>(obj.clone()) {
        return Some(c);
    }
    if let Some(c) = downcast_rc::<crate::extras::mpu9250::MPU9250>(obj.clone()) {
        return Some(c);
    }
    if let Some(c) = downcast_rc::<crate::extras::lis2dw::LIS2DW>(obj.clone()) {
        return Some(c);
    }
    if let Some(c) = downcast_rc::<crate::extras::icm20948::ICM20948>(obj.clone()) {
        return Some(c);
    }
    None
}

const CMD_ACCELEROMETER_MEASURE_HELP: &str = "Start/stop accelerometer";
const CMD_ACCELEROMETER_QUERY_HELP: &str = "Query accelerometer for the current values";
const CMD_ACCELEROMETER_DEBUG_READ_HELP: &str = "Query register (for debugging)";
const CMD_ACCELEROMETER_DEBUG_WRITE_HELP: &str = "Set register (for debugging)";

// Helper class for G-Code commands
pub struct AccelCommandHelper {
    printer: Rc<Printer>,
    chip: RefCell<Weak<dyn AccelChip>>,
    bg_client: RefCell<Option<Rc<AccelQueryHelper>>>,
    base_name: String,
    name: String,
}

impl AccelCommandHelper {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let name_parts: Vec<&str> = config.get_name().split_whitespace().collect();
        let base_name = name_parts[0].to_string();
        let name = name_parts[name_parts.len() - 1].to_string();
        let none: Weak<dyn AccelChip> = Weak::<ADXL345>::new();
        let h = Rc::new(AccelCommandHelper { printer: config.get_printer(), chip: RefCell::new(none), bg_client: RefCell::new(None), base_name, name: name.clone() });
        if let Err(e) = h.register_commands(Some(&name)) {
            if matches!(e.base(), KError::Config(_)) {
                return Err(KError::config(format!("Accelerometer with name '{}' already defined", name)).with_context(e));
            }
            return Err(e);
        }
        if name_parts.len() == 1 && (name == "adxl345" || !config.has_section("adxl345")) {
            h.register_commands(None)?;
        }
        Ok(h)
    }

    fn register_commands(self: &Rc<Self>, name: Option<&str>) -> KResult<()> {
        // Register commands
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        type ACmd = fn(&AccelCommandHelper, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, ACmd, &str); 4] = [
            ("ACCELEROMETER_MEASURE", AccelCommandHelper::cmd_accelerometer_measure, CMD_ACCELEROMETER_MEASURE_HELP),
            ("ACCELEROMETER_QUERY", AccelCommandHelper::cmd_accelerometer_query, CMD_ACCELEROMETER_QUERY_HELP),
            ("ACCELEROMETER_DEBUG_READ", AccelCommandHelper::cmd_accelerometer_debug_read, CMD_ACCELEROMETER_DEBUG_READ_HELP),
            ("ACCELEROMETER_DEBUG_WRITE", AccelCommandHelper::cmd_accelerometer_debug_write, CMD_ACCELEROMETER_DEBUG_WRITE_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let h = self.clone();
            let repr = format!(
                "<bound method AccelCommandHelper.cmd_{} of <klippy.extras.adxl345.AccelCommandHelper object at {:#x}>>",
                cmd,
                Rc::as_ptr(self) as usize
            );
            gcode.register_mux_command_repr(cmd, "CHIP", name, Rc::new(move |g| f(&h, g)), Some(desc), Some(repr))?;
        }
        Ok(())
    }

    pub fn set_chip(&self, chip: Weak<dyn AccelChip>) {
        *self.chip.borrow_mut() = chip;
    }

    fn chip(&self) -> KResult<Rc<dyn AccelChip>> {
        self.chip.borrow().upgrade().ok_or_else(|| KError::Other("accelerometer chip is gone".into()))
    }

    fn cmd_accelerometer_measure(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(bg_client) = self.bg_client.borrow().clone() else {
            // Start measurements
            *self.bg_client.borrow_mut() = Some(self.chip()?.start_internal_client()?);
            gcmd.respond_info("accelerometer measurements started", true);
            return Ok(());
        };
        // End measurements
        let name = gcmd.get_or("NAME", &strftime_local("%Y%m%d_%H%M%S"));
        if !is_valid_name(&name) {
            return Err(gcmd.error("Invalid NAME parameter"));
        }
        *self.bg_client.borrow_mut() = None;
        bg_client.finish_measurements()?;
        // Write data to file
        let tmpdir = gettempdir();
        let filename = if self.base_name == self.name {
            format!("{}/{}-{}.csv", tmpdir, self.base_name, name)
        } else {
            format!("{}/{}-{}-{}.csv", tmpdir, self.base_name, self.name, name)
        };
        bg_client.write_to_file(&filename);
        gcmd.respond_info(&format!("Writing raw accelerometer data to {} file", filename), true);
        Ok(())
    }

    fn cmd_accelerometer_query(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut num_samples = gcmd.get_int("SAMPLES", Some(1), Lim::new())?;
        let mut samples: Vec<AccelMeasurement> = vec![];
        while num_samples > 0 {
            let aclient = self.chip()?.start_internal_client()?;
            self.printer.lookup::<ToolHead>("toolhead")?.dwell(1.0)?;
            aclient.finish_measurements()?;
            let values = aclient.get_samples();
            if values.is_empty() {
                return Err(gcmd.error("No accelerometer measurements found"));
            }
            let take = py_min(values.len() as f64, num_samples as f64) as usize;
            num_samples -= take as i64;
            samples.extend_from_slice(&values[values.len() - take..]);
        }

        let n = samples.len() as f64;
        let accel_x = py_sum(samples.iter().map(|s| s.accel_x)) / n;
        let accel_y = py_sum(samples.iter().map(|s| s.accel_y)) / n;
        let accel_z = py_sum(samples.iter().map(|s| s.accel_z)) / n;

        let return_type = gcmd.get_or("RETURN", "vector");
        if return_type == "tilt" {
            let tilt_x = accel_x.atan2(accel_z).to_degrees();
            let tilt_y = accel_y.atan2(accel_z).to_degrees();
            gcmd.respond_info(&format!("accelerometer plane tilt degrees (x, y): {:.6}, {:.6}", tilt_x, tilt_y), true);
        } else if return_type == "vector" {
            gcmd.respond_info(&format!("accelerometer values (x, y, z): {:.6}, {:.6}, {:.6}", accel_x, accel_y, accel_z), true);
        } else {
            return Err(gcmd.error(format!("Unknown 'return' type '{}'", return_type)));
        }
        Ok(())
    }

    fn get_reg_param(gcmd: &GCodeCommand, name: &str, maxval: f64) -> KResult<i64> {
        Ok(gcmd.get_parsed(name, None, py_int_base0, |v| *v as f64, &Lim::new().min(0.0).max(maxval))?.unwrap_or(0))
    }

    fn cmd_accelerometer_debug_read(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reg = Self::get_reg_param(gcmd, "REG", 127.0)?;
        let val = self.chip()?.read_reg(reg)?;
        gcmd.respond_info(&format!("Accelerometer REG[0x{:x}] = 0x{:x}", reg, val), true);
        Ok(())
    }

    fn cmd_accelerometer_debug_write(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reg = Self::get_reg_param(gcmd, "REG", 127.0)?;
        let val = Self::get_reg_param(gcmd, "VAL", 255.0)?;
        self.chip()?.set_reg(reg, val)
    }
}

/// name.replace("-", "").replace("_", "").isalnum()
pub fn is_valid_name(name: &str) -> bool {
    let s: String = name.chars().filter(|c| *c != '-' && *c != '_').collect();
    !s.is_empty() && s.chars().all(char::is_alphanumeric)
}

/// an axes_map entry: (raw axis index, scale)
pub type AxisMap = [(usize, f64); 3];

// Helper to read the axes_map parameter from the config
pub fn read_axes_map(config: &ConfigWrapper, scale_x: f64, scale_y: f64, scale_z: f64) -> KResult<AxisMap> {
    let am = |a: &str| match a {
        "x" => Some((0, scale_x)),
        "y" => Some((1, scale_y)),
        "z" => Some((2, scale_z)),
        "-x" => Some((0, -scale_x)),
        "-y" => Some((1, -scale_y)),
        "-z" => Some((2, -scale_z)),
        _ => None,
    };
    let default = PyVal::Tuple(["x", "y", "z"].iter().map(|s| PyVal::Str(s.to_string())).collect());
    let axes_map = config.getlist("axes_map", default, ",", Some(3))?.unwrap_or_default();
    if axes_map.iter().any(|a| am(a).is_none()) {
        return Err(KError::config("Invalid axes_map parameter"));
    }
    let m: Vec<(usize, f64)> = axes_map.iter().map(|a| am(a.trim()).unwrap()).collect();
    Ok([m[0], m[1], m[2]])
}

/// one converted sample: (round(ptime, 6), round(raw * scale, 6) per axis)
pub fn convert_sample(axes_map: &AxisMap, ptime: f64, raw_xyz: [i64; 3]) -> [f64; 4] {
    let v = |i: usize| py_round(raw_xyz[axes_map[i].0] as f64 * axes_map[i].1, 6);
    [py_round(ptime, 6), v(0), v(1), v(2)]
}

pub const BATCH_UPDATES: f64 = 0.100;

// Printer class that controls ADXL345 chip
pub struct ADXL345 {
    printer: Rc<Printer>,
    _cmd_helper: RefCell<Option<Rc<AccelCommandHelper>>>,
    axes_map: AxisMap,
    data_rate: i64,
    spi: Rc<McuSpi>,
    mcu: Rc<Mcu>,
    oid: i64,
    query_adxl345_cmd: RefCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    last_error_count: Cell<i64>,
    pub(crate) batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub name: String,
}

impl ADXL345 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let slot: Rc<RefCell<Weak<ADXL345>>> = Rc::new(RefCell::new(Weak::new()));
        // AccelCommandHelper(config, self): the chip is set once built
        let cmd_helper = AccelCommandHelper::new(config)?;
        let axes_map = read_axes_map(config, SCALE_XY, SCALE_XY, SCALE_Z)?;
        let data_rate = config.getint("rate", 3200, Lim::new())?;
        if !QUERY_RATES.iter().any(|(r, _)| *r == data_rate) {
            return Err(KError::config(format!("Invalid rate parameter: {}", data_rate)));
        }
        // Setup mcu sensor_adxl345 bulk query code
        let spi = mcu_spi_from_config(config, 3, "cs_pin", 5000000, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid();
        mcu.add_config_cmd(&format!("config_adxl345 oid={} spi_oid={}", oid, spi.get_oid()), false, false);
        mcu.add_config_cmd(&format!("query_adxl345 oid={} rest_ticks=0", oid), false, true);
        let sl = slot.clone();
        mcu.register_config_callback(Rc::new(move || match sl.borrow().upgrade() {
            Some(a) => a.build_config(),
            None => Ok(()),
        }));
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, "BBBBB");
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let a = Rc::new(ADXL345 {
            printer: printer.clone(),
            _cmd_helper: RefCell::new(None),
            axes_map,
            data_rate,
            spi,
            mcu,
            oid,
            query_adxl345_cmd: RefCell::new(None),
            ffreader,
            last_error_count: Cell::new(0),
            batch_bulk: RefCell::new(None),
            name: name.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&a);
        let chip: Rc<dyn AccelChip> = a.clone();
        cmd_helper.set_chip(Rc::downgrade(&chip));
        *a._cmd_helper.borrow_mut() = Some(cmd_helper);
        // Process messages in batches
        let (w1, w2, w3) = (Rc::downgrade(&a), Rc::downgrade(&a), Rc::downgrade(&a));
        let batch_bulk = BatchBulkHelper::new(
            &printer,
            Rc::new(move |eventtime| match w1.upgrade() {
                Some(a) => a.process_batch(eventtime),
                None => Ok(PyVal::dict()),
            }),
            Some(Rc::new(move || match w2.upgrade() {
                Some(a) => a.start_measurements(),
                None => Ok(()),
            })),
            Some(Rc::new(move || match w3.upgrade() {
                Some(a) => a.finish_measurements(),
                None => Ok(()),
            })),
            BATCH_UPDATES,
        );
        batch_bulk.add_mux_endpoint("adxl345/dump_adxl345", "sensor", &name, accel_header())?;
        *a.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(a)
    }

    fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        *self.query_adxl345_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_adxl345 oid=%c rest_ticks=%u", Some(cmdqueue.clone()))?);
        self.ffreader.setup_query_command("query_adxl345_status oid=%c", self.oid, Some(cmdqueue))
    }

    fn check_connected(&self) -> KResult<()> {
        if self.mcu.non_critical_disconnected.get() {
            return Err(KError::command(format!(
                "ADXL: {} could not connect because mcu: {} is non_critical_disconnected!",
                self.name,
                self.mcu.get_name()
            )));
        }
        Ok(())
    }

    fn query_cmd(&self) -> KResult<std::cell::Ref<'_, CommandWrapper>> {
        let c = self.query_adxl345_cmd.borrow();
        if c.is_none() {
            return Err(KError::Other("'NoneType' object has no attribute 'send'".into()));
        }
        Ok(std::cell::Ref::map(c, |c| c.as_ref().unwrap()))
    }

    // Measurement decoding
    fn convert_samples(&self, samples: &[RawSample]) -> Vec<[f64; 4]> {
        let mut out = Vec::with_capacity(samples.len());
        for (ptime, f) in samples {
            let (xlow, ylow, zlow, xzhigh, yzhigh) = (f[0], f[1], f[2], f[3], f[4]);
            if yzhigh & 0x80 != 0 {
                self.last_error_count.set(self.last_error_count.get() + 1);
                continue;
            }
            let rx = (xlow | ((xzhigh & 0x1F) << 8)) - ((xzhigh & 0x10) << 9);
            let ry = (ylow | ((yzhigh & 0x1F) << 8)) - ((yzhigh & 0x10) << 9);
            let rz = (zlow | ((xzhigh & 0xE0) << 3) | ((yzhigh & 0xE0) << 6)) - ((yzhigh & 0x40) << 7);
            out.push(convert_sample(&self.axes_map, *ptime, [rx, ry, rz]));
        }
        out
    }

    // Start, stop, and process message batches
    fn start_measurements(&self) -> KResult<()> {
        // In case of miswiring, testing ADXL345 device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let dev_id = self.read_reg(REG_DEVID as i64)?;
        if dev_id != ADXL345_DEV_ID as i64 {
            return Err(KError::command(format!(
                "Invalid adxl345 id (got {:x} vs {:x}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty adxl345 chip.",
                dev_id, ADXL345_DEV_ID
            )));
        }
        // Setup chip in requested query rate
        self.set_reg(REG_POWER_CTL as i64, 0x00)?;
        self.set_reg(REG_DATA_FORMAT as i64, 0x0B)?;
        self.set_reg(REG_FIFO_CTL as i64, 0x00)?;
        let rate = QUERY_RATES.iter().find(|(r, _)| *r == self.data_rate).map(|(_, v)| *v).unwrap_or(0);
        self.set_reg(REG_BW_RATE as i64, rate as i64)?;
        self.set_reg(REG_FIFO_CTL as i64, SET_FIFO_CTL as i64)?;
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(4.0 / self.data_rate as f64);
        self.query_cmd()?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        self.set_reg(REG_POWER_CTL as i64, 0x08)?;
        logging::info(&format!("ADXL345 starting '{}' measurements", self.name));
        // Initialize clock tracking
        self.ffreader.note_start()?;
        self.last_error_count.set(0);
        Ok(())
    }

    fn finish_measurements(&self) -> KResult<()> {
        // Halt bulk reading
        self.set_reg(REG_POWER_CTL as i64, 0x00)?;
        self.query_cmd()?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("ADXL345 finished '{}' measurements", self.name));
        Ok(())
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let samples = self.ffreader.pull_samples()?;
        let samples = self.convert_samples(&samples);
        Ok(batch_msg(&samples, self.last_error_count.get(), self.ffreader.get_last_overflows()))
    }
}

/// {"header": ("time", "x_acceleration", "y_acceleration", "z_acceleration")}
pub fn accel_header() -> PyVal {
    let hdr = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];
    let mut d = IndexMap::new();
    d.insert("header".to_string(), PyVal::Tuple(hdr.iter().map(|s| PyVal::Str(s.to_string())).collect()));
    PyVal::Dict(d)
}

impl AccelChip for ADXL345 {
    fn start_internal_client(&self) -> KResult<Rc<AccelQueryHelper>> {
        self.check_connected()?;
        let aqh = AccelQueryHelper::new(&self.printer)?;
        let a = aqh.clone();
        let bb = self.batch_bulk.borrow().clone().ok_or_else(|| KError::Other("adxl345 batch helper missing".into()))?;
        bb.add_client(Rc::new(move |msg| a.handle_batch(msg)))?;
        Ok(aqh)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        let params = self.spi.spi_transfer(&[reg as u8 | REG_MOD_READ, 0x00], 0, 0)?;
        match params.get("response") {
            Some(Val::Bytes(b)) if b.len() > 1 => Ok(b[1] as i64),
            _ => Err(KError::Other("bytearray index out of range".into())),
        }
    }

    fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        self.spi.spi_send(&[reg as u8, (val & 0xFF) as u8], 0, 0)?;
        let stored_val = self.read_reg(reg)?;
        if stored_val != val {
            return Err(KError::command(format!(
                "Failed to set ADXL345 register [0x{:x}] to 0x{:x}: got 0x{:x}. This is generally indicative of connection problems (e.g. faulty wiring) or a faulty adxl345 chip.",
                reg, val, stored_val
            )));
        }
        Ok(())
    }

    fn chip_name(&self) -> String {
        self.name.clone()
    }
}

impl PrinterObject for ADXL345 {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        if let Some(bb) = self.batch_bulk.borrow_mut().take() {
            bb.teardown();
        }
        *self._cmd_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ADXL345::new(config)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ADXL345::new(config)?)
}
