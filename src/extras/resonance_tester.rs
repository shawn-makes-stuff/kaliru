// A utility class to test resonances of the printer
// (port of klippy/extras/resonance_tester.py)
//
// Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{strftime_local, ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::{KError, KResult};
use crate::extras::adxl345::{as_accel_chip, is_valid_name, AccelChip, AccelQueryHelper};
use crate::extras::input_shaper::InputShaper;
use crate::extras::shaper_calibrate::{CalibrationData, CalibrationResult, ShaperCalibrate};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::kinematics::cartesian::CartKinematics;
use crate::logging;
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_max, py_sum};
use crate::pyglob::gettempdir;
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// python's repr of an AccelQueryHelper
fn aclient_repr(aclient: &Rc<AccelQueryHelper>) -> String {
    format!("<klippy.extras.adxl345.AccelQueryHelper object at {:#x}>", Rc::as_ptr(aclient) as usize)
}

/// a calibration_data[axis] that is still None
fn no_data(attr: &str) -> KError {
    KError::Py("AttributeError".into(), format!("'NoneType' object has no attribute '{}'", attr))
}

#[derive(Clone)]
pub struct TestAxis {
    name: String,
    vib_dir: (f64, f64),
}

impl TestAxis {
    fn new(axis: Option<&str>, vib_dir: Option<(f64, f64)>) -> Self {
        let name = match (axis, vib_dir) {
            (Some(a), _) => a.to_string(),
            (None, Some(d)) => format!("axis={:.3},{:.3}", d.0, d.1),
            (None, None) => String::new(),
        };
        let vib_dir = match vib_dir {
            None => {
                if axis == Some("x") {
                    (1.0, 0.0)
                } else {
                    (0.0, 1.0)
                }
            }
            Some(d) => {
                let s = py_sum([d.0 * d.0, d.1 * d.1]).sqrt();
                (d.0 / s, d.1 / s)
            }
        };
        TestAxis { name, vib_dir }
    }

    fn matches(&self, chip_axis: &str) -> bool {
        (self.vib_dir.0 != 0.0 && chip_axis.contains('x')) || (self.vib_dir.1 != 0.0 && chip_axis.contains('y'))
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn get_point(&self, l: f64) -> (f64, f64) {
        (self.vib_dir.0 * l, self.vib_dir.1 * l)
    }
}

fn parse_axis(gcmd: &GCodeCommand, raw_axis: &str) -> KResult<TestAxis> {
    let raw_axis = raw_axis.to_lowercase();
    if raw_axis == "x" || raw_axis == "y" {
        return Ok(TestAxis::new(Some(&raw_axis), None));
    }
    let dirs: Vec<&str> = raw_axis.split(',').collect();
    if dirs.len() != 2 {
        return Err(gcmd.error(format!("Invalid format of axis '{}'", raw_axis)));
    }
    match (py_float(dirs[0].trim()), py_float(dirs[1].trim())) {
        (Some(dir_x), Some(dir_y)) => Ok(TestAxis::new(None, Some((dir_x, dir_y)))),
        _ => Err(gcmd.error(format!("Unable to parse axis direction '{}'", raw_axis))),
    }
}

/// a toolhead get_status() number
fn toolhead_status(toolhead: &ToolHead, systime: f64, key: &str) -> f64 {
    match PrinterObject::get_status(toolhead, systime) {
        Some(PyVal::Dict(d)) => d.get(key).and_then(|v| v.as_f64()).unwrap_or(0.0),
        _ => 0.0,
    }
}

/// with suspend_limits(printer, max_accel, max_velocity, input_shaping): body
fn suspend_limits(printer: &Rc<Printer>, max_accel: f64, max_velocity: f64, input_shaping: bool, body: impl FnOnce() -> KResult<()>) -> KResult<()> {
    // Override maximum acceleration and cruise ratio
    // based on the maximum test frequency
    let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
    let mut input_shaper = printer.lookup_object_opt("input_shaper").and_then(downcast_rc::<InputShaper>);
    match &input_shaper {
        Some(is) if !input_shaping => {
            is.disable_shaping()?;
            gcode.respond_info("Disabled [input_shaper] for resonance testing", true);
        }
        _ => input_shaper = None,
    }
    let toolhead = printer.lookup::<ToolHead>("toolhead")?;
    let systime = printer.get_reactor().monotonic();
    let old_max_accel = toolhead_status(&toolhead, systime, "max_accel");
    let old_minimum_cruise_ratio = toolhead_status(&toolhead, systime, "minimum_cruise_ratio");
    let old_max_velocity = toolhead_status(&toolhead, systime, "max_velocity");
    gcode.run_script_from_command(&format!(
        "SET_VELOCITY_LIMIT ACCEL={:.3} MINIMUM_CRUISE_RATIO=0 VELOCITY={:.3}",
        max_accel, max_velocity
    ))?;
    let kin = toolhead.get_kinematics();
    let limited = kin.as_any().downcast_ref::<CartKinematics>().and_then(|k| k.limited.as_ref());
    let snap = limited.map(|l| l.suspend_limits(max_accel, max_velocity));
    let res = body();
    // finally:
    let fin = (|| -> KResult<()> {
        // Restore input shaper if it was disabled for resonance testing
        if let Some(is) = &input_shaper {
            is.enable_shaping()?;
            gcode.respond_info("Re-enabled [input_shaper]", true);
        }
        // Restore the original acceleration values
        gcode.run_script_from_command(&format!(
            "SET_VELOCITY_LIMIT ACCEL={:.3} MINIMUM_CRUISE_RATIO={:.3} VELOCITY={:.3}",
            old_max_accel, old_minimum_cruise_ratio, old_max_velocity
        ))?;
        if let (Some(l), Some(s)) = (limited, &snap) {
            l.restore_limits(s);
        }
        Ok(())
    })();
    res?;
    fin
}

/// a test sequence entry: (time, accel, freq)
type TestSeq = Vec<(f64, f64, f64)>;

struct VibrationPulseTestGenerator {
    min_freq: f64,
    max_freq: f64,
    accel_per_hz: f64,
    hz_per_sec: f64,
    freq_start: Cell<f64>,
    freq_end: Cell<f64>,
    test_accel_per_hz: Cell<f64>,
    test_hz_per_sec: Cell<f64>,
}

impl VibrationPulseTestGenerator {
    fn new(config: &ConfigWrapper) -> KResult<Self> {
        let min_freq = config.getfloat("min_freq", 5.0, Lim::new().min(1.0))?;
        // Defaults are such that max_freq * accel_per_hz == 10000 (max_accel)
        let max_freq = config.getfloat("max_freq", 135.0, Lim::new().min(min_freq).max(300.0))?;
        let accel_per_hz = config.getfloat("accel_per_hz", 75.0, Lim::new().above(0.0))?;
        let hz_per_sec = config.getfloat("hz_per_sec", 1.0, Lim::new().min(0.1).max(2.0))?;
        Ok(VibrationPulseTestGenerator {
            min_freq,
            max_freq,
            accel_per_hz,
            hz_per_sec,
            freq_start: Cell::new(min_freq),
            freq_end: Cell::new(max_freq),
            test_accel_per_hz: Cell::new(accel_per_hz),
            test_hz_per_sec: Cell::new(hz_per_sec),
        })
    }

    fn prepare_test(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.freq_start.set(gcmd.get_float("FREQ_START", Some(self.min_freq), Lim::new().min(1.0))?);
        self.freq_end.set(gcmd.get_float("FREQ_END", Some(self.max_freq), Lim::new().min(self.freq_start.get()).max(300.0))?);
        self.test_accel_per_hz.set(gcmd.get_float("ACCEL_PER_HZ", Some(self.accel_per_hz), Lim::new().above(0.0))?);
        self.test_hz_per_sec.set(gcmd.get_float("HZ_PER_SEC", Some(self.hz_per_sec), Lim::new().above(0.0).max(2.0))?);
        Ok(())
    }

    fn get_max_freq(&self) -> f64 {
        self.freq_end.get()
    }

    fn get_accel_per_hz(&self) -> f64 {
        // The value in effect for the current test (including any
        // ACCEL_PER_HZ override), not the configured default
        self.test_accel_per_hz.get()
    }

    fn gen_test(&self) -> TestSeq {
        let mut freq = self.freq_start.get();
        let mut res = vec![];
        let mut sign = 1.0;
        let mut time = 0.0;
        while freq <= self.freq_end.get() + 0.000001 {
            let t_seg = 0.25 / freq;
            let accel = self.test_accel_per_hz.get() * freq;
            time += t_seg;
            res.push((time, sign * accel, freq));
            time += t_seg;
            res.push((time, -sign * accel, freq));
            freq += 2.0 * t_seg * self.test_hz_per_sec.get();
            sign = -sign;
        }
        res
    }
}

struct SweepingVibrationsTestGenerator {
    vibration_generator: VibrationPulseTestGenerator,
    sweeping_accel: f64,
    sweeping_period: f64,
    test_sweeping_accel: Cell<f64>,
    test_sweeping_period: Cell<f64>,
}

impl SweepingVibrationsTestGenerator {
    fn new(config: &ConfigWrapper) -> KResult<Self> {
        let vibration_generator = VibrationPulseTestGenerator::new(config)?;
        let sweeping_accel = config.getfloat("sweeping_accel", 400.0, Lim::new().above(0.0))?;
        let sweeping_period = config.getfloat("sweeping_period", 0.0, Lim::new().min(0.0))?;
        Ok(SweepingVibrationsTestGenerator {
            vibration_generator,
            sweeping_accel,
            sweeping_period,
            test_sweeping_accel: Cell::new(sweeping_accel),
            test_sweeping_period: Cell::new(sweeping_period),
        })
    }

    fn prepare_test(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.vibration_generator.prepare_test(gcmd)?;
        self.test_sweeping_accel.set(gcmd.get_float("SWEEPING_ACCEL", Some(self.sweeping_accel), Lim::new().above(0.0))?);
        self.test_sweeping_period.set(gcmd.get_float("SWEEPING_PERIOD", Some(self.sweeping_period), Lim::new().min(0.0))?);
        Ok(())
    }

    fn gen_test(&self) -> TestSeq {
        let test_seq = self.vibration_generator.gen_test();
        let mut accel_fraction = 2.0f64.sqrt() * 0.125;
        let (mut t_rem, sweeping_accel) = if self.test_sweeping_period.get() != 0.0 {
            (self.test_sweeping_period.get() * accel_fraction, self.test_sweeping_accel.get())
        } else {
            (f64::INFINITY, 0.0)
        };
        let mut res = vec![];
        let mut last_t = 0.0;
        let mut sig = 1.0;
        accel_fraction += 0.25;
        for (next_t, accel, freq) in test_seq {
            let mut t_seg = next_t - last_t;
            while t_rem <= t_seg {
                last_t += t_rem;
                res.push((last_t, accel + sweeping_accel * sig, freq));
                t_seg -= t_rem;
                t_rem = self.test_sweeping_period.get() * accel_fraction;
                accel_fraction = 0.5;
                sig = -sig;
            }
            t_rem -= t_seg;
            res.push((next_t, accel + sweeping_accel * sig, freq));
            last_t = next_t;
        }
        res
    }
}

struct ResonanceTestExecutor {
    printer: Rc<Printer>,
}

impl ResonanceTestExecutor {
    fn run_test(&self, test_seq: &TestSeq, axis: &TestAxis, gcmd: &GCodeCommand) -> KResult<()> {
        // Derive the required limits from the actual test sequence, so that
        // ACCEL_PER_HZ overrides and the sweeping acceleration are covered
        let mut max_accel = 0.0;
        let (mut max_v, mut v, mut last_t) = (0.0, 0.0, 0.0);
        for &(next_t, accel, _freq) in test_seq {
            max_accel = py_max(max_accel, accel.abs());
            v += accel * (next_t - last_t);
            max_v = py_max(max_v, f64::abs(v));
            last_t = next_t;
        }
        let input_shaping = gcmd.get_int("INPUT_SHAPING", Some(0), Lim::new())? != 0;
        suspend_limits(&self.printer, max_accel + 10.0, max_v + 1.0, input_shaping, || self.do_run_test(test_seq, axis, gcmd))
    }

    fn do_run_test(&self, test_seq: &TestSeq, axis: &TestAxis, gcmd: &GCodeCommand) -> KResult<()> {
        let reactor = self.printer.get_reactor();
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let tpos = toolhead.get_position();
        let (mut x, mut y) = (tpos[0], tpos[1]);
        let pos = |px: f64, py: f64| -> Vec<f64> {
            let mut p = vec![px, py];
            p.extend_from_slice(&tpos[2..]);
            p
        };
        // Override maximum acceleration and acceleration to
        // deceleration based on the maximum test frequency
        let systime = reactor.monotonic();
        let old_max_accel = toolhead_status(&toolhead, systime, "max_accel");
        let (mut last_v, mut last_t, mut last_freq) = (0.0f64, 0.0, 0.0f64);
        for &(next_t, accel, freq) in test_seq {
            let t_seg = next_t - last_t;
            toolhead.set_max_velocities(None, Some(accel.abs()), None, None);
            let mut v = last_v + accel * t_seg;
            let mut abs_v = v.abs();
            if abs_v < 0.000001 {
                v = 0.0;
                abs_v = 0.0;
            }
            let abs_last_v = last_v.abs();
            let v2 = v * v;
            let last_v2 = last_v * last_v;
            let half_inv_accel = 0.5 / accel;
            let d = (v2 - last_v2) * half_inv_accel;
            let (dx, dy) = axis.get_point(d);
            let nx = x + dx;
            let ny = y + dy;
            toolhead.limit_next_junction_speed(abs_last_v);
            if v * last_v < 0.0 {
                // The move first goes to a complete stop, then changes direction
                let d_decel = -last_v2 * half_inv_accel;
                let (decel_x, decel_y) = axis.get_point(d_decel);
                toolhead.move_to(&pos(x + decel_x, y + decel_y), abs_last_v)?;
                toolhead.move_to(&pos(nx, ny), abs_v)?;
            } else {
                toolhead.move_to(&pos(nx, ny), py_max(abs_v, abs_last_v))?;
            }
            if freq.floor() > last_freq.floor() {
                gcmd.respond_info(&format!("Testing frequency {:.0} Hz", freq), true);
                reactor.pause(reactor.monotonic() + 0.01)?;
            }
            x = nx;
            y = ny;
            last_t = next_t;
            last_v = v;
            last_freq = freq;
        }
        if last_v != 0.0 {
            // Decelerate to a stop, continuing in the direction of motion
            let d_decel = 0.5 * last_v * last_v.abs() / old_max_accel;
            let (decel_x, decel_y) = axis.get_point(d_decel);
            toolhead.set_max_velocities(None, Some(old_max_accel), None, None);
            toolhead.move_to(&pos(x + decel_x, y + decel_y), last_v.abs())?;
        }
        Ok(())
    }
}

const CMD_TEST_RESONANCES_HELP: &str = "Runs the resonance test for a specified axis";
const CMD_SHAPER_CALIBRATE_HELP: &str = "Similar to TEST_RESONANCES but suggest input shaper config";
const CMD_MEASURE_AXES_NOISE_HELP: &str = "Measures noise of all enabled accelerometer chips";

pub struct ResonanceTester {
    printer: Rc<Printer>,
    move_speed: f64,
    generator: SweepingVibrationsTestGenerator,
    executor: ResonanceTestExecutor,
    accel_chip_names: Vec<(String, String)>,
    accel_chips: RefCell<Vec<(String, PObj)>>,
    max_smoothing: Option<f64>,
    probe_points: Vec<Vec<f64>>,
}

/// an object as an accelerometer chip (python duck types it)
fn accel_chip(obj: &PObj) -> KResult<Rc<dyn AccelChip>> {
    as_accel_chip(obj).ok_or_else(|| KError::Other("object has no attribute 'start_internal_client'".into()))
}

impl ResonanceTester {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let move_speed = config.getfloat("move_speed", 50.0, Lim::new().above(0.0))?;
        let generator = SweepingVibrationsTestGenerator::new(config)?;
        let executor = ResonanceTestExecutor { printer: printer.clone() };

        let accel_chips = config.get_opt("accel_chips")?;
        let accel_chip = config.get_opt("accel_chip")?;
        let accel_chip_x = config.get_opt("accel_chip_x")?;
        let accel_chip_y = config.get_opt("accel_chip_y")?;

        // priority: accel_chips > accel_chip_x/y > accel_chip
        let accel_chip_names: Vec<(String, String)> = if let Some(chips) = accel_chips {
            // Parse comma-separated list of chips
            chips.split(',').map(|chip| ("xy".to_string(), chip.trim().to_string())).collect()
        } else if let Some(x) = accel_chip_x {
            let y = accel_chip_y.ok_or_else(|| KError::Other("'NoneType' object has no attribute 'strip'".into()))?;
            let names = vec![("x".to_string(), x.trim().to_string()), ("y".to_string(), y.trim().to_string())];
            if names[0].1 == names[1].1 {
                vec![("xy".to_string(), names[0].1.clone())]
            } else {
                names
            }
        } else if let Some(c) = accel_chip {
            vec![("xy".to_string(), c.trim().to_string())]
        } else {
            return Err(KError::config(
                "No accelerometer chips configured. At least one of accel_chips, accel_chip, or accel_chip_x/accel_chip_y must be specified.",
            ));
        };

        let max_smoothing = config.getfloat_opt("max_smoothing", Lim::new().min(0.05))?;
        let v = config.getlists("probe_points", Dflt::Required, &[",", "\n"], Some(3), ListParser::Float, true)?;
        let probe_points: Vec<Vec<f64>> = v
            .as_ref()
            .and_then(|v| v.items())
            .unwrap_or(&[])
            .iter()
            .map(|p| p.items().unwrap_or(&[]).iter().map(|x| x.as_f64().unwrap_or(0.0)).collect())
            .collect();

        let rt = Rc::new(ResonanceTester {
            printer: printer.clone(),
            move_speed,
            generator,
            executor,
            accel_chip_names,
            accel_chips: RefCell::new(vec![]),
            max_smoothing,
            probe_points,
        });

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type RCmd = fn(&ResonanceTester, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, RCmd, &str); 3] = [
            ("MEASURE_AXES_NOISE", ResonanceTester::cmd_measure_axes_noise, CMD_MEASURE_AXES_NOISE_HELP),
            ("TEST_RESONANCES", ResonanceTester::cmd_test_resonances, CMD_TEST_RESONANCES_HELP),
            ("SHAPER_CALIBRATE", ResonanceTester::cmd_shaper_calibrate, CMD_SHAPER_CALIBRATE_HELP),
        ];
        for (name, f, desc) in cmds {
            let w = Rc::downgrade(&rt);
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(r) => f(&r, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        let w = Rc::downgrade(&rt);
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match w.upgrade() {
                Some(r) => r.connect(),
                None => Ok(()),
            }),
        );
        Ok(rt)
    }

    fn connect(&self) -> KResult<()> {
        let mut chips = vec![];
        for (chip_axis, chip_name) in &self.accel_chip_names {
            match self.printer.lookup_object(chip_name) {
                Ok(chip) => chips.push((chip_axis.clone(), chip)),
                Err(e) => {
                    logging::exception_err(&format!("Error looking up accelerometer chip '{}': {}", chip_name, e), &e);
                    return Err(e);
                }
            }
        }
        *self.accel_chips.borrow_mut() = chips;
        Ok(())
    }

    /// _run_test(gcmd, axes, helper, ...): the calibration data per axis
    fn run_test(
        &self,
        gcmd: &GCodeCommand,
        axes: &[TestAxis],
        helper: Option<&ShaperCalibrate>,
        raw_name_suffix: Option<&str>,
        accel_chips: Option<&[PObj]>,
        test_point: Option<&[f64]>,
    ) -> KResult<Vec<Option<CalibrationData>>> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut calibration_data: Vec<Option<CalibrationData>> = vec![None; axes.len()];

        // Reads all test overrides (FREQ_START/END, ACCEL_PER_HZ, ...)
        // from gcmd
        self.generator.prepare_test(gcmd)?;

        let test_points: Vec<Vec<f64>> = match test_point {
            Some(p) => vec![p.to_vec()],
            None => self.probe_points.clone(),
        };

        for point in &test_points {
            let coord: Vec<Option<f64>> = point.iter().map(|v| Some(*v)).collect();
            toolhead.manual_move(&coord, self.move_speed)?;
            if test_points.len() > 1 || test_point.is_some() {
                gcmd.respond_info(&format!("Probing point ({:.3}, {:.3}, {:.3})", point[0], point[1], point[2]), true);
            }
            for (ai, axis) in axes.iter().enumerate() {
                toolhead.wait_moves()?;
                toolhead.dwell(0.500)?;
                if axes.len() > 1 {
                    gcmd.respond_info(&format!("Testing axis {}", axis.get_name()), true);
                }

                let mut raw_values: Vec<(Rc<AccelQueryHelper>, String)> = vec![];
                match accel_chips {
                    None => {
                        for (chip_axis, chip) in self.accel_chips.borrow().iter() {
                            if axis.matches(chip_axis) {
                                let chip = accel_chip(chip)?;
                                raw_values.push((chip.start_internal_client()?, chip.chip_name()));
                            }
                        }
                    }
                    Some(chips) => {
                        for chip in chips {
                            let chip = accel_chip(chip)?;
                            raw_values.push((chip.start_internal_client()?, chip.chip_name()));
                        }
                    }
                }

                // Generate moves
                let test_seq = self.generator.gen_test();
                self.executor.run_test(&test_seq, axis, gcmd)?;
                for (aclient, chip_name) in &raw_values {
                    aclient.finish_measurements()?;
                    if let Some(suffix) = raw_name_suffix {
                        let p = if test_points.len() > 1 { Some(point.as_slice()) } else { None };
                        let raw_name = get_filename("raw_data", suffix, Some(axis), p, Some(chip_name));
                        aclient.write_to_file(&raw_name);
                        gcmd.respond_info(&format!("Writing raw accelerometer data to {} file", raw_name), true);
                    }
                }
                let Some(helper) = helper else { continue };
                for (aclient, chip_name) in &raw_values {
                    if !aclient.has_valid_samples() {
                        return Err(gcmd.error(format!("accelerometer '{}' measured no data", chip_name)));
                    }
                    let new_data = helper.process_accelerometer_data(aclient.get_samples(), &aclient_repr(aclient))?;
                    match &mut calibration_data[ai] {
                        None => calibration_data[ai] = Some(new_data),
                        Some(d) => d.add_data(&new_data),
                    }
                }
            }
        }
        Ok(calibration_data)
    }

    fn get_max_calibration_freq(&self) -> f64 {
        1.5 * self.generator.vibration_generator.get_max_freq()
    }

    #[allow(clippy::too_many_arguments)]
    fn save_calibration_data(
        &self,
        base_name: &str,
        name_suffix: &str,
        shaper_calibrate: &ShaperCalibrate,
        axis: &TestAxis,
        calibration_data: Option<&CalibrationData>,
        all_shapers: Option<&[CalibrationResult]>,
        point: Option<&[f64]>,
        max_freq: f64,
    ) -> KResult<String> {
        let output = get_filename(base_name, name_suffix, Some(axis), point, None);
        let data = calibration_data.ok_or_else(|| no_data("freq_bins"))?;
        let accel_per_hz = self.generator.vibration_generator.get_accel_per_hz();
        shaper_calibrate.save_calibration_data(&output, data, all_shapers, Some(max_freq), accel_per_hz)?;
        Ok(output)
    }

    fn parse_chips(&self, accel_chips: &str) -> KResult<Vec<PObj>> {
        accel_chips.split(',').map(|chip_name| self.printer.lookup_object(chip_name.trim())).collect()
    }

    fn cmd_test_resonances(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Parse parameters
        let axis = parse_axis(gcmd, &gcmd.get("AXIS")?.to_lowercase())?;
        let chips_str = gcmd.get_opt("CHIPS");
        let test_point = match gcmd.get_opt("POINT").filter(|p| !p.is_empty()) {
            None => None,
            Some(tp) => {
                let test_coords: Vec<&str> = tp.split(',').collect();
                if test_coords.len() != 3 {
                    return Err(gcmd.error("Invalid POINT parameter, must be 'x,y,z'"));
                }
                let parsed: Option<Vec<f64>> = test_coords.iter().map(|p| py_float(p.trim())).collect();
                match parsed {
                    Some(p) => Some(p),
                    None => {
                        return Err(gcmd.error(
                            "Invalid POINT parameter, must be 'x,y,z' where x, y and z are valid floating point numbers",
                        ))
                    }
                }
            }
        };

        let accel_chips = match chips_str.filter(|c| !c.is_empty()) {
            Some(c) => Some(self.parse_chips(&c)?),
            None => None,
        };

        let outputs: Vec<String> = gcmd.get_or("OUTPUT", "resonances").to_lowercase().split(',').map(|s| s.to_string()).collect();
        for output in &outputs {
            if output != "resonances" && output != "raw_data" {
                return Err(gcmd.error(format!("Unsupported output '{}', only 'resonances' and 'raw_data' are supported", output)));
            }
        }
        let name_suffix = gcmd.get_or("NAME", &strftime_local("%Y%m%d_%H%M%S"));
        if !is_valid_name(&name_suffix) {
            return Err(gcmd.error("Invalid NAME parameter"));
        }
        let csv_output = outputs.iter().any(|o| o == "resonances");
        let raw_output = outputs.iter().any(|o| o == "raw_data");

        // Check for active fans and display warning if found
        self.check_active_fans(gcmd);

        // Setup calculation of resonances
        let helper = if csv_output { Some(ShaperCalibrate::new(&self.printer)) } else { None };

        let data = self.run_test(
            gcmd,
            std::slice::from_ref(&axis),
            helper.as_ref(),
            if raw_output { Some(&name_suffix) } else { None },
            accel_chips.as_deref(),
            test_point.as_deref(),
        )?;
        if let Some(helper) = &helper {
            let max_freq = self.get_max_calibration_freq();
            let csv_name = self.save_calibration_data("resonances", &name_suffix, helper, &axis, data[0].as_ref(), None, test_point.as_deref(), max_freq)?;
            gcmd.respond_info(&format!("Resonances data written to {} file", csv_name), true);
        }
        Ok(())
    }

    fn cmd_shaper_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Parse parameters
        let calibrate_axes = match gcmd.get_opt("AXIS").filter(|a| !a.is_empty()) {
            None => vec![TestAxis::new(Some("x"), None), TestAxis::new(Some("y"), None)],
            Some(axis) if axis.to_lowercase() != "x" && axis.to_lowercase() != "y" => {
                return Err(gcmd.error(format!("Unsupported axis '{}'", axis)));
            }
            Some(axis) => vec![TestAxis::new(Some(&axis.to_lowercase()), None)],
        };
        let accel_chips = match gcmd.get_opt("CHIPS").filter(|c| !c.is_empty()) {
            Some(c) => Some(self.parse_chips(&c)?),
            None => None,
        };
        let lim = Lim::new().min(0.05);
        let max_smoothing = match self.max_smoothing {
            Some(m) => Some(gcmd.get_float("MAX_SMOOTHING", Some(m), lim)?),
            None => gcmd.get_float_opt("MAX_SMOOTHING", lim)?,
        };
        // COSMOS 0001-force-specific-input-shaper
        let force_shaper = gcmd.get_opt("FORCE_SHAPER").map(|s| s.to_lowercase());
        if let Some(f) = &force_shaper {
            if !crate::extras::shaper_defs::INPUT_SHAPERS.iter().any(|s| s.name == f) {
                return Err(gcmd.error(format!("Unsupported shaper type: {}", f)));
            }
        }
        let name_suffix = gcmd.get_or("NAME", &strftime_local("%Y%m%d_%H%M%S"));
        if !is_valid_name(&name_suffix) {
            return Err(gcmd.error("Invalid NAME parameter"));
        }

        let input_shaper = self.printer.lookup_object_opt("input_shaper").and_then(downcast_rc::<InputShaper>);

        // Check for active fans and display warning if found
        self.check_active_fans(gcmd);

        // Setup shaper calibration
        let helper = ShaperCalibrate::new(&self.printer);

        let mut calibration_data = self.run_test(gcmd, &calibrate_axes, Some(&helper), None, accel_chips.as_deref(), None)?;

        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        for (ai, axis) in calibrate_axes.iter().enumerate() {
            let axis_name = axis.get_name().to_string();
            gcmd.respond_info(&format!("Calculating the best input shaper parameters for {} axis", axis_name), true);
            let data = calibration_data[ai].as_mut().ok_or_else(|| no_data("normalize_to_frequencies"))?;
            data.normalize_to_frequencies();
            let systime = self.printer.get_reactor().monotonic();
            let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
            let scv = toolhead_status(&toolhead, systime, "square_corner_velocity");
            let max_freq = self.get_max_calibration_freq();
            let logger = |m: &str| gcmd.respond_info(m, true);
            let (best_shaper, all_shapers) = helper.find_best_shaper(data, force_shaper.as_ref().map(std::slice::from_ref), scv, max_smoothing, Some(max_freq), Some(&logger))?;
            gcmd.respond_info(
                &format!(
                    "Recommended shaper_type_{} = {}, shaper_freq_{} = {:.1} Hz",
                    axis_name, best_shaper.name, axis_name, best_shaper.freq
                ),
                true,
            );
            if let Some(input_shaper) = &input_shaper {
                // apply_params
                let upper = axis_name.to_uppercase();
                let mut params = indexmap::IndexMap::new();
                params.insert(format!("SHAPER_TYPE_{}", upper), best_shaper.name.to_string());
                params.insert(format!("SHAPER_FREQ_{}", upper), crate::pyfmt::py_float_repr(best_shaper.freq));
                input_shaper.cmd_set_input_shaper(&gcode.create_gcode_command("SET_INPUT_SHAPER", "SET_INPUT_SHAPER", params))?;
            }
            // save_params
            {
                let mut cf = configfile.pconfig.borrow_mut();
                cf.set("input_shaper", &format!("shaper_type_{}", axis_name), best_shaper.name)?;
                cf.set("input_shaper", &format!("shaper_freq_{}", axis_name), &format!("{:.1}", best_shaper.freq))?;
            }
            let csv_name =
                self.save_calibration_data("calibration_data", &name_suffix, &helper, axis, calibration_data[ai].as_ref(), Some(&all_shapers), None, max_freq)?;
            gcmd.respond_info(&format!("Shaper calibration data written to {} file", csv_name), true);
        }
        gcmd.respond_info("The SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer.", true);
        Ok(())
    }

    fn cmd_measure_axes_noise(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let meas_time = gcmd.get_float("MEAS_TIME", Some(2.0), Lim::new().above(0.0))?;
        let mut raw_values = vec![];
        for (chip_axis, chip) in self.accel_chips.borrow().iter() {
            raw_values.push((chip_axis.clone(), accel_chip(chip)?.start_internal_client()?));
        }
        self.printer.lookup::<ToolHead>("toolhead")?.dwell(meas_time)?;
        for (_, aclient) in &raw_values {
            aclient.finish_measurements()?;
        }
        let helper = ShaperCalibrate::new(&self.printer);
        for (chip_axis, aclient) in &raw_values {
            if !aclient.has_valid_samples() {
                return Err(gcmd.error(format!("{}-axis accelerometer measured no data", chip_axis)));
            }
            let data = helper.process_accelerometer_data(aclient.get_samples(), &aclient_repr(aclient))?;
            let mean = |v: &[f64]| crate::mathutil::np_pairwise_sum(v) / v.len() as f64;
            gcmd.respond_info(
                &format!(
                    "Axes noise for {}-axis accelerometer: {:.6} (x), {:.6} (y), {:.6} (z)",
                    chip_axis,
                    mean(&data.psd_x),
                    mean(&data.psd_y),
                    mean(&data.psd_z)
                ),
                true,
            );
        }
        Ok(())
    }

    fn check_active_fans(&self, gcmd: &GCodeCommand) {
        let mut active_fans = vec![];
        for (name, obj) in self.printer.lookup_objects(None) {
            let module_name = name.split(' ').next().unwrap_or("");
            if !module_name.to_lowercase().contains("fan") || !obj.has_status() {
                continue;
            }
            let speed = match obj.get_status(self.printer.get_reactor().monotonic()) {
                Some(PyVal::Dict(d)) => d.get("speed").and_then(|v| v.as_f64()).unwrap_or(0.0),
                _ => continue,
            };
            if speed > 0.0 {
                let fan_name = if name == "fan" {
                    "fan".to_string()
                } else {
                    name.split_once(' ').map(|(_, n)| n.to_string()).unwrap_or(name.clone())
                };
                active_fans.push(fan_name);
            }
        }
        if !active_fans.is_empty() {
            gcmd.respond_info(
                &format!(
                    "WARNING: Active fans detected: {}\nFan vibrations may affect input shaper calibration accuracy.\nFor best results, stop fans or wait for them to stop automatically.",
                    active_fans.join(", ")
                ),
                true,
            );
        }
    }
}

fn get_filename(base: &str, name_suffix: &str, axis: Option<&TestAxis>, point: Option<&[f64]>, chip_name: Option<&str>) -> String {
    let mut name = base.to_string();
    if let Some(axis) = axis {
        name += &format!("_{}", axis.get_name());
    }
    if let Some(chip_name) = chip_name.filter(|c| !c.is_empty()) {
        name += &format!("_{}", chip_name.replace(' ', "_"));
    }
    if let Some(p) = point {
        name += &format!("_{:.3}_{:.3}_{:.3}", p[0], p[1], p[2]);
    }
    name += &format!("_{}", name_suffix);
    format!("{}/{}.csv", gettempdir(), name)
}

impl PrinterObject for ResonanceTester {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        self.accel_chips.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ResonanceTester::new(config)?)
}
