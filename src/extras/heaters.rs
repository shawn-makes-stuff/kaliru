// Tracking of PWM controlled heaters and their temperature control
// (port of klippy/extras/heaters.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The python lock is not needed: temperature callbacks run on the reactor
// thread here (serial responses are dispatched there).
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::control_mpc::{ControlMPC, FILAMENT_TEMP_SRC_AMBIENT, FILAMENT_TEMP_SRC_FIXED, FILAMENT_TEMP_SRC_SENSOR};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::PwmPin;
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::Reactor;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const KELVIN_TO_CELSIUS: f64 = -273.15;
pub const MAX_HEAT_TIME: f64 = 3.0;
pub const AMBIENT_TEMP: f64 = 25.0;
pub const PID_PARAM_BASE: f64 = 255.0;
pub const MAX_MAINTHREAD_TIME: f64 = 5.0;
pub const QUELL_STALE_TIME: f64 = 7.0;
pub const MIN_PWM_CHANGE_RATIO: f64 = 0.05;
pub const PID_PROFILE_VERSION: i64 = 1;
/// PID_PROFILE_OPTIONS: (key, is_float (else str), placeholder)
pub const PID_PROFILE_OPTIONS: &[(&str, bool, &str)] = &[
    ("pid_target", true, "%.2f"),
    ("pid_tolerance", true, "%.4f"),
    ("control", false, "%s"),
    ("smooth_time", true, "%.3f"),
    ("pid_kp", true, "%.3f"),
    ("pid_ki", true, "%.3f"),
    ("pid_kd", true, "%.3f"),
];
pub const PID_CONTROL_TYPES: &[&str] = &["pid", "pid_v", "dual_loop_pid"];
pub const DUAL_LOOP_PID_INNER_TARGET_OPTION: &str = "inner_target_temp";
pub const DUAL_LOOP_PID_INNER_TARGET_DEPRECATED_OPTION: &str = "inner_max_temp";

use crate::pyfmt::{py_max, py_min};

/// "%x" % value for the PID_PROFILE placeholders
fn placeholder_fmt(placeholder: &str, v: &PyVal) -> KResult<String> {
    crate::pyformat::percent_format(placeholder, v).map_err(|(class, m)| KError::Py(class.into(), m))
}

pub fn lookup_dual_loop_pid_inner_target_temp(config: &ConfigWrapper) -> KResult<f64> {
    let inner_target_temp = config.getfloat_opt(DUAL_LOOP_PID_INNER_TARGET_OPTION, Lim::new())?;
    let inner_max_temp = config.getfloat_opt(DUAL_LOOP_PID_INNER_TARGET_DEPRECATED_OPTION, Lim::new())?;
    if inner_target_temp.is_some() && inner_max_temp.is_some() {
        return Err(KError::config(format!(
            "Options '{}' and '{}' may not both be specified",
            DUAL_LOOP_PID_INNER_TARGET_OPTION, DUAL_LOOP_PID_INNER_TARGET_DEPRECATED_OPTION
        )));
    }
    if let Some(t) = inner_target_temp {
        return Ok(t);
    }
    if let Some(t) = inner_max_temp {
        config.deprecate(DUAL_LOOP_PID_INNER_TARGET_DEPRECATED_OPTION, None);
        return Ok(t);
    }
    Ok(config.getfloat(DUAL_LOOP_PID_INNER_TARGET_OPTION, Dflt::Required, Lim::new())?)
}

// ----------------------------------------------------------------------
// Sensors and profiles
// ----------------------------------------------------------------------

/// a temperature sensor as a heater uses it (thermistors, adc sensors, ...)
pub trait TempSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()>;
    fn setup_callback(&self, cb: TempCallback);
    fn get_report_time_delta(&self) -> f64;
    fn as_any(&self) -> &dyn Any;
}

/// a temperature callback: cb(read_time, temp)
pub type TempCallback = Rc<dyn Fn(f64, f64) -> KResult<()>>;
pub type SensorFactory = Rc<dyn Fn(&ConfigWrapper) -> KResult<Rc<dyn TempSensor>>>;

/// a profile value: a python value or an object (ambient sensor, fan)
#[derive(Clone)]
pub enum ProfVal {
    V(PyVal),
    Obj(Option<PObj>),
}

/// a heater profile (a python dict shared between the profile manager and
/// the control built from it)
pub type Profile = Rc<RefCell<IndexMap<String, ProfVal>>>;

pub fn prof_get(p: &Profile, key: &str) -> PyVal {
    match p.borrow().get(key) {
        Some(ProfVal::V(v)) => v.clone(),
        _ => PyVal::None,
    }
}

pub fn prof_f64(p: &Profile, key: &str) -> Option<f64> {
    prof_get(p, key).as_f64()
}

pub fn prof_str(p: &Profile, key: &str) -> String {
    prof_get(p, key).str()
}

pub fn prof_obj(p: &Profile, key: &str) -> Option<PObj> {
    match p.borrow().get(key) {
        Some(ProfVal::Obj(o)) => o.clone(),
        _ => None,
    }
}

pub fn prof_set(p: &Profile, key: &str, v: PyVal) {
    p.borrow_mut().insert(key.to_string(), ProfVal::V(v));
}

fn new_profile(name: &str) -> Profile {
    let p: Profile = Rc::new(RefCell::new(IndexMap::new()));
    prof_set(&p, "name", PyVal::from(name));
    p
}

/// a pid value that must be present (python divides it: TypeError on None)
fn pid_param(p: &Profile, key: &str) -> KResult<f64> {
    prof_f64(p, key).ok_or_else(|| KError::Other("unsupported operand type(s) for /: 'NoneType' and 'float'".into()))
}

// ----------------------------------------------------------------------
// Controls
// ----------------------------------------------------------------------

/// the temperature control algorithm of a heater
pub trait Control {
    /// temperature_update(read_time, temp, target_temp[, secondary_temp])
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()>;
    fn check_busy(&self, eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool;
    fn update_smooth_time(&self) -> KResult<()> {
        Ok(())
    }
    fn get_profile(&self) -> Profile;
    fn get_type(&self) -> &'static str;
    /// get_status(); None where the control has none
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        None
    }
    /// check_valid() (MPC); python only calls it where it exists
    fn check_valid(&self) -> KResult<()> {
        Ok(())
    }
    fn as_any(&self) -> &dyn Any;
}

pub type ControlRef = Rc<dyn Control>;

/// the TypeError of calling a 3 argument temperature_update with 4
pub fn no_secondary(cls: &str, secondary_temp: Option<f64>) -> KResult<()> {
    if secondary_temp.is_some() {
        return Err(KError::Other(format!("{}.temperature_update() takes 4 positional arguments but 5 were given", cls)));
    }
    Ok(())
}

// Bang-bang control algo
pub struct ControlBangBang {
    profile: Profile,
    heater: Rc<Heater>,
    heater_max_power: f64,
    max_delta: f64,
    heating: Cell<bool>,
}

impl ControlBangBang {
    pub fn new(profile: &Profile, heater: &Rc<Heater>) -> KResult<Self> {
        let max_delta = prof_f64(profile, "max_delta").unwrap_or(0.0);
        Ok(ControlBangBang { profile: profile.clone(), heater: heater.clone(), heater_max_power: heater.get_max_power(), max_delta, heating: Cell::new(false) })
    }
}

impl Control for ControlBangBang {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        no_secondary("ControlBangBang", secondary_temp)?;
        if self.heating.get() && temp >= target_temp + self.max_delta {
            self.heating.set(false);
        } else if !self.heating.get() && temp <= target_temp - self.max_delta {
            self.heating.set(true);
        }
        if self.heating.get() {
            self.heater.set_pwm(read_time, self.heater_max_power)
        } else {
            self.heater.set_pwm(read_time, 0.0)
        }
    }
    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        smoothed_temp < target_temp - self.max_delta
    }
    fn get_profile(&self) -> Profile {
        self.profile.clone()
    }
    fn get_type(&self) -> &'static str {
        "watermark"
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

// Proportional Integral Derivative (PID) control algo
pub const PID_SETTLE_DELTA: f64 = 1.0;
pub const PID_SETTLE_SLOPE: f64 = 0.1;

pub struct ControlPID {
    profile: Profile,
    heater: Rc<Heater>,
    heater_max_power: f64,
    pub kp: Cell<f64>,
    pub ki: Cell<f64>,
    pub kd: Cell<f64>,
    min_deriv_time: f64,
    temp_integ_max: Cell<f64>,
    prev_temp: Cell<f64>,
    prev_temp_time: Cell<f64>,
    prev_temp_deriv: Cell<f64>,
    pub prev_temp_integ: Cell<f64>,
    /// "pid" or the ControlInnerPID of a dual loop
    kind: &'static str,
}

impl ControlPID {
    pub fn new(profile: &Profile, heater: &Rc<Heater>, load_clean: bool) -> KResult<Self> {
        let heater_max_power = heater.get_max_power();
        let kp = pid_param(profile, "pid_kp")? / PID_PARAM_BASE;
        let ki = pid_param(profile, "pid_ki")? / PID_PARAM_BASE;
        let kd = pid_param(profile, "pid_kd")? / PID_PARAM_BASE;
        let min_deriv_time = prof_f64(profile, "smooth_time").unwrap_or_else(|| heater.get_smooth_time());
        heater.set_inv_smooth_time(1.0 / min_deriv_time);
        let mut temp_integ_max = 0.0;
        if ki != 0.0 {
            temp_integ_max = heater_max_power / ki;
        }
        let prev_temp = if load_clean { AMBIENT_TEMP } else { heater.get_temp(heater.reactor.monotonic()).0 };
        Ok(ControlPID {
            profile: profile.clone(),
            heater: heater.clone(),
            heater_max_power,
            kp: Cell::new(kp),
            ki: Cell::new(ki),
            kd: Cell::new(kd),
            min_deriv_time,
            temp_integ_max: Cell::new(temp_integ_max),
            prev_temp: Cell::new(prev_temp),
            prev_temp_time: Cell::new(0.0),
            prev_temp_deriv: Cell::new(0.0),
            prev_temp_integ: Cell::new(0.0),
            kind: "pid",
        })
    }

    /// ControlInnerPID: the inner loop gains of a dual loop pid
    pub fn new_inner(profile: &Profile, heater: &Rc<Heater>, load_clean: bool) -> KResult<Self> {
        let mut c = Self::new(profile, heater, load_clean)?;
        c.kp.set(pid_param(profile, "inner_pid_kp")? / PID_PARAM_BASE);
        c.ki.set(pid_param(profile, "inner_pid_ki")? / PID_PARAM_BASE);
        c.kd.set(pid_param(profile, "inner_pid_kd")? / PID_PARAM_BASE);
        if c.ki.get() != 0.0 {
            c.temp_integ_max.set(c.heater_max_power / c.ki.get());
        }
        c.kind = "inner";
        Ok(c)
    }

    pub fn calculate_output(&self, read_time: f64, temp: f64, target_temp: f64) -> (f64, f64) {
        let time_diff = read_time - self.prev_temp_time.get();
        // Calculate change of temperature
        let temp_diff = temp - self.prev_temp.get();
        let temp_deriv = if time_diff >= self.min_deriv_time {
            temp_diff / time_diff
        } else {
            (self.prev_temp_deriv.get() * (self.min_deriv_time - time_diff) + temp_diff) / self.min_deriv_time
        };
        // Calculate accumulated temperature "error"
        let temp_err = target_temp - temp;
        let mut temp_integ = self.prev_temp_integ.get() + temp_err * time_diff;
        temp_integ = py_max(0.0, py_min(self.temp_integ_max.get(), temp_integ));
        // Calculate output
        let co = self.kp.get() * temp_err + self.ki.get() * temp_integ - self.kd.get() * temp_deriv;
        let bounded_co = py_max(0.0, py_min(self.heater_max_power, co));
        // Store state for next measurement
        self.prev_temp.set(temp);
        self.prev_temp_time.set(read_time);
        self.prev_temp_deriv.set(temp_deriv);
        if co == bounded_co {
            self.prev_temp_integ.set(temp_integ);
        }
        (co, bounded_co)
    }
}

impl Control for ControlPID {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        no_secondary(if self.kind == "pid" { "ControlPID" } else { "ControlInnerPID" }, secondary_temp)?;
        let (_, bounded_co) = self.calculate_output(read_time, temp, target_temp);
        self.heater.set_pwm(read_time, bounded_co)
    }
    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        let temp_diff = target_temp - smoothed_temp;
        temp_diff.abs() > PID_SETTLE_DELTA || self.prev_temp_deriv.get().abs() > PID_SETTLE_SLOPE
    }
    fn get_profile(&self) -> Profile {
        self.profile.clone()
    }
    fn get_type(&self) -> &'static str {
        "pid"
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

// Velocity (PID) control algo
pub struct ControlVelocityPID {
    profile: Profile,
    heater: Rc<Heater>,
    heater_max_power: f64,
    pub kp: Cell<f64>,
    pub ki: Cell<f64>,
    pub kd: Cell<f64>,
    smooth_time: Cell<f64>,
    temps: RefCell<Vec<f64>>,
    times: RefCell<Vec<f64>>,
    d1: Cell<f64>,
    d2: Cell<f64>,
    pwm: Cell<f64>,
}

impl ControlVelocityPID {
    pub fn new(profile: &Profile, heater: &Rc<Heater>, load_clean: bool) -> KResult<Self> {
        let kp = pid_param(profile, "pid_kp")? / PID_PARAM_BASE;
        let ki = pid_param(profile, "pid_ki")? / PID_PARAM_BASE;
        let kd = pid_param(profile, "pid_kd")? / PID_PARAM_BASE;
        let smooth_time = prof_f64(profile, "smooth_time").unwrap_or_else(|| heater.get_smooth_time());
        heater.set_inv_smooth_time(1.0 / smooth_time);
        let t = if load_clean { AMBIENT_TEMP } else { heater.get_temp(heater.reactor.monotonic()).0 };
        Ok(ControlVelocityPID {
            profile: profile.clone(),
            heater: heater.clone(),
            heater_max_power: heater.get_max_power(),
            kp: Cell::new(kp),
            ki: Cell::new(ki),
            kd: Cell::new(kd),
            smooth_time: Cell::new(smooth_time),
            temps: RefCell::new(vec![t; 3]),
            times: RefCell::new(vec![0.0; 3]),
            d1: Cell::new(0.0),
            d2: Cell::new(0.0),
            pwm: Cell::new(if load_clean { 0.0 } else { heater.last_pwm_value.get() }),
        })
    }
}

impl Control for ControlVelocityPID {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        no_secondary("ControlVelocityPID", secondary_temp)?;
        let (t, tm) = {
            let mut temps = self.temps.borrow_mut();
            let mut times = self.times.borrow_mut();
            // update the temp and time lists
            temps.remove(0);
            temps.push(temp);
            times.remove(0);
            times.push(read_time);
            (temps.clone(), times.clone())
        };
        // calculate the 1st derivative: p part in velocity form
        let d1 = t[2] - t[1];
        // calculate the error : i part in velocity form
        let mut error = tm[2] - tm[1];
        error *= target_temp - t[2];
        // calculate the 2nd derivative: d part in velocity form
        let mut d2 = t[2] - 2.0 * t[1] + t[0];
        d2 /= tm[2] - tm[1];
        // smooth both the derivatives using a modified moving average
        let n = py_max(1.0, self.smooth_time.get() / (tm[2] - tm[1]));
        self.d1.set(((n - 1.0) * self.d1.get() + d1) / n);
        self.d2.set(((n - 1.0) * self.d2.get() + d2) / n);
        // calculate the output
        let p = self.kp.get() * -self.d1.get();
        let i = self.ki.get() * error;
        let d = self.kd.get() * -self.d2.get();
        self.pwm.set(py_max(0.0, py_min(self.heater_max_power, self.pwm.get() + p + i + d)));
        if target_temp == 0.0 {
            self.pwm.set(0.0);
        }
        // update the heater
        self.heater.set_pwm(read_time, self.pwm.get())
    }
    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        let temp_diff = target_temp - smoothed_temp;
        temp_diff.abs() > PID_SETTLE_DELTA || self.d1.get().abs() > PID_SETTLE_SLOPE
    }
    fn update_smooth_time(&self) -> KResult<()> {
        self.smooth_time.set(self.heater.get_smooth_time());
        Ok(())
    }
    fn get_profile(&self) -> Profile {
        self.profile.clone()
    }
    fn get_type(&self) -> &'static str {
        "pid_v"
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

// Dual Loop PID control algo
pub struct ControlDualLoopPID {
    profile: Profile,
    heater: Rc<Heater>,
    heater_max_power: f64,
    pub primary_pid: ControlPID,
    pub secondary_pid: ControlPID,
    pub inner_target_temp: f64,
}

impl ControlDualLoopPID {
    pub fn new(profile: &Profile, heater: &Rc<Heater>, load_clean: bool) -> KResult<Self> {
        let primary_pid = ControlPID::new(profile, heater, load_clean)?;
        let secondary_pid = ControlPID::new_inner(profile, heater, load_clean)?;
        let inner_target_temp = lookup_dual_loop_pid_inner_target_temp(&heater.config)?;
        Ok(ControlDualLoopPID {
            profile: profile.clone(),
            heater: heater.clone(),
            heater_max_power: heater.get_max_power(),
            primary_pid,
            secondary_pid,
            inner_target_temp,
        })
    }
}

impl Control for ControlDualLoopPID {
    fn temperature_update(&self, read_time: f64, primary_temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        let Some(secondary_temp) = secondary_temp else {
            return Err(KError::Other("Secondary temperature must be provided!".into()));
        };
        let primary_prev_temp_integ = self.primary_pid.prev_temp_integ.get();
        let (primary_co, _) = self.primary_pid.calculate_output(read_time, primary_temp, target_temp);
        let secondary_prev_temp_integ = self.secondary_pid.prev_temp_integ.get();
        let (secondary_co, _) = self.secondary_pid.calculate_output(read_time, secondary_temp, self.inner_target_temp);
        let co = py_min(primary_co, secondary_co);
        let bounded_co = py_max(0.0, py_min(self.heater_max_power, co));
        // If the other loop reduced the final heater output, don't let this
        // loop retain an integrator update based on power that was never
        // actually applied to the heater.
        if primary_co != bounded_co {
            self.primary_pid.prev_temp_integ.set(primary_prev_temp_integ);
        }
        if secondary_co != bounded_co {
            self.secondary_pid.prev_temp_integ.set(secondary_prev_temp_integ);
        }
        self.heater.set_pwm(read_time, bounded_co)
    }
    fn check_busy(&self, eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        self.primary_pid.check_busy(eventtime, smoothed_temp, target_temp)
    }
    fn get_profile(&self) -> Profile {
        self.profile.clone()
    }
    fn get_type(&self) -> &'static str {
        "dual_loop_pid"
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

// ----------------------------------------------------------------------
// Heater
// ----------------------------------------------------------------------

const CMD_SET_HEATER_TEMPERATURE_HELP: &str = "Sets a heater temperature";
const CMD_COLD_EXTRUDE_HELP: &str = "Control cold extrusions";
const CMD_SET_SMOOTH_TIME_HELP: &str = "Set the smooth time for the given heater";
const CMD_SET_HEATER_PID_HELP: &str = "Sets a heater PID parameter";
const CMD_PID_PROFILE_HELP: &str = "PID Profile Persistent Storage management";

pub struct Heater {
    me: Weak<Heater>,
    pub printer: Rc<Printer>,
    pub name: String,
    pub short_name: String,
    pub reactor: Rc<Reactor>,
    pub config: ConfigWrapper,
    configfile: Rc<ConfigFileObject>,
    /// keeps the sensor alive (the adc callback holds it weakly)
    _sensor: Rc<dyn TempSensor>,
    /// DualSensorHeater's inner sensor (a temperature_sensor object)
    secondary_sensor: Option<PObj>,
    mpc_sensors: RefCell<Vec<Rc<dyn MpcSensor>>>,
    pub min_temp: f64,
    pub max_temp: f64,
    pwm_delay: f64,
    lost_update_tolerance: f64,
    pub min_extrude_temp: Cell<f64>,
    pub can_extrude: Cell<bool>,
    cold_extrude: Cell<bool>,
    max_power: f64,
    min_pwm_change: f64,
    config_smooth_time: f64,
    smooth_time: Cell<f64>,
    inv_smooth_time: Cell<f64>,
    verify_mainthread_time: Cell<f64>,
    pub(crate) last_temp: Cell<f64>,
    pub(crate) smoothed_temp: Cell<f64>,
    pub(crate) target_temp: Cell<f64>,
    last_temp_time: Cell<f64>,
    next_pwm_time: Cell<f64>,
    pub last_pwm_value: Cell<f64>,
    mcu_pwm: Rc<dyn PwmPin>,
    gcode: RefCell<Option<Rc<GCodeDispatch>>>,
    profiles: RefCell<IndexMap<String, Profile>>,
    incompatible_profiles: RefCell<Vec<String>>,
    control: RefCell<Option<ControlRef>>,
    set_temp_hook: RefCell<Option<Rc<dyn Fn(f64) -> KResult<()>>>>,
}

/// an mpc temperature sensor fed by the heater (mpc_*_temperature)
pub trait MpcSensor {
    fn process_temp_update(&self, control: &ControlRef, read_time: f64) -> KResult<()>;
}

impl Heater {
    pub fn new(config: &ConfigWrapper, sensor: Rc<dyn TempSensor>, secondary_sensor: Option<PObj>) -> KResult<Rc<Heater>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();
        let short_name = name.split_whitespace().last().unwrap_or("").to_string();
        let reactor = printer.get_reactor();
        let configfile = printer.lookup::<ConfigFileObject>("configfile")?;
        // Setup sensor
        let min_temp = config.getfloat("min_temp", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
        let max_temp = config.getfloat("max_temp", Dflt::Required, Lim::new().above(min_temp))?;
        sensor.setup_minmax(min_temp, max_temp)?;
        // setup_callback() below, once the heater exists
        let pwm_delay = sensor.get_report_time_delta();
        let lost_update_tolerance = (config.getint("lost_update_tolerance", 2, Lim::new().min(0))? + 1) as f64;
        // Setup temperature checks
        let min_extrude_temp = config.getfloat("min_extrude_temp", 170.0, Lim::new().min(min_temp).max(max_temp))?;
        let is_fileoutput = printer.start_arg("debugoutput").is_some();
        let can_extrude = min_extrude_temp <= 0.0 || is_fileoutput;
        let max_power = config.getfloat("max_power", 1.0, Lim::new().above(0.0).max(1.0))?;
        let config_smooth_time = config.getfloat("smooth_time", 1.0, Lim::new().above(0.0))?;
        // Those are necessary so the klipper config check does not complain
        config.get_opt("control")?;
        config.getfloat_opt("pid_kp", Lim::new())?;
        config.getfloat_opt("pid_ki", Lim::new())?;
        config.getfloat_opt("pid_kd", Lim::new())?;
        config.getfloat_opt("max_delta", Lim::new())?;
        // Setup output heater pin
        let heater_pin = config.get("heater_pin", Dflt::Required)?;
        let mcu_pwm = crate::pins::setup_pwm_pin(&printer, &heater_pin)?;
        let pwm_cycle_time = config.getfloat("pwm_cycle_time", 0.100, Lim::new().above(0.0).max(pwm_delay))?;
        mcu_pwm.setup_cycle_time(pwm_cycle_time, false)?;
        mcu_pwm.setup_max_duration(MAX_HEAT_TIME);
        let heater = Rc::new_cyclic(|me| Heater {
            me: me.clone(),
            printer: printer.clone(),
            name,
            short_name: short_name.clone(),
            reactor,
            config: config.clone(),
            configfile,
            _sensor: sensor.clone(),
            secondary_sensor,
            mpc_sensors: RefCell::new(vec![]),
            min_temp,
            max_temp,
            pwm_delay,
            lost_update_tolerance,
            min_extrude_temp: Cell::new(min_extrude_temp),
            can_extrude: Cell::new(can_extrude),
            cold_extrude: Cell::new(false),
            max_power,
            min_pwm_change: max_power * MIN_PWM_CHANGE_RATIO,
            config_smooth_time,
            smooth_time: Cell::new(config_smooth_time),
            inv_smooth_time: Cell::new(1.0 / config_smooth_time),
            verify_mainthread_time: Cell::new(-999.0),
            last_temp: Cell::new(0.0),
            smoothed_temp: Cell::new(0.0),
            target_temp: Cell::new(0.0),
            last_temp_time: Cell::new(0.0),
            next_pwm_time: Cell::new(0.0),
            last_pwm_value: Cell::new(0.0),
            mcu_pwm,
            gcode: RefCell::new(None),
            profiles: RefCell::new(IndexMap::new()),
            incompatible_profiles: RefCell::new(vec![]),
            control: RefCell::new(None),
            set_temp_hook: RefCell::new(None),
        });
        let w = Rc::downgrade(&heater);
        sensor.setup_callback(Rc::new(move |read_time, temp| match w.upgrade() {
            Some(h) => h.temperature_callback(read_time, temp),
            None => Ok(()),
        }));
        // Load additional modules
        printer.load_object(config, &format!("verify_heater {}", short_name), true)?;
        printer.load_object(config, "pid_calibrate", true)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        *heater.gcode.borrow_mut() = Some(gcode.clone());
        heater.pm_init()?;
        let default_profile = heater.pm_init_default_profile()?;
        let control = heater.lookup_control(&default_profile, true)?;
        *heater.control.borrow_mut() = Some(control);
        type HCmd = fn(&Rc<Heater>, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, String, HCmd, &str); 5] = [
            ("SET_HEATER_TEMPERATURE", short_name.clone(), Heater::cmd_set_heater_temperature, CMD_SET_HEATER_TEMPERATURE_HELP),
            ("COLD_EXTRUDE", heater.name.clone(), Heater::cmd_cold_extrude, CMD_COLD_EXTRUDE_HELP),
            ("SET_SMOOTH_TIME", short_name.clone(), Heater::cmd_set_smooth_time, CMD_SET_SMOOTH_TIME_HELP),
            ("PID_PROFILE", short_name.clone(), Heater::pm_cmd_pid_profile, CMD_PID_PROFILE_HELP),
            ("SET_HEATER_PID", short_name.clone(), Heater::cmd_set_heater_pid, CMD_SET_HEATER_PID_HELP),
        ];
        for (cmd, value, f, desc) in cmds {
            let h = heater.clone();
            gcode.register_mux_command(cmd, "HEATER", Some(&value), Rc::new(move |g| f(&h, g)), Some(desc))?;
        }
        let h = heater.clone();
        printer.register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                h.verify_mainthread_time.set(-999.0);
                Ok(())
            }),
        );
        Ok(heater)
    }

    fn rc(&self) -> Rc<Heater> {
        self.me.upgrade().expect("heater")
    }

    fn gcode(&self) -> Rc<GCodeDispatch> {
        self.gcode.borrow().clone().expect("gcode")
    }

    pub fn lookup_control(&self, profile: &Profile, load_clean: bool) -> KResult<ControlRef> {
        let me = self.rc();
        let control = prof_str(profile, "control");
        Ok(match control.as_str() {
            "watermark" => Rc::new(ControlBangBang::new(profile, &me)?),
            "pid" => Rc::new(ControlPID::new(profile, &me, load_clean)?),
            "pid_v" => Rc::new(ControlVelocityPID::new(profile, &me, load_clean)?),
            "mpc" => ControlMPC::new(profile, &me, load_clean, true)?,
            "dual_loop_pid" => Rc::new(ControlDualLoopPID::new(profile, &me, load_clean)?),
            other => return Err(KError::Other(crate::pyfmt::py_str_repr(other))),
        })
    }

    pub fn set_pwm(&self, read_time: f64, mut value: f64) -> KResult<()> {
        if self.target_temp.get() <= 0.0 || read_time > self.verify_mainthread_time.get() {
            value = 0.0;
        }
        if (read_time < self.next_pwm_time.get() || self.last_pwm_value.get() == 0.0)
            && (value - self.last_pwm_value.get()).abs() < self.min_pwm_change
        {
            // No significant change in value - can suppress update
            return Ok(());
        }
        let pwm_time = read_time + self.pwm_delay;
        self.next_pwm_time.set(pwm_time + MAX_HEAT_TIME - (self.lost_update_tolerance * self.pwm_delay + 0.001));
        self.last_pwm_value.set(value);
        self.mcu_pwm.set_pwm(pwm_time, value)
    }

    fn temperature_callback(&self, read_time: f64, temp: f64) -> KResult<()> {
        let time_diff = read_time - self.last_temp_time.get();
        self.last_temp.set(temp);
        self.last_temp_time.set(read_time);
        let control = self.get_control();
        match &self.secondary_sensor {
            None => control.temperature_update(read_time, temp, self.target_temp.get(), None)?,
            Some(s) => {
                // DualSensorHeater
                let secondary_temp = match s.get_status(read_time) {
                    Some(PyVal::Dict(d)) => d.get("temperature").and_then(|v| v.as_f64()),
                    _ => None,
                };
                let secondary_temp =
                    secondary_temp.ok_or_else(|| KError::Other("'temperature'".into()))?;
                control.temperature_update(read_time, temp, self.target_temp.get(), Some(secondary_temp))?;
            }
        }
        let temp_diff = temp - self.smoothed_temp.get();
        let adj_time = py_min(time_diff * self.inv_smooth_time.get(), 1.0);
        self.smoothed_temp.set(self.smoothed_temp.get() + temp_diff * adj_time);
        self.can_extrude.set(self.smoothed_temp.get() >= self.min_extrude_temp.get() || self.cold_extrude.get());
        if self.secondary_sensor.is_none() {
            let sensors = self.mpc_sensors.borrow().clone();
            for mpc_sensor in sensors {
                mpc_sensor.process_temp_update(&self.get_control(), read_time)?;
            }
        }
        Ok(())
    }

    // External commands
    pub fn get_name(&self) -> &str {
        &self.name
    }

    pub fn add_mpc_sensor(&self, mpc_sensor: Rc<dyn MpcSensor>) {
        self.mpc_sensors.borrow_mut().push(mpc_sensor);
    }

    /// self.smoothed_temp
    pub fn smoothed_temp(&self) -> f64 {
        self.smoothed_temp.get()
    }

    /// self.sensor
    pub fn sensor(&self) -> Rc<dyn TempSensor> {
        self._sensor.clone()
    }

    /// self.mcu_pwm.get_mcu()
    pub fn pwm_mcu(&self) -> Rc<crate::mcu::Mcu> {
        self.mcu_pwm.get_mcu()
    }

    pub fn get_pwm_delay(&self) -> f64 {
        self.pwm_delay
    }

    pub fn get_max_power(&self) -> f64 {
        self.max_power
    }

    pub fn get_smooth_time(&self) -> f64 {
        self.smooth_time.get()
    }

    pub fn set_inv_smooth_time(&self, inv_smooth_time: f64) {
        self.inv_smooth_time.set(inv_smooth_time);
    }

    pub fn set_temp(&self, degrees: f64) -> KResult<()> {
        let hook = self.set_temp_hook.borrow().clone();
        match hook {
            Some(hook) => hook(degrees),
            None => self.raw_set_temp(degrees),
        }
    }

    /// heater.set_temp = guarded_set_temp (indx): the hook replaces
    /// set_temp; raw_set_temp() stays the original method
    pub fn set_set_temp_hook(&self, hook: Option<Rc<dyn Fn(f64) -> KResult<()>>>) {
        *self.set_temp_hook.borrow_mut() = hook;
    }

    /// heater.mcu_pwm
    pub fn mcu_pwm(&self) -> Rc<dyn PwmPin> {
        self.mcu_pwm.clone()
    }

    pub fn raw_set_temp(&self, degrees: f64) -> KResult<()> {
        if degrees != 0.0 && (degrees < self.min_temp || degrees > self.max_temp) {
            return Err(KError::command(format!(
                "Requested temperature ({:.1}) out of range ({:.1}:{:.1})",
                degrees, self.min_temp, self.max_temp
            )));
        }
        if degrees != 0.0 {
            self.get_control().check_valid()?;
        }
        self.target_temp.set(degrees);
        Ok(())
    }

    pub fn get_temp(&self, eventtime: f64) -> (f64, f64) {
        let est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime);
        let quell_time = est_print_time - QUELL_STALE_TIME;
        if self.last_temp_time.get() < quell_time {
            return (0.0, self.target_temp.get());
        }
        (self.smoothed_temp.get(), self.target_temp.get())
    }

    pub fn check_busy(&self, eventtime: f64) -> bool {
        self.get_control().check_busy(eventtime, self.smoothed_temp.get(), self.target_temp.get())
    }

    pub fn set_control(&self, control: ControlRef, keep_target: bool) -> ControlRef {
        let old_control = self.control.borrow_mut().replace(control).expect("control");
        if !keep_target {
            self.target_temp.set(0.0);
        }
        old_control
    }

    pub fn get_control(&self) -> ControlRef {
        self.control.borrow().clone().expect("control")
    }

    pub fn alter_target(&self, mut target_temp: f64) {
        if target_temp != 0.0 {
            target_temp = py_max(self.min_temp, py_min(self.max_temp, target_temp));
        }
        self.target_temp.set(target_temp);
    }

    pub fn stats(&self, eventtime: f64) -> (bool, String) {
        let est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime);
        if !self.printer.is_shutdown() {
            self.verify_mainthread_time.set(est_print_time + MAX_MAINTHREAD_TIME);
        }
        let (target_temp, last_temp, last_pwm_value) = (self.target_temp.get(), self.last_temp.get(), self.last_pwm_value.get());
        let is_active = target_temp != 0.0 || last_temp > 50.0;
        (
            is_active,
            format!("{}: target={:.0} temp={:.1} pwm={:.3}", self.short_name, target_temp, last_temp, last_pwm_value),
        )
    }

    pub fn get_status(&self, eventtime: f64) -> PyVal {
        let control = self.get_control();
        let control_stats = control.get_status(eventtime);
        let mut ret = IndexMap::new();
        ret.insert("temperature".to_string(), PyVal::Float(crate::pyfmt::py_round(self.smoothed_temp.get(), 2)));
        ret.insert("target".to_string(), PyVal::Float(self.target_temp.get()));
        ret.insert("power".to_string(), PyVal::Float(self.last_pwm_value.get()));
        ret.insert("pid_profile".to_string(), prof_get(&control.get_profile(), "name"));
        if let Some(cs) = control_stats {
            ret.insert("control_stats".to_string(), cs);
        }
        PyVal::Dict(ret)
    }

    pub fn is_adc_faulty(&self) -> bool {
        self.last_temp.get() > self.max_temp || self.last_temp.get() < self.min_temp
    }

    pub fn set_cold_extrude(&self, cold_extrude: Option<i64>, min_extrude_temp: Option<f64>) -> KResult<()> {
        if cold_extrude.is_none() && min_extrude_temp.is_none() {
            self.gcode().respond_info(
                &format!(
                    "Cold extrudes are {} (min temp {:.2}C)",
                    if self.cold_extrude.get() { "enabled" } else { "disabled" },
                    self.min_extrude_temp.get()
                ),
                true,
            );
            return Ok(());
        }
        self.cold_extrude.set(cold_extrude.map(|c| c != 0).unwrap_or(false));
        if let Some(t) = min_extrude_temp {
            self.min_extrude_temp.set(t);
            self.configfile.pconfig.borrow_mut().set(&self.name, "min_extrude_temp", &crate::pyfmt::py_float_repr(t))?;
            self.gcode().respond_info(
                &format!(
                    "min_extrude_temp has been set to {:.2}C for [{}] for the current session.\nThe SAVE_CONFIG command will update the printer config file and restart the printer.",
                    t, self.name
                ),
                true,
            );
        }
        self.can_extrude.set(self.smoothed_temp.get() >= self.min_extrude_temp.get() || self.cold_extrude.get());
        Ok(())
    }

    fn cmd_set_heater_temperature(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let temp = gcmd.get_float("TARGET", Some(0.0), Lim::new())?;
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        pheaters.set_temperature(self, temp, false)
    }

    fn cmd_cold_extrude(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let cold_extrude = gcmd.get_int_opt("ENABLE", Lim::new().min(0).max(1))?;
        let min_extrude_temp = gcmd.get_float_opt("MIN_EXTRUDE_TEMP", Lim::new().min(self.min_temp).max(self.max_temp))?;
        self.set_cold_extrude(cold_extrude, min_extrude_temp)
    }

    fn cmd_set_smooth_time(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let save_to_profile = gcmd.get_int("SAVE_TO_PROFILE", Some(0), Lim::new().min(0).max(1))?;
        self.smooth_time.set(gcmd.get_float("SMOOTH_TIME", Some(self.config_smooth_time), Lim::new().min(0.0))?);
        self.inv_smooth_time.set(1.0 / self.smooth_time.get());
        self.get_control().update_smooth_time()?;
        if save_to_profile != 0 {
            prof_set(&self.get_control().get_profile(), "smooth_time", PyVal::Float(self.smooth_time.get()));
            self.pm_save_profile(None, true)?;
        }
        Ok(())
    }

    fn cmd_set_heater_pid(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let control = self.get_control();
        let (kp, ki, kd) = if let Some(c) = control.as_any().downcast_ref::<ControlPID>() {
            (&c.kp, &c.ki, &c.kd)
        } else if let Some(c) = control.as_any().downcast_ref::<ControlVelocityPID>() {
            (&c.kp, &c.ki, &c.kd)
        } else {
            return Err(gcmd.error("Not a PID/PID_V controlled heater"));
        };
        if let Some(v) = gcmd.get_float_opt("KP", Lim::new())? {
            kp.set(v / PID_PARAM_BASE);
        }
        if let Some(v) = gcmd.get_float_opt("KI", Lim::new())? {
            ki.set(v / PID_PARAM_BASE);
        }
        if let Some(v) = gcmd.get_float_opt("KD", Lim::new())? {
            kd.set(v / PID_PARAM_BASE);
        }
        Ok(())
    }

    // ------------------------------------------------------ ProfileManager

    fn pm_init(&self) -> KResult<()> {
        // Fetch stored profiles from Config
        let stored_profs = self.config.get_prefix_sections(&format!("pid_profile {}", self.short_name));
        for profile in stored_profs {
            let name = profile.get_name().splitn(3, ' ').last().unwrap_or("").to_string();
            self.pm_init_profile(&profile, &name)?;
        }
        Ok(())
    }

    fn pm_init_profile(&self, config_section: &ConfigWrapper, name: &str) -> KResult<Option<Profile>> {
        let version = config_section.getint("pid_version", 1, Lim::new())?;
        if version != PID_PROFILE_VERSION {
            logging::info(&format!(
                "Profile [{}] not compatible with this version of pid_profile.\nProfile Version: {} Current Version: {}",
                name, version, PID_PROFILE_VERSION
            ));
            self.incompatible_profiles.borrow_mut().push(name.to_string());
            return Ok(None);
        }
        let temp_profile: Profile = Rc::new(RefCell::new(IndexMap::new()));
        let control = self.pm_check_value_config("control", config_section, false, false)?.str();
        let fv = |v: Option<f64>| v.map(PyVal::Float).unwrap_or(PyVal::None);
        if control == "watermark" {
            prof_set(&temp_profile, "max_delta", fv(Some(config_section.getfloat("max_delta", 2.0, Lim::new().above(0.0))?)));
        } else if control == "mpc" {
            let g = |key: &str, default: Option<f64>, lim: Lim| -> KResult<PyVal> {
                Ok(match default {
                    Some(d) => PyVal::Float(config_section.getfloat(key, d, lim)?),
                    None => fv(config_section.getfloat_opt(key, lim)?),
                })
            };
            prof_set(&temp_profile, "block_heat_capacity", g("block_heat_capacity", None, Lim::new().above(0.0))?);
            prof_set(&temp_profile, "ambient_transfer", g("ambient_transfer", None, Lim::new().min(0.0))?);
            prof_set(&temp_profile, "target_reach_time", g("target_reach_time", Some(2.0), Lim::new().above(0.0))?);
            prof_set(&temp_profile, "smoothing", g("smoothing", Some(0.83), Lim::new().above(0.0).max(1.0))?);
            prof_set(&temp_profile, "heater_power", PyVal::Float(config_section.getfloat("heater_power", Dflt::Required, Lim::new().above(0.0))?));
            prof_set(&temp_profile, "sensor_responsiveness", g("sensor_responsiveness", None, Lim::new().above(0.0))?);
            prof_set(&temp_profile, "min_ambient_change", g("min_ambient_change", Some(1.0), Lim::new().above(0.0))?);
            prof_set(&temp_profile, "steady_state_rate", g("steady_state_rate", Some(0.5), Lim::new().above(0.0))?);
            prof_set(&temp_profile, "filament_diameter", g("filament_diameter", Some(1.75), Lim::new().above(0.0))?);
            prof_set(&temp_profile, "filament_density", g("filament_density", Some(1.2), Lim::new().above(0.0))?);
            prof_set(&temp_profile, "filament_heat_capacity", g("filament_heat_capacity", Some(1.8), Lim::new().above(0.0))?);
            prof_set(&temp_profile, "maximum_retract", g("maximum_retract", Some(2.0), Lim::new().above(0.0))?);

            let filament_temp_src_raw = config_section.get("filament_temperature_source", "ambient")?;
            let temp = crate::pyfmt::py_strip(&filament_temp_src_raw.to_lowercase()).to_string();
            let filament_temp_src = if temp == "sensor" {
                PyVal::Tuple(vec![PyVal::from(FILAMENT_TEMP_SRC_SENSOR)])
            } else if temp == "ambient" {
                PyVal::Tuple(vec![PyVal::from(FILAMENT_TEMP_SRC_AMBIENT)])
            } else {
                let value = crate::pyfmt::py_float(&temp).ok_or_else(|| {
                    KError::config(format!(
                        "Unable to parse option 'filament_temperature_source' in section '{}'",
                        config_section.get_name()
                    ))
                    .with_context(KError::Py("ValueError".into(), crate::pyfmt::py_float_error(&temp)))
                })?;
                PyVal::Tuple(vec![PyVal::from(FILAMENT_TEMP_SRC_FIXED), PyVal::Float(value)])
            };
            prof_set(&temp_profile, "filament_temp_src", filament_temp_src);

            let printer = config_section.get_printer();
            let lookup = |opt: &str| -> KResult<Option<(String, PObj)>> {
                let Some(name) = config_section.get_opt(opt)? else { return Ok(None) };
                let obj = match printer.load_object(config_section, &name, false)?.filter(|o| !o.is_none()) {
                    Some(o) => Some(o),
                    None => printer.lookup_object_opt(&name).filter(|o| !o.is_none()),
                };
                Ok(obj.map(|o| (name, o)))
            };
            let ambient_sensor_name = config_section.get_opt("ambient_temp_sensor")?;
            let ambient_sensor = match &ambient_sensor_name {
                None => None,
                Some(n) => match lookup("ambient_temp_sensor")? {
                    Some((_, o)) => Some(o),
                    None => return Err(KError::config(format!("Unknown ambient_temp_sensor '{}' specified", n))),
                },
            };
            temp_profile.borrow_mut().insert("ambient_temp_sensor".into(), ProfVal::Obj(ambient_sensor));

            let fan_name = config_section.get_opt("cooling_fan")?;
            let fan = match &fan_name {
                None => None,
                Some(n) => match lookup("cooling_fan")? {
                    None => return Err(KError::config(format!("Unknown part_cooling_fan '{}' specified", n))),
                    Some((_, o)) => match o.fan() {
                        Some(fan) => Some(fan as PObj),
                        None => return Err(KError::config(format!("part_cooling_fan '{}' is not a valid fan object", n))),
                    },
                },
            };
            temp_profile.borrow_mut().insert("cooling_fan".into(), ProfVal::Obj(fan));
            let fat = config_section.getfloatlist("fan_ambient_transfer", PyVal::List(vec![]), ",", None)?.unwrap_or_default();
            prof_set(&temp_profile, "fan_ambient_transfer", PyVal::List(fat.into_iter().map(PyVal::Float).collect()));
        } else if control == "pid" || control == "pid_v" {
            for (key, is_float, _) in PID_PROFILE_OPTIONS {
                let can_be_none = *key != "pid_kp" && *key != "pid_ki" && *key != "pid_kd";
                let v = self.pm_check_value_config(key, config_section, *is_float, can_be_none)?;
                prof_set(&temp_profile, key, v);
            }
            if name == "default" {
                prof_set(&temp_profile, "smooth_time", PyVal::None);
            }
        } else if control == "dual_loop_pid" {
            for (key, is_float, _) in PID_PROFILE_OPTIONS {
                let can_be_none = !["pid_kp", "pid_ki", "pid_kd"].contains(key);
                let v = self.pm_check_value_config(key, config_section, *is_float, can_be_none)?;
                prof_set(&temp_profile, key, v);
                // Add the keys for the outer/primary loop
                if ["pid_kp", "pid_ki", "pid_kd"].contains(key) {
                    let inner_key = format!("inner_{}", key);
                    let v = self.pm_check_value_config(&inner_key, config_section, *is_float, can_be_none)?;
                    prof_set(&temp_profile, &inner_key, v);
                }
            }
            if name == "default" {
                prof_set(&temp_profile, "smooth_time", PyVal::None);
            }
        } else {
            return Err(KError::config(format!(
                "Unknown control type '{}' in [pid_profile {} {}].",
                control, self.short_name, name
            )));
        }
        prof_set(&temp_profile, "control", PyVal::from(control.as_str()));
        prof_set(&temp_profile, "name", PyVal::from(name));
        self.profiles.borrow_mut().insert(name.to_string(), temp_profile.clone());
        Ok(Some(temp_profile))
    }

    fn pm_check_value_config(&self, key: &str, config_section: &ConfigWrapper, is_float: bool, can_be_none: bool) -> KResult<PyVal> {
        let value = if is_float {
            config_section.getfloat_opt(key, Lim::new())?.map(PyVal::Float).unwrap_or(PyVal::None)
        } else {
            config_section.get_opt(key)?.map(PyVal::Str).unwrap_or(PyVal::None)
        };
        if !can_be_none && value == PyVal::None {
            return Err(KError::command(format!(
                "pid_profile: '{}' has to be specified in [pid_profile {} {}].",
                key,
                self.short_name,
                config_section.get_name()
            )));
        }
        Ok(value)
    }

    fn pm_compute_section_name(&self, profile_name: &str) -> String {
        if profile_name == "default" {
            self.short_name.clone()
        } else {
            format!("pid_profile {} {}", self.short_name, profile_name)
        }
    }

    /// _check_value_gcmd(name, default, gcmd, type, can_be_none)
    fn pm_check_value_gcmd(
        &self,
        name: &str,
        default: PyVal,
        gcmd: &GCodeCommand,
        ty: &str,
        can_be_none: bool,
        lim: Lim,
    ) -> KResult<PyVal> {
        let value = match ty {
            "int" => match gcmd.get_int_opt(name, lim)? {
                Some(v) => PyVal::Int(v),
                None => default,
            },
            "float" => match gcmd.get_float_opt(name, lim)? {
                Some(v) => PyVal::Float(v),
                None => default,
            },
            _ => match gcmd.get_opt(name) {
                Some(v) => PyVal::Str(v),
                None => default,
            },
        };
        if !can_be_none && value == PyVal::None {
            return Err(KError::command(format!("pid_profile: '{}' has to be specified.", name)));
        }
        if ty == "lower" {
            return match value {
                PyVal::Str(s) => Ok(PyVal::Str(s.to_lowercase())),
                PyVal::None => Err(KError::Other("'NoneType' object has no attribute 'lower'".into())),
                v => Ok(v),
            };
        }
        Ok(value)
    }

    pub fn pm_init_default_profile(&self) -> KResult<Profile> {
        let config = self.config.clone();
        self.pm_init_profile(&config, "default")?
            .ok_or_else(|| KError::Other("default heater profile is not compatible".into()))
    }

    fn fmt(v: &PyVal, f: &str) -> KResult<String> {
        placeholder_fmt(f, v)
    }

    fn pm_set_values(&self, profile_name: &str, gcmd: &GCodeCommand) -> KResult<()> {
        let current_profile = self.get_control().get_profile();
        let target = self.pm_check_value_gcmd("TARGET", PyVal::None, gcmd, "float", false, Lim::new())?;
        let tolerance = self.pm_check_value_gcmd("TOLERANCE", prof_get(&current_profile, "pid_tolerance"), gcmd, "float", false, Lim::new())?;
        let control = self.pm_check_value_gcmd("CONTROL", prof_get(&current_profile, "control"), gcmd, "lower", false, Lim::new())?;
        let kp = self.pm_check_value_gcmd("KP", PyVal::None, gcmd, "float", false, Lim::new())?;
        let ki = self.pm_check_value_gcmd("KI", PyVal::None, gcmd, "float", false, Lim::new())?;
        let kd = self.pm_check_value_gcmd("KD", PyVal::None, gcmd, "float", false, Lim::new())?;
        let smooth_time = self.pm_check_value_gcmd("SMOOTH_TIME", PyVal::None, gcmd, "float", true, Lim::new())?;
        let keep_target = self.pm_check_value_gcmd("KEEP_TARGET", PyVal::Int(0), gcmd, "int", true, Lim::new().min(0).max(1))?;
        let load_clean = self.pm_check_value_gcmd("LOAD_CLEAN", PyVal::Int(0), gcmd, "int", true, Lim::new().min(0).max(1))?;
        let temp_profile = new_profile(profile_name);
        temp_profile.borrow_mut().clear();
        for (k, v) in [
            ("pid_target", &target),
            ("pid_tolerance", &tolerance),
            ("control", &control),
            ("smooth_time", &smooth_time),
            ("pid_kp", &kp),
            ("pid_ki", &ki),
            ("pid_kd", &kd),
        ] {
            prof_set(&temp_profile, k, v.clone());
        }
        let control_s = control.str();
        let mut inner = (PyVal::None, PyVal::None, PyVal::None);
        if control_s == "dual_loop_pid" {
            inner.0 = self.pm_check_value_gcmd("INNER_KP", prof_get(&current_profile, "inner_pid_kp"), gcmd, "float", false, Lim::new())?;
            inner.1 = self.pm_check_value_gcmd("INNER_KI", prof_get(&current_profile, "inner_pid_ki"), gcmd, "float", false, Lim::new())?;
            inner.2 = self.pm_check_value_gcmd("INNER_KD", prof_get(&current_profile, "inner_pid_kd"), gcmd, "float", false, Lim::new())?;
            prof_set(&temp_profile, "inner_pid_kp", inner.0.clone());
            prof_set(&temp_profile, "inner_pid_ki", inner.1.clone());
            prof_set(&temp_profile, "inner_pid_kd", inner.2.clone());
        }
        let temp_control = self.lookup_control(&temp_profile, load_clean.truthy())?;
        self.set_control(temp_control, keep_target.truthy());
        let mut msg = format!(
            "PID Parameters:\nTarget: {},\nTolerance: {}\nControl: {}\n",
            Self::fmt(&target, "%.2f")?,
            Self::fmt(&tolerance, "%.4f")?,
            control_s
        );
        if smooth_time != PyVal::None {
            msg += &format!("Smooth Time: {}\n", Self::fmt(&smooth_time, "%.3f")?);
        }
        msg += &format!("pid_Kp={} pid_Ki={} pid_Kd={}\n", Self::fmt(&kp, "%.3f")?, Self::fmt(&ki, "%.3f")?, Self::fmt(&kd, "%.3f")?);
        if control_s == "dual_loop_pid" {
            msg += &format!(
                "inner_pid_Kp={} inner_pid_Ki={} inner_pid_Kd={}\n",
                Self::fmt(&inner.0, "%.3f")?,
                Self::fmt(&inner.1, "%.3f")?,
                Self::fmt(&inner.2, "%.3f")?
            );
        }
        msg += "have been set as current profile.";
        self.gcode().respond_info(&msg, true);
        self.pm_save_profile(Some(profile_name), true)
    }

    fn pm_profile_fields_msg(profile: &Profile) -> String {
        // Generic field dump for non-PID profile types
        let mut msg = format!("Control: {}\n", prof_str(profile, "control"));
        let mut items: Vec<(String, ProfVal)> = profile.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        items.sort_by(|a, b| a.0.cmp(&b.0));
        for (key, value) in items {
            if key == "control" || key == "name" {
                continue;
            }
            if let ProfVal::V(v @ (PyVal::Bool(_) | PyVal::Int(_) | PyVal::Float(_) | PyVal::Str(_))) = &value {
                msg += &format!("{}: {}\n", key, v.str());
            }
        }
        msg += &format!("name: {}", prof_str(profile, "name"));
        msg
    }

    fn pm_get_values(&self, _profile_name: &str, _gcmd: &GCodeCommand) -> KResult<()> {
        let temp_profile = self.get_control().get_profile();
        let control = prof_str(&temp_profile, "control");
        if !PID_CONTROL_TYPES.contains(&control.as_str()) {
            self.gcode().respond_info(&Self::pm_profile_fields_msg(&temp_profile), true);
            return Ok(());
        }
        let smooth_time = match prof_get(&temp_profile, "smooth_time") {
            PyVal::None => PyVal::Float(self.get_smooth_time()),
            v => v,
        };
        let mut msg = format!(
            "PID Parameters:\nTarget: {},\nTolerance: {}\nControl: {}\nSmooth Time: {}\npid_Kp={} pid_Ki={} pid_Kd={}\n",
            Self::fmt(&prof_get(&temp_profile, "pid_target"), "%.2f")?,
            Self::fmt(&prof_get(&temp_profile, "pid_tolerance"), "%.4f")?,
            control,
            Self::fmt(&smooth_time, "%.3f")?,
            Self::fmt(&prof_get(&temp_profile, "pid_kp"), "%.3f")?,
            Self::fmt(&prof_get(&temp_profile, "pid_ki"), "%.3f")?,
            Self::fmt(&prof_get(&temp_profile, "pid_kd"), "%.3f")?
        );
        if control == "dual_loop_pid" {
            msg += &format!(
                "inner_pid_Kp={} inner_pid_Ki={} inner_pid_Kd={}\n",
                Self::fmt(&prof_get(&temp_profile, "inner_pid_kp"), "%.3f")?,
                Self::fmt(&prof_get(&temp_profile, "inner_pid_ki"), "%.3f")?,
                Self::fmt(&prof_get(&temp_profile, "inner_pid_kd"), "%.3f")?
            );
        }
        msg += &format!("name: {}", prof_str(&temp_profile, "name"));
        self.gcode().respond_info(&msg, true);
        Ok(())
    }

    pub fn pm_save_profile(&self, profile_name: Option<&str>, verbose: bool) -> KResult<()> {
        let temp_profile = self.get_control().get_profile();
        let control = prof_str(&temp_profile, "control");
        if !PID_CONTROL_TYPES.contains(&control.as_str()) && control != "watermark" {
            self.gcode().respond_info(&format!("Saving [{}] profiles with PID_PROFILE SAVE is not supported.", control), true);
            return Ok(());
        }
        let profile_name = profile_name.map(|s| s.to_string()).unwrap_or_else(|| prof_str(&temp_profile, "name"));
        let section_name = self.pm_compute_section_name(&profile_name);
        let mut cf = self.configfile.pconfig.borrow_mut();
        cf.set(&section_name, "pid_version", &PID_PROFILE_VERSION.to_string())?;
        if control == "watermark" {
            cf.set(&section_name, "control", &control)?;
            cf.set(&section_name, "max_delta", &Self::fmt(&prof_get(&temp_profile, "max_delta"), "%.4f")?)?;
        } else {
            let is_dual_loop = control == "dual_loop_pid";
            for (key, _, placeholder) in PID_PROFILE_OPTIONS {
                let value = prof_get(&temp_profile, key);
                if value != PyVal::None {
                    cf.set(&section_name, key, &Self::fmt(&value, placeholder)?)?;
                }
                // Mirror the inner/secondary loop keys read in _init_profile
                if is_dual_loop && ["pid_kp", "pid_ki", "pid_kd"].contains(key) {
                    let inner_key = format!("inner_{}", key);
                    let inner_value = prof_get(&temp_profile, &inner_key);
                    if inner_value != PyVal::None {
                        cf.set(&section_name, &inner_key, &Self::fmt(&inner_value, placeholder)?)?;
                    }
                }
            }
        }
        drop(cf);
        prof_set(&temp_profile, "name", PyVal::from(profile_name.as_str()));
        self.profiles.borrow_mut().insert(profile_name.clone(), temp_profile);
        if verbose {
            self.gcode().respond_info(
                &format!(
                    "Current PID profile for heater [{}] has been saved to profile [{}] for the current session.  The SAVE_CONFIG command will\nupdate the printer config file and restart the printer.",
                    self.short_name, profile_name
                ),
                true,
            );
        }
        Ok(())
    }

    fn pm_load_profile(&self, profile_name: &str, gcmd: &GCodeCommand) -> KResult<()> {
        let verbose = self.pm_check_value_gcmd("VERBOSE", PyVal::from("low"), gcmd, "lower", true, Lim::new())?.str();
        let load_clean = self.pm_check_value_gcmd("LOAD_CLEAN", PyVal::Int(0), gcmd, "int", true, Lim::new().min(0).max(1))?;
        if profile_name == prof_str(&self.get_control().get_profile(), "name") && !load_clean.truthy() {
            if verbose == "high" || verbose == "low" {
                self.gcode().respond_info(
                    &format!("PID Profile [{}] already loaded for heater [{}].", profile_name, self.short_name),
                    true,
                );
            }
            return Ok(());
        }
        let keep_target = self.pm_check_value_gcmd("KEEP_TARGET", PyVal::Int(0), gcmd, "int", true, Lim::new().min(0).max(1))?;
        let mut profile = self.profiles.borrow().get(profile_name).cloned();
        let mut defaulted = false;
        let default = gcmd.get_opt("DEFAULT");
        if profile.is_none() {
            let Some(default) = &default else {
                return Err(KError::command(format!(
                    "pid_profile: Unknown profile [{}] for heater [{}].",
                    profile_name, self.short_name
                )));
            };
            profile = self.profiles.borrow().get(default).cloned();
            defaulted = true;
            if profile.is_none() {
                return Err(KError::command(format!(
                    "pid_profile: Unknown default profile [{}] for heater [{}].",
                    default, self.short_name
                )));
            }
        }
        let profile = profile.unwrap();
        let control = self.lookup_control(&profile, load_clean.truthy())?;
        self.set_control(control, keep_target.truthy());

        if verbose != "high" && verbose != "low" {
            return Ok(());
        }
        if defaulted {
            self.gcode().respond_info(
                &format!(
                    "Couldn't find profile [{}] for heater [{}], defaulted to [{}].",
                    profile_name,
                    self.short_name,
                    default.unwrap_or_default()
                ),
                true,
            );
        }
        self.gcode().respond_info(&format!("PID Profile [{}] loaded for heater [{}].\n", prof_str(&profile, "name"), self.short_name), true);
        if verbose == "high" {
            let control = prof_str(&profile, "control");
            if !PID_CONTROL_TYPES.contains(&control.as_str()) {
                self.gcode().respond_info(&Self::pm_profile_fields_msg(&profile), true);
                return Ok(());
            }
            let smooth_time = match prof_get(&profile, "smooth_time") {
                PyVal::None => PyVal::Float(self.get_smooth_time()),
                v => v,
            };
            let mut msg = format!(
                "Target: {}\nTolerance: {}\nControl: {}\n",
                Self::fmt(&prof_get(&profile, "pid_target"), "%.2f")?,
                Self::fmt(&prof_get(&profile, "pid_tolerance"), "%.4f")?,
                control
            );
            if smooth_time != PyVal::None {
                msg += &format!("Smooth Time: {}\n", Self::fmt(&smooth_time, "%.3f")?);
            }
            msg += &format!(
                "PID Parameters: pid_Kp={} pid_Ki={} pid_Kd={}\n",
                Self::fmt(&prof_get(&profile, "pid_kp"), "%.3f")?,
                Self::fmt(&prof_get(&profile, "pid_ki"), "%.3f")?,
                Self::fmt(&prof_get(&profile, "pid_kd"), "%.3f")?
            );
            if control == "dual_loop_pid" {
                msg += &format!(
                    "Inner PID Parameters: inner_pid_Kp={} inner_pid_Ki={} inner_pid_Kd={}\n",
                    Self::fmt(&prof_get(&profile, "inner_pid_kp"), "%.3f")?,
                    Self::fmt(&prof_get(&profile, "inner_pid_ki"), "%.3f")?,
                    Self::fmt(&prof_get(&profile, "inner_pid_kd"), "%.3f")?
                );
            }
            self.gcode().respond_info(&msg, true);
        }
        Ok(())
    }

    fn pm_remove_profile(&self, profile_name: &str, _gcmd: &GCodeCommand) -> KResult<()> {
        if self.profiles.borrow().contains_key(profile_name) {
            let section_name = self.pm_compute_section_name(profile_name);
            self.configfile.pconfig.borrow_mut().remove_section(&section_name);
            self.profiles.borrow_mut().shift_remove(profile_name);
            self.gcode().respond_info(
                &format!(
                    "Profile [{}] for heater [{}] removed from storage for this session.\nThe SAVE_CONFIG command will update the printer\nconfiguration and restart the printer",
                    profile_name, self.short_name
                ),
                true,
            );
        } else {
            self.gcode().respond_info(&format!("No profile named [{}] to remove", profile_name), true);
        }
        Ok(())
    }

    fn pm_cmd_pid_profile(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        type PmFn = fn(&Heater, &str, &GCodeCommand) -> KResult<()>;
        let options: [(&str, PmFn); 5] = [
            ("LOAD", Heater::pm_load_profile),
            ("SAVE", |h, name, _| h.pm_save_profile(Some(name), true)),
            ("GET_VALUES", Heater::pm_get_values),
            ("SET_VALUES", Heater::pm_set_values),
            ("REMOVE", Heater::pm_remove_profile),
        ];
        for (key, f) in options {
            if let Some(profile_name) = gcmd.get_opt(key) {
                if crate::pyfmt::py_strip(&profile_name).is_empty() {
                    return Err(KError::command("pid_profile: Profile must be specified"));
                }
                return f(self, &profile_name, gcmd);
            }
        }
        Err(KError::command(format!("pid_profile: Invalid syntax '{}'", gcmd.get_commandline())))
    }

    /// not in python: drop the cycle through the control
    pub fn teardown(&self) {
        *self.control.borrow_mut() = None;
        self.mpc_sensors.borrow_mut().clear();
    }
}

impl PrinterObject for Heater {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(Heater::get_status(self, eventtime))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(Heater::stats(self, eventtime))
    }
    fn get_temp(&self, eventtime: f64) -> Option<(f64, f64)> {
        Some(Heater::get_temp(self, eventtime))
    }
    fn teardown(&self) {
        Heater::teardown(self);
    }
}

// ----------------------------------------------------------------------
// Sensor and heater lookup
// ----------------------------------------------------------------------

const TEMPERATURE_SENSORS_CFG: &str = include_str!("temperature_sensors.cfg");
const TEMPERATURE_SENSORS_FILENAME: &str = "klippy/extras/temperature_sensors.cfg";

const CMD_TURN_OFF_HEATERS_HELP: &str = "Turn off all heaters";
const CMD_TEMPERATURE_WAIT_HELP: &str = "Wait for a temperature on a sensor";

/// An entry of PrinterHeaters.heaters: kaliru's heater, or a python
/// plugin's object a plugin put there (an ACE dryer)
#[derive(Clone)]
pub enum HeaterSlot {
    Kaliru(Rc<Heater>),
    Plugin(PObj),
}

impl HeaterSlot {
    pub fn as_pobj(&self) -> PObj {
        match self {
            HeaterSlot::Kaliru(h) => h.clone(),
            HeaterSlot::Plugin(p) => p.clone(),
        }
    }
}

pub struct PrinterHeaters {
    printer: Rc<Printer>,
    sensor_factories: RefCell<IndexMap<String, SensorFactory>>,
    pub heaters: RefCell<IndexMap<String, HeaterSlot>>,
    gcode_id_to_sensor: RefCell<IndexMap<String, PObj>>,
    available_heaters: RefCell<Vec<String>>,
    available_sensors: RefCell<Vec<String>>,
    available_monitors: RefCell<Vec<String>>,
    has_started: Cell<bool>,
    have_load_sensors: Cell<bool>,
    gcode: Rc<GCodeDispatch>,
}

/// sensor.get_temp(eventtime) of a registered sensor object
fn obj_get_temp(sensor: &PObj, eventtime: f64) -> KResult<(f64, f64)> {
    sensor.get_temp(eventtime).ok_or_else(|| KError::Other("object has no attribute 'get_temp'".into()))
}

impl PrinterHeaters {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let ph = Rc::new(PrinterHeaters {
            printer: printer.clone(),
            sensor_factories: RefCell::new(IndexMap::new()),
            heaters: RefCell::new(IndexMap::new()),
            gcode_id_to_sensor: RefCell::new(IndexMap::new()),
            available_heaters: RefCell::new(vec![]),
            available_sensors: RefCell::new(vec![]),
            available_monitors: RefCell::new(vec![]),
            has_started: Cell::new(false),
            have_load_sensors: Cell::new(false),
            gcode: gcode.clone(),
        });
        let p = ph.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                p.has_started.set(true);
                Ok(())
            }),
        );
        let p = ph.clone();
        printer.register_event_handler("gcode:request_restart", Rc::new(move |_| p.turn_off_all_heaters()));
        // Register commands
        let p = ph.clone();
        gcode.register_command("TURN_OFF_HEATERS", Some(Rc::new(move |_| p.turn_off_all_heaters())), false, Some(CMD_TURN_OFF_HEATERS_HELP))?;
        let p = ph.clone();
        gcode.register_command("M105", Some(Rc::new(move |g| p.cmd_m105(g))), true, None)?;
        let p = ph.clone();
        gcode.register_command(
            "TEMPERATURE_WAIT",
            Some(Rc::new(move |g| p.cmd_temperature_wait(g))),
            false,
            Some(CMD_TEMPERATURE_WAIT_HELP),
        )?;
        Ok(ph)
    }

    fn load_config(&self, config: &ConfigWrapper) -> KResult<()> {
        self.have_load_sensors.set(true);
        // Load default temperature sensors
        // the file ships inside the binary (python reads it next to heaters.py)
        let pconfig = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let dconfig = pconfig.pconfig.borrow().build_config_wrapper(TEMPERATURE_SENSORS_CFG, TEMPERATURE_SENSORS_FILENAME);
        let dconfig = match dconfig {
            Ok(c) => c.with_printer(&self.printer),
            Err(e) => {
                logging::exception_err("Unable to load temperature_sensors.cfg", &KError::from(e));
                return Err(KError::config(format!("Cannot load config '{}'", TEMPERATURE_SENSORS_FILENAME)));
            }
        };
        for c in dconfig.get_prefix_sections("") {
            self.printer.load_object(&dconfig, c.get_name(), true)?;
        }
        let _ = config;
        Ok(())
    }

    pub fn add_sensor_factory(&self, sensor_type: &str, sensor_factory: SensorFactory) {
        self.sensor_factories.borrow_mut().insert(sensor_type.to_string(), sensor_factory);
    }

    pub fn setup_heater(&self, config: &ConfigWrapper, gcode_id: Option<&str>) -> KResult<Rc<Heater>> {
        let heater_name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        if self.heaters.borrow().contains_key(&heater_name) {
            return Err(KError::config(format!("Heater {} already registered", heater_name)));
        }
        // Setup sensor (primary/outer sensor for dual loop)
        let sensor = self.setup_sensor(config)?;
        // Setup inner sensor (inner/secondary sensor only for dual loop pid)
        let mut inner_sensor = None;
        if let Some(inner_sensor_name) = config.get_opt("inner_sensor_name")? {
            let full_name = format!("temperature_sensor {}", inner_sensor_name);
            inner_sensor = Some(self.printer.lookup_object(&full_name)?);
        }
        // Create heater
        // (DualSensorHeater's "dual_loop_pid requires a secondary sensor"
        // cannot trigger: lookup_object raises first)
        let heater = Heater::new(config, sensor, inner_sensor)?;
        self.heaters.borrow_mut().insert(heater_name, HeaterSlot::Kaliru(heater.clone()));
        self.register_sensor(config, heater.clone(), gcode_id)?;
        self.available_heaters.borrow_mut().push(config.get_name().to_string());
        Ok(heater)
    }

    pub fn get_all_heaters(&self) -> Vec<String> {
        self.available_heaters.borrow().clone()
    }

    pub fn lookup_heater(&self, heater_name: &str) -> KResult<Rc<Heater>> {
        let name = match heater_name.split_once(' ') {
            Some((_, rest)) => rest,
            None => heater_name,
        };
        match self.heaters.borrow().get(name) {
            Some(HeaterSlot::Kaliru(h)) => Ok(h.clone()),
            // kaliru code cannot drive a python plugin's heater by its type
            Some(HeaterSlot::Plugin(_)) => Err(KError::config(format!("Heater '{}' is a python plugin's object", name))),
            None => Err(KError::config(format!("Unknown heater '{}'", name))),
        }
    }

    /// lookup_heater() as python sees it: whatever object is registered
    pub fn lookup_heater_obj(&self, heater_name: &str) -> KResult<PObj> {
        let name = match heater_name.split_once(' ') {
            Some((_, rest)) => rest,
            None => heater_name,
        };
        self.heaters.borrow().get(name).map(|s| s.as_pobj()).ok_or_else(|| KError::config(format!("Unknown heater '{}'", name)))
    }

    /// available_heaters / available_sensors / available_monitors (python plugins append to them)
    pub fn name_list(&self, which: &str) -> Option<&RefCell<Vec<String>>> {
        match which {
            "available_heaters" => Some(&self.available_heaters),
            "available_sensors" => Some(&self.available_sensors),
            "available_monitors" => Some(&self.available_monitors),
            _ => None,
        }
    }

    pub fn setup_sensor(&self, config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
        if !self.have_load_sensors.get() {
            self.load_config(config)?;
        }
        let sensor_type = config.get("sensor_type", Dflt::Required)?;
        let factory = self.sensor_factories.borrow().get(&sensor_type).cloned();
        match factory {
            None => Err(KError::config(format!("Unknown temperature sensor '{}'", sensor_type))),
            Some(f) => f(config),
        }
    }

    pub fn register_sensor(&self, config: &ConfigWrapper, psensor: PObj, gcode_id: Option<&str>) -> KResult<()> {
        self.available_sensors.borrow_mut().push(config.get_name().to_string());
        let gcode_id = match gcode_id {
            Some(g) => g.to_string(),
            None => match config.get_opt("gcode_id")? {
                Some(g) => g,
                None => return Ok(()),
            },
        };
        if self.gcode_id_to_sensor.borrow().contains_key(&gcode_id) {
            return Err(KError::config(format!("G-Code sensor id {} already registered", gcode_id)));
        }
        self.gcode_id_to_sensor.borrow_mut().insert(gcode_id, psensor);
        Ok(())
    }

    pub fn register_monitor(&self, config: &ConfigWrapper) {
        self.available_monitors.borrow_mut().push(config.get_name().to_string());
    }

    pub fn turn_off_all_heaters(&self) -> KResult<()> {
        let heaters: Vec<HeaterSlot> = self.heaters.borrow().values().cloned().collect();
        for heater in heaters {
            match heater {
                HeaterSlot::Kaliru(h) => h.set_temp(0.0)?,
                HeaterSlot::Plugin(p) => {
                    if let Some(res) = p.call_method("set_temp", &[PyVal::Float(0.0)]) {
                        res?;
                    }
                }
            }
        }
        Ok(())
    }

    // G-Code M105 temperature reporting
    fn get_temp_msg(&self, eventtime: f64) -> KResult<String> {
        // Tn:XXX /YYY B:XXX /YYY
        let mut out = vec![];
        if self.has_started.get() {
            let mut sensors: Vec<(String, PObj)> =
                self.gcode_id_to_sensor.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
            sensors.sort_by(|a, b| a.0.cmp(&b.0));
            for (gcode_id, sensor) in sensors {
                let (cur, target) = obj_get_temp(&sensor, eventtime)?;
                out.push(format!("{}:{:.1} /{:.1}", gcode_id, cur, target));
            }
        }
        if out.is_empty() {
            return Ok("T:0".into());
        }
        Ok(out.join(" "))
    }

    fn cmd_m105(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Get Extruder Temperature
        let reactor = self.printer.get_reactor();
        let msg = self.get_temp_msg(reactor.monotonic())?;
        let did_ack = gcmd.ack(Some(&msg));
        if !did_ack {
            gcmd.respond_raw(&msg);
        }
        Ok(())
    }

    fn wait_for_temperature(&self, heater: &Rc<Heater>) -> KResult<()> {
        // Helper to wait on heater.check_busy() and report M105 temperatures
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(());
        }
        self.printer.wait_while(
            |eventtime| {
                self.gcode.respond_raw(&self.get_temp_msg(eventtime)?);
                Ok(heater.check_busy(eventtime))
            },
            true,
            1.0,
        )
    }

    pub fn set_temperature(&self, heater: &Rc<Heater>, temp: f64, wait: bool) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.register_lookahead_callback(Rc::new(|_| Ok(())))?;
        heater.set_temp(temp)?;
        if wait && temp != 0.0 {
            self.wait_for_temperature(heater)?;
        }
        Ok(())
    }

    fn cmd_temperature_wait(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let sensor_name = gcmd.get("SENSOR")?;
        if !self.available_sensors.borrow().contains(&sensor_name) {
            return Err(gcmd.error(format!("Unknown sensor '{}'", sensor_name)));
        }
        let min_temp = gcmd.get_float("MINIMUM", Some(f64::NEG_INFINITY), Lim::new())?;
        let max_temp = gcmd.get_float("MAXIMUM", Some(f64::INFINITY), Lim::new().above(min_temp))?;
        let error_on_cancel = gcmd.get_opt("ALLOW_CANCEL").is_none();
        if min_temp == f64::NEG_INFINITY && max_temp == f64::INFINITY {
            return Err(gcmd.error("Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM."));
        }
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(());
        }
        let heater = self.heaters.borrow().get(&sensor_name).cloned();
        let sensor: PObj = match heater {
            Some(h) => h.as_pobj(),
            None => self.printer.lookup_object(&sensor_name)?,
        };
        self.printer.wait_while(
            |eventtime| {
                let (temp, _) = obj_get_temp(&sensor, eventtime)?;
                if temp >= min_temp && temp <= max_temp {
                    return Ok(false);
                }
                gcmd.respond_raw(&self.get_temp_msg(eventtime)?);
                Ok(true)
            },
            error_on_cancel,
            1.0,
        )
    }
}

impl PrinterObject for PrinterHeaters {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let list = |v: &RefCell<Vec<String>>| PyVal::List(v.borrow().iter().map(|s| PyVal::from(s.as_str())).collect());
        let mut d = IndexMap::new();
        d.insert("available_heaters".to_string(), list(&self.available_heaters));
        d.insert("available_sensors".to_string(), list(&self.available_sensors));
        d.insert("available_monitors".to_string(), list(&self.available_monitors));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        for h in self.heaters.borrow().values() {
            if let HeaterSlot::Kaliru(h) = h {
                h.teardown();
            }
        }
        self.heaters.borrow_mut().clear();
        self.gcode_id_to_sensor.borrow_mut().clear();
        self.sensor_factories.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterHeaters::new(config)?)
}

/// printer.load_object(config, "heaters")
pub fn load_heaters(config: &ConfigWrapper) -> KResult<Rc<PrinterHeaters>> {
    config
        .get_printer()
        .load_object(config, "heaters", true)?
        .and_then(crate::printer::downcast_rc::<PrinterHeaters>)
        .ok_or_else(|| KError::Other("heaters has an unexpected type".into()))
}

/// printer.lookup_object("heaters")
pub fn lookup_heaters(config: &ConfigWrapper) -> KResult<Rc<PrinterHeaters>> {
    config.get_printer().lookup::<PrinterHeaters>("heaters")
}

