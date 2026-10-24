// Z-Probe support (port of klippy/extras/probe.py)
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::gcode_move::GCodeMove;
use crate::extras::homing::{Homing, HomingMove, PrinterHoming};
use crate::extras::manual_probe::{verify_no_manual_probe, ManualProbeHelper};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::value::Value;
use crate::logging;
use crate::mcu::{Endstop, EndstopRef, McuEndstop, Mcu, StepperRef};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_max, py_min, py_round, py_sum};
use crate::pyval::PyVal;
use crate::reactor::ReactorCompletion;
use crate::stepper::RailRef;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const HINT_TIMEOUT: &str = "
If the probe did not move far enough to trigger, then
consider reducing the Z axis minimum position so the probe
can travel further (the Z minimum position can be negative).
";

/// Probe strategy constants (an IntEnum: str() is the number)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum RetryStrategy {
    /// bad probes fail with a command error
    Fail = 0,
    /// bad probes are ignored
    Ignore = 1,
    /// bad probes are retried in the same location, assumes no fouling
    Retry = 2,
    /// bad probes are retried in a circular pattern to avoid
    Circle = 3,
}

impl RetryStrategy {
    const ALL: [RetryStrategy; 4] = [RetryStrategy::Fail, RetryStrategy::Ignore, RetryStrategy::Retry, RetryStrategy::Circle];

    fn name(self) -> &'static str {
        match self {
            RetryStrategy::Fail => "FAIL",
            RetryStrategy::Ignore => "IGNORE",
            RetryStrategy::Retry => "RETRY",
            RetryStrategy::Circle => "CIRCLE",
        }
    }
}

pub const MAX_CIRCLE_RETRIES: i64 = 6 * 3;

pub const RAPID_SCAN_WINDOW_SIZE: f64 = 2.0;

pub struct RetryPolicy {
    cfg_retry_speed: f64,
    cfg_bad_probe_strategy: RetryStrategy,
    cfg_bad_probe_retries: i64,
    cfg_pattern_spacing: f64,
    pub retry_speed: Cell<f64>,
    pub bad_probe_strategy: Cell<RetryStrategy>,
    pub bad_probe_retries: Cell<i64>,
    pub pattern_spacing: Cell<f64>,
}

impl RetryPolicy {
    pub fn new(config: &ConfigWrapper) -> KResult<Self> {
        let speed = config.getfloat("speed", 5.0, Lim::new().above(0.0))?;
        let cfg_retry_speed = config.getfloat("retry_speed", speed, Lim::new().above(0.0))?;
        // Probe retry configuration
        let choices: Vec<(String, RetryStrategy)> = RetryStrategy::ALL.iter().map(|s| (s.name().to_lowercase(), *s)).collect();
        let choices: Vec<(&str, RetryStrategy)> = choices.iter().map(|(n, s)| (n.as_str(), *s)).collect();
        let cfg_bad_probe_strategy = config.getchoice("bad_probe_strategy", &choices, "retry")?;
        let mut lim = Lim::new().min(0);
        if cfg_bad_probe_strategy == RetryStrategy::Circle {
            lim = lim.max(MAX_CIRCLE_RETRIES);
        }
        let cfg_bad_probe_retries = config.getint("bad_probe_retries", 6, lim)?;
        let cfg_pattern_spacing = config.getfloat("pattern_spacing", 2.0, Lim::new().above(0.0))?;
        // initialize values from config
        Ok(RetryPolicy {
            cfg_retry_speed,
            cfg_bad_probe_strategy,
            cfg_bad_probe_retries,
            cfg_pattern_spacing,
            retry_speed: Cell::new(cfg_retry_speed),
            bad_probe_strategy: Cell::new(cfg_bad_probe_strategy),
            bad_probe_retries: Cell::new(cfg_bad_probe_retries),
            pattern_spacing: Cell::new(cfg_pattern_spacing),
        })
    }

    // update the settings from a GCode Command instance
    pub fn customize(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.retry_speed.set(gcmd.get_float("RETRY_SPEED", Some(self.cfg_retry_speed), Lim::new().above(0.0))?);
        let strategy_name = gcmd.get_or("BAD_PROBE_STRATEGY", self.cfg_bad_probe_strategy.name()).to_uppercase();
        let strategy = RetryStrategy::ALL
            .iter()
            .find(|s| s.name() == strategy_name)
            .copied()
            .ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(&strategy_name)))?;
        self.bad_probe_strategy.set(strategy);
        let mut lim = Lim::new().min(0);
        if strategy == RetryStrategy::Circle {
            lim = lim.max(MAX_CIRCLE_RETRIES);
        }
        self.bad_probe_retries.set(gcmd.get_int("BAD_PROBE_RETRIES", Some(self.cfg_bad_probe_retries), lim)?);
        self.pattern_spacing.set(gcmd.get_float("PATTERN_SPACING", Some(self.cfg_pattern_spacing), Lim::new().above(0.0))?);
        Ok(())
    }
}

pub struct GcodeNozzleScrubber {
    printer: Rc<Printer>,
    template: Rc<Template>,
    cfg_scrubbing_frequency: i64,
    scrubbing_frequency: Cell<i64>,
}

impl GcodeNozzleScrubber {
    pub fn new(config: &ConfigWrapper) -> KResult<Self> {
        let gcode_macro = load_gcode_macro(config)?;
        let template = gcode_macro.load_template(config, "nozzle_scrubber_gcode", Some(""))?;
        let cfg_scrubbing_frequency = config.getint("scrubbing_frequency", 0, Lim::new().min(0))?;
        Ok(GcodeNozzleScrubber {
            printer: config.get_printer(),
            template,
            cfg_scrubbing_frequency,
            scrubbing_frequency: Cell::new(cfg_scrubbing_frequency),
        })
    }

    pub fn clean_nozzle(&self, gcmd: &GCodeCommand, attempt: i64, retries: i64) -> KResult<()> {
        self.scrubbing_frequency.set(gcmd.get_int("SCRUBBING_FREQUENCY", Some(self.cfg_scrubbing_frequency), Lim::new().min(0))?);
        // (`not self.config.template` is never true: a Template is truthy)
        if attempt == 0 || self.scrubbing_frequency.get() <= 0 {
            return Ok(());
        }
        if attempt % self.scrubbing_frequency.get() != 0 {
            return Ok(());
        }
        let mut context = self.template.create_template_context();
        let mut params = IndexMap::new();
        params.insert("ATTEMPT".to_string(), PyVal::Int(attempt));
        params.insert("RETRIES".to_string(), PyVal::Int(retries));
        context.insert("params".to_string(), Value::from_pyval(&PyVal::Dict(params)));
        self.template.run_gcode_from_command(Some(context))?;
        // (python builds a command_error here without raising it)
        let _ = &self.printer;
        Ok(())
    }
}

pub struct ProbeRetryState {
    bad_probe_count: Cell<i64>,
    start_pos: (f64, f64),
    fouled_count: Cell<usize>,
    circle_lookup: Vec<(f64, f64)>,
}

impl ProbeRetryState {
    fn build_hexagonal_offsets(distance_step: f64) -> Vec<(f64, f64)> {
        let mut lookup = vec![(0.0, 0.0)];
        let max_rings = 2;
        let size = distance_step / 3f64.sqrt();
        // Hexagonal direction vectors in axial coordinates
        let directions: [(i64, i64); 6] = [(1, 0), (1, -1), (0, -1), (-1, 0), (-1, 1), (0, 1)];
        for ring in 1..=max_rings {
            // Start at ring steps in direction 4 (up-left)
            let (mut q, mut r) = (-ring, ring);
            // Walk around the hexagon ring
            for direction in directions {
                for _step in 0..ring {
                    // Convert axial to Cartesian
                    let x = size * (3f64.sqrt() * q as f64 + 3f64.sqrt() / 2.0 * r as f64);
                    let y = size * (3.0 / 2.0 * r as f64);
                    lookup.push((x, y));
                    // Move to next hexagonal cell
                    let (dq, dr) = direction;
                    q += dq;
                    r += dr;
                }
            }
        }
        lookup
    }

    fn new(pos: (f64, f64), retry_policy: &RetryPolicy) -> Self {
        // Build circular lookup based on pattern spacing
        logging::info(&format!("retry policy={}", retry_policy.bad_probe_strategy.get() as i64));
        let circle_lookup = if retry_policy.bad_probe_strategy.get() == RetryStrategy::Circle {
            Self::build_hexagonal_offsets(retry_policy.pattern_spacing.get())
        } else {
            vec![]
        };
        ProbeRetryState { bad_probe_count: Cell::new(0), start_pos: pos, fouled_count: Cell::new(0), circle_lookup }
    }

    fn has_retries_remaining(&self, policy: &RetryPolicy) -> bool {
        self.bad_probe_count.get() <= policy.bad_probe_retries.get()
    }

    fn reset(&self) {
        self.bad_probe_count.set(0);
    }

    /// Evaluate probe result based on strategy. Ok(true): the probe is
    /// accepted, Ok(false): retry it; Err for FAIL or exhausted retries.
    fn evaluate_probe(&self, policy: &RetryPolicy, is_good: Option<bool>, gcmd: &GCodeCommand) -> KResult<bool> {
        let strategy = policy.bad_probe_strategy.get();
        if is_good == Some(true) || strategy == RetryStrategy::Ignore {
            return Ok(true);
        }
        if strategy == RetryStrategy::Fail {
            return Err(gcmd.error("Probe failed because it was deemed bad quality"));
        }
        self.bad_probe_count.set(self.bad_probe_count.get() + 1);
        if strategy == RetryStrategy::Circle {
            self.fouled_count.set(self.fouled_count.get() + 1);
        }
        if !self.has_retries_remaining(policy) {
            return Err(gcmd.error(format!("Probing failed after {} bad probes", self.bad_probe_count.get())));
        }
        // Probe rejected, inform user and retry
        gcmd.respond_info(
            &format!("Bad probe detected. Retrying ({}/{})...", self.bad_probe_count.get(), policy.bad_probe_retries.get()),
            true,
        );
        Ok(false)
    }

    fn get_position(&self, policy: &RetryPolicy) -> KResult<[Option<f64>; 3]> {
        if policy.bad_probe_strategy.get() != RetryStrategy::Circle {
            return Ok([Some(self.start_pos.0), Some(self.start_pos.1), None]);
        }
        let (x, y) = *self
            .circle_lookup
            .get(self.fouled_count.get())
            .ok_or_else(|| KError::Other("list index out of range".into()))?;
        Ok([Some(self.start_pos.0 + x), Some(self.start_pos.1 + y), None])
    }
}

pub struct RetrySession {
    pub retry_policy: RetryPolicy,
    scrubber: GcodeNozzleScrubber,
    point_lookup: RefCell<IndexMap<(u64, u64), Rc<ProbeRetryState>>>,
    pos: Cell<Option<(f64, f64)>>,
    gcmd: RefCell<Option<GCodeCommand>>,
    retry_state: RefCell<Option<Rc<ProbeRetryState>>>,
}

impl RetrySession {
    pub fn new(config: &ConfigWrapper) -> KResult<Self> {
        Ok(RetrySession {
            retry_policy: RetryPolicy::new(config)?,
            scrubber: GcodeNozzleScrubber::new(config)?,
            point_lookup: RefCell::new(IndexMap::new()),
            pos: Cell::new(None),
            gcmd: RefCell::new(None),
            retry_state: RefCell::new(None),
        })
    }

    /// round(pos, 1) as a dict key (floats compare by value; -0.0 == 0.0)
    fn quantize_position(pos: (f64, f64)) -> (u64, u64) {
        let key = |v: f64| {
            let r = py_round(v, 1);
            (if r == 0.0 { 0.0 } else { r }).to_bits()
        };
        (key(pos.0), key(pos.1))
    }

    pub fn start(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.retry_policy.customize(gcmd)?;
        *self.gcmd.borrow_mut() = Some(gcmd.clone());
        Ok(())
    }

    pub fn end(&self) {
        *self.gcmd.borrow_mut() = None;
        self.pos.set(None);
        *self.retry_state.borrow_mut() = None;
        self.point_lookup.borrow_mut().clear();
    }

    /// Set the current ideal position being probed
    pub fn set_position(&self, pos: &[f64]) {
        let p = (pos[0], pos[1]);
        self.pos.set(Some(p));
        let quantized_pos = Self::quantize_position(p);
        let state = self
            .point_lookup
            .borrow_mut()
            .entry(quantized_pos)
            .or_insert_with(|| Rc::new(ProbeRetryState::new(p, &self.retry_policy)))
            .clone();
        *self.retry_state.borrow_mut() = Some(state);
    }

    pub fn get_position(&self) -> (f64, f64) {
        self.pos.get().unwrap_or((0.0, 0.0))
    }

    fn state(&self) -> KResult<Rc<ProbeRetryState>> {
        self.retry_state.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_position'".into()))
    }

    fn gcmd(&self) -> KResult<GCodeCommand> {
        self.gcmd.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'error'".into()))
    }

    /// Get the actual probe position for the current 'ideal' position
    pub fn get_probe_position(&self) -> KResult<[Option<f64>; 3]> {
        self.state()?.get_position(&self.retry_policy)
    }

    pub fn can_retry(&self) -> KResult<bool> {
        Ok(self.state()?.has_retries_remaining(&self.retry_policy))
    }

    pub fn get_bad_probe_count(&self) -> KResult<i64> {
        Ok(self.state()?.bad_probe_count.get())
    }

    pub fn evaluate_probe(&self, is_good: Option<bool>) -> KResult<bool> {
        self.state()?.evaluate_probe(&self.retry_policy, is_good, &self.gcmd()?)
    }

    /// Scrub nozzle based on attempts at the current position
    pub fn scrub_nozzle(&self) -> KResult<()> {
        let attempt = self.state()?.bad_probe_count.get();
        self.scrubber.clean_nozzle(&self.gcmd()?, attempt, self.retry_policy.bad_probe_retries.get())
    }

    pub fn reset_all(&self) {
        for pos in self.point_lookup.borrow().values() {
            pos.reset();
        }
    }

    pub fn set_retry_strategy(&self, retry_strategy: RetryStrategy) {
        self.retry_policy.bad_probe_strategy.set(retry_strategy);
    }
}

// ----------------------------------------------------------------------
// Probe list
// ----------------------------------------------------------------------

pub struct ProbeList {
    printer: Rc<Printer>,
    /// name -> (probe object, is_default_probe)
    pub probes: RefCell<IndexMap<String, (PObj, bool)>>,
    pub default_probe: RefCell<Option<(String, PObj)>>,
}

impl PrinterObject for ProbeList {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        let probes: Vec<PyVal> = self.probes.borrow().keys().map(|k| PyVal::Str(k.clone())).collect();
        d.insert("probes".to_string(), PyVal::List(probes));
        // self.default_probe and self.default_probe[0]
        let default = self.default_probe.borrow().as_ref().map(|(n, _)| PyVal::Str(n.clone())).unwrap_or(PyVal::None);
        d.insert("default_probe".to_string(), default);
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        self.probes.borrow_mut().clear();
        *self.default_probe.borrow_mut() = None;
    }
}

/// ProbeList.get_list(printer)
pub fn probe_list_get_list(printer: &Rc<Printer>) -> KResult<Rc<ProbeList>> {
    if let Some(o) = printer.lookup_object_opt("probe_list") {
        return downcast_rc::<ProbeList>(o).ok_or_else(|| KError::Other("probe_list has an unexpected type".into()));
    }
    let probes = ProbeList::new(printer)?;
    printer.add_object("probe_list", probes.clone())?;
    Ok(probes)
}

/// a probe object as a PrinterProbe (the probe class every ported probe uses)
pub fn as_printer_probe(obj: &PObj) -> Option<Rc<PrinterProbe>> {
    downcast_rc::<PrinterProbe>(obj.clone())
}

impl ProbeList {
    fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let pl = Rc::new(ProbeList { printer: printer.clone(), probes: RefCell::new(IndexMap::new()), default_probe: RefCell::new(None) });
        let p = pl.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                p.handle_connect();
                Ok(())
            }),
        );
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let p = pl.clone();
        gcode.register_command("LIST_PROBES", Some(Rc::new(move |gcmd| p.cmd_list_probes(gcmd))), false, Some("List all registered probes"))?;
        Ok(pl)
    }

    fn handle_connect(&self) {
        if self.default_probe.borrow().is_none() {
            if let Some(probe) = self.printer.lookup_object_opt("probe") {
                *self.default_probe.borrow_mut() = Some(("probe".into(), probe));
            }
        }
    }

    pub fn get_all(&self) -> IndexMap<String, PObj> {
        self.probes.borrow().iter().map(|(k, (p, _))| (k.clone(), p.clone())).collect()
    }

    pub fn get_probe(&self, name: &str) -> Option<PObj> {
        self.probes.borrow().get(name).map(|(p, _)| p.clone())
    }

    pub fn get_default_probe(&self) -> Option<PObj> {
        self.default_probe.borrow().as_ref().map(|(_, p)| p.clone())
    }

    /// get_command_probe(gcmd, default=sentinel, no_default_error); a
    /// `default` of Some(None) is python's default=None
    pub fn get_command_probe(&self, gcmd: &GCodeCommand, default: Option<Option<PObj>>, no_default_error: Option<&str>) -> KResult<Option<PObj>> {
        let no_default_error = no_default_error.unwrap_or("No default probe registered, explicitly select probe by passing `PROBE=`");
        if let Some(requested_probe) = gcmd.get_opt("PROBE") {
            return match self.get_probe(&requested_probe) {
                Some(p) => Ok(Some(p)),
                None => Err(gcmd.error(format!("Unknown requested probe {}", requested_probe))),
            };
        }
        if self.default_probe.borrow().is_none() && default.is_none() {
            return Err(gcmd.error(no_default_error));
        }
        Ok(self.get_default_probe().or(default.flatten()))
    }

    /// get_config_probe(config, default=sentinel)
    pub fn get_config_probe(&self, config: &ConfigWrapper, default: Option<Option<PObj>>) -> KResult<Option<PObj>> {
        let probe_name = config.get_opt("probe")?;
        let probe = match &probe_name {
            None => self.get_default_probe(),
            Some(n) => self.get_probe(n),
        };
        if probe.is_none() && default.is_none() {
            return Err(match probe_name {
                None => KError::config("No default probe available"),
                Some(n) => KError::config(format!("Requested probe {} couldn't be found", n)),
            });
        }
        Ok(probe.or(default.flatten()))
    }

    pub fn add_probe_object(&self, probe_name: &str, is_default_probe: bool, obj: PObj, config: &ConfigWrapper) -> KResult<PObj> {
        if self.probes.borrow().contains_key(probe_name) {
            return Err(KError::config(format!("Duplicate probe name {}", probe_name)));
        }
        if is_default_probe {
            // If we allow a [probe] config section together with a different default probe, the
            // loader might ignore the true [probe] section entirely.
            if config.get_name() != "probe" && config.has_section("probe") {
                return Err(KError::config(
                    "Having a [probe] section together with another probe marked `register_as_probe: True` is not supported",
                ));
            }
            // The default probe gets registered as "probe" for compatibility with external plugins.
            self.printer.add_object("probe", obj.clone())?;
            *self.default_probe.borrow_mut() = Some((probe_name.to_string(), obj.clone()));
        }
        self.probes.borrow_mut().insert(probe_name.to_string(), (obj.clone(), is_default_probe));
        Ok(obj)
    }

    fn cmd_list_probes(&self, gcmd: &GCodeCommand) -> KResult<()> {
        gcmd.respond_info("Registered probes:", true);
        for (key, (_, is_default)) in self.probes.borrow().iter() {
            gcmd.respond_info(&format!("- {}{}", key, if *is_default { " [default]" } else { "" }), true);
        }
        Ok(())
    }
}

// ----------------------------------------------------------------------
// PrinterProbe
// ----------------------------------------------------------------------

/// what PrinterProbe uses of its mcu_probe (an endstop plus the probe hooks)
pub trait McuProbe: Endstop {
    /// multi_probe_begin(always_restore_toolhead); probes whose python
    /// method takes no argument ignore it (the TypeError fallback)
    fn multi_probe_begin(&self, always_restore_toolhead: bool) -> KResult<()>;
    fn multi_probe_end(&self) -> KResult<()>;
    /// probing_move(pos, speed, gcmd) -> (epos, is_good)
    fn probing_move(&self, pos: &[f64], speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)>;
    fn probe_prepare(&self, hmove: &Rc<HomingMove>) -> KResult<()>;
    fn probe_finish(&self, hmove: &Rc<HomingMove>) -> KResult<()>;
    /// hasattr(mcu_probe, "begin_collect_distance")
    fn can_collect_distance(&self) -> bool {
        false
    }
    fn begin_collect_distance(&self) -> KResult<Rc<dyn ProbeDistanceCollector>> {
        Err(KError::Other("probe has no begin_collect_distance".into()))
    }
}

/// a ProbeDistanceResult: (time, distance)
pub type ProbeDistanceResult = (f64, f64);

// Probes that support distance collection(eddy current sensors e.g.) implement a `begin_collect_distance()`
// method which returns this object.
pub trait ProbeDistanceCollector {
    // Finish collecting samples, and return all collected samples.
    // Finish can be called multiple times, but only the first call must always
    // return meaningful data. Any later calls can return any(or no) data, as
    // long as collection is eventually stopped.
    fn finish(&self, before_time: f64) -> KResult<Vec<ProbeDistanceResult>>;

    // Minimum expected sample rate during the collection. The rapid scanning
    // routine will add some slack, but providing samples at a much lower rate
    // than this will result in missing samples. (the only collector, eddy
    // current, returns the sensor's int rate)
    fn sample_rate(&self) -> i64;
}

/// `a is b` of two endstop objects
fn same_endstop(a: &EndstopRef, b: &EndstopRef) -> bool {
    std::ptr::eq(Rc::as_ptr(a) as *const (), Rc::as_ptr(b) as *const ())
}

const CMD_PROBE_HELP: &str = "Probe Z-height at current XY position";
const CMD_QUERY_PROBE_HELP: &str = "Return the status of the z-probe";
const CMD_PROBE_ACCURACY_HELP: &str = "Probe Z-height accuracy at current XY position";
const CMD_PROBE_CALIBRATE_HELP: &str = "Calibrate the probe's z_offset";
const CMD_Z_OFFSET_APPLY_PROBE_HELP: &str = "Adjust the probe's z_offset";

pub struct PrinterProbe {
    me: Weak<PrinterProbe>,
    printer: Rc<Printer>,
    pub name: String,
    pub probe_name: String,
    pub mcu_probe: Rc<dyn McuProbe>,
    pub speed: f64,
    retry_speed: f64,
    pub lift_speed: f64,
    x_offset: f64,
    y_offset: f64,
    pub z_offset: f64,
    drop_first_result: bool,
    probe_calibrate_z: Cell<f64>,
    multi_probe_pending: Cell<bool>,
    last_state: RefCell<PyVal>,
    last_z_result: Cell<f64>,
    gcode_move: Rc<GCodeMove>,
    retry_session: RetrySession,
    z_position: f64,
    pub sample_count: i64,
    pub sample_retract_dist: f64,
    pub samples_result: String,
    pub samples_tolerance: f64,
    pub samples_retries: i64,
    pub is_default_probe: bool,
    gcode: Rc<GCodeDispatch>,
}

impl PrinterProbe {
    pub fn new(config: &ConfigWrapper, mcu_probe: Rc<dyn McuProbe>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();
        let probe_name = name.split(' ').last().unwrap_or("").to_string();
        let speed = config.getfloat("speed", 5.0, Lim::new().above(0.0))?;
        let retry_speed = config.getfloat("retry_speed", speed, Lim::new().above(0.0))?;
        let lift_speed = config.getfloat("lift_speed", speed, Lim::new().above(0.0))?;
        let x_offset = config.getfloat("x_offset", 0.0, Lim::new())?;
        let y_offset = config.getfloat("y_offset", 0.0, Lim::new())?;
        let z_offset = config.getfloat("z_offset", Dflt::Required, Lim::new())?;
        let drop_first_result = config.getboolean("drop_first_result", false)?;
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        let retry_session = RetrySession::new(config)?;
        // Infer Z position to move to during a probe
        let z_position = if config.has_section("stepper_z") {
            let zconfig = config.getsection("stepper_z");
            zconfig.getfloat("position_min", 0.0, Lim::new().no_note())?
        } else {
            let pconfig = config.getsection("printer");
            pconfig.getfloat("minimum_z_position", 0.0, Lim::new().no_note())?
        };
        // Multi-sample support (for improved accuracy)
        let sample_count = config.getint("samples", 1, Lim::new().min(1))?;
        let sample_retract_dist = config.getfloat("sample_retract_dist", 2.0, Lim::new().above(0.0))?;
        let samples_result = config.getchoice("samples_result", &[("median", "median"), ("average", "average")], "average")?;
        let samples_tolerance = config.getfloat("samples_tolerance", 0.100, Lim::new().min(0.0))?;
        let samples_retries = config.getint("samples_tolerance_retries", 0, Lim::new().min(0))?;
        // Register z_virtual_endstop pin
        let is_default_probe = config.getboolean("register_as_probe", true)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let probe = Rc::new_cyclic(|me| PrinterProbe {
            me: me.clone(),
            printer: printer.clone(),
            name,
            probe_name: probe_name.clone(),
            mcu_probe,
            speed,
            retry_speed,
            lift_speed,
            x_offset,
            y_offset,
            z_offset,
            drop_first_result,
            probe_calibrate_z: Cell::new(0.0),
            multi_probe_pending: Cell::new(false),
            last_state: RefCell::new(PyVal::Bool(false)),
            last_z_result: Cell::new(0.0),
            gcode_move,
            retry_session,
            z_position,
            sample_count,
            sample_retract_dist,
            samples_result: samples_result.to_string(),
            samples_tolerance,
            samples_retries,
            is_default_probe,
            gcode: gcode.clone(),
        });
        {
            let ppins = printer.lookup::<PinsObject>("pins")?;
            let mut pins = ppins.0.borrow_mut();
            pins.register_chip(&probe_name, probe.clone()).map_err(|e| KError::Pins(e.0))?;
            if is_default_probe && probe_name != "probe" {
                pins.register_chip("probe", probe.clone()).map_err(|e| KError::Pins(e.0))?;
            }
        }
        probe_list_get_list(&printer)?.add_probe_object(&probe_name, is_default_probe, probe.clone(), config)?;
        // Register homing event handlers
        let p = probe.clone();
        printer.register_event_handler(
            "homing:homing_move_begin",
            Rc::new(move |args| {
                let hmove = args[0].downcast_ref::<Rc<HomingMove>>().expect("hmove");
                if hmove.get_mcu_endstops().iter().any(|e| same_endstop(e, &p.endstop())) {
                    p.mcu_probe.probe_prepare(hmove)?;
                }
                Ok(())
            }),
        );
        let p = probe.clone();
        printer.register_event_handler(
            "homing:homing_move_end",
            Rc::new(move |args| {
                let hmove = args[0].downcast_ref::<Rc<HomingMove>>().expect("hmove");
                if hmove.get_mcu_endstops().iter().any(|e| same_endstop(e, &p.endstop())) {
                    p.mcu_probe.probe_finish(hmove)?;
                }
                Ok(())
            }),
        );
        let p = probe.clone();
        printer.register_event_handler(
            "homing:home_rails_begin",
            Rc::new(move |args| {
                if p.rails_use_probe(args) {
                    p.multi_probe_begin(true)?;
                }
                Ok(())
            }),
        );
        let p = probe.clone();
        printer.register_event_handler(
            "homing:home_rails_end",
            Rc::new(move |args| {
                if p.rails_use_probe(args) {
                    p.multi_probe_end()?;
                }
                Ok(())
            }),
        );
        let p = probe.clone();
        printer.register_event_handler(
            "gcode:command_error",
            Rc::new(move |_| {
                if let Err(e) = p.multi_probe_end() {
                    logging::exception_err("Multi-probe end", &e);
                }
                Ok(())
            }),
        );
        // Register PROBE/QUERY_PROBE commands
        probe.register_commands(Some(&probe_name))?;
        if is_default_probe {
            probe.register_commands(None)?;
        }
        Ok(probe)
    }

    fn rc(&self) -> Rc<PrinterProbe> {
        self.me.upgrade().expect("probe")
    }

    fn endstop(&self) -> EndstopRef {
        self.mcu_probe.clone()
    }

    /// the home_rails_begin / _end arguments (homing_state, rails) include
    /// a rail with this probe as endstop
    fn rails_use_probe(&self, args: &[&dyn Any]) -> bool {
        let _homing_state = args[0].downcast_ref::<Rc<Homing>>();
        let rails = args[1].downcast_ref::<Vec<RailRef>>().expect("rails");
        let me = self.endstop();
        rails.iter().flat_map(|r| r.get_endstops()).any(|(es, _)| same_endstop(&es, &me))
    }

    fn register_commands(&self, key: Option<&str>) -> KResult<()> {
        type PCmd = fn(&Rc<PrinterProbe>, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, PCmd, &str); 5] = [
            ("PROBE", PrinterProbe::cmd_probe, CMD_PROBE_HELP),
            ("QUERY_PROBE", PrinterProbe::cmd_query_probe, CMD_QUERY_PROBE_HELP),
            ("PROBE_CALIBRATE", PrinterProbe::cmd_probe_calibrate, CMD_PROBE_CALIBRATE_HELP),
            ("PROBE_ACCURACY", PrinterProbe::cmd_probe_accuracy, CMD_PROBE_ACCURACY_HELP),
            ("Z_OFFSET_APPLY_PROBE", PrinterProbe::cmd_z_offset_apply_probe, CMD_Z_OFFSET_APPLY_PROBE_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let p = self.rc();
            self.gcode.register_mux_command(cmd, "PROBE", key, Rc::new(move |g| f(&p, g)), Some(desc))?;
        }
        Ok(())
    }

    pub fn multi_probe_begin(&self, always_restore_toolhead: bool) -> KResult<()> {
        self.mcu_probe.multi_probe_begin(always_restore_toolhead)?;
        self.multi_probe_pending.set(true);
        Ok(())
    }

    pub fn multi_probe_end(&self) -> KResult<()> {
        if self.multi_probe_pending.get() {
            self.multi_probe_pending.set(false);
            self.mcu_probe.multi_probe_end()?;
        }
        Ok(())
    }

    pub fn get_lift_speed(&self, gcmd: Option<&GCodeCommand>) -> KResult<f64> {
        match gcmd {
            Some(g) => g.get_float("LIFT_SPEED", Some(self.lift_speed), Lim::new().above(0.0)),
            None => Ok(self.lift_speed),
        }
    }

    pub fn get_offsets(&self) -> (f64, f64, f64) {
        (self.x_offset, self.y_offset, self.z_offset)
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.printer.lookup::<ToolHead>("toolhead")
    }

    pub fn probing_move(&self, speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let toolhead = self.toolhead()?;
        let curtime = self.printer.get_reactor().monotonic();
        let homed = match PrinterObject::get_status(&*toolhead, curtime) {
            Some(PyVal::Dict(d)) => d.get("homed_axes").map(|v| v.str()).unwrap_or_default(),
            _ => String::new(),
        };
        if !homed.contains('z') {
            return Err(KError::command("Must home before probe"));
        }
        let mut pos = toolhead.get_position();
        pos[2] = self.z_position;
        match self.mcu_probe.probing_move(&pos, speed, gcmd) {
            Err(e) if e.is_command_error() => {
                let mut reason = e.message().to_string();
                if reason.contains("Timeout during endstop homing") {
                    reason += HINT_TIMEOUT;
                }
                Err(KError::command(reason))
            }
            r => r,
        }
    }

    fn probe(&self, speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let pos = self.toolhead()?.get_position();
        let (mut epos, is_good) = self.probing_move(speed, gcmd)?;
        // get z compensation from axis_twist_compensation
        let axis_twist_compensation = self
            .printer
            .lookup_object_opt("axis_twist_compensation")
            .and_then(downcast_rc::<crate::extras::axis_twist_compensation::AxisTwistCompensation>);
        let mut z_compensation = 0.0;
        if let Some(atc) = axis_twist_compensation {
            z_compensation = atc.get_z_compensation_value(&pos);
        }
        // add z compensation to probe position
        epos[2] += z_compensation;
        self.gcode.respond_info(&format!("probe at {:.3},{:.3} is z={:.6}", epos[0], epos[1], epos[2]), true);
        epos.truncate(3);
        Ok((epos, is_good))
    }

    fn move_to(&self, coord: &[Option<f64>], speed: f64) -> KResult<()> {
        self.toolhead()?.manual_move(coord, speed)
    }

    fn calc_mean(positions: &[Vec<f64>]) -> Vec<f64> {
        let count = positions.len() as f64;
        (0..3).map(|i| crate::pyfmt::py_sum(positions.iter().map(|p| p[i])) / count).collect()
    }

    fn calc_median(positions: &[Vec<f64>]) -> Vec<f64> {
        let mut z_sorted = positions.to_vec();
        z_sorted.sort_by(|a, b| a[2].partial_cmp(&b[2]).unwrap_or(std::cmp::Ordering::Equal));
        let middle = positions.len() / 2;
        if (positions.len() & 1) == 1 {
            // odd number of samples
            return z_sorted[middle].clone();
        }
        // even number of samples
        Self::calc_mean(&z_sorted[middle - 1..middle + 1])
    }

    fn discard_first_result(&self, speed: f64, retry_session: &RetrySession, gcmd: &GCodeCommand) -> KResult<()> {
        if self.drop_first_result {
            let (_pos, is_good) = self.probe(speed, gcmd)?;
            // marks the initial probing location as fouled if needed
            retry_session.evaluate_probe(is_good)?;
            self.retract(gcmd)?;
        }
        Ok(())
    }

    // Raise the toolhead at the current x/y location
    fn retract(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST", Some(self.sample_retract_dist), Lim::new().above(0.0))?;
        let lift_speed = self.get_lift_speed(Some(gcmd))?;
        let pos = self.toolhead()?.get_position();
        self.move_to(&[None, None, Some(pos[2] + sample_retract_dist)], lift_speed)
    }

    /// Probe for a single good result with retries based on strategy
    fn run_probe_with_retries(&self, speed: f64, retry_session: &RetrySession, gcmd: &GCodeCommand) -> KResult<Vec<f64>> {
        while retry_session.can_retry()? {
            self.move_to(&retry_session.get_probe_position()?, self.retry_speed)?;
            // Probe position
            let (pos, is_good) = self.probe(speed, gcmd)?;
            if retry_session.evaluate_probe(is_good)? {
                // return the x/y of the original requested location
                let (x, y) = retry_session.get_position();
                return Ok(vec![x, y, pos[2]]);
            }
            // Probe was rejected, retry
            self.retract(gcmd)?;
            retry_session.scrub_nozzle()?;
        }
        let attempts = retry_session.get_bad_probe_count()?;
        Err(gcmd.error(format!("Probing failed to collect a good sample after {} attempts", attempts)))
    }

    pub fn run_probe(&self, gcmd: &GCodeCommand, retry_session: Option<&RetrySession>) -> KResult<Vec<f64>> {
        let speed = gcmd.get_float("PROBE_SPEED", Some(self.speed), Lim::new().above(0.0))?;
        let sample_count = gcmd.get_int("SAMPLES", Some(self.sample_count), Lim::new().min(1))?;
        let samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE", Some(self.samples_tolerance), Lim::new().min(0.0))?;
        let samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES", Some(self.samples_retries), Lim::new().min(0))?;
        let samples_result = gcmd.get_or("SAMPLES_RESULT", &self.samples_result);
        let must_notify_multi_probe = !self.multi_probe_pending.get();
        if must_notify_multi_probe {
            self.multi_probe_begin(true)?;
        }
        // Initialize probe retry state
        let local_retry_session = match retry_session {
            None => {
                self.retry_session.start(gcmd)?;
                self.retry_session.set_position(&self.toolhead()?.get_position());
                &self.retry_session
            }
            Some(rs) => rs,
        };
        // save X and Y position
        let mut retries = 0;
        let mut positions: Vec<Vec<f64>> = vec![];
        self.discard_first_result(speed, local_retry_session, gcmd)?;
        while (positions.len() as i64) < sample_count {
            // Probe position with retries
            let pos = self.run_probe_with_retries(speed, local_retry_session, gcmd)?;
            positions.push(pos);
            // Check samples tolerance
            let z: Vec<f64> = positions.iter().map(|p| p[2]).collect();
            let zmax = z[1..].iter().fold(z[0], |a, &b| py_max(a, b));
            let zmin = z[1..].iter().fold(z[0], |a, &b| py_min(a, b));
            if zmax - zmin > samples_tolerance {
                if retries >= samples_retries {
                    return Err(gcmd.error("Probe samples exceed samples_tolerance"));
                }
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...", true);
                retries += 1;
                positions.clear();
            }
            // Retract
            if (positions.len() as i64) < sample_count {
                self.retract(gcmd)?;
            }
        }
        if must_notify_multi_probe {
            self.multi_probe_end()?;
        }
        if retry_session.is_none() {
            self.retry_session.end();
        }
        // Calculate and return result
        if samples_result == "median" {
            return Ok(Self::calc_median(&positions));
        }
        Ok(Self::calc_mean(&positions))
    }

    fn cmd_probe(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let pos = self.run_probe(gcmd, None)?;
        gcmd.respond_info(&format!("Result is z={:.6}", pos[2]), true);
        self.last_z_result.set(pos[2]);
        let home = gcmd.get_or("HOME", "").to_lowercase();
        if home == "z" {
            let toolhead = self.toolhead()?;
            toolhead.get_last_move_time()?;
            let mut toolhead_pos = toolhead.get_position();
            // COSMOS 0001-probe-home-z-includes-z-offset
            toolhead_pos[2] = toolhead_pos[2] - self.last_z_result.get() + self.z_offset;
            toolhead.set_position(&toolhead_pos, "z")?;
        }
        Ok(())
    }

    fn cmd_query_probe(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let print_time = self.toolhead()?.get_last_move_time()?;
        let res = self.mcu_probe.query_endstop(print_time)?;
        *self.last_state.borrow_mut() = PyVal::Int(res);
        gcmd.respond_info(&format!("probe: {}", if res != 0 { "TRIGGERED" } else { "open" }), true);
        Ok(())
    }

    fn cmd_probe_accuracy(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let speed = gcmd.get_float("PROBE_SPEED", Some(self.speed), Lim::new().above(0.0))?;
        let lift_speed = self.get_lift_speed(Some(gcmd))?;
        let sample_count = gcmd.get_int("SAMPLES", Some(10), Lim::new().min(1))?;
        let sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST", Some(self.sample_retract_dist), Lim::new().above(0.0))?;
        let toolhead = self.toolhead()?;
        let pos = toolhead.get_position();
        gcmd.respond_info(
            &format!(
                "PROBE_ACCURACY at X:{:.3} Y:{:.3} Z:{:.3} (samples={} retract={:.3} speed={:.1} lift_speed={:.1})\n",
                pos[0], pos[1], pos[2], sample_count, sample_retract_dist, speed, lift_speed
            ),
            true,
        );
        // Probe bed sample_count times
        self.multi_probe_begin(true)?;
        // Initialize probe retry state
        self.retry_session.start(gcmd)?;
        // force FAIL behavior for PROBE_ACCURACY, never accept bad probes
        self.retry_session.set_retry_strategy(RetryStrategy::Fail);
        self.retry_session.set_position(&toolhead.get_position());
        let mut positions: Vec<Vec<f64>> = vec![];
        self.discard_first_result(speed, &self.retry_session, gcmd)?;
        while (positions.len() as i64) < sample_count {
            // Probe position
            let pos = self.run_probe_with_retries(speed, &self.retry_session, gcmd)?;
            positions.push(pos);
            // Retract
            self.retract(gcmd)?;
        }
        self.multi_probe_end()?;
        self.retry_session.end();
        // Calculate maximum, minimum and average values
        let z: Vec<f64> = positions.iter().map(|p| p[2]).collect();
        let max_value = z[1..].iter().fold(z[0], |a, &b| py_max(a, b));
        let min_value = z[1..].iter().fold(z[0], |a, &b| py_min(a, b));
        let range_value = max_value - min_value;
        let avg_value = Self::calc_mean(&positions)[2];
        let median = Self::calc_median(&positions)[2];
        // calculate the standard deviation
        let mut deviation_sum = 0.0;
        for p in &positions {
            deviation_sum += (p[2] - avg_value).powf(2.0);
        }
        let sigma = (deviation_sum / positions.len() as f64).powf(0.5);
        // calculate the average delta between successive probes
        let mut delta_sum = 0.0;
        for i in 1..positions.len() {
            delta_sum += (positions[i][2] - positions[i - 1][2]).abs();
        }
        if positions.len() < 2 {
            return Err(KError::Other("division by zero".into()));
        }
        let avg_delta = delta_sum / (positions.len() - 1) as f64;
        // Show information
        gcmd.respond_info(
            &format!(
                "probe accuracy results: maximum {:.6}, minimum {:.6}, range {:.6}, average {:.6}, median {:.6}, standard deviation {:.6}, average delta {:.6}",
                max_value, min_value, range_value, avg_value, median, sigma, avg_delta
            ),
            true,
        );
        Ok(())
    }

    fn probe_calibrate_finalize(&self, kin_pos: Option<Vec<f64>>) -> KResult<()> {
        let Some(kin_pos) = kin_pos else { return Ok(()) };
        let z_offset = self.probe_calibrate_z.get() - kin_pos[2];
        self.gcode.respond_info(
            &format!(
                "{}: z_offset: {:.3}\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
                self.name, z_offset
            ),
            true,
        );
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(&self.name, "z_offset", &format!("{:.3}", z_offset))?;
        Ok(())
    }

    fn cmd_probe_calibrate(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        verify_no_manual_probe(&self.printer)?;
        // Perform initial probe
        let lift_speed = self.get_lift_speed(Some(gcmd))?;
        let mut curpos = self.run_probe(gcmd, None)?;
        // Move away from the bed
        self.probe_calibrate_z.set(curpos[2]);
        curpos[2] += 5.0;
        let opt = |v: &[f64]| v.iter().map(|x| Some(*x)).collect::<Vec<_>>();
        self.move_to(&opt(&curpos), lift_speed)?;
        // Move the nozzle over the probe point
        curpos[0] += self.x_offset;
        curpos[1] += self.y_offset;
        self.move_to(&opt(&curpos), self.speed)?;
        // Start manual probe
        let p = self.clone();
        ManualProbeHelper::start(&self.printer, gcmd, Rc::new(move |kin_pos| p.probe_calibrate_finalize(kin_pos)))
    }

    fn cmd_z_offset_apply_probe(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        let offset = self.gcode_move.homing_position()[2];
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        if offset == 0.0 {
            self.gcode.respond_info("Nothing to do: Z Offset is 0", true);
        } else {
            let new_calibrate = self.z_offset - offset;
            self.gcode.respond_info(
                &format!(
                    "{}: z_offset: {:.3}\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
                    self.name, new_calibrate
                ),
                true,
            );
            configfile.pconfig.borrow_mut().set(&self.name, "z_offset", &format!("{:.3}", new_calibrate))?;
        }
        Ok(())
    }
}

impl PinChip for PrinterProbe {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let p = pin_params.borrow();
        if pin_type != "endstop" || p.pin != "z_virtual_endstop" {
            return Err("Probe virtual endstop only useful as endstop pin".into());
        }
        if p.invert != 0 || p.pullup != 0 {
            return Err("Can not pullup/invert probe virtual endstop".into());
        }
        Ok(Rc::new(self.endstop()) as Rc<dyn Any>)
    }
}

impl PrinterObject for PrinterProbe {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("name".to_string(), PyVal::Str(self.name.clone()));
        d.insert("last_query".to_string(), self.last_state.borrow().clone());
        d.insert("last_z_result".to_string(), PyVal::Float(self.last_z_result.get()));
        Some(PyVal::Dict(d))
    }
}

// ----------------------------------------------------------------------
// ProbeEndstopWrapper
// ----------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
pub enum Multi {
    Off,
    First,
    On,
}

// Endstop wrapper that enables probe specific features
pub struct ProbeEndstopWrapper {
    me: Weak<ProbeEndstopWrapper>,
    printer: Rc<Printer>,
    position_endstop: f64,
    stow_on_each_sample: bool,
    activate_gcode: Rc<Template>,
    deactivate_gcode: Rc<Template>,
    mcu_endstop: Rc<McuEndstop>,
    multi: Cell<Multi>,
}

impl ProbeEndstopWrapper {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let position_endstop = config.getfloat("z_offset", Dflt::Required, Lim::new())?;
        let stow_on_each_sample = config.getboolean("deactivate_on_each_sample", true)?;
        let gcode_macro = load_gcode_macro(config)?;
        let activate_gcode = gcode_macro.load_template(config, "activate_gcode", Some(""))?;
        let deactivate_gcode = gcode_macro.load_template(config, "deactivate_gcode", Some(""))?;
        // Create an "endstop" object to handle the probe pin
        let pin = config.get("pin", Dflt::Required)?;
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin, true, true, None).map_err(|e| KError::Pins(e.0))?;
        let chip = pin_params.borrow().chip.clone();
        let mcu_endstop = chip
            .setup_pin("endstop", &pin_params)
            .map_err(KError::Pins)?
            .downcast::<McuEndstop>()
            .map_err(|_| KError::Other("probe pin chip has no mcu endstop".into()))?;
        let w = Rc::new_cyclic(|me| ProbeEndstopWrapper {
            me: me.clone(),
            printer: printer.clone(),
            position_endstop,
            stow_on_each_sample,
            activate_gcode,
            deactivate_gcode,
            mcu_endstop,
            // multi probes state
            multi: Cell::new(Multi::Off),
        });
        let ww = w.clone();
        printer.register_event_handler("klippy:mcu_identify", Rc::new(move |_| ww.handle_mcu_identify()));
        Ok(w)
    }

    fn handle_mcu_identify(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.is_active_axis('z') {
                self.mcu_endstop.add_stepper(&stepper)?;
            }
        }
        Ok(())
    }

    fn run_script(&self, template: &Template, what: &str) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let start_pos = toolhead.get_position();
        template.run_gcode_from_command(None)?;
        if toolhead.get_position()[..3] != start_pos[..3] {
            return Err(KError::command(format!("Toolhead moved during probe {} script", what)));
        }
        Ok(())
    }

    fn raise_probe(&self) -> KResult<()> {
        self.run_script(&self.deactivate_gcode, "deactivate_gcode")
    }

    fn lower_probe(&self) -> KResult<()> {
        self.run_script(&self.activate_gcode, "activate_gcode")
    }
}

impl Endstop for ProbeEndstopWrapper {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_endstop.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.mcu_endstop.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        Endstop::get_steppers(&*self.mcu_endstop)
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        Endstop::home_start(&*self.mcu_endstop, print_time, sample_time, sample_count, rest_time, triggered)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        Endstop::home_wait(&*self.mcu_endstop, home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        Endstop::query_endstop(&*self.mcu_endstop, print_time)
    }
    fn get_position_endstop(&self) -> Option<f64> {
        Some(self.position_endstop)
    }
}

impl McuProbe for ProbeEndstopWrapper {
    fn multi_probe_begin(&self, _always_restore_toolhead: bool) -> KResult<()> {
        if self.stow_on_each_sample {
            return Ok(());
        }
        self.multi.set(Multi::First);
        Ok(())
    }

    fn multi_probe_end(&self) -> KResult<()> {
        if self.stow_on_each_sample {
            return Ok(());
        }
        self.raise_probe()?;
        self.multi.set(Multi::Off);
        Ok(())
    }

    fn probing_move(&self, pos: &[f64], speed: f64, _gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let me: EndstopRef = self.me.upgrade().expect("probe endstop");
        Ok((phoming.probing_move(me, pos, speed)?, Some(true)))
    }

    fn probe_prepare(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        let multi = self.multi.get();
        if multi == Multi::Off || multi == Multi::First {
            self.lower_probe()?;
            if multi == Multi::First {
                self.multi.set(Multi::On);
            }
        }
        Ok(())
    }

    fn probe_finish(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        if self.multi.get() == Multi::Off {
            self.raise_probe()?;
        }
        Ok(())
    }
}

// ----------------------------------------------------------------------
// ProbePointsHelper
// ----------------------------------------------------------------------

/// what a ProbePointsHelper finalize callback returns: a number (0 is
/// done), "retry", or anything else (done)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum FinalizeResult {
    Num(f64),
    Retry,
    Done,
}

/// finalize_callback(probe_offsets, results)
pub type PointsFinalize = Rc<dyn Fn((f64, f64, f64), &[Vec<f64>]) -> KResult<FinalizeResult>>;

// Helper code that can probe a series of points and report the
// position at each point.
pub struct ProbePointsHelper {
    me: Weak<ProbePointsHelper>,
    printer: Rc<Printer>,
    finalize_callback: PointsFinalize,
    probe_points: RefCell<Vec<(f64, f64)>>,
    name: String,
    gcode: Rc<GCodeDispatch>,
    horizontal_move_z: Cell<f64>,
    default_horizontal_move_z: f64,
    enable_horizontal_z_clearance: bool,
    horizontal_z_clearance: Cell<Option<f64>>,
    default_horizontal_z_clearance: Option<f64>,
    adaptive_horizontal_move_z: bool,
    min_horizontal_move_z: f64,
    speed: f64,
    use_offsets: Cell<bool>,
    alternate_probe_direction: bool,
    start_reverse: bool,
    enforce_lift_speed: Cell<bool>,
    retry_session: RetrySession,
    lift_speed: Cell<f64>,
    probe_offsets: Cell<(f64, f64, f64)>,
    results: RefCell<Vec<Vec<f64>>>,
    probe_pass_direction_reversed: Cell<bool>,
}

impl ProbePointsHelper {
    pub fn new(
        config: &ConfigWrapper,
        finalize_callback: PointsFinalize,
        default_points: Option<Vec<(f64, f64)>>,
        option_name: &str,
        use_offsets: bool,
        enable_horizontal_z_clearance: bool,
    ) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // Read config settings
        let mut probe_points = default_points.clone().unwrap_or_default();
        if default_points.is_none() || config.get_opt(option_name)?.is_some() {
            let v = config.getlists(option_name, Dflt::Required, &[",", "\n"], Some(2), ListParser::Float, true)?;
            probe_points = v
                .as_ref()
                .and_then(|v| v.items())
                .unwrap_or(&[])
                .iter()
                .map(|p| {
                    let i = p.items().unwrap_or(&[]);
                    (i[0].as_f64().unwrap_or(0.0), i[1].as_f64().unwrap_or(0.0))
                })
                .collect();
        }
        let def_move_z = config.getfloat("horizontal_move_z", 5.0, Lim::new())?;
        // horizontal_z_clearance mode is off by default
        let mut z_clearance = None;
        if enable_horizontal_z_clearance {
            z_clearance = config.getfloat_opt("horizontal_z_clearance", Lim::new())?;
        }
        let adaptive_horizontal_move_z = config.getboolean("adaptive_horizontal_move_z", false)?;
        let min_horizontal_move_z = config.getfloat("min_horizontal_move_z", 1.0, Lim::new())?;
        let speed = config.getfloat("speed", 50.0, Lim::new().above(0.0))?;
        let use_offsets = config.getboolean("use_probe_xy_offsets", use_offsets)?;
        let alternate_probe_direction = config.getboolean("alternate_probe_direction", false)?;
        let start_reverse = if alternate_probe_direction { config.getboolean("start_reverse", false)? } else { false };

        let enforce_lift_speed = config.getboolean("enforce_lift_speed", false)?;

        // Probe retry configuration
        let retry_session = RetrySession::new(config)?;
        Ok(Rc::new_cyclic(|me| ProbePointsHelper {
            me: me.clone(),
            printer: printer.clone(),
            finalize_callback,
            probe_points: RefCell::new(probe_points),
            name,
            gcode,
            horizontal_move_z: Cell::new(def_move_z),
            default_horizontal_move_z: def_move_z,
            enable_horizontal_z_clearance,
            horizontal_z_clearance: Cell::new(z_clearance),
            default_horizontal_z_clearance: z_clearance,
            adaptive_horizontal_move_z,
            min_horizontal_move_z,
            speed,
            use_offsets: Cell::new(use_offsets),
            alternate_probe_direction,
            start_reverse,
            enforce_lift_speed: Cell::new(enforce_lift_speed),
            retry_session,
            // Internal probing state
            lift_speed: Cell::new(speed),
            probe_offsets: Cell::new((0.0, 0.0, 0.0)),
            results: RefCell::new(vec![]),
            probe_pass_direction_reversed: Cell::new(false),
        }))
    }

    pub fn get_probe_points(&self) -> Vec<(f64, f64)> {
        self.probe_points.borrow().clone()
    }

    pub fn minimum_points(&self, n: usize) -> KResult<()> {
        if self.probe_points.borrow().len() < n {
            return Err(KError::config(format!("Need at least {} probe points for {}", n, self.name)));
        }
        Ok(())
    }

    pub fn update_probe_points(&self, points: Vec<(f64, f64)>, min_points: usize) -> KResult<()> {
        *self.probe_points.borrow_mut() = points;
        self.minimum_points(min_points)
    }

    pub fn use_xy_offsets(&self, use_offsets: bool) {
        self.use_offsets.set(use_offsets);
    }

    pub fn get_lift_speed(&self, gcmd: Option<&GCodeCommand>) -> KResult<f64> {
        match gcmd {
            Some(g) => g.get_float("LIFT_SPEED", Some(self.lift_speed.get()), Lim::new().above(0.0)),
            None => Ok(self.lift_speed.get()),
        }
    }

    fn store_probe_result(&self, position: Vec<f64>) {
        if self.probe_pass_direction_reversed.get() {
            self.results.borrow_mut().insert(0, position);
        } else {
            self.results.borrow_mut().push(position);
        }
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.printer.lookup::<ToolHead>("toolhead")
    }

    fn lift_toolhead(&self) -> KResult<()> {
        let toolhead = self.toolhead()?;
        // Lift toolhead
        let mut speed = self.lift_speed.get();
        let have_results = !self.results.borrow().is_empty();
        if !have_results && !self.enforce_lift_speed.get() {
            // Use full speed to first probe position
            speed = self.speed;
        }
        let mut z_pos = self.horizontal_move_z.get();
        // use horizontal_z_clearance for inter-point moves
        if let Some(c) = self.horizontal_z_clearance.get() {
            if have_results {
                z_pos = toolhead.get_position()[2] + c;
            }
        }
        toolhead.manual_move(&[None, None, Some(z_pos)], speed)
    }

    fn next_pos(&self) -> KResult<[Option<f64>; 3]> {
        let result_count = self.results.borrow().len();
        let points = self.probe_points.borrow();
        let point_index = if self.probe_pass_direction_reversed.get() { points.len() - result_count - 1 } else { result_count };
        let (mut x, mut y) = points[point_index];
        if self.use_offsets.get() {
            x -= self.probe_offsets.get().0;
            y -= self.probe_offsets.get().1;
        }
        self.retry_session.set_position(&[x, y]);
        self.retry_session.get_probe_position()
    }

    fn move_next(&self) -> KResult<bool> {
        let toolhead = self.toolhead()?;
        // Check if done probing
        let mut done = false;
        let finalize = self.results.borrow().len() >= self.probe_points.borrow().len();
        if finalize {
            toolhead.get_last_move_time()?;
            let results = self.results.borrow().clone();
            let res = (self.finalize_callback)(self.probe_offsets.get(), &results)?;
            match res {
                FinalizeResult::Num(res) => {
                    if res == 0.0 {
                        done = true;
                    }
                    if self.adaptive_horizontal_move_z {
                        // then res is error
                        let error = res.ceil();
                        self.horizontal_move_z.set(py_max(error + self.probe_offsets.get().2, self.min_horizontal_move_z));
                    }
                }
                FinalizeResult::Retry => {}
                FinalizeResult::Done => done = true,
            }
        }
        self.lift_toolhead()?;
        if finalize {
            self.results.borrow_mut().clear();
            if !done && self.alternate_probe_direction {
                self.probe_pass_direction_reversed.set(!self.probe_pass_direction_reversed.get());
            }
        }
        if done {
            return Ok(true);
        }
        // Move to next XY probe point
        toolhead.manual_move(&self.next_pos()?, self.speed)?;
        Ok(false)
    }

    pub fn start_probe(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.retry_session.start(gcmd)?;
        self.retry_session.reset_all();
        verify_no_manual_probe(&self.printer)?;
        // Lookup objects
        let plist = probe_list_get_list(&self.printer)?;
        let probe = plist.get_command_probe(gcmd, Some(None), None)?;
        let mut method = gcmd.get_or("METHOD", "automatic").to_lowercase();
        if method != "manual" && probe.is_none() && !plist.get_all().is_empty() {
            return Err(gcmd.error(
                "No default probe, but named probes are available. Either select a probe with PROBE= or use METHOD=manual to enter manual probing mode.",
            ));
        }

        self.results.borrow_mut().clear();
        self.probe_pass_direction_reversed.set(self.start_reverse);

        self.horizontal_move_z.set(gcmd.get_float("HORIZONTAL_MOVE_Z", Some(self.default_horizontal_move_z), Lim::new())?);
        if self.enable_horizontal_z_clearance {
            let c = match self.default_horizontal_z_clearance {
                Some(d) => Some(gcmd.get_float("HORIZONTAL_Z_CLEARANCE", Some(d), Lim::new())?),
                None => gcmd.get_float_opt("HORIZONTAL_Z_CLEARANCE", Lim::new())?,
            };
            self.horizontal_z_clearance.set(c);
        }

        if let Some(e) = gcmd.get_int_opt("ENFORCE_LIFT_SPEED", Lim::new().min(0).max(1))? {
            self.enforce_lift_speed.set(e != 0);
        }

        let probe = probe.map(|p| as_printer_probe(&p).ok_or_else(|| KError::Other("probe has an unexpected type".into()))).transpose()?;
        if let Some(probe) = &probe {
            if method == "rapid_scan" {
                let can_scan = probe.mcu_probe.can_collect_distance();
                if can_scan {
                    return self.start_rapid_scan(gcmd, probe);
                } else {
                    gcmd.respond_info(&format!("METHOD=rapid_scan not supported for probe {}, using automatic", probe.name), true);
                    method = "automatic".into();
                }
            }
        }

        let probe = match probe {
            Some(p) if method == "automatic" => p,
            _ => {
                // Manual probe
                self.lift_speed.set(self.speed);
                self.probe_offsets.set((0.0, 0.0, 0.0));
                return self.manual_probe_start();
            }
        };
        // Perform automatic probing
        self.lift_speed.set(probe.get_lift_speed(Some(gcmd))?);
        self.probe_offsets.set(probe.get_offsets());
        if self.horizontal_move_z.get() < self.probe_offsets.get().2 {
            return Err(gcmd.error("horizontal_move_z can't be less than probe's z_offset"));
        }
        probe.multi_probe_begin(false)?;
        loop {
            let done = self.move_next()?;
            if done {
                break;
            }
            let pos = probe.run_probe(gcmd, Some(&self.retry_session))?;
            let p = PyVal::List(pos.iter().map(|v| PyVal::Float(*v)).collect());
            logging::info(&format!("Probe pos:{}", p.repr()));
            self.store_probe_result(pos);
        }
        probe.multi_probe_end()?;
        self.retry_session.end();
        Ok(())
    }

    fn start_rapid_scan(&self, gcmd: &GCodeCommand, probe: &Rc<PrinterProbe>) -> KResult<()> {
        let toolhead = self.toolhead()?;

        self.probe_offsets.set(probe.get_offsets());
        if self.horizontal_move_z.get() < self.probe_offsets.get().2 {
            return Err(gcmd.error("horizontal_move_z can't be less than probe's z_offset"));
        }
        let mut scan_height = self.horizontal_move_z.get();
        self.lift_speed.set(probe.get_lift_speed(Some(gcmd))?);
        let (max_speed, _) = toolhead.get_max_velocity();
        let probe_speed = gcmd.get_float("PROBE_SPEED", Some(probe.speed), Lim::new().above(0.0))?;
        let scan_speed = gcmd.get_float("SCAN_SPEED", Some(py_min(self.speed, max_speed)), Lim::new().above(0.0).max(max_speed))?;

        let window = RAPID_SCAN_WINDOW_SIZE / scan_speed;

        let axis_twist_compensation = self
            .printer
            .lookup_object_opt("axis_twist_compensation")
            .and_then(downcast_rc::<crate::extras::axis_twist_compensation::AxisTwistCompensation>);

        probe.multi_probe_begin(false)?;

        let all = |p: &[f64]| p.iter().map(|v| Some(*v)).collect::<Vec<_>>();
        let xy = |p: (f64, f64)| [Some(p.0), Some(p.1)];
        let mut done = false;
        while !done {
            // If needed, raise up so we can move down to pull out backlash
            let mut cur_pos = toolhead.get_position();
            if cur_pos[2] < scan_height + 0.5 {
                cur_pos[2] = scan_height + 0.5;
                toolhead.manual_move(&all(&cur_pos), self.lift_speed.get())?;
            }

            // Move down to scan height
            cur_pos[2] = scan_height;
            toolhead.manual_move(&all(&cur_pos), probe_speed)?;

            let path = self.generate_rapid_path();

            toolhead.manual_move(&xy(path[0].0), scan_speed)?;
            toolhead.wait_moves()?;

            let session = probe.mcu_probe.begin_collect_distance()?;
            let point_times: Rc<RefCell<Vec<(f64, (f64, f64))>>> = Rc::new(RefCell::new(vec![]));
            let scan = || -> KResult<Vec<ProbeDistanceResult>> {
                let sample_rate = session.sample_rate();
                let sample_interval = 1.0 / sample_rate as f64;

                if window < sample_interval {
                    let max_speed = py_round(2.0 * sample_rate as f64, 1);
                    return Err(gcmd.error(format!(
                        "Sample rate is not high enough for the requested speed. At {} Hz, maximum possible speed is ~{} mm/s but {} mm/s was requested. Actual attainable speed may be lower.",
                        sample_rate,
                        py_float_repr(max_speed),
                        py_float_repr(scan_speed)
                    )));
                }

                toolhead.dwell(sample_interval)?;

                for (point, capture) in &path[1..] {
                    toolhead.manual_move(&xy(*point), scan_speed)?;
                    if *capture {
                        let (pt, p) = (point_times.clone(), *point);
                        toolhead.register_lookahead_callback(Rc::new(move |time| {
                            pt.borrow_mut().push((time, p));
                            Ok(())
                        }))?;
                    }
                }

                toolhead.wait_moves()?;
                toolhead.dwell(sample_interval)?;
                session.finish(toolhead.get_last_move_time()?)
            };
            let samples = match scan() {
                Ok(samples) => samples,
                Err(e) => {
                    session.finish(toolhead.get_last_move_time()?)?;
                    return Err(e);
                }
            };

            // In `point_times` we now have the timestamp of reaching each point in `self.probe_points`,
            // and we have all the (time,dist) samples. Both will be monotonically increasing in time,
            // so walk through them together and calculate the bed surface z.

            let mut j = 0;

            // Window catches all samples within a 2mm disc centered on the target point,
            // based on the maximum speed during the scan. Moving slower results in a bigger time window
            // and thus more samples for that point.
            let half_window = window / 2.0;
            for &(time, point) in point_times.borrow().iter() {
                let earliest = time - half_window;
                let latest = time + half_window;
                while j < samples.len() && samples[j].0 < earliest {
                    j += 1;
                }
                let mut end = j;
                while end < samples.len() && samples[end].0 < latest {
                    end += 1;
                }
                if j == end {
                    return Err(gcmd.error(format!(
                        "Scanning failed. Point at ({},{}) had no samples collected for it.",
                        py_float_repr(point.0),
                        py_float_repr(point.1)
                    )));
                }
                let relevant = &samples[j..end];
                let invalid = relevant.iter().any(|p| p.1.abs() >= 99.0);
                if invalid {
                    return Err(gcmd.error(format!(
                        "Scanning failed. Point at ({},{}) gathered one or more invalid samples.",
                        py_float_repr(point.0),
                        py_float_repr(point.1)
                    )));
                }
                let distance = py_sum(relevant.iter().map(|p| p.1)) / (end - j) as f64;
                let mut z = scan_height - distance + self.probe_offsets.get().2;
                if let Some(atc) = &axis_twist_compensation {
                    z += atc.get_z_compensation_value(&[point.0, point.1, scan_height]);
                }
                self.store_probe_result(vec![point.0, point.1, z]);
            }

            let results = self.results.borrow().clone();
            let res = (self.finalize_callback)(self.probe_offsets.get(), &results)?;
            self.results.borrow_mut().clear();
            match res {
                FinalizeResult::Num(res) => {
                    if res == 0.0 {
                        done = true;
                    }
                    if self.adaptive_horizontal_move_z {
                        // then res is error
                        let error = res.ceil();
                        scan_height = py_max(error + self.probe_offsets.get().2, self.min_horizontal_move_z);
                    }
                }
                FinalizeResult::Retry => {}
                FinalizeResult::Done => done = true,
            }
            if !done && self.alternate_probe_direction {
                self.probe_pass_direction_reversed.set(!self.probe_pass_direction_reversed.get());
            }
        }

        probe.multi_probe_end()?;
        self.retry_session.end();
        Ok(())
    }

    fn generate_rapid_path(&self) -> Vec<((f64, f64), bool)> {
        // TODO: When changing direction, add an arc where the points don't capture
        let mut points = vec![];

        let mut source = self.probe_points.borrow().clone();
        if self.probe_pass_direction_reversed.get() {
            source.reverse();
        }

        let offsets = self.probe_offsets.get();
        let (mut x, mut y) = source[0];
        if self.use_offsets.get() {
            x -= offsets.0;
            y -= offsets.1;
        }
        points.push(((x, y), false));

        for &(mut x, mut y) in &source {
            if self.use_offsets.get() {
                x -= offsets.0;
                y -= offsets.1;
            }
            points.push(((x, y), true));
        }
        points
    }

    fn manual_probe_start(&self) -> KResult<()> {
        let done = self.move_next()?;
        if !done {
            let gcmd = self.gcode.create_gcode_command("", "", IndexMap::new());
            let me = self.me.upgrade().expect("probe points helper");
            ManualProbeHelper::start(&self.printer, &gcmd, Rc::new(move |kin_pos| me.manual_probe_finalize(kin_pos)))?;
        }
        Ok(())
    }

    fn manual_probe_finalize(&self, kin_pos: Option<Vec<f64>>) -> KResult<()> {
        let Some(kin_pos) = kin_pos else { return Ok(()) };
        self.store_probe_result(kin_pos);
        self.manual_probe_start()
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let mcu_probe = ProbeEndstopWrapper::new(config)?;
    Ok(PrinterProbe::new(config, mcu_probe)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    load_config(config)
}
