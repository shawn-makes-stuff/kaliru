// Generic Filament Sensor Module (port of klippy/extras/filament_switch_sensor.py)
//
// Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::buttons::PrinterButtons;
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::pause_resume::PauseResume;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::py_max;
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const CHECK_RUNOUT_TIMEOUT: f64 = 0.250;

const CMD_QUERY_FILAMENT_SENSOR_HELP: &str = "Query the status of the Filament Sensor";
const CMD_SET_FILAMENT_SENSOR_HELP: &str = "Sets the filament sensor on/off";

/// what RunoutHelper uses of its sensor (python's defined_sensor)
pub trait DefinedSensor {
    fn get_extruder_pos(&self, eventtime: Option<f64>) -> KResult<f64>;
    fn sensor_get_status(&self, eventtime: f64) -> IndexMap<String, PyVal>;
    fn get_info(&self, gcmd: &GCodeCommand) -> KResult<bool>;
    fn reset_needed(&self, enable: Option<i64>, always_fire_events: Option<i64>) -> bool;
    fn set_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<bool>;
    fn reset(&self) -> KResult<()>;
}


/// runout_gcode / insert_gcode: a template, or whatever a python plugin
/// put there (AFC sets runout_gcode = 1 and takes the runout itself)
#[derive(Clone)]
pub enum GcodeSlot {
    Template(Rc<Template>),
    Plugin(crate::printer::PObj),
}

pub struct RunoutHelper {
    me: Weak<RunoutHelper>,
    pub name: String,
    defined_sensor: RefCell<Option<Weak<dyn DefinedSensor>>>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    gcode: Rc<GCodeDispatch>,
    pub runout_pause: Cell<bool>,
    pub runout_gcode: RefCell<Option<GcodeSlot>>,
    pub immediate_runout_gcode: RefCell<Option<Rc<Template>>>,
    pub insert_gcode: RefCell<Option<GcodeSlot>>,
    pause_delay: f64,
    pub event_delay: Cell<f64>,
    pub runout_distance: Cell<f64>,
    pub min_event_systime: Cell<f64>,
    pub filament_present: Cell<bool>,
    pub sensor_enabled: Cell<bool>,
    pub smart: Cell<bool>,
    pub always_fire_events: Cell<bool>,
    runout_position: Cell<f64>,
    pub runout_elapsed: Cell<f64>,
    runout_distance_timer: RefCell<Option<Timer>>,
}

impl RunoutHelper {
    pub fn new(config: &ConfigWrapper, runout_distance: f64) -> KResult<Rc<Self>> {
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // Read config
        let runout_pause = config.getboolean("pause_on_runout", true)?;
        if runout_pause {
            printer.load_object(config, "pause_resume", true)?;
        }
        let gcode_macro = load_gcode_macro(config)?;
        let mut runout_gcode = None;
        if runout_pause || config.get_opt("runout_gcode")?.is_some() {
            runout_gcode = Some(gcode_macro.load_template(config, "runout_gcode", Some(""))?);
        }
        let mut insert_gcode = None;
        if config.get_opt("insert_gcode")?.is_some() {
            insert_gcode = Some(gcode_macro.load_template(config, "insert_gcode", None)?);
        }
        let pause_delay = config.getfloat("pause_delay", 0.5, Lim::new().min(0.0))?;
        let event_delay = config.getfloat("event_delay", 3.0, Lim::new().min(0.0))?;
        let smart = config.getboolean("smart", false)?;
        let always_fire_events = config.getboolean("always_fire_events", false)?;
        let rh = Rc::new_cyclic(|me| RunoutHelper {
            me: me.clone(),
            name: name.clone(),
            defined_sensor: RefCell::new(None),
            printer: printer.clone(),
            reactor: reactor.clone(),
            gcode: gcode.clone(),
            runout_pause: Cell::new(runout_pause),
            runout_gcode: RefCell::new(runout_gcode.map(GcodeSlot::Template)),
            immediate_runout_gcode: RefCell::new(None),
            insert_gcode: RefCell::new(insert_gcode.map(GcodeSlot::Template)),
            pause_delay,
            event_delay: Cell::new(event_delay),
            runout_distance: Cell::new(runout_distance),
            // Internal state
            min_event_systime: Cell::new(NEVER),
            filament_present: Cell::new(false),
            sensor_enabled: Cell::new(true),
            smart: Cell::new(smart),
            always_fire_events: Cell::new(always_fire_events),
            runout_position: Cell::new(0.0),
            runout_elapsed: Cell::new(0.0),
            runout_distance_timer: RefCell::new(None),
        });
        // Register commands and event handlers
        let r = rh.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                r.min_event_systime.set(r.reactor.monotonic() + 2.0);
                Ok(())
            }),
        );
        let r = rh.clone();
        gcode.register_mux_command(
            "QUERY_FILAMENT_SENSOR",
            "SENSOR",
            Some(&name),
            Rc::new(move |g| r.cmd_query_filament_sensor(g)),
            Some(CMD_QUERY_FILAMENT_SENSOR_HELP),
        )?;
        let r = rh.clone();
        gcode.register_mux_command(
            "SET_FILAMENT_SENSOR",
            "SENSOR",
            Some(&name),
            Rc::new(move |g| r.cmd_set_filament_sensor(g)),
            Some(CMD_SET_FILAMENT_SENSOR_HELP),
        )?;
        Ok(rh)
    }

    pub fn set_defined_sensor(&self, s: Weak<dyn DefinedSensor>) {
        *self.defined_sensor.borrow_mut() = Some(s);
    }

    fn sensor(&self) -> KResult<Rc<dyn DefinedSensor>> {
        self.defined_sensor.borrow().as_ref().and_then(|w| w.upgrade()).ok_or_else(|| KError::Other("filament sensor gone".into()))
    }

    fn rc(&self) -> Rc<RunoutHelper> {
        self.me.upgrade().expect("runout helper")
    }

    fn runout_event_handler(&self, eventtime: f64) -> KResult<()> {
        // a python plugin may have replaced it (AFC does)
        #[cfg(feature = "plugins")]
        if let Some(r) = crate::plugins::dispatch(self, "klippy.extras.filament_switch_sensor.RunoutHelper._runout_event_handler", &[&eventtime]) {
            return r.map(|_| ());
        }
        let immediate = self.immediate_runout_gcode.borrow().clone();
        if let Some(t) = immediate {
            self.exec_gcode("", &GcodeSlot::Template(t));
        }
        // Pausing from inside an event requires that the pause portion
        // of pause_resume execute immediately.
        if self.runout_distance.get() > 0.0 {
            if self.runout_distance_timer.borrow().is_none() {
                self.runout_position.set(self.sensor()?.get_extruder_pos(Some(eventtime))?);
                let w = self.me.clone();
                let timer = self.reactor.register_timer(
                    Box::new(move |et| match w.upgrade() {
                        Some(r) => r.pause_after_distance(et),
                        None => NEVER,
                    }),
                    NOW,
                );
                *self.runout_distance_timer.borrow_mut() = Some(timer);
            }
            Ok(())
        } else {
            self.execute_runout(eventtime)
        }
    }

    fn execute_runout(&self, eventtime: f64) -> KResult<()> {
        let mut pause_prefix = "";
        if self.runout_pause.get() {
            let pause_resume = self.printer.lookup::<PauseResume>("pause_resume")?;
            pause_resume.send_pause_command()?;
            pause_prefix = "PAUSE\n";
            self.printer.get_reactor().pause(eventtime + self.pause_delay)?;
        }
        let runout_gcode = self.runout_gcode.borrow().clone();
        if let Some(t) = runout_gcode {
            self.exec_gcode(pause_prefix, &t);
        } else {
            logging::exception_err("Script running error", &KError::Other("'NoneType' object has no attribute 'render'".into()));
            self.min_event_systime.set(self.reactor.monotonic() + self.event_delay.get());
        }
        self.reset_runout_distance_info();
        Ok(())
    }

    pub fn reset_runout_distance_info(&self) {
        self.runout_elapsed.set(0.0);
        if let Some(t) = self.runout_distance_timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
    }

    fn pause_after_distance(&self, eventtime: f64) -> f64 {
        let pos = match self.sensor().and_then(|s| s.get_extruder_pos(Some(eventtime))) {
            Ok(p) => p,
            Err(e) => {
                logging::exception_err("filament sensor position", &e);
                return NEVER;
            }
        };
        let runout_elapsed = py_max(0.0, pos - self.runout_position.get());
        if runout_elapsed < self.runout_distance.get() {
            self.runout_elapsed.set(runout_elapsed);
            eventtime + CHECK_RUNOUT_TIMEOUT
        } else {
            if let Err(e) = self.execute_runout(eventtime) {
                logging::exception_err("filament sensor runout", &e);
            }
            NEVER
        }
    }

    fn exec_gcode(&self, prefix: &str, template: &GcodeSlot) {
        let rendered = match template {
            GcodeSlot::Template(t) => t.render(None),
            // template.render() on a python plugin's object
            GcodeSlot::Plugin(p) => match p.call_method("render", &[]) {
                Some(r) => r.map(|v| v.str()),
                None => Err(KError::Other("object has no attribute 'render'".into())),
            },
        };
        let r = rendered.and_then(|s| self.gcode.run_script(&format!("{}{}\nM400", prefix, s)));
        if let Err(e) = r {
            logging::exception_err("Script running error", &e);
        }
        self.min_event_systime.set(self.reactor.monotonic() + self.event_delay.get());
    }

    pub fn note_filament_present(&self, eventtime: f64, is_filament_present: Option<bool>, force: bool, immediate: bool) -> KResult<()> {
        // a python plugin may have replaced it (AFC's debounced handler)
        #[cfg(feature = "plugins")]
        if let Some(r) = crate::plugins::dispatch(
            self,
            "klippy.extras.filament_switch_sensor.RunoutHelper.note_filament_present",
            &[&eventtime, &is_filament_present, &force, &immediate],
        ) {
            return r.map(|_| ());
        }
        let is_filament_present = is_filament_present.unwrap_or(self.filament_present.get());
        if is_filament_present == self.filament_present.get() && !force {
            return Ok(());
        }
        self.filament_present.set(is_filament_present);
        // with debouncing the event time is passed into us as we are
        // called with a delay so the current time is not the event time
        if eventtime < self.min_event_systime.get() || (!self.always_fire_events.get() && !self.sensor_enabled.get()) {
            // do not process during the initialization time, duplicates,
            // during the event delay time, while an event is running, or
            // when the sensor is disabled
            return Ok(());
        }
        let ev = if is_filament_present { "filament:insert" } else { "filament:runout" };
        self.printer.send_event(ev, &[&eventtime, &self.name])?;
        if !self.sensor_enabled.get() {
            return Ok(());
        }
        // Determine "printing" status
        let now = self.reactor.monotonic();
        let state_of = |name: &str| -> KResult<String> {
            let obj = self.printer.lookup_object(name)?;
            Ok(match obj.get_status(now) {
                Some(PyVal::Dict(d)) => d.get("state").map(|v| v.str()).unwrap_or_default(),
                _ => String::new(),
            })
        };
        let is_printing =
            if self.smart.get() { state_of("print_stats")? == "printing" } else { state_of("idle_timeout")? == "Printing" };
        // Perform filament action associated with status change (if any)
        if is_filament_present {
            if !is_printing && self.insert_gcode.borrow().is_some() {
                // insert detected
                self.min_event_systime.set(NEVER);
                logging::info(&format!("Filament Sensor {}: insert event detected, Time {:.2}", self.name, now));
                let r = self.rc();
                self.reactor.register_callback(
                    move |_| {
                        let insert_gcode = r.insert_gcode.borrow().clone();
                        if let Some(t) = insert_gcode {
                            r.exec_gcode("", &t);
                        }
                    },
                    NOW,
                );
            }
        } else if is_printing && self.runout_gcode.borrow().is_some() {
            // runout detected
            self.min_event_systime.set(NEVER);
            logging::info(&format!("Filament Sensor {}: runout event detected, Time {:.2}", self.name, now));
            let r = self.rc();
            self.reactor.register_callback(
                move |et| {
                    let res = if immediate { r.execute_runout(et) } else { r.runout_event_handler(et) };
                    if let Err(e) = res {
                        logging::exception_err("filament sensor runout", &e);
                    }
                },
                NOW,
            );
        }
        Ok(())
    }

    pub fn get_status(&self, eventtime: f64) -> PyVal {
        let mut status = IndexMap::new();
        status.insert("filament_detected".to_string(), PyVal::Bool(self.filament_present.get()));
        status.insert("enabled".to_string(), PyVal::Bool(self.sensor_enabled.get()));
        status.insert("smart".to_string(), PyVal::Bool(self.smart.get()));
        status.insert("always_fire_events".to_string(), PyVal::Bool(self.always_fire_events.get()));
        if let Ok(s) = self.sensor() {
            status.extend(s.sensor_get_status(eventtime));
        }
        PyVal::Dict(status)
    }

    fn cmd_query_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let msg = format!(
            "Filament Sensor {}: filament {}",
            self.name,
            if self.filament_present.get() { "detected" } else { "not detected" }
        );
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    fn cmd_set_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let b = Lim::new().min(0).max(1);
        let enable = gcmd.get_int_opt("ENABLE", b.clone())?;
        let reset = gcmd.get_int_opt("RESET", b.clone())?;
        let smart = gcmd.get_int_opt("SMART", b.clone())?;
        let always_fire_events = gcmd.get_int_opt("ALWAYS_FIRE_EVENTS", b)?;
        let sensor = self.sensor()?;
        let mut reset_needed = false;
        if enable.is_none() && reset.is_none() && smart.is_none() && always_fire_events.is_none() && sensor.get_info(gcmd)? {
            return Ok(());
        }
        if let Some(e) = enable {
            self.sensor_enabled.set(e != 0);
        }
        if reset.map(|r| r != 0).unwrap_or(false) {
            reset_needed = true;
        }
        if let Some(s) = smart {
            self.smart.set(s != 0);
        }
        if let Some(a) = always_fire_events {
            self.always_fire_events.set(a != 0);
        }
        if sensor.set_filament_sensor(gcmd)? {
            reset_needed = true;
        }
        if sensor.reset_needed(enable, always_fire_events) {
            reset_needed = true;
        }
        if reset_needed {
            sensor.reset()?;
        }
        Ok(())
    }
}

pub struct SwitchSensor {
    printer: Rc<Printer>,
    check_on_print_start: Cell<bool>,
    reactor: Rc<Reactor>,
    mcu: RefCell<Option<Rc<Mcu>>>,
    pub runout_helper: Rc<RunoutHelper>,
}

impl SwitchSensor {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode_macro = load_gcode_macro(config)?;
        let buttons = printer.load_as::<PrinterButtons>(config, "buttons")?;
        let switch_pin = config.get("switch_pin", Dflt::Required)?;
        let runout_distance = config.getfloat("runout_distance", 0.0, Lim::new().min(0.0))?;
        // the button callback needs the sensor: route it through a slot
        let slot: Rc<RefCell<Weak<SwitchSensor>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        buttons.register_debounce_button(
            &switch_pin,
            Rc::new(move |eventtime, state| match sl.borrow().upgrade() {
                Some(s) => s.runout_helper.note_filament_present(eventtime, Some(state != 0), false, false),
                None => Ok(()),
            }),
            config,
        )?;

        let check_on_print_start = config.getboolean("check_on_print_start", false)?;
        let runout_helper = RunoutHelper::new(config, runout_distance)?;
        if config.get_opt("immediate_runout_gcode")?.is_some() {
            *runout_helper.immediate_runout_gcode.borrow_mut() =
                Some(gcode_macro.load_template(config, "immediate_runout_gcode", Some(""))?);
        }
        let s = Rc::new(SwitchSensor {
            printer: printer.clone(),
            check_on_print_start: Cell::new(check_on_print_start),
            reactor: printer.get_reactor(),
            mcu: RefCell::new(None),
            runout_helper: runout_helper.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&s);
        let ds: Rc<dyn DefinedSensor> = s.clone();
        runout_helper.set_defined_sensor(Rc::downgrade(&ds));
        let ss = s.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                *ss.mcu.borrow_mut() = Some(ss.printer.lookup::<Mcu>("mcu")?);
                Ok(())
            }),
        );
        let ss = s.clone();
        printer.register_event_handler(
            "idle_timeout:printing",
            Rc::new(move |_| {
                if ss.check_on_print_start.get() {
                    ss.runout_helper.note_filament_present(ss.reactor.monotonic(), None, true, true)?;
                }
                Ok(())
            }),
        );
        Ok(s)
    }

    fn get_sensor_status(&self) -> String {
        let rh = &self.runout_helper;
        let tf = |b: bool| if b { "true" } else { "false" };
        format!(
            "Filament Sensor {}: {}\nFilament Detected: {}\nRunout Distance: {:.2}\nRunout Elapsed: {:.2}\nSmart: {}\nAlways Fire Events: {}\nCheck on Print Start: {}",
            rh.name,
            if rh.sensor_enabled.get() { "enabled" } else { "disabled" },
            tf(rh.filament_present.get()),
            rh.runout_distance.get(),
            rh.runout_elapsed.get(),
            tf(rh.smart.get()),
            tf(rh.always_fire_events.get()),
            tf(self.check_on_print_start.get())
        )
    }
}

impl DefinedSensor for SwitchSensor {
    fn get_extruder_pos(&self, eventtime: Option<f64>) -> KResult<f64> {
        let eventtime = eventtime.unwrap_or_else(|| self.reactor.monotonic());
        let mcu = self.mcu.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object is not callable".into()))?;
        let print_time = mcu.estimated_print_time(eventtime);
        let extruder = self.printer.lookup::<ToolHead>("toolhead")?.get_extruder();
        Ok(extruder.find_past_position(print_time))
    }

    fn sensor_get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let mut d = IndexMap::new();
        d.insert("runout_distance".to_string(), PyVal::Float(self.runout_helper.runout_distance.get()));
        d.insert("runout_elapsed".to_string(), PyVal::Float(self.runout_helper.runout_elapsed.get()));
        d.insert("check_on_print_start".to_string(), PyVal::Bool(self.check_on_print_start.get()));
        d
    }

    fn get_info(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        let runout_distance = gcmd.get_float_opt("RUNOUT_DISTANCE", Lim::new().min(0.0))?;
        let check_on_print_start = gcmd.get_int_opt("CHECK_ON_PRINT_START", Lim::new().min(0).max(1))?;
        if runout_distance.is_none() && check_on_print_start.is_none() {
            gcmd.respond_info(&self.get_sensor_status(), true);
            return Ok(true);
        }
        Ok(false)
    }

    fn reset_needed(&self, enable: Option<i64>, always_fire_events: Option<i64>) -> bool {
        // (python compares the int ENABLE with sensor_enabled, which the
        // caller just set from it)
        if let Some(e) = enable {
            if (e != 0) != self.runout_helper.sensor_enabled.get() {
                return true;
            }
        }
        always_fire_events.map(|a| a != 0).unwrap_or(false)
    }

    fn set_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        let runout_distance = gcmd.get_float_opt("RUNOUT_DISTANCE", Lim::new().min(0.0))?;
        let check_on_print_start = gcmd.get_int_opt("CHECK_ON_PRINT_START", Lim::new().min(0).max(1))?;
        if let Some(r) = runout_distance {
            self.runout_helper.runout_distance.set(r);
        }
        if let Some(c) = check_on_print_start {
            self.check_on_print_start.set(c != 0);
        }
        // No reset is needed when changing the runout_distance, so we always
        // return False
        Ok(false)
    }

    fn reset(&self) -> KResult<()> {
        self.runout_helper.reset_runout_distance_info();
        self.runout_helper.note_filament_present(self.reactor.monotonic(), Some(self.runout_helper.filament_present.get()), true, false)
    }
}

impl PrinterObject for SwitchSensor {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.runout_helper.get_status(eventtime))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(SwitchSensor::new(config)?)
}
