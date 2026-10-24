// Filament Motion Sensor Module (port of klippy/extras/filament_motion_sensor.py)
//
// Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::buttons::PrinterButtons;
use crate::extras::filament_switch_sensor::{DefinedSensor, RunoutHelper};
use crate::gcode::GCodeCommand;
use crate::kinematics::extruder::PrinterExtruder;
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::toolhead::ExtraAxis;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const CHECK_RUNOUT_TIMEOUT: f64 = 0.250;

/// extruder.find_past_position(print_time) of an extruder-like object
pub fn find_past_position(obj: &PObj, print_time: f64) -> KResult<f64> {
    if let Some(pe) = obj.as_any().downcast_ref::<PrinterExtruder>() {
        return Ok(pe.find_past_position(print_time));
    }
    if let Some(es) = obj.as_any().downcast_ref::<crate::extras::extruder_stepper::PrinterExtruderStepper>() {
        return Ok(es.find_past_position(print_time));
    }
    Err(KError::Other("object has no attribute 'find_past_position'".into()))
}

pub struct EncoderSensor {
    printer: Rc<Printer>,
    extruder_name: String,
    detection_length: Cell<f64>,
    reactor: Rc<Reactor>,
    runout_helper: Rc<RunoutHelper>,
    extruder: RefCell<Option<PObj>>,
    mcu: RefCell<Option<Rc<Mcu>>>,
    filament_runout_pos: Cell<Option<f64>>,
    timer: RefCell<Option<Timer>>,
}

impl EncoderSensor {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        // Read config
        let printer = config.get_printer();
        let switch_pin = config.get("switch_pin", Dflt::Required)?;
        let extruder_name = config.get("extruder", Dflt::Required)?;
        let detection_length = config.getfloat("detection_length", 7.0, Lim::new().above(0.0))?;
        // Configure pins
        let buttons = printer
            .load_object(config, "buttons", true)?
            .and_then(downcast_rc::<PrinterButtons>)
            .ok_or_else(|| KError::Other("buttons has an unexpected type".into()))?;
        let slot: Rc<RefCell<Weak<EncoderSensor>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        buttons.register_buttons(
            &[&switch_pin],
            Rc::new(move |eventtime, _state| match sl.borrow().upgrade() {
                Some(s) => s.encoder_event(eventtime),
                None => Ok(()),
            }),
        )?;
        // Get printer objects
        let runout_helper = RunoutHelper::new(config, 0.0)?;
        let s = Rc::new(EncoderSensor {
            printer: printer.clone(),
            extruder_name,
            detection_length: Cell::new(detection_length),
            reactor: printer.get_reactor(),
            runout_helper: runout_helper.clone(),
            extruder: RefCell::new(None),
            mcu: RefCell::new(None),
            // Initialise internal state
            filament_runout_pos: Cell::new(None),
            timer: RefCell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&s);
        let ds: Rc<dyn DefinedSensor> = s.clone();
        runout_helper.set_defined_sensor(Rc::downgrade(&ds));
        // Register commands and event handlers
        let ss = s.clone();
        printer.register_event_handler("klippy:ready", Rc::new(move |_| ss.handle_ready()));
        let ss = s.clone();
        printer.register_event_handler("idle_timeout:printing", Rc::new(move |_| ss.set_timer(NOW)));
        for ev in ["idle_timeout:ready", "idle_timeout:idle"] {
            let ss = s.clone();
            printer.register_event_handler(ev, Rc::new(move |_| ss.set_timer(NEVER)));
        }
        Ok(s)
    }

    fn set_timer(&self, waketime: f64) -> KResult<()> {
        match self.timer.borrow().as_ref() {
            Some(t) => {
                self.reactor.update_timer(t, waketime);
                Ok(())
            }
            None => Err(KError::Other("'EncoderSensor' object has no attribute '_extruder_pos_update_timer'".into())),
        }
    }

    fn update_filament_runout_pos(&self, eventtime: Option<f64>) -> KResult<()> {
        let pos = self.get_extruder_pos(eventtime)?;
        self.filament_runout_pos.set(Some(pos + self.detection_length.get()));
        Ok(())
    }

    fn handle_ready(self: &Rc<Self>) -> KResult<()> {
        *self.extruder.borrow_mut() = Some(self.printer.lookup_object(&self.extruder_name)?);
        *self.mcu.borrow_mut() = Some(self.printer.lookup::<Mcu>("mcu")?);
        self.update_filament_runout_pos(None)?;
        let w = Rc::downgrade(self);
        let timer = self.reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.extruder_pos_update_event(eventtime),
                None => NEVER,
            }),
            NEVER,
        );
        *self.timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn extruder_pos_update_event(&self, eventtime: f64) -> f64 {
        let r = self.get_extruder_pos(Some(eventtime)).and_then(|extruder_pos| {
            // Check for filament runout
            let present = extruder_pos < self.filament_runout_pos.get().unwrap_or(0.0);
            self.runout_helper.note_filament_present(eventtime, Some(present), false, false)
        });
        if let Err(e) = r {
            logging::exception_err("filament motion sensor update", &e);
        }
        eventtime + CHECK_RUNOUT_TIMEOUT
    }

    fn encoder_event(&self, eventtime: f64) -> KResult<()> {
        if self.extruder.borrow().is_some() {
            self.update_filament_runout_pos(Some(eventtime))?;
            // Check for filament insertion
            // Filament is always assumed to be present on an encoder event
            self.runout_helper.note_filament_present(eventtime, Some(true), false, false)?;
        }
        Ok(())
    }

    fn get_sensor_status(&self) -> String {
        let rh = &self.runout_helper;
        let tf = |b: bool| if b { "true" } else { "false" };
        format!(
            "Filament Sensor {}: {}\nFilament Detected: {}\nDetection Length: {:.2}\nSmart: {}\nAlways Fire Events: {}",
            rh.name,
            if rh.sensor_enabled.get() { "enabled" } else { "disabled" },
            tf(rh.filament_present.get()),
            self.detection_length.get(),
            tf(rh.smart.get()),
            tf(rh.always_fire_events.get())
        )
    }
}

impl DefinedSensor for EncoderSensor {
    fn get_extruder_pos(&self, eventtime: Option<f64>) -> KResult<f64> {
        let eventtime = eventtime.unwrap_or_else(|| self.reactor.monotonic());
        let mcu = self.mcu.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object is not callable".into()))?;
        let print_time = mcu.estimated_print_time(eventtime);
        let extruder = self.extruder.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'find_past_position'".into()))?;
        find_past_position(&extruder, print_time)
    }

    fn sensor_get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let mut d = IndexMap::new();
        d.insert("detection_length".to_string(), PyVal::Float(self.detection_length.get()));
        d
    }

    fn get_info(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        if gcmd.get_float_opt("DETECTION_LENGTH", Lim::new().min(0.0))?.is_none() {
            gcmd.respond_info(&self.get_sensor_status(), true);
            return Ok(true);
        }
        Ok(false)
    }

    fn reset_needed(&self, enable: Option<i64>, _always_fire_events: Option<i64>) -> bool {
        enable.map(|e| e != 0).unwrap_or(false) && !self.runout_helper.sensor_enabled.get()
    }

    fn set_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        let mut reset_needed = false;
        if let Some(detection_length) = gcmd.get_float_opt("DETECTION_LENGTH", Lim::new().min(0.0))? {
            if detection_length != self.detection_length.get() {
                reset_needed = true;
            }
            self.detection_length.set(detection_length);
        }
        Ok(reset_needed)
    }

    fn reset(&self) -> KResult<()> {
        self.update_filament_runout_pos(None)?;
        self.runout_helper.note_filament_present(self.reactor.monotonic(), Some(true), false, false)
    }
}

impl PrinterObject for EncoderSensor {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.runout_helper.get_status(eventtime))
    }
    fn teardown(&self) {
        *self.extruder.borrow_mut() = None;
        if let Some(t) = self.timer.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(EncoderSensor::new(config)?)
}
