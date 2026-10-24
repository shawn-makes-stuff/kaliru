// Support for enable pins on stepper motor drivers
// (port of klippy/extras/stepper_enable.py)
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::DigitalOutPin;
use crate::pins::{any_to_digital_out, PinsObject};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const DISABLE_STALL_TIME: f64 = 0.100;

// Tracking of shared stepper enable pins
pub struct StepperEnablePin {
    mcu_enable: Option<Rc<dyn DigitalOutPin>>,
    enable_count: Cell<i64>,
    pub is_dedicated: Cell<bool>,
}

impl StepperEnablePin {
    fn new(mcu_enable: Option<Rc<dyn DigitalOutPin>>, enable_count: i64) -> Rc<Self> {
        Rc::new(StepperEnablePin { mcu_enable, enable_count: Cell::new(enable_count), is_dedicated: Cell::new(true) })
    }

    pub fn set_enable(&self, print_time: f64) -> KResult<()> {
        if self.enable_count.get() == 0 {
            if let Some(e) = &self.mcu_enable {
                e.set_digital(print_time, true)?;
            }
        }
        self.enable_count.set(self.enable_count.get() + 1);
        Ok(())
    }

    pub fn set_disable(&self, print_time: f64) -> KResult<()> {
        self.enable_count.set(self.enable_count.get() - 1);
        if self.enable_count.get() == 0 {
            if let Some(e) = &self.mcu_enable {
                e.set_digital(print_time, false)?;
            }
        }
        Ok(())
    }
}

pub fn setup_enable_pin(printer: &Printer, pin: Option<&str>) -> KResult<Rc<StepperEnablePin>> {
    let Some(pin) = pin else {
        // No enable line (stepper always enabled)
        let enable = StepperEnablePin::new(None, 9999);
        enable.is_dedicated.set(false);
        return Ok(enable);
    };
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let pin_params = ppins.0.borrow_mut().lookup_pin(pin, true, false, Some("stepper_enable"))?;
    let existing = pin_params.borrow().class.clone();
    if let Some(enable) = existing {
        // Shared enable line
        let enable = enable
            .downcast::<StepperEnablePin>()
            .map_err(|_| KError::Other("'stepper_enable' pin class has an unexpected type".into()))?;
        enable.is_dedicated.set(false);
        return Ok(enable);
    }
    let chip = pin_params.borrow().chip.clone();
    let mcu_enable = any_to_digital_out(chip.setup_pin("digital_out", &pin_params).map_err(KError::Pins)?)
        .ok_or_else(|| KError::Other("digital_out pin of an unexpected type".into()))?;
    mcu_enable.setup_max_duration(0.0);
    let enable = StepperEnablePin::new(Some(mcu_enable), 0);
    pin_params.borrow_mut().class = Some(enable.clone());
    Ok(enable)
}

/// an enable state callback: cb(print_time, is_enabled)
pub type StateCallback = Rc<dyn Fn(f64, bool) -> KResult<()>>;

// Enable line tracking for each stepper motor
pub struct EnableTracking {
    me: std::rc::Weak<EnableTracking>,
    stepper: Rc<McuStepper>,
    enable: Rc<StepperEnablePin>,
    callbacks: RefCell<Vec<StateCallback>>,
    is_enabled: Cell<bool>,
}

impl EnableTracking {
    fn new(stepper: &Rc<McuStepper>, enable: Rc<StepperEnablePin>) -> KResult<Rc<Self>> {
        let et = Rc::new_cyclic(|me| EnableTracking {
            me: me.clone(),
            stepper: stepper.clone(),
            enable,
            callbacks: RefCell::new(vec![]),
            is_enabled: Cell::new(false),
        });
        et.add_enable_callback()?;
        Ok(et)
    }

    fn add_enable_callback(&self) -> KResult<()> {
        let w = self.me.clone();
        self.stepper.add_active_callback(Rc::new(move |print_time| match w.upgrade() {
            Some(et) => et.motor_enable(print_time),
            None => Ok(()),
        }))
    }

    pub fn register_state_callback(&self, callback: StateCallback) {
        self.callbacks.borrow_mut().push(callback);
    }

    pub fn motor_enable(&self, print_time: f64) -> KResult<()> {
        if !self.is_enabled.get() {
            let cbs = self.callbacks.borrow().clone();
            for cb in cbs {
                cb(print_time, true)?;
            }
            self.enable.set_enable(print_time)?;
            self.is_enabled.set(true);
        }
        Ok(())
    }

    pub fn motor_disable(&self, print_time: f64) -> KResult<()> {
        if self.is_enabled.get() {
            // Enable stepper on future stepper movement
            let cbs = self.callbacks.borrow().clone();
            for cb in cbs {
                cb(print_time, false)?;
            }
            self.enable.set_disable(print_time)?;
            self.is_enabled.set(false);
            self.add_enable_callback()?;
        }
        Ok(())
    }

    /// self.stepper
    pub fn stepper(&self) -> Rc<McuStepper> {
        self.stepper.clone()
    }

    pub fn is_motor_enabled(&self) -> bool {
        self.is_enabled.get()
    }

    pub fn has_dedicated_enable(&self) -> bool {
        self.enable.is_dedicated.get()
    }
}

const CMD_SET_STEPPER_ENABLE_HELP: &str = "Enable/disable individual stepper by name";

// Global stepper enable line tracking
pub struct PrinterStepperEnable {
    printer: Rc<Printer>,
    pub(crate) enable_lines: RefCell<IndexMap<String, Rc<EnableTracking>>>,
}

impl PrinterStepperEnable {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let se = Rc::new(PrinterStepperEnable { printer: printer.clone(), enable_lines: RefCell::new(IndexMap::new()) });
        let s = se.clone();
        printer.register_event_handler("gcode:request_restart", Rc::new(move |_| s.motor_off()));
        // Register M18/M84 commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let s = se.clone();
        gcode.register_command("M18", Some(Rc::new(move |_| s.motor_off())), false, None)?;
        let s = se.clone();
        gcode.register_command("M84", Some(Rc::new(move |_| s.motor_off())), false, None)?;
        let s = se.clone();
        gcode.register_command(
            "SET_STEPPER_ENABLE",
            Some(Rc::new(move |g| s.cmd_set_stepper_enable(g))),
            false,
            Some(CMD_SET_STEPPER_ENABLE_HELP),
        )?;
        Ok(se)
    }

    pub fn register_stepper(&self, config: &ConfigWrapper, mcu_stepper: &Rc<McuStepper>) -> KResult<()> {
        let name = mcu_stepper.get_name(false);
        let enable = setup_enable_pin(&self.printer, config.get_opt("enable_pin")?.as_deref())?;
        let et = EnableTracking::new(mcu_stepper, enable)?;
        self.enable_lines.borrow_mut().insert(name, et);
        Ok(())
    }

    pub fn set_motors_enable(&self, stepper_names: &[String], enable: bool) -> KResult<bool> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        // Flush steps to ensure all auto enable callbacks invoked
        toolhead.flush_step_generation()?;
        let mut print_time = None;
        let mut did_change = false;
        for stepper_name in stepper_names {
            let el = self
                .enable_lines
                .borrow()
                .get(stepper_name)
                .cloned()
                .ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(stepper_name)))?;
            if el.is_motor_enabled() == enable {
                continue;
            }
            let pt = match print_time {
                Some(pt) => pt,
                None => {
                    // Dwell for sufficient delay from any previous auto enable
                    if !enable {
                        toolhead.dwell(DISABLE_STALL_TIME)?;
                    }
                    let pt = toolhead.get_last_move_time()?;
                    print_time = Some(pt);
                    pt
                }
            };
            if enable {
                el.motor_enable(pt)?;
            } else {
                el.motor_disable(pt)?;
            }
            did_change = true;
        }
        // Dwell to ensure sufficient delay prior to a future auto enable
        if did_change && !enable {
            toolhead.dwell(DISABLE_STALL_TIME)?;
        }
        Ok(did_change)
    }

    pub fn motor_off(&self) -> KResult<()> {
        self.set_motors_enable(&self.get_steppers(), false)?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.get_kinematics().clear_homing_state("xyz");
        self.printer.send_event("stepper_enable:motor_off", &[])
    }

    fn cmd_set_stepper_enable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let stepper_name = gcmd.get_opt("STEPPER");
        let known = stepper_name.as_ref().map(|n| self.enable_lines.borrow().contains_key(n)).unwrap_or(false);
        let Some(stepper_name) = stepper_name.filter(|_| known) else {
            let shown = gcmd.get_opt("STEPPER").unwrap_or_else(|| "None".into());
            gcmd.respond_info(&format!("SET_STEPPER_ENABLE: Invalid stepper \"{}\"", shown), true);
            return Ok(());
        };
        let stepper_enable = gcmd.get_int("ENABLE", Some(1), Lim::new())?;
        self.set_motors_enable(&[stepper_name.clone()], stepper_enable != 0)?;
        if stepper_enable != 0 {
            logging::info(&format!("{} has been manually enabled", stepper_name));
        } else {
            logging::info(&format!("{} has been manually disabled", stepper_name));
        }
        Ok(())
    }

    pub fn lookup_enable(&self, name: &str) -> KResult<Rc<EnableTracking>> {
        self.enable_lines.borrow().get(name).cloned().ok_or_else(|| KError::config(format!("Unknown stepper '{}'", name)))
    }

    pub fn get_steppers(&self) -> Vec<String> {
        self.enable_lines.borrow().keys().cloned().collect()
    }
}

impl PrinterObject for PrinterStepperEnable {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let steppers: IndexMap<String, PyVal> =
            self.enable_lines.borrow().iter().map(|(name, et)| (name.clone(), PyVal::Bool(et.is_motor_enabled()))).collect();
        let mut d = IndexMap::new();
        d.insert("steppers".to_string(), PyVal::Dict(steppers));
        Some(PyVal::Dict(d))
    }
}

/// printer.lookup_object("stepper_enable").motor_off()
pub fn motor_off(printer: &Printer) -> KResult<()> {
    printer.lookup::<PrinterStepperEnable>("stepper_enable")?.motor_off()
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterStepperEnable::new(config)?)
}
