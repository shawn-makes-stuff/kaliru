// Support for filament width sensor
// (port of klippy/extras/tsl1401cl_filament_width_sensor.py)
//
// Copyright (C) 2019  Mustafa YILDIZ <mydiz@hotmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::AdcPin;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_round};
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::toolhead::ToolHead;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const ADC_REPORT_TIME: f64 = 0.500;
const ADC_SAMPLE_TIME: f64 = 0.001;
const ADC_SAMPLE_COUNT: i64 = 8;
const MEASUREMENT_INTERVAL_MM: f64 = 10.0;

pub struct FilamentWidthSensor {
    me: Weak<FilamentWidthSensor>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    nominal_filament_dia: f64,
    measurement_delay: f64,
    max_diameter: f64,
    min_diameter: f64,
    is_active: Cell<bool>,
    // filament array [position, filamentWidth]
    filament_array: RefCell<Vec<(f64, f64)>>,
    last_filament_width_reading: Cell<f64>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    _mcu_adc: Rc<dyn AdcPin>,
    extrude_factor_update_timer: RefCell<Option<Timer>>,
    gcode: Rc<GCodeDispatch>,
}

impl FilamentWidthSensor {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let pin = config.get("pin", Dflt::Required)?;
        let nominal_filament_dia = config.getfloat("default_nominal_filament_diameter", Dflt::Required, Lim::new().above(1.0))?;
        let measurement_delay = config.getfloat("measurement_delay", Dflt::Required, Lim::new().above(0.0))?;
        let measurement_max_difference = config.getfloat("max_difference", Dflt::Required, Lim::new().above(0.0))?;
        let slot: Rc<RefCell<Weak<FilamentWidthSensor>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(s) => s.handle_ready(),
                None => Ok(()),
            }),
        );
        // Start adc
        let mcu_adc = crate::pins::setup_adc_pin(&printer, &pin)?;
        mcu_adc.setup_adc_sample(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT, 0.0, 1.0, 0);
        let sl = slot.clone();
        mcu_adc.setup_adc_callback(
            ADC_REPORT_TIME,
            Rc::new(move |_read_time, read_value| {
                if let Some(s) = sl.borrow().upgrade() {
                    // read sensor value
                    s.last_filament_width_reading.set(py_round(read_value * 5.0, 2));
                }
                Ok(())
            }),
        );
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let s = Rc::new_cyclic(|me| FilamentWidthSensor {
            me: me.clone(),
            printer: printer.clone(),
            reactor: reactor.clone(),
            nominal_filament_dia,
            measurement_delay,
            max_diameter: nominal_filament_dia + measurement_max_difference,
            min_diameter: nominal_filament_dia - measurement_max_difference,
            is_active: Cell::new(true),
            filament_array: RefCell::new(vec![]),
            last_filament_width_reading: Cell::new(0.0),
            toolhead: RefCell::new(None),
            _mcu_adc: mcu_adc,
            extrude_factor_update_timer: RefCell::new(None),
            gcode: gcode.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&s);
        // extrude factor updating
        let w = s.me.clone();
        *s.extrude_factor_update_timer.borrow_mut() = Some(reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.extrude_factor_update_event(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));
        // Register commands
        type FCmd = fn(&FilamentWidthSensor, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, FCmd); 4] = [
            ("QUERY_FILAMENT_WIDTH", FilamentWidthSensor::cmd_m407),
            ("RESET_FILAMENT_WIDTH_SENSOR", FilamentWidthSensor::cmd_clear_filament_array),
            ("DISABLE_FILAMENT_WIDTH_SENSOR", FilamentWidthSensor::cmd_m406),
            ("ENABLE_FILAMENT_WIDTH_SENSOR", FilamentWidthSensor::cmd_m405),
        ];
        for (cmd, f) in cmds {
            let w = s.me.clone();
            gcode.register_command(
                cmd,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(s) => f(&s, g),
                    None => Ok(()),
                })),
                false,
                None,
            )?;
        }
        Ok(s)
    }

    fn timer(&self) -> Option<Timer> {
        self.extrude_factor_update_timer.borrow().clone()
    }

    // Initialization
    fn handle_ready(&self) -> KResult<()> {
        // Load printer objects
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        // Start extrude factor update timer
        if let Some(t) = self.timer() {
            self.reactor.update_timer(&t, NOW);
        }
        Ok(())
    }

    fn update_filament_array(&self, last_epos: f64) {
        let reading = self.last_filament_width_reading.get();
        let mut fa = self.filament_array.borrow_mut();
        // Fill array
        match fa.last() {
            Some(last) => {
                // Get last reading position in array & calculate next
                // reading position
                let next_reading_position = last.0 + MEASUREMENT_INTERVAL_MM;
                if next_reading_position <= (last_epos + self.measurement_delay) {
                    fa.push((last_epos + self.measurement_delay, reading));
                }
            }
            // add first item to array
            None => fa.push((self.measurement_delay + last_epos, reading)),
        }
    }

    fn extrude_factor_update_event(&self, eventtime: f64) -> f64 {
        if let Err(e) = self.update_extrude_factor() {
            logging::exception_err("filament width update", &e);
        }
        if self.is_active.get() {
            eventtime + 1.0
        } else {
            NEVER
        }
    }

    fn update_extrude_factor(&self) -> KResult<()> {
        // Update extrude factor
        let Some(toolhead) = self.toolhead.borrow().clone() else { return Ok(()) };
        let last_epos = toolhead.get_position()[3];
        // Update filament array for lastFilamentWidthReading
        self.update_filament_array(last_epos);
        // Does filament exists
        if self.last_filament_width_reading.get() > 0.5 {
            let first = self.filament_array.borrow().first().copied();
            if let Some(first) = first {
                // Get first position in filament array
                let pending_position = first.0;
                if pending_position <= last_epos {
                    // Get first item in filament_array queue
                    let item = self.filament_array.borrow_mut().remove(0);
                    let filament_width = item.1;
                    if filament_width <= self.max_diameter && filament_width >= self.min_diameter {
                        let percentage = (self.nominal_filament_dia.powf(2.0) / filament_width.powf(2.0) * 100.0).round_ties_even() as i64;
                        self.gcode.run_script(&format!("M221 S{}", percentage))?;
                    } else {
                        self.gcode.run_script("M221 S100")?;
                    }
                }
            }
        } else {
            self.gcode.run_script("M221 S100")?;
            self.filament_array.borrow_mut().clear();
        }
        Ok(())
    }

    fn cmd_m407(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reading = self.last_filament_width_reading.get();
        let response = if reading > 0.0 { format!("Filament dia (measured mm): {}", py_float_repr(reading)) } else { "Filament NOT present".to_string() };
        gcmd.respond_info(&response, true);
        Ok(())
    }

    fn cmd_clear_filament_array(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.filament_array.borrow_mut().clear();
        gcmd.respond_info("Filament width measurements cleared!", true);
        // Set extrude multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")
    }

    fn cmd_m405(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut response = "Filament width sensor Turned On";
        if self.is_active.get() {
            response = "Filament width sensor is already On";
        } else {
            self.is_active.set(true);
            // Start extrude factor update timer
            if let Some(t) = self.timer() {
                self.reactor.update_timer(&t, NOW);
            }
        }
        gcmd.respond_info(response, true);
        Ok(())
    }

    fn cmd_m406(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut response = "Filament width sensor Turned Off";
        if !self.is_active.get() {
            response = "Filament width sensor is already Off";
        } else {
            self.is_active.set(false);
            // Stop extrude factor update timer
            if let Some(t) = self.timer() {
                self.reactor.update_timer(&t, NEVER);
            }
            // Clear filament array
            self.filament_array.borrow_mut().clear();
            // Set extrude multiplier to 100%
            self.gcode.run_script_from_command("M221 S100")?;
        }
        gcmd.respond_info(response, true);
        Ok(())
    }
}

impl PrinterObject for FilamentWidthSensor {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        if let Some(t) = self.extrude_factor_update_timer.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(FilamentWidthSensor::new(config)?)
}
