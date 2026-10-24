// Heater/sensor verification code (port of klippy/extras/verify_heater.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::heaters::{Heater, PrinterHeaters};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::reactor::{Timer, NEVER, NOW};
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const HINT_THERMAL: &str = "
See the 'verify_heater' section in docs/Config_Reference.md
for the parameters that control this check.
";

pub struct HeaterCheck {
    printer: Rc<Printer>,
    heater_name: String,
    heater: RefCell<Option<Rc<Heater>>>,
    hysteresis: f64,
    max_error: f64,
    heating_gain: f64,
    check_gain_time: f64,
    approaching_target: Cell<bool>,
    starting_approach: Cell<bool>,
    last_target: Cell<f64>,
    goal_temp: Cell<f64>,
    error: Cell<f64>,
    goal_systime: Cell<f64>,
    check_timer: RefCell<Option<Timer>>,
}

impl HeaterCheck {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let heater_name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let hysteresis = config.getfloat("hysteresis", 5.0, Lim::new().min(0.0))?;
        let max_error = config.getfloat("max_error", 120.0, Lim::new().min(0.0))?;
        let heating_gain = config.getfloat("heating_gain", 2.0, Lim::new().above(0.0))?;
        let default_gain_time = if heater_name == "heater_bed" { 60.0 } else { 20.0 };
        let check_gain_time = config.getfloat("check_gain_time", default_gain_time, Lim::new().min(1.0))?;
        let hc = Rc::new(HeaterCheck {
            printer: printer.clone(),
            heater_name,
            heater: RefCell::new(None),
            hysteresis,
            max_error,
            heating_gain,
            check_gain_time,
            approaching_target: Cell::new(false),
            starting_approach: Cell::new(false),
            last_target: Cell::new(0.0),
            goal_temp: Cell::new(0.0),
            error: Cell::new(0.0),
            goal_systime: Cell::new(NEVER),
            check_timer: RefCell::new(None),
        });
        let h = hc.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| h.handle_connect()));
        let h = hc.clone();
        printer.register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                h.handle_shutdown();
                Ok(())
            }),
        );
        Ok(hc)
    }

    fn handle_connect(self: &Rc<Self>) -> KResult<()> {
        if self.printer.start_arg("debugoutput").is_some() {
            // Disable verify_heater if outputting to a debug file
            return Ok(());
        }
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        *self.heater.borrow_mut() = Some(pheaters.lookup_heater(&self.heater_name)?);
        logging::info(&format!("Starting heater checks for {}", self.heater_name));
        let reactor = self.printer.get_reactor();
        let w = Rc::downgrade(self);
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(h) => h.check_event(eventtime),
                None => NEVER,
            }),
            NOW,
        );
        *self.check_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn handle_shutdown(&self) {
        if let Some(t) = self.check_timer.borrow().as_ref() {
            self.printer.get_reactor().update_timer(t, NEVER);
        }
    }

    fn check_event(&self, eventtime: f64) -> f64 {
        let heater = self.heater.borrow().clone().expect("heater");
        let (temp, target) = heater.get_temp(eventtime);
        if temp >= target - self.hysteresis || target <= 0.0 {
            // Temperature near target - reset checks
            if self.approaching_target.get() && target != 0.0 {
                logging::info(&format!("Heater {} within range of {:.3}", self.heater_name, target));
            }
            self.approaching_target.set(false);
            self.starting_approach.set(false);
            if temp <= target + self.hysteresis {
                self.error.set(0.0);
            }
            self.last_target.set(target);
            return eventtime + 1.0;
        }
        self.error.set(self.error.get() + ((target - self.hysteresis) - temp));
        if !self.approaching_target.get() {
            if target != self.last_target.get() {
                // Target changed - reset checks
                logging::info(&format!("Heater {} approaching new target of {:.3}", self.heater_name, target));
                self.approaching_target.set(true);
                self.starting_approach.set(true);
                self.goal_temp.set(temp + self.heating_gain);
                self.goal_systime.set(eventtime + self.check_gain_time);
            } else if self.error.get() >= self.max_error {
                // Failure due to inability to maintain target temperature
                return self.heater_fault();
            }
        } else if temp >= self.goal_temp.get() {
            // Temperature approaching target - reset checks
            self.starting_approach.set(false);
            self.error.set(0.0);
            self.goal_temp.set(temp + self.heating_gain);
            self.goal_systime.set(eventtime + self.check_gain_time);
        } else if eventtime >= self.goal_systime.get() {
            // Temperature is no longer approaching target
            self.approaching_target.set(false);
            logging::info(&format!("Heater {} no longer approaching target {:.3}", self.heater_name, target));
        } else if self.starting_approach.get() {
            let g = temp + self.heating_gain;
            if g < self.goal_temp.get() {
                self.goal_temp.set(g);
            }
        }
        self.last_target.set(target);
        eventtime + 1.0
    }

    fn heater_fault(&self) -> f64 {
        let msg = format!("Heater {} not heating at expected rate", self.heater_name);
        logging::error(&msg);
        self.printer.invoke_shutdown(&format!("{}{}", msg, HINT_THERMAL), None);
        NEVER
    }
}

impl PrinterObject for HeaterCheck {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        *self.heater.borrow_mut() = None;
        if let Some(t) = self.check_timer.borrow_mut().take() {
            self.printer.get_reactor().forget_timer(&t);
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(HeaterCheck::new(config)?)
}
