// INDX heatsink fan (port of klippy/extras/indx/heatsink_fan.py)
use super::{compat_config, CVal};
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::fan::Fan;
use crate::extras::heaters::Heater;
use crate::extras::stepper_enable::{EnableTracking, PrinterStepperEnable};
use crate::mcu::{Mcu, MIN_SCHEDULE_TIME_INIT};
use crate::printer::{Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER};
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const BLOCKED_CHECK_SECONDS: i64 = 5;

pub struct IndxHeatsinkFan {
    printer: Rc<Printer>,
    mcu: Rc<Mcu>,
    pub fan: Rc<Fan>,
    check_timer: RefCell<Option<Timer>>,
    active_stopped_count: Cell<i64>,
    is_active: Cell<bool>,
    pub steppers: RefCell<Vec<Rc<EnableTracking>>>,
    heaters: RefCell<Vec<Rc<Heater>>>,
}

impl IndxHeatsinkFan {
    pub fn new(config: &ConfigWrapper, printer: &Rc<Printer>, mcu: &Rc<Mcu>, toolboard_name: &str) -> KResult<Rc<Self>> {
        let mcu_name = mcu.get_name();
        let fan_config = compat_config(
            config,
            &format!("fan {}_heatsink", toolboard_name),
            &[
                ("shutdown_speed", CVal::F(1.0)),
                ("pin", CVal::S(format!("{}:fan0_out", mcu_name))),
                ("tachometer_pin", CVal::S(format!("{}:fan0_tacho", mcu_name))),
                ("tachometer_poll_interval", CVal::F(30.0 / (2.0 * 25000.0))),
            ],
        );
        let fan = Fan::new(&fan_config, 0.0)?;
        let f = Rc::new(IndxHeatsinkFan {
            printer: printer.clone(),
            mcu: mcu.clone(),
            fan,
            check_timer: RefCell::new(None),
            active_stopped_count: Cell::new(0),
            is_active: Cell::new(false),
            steppers: RefCell::new(vec![]),
            heaters: RefCell::new(vec![]),
        });
        // Expose speed/rpm to frontends, which list "heater_fan" objects
        printer.add_object(&format!("heater_fan {}_heatsink", toolboard_name), f.clone())?;

        let w = Rc::downgrade(&f);
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| match w.upgrade() {
                Some(f) => f.handle_ready(),
                None => Ok(()),
            }),
        );
        let w = Rc::downgrade(&f);
        printer.register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                if let Some(f) = w.upgrade() {
                    f.handle_shutdown();
                }
                Ok(())
            }),
        );
        Ok(f)
    }

    fn handle_ready(self: &Rc<Self>) -> KResult<()> {
        let reactor = self.printer.get_reactor();
        let w = Rc::downgrade(self);
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(f) => f.check_event(eventtime),
                None => NEVER,
            }),
            reactor.monotonic() + MIN_SCHEDULE_TIME_INIT,
        );
        *self.check_timer.borrow_mut() = Some(timer);

        let stepper_enable = self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
        let mut steppers = vec![];
        for s in stepper_enable.get_steppers() {
            steppers.push(stepper_enable.lookup_enable(&s)?);
        }
        let mcu_name = self.mcu.get_name();
        *self.steppers.borrow_mut() = steppers.into_iter().filter(|s| s.stepper().get_mcu().get_name() == mcu_name).collect();
        Ok(())
    }

    fn handle_shutdown(&self) {
        if let Some(t) = self.check_timer.borrow().as_ref() {
            self.printer.get_reactor().update_timer(t, NEVER);
        }
    }

    pub fn add_heater(&self, heater: Rc<Heater>) {
        self.heaters.borrow_mut().push(heater);
    }

    fn check_event(&self, eventtime: f64) -> f64 {
        let mut activate = false;

        for heater in self.heaters.borrow().iter() {
            let (current_temp, target_temp) = heater.get_temp(eventtime);
            activate |= target_temp != 0.0;
            activate |= current_temp >= 40.0;
        }
        for s in self.steppers.borrow().iter() {
            activate |= s.is_motor_enabled();
        }

        let rpm = match self.fan.get_status(eventtime) {
            PyVal::Dict(d) => d.get("rpm").and_then(|v| v.as_f64()).unwrap_or(0.0),
            _ => 0.0,
        };

        if self.is_active.get() && activate {
            if rpm < 1000.0 {
                self.active_stopped_count.set(self.active_stopped_count.get() + 1);
                if self.active_stopped_count.get() >= BLOCKED_CHECK_SECONDS {
                    self.printer.invoke_shutdown("INDX Heatsink fan block detected", None);
                    return NEVER;
                }
            }
        } else {
            self.active_stopped_count.set(0);
        }

        let r = if !self.is_active.get() && activate {
            self.is_active.set(true);
            self.fan.set_speed(1.0, None)
        } else if self.is_active.get() && !activate {
            self.is_active.set(false);
            self.fan.set_speed(0.0, None)
        } else {
            Ok(())
        };
        if let Err(e) = r {
            crate::logging::exception_err("INDX heatsink fan", &e);
        }

        eventtime + 1.0
    }
}

impl PrinterObject for IndxHeatsinkFan {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.fan.get_status(eventtime))
    }
    fn teardown(&self) {
        self.heaters.borrow_mut().clear();
        self.steppers.borrow_mut().clear();
        if let Some(t) = self.check_timer.borrow_mut().take() {
            self.printer.get_reactor().forget_timer(&t);
        }
    }
}
