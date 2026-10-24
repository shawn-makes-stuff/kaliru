// A simple timer for executing gcode templates
// (port of klippy/extras/delayed_gcode.py)
//
// Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use crate::reactor::{Reactor, Timer, NEVER};
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub struct DelayedGcode {
    me: Weak<DelayedGcode>,
    reactor: Rc<Reactor>,
    gcode: Rc<GCodeDispatch>,
    timer_gcode: Rc<Template>,
    duration: Cell<f64>,
    timer_handler: RefCell<Option<Timer>>,
    inside_timer: Cell<bool>,
    repeat: Cell<bool>,
}

impl DelayedGcode {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        if config.get_name().split_whitespace().count() > 2 {
            return Err(KError::config(format!("Name of section '{}' contains illegal whitespace", config.get_name())));
        }
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().nth(1).unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let gcode_macro = load_gcode_macro(config)?;
        let cmd_desc = config.get("description", "Update the duration of a delayed_gcode")?;
        let timer_gcode = gcode_macro.load_template(config, "gcode", None)?;
        let duration = config.getfloat("initial_duration", 0.0, Lim::new().min(0.0))?;
        let d = Rc::new_cyclic(|me| DelayedGcode {
            me: me.clone(),
            reactor: printer.get_reactor(),
            gcode: gcode.clone(),
            timer_gcode,
            duration: Cell::new(duration),
            timer_handler: RefCell::new(None),
            inside_timer: Cell::new(false),
            repeat: Cell::new(false),
        });
        let w = d.me.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                if let Some(d) = w.upgrade() {
                    d.handle_ready();
                }
                Ok(())
            }),
        );
        let w = d.me.clone();
        gcode.register_mux_command(
            "UPDATE_DELAYED_GCODE",
            "ID",
            Some(&name),
            Rc::new(move |g| match w.upgrade() {
                Some(d) => d.cmd_update_delayed_gcode(g),
                None => Ok(()),
            }),
            Some(&cmd_desc),
        )?;
        Ok(d)
    }

    fn handle_ready(&self) {
        let mut waketime = NEVER;
        if self.duration.get() != 0.0 {
            waketime = self.reactor.monotonic() + self.duration.get();
        }
        let w = self.me.clone();
        *self.timer_handler.borrow_mut() = Some(self.reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(d) => d.gcode_timer_event(eventtime),
                None => NEVER,
            }),
            waketime,
        ));
    }

    fn gcode_timer_event(&self, eventtime: f64) -> f64 {
        self.inside_timer.set(true);
        let r = self.timer_gcode.render(None).and_then(|s| self.gcode.run_script(&s));
        if let Err(e) = r {
            logging::exception_err("Script running error", &e);
        }
        let mut nextwake = NEVER;
        if self.repeat.get() {
            nextwake = eventtime + self.duration.get();
        }
        self.inside_timer.set(false);
        self.repeat.set(false);
        nextwake
    }

    fn cmd_update_delayed_gcode(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.duration.set(gcmd.get_float("DURATION", None, Lim::new().min(0.0))?);
        if self.inside_timer.get() {
            self.repeat.set(self.duration.get() != 0.0);
        } else {
            let mut waketime = NEVER;
            if self.duration.get() != 0.0 {
                waketime = self.reactor.monotonic() + self.duration.get();
            }
            let timer = self.timer_handler.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'waketime'".into()))?;
            self.reactor.update_timer(&timer, waketime);
        }
        Ok(())
    }
}

impl PrinterObject for DelayedGcode {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        if let Some(t) = self.timer_handler.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(DelayedGcode::new(config)?)
}
