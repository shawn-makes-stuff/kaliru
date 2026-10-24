// Support for disabling the printer on an idle timeout
// (port of klippy/extras/idle_timeout.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const DEFAULT_IDLE_GCODE: &str = "
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
M84
";

pub const PIN_MIN_TIME: f64 = 0.100;
pub const READY_TIMEOUT: f64 = 0.500;

const CMD_SET_IDLE_TIMEOUT_HELP: &str = "Set the idle timeout in seconds";

pub struct IdleTimeout {
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    gcode: Rc<GCodeDispatch>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    timeout_timer: RefCell<Option<Timer>>,
    pub(crate) idle_timeout: Cell<f64>,
    idle_gcode: Rc<Template>,
    pub(crate) state: RefCell<&'static str>,
    pub(crate) last_print_start_systime: Cell<f64>,
}

fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

impl IdleTimeout {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // the ready handler is registered first
        let slot: Rc<RefCell<Option<Rc<IdleTimeout>>>> = Rc::new(RefCell::new(None));
        let s = slot.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                let it = s.borrow().clone();
                if let Some(it) = it {
                    it.handle_ready()?;
                }
                Ok(())
            }),
        );
        let idle_timeout = config.getfloat("timeout", 600.0, Lim::new().min(0))?;
        let gcode_macro = load_gcode_macro(config)?;
        let idle_gcode = gcode_macro.load_template(config, "gcode", Some(DEFAULT_IDLE_GCODE))?;
        let it = Rc::new(IdleTimeout {
            printer: printer.clone(),
            reactor,
            gcode: gcode.clone(),
            toolhead: RefCell::new(None),
            timeout_timer: RefCell::new(None),
            idle_timeout: Cell::new(idle_timeout),
            idle_gcode,
            state: RefCell::new("Idle"),
            last_print_start_systime: Cell::new(0.0),
        });
        *slot.borrow_mut() = Some(it.clone());
        let i = it.clone();
        gcode.register_command(
            "SET_IDLE_TIMEOUT",
            Some(Rc::new(move |g| i.cmd_set_idle_timeout(g))),
            false,
            Some(CMD_SET_IDLE_TIMEOUT_HELP),
        )?;
        Ok(it)
    }

    fn handle_ready(self: &Rc<Self>) -> KResult<()> {
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        let w = Rc::downgrade(self);
        let timer = self.reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(it) => it.timeout_handler(eventtime),
                None => NEVER,
            }),
            NEVER,
        );
        *self.timeout_timer.borrow_mut() = Some(timer);
        let i = self.clone();
        self.printer.register_event_handler(
            "toolhead:sync_print_time",
            Rc::new(move |args| {
                let f = |n: usize| args.get(n).and_then(|a| a.downcast_ref::<f64>()).copied().unwrap_or(0.0);
                i.handle_sync_print_time(f(0), f(1), f(2));
                Ok(())
            }),
        );
        Ok(())
    }

    fn toolhead(&self) -> Rc<ToolHead> {
        self.toolhead.borrow().clone().expect("toolhead")
    }

    fn transition_idle_state(&self, eventtime: f64) -> f64 {
        *self.state.borrow_mut() = "Printing";
        let res = self.idle_gcode.render(None).and_then(|script| self.gcode.run_script(&script));
        if res.is_err() {
            logging::exception("idle timeout gcode execution");
            *self.state.borrow_mut() = "Ready";
            return eventtime + 1.0;
        }
        let print_time = match self.toolhead().get_last_move_time() {
            Ok(t) => t,
            Err(e) => {
                self.printer.unhandled_exception(&e);
                return NEVER;
            }
        };
        *self.state.borrow_mut() = "Idle";
        if let Err(e) = self.printer.send_event("idle_timeout:idle", &[&print_time]) {
            self.printer.unhandled_exception(&e);
        }
        NEVER
    }

    fn check_idle_timeout(&self, eventtime: f64) -> f64 {
        // timeout is disabled
        if !(self.idle_timeout.get() > 0.0) {
            return eventtime + 60.0;
        }
        // Make sure toolhead class isn't busy
        let (print_time, est_print_time, lookahead_empty) = self.toolhead().check_busy(eventtime);
        let idle_time = est_print_time - print_time;
        if !lookahead_empty || idle_time < 1.0 {
            // Toolhead is busy
            return eventtime + self.idle_timeout.get();
        }
        if idle_time < self.idle_timeout.get() {
            // Wait for idle timeout
            return eventtime + self.idle_timeout.get() - idle_time;
        }
        if self.gcode.get_mutex().test() {
            // Gcode class busy
            return eventtime + 1.0;
        }
        // Idle timeout has elapsed
        self.transition_idle_state(eventtime)
    }

    fn timeout_handler(&self, eventtime: f64) -> f64 {
        if self.printer.is_shutdown() {
            return NEVER;
        }
        if *self.state.borrow() == "Ready" {
            return self.check_idle_timeout(eventtime);
        }
        // Check if need to transition to "ready" state
        let (print_time, est_print_time, lookahead_empty) = self.toolhead().check_busy(eventtime);
        let buffer_time = py_min(2.0, print_time - est_print_time);
        if !lookahead_empty {
            // Toolhead is busy
            return eventtime + READY_TIMEOUT + py_max(0.0, buffer_time);
        }
        if buffer_time > -READY_TIMEOUT {
            // Wait for ready timeout
            return eventtime + READY_TIMEOUT + buffer_time;
        }
        if self.gcode.get_mutex().test() {
            // Gcode class busy
            return eventtime + READY_TIMEOUT;
        }
        // Transition to "ready" state
        *self.state.borrow_mut() = "Ready";
        let t = est_print_time + PIN_MIN_TIME;
        if let Err(e) = self.printer.send_event("idle_timeout:ready", &[&t]) {
            self.printer.unhandled_exception(&e);
        }
        eventtime + self.idle_timeout.get()
    }

    fn handle_sync_print_time(&self, curtime: f64, print_time: f64, est_print_time: f64) {
        if *self.state.borrow() == "Printing" {
            return;
        }
        // Transition to "printing" state
        *self.state.borrow_mut() = "Printing";
        self.last_print_start_systime.set(curtime);
        let check_time = READY_TIMEOUT + print_time - est_print_time;
        if let Some(t) = self.timeout_timer.borrow().as_ref() {
            self.reactor.update_timer(t, curtime + check_time);
        }
        let t = est_print_time + PIN_MIN_TIME;
        if let Err(e) = self.printer.send_event("idle_timeout:printing", &[&t]) {
            self.printer.unhandled_exception(&e);
        }
    }

    fn cmd_set_idle_timeout(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let timeout = gcmd.get_float("TIMEOUT", Some(self.idle_timeout.get()), Lim::new().above(0.0))?;
        self.idle_timeout.set(timeout);
        gcmd.respond_info(&format!("idle_timeout: Timeout set to {:.2} s", timeout), true);
        if *self.state.borrow() == "Ready" {
            let checktime = self.reactor.monotonic() + timeout;
            if let Some(t) = self.timeout_timer.borrow().as_ref() {
                self.reactor.update_timer(t, checktime);
            }
        }
        Ok(())
    }
}

impl PrinterObject for IdleTimeout {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        let mut printing_time = 0.0;
        if *self.state.borrow() == "Printing" {
            printing_time = eventtime - self.last_print_start_systime.get();
        }
        let mut d = IndexMap::new();
        d.insert("state".to_string(), PyVal::from(*self.state.borrow()));
        d.insert("printing_time".to_string(), PyVal::Float(printing_time));
        d.insert("idle_timeout".to_string(), PyVal::Float(self.idle_timeout.get()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.toolhead.borrow_mut() = None;
        if let Some(t) = self.timeout_timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(IdleTimeout::new(config)?)
}
