// PWM and digital output pin handling (port of klippy/extras/output_pin.py)
//
// Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::display_templates::{literal_eval, lookup_display_templates, DisplayTemplate};
use crate::extras::gcode_macro::{create_template_context, kerror_exc, load_gcode_macro};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::filters::bind;
use crate::jinja::value::{CallCtx, Exc, Key, Object, VResult, Value};
use crate::logging;
use crate::mcu::{DigitalOutPin, Mcu, PwmPin};
use crate::pins::{setup_digital_out_pin, setup_pwm_pin};
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::py_max;
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// ----------------------------------------------------------------------
// G-Code request queuing helper
// ----------------------------------------------------------------------

pub const PIN_MIN_TIME: f64 = 0.100;

/// the "discard" / "delay" actions a request callback may return
#[derive(Clone, Copy, PartialEq)]
pub enum Action {
    Discard,
    Delay,
}

/// callback(print_time, value) -> None or (action, min_wait)
pub type RequestCallback = Rc<dyn Fn(f64, f64) -> KResult<Option<(Action, f64)>>>;

// Helper code to queue g-code requests
pub struct GCodeRequestQueue {
    printer: Rc<Printer>,
    mcu: Rc<Mcu>,
    callback: RequestCallback,
    rqueue: RefCell<Vec<(f64, f64)>>,
    next_min_flush_time: Cell<f64>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    motion_queuing: Rc<PrinterMotionQueuing>,
}

impl GCodeRequestQueue {
    pub fn new(config: &ConfigWrapper, mcu: &Rc<Mcu>, callback: RequestCallback) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let motion_queuing = printer
            .load_object(config, "motion_queuing", true)?
            .and_then(downcast_rc::<PrinterMotionQueuing>)
            .ok_or_else(|| KError::Other("motion_queuing has an unexpected type".into()))?;
        let q = Rc::new(GCodeRequestQueue {
            printer: printer.clone(),
            mcu: mcu.clone(),
            callback,
            rqueue: RefCell::new(vec![]),
            next_min_flush_time: Cell::new(0.0),
            toolhead: RefCell::new(None),
            motion_queuing: motion_queuing.clone(),
        });
        let w = Rc::downgrade(&q);
        motion_queuing.register_flush_callback(
            Rc::new(move |must_flush_time, max_step_gen_time| match w.upgrade() {
                Some(q) => q.flush_notification(must_flush_time, max_step_gen_time),
                None => Ok(()),
            }),
            false,
        );
        let w = Rc::downgrade(&q);
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                if let Some(q) = w.upgrade() {
                    *q.toolhead.borrow_mut() = Some(q.printer.lookup::<ToolHead>("toolhead")?);
                }
                Ok(())
            }),
        );
        Ok(q)
    }

    fn flush_notification(&self, must_flush_time: f64, _max_step_gen_time: f64) -> KResult<()> {
        let _min_sched_time = self.mcu.min_schedule_time();
        loop {
            let (next_time, pos, req_val) = {
                let rqueue = self.rqueue.borrow();
                if rqueue.is_empty() {
                    return Ok(());
                }
                let next_time = py_max(rqueue[0].0, self.next_min_flush_time.get());
                if next_time > must_flush_time {
                    return Ok(());
                }
                // Skip requests that have been overridden with a following request
                let mut pos = 0;
                while pos + 1 < rqueue.len() && rqueue[pos + 1].0 <= next_time {
                    pos += 1;
                }
                (next_time, pos, rqueue[pos].1)
            };
            // Invoke callback for the request
            let mut min_wait = 0.0;
            let mut del = pos + 1;
            if let Some((action, mw)) = (self.callback)(next_time, req_val)? {
                // Handle special cases
                min_wait = mw;
                if action == Action::Discard {
                    self.rqueue.borrow_mut().drain(..pos + 1);
                    continue;
                }
                if action == Action::Delay {
                    del = pos;
                }
            }
            self.rqueue.borrow_mut().drain(..del);
            self.next_min_flush_time.set(next_time + py_max(min_wait, PIN_MIN_TIME));
            // Ensure following queue items are flushed
            self.motion_queuing.note_mcu_movequeue_activity(self.next_min_flush_time.get(), false);
        }
    }

    fn queue_request(&self, print_time: f64, value: f64) {
        self.rqueue.borrow_mut().push((print_time, value));
        self.motion_queuing.note_mcu_movequeue_activity(print_time, false);
    }

    pub fn queue_gcode_request(self: &Rc<Self>, value: f64) -> KResult<()> {
        let toolhead = self
            .toolhead
            .borrow()
            .clone()
            .ok_or_else(|| KError::Other("'NoneType' object has no attribute 'register_lookahead_callback'".into()))?;
        let q = self.clone();
        toolhead.register_lookahead_callback(Rc::new(move |pt| {
            q.queue_request(pt, value);
            Ok(())
        }))
    }

    pub fn send_async_request(&self, value: f64, print_time: Option<f64>) -> KResult<()> {
        let print_time = match print_time {
            Some(pt) => pt,
            None => {
                let systime = self.printer.get_reactor().monotonic();
                self.mcu.estimated_print_time(systime + PIN_MIN_TIME)
            }
        };
        loop {
            let next_time = py_max(print_time, self.next_min_flush_time.get());
            // Invoke callback for the request
            let (mut action, mut min_wait) = (None, 0.0);
            if let Some((a, mw)) = (self.callback)(next_time, value)? {
                // Handle special cases
                action = Some(a);
                min_wait = mw;
                if a == Action::Discard {
                    break;
                }
            }
            self.next_min_flush_time.set(next_time + py_max(min_wait, PIN_MIN_TIME));
            if action != Some(Action::Delay) {
                break;
            }
        }
        Ok(())
    }
}

// ----------------------------------------------------------------------
// Template evaluation helper
// ----------------------------------------------------------------------

// Time between each template update
pub const RENDER_TIME: f64 = 0.500;

/// callback(text) of an active template
pub type TemplateCallback = Rc<dyn Fn(&str) -> KResult<()>>;
/// the optional flush callback of set_template
pub type TemplateFlush = Rc<dyn Fn() -> KResult<()>>;

/// the template context's render(name, **kwargs)
struct RenderFn {
    templates: IndexMap<String, Rc<DisplayTemplate>>,
    context: Rc<RefCell<IndexMap<String, Value>>>,
}

#[derive(Clone)]
struct RenderFnRef(Rc<RenderFn>);

impl Object for RenderFnRef {
    fn type_name(&self) -> String {
        "function".into()
    }
    fn repr(&self) -> String {
        "<function PrinterTemplateEvaluator._render.<locals>.render>".into()
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        Some((|| {
            let (a, _, kw) = bind("render", &[("name", None)], args, kwargs, false, true)?;
            let name = a[0].to_str();
            let t = self.0.templates.get(&name).cloned().ok_or_else(|| Exc::new("KeyError", &a[0].repr()))?;
            let context = self.0.context.borrow().clone();
            t.render(&context, &kw).map(Value::from).map_err(kerror_exc)
        })())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// an active template: (uid, template, lparams, flush_callback)
struct Active {
    callback: TemplateCallback,
    uid: Option<String>,
    template: Rc<DisplayTemplate>,
    lparams: IndexMap<String, Value>,
    flush_callback: Option<TemplateFlush>,
}

fn same_cb<T: ?Sized>(a: &Rc<T>, b: &Rc<T>) -> bool {
    std::ptr::eq(Rc::as_ptr(a) as *const (), Rc::as_ptr(b) as *const ())
}

// Main template evaluation code
pub struct PrinterTemplateEvaluator {
    me: Weak<PrinterTemplateEvaluator>,
    printer: Rc<Printer>,
    active_templates: RefCell<Vec<Active>>,
    render_timer: RefCell<Option<Timer>>,
    templates: IndexMap<String, Rc<DisplayTemplate>>,
}

impl PrinterTemplateEvaluator {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Load templates
        let dtemplates = lookup_display_templates(config)?;
        let templates = dtemplates.display_templates.clone();
        load_gcode_macro(config)?;
        Ok(Rc::new_cyclic(|me| PrinterTemplateEvaluator {
            me: me.clone(),
            printer,
            active_templates: RefCell::new(vec![]),
            render_timer: RefCell::new(None),
            templates,
        }))
    }

    fn activate_timer(&self) {
        if self.render_timer.borrow().is_some() || self.active_templates.borrow().is_empty() {
            return;
        }
        let reactor = self.printer.get_reactor();
        let w = self.me.clone();
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(te) => te.render(eventtime),
                None => NEVER,
            }),
            NOW,
        );
        *self.render_timer.borrow_mut() = Some(timer);
    }

    fn activate_template(
        &self,
        callback: TemplateCallback,
        template: Option<Rc<DisplayTemplate>>,
        lparams: IndexMap<String, Value>,
        flush_callback: Option<TemplateFlush>,
    ) {
        let mut active = self.active_templates.borrow_mut();
        let pos = active.iter().position(|a| same_cb(&a.callback, &callback));
        let Some(template) = template else {
            if let Some(pos) = pos {
                active.remove(pos);
            }
            return;
        };
        // Build a unique id to make it possible to cache duplicate rendering
        // (None when lparams is not static: unhashable values)
        let hashable = lparams.values().all(|v| Key::hashable(v).is_ok());
        let uid = hashable.then(|| {
            let mut items: Vec<(&String, &Value)> = lparams.iter().collect();
            items.sort_by(|a, b| a.0.cmp(b.0));
            let items: Vec<String> = items.iter().map(|(k, v)| format!("{}={}", k, v.repr())).collect();
            format!("{:p}|{}", Rc::as_ptr(&template), items.join(","))
        });
        let entry = Active { callback, uid, template, lparams, flush_callback };
        match pos {
            Some(pos) => active[pos] = entry,
            None => active.push(entry),
        }
    }

    fn render(&self, eventtime: f64) -> f64 {
        if self.active_templates.borrow().is_empty() {
            // Nothing to do - unregister timer
            if let Some(t) = self.render_timer.borrow_mut().take() {
                self.printer.get_reactor().unregister_timer(&t);
            }
            return NEVER;
        }
        // Setup gcode_macro template context
        let context = Rc::new(RefCell::new(create_template_context(&self.printer, Some(eventtime))));
        let render_fn = RenderFnRef(Rc::new(RenderFn { templates: self.templates.clone(), context: context.clone() }));
        context.borrow_mut().insert("render".into(), Value::object(render_fn));
        // Render all templates
        let mut flush_callbacks: Vec<TemplateFlush> = vec![];
        let mut render_cache: IndexMap<String, String> = IndexMap::new();
        type Entry = (TemplateCallback, Option<String>, Rc<DisplayTemplate>, IndexMap<String, Value>, Option<TemplateFlush>);
        let active: Vec<Entry> = self
            .active_templates
            .borrow()
            .iter()
            .map(|a| (a.callback.clone(), a.uid.clone(), a.template.clone(), a.lparams.clone(), a.flush_callback.clone()))
            .collect();
        for (callback, uid, template, lparams, flush_callback) in active {
            let cached = uid.as_ref().and_then(|u| render_cache.get(u).cloned());
            let text = match cached {
                Some(t) => t,
                None => {
                    let ctx = context.borrow().clone();
                    let text = match template.render(&ctx, &lparams) {
                        Ok(t) => t,
                        Err(e) => {
                            logging::exception_err("display template render error", &e);
                            String::new()
                        }
                    };
                    if let Some(u) = &uid {
                        render_cache.insert(u.clone(), text.clone());
                    }
                    text
                }
            };
            if let Some(f) = flush_callback {
                if !flush_callbacks.iter().any(|g| same_cb(g, &f)) {
                    flush_callbacks.push(f);
                }
            }
            if let Err(e) = callback(&text) {
                logging::exception_err("template callback error", &e);
            }
        }
        // Remove circular references for better gc
        context.borrow_mut().clear();
        // Invoke optional flush callbacks
        for flush_callback in flush_callbacks {
            if let Err(e) = flush_callback() {
                logging::exception_err("template flush callback error", &e);
            }
        }
        eventtime + RENDER_TIME
    }

    pub fn set_template(&self, gcmd: &GCodeCommand, callback: TemplateCallback, flush_callback: Option<TemplateFlush>) -> KResult<()> {
        let mut template = None;
        let mut lparams = IndexMap::new();
        let tpl_name = gcmd.get("TEMPLATE")?;
        if !tpl_name.is_empty() {
            let Some(t) = self.templates.get(&tpl_name).cloned() else {
                return Err(gcmd.error(format!("Unknown display_template '{}'", tpl_name)));
            };
            let tparams = t.get_params();
            for (p, v) in gcmd.get_command_parameters() {
                if !p.starts_with("PARAM_") {
                    continue;
                }
                let p = p.to_lowercase();
                if !tparams.contains_key(&p) {
                    return Err(gcmd.error(format!("Invalid display_template parameter: {}", p)));
                }
                match literal_eval(&v) {
                    Ok(val) => {
                        lparams.insert(p, val);
                    }
                    Err((cls, _)) if cls == "ValueError" => {
                        return Err(gcmd.error(format!("Unable to parse '{}' as a literal", v)));
                    }
                    Err((cls, msg)) => return Err(KError::Py(cls.to_string(), msg.to_string())),
                }
            }
            template = Some(t);
        }
        self.activate_template(callback, template, lparams, flush_callback);
        self.activate_timer();
        Ok(())
    }
}

impl PrinterObject for PrinterTemplateEvaluator {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        self.active_templates.borrow_mut().clear();
        if let Some(t) = self.render_timer.borrow_mut().take() {
            self.printer.get_reactor().forget_timer(&t);
        }
    }
}

pub fn lookup_template_eval(config: &ConfigWrapper) -> KResult<Rc<PrinterTemplateEvaluator>> {
    let printer = config.get_printer();
    if let Some(o) = printer.lookup_object_opt("template_evaluator") {
        return downcast_rc::<PrinterTemplateEvaluator>(o).ok_or_else(|| KError::Other("template_evaluator has an unexpected type".into()));
    }
    let te = PrinterTemplateEvaluator::new(config)?;
    printer.add_object("template_evaluator", te.clone())?;
    Ok(te)
}

// ----------------------------------------------------------------------
// Main output pin handling
// ----------------------------------------------------------------------

pub const MAX_SCHEDULE_TIME: f64 = 5.0;

const CMD_SET_PIN_HELP: &str = "Set the value of an output pin";

enum OutPin {
    Pwm(Rc<dyn PwmPin>),
    Digital(Rc<dyn DigitalOutPin>),
}

pub struct PrinterOutputPin {
    is_pwm: bool,
    mcu_pin: OutPin,
    scale: f64,
    last_value: Cell<f64>,
    gcrq: RefCell<Option<Rc<GCodeRequestQueue>>>,
    template_eval: RefCell<Option<Rc<PrinterTemplateEvaluator>>>,
    template_cb: RefCell<Option<TemplateCallback>>,
}

impl PrinterOutputPin {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Determine pin type
        let is_pwm = config.getboolean("pwm", false)?;
        let (mcu_pin, scale, mcu) = if is_pwm {
            let p = setup_pwm_pin(&printer, &config.get("pin", Dflt::Required)?)?;
            let cycle_time = config.getfloat("cycle_time", 0.100, Lim::new().above(0.0).max(MAX_SCHEDULE_TIME))?;
            let hardware_pwm = config.getboolean("hardware_pwm", false)?;
            p.setup_cycle_time(cycle_time, hardware_pwm)?;
            let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
            p.setup_max_duration(0.0);
            let mcu = p.get_mcu();
            (OutPin::Pwm(p), scale, mcu)
        } else {
            let p = setup_digital_out_pin(&printer, &config.get("pin", Dflt::Required)?)?;
            p.setup_max_duration(0.0);
            let mcu = p.get_mcu();
            (OutPin::Digital(p), 1.0, mcu)
        };
        // Determine start and shutdown values
        let last_value = config.getfloat("value", 0.0, Lim::new().min(0.0).max(scale))? / scale;
        let shutdown_value = config.getfloat("shutdown_value", 0.0, Lim::new().min(0.0).max(scale))? / scale;
        match &mcu_pin {
            OutPin::Pwm(p) => p.setup_start_value(last_value, shutdown_value),
            OutPin::Digital(p) => p.setup_start_value(last_value != 0.0, shutdown_value != 0.0),
        }
        let op = Rc::new(PrinterOutputPin {
            is_pwm,
            mcu_pin,
            scale,
            last_value: Cell::new(last_value),
            gcrq: RefCell::new(None),
            template_eval: RefCell::new(None),
            template_cb: RefCell::new(None),
        });
        // Create gcode request queue
        let w = Rc::downgrade(&op);
        let gcrq = GCodeRequestQueue::new(
            config,
            &mcu,
            Rc::new(move |print_time, value| match w.upgrade() {
                Some(o) => o.set_pin(print_time, value),
                None => Ok(None),
            }),
        )?;
        *op.gcrq.borrow_mut() = Some(gcrq);
        // Template handling
        *op.template_eval.borrow_mut() = Some(lookup_template_eval(config)?);
        let w = Rc::downgrade(&op);
        *op.template_cb.borrow_mut() = Some(Rc::new(move |text: &str| match w.upgrade() {
            Some(o) => o.template_update(text),
            None => Ok(()),
        }));
        // Register commands
        let pin_name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let o = op.clone();
        gcode.register_mux_command("SET_PIN", "PIN", Some(&pin_name), Rc::new(move |g| o.cmd_set_pin(g)), Some(CMD_SET_PIN_HELP))?;
        Ok(op)
    }

    fn set_pin(&self, print_time: f64, value: f64) -> KResult<Option<(Action, f64)>> {
        if value == self.last_value.get() {
            return Ok(Some((Action::Discard, 0.0)));
        }
        self.last_value.set(value);
        match &self.mcu_pin {
            OutPin::Pwm(p) => p.set_pwm(print_time, value)?,
            OutPin::Digital(p) => p.set_digital(print_time, value != 0.0)?,
        }
        Ok(None)
    }

    fn gcrq(&self) -> Rc<GCodeRequestQueue> {
        self.gcrq.borrow().clone().expect("gcrq")
    }

    fn template_update(&self, text: &str) -> KResult<()> {
        let value = match crate::pyfmt::py_float(text) {
            Some(v) => v,
            None => {
                logging::exception_tb(
                    "output_pin template render error",
                    "ValueError",
                    &format!("could not convert string to float: {}", crate::pyfmt::py_str_repr(text)),
                );
                0.0
            }
        };
        self.gcrq().send_async_request(value, None)
    }

    fn cmd_set_pin(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let value = gcmd.get_float_opt("VALUE", Lim::new().min(0.0).max(self.scale))?;
        let template = gcmd.get_opt("TEMPLATE");
        if value.is_none() == template.is_none() {
            return Err(gcmd.error("SET_PIN command must specify VALUE or TEMPLATE"));
        }
        // Check for template setting
        if template.is_some() {
            let te = self.template_eval.borrow().clone().expect("template eval");
            let cb = self.template_cb.borrow().clone().expect("template callback");
            return te.set_template(gcmd, cb, None);
        }
        // Read requested value
        let value = value.unwrap() / self.scale;
        if !self.is_pwm && value != 0.0 && value != 1.0 {
            return Err(gcmd.error("Invalid pin value"));
        }
        // Queue requested value
        self.gcrq().queue_gcode_request(value)
    }
}

impl PrinterObject for PrinterOutputPin {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("value".to_string(), PyVal::Float(self.last_value.get()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.gcrq.borrow_mut() = None;
        *self.template_cb.borrow_mut() = None;
        *self.template_eval.borrow_mut() = None;
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterOutputPin::new(config)?)
}
