// Parse gcode commands (port of klippy/gcode.py)
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::Lim;
use crate::error::{KError, KResult};
use crate::logging;
use crate::printer::{Printer, PrinterObject};
use crate::pyfmt::{py_float, py_int, py_isspace, py_shlex_split, py_str_repr, py_strip};
use crate::pyval::PyVal;
use crate::reactor::{FileHandler, ReactorMutex};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::os::fd::RawFd;
use std::rc::Rc;

/// collections.namedtuple("Coord", ("x", "y", "z", "e"))
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coord {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub e: f64,
}

impl Coord {
    pub fn to_pyval(&self) -> PyVal {
        coord_pyval(&[self.x, self.y, self.z, self.e])
    }
}

/// Coord(*values) as a python value
pub fn coord_pyval(values: &[f64]) -> PyVal {
    PyVal::NamedTuple("Coord", &["x", "y", "z", "e"], values.iter().map(|&v| PyVal::Float(v)).collect())
}

pub type Handler = Rc<dyn Fn(&GCodeCommand) -> KResult<()>>;

pub type OutputCallback = Rc<dyn Fn(&str)>;

/// What an output callback is a bound method of (python plugins look at
/// `cb.__self__`): the object and its Rust type's path, or the python
/// callable itself for one a plugin registered
pub type CallbackOrigin = (Rc<dyn Any>, &'static str);

pub struct GCodeCommand {
    command: RefCell<String>,
    commandline: String,
    params: RefCell<IndexMap<String, String>>,
    need_ack: Cell<bool>,
    gcode: Rc<GCodeDispatch>,
}

/// a copy of the command (python keeps references to the one object; the
/// holders here only read its params and report through it)
impl Clone for GCodeCommand {
    fn clone(&self) -> Self {
        GCodeCommand {
            command: RefCell::new(self.command.borrow().clone()),
            commandline: self.commandline.clone(),
            params: RefCell::new(self.params.borrow().clone()),
            need_ack: Cell::new(self.need_ack.get()),
            gcode: self.gcode.clone(),
        }
    }
}

fn num_str(v: &PyVal) -> String {
    v.str()
}

impl GCodeCommand {
    pub fn get_command(&self) -> String {
        self.command.borrow().clone()
    }

    pub fn get_commandline(&self) -> &str {
        &self.commandline
    }

    /// self._params, self._command, self._need_ack (python plugins read and
    /// write them: their GCodeCommand methods are Kalico's own code)
    pub fn params_cell(&self) -> &RefCell<IndexMap<String, String>> {
        &self.params
    }

    pub fn set_command(&self, command: &str) {
        *self.command.borrow_mut() = command.to_string();
    }

    pub fn need_ack(&self) -> bool {
        self.need_ack.get()
    }

    pub fn set_need_ack(&self, v: bool) {
        self.need_ack.set(v);
    }

    pub fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.borrow().clone()
    }

    pub fn get_raw_command_parameters(&self) -> String {
        let command = self.get_command();
        let origline: Vec<char> = self.commandline.chars().collect();
        let mut param_start = command.chars().count();
        let mut param_end = origline.len();
        let head: String = origline[..param_start.min(origline.len())].iter().collect();
        if head.to_uppercase() != command {
            // Skip any gcode line-number and ignore any trailing checksum
            let upper: Vec<char> = self.commandline.to_uppercase().chars().collect();
            let cmd_chars: Vec<char> = command.chars().collect();
            // str.find(): -1 when absent
            let found = upper.windows(cmd_chars.len().max(1)).position(|w| w == &cmd_chars[..]).map(|p| p as i64);
            let found = if cmd_chars.is_empty() { Some(0) } else { found };
            param_start = (param_start as i64 + found.unwrap_or(-1)) as usize;
            if let Some(end) = origline.iter().rposition(|c| *c == '*') {
                let rest: String = origline[end + 1..].iter().collect();
                if !rest.is_empty() && rest.chars().all(|c| c.is_numeric()) {
                    param_end = end;
                }
            }
        }
        if origline.get(param_start).map(|c| py_isspace(*c)).unwrap_or(false) {
            param_start += 1;
        }
        let end = param_end.min(origline.len());
        origline[param_start.min(end)..end].iter().collect()
    }

    pub fn ack(&self, msg: Option<&str>) -> bool {
        if !self.need_ack.get() {
            return false;
        }
        let ok_msg = match msg {
            Some(m) if !m.is_empty() => format!("ok {}", m),
            _ => "ok".to_string(),
        };
        self.respond_raw(&ok_msg);
        self.need_ack.set(false);
        true
    }

    pub fn respond_info(&self, msg: &str, log: bool) {
        self.gcode.respond_info(msg, log);
    }

    pub fn respond_raw(&self, msg: &str) {
        self.gcode.respond_raw(msg);
    }

    pub fn error(&self, msg: impl Into<String>) -> KError {
        KError::Command(msg.into())
    }

    /// get(name, default=sentinel): Err when missing
    pub fn get(&self, name: &str) -> KResult<String> {
        self.params
            .borrow()
            .get(name)
            .cloned()
            .ok_or_else(|| self.error(format!("Error on '{}': missing {}", self.commandline, name)))
    }

    /// get(name, None)
    pub fn get_opt(&self, name: &str) -> Option<String> {
        self.params.borrow().get(name).cloned()
    }

    /// get(name, default)
    pub fn get_or(&self, name: &str, default: &str) -> String {
        self.get_opt(name).unwrap_or_else(|| default.to_string())
    }

    fn check(&self, name: &str, v: f64, lim: &Lim) -> KResult<()> {
        let bound = |b: &Option<PyVal>| b.as_ref().map(|b| (b.as_f64().unwrap_or(f64::NAN), num_str(b)));
        if let Some((b, s)) = bound(&lim.minval) {
            if v < b {
                return Err(self.error(format!("Error on '{}': {} must have minimum of {}", self.commandline, name, s)));
            }
        }
        if let Some((b, s)) = bound(&lim.maxval) {
            if v > b {
                return Err(self.error(format!("Error on '{}': {} must have maximum of {}", self.commandline, name, s)));
            }
        }
        if let Some((b, s)) = bound(&lim.above) {
            if v <= b {
                return Err(self.error(format!("Error on '{}': {} must be above {}", self.commandline, name, s)));
            }
        }
        if let Some((b, s)) = bound(&lim.below) {
            if v >= b {
                return Err(self.error(format!("Error on '{}': {} must be below {}", self.commandline, name, s)));
            }
        }
        Ok(())
    }

    /// get(name, default, parser=parse, ...); default None = required
    pub fn get_parsed<T>(&self, name: &str, default: Option<Option<T>>, parse: impl Fn(&str) -> Option<T>, as_f64: impl Fn(&T) -> f64, lim: &Lim) -> KResult<Option<T>> {
        let Some(value) = self.get_opt(name) else {
            return match default {
                None => Err(self.error(format!("Error on '{}': missing {}", self.commandline, name))),
                Some(d) => Ok(d),
            };
        };
        let v = parse(&value).ok_or_else(|| self.error(format!("Error on '{}': unable to parse {}", self.commandline, value)))?;
        self.check(name, as_f64(&v), lim)?;
        Ok(Some(v))
    }

    /// get_int(name, default, minval, maxval); default None = required
    pub fn get_int(&self, name: &str, default: Option<i64>, lim: Lim) -> KResult<i64> {
        Ok(self.get_parsed(name, default.map(Some), py_int, |v| *v as f64, &lim)?.unwrap_or(0))
    }

    /// get_int(name, None, ...)
    pub fn get_int_opt(&self, name: &str, lim: Lim) -> KResult<Option<i64>> {
        self.get_parsed(name, Some(None), py_int, |v| *v as f64, &lim)
    }

    /// get_float(name, default, ...); default None = required. A default is
    /// returned as given (python may hand back an int there)
    pub fn get_float(&self, name: &str, default: Option<f64>, lim: Lim) -> KResult<f64> {
        Ok(self.get_parsed(name, default.map(Some), |s| crate::mathutil::safe_float(s).ok(), |v| *v, &lim)?.unwrap_or(0.))
    }

    pub fn get_float_opt(&self, name: &str, lim: Lim) -> KResult<Option<f64>> {
        self.get_parsed(name, Some(None), |s| crate::mathutil::safe_float(s).ok(), |v| *v, &lim)
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Active {
    Base,
    Ready,
}

// Parse and dispatch G-Code commands
pub struct GCodeDispatch {
    printer: Rc<Printer>,
    is_fileinput: bool,
    is_printer_ready: Cell<bool>,
    mutex: Rc<ReactorMutex>,
    output_callbacks: RefCell<Vec<(OutputCallback, Option<CallbackOrigin>)>>,
    // python: self.gcode_handlers is the base or the ready dict itself
    active: Cell<Active>,
    base_gcode_handlers: RefCell<IndexMap<String, Handler>>,
    ready_gcode_handlers: RefCell<IndexMap<String, Handler>>,
    mux_commands: RefCell<IndexMap<String, (String, IndexMap<Option<String>, (Handler, Option<String>)>)>>,
    gcode_help: RefCell<IndexMap<String, String>>,
    status_commands: RefCell<PyVal>,
    interrupt_counter: Cell<i64>,
    script_context: Cell<i64>,
}

impl PrinterObject for GCodeDispatch {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn teardown(&self) {
        self.base_gcode_handlers.borrow_mut().clear();
        self.ready_gcode_handlers.borrow_mut().clear();
        self.mux_commands.borrow_mut().clear();
        self.output_callbacks.borrow_mut().clear();
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("commands".to_string(), self.status_commands.borrow().clone());
        Some(PyVal::Dict(d))
    }
}

impl GCodeDispatch {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let gcode = Rc::new(GCodeDispatch {
            printer: printer.clone(),
            is_fileinput: printer.start_arg("debuginput").map(|v| v.truthy()).unwrap_or(false),
            is_printer_ready: Cell::new(false),
            mutex: printer.get_reactor().mutex(false),
            output_callbacks: RefCell::new(vec![]),
            active: Cell::new(Active::Base),
            base_gcode_handlers: RefCell::new(IndexMap::new()),
            ready_gcode_handlers: RefCell::new(IndexMap::new()),
            mux_commands: RefCell::new(IndexMap::new()),
            gcode_help: RefCell::new(IndexMap::new()),
            status_commands: RefCell::new(PyVal::dict()),
            interrupt_counter: Cell::new(0),
            script_context: Cell::new(0),
        });
        let g = gcode.clone();
        printer.register_event_handler("klippy:ready", Rc::new(move |_| g.handle_ready()));
        let g = gcode.clone();
        printer.register_event_handler("klippy:shutdown", Rc::new(move |_| g.handle_shutdown()));
        let g = gcode.clone();
        printer.register_event_handler("klippy:disconnect", Rc::new(move |_| g.handle_disconnect()));
        // Register commands needed before config file is loaded
        type Cmd = fn(&Rc<GCodeDispatch>, &GCodeCommand) -> KResult<()>;
        let handlers: [(&str, Cmd, Option<&str>); 10] = [
            ("M110", |_, _| Ok(()), None),
            ("M112", |g, _| g.cmd_m112(), None),
            ("M115", |g, c| g.cmd_m115(c), None),
            ("RESTART", |g, _| g.request_restart("restart"), Some("Reload config file and restart host software")),
            ("FIRMWARE_RESTART", |g, _| g.request_restart("firmware_restart"), Some("Restart firmware, host, and reload config")),
            ("ECHO", |_, c| {
                c.respond_info(c.get_commandline(), false);
                Ok(())
            }, None),
            ("STATUS", |g, c| g.cmd_status(c), Some("Report the printer status")),
            ("HELP", |g, c| g.cmd_help(c), Some("Report the list of available extended G-Code commands")),
            ("HEATER_INTERRUPT", |g, _| {
                g.increment_interrupt_counter();
                Ok(())
            }, None),
            ("LOG_ROLLOVER", |g, _| {
                // python: self.printer.bglogger.doRollover() (AttributeError without -l)
                match &g.printer.bglogger {
                    Some(bg) => {
                        bg.do_rollover();
                        Ok(())
                    }
                    None => Err(KError::Other("'NoneType' object has no attribute 'doRollover'".into())),
                }
            }, None),
        ];
        for (cmd, func, desc) in handlers {
            let g = gcode.clone();
            gcode.register_command(cmd, Some(Rc::new(move |c: &GCodeCommand| func(&g, c))), true, desc)?;
        }
        Ok(gcode)
    }

    pub fn get_interrupt_counter(&self) -> i64 {
        self.interrupt_counter.get()
    }

    pub fn increment_interrupt_counter(&self) {
        self.interrupt_counter.set(self.interrupt_counter.get() + 1);
    }

    pub fn is_traditional_gcode(&self, cmd: &str) -> bool {
        // A "traditional" g-code command is a letter and followed by a number
        let upper = cmd.to_uppercase();
        let Some(first) = upper.split_whitespace().next() else { return false };
        let chars: Vec<char> = first.chars().collect();
        if chars.is_empty() {
            return false;
        }
        let rest: String = chars[1..].iter().collect();
        if py_float(&rest).is_none() {
            return false;
        }
        // cmd[1] raises IndexError (-> False) for one char commands
        let Some(c1) = chars.get(1) else { return false };
        chars[0].is_uppercase() && c1.is_numeric()
    }

    fn handlers_get(&self, cmd: &str) -> Option<Handler> {
        match self.active.get() {
            Active::Base => self.base_gcode_handlers.borrow().get(cmd).cloned(),
            Active::Ready => self.ready_gcode_handlers.borrow().get(cmd).cloned(),
        }
    }

    fn handler_names(&self) -> Vec<String> {
        match self.active.get() {
            Active::Base => self.base_gcode_handlers.borrow().keys().cloned().collect(),
            Active::Ready => self.ready_gcode_handlers.borrow().keys().cloned().collect(),
        }
    }

    /// register_command(cmd, func, when_not_ready, desc); func None
    /// unregisters and returns the old ready handler
    pub fn register_command(self: &Rc<Self>, cmd: &str, func: Option<Handler>, when_not_ready: bool, desc: Option<&str>) -> KResult<Option<Handler>> {
        let Some(mut func) = func else {
            let old_cmd = self.ready_gcode_handlers.borrow_mut().shift_remove(cmd);
            self.base_gcode_handlers.borrow_mut().shift_remove(cmd);
            self.build_status_commands();
            return Ok(old_cmd);
        };
        if self.ready_gcode_handlers.borrow().contains_key(cmd) {
            return Err(KError::config(format!("gcode command {} already registered", cmd)));
        }
        if !self.is_traditional_gcode(cmd) {
            let chars: Vec<char> = cmd.chars().collect();
            let alnum = cmd.replace('_', "A");
            if cmd.to_uppercase() != cmd
                || alnum.is_empty()
                || !alnum.chars().all(|c| c.is_alphanumeric())
                || chars.first().map(|c| c.is_numeric()).unwrap_or(true)
                || chars.get(1).map(|c| c.is_numeric()).unwrap_or(false)
            {
                return Err(KError::config(format!("Can't register '{}' as it is an invalid name", cmd)));
            }
            let origfunc = func;
            let g = self.clone();
            func = Rc::new(move |params: &GCodeCommand| {
                g.get_extended_params(params)?;
                origfunc(params)
            });
        }
        self.ready_gcode_handlers.borrow_mut().insert(cmd.to_string(), func.clone());
        if when_not_ready {
            self.base_gcode_handlers.borrow_mut().insert(cmd.to_string(), func);
        }
        if let Some(desc) = desc {
            self.gcode_help.borrow_mut().insert(cmd.to_string(), desc.to_string());
        }
        self.build_status_commands();
        Ok(None)
    }

    /// register_mux_command(cmd, key, value, func, desc); value None is
    /// python's None key
    pub fn register_mux_command(self: &Rc<Self>, cmd: &str, key: &str, value: Option<&str>, func: Handler, desc: Option<&str>) -> KResult<()> {
        self.register_mux_command_repr(cmd, key, value, func, desc, None)
    }

    /// register_mux_command with python's repr of the handler (a bound
    /// method), shown when a value is registered twice
    pub fn register_mux_command_repr(self: &Rc<Self>, cmd: &str, key: &str, value: Option<&str>, func: Handler, desc: Option<&str>, repr: Option<String>) -> KResult<()> {
        if !self.mux_commands.borrow().contains_key(cmd) {
            let g = self.clone();
            let c = cmd.to_string();
            self.register_command(cmd, Some(Rc::new(move |gcmd: &GCodeCommand| g.cmd_mux(&c, gcmd))), false, desc)?;
            self.mux_commands.borrow_mut().insert(cmd.to_string(), (key.to_string(), IndexMap::new()));
        }
        let mut mux = self.mux_commands.borrow_mut();
        let (prev_key, prev_values) = mux.get_mut(cmd).unwrap();
        let show = |v: Option<&str>| v.map(|s| s.to_string()).unwrap_or_else(|| "None".into());
        if prev_key != key {
            return Err(KError::config(format!("mux command {} {} {} may have only one key ({})", cmd, key, show(value), prev_key)));
        }
        if prev_values.contains_key(&value.map(|s| s.to_string())) {
            // python prints the dict of registered values (handler reprs)
            let keys: Vec<String> = prev_values
                .iter()
                .map(|(k, (_, r))| {
                    let k = k.as_deref().map(py_str_repr).unwrap_or_else(|| "None".into());
                    match r {
                        Some(r) => format!("{}: {}", k, r),
                        None => k,
                    }
                })
                .collect();
            return Err(KError::config(format!("mux command {} {} {} already registered ({{{}}})", cmd, key, show(value), keys.join(", "))));
        }
        prev_values.insert(value.map(|s| s.to_string()), (func, repr));
        Ok(())
    }

    /// the names in base_gcode_handlers / ready_gcode_handlers / the active
    /// dict (gcode_handlers), for python plugins
    pub fn handler_dict_names(&self, which: &str) -> Vec<String> {
        match which {
            "base" => self.base_gcode_handlers.borrow().keys().cloned().collect(),
            "ready" => self.ready_gcode_handlers.borrow().keys().cloned().collect(),
            _ => match self.active.get() {
                Active::Base => self.base_gcode_handlers.borrow().keys().cloned().collect(),
                Active::Ready => self.ready_gcode_handlers.borrow().keys().cloned().collect(),
            },
        }
    }

    /// a handler of one of those dicts
    pub fn handler_in(&self, which: &str, name: &str) -> Option<Handler> {
        match which {
            "base" => self.base_gcode_handlers.borrow().get(name).cloned(),
            "ready" => self.ready_gcode_handlers.borrow().get(name).cloned(),
            _ => self.handlers_get(name),
        }
    }

    pub fn get_command_help(&self) -> IndexMap<String, String> {
        self.gcode_help.borrow().clone()
    }

    fn build_status_commands(&self) {
        let mut commands: IndexMap<String, PyVal> = IndexMap::new();
        for cmd in self.handler_names() {
            commands.insert(cmd, PyVal::dict());
        }
        for (cmd, help) in self.gcode_help.borrow().iter() {
            if let Some(PyVal::Dict(d)) = commands.get_mut(cmd) {
                d.insert("help".to_string(), PyVal::Str(help.clone()));
            }
        }
        *self.status_commands.borrow_mut() = PyVal::Dict(commands);
    }

    /// register_output_handler(obj.method): the callback and its object
    pub fn register_output_handler_from(&self, cb: OutputCallback, origin: CallbackOrigin) {
        self.output_callbacks.borrow_mut().push((cb, Some(origin)));
    }

    /// self.output_callbacks (python plugins)
    pub fn output_callbacks(&self) -> Vec<(OutputCallback, Option<CallbackOrigin>)> {
        self.output_callbacks.borrow().clone()
    }

    pub fn register_output_handler(&self, cb: OutputCallback) {
        self.output_callbacks.borrow_mut().push((cb, None));
    }

    fn handle_shutdown(&self) -> KResult<()> {
        if !self.is_printer_ready.get() {
            return Ok(());
        }
        self.is_printer_ready.set(false);
        self.active.set(Active::Base);
        self.build_status_commands();
        self.respond_state("Shutdown");
        Ok(())
    }

    fn handle_disconnect(&self) -> KResult<()> {
        self.respond_state("Disconnect");
        Ok(())
    }

    fn handle_ready(&self) -> KResult<()> {
        self.is_printer_ready.set(true);
        self.active.set(Active::Ready);
        self.build_status_commands();
        self.respond_state("Ready");
        Ok(())
    }

    /// re.split("([A-Z_]+|[A-Z*])", line)
    fn args_split(line: &str) -> Vec<String> {
        let mut parts = vec![String::new()];
        let chars: Vec<char> = line.chars().collect();
        let mut i = 0;
        while i < chars.len() {
            let c = chars[i];
            if c.is_ascii_uppercase() || c == '_' {
                let mut tok = String::new();
                while i < chars.len() && (chars[i].is_ascii_uppercase() || chars[i] == '_') {
                    tok.push(chars[i]);
                    i += 1;
                }
                parts.push(tok);
                parts.push(String::new());
            } else if c == '*' {
                parts.push("*".into());
                parts.push(String::new());
                i += 1;
            } else {
                parts.last_mut().unwrap().push(c);
                i += 1;
            }
        }
        parts
    }

    pub fn process_commands(self: &Rc<Self>, commands: &[String], need_ack: bool) -> KResult<()> {
        for line in commands {
            // Ignore comments and leading/trailing spaces
            let origline = py_strip(line).to_string();
            let mut line = origline.clone();
            if let Some(cpos) = line.find(';') {
                line.truncate(cpos);
            }
            // Break line into parts and determine command
            let parts = Self::args_split(&line.to_uppercase());
            let cmd = if parts[..parts.len().min(2)].concat() == "N" {
                // Skip line number at start of command
                py_strip(&parts.get(3..5.min(parts.len())).map(|p| p.concat()).unwrap_or_default()).to_string()
            } else {
                py_strip(&parts[..parts.len().min(3)].concat()).to_string()
            };
            // Build gcode "params" dictionary
            let mut params = IndexMap::new();
            let mut i = 1;
            while i < parts.len() {
                params.insert(parts[i].clone(), py_strip(parts.get(i + 1).map(|s| s.as_str()).unwrap_or("")).to_string());
                i += 2;
            }
            let gcmd = Rc::new(GCodeCommand {
                command: RefCell::new(cmd.clone()),
                commandline: origline,
                params: RefCell::new(params),
                need_ack: Cell::new(need_ack),
                gcode: self.clone(),
            });
            // Invoke handler for command
            if self.script_context.get() > 0 && cmd == "RETURN" {
                return Ok(());
            }
            // (python plugins keep the command object itself: crate::running)
            let res = crate::running::run(&gcmd, |g| match self.handlers_get(&cmd) {
                Some(handler) => handler(g),
                None => self.cmd_default(g),
            });
            if let Err(e) = res {
                if e.is_command_error() {
                    self.respond_error(&e.to_string());
                    self.printer.send_event("gcode:command_error", &[])?;
                } else {
                    let msg = format!("Internal error on command:\"{}\"", cmd);
                    logging::exception_err(&msg, &e);
                    self.printer.invoke_shutdown(&msg, None);
                    self.respond_error(&msg);
                }
                if !need_ack {
                    return Err(e);
                }
            }
            gcmd.ack(None);
        }
        Ok(())
    }

    pub fn run_script_from_command(self: &Rc<Self>, script: &str) -> KResult<()> {
        self.script_context.set(self.script_context.get() + 1);
        let lines: Vec<String> = script.split('\n').map(|s| s.to_string()).collect();
        let res = self.process_commands(&lines, false);
        self.script_context.set(self.script_context.get() - 1);
        res
    }

    pub fn run_script(self: &Rc<Self>, script: &str) -> KResult<()> {
        let lines: Vec<String> = script.split('\n').map(|s| s.to_string()).collect();
        if script.contains("INTERRUPT") {
            return self.process_commands(&lines, false);
        }
        self.mutex.lock()?;
        let res = self.process_commands(&lines, false);
        self.mutex.unlock();
        res
    }

    pub fn get_mutex(&self) -> Rc<ReactorMutex> {
        self.mutex.clone()
    }

    pub fn create_gcode_command(self: &Rc<Self>, command: &str, commandline: &str, params: IndexMap<String, String>) -> GCodeCommand {
        GCodeCommand {
            command: RefCell::new(command.to_string()),
            commandline: commandline.to_string(),
            params: RefCell::new(params),
            need_ack: Cell::new(false),
            gcode: self.clone(),
        }
    }

    // Response handling
    pub fn respond_raw(&self, msg: &str) {
        let cbs: Vec<OutputCallback> = self.output_callbacks.borrow().iter().map(|(cb, _)| cb.clone()).collect();
        for cb in cbs {
            cb(msg);
        }
    }

    pub fn respond_info(&self, msg: &str, log: bool) {
        if log {
            logging::info(msg);
        }
        let lines: Vec<&str> = py_strip(msg).split('\n').map(py_strip).collect();
        self.respond_raw(&format!("// {}", lines.join("\n// ")));
    }

    pub fn respond_error(&self, msg: &str) {
        logging::warning(msg);
        let lines: Vec<&str> = py_strip(msg).split('\n').collect();
        if lines.len() > 1 {
            self.respond_info(&lines.join("\n"), false);
        }
        self.respond_raw(&format!("!! {}", py_strip(lines[0])));
        if self.is_fileinput {
            self.printer.request_exit("error_exit");
        }
    }

    fn respond_state(&self, state: &str) {
        self.respond_info(&format!("Klipper state: {}", state), false);
    }

    // Parameter parsing helpers
    pub fn get_extended_params(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let rawparams = gcmd.get_raw_command_parameters();
        // Extract args while allowing shell style quoting
        let malformed = || KError::command(format!("Malformed command '{}'", gcmd.get_commandline()));
        let args = py_shlex_split(&rawparams, "#;").map_err(|_| malformed())?;
        let mut eparams = IndexMap::new();
        for earg in args {
            // earg.split("=", 1) unpacked into k, v
            let (k, v) = earg.split_once('=').ok_or_else(malformed)?;
            eparams.insert(k.to_uppercase(), v.to_string());
        }
        // Update gcmd with new parameters
        *gcmd.params.borrow_mut() = eparams;
        Ok(())
    }

    // G-Code special command handlers
    fn cmd_default(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = gcmd.get_command();
        if cmd == "M105" {
            // Don't warn about temperature requests when not ready
            gcmd.ack(Some("T:0"));
            return Ok(());
        }
        if cmd == "M21" {
            // Don't warn about sd card init when not ready
            return Ok(());
        }
        if !self.is_printer_ready.get() {
            return Err(gcmd.error(self.printer.get_state_message().0));
        }
        if cmd.is_empty() {
            let cmdline = gcmd.get_commandline();
            if !cmdline.is_empty() {
                logging::debug(cmdline);
            }
            return Ok(());
        }
        if cmd.contains(' ') {
            // Handle M117/M118 gcode with numeric and special characters
            let realcmd = cmd.split_whitespace().next().unwrap_or("").to_string();
            if ["M117", "M118", "M23"].contains(&realcmd.as_str()) {
                if let Some(handler) = self.handlers_get(&realcmd) {
                    *gcmd.command.borrow_mut() = realcmd;
                    return handler(gcmd);
                }
            }
        } else if (cmd == "M140" || cmd == "M104") && gcmd.get_float("S", Some(0.0), Lim::new())? == 0.0 {
            // Don't warn about requests to turn off heaters when not present
            return Ok(());
        } else if cmd == "M107" || (cmd == "M106" && (gcmd.get_float("S", Some(1.0), Lim::new())? == 0.0 || self.is_fileinput)) {
            // Don't warn about requests to turn off fan when fan not present
            return Ok(());
        }
        gcmd.respond_info(&format!("Unknown command:\"{}\"", cmd), true);
        self.printer.send_event("gcode:unknown_command", &[&cmd])
    }

    fn cmd_mux(&self, command: &str, gcmd: &GCodeCommand) -> KResult<()> {
        let (key, func) = {
            let mux = self.mux_commands.borrow();
            let (key, values) = mux.get(command).unwrap();
            let key_param = if values.contains_key(&None) { gcmd.get_opt(key) } else { Some(gcmd.get(key)?) };
            match values.get(&key_param) {
                Some((f, _)) => (key.clone(), f.clone()),
                None => {
                    let shown = key_param.unwrap_or_else(|| "None".into());
                    return Err(gcmd.error(format!("The value '{}' is not valid for {}", shown, key)));
                }
            }
        };
        let _ = key;
        func(gcmd)
    }

    // Low-level G-Code commands that are needed before the config file is loaded
    pub fn cmd_m112(&self) -> KResult<()> {
        // Emergency Stop
        self.printer.invoke_shutdown("Shutdown due to M112 command", None);
        Ok(())
    }

    fn cmd_m115(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Get Firmware Version and Capabilities
        let software_version = self.printer.start_arg("software_version").map(|v| v.str()).unwrap_or_else(|| "None".into());
        let msg = format!("FIRMWARE_NAME:Kalico FIRMWARE_VERSION:{}", software_version);
        if !gcmd.ack(Some(&msg)) {
            gcmd.respond_info(&msg, true);
        }
        Ok(())
    }

    pub fn request_restart(&self, result: &str) -> KResult<()> {
        if self.is_printer_ready.get() {
            let toolhead = self.printer.lookup::<crate::toolhead::ToolHead>("toolhead")?;
            let print_time = toolhead.get_last_move_time()?;
            if result == "exit" {
                logging::info(&format!("Exiting (print time {:.3}s)", print_time));
            }
            self.printer.send_event("gcode:request_restart", &[&print_time])?;
            toolhead.dwell(0.500)?;
            toolhead.wait_moves()?;
        }
        self.printer.request_exit(result);
        Ok(())
    }

    fn cmd_status(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.is_printer_ready.get() {
            self.respond_state("Ready");
            return Ok(());
        }
        let msg = self.printer.get_state_message().0;
        let msg = format!("{}\nKlipper state: Not ready", crate::pyfmt::py_rstrip(&msg));
        Err(gcmd.error(msg))
    }

    fn cmd_help(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut cmdhelp = vec![];
        if !self.is_printer_ready.get() {
            cmdhelp.push("Printer is not ready - not all commands available.".to_string());
        }
        cmdhelp.push("Available extended commands:".to_string());
        let mut names = self.handler_names();
        names.sort();
        let help = self.gcode_help.borrow();
        for cmd in names {
            if let Some(h) = help.get(&cmd) {
                cmdhelp.push(format!("{:<10}: {}", cmd, h));
            }
        }
        gcmd.respond_info(&cmdhelp.join("\n"), false);
        Ok(())
    }
}

// Support reading gcode from a pseudo-tty interface
pub struct GCodeIO {
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    gcode_mutex: Rc<ReactorMutex>,
    fd: RawFd,
    is_printer_ready: Cell<bool>,
    is_processing_data: Cell<bool>,
    is_fileinput: bool,
    pipe_is_active: Cell<bool>,
    fd_handle: RefCell<Option<FileHandler>>,
    partial_input: RefCell<String>,
    pending_commands: RefCell<Vec<String>>,
    bytes_read: Cell<usize>,
    input_log: RefCell<VecDeque<(f64, String)>>,
}

impl PrinterObject for GCodeIO {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        self.fd_handle.borrow_mut().take();
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        Some((false, format!("gcodein={}", self.bytes_read.get())))
    }
}

/// m112_r = r"^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)" (match)
fn m112_match(line: &str) -> bool {
    let c: Vec<char> = line.chars().collect();
    let mut i = 0;
    if matches!(c.first(), Some('n' | 'N')) && c.get(1).map(|x| x.is_ascii_digit()).unwrap_or(false) {
        i = 1;
        while c.get(i).map(|x| x.is_ascii_digit()).unwrap_or(false) {
            i += 1;
        }
    }
    let try_at = |mut i: usize| {
        while c.get(i).map(|x| py_isspace(*x)).unwrap_or(false) {
            i += 1;
        }
        if !matches!(c.get(i), Some('m' | 'M')) || c.get(i + 1..i + 4).map(|s| s != ['1', '1', '2']).unwrap_or(true) {
            return false;
        }
        c.get(i + 4).map(|x| py_isspace(*x)).unwrap_or(true)
    };
    // the optional line number may also be skipped (regex backtracking)
    try_at(i) || (i > 0 && try_at(0))
}

impl GCodeIO {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let fd = printer.start_arg("gcode_fd").and_then(|v| v.as_i64()).unwrap_or(-1) as RawFd;
        let io = Rc::new(GCodeIO {
            printer: printer.clone(),
            gcode_mutex: gcode.get_mutex(),
            gcode: gcode.clone(),
            fd,
            is_printer_ready: Cell::new(false),
            is_processing_data: Cell::new(false),
            is_fileinput: printer.start_arg("debuginput").map(|v| v.truthy()).unwrap_or(false),
            pipe_is_active: Cell::new(true),
            fd_handle: RefCell::new(None),
            partial_input: RefCell::new(String::new()),
            pending_commands: RefCell::new(vec![]),
            bytes_read: Cell::new(0),
            input_log: RefCell::new(VecDeque::new()),
        });
        if !io.is_fileinput {
            let i = io.clone();
            let origin: CallbackOrigin = (io.clone(), std::any::type_name::<GCodeIO>());
            gcode.register_output_handler_from(Rc::new(move |msg| i.respond_raw(msg)), origin);
            io.register_fd();
        }
        let i = io.clone();
        printer.register_event_handler("klippy:ready", Rc::new(move |_| i.handle_ready()));
        let i = io.clone();
        printer.register_event_handler("klippy:shutdown", Rc::new(move |_| i.handle_shutdown()));
        let i = io.clone();
        printer.register_event_handler(
            "klippy:analyze_shutdown",
            Rc::new(move |_| {
                i.handle_analyze_shutdown();
                Ok(())
            }),
        );
        Ok(io)
    }

    fn register_fd(self: &Rc<Self>) {
        let i = self.clone();
        let h = self.printer.reactor.register_fd(self.fd, Box::new(move |e| i.process_data(e)), None);
        *self.fd_handle.borrow_mut() = Some(h);
    }

    fn unregister_fd(&self) {
        if let Some(h) = self.fd_handle.borrow_mut().take() {
            self.printer.reactor.unregister_fd(&h);
        }
    }

    fn handle_ready(self: &Rc<Self>) -> KResult<()> {
        self.is_printer_ready.set(true);
        if self.is_fileinput && self.fd_handle.borrow().is_none() {
            self.register_fd();
        }
        Ok(())
    }

    fn handle_analyze_shutdown(&self) {
        let log = self.input_log.borrow();
        let mut out = vec![format!("Dumping gcode input {} blocks", log.len())];
        for (eventtime, data) in log.iter() {
            out.push(format!("Read {:.6}: {}", eventtime, py_str_repr(data)));
        }
        logging::info(&out.join("\n"));
    }

    fn handle_shutdown(&self) -> KResult<()> {
        if !self.is_printer_ready.get() {
            return Ok(());
        }
        self.is_printer_ready.set(false);
        if self.is_fileinput {
            self.printer.request_exit("error_exit");
        }
        Ok(())
    }

    fn process_data(self: &Rc<Self>, eventtime: f64) {
        if let Err(e) = self.process_data_inner(eventtime) {
            self.printer.unhandled_exception(&e);
        }
    }

    fn process_data_inner(self: &Rc<Self>, eventtime: f64) -> KResult<()> {
        // Read input, separate by newline, and add to pending_commands
        let mut buf = [0u8; 4096];
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n < 0 {
            logging::exception("Read g-code");
            return Ok(());
        }
        let Ok(data) = String::from_utf8(buf[..n as usize].to_vec()) else {
            logging::exception("Read g-code");
            return Ok(());
        };
        {
            let mut log = self.input_log.borrow_mut();
            log.push_back((eventtime, data.clone()));
            if log.len() > 50 {
                log.pop_front();
            }
        }
        self.bytes_read.set(self.bytes_read.get() + data.chars().count());
        let mut lines: Vec<String> = data.split('\n').map(|s| s.to_string()).collect();
        lines[0] = format!("{}{}", self.partial_input.borrow(), lines[0]);
        *self.partial_input.borrow_mut() = lines.pop().unwrap();
        self.pending_commands.borrow_mut().extend(lines.iter().cloned());
        self.pipe_is_active.set(true);
        // Special handling for debug file input EOF
        if data.is_empty() && self.is_fileinput {
            if !self.is_processing_data.get() {
                self.unregister_fd();
                self.gcode.request_restart("exit")?;
            }
            self.pending_commands.borrow_mut().push(String::new());
        }
        // Handle case where multiple commands pending
        if self.pending_commands.borrow().len() < 20 {
            // Check for M112 out-of-order
            for line in &lines {
                if m112_match(line) {
                    self.gcode.cmd_m112()?;
                }
            }
        }
        if self.is_processing_data.get() {
            if self.pending_commands.borrow().len() >= 20 {
                // Stop reading input
                self.unregister_fd();
            }
            return Ok(());
        }
        // Process commands
        self.is_processing_data.set(true);
        let mut pending_commands = std::mem::take(&mut *self.pending_commands.borrow_mut());
        while !pending_commands.is_empty() {
            self.gcode_mutex.lock()?;
            let res = self.gcode.process_commands(&pending_commands, true);
            self.gcode_mutex.unlock();
            res?;
            pending_commands = std::mem::take(&mut *self.pending_commands.borrow_mut());
        }
        self.is_processing_data.set(false);
        if self.fd_handle.borrow().is_none() {
            self.register_fd();
        }
        Ok(())
    }

    fn respond_raw(&self, msg: &str) {
        if self.pipe_is_active.get() {
            let data = format!("{}\n", msg);
            let r = unsafe { libc::write(self.fd, data.as_ptr() as *const libc::c_void, data.len()) };
            if r < 0 {
                logging::exception("Write g-code response");
                self.pipe_is_active.set(false);
            }
        }
    }
}

pub fn add_early_printer_objects(printer: &Rc<Printer>) -> KResult<()> {
    printer.add_object("gcode", GCodeDispatch::new(printer)?)?;
    printer.add_object("gcode_io", GCodeIO::new(printer)?)?;
    Ok(())
}
