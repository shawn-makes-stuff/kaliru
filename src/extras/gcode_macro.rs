// Add ability to define custom g-code macros (port of klippy/extras/gcode_macro.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Jinja templates run on the jinja module (a port of Jinja2 3.1.6), python
// scripts on RustPython (gcode_python.rs, cargo feature "python").
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::filters::bind;
use crate::jinja::value::{CallCtx, Exc, Key, Object, VResult};
use crate::jinja::{self, Environment, Value};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// a klippy error raised through a template, with its python class
pub fn kerror_exc(e: KError) -> Exc {
    Exc { cls: e.py_class().to_string(), msg: e.to_string(), kerror: Some(e) }
}

// Wrapper for access to printer object get_status() methods
pub struct GetStatusWrapperJinja {
    printer: Rc<Printer>,
    eventtime: Cell<Option<f64>>,
    cache: RefCell<IndexMap<String, Value>>,
}

impl GetStatusWrapperJinja {
    pub fn new(printer: &Rc<Printer>, eventtime: Option<f64>) -> Self {
        GetStatusWrapperJinja { printer: printer.clone(), eventtime: Cell::new(eventtime), cache: RefCell::new(IndexMap::new()) }
    }

    fn get(&self, val: &Value) -> VResult<Value> {
        let sval = crate::pyfmt::py_strip(&val.to_str()).to_string();
        if let Some(v) = self.cache.borrow().get(&sval) {
            return Ok(v.clone());
        }
        let po = self.printer.lookup_object_opt(&sval);
        let po = match po {
            Some(po) if po.has_status() => po,
            _ => return Err(Exc::key_error(val)),
        };
        let eventtime = match self.eventtime.get() {
            Some(e) => e,
            None => {
                let e = self.printer.get_reactor().monotonic();
                self.eventtime.set(Some(e));
                e
            }
        };
        let status = po.try_get_status(eventtime).map_err(kerror_exc)?.unwrap_or(PyVal::None);
        let res = Value::from_pyval(&status);
        self.cache.borrow_mut().insert(sval, res.clone());
        Ok(res)
    }
}

impl Object for GetStatusWrapperJinja {
    fn type_name(&self) -> String {
        "GetStatusWrapperJinja".into()
    }
    fn module(&self) -> String {
        "klippy.extras.gcode_macro".into()
    }
    fn getitem(&self, key: &Value) -> Option<VResult<Value>> {
        Some(self.get(key))
    }
    fn contains(&self, v: &Value) -> Option<VResult<bool>> {
        Some(match self.get(v) {
            Ok(_) => Ok(true),
            Err(e) if e.cls == "KeyError" => Ok(false),
            Err(e) => Err(e),
        })
    }
    fn iter(&self) -> Option<VResult<Vec<Value>>> {
        let mut out = vec![];
        for (name, _) in self.printer.lookup_objects(None) {
            let v = Value::from(name.as_str());
            match self.get(&v) {
                Ok(_) => out.push(v),
                Err(e) if e.cls == "KeyError" => {}
                Err(e) => return Some(Err(e)),
            }
        }
        Some(Ok(out))
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// the PrinterGCodeMacro._action_* bound methods
#[derive(Clone)]
pub(crate) struct Action {
    printer: Rc<Printer>,
    name: &'static str,
}

impl Object for Action {
    fn type_name(&self) -> String {
        "method".into()
    }
    fn repr(&self) -> String {
        format!("<bound method PrinterGCodeMacro.{} of <klippy.extras.gcode_macro.PrinterGCodeMacro object>>", self.name)
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        Some(self.run(args, kwargs))
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl Action {
    #[cfg_attr(not(feature = "python"), allow(dead_code))]
    pub(crate) fn new(printer: &Rc<Printer>, name: &'static str) -> Self {
        Action { printer: printer.clone(), name }
    }

    pub(crate) fn run(&self, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
        let fname = format!("PrinterGCodeMacro.{}", self.name);
        match self.name {
            "_action_emergency_stop" => {
                let (a, _, _) = bind(&fname, &[("msg", Some(Value::from("action_emergency_stop")))], args, kwargs, false, false)?;
                self.printer.invoke_shutdown(&format!("Shutdown due to {}", a[0].to_str()), None);
                Ok(Value::from(""))
            }
            "_action_respond_info" => {
                let (a, _, _) = bind(&fname, &[("msg", None)], args, kwargs, false, false)?;
                let gcode = self.printer.lookup::<GCodeDispatch>("gcode").map_err(kerror_exc)?;
                if !matches!(a[0], Value::Str(_) | Value::Undefined(_)) {
                    // logging.info(msg), then msg.strip() fails
                    logging::info(&a[0].to_str());
                    return Err(Exc::attribute_error(format!("'{}' object has no attribute 'strip'", a[0].type_name())));
                }
                gcode.respond_info(&a[0].to_str(), true);
                Ok(Value::from(""))
            }
            "_action_log" => {
                let (a, _, _) = bind(&fname, &[("msg", None)], args, kwargs, false, false)?;
                logging::info(&a[0].to_str());
                Ok(Value::from(""))
            }
            "_action_raise_error" => {
                let (a, _, _) = bind(&fname, &[("msg", None)], args, kwargs, false, false)?;
                Err(kerror_exc(KError::command(a[0].to_str())))
            }
            _ => {
                // _action_call_remote_method(method, **kwargs)
                let (a, _, kw) = bind(&fname, &[("method", None)], args, kwargs, false, true)?;
                let webhooks = self.printer.lookup::<crate::webhooks::WebHooks>("webhooks").map_err(kerror_exc)?;
                let kw: IndexMap<String, PyVal> = kw.into_iter().map(|(k, v)| (k, v.to_pyval())).collect();
                match webhooks.call_remote_method(&a[0].to_str(), kw) {
                    Err(e) if e.is_command_error() => logging::exception("Remote Call Error"),
                    Err(e) => return Err(kerror_exc(e)),
                    Ok(()) => {}
                }
                Ok(Value::from(""))
            }
        }
    }
}

/// python's str.splitlines()
fn splitlines(s: &str) -> Vec<String> {
    crate::jinja::methods::str_splitlines(s, false)
}

// Wrapper around a Jinja2 template
pub struct TemplateWrapperJinja {
    printer: Rc<Printer>,
    name: String,
    gcode: Rc<GCodeDispatch>,
    template: jinja::Template,
}

impl TemplateWrapperJinja {
    fn new(printer: &Rc<Printer>, env: &Environment, name: &str, script: &str) -> KResult<Self> {
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let template = match env.from_string(script) {
            Ok(t) => t,
            Err(e) => {
                let lines = splitlines(script);
                let line = lines
                    .get(e.lineno.wrapping_sub(1))
                    .cloned()
                    .ok_or_else(|| KError::Other("list index out of range".into()))?;
                let msg = format!("Error loading template '{}'\nline {}: {} # {}", name, e.lineno, line, e.message);
                let cls = if e.assertion {
                    "jinja2.exceptions.TemplateAssertionError"
                } else {
                    "jinja2.exceptions.TemplateSyntaxError"
                };
                logging::exception_tb(&msg, cls, &e.message);
                return Err(KError::command(msg).with_context(KError::Py(cls.into(), e.message.clone())));
            }
        };
        Ok(TemplateWrapperJinja { printer: printer.clone(), name: name.to_string(), gcode, template })
    }

    pub fn render(&self, context: Option<IndexMap<String, Value>>) -> KResult<String> {
        let context = match context {
            Some(c) => c,
            None => create_template_context(&self.printer, None),
        };
        match self.template.render(context) {
            Ok(s) => Ok(s),
            Err(e) => {
                let msg = format!("Error evaluating '{}': {}", self.name, e.format_only());
                logging::exception_tb(&msg, &e.cls, &e.msg);
                Err(KError::command(msg))
            }
        }
    }

    pub fn run_gcode_from_command(&self, context: Option<IndexMap<String, Value>>) -> KResult<()> {
        let script = self.render(context)?;
        self.gcode.run_script_from_command(&script)
    }
}

/// Template.function: the wrapper of a jinja or a python script
#[derive(Clone)]
enum Function {
    Jinja(Rc<TemplateWrapperJinja>),
    #[cfg(feature = "python")]
    Python(Rc<crate::extras::gcode_python::TemplateWrapperPython>),
}

/// gcode_macro.Template: a jinja or a python script
pub struct Template {
    printer: Rc<Printer>,
    name: String,
    function: RefCell<Function>,
}

impl Template {
    fn new(printer: &Rc<Printer>, env: &Environment, name: &str, script: &str, script_type: &str) -> KResult<Rc<Self>> {
        let function = Self::make(printer, env, name, script_type, script)?;
        Ok(Rc::new(Template { printer: printer.clone(), name: name.to_string(), function: RefCell::new(function) }))
    }

    fn make(printer: &Rc<Printer>, env: &Environment, name: &str, script_type: &str, script: &str) -> KResult<Function> {
        if script_type == "python" {
            #[cfg(feature = "python")]
            return Ok(Function::Python(Rc::new(crate::extras::gcode_python::TemplateWrapperPython::new(printer, name, script)?)));
            #[cfg(not(feature = "python"))]
            return Err(KError::config(format!("Error loading template '{}': this Kaliru is built without python macros", name)));
        }
        Ok(Function::Jinja(Rc::new(TemplateWrapperJinja::new(printer, env, name, script)?)))
    }

    pub fn reload(&self, env: &Environment, script_type: &str, script: &str) -> KResult<()> {
        let f = Self::make(&self.printer, env, &self.name, script_type, script)?;
        *self.function.borrow_mut() = f;
        Ok(())
    }

    pub fn render(&self, context: Option<IndexMap<String, Value>>) -> KResult<String> {
        let f = self.function.borrow().clone();
        match f {
            Function::Jinja(f) => f.render(context),
            #[cfg(feature = "python")]
            Function::Python(_) => {
                Err(KError::Py("AttributeError".into(), "'TemplateWrapperPython' object has no attribute 'render'".into()))
            }
        }
    }

    pub fn run_gcode_from_command(&self, context: Option<IndexMap<String, Value>>) -> KResult<()> {
        let f = self.function.borrow().clone();
        match f {
            Function::Jinja(f) => f.run_gcode_from_command(context),
            #[cfg(feature = "python")]
            Function::Python(f) => f.run_gcode_from_command(context),
        }
    }

    pub fn create_template_context(&self) -> IndexMap<String, Value> {
        create_template_context(&self.printer, None)
    }
}

/// PrinterGCodeMacro.create_template_context(eventtime)
pub fn create_template_context(printer: &Rc<Printer>, eventtime: Option<f64>) -> IndexMap<String, Value> {
    let mut ctx = IndexMap::new();
    ctx.insert("printer".to_string(), Value::object(GetStatusWrapperJinja::new(printer, eventtime)));
    for (key, name) in [
        ("action_emergency_stop", "_action_emergency_stop"),
        ("action_respond_info", "_action_respond_info"),
        ("action_log", "_action_log"),
        ("action_raise_error", "_action_raise_error"),
        ("action_call_remote_method", "_action_call_remote_method"),
    ] {
        ctx.insert(key.to_string(), Value::object(Action { printer: printer.clone(), name }));
    }
    ctx.insert("math".to_string(), jinja::pymath::math_module());
    ctx
}

// Main gcode macro template tracking
pub struct PrinterGCodeMacro {
    printer: Rc<Printer>,
    env: Environment,
}

impl PrinterGCodeMacro {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gm = Rc::new(PrinterGCodeMacro { printer: printer.clone(), env: Environment::new("{%", "%}", "{", "}") });
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let g = gm.clone();
        gcode.register_command("RELOAD_GCODE_MACROS", Some(Rc::new(move |gcmd| g.cmd_reload_gcode_macros(gcmd))), false, None)?;
        Ok(gm)
    }

    /// load_template(config, option, default)
    pub fn load_template(&self, config: &ConfigWrapper, option: &str, default: Option<&str>) -> KResult<Rc<Template>> {
        let name = format!("{}:{}", config.get_name(), option);
        let (script_type, script) = match default {
            None => config.getscript(option, Dflt::Required)?,
            Some(d) => config.getscript(option, d)?,
        };
        Template::new(&self.printer, &self.env, &name, &script, script_type)
    }

    fn cmd_reload_gcode_macros(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let config_file = self.printer.start_arg("config_file").map(|v| v.str()).unwrap_or_default();
        let mut pconfig = crate::configfile::PrinterConfig::new(&config_file);
        let new_config = pconfig.read_main_config()?;
        let names: Vec<String> = new_config.get_prefix_sections("gcode_macro").iter().map(|s| s.get_name().to_string()).collect();
        for (name, obj) in self.printer.lookup_objects(Some("gcode_macro")) {
            let Some(m) = obj.as_any().downcast_ref::<GCodeMacro>() else { continue };
            if names.contains(&name) {
                let new_section = new_config.getsection(&name);
                let (script_type, new_script) = new_section.getscript("gcode", Dflt::Required)?;
                m.template.reload(&self.env, script_type, &new_script)?;
            }
        }
        Ok(())
    }
}

impl PrinterObject for PrinterGCodeMacro {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterGCodeMacro::new(config)?)
}

/// printer.load_object(config, "gcode_macro")
pub fn load_gcode_macro(config: &ConfigWrapper) -> KResult<Rc<PrinterGCodeMacro>> {
    let printer = config.get_printer();
    printer
        .load_object(config, "gcode_macro", true)?
        .and_then(crate::printer::downcast_rc::<PrinterGCodeMacro>)
        .ok_or_else(|| KError::Other("gcode_macro has an unexpected type".into()))
}

// ----------------------------------------------------------------------
// GCode macro
// ----------------------------------------------------------------------

const CMD_SET_GCODE_VARIABLE_HELP: &str = "Set the value of a G-Code macro variable";

pub struct GCodeMacro {
    alias: String,
    template: Rc<Template>,
    gcode: Rc<GCodeDispatch>,
    rename_existing: Option<String>,
    cmd_desc: String,
    in_script: Cell<bool>,
    pub variables: RefCell<IndexMap<String, Value>>,
}

impl GCodeMacro {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        if config.get_name().split_whitespace().count() > 2 {
            return Err(KError::config(format!("Name of section '{}' contains illegal whitespace", config.get_name())));
        }
        let name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let alias = name.to_uppercase();
        let printer = config.get_printer();
        let gcode_macro = load_gcode_macro(config)?;
        let template = gcode_macro.load_template(config, "gcode", None)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let rename_existing = config.get_opt("rename_existing")?;
        let cmd_desc = config.get("description", "G-Code macro")?;
        let m = Rc::new(GCodeMacro {
            alias: alias.clone(),
            template,
            gcode: gcode.clone(),
            rename_existing: rename_existing.clone(),
            cmd_desc: cmd_desc.clone(),
            in_script: Cell::new(false),
            variables: RefCell::new(IndexMap::new()),
        });
        if let Some(rename) = &rename_existing {
            if gcode.is_traditional_gcode(&alias) != gcode.is_traditional_gcode(rename) {
                return Err(KError::config(format!(
                    "G-Code macro rename of different types ('{}' vs '{}')",
                    alias, rename
                )));
            }
            let mc = m.clone();
            printer.register_event_handler("klippy:connect", Rc::new(move |_| mc.handle_connect()));
        } else {
            let mc = m.clone();
            gcode.register_command(&alias, Some(Rc::new(move |g| mc.cmd(g))), false, Some(&cmd_desc))?;
        }
        let mc = m.clone();
        gcode.register_mux_command(
            "SET_GCODE_VARIABLE",
            "MACRO",
            Some(&name),
            Rc::new(move |g| mc.cmd_set_gcode_variable(g)),
            Some(CMD_SET_GCODE_VARIABLE_HELP),
        )?;
        let prefix = "variable_";
        for option in config.get_prefix_options(prefix)? {
            let text = config.get(&option, Dflt::Required)?;
            match crate::pyliteral::literal_eval_json_noted(&text) {
                Ok(literal) => {
                    m.variables.borrow_mut().insert(option[prefix.len()..].to_string(), literal);
                }
                Err(((cls, e), notes)) => {
                    let msg = format!("Option '{}' in section '{}' is not a valid literal: {}", option, config.get_name(), e);
                    // (the exception's notes print below its message)
                    let text = notes.iter().fold(e, |t, n| t + "\n" + n);
                    return Err(KError::config(msg).with_context(KError::Py(cls.to_string(), text)));
                }
            }
        }
        Ok(m)
    }

    fn handle_connect(self: &Rc<Self>) -> KResult<()> {
        let prev_cmd = self.gcode.register_command(&self.alias, None, false, None)?;
        let Some(prev_cmd) = prev_cmd else {
            return Err(KError::config(format!("Existing command '{}' not found in gcode_macro rename", self.alias)));
        };
        let pdesc = format!("Renamed builtin of '{}'", self.alias);
        let rename = self.rename_existing.clone().unwrap_or_default();
        self.gcode.register_command(&rename, Some(prev_cmd), false, Some(&pdesc))?;
        let mc = self.clone();
        self.gcode.register_command(&self.alias, Some(Rc::new(move |g| mc.cmd(g))), false, Some(&self.cmd_desc))?;
        Ok(())
    }

    fn cmd_set_gcode_variable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let variable = gcmd.get("VARIABLE")?;
        let value = gcmd.get("VALUE")?;
        if !self.variables.borrow().contains_key(&variable) {
            return Err(gcmd.error(format!("Unknown gcode_macro variable '{}'", variable)));
        }
        let literal = crate::pyliteral::literal_eval_json(&value).map_err(|(_, e)| {
            gcmd.error(format!("Unable to parse '{}' as a literal: {} in '{}'", value, e, gcmd.get_commandline()))
        })?;
        let mut v = self.variables.borrow().clone();
        v.insert(variable, literal);
        *self.variables.borrow_mut() = v;
        Ok(())
    }

    fn cmd(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.in_script.get() {
            return Err(gcmd.error(format!("Macro {} called recursively", self.alias)));
        }
        let mut kwparams: IndexMap<String, Value> = self.variables.borrow().clone();
        kwparams.extend(self.template.create_template_context());
        let params: IndexMap<Key, Value> =
            gcmd.get_command_parameters().into_iter().map(|(k, v)| (Key::str(&k), Value::from(v))).collect();
        kwparams.insert("params".into(), Value::dict(params));
        kwparams.insert("rawparams".into(), Value::from(gcmd.get_raw_command_parameters()));
        self.in_script.set(true);
        let res = self.template.run_gcode_from_command(Some(kwparams));
        self.in_script.set(false);
        res
    }
}

impl PrinterObject for GCodeMacro {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(self.variables.borrow().iter().map(|(k, v)| (k.clone(), v.to_pyval())).collect()))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(GCodeMacro::new(config)?)
}

/// the template's printer
impl Template {
    pub fn get_printer(&self) -> Rc<Printer> {
        self.printer.clone()
    }
}
