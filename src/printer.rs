// Main code for host side printer firmware (port of klippy/printer.py)
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Object model (see NOTES.md "Object model"):
// - printer objects are Rc<dyn PrinterObject>; get_status() / stats() are
//   the printer-wide duck-typed methods (None = python has no such method),
//   module specific access goes through as_any() downcasts
// - event handlers get their parameters as &[&dyn Any] and return KResult
// - the python exception classes are crate::error::KError
// - modules come from the static registry crate::extras::MODULES (python
//   scans klippy/extras + plugins with pkgutil); modules not ported yet
//   raise a config error instead of being skipped
// - objects and closures hold Rc<Printer>; the cycles are broken when a
//   printer instance ends (teardown())
use crate::configfile::{ConfigWrapper, PrinterConfig};
use crate::error::{KError, KResult};
use crate::logging;
use crate::pyval::PyVal;
use crate::queuelogger::{self, QueueListener};
use crate::reactor::{Reactor, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const APP_NAME: &str = "Kalico";

pub const MESSAGE_READY: &str = "Printer is ready";

pub const MESSAGE_STARTUP: &str = "
Printer is not ready
The klippy host software is attempting to connect.  Please
retry in a few moments.
";

pub const MESSAGE_RESTART: &str = "
Once the underlying issue is corrected, use the \"RESTART\"
command to reload the config and restart the host software.
Printer is halted
";

pub trait PrinterObject: Any {
    fn as_any(&self) -> &dyn Any;
    /// the Rust type's path (python plugins find the Kalico class by it)
    fn type_name(&self) -> &'static str {
        std::any::type_name::<Self>()
    }
    /// hasattr(obj, "get_status"); override together with get_status()
    fn has_status(&self) -> bool {
        false
    }
    /// get_status(eventtime); None where the python object has none
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        None
    }
    /// get_status(eventtime) where it can raise (an AttributeError and the
    /// like); callers that report exceptions use this one
    fn try_get_status(&self, eventtime: f64) -> KResult<Option<PyVal>> {
        Ok(self.get_status(eventtime))
    }
    /// hasattr(obj, "stats"); override together with stats()
    fn has_stats(&self) -> bool {
        false
    }
    /// stats(eventtime) (statistics module); None where python has none
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    /// get_temp(eventtime) -> (current, target) of temperature objects;
    /// None where the python object has no get_temp
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        None
    }
    /// the `.fan` attribute of fan objects (hasattr(obj, "fan"))
    fn fan(&self) -> Option<Rc<crate::extras::fan::Fan>> {
        None
    }
    /// `obj is None`: a module whose load_config returned None
    fn is_none(&self) -> bool {
        false
    }
    /// obj.<name>(*args) by name, for objects Rust code only knows as
    /// printer objects (a python plugin's): None where kaliru's own objects
    /// are called through their Rust type instead
    fn call_method(&self, _name: &str, _args: &[PyVal]) -> Option<KResult<PyVal>> {
        None
    }
    /// not in python: drop references back to the printer / self when the
    /// printer instance ends (see Printer::teardown)
    fn teardown(&self) {}
}

pub type PObj = Rc<dyn PrinterObject>;

/// the None a load_config without a return value stores as the object
pub struct NoneObject;

impl PrinterObject for NoneObject {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn is_none(&self) -> bool {
        true
    }
}

pub fn none_object() -> PObj {
    Rc::new(NoneObject)
}

pub type EventCallback = Rc<dyn Fn(&[&dyn Any]) -> KResult<()>>;

pub type LoadFn = fn(&ConfigWrapper) -> KResult<PObj>;

/// an entry of the module registry (python: a module in extras/)
pub struct PrinterModule {
    pub name: &'static str,
    /// false: exists in Kalico, not ported yet
    pub ported: bool,
    pub load_config: Option<LoadFn>,
    pub load_config_prefix: Option<LoadFn>,
    pub register_components: Option<fn(&SubsystemComponentCollection) -> KResult<()>>,
}

pub struct SubsystemComponentCollection {
    subsystems: RefCell<IndexMap<String, IndexMap<String, Rc<dyn Any>>>>,
}

impl SubsystemComponentCollection {
    pub fn get_components(&self, subsystem_name: &str) -> IndexMap<String, Rc<dyn Any>> {
        self.subsystems.borrow_mut().entry(subsystem_name.to_string()).or_default().clone()
    }

    pub fn register_component(&self, subsystem_name: &str, component_name: &str, component: Rc<dyn Any>) -> KResult<()> {
        let mut subs = self.subsystems.borrow_mut();
        let subsystem = subs.entry(subsystem_name.to_string()).or_default();
        if subsystem.contains_key(component_name) {
            return Err(KError::config(format!(
                "Component '{}:{}' is already registered",
                subsystem_name, component_name
            )));
        }
        subsystem.insert(component_name.to_string(), component);
        Ok(())
    }
}

pub struct Printer {
    pub bglogger: Option<Rc<QueueListener>>,
    pub start_args: RefCell<IndexMap<String, PyVal>>,
    pub reactor: Rc<Reactor>,
    state_message: RefCell<String>,
    in_shutdown_state: Cell<bool>,
    run_result: RefCell<Option<String>>,
    event_handlers: RefCell<IndexMap<String, Vec<EventCallback>>>,
    pub components: SubsystemComponentCollection,
    objects: RefCell<IndexMap<String, PObj>>,
    /// the PrinterConfig ("configfile" object)
    pub pconfig: RefCell<Option<Rc<ConfigFileObject>>>,
    /// python modules (klippy/extras files kaliru has no built-in for,
    /// klippy/plugins): name -> where (Kalico's printer_modules)
    #[cfg(feature = "plugins")]
    py_modules: RefCell<IndexMap<String, crate::plugins::Package>>,
}

/// the "configfile" printer object
pub struct ConfigFileObject {
    pub pconfig: RefCell<PrinterConfig>,
}

impl ConfigFileObject {
    fn cmd_save_config(&self, printer: &Printer, gcmd: &crate::gcode::GCodeCommand) -> KResult<()> {
        let autosave_includes = crate::extras::danger_options::get_danger_options().autosave_includes;
        let saved = self.pconfig.borrow_mut().save_config(autosave_includes).map_err(KError::command)?;
        if !saved {
            return Ok(());
        }
        let gcode = printer.lookup::<crate::gcode::GCodeDispatch>("gcode")?;
        // If requested restart or no restart just flag config saved
        let require_restart = gcmd.get_int("RESTART", Some(1), crate::configfile::Lim::new().min(0).max(1))?;
        if require_restart != 0 {
            // Request a restart
            gcode.request_restart("restart")
        } else {
            // flag config updated to false since config saved with no restart
            self.pconfig.borrow_mut().save_config_pending = false;
            gcode.respond_info("Config update without restart successful", true);
            Ok(())
        }
    }
}

impl PrinterObject for ConfigFileObject {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.pconfig.borrow().get_status(eventtime))
    }
}

impl Printer {
    pub fn new(main_reactor: Rc<Reactor>, bglogger: Option<Rc<QueueListener>>, start_args: IndexMap<String, PyVal>) -> KResult<Rc<Printer>> {
        let printer = Rc::new(Printer {
            bglogger,
            start_args: RefCell::new(start_args),
            reactor: main_reactor.clone(),
            state_message: RefCell::new(MESSAGE_STARTUP.into()),
            in_shutdown_state: Cell::new(false),
            run_result: RefCell::new(None),
            event_handlers: RefCell::new(IndexMap::new()),
            components: SubsystemComponentCollection { subsystems: RefCell::new(IndexMap::new()) },
            objects: RefCell::new(IndexMap::new()),
            pconfig: RefCell::new(None),
            #[cfg(feature = "plugins")]
            py_modules: RefCell::new(IndexMap::new()),
        });
        let p = printer.clone();
        main_reactor.register_callback(move |e| p.connect(e), NOW);
        // Init printer components that must be setup prior to config
        crate::gcode::add_early_printer_objects(&printer)?;
        crate::webhooks::add_early_printer_objects(&printer)?;
        Ok(printer)
    }

    /// start_args.get(name) (None when missing or None)
    pub fn start_arg(&self, name: &str) -> Option<PyVal> {
        self.start_args.borrow().get(name).filter(|v| **v != PyVal::None).cloned()
    }

    pub fn get_reactor(&self) -> Rc<Reactor> {
        self.reactor.clone()
    }

    pub fn get_state_message(&self) -> (String, &'static str) {
        let msg = self.state_message.borrow().clone();
        let category = if msg == MESSAGE_READY {
            "ready"
        } else if msg == MESSAGE_STARTUP {
            "startup"
        } else if self.in_shutdown_state.get() {
            "shutdown"
        } else {
            "error"
        };
        (msg, category)
    }

    pub fn is_shutdown(&self) -> bool {
        self.in_shutdown_state.get()
    }

    fn set_state(&self, msg: &str) {
        let cur = self.state_message.borrow().clone();
        if cur == MESSAGE_READY || cur == MESSAGE_STARTUP {
            *self.state_message.borrow_mut() = msg.to_string();
        }
        if msg != MESSAGE_READY && self.start_arg("debuginput").is_some() {
            self.request_exit("error_exit");
        }
    }

    pub fn update_error_msg(&self, oldmsg: &str, newmsg: &str) {
        let cur = self.state_message.borrow().clone();
        if cur != oldmsg
            || cur == MESSAGE_READY
            || cur == MESSAGE_STARTUP
            || newmsg == MESSAGE_READY
            || newmsg == MESSAGE_STARTUP
        {
            return;
        }
        *self.state_message.borrow_mut() = newmsg.to_string();
        logging::error(newmsg);
    }

    pub fn add_object(&self, name: &str, obj: PObj) -> KResult<()> {
        let mut objects = self.objects.borrow_mut();
        if objects.contains_key(name) {
            return Err(KError::config(format!("Printer object '{}' already created", name)));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }

    /// lookup_object(name) without default
    pub fn lookup_object(&self, name: &str) -> KResult<PObj> {
        self.lookup_object_opt(name).ok_or_else(|| KError::config(format!("Unknown config object '{}'", name)))
    }

    /// lookup_object(name, None)
    pub fn lookup_object_opt(&self, name: &str) -> Option<PObj> {
        self.objects.borrow().get(name).cloned()
    }

    /// lookup_object(name) downcast to the object's rust type
    pub fn lookup<T: 'static>(&self, name: &str) -> KResult<Rc<T>> {
        let obj = self.lookup_object(name)?;
        downcast_rc::<T>(obj).ok_or_else(|| KError::Other(format!("printer object '{}' has an unexpected type", name)))
    }

    pub fn lookup_components(&self, subsystem_name: &str) -> IndexMap<String, Rc<dyn Any>> {
        self.components.get_components(subsystem_name)
    }

    pub fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, PObj)> {
        let objects = self.objects.borrow();
        let Some(module) = module else {
            return objects.iter().map(|(n, o)| (n.clone(), o.clone())).collect();
        };
        let prefix = format!("{} ", module);
        let mut objs: Vec<(String, PObj)> =
            objects.iter().filter(|(n, _)| n.starts_with(&prefix)).map(|(n, o)| (n.clone(), o.clone())).collect();
        if let Some(o) = objects.get(module) {
            objs.insert(0, (module.to_string(), o.clone()));
        }
        objs
    }

    /// load_object(config, section) as its concrete type
    pub fn load_as<T: 'static>(self: &Rc<Self>, config: &ConfigWrapper, name: &str) -> KResult<Rc<T>> {
        self.load_object(config, name, true)?
            .and_then(downcast_rc::<T>)
            .ok_or_else(|| KError::Other(format!("{} has an unexpected type", name)))
    }

    /// load_object(config, section, default): `required` false is python's
    /// default=None (Ok(None) when there is no such module)
    pub fn load_object(self: &Rc<Self>, config: &ConfigWrapper, section: &str, required: bool) -> KResult<Option<PObj>> {
        if let Some(o) = self.lookup_object_opt(section) {
            return Ok(Some(o));
        }
        // create objects entry from module
        let module_name = section.split_whitespace().next().unwrap_or("");
        #[cfg(feature = "plugins")]
        if let Some(package) = self.py_modules.borrow().get(module_name).cloned() {
            crate::plugins::set_printer(self);
            let obj = match crate::plugins::load_config(config, section, &package)? {
                Some(o) => o,
                None if !required => return Ok(None),
                None => return Err(KError::config(format!("Unable to load module '{}'", module_name))),
            };
            self.objects.borrow_mut().insert(section.to_string(), obj.clone());
            return Ok(Some(obj));
        }
        let Some(pm) = crate::extras::MODULES.iter().find(|m| m.name == module_name) else {
            // unable to find that module
            if !required {
                return Ok(None);
            }
            return Err(KError::config(format!("Module '{}' not found", module_name)));
        };
        if !pm.ported {
            return Err(KError::config(format!("Module '{}' is not ported to Kaliru yet", module_name)));
        }
        let is_prefix = pm.name != section;
        let init_func = if is_prefix { pm.load_config_prefix } else { pm.load_config };
        let Some(init_func) = init_func else {
            if !required {
                return Ok(None);
            }
            return Err(KError::config(format!("Unable to load module '{}'", module_name)));
        };
        let obj = init_func(&config.getsection(section))?;
        self.objects.borrow_mut().insert(section.to_string(), obj.clone());
        Ok(Some(obj))
    }

    fn read_config(self: &Rc<Self>) -> KResult<()> {
        let config_file = self.start_arg("config_file").map(|v| v.str()).unwrap_or_default();
        let pobj = Rc::new(ConfigFileObject { pconfig: RefCell::new(PrinterConfig::new(config_file.as_str())) });
        self.objects.borrow_mut().insert("configfile".into(), pobj.clone());
        *self.pconfig.borrow_mut() = Some(pobj.clone());
        // PrinterConfig.__init__ registers SAVE_CONFIG
        let gcode = self.lookup::<crate::gcode::GCodeDispatch>("gcode")?;
        let (po, p) = (pobj.clone(), self.clone());
        gcode.register_command(
            "SAVE_CONFIG",
            Some(Rc::new(move |gcmd| po.cmd_save_config(&p, gcmd))),
            false,
            Some("Overwrite config file and restart"),
        )?;
        let config = pobj.pconfig.borrow_mut().read_main_config()?.with_printer(self);
        // _load_modules: the built-in modules are kaliru's; python modules
        // come from klippy/extras (third-party files) and klippy/plugins
        let allow_overrides = config.getsection("danger_options").getboolean_value_nv("allow_plugin_override", false)?;
        #[cfg(feature = "plugins")]
        self.load_py_modules(allow_overrides)?;
        #[cfg(not(feature = "plugins"))]
        let _ = allow_overrides;
        self.load_object(&config, "danger_options", true)?;
        if self.bglogger.is_some() && crate::extras::danger_options::get_danger_options().log_config_file_at_startup {
            let text = pobj.pconfig.borrow().log_config(&config);
            self.set_rollover_info("config", Some(&text), true);
        }
        // Register subsystem components
        for pm in crate::extras::MODULES {
            if let (true, Some(register)) = (pm.ported, pm.register_components) {
                register(&self.components)?;
            }
        }
        // Create printer objects
        crate::pins::add_printer_objects(self, &config)?;
        crate::mcu::add_printer_objects(self, &config)?;
        crate::extras::probe::probe_list_get_list(self)?;
        for section_config in config.get_prefix_sections("") {
            self.load_object(&config, section_config.get_name(), false)?;
        }
        // Kalico on-by-default extras
        for section in ["force_move", "respond", "exclude_object", "telemetry"] {
            self.load_object(&config, section, false)?;
        }
        if self.start_arg("debuginput").is_some() {
            self.load_object(&config, "testing", false)?;
        }
        crate::toolhead::add_printer_objects(self, &config)?;
        // Validate that there are no undefined parameters in the config file
        let error_on_unused = crate::extras::danger_options::get_danger_options().error_on_unused_config_options;
        let names: Vec<String> = self.objects.borrow().keys().cloned().collect();
        pobj.pconfig.borrow_mut().check_unused_options(&config, error_on_unused, &names)?;
        Ok(())
    }

    fn connect(self: &Rc<Self>, _eventtime: f64) {
        let res = (|| -> KResult<bool> {
            self.read_config()?;
            self.send_event("klippy:mcu_identify", &[])?;
            let handlers = self.handlers("klippy:connect");
            for cb in handlers {
                if *self.state_message.borrow() != MESSAGE_STARTUP {
                    return Ok(false);
                }
                cb(&[])?;
            }
            Ok(true)
        })();
        match res {
            Ok(true) => {}
            Ok(false) => return,
            Err(e) if matches!(e.base(), KError::Config(_) | KError::Pins(_)) => {
                logging::exception_err("Config error", &e);
                self.set_state(&format!("{}\n{}", e, MESSAGE_RESTART));
                return;
            }
            Err(e) if matches!(e.base(), KError::Protocol(_) | KError::Enumeration { .. } | KError::Mcu(_)) => {
                let msg = if e.is_protocol_error() { "Protocol error" } else { "MCU error during connect" };
                logging::exception_err(msg, &e);
                self.set_state(msg);
                let mut details = IndexMap::new();
                details.insert("error".to_string(), PyVal::Str(e.to_string()));
                let details = PyVal::Dict(details);
                let _ = self.send_event("klippy:notify_mcu_error", &[&msg.to_string(), &details]);
                crate::util::dump_mcu_build();
                return;
            }
            Err(e) => {
                logging::exception_err("Unhandled exception during connect", &e);
                self.set_state(&format!("Internal error during connect: {}\n{}", e, MESSAGE_RESTART));
                return;
            }
        }
        self.set_state(MESSAGE_READY);
        let res = (|| -> KResult<()> {
            let _guard = self.reactor.assert_no_pause();
            for cb in self.handlers("klippy:ready") {
                if *self.state_message.borrow() != MESSAGE_READY {
                    return Ok(());
                }
                cb(&[])?;
            }
            Ok(())
        })();
        if let Err(e) = res {
            logging::exception_err("Unhandled exception during ready callback", &e);
            self.invoke_shutdown(&format!("Internal error during ready callback: {}", e), None);
        }
    }

    pub fn run(self: &Rc<Self>) -> Option<String> {
        let systime = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.);
        let monotime = self.reactor.monotonic();
        logging::info(&format!(
            "Start printer at {} ({:.1} {:.1})",
            crate::util::asctime_local(systime),
            systime,
            monotime
        ));
        // Enter main reactor loop
        self.reactor.run();
        // Check restart flags
        let run_result = self.run_result.borrow().clone();
        let res = (|| -> KResult<()> {
            if run_result.as_deref() == Some("firmware_restart") {
                self.send_event("klippy:firmware_restart", &[])?;
            }
            self.send_event("klippy:disconnect", &[])
        })();
        if let Err(e) = res {
            logging::exception_err("Unhandled exception during post run", &e);
        }
        run_result
    }

    /// A reactor callback raised: python leaves reactor.run() ("Unhandled
    /// exception during run") and shuts down from a new callback. Ported
    /// callbacks that can fail report through here.
    pub fn unhandled_exception(self: &Rc<Self>, e: &KError) {
        let msg = "Unhandled exception during run";
        logging::exception_err(msg, e);
        let p = self.clone();
        self.reactor.register_callback(move |_| p.invoke_shutdown(msg, None), NOW);
    }

    pub fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool) {
        if log {
            if let Some(info) = info {
                logging::info(info);
            }
        }
        if let Some(bg) = &self.bglogger {
            bg.set_rollover_info(name, info);
        }
    }

    pub fn invoke_shutdown(&self, msg: &str, details: Option<PyVal>) {
        if self.in_shutdown_state.get() {
            return;
        }
        let details = details.unwrap_or_else(PyVal::dict);
        logging::error(&format!("Transition to shutdown state: {}", msg));
        self.in_shutdown_state.set(true);
        self.set_state(msg);
        let _guard = self.reactor.assert_no_pause();
        for cb in self.handlers("klippy:shutdown") {
            if cb(&[]).is_err() {
                logging::exception("Exception during shutdown handler");
            }
        }
        for cb in self.handlers("klippy:analyze_shutdown") {
            if cb(&[&msg.to_string(), &details]).is_err() {
                logging::exception("Exception in analyze_shutdown handler");
            }
        }
    }

    pub fn invoke_async_shutdown(self: &Rc<Self>, msg: &str, details: Option<PyVal>) {
        let (p, msg) = (self.clone(), msg.to_string());
        self.reactor.register_async_callback_local(move |_| p.invoke_shutdown(&msg, details), NOW);
    }

    pub fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.event_handlers.borrow_mut().entry(event.to_string()).or_default().push(callback);
    }

    fn handlers(&self, event: &str) -> Vec<EventCallback> {
        self.event_handlers.borrow().get(event).cloned().unwrap_or_default()
    }

    /// send_event(event, *params); python's list of results is never used
    pub fn send_event(&self, event: &str, params: &[&dyn Any]) -> KResult<()> {
        for cb in self.handlers(event) {
            cb(params)?;
        }
        Ok(())
    }

    pub fn request_exit(&self, result: &str) {
        if self.run_result.borrow().is_none() {
            *self.run_result.borrow_mut() = Some(result.to_string());
        }
        self.reactor.end();
    }

    /// wait_while(condition_cb, error_on_cancel, interval)
    pub fn wait_while(&self, mut condition_cb: impl FnMut(f64) -> KResult<bool>, error_on_cancel: bool, interval: f64) -> KResult<()> {
        let gcode = self.lookup::<crate::gcode::GCodeDispatch>("gcode")?;
        let counter = gcode.get_interrupt_counter();
        let mut eventtime = self.reactor.monotonic();
        while condition_cb(eventtime)? {
            if self.is_shutdown() || counter != gcode.get_interrupt_counter() {
                if error_on_cancel {
                    return Err(KError::WaitInterruption("Command interrupted".into()));
                }
                return Ok(());
            }
            eventtime = self.reactor.pause(eventtime + interval)?;
        }
        Ok(())
    }

    /// Kalico's _load_modules for python modules: klippy/extras files
    /// kaliru has no built-in for, then klippy/plugins (a name in both is an
    /// error unless allow_plugin_override)
    #[cfg(feature = "plugins")]
    fn load_py_modules(&self, allow_overrides: bool) -> KResult<()> {
        use crate::plugins::{list_modules, Package};
        let mut mods = IndexMap::new();
        let builtin = |n: &str| crate::extras::MODULES.iter().any(|m| m.name == n);
        for name in list_modules(&Package::Extras) {
            if !builtin(&name) {
                mods.insert(name, Package::Extras);
            }
        }
        for name in list_modules(&Package::Plugins) {
            if (builtin(&name) || mods.contains_key(&name)) && !allow_overrides {
                return Err(KError::config(format!("Module '{}' found in both extras and plugins!", name)));
            }
            mods.insert(name, Package::Plugins);
        }
        *self.py_modules.borrow_mut() = mods;
        Ok(())
    }

    /// printer.objects[name] = obj (python plugins: a dict assignment keeps
    /// the position of an existing name)
    #[cfg(feature = "plugins")]
    pub fn objects_set(&self, name: &str, obj: PObj) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }

    /// printer.objects.pop(name)
    #[cfg(feature = "plugins")]
    pub fn objects_remove(&self, name: &str) -> Option<PObj> {
        self.objects.borrow_mut().shift_remove(name)
    }

    /// list(printer.objects)
    #[cfg(feature = "plugins")]
    pub fn object_names(&self) -> Vec<String> {
        self.objects.borrow().keys().cloned().collect()
    }

    /// not in python (garbage collected there): drop the objects and
    /// handlers that hold Rc<Printer>, after the reactor was finalized
    pub fn teardown(&self) {
        #[cfg(feature = "plugins")]
        crate::plugins::teardown();
        let objects: Vec<PObj> = self.objects.borrow().values().cloned().collect();
        for o in objects {
            o.teardown();
        }
        self.objects.borrow_mut().clear();
        self.event_handlers.borrow_mut().clear();
        *self.pconfig.borrow_mut() = None;
    }
}

/// Rc<dyn PrinterObject> -> Rc<T>
pub fn downcast_rc<T: 'static>(obj: PObj) -> Option<Rc<T>> {
    if obj.as_any().is::<T>() {
        let raw = Rc::into_raw(obj) as *const T;
        Some(unsafe { Rc::from_raw(raw) })
    } else {
        None
    }
}

// ----------------------------------------------------------------------
// Startup
// ----------------------------------------------------------------------

const USAGE: &str = "Usage: klippy [options] <config file>";

fn opts_error(msg: &str) -> ! {
    eprintln!("{}\n\nklippy: error: {}", USAGE, msg);
    std::process::exit(2);
}

fn print_help() -> ! {
    println!(
        "{}

Options:
  -h, --help            show this help message and exit
  -i DEBUGINPUT, --debuginput=DEBUGINPUT
                        read commands from file instead of from tty port
  -I INPUTTTY, --input-tty=INPUTTTY
                        input tty name (default is /tmp/printer)
  -a APISERVER_FILE, --api-server=APISERVER_FILE
                        api server unix domain socket filename
  --api-server-user=APISERVER_USER
                        api server unix domain socket user
  --api-server-group=APISERVER_GROUP
                        api server unix domain socket group
  --api-server-file-mode=APISERVER_FILE_MODE
                        api server unix domain socket file mode
  -l LOGFILE, --logfile=LOGFILE
                        write log to file instead of stderr
  --rotate-log-at-restart
                        rotate the log file at every restart
  -v                    enable debug messages
  -o DEBUGOUTPUT, --debugoutput=DEBUGOUTPUT
                        write output to file instead of to serial port
  -d DICTIONARY, --dictionary=DICTIONARY
                        file to read for mcu protocol dictionary
  --import-test         perform an import module test",
        USAGE
    );
    std::process::exit(0);
}

#[derive(Default)]
struct Options {
    debuginput: Option<String>,
    inputtty: Option<String>,
    apiserver_file: Option<String>,
    apiserver_user: Option<String>,
    apiserver_group: Option<String>,
    apiserver_file_mode: Option<String>,
    logfile: Option<String>,
    rotate_log_at_restart: bool,
    verbose: bool,
    debugoutput: Option<String>,
    dictionary: Option<IndexMap<String, String>>,
    import_test: bool,
}

/// optparse: "-x value", "-xvalue", "--long value", "--long=value", "--"
fn parse_args(argv: &[String]) -> (Options, Vec<String>) {
    let mut o = Options::default();
    let mut args = vec![];
    let long_opts = [
        "--debuginput", "--input-tty", "--api-server", "--api-server-user", "--api-server-group",
        "--api-server-file-mode", "--logfile", "--rotate-log-at-restart", "--debugoutput", "--dictionary",
        "--import-test", "--help",
    ];
    let mut i = 0;
    let set = |o: &mut Options, key: &str, value: String| match key {
        "i" | "--debuginput" => o.debuginput = Some(value),
        "I" | "--input-tty" => o.inputtty = Some(value),
        "a" | "--api-server" => o.apiserver_file = Some(value),
        "--api-server-user" => o.apiserver_user = Some(value),
        "--api-server-group" => o.apiserver_group = Some(value),
        "--api-server-file-mode" => o.apiserver_file_mode = Some(value),
        "l" | "--logfile" => o.logfile = Some(value),
        "o" | "--debugoutput" => o.debugoutput = Some(value),
        "d" | "--dictionary" => {
            // arg_dictionary
            let (key, fname) = match value.split_once('=') {
                Some((mcu, f)) => (format!("dictionary_{}", mcu), f.to_string()),
                None => ("dictionary".to_string(), value),
            };
            o.dictionary.get_or_insert_with(IndexMap::new).insert(key, fname);
        }
        _ => unreachable!(),
    };
    while i < argv.len() {
        let a = &argv[i];
        i += 1;
        if a == "--" {
            args.extend(argv[i..].iter().cloned());
            break;
        }
        if let Some(long) = a.strip_prefix("--") {
            let (name, inline) = match long.split_once('=') {
                Some((n, v)) => (format!("--{}", n), Some(v.to_string())),
                None => (a.clone(), None),
            };
            // optparse accepts unambiguous prefixes of long options
            let matches: Vec<&&str> = long_opts.iter().filter(|l| l.starts_with(name.as_str())).collect();
            let name = match matches.as_slice() {
                [one] => one.to_string(),
                [] => opts_error(&format!("no such option: {}", name)),
                many if many.iter().any(|m| **m == name) => name,
                many => opts_error(&format!(
                    "ambiguous option: {} ({}?)",
                    name,
                    many.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(", ")
                )),
            };
            match name.as_str() {
                "--help" => print_help(),
                "--rotate-log-at-restart" | "--import-test" => {
                    if inline.is_some() {
                        opts_error(&format!("{} option does not take a value", name));
                    }
                    if name == "--import-test" {
                        o.import_test = true;
                    } else {
                        o.rotate_log_at_restart = true;
                    }
                }
                _ => {
                    let value = match inline {
                        Some(v) => v,
                        None => {
                            if i >= argv.len() {
                                opts_error(&format!("{} option requires 1 argument", name));
                            }
                            i += 1;
                            argv[i - 1].clone()
                        }
                    };
                    set(&mut o, &name, value);
                }
            }
            continue;
        }
        if a.len() > 1 && a.starts_with('-') {
            let chars: Vec<char> = a[1..].chars().collect();
            let mut j = 0;
            while j < chars.len() {
                let c = chars[j];
                j += 1;
                match c {
                    'h' => print_help(),
                    'v' => o.verbose = true,
                    'i' | 'I' | 'a' | 'l' | 'o' | 'd' => {
                        let value = if j < chars.len() {
                            let v: String = chars[j..].iter().collect();
                            j = chars.len();
                            v
                        } else {
                            if i >= argv.len() {
                                opts_error(&format!("-{} option requires 1 argument", c));
                            }
                            i += 1;
                            argv[i - 1].clone()
                        };
                        set(&mut o, &c.to_string(), value);
                    }
                    _ => opts_error(&format!("no such option: -{}", c)),
                }
            }
            continue;
        }
        args.push(a.clone());
    }
    (o, args)
}

pub fn main() {
    crate::util::fix_sigint();
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let (options, args) = parse_args(&argv);
    if options.import_test {
        // python imports every extras / kinematics module; all of kaliru is
        // compiled in
        std::process::exit(0);
    }
    if args.len() != 1 {
        opts_error("Incorrect number of arguments");
    }
    let mut start_args: IndexMap<String, PyVal> = IndexMap::new();
    start_args.insert("config_file".into(), PyVal::Str(args[0].clone()));
    start_args.insert("apiserver_file".into(), options.apiserver_file.clone().into());
    start_args.insert("apiserver_user".into(), options.apiserver_user.clone().into());
    start_args.insert("apiserver_group".into(), options.apiserver_group.clone().into());
    start_args.insert("apiserver_file_mode".into(), options.apiserver_file_mode.clone().into());
    start_args.insert("start_reason".into(), PyVal::Str("startup".into()));

    let debuglevel = if options.verbose { logging::DEBUG } else { logging::INFO };
    // the debug input file stays open for the process lifetime
    let mut _debuginput: Option<std::fs::File> = None;
    if let Some(di) = &options.debuginput {
        start_args.insert("debuginput".into(), PyVal::Str(di.clone()));
        let f = std::fs::File::open(di).unwrap_or_else(|e| {
            eprintln!("{}", crate::pyfmt::py_oserror(&e, di));
            std::process::exit(1);
        });
        use std::os::fd::AsRawFd;
        start_args.insert("gcode_fd".into(), PyVal::Int(f.as_raw_fd() as i64));
        _debuginput = Some(f);
    } else {
        let tty = options.inputtty.clone().unwrap_or_else(|| "/tmp/printer".into());
        match crate::util::create_pty(&tty) {
            Ok(fd) => {
                start_args.insert("gcode_fd".into(), PyVal::Int(fd as i64));
            }
            Err(e) => {
                eprintln!("{}", e);
                std::process::exit(1);
            }
        }
    }
    if let Some(out) = &options.debugoutput {
        start_args.insert("debugoutput".into(), PyVal::Str(out.clone()));
        // start_args.update(options.dictionary) (TypeError in python if no -d)
        for (k, v) in options.dictionary.clone().unwrap_or_default() {
            start_args.insert(k, PyVal::Str(v));
        }
    }
    let mut bglogger: Option<Rc<QueueListener>> = None;
    if let Some(logfile) = &options.logfile {
        start_args.insert("log_file".into(), PyVal::Str(logfile.clone()));
        match queuelogger::setup_bg_logging(logfile, debuglevel, options.rotate_log_at_restart) {
            Ok(ql) => bglogger = Some(Rc::new(ql)),
            Err(e) => {
                eprintln!("{}", crate::pyfmt::py_oserror(&e, logfile));
                std::process::exit(1);
            }
        }
        if options.rotate_log_at_restart {
            bglogger.as_ref().unwrap().do_rollover();
        }
    } else {
        logging::set_level(debuglevel);
    }
    logging::info("=======================");
    logging::info("Starting Klippy...");
    let git_info = crate::util::get_git_version();
    let git_vers = git_info.version.clone();

    let mut extra_git_desc = String::new();
    extra_git_desc.push_str(&format!("\nBranch: {}", git_info.branch));
    extra_git_desc.push_str(&format!("\nRemote: {}", git_info.remote));
    extra_git_desc.push_str(&format!("\nTracked URL: {}", git_info.url));
    start_args.insert("software_version".into(), PyVal::Str(git_vers.clone()));
    start_args.insert("git_branch".into(), PyVal::Str(git_info.branch.clone()));
    start_args.insert("git_remote".into(), PyVal::Str(git_info.remote.clone()));
    start_args.insert("cpu_info".into(), PyVal::Str(crate::util::get_cpu_info()));
    let mut versions = String::new();
    if bglogger.is_some() {
        // ponytail: python prints sys.argv and sys.version here
        let argv_repr = PyVal::List(std::env::args().map(PyVal::Str).collect()).repr();
        versions = [
            format!("Args: {}", argv_repr),
            format!("App Name: {}", APP_NAME),
            format!("Git version: {}{}", PyVal::Str(git_vers.clone()).repr(), extra_git_desc),
            format!("CPU: {}", start_args["cpu_info"].str()),
            format!("Python: {}", PyVal::Str(format!("kaliru {}", env!("CARGO_PKG_VERSION"))).repr()),
        ]
        .join("\n");
        logging::info(&versions);
    } else if options.debugoutput.is_none() {
        logging::warning("No log file specified! Severe timing issues may result!");
    }

    // Start Printer() class
    let res;
    loop {
        if let Some(bg) = &bglogger {
            bg.clear_rollover_info();
            bg.set_rollover_info("versions", Some(&versions));
        }
        let main_reactor = Reactor::new(true);
        let printer = match Printer::new(main_reactor.clone(), bglogger.clone(), start_args.clone()) {
            Ok(p) => p,
            Err(e) => {
                // python: an exception in Printer() ends the process
                logging::error(&e.to_string());
                std::process::exit(1);
            }
        };
        let r = printer.run();
        if matches!(r.as_deref(), Some("exit") | Some("error_exit")) {
            res = r;
            break;
        }
        std::thread::sleep(std::time::Duration::from_secs(1));
        main_reactor.finalize();
        printer.teardown();
        logging::info("Restarting printer");
        start_args.insert("start_reason".into(), r.map(PyVal::Str).unwrap_or(PyVal::None));
        if options.rotate_log_at_restart {
            if let Some(bg) = &bglogger {
                bg.do_rollover();
            }
        }
    }

    if let Some(bg) = &bglogger {
        bg.stop();
    }

    if res.as_deref() == Some("error_exit") {
        std::process::exit(255);
    }
}
