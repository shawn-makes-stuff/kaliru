// Python plugins (docs/PLUGINS.md): Kalico modules kaliru does not have built
// in (third-party files in klippy/extras, klippy/plugins) run on the host's
// CPython, loaded the first time such a module is needed. Without one,
// nothing here runs and libpython is never mapped.
//
// klippy directory: $KALIRU_KLIPPY_DIR, else `klippy` next to the binary
// (Kalico's layout: klippy/extras, klippy/plugins).
mod callable;
mod gil;
pub mod capi;
pub mod convert;
mod greenlets;
mod impls;
mod mirror_ids;
pub mod native;
pub mod objects;
pub mod py;
mod sources;

use crate::plugins::gil::Gil;
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use py::PyRef;
use std::any::Any;
use std::cell::Cell;
use std::path::PathBuf;
use std::rc::Rc;

static STARTED: Gil<Cell<bool>> = Gil::new(|| Cell::new(false));

/// Whether the interpreter runs (then the reactor thread holds the GIL)
pub fn started() -> bool {
    STARTED.with(|s| s.get())
}

/// Run `f` without the GIL (the reactor's poll) when python runs
pub fn allow_threads<R>(f: impl FnOnce() -> R) -> R {
    if !started() {
        return f();
    }
    let ts = unsafe { capi::PyEval_SaveThread() };
    let r = f();
    unsafe { capi::PyEval_RestoreThread(ts) };
    r
}

pub fn klippy_dir() -> PathBuf {
    if let Some(d) = std::env::var_os("KALIRU_KLIPPY_DIR") {
        return PathBuf::from(d);
    }
    let exe = std::env::current_exe().unwrap_or_default();
    exe.parent().map(|p| p.join("klippy")).unwrap_or_default()
}

/// Where a python module was found (pkgutil.iter_modules of the directory)
#[derive(Clone, Debug, PartialEq)]
pub enum Package {
    Extras,
    Plugins,
}

impl Package {
    fn dir(&self) -> &'static str {
        match self {
            Package::Extras => "extras",
            Package::Plugins => "plugins",
        }
    }
}

/// Module names in klippy/<package> as pkgutil.iter_modules lists them:
/// `*.py` files and directories with an `__init__.py`
pub fn list_modules(package: &Package) -> Vec<String> {
    let dir = klippy_dir().join(package.dir());
    let Ok(rd) = std::fs::read_dir(&dir) else { return vec![] };
    let mut out = vec![];
    for e in rd.flatten() {
        let path = e.path();
        let name = e.file_name().to_string_lossy().into_owned();
        if path.is_dir() {
            if path.join("__init__.py").is_file() && !name.contains('.') {
                out.push(name);
            }
        } else if let Some(stem) = name.strip_suffix(".py") {
            if stem != "__init__" && !stem.contains('.') {
                out.push(stem.to_string());
            }
        }
    }
    out.sort();
    out
}

/// Start the interpreter (once)
pub fn ensure_started() -> KResult<()> {
    if started() {
        return Ok(());
    }
    capi::load().map_err(|e| KError::config(format!("Python plugins need CPython: {}", e)))?;
    unsafe { capi::Py_InitializeEx(0) };
    native::create_module()?;
    // the prelude as module `_k`
    let m = unsafe { PyRef::owned_or_err(capi::PyImport_AddModuleRef(c"_k".as_ptr()))? };
    let src = std::ffi::CString::new(include_str!("prelude.py")).unwrap();
    unsafe {
        let code = PyRef::owned_or_err(capi::Py_CompileString(src.as_ptr(), c"<kaliru prelude>".as_ptr(), 257))?;
        let globals = capi::PyModule_GetDict(m.as_ptr());
        // what exec() adds: imports from inside the module look it up
        let builtins = py::import("builtins")?;
        capi::PyDict_SetItemString(globals, c"__builtins__".as_ptr(), builtins.as_ptr());
        PyRef::owned_or_err(capi::PyEval_EvalCode(code.as_ptr(), globals, globals))?;
    }
    let dir = py::py_str(&klippy_dir().to_string_lossy());
    let level = py::py_int(crate::logging::get_level() as i64);
    m.call_method("install", &[&dir, &native::mirror_modules(), &level])?;
    let mut reg = native::Registry::default();
    impls::register(&mut reg);
    native::install_handlers(reg);
    greenlets::install();
    STARTED.with(|s| s.set(true));
    Ok(())
}

/// A printer object a python module created
pub struct PyPrinterObject {
    pub obj: PyRef,
}

impl PyPrinterObject {
    fn call(&self, name: &str, eventtime: f64) -> KResult<Option<PyRef>> {
        if !self.obj.hasattr(name)? {
            return Ok(None);
        }
        self.obj.call_method(name, &[&py::py_float(eventtime)]).map(Some)
    }
}

impl PrinterObject for PyPrinterObject {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        self.obj.hasattr("get_status").unwrap_or(false)
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        self.try_get_status(eventtime).ok().flatten()
    }
    fn try_get_status(&self, eventtime: f64) -> KResult<Option<PyVal>> {
        match self.call("get_status", eventtime)? {
            Some(v) => Ok(Some(py::to_pyval(&v)?)),
            None => Ok(None),
        }
    }
    fn has_stats(&self) -> bool {
        self.obj.hasattr("stats").unwrap_or(false)
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        let v = self.call("stats", eventtime).ok()??;
        let t = py::to_pyval(&v).ok()?;
        match t {
            PyVal::Tuple(t) | PyVal::List(t) if t.len() == 2 => {
                let busy = !matches!(t[0], PyVal::Bool(false) | PyVal::None | PyVal::Int(0));
                Some((busy, t[1].str()))
            }
            _ => None,
        }
    }
    fn call_method(&self, name: &str, args: &[PyVal]) -> Option<KResult<PyVal>> {
        let res = (|| {
            let args: Vec<PyRef> = args.iter().map(py::from_pyval).collect();
            let refs: Vec<&PyRef> = args.iter().collect();
            py::to_pyval(&self.obj.call_method(name, &refs)?)
        })();
        Some(res)
    }
    fn get_temp(&self, eventtime: f64) -> Option<(f64, f64)> {
        let v = self.call("get_temp", eventtime).ok()??;
        match py::to_pyval(&v).ok()? {
            PyVal::Tuple(t) | PyVal::List(t) if t.len() == 2 => Some((t[0].as_f64()?, t[1].as_f64()?)),
            _ => None,
        }
    }
}

/// A printer object as python sees it
pub fn pobj_to_py(obj: &PObj) -> KResult<PyRef> {
    if let Some(p) = obj.as_any().downcast_ref::<PyPrinterObject>() {
        return Ok(p.obj.clone());
    }
    if obj.is_none() {
        return Ok(py::py_none());
    }
    objects::wrap_pobj(obj.clone())
}

/// A python object as a printer object (add_object and the like)
pub fn py_to_pobj(o: &PyRef) -> PObj {
    if let Some(p) = objects::bound_pobj(o) {
        return p;
    }
    Rc::new(PyPrinterObject { obj: o.clone() })
}

/// printer.load_object() of a python module: import it, call its
/// load_config / load_config_prefix
pub fn load_config(config: &ConfigWrapper, section: &str, package: &Package) -> KResult<Option<PObj>> {
    ensure_started()?;
    let name = section.split_whitespace().next().unwrap_or("");
    let modname = format!("klippy.{}.{}", package.dir(), name);
    let module = py::import(&modname).map_err(|e| {
        crate::logging::exception_err(&format!("Failed to load module '{}'.", name), &e);
        e
    })?;
    let is_prefix = name != section;
    let func = if is_prefix { "load_config_prefix" } else { "load_config" };
    if !module.hasattr(func)? {
        return Ok(None);
    }
    let cfg = objects::wrap(Rc::new(config.getsection(section)))?;
    let obj = module.call_method(func, &[&cfg])?;
    if obj.is_none() {
        return Ok(Some(crate::printer::none_object()));
    }
    Ok(Some(py_to_pobj(&obj)))
}

pub use impls::set_printer;

/// Kaliru's method `qualname` of `obj` about to run: if a python plugin
/// replaced it (on the object, its class, or a subclass), call the
/// replacement instead - what python's dynamic dispatch does in Kalico. The
/// check sits in kaliru's method, so every kaliru caller honours it; a
/// replacement calling the original goes through `_k.call` and is let
/// through (native.rs BYPASS).
pub fn dispatch<T: 'static>(obj: &T, qualname: &str, args: &[&dyn Any]) -> Option<KResult<PyVal>> {
    if !started() || !native::any_patched() {
        return None;
    }
    let addr = obj as *const T as *const () as usize;
    if native::bypassing(addr, qualname) {
        return None;
    }
    let py = objects::existing_addr(addr)?;
    let name = qualname.rsplit('.').next().unwrap_or(qualname);
    let res = (|| {
        let f = py::lookup("_k", "overridden")?.call(&[&py, &py::py_str(name)])?;
        if f.is_none() {
            return Ok(None);
        }
        let py_args = args.iter().map(|a| impls::any_to_py(*a)).collect::<KResult<Vec<_>>>()?;
        let refs: Vec<&PyRef> = py_args.iter().collect();
        py::to_pyval(&f.call(&refs)?).map(Some)
    })();
    match res {
        Ok(None) => None,
        Ok(Some(v)) => Some(Ok(v)),
        Err(e) => Some(Err(e)),
    }
}

/// The end of a printer instance: drop the bindings to its objects
pub fn teardown() {
    if started() {
        objects::teardown();
    }
}
