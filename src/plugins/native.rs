// The native module `_kaliru` the mirror modules call (prelude.py binds it as
// _k.call / _k.fcall / _k.new ...), and the registry of what each mirrored
// method, constructor and attribute does in kaliru.
//
// Handlers are registered by Kalico qualified name ("klippy.gcode.
// GCodeDispatch.register_command"); mirror_ids.rs gives each name the ID the
// generated python passes. An ID without a handler raises
// NotImplementedError naming it.
use crate::plugins::gil::Gil;
use super::capi::*;
use super::mirror_ids::{IDS, MODULES};
use super::objects;
use super::py::*;
use crate::error::{KError, KResult};
use std::any::Any;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::ffi::c_char;
use std::rc::Rc;

/// The arguments of a mirrored call: the python object (methods) and the
/// arguments in the order of Kalico's signature (defaults filled in)
pub struct Args {
    pub slf: Option<PyRef>,
    pub args: Vec<PyRef>,
}

impl Args {
    pub fn arg(&self, i: usize) -> &PyRef {
        &self.args[i]
    }

    pub fn this<T: 'static>(&self) -> KResult<Rc<T>> {
        objects::unwrap_as::<T>(self.slf.as_ref().expect("method without self"))
    }

    pub fn f64(&self, i: usize) -> KResult<f64> {
        self.args[i].as_f64()
    }

    pub fn opt_f64(&self, i: usize) -> KResult<Option<f64>> {
        if self.args[i].is_none() { Ok(None) } else { self.f64(i).map(Some) }
    }

    pub fn string(&self, i: usize) -> KResult<String> {
        self.args[i].as_string()
    }

    pub fn opt_string(&self, i: usize) -> KResult<Option<String>> {
        if self.args[i].is_none() { Ok(None) } else { self.string(i).map(Some) }
    }

    pub fn bool(&self, i: usize) -> KResult<bool> {
        self.args[i].is_true()
    }
}

pub type Method = fn(&Args) -> KResult<PyRef>;
pub type Ctor = fn(&Args) -> KResult<Rc<dyn Any>>;
pub type Getter = fn(&PyRef) -> KResult<PyRef>;
pub type Setter = fn(&PyRef, &PyRef) -> KResult<()>;

/// A list attribute kept in kaliru: read it whole, write it whole
/// (prelude.py ListView); the field's qualified name picks the list
#[derive(Clone, Copy)]
pub struct ListField {
    pub get: fn(&PyRef, &str) -> KResult<Vec<PyRef>>,
    pub set: fn(&PyRef, &str, Vec<PyRef>) -> KResult<()>,
}

/// A dict attribute kept in kaliru (prelude.py DictView)
#[derive(Clone, Copy)]
pub struct DictField {
    pub get: fn(&PyRef, &str, &PyRef) -> KResult<PyRef>,
    pub set: fn(&PyRef, &str, &PyRef, &PyRef) -> KResult<()>,
    pub del: fn(&PyRef, &str, &PyRef) -> KResult<()>,
    pub keys: fn(&PyRef, &str) -> KResult<Vec<PyRef>>,
}

#[derive(Clone, Copy)]
pub enum Handler {
    Method(Method),
    Ctor(Ctor),
    Attr(Getter, Option<Setter>),
    List(&'static str, ListField),
    Dict(&'static str, DictField),
}

/// Collects handlers by qualified name (src/plugins/impls/*), and the
/// layer's own helpers by name (`_kaliru.ext`)
#[derive(Default)]
pub struct Registry(HashMap<&'static str, Handler>, HashMap<&'static str, Method>);

impl Registry {
    pub fn method(&mut self, name: &'static str, f: Method) {
        self.0.insert(name, Handler::Method(f));
    }
    pub fn ctor(&mut self, name: &'static str, f: Ctor) {
        self.0.insert(name, Handler::Ctor(f));
    }
    pub fn attr(&mut self, name: &'static str, get: Getter, set: Option<Setter>) {
        self.0.insert(name, Handler::Attr(get, set));
    }
    pub fn ext(&mut self, name: &'static str, f: Method) {
        self.1.insert(name, f);
    }
    pub fn list(&mut self, name: &'static str, f: ListField) {
        self.0.insert(name, Handler::List(name, f));
    }
    pub fn dict(&mut self, name: &'static str, f: DictField) {
        self.0.insert(name, Handler::Dict(name, f));
    }
}

static HANDLERS: Gil<RefCell<Vec<Option<Handler>>>> = Gil::new(|| RefCell::new(Vec::new()));
// (class qualname or "@<object handle>", name) a plugin replaced
static PATCHED: Gil<RefCell<HashSet<(String, String)>>> = Gil::new(|| RefCell::new(HashSet::new()));
static EXT: Gil<RefCell<HashMap<&'static str, Method>>> = Gil::new(|| RefCell::new(HashMap::new()));
// (kaliru object address, method ID, greenlet) of the mirrored calls running:
// a plugin's replacement calling the original reaches kaliru's method, which
// must not dispatch to the replacement again
static BYPASS: Gil<RefCell<Vec<(usize, usize, usize)>>> = Gil::new(|| RefCell::new(Vec::new()));

/// Resolve the registry against the generated IDs; names that are not
/// Kalico's are a bug in the impls
pub fn install_handlers(reg: Registry) {
    let mut by_id = vec![None; IDS.len()];
    let mut unknown: Vec<&str> = vec![];
    let index: HashMap<&str, usize> = IDS.iter().enumerate().map(|(i, n)| (*n, i)).collect();
    // get_status / stats / get_temp: the PrinterObject methods unless a
    // class has its own handler
    for (i, name) in IDS.iter().enumerate() {
        if let Some(f) = super::impls::generic_methods(name) {
            by_id[i] = Some(Handler::Method(f));
        }
    }
    for (name, h) in reg.0.iter().map(|(n, h)| (*n, *h)) {
        match index.get(name) {
            Some(&i) => by_id[i] = Some(h),
            None => unknown.push(name),
        }
    }
    assert!(unknown.is_empty(), "plugin handlers for names Kalico does not have: {:?}", unknown);
    HANDLERS.with(|h| *h.borrow_mut() = by_id);
    EXT.with(|e| *e.borrow_mut() = reg.1);
}

fn handler(id: usize) -> KResult<Handler> {
    HANDLERS
        .with(|h| h.borrow().get(id).copied().flatten())
        .ok_or_else(|| KError::Py("NotImplementedError".into(), format!("kaliru: {} is not mirrored yet", IDS.get(id).unwrap_or(&"?"))))
}

/// Whether any plugin replaced any mirrored method (dispatch's fast path)
pub fn any_patched() -> bool {
    PATCHED.with(|p| !p.borrow().is_empty())
}

fn greenlet_key() -> usize {
    crate::greenlet::getcurrent().map(|g| std::rc::Rc::as_ptr(&g) as usize).unwrap_or(0)
}

/// Whether this method of this object is being called from python as the
/// original (a replacement calling it)
pub fn bypassing(addr: usize, qualname: &str) -> bool {
    let Some(id) = IDS.iter().position(|n| *n == qualname) else { return false };
    let g = greenlet_key();
    BYPASS.with(|b| b.borrow().iter().any(|e| *e == (addr, id, g)))
}

// ------------------------------------------------------------------ module

unsafe fn args_vec(args: *const *mut PyObject, nargs: Py_ssize_t, skip: usize) -> Vec<PyRef> {
    let n = (nargs as usize) & !(1usize << (usize::BITS - 1)); // PY_VECTORCALL_ARGUMENTS_OFFSET
    (skip..n).map(|i| PyRef::borrowed(*args.add(i))).collect()
}

fn result(r: KResult<PyRef>) -> *mut PyObject {
    match r {
        Ok(v) => v.into_ptr(),
        Err(e) => {
            raise(&e);
            std::ptr::null_mut()
        }
    }
}

macro_rules! native {
    ($name:ident, |$a:ident| $body:expr) => {
        unsafe extern "C" fn $name(
            _m: *mut PyObject,
            args: *const *mut PyObject,
            nargs: Py_ssize_t,
            _kw: *mut PyObject,
        ) -> *mut PyObject {
            let $a = args_vec(args, nargs, 0);
            result((|| -> KResult<PyRef> { $body })())
        }
    };
}

fn id_of(o: &PyRef) -> KResult<usize> {
    Ok(o.as_i64()? as usize)
}

// call(obj, id, *args)
native!(k_call, |a| {
    let id = id_of(&a[1])?;
    let h = handler(id)?;
    let Handler::Method(f) = h else { return Err(KError::Other("kaliru: not a method".into())) };
    let entry = objects::unwrap(&a[0]).ok().map(|r| (std::rc::Rc::as_ptr(&r) as *const () as usize, id, greenlet_key()));
    if let Some(e) = entry {
        BYPASS.with(|b| b.borrow_mut().push(e));
    }
    let res = f(&Args { slf: Some(a[0].clone()), args: a[2..].to_vec() });
    if let Some(e) = entry {
        // (by identity: the call may have paused while others came and went)
        BYPASS.with(|b| {
            let mut b = b.borrow_mut();
            if let Some(pos) = b.iter().rposition(|x| *x == e) {
                b.remove(pos);
            }
        });
    }
    res
});

// fcall(id, *args)
native!(k_fcall, |a| {
    let Handler::Method(f) = handler(id_of(&a[0])?)? else { return Err(KError::Other("kaliru: not a function".into())) };
    f(&Args { slf: None, args: a[1..].to_vec() })
});

// new(obj, id, *args): obj.__init__
native!(k_new, |a| {
    let Handler::Ctor(f) = handler(id_of(&a[1])?)? else { return Err(KError::Other("kaliru: not a constructor".into())) };
    let rust = f(&Args { slf: Some(a[0].clone()), args: a[2..].to_vec() })?;
    objects::bind(&a[0], rust)?;
    Ok(py_none())
});

// get(obj, id)
native!(k_get, |a| {
    match handler(id_of(&a[1])?)? {
        Handler::Attr(get, _) => get(&a[0]),
        Handler::List(name, _) => lookup("_k", "ListView")?.call(&[&a[0], &py_str(name)]),
        Handler::Dict(name, _) => lookup("_k", "DictView")?.call(&[&a[0], &py_str(name)]),
        _ => Err(KError::Other("kaliru: not an attribute".into())),
    }
});

fn field_handler(name: &PyRef) -> KResult<Handler> {
    let name = name.as_string()?;
    let id = IDS.iter().position(|n| *n == name).ok_or_else(|| KError::Other(format!("kaliru: no field {}", name)))?;
    handler(id)
}

// list_get(owner, field) / list_set(owner, field, items) (prelude.py ListView)
fn list_ops(a: &[PyRef]) -> KResult<(ListField, String)> {
    match field_handler(&a[1])? {
        Handler::List(n, f) => Ok((f, n.to_string())),
        _ => Err(KError::Other("kaliru: not a list field".into())),
    }
}

fn dict_ops(a: &[PyRef]) -> KResult<(DictField, String)> {
    match field_handler(&a[1])? {
        Handler::Dict(n, f) => Ok((f, n.to_string())),
        _ => Err(KError::Other("kaliru: not a dict field".into())),
    }
}

// set(obj, id, value)
native!(k_set, |a| {
    let id = id_of(&a[1])?;
    let Handler::Attr(_, set) = handler(id)? else { return Err(KError::Other("kaliru: not an attribute".into())) };
    let Some(set) = set else {
        return Err(KError::Py("NotImplementedError".into(), format!("kaliru: setting {} is not mirrored yet", IDS[id])));
    };
    set(&a[0], &a[2])?;
    Ok(py_none())
});

native!(k_delete, |a| {
    Err(KError::Py("NotImplementedError".into(), format!("kaliru: deleting {} is not mirrored", IDS[id_of(&a[1])?])))
});

fn qualname(cls: &PyRef) -> KResult<String> {
    Ok(format!("{}.{}", cls.getattr("__module__")?.as_string()?, cls.getattr("__qualname__")?.as_string()?))
}

// patched_class(cls, name): a mirrored class's attribute set, or a subclass's
native!(k_patched_class, |a| {
    let (q, name) = (qualname(&a[0])?, a[1].as_string()?);
    PATCHED.with(|p| p.borrow_mut().insert((q, name)));
    Ok(py_none())
});

// patched_instance(obj, name)
native!(k_patched_instance, |a| {
    if let Ok(h) = a[0].getattr("_k_h") {
        let (key, name) = (format!("@{}", h.as_i64()?), a[1].as_string()?);
        PATCHED.with(|p| p.borrow_mut().insert((key, name)));
    }
    Ok(py_none())
});

// module_value(module, name): module names not copied into a mirror
native!(k_module_value, |a| {
    let (m, n) = (a[0].as_string()?, a[1].as_string()?);
    Err(KError::Py("AttributeError".into(), format!("module '{}' has no attribute '{}'", m, n)))
});

// source(module): the generated mirror source
native!(k_source, |a| {
    let name = a[0].as_string()?;
    let src = super::sources::source(&name)
        .ok_or_else(|| KError::Py("ImportError".into(), format!("kaliru: no mirror of {}", name)))?;
    Ok(py_str(&src))
});

// log(levelno, text): python logging records into kaliru's log
native!(k_log, |a| {
    crate::logging::log(a[0].as_i64()? as i32, &a[1].as_string()?);
    Ok(py_none())
});

// ext(name, *args): the layer's own helpers (prelude.py views), not Kalico names
native!(k_ext, |a| {
    let name = a[0].as_string()?;
    let rest = &a[1..];
    match name.as_str() {
        "list_get" => {
            let (f, n) = list_ops(rest)?;
            return Ok(py_list((f.get)(&rest[0], &n)?));
        }
        "list_set" => {
            let (f, n) = list_ops(rest)?;
            let items: Vec<PyRef> = super::convert::FromPy::from_py(&rest[2])?;
            (f.set)(&rest[0], &n, items)?;
            return Ok(py_none());
        }
        "dict_get" => {
            let (f, n) = dict_ops(rest)?;
            return (f.get)(&rest[0], &n, &rest[2]);
        }
        "dict_set" => {
            let (f, n) = dict_ops(rest)?;
            (f.set)(&rest[0], &n, &rest[2], &rest[3])?;
            return Ok(py_none());
        }
        "dict_del" => {
            let (f, n) = dict_ops(rest)?;
            (f.del)(&rest[0], &n, &rest[2])?;
            return Ok(py_none());
        }
        "dict_keys" => {
            let (f, n) = dict_ops(rest)?;
            return Ok(py_list((f.keys)(&rest[0], &n)?));
        }
        _ => {}
    }
    let f = EXT
        .with(|e| e.borrow().get(name.as_str()).copied())
        .ok_or_else(|| KError::Other(format!("kaliru: no layer helper {}", name)))?;
    f(&Args { slf: None, args: a[1..].to_vec() })
});

const FLAGS: std::ffi::c_int = METH_FASTCALL | METH_KEYWORDS;

macro_rules! def {
    ($name:literal, $f:ident) => {
        PyMethodDef { ml_name: concat!($name, "\0").as_ptr() as *const c_char, ml_meth: $f, ml_flags: FLAGS, ml_doc: std::ptr::null() }
    };
}

static DEFS: [PyMethodDef; 12] = [
    def!("ext", k_ext),
    def!("log", k_log),
    def!("call", k_call),
    def!("fcall", k_fcall),
    def!("new", k_new),
    def!("get", k_get),
    def!("set", k_set),
    def!("delete", k_delete),
    def!("patched_class", k_patched_class),
    def!("patched_instance", k_patched_instance),
    def!("module_value", k_module_value),
    def!("source", k_source),
];

/// Create `_kaliru` in sys.modules
pub fn create_module() -> KResult<()> {
    unsafe {
        let m = PyRef::owned_or_err(PyImport_AddModuleRef(c"_kaliru".as_ptr()))?;
        for d in DEFS.iter() {
            let f = PyRef::owned_or_err(PyCMethod_New(d, std::ptr::null_mut(), m.as_ptr(), std::ptr::null_mut()))?;
            if PyModule_AddObjectRef(m.as_ptr(), d.ml_name, f.as_ptr()) != 0 {
                return Err(fetch_error());
            }
        }
    }
    Ok(())
}

/// (module name, is package) of every mirror module, as the prelude's finder wants
pub fn mirror_modules() -> PyRef {
    py_list(MODULES.iter().map(|(n, p)| py_tuple(vec![py_str(n), py_bool(*p)])).collect())
}
