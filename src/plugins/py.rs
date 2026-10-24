// Python object references and the conversions the plugin layer needs:
// PyRef owns one reference (Drop = Py_DecRef; the GIL is held wherever
// kaliru runs, see greenlets.rs), errors map both ways between KError and
// python exceptions.
use crate::plugins::gil::Gil;
use super::capi::*;
use crate::error::{ForeignError, KError, KResult};
use crate::pyval::PyVal;
use std::cell::RefCell;
use std::collections::HashMap;
use std::ffi::{c_char, CString};
use std::ptr::NonNull;
use std::rc::Rc;

pub struct PyRef(NonNull<PyObject>);

impl PyRef {
    /// a new reference returned by the C API; None (with the python error
    /// set) when it is NULL
    pub unsafe fn owned(p: *mut PyObject) -> Option<PyRef> {
        NonNull::new(p).map(PyRef)
    }

    /// a new reference or the pending python error as a KError
    pub unsafe fn owned_or_err(p: *mut PyObject) -> KResult<PyRef> {
        Self::owned(p).ok_or_else(fetch_error)
    }

    /// a borrowed reference, made owned
    pub unsafe fn borrowed(p: *mut PyObject) -> PyRef {
        Py_IncRef(p);
        PyRef(NonNull::new(p).expect("borrowed NULL"))
    }

    pub fn as_ptr(&self) -> *mut PyObject {
        self.0.as_ptr()
    }

    /// hand the reference to the C API (which steals it)
    pub fn into_ptr(self) -> *mut PyObject {
        let p = self.as_ptr();
        std::mem::forget(self);
        p
    }

    pub fn is(&self, other: *mut PyObject) -> bool {
        self.as_ptr() == other
    }

    pub fn is_none(&self) -> bool {
        self.as_ptr() == none()
    }

    pub fn getattr(&self, name: &str) -> KResult<PyRef> {
        let c = CString::new(name).unwrap();
        unsafe { PyRef::owned_or_err(PyObject_GetAttrString(self.as_ptr(), c.as_ptr())) }
    }

    pub fn hasattr(&self, name: &str) -> KResult<bool> {
        let c = CString::new(name).unwrap();
        match unsafe { PyObject_HasAttrStringWithError(self.as_ptr(), c.as_ptr()) } {
            -1 => Err(fetch_error()),
            r => Ok(r == 1),
        }
    }

    pub fn setattr(&self, name: &str, v: &PyRef) -> KResult<()> {
        let c = CString::new(name).unwrap();
        if unsafe { PyObject_SetAttrString(self.as_ptr(), c.as_ptr(), v.as_ptr()) } != 0 {
            return Err(fetch_error());
        }
        Ok(())
    }

    pub fn call(&self, args: &[&PyRef]) -> KResult<PyRef> {
        let ptrs: Vec<*mut PyObject> = args.iter().map(|a| a.as_ptr()).collect();
        unsafe { PyRef::owned_or_err(PyObject_Vectorcall(self.as_ptr(), ptrs.as_ptr(), ptrs.len(), std::ptr::null_mut())) }
    }

    pub fn call_method(&self, name: &str, args: &[&PyRef]) -> KResult<PyRef> {
        self.getattr(name)?.call(args)
    }

    pub fn is_true(&self) -> KResult<bool> {
        match unsafe { PyObject_IsTrue(self.as_ptr()) } {
            -1 => Err(fetch_error()),
            r => Ok(r == 1),
        }
    }

    pub fn is_instance(&self, cls: &PyRef) -> KResult<bool> {
        match unsafe { PyObject_IsInstance(self.as_ptr(), cls.as_ptr()) } {
            -1 => Err(fetch_error()),
            r => Ok(r == 1),
        }
    }

    /// str(obj)
    pub fn str(&self) -> KResult<String> {
        let s = unsafe { PyRef::owned_or_err(PyObject_Str(self.as_ptr()))? };
        s.as_string()
    }

    /// repr(obj)
    pub fn repr(&self) -> KResult<String> {
        let s = unsafe { PyRef::owned_or_err(PyObject_Repr(self.as_ptr()))? };
        s.as_string()
    }

    /// the contents of a str object
    pub fn as_string(&self) -> KResult<String> {
        let mut len: Py_ssize_t = 0;
        let p = unsafe { PyUnicode_AsUTF8AndSize(self.as_ptr(), &mut len) };
        if p.is_null() {
            return Err(fetch_error());
        }
        let bytes = unsafe { std::slice::from_raw_parts(p as *const u8, len as usize) };
        Ok(String::from_utf8_lossy(bytes).into_owned())
    }

    pub fn as_f64(&self) -> KResult<f64> {
        let v = unsafe { PyFloat_AsDouble(self.as_ptr()) };
        if v == -1.0 && unsafe { !PyErr_Occurred().is_null() } {
            return Err(fetch_error());
        }
        Ok(v)
    }

    pub fn as_i64(&self) -> KResult<i64> {
        let v = unsafe { PyLong_AsLongLong(self.as_ptr()) };
        if v == -1 && unsafe { !PyErr_Occurred().is_null() } {
            return Err(fetch_error());
        }
        Ok(v)
    }
}

impl Clone for PyRef {
    fn clone(&self) -> Self {
        unsafe { PyRef::borrowed(self.as_ptr()) }
    }
}

impl Drop for PyRef {
    fn drop(&mut self) {
        unsafe { Py_DecRef(self.as_ptr()) }
    }
}

pub fn py_none() -> PyRef {
    unsafe { PyRef::borrowed(none()) }
}

pub fn py_bool(b: bool) -> PyRef {
    unsafe { PyRef::owned(PyBool_FromLong(b as _)).unwrap() }
}

pub fn py_float(v: f64) -> PyRef {
    unsafe { PyRef::owned(PyFloat_FromDouble(v)).unwrap() }
}

pub fn py_int(v: i64) -> PyRef {
    unsafe { PyRef::owned(PyLong_FromLongLong(v)).unwrap() }
}

pub fn py_str(s: &str) -> PyRef {
    unsafe { PyRef::owned(PyUnicode_FromStringAndSize(s.as_ptr() as *const c_char, s.len() as Py_ssize_t)).unwrap() }
}

pub fn py_tuple(items: Vec<PyRef>) -> PyRef {
    unsafe {
        let t = PyRef::owned(PyTuple_New(items.len() as Py_ssize_t)).unwrap();
        for (i, it) in items.into_iter().enumerate() {
            PyTuple_SetItem(t.as_ptr(), i as Py_ssize_t, it.into_ptr());
        }
        t
    }
}

pub fn py_list(items: Vec<PyRef>) -> PyRef {
    unsafe {
        let l = PyRef::owned(PyList_New(0)).unwrap();
        for it in items {
            PyList_Append(l.as_ptr(), it.as_ptr());
        }
        l
    }
}

pub fn import(name: &str) -> KResult<PyRef> {
    let c = CString::new(name).unwrap();
    unsafe { PyRef::owned_or_err(PyImport_ImportModule(c.as_ptr())) }
}

// "module:attr" -> object (exception classes and the like)
static CACHE: Gil<RefCell<HashMap<String, PyRef>>> = Gil::new(|| RefCell::new(HashMap::new()));

/// module.attr, cached for the life of the interpreter
pub fn lookup(module: &str, attr: &str) -> KResult<PyRef> {
    let key = format!("{}:{}", module, attr);
    if let Some(v) = CACHE.with(|c| c.borrow().get(&key).cloned()) {
        return Ok(v);
    }
    let v = import(module)?.getattr(attr)?;
    CACHE.with(|c| c.borrow_mut().insert(key, v.clone()));
    Ok(v)
}

pub fn clear_cache() {
    CACHE.with(|c| c.borrow_mut().clear());
}

// ------------------------------------------------------------------ PyVal

pub fn from_pyval(v: &PyVal) -> PyRef {
    match v {
        PyVal::None => py_none(),
        PyVal::Bool(b) => py_bool(*b),
        PyVal::Int(i) => py_int(*i),
        PyVal::Float(f) => py_float(*f),
        PyVal::Str(s) => py_str(s),
        PyVal::List(l) => py_list(l.iter().map(from_pyval).collect()),
        PyVal::Tuple(t) | PyVal::NamedTuple(_, _, t) => py_tuple(t.iter().map(from_pyval).collect()),
        PyVal::Dict(d) => unsafe {
            let o = PyRef::owned(PyDict_New()).unwrap();
            for (k, v) in d {
                PyDict_SetItem(o.as_ptr(), py_str(k).as_ptr(), from_pyval(v).as_ptr());
            }
            o
        },
    }
}

/// a python value as PyVal (get_status data, config defaults); other
/// objects are an error naming their type
pub fn to_pyval(o: &PyRef) -> KResult<PyVal> {
    let tname = type_name(o)?;
    Ok(match tname.as_str() {
        "NoneType" => PyVal::None,
        "bool" => PyVal::Bool(o.is_true()?),
        "int" => PyVal::Int(o.as_i64()?),
        "float" => PyVal::Float(o.as_f64()?),
        "str" => PyVal::Str(o.as_string()?),
        "list" | "tuple" => {
            let n = unsafe {
                if tname == "list" { PyList_Size(o.as_ptr()) } else { PyTuple_Size(o.as_ptr()) }
            };
            let mut items = vec![];
            for i in 0..n {
                let it = unsafe {
                    PyRef::borrowed(if tname == "list" { PyList_GetItem(o.as_ptr(), i) } else { PyTuple_GetItem(o.as_ptr(), i) })
                };
                items.push(to_pyval(&it)?);
            }
            if tname == "list" { PyVal::List(items) } else { PyVal::Tuple(items) }
        }
        "dict" => {
            let mut d = indexmap::IndexMap::new();
            let (mut pos, mut k, mut v) = (0, std::ptr::null_mut(), std::ptr::null_mut());
            while unsafe { PyDict_Next(o.as_ptr(), &mut pos, &mut k, &mut v) } != 0 {
                let (k, v) = unsafe { (PyRef::borrowed(k), PyRef::borrowed(v)) };
                d.insert(k.str()?, to_pyval(&v)?);
            }
            PyVal::Dict(d)
        }
        _ => {
            // subclasses of the basic types (namedtuples, OrderedDict,
            // str / int enums: AFC's State) as their base value - what
            // json.dumps sends; a str enum's str() is not kept (docs/PLUGINS.md)
            for (base, conv) in [("bool", "bool"), ("int", "int"), ("float", "float"), ("str", "str"), ("tuple", "tuple"), ("list", "list"), ("dict", "dict")] {
                if o.is_instance(&lookup("builtins", base)?)? {
                    let plain = if base == "str" {
                        // the string itself, not str() (an enum's would be 'State.IDLE')
                        return Ok(PyVal::Str(o.as_string()?));
                    } else {
                        lookup("builtins", conv)?.call(&[o])?
                    };
                    return to_pyval(&plain);
                }
            }
            return Err(KError::Py("TypeError".into(), format!("kaliru: cannot pass a '{}' object here", tname)));
        }
    })
}

pub fn type_name(o: &PyRef) -> KResult<String> {
    unsafe { PyRef::borrowed(o.as_ptr()) }.getattr("__class__")?.getattr("__name__")?.as_string()
}

// ----------------------------------------------------------------- errors

// KError kinds and the klippy exception class each one is
const KINDS: &[(&str, &str)] = &[
    ("klippy.printer", "WaitInterruption"),
    ("klippy.gcode", "CommandError"),
    ("klippy.msgproto", "enumeration_error"),
    ("klippy.msgproto", "error"),
    ("klippy.configfile", "error"),
    ("klippy.pins", "error"),
    ("klippy.serialhdl", "error"),
    ("klippy.mcu", "error"),
    ("klippy.reactor", "ReactorError"),
];

/// the pending python exception as a KError (Foreign: it keeps the object)
pub fn fetch_error() -> KError {
    let exc = unsafe { PyRef::owned(PyErr_GetRaisedException()) };
    let Some(exc) = exc else {
        return KError::Other("kaliru: python error without an exception".into());
    };
    exception_to_kerror(exc)
}

// set while an exception is being converted: a failure inside the
// conversion (a class lookup that raises) must not convert recursively
static CONVERTING: Gil<std::cell::Cell<bool>> = Gil::new(|| std::cell::Cell::new(false));

pub fn exception_to_kerror(exc: PyRef) -> KError {
    if CONVERTING.with(|c| c.replace(true)) {
        let msg = exc.str().unwrap_or_default();
        return KError::Other(format!("kaliru: error while converting a python exception: {}", msg));
    }
    let e = exception_to_kerror_inner(exc);
    CONVERTING.with(|c| c.set(false));
    e
}

fn exception_to_kerror_inner(exc: PyRef) -> KError {
    let msg = exc.str().unwrap_or_default();
    let cls = exc.getattr("__class__").ok();
    let class = cls
        .as_ref()
        .and_then(|c| {
            let m = c.getattr("__module__").ok()?.as_string().ok()?;
            let q = c.getattr("__qualname__").ok()?.as_string().ok()?;
            Some(if m == "builtins" { q } else { format!("{}.{}", m, q) })
        })
        .unwrap_or_else(|| "Exception".into());
    let mut kind = KError::Py(class.rsplit('.').next().unwrap_or("Exception").to_string(), msg.clone());
    for (module, name) in KINDS {
        let Ok(k) = lookup(module, name) else { continue };
        if exc.is_instance(&k).unwrap_or(false) {
            kind = match (*module, *name) {
                ("klippy.printer", _) => KError::WaitInterruption(msg.clone()),
                ("klippy.gcode", _) => KError::Command(msg.clone()),
                ("klippy.msgproto", "enumeration_error") => KError::Protocol(msg.clone()),
                ("klippy.msgproto", _) => KError::Protocol(msg.clone()),
                ("klippy.configfile", _) => KError::Config(msg.clone()),
                ("klippy.pins", _) => KError::Pins(msg.clone()),
                ("klippy.serialhdl", _) => KError::Serial(msg.clone()),
                ("klippy.mcu", _) => KError::Mcu(msg.clone()),
                _ => KError::Reactor(msg.clone()),
            };
            break;
        }
    }
    let traceback = lookup("traceback", "format_exception")
        .and_then(|f| f.call(&[&exc]))
        .and_then(|l| lookup("builtins", "str")?.getattr("join")?.call(&[&py_str(""), &l]))
        .and_then(|t| t.as_string())
        .ok();
    KError::Foreign(Rc::new(ForeignError { kind, class, obj: Box::new(exc), traceback }))
}

/// raise `e` in python (sets the error indicator; the caller returns NULL)
pub fn raise(e: &KError) {
    match kerror_to_exception(e) {
        Ok(exc) => unsafe { PyErr_SetRaisedException(exc.into_ptr()) },
        Err(e2) => {
            // building the exception failed: say both
            let msg = CString::new(format!("kaliru: {} (while raising: {})", e2, e)).unwrap_or_default();
            unsafe { PyErr_SetString(exc_class("PyExc_RuntimeError"), msg.as_ptr()) }
        }
    }
}

fn kerror_to_exception(e: &KError) -> KResult<PyRef> {
    let (cls, msg) = match e {
        KError::Foreign(f) => {
            if let Some(obj) = f.obj.downcast_ref::<PyRef>() {
                return Ok(obj.clone());
            }
            (lookup("builtins", "Exception")?, f.kind.message().to_string())
        }
        KError::Chained(err, ctx) => {
            let exc = kerror_to_exception(err)?;
            let c = kerror_to_exception(ctx)?;
            exc.setattr("__context__", &c)?;
            return Ok(exc);
        }
        KError::Config(m) => (lookup("klippy.configfile", "error")?, m.clone()),
        KError::Pins(m) => (lookup("klippy.pins", "error")?, m.clone()),
        KError::Protocol(m) => (lookup("klippy.msgproto", "error")?, m.clone()),
        KError::Enumeration { msg, .. } => (lookup("klippy.msgproto", "enumeration_error")?, msg.clone()),
        KError::Serial(m) => (lookup("klippy.serialhdl", "error")?, m.clone()),
        KError::Mcu(m) => (lookup("klippy.mcu", "error")?, m.clone()),
        KError::Command(m) => (lookup("klippy.gcode", "CommandError")?, m.clone()),
        KError::WaitInterruption(m) => (lookup("klippy.printer", "WaitInterruption")?, m.clone()),
        KError::CommandSub(class, m) => {
            let (module, name) = class.rsplit_once('.').unwrap_or(("klippy.gcode", class));
            (lookup(module, name)?, m.clone())
        }
        KError::Reactor(m) => (lookup("klippy.reactor", "ReactorError")?, m.clone()),
        KError::Py(class, m) => {
            let (module, name) = class.rsplit_once('.').unwrap_or(("builtins", class));
            (lookup(module, name).or_else(|_| lookup("builtins", "Exception"))?, m.clone())
        }
        KError::Other(m) => (lookup("builtins", "Exception")?, m.clone()),
    };
    cls.call(&[&py_str(&msg)])
}
