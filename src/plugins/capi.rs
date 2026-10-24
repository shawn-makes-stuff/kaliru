// The CPython C API, loaded at run time (dlopen) the first time a plugin is
// needed: a kaliru without plugins never maps libpython. The library is
// opened RTLD_GLOBAL so C extension modules (cffi, numpy, ...) resolve their
// libpython symbols from it, as they do in the python executable.
//
// Every function is called through the table: `capi::PyLong_AsLong(o)` is
// `(api().PyLong_AsLong)(o)`. Objects are opaque; only the layouts of
// PyMethodDef, PyGetSetDef, PyType_Slot and PyType_Spec are relied on
// (CPython 3.15 Include/ headers).
#![allow(non_snake_case, non_camel_case_types, clippy::missing_safety_doc)]
use std::ffi::{c_char, c_double, c_int, c_long, c_longlong, c_uint, c_void, CStr, CString};
use std::sync::OnceLock;

pub type PyObject = c_void;
pub type PyTypeObject = c_void;
pub type PyThreadState = c_void;
pub type PyInterpreterState = c_void;
pub type Py_ssize_t = isize;

pub type PyCFunctionFastKw = unsafe extern "C" fn(
    slf: *mut PyObject,
    args: *const *mut PyObject,
    nargs: Py_ssize_t,
    kwnames: *mut PyObject,
) -> *mut PyObject;

#[repr(C)]
pub struct PyMethodDef {
    pub ml_name: *const c_char,
    pub ml_meth: PyCFunctionFastKw,
    pub ml_flags: c_int,
    pub ml_doc: *const c_char,
}
unsafe impl Sync for PyMethodDef {}

pub const METH_FASTCALL: c_int = 0x0080;
pub const METH_KEYWORDS: c_int = 0x0002;

/// Py_GetConstantBorrowed ids (object.h)
pub const PY_CONSTANT_NONE: c_uint = 0;
pub const PY_CONSTANT_FALSE: c_uint = 1;
pub const PY_CONSTANT_TRUE: c_uint = 2;

/// The oldest CPython that runs python on greenlets
/// (PyUnstable_ThreadState_SetStackProtection, docs/PLUGINS.md)
pub const MIN_VERSION: (u32, u32) = (3, 15);

macro_rules! capi {
    ($( fn $name:ident($($a:ident: $t:ty),*) $(-> $r:ty)?; )*) => {
        pub struct Api {
            $( pub $name: unsafe extern "C" fn($($t),*) $(-> $r)?, )*
        }

        impl Api {
            unsafe fn load(lib: *mut c_void) -> Result<Api, String> {
                Ok(Api {
                    $( $name: {
                        let p = libc::dlsym(lib, concat!(stringify!($name), "\0").as_ptr() as *const c_char);
                        if p.is_null() {
                            return Err(format!("libpython has no {}", stringify!($name)));
                        }
                        std::mem::transmute::<*mut c_void, unsafe extern "C" fn($($t),*) $(-> $r)?>(p)
                    }, )*
                })
            }
        }

        $(
            #[inline]
            pub unsafe fn $name($($a: $t),*) $(-> $r)? {
                (api().$name)($($a),*)
            }
        )*
    };
}

capi! {
    fn Py_InitializeEx(initsigs: c_int);
    fn Py_IsInitialized() -> c_int;
    fn Py_GetConstantBorrowed(id: c_uint) -> *mut PyObject;
    fn Py_IncRef(o: *mut PyObject);
    fn Py_DecRef(o: *mut PyObject);

    fn PyImport_ImportModule(name: *const c_char) -> *mut PyObject;
    fn PyImport_AddModuleRef(name: *const c_char) -> *mut PyObject;
    fn PyModule_AddObjectRef(m: *mut PyObject, name: *const c_char, v: *mut PyObject) -> c_int;
    fn PyCMethod_New(ml: *const PyMethodDef, slf: *mut PyObject, module: *mut PyObject, cls: *mut c_void) -> *mut PyObject;
    fn Py_CompileString(src: *const c_char, filename: *const c_char, start: c_int) -> *mut PyObject;
    fn PyEval_EvalCode(co: *mut PyObject, globals: *mut PyObject, locals: *mut PyObject) -> *mut PyObject;
    fn PyModule_GetDict(m: *mut PyObject) -> *mut PyObject;

    fn PyObject_GetAttrString(o: *mut PyObject, name: *const c_char) -> *mut PyObject;
    fn PyObject_SetAttrString(o: *mut PyObject, name: *const c_char, v: *mut PyObject) -> c_int;
    fn PyObject_HasAttrStringWithError(o: *mut PyObject, name: *const c_char) -> c_int;
    fn PyObject_Vectorcall(callable: *mut PyObject, args: *const *mut PyObject, nargsf: usize, kwnames: *mut PyObject) -> *mut PyObject;
    fn PyObject_CallNoArgs(callable: *mut PyObject) -> *mut PyObject;
    fn PyObject_IsTrue(o: *mut PyObject) -> c_int;
    fn PyObject_Str(o: *mut PyObject) -> *mut PyObject;
    fn PyObject_Repr(o: *mut PyObject) -> *mut PyObject;
    fn PyObject_IsInstance(o: *mut PyObject, cls: *mut PyObject) -> c_int;
    fn PyCallable_Check(o: *mut PyObject) -> c_int;

    fn PyLong_FromLongLong(v: c_longlong) -> *mut PyObject;
    fn PyLong_AsLongLong(o: *mut PyObject) -> c_longlong;
    fn PyLong_AsLong(o: *mut PyObject) -> c_long;
    fn PyFloat_FromDouble(v: c_double) -> *mut PyObject;
    fn PyFloat_AsDouble(o: *mut PyObject) -> c_double;
    fn PyBool_FromLong(v: c_long) -> *mut PyObject;
    fn PyUnicode_FromStringAndSize(s: *const c_char, len: Py_ssize_t) -> *mut PyObject;
    fn PyUnicode_AsUTF8AndSize(o: *mut PyObject, len: *mut Py_ssize_t) -> *const c_char;
    fn PyTuple_New(len: Py_ssize_t) -> *mut PyObject;
    fn PyTuple_SetItem(t: *mut PyObject, i: Py_ssize_t, v: *mut PyObject) -> c_int;
    fn PyTuple_GetItem(t: *mut PyObject, i: Py_ssize_t) -> *mut PyObject;
    fn PyTuple_Size(t: *mut PyObject) -> Py_ssize_t;
    fn PyList_New(len: Py_ssize_t) -> *mut PyObject;
    fn PyList_Append(l: *mut PyObject, v: *mut PyObject) -> c_int;
    fn PyList_Size(l: *mut PyObject) -> Py_ssize_t;
    fn PyList_GetItem(l: *mut PyObject, i: Py_ssize_t) -> *mut PyObject;
    fn PyDict_New() -> *mut PyObject;
    fn PyDict_SetItem(d: *mut PyObject, k: *mut PyObject, v: *mut PyObject) -> c_int;
    fn PyDict_SetItemString(d: *mut PyObject, k: *const c_char, v: *mut PyObject) -> c_int;
    fn PyDict_GetItemStringRef(d: *mut PyObject, k: *const c_char, result: *mut *mut PyObject) -> c_int;
    fn PyDict_Next(d: *mut PyObject, pos: *mut Py_ssize_t, k: *mut *mut PyObject, v: *mut *mut PyObject) -> c_int;

    fn PyCapsule_New(ptr: *mut c_void, name: *const c_char, destructor: Option<unsafe extern "C" fn(*mut PyObject)>) -> *mut PyObject;
    fn PyCapsule_GetPointer(capsule: *mut PyObject, name: *const c_char) -> *mut c_void;

    fn PyErr_Occurred() -> *mut PyObject;
    fn PyErr_SetString(exc: *mut PyObject, msg: *const c_char);
    fn PyErr_SetObject(exc: *mut PyObject, v: *mut PyObject);
    fn PyErr_GetRaisedException() -> *mut PyObject;
    fn PyErr_SetRaisedException(exc: *mut PyObject);
    fn PyErr_Print();
    fn PyErr_NewException(name: *const c_char, base: *mut PyObject, dict: *mut PyObject) -> *mut PyObject;

    fn PyThreadState_Get() -> *mut PyThreadState;
    fn PyThreadState_New(interp: *mut PyInterpreterState) -> *mut PyThreadState;
    fn PyThreadState_Swap(t: *mut PyThreadState) -> *mut PyThreadState;
    fn PyThreadState_Clear(t: *mut PyThreadState);
    fn PyThreadState_Delete(t: *mut PyThreadState);
    fn PyInterpreterState_Get() -> *mut PyInterpreterState;
    fn PyEval_SaveThread() -> *mut PyThreadState;
    fn PyEval_RestoreThread(t: *mut PyThreadState);
    fn PyUnstable_ThreadState_SetStackProtection(t: *mut PyThreadState, start: *mut c_void, size: usize) -> c_int;
}

static API: OnceLock<Api> = OnceLock::new();

/// The loaded table; panics if load() has not succeeded
pub fn api() -> &'static Api {
    API.get().expect("libpython not loaded")
}

pub fn is_loaded() -> bool {
    API.get().is_some()
}

/// The libpython names tried, newest first, after $KALIRU_LIBPYTHON
fn candidates() -> Vec<String> {
    let mut out = vec![];
    if let Ok(p) = std::env::var("KALIRU_LIBPYTHON") {
        out.push(p);
    }
    for minor in (MIN_VERSION.1..=MIN_VERSION.1 + 10).rev() {
        out.push(format!("libpython3.{minor}.so.1.0"));
        out.push(format!("libpython3.{minor}.so"));
    }
    out
}

/// dlopen libpython and resolve the table (once). Err names what was tried.
pub fn load() -> Result<(), String> {
    if is_loaded() {
        return Ok(());
    }
    let mut errors = vec![];
    for name in candidates() {
        let c = CString::new(name.clone()).unwrap();
        let lib = unsafe { libc::dlopen(c.as_ptr(), libc::RTLD_NOW | libc::RTLD_GLOBAL) };
        if lib.is_null() {
            let e = unsafe { libc::dlerror() };
            if !e.is_null() && std::env::var("KALIRU_LIBPYTHON").as_deref() == Ok(name.as_str()) {
                errors.push(unsafe { CStr::from_ptr(e) }.to_string_lossy().into_owned());
            }
            continue;
        }
        let api = unsafe { Api::load(lib) }?;
        let _ = API.set(api);
        return Ok(());
    }
    errors.insert(0, format!("no CPython {}.{}+ shared library found (libpython3.N.so)", MIN_VERSION.0, MIN_VERSION.1));
    Err(errors.join("; "))
}

/// A `PyExc_*` exception class (a data symbol of libpython)
pub fn exc_class(name: &str) -> *mut PyObject {
    let c = CString::new(name).unwrap();
    unsafe {
        let p = libc::dlsym(libc::RTLD_DEFAULT, c.as_ptr()) as *const *mut PyObject;
        if p.is_null() { std::ptr::null_mut() } else { *p }
    }
}

pub fn none() -> *mut PyObject {
    unsafe { Py_GetConstantBorrowed(PY_CONSTANT_NONE) }
}
