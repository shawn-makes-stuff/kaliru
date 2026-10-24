// The CPython C API functions the bridge uses (Include/*.h, CPython 3.15).
// Objects are opaque pointers; only PyMethodDef's layout is relied on.
#![allow(non_snake_case, non_camel_case_types)]
use std::ffi::{c_char, c_double, c_int, c_long, c_void};

pub type PyObject = c_void;
pub type PyThreadState = c_void;
pub type PyInterpreterState = c_void;
pub type Py_ssize_t = isize;

pub type PyCFunctionFast =
    unsafe extern "C" fn(slf: *mut PyObject, args: *const *mut PyObject, nargs: Py_ssize_t) -> *mut PyObject;

#[repr(C)]
pub struct PyMethodDef {
    pub ml_name: *const c_char,
    pub ml_meth: PyCFunctionFast,
    pub ml_flags: c_int,
    pub ml_doc: *const c_char,
}
unsafe impl Sync for PyMethodDef {}

pub const METH_FASTCALL: c_int = 0x0080;

extern "C" {
    pub fn Py_InitializeEx(initsigs: c_int);
    pub fn PyRun_SimpleString(command: *const c_char) -> c_int;
    pub fn PyImport_AddModuleRef(name: *const c_char) -> *mut PyObject;
    pub fn PyCMethod_New(ml: *const PyMethodDef, slf: *mut PyObject, module: *mut PyObject, cls: *mut c_void) -> *mut PyObject;
    pub fn PyObject_SetAttrString(o: *mut PyObject, name: *const c_char, v: *mut PyObject) -> c_int;
    pub fn PyLong_AsLong(o: *mut PyObject) -> c_long;
    pub fn PyFloat_AsDouble(o: *mut PyObject) -> c_double;
    pub fn PyFloat_FromDouble(v: c_double) -> *mut PyObject;
    pub fn PyErr_Occurred() -> *mut PyObject;
    pub fn Py_DecRef(o: *mut PyObject);
    pub fn PyObject_GetAttrString(o: *mut PyObject, name: *const c_char) -> *mut PyObject;
    pub fn PyObject_CallNoArgs(callable: *mut PyObject) -> *mut PyObject;

    pub fn PyThreadState_Get() -> *mut PyThreadState;
    pub fn PyThreadState_New(interp: *mut PyInterpreterState) -> *mut PyThreadState;
    pub fn PyThreadState_Swap(tstate: *mut PyThreadState) -> *mut PyThreadState;
    pub fn PyThreadState_Clear(tstate: *mut PyThreadState);
    pub fn PyThreadState_Delete(tstate: *mut PyThreadState);
    pub fn PyInterpreterState_Get() -> *mut PyInterpreterState;
    pub fn PyEval_SaveThread() -> *mut PyThreadState;
    pub fn PyEval_RestoreThread(tstate: *mut PyThreadState);
    pub fn PyUnstable_ThreadState_SetStackProtection(
        tstate: *mut PyThreadState,
        stack_start_addr: *mut c_void,
        stack_size: usize,
    ) -> c_int;
}
