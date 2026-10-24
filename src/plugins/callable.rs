// Rust closures as python callables: what Kalico hands plugins as bound
// methods stored in attributes (gcmd.respond_info, reactor.monotonic) and
// old handlers returned by register_command.
use super::capi::*;
use super::py::*;
use crate::error::KResult;
use std::ffi::c_char;
use std::rc::Rc;

pub type RustFn = Rc<dyn Fn(&[PyRef]) -> KResult<PyRef>>;

const CAPSULE: &[u8] = b"kaliru.function\0";

unsafe extern "C" fn trampoline(
    slf: *mut PyObject,
    args: *const *mut PyObject,
    nargs: Py_ssize_t,
    _kw: *mut PyObject,
) -> *mut PyObject {
    let f = &*(PyCapsule_GetPointer(slf, CAPSULE.as_ptr() as *const c_char) as *const RustFn);
    let n = (nargs as usize) & !(1usize << (usize::BITS - 1));
    let a: Vec<PyRef> = (0..n).map(|i| PyRef::borrowed(*args.add(i))).collect();
    match f(&a) {
        Ok(v) => v.into_ptr(),
        Err(e) => {
            raise(&e);
            std::ptr::null_mut()
        }
    }
}

unsafe extern "C" fn destructor(capsule: *mut PyObject) {
    drop(Box::from_raw(PyCapsule_GetPointer(capsule, CAPSULE.as_ptr() as *const c_char) as *mut RustFn));
}

static DEF: PyMethodDef = PyMethodDef {
    ml_name: c"kaliru_function".as_ptr(),
    ml_meth: trampoline,
    ml_flags: METH_FASTCALL | METH_KEYWORDS,
    ml_doc: std::ptr::null(),
};

/// A python callable running `f`
pub fn py_function(f: RustFn) -> KResult<PyRef> {
    unsafe {
        let cap = PyRef::owned_or_err(PyCapsule_New(
            Box::into_raw(Box::new(f)) as *mut _,
            CAPSULE.as_ptr() as *const c_char,
            Some(destructor),
        ))?;
        PyRef::owned_or_err(PyCMethod_New(&DEF, cap.as_ptr(), std::ptr::null_mut(), std::ptr::null_mut()))
    }
}
