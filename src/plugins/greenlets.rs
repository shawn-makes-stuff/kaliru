// Python on kaliru's greenlets (docs/PLUGINS.md "Greenlets", verified by the
// prototype in pybridge/: without either part below it segfaults).
//
// Python frames can be live on several greenlets at once (a plugin command
// that waits while a timer runs another plugin callback). Each greenlet that
// runs python gets its own PyThreadState: its frame stack, exception state and
// recursion depth. The switch hook makes the running greenlet's thread state
// current and tells CPython which C stack it runs on
// (PyUnstable_ThreadState_SetStackProtection, CPython 3.15; the address is
// the stack's lowest one).
//
// Precondition: the reactor thread holds the GIL at every switch (it releases
// it only while blocked in poll, where no switch happens).
use super::capi::*;
use crate::greenlet::{getcurrent, set_switch_hook, Greenlet, SwitchEvent};
use std::cell::{Cell, RefCell};
use std::collections::HashMap;

thread_local! {
    static MAIN: Cell<*mut PyThreadState> = const { Cell::new(std::ptr::null_mut()) };
    // greenlet address -> its thread state
    static STATES: RefCell<HashMap<usize, *mut PyThreadState>> = RefCell::new(HashMap::new());
}

/// Call on the reactor thread, with the GIL held, after Py_Initialize.
/// Python starts on demand, usually inside a reactor greenlet (a module's
/// load_config): that greenlet keeps the thread state python started on
/// (its frames are there) and the main context gets a new one.
pub fn install() {
    let current = unsafe { PyThreadState_Get() };
    match getcurrent() {
        Some(g) => unsafe {
            STATES.with(|s| s.borrow_mut().insert(key(&g), current));
            let (lo, size) = g.stack();
            if PyUnstable_ThreadState_SetStackProtection(current, lo as *mut _, size) != 0 {
                panic!("PyUnstable_ThreadState_SetStackProtection failed");
            }
            MAIN.with(|m| m.set(PyThreadState_New(PyInterpreterState_Get())));
        },
        None => MAIN.with(|m| m.set(current)),
    }
    set_switch_hook(Some(hook));
}

fn key(g: &Greenlet) -> usize {
    g as *const Greenlet as usize
}

fn hook(ev: SwitchEvent) {
    unsafe {
        match ev {
            SwitchEvent::Enter(g) => {
                let t = STATES.with(|s| {
                    *s.borrow_mut().entry(key(g)).or_insert_with(|| PyThreadState_New(PyInterpreterState_Get()))
                });
                PyThreadState_Swap(t);
                // nothing else may call the C API before the stack changes
                let (lo, size) = g.stack();
                if PyUnstable_ThreadState_SetStackProtection(t, lo as *mut _, size) != 0 {
                    panic!("PyUnstable_ThreadState_SetStackProtection failed");
                }
            }
            SwitchEvent::Main => {
                PyThreadState_Swap(MAIN.with(|m| m.get()));
            }
            SwitchEvent::Dead(g) => {
                if let Some(t) = STATES.with(|s| s.borrow_mut().remove(&key(g))) {
                    PyThreadState_Clear(t);
                    PyThreadState_Delete(t);
                }
            }
        }
    }
}
