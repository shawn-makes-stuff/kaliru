// Objects handed to callbacks by reference while they run (a gcode command,
// a web request). Python plugins keep references to the object itself (to
// respond through it later, or to compare it), so the plugin layer needs the
// Rc behind the reference: run() records it for the duration of the call.
// No python counterpart file.
use std::any::Any;
use std::cell::RefCell;
use std::rc::Rc;

thread_local! {
    static RUNNING: RefCell<Vec<Rc<dyn Any>>> = const { RefCell::new(Vec::new()) };
}

/// f(&obj) with obj findable by running() meanwhile. Callbacks pause, so
/// other greenlets' objects come and go in between: removal is by identity.
pub fn run<T: 'static, R>(obj: &Rc<T>, f: impl FnOnce(&T) -> R) -> R {
    let any: Rc<dyn Any> = obj.clone();
    RUNNING.with(|r| r.borrow_mut().push(any.clone()));
    let res = f(obj);
    RUNNING.with(|r| r.borrow_mut().retain(|o| !Rc::ptr_eq(o, &any)));
    res
}

/// The Rc of `obj` if it is being run
pub fn running<T: 'static>(obj: &T) -> Option<Rc<T>> {
    RUNNING.with(|r| {
        r.borrow()
            .iter()
            .rev()
            .find(|o| std::ptr::eq(Rc::as_ptr(o) as *const (), obj as *const T as *const ()))
            .and_then(|o| o.clone().downcast::<T>().ok())
    })
}
