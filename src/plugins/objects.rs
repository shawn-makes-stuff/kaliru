// Kaliru objects seen from python: each Rust object handed to a plugin gets
// one python object of its mirrored class (so `is` holds), created without
// running __init__ and carrying a handle (`_k_h`) back to the Rust object.
// Objects a plugin constructs (`ConfigWrapper(...)`, a subclass of a
// mirrored class) are bound to their Rust object by _k.new.
//
// The table holds both sides until teardown() (the end of a printer
// instance, as python klippy drops all printer objects on RESTART).
use crate::plugins::gil::Gil;
use super::py::{lookup, py_int, PyRef};
use crate::error::{KError, KResult};
use crate::printer::PObj;
use std::any::{Any, TypeId};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

struct Entry {
    rust: Rc<dyn Any>,
    py: PyRef,
    // the same object as a printer object, when it is one
    pobj: Option<PObj>,
}

#[derive(Default)]
struct Table {
    entries: Vec<Option<Entry>>,
    // Rust object address -> handle
    by_addr: HashMap<usize, usize>,
    // Rust type -> mirrored class (module, class name)
    classes: HashMap<TypeId, (&'static str, &'static str)>,
    // printer object types that also travel as plain Rc (a heater as
    // extruder.heater): how to see one as a printer object
    pobj_casts: HashMap<TypeId, fn(Rc<dyn Any>) -> Option<PObj>>,
}

static TABLE: Gil<RefCell<Table>> = Gil::new(|| RefCell::new(Table::default()));

fn addr(rust: &Rc<dyn Any>) -> usize {
    Rc::as_ptr(rust) as *const () as usize
}

/// Register the mirrored class of a Rust type whose path does not name it
/// ("klippy.mcu", "MCU" for kaliru::mcu::Mcu); others are found by path
pub fn register_class<T: 'static>(module: &'static str, class: &'static str) {
    TABLE.with(|t| t.borrow_mut().classes.insert(TypeId::of::<T>(), (module, class)));
}

/// A printer object type that kaliru also hands out as a plain Rc<T>: its
/// python object stays usable where kaliru wants the printer object
pub fn register_pobj<T: crate::printer::PrinterObject + 'static>() {
    fn cast<T: crate::printer::PrinterObject + 'static>(a: Rc<dyn Any>) -> Option<PObj> {
        a.downcast::<T>().ok().map(|t| t as PObj)
    }
    TABLE.with(|t| t.borrow_mut().pobj_casts.insert(TypeId::of::<T>(), cast::<T>));
}

/// The Kalico class of a Rust type: registered, else by its path
/// (kaliru::extras::heaters::Heater is klippy.extras.heaters.Heater)
fn class_of(tid: TypeId, type_name: &str) -> KResult<PyRef> {
    if let Some((module, class)) = TABLE.with(|t| t.borrow().classes.get(&tid).copied()) {
        return lookup(module, class);
    }
    let path = type_name.split('<').next().unwrap_or(type_name);
    let mut parts: Vec<&str> = path.split("::").collect();
    let class = parts.pop().unwrap_or("");
    if parts.first() == Some(&"kaliru") && parts.len() > 1 {
        parts[0] = "klippy";
        if let Ok(c) = lookup(&parts.join("."), class) {
            return Ok(c);
        }
    }
    Err(KError::Py(
        "TypeError".into(),
        format!("kaliru: {} has no Kalico class yet (plugins/objects.rs register_class)", type_name),
    ))
}

/// The python object of a Rust object (created on first use)
pub fn wrap<T: 'static>(rust: Rc<T>) -> KResult<PyRef> {
    wrap_impl(rust, std::any::type_name::<T>(), None)
}

/// wrap() of an object known by its Rust type's path only
pub fn wrap_any(rust: Rc<dyn Any>, type_name: &str) -> KResult<PyRef> {
    wrap_impl(rust, type_name, None)
}

/// wrap() of a printer object (keeps it usable as one when python hands it back)
pub fn wrap_pobj(pobj: PObj) -> KResult<PyRef> {
    let type_name = pobj.type_name();
    let rust: Rc<dyn Any> = pobj.clone();
    wrap_impl(rust, type_name, Some(pobj))
}

fn wrap_impl(rust: Rc<dyn Any>, type_name: &str, pobj: Option<PObj>) -> KResult<PyRef> {
    let pobj = pobj.or_else(|| {
        let cast = TABLE.with(|t| t.borrow().pobj_casts.get(&(*rust).type_id()).copied());
        cast.and_then(|c| c(rust.clone()))
    });
    if let Some(py) = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let h = *t.by_addr.get(&addr(&rust))?;
        let e = t.entries[h].as_mut()?;
        // first met as a plain object, now known as a printer object too
        if e.pobj.is_none() {
            e.pobj = pobj.clone();
        }
        Some(e.py.clone())
    }) {
        return Ok(py);
    }
    let cls = class_of((*rust).type_id(), type_name)?;
    // cls.__new__(cls): the object exists already, __init__ must not run
    let py = cls.getattr("__new__")?.call(&[&cls])?;
    bind_impl(&py, rust, pobj)?;
    Ok(py)
}

/// Make `py` the python object of `rust` (_k.new)
pub fn bind(py: &PyRef, rust: Rc<dyn Any>) -> KResult<()> {
    bind_impl(py, rust, None)
}

fn bind_impl(py: &PyRef, rust: Rc<dyn Any>, pobj: Option<PObj>) -> KResult<()> {
    let handle = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let h = t.entries.len();
        t.by_addr.insert(addr(&rust), h);
        t.entries.push(Some(Entry { rust, py: py.clone(), pobj }));
        h
    });
    // object.__setattr__: Object.__setattr__ would report a patch
    lookup("builtins", "object")?.getattr("__setattr__")?.call(&[py, &super::py::py_str("_k_h"), &py_int(handle as i64)])?;
    Ok(())
}

/// The Rust object behind a python object of a mirrored class
pub fn unwrap(py: &PyRef) -> KResult<Rc<dyn Any>> {
    let h = match py.getattr("_k_h") {
        Ok(h) => h.as_i64()? as usize,
        Err(_) => {
            return Err(KError::Py(
                "TypeError".into(),
                "kaliru: the object is not bound to a kaliru object (its __init__ did not run)".into(),
            ))
        }
    };
    TABLE.with(|t| t.borrow().entries.get(h).and_then(|e| e.as_ref().map(|e| e.rust.clone())))
        .ok_or_else(|| KError::Py("ReferenceError".into(), "kaliru: the object no longer exists".into()))
}

/// The Rust object of a given type behind a python object
pub fn unwrap_as<T: 'static>(py: &PyRef) -> KResult<Rc<T>> {
    unwrap(py)?.downcast::<T>().map_err(|_| {
        KError::Py("TypeError".into(), format!("kaliru: expected a {}", std::any::type_name::<T>()))
    })
}

/// The printer object behind a python object, if it is a kaliru one
pub fn bound_pobj(py: &PyRef) -> Option<PObj> {
    let h = py.getattr("_k_h").ok()?.as_i64().ok()? as usize;
    TABLE.with(|t| t.borrow().entries.get(h).and_then(|e| e.as_ref().and_then(|e| e.pobj.clone())))
}

/// The python object bound to the kaliru object at this address, if any
pub fn existing_addr(addr: usize) -> Option<PyRef> {
    TABLE.with(|t| {
        let t = t.borrow();
        t.by_addr.get(&addr).and_then(|h| t.entries[*h].as_ref().map(|e| e.py.clone()))
    })
}

/// The python object already bound to a Rust object, if any
pub fn existing(rust: &Rc<dyn Any>) -> Option<PyRef> {
    TABLE.with(|t| {
        let t = t.borrow();
        t.by_addr.get(&addr(rust)).and_then(|h| t.entries[*h].as_ref().map(|e| e.py.clone()))
    })
}

/// Drop every binding (end of a printer instance)
pub fn teardown() {
    let entries = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        t.by_addr.clear();
        std::mem::take(&mut t.entries)
    });
    drop(entries);
}
