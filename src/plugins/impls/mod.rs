// What each mirrored Kalico method, constructor and attribute does in
// kaliru, one file per klippy module. Registered by qualified name
// (native::Registry); argument order is Kalico's signature with its
// defaults filled in by the generated python.
mod accel;
mod buttons;
mod configfile;
mod extras_misc;
mod extras_state;
mod extruder;
mod filament_switch_sensor;
mod gcode;
mod heaters;
mod mcu;
mod pins;
mod printer;
mod reactor;
pub(crate) mod toolhead;
mod webhooks;

use crate::plugins::gil::Gil;
use super::native::Registry;
use super::py::*;
use super::{objects, pobj_to_py};
use crate::error::{KError, KResult};
use crate::printer::{PObj, Printer};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

pub fn register(r: &mut Registry) {
    printer::register(r);
    accel::register(r);
    buttons::register(r);
    extras_misc::register(r);
    extras_state::register(r);
    extruder::register(r);
    configfile::register(r);
    filament_switch_sensor::register(r);
    gcode::register(r);
    heaters::register(r);
    mcu::register(r);
    pins::register(r);
    reactor::register(r);
    toolhead::register(r);
    webhooks::register(r);
}

/// get_status / stats / get_temp of any printer object (kaliru's
/// PrinterObject methods), for every Kalico class that has them
pub fn generic_methods(name: &str) -> Option<super::native::Method> {
    if name.ends_with(".get_status") {
        Some(|a| {
            let o = pobj_of(a)?;
            match o.try_get_status(a.f64(0)?)? {
                Some(v) => Ok(from_pyval(&v)),
                None => Err(KError::Py("AttributeError".into(), "get_status".into())),
            }
        })
    } else if name.ends_with(".stats") {
        Some(|a| {
            let o = pobj_of(a)?;
            match o.stats(a.f64(0)?) {
                Some((busy, msg)) => Ok(py_tuple(vec![py_bool(busy), py_str(&msg)])),
                None => Err(KError::Py("AttributeError".into(), "stats".into())),
            }
        })
    } else if name.ends_with(".get_temp") {
        Some(|a| {
            let o = pobj_of(a)?;
            match o.get_temp(a.f64(0)?) {
                Some((cur, target)) => Ok(py_tuple(vec![py_float(cur), py_float(target)])),
                None => Err(KError::Py("AttributeError".into(), "get_temp".into())),
            }
        })
    } else {
        None
    }
}

fn pobj_of(a: &super::native::Args) -> KResult<PObj> {
    let slf = a.slf.as_ref().expect("method without self");
    // a kinematics object: its own get_status
    if let Ok(k) = objects::unwrap_as::<toolhead::KinRef>(slf) {
        return Ok(Rc::new(KinStatus(k)));
    }
    objects::bound_pobj(slf).ok_or_else(|| KError::Py("NotImplementedError".into(), "kaliru: not a printer object".into()))
}

// the printer of the running instance (callbacks report through it)
static PRINTER: Gil<RefCell<Weak<Printer>>> = Gil::new(|| RefCell::new(Weak::new()));

pub fn set_printer(p: &Rc<Printer>) {
    PRINTER.with(|c| *c.borrow_mut() = Rc::downgrade(p));
}

pub fn printer() -> Option<Rc<Printer>> {
    PRINTER.with(|c| c.borrow().upgrade())
}

/// A python callback failed where Kalico would leave reactor.run()
pub fn report_unhandled(e: &KError) {
    if let Some(p) = printer() {
        p.unhandled_exception(e);
    }
}

/// get_status of a kinematics object, through the printer object interface
struct KinStatus(Rc<toolhead::KinRef>);

impl crate::printer::PrinterObject for KinStatus {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(self.0 .0.get_status(eventtime)))
    }
}

/// An event / callback argument as python sees it
pub fn any_to_py(a: &dyn Any) -> KResult<PyRef> {
    if let Some(v) = a.downcast_ref::<PyRef>() {
        return Ok(v.clone());
    }
    if let Some(v) = a.downcast_ref::<f64>() {
        return Ok(py_float(*v));
    }
    if let Some(v) = a.downcast_ref::<i64>() {
        return Ok(py_int(*v));
    }
    if let Some(v) = a.downcast_ref::<bool>() {
        return Ok(py_bool(*v));
    }
    if let Some(v) = a.downcast_ref::<String>() {
        return Ok(py_str(v));
    }
    if let Some(v) = a.downcast_ref::<&str>() {
        return Ok(py_str(v));
    }
    if let Some(v) = a.downcast_ref::<PyVal>() {
        return Ok(from_pyval(v));
    }
    if let Some(v) = a.downcast_ref::<PObj>() {
        return pobj_to_py(v);
    }
    if let Some(v) = a.downcast_ref::<Option<bool>>() {
        return Ok(v.map(py_bool).unwrap_or_else(py_none));
    }
    if let Some(v) = a.downcast_ref::<Option<f64>>() {
        return Ok(v.map(py_float).unwrap_or_else(py_none));
    }
    if let Some(v) = a.downcast_ref::<Option<i64>>() {
        return Ok(v.map(py_int).unwrap_or_else(py_none));
    }
    if let Some(v) = a.downcast_ref::<Option<String>>() {
        return Ok(v.as_deref().map(py_str).unwrap_or_else(py_none));
    }
    Err(KError::Py("TypeError".into(), format!("kaliru: event argument of type {:?} is not mirrored yet", a.type_id())))
}
