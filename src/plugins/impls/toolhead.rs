// klippy/toolhead.py: ToolHead
use super::*;
use crate::kinematics::extruder::PrinterExtruder;
use crate::plugins::convert::{FromPy, ToPy};
use crate::kinematics::Kinematics;
use crate::toolhead::{ExtraAxisRef, ToolHead};
use crate::{cell_attr, method, ro_attr};

/// The python object of an extra axis: the printer object of that name
/// (extruders), so it is the object lookup_object() returns
fn extra_axis_to_py(ea: &ExtraAxisRef) -> KResult<PyRef> {
    let name = ea.get_name();
    match printer().and_then(|p| p.lookup_object_opt(&name)) {
        Some(o) => o.to_py(),
        None => Err(KError::Py("NotImplementedError".into(), format!("kaliru: extra axis '{}' as a python object", name))),
    }
}

/// An extruder python hands back (set_extruder): kaliru's own
fn extra_axis_from_py(o: &PyRef) -> KResult<ExtraAxisRef> {
    let pobj: PObj = FromPy::from_py(o)?;
    match crate::printer::downcast_rc::<PrinterExtruder>(pobj) {
        Some(e) => Ok(e),
        None => Err(KError::Py("NotImplementedError".into(), "kaliru: a python plugin's extra axis".into())),
    }
}

/// A kinematics object as python sees it (one per kinematics, kept for `is`)
pub struct KinRef(pub Rc<dyn Kinematics>);

static KINS: crate::plugins::gil::Gil<std::cell::RefCell<std::collections::HashMap<usize, Rc<KinRef>>>> =
    crate::plugins::gil::Gil::new(|| std::cell::RefCell::new(std::collections::HashMap::new()));

fn kin_to_py(kin: Rc<dyn Kinematics>) -> KResult<PyRef> {
    let key = Rc::as_ptr(&kin) as *const () as usize;
    let k = KINS.with(|m| m.borrow_mut().entry(key).or_insert_with(|| Rc::new(KinRef(kin.clone()))).clone());
    let type_name = k.0.type_name();
    objects::wrap_any(k, type_name)
}

pub fn register(r: &mut Registry) {
    r.method("klippy.toolhead.ToolHead.get_kinematics", |a| kin_to_py(a.this::<ToolHead>()?.get_kinematics()));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, get_position(t) => t.get_position());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, set_position(t, newpos: Vec<f64>, homing_axes: String) => t.set_position(&newpos, &homing_axes));
    // move(newpos, speed) (`move` is a Rust keyword: registered by name)
    r.method("klippy.toolhead.ToolHead.move", |a| {
        let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        a.this::<ToolHead>()?.move_to(&newpos, a.f64(1)?)?;
        Ok(py_none())
    });
    method!(r, "klippy.toolhead.ToolHead", ToolHead, manual_move(t, coord: Vec<Option<f64>>, speed: f64) => t.manual_move(&coord, speed));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, dwell(t, delay: f64) => t.dwell(delay));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, wait_moves(t) => t.wait_moves());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, get_last_move_time(t) => t.get_last_move_time());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, flush_step_generation(t) => t.flush_step_generation());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, limit_next_junction_speed(t, speed: f64) => t.limit_next_junction_speed(speed));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, check_busy(t, eventtime: f64) => t.check_busy(eventtime));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, get_max_velocity(t) => t.get_max_velocity());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, set_accel(t, accel: f64) => t.set_accel(accel));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, set_max_velocities(
        t, max_velocity: Option<f64>, max_accel: Option<f64>, square_corner_velocity: Option<f64>, min_cruise_ratio: Option<f64>) => {
        let (a, b, c, d) = t.set_max_velocities(max_velocity, max_accel, square_corner_velocity, min_cruise_ratio);
        crate::plugins::py::py_tuple(vec![py_float(a), py_float(b), py_float(c), py_float(d)])
    });
    r.method("klippy.toolhead.ToolHead.get_extruder", |a| extra_axis_to_py(&a.this::<ToolHead>()?.get_extruder()));
    r.method("klippy.toolhead.ToolHead.set_extruder", |a| {
        let ea = extra_axis_from_py(a.arg(0))?;
        a.this::<ToolHead>()?.set_extruder(ea, a.f64(1)?);
        Ok(py_none())
    });
    // register_lookahead_callback(callback): callback(print_time)
    r.method("klippy.toolhead.ToolHead.register_lookahead_callback", |a| {
        let cb = a.arg(0).clone();
        a.this::<ToolHead>()?.register_lookahead_callback(Rc::new(move |pt| {
            cb.call(&[&py_float(pt)])?;
            Ok(())
        }))?;
        Ok(py_none())
    });
    ro_attr!(r, "klippy.toolhead.ToolHead.mcu", ToolHead, |t| t.mcu.clone());
    cell_attr!(r, "klippy.toolhead.ToolHead.print_time", ToolHead, print_time);
    cell_attr!(r, "klippy.toolhead.ToolHead.can_pause", ToolHead, can_pause);
    cell_attr!(r, "klippy.toolhead.ToolHead.check_stall_time", ToolHead, check_stall_time);
    cell_attr!(r, "klippy.toolhead.ToolHead.need_check_pause", ToolHead, need_check_pause);
    cell_attr!(r, "klippy.toolhead.ToolHead.max_velocity", ToolHead, max_velocity);
    cell_attr!(r, "klippy.toolhead.ToolHead.max_accel", ToolHead, max_accel);
    cell_attr!(r, "klippy.toolhead.ToolHead.min_cruise_ratio", ToolHead, min_cruise_ratio);
    cell_attr!(r, "klippy.toolhead.ToolHead.square_corner_velocity", ToolHead, square_corner_velocity);
    ro_attr!(r, "klippy.toolhead.ToolHead.special_queuing_state", ToolHead, |t| t.special_queuing_state.borrow().to_string());
}
