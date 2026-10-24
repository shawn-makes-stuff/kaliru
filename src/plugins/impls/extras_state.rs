// Fields of klippy/extras objects plugins read and write: gcode_move,
// idle_timeout, heaters.Heater
use super::*;
use crate::extras::gcode_move::GCodeMove;
use crate::extras::heaters::Heater;
use crate::extras::idle_timeout::IdleTimeout;
use crate::plugins::callable::py_function;
use crate::plugins::convert::FromPy;
use crate::{cell_attr, method, ro_attr, vec_attr};

pub fn register(r: &mut Registry) {
    // gcode_move.GCodeMove
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.absolute_coord", GCodeMove, absolute_coord);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.absolute_extrude", GCodeMove, absolute_extrude);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.speed", GCodeMove, speed);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.speed_factor", GCodeMove, speed_factor);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.extrude_factor", GCodeMove, extrude_factor);
    vec_attr!(r, "klippy.extras.gcode_move.GCodeMove.base_position", GCodeMove, base_position);
    vec_attr!(r, "klippy.extras.gcode_move.GCodeMove.last_position", GCodeMove, last_position);
    vec_attr!(r, "klippy.extras.gcode_move.GCodeMove.homing_position", GCodeMove, homing_position);
    // self.move_with_transform / position_with_transform: the transform's methods
    r.attr("klippy.extras.gcode_move.GCodeMove.move_with_transform", |o| {
        let g = objects::unwrap_as::<GCodeMove>(o)?;
        py_function(Rc::new(move |a: &[PyRef]| {
            let pos: Vec<f64> = FromPy::from_py(&a[0])?;
            g.move_with_transform(&pos, a[1].as_f64()?)?;
            Ok(py_none())
        }))
    }, None);
    r.attr("klippy.extras.gcode_move.GCodeMove.position_with_transform", |o| {
        let g = objects::unwrap_as::<GCodeMove>(o)?;
        py_function(Rc::new(move |_a: &[PyRef]| crate::plugins::convert::ToPy::to_py(g.position_with_transform())))
    }, None);

    // idle_timeout.IdleTimeout
    cell_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.idle_timeout", IdleTimeout, idle_timeout);
    cell_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.last_print_start_systime", IdleTimeout, last_print_start_systime);
    ro_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.state", IdleTimeout, |s| s.state.borrow().to_string());

    // heaters.Heater
    ro_attr!(r, "klippy.extras.heaters.Heater.name", Heater, |h| h.name.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.short_name", Heater, |h| h.short_name.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.min_temp", Heater, |h| h.min_temp);
    ro_attr!(r, "klippy.extras.heaters.Heater.max_temp", Heater, |h| h.max_temp);
    cell_attr!(r, "klippy.extras.heaters.Heater.min_extrude_temp", Heater, min_extrude_temp);
    cell_attr!(r, "klippy.extras.heaters.Heater.can_extrude", Heater, can_extrude);
    ro_attr!(r, "klippy.extras.heaters.Heater.last_temp", Heater, |h| h.last_temp.get());
    ro_attr!(r, "klippy.extras.heaters.Heater.smoothed_temp", Heater, |h| h.smoothed_temp.get());
    ro_attr!(r, "klippy.extras.heaters.Heater.target_temp", Heater, |h| h.target_temp.get());
    method!(r, "klippy.extras.heaters.Heater", Heater, get_name(h) => h.get_name().to_string());
    method!(r, "klippy.extras.heaters.Heater", Heater, set_temp(h, degrees: f64) => h.set_temp(degrees));
    method!(r, "klippy.extras.heaters.Heater", Heater, check_busy(h, eventtime: f64) => h.check_busy(eventtime));
    method!(r, "klippy.extras.heaters.Heater", Heater, get_max_power(h) => h.get_max_power());
    method!(r, "klippy.extras.heaters.Heater", Heater, get_smooth_time(h) => h.get_smooth_time());
    method!(r, "klippy.extras.heaters.Heater", Heater, get_pwm_delay(h) => h.get_pwm_delay());
    method!(r, "klippy.extras.heaters.Heater", Heater, alter_target(h, target_temp: f64) => h.alter_target(target_temp));
}
