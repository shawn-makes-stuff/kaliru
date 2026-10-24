// klippy/extras/filament_switch_sensor.py: SwitchSensor, RunoutHelper. AFC
// rewires these: it sets runout_helper fields to its own values and
// replaces note_filament_present / _runout_event_handler (kaliru's methods
// dispatch to the replacement: plugins::dispatch).
use super::*;
use crate::extras::filament_switch_sensor::{GcodeSlot, RunoutHelper, SwitchSensor};
use crate::{cell_attr, method, ro_attr};
use crate::plugins::convert::ToPy;
use crate::plugins::py_to_pobj;

fn rh(o: &PyRef) -> KResult<Rc<RunoutHelper>> {
    objects::unwrap_as::<RunoutHelper>(o)
}

pub fn register(r: &mut Registry) {
    r.attr("klippy.extras.filament_switch_sensor.SwitchSensor.runout_helper", |o| {
        objects::unwrap_as::<SwitchSensor>(o)?.runout_helper.clone().to_py()
    }, None);
    method!(r, "klippy.extras.filament_switch_sensor.RunoutHelper", RunoutHelper, note_filament_present(
        h, eventtime: f64, is_filament_present: Option<bool>, force: bool, immediate: bool) =>
        h.note_filament_present(eventtime, is_filament_present, force, immediate));
    method!(r, "klippy.extras.filament_switch_sensor.RunoutHelper", RunoutHelper, get_status(h, eventtime: f64) => h.get_status(eventtime));
    method!(r, "klippy.extras.filament_switch_sensor.RunoutHelper", RunoutHelper, reset_runout_distance_info(h) => h.reset_runout_distance_info());

    // fields
    ro_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.name", RunoutHelper, |h| h.name.clone());
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.smart", RunoutHelper, smart);
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.always_fire_events", RunoutHelper, always_fire_events);
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.runout_distance", RunoutHelper, runout_distance);
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.runout_elapsed", RunoutHelper, runout_elapsed);
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.sensor_enabled", |o| Ok(py_bool(rh(o)?.sensor_enabled.get())), Some(|o, v| {
        rh(o)?.sensor_enabled.set(v.is_true()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.filament_present", |o| Ok(py_bool(rh(o)?.filament_present.get())), Some(|o, v| {
        rh(o)?.filament_present.set(v.is_true()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.runout_pause", |o| Ok(py_bool(rh(o)?.runout_pause.get())), Some(|o, v| {
        rh(o)?.runout_pause.set(v.is_true()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.event_delay", |o| Ok(py_float(rh(o)?.event_delay.get())), Some(|o, v| {
        rh(o)?.event_delay.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.min_event_systime", |o| Ok(py_float(rh(o)?.min_event_systime.get())), Some(|o, v| {
        rh(o)?.min_event_systime.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.runout_gcode", |o| slot_to_py(&rh(o)?.runout_gcode.borrow()), Some(|o, v| {
        *rh(o)?.runout_gcode.borrow_mut() = slot_from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.insert_gcode", |o| slot_to_py(&rh(o)?.insert_gcode.borrow()), Some(|o, v| {
        *rh(o)?.insert_gcode.borrow_mut() = slot_from_py(v)?;
        Ok(())
    }));
}

fn slot_to_py(slot: &Option<GcodeSlot>) -> KResult<PyRef> {
    match slot {
        None => Ok(py_none()),
        Some(GcodeSlot::Template(t)) => t.clone().to_py(),
        Some(GcodeSlot::Plugin(p)) => p.clone().to_py(),
    }
}

fn slot_from_py(v: &PyRef) -> KResult<Option<GcodeSlot>> {
    if v.is_none() {
        return Ok(None);
    }
    if let Ok(t) = objects::unwrap_as::<crate::extras::gcode_macro::Template>(v) {
        return Ok(Some(GcodeSlot::Template(t)));
    }
    Ok(Some(GcodeSlot::Plugin(py_to_pobj(v))))
}
