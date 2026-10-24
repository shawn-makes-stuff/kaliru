// klippy/kinematics/extruder.py: PrinterExtruder
use super::*;
use crate::kinematics::extruder::PrinterExtruder;
use crate::toolhead::ExtraAxis;
use crate::{cell_attr, method, ro_attr};

pub fn register(r: &mut Registry) {
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, get_name(e) => e.name.clone());
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, get_heater(e) => e.heater.clone());
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, find_past_position(e, print_time: f64) =>
        ExtraAxis::find_past_position(&*e, print_time));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, get_axis_gcode_id(e) => ExtraAxis::get_axis_gcode_id(&*e));
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.name", PrinterExtruder, |e| e.name.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.heater", PrinterExtruder, |e| e.heater.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.extruder_stepper", PrinterExtruder, |e| e.extruder_stepper.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.filament_area", PrinterExtruder, |e| e.filament_area);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.config_max_cross_section", PrinterExtruder, |e| e.config_max_cross_section);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_e_velocity", PrinterExtruder, |e| e.max_e_velocity);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_e_accel", PrinterExtruder, |e| e.max_e_accel);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_e_dist", PrinterExtruder, |e| e.max_e_dist);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.instant_corner_v", PrinterExtruder, |e| e.instant_corner_v);
    cell_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.last_position", PrinterExtruder, last_position);
    cell_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.nozzle_diameter", PrinterExtruder, nozzle_diameter);
    cell_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_extrude_ratio", PrinterExtruder, max_extrude_ratio);
}
