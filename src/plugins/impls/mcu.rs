// klippy/mcu.py: MCU, the clock and identity helpers a plugin uses
// (AFC's get_extruder_pos: estimated_print_time)
use super::*;
use crate::mcu::Mcu;
use crate::method;
use crate::plugins::convert::ToPy;

pub fn register(r: &mut Registry) {
    method!(r, "klippy.mcu.MCU", Mcu, get_name(m) => m.get_name().to_string());
    method!(r, "klippy.mcu.MCU", Mcu, get_printer(m) => m.get_printer());
    method!(r, "klippy.mcu.MCU", Mcu, is_fileoutput(m) => m.is_fileoutput());
    method!(r, "klippy.mcu.MCU", Mcu, create_oid(m) => m.create_oid());
    method!(r, "klippy.mcu.MCU", Mcu, seconds_to_clock(m, time: f64) => m.seconds_to_clock(time));
    method!(r, "klippy.mcu.MCU", Mcu, min_schedule_time(m) => m.min_schedule_time());
    method!(r, "klippy.mcu.MCU", Mcu, max_nominal_duration(m) => m.max_nominal_duration());
    method!(r, "klippy.mcu.MCU", Mcu, get_constant_float(m, name: String) => m.get_constant_float(&name));
    method!(r, "klippy.mcu.MCU", Mcu, print_time_to_clock(m, print_time: f64) => m.print_time_to_clock(print_time));
    method!(r, "klippy.mcu.MCU", Mcu, clock_to_print_time(m, clock: f64) => m.clock_to_print_time(clock));
    method!(r, "klippy.mcu.MCU", Mcu, estimated_print_time(m, eventtime: f64) => m.estimated_print_time(eventtime));
    method!(r, "klippy.mcu.MCU", Mcu, clock32_to_clock64(m, clock32: i64) => m.clock32_to_clock64(clock32));
}
