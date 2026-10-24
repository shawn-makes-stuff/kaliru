// klippy/extras: pause_resume.PauseResume, gcode_macro.GCodeMacro
// (variables), stepper_enable.PrinterStepperEnable / EnableTracking
use super::*;
use crate::extras::gcode_macro::GCodeMacro;
use crate::extras::pause_resume::PauseResume;
use crate::extras::stepper_enable::{EnableTracking, PrinterStepperEnable};
use crate::jinja::value::Value;
use crate::method;
use crate::plugins::convert::ToPy;
use crate::plugins::native::DictField;

pub fn register(r: &mut Registry) {
    method!(r, "klippy.extras.pause_resume.PauseResume", PauseResume, send_pause_command(p) => p.send_pause_command());
    method!(r, "klippy.extras.pause_resume.PauseResume", PauseResume, send_resume_command(p) => p.send_resume_command());
    method!(r, "klippy.extras.pause_resume.PauseResume", PauseResume, is_sd_active(p) => p.is_sd_active());
    r.method("klippy.extras.pause_resume.PauseResume.cmd_PAUSE", |a| {
        let g = super::gcode::gcmd_from_py(a.arg(0))?;
        a.this::<PauseResume>()?.cmd_pause(&g)?;
        Ok(py_none())
    });
    r.method("klippy.extras.pause_resume.PauseResume.cmd_RESUME", |a| {
        let g = super::gcode::gcmd_from_py(a.arg(0))?;
        a.this::<PauseResume>()?.cmd_resume(&g)?;
        Ok(py_none())
    });

    // GCodeMacro.variables: {name: value}, values kaliru's jinja values
    r.dict(
        "klippy.extras.gcode_macro.GCodeMacro.variables",
        DictField {
            get: |o, _f, key| match objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow().get(&key.as_string()?) {
                Some(v) => Ok(from_pyval(&v.to_pyval())),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                let v = Value::from_pyval(&to_pyval(value)?);
                objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow_mut().insert(key.as_string()?, v);
                Ok(())
            },
            del: |o, _f, key| match objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );

    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, lookup_enable(s, name: String) => s.lookup_enable(&name));
    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, get_steppers(s) => s.get_steppers());
    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, motor_off(s) => s.motor_off());
    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, set_motors_enable(s, names: Vec<String>, enable: bool) =>
        s.set_motors_enable(&names, enable));
    r.dict(
        "klippy.extras.stepper_enable.PrinterStepperEnable.enable_lines",
        DictField {
            get: |o, _f, key| match objects::unwrap_as::<PrinterStepperEnable>(o)?.enable_lines.borrow().get(&key.as_string()?) {
                Some(e) => e.clone().to_py(),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |_o, f, _k, _v| Err(KError::Py("NotImplementedError".into(), format!("kaliru: assigning into {}", f))),
            del: |_o, f, _k| Err(KError::Py("NotImplementedError".into(), format!("kaliru: deleting from {}", f))),
            keys: |o, _f| Ok(objects::unwrap_as::<PrinterStepperEnable>(o)?.enable_lines.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, motor_enable(e, print_time: f64) => e.motor_enable(print_time));
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, motor_disable(e, print_time: f64) => e.motor_disable(print_time));
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, is_motor_enabled(e) => e.is_motor_enabled());
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, has_dedicated_enable(e) => e.has_dedicated_enable());
}
