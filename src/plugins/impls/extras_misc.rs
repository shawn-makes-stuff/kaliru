// klippy/extras: pause_resume.PauseResume, gcode_macro.GCodeMacro
// (variables) / PrinterGCodeMacro / Template / TemplateWrapperJinja,
// stepper_enable.PrinterStepperEnable / EnableTracking
use super::*;
use crate::extras::gcode_macro::{GCodeMacro, PrinterGCodeMacro, Template, TemplateWrapperJinja};
use crate::extras::pause_resume::PauseResume;
use crate::extras::stepper_enable::{EnableTracking, PrinterStepperEnable};
use crate::jinja::value::Value;
use indexmap::IndexMap;
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

    // templates: Template delegates to its function (TemplateWrapperJinja /
    // TemplateWrapperPython) through __getattr__, as in Kalico
    r.method("klippy.extras.gcode_macro.PrinterGCodeMacro.load_template", |a| {
        let config = super::configfile::config_from_py(a.arg(0))?;
        let default = a.opt_string(2)?;
        a.this::<PrinterGCodeMacro>()?.load_template(&config, &a.string(1)?, default.as_deref())?.to_py()
    });
    r.attr("klippy.extras.gcode_macro.Template.function", |o| template_function(o), None);
    r.method("klippy.extras.gcode_macro.Template.__getattr__", |a| template_function(a.slf.as_ref().unwrap())?.getattr(&a.string(0)?));
    method!(r, "klippy.extras.gcode_macro.TemplateWrapperJinja", TemplateWrapperJinja, render(t, context: Option<PyRef>) =>
        t.render(template_context(context)?));
    method!(r, "klippy.extras.gcode_macro.TemplateWrapperJinja", TemplateWrapperJinja, run_gcode_from_command(t, context: Option<PyRef>) =>
        t.run_gcode_from_command(template_context(context)?));
    #[cfg(feature = "python")]
    objects::register_class::<crate::extras::gcode_python::TemplateWrapperPython>("klippy.extras.gcode_macro", "TemplateWrapperPython");
    #[cfg(feature = "python")]
    method!(r, "klippy.extras.gcode_macro.TemplateWrapperPython", crate::extras::gcode_python::TemplateWrapperPython,
        run_gcode_from_command(t, context: Option<PyRef>) => t.run_gcode_from_command(template_context(context)?));

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

fn template_function(o: &PyRef) -> KResult<PyRef> {
    let (f, type_name) = objects::unwrap_as::<Template>(o)?.function();
    objects::wrap_any(f, type_name)
}

/// a render context python passes (a dict), as kaliru's jinja values
fn template_context(context: Option<PyRef>) -> KResult<Option<IndexMap<String, Value>>> {
    let Some(c) = context.filter(|c| !c.is_none()) else { return Ok(None) };
    match to_pyval(&c)? {
        crate::pyval::PyVal::Dict(d) => Ok(Some(d.iter().map(|(k, v)| (k.clone(), Value::from_pyval(v))).collect())),
        _ => Err(KError::Py("TypeError".into(), "kaliru: a template context must be a dict".into())),
    }
}
