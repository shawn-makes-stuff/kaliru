// klippy/gcode.py: GCodeDispatch, GCodeCommand. GCodeCommand runs as
// Kalico's own code (tools/mirror_verbatim.txt) on its attributes, live
// views of kaliru's command; one a plugin builds (create_gcode_command) is
// plain python until kaliru needs it (gcmd_from_py).
use super::*;
use crate::gcode::{GCodeCommand, GCodeDispatch, Handler};
use crate::method;
use crate::plugins::callable::py_function;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::{Args, DictField};
use indexmap::IndexMap;

fn gc(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    objects::unwrap_as::<GCodeCommand>(o)
}

fn gd(o: &PyRef) -> KResult<Rc<GCodeDispatch>> {
    objects::unwrap_as::<GCodeDispatch>(o)
}

pub fn register(r: &mut Registry) {
    // ------------------------------------------------------- GCodeCommand
    r.attr("klippy.gcode.GCodeCommand._command", |o| Ok(py_str(&gc(o)?.get_command())), Some(|o, v| {
        gc(o)?.set_command(&v.as_string()?);
        Ok(())
    }));
    r.attr("klippy.gcode.GCodeCommand._commandline", |o| Ok(py_str(gc(o)?.get_commandline())), None);
    r.attr("klippy.gcode.GCodeCommand._need_ack", |o| Ok(py_bool(gc(o)?.need_ack())), Some(|o, v| {
        gc(o)?.set_need_ack(v.is_true()?);
        Ok(())
    }));
    r.dict(
        "klippy.gcode.GCodeCommand._params",
        DictField {
            get: |o, _f, key| match gc(o)?.params_cell().borrow().get(&key.as_string()?) {
                Some(v) => Ok(py_str(v)),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                gc(o)?.params_cell().borrow_mut().insert(key.as_string()?, value.as_string()?);
                Ok(())
            },
            del: |o, _f, key| match gc(o)?.params_cell().borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(gc(o)?.params_cell().borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
    // self.respond_info = gcode.respond_info (a bound method of the dispatch)
    r.attr("klippy.gcode.GCodeCommand.respond_info", |o| {
        let g = gc(o)?;
        py_function(Rc::new(move |args: &[PyRef]| {
            let log = match args.get(1) {
                Some(l) => l.is_true()?,
                None => true,
            };
            g.respond_info(&args[0].as_string()?, log);
            Ok(py_none())
        }))
    }, None);
    r.attr("klippy.gcode.GCodeCommand.respond_raw", |o| {
        let g = gc(o)?;
        py_function(Rc::new(move |args: &[PyRef]| {
            g.respond_raw(&args[0].as_string()?);
            Ok(py_none())
        }))
    }, None);

    // ------------------------------------------------------ GCodeDispatch
    r.method("klippy.gcode.GCodeDispatch.register_command", register_command);
    r.method("klippy.gcode.GCodeDispatch.register_mux_command", |a| {
        let func = handler_of(a.arg(3).clone());
        a.this::<GCodeDispatch>()?.register_mux_command(&a.string(0)?, &a.string(1)?, a.opt_string(2)?.as_deref(), func, a.opt_string(4)?.as_deref())?;
        Ok(py_none())
    });
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, get_interrupt_counter(g) => g.get_interrupt_counter());
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, increment_interrupt_counter(g) => g.increment_interrupt_counter());
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, is_traditional_gcode(g, cmd: String) => g.is_traditional_gcode(&cmd));
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, get_command_help(g) => {
        let d: IndexMap<String, PyVal> = g.get_command_help().into_iter().map(|(k, v)| (k, PyVal::Str(v))).collect();
        PyVal::Dict(d)
    });
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, run_script_from_command(g, script: String) => g.run_script_from_command(&script));
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, run_script(g, script: String) => g.run_script(&script));
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, get_mutex(g) => g.get_mutex());
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, respond_info(g, msg: String, log: bool) => g.respond_info(&msg, log));
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, respond_raw(g, msg: String) => g.respond_raw(&msg));
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, _respond_error(g, msg: String) => g.respond_error(&msg));
    method!(r, "klippy.gcode.GCodeDispatch", GCodeDispatch, request_restart(g, result: String) => g.request_restart(&result));
    r.attr("klippy.gcode.GCodeDispatch.mutex", |o| gd(o)?.get_mutex().to_py(), None);
    r.dict("klippy.gcode.GCodeDispatch.ready_gcode_handlers", handlers_field());
    r.dict("klippy.gcode.GCodeDispatch.base_gcode_handlers", handlers_field());
    r.dict("klippy.gcode.GCodeDispatch.gcode_handlers", handlers_field());

    // register_output_handler(cb): the python callable is kept as its origin
    r.method("klippy.gcode.GCodeDispatch.register_output_handler", |a| {
        let cb = a.arg(0).clone();
        let f = cb.clone();
        let call: crate::gcode::OutputCallback = Rc::new(move |msg: &str| {
            if let Err(e) = f.call(&[&py_str(msg)]) {
                crate::logging::exception_err("python output handler", &e);
            }
        });
        a.this::<GCodeDispatch>()?.register_output_handler_from(call, (Rc::new(cb), "python"));
        Ok(py_none())
    });
    // output_callbacks: python callables; kaliru's are bound methods of
    // their objects (plugins check cb.__self__)
    r.attr("klippy.gcode.GCodeDispatch.output_callbacks", |o| {
        let mut out = vec![];
        for (cb, origin) in gd(o)?.output_callbacks() {
            let f = py_function(Rc::new(move |args: &[PyRef]| {
                cb(&args[args.len() - 1].as_string()?);
                Ok(py_none())
            }))?;
            out.push(match origin {
                Some((obj, _)) if obj.is::<PyRef>() => (*obj.downcast::<PyRef>().unwrap()).clone(),
                Some((obj, type_name)) => {
                    let owner = objects::wrap_any(obj, type_name)?;
                    lookup("types", "MethodType")?.call(&[&f, &owner])?
                }
                None => f,
            });
        }
        Ok(py_list(out))
    }, None);
}

/// base / ready / the active handler dict, by the field's name
fn which_of(field: &str) -> &'static str {
    if field.ends_with(".ready_gcode_handlers") {
        "ready"
    } else if field.ends_with(".base_gcode_handlers") {
        "base"
    } else {
        "active"
    }
}

fn handlers_field() -> DictField {
    DictField {
        get: |o, f, key| match gd(o)?.handler_in(which_of(f), &key.as_string()?) {
            Some(h) => py_function(Rc::new(move |args: &[PyRef]| {
                let gcmd = gcmd_from_py(&args[0])?;
                h(&gcmd)?;
                Ok(py_none())
            })),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |_o, f, _key, _value| {
            Err(KError::Py("NotImplementedError".into(), format!("kaliru: assigning into {} (use register_command)", f)))
        },
        del: |_o, f, _key| Err(KError::Py("NotImplementedError".into(), format!("kaliru: deleting from {} (use register_command)", f))),
        keys: |o, f| Ok(gd(o)?.handler_dict_names(which_of(f)).iter().map(|k| py_str(k)).collect()),
    }
}

/// The python object of a command handed to a handler: the command itself
/// while the dispatch runs it, else a copy
pub fn gcmd_to_py(gcmd: &GCodeCommand) -> KResult<PyRef> {
    match crate::running::running(gcmd) {
        Some(rc) => rc.to_py(),
        None => Rc::new(gcmd.clone()).to_py(),
    }
}

/// The kaliru command behind a python GCodeCommand: kaliru's own, or one a
/// plugin built (create_gcode_command)
pub fn gcmd_from_py(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    if o.getattr("_k_h").map(|h| !h.is_none()).unwrap_or(false) {
        return gc(o);
    }
    let gcode = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup::<GCodeDispatch>("gcode")?;
    let params: IndexMap<String, String> = match PyVal::from_py(&o.getattr("_params")?)? {
        PyVal::Dict(d) => d.into_iter().map(|(k, v)| (k, v.str())).collect(),
        _ => IndexMap::new(),
    };
    let cmd = gcode.create_gcode_command(&o.getattr("_command")?.as_string()?, &o.getattr("_commandline")?.as_string()?, params);
    cmd.set_need_ack(o.getattr("_need_ack")?.is_true()?);
    Ok(Rc::new(cmd))
}

/// A python gcode handler as kaliru's
pub fn handler_of(func: PyRef) -> Handler {
    Rc::new(move |gcmd: &GCodeCommand| {
        let g = gcmd_to_py(gcmd)?;
        func.call(&[&g])?;
        Ok(())
    })
}

// register_command(cmd, func, when_not_ready=False, desc=None)
fn register_command(a: &Args) -> KResult<PyRef> {
    let g = a.this::<GCodeDispatch>()?;
    let func = if a.arg(1).is_none() { None } else { Some(handler_of(a.arg(1).clone())) };
    let old = g.register_command(&a.string(0)?, func, a.bool(2)?, a.opt_string(3)?.as_deref())?;
    match old {
        None => Ok(py_none()),
        Some(h) => py_function(Rc::new(move |args: &[PyRef]| {
            let gcmd = gcmd_from_py(&args[0])?;
            h(&gcmd)?;
            Ok(py_none())
        })),
    }
}
