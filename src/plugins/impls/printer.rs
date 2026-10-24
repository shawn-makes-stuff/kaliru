// klippy/printer.py: Printer
use super::*;
use crate::plugins::convert::FromPy;
use crate::plugins::native::Args;
use crate::plugins::objects;
use crate::method;

pub fn register(r: &mut Registry) {
    method!(r, "klippy.printer.Printer", Printer, get_reactor(p) => p.get_reactor());
    method!(r, "klippy.printer.Printer", Printer, get_start_args(p) => start_args(&p));
    method!(r, "klippy.printer.Printer", Printer, get_state_message(p) => p.get_state_message());
    method!(r, "klippy.printer.Printer", Printer, is_shutdown(p) => p.is_shutdown());
    method!(r, "klippy.printer.Printer", Printer, update_error_msg(p, old: String, new: String) => p.update_error_msg(&old, &new));
    method!(r, "klippy.printer.Printer", Printer, add_object(p, name: String, obj: PObj) => p.add_object(&name, obj));
    r.method("klippy.printer.Printer.lookup_object", lookup_object);
    method!(r, "klippy.printer.Printer", Printer, lookup_objects(p, module: Option<String>) => p.lookup_objects(module.as_deref()));
    r.method("klippy.printer.Printer.load_object", load_object);
    method!(r, "klippy.printer.Printer", Printer, set_rollover_info(p, name: String, info: Option<String>, log: bool) =>
        p.set_rollover_info(&name, info.as_deref(), log));
    method!(r, "klippy.printer.Printer", Printer, invoke_shutdown(p, msg: String, details: Option<PyVal>) => p.invoke_shutdown(&msg, details));
    method!(r, "klippy.printer.Printer", Printer, invoke_async_shutdown(p, msg: String, details: Option<PyVal>) =>
        p.invoke_async_shutdown(&msg, details));
    r.method("klippy.printer.Printer.register_event_handler", register_event_handler);
    r.method("klippy.printer.Printer.send_event", send_event);
    method!(r, "klippy.printer.Printer", Printer, request_exit(p, result: String) => p.request_exit(&result));
    r.method("klippy.printer.Printer.wait_while", wait_while);
    // attributes
    r.attr("klippy.printer.Printer.reactor", |o| objects::unwrap_as::<Printer>(o)?.get_reactor().to_py(), None);
    r.attr("klippy.printer.Printer.objects", |o| lookup("_k", "ObjectsView")?.call(&[o]), None);
    r.attr("klippy.printer.Printer.start_args", |o| {
        let p = objects::unwrap_as::<Printer>(o)?;
        start_args(&p)
    }, None);
    r.attr("klippy.printer.Printer.state_message", |o| Ok(py_str(&objects::unwrap_as::<Printer>(o)?.get_state_message().0)), None);
    // printer.objects (prelude.py ObjectsView)
    fn pr(a: &Args) -> KResult<Rc<Printer>> {
        objects::unwrap_as::<Printer>(a.arg(0))
    }
    r.ext("objects_get", |a| {
        let key = a.string(1)?;
        match pr(a)?.lookup_object_opt(&key) {
            Some(o) => pobj_to_py(&o),
            None => Err(KError::Py("KeyError".into(), py_str(&key).repr()?)),
        }
    });
    r.ext("objects_set", |a| {
        pr(a)?.objects_set(&a.string(1)?, crate::plugins::py_to_pobj(a.arg(2)));
        Ok(py_none())
    });
    r.ext("objects_del", |a| {
        let key = a.string(1)?;
        match pr(a)?.objects_remove(&key) {
            Some(_) => Ok(py_none()),
            None => Err(KError::Py("KeyError".into(), py_str(&key).repr()?)),
        }
    });
    r.ext("objects_keys", |a| pr(a)?.object_names().to_py());
    r.attr("klippy.printer.Printer.in_shutdown_state", |o| Ok(py_bool(objects::unwrap_as::<Printer>(o)?.is_shutdown())), None);
}

use crate::plugins::convert::ToPy;

/// get_start_args(): the dict (a copy: kaliru keeps its own)
fn start_args(p: &Printer) -> KResult<PyRef> {
    Ok(from_pyval(&PyVal::Dict(p.start_args.borrow().clone())))
}

// lookup_object(name, default=configfile.sentinel)
fn lookup_object(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let name = a.string(0)?;
    match p.lookup_object_opt(&name) {
        Some(o) => pobj_to_py(&o),
        None if a.arg(1).is(lookup("klippy.configfile", "sentinel")?.as_ptr()) => {
            Err(KError::config(format!("Unknown config object '{}'", name)))
        }
        None => Ok(a.arg(1).clone()),
    }
}

// load_object(config, section, default=configfile.sentinel)
fn load_object(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let config = super::configfile::config_from_py(a.arg(0))?;
    let section = a.string(1)?;
    let required = a.arg(2).is(lookup("klippy.configfile", "sentinel")?.as_ptr());
    match p.load_object(&config, &section, required)? {
        Some(o) => pobj_to_py(&o),
        None => Ok(a.arg(2).clone()),
    }
}

// register_event_handler(event, callback)
fn register_event_handler(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let cb = a.arg(1).clone();
    p.register_event_handler(
        &a.string(0)?,
        Rc::new(move |args: &[&dyn Any]| {
            let py_args = args.iter().map(|x| any_to_py(*x)).collect::<KResult<Vec<_>>>()?;
            let refs: Vec<&PyRef> = py_args.iter().collect();
            cb.call(&refs)?;
            Ok(())
        }),
    );
    Ok(py_none())
}

// send_event(event, *params): python handlers get the objects as given
fn send_event(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let params: Vec<PyRef> = Vec::from_py(a.arg(1))?;
    let refs: Vec<&dyn Any> = params.iter().map(|x| x as &dyn Any).collect();
    p.send_event(&a.string(0)?, &refs)?;
    Ok(py_none())
}

// wait_while(condition_cb, error_on_cancel=True, interval=1.0)
fn wait_while(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let cb = a.arg(0).clone();
    p.wait_while(|eventtime| cb.call(&[&py_float(eventtime)])?.is_true(), a.bool(1)?, a.f64(2)?)?;
    Ok(py_none())
}
