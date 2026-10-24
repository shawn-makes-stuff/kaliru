// klippy/webhooks.py: WebHooks, WebRequest
use super::*;
use crate::method;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::Args;
use crate::webhooks::{Ty, WebHooks, WebRequest};
use indexmap::IndexMap;

pub fn register(r: &mut Registry) {
    r.method("klippy.webhooks.WebHooks.register_endpoint", |a| {
        let cb = a.arg(1).clone();
        a.this::<WebHooks>()?.register_endpoint(&a.string(0)?, endpoint_of(cb))?;
        Ok(py_none())
    });
    r.method("klippy.webhooks.WebHooks.register_mux_endpoint", |a| {
        let cb = a.arg(3).clone();
        a.this::<WebHooks>()?.register_mux_endpoint(&a.string(0)?, &a.string(1)?, a.opt_string(2)?.as_deref(), endpoint_of(cb))?;
        Ok(py_none())
    });
    // call_remote_method(method, **kwargs)
    r.method("klippy.webhooks.WebHooks.call_remote_method", |a| {
        let kwargs: PyVal = FromPy::from_py(a.arg(1))?;
        let PyVal::Dict(kw) = kwargs else { return Err(KError::Other("kaliru: kwargs".into())) };
        a.this::<WebHooks>()?.call_remote_method(&a.string(0)?, kw)?;
        Ok(py_none())
    });
    // _remote_methods: {method: {...}}; plugins test membership
    r.attr("klippy.webhooks.WebHooks._remote_methods", |o| {
        let names = objects::unwrap_as::<WebHooks>(o)?.remote_method_names();
        let d: IndexMap<String, PyVal> = names.into_iter().map(|n| (n, PyVal::Dict(IndexMap::new()))).collect();
        Ok(from_pyval(&PyVal::Dict(d)))
    }, None);

    method!(r, "klippy.webhooks.WebRequest", WebRequest, get_method(w) => w.get_method().to_string());
    r.method("klippy.webhooks.WebRequest.get", |a| {
        let types = types_of(a.arg(2))?;
        get_item(a, |w, item, d| w.get(item, d, types.as_deref()))
    });
    r.method("klippy.webhooks.WebRequest.get_str", |a| get_item(a, |w, item, d| w.get_str(item, d)));
    r.method("klippy.webhooks.WebRequest.get_int", |a| get_item(a, |w, item, d| w.get_int(item, d)));
    r.method("klippy.webhooks.WebRequest.get_float", |a| get_item(a, |w, item, d| w.get_float(item, d).map(PyVal::Float)));
    r.method("klippy.webhooks.WebRequest.get_dict", |a| get_item(a, |w, item, d| w.get_dict(item, d)));
    method!(r, "klippy.webhooks.WebRequest", WebRequest, set_error(w, error: PyRef) => w.set_error(&error.str()?));
    method!(r, "klippy.webhooks.WebRequest", WebRequest, send(w, data: PyVal) => w.send(data));
}

/// A python endpoint callback as kaliru's: the request object itself
fn endpoint_of(cb: PyRef) -> crate::webhooks::Endpoint {
    Rc::new(move |w: &WebRequest| {
        let req = match crate::running::running(w) {
            Some(rc) => rc.to_py()?,
            None => return Err(KError::Other("kaliru: web request outside its dispatch".into())),
        };
        cb.call(&[&req])?;
        Ok(())
    })
}

/// get*(item, default=Sentinel): the python default comes back as given
fn get_item(a: &Args, get: impl FnOnce(&WebRequest, &str, Option<PyVal>) -> KResult<PyVal>) -> KResult<PyRef> {
    let w = a.this::<WebRequest>()?;
    let item = a.string(0)?;
    let sentinel = lookup("klippy.webhooks", "Sentinel")?;
    if !w.params.contains_key(&item) && !a.arg(1).is(sentinel.as_ptr()) {
        return Ok(a.arg(1).clone());
    }
    Ok(from_pyval(&get(&w, &item, None)?))
}

/// types=(str, int, ...) as kaliru's Ty list
fn types_of(o: &PyRef) -> KResult<Option<Vec<Ty>>> {
    if o.is_none() {
        return Ok(None);
    }
    let items: Vec<PyRef> = FromPy::from_py(o)?;
    let mut out = vec![];
    for t in items {
        let name = t.getattr("__name__")?.as_string()?;
        out.push(match name.as_str() {
            "str" => Ty::Str,
            "int" => Ty::Int,
            "float" => Ty::Float,
            "dict" => Ty::Dict,
            _ => return Err(KError::Py("NotImplementedError".into(), format!("kaliru: WebRequest.get types={}", name))),
        });
    }
    Ok(Some(out))
}
