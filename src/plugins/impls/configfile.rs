// klippy/configfile.py: ConfigWrapper runs as Kalico's own code
// (tools/mirror_verbatim.txt) on its attributes; for kaliru's configs those
// are live views of kaliru's parser (prelude.py FileConfig) and access
// tracking (AccessTracking). A ConfigWrapper a plugin builds is plain python
// until kaliru needs it (config_from_py).
use super::*;
use crate::configfile::{AccessTracking, ConfigWrapper};
use crate::configparser::RawConfigParser;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::Args;
use std::cell::RefCell;

type Parser = RefCell<RawConfigParser>;
type Tracking = RefCell<AccessTracking>;

pub fn register(r: &mut Registry) {
    objects::register_class::<Parser>("_k", "FileConfig");
    objects::register_class::<Tracking>("_k", "AccessTracking");
    fn cw(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
        objects::unwrap_as::<ConfigWrapper>(o)
    }
    r.attr("klippy.configfile.ConfigWrapper.printer", |o| cw(o)?.get_printer().to_py(), None);
    r.attr("klippy.configfile.ConfigWrapper.fileconfig", |o| cw(o)?.fileconfig.clone().to_py(), None);
    r.attr("klippy.configfile.ConfigWrapper.access_tracking", |o| cw(o)?.access_tracking.clone().to_py(), None);
    r.attr("klippy.configfile.ConfigWrapper.section", |o| Ok(py_str(cw(o)?.get_name())), None);

    // FileConfig primitives (python names its errors itself, prelude.py)
    fn fc(a: &Args) -> KResult<Rc<Parser>> {
        objects::unwrap_as::<Parser>(a.arg(0))
    }
    r.ext("fc_sections", |a| fc(a)?.borrow().sections().to_py());
    r.ext("fc_has_section", |a| Ok(py_bool(fc(a)?.borrow().has_section(&a.string(1)?))));
    r.ext("fc_add_section", |a| {
        fc(a)?.borrow_mut().add_section(&a.string(1)?)?;
        Ok(py_none())
    });
    r.ext("fc_remove_section", |a| Ok(py_bool(fc(a)?.borrow_mut().remove_section(&a.string(1)?))));
    r.ext("fc_options", |a| Ok(fc(a)?.borrow().options(&a.string(1)?)?.to_py()?));
    r.ext("fc_has_option", |a| Ok(py_bool(fc(a)?.borrow().has_option(&a.string(1)?, &a.string(2)?))));
    r.ext("fc_get", |a| {
        let v = fc(a)?.borrow().get_raw(&a.string(1)?, &a.string(2)?, a.bool(3)?)?;
        Ok(py_str(&v))
    });
    r.ext("fc_set", |a| {
        fc(a)?.borrow_mut().set(&a.string(1)?, &a.string(2)?, &a.string(3)?)?;
        Ok(py_none())
    });
    r.ext("fc_remove_option", |a| Ok(py_bool(fc(a)?.borrow_mut().remove_option(&a.string(1)?, &a.string(2)?)?)));

    // AccessTracking: {(section, option): value}
    fn at(a: &Args) -> KResult<Rc<Tracking>> {
        objects::unwrap_as::<Tracking>(a.arg(0))
    }
    r.ext("at_get", |a| {
        let key: (String, String) = FromPy::from_py(a.arg(1))?;
        match at(a)?.borrow().get(&key) {
            Some(v) => Ok(from_pyval(v)),
            None => Err(KError::Py("KeyError".into(), a.arg(1).repr()?)),
        }
    });
    r.ext("at_set", |a| {
        let key: (String, String) = FromPy::from_py(a.arg(1))?;
        at(a)?.borrow_mut().insert(key, to_pyval(a.arg(2))?);
        Ok(py_none())
    });
    r.ext("at_del", |a| {
        let key: (String, String) = FromPy::from_py(a.arg(1))?;
        match at(a)?.borrow_mut().shift_remove(&key) {
            Some(_) => Ok(py_none()),
            None => Err(KError::Py("KeyError".into(), a.arg(1).repr()?)),
        }
    });
    r.ext("at_keys", |a| at(a)?.borrow().keys().cloned().collect::<Vec<_>>().to_py());
}

/// The kaliru config behind a python ConfigWrapper: kaliru's own, or one a
/// plugin built (its parser copied: kaliru objects read their config when
/// they are created)
pub fn config_from_py(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    if o.getattr("_k_h").map(|h| !h.is_none()).unwrap_or(false) {
        return objects::unwrap_as::<ConfigWrapper>(o);
    }
    let printer: Rc<Printer> = FromPy::from_py(&o.getattr("printer")?)?;
    let pyfc = o.getattr("fileconfig")?;
    let fileconfig = match objects::unwrap_as::<Parser>(&pyfc) {
        Ok(p) => p,
        Err(_) => Rc::new(RefCell::new(copy_parser(&pyfc)?)),
    };
    let access_tracking = match objects::unwrap_as::<Tracking>(&o.getattr("access_tracking")?) {
        Ok(t) => t,
        Err(_) => Rc::new(RefCell::new(AccessTracking::new())),
    };
    let section = o.getattr("section")?.as_string()?;
    Ok(Rc::new(ConfigWrapper::from_parts(&printer, fileconfig, access_tracking, &section)))
}

/// A python RawConfigParser's contents as a kaliru parser (raw values; a
/// None value - set(section, option) - becomes "": kaliru's parser has no
/// valueless options, docs/PLUGINS.md)
fn copy_parser(fc: &PyRef) -> KResult<RawConfigParser> {
    let mut out = ConfigWrapper::empty_fileconfig();
    let sections: Vec<String> = FromPy::from_py(&fc.call_method("sections", &[])?)?;
    for s in sections {
        out.add_section(&s)?;
        let section = fc.getattr("_sections")?.call_method("__getitem__", &[&py_str(&s)])?;
        let items: Vec<(String, Option<String>)> = FromPy::from_py(&section.call_method("items", &[])?)?;
        for (k, v) in items {
            out.set(&s, &k, v.as_deref().unwrap_or(""))?;
        }
    }
    Ok(out)
}
