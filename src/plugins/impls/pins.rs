// klippy/pins.py: PrinterPins; a python plugin's pin chip (register_chip)
use super::*;
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;

use crate::pins::{PinChip, PinParams, PinsObject, SharedPinParams};
use indexmap::IndexMap;

pub fn register(r: &mut Registry) {
    objects::register_class::<PinsObject>("klippy.pins", "PrinterPins");
    objects::register_class::<crate::mcu::Mcu>("klippy.mcu", "MCU");
    r.method("klippy.pins.PrinterPins.register_chip", |a| {
        let chip: Rc<dyn PinChip> = Rc::new(PyPinChip { obj: a.arg(1).clone() });
        a.this::<PinsObject>()?.0.borrow_mut().register_chip(&a.string(0)?, chip).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
    // parse_pin(pin_desc, can_invert=False, can_pullup=False)
    r.method("klippy.pins.PrinterPins.parse_pin", |a| {
        let p = a.this::<PinsObject>()?.0.borrow().parse_pin(&a.string(0)?, a.bool(1)?, a.bool(2)?).map_err(|e| KError::Pins(e.0))?;
        pin_params_dict(&p)
    });
    // lookup_pin(pin_desc, can_invert=False, can_pullup=False, share_type=None)
    r.method("klippy.pins.PrinterPins.lookup_pin", |a| {
        let shared: SharedPinParams = a
            .this::<PinsObject>()?
            .0
            .borrow_mut()
            .lookup_pin(&a.string(0)?, a.bool(1)?, a.bool(2)?, a.opt_string(3)?.as_deref())
            .map_err(|e| KError::Pins(e.0))?;
        let p = shared.borrow();
        pin_params_dict(&p)
    });
    r.method("klippy.pins.PrinterPins.allow_multi_use_pin", |a| {
        a.this::<PinsObject>()?.0.borrow_mut().allow_multi_use_pin(&a.string(0)?).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
}

/// The pin_params dict python gets (a copy: kaliru keeps the shared one)
fn pin_params_dict(p: &PinParams) -> KResult<PyRef> {
    let mut d: IndexMap<String, PyVal> = IndexMap::new();
    d.insert("chip_name".into(), PyVal::Str(p.chip_name.clone()));
    d.insert("pin".into(), PyVal::Str(p.pin.clone()));
    d.insert("invert".into(), PyVal::Int(p.invert));
    d.insert("pullup".into(), PyVal::Int(p.pullup));
    if let Some(s) = &p.share_type {
        d.insert("share_type".into(), PyVal::Str(s.clone()));
    }
    let dict = from_pyval(&PyVal::Dict(d));
    dict.call_method("__setitem__", &[&py_str("chip"), &chip_object(p)?])?;
    Ok(dict)
}

/// pin_params["chip"]: the plugin's own object, or kaliru's MCU
fn chip_object(p: &PinParams) -> KResult<PyRef> {
    if let Some(o) = p.chip.python_object() {
        if let Ok(py) = o.downcast::<PyRef>() {
            return Ok((*py).clone());
        }
    }
    let name = if p.chip_name == "mcu" { "mcu".to_string() } else { format!("mcu {}", p.chip_name) };
    match printer().and_then(|pr| pr.lookup_object_opt(&name)) {
        Some(o) => o.to_py(),
        None => Err(KError::Py(
            "NotImplementedError".into(),
            format!("kaliru: pin_params['chip'] of chip '{}' is not mirrored yet", p.chip_name),
        )),
    }
}

/// A python plugin's pin chip (pins.register_chip(name, obj))
struct PyPinChip {
    obj: PyRef,
}

impl PinChip for PyPinChip {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn std::any::Any>, String> {
        let params = pin_params_dict(&pin_params.borrow()).map_err(|e| e.to_string())?;
        let pin = self.obj.call_method("setup_pin", &[&py_str(pin_type), &params]).map_err(|e| e.to_string())?;
        Ok(Rc::new(pin))
    }

    fn register_config_callback(&self, cb: Rc<dyn Fn() -> KResult<()>>) -> Option<KResult<()>> {
        let f = py_function(Rc::new(move |_a: &[PyRef]| {
            cb()?;
            Ok(py_none())
        }));
        Some(f.and_then(|f| self.obj.call_method("register_config_callback", &[&f])).map(|_| ()))
    }

    fn python_object(&self) -> Option<Rc<dyn std::any::Any>> {
        Some(Rc::new(self.obj.clone()))
    }
}

