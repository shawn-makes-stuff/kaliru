// klippy/extras/buttons.py: PrinterButtons (python callbacks)
use super::*;
use crate::extras::buttons::{ButtonCallback, PrinterButtons, PushCallback};
use crate::method;

/// callback(eventtime, state)
fn button_cb(f: PyRef) -> ButtonCallback {
    Rc::new(move |et, state| {
        f.call(&[&py_float(et), &py_int(state)])?;
        Ok(())
    })
}

/// callback(eventtime)
fn push_cb(f: PyRef) -> PushCallback {
    Rc::new(move |et| {
        f.call(&[&py_float(et)])?;
        Ok(())
    })
}

pub fn register(r: &mut Registry) {
    method!(r, "klippy.extras.buttons.PrinterButtons", PrinterButtons, register_buttons(b, pins: Vec<String>, callback: PyRef) => {
        let pins: Vec<&str> = pins.iter().map(|s| s.as_str()).collect();
        b.register_buttons(&pins, button_cb(callback))
    });
    method!(r, "klippy.extras.buttons.PrinterButtons", PrinterButtons, register_button_push(b, pin: String, callback: PyRef) =>
        b.register_button_push(&pin, push_cb(callback)));
    method!(r, "klippy.extras.buttons.PrinterButtons", PrinterButtons, register_adc_button(
        b, pin: String, min_val: f64, max_val: f64, pullup: f64, callback: PyRef) =>
        b.register_adc_button(&pin, min_val, max_val, pullup, button_cb(callback)));
    method!(r, "klippy.extras.buttons.PrinterButtons", PrinterButtons, register_adc_button_push(
        b, pin: String, min_val: f64, max_val: f64, pullup: f64, callback: PyRef) =>
        b.register_adc_button_push(&pin, min_val, max_val, pullup, push_cb(callback)));
    r.method("klippy.extras.buttons.PrinterButtons.register_debounce_button", |a| {
        let config = super::configfile::config_from_py(a.arg(2))?;
        a.this::<PrinterButtons>()?.register_debounce_button(&a.string(0)?, button_cb(a.arg(1).clone()), &config)?;
        Ok(py_none())
    });
    r.method("klippy.extras.buttons.PrinterButtons.register_debounce_adc_button", |a| {
        let config = super::configfile::config_from_py(a.arg(5))?;
        a.this::<PrinterButtons>()?.register_debounce_adc_button(
            &a.string(0)?, a.f64(1)?, a.f64(2)?, a.f64(3)?, button_cb(a.arg(4).clone()), &config)?;
        Ok(py_none())
    });
    method!(r, "klippy.extras.buttons.PrinterButtons", PrinterButtons, register_rotary_encoder(
        b, pin1: String, pin2: String, cw: PyRef, ccw: PyRef, steps_per_detent: i64) =>
        b.register_rotary_encoder(&pin1, &pin2, push_cb(cw), push_cb(ccw), steps_per_detent));
}
