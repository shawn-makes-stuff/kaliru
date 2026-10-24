// Support for button detection and callbacks (port of klippy/extras/buttons.py)
//
// Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::query_adc::QueryADC;
use crate::logging;
use crate::mcu::{get_printer_mcu, ints, pint, CommandWrapper, Mcu};
use crate::msgproto::{Params, Val};
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_strip};
use crate::reactor::Reactor;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// callback(eventtime, state)
pub type ButtonCallback = Rc<dyn Fn(f64, i64) -> KResult<()>>;
/// callback(eventtime)
pub type PushCallback = Rc<dyn Fn(f64) -> KResult<()>>;

/// run a button callback from the reactor (python lets its exception
/// escape the reactor; it is logged here)
fn run_callback(reactor: &Reactor, cb: ButtonCallback, eventtime_override: Option<f64>, state: i64) {
    reactor.register_async_callback_local(
        move |et| {
            if let Err(e) = cb(eventtime_override.unwrap_or(et), state) {
                logging::exception_err("Exception in button callback", &e);
            }
        },
        crate::reactor::NOW,
    );
}

// ----------------------------------------------------------------------
// Button state tracking
// ----------------------------------------------------------------------

pub const QUERY_TIME: f64 = 0.002;
pub const RETRANSMIT_COUNT: i64 = 50;

pub struct McuButtons {
    reactor: Rc<Reactor>,
    mcu: Rc<Mcu>,
    pin_list: RefCell<Vec<(String, i64)>>,
    callbacks: RefCell<Vec<(i64, i64, ButtonCallback)>>,
    invert: Cell<i64>,
    last_button: Cell<i64>,
    ack_cmd: RefCell<Option<CommandWrapper>>,
    ack_count: Cell<i64>,
    oid: Cell<i64>,
}

impl McuButtons {
    fn new(printer: &Printer, mcu: &Rc<Mcu>) -> Rc<Self> {
        let b = Rc::new(McuButtons {
            reactor: printer.get_reactor(),
            mcu: mcu.clone(),
            pin_list: RefCell::new(vec![]),
            callbacks: RefCell::new(vec![]),
            invert: Cell::new(0),
            last_button: Cell::new(0),
            ack_cmd: RefCell::new(None),
            ack_count: Cell::new(0),
            oid: Cell::new(0),
        });
        let bb = b.clone();
        mcu.register_config_callback(Rc::new(move || bb.build_config()));
        b
    }

    fn setup_buttons(&self, pins: &[(String, i64, i64)], callback: ButtonCallback) {
        let mut mask = 0;
        let mut pin_list = self.pin_list.borrow_mut();
        let shift = pin_list.len() as i64;
        for (pin, invert, pullup) in pins {
            if *invert != 0 {
                self.invert.set(self.invert.get() | (1 << pin_list.len()));
            }
            mask |= 1 << pin_list.len();
            pin_list.push((pin.clone(), *pullup));
        }
        self.callbacks.borrow_mut().push((mask, shift, callback));
    }

    fn build_config(self: &Rc<Self>) -> KResult<()> {
        let pin_list = self.pin_list.borrow().clone();
        if pin_list.is_empty() {
            return Ok(());
        }
        // The firmware restarts its ack counter on buttons_query, which is
        // also re-sent when a non-critical mcu reconnects after a reset
        self.ack_count.set(0);
        self.last_button.set(0);
        let oid = self.mcu.create_oid();
        self.oid.set(oid);
        self.mcu.add_config_cmd(&format!("config_buttons oid={} button_count={}", oid, pin_list.len()), false, false);
        for (i, (pin, pull_up)) in pin_list.iter().enumerate() {
            self.mcu.add_config_cmd(&format!("buttons_add oid={} pos={} pin={} pull_up={}", oid, i, pin, pull_up), true, false);
        }
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.ack_cmd.borrow_mut() = Some(self.mcu.lookup_command("buttons_ack oid=%c count=%c", Some(cmd_queue))?);
        let clock = self.mcu.get_query_slot(oid);
        let rest_ticks = self.mcu.seconds_to_clock(QUERY_TIME);
        self.mcu.add_config_cmd(
            &format!(
                "buttons_query oid={} clock={} rest_ticks={} retransmit_count={} invert={}",
                oid,
                clock,
                rest_ticks,
                RETRANSMIT_COUNT,
                self.invert.get()
            ),
            true,
            false,
        );
        let b = self.clone();
        self.mcu.register_response(Some(Rc::new(move |p: &Params| b.handle_buttons_state(p))), "buttons_state", Some(oid));
        Ok(())
    }

    fn handle_buttons_state(&self, params: &Params) -> KResult<()> {
        // Expand the message ack_count from 8-bit
        let ack_count = self.ack_count.get();
        let mut ack_diff = (pint(params, "ack_count") - ack_count) & 0xFF;
        ack_diff -= (ack_diff & 0x80) << 1;
        let msg_ack_count = ack_count + ack_diff;
        // Determine new buttons
        let buttons = match params.get("state") {
            Some(Val::Bytes(b)) => b.clone(),
            _ => vec![],
        };
        let new_count = msg_ack_count + buttons.len() as i64 - self.ack_count.get();
        if new_count <= 0 {
            return Ok(());
        }
        let start = buttons.len().saturating_sub(new_count as usize);
        let new_buttons = buttons[start..].to_vec();
        // Send ack to MCU
        if let Some(cmd) = self.ack_cmd.borrow().as_ref() {
            cmd.send(&ints(&[self.oid.get(), new_count]), 0, 0)?;
        }
        self.ack_count.set(self.ack_count.get() + new_count);
        // Invoke callbacks with this event in main thread
        let btime = match params.get("#receive_time") {
            Some(Val::Float(f)) => *f,
            _ => 0.0,
        };
        for button in new_buttons {
            let button = button as i64 ^ self.invert.get();
            let changed = button ^ self.last_button.get();
            self.last_button.set(button);
            for (mask, shift, callback) in self.callbacks.borrow().iter() {
                if changed & mask != 0 {
                    let state = (button & mask) >> shift;
                    run_callback(&self.reactor, callback.clone(), Some(btime), state);
                }
            }
        }
        Ok(())
    }
}

// ----------------------------------------------------------------------
// ADC button tracking
// ----------------------------------------------------------------------

pub const ADC_REPORT_TIME: f64 = 0.015;
pub const ADC_DEBOUNCE_TIME: f64 = 0.025;
pub const ADC_SAMPLE_TIME: f64 = 0.001;
pub const ADC_SAMPLE_COUNT: i64 = 6;

pub struct McuAdcButtons {
    reactor: Rc<Reactor>,
    buttons: RefCell<Vec<(f64, f64, ButtonCallback)>>,
    last_button: Cell<Option<usize>>,
    last_pressed: Cell<Option<usize>>,
    last_debouncetime: Cell<f64>,
    pullup: f64,
    min_value: Cell<f64>,
    max_value: Cell<f64>,
    _mcu_adc: Rc<dyn crate::mcu::AdcPin>,
}

impl McuAdcButtons {
    fn new(printer: &Rc<Printer>, pin: &str, pullup: f64) -> KResult<Rc<Self>> {
        let mcu_adc = crate::pins::setup_adc_pin(printer, pin)?;
        mcu_adc.setup_adc_sample(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT, 0.0, 1.0, 0);
        let b = Rc::new(McuAdcButtons {
            reactor: printer.get_reactor(),
            buttons: RefCell::new(vec![]),
            last_button: Cell::new(None),
            last_pressed: Cell::new(None),
            last_debouncetime: Cell::new(0.0),
            pullup,
            min_value: Cell::new(999999999999.9),
            max_value: Cell::new(0.0),
            _mcu_adc: mcu_adc.clone(),
        });
        let w = Rc::downgrade(&b);
        mcu_adc.setup_adc_callback(
            ADC_REPORT_TIME,
            Rc::new(move |read_time, read_value| {
                if let Some(b) = w.upgrade() {
                    b.adc_callback(read_time, read_value);
                }
                Ok(())
            }),
        );
        let query_adc = printer.lookup::<QueryADC>("query_adc")?;
        query_adc.register_adc(&format!("adc_button:{}", py_strip(pin)), mcu_adc);
        Ok(b)
    }

    fn setup_button(&self, min_value: f64, max_value: f64, callback: ButtonCallback) {
        self.min_value.set(py_min(self.min_value.get(), min_value));
        self.max_value.set(py_max(self.max_value.get(), max_value));
        self.buttons.borrow_mut().push((min_value, max_value, callback));
    }

    fn adc_callback(&self, read_time: f64, read_value: f64) {
        let adc = py_max(0.00001, py_min(0.99999, read_value));
        let value = self.pullup * adc / (1.0 - adc);

        // Determine button pressed
        let mut btn = None;
        if self.min_value.get() <= value && value <= self.max_value.get() {
            for (i, (min_value, max_value, _cb)) in self.buttons.borrow().iter().enumerate() {
                if *min_value < value && value < *max_value {
                    btn = Some(i);
                    break;
                }
            }
        }

        // If the button changed, due to noise or pressing:
        if btn != self.last_button.get() {
            // reset the debouncing timer
            self.last_debouncetime.set(read_time);
        }

        // button debounce check & new button pressed
        if (read_time - self.last_debouncetime.get()) >= ADC_DEBOUNCE_TIME
            && self.last_button.get() == btn
            && self.last_pressed.get() != btn
        {
            // release last_pressed
            if let Some(lp) = self.last_pressed.get() {
                self.call_button(lp, false);
                self.last_pressed.set(None);
            }
            if let Some(b) = btn {
                self.call_button(b, true);
                self.last_pressed.set(Some(b));
            }
        }

        self.last_button.set(btn);
    }

    fn call_button(&self, button: usize, state: bool) {
        let callback = self.buttons.borrow()[button].2.clone();
        run_callback(&self.reactor, callback, None, state as i64);
    }
}

// ----------------------------------------------------------------------
// Rotary Encoders
// ----------------------------------------------------------------------

// Rotary encoder handler https://github.com/brianlow/Rotary
// Copyright 2011 Ben Buxton (bb@cactii.net).
// Licenced under the GNU GPL Version 3.
const R_START: usize = 0x0;
const R_DIR_CW: usize = 0x10;
const R_DIR_CCW: usize = 0x20;
const R_DIR_MSK: usize = 0x30;

mod full_step {
    use super::{R_DIR_CCW, R_DIR_CW, R_START};
    const R_CW_FINAL: usize = 0x1;
    const R_CW_BEGIN: usize = 0x2;
    const R_CW_NEXT: usize = 0x3;
    const R_CCW_BEGIN: usize = 0x4;
    const R_CCW_FINAL: usize = 0x5;
    const R_CCW_NEXT: usize = 0x6;

    // Use the full-step state table (emits a code at 00 only)
    pub const ENCODER_STATES: &[[usize; 4]] = &[
        // R_START
        [R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START],
        // R_CW_FINAL
        [R_CW_NEXT, R_START, R_CW_FINAL, R_START | R_DIR_CW],
        // R_CW_BEGIN
        [R_CW_NEXT, R_CW_BEGIN, R_START, R_START],
        // R_CW_NEXT
        [R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START],
        // R_CCW_BEGIN
        [R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START],
        // R_CCW_FINAL
        [R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | R_DIR_CCW],
        // R_CCW_NEXT
        [R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START],
    ];
}

mod half_step {
    use super::{R_DIR_CCW, R_DIR_CW, R_START};
    // Use the half-step state table (emits a code at 00 and 11)
    const R_CCW_BEGIN: usize = 0x1;
    const R_CW_BEGIN: usize = 0x2;
    const R_START_M: usize = 0x3;
    const R_CW_BEGIN_M: usize = 0x4;
    const R_CCW_BEGIN_M: usize = 0x5;

    pub const ENCODER_STATES: &[[usize; 4]] = &[
        // R_START (00)
        [R_START_M, R_CW_BEGIN, R_CCW_BEGIN, R_START],
        // R_CCW_BEGIN
        [R_START_M | R_DIR_CCW, R_START, R_CCW_BEGIN, R_START],
        // R_CW_BEGIN
        [R_START_M | R_DIR_CW, R_CW_BEGIN, R_START, R_START],
        // R_START_M (11)
        [R_START_M, R_CCW_BEGIN_M, R_CW_BEGIN_M, R_START],
        // R_CW_BEGIN_M
        [R_START_M, R_START_M, R_CW_BEGIN_M, R_START | R_DIR_CW],
        // R_CCW_BEGIN_M
        [R_START_M, R_CCW_BEGIN_M, R_START_M, R_START | R_DIR_CCW],
    ];
}

struct RotaryEncoder {
    states: &'static [[usize; 4]],
    cw_callback: PushCallback,
    ccw_callback: PushCallback,
    encoder_state: Cell<usize>,
}

impl RotaryEncoder {
    fn encoder_callback(&self, eventtime: f64, state: i64) -> KResult<()> {
        let row = self.states.get(self.encoder_state.get() & 0xF).ok_or_else(|| KError::Other("tuple index out of range".into()))?;
        let es = row[(state & 0x3) as usize];
        self.encoder_state.set(es);
        if es & R_DIR_MSK == R_DIR_CW {
            (self.cw_callback)(eventtime)?;
        } else if es & R_DIR_MSK == R_DIR_CCW {
            (self.ccw_callback)(eventtime)?;
        }
        Ok(())
    }
}

pub struct DebounceButton {
    reactor: Rc<Reactor>,
    button_action: ButtonCallback,
    debounce_delay: f64,
    logical_state: Cell<Option<i64>>,
    physical_state: Cell<Option<i64>>,
    latest_eventtime: Cell<f64>,
}

impl DebounceButton {
    pub fn new(config: &ConfigWrapper, button_action: ButtonCallback) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let debounce_delay = config.getfloat("debounce_delay", 0.0, Lim::new().min(0.0))?;
        Ok(Rc::new(DebounceButton {
            reactor: printer.get_reactor(),
            button_action,
            debounce_delay,
            logical_state: Cell::new(None),
            physical_state: Cell::new(None),
            latest_eventtime: Cell::new(0.0),
        }))
    }

    fn button_handler(self: &Rc<Self>, eventtime: f64, state: i64) -> KResult<()> {
        self.physical_state.set(Some(state));
        self.latest_eventtime.set(eventtime);
        // if there would be no state transition, ignore the event:
        if self.logical_state.get() == self.physical_state.get() {
            return Ok(());
        }
        let trigger_time = eventtime + self.debounce_delay;
        let d = self.clone();
        self.reactor.register_callback(
            move |et| {
                if let Err(e) = d.debounce_event(et) {
                    logging::exception_err("Exception in debounce callback", &e);
                }
            },
            trigger_time,
        );
        Ok(())
    }

    fn debounce_event(&self, eventtime: f64) -> KResult<()> {
        // if there would be no state transition, ignore the event:
        if self.logical_state.get() == self.physical_state.get() {
            return Ok(());
        }
        // if there were more recent events, they supersede this one:
        if (eventtime - self.debounce_delay) < self.latest_eventtime.get() {
            return Ok(());
        }
        // enact state transition and trigger action
        self.logical_state.set(self.physical_state.get());
        (self.button_action)(self.latest_eventtime.get(), self.logical_state.get().unwrap_or(0))
    }
}

// ----------------------------------------------------------------------
// Button registration code
// ----------------------------------------------------------------------

pub struct PrinterButtons {
    printer: Rc<Printer>,
    mcu_buttons: RefCell<IndexMap<String, Rc<McuButtons>>>,
    adc_buttons: RefCell<IndexMap<String, Rc<McuAdcButtons>>>,
}

impl PrinterButtons {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        printer.load_object(config, "query_adc", true)?;
        Ok(Rc::new(PrinterButtons { printer, mcu_buttons: RefCell::new(IndexMap::new()), adc_buttons: RefCell::new(IndexMap::new()) }))
    }

    pub fn register_adc_button(&self, pin: &str, min_val: f64, max_val: f64, pullup: f64, callback: ButtonCallback) -> KResult<()> {
        let existing = self.adc_buttons.borrow().get(pin).cloned();
        let adc_buttons = match existing {
            Some(b) => b,
            None => {
                let b = McuAdcButtons::new(&self.printer, pin, pullup)?;
                self.adc_buttons.borrow_mut().insert(pin.to_string(), b.clone());
                b
            }
        };
        adc_buttons.setup_button(min_val, max_val, callback);
        Ok(())
    }

    pub fn register_adc_button_push(&self, pin: &str, min_val: f64, max_val: f64, pullup: f64, callback: PushCallback) -> KResult<()> {
        self.register_adc_button(
            pin,
            min_val,
            max_val,
            pullup,
            Rc::new(move |eventtime, state| if state != 0 { callback(eventtime) } else { Ok(()) }),
        )
    }

    pub fn register_debounce_button(&self, pin: &str, callback: ButtonCallback, config: &ConfigWrapper) -> KResult<()> {
        let debounce = DebounceButton::new(config, callback)?;
        self.register_buttons(&[pin], Rc::new(move |et, s| debounce.button_handler(et, s)))
    }

    pub fn register_debounce_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: ButtonCallback,
        config: &ConfigWrapper,
    ) -> KResult<()> {
        let debounce = DebounceButton::new(config, callback)?;
        self.register_adc_button(pin, min_val, max_val, pullup, Rc::new(move |et, s| debounce.button_handler(et, s)))
    }

    pub fn register_buttons(&self, pins: &[&str], callback: ButtonCallback) -> KResult<()> {
        // Parse pins
        let ppins = self.printer.lookup::<PinsObject>("pins")?;
        let mut mcu_name: Option<String> = None;
        let mut pin_params_list = vec![];
        let mut chip = None;
        for pin in pins {
            let pin_params = ppins.0.borrow_mut().lookup_pin(pin, true, true, None).map_err(|e| KError::Pins(e.0))?;
            let p = pin_params.borrow();
            chip = Some(p.chip.clone());
            if mcu_name.as_ref().map(|n| *n != p.chip_name).unwrap_or(false) {
                return Err(KError::Pins("button pins must be on same mcu".into()));
            }
            mcu_name = Some(p.chip_name.clone());
            pin_params_list.push((p.pin.clone(), p.invert, p.pullup));
        }
        let mcu_name = mcu_name.unwrap_or_default();
        // a python plugin's chip: MCU_buttons(printer, chip) only registers
        // its config callback with it
        if let Some(chip) = chip {
            let build_config: Rc<dyn Fn() -> KResult<()>> =
                Rc::new(|| Err(KError::Other("kaliru: button config on a python plugin's chip is not mirrored".into())));
            if let Some(res) = chip.register_config_callback(build_config) {
                return res;
            }
        }
        // Register pins and callback with the appropriate MCU
        let existing = self.mcu_buttons.borrow().get(&mcu_name).cloned();
        let mcu_buttons = match existing {
            Some(b) if b.pin_list.borrow().len() + pin_params_list.len() <= 8 => b,
            _ => {
                let mcu = get_printer_mcu(&self.printer, &mcu_name)?;
                let b = McuButtons::new(&self.printer, &mcu);
                self.mcu_buttons.borrow_mut().insert(mcu_name, b.clone());
                b
            }
        };
        mcu_buttons.setup_buttons(&pin_params_list, callback);
        Ok(())
    }

    pub fn register_rotary_encoder(
        &self,
        pin1: &str,
        pin2: &str,
        cw_callback: PushCallback,
        ccw_callback: PushCallback,
        steps_per_detent: i64,
    ) -> KResult<()> {
        let states = match steps_per_detent {
            2 => half_step::ENCODER_STATES,
            4 => full_step::ENCODER_STATES,
            _ => return Err(KError::config(format!("{} steps per detent not supported", steps_per_detent))),
        };
        let re = Rc::new(RotaryEncoder { states, cw_callback, ccw_callback, encoder_state: Cell::new(R_START) });
        self.register_buttons(&[pin1, pin2], Rc::new(move |et, s| re.encoder_callback(et, s)))
    }

    pub fn register_button_push(&self, pin: &str, callback: PushCallback) -> KResult<()> {
        self.register_buttons(&[pin], Rc::new(move |eventtime, state| if state != 0 { callback(eventtime) } else { Ok(()) }))
    }
}

impl PrinterObject for PrinterButtons {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        self.mcu_buttons.borrow_mut().clear();
        self.adc_buttons.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterButtons::new(config)?)
}
