// Pin name handling (port of klippy/pins.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::pyfmt::{py_isspace, py_strip};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// pins.error
#[derive(Clone, Debug)]
pub struct Error(pub String);

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

fn error(msg: String) -> Error {
    Error(msg)
}

// pin descriptions may not hold "^~!:" or whitespace
fn invalid_pin_name(pin: &str) -> bool {
    pin.contains(['^', '~', '!', ':']) || pin.chars().any(py_isspace)
}

/// A pin chip registered with register_chip() (an mcu or an extras chip).
/// setup_pin() returns the chip's pin object (MCU_endstop, MCU_digital_out...).
pub trait PinChip {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> std::result::Result<Rc<dyn Any>, String>;
    /// MCU_buttons(printer, chip) on a chip that is not one of kaliru's MCUs
    /// (a python plugin's): chip.register_config_callback(build_config).
    /// None for kaliru's chips, whose buttons go through their MCU
    fn register_config_callback(&self, _cb: Rc<dyn Fn() -> crate::error::KResult<()>>) -> Option<crate::error::KResult<()>> {
        None
    }
    /// the python object of a plugin's chip (pin_params["chip"])
    fn python_object(&self) -> Option<Rc<dyn Any>> {
        None
    }
}

/// The pin_params dict. Shared: lookup_pin() hands the same object to every
/// user of a shared pin, and users attach a "class" object to it.
pub struct PinParams {
    pub chip: Rc<dyn PinChip>,
    pub chip_name: String,
    pub pin: String,
    pub invert: i64,
    pub pullup: i64,
    pub share_type: Option<String>,
    pub class: Option<Rc<dyn Any>>,
}

pub type SharedPinParams = Rc<RefCell<PinParams>>;

// ----------------------------------------------------------------------
// Command translation
// ----------------------------------------------------------------------

pub struct PinResolver {
    pub validate_aliases: bool,
    pub reserved: IndexMap<String, String>,
    pub aliases: IndexMap<String, String>,
    pub active_pins: IndexMap<String, String>,
}

impl PinResolver {
    pub fn new(validate_aliases: bool) -> Self {
        PinResolver {
            validate_aliases,
            reserved: IndexMap::new(),
            aliases: IndexMap::new(),
            active_pins: IndexMap::new(),
        }
    }

    pub fn reserve_pin(&mut self, pin: &str, reserve_name: &str) -> Result<()> {
        if let Some(r) = self.reserved.get(pin) {
            if r != reserve_name {
                return Err(error(format!("Pin {} reserved for {} - can't reserve for {}", pin, r, reserve_name)));
            }
        }
        self.reserved.insert(pin.to_string(), reserve_name.to_string());
        Ok(())
    }

    pub fn alias_pin(&mut self, alias: &str, pin: &str) -> Result<()> {
        if let Some(a) = self.aliases.get(alias) {
            if a != pin {
                return Err(error(format!("Alias {} mapped to {} - can't alias to {}", alias, a, pin)));
            }
        }
        if invalid_pin_name(pin) {
            return Err(error(format!("Invalid pin alias '{}'\n", pin)));
        }
        let pin = self.aliases.get(pin).cloned().unwrap_or_else(|| pin.to_string());
        self.aliases.insert(alias.to_string(), pin.clone());
        for existing_pin in self.aliases.values_mut() {
            if existing_pin == alias {
                *existing_pin = pin.clone();
            }
        }
        Ok(())
    }

    /// re_pin.sub(): every "[ _]pin=<name>" gets the resolved pin name
    pub fn update_command(&mut self, cmd: &str) -> Result<String> {
        let mut out = String::with_capacity(cmd.len());
        let mut i = 0;
        let b = cmd.as_bytes();
        while i < cmd.len() {
            if (b[i] == b' ' || b[i] == b'_') && cmd[i + 1..].starts_with("pin=") {
                let start = i + 5;
                let end = cmd[start..].find(' ').map(|p| start + p).unwrap_or(cmd.len());
                let name = &cmd[start..end];
                let pin_id = self.aliases.get(name).cloned().unwrap_or_else(|| name.to_string());
                let active = self.active_pins.entry(pin_id.clone()).or_insert_with(|| name.to_string()).clone();
                if name != active && self.validate_aliases {
                    return Err(error(format!("pin {} is an alias for {}", name, active)));
                }
                if let Some(r) = self.reserved.get(&pin_id) {
                    return Err(error(format!("pin {} is reserved for {}", name, r)));
                }
                out.push_str(&cmd[i..start]);
                out.push_str(&pin_id);
                i = end;
                continue;
            }
            let c = cmd[i..].chars().next().unwrap();
            out.push(c);
            i += c.len_utf8();
        }
        Ok(out)
    }
}

// ----------------------------------------------------------------------
// Pin to chip mapping
// ----------------------------------------------------------------------

pub struct PrinterPins {
    pub chips: IndexMap<String, Rc<dyn PinChip>>,
    pub active_pins: IndexMap<String, SharedPinParams>,
    pub pin_resolvers: IndexMap<String, PinResolver>,
    pub allow_multi_use_pins: HashMap<String, bool>,
}

impl Default for PrinterPins {
    fn default() -> Self {
        Self::new()
    }
}

impl PrinterPins {
    pub fn new() -> Self {
        PrinterPins {
            chips: IndexMap::new(),
            active_pins: IndexMap::new(),
            pin_resolvers: IndexMap::new(),
            allow_multi_use_pins: HashMap::new(),
        }
    }

    pub fn parse_pin(&self, pin_desc: &str, can_invert: bool, can_pullup: bool) -> Result<PinParams> {
        let mut desc = py_strip(pin_desc);
        let (mut pullup, mut invert) = (0, 0);
        if can_pullup && (desc.starts_with('^') || desc.starts_with('~')) {
            pullup = 1;
            if desc.starts_with('~') {
                pullup = -1;
            }
            desc = py_strip(&desc[1..]);
        }
        if can_invert && desc.starts_with('!') {
            invert = 1;
            desc = py_strip(&desc[1..]);
        }
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu", desc),
            Some((c, p)) => (py_strip(c), py_strip(p)),
        };
        let chip = self
            .chips
            .get(chip_name)
            .ok_or_else(|| error(format!("Unknown pin chip name '{}'", chip_name)))?;
        if invalid_pin_name(pin) {
            let mut format = String::new();
            if can_pullup {
                format.push_str("[^~] ");
            }
            if can_invert {
                format.push_str("[!] ");
            }
            return Err(error(format!(
                "Invalid pin description '{}'\nFormat is: {}[chip_name:] pin_name",
                pin_desc, format
            )));
        }
        Ok(PinParams {
            chip: chip.clone(),
            chip_name: chip_name.to_string(),
            pin: pin.to_string(),
            invert,
            pullup,
            share_type: None,
            class: None,
        })
    }

    pub fn lookup_pin(
        &mut self,
        pin_desc: &str,
        can_invert: bool,
        can_pullup: bool,
        share_type: Option<&str>,
    ) -> Result<SharedPinParams> {
        let mut pin_params = self.parse_pin(pin_desc, can_invert, can_pullup)?;
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        if let Some(share_params) = self.active_pins.get(&share_name) {
            let sp = share_params.borrow();
            if self.allow_multi_use_pins.contains_key(&share_name) {
            } else if share_type.is_none() || share_type != sp.share_type.as_deref() {
                return Err(error(format!("pin {} used multiple times in config", pin_params.pin)));
            } else if pin_params.invert != sp.invert || pin_params.pullup != sp.pullup {
                return Err(error(format!("Shared pin {} must have same polarity", pin_params.pin)));
            }
            drop(sp);
            return Ok(share_params.clone());
        }
        pin_params.share_type = share_type.map(|s| s.to_string());
        let shared = Rc::new(RefCell::new(pin_params));
        self.active_pins.insert(share_name, shared.clone());
        Ok(shared)
    }

    /// setup_pin(); Err(Ok(e)) is a pins.error, Err(Err(s)) an error from
    /// the chip's setup_pin()
    pub fn setup_pin(
        &mut self,
        pin_type: &str,
        pin_desc: &str,
    ) -> std::result::Result<Rc<dyn Any>, std::result::Result<Error, String>> {
        let can_invert = matches!(pin_type, "endstop" | "digital_out" | "pwm");
        let can_pullup = pin_type == "endstop";
        let pin_params = self.lookup_pin(pin_desc, can_invert, can_pullup, None).map_err(Ok)?;
        let chip = pin_params.borrow().chip.clone();
        chip.setup_pin(pin_type, &pin_params).map_err(Err)
    }

    pub fn reset_pin_sharing(&mut self, pin_params: &PinParams) {
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        // python: del (KeyError if missing)
        self.active_pins.shift_remove(&share_name);
    }

    pub fn get_pin_resolver(&mut self, chip_name: &str) -> Result<&mut PinResolver> {
        self.pin_resolvers
            .get_mut(chip_name)
            .ok_or_else(|| error(format!("Unknown chip name '{}'", chip_name)))
    }

    pub fn register_chip(&mut self, chip_name: &str, chip: Rc<dyn PinChip>) -> Result<()> {
        let chip_name = py_strip(chip_name);
        if self.chips.contains_key(chip_name) {
            return Err(error(format!("Duplicate chip name '{}'", chip_name)));
        }
        self.chips.insert(chip_name.to_string(), chip);
        self.pin_resolvers.insert(chip_name.to_string(), PinResolver::new(true));
        Ok(())
    }

    pub fn allow_multi_use_pin(&mut self, pin_desc: &str) -> Result<()> {
        let pin_params = self.parse_pin(pin_desc, false, false)?;
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        self.allow_multi_use_pins.insert(share_name, true);
        Ok(())
    }
}

/// the "pins" printer object
pub struct PinsObject(pub RefCell<PrinterPins>);

impl crate::printer::PrinterObject for PinsObject {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn add_printer_objects(printer: &Rc<crate::printer::Printer>, _config: &crate::configfile::ConfigWrapper) -> crate::error::KResult<()> {
    printer.add_object("pins", Rc::new(PinsObject(RefCell::new(PrinterPins::new()))))
}

/// printer.lookup_object("pins").setup_pin(pin_type, pin_desc)
pub fn setup_pin(printer: &crate::printer::Printer, pin_type: &str, pin_desc: &str) -> crate::error::KResult<Rc<dyn Any>> {
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let can_invert = matches!(pin_type, "endstop" | "digital_out" | "pwm");
    let can_pullup = pin_type == "endstop";
    // the pins borrow ends before the chip's setup_pin (a virtual pin chip
    // may set up further pins from there)
    let pin_params = ppins.0.borrow_mut().lookup_pin(pin_desc, can_invert, can_pullup, None);
    let pin_params = pin_params.map_err(|e| crate::error::KError::Pins(e.0))?;
    let chip = pin_params.borrow().chip.clone();
    chip.setup_pin(pin_type, &pin_params).map_err(crate::error::KError::Pins)
}

/// setup_pin() for an mcu pin class (the concrete chip object)
pub fn setup_pin_as<T: 'static>(printer: &crate::printer::Printer, pin_type: &str, pin_desc: &str) -> crate::error::KResult<Rc<T>> {
    setup_pin(printer, pin_type, pin_desc)?
        .downcast::<T>()
        .map_err(|_| crate::error::KError::Other(format!("pin '{}' of type {} is not supported here by Kaliru yet", pin_desc, pin_type)))
}

/// setup_pin("pwm", pin_desc) as a pwm pin (an McuPwm or another chip's
/// boxed Rc<dyn PwmPin>)
pub fn setup_pwm_pin(printer: &crate::printer::Printer, pin_desc: &str) -> crate::error::KResult<Rc<dyn crate::mcu::PwmPin>> {
    let obj = setup_pin(printer, "pwm", pin_desc)?;
    match obj.downcast::<crate::mcu::McuPwm>() {
        Ok(p) => Ok(p),
        Err(obj) => obj
            .downcast::<Rc<dyn crate::mcu::PwmPin>>()
            .map(|p| (*p).clone())
            .map_err(|_| crate::error::KError::Other(format!("pin '{}' of type pwm is not supported here by Kaliru yet", pin_desc))),
    }
}

/// the object setup_pin("digital_out", ...) returns, as a digital output
/// pin: an McuDigitalOut, or another chip's boxed Rc<dyn DigitalOutPin>
pub fn any_to_digital_out(obj: Rc<dyn Any>) -> Option<Rc<dyn crate::mcu::DigitalOutPin>> {
    match obj.downcast::<crate::mcu::McuDigitalOut>() {
        Ok(p) => Some(p),
        Err(obj) => obj.downcast::<Rc<dyn crate::mcu::DigitalOutPin>>().ok().map(|p| (*p).clone()),
    }
}

/// setup_pin("digital_out", pin_desc) as a digital output pin
pub fn setup_digital_out_pin(printer: &crate::printer::Printer, pin_desc: &str) -> crate::error::KResult<Rc<dyn crate::mcu::DigitalOutPin>> {
    any_to_digital_out(setup_pin(printer, "digital_out", pin_desc)?)
        .ok_or_else(|| crate::error::KError::Other(format!("pin '{}' of type digital_out is not supported here by Kaliru yet", pin_desc)))
}

/// setup_pin("adc", pin_desc) as an adc pin
pub fn setup_adc_pin(printer: &crate::printer::Printer, pin_desc: &str) -> crate::error::KResult<Rc<dyn crate::mcu::AdcPin>> {
    let obj = setup_pin(printer, "adc", pin_desc)?;
    match obj.downcast::<crate::mcu::McuAdc>() {
        Ok(p) => Ok(p),
        Err(obj) => obj
            .downcast::<Rc<dyn crate::mcu::AdcPin>>()
            .map(|p| (*p).clone())
            .map_err(|_| crate::error::KError::Other(format!("pin '{}' of type adc is not supported here by Kaliru yet", pin_desc))),
    }
}
