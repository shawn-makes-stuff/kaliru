// Temperature measurements with thermistors (port of klippy/extras/thermistor.py)
// Rusticc: with COSMOS's 0001-implement-pulldown-resistor-in-thermistor-calc
// (pulldown_resistor: a resistor in parallel with the thermistor, the CC1's
// CANVAS toolhead board has 100k)
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::adc_temperature::{custom_name, math_domain_error, AdcConvert, PrinterADCtoTemperature};
use crate::extras::heaters::{load_heaters, TempSensor};
use crate::logging;
use crate::printer::{none_object, PObj};
use crate::pyfmt::{py_max, py_min};
use std::cell::Cell;
use std::rc::Rc;

pub const KELVIN_TO_CELSIUS: f64 = -273.15;

/// math.log(x)
fn py_log(x: f64) -> KResult<f64> {
    if x <= 0.0 {
        return Err(math_domain_error());
    }
    Ok(x.ln())
}

// Analog voltage to temperature converter for thermistors
pub struct Thermistor {
    pullup: f64,
    inline_resistor: f64,
    pulldown_resistor: f64,
    c1: Cell<f64>,
    c2: Cell<f64>,
    c3: Cell<f64>,
}

impl Thermistor {
    pub fn new(pullup: f64, inline_resistor: f64, pulldown_resistor: f64) -> Self {
        Thermistor { pullup, inline_resistor, pulldown_resistor, c1: Cell::new(0.0), c2: Cell::new(0.0), c3: Cell::new(0.0) }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn setup_coefficients(&self, t1: f64, r1: f64, t2: f64, r2: f64, t3: f64, r3: f64, name: &str) -> KResult<()> {
        // Calculate Steinhart-Hart coefficents from temp measurements.
        // Arrange samples as 3 linear equations and solve for c1, c2, and c3.
        let inv_t1 = 1.0 / (t1 - KELVIN_TO_CELSIUS);
        let inv_t2 = 1.0 / (t2 - KELVIN_TO_CELSIUS);
        let inv_t3 = 1.0 / (t3 - KELVIN_TO_CELSIUS);
        let ln_r1 = py_log(r1)?;
        let ln_r2 = py_log(r2)?;
        let ln_r3 = py_log(r3)?;
        let (ln3_r1, ln3_r2, ln3_r3) = (ln_r1.powf(3.0), ln_r2.powf(3.0), ln_r3.powf(3.0));

        let (inv_t12, inv_t13) = (inv_t1 - inv_t2, inv_t1 - inv_t3);
        let (ln_r12, ln_r13) = (ln_r1 - ln_r2, ln_r1 - ln_r3);
        let (ln3_r12, ln3_r13) = (ln3_r1 - ln3_r2, ln3_r1 - ln3_r3);

        self.c3.set((inv_t12 - inv_t13 * ln_r12 / ln_r13) / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13));
        if self.c3.get() <= 0.0 {
            let beta = ln_r13 / inv_t13;
            logging::warning(&format!("Using thermistor beta {:.3} in heater {}", beta, name));
            return self.setup_coefficients_beta(t1, r1, beta);
        }
        self.c2.set((inv_t12 - self.c3.get() * ln3_r12) / ln_r12);
        self.c1.set(inv_t1 - self.c2.get() * ln_r1 - self.c3.get() * ln3_r1);
        Ok(())
    }

    pub fn setup_coefficients_beta(&self, t1: f64, r1: f64, beta: f64) -> KResult<()> {
        // Calculate equivalent Steinhart-Hart coefficents from beta
        let inv_t1 = 1.0 / (t1 - KELVIN_TO_CELSIUS);
        let ln_r1 = py_log(r1)?;
        self.c3.set(0.0);
        self.c2.set(1.0 / beta);
        self.c1.set(inv_t1 - self.c2.get() * ln_r1);
        Ok(())
    }

    fn remove_pulldown(&self, r: f64) -> f64 {
        if self.pulldown_resistor == 0.0 {
            return r;
        }
        // The ADC sees the thermistor branch in parallel with pulldown_resistor.
        let r = py_min(r, self.pulldown_resistor * 0.999999);
        r * self.pulldown_resistor / (self.pulldown_resistor - r)
    }

    fn apply_pulldown(&self, r: f64) -> f64 {
        if self.pulldown_resistor == 0.0 {
            return r;
        }
        r * self.pulldown_resistor / (r + self.pulldown_resistor)
    }
}

impl AdcConvert for Thermistor {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        // Calculate temperature from adc
        let adc = py_max(0.00001, py_min(0.99999, adc));
        let r = self.pullup * adc / (1.0 - adc);
        let r = self.remove_pulldown(r) - self.inline_resistor;
        let r = py_max(0.0000001, r);
        let ln_r = py_log(r)?;
        let inv_t = self.c1.get() + self.c2.get() * ln_r + self.c3.get() * ln_r.powf(3.0);
        Ok(1.0 / inv_t + KELVIN_TO_CELSIUS)
    }

    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        // Calculate adc reading from a temperature
        if temp <= KELVIN_TO_CELSIUS {
            return Ok(1.0);
        }
        let inv_t = 1.0 / (temp - KELVIN_TO_CELSIUS);
        let ln_r = if self.c3.get() != 0.0 {
            // Solve for ln_r using Cardano's formula
            let y = (self.c1.get() - inv_t) / (2.0 * self.c3.get());
            let x2 = (self.c2.get() / (3.0 * self.c3.get())).powf(3.0) + y.powf(2.0);
            if x2 < 0.0 {
                return Err(math_domain_error());
            }
            let x = x2.sqrt();
            let pow13 = |v: f64| if v < 0.0 { Err(math_domain_error()) } else { Ok(v.powf(1.0 / 3.0)) };
            pow13(x - y)? - pow13(x + y)?
        } else {
            (inv_t - self.c1.get()) / self.c2.get()
        };
        let r = ln_r.exp() + self.inline_resistor;
        let r = self.apply_pulldown(r);
        Ok(r / (self.pullup + r))
    }
}

/// thermistor parameters: (t1, r1, beta) or the three points, and the
/// pulldown_resistor
#[derive(Clone)]
pub enum ThermistorParams {
    Beta { t1: f64, r1: f64, beta: f64, pulldown_resistor: f64 },
    Points { t1: f64, r1: f64, t2: f64, r2: f64, t3: f64, r3: f64, pulldown_resistor: f64 },
}

// Create an ADC converter with a thermistor
pub fn printer_thermistor(config: &ConfigWrapper, params: &ThermistorParams) -> KResult<Rc<dyn TempSensor>> {
    let pullup = config.getfloat("pullup_resistor", 4700.0, Lim::new().above(0.0))?;
    let inline_resistor = config.getfloat("inline_resistor", 0.0, Lim::new().min(0.0))?;
    let (ThermistorParams::Beta { pulldown_resistor, .. } | ThermistorParams::Points { pulldown_resistor, .. }) = *params;
    let pulldown_resistor = config.getfloat("pulldown_resistor", pulldown_resistor, Lim::new().min(0.0))?;
    let thermistor = Thermistor::new(pullup, inline_resistor, pulldown_resistor);
    match *params {
        ThermistorParams::Beta { t1, r1, beta, .. } => thermistor.setup_coefficients_beta(t1, r1, beta)?,
        ThermistorParams::Points { t1, r1, t2, r2, t3, r3, .. } => {
            thermistor.setup_coefficients(t1, r1, t2, r2, t3, r3, config.get_name())?
        }
    }
    Ok(PrinterADCtoTemperature::new(config, Rc::new(thermistor))?)
}

// Custom defined thermistors from the config file
pub fn custom_thermistor(config: &ConfigWrapper) -> KResult<ThermistorParams> {
    let t1 = config.getfloat("temperature1", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
    let r1 = config.getfloat("resistance1", Dflt::Required, Lim::new().min(0.0))?;
    let beta = config.getfloat_opt("beta", Lim::new().above(0.0))?;
    let pulldown_resistor = config.getfloat("pulldown_resistor", 0.0, Lim::new().min(0.0))?;
    if let Some(beta) = beta {
        return Ok(ThermistorParams::Beta { t1, r1, beta, pulldown_resistor });
    }
    let t2 = config.getfloat("temperature2", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
    let r2 = config.getfloat("resistance2", Dflt::Required, Lim::new().min(0.0))?;
    let t3 = config.getfloat("temperature3", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
    let r3 = config.getfloat("resistance3", Dflt::Required, Lim::new().min(0.0))?;
    let mut pts = [(t1, r1), (t2, r2), (t3, r3)];
    pts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let [(t1, r1), (t2, r2), (t3, r3)] = pts;
    Ok(ThermistorParams::Points { t1, r1, t2, r2, t3, r3, pulldown_resistor })
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let name = custom_name(config);
    let params = custom_thermistor(config)?;
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(&name, Rc::new(move |config: &ConfigWrapper| printer_thermistor(config, &params)));
    Ok(none_object())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// calc_temp / calc_adc with pulldown_resistor, as COSMOS's python gives them
    #[test]
    fn pulldown() {
        let t = Thermistor::new(4700.0, 0.0, 100000.0);
        t.setup_coefficients_beta(25.0, 100000.0, 3950.0).unwrap();
        let temps: Vec<f64> = [0.0, 0.01, 0.1, 0.5, 0.9, 0.95, 0.99, 0.999, 1.0].iter().map(|a| t.calc_temp(*a).unwrap()).collect();
        assert_eq!(temps, [-3260.690620222191, 432.6971709805599, 220.6760141405802, 112.63371896172578, 32.15469230477049, -16.157906540315253, -127.19898313194969, -127.19898313194969, -127.19898313194969]);
        let adcs: Vec<f64> = [-300.0, 0.0, 25.0, 100.0, 200.0, 300.0].iter().map(|x| t.calc_adc(*x).unwrap()).collect();
        assert_eq!(adcs, [1.0, 0.9425252593532268, 0.9140767824497258, 0.5811196468025761, 0.13588794252253147, 0.03554538964588165]);
        let t = Thermistor::new(4700.0, 50.0, 47000.0);
        t.setup_coefficients(25.0, 100000.0, 150.0, 1641.9, 250.0, 226.15, "").unwrap();
        let temps: Vec<f64> = [0.0, 0.01, 0.1, 0.5, 0.9, 0.95, 0.99, 0.999, 1.0].iter().map(|a| t.calc_temp(*a).unwrap()).collect();
        assert_eq!(temps, [-591.0510318859599, -591.0510318859599, 207.50043532466458, 106.85399436502223, -4.657397858758088, -137.5353184396466, -137.5353184396466, -137.5353184396466, -137.5353184396466]);
        let adcs: Vec<f64> = [-300.0, 0.0, 25.0, 100.0, 200.0, 300.0].iter().map(|x| t.calc_adc(*x).unwrap()).collect();
        assert_eq!(adcs, [1.0, 0.8975267344533139, 0.8718574354058647, 0.5449169839822805, 0.11239447773464356, 0.032275935954009405]);
        let t = Thermistor::new(4700.0, 0.0, 0.0);
        t.setup_coefficients_beta(25.0, 100000.0, 3950.0).unwrap();
        let temps: Vec<f64> = [0.0, 0.01, 0.1, 0.5, 0.9, 0.95, 0.99, 0.999, 1.0].iter().map(|a| t.calc_temp(*a).unwrap()).collect();
        assert_eq!(temps, [-3260.689558202203, 432.75707098972225, 220.9994789393936, 114.45613240847592, 45.707426361634475, 27.568766143873177, -6.003183074855997, -42.122234336944786, -91.15315302601343]);
        let adcs: Vec<f64> = [-300.0, 0.0, 25.0, 100.0, 200.0, 300.0].iter().map(|x| t.calc_adc(*x).unwrap()).collect();
        assert_eq!(adcs, [1.0, 0.9862132098380977, 0.9551098376313276, 0.5974372247265058, 0.13676140111700302, 0.03560487233171683]);
    }
}
