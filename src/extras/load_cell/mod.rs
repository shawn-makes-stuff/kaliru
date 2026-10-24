// Load cell module setup (port of klippy/extras/load_cell/__init__.py and
// interfaces.py)
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
pub mod ads1220;
pub mod ads131m0x;
pub mod cs1237;
pub mod hx71x;
pub mod hx711s;
#[allow(clippy::module_inception)]
pub mod load_cell;
pub mod load_cell_probe;
pub mod sos_filter;
pub mod tap_analysis;
pub mod tap_quality_classifier;

use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, ClientCallback};
use crate::mcu::Mcu;
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, SubsystemComponentCollection};
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::rc::Rc;

/// BulkAdcSensor: clients receive the BulkAdcData dict ({"data": [rows],
/// "errors": n, "overflows": n}) and return False to disconnect
pub trait BulkAdcSensor {
    /// Get the MCU the sensor is attached to
    fn get_mcu(&self) -> Rc<Mcu>;
    /// Return the sample per second of the sensor
    fn get_samples_per_second(&self) -> i64;
    /// Return the range of the sensor as (min, max)
    fn get_range(&self) -> (i64, i64);
    /// Return number of ADC channels emitted per sample
    fn get_channel_count(&self) -> usize;
    /// Attach a client to the sensor to receive BulkAdcData
    fn add_client(&self, callback: ClientCallback) -> KResult<()>;
    /// McuLoadCellProbe: attach a sensor to a LoadCell object on the MCU
    fn attach_load_cell_probe(&self, load_cell_probe_oid: i64) -> KResult<()>;
}

/// a sensor class of the load_cell_sensors components
pub type SensorFactory = fn(&ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>>;

/// the sensors that implement BulkAdcSensor, in python's dict union order
pub fn sensor_types() -> [(&'static str, SensorFactory); 8] {
    [
        ("hx711", hx71x::new_hx711),
        ("hx717", hx71x::new_hx717),
        ("ads1220", ads1220::new_ads1220),
        ("ads131m02", ads131m0x::new_ads131m02),
        ("ads131m04", ads131m0x::new_ads131m04),
        ("cs1237", cs1237::new_cs1237),
        // COSMOS 0007-Add-HX711s-load-cell-probe-driver
        ("hx711s", hx711s::new_hx711s),
        ("hx717s", hx711s::new_hx717s),
    ]
}

// register sensors that implement BulkAdcSensor
pub fn register_components(subsystem: &SubsystemComponentCollection) -> KResult<()> {
    for (name, sensor) in sensor_types() {
        subsystem.register_component("load_cell_sensors", name, Rc::new(sensor))?;
    }
    Ok(())
}

/// config.getchoice("sensor_type", printer.lookup_components(subsystem))
pub fn lookup_sensor_class<T: Copy + 'static>(config: &ConfigWrapper, subsystem: &str) -> KResult<T> {
    let sensors = config.get_printer().lookup_components(subsystem);
    let choices: Vec<(&str, T)> = sensors
        .iter()
        .filter_map(|(name, c)| c.downcast_ref::<T>().map(|f| (name.as_str(), *f)))
        .collect();
    Ok(config.getchoice("sensor_type", &choices, crate::configfile::Dflt::Required)?)
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let sensor_class = lookup_sensor_class::<SensorFactory>(config, "load_cell_sensors")?;
    let sensor = sensor_class(config)?;
    Ok(load_cell::LoadCell::new(config, sensor)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    load_config(config)
}

/// the error of a sensor command used before klippy:mcu_identify built it
pub(crate) fn no_cmd() -> KError {
    KError::Other("'NoneType' object has no attribute 'send'".into())
}

/// bulk_sensor.BatchBulkHelper(printer, s._process_batch,
/// s._start_measurements, s._finish_measurements, interval)
pub(crate) fn batch_helper<S: 'static>(
    printer: &Rc<Printer>,
    s: &Rc<S>,
    process: fn(&S, f64) -> KResult<PyVal>,
    start: fn(&S) -> KResult<()>,
    finish: fn(&S) -> KResult<()>,
    interval: f64,
) -> Rc<BatchBulkHelper> {
    let (w1, w2, w3) = (Rc::downgrade(s), Rc::downgrade(s), Rc::downgrade(s));
    BatchBulkHelper::new(
        printer,
        Rc::new(move |eventtime| match w1.upgrade() {
            Some(s) => process(&s, eventtime),
            None => Ok(PyVal::dict()),
        }),
        Some(Rc::new(move || match w2.upgrade() {
            Some(s) => start(&s),
            None => Ok(()),
        })),
        Some(Rc::new(move || match w3.upgrade() {
            Some(s) => finish(&s),
            None => Ok(()),
        })),
        interval,
    )
}

/// 1 / (1 << 23): counts to a +/-1.0 fraction of a 24 bit adc
pub(crate) const ADC_FACTOR: f64 = 1.0 / (1 << 23) as f64;

/// a 24 bit adc sample row: (round(ptime, 6), val, round(val * adc_factor, 9))
pub(crate) fn adc_row(ptime: f64, val: i64) -> PyVal {
    PyVal::Tuple(vec![PyVal::Float(py_round(ptime, 6)), PyVal::Int(val), PyVal::Float(py_round(val as f64 * ADC_FACTOR, 9))])
}

/// the {"data": samples, "errors": n, "overflows": n} batch message
pub(crate) fn adc_batch(data: Vec<PyVal>, errors: i64, overflows: i64) -> PyVal {
    let mut d = IndexMap::new();
    d.insert("data".to_string(), PyVal::List(data));
    d.insert("errors".to_string(), PyVal::Int(errors));
    d.insert("overflows".to_string(), PyVal::Int(overflows));
    PyVal::Dict(d)
}

/// ppins.lookup_pin(pin) of a sensor pin: (its mcu, pin name)
pub(crate) fn lookup_sensor_pin(config: &ConfigWrapper, option: &str) -> KResult<(Rc<Mcu>, String)> {
    lookup_pin(config, &config.get(option, crate::configfile::Dflt::Required)?)
}

/// ppins.lookup_pin(pin): (its mcu, pin name)
pub(crate) fn lookup_pin(config: &ConfigWrapper, pin: &str) -> KResult<(Rc<Mcu>, String)> {
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let pp = ppins.0.borrow_mut().lookup_pin(pin, false, false, None).map_err(|e| KError::Pins(e.0))?;
    let mcu = crate::extras::bus::pin_mcu(config, &pp)?;
    let name = pp.borrow().pin.clone();
    Ok((mcu, name))
}
