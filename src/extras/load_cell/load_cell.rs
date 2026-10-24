// Load Cell Implementation (port of klippy/extras/load_cell/load_cell.py)
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::BulkAdcSensor;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::BatchWebhooksClient;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{ConfigFileObject, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_round, py_sum};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, ReactorCompletion};
use crate::toolhead::ToolHead;
use crate::webhooks::WebHooks;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::rc::{Rc, Weak};

/// sum(data) / len(data) of float data
fn avg(data: &[f64]) -> f64 {
    py_sum(data.iter().copied()) / data.len() as f64
}

/// int average: sum(ints) / len(ints)
fn avg_int(data: &[i64]) -> f64 {
    data.iter().sum::<i64>() as f64 / data.len() as f64
}

fn has_flag(gcmd: &GCodeCommand, name: &str) -> bool {
    let v = gcmd.get_or(name, "False").trim().to_lowercase();
    v == "1" || v == "true"
}

// SampleStructure: COLUMN = column index, colum header label
const SAMPLE_COLUMNS: [&str; 4] = ["time", "force (g)", "counts", "tare_counts"];
const FORCE_LABEL: &str = "force (g)";
const COUNTS_LABEL: &str = "counts";

fn header_labels(channel_count: usize) -> Vec<String> {
    let mut labels: Vec<String> = SAMPLE_COLUMNS.iter().map(|s| s.to_string()).collect();
    for idx in 0..channel_count {
        labels.push(format!("channel-{} {}", idx, FORCE_LABEL));
        labels.push(format!("channel-{} {}", idx, COUNTS_LABEL));
    }
    labels
}

/// one row of force data: [time, force (g), counts, tare_counts,
/// channel-0 force (g), channel-0 counts, ...]
#[derive(Clone, Debug)]
pub struct ForceSample {
    pub time: f64,
    pub force: Option<f64>,
    pub counts: i64,
    pub tare_counts: Option<i64>,
    pub channels: Vec<(Option<f64>, i64)>,
}

fn opt_f(v: Option<f64>) -> PyVal {
    v.map(PyVal::Float).unwrap_or(PyVal::None)
}

fn opt_i(v: Option<i64>) -> PyVal {
    v.map(PyVal::Int).unwrap_or(PyVal::None)
}

impl ForceSample {
    fn to_pyval(&self) -> PyVal {
        let mut row = vec![PyVal::Float(self.time), opt_f(self.force), PyVal::Int(self.counts), opt_i(self.tare_counts)];
        for (force, counts) in &self.channels {
            row.push(opt_f(*force));
            row.push(PyVal::Int(*counts));
        }
        PyVal::List(row)
    }
}

/// a force message: {"data": samples, "errors": n, "overflows": n}
#[derive(Clone)]
pub struct ForceMsg {
    pub data: Vec<ForceSample>,
    pub errors: PyVal,
    pub overflows: PyVal,
}

/// a message ApiClientHelper sends: json'able for webhooks clients
pub(crate) trait ApiMsg {
    fn to_pyval(&self) -> PyVal;
}

impl ApiMsg for PyVal {
    fn to_pyval(&self) -> PyVal {
        self.clone()
    }
}

impl ApiMsg for ForceMsg {
    fn to_pyval(&self) -> PyVal {
        let mut d = IndexMap::new();
        d.insert("data".to_string(), PyVal::List(self.data.iter().map(|s| s.to_pyval()).collect()));
        d.insert("errors".to_string(), self.errors.clone());
        d.insert("overflows".to_string(), self.overflows.clone());
        PyVal::Dict(d)
    }
}

pub type ForceClient = Rc<dyn Fn(&ForceMsg) -> bool>;

// Helper for event driven webhooks and subscription based API clients
pub(crate) struct ApiClientHelper<M: ApiMsg + 'static = ForceMsg> {
    printer: Rc<Printer>,
    client_cbs: RefCell<Vec<Rc<dyn Fn(&M) -> bool>>>,
    webhooks_start_resp: RefCell<PyVal>,
}

impl<M: ApiMsg + 'static> ApiClientHelper<M> {
    pub(crate) fn new(printer: &Rc<Printer>) -> Rc<Self> {
        Rc::new(ApiClientHelper { printer: printer.clone(), client_cbs: RefCell::new(vec![]), webhooks_start_resp: RefCell::new(PyVal::dict()) })
    }

    // send data to clients
    pub(crate) fn send(&self, msg: &M) {
        let cbs = self.client_cbs.borrow().clone();
        for client_cb in cbs {
            if !client_cb(msg) {
                // This client no longer needs updates - unregister it
                let mut c = self.client_cbs.borrow_mut();
                if let Some(pos) = c.iter().position(|cb| Rc::ptr_eq(cb, &client_cb)) {
                    c.remove(pos);
                }
            }
        }
    }

    // Add a client that gets data callbacks
    pub(crate) fn add_client(&self, client_cb: Rc<dyn Fn(&M) -> bool>) {
        self.client_cbs.borrow_mut().push(client_cb);
    }

    // Set up a webhooks endpoint with a static header
    pub(crate) fn add_mux_endpoint(self: &Rc<Self>, path: &str, key: &str, value: &str, webhooks_start_resp: PyVal) -> KResult<()> {
        *self.webhooks_start_resp.borrow_mut() = webhooks_start_resp;
        let wh = self.printer.lookup::<WebHooks>("webhooks")?;
        let w = Rc::downgrade(self);
        wh.register_mux_endpoint(
            path,
            key,
            Some(value),
            Rc::new(move |web_request| {
                let Some(h) = w.upgrade() else { return Ok(()) };
                // Add Webhooks client and send header
                let whbatch = BatchWebhooksClient::new(web_request)?;
                h.add_client(Rc::new(move |msg| whbatch.handle_batch(&msg.to_pyval())));
                let resp = h.webhooks_start_resp.borrow().clone();
                web_request.send(resp)
            }),
        )
    }
}

const CMD_LOAD_CELL_TARE_HELP: &str = "Set the Zero point of the load cell";
const CMD_CALIBRATE_LOAD_CELL_HELP: &str = "Start interactive calibration tool";
const CMD_LOAD_CELL_READ_HELP: &str = "Take a reading from the load cell";
const CMD_LOAD_CELL_DIAGNOSTIC_HELP: &str = "Check the health of the load cell";

// Class for handling gcode commands related to load cells
struct LoadCellCommandHelper {
    printer: Rc<Printer>,
    load_cell: Weak<LoadCell>,
}

impl LoadCellCommandHelper {
    fn new(config: &ConfigWrapper, load_cell: &Rc<LoadCell>) -> KResult<()> {
        let h = Rc::new(LoadCellCommandHelper { printer: config.get_printer(), load_cell: Rc::downgrade(load_cell) });
        let name_parts: Vec<&str> = config.get_name().split_whitespace().collect();
        let name = name_parts[name_parts.len() - 1];
        h.register_commands(Some(name))?;
        if name_parts.len() == 1 {
            h.register_commands(None)?;
        }
        Ok(())
    }

    fn register_commands(self: &Rc<Self>, name: Option<&str>) -> KResult<()> {
        // Register commands
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        type LCmd = fn(&LoadCellCommandHelper, &Rc<LoadCell>, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, LCmd, &str); 4] = [
            ("LOAD_CELL_TARE", LoadCellCommandHelper::cmd_load_cell_tare, CMD_LOAD_CELL_TARE_HELP),
            ("LOAD_CELL_CALIBRATE", LoadCellCommandHelper::cmd_load_cell_calibrate, CMD_CALIBRATE_LOAD_CELL_HELP),
            ("LOAD_CELL_READ", LoadCellCommandHelper::cmd_load_cell_read, CMD_LOAD_CELL_READ_HELP),
            ("LOAD_CELL_DIAGNOSTIC", LoadCellCommandHelper::cmd_load_cell_diagnostic, CMD_LOAD_CELL_DIAGNOSTIC_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let h = self.clone();
            gcode.register_mux_command(
                cmd,
                "LOAD_CELL",
                name,
                Rc::new(move |g| match h.load_cell.upgrade() {
                    Some(lc) => f(&h, &lc, g),
                    None => Ok(()),
                }),
                Some(desc),
            )?;
        }
        Ok(())
    }

    fn cmd_load_cell_tare(&self, lc: &Rc<LoadCell>, gcmd: &GCodeCommand) -> KResult<()> {
        let reporter = ForceReporter::new(gcmd, lc)?;
        lc.tare(&reporter.channel_counts)?;
        reporter.report(ZeroReference::ReferenceTare, "Load cell tare value: 0 = ", true, true);
        calibration_warning(gcmd, lc, reporter.counts);
        Ok(())
    }

    fn cmd_load_cell_calibrate(&self, lc: &Rc<LoadCell>, gcmd: &GCodeCommand) -> KResult<()> {
        // just tare or run the whole guided calibration experience?
        if has_flag(gcmd, "TARE") {
            self.calibration_tare(lc, gcmd)
        } else {
            LoadCellGuidedCalibrationHelper::start(&self.printer, lc)
        }
    }

    // set reference tare counts on a calibrated load cell
    fn calibration_tare(&self, lc: &Rc<LoadCell>, gcmd: &GCodeCommand) -> KResult<()> {
        // it is invalid to set the reference tare counts on an uncalibrated
        // load cell. Prompt user to run full calibration
        if !lc.is_calibrated() {
            return Err(gcmd.error("Load cell has not been calibrated. Run LOAD_CELL_CALIBRATE first."));
        }
        let reporter = ForceReporter::new(gcmd, lc)?;
        reporter.report(ZeroReference::Zero, "New reference tare value: 0 = ", true, true);
        self.report_calibration_change(lc, gcmd, &reporter);
        calibration_warning(gcmd, lc, reporter.counts);
        let save = has_flag(gcmd, "SAVE");
        lc.set_reference_tare_counts(Some(reporter.channel_counts.clone()), save)
    }

    fn report_calibration_change(&self, lc: &LoadCell, gcmd: &GCodeCommand, reporter: &ForceReporter) {
        let change = reporter.counts - lc.reference_tare_counts.get().unwrap_or(0);
        let formatter = ForceFormatter { counts: change, percent: lc.counts_to_percent(change), force: lc.counts_to_grams(change, ZeroReference::Zero, None) };
        gcmd.respond_info(&format!("Calibration changed by: {}", formatter.format(true, false)), true);
    }

    fn cmd_load_cell_read(&self, lc: &Rc<LoadCell>, gcmd: &GCodeCommand) -> KResult<()> {
        let reporter = ForceReporter::new(gcmd, lc)?;
        reporter.report(ZeroReference::Tare, "", true, true);
        Ok(())
    }

    fn cmd_load_cell_diagnostic(&self, lc: &Rc<LoadCell>, gcmd: &GCodeCommand) -> KResult<()> {
        gcmd.respond_info("Collecting load cell data for 10 seconds...", true);
        let collector = lc.get_collector();
        let reactor = self.printer.get_reactor();
        collector.start_collecting(None);
        reactor.pause(reactor.monotonic() + 10.0)?;
        let (samples, errors) = collector.stop_collecting();
        match errors {
            Some((e, o)) => gcmd.respond_info(&format!("Sensor reported errors: {} errors, {} overflows", e, o), true),
            None => gcmd.respond_info("Sensor reported no errors", true),
        }
        if samples.is_empty() {
            return Err(gcmd.error("No samples returned from sensor!"));
        }
        let counts: Vec<i64> = samples.iter().map(|s| s.counts).collect();
        let (range_min, range_max) = lc.saturation_range();
        let mut good_count = 0;
        let mut saturation_count = 0;
        for &sample in &counts {
            if sample >= range_max || sample <= range_min {
                saturation_count += 1;
            } else {
                good_count += 1;
            }
        }
        gcmd.respond_info(&format!("Samples Collected: {}", samples.len()), true);
        if samples.len() > 2 {
            let sensor_sps = lc.sensor.get_samples_per_second();
            let sps = samples.len() as f64 / (samples[samples.len() - 1].time - samples[0].time);
            gcmd.respond_info(&format!("Measured samples per second: {:.1}, configured: {:.1}", sps, sensor_sps as f64), true);
        }
        let unique: std::collections::HashSet<i64> = counts.iter().copied().collect();
        gcmd.respond_info(&format!("Good samples: {}, Saturated samples: {}, Unique values: {}", good_count, saturation_count, unique.len()), true);
        let max_pct = lc.counts_to_percent(*counts.iter().max().unwrap_or(&0));
        let min_pct = lc.counts_to_percent(*counts.iter().min().unwrap_or(&0));
        gcmd.respond_info(&format!("Sample range: [{:.2}% to {:.2}%]", min_pct, max_pct), true);
        gcmd.respond_info(&format!("Sample range / sensor capacity: {:.5}%", (max_pct - min_pct) / 2.0), true);
        Ok(())
    }
}

// warn if the tare point is far enough from zero to reduce sensor range
fn calibration_warning(gcmd: &GCodeCommand, load_cell: &LoadCell, counts: i64) {
    if load_cell.counts_to_percent(counts) <= 2.0 {
        return;
    }
    gcmd.respond_info(
        "WARNING: tare value is more than 2% away from 0!\nThe load cell's range will be impacted.\nCheck for external force on the load cell.",
        true,
    );
}

const CMD_ABORT_HELP: &str = "Abort load cell calibration tool";
const CMD_ACCEPT_HELP: &str = "Accept calibration results and apply to load cell";
const CMD_TARE_HELP: &str = "Tare the load cell";
const CMD_CALIBRATE_HELP: &str = "Enter the load cell value in grams";

// Class to guide the user through calibrating a load cell
struct LoadCellGuidedCalibrationHelper {
    gcode: Rc<GCodeDispatch>,
    load_cell: Rc<LoadCell>,
    tare_counts: RefCell<Option<Vec<i64>>>,
    counts_per_gram: Cell<Option<f64>>,
    tare_percent: Cell<f64>,
}

impl LoadCellGuidedCalibrationHelper {
    fn start(printer: &Rc<Printer>, load_cell: &Rc<LoadCell>) -> KResult<()> {
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let h = Rc::new(LoadCellGuidedCalibrationHelper {
            gcode: gcode.clone(),
            load_cell: load_cell.clone(),
            tare_counts: RefCell::new(None),
            counts_per_gram: Cell::new(None),
            tare_percent: Cell::new(0.0),
        });
        h.register_commands()?;
        gcode.respond_info(
            "Starting load cell calibration. \n1.) Remove all load and run TARE. \n2.) Apply a known load, run CALIBRATE GRAMS=nnn. \nComplete calibration with the ACCEPT command.\nUse the ABORT command to quit.",
            true,
        );
        Ok(())
    }

    fn verify_no_active_calibration(&self) -> KResult<()> {
        let dummy: crate::gcode::Handler = Rc::new(|_| Ok(()));
        if let Err(e) = self.gcode.register_command("TARE", Some(dummy), false, None) {
            if matches!(e.base(), KError::Config(_)) {
                return Err(KError::command("Already Calibrating a Load Cell. Use ABORT to quit."));
            }
            return Err(e);
        }
        self.gcode.register_command("TARE", None, false, None)?;
        Ok(())
    }

    fn register_commands(self: &Rc<Self>) -> KResult<()> {
        self.verify_no_active_calibration()?;
        type GCmd = fn(&LoadCellGuidedCalibrationHelper, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, GCmd, &str); 4] = [
            ("ABORT", LoadCellGuidedCalibrationHelper::cmd_abort, CMD_ABORT_HELP),
            ("ACCEPT", LoadCellGuidedCalibrationHelper::cmd_accept, CMD_ACCEPT_HELP),
            ("TARE", LoadCellGuidedCalibrationHelper::cmd_tare, CMD_TARE_HELP),
            ("CALIBRATE", LoadCellGuidedCalibrationHelper::cmd_calibrate, CMD_CALIBRATE_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let h = self.clone();
            self.gcode.register_command(cmd, Some(Rc::new(move |g| f(&h, g))), false, Some(desc))?;
        }
        Ok(())
    }

    // convert the delta of counts to a counts/gram metric
    fn counts_per_gram(&self, grams: f64, cal_counts: &[i64]) -> f64 {
        let tare_sum: i64 = self.tare_counts.borrow().as_ref().map(|t| t.iter().sum()).unwrap_or(0);
        let cal_sum: i64 = cal_counts.iter().sum();
        (tare_sum - cal_sum).abs() as f64 / grams
    }

    // calculate max force that the load cell can register
    // given tare bias, at saturation in kilograms
    fn capacity_kg(&self, counts_per_gram: f64) -> f64 {
        let (_, range_max) = self.load_cell.saturation_range();
        let tare_sum: i64 = self.tare_counts.borrow().as_ref().map(|t| t.iter().sum()).unwrap_or(0);
        ((range_max - tare_sum.abs()) as f64 / counts_per_gram).trunc() / 1000.0
    }

    fn finalize(&self, save_results: bool) -> KResult<()> {
        for name in ["ABORT", "ACCEPT", "TARE", "CALIBRATE"] {
            self.gcode.register_command(name, None, false, None)?;
        }
        if !save_results {
            self.gcode.respond_info("Load cell calibration aborted", true);
            return Ok(());
        }
        if self.counts_per_gram.get().is_none() || self.tare_counts.borrow().is_none() {
            self.gcode.respond_info("Calibration process is incomplete, aborting", true);
        }
        let tare_counts = self.tare_counts.borrow().clone();
        self.load_cell.set_calibration(self.counts_per_gram.get(), tare_counts.clone(), true)?;
        let tare_counts = tare_counts.unwrap_or_default();
        let ref_tare_str = tare_counts.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(", ");
        self.gcode.respond_info(
            &format!(
                "Load cell calibration settings:\n\ncounts_per_gram: {:.6}\nreference_tare_counts: {}\n\nThe SAVE_CONFIG command will update the printer config file with the above and restart the printer.",
                self.counts_per_gram.get().unwrap_or(0.0),
                ref_tare_str
            ),
            true,
        );
        self.load_cell.tare(&tare_counts)
    }

    fn cmd_abort(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.finalize(false)
    }

    fn cmd_accept(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.finalize(true)
    }

    fn cmd_tare(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.counts_per_gram.set(None); // require re-calibration on tare
        let reporter = ForceReporter::new(gcmd, &self.load_cell)?;
        *self.tare_counts.borrow_mut() = Some(reporter.channel_counts.clone());
        reporter.report(ZeroReference::Zero, "Load cell tare value: 0 = ", false, true);
        self.tare_percent.set(self.load_cell.counts_to_percent(reporter.counts));
        calibration_warning(gcmd, &self.load_cell, reporter.counts);
        gcmd.respond_info(
            &format!("Now apply a known force to the load cell and enter {}the force value with:\n CALIBRATE GRAMS=nnn", " ".repeat(25)),
            true,
        );
        Ok(())
    }

    fn cmd_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(tare_counts) = self.tare_counts.borrow().clone() else {
            gcmd.respond_info("You must use TARE first.", true);
            return Ok(());
        };
        let grams = gcmd.get_float("GRAMS", None, Lim::new().min(50.0).max(25000.0))?;
        let cal_counts = self.load_cell.avg_counts(None)?;
        let cal_sum: i64 = cal_counts.iter().sum();
        let cal_percent = self.load_cell.counts_to_percent(cal_sum);
        let c_per_g = self.counts_per_gram(grams, &cal_counts);
        let cap_kg = self.capacity_kg(c_per_g);
        gcmd.respond_info(
            &format!(
                "Calibration value: {:.2}% ({}), Counts/gram: {:.5}, {}Total capacity: +/- {:.2}Kg",
                cal_percent,
                cal_sum,
                c_per_g,
                " ".repeat(12),
                cap_kg
            ),
            true,
        );
        let (range_min, range_max) = self.load_cell.saturation_range();
        if cal_sum >= range_max || cal_sum <= range_min {
            return Err(KError::command("ERROR: Sensor is saturated with too much load!\nUse less force to calibrate the load cell."));
        }
        if cal_counts == tare_counts {
            return Err(KError::command(
                "ERROR: Tare and Calibration readings are the same!\nCheck wiring and validate sensor with READ_LOAD_CELL command.",
            ));
        }
        // COSMOS 0002-reduce-calibration-difference-tolerance: 0.1 %
        if (cal_percent - self.tare_percent.get()).abs() < 0.1 {
            return Err(KError::command(
                "ERROR: Tare and Calibration readings are less than 0.1% different!\nUse more force when calibrating or a higher sensor gain.",
            ));
        }
        // only set _counts_per_gram after all errors are raised
        self.counts_per_gram.set(Some(c_per_g));
        if cap_kg < 1.0 {
            gcmd.respond_info("WARNING: Load cell capacity is less than 1kg!\nCheck wiring and consider using a lower sensor gain.", true);
        }
        if cap_kg > 25.0 {
            gcmd.respond_info("WARNING: Load cell capacity is more than 25Kg!\nCheck wiring and consider using a higher sensor gain.", true);
        }
        gcmd.respond_info("Accept calibration with the ACCEPT command.", true);
        Ok(())
    }
}

/// the errors of a collection: python's (errors, overflows), or 0
pub type CollectErrors = Option<(i64, i64)>;

// Utility to collect some samples from the LoadCell for later analysis
// Optionally blocks execution while collecting with reactor.pause()
// can collect a minimum n samples or collect until a specific print_time
pub struct LoadCellSampleCollector {
    me: Weak<LoadCellSampleCollector>,
    load_cell: Weak<LoadCell>,
    reactor: Rc<Reactor>,
    mcu: Rc<Mcu>,
    min_time: Cell<f64>,
    max_time: Cell<f64>,
    min_count: Cell<f64>,
    is_started: Cell<bool>,
    completion: RefCell<Option<Rc<ReactorCompletion<bool>>>>,
    samples: RefCell<Vec<ForceSample>>,
    errors: Cell<i64>,
    overflows: Cell<i64>,
}

impl LoadCellSampleCollector {
    fn new(printer: &Rc<Printer>, load_cell: &Rc<LoadCell>) -> Rc<Self> {
        Rc::new_cyclic(|me| LoadCellSampleCollector {
            me: me.clone(),
            load_cell: Rc::downgrade(load_cell),
            reactor: printer.get_reactor(),
            mcu: load_cell.sensor.get_mcu(),
            min_time: Cell::new(0.0),
            max_time: Cell::new(f64::INFINITY),
            min_count: Cell::new(f64::INFINITY),
            is_started: Cell::new(false),
            completion: RefCell::new(None),
            samples: RefCell::new(vec![]),
            errors: Cell::new(0),
            overflows: Cell::new(0),
        })
    }

    // move from the started to stopped state and trigger the completion
    fn complete(&self) {
        self.is_started.set(false);
        if let Some(c) = self.completion.borrow_mut().take() {
            c.complete(true);
        }
    }

    fn on_samples(&self, msg: &ForceMsg) -> bool {
        if !self.is_started.get() {
            return false; // already stopped, ignore
        }
        self.errors.set(self.errors.get() + msg.errors.as_i64().unwrap_or(0));
        self.overflows.set(self.overflows.get() + msg.overflows.as_i64().unwrap_or(0));
        for sample in &msg.data {
            let time = sample.time;
            if self.min_time.get() <= time && time <= self.max_time.get() {
                self.samples.borrow_mut().push(sample.clone());
            }
            if time > self.max_time.get() {
                self.complete();
            }
        }
        if self.samples.borrow().len() as f64 >= self.min_count.get() {
            self.complete();
        }
        self.is_started.get()
    }

    fn finish_collecting(&self) -> (Vec<ForceSample>, CollectErrors) {
        self.is_started.set(false);
        self.min_time.set(0.0);
        self.max_time.set(f64::INFINITY);
        self.min_count.set(f64::INFINITY);
        let samples = std::mem::take(&mut *self.samples.borrow_mut());
        let errors = self.errors.replace(0);
        let overflows = self.overflows.replace(0);
        (samples, if errors != 0 || overflows != 0 { Some((errors, overflows)) } else { None })
    }

    fn collect_until_timeout(&self, timeout: f64) -> KResult<(Vec<ForceSample>, CollectErrors)> {
        self.start_collecting(None);
        // calculate print time delay and convert to reactor time
        let now = self.reactor.monotonic();
        let print_time = self.mcu.estimated_print_time(now);
        let wake_time = now + (timeout - print_time);
        if self.is_started.get() {
            let completion = self.reactor.completion::<bool>();
            *self.completion.borrow_mut() = Some(completion.clone());
            let result = completion.wait(wake_time, None)?;
            if result.is_none() {
                self.finish_collecting();
                return Err(KError::command(format!(
                    "LoadCellSampleCollector timed out! Errors: {}, Overflows: {}",
                    self.errors.get(),
                    self.overflows.get()
                )));
            }
        }
        Ok(self.finish_collecting())
    }

    // start collecting with no automatic end to collection
    pub fn start_collecting(&self, min_time: Option<f64>) {
        if self.is_started.get() {
            return;
        }
        if let Some(t) = min_time {
            self.min_time.set(t);
        }
        self.is_started.set(true);
        let w = self.me.clone();
        if let Some(lc) = self.load_cell.upgrade() {
            lc.add_client(Rc::new(move |msg| w.upgrade().map(|c| c.on_samples(msg)).unwrap_or(false)));
        }
    }

    // stop collecting immediately and return results
    pub fn stop_collecting(&self) -> (Vec<ForceSample>, CollectErrors) {
        self.finish_collecting()
    }

    // block execution until at least min_count samples are collected
    // will return all samples collected, not just up to min_count
    pub fn collect_min(&self, min_count: i64) -> KResult<(Vec<ForceSample>, CollectErrors)> {
        self.min_count.set(min_count as f64);
        if self.samples.borrow().len() as i64 >= min_count {
            return Ok(self.finish_collecting());
        }
        let print_time = self.mcu.estimated_print_time(self.reactor.monotonic());
        let start_time = py_max(print_time, self.min_time.get());
        let lc = self.load_cell.upgrade().ok_or_else(|| KError::Other("load cell is gone".into()))?;
        let sps = lc.sensor.get_samples_per_second();
        self.collect_until_timeout(start_time + 1.0 + (min_count as f64 / sps as f64))
    }

    // returns when a sample is collected with a timestamp after print_time
    pub fn collect_until(&self, print_time: f64) -> KResult<(Vec<ForceSample>, CollectErrors)> {
        self.max_time.set(print_time);
        if self.samples.borrow().last().map(|s| s.time >= print_time).unwrap_or(false) {
            return Ok(self.finish_collecting());
        }
        self.collect_until_timeout(self.max_time.get() + 1.0)
    }
}

// Printer class that controls the load cell
const MIN_COUNTS_PER_GRAM: f64 = 1.0;

struct ForceFormatter {
    counts: i64,
    percent: f64,
    force: Option<f64>,
}

impl ForceFormatter {
    fn format(&self, show_force: bool, show_counts: bool) -> String {
        let mut parts: Vec<String> = vec![];
        if show_force {
            match self.force {
                None => parts.push("---.-g".into()),
                Some(f) => parts.push(format!("{:.1}g", f)),
            }
            parts.push("/".into());
        }
        parts.push(format!("{:.2}%", self.percent));
        if show_counts {
            parts.push(format!("({})", self.counts));
        }
        parts.join(" ")
    }
}

// enum of zero references for force calculations
#[derive(Clone, Copy, PartialEq)]
pub enum ZeroReference {
    /// the raw value 0 from the sensor
    Zero,
    /// the reference_tare_counts value
    ReferenceTare,
    /// the tare_counts value
    Tare,
}

struct ForceReporter<'a> {
    channel_counts: Vec<i64>,
    counts: i64,
    gcmd: &'a GCodeCommand,
    load_cell: Rc<LoadCell>,
}

impl<'a> ForceReporter<'a> {
    fn new(gcmd: &'a GCodeCommand, load_cell: &Rc<LoadCell>) -> KResult<Self> {
        let channel_counts = load_cell.avg_counts(None)?;
        let counts = channel_counts.iter().sum();
        Ok(ForceReporter { channel_counts, counts, gcmd, load_cell: load_cell.clone() })
    }

    // build the summed measurement against the zero reference
    fn summed(&self, reference: ZeroReference) -> ForceFormatter {
        let lc = &self.load_cell;
        ForceFormatter { counts: self.counts, percent: lc.counts_to_percent(self.counts), force: lc.counts_to_grams(self.counts, reference, None) }
    }

    // build the per-channel breakdown against the zero reference
    fn channels(&self, reference: ZeroReference) -> Vec<ForceFormatter> {
        let lc = &self.load_cell;
        self.channel_counts
            .iter()
            .enumerate()
            .map(|(idx, ch_counts)| ForceFormatter {
                counts: *ch_counts,
                percent: lc.channel_counts_to_percent(*ch_counts),
                force: lc.counts_to_grams(*ch_counts, reference, Some(idx)),
            })
            .collect()
    }

    // report the summed measurement against the zero reference,
    // optionally with a per-channel breakdown
    fn report(&self, reference: ZeroReference, prefix: &str, show_force: bool, show_channels: bool) {
        let summed = self.summed(reference);
        self.gcmd.respond_info(&format!("{}{}", prefix, summed.format(show_force, true)), true);
        // single channel sensors don't show a per-channel breakdown
        if !show_channels || self.channel_counts.len() == 1 {
            return;
        }
        for (idx, channel) in self.channels(reference).iter().enumerate() {
            let line = channel.format(show_force, true);
            self.gcmd.respond_info(&format!("    Channel {}: {}", idx, line), true);
        }
    }
}

pub struct LoadCell {
    me: Weak<LoadCell>,
    printer: Rc<Printer>,
    config_name: String,
    pub name: String,
    pub sensor: Rc<dyn BulkAdcSensor>,
    channel_count: usize,
    buffer_size: usize,
    force_buffer: RefCell<VecDeque<f64>>,
    channel_force_buffers: RefCell<Vec<VecDeque<f64>>>,
    reference_tare_counts_per_channel: RefCell<Option<Vec<i64>>>,
    reference_tare_counts: Cell<Option<i64>>,
    tare_counts_per_channel: RefCell<Option<Vec<i64>>>,
    tare_counts: Cell<Option<i64>>,
    tare_force: Cell<Option<f64>>,
    counts_per_gram: Cell<Option<f64>>,
    invert: f64,
    clients: Rc<ApiClientHelper>,
}

impl LoadCell {
    pub fn new(config: &ConfigWrapper, sensor: Rc<dyn BulkAdcSensor>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let config_name = config.get_name().to_string();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let channel_count = sensor.get_channel_count();
        let buffer_size = (sensor.get_samples_per_second() / 2) as usize;
        let ref_tare = config.getintlist("reference_tare_counts", crate::configfile::Dflt::None, ",", None)?;
        if let Some(rt) = &ref_tare {
            if rt.len() != channel_count {
                return Err(KError::config(format!("reference_tare_counts has {} values, expected {}", rt.len(), channel_count)));
            }
        }
        let reference_tare_counts = ref_tare.as_ref().map(|rt| rt.iter().sum::<i64>());
        let counts_per_gram = config.getfloat_opt("counts_per_gram", Lim::new().min(MIN_COUNTS_PER_GRAM))?;
        let invert = config.getchoice("sensor_orientation", &[("normal", 1.0), ("inverted", -1.0)], "normal")?;
        let clients = ApiClientHelper::new(&printer);
        let lc = Rc::new_cyclic(|me| LoadCell {
            me: me.clone(),
            printer: printer.clone(),
            config_name,
            name: name.clone(),
            sensor,
            channel_count,
            buffer_size,
            force_buffer: RefCell::new(VecDeque::new()),
            channel_force_buffers: RefCell::new(vec![VecDeque::new(); channel_count]),
            reference_tare_counts_per_channel: RefCell::new(ref_tare.clone()),
            reference_tare_counts: Cell::new(reference_tare_counts),
            tare_counts_per_channel: RefCell::new(ref_tare),
            tare_counts: Cell::new(reference_tare_counts),
            tare_force: Cell::new(reference_tare_counts.map(|_| 0.0)),
            counts_per_gram: Cell::new(counts_per_gram),
            invert,
            clients: clients.clone(),
        });
        LoadCellCommandHelper::new(config, &lc)?;
        // Client support:
        let mut header = IndexMap::new();
        header.insert("header".to_string(), PyVal::List(header_labels(channel_count).into_iter().map(PyVal::Str).collect()));
        clients.add_mux_endpoint("load_cell/dump_force", "load_cell", &name, PyVal::Dict(header))?;
        // startup, when klippy is ready, start capturing data
        let w = Rc::downgrade(&lc);
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                if let Some(lc) = w.upgrade() {
                    lc.handle_ready();
                }
                Ok(())
            }),
        );
        Ok(lc)
    }

    fn rc(&self) -> Rc<LoadCell> {
        self.me.upgrade().expect("load_cell")
    }

    fn handle_do_ready(&self) -> KResult<()> {
        let w = self.me.clone();
        self.sensor.add_client(Rc::new(move |msg| w.upgrade().map(|lc| lc.sensor_data_event(msg)).unwrap_or(false)))?;
        let w = self.me.clone();
        self.add_client(Rc::new(move |msg| w.upgrade().map(|lc| lc.track_force(msg)).unwrap_or(false)));
        // announce calibration status on ready
        let me = self.rc();
        if self.is_calibrated() {
            self.printer.send_event("load_cell:calibrate", &[&me])?;
        }
        if self.is_tared() {
            self.printer.send_event("load_cell:tare", &[&me])?;
        }
        Ok(())
    }

    fn handle_ready(&self) {
        let w = self.me.clone();
        self.printer.get_reactor().register_callback(
            move |_eventtime| {
                if let Some(lc) = w.upgrade() {
                    if let Err(e) = lc.handle_do_ready() {
                        logging::exception_err("load_cell ready", &e);
                    }
                }
            },
            crate::reactor::NOW,
        );
    }

    // convert raw counts to grams and broadcast to clients
    fn sensor_data_event(&self, msg: &PyVal) -> bool {
        let PyVal::Dict(d) = msg else { return false };
        let errors = d.get("errors").cloned().unwrap_or(PyVal::None);
        let overflows = d.get("overflows").cloned().unwrap_or(PyVal::None);
        let data = match d.get("data") {
            Some(PyVal::List(rows)) => rows,
            _ => return false,
        };
        let mut samples = vec![];
        for row in data {
            let items = match row {
                PyVal::List(v) | PyVal::Tuple(v) => v.as_slice(),
                _ => &[],
            };
            let channel_counts: Vec<i64> = items.iter().skip(1).step_by(2).map(|v| v.as_i64().unwrap_or(0)).collect();
            if channel_counts.len() != self.channel_count {
                logging::info(&format!("Missing Sensor Channels! Expected: {}, found {}", self.channel_count, channel_counts.len()));
            }
            let sum_counts: i64 = channel_counts.iter().sum();
            let time = items.first().and_then(|v| v.as_f64()).unwrap_or(0.0);
            let channels =
                channel_counts.iter().enumerate().map(|(idx, counts)| (self.counts_to_grams(*counts, ZeroReference::Tare, Some(idx)), *counts)).collect();
            samples.push(ForceSample {
                time,
                force: self.counts_to_grams(sum_counts, ZeroReference::Tare, None),
                counts: sum_counts,
                tare_counts: self.tare_counts.get(),
                channels,
            });
        }
        self.clients.send(&ForceMsg { data: samples, errors, overflows });
        true
    }

    // get internal events of force data
    pub fn add_client(&self, callback: ForceClient) {
        self.clients.add_client(callback);
    }

    pub fn tare(&self, tare_counts_per_channel: &[i64]) -> KResult<()> {
        *self.tare_counts_per_channel.borrow_mut() = Some(tare_counts_per_channel.to_vec());
        let tare_counts: i64 = tare_counts_per_channel.iter().sum();
        self.tare_counts.set(Some(tare_counts));
        if self.is_calibrated() {
            let tare_delta = (tare_counts - self.reference_tare_counts.get().unwrap_or(0)) as f64;
            self.tare_force.set(Some(py_round(self.invert * (tare_delta / self.counts_per_gram.get().unwrap_or(1.0)), 1)));
        }
        let me = self.rc();
        self.printer.send_event("load_cell:tare", &[&me])
    }

    pub fn set_reference_tare_counts(&self, tare_counts_per_channel: Option<Vec<i64>>, save: bool) -> KResult<()> {
        let Some(tare) = tare_counts_per_channel else {
            return Err(KError::command("Missing tare counts"));
        };
        *self.reference_tare_counts_per_channel.borrow_mut() = Some(tare.clone());
        self.reference_tare_counts.set(Some(tare.iter().sum()));
        self.tare(&tare)?;
        if save {
            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            let value = tare.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(", ");
            configfile.pconfig.borrow_mut().set(&self.config_name, "reference_tare_counts", &value)?;
        }
        Ok(())
    }

    pub fn set_calibration(&self, counts_per_gram: Option<f64>, reference_tare_counts_per_channel: Option<Vec<i64>>, save: bool) -> KResult<()> {
        let cpg = match counts_per_gram {
            Some(c) if c.abs() >= MIN_COUNTS_PER_GRAM => c,
            _ => return Err(KError::command("Invalid counts per gram value")),
        };
        self.counts_per_gram.set(Some(cpg));
        self.set_reference_tare_counts(reference_tare_counts_per_channel, save)?;
        if save {
            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            configfile.pconfig.borrow_mut().set(&self.config_name, "counts_per_gram", &format!("{:.5}", cpg))?;
        }
        let me = self.rc();
        self.printer.send_event("load_cell:calibrate", &[&me])
    }

    pub fn counts_to_grams(&self, counts: i64, reference: ZeroReference, channel: Option<usize>) -> Option<f64> {
        if !self.is_calibrated() {
            return None;
        }
        let zeros = vec![0; self.channel_count];
        let ref_tare = match reference {
            ZeroReference::ReferenceTare => self.reference_tare_counts_per_channel.borrow().clone().unwrap_or(zeros),
            ZeroReference::Tare => self.tare_counts_per_channel.borrow().clone().unwrap_or(zeros),
            ZeroReference::Zero => zeros,
        };
        let ref_counts: i64 = match channel {
            None => ref_tare.iter().sum(),
            Some(c) => ref_tare.get(c).copied().unwrap_or(0),
        };
        let sample_delta = (counts - ref_counts) as f64;
        Some(self.invert * (sample_delta / self.counts_per_gram.get().unwrap_or(1.0)))
    }

    // The maximum range of a single ADC channel, based on its bit width
    pub fn channel_saturation_range(&self) -> (i64, i64) {
        self.sensor.get_range()
    }

    // The maximum range for the sum of all channels
    pub fn saturation_range(&self) -> (i64, i64) {
        let (range_min, range_max) = self.channel_saturation_range();
        (range_min * self.channel_count as i64, range_max * self.channel_count as i64)
    }

    // convert raw counts to a +/- percentage of the sensors total range
    pub fn counts_to_percent(&self, counts: i64) -> f64 {
        let (_, range_max) = self.saturation_range();
        (counts as f64 / range_max as f64) * 100.0
    }

    pub fn channel_counts_to_percent(&self, counts: i64) -> f64 {
        let (_, range_max) = self.channel_saturation_range();
        (counts as f64 / range_max as f64) * 100.0
    }

    // read 1 second of load cell data and average it
    // performs safety checks for errors and saturation
    pub fn avg_counts(&self, num_samples: Option<i64>) -> KResult<Vec<i64>> {
        let num_samples = num_samples.unwrap_or_else(|| self.sensor.get_samples_per_second());
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        // callers expect a load from the context of the call, all moves must
        // finish before samples are valid
        let collector = self.get_collector();
        collector.start_collecting(Some(toolhead.get_last_move_time()?));
        let (samples, errors) = collector.collect_min(num_samples)?;
        self.validate_samples(&samples, errors)?;
        Ok(self.avg_counts_from_samples(&samples))
    }

    pub fn validate_samples(&self, samples: &[ForceSample], errors: CollectErrors) -> KResult<()> {
        if let Some((e, o)) = errors {
            return Err(KError::command(format!("Sensor reported {} errors while sampling", e + o)));
        }
        // check individual channels for saturated readings
        let (range_min, range_max) = self.channel_saturation_range();
        for sample in samples {
            for (_, counts) in &sample.channels {
                if *counts >= range_max || *counts <= range_min {
                    return Err(KError::command("Some samples are saturated (+/-100%)"));
                }
            }
        }
        Ok(())
    }

    // Provide ongoing force tracking/averaging for status updates
    fn track_force(&self, msg: &ForceMsg) -> bool {
        if !(self.is_calibrated() && self.is_tared()) {
            return true;
        }
        let push = |buf: &mut VecDeque<f64>, v: f64| {
            if buf.len() >= self.buffer_size {
                buf.pop_front();
            }
            if self.buffer_size > 0 {
                buf.push_back(v);
            }
        };
        for sample in &msg.data {
            push(&mut self.force_buffer.borrow_mut(), sample.force.unwrap_or(0.0));
            let mut bufs = self.channel_force_buffers.borrow_mut();
            for idx in 0..self.channel_count {
                let f = sample.channels.get(idx).and_then(|c| c.0).unwrap_or(0.0);
                push(&mut bufs[idx], f);
            }
        }
        true
    }

    fn force_g(&self) -> Option<IndexMap<String, PyVal>> {
        let fb = self.force_buffer.borrow();
        if self.is_calibrated() && self.is_tared() && !fb.is_empty() {
            let mut channel_force = vec![];
            for buf in self.channel_force_buffers.borrow().iter() {
                if buf.is_empty() {
                    return None;
                }
                let v: Vec<f64> = buf.iter().copied().collect();
                channel_force.push(PyVal::Float(py_round(avg(&v), 1)));
            }
            let v: Vec<f64> = fb.iter().copied().collect();
            let mut d = IndexMap::new();
            d.insert("force_g".to_string(), PyVal::Float(py_round(avg(&v), 1)));
            d.insert("force_g_per_channel".to_string(), PyVal::List(channel_force));
            d.insert("min_force_g".to_string(), PyVal::Float(py_round(v.iter().copied().reduce(py_min).unwrap_or(0.0), 1)));
            d.insert("max_force_g".to_string(), PyVal::Float(py_round(v.iter().copied().reduce(py_max).unwrap_or(0.0), 1)));
            return Some(d);
        }
        Some(IndexMap::new())
    }

    pub fn is_tared(&self) -> bool {
        self.tare_counts.get().is_some()
    }

    pub fn is_calibrated(&self) -> bool {
        self.counts_per_gram.get().is_some() && self.reference_tare_counts.get().is_some()
    }

    pub fn get_reference_tare_counts(&self) -> Option<i64> {
        self.reference_tare_counts.get()
    }

    pub fn get_tare_counts(&self) -> Option<i64> {
        self.tare_counts.get()
    }

    pub fn get_counts_per_gram(&self) -> Option<f64> {
        self.counts_per_gram.get()
    }

    // convert samples into average per channel
    fn avg_counts_from_samples(&self, samples: &[ForceSample]) -> Vec<i64> {
        (0..self.channel_count)
            .map(|idx| {
                let column: Vec<i64> = samples.iter().map(|s| s.channels.get(idx).map(|c| c.1).unwrap_or(0)).collect();
                avg_int(&column).trunc() as i64
            })
            .collect()
    }

    pub fn get_collector(&self) -> Rc<LoadCellSampleCollector> {
        LoadCellSampleCollector::new(&self.printer, &self.rc())
    }
}

impl PrinterObject for LoadCell {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        // (python's None.update() raises when a channel buffer is empty)
        let mut status = self.force_g()?;
        let list = |v: &Option<Vec<i64>>| v.as_ref().map(|v| PyVal::List(v.iter().map(|c| PyVal::Int(*c)).collect())).unwrap_or(PyVal::None);
        status.insert("is_calibrated".to_string(), PyVal::Bool(self.is_calibrated()));
        status.insert("counts_per_gram".to_string(), opt_f(self.counts_per_gram.get()));
        status.insert("reference_tare_counts".to_string(), opt_i(self.reference_tare_counts.get()));
        status.insert("reference_tare_counts_per_channel".to_string(), list(&self.reference_tare_counts_per_channel.borrow()));
        status.insert("tare_counts".to_string(), opt_i(self.tare_counts.get()));
        status.insert("tare_counts_per_channel".to_string(), list(&self.tare_counts_per_channel.borrow()));
        status.insert("tare_force".to_string(), opt_f(self.tare_force.get()));
        Some(PyVal::Dict(status))
    }
    fn teardown(&self) {
        self.clients.client_cbs.borrow_mut().clear();
    }
}
