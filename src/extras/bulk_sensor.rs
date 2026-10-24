// Tools for reading bulk sensor data from the mcu
// (port of klippy/extras/bulk_sensor.py)
//
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::serialqueue::CommandQueue;
use crate::error::{KError, KResult};
use crate::logging;
use crate::mcu::{CommandQueryWrapper, Mcu};
use crate::msgproto::{Params, Val};
use crate::printer::Printer;
use crate::pyfmt::py_max;
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER};
use crate::webhooks::{ClientConnection, WebHooks, WebRequest};
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const BATCH_INTERVAL: f64 = 0.500;

/// batch_cb(eventtime) -> msg (an empty dict when there is nothing)
pub type BatchCallback = Rc<dyn Fn(f64) -> KResult<PyVal>>;
pub type StartStopCallback = Rc<dyn Fn() -> KResult<()>>;
/// client_cb(msg) -> keep receiving
pub type ClientCallback = Rc<dyn Fn(&PyVal) -> bool>;

// Helper to process accumulated messages in periodic batches
pub struct BatchBulkHelper {
    me: Weak<BatchBulkHelper>,
    printer: Rc<Printer>,
    batch_cb: BatchCallback,
    start_cb: Option<StartStopCallback>,
    stop_cb: Option<StartStopCallback>,
    pub(crate) is_started: Cell<bool>,
    batch_interval: f64,
    batch_timer: RefCell<Option<Timer>>,
    client_cbs: RefCell<Vec<ClientCallback>>,
    webhooks_start_resp: RefCell<PyVal>,
}

impl BatchBulkHelper {
    pub fn new(
        printer: &Rc<Printer>,
        batch_cb: BatchCallback,
        start_cb: Option<StartStopCallback>,
        stop_cb: Option<StartStopCallback>,
        batch_interval: f64,
    ) -> Rc<Self> {
        Rc::new_cyclic(|me| BatchBulkHelper {
            me: me.clone(),
            printer: printer.clone(),
            batch_cb,
            start_cb,
            stop_cb,
            is_started: Cell::new(false),
            batch_interval,
            batch_timer: RefCell::new(None),
            client_cbs: RefCell::new(vec![]),
            webhooks_start_resp: RefCell::new(PyVal::dict()),
        })
    }

    // Periodic batch processing
    fn start(&self) -> KResult<()> {
        if self.is_started.get() {
            return Ok(());
        }
        self.is_started.set(true);
        if let Some(cb) = &self.start_cb {
            if let Err(e) = cb() {
                if e.is_command_error() {
                    logging::exception_err("BatchBulkHelper start callback error", &e);
                    self.is_started.set(false);
                    self.client_cbs.borrow_mut().clear();
                }
                return Err(e);
            }
        }
        let reactor = self.printer.get_reactor();
        let systime = reactor.monotonic();
        let waketime = systime + self.batch_interval;
        let w = self.me.clone();
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(b) => b.proc_batch(eventtime),
                None => NEVER,
            }),
            waketime,
        );
        *self.batch_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn stop(&self) {
        self.client_cbs.borrow_mut().clear();
        if let Some(t) = self.batch_timer.borrow_mut().take() {
            self.printer.get_reactor().unregister_timer(&t);
        }
        if !self.is_started.get() {
            return;
        }
        if let Some(cb) = &self.stop_cb {
            if let Err(e) = cb() {
                logging::exception_err("BatchBulkHelper stop callback error", &e);
                self.client_cbs.borrow_mut().clear();
            }
        }
        self.is_started.set(false);
        if !self.client_cbs.borrow().is_empty() {
            // New client started while in process of stopping
            if let Err(e) = self.start() {
                logging::exception_err("BatchBulkHelper start callback error", &e);
            }
        }
    }

    fn proc_batch(&self, eventtime: f64) -> f64 {
        let msg = match (self.batch_cb)(eventtime) {
            Ok(m) => m,
            Err(e) => {
                logging::exception_err("BatchBulkHelper batch callback error", &e);
                self.stop();
                return NEVER;
            }
        };
        if !msg.truthy() {
            return eventtime + self.batch_interval;
        }
        let cbs = self.client_cbs.borrow().clone();
        for client_cb in cbs {
            let res = client_cb(&msg);
            if !res {
                // This client no longer needs updates - unregister it
                let mut c = self.client_cbs.borrow_mut();
                if let Some(pos) = c.iter().position(|cb| Rc::ptr_eq(cb, &client_cb)) {
                    c.remove(pos);
                }
                let empty = c.is_empty();
                drop(c);
                if empty {
                    self.stop();
                    return NEVER;
                }
            }
        }
        eventtime + self.batch_interval
    }

    // Client registration
    pub fn add_client(&self, client_cb: ClientCallback) -> KResult<()> {
        self.client_cbs.borrow_mut().push(client_cb);
        self.start()
    }

    // Webhooks registration
    fn add_api_client(&self, web_request: &WebRequest) -> KResult<()> {
        let whbatch = BatchWebhooksClient::new(web_request)?;
        self.add_client(Rc::new(move |msg| whbatch.handle_batch(msg)))?;
        web_request.send(self.webhooks_start_resp.borrow().clone())
    }

    pub fn add_mux_endpoint(&self, path: &str, key: &str, value: &str, webhooks_start_resp: PyVal) -> KResult<()> {
        *self.webhooks_start_resp.borrow_mut() = webhooks_start_resp;
        let wh = self.printer.lookup::<WebHooks>("webhooks")?;
        let w = self.me.clone();
        wh.register_mux_endpoint(
            path,
            key,
            Some(value),
            Rc::new(move |wr| match w.upgrade() {
                Some(b) => b.add_api_client(wr),
                None => Ok(()),
            }),
        )
    }

    /// not in python: drop the timer and clients
    pub fn teardown(&self) {
        self.client_cbs.borrow_mut().clear();
        if let Some(t) = self.batch_timer.borrow_mut().take() {
            self.printer.get_reactor().forget_timer(&t);
        }
    }
}

// A webhooks wrapper for use by BatchBulkHelper
pub struct BatchWebhooksClient {
    cconn: Option<Rc<ClientConnection>>,
    template: PyVal,
}

impl BatchWebhooksClient {
    pub fn new(web_request: &WebRequest) -> KResult<Self> {
        Ok(BatchWebhooksClient {
            cconn: web_request.get_client_connection(),
            template: web_request.get_dict("response_template", Some(PyVal::dict()))?,
        })
    }

    pub fn handle_batch(&self, msg: &PyVal) -> bool {
        let Some(cconn) = &self.cconn else { return false };
        if cconn.is_closed() {
            return false;
        }
        let mut tmp = match &self.template {
            PyVal::Dict(d) => d.clone(),
            _ => Default::default(),
        };
        tmp.insert("params".to_string(), msg.clone());
        cconn.send(&PyVal::Dict(tmp));
        true
    }
}

// Helper class to store incoming messages in a queue
pub struct BulkDataQueue {
    // Measurement storage (python: accessed from a background thread)
    raw_samples: Rc<RefCell<Vec<Params>>>,
}

impl BulkDataQueue {
    pub fn new(mcu: &Mcu, msg_name: &str, oid: Option<i64>) -> Self {
        let raw_samples: Rc<RefCell<Vec<Params>>> = Rc::new(RefCell::new(vec![]));
        // Register callback with mcu
        let rs = raw_samples.clone();
        mcu.register_response(
            Some(Rc::new(move |params: &Params| {
                rs.borrow_mut().push(params.clone());
                Ok(())
            })),
            msg_name,
            oid,
        );
        BulkDataQueue { raw_samples }
    }

    pub fn pull_queue(&self) -> Vec<Params> {
        std::mem::take(&mut *self.raw_samples.borrow_mut())
    }

    pub fn clear_queue(&self) {
        self.pull_queue();
    }
}

// Clock synchronization
//
// It is common for sensors to produce measurements at a fixed
// frequency.  If the mcu can reliably obtain all of these
// measurements, then the code here can calculate a precision timestamp
// for them.  That is, it can determine the actual sensor measurement
// frequency, the time of the first measurement, and thus a precise
// time for all measurements.
//
// This system works by having the mcu periodically report a precision
// timestamp along with the total number of measurements the sensor has
// taken as of that time.  In brief, knowing the total number of
// measurements taken over an extended period provides an accurate
// estimate of measurement frequency, which can then also be utilized
// to determine the time of the first measurement.

// Helper class for chip clock synchronization via linear regression
pub struct ClockSyncRegression {
    mcu: Rc<Mcu>,
    chip_clock_smooth: f64,
    decay: f64,
    last_chip_clock: Cell<f64>,
    last_exp_mcu_clock: Cell<f64>,
    mcu_clock_avg: Cell<f64>,
    mcu_clock_variance: Cell<f64>,
    chip_clock_avg: Cell<f64>,
    chip_clock_covariance: Cell<f64>,
}

impl ClockSyncRegression {
    pub fn new(mcu: &Rc<Mcu>, chip_clock_smooth: f64, decay: f64) -> Self {
        ClockSyncRegression {
            mcu: mcu.clone(),
            chip_clock_smooth,
            decay,
            last_chip_clock: Cell::new(0.0),
            last_exp_mcu_clock: Cell::new(0.0),
            mcu_clock_avg: Cell::new(0.0),
            mcu_clock_variance: Cell::new(0.0),
            chip_clock_avg: Cell::new(0.0),
            chip_clock_covariance: Cell::new(0.0),
        }
    }

    pub fn reset(&self, mcu_clock: f64, chip_clock: f64) {
        self.mcu_clock_avg.set(mcu_clock);
        self.chip_clock_avg.set(chip_clock);
        self.mcu_clock_variance.set(0.0);
        self.chip_clock_covariance.set(0.0);
        self.last_chip_clock.set(0.0);
        self.last_exp_mcu_clock.set(0.0);
    }

    pub fn update(&self, mcu_clock: f64, chip_clock: f64) {
        // Update linear regression
        let decay = self.decay;
        let diff_mcu_clock = mcu_clock - self.mcu_clock_avg.get();
        self.mcu_clock_avg.set(self.mcu_clock_avg.get() + decay * diff_mcu_clock);
        self.mcu_clock_variance.set((1.0 - decay) * (self.mcu_clock_variance.get() + diff_mcu_clock.powi(2) * decay));
        let diff_chip_clock = chip_clock - self.chip_clock_avg.get();
        self.chip_clock_avg.set(self.chip_clock_avg.get() + decay * diff_chip_clock);
        self.chip_clock_covariance
            .set((1.0 - decay) * (self.chip_clock_covariance.get() + diff_mcu_clock * diff_chip_clock * decay));
    }

    pub fn set_last_chip_clock(&self, chip_clock: f64) -> KResult<()> {
        let (base_mcu, base_chip, inv_cfreq) = self.get_clock_translation()?;
        self.last_chip_clock.set(chip_clock);
        self.last_exp_mcu_clock.set(base_mcu + (chip_clock - base_chip) * inv_cfreq);
        Ok(())
    }

    pub fn get_clock_translation(&self) -> KResult<(f64, f64, f64)> {
        if self.chip_clock_covariance.get() == 0.0 {
            return Err(KError::Other("float division by zero".into()));
        }
        let inv_chip_freq = self.mcu_clock_variance.get() / self.chip_clock_covariance.get();
        if self.last_chip_clock.get() == 0.0 {
            return Ok((self.mcu_clock_avg.get(), self.chip_clock_avg.get(), inv_chip_freq));
        }
        // Find mcu clock associated with future chip_clock
        let s_chip_clock = self.last_chip_clock.get() + self.chip_clock_smooth;
        let scdiff = s_chip_clock - self.chip_clock_avg.get();
        let s_mcu_clock = self.mcu_clock_avg.get() + scdiff * inv_chip_freq;
        // Calculate frequency to converge at future point
        let mdiff = s_mcu_clock - self.last_exp_mcu_clock.get();
        let s_inv_chip_freq = mdiff / self.chip_clock_smooth;
        Ok((self.last_exp_mcu_clock.get(), self.last_chip_clock.get(), s_inv_chip_freq))
    }

    pub fn get_time_translation(&self) -> KResult<(f64, f64, f64)> {
        let (base_mcu, base_chip, inv_cfreq) = self.get_clock_translation()?;
        let base_time = self.mcu.clock_to_print_time(base_mcu);
        let inv_freq = self.mcu.clock_to_print_time(base_mcu + inv_cfreq) - base_time;
        Ok((base_time, base_chip, inv_freq))
    }
}

pub const MAX_BULK_MSG_SIZE: usize = 51;

/// a struct.Struct format of fixed size integer fields ("BBBBB", ">hhh")
struct SampleFormat {
    big_endian: bool,
    fields: Vec<(usize, bool)>,
}

impl SampleFormat {
    fn new(fmt: &str) -> Self {
        let (big_endian, body) = match fmt.chars().next() {
            Some('>') | Some('!') => (true, &fmt[1..]),
            Some('<') | Some('=') | Some('@') => (false, &fmt[1..]),
            _ => (false, fmt),
        };
        let fields = body
            .chars()
            .map(|c| match c {
                'b' => (1, true),
                'B' => (1, false),
                'h' => (2, true),
                'H' => (2, false),
                'i' | 'l' => (4, true),
                'I' | 'L' => (4, false),
                _ => panic!("unsupported struct format {}", fmt),
            })
            .collect();
        SampleFormat { big_endian, fields }
    }

    fn size(&self) -> usize {
        self.fields.iter().map(|f| f.0).sum()
    }

    fn unpack_from(&self, data: &[u8], offset: usize) -> Vec<i64> {
        let mut pos = offset;
        let mut out = Vec::with_capacity(self.fields.len());
        for &(size, signed) in &self.fields {
            let b = &data[pos..pos + size];
            let mut v: u64 = 0;
            for i in 0..size {
                let byte = if self.big_endian { b[i] } else { b[size - 1 - i] };
                v = (v << 8) | byte as u64;
            }
            let bits = size * 8;
            let val = if signed && v & (1 << (bits - 1)) != 0 { v as i64 - (1i64 << bits) } else { v as i64 };
            out.push(val);
            pos += size;
        }
        out
    }
}

/// a FixedFreqReader sample: (ptime, unpacked fields)
pub type RawSample = (f64, Vec<i64>);

// Read sensor_bulk_data and calculate timestamps for devices that take
// samples at a fixed frequency (and produce fixed data size samples).
pub struct FixedFreqReader {
    mcu: Rc<Mcu>,
    clock_sync: ClockSyncRegression,
    unpack: SampleFormat,
    bytes_per_sample: usize,
    samples_per_block: usize,
    last_sequence: Cell<i64>,
    max_query_duration: Cell<i64>,
    last_overflows: Cell<i64>,
    bulk_queue: RefCell<Option<BulkDataQueue>>,
    oid: Cell<Option<i64>>,
    query_status_cmd: RefCell<Option<CommandQueryWrapper>>,
}

impl FixedFreqReader {
    pub fn new(mcu: &Rc<Mcu>, chip_clock_smooth: f64, unpack_fmt: &str) -> Self {
        let unpack = SampleFormat::new(unpack_fmt);
        let bytes_per_sample = unpack.size();
        FixedFreqReader {
            mcu: mcu.clone(),
            clock_sync: ClockSyncRegression::new(mcu, chip_clock_smooth, 1.0 / 20.0),
            unpack,
            bytes_per_sample,
            samples_per_block: MAX_BULK_MSG_SIZE / bytes_per_sample,
            last_sequence: Cell::new(0),
            max_query_duration: Cell::new(0),
            last_overflows: Cell::new(0),
            bulk_queue: RefCell::new(None),
            oid: Cell::new(None),
            query_status_cmd: RefCell::new(None),
        }
    }

    pub fn setup_query_command(&self, msgformat: &str, oid: i64, cq: Option<CommandQueue>) -> KResult<()> {
        // Lookup sensor query command (that responds with sensor_bulk_status)
        self.oid.set(Some(oid));
        *self.query_status_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            msgformat,
            "sensor_bulk_status oid=%c clock=%u query_ticks=%u next_sequence=%hu buffered=%u possible_overflows=%hu",
            Some(oid),
            cq,
            false,
        )?);
        // Read sensor_bulk_data messages and store in a queue
        *self.bulk_queue.borrow_mut() = Some(BulkDataQueue::new(&self.mcu, "sensor_bulk_data", Some(oid)));
        Ok(())
    }

    pub fn get_last_overflows(&self) -> i64 {
        self.last_overflows.get()
    }

    fn clear_duration_filter(&self) {
        self.max_query_duration.set(1 << 31);
    }

    fn queue(&self) -> KResult<std::cell::Ref<'_, BulkDataQueue>> {
        let q = self.bulk_queue.borrow();
        if q.is_none() {
            return Err(KError::Other("'NoneType' object has no attribute 'clear_queue'".into()));
        }
        Ok(std::cell::Ref::map(q, |q| q.as_ref().unwrap()))
    }

    pub fn note_start(&self) -> KResult<()> {
        self.last_sequence.set(0);
        self.last_overflows.set(0);
        // Clear local queue (clear any stale samples from previous session)
        self.queue()?.clear_queue();
        // Set initial clock
        self.clear_duration_filter();
        self.update_clock(true)?;
        self.clear_duration_filter();
        Ok(())
    }

    pub fn note_end(&self) -> KResult<()> {
        // Clear local queue (free no longer needed memory)
        self.queue()?.clear_queue();
        Ok(())
    }

    fn update_clock(&self, is_reset: bool) -> KResult<()> {
        let params = {
            let cmd = self.query_status_cmd.borrow();
            let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
            cmd.send(&[Val::Int(self.oid.get().unwrap_or(0))], 0, 0, true)?
        };
        let p = |k: &str| params.get(k).and_then(|v| v.as_int()).unwrap_or(0);
        let mcu_clock = self.mcu.clock32_to_clock64(p("clock"));
        let seq_diff = (p("next_sequence") - self.last_sequence.get()) & 0xFFFF;
        self.last_sequence.set(self.last_sequence.get() + seq_diff);
        let buffered = p("buffered");
        let po_diff = (p("possible_overflows") - self.last_overflows.get()) & 0xFFFF;
        self.last_overflows.set(self.last_overflows.get() + po_diff);
        let duration = p("query_ticks");
        if duration > self.max_query_duration.get() {
            // Skip measurement as a high query time could skew clock tracking
            let m = py_max((2 * self.max_query_duration.get()) as f64, self.mcu.seconds_to_clock(0.000005) as f64);
            self.max_query_duration.set(m as i64);
            return Ok(());
        }
        self.max_query_duration.set(2 * duration);
        let msg_count = self.last_sequence.get() * self.samples_per_block as i64 + buffered / self.bytes_per_sample as i64;
        // The "chip clock" is the message counter plus .5 for average
        // inaccuracy of query responses and plus .5 for assumed offset
        // of hardware processing time.
        let chip_clock = msg_count + 1;
        let avg_mcu_clock = mcu_clock + duration / 2;
        if is_reset {
            self.clock_sync.reset(avg_mcu_clock as f64, chip_clock as f64);
        } else {
            self.clock_sync.update(avg_mcu_clock as f64, chip_clock as f64);
        }
        Ok(())
    }

    // Convert sensor_bulk_data responses into list of samples
    pub fn pull_samples(&self) -> KResult<Vec<RawSample>> {
        // Query MCU for sample timing and update clock synchronization
        self.update_clock(false)?;
        // Pull sensor_bulk_data messages from local queue
        let raw_samples = self.queue()?.pull_queue();
        if raw_samples.is_empty() {
            return Ok(vec![]);
        }
        // Load variables to optimize inner loop below
        let last_sequence = self.last_sequence.get();
        let (time_base, chip_base, inv_freq) = self.clock_sync.get_time_translation()?;
        let bytes_per_sample = self.bytes_per_sample;
        let samples_per_block = self.samples_per_block as i64;
        // Process every message in raw_samples
        let mut seq = 0;
        let mut i = 0;
        let mut samples = Vec::with_capacity(raw_samples.len() * self.samples_per_block);
        for params in &raw_samples {
            let mut seq_diff = (params.get("sequence").and_then(|v| v.as_int()).unwrap_or(0) - last_sequence) & 0xFFFF;
            seq_diff -= (seq_diff & 0x8000) << 1;
            seq = last_sequence + seq_diff;
            let msg_cdiff = (seq * samples_per_block) as f64 - chip_base;
            let data: &[u8] = match params.get("data") {
                Some(Val::Bytes(b)) => b,
                _ => &[],
            };
            for j in 0..data.len() / bytes_per_sample {
                i = j;
                let ptime = time_base + (msg_cdiff + j as f64) * inv_freq;
                samples.push((ptime, self.unpack.unpack_from(data, j * bytes_per_sample)));
            }
        }
        self.clock_sync.set_last_chip_clock((seq * samples_per_block + i as i64) as f64)?;
        Ok(samples)
    }
}
