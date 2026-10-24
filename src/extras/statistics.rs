// Support for logging periodic statistics (port of klippy/extras/statistics.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::danger_options::get_danger_options;
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::io::{Read, Seek};
use std::rc::Rc;

pub struct PrinterSysStats {
    last_process_time: Cell<f64>,
    total_process_time: Cell<f64>,
    last_load_avg: Cell<f64>,
    last_mem_avail: Cell<i64>,
    mem_file: RefCell<Option<std::fs::File>>,
}

/// time.process_time()
fn process_time() -> f64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_PROCESS_CPUTIME_ID, &mut ts) };
    ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9
}

/// os.getloadavg()[0]
fn getloadavg() -> f64 {
    let mut loadavg = [0f64; 3];
    unsafe { libc::getloadavg(loadavg.as_mut_ptr(), 3) };
    loadavg[0]
}

impl PrinterSysStats {
    fn new(config: &ConfigWrapper) -> Rc<Self> {
        let printer = config.get_printer();
        let s = Rc::new(PrinterSysStats {
            last_process_time: Cell::new(0.0),
            total_process_time: Cell::new(0.0),
            last_load_avg: Cell::new(0.0),
            last_mem_avail: Cell::new(0),
            mem_file: RefCell::new(std::fs::File::open("/proc/meminfo").ok()),
        });
        let sc = s.clone();
        printer.register_event_handler(
            "klippy:disconnect",
            Rc::new(move |_| {
                *sc.mem_file.borrow_mut() = None;
                Ok(())
            }),
        );
        s
    }

    fn read_mem_avail(&self) -> Option<i64> {
        let mut mem_file = self.mem_file.borrow_mut();
        let f = mem_file.as_mut()?;
        f.rewind().ok()?;
        let mut data = String::new();
        f.read_to_string(&mut data).ok()?;
        for line in data.split('\n') {
            if line.starts_with("MemAvailable:") {
                return line.split_whitespace().nth(1).and_then(crate::pyfmt::py_int);
            }
        }
        None
    }
}

impl PrinterObject for PrinterSysStats {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        // Get core usage stats
        let ptime = process_time();
        let pdiff = ptime - self.last_process_time.get();
        self.last_process_time.set(ptime);
        if pdiff > 0.0 {
            self.total_process_time.set(self.total_process_time.get() + pdiff);
        }
        self.last_load_avg.set(getloadavg());
        let mut msg = format!("sysload={:.2} cputime={:.3}", self.last_load_avg.get(), self.total_process_time.get());
        // Get available system memory
        if let Some(mem_avail) = self.read_mem_avail() {
            self.last_mem_avail.set(mem_avail);
            msg = format!("{} memavail={}", msg, mem_avail);
        }
        Some((false, msg))
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("sysload".to_string(), PyVal::Float(self.last_load_avg.get()));
        d.insert("cputime".to_string(), PyVal::Float(self.total_process_time.get()));
        d.insert("memavail".to_string(), PyVal::Int(self.last_mem_avail.get()));
        Some(PyVal::Dict(d))
    }
}

pub struct PrinterStats {
    printer: Rc<Printer>,
    stats_timer: RefCell<Option<Timer>>,
    stats_cb: RefCell<Vec<PObj>>,
}

impl PrinterStats {
    fn new(config: &ConfigWrapper) -> Rc<Self> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let ps = Rc::new(PrinterStats { printer: printer.clone(), stats_timer: RefCell::new(None), stats_cb: RefCell::new(vec![]) });
        let w = Rc::downgrade(&ps);
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(ps) => ps.generate_stats(eventtime),
                None => NEVER,
            }),
            NEVER,
        );
        *ps.stats_timer.borrow_mut() = Some(timer);
        let p = ps.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                p.handle_ready();
                Ok(())
            }),
        );
        ps
    }

    fn handle_ready(&self) {
        *self.stats_cb.borrow_mut() =
            self.printer.lookup_objects(None).into_iter().filter(|(_, o)| o.has_stats()).map(|(_, o)| o).collect();
        if self.printer.start_arg("debugoutput").is_none() {
            let reactor = self.printer.get_reactor();
            if let Some(t) = self.stats_timer.borrow().as_ref() {
                reactor.update_timer(t, NOW);
            }
        }
    }

    fn generate_stats(&self, eventtime: f64) -> f64 {
        let cbs = self.stats_cb.borrow().clone();
        let stats: Vec<(bool, String)> = cbs.iter().filter_map(|o| o.stats(eventtime)).collect();
        if stats.iter().any(|s| s.0) && get_danger_options().log_statistics {
            let stats_str: Vec<&str> = stats.iter().filter(|s| !s.1.is_empty()).map(|s| s.1.as_str()).collect();
            logging::info(&format!("Stats {:.1}: {}", eventtime, stats_str.join(" ")));
        }
        eventtime + 1.0
    }
}

impl PrinterObject for PrinterStats {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        self.stats_cb.borrow_mut().clear();
        if let Some(t) = self.stats_timer.borrow_mut().take() {
            self.printer.get_reactor().forget_timer(&t);
        }
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    config.get_printer().add_object("system_stats", PrinterSysStats::new(config))?;
    Ok(PrinterStats::new(config))
}
