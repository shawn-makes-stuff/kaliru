// Helper code for low-level motion queuing and flushing
// (port of klippy/extras/motion_queuing.py)
//
// Copyright (C) 2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve;
use crate::chelper::serialqueue::SerialQueue;
use crate::chelper::steppersync::{StepperSyncMgr, StepperSyncRef, SyncEmitterRef};
use crate::chelper::trapq::{TrapQ, TrapQRef};
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::reactor::{Reactor, ReactorCompletion, Timer, NEVER, NOW};
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::Arc;

pub const BGFLUSH_LOW_TIME: f64 = 0.200;
pub const BGFLUSH_HIGH_TIME: f64 = 0.400;
pub const BGFLUSH_SG_LOW_TIME: f64 = 0.450;
pub const BGFLUSH_SG_HIGH_TIME: f64 = 0.700;
pub const BGFLUSH_EXTRA_TIME: f64 = 0.250;

pub const MOVE_HISTORY_EXPIRE: f64 = 30.0;
pub const MIN_KIN_TIME: f64 = 0.100;
pub const STEPCOMPRESS_FLUSH_TIME: f64 = 0.050;
pub const SDS_CHECK_TIME: f64 = 0.001; // step+dir+step filter in stepcompress.c

pub const DRIP_SEGMENT_TIME: f64 = 0.050;
pub const DRIP_TIME: f64 = 0.100;

/// a flush callback: cb(flush_time, step_gen_time)
pub type FlushCallback = Rc<dyn Fn(f64, f64) -> KResult<()>>;

pub struct PrinterMotionQueuing {
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    // C trapq tracking
    trapqs: RefCell<Vec<TrapQRef>>,
    // C steppersync tracking
    steppersyncmgr: RefCell<StepperSyncMgr>,
    syncemitters: RefCell<Vec<SyncEmitterRef>>,
    steppersyncs: RefCell<Vec<(Rc<Mcu>, StepperSyncRef)>>,
    // History expiration
    clear_history_time: Cell<f64>,
    // Flush notification callbacks
    flush_callbacks: RefCell<Vec<FlushCallback>>,
    // Kinematic step generation scan window time tracking
    kin_flush_delay: Cell<f64>,
    // MCU tracking
    pub all_mcus: Vec<Rc<Mcu>>,
    pub mcu: Rc<Mcu>,
    can_pause: Cell<bool>,
    // Flush tracking
    flush_timer: RefCell<Option<Timer>>,
    do_kick_flush_timer: Cell<bool>,
    last_flush_time: Cell<f64>,
    last_step_gen_time: Cell<f64>,
    need_flush_time: Cell<f64>,
    need_step_gen_time: Cell<f64>,
    // "Drip" timing (for homing and probing moves)
    drip_start_times: RefCell<Vec<f64>>,
}

impl PrinterMotionQueuing {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let all_mcus: Vec<Rc<Mcu>> =
            printer.lookup_objects(Some("mcu")).into_iter().filter_map(|(_, m)| downcast_rc::<Mcu>(m)).collect();
        let mcu = all_mcus.first().cloned().ok_or_else(|| KError::Other("list index out of range".into()))?;
        let can_pause = !mcu.is_fileoutput();
        let mq = Rc::new(PrinterMotionQueuing {
            printer: printer.clone(),
            reactor: reactor.clone(),
            trapqs: RefCell::new(vec![]),
            steppersyncmgr: RefCell::new(StepperSyncMgr::alloc()),
            syncemitters: RefCell::new(vec![]),
            steppersyncs: RefCell::new(vec![]),
            clear_history_time: Cell::new(0.0),
            flush_callbacks: RefCell::new(vec![]),
            kin_flush_delay: Cell::new(SDS_CHECK_TIME),
            all_mcus,
            mcu,
            can_pause: Cell::new(can_pause),
            flush_timer: RefCell::new(None),
            do_kick_flush_timer: Cell::new(true),
            last_flush_time: Cell::new(0.0),
            last_step_gen_time: Cell::new(0.0),
            need_flush_time: Cell::new(0.0),
            need_step_gen_time: Cell::new(0.0),
            drip_start_times: RefCell::new(vec![]),
        });
        // Flush tracking
        let w = Rc::downgrade(&mq);
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(mq) if can_pause => mq.flush_handler(eventtime),
                Some(mq) => mq.flush_handler_debug(eventtime),
                None => NEVER,
            }),
            NEVER,
        );
        *mq.flush_timer.borrow_mut() = Some(timer);
        // Register handlers
        let m = mq.clone();
        printer.register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                m.can_pause.set(false);
                Ok(())
            }),
        );
        Ok(mq)
    }

    // C trapq tracking
    pub fn allocate_trapq(&self) -> TrapQRef {
        let trapq = TrapQ::alloc();
        self.trapqs.borrow_mut().push(trapq.clone());
        trapq
    }

    pub fn wipe_trapq(&self, trapq: &TrapQRef) {
        // Expire any remaining movement in the trapq (force to history list)
        trapq.borrow_mut().finalize_moves(NEVER, 0.0);
    }

    // C steppersync tracking
    fn lookup_steppersync(&self, mcu: &Rc<Mcu>) -> StepperSyncRef {
        for (ss_mcu, ss) in self.steppersyncs.borrow().iter() {
            if Rc::ptr_eq(ss_mcu, mcu) {
                return ss.clone();
            }
        }
        let ss = self.steppersyncmgr.borrow_mut().alloc_steppersync();
        self.steppersyncs.borrow_mut().push((mcu.clone(), ss.clone()));
        ss
    }

    pub fn allocate_syncemitter(&self, mcu: &Rc<Mcu>, name: &str, alloc_stepcompress: bool) -> SyncEmitterRef {
        // name.encode("utf-8")[:15]
        let mut end = name.len().min(15);
        while !name.is_char_boundary(end) {
            end -= 1;
        }
        let ss = self.lookup_steppersync(mcu);
        let se = ss.borrow_mut().alloc_syncemitter(&name[..end], alloc_stepcompress);
        self.syncemitters.borrow_mut().push(se.clone());
        se
    }

    pub fn setup_mcu_movequeue(&self, mcu: &Rc<Mcu>, serialqueue: Option<Arc<SerialQueue>>, move_count: i64) -> KResult<()> {
        // Setup steppersync object for the mcu's main movequeue
        let ss = self.lookup_steppersync(mcu);
        ss.borrow_mut().setup_movequeue(serialqueue, move_count.max(0) as usize);
        let mcu_freq = mcu.seconds_to_clock(1.0) as f64;
        ss.borrow_mut().set_time(0.0, mcu_freq);
        Ok(())
    }

    // Flush notification callbacks
    pub fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool) {
        let mut fcbs = self.flush_callbacks.borrow_mut();
        if can_add_trapq {
            fcbs.insert(0, callback);
        } else {
            fcbs.push(callback);
        }
    }

    pub fn unregister_flush_callback(&self, callback: &FlushCallback) {
        let mut fcbs = self.flush_callbacks.borrow_mut();
        if let Some(pos) = fcbs.iter().position(|cb| Rc::ptr_eq(cb, callback)) {
            fcbs.remove(pos);
        }
    }

    // Kinematic step generation scan window time tracking
    pub fn get_kin_flush_delay(&self) -> f64 {
        self.kin_flush_delay.get()
    }

    pub fn check_step_generation_scan_windows(&self) {
        let mut kin_flush_delay = SDS_CHECK_TIME;
        for se in self.syncemitters.borrow().iter() {
            let Some(sk) = se.get_stepper_kinematics() else { continue };
            let sk = sk.borrow();
            if itersolve::get_trapq(&sk).is_none() {
                continue;
            }
            let pre_active = itersolve::get_gen_steps_pre_active(&sk);
            let post_active = itersolve::get_gen_steps_post_active(&sk);
            kin_flush_delay = py_max3(kin_flush_delay, pre_active, post_active);
        }
        self.kin_flush_delay.set(kin_flush_delay);
    }

    // Flush tracking
    fn advance_flush_time(&self, want_flush_time: f64, want_step_gen_time: f64) -> KResult<()> {
        let flush_time =
            py_max3(want_flush_time, self.last_flush_time.get(), want_step_gen_time - STEPCOMPRESS_FLUSH_TIME);
        let step_gen_time = py_max3(want_step_gen_time, self.last_step_gen_time.get(), flush_time);
        // Invoke flush callbacks (if any)
        {
            let _guard = self.reactor.assert_no_pause();
            let fcbs = self.flush_callbacks.borrow().clone();
            for cb in fcbs {
                cb(flush_time, step_gen_time)?;
            }
        }
        // Determine maximum history to keep
        let trapq_free_time = step_gen_time - self.kin_flush_delay.get();
        let mut clear_history_time = self.clear_history_time.get();
        if !self.can_pause.get() {
            clear_history_time = py_max(0.0, trapq_free_time - MOVE_HISTORY_EXPIRE);
        }
        // Generate stepper movement and transmit
        let ret = self.steppersyncmgr.borrow().gen_steps(flush_time, step_gen_time, clear_history_time);
        if ret != 0 {
            return Err(KError::Mcu("Internal error in stepcompress".into()));
        }
        self.last_flush_time.set(flush_time);
        self.last_step_gen_time.set(step_gen_time);
        // Move processed trapq entries to history list, and expire old history
        for trapq in self.trapqs.borrow().iter() {
            trapq.borrow_mut().finalize_moves(trapq_free_time, clear_history_time);
        }
        Ok(())
    }

    fn await_flush_time(&self, want_flush_time: f64) -> KResult<()> {
        loop {
            if self.last_flush_time.get() >= want_flush_time || !self.can_pause.get() {
                return Ok(());
            }
            let systime = self.reactor.monotonic();
            let est_print_time = self.mcu.estimated_print_time(systime);
            let wait = want_flush_time - BGFLUSH_HIGH_TIME - est_print_time;
            if wait <= 0.0 {
                return Ok(());
            }
            self.reactor.pause(systime + py_min(1.0, wait))?;
        }
    }

    pub fn flush_all_steps(&self) -> KResult<()> {
        let flush_time = self.need_step_gen_time.get();
        self.await_flush_time(flush_time)?;
        self.advance_flush_time(flush_time, 0.0)
    }

    pub fn calc_step_gen_restart(&self, est_print_time: f64) -> f64 {
        let kin_time = py_max(est_print_time + MIN_KIN_TIME, self.last_step_gen_time.get());
        kin_time + self.kin_flush_delay.get()
    }

    fn flush_handler(&self, eventtime: f64) -> f64 {
        match self.flush_handler_inner(eventtime) {
            Ok(waketime) => waketime,
            Err(_) => {
                logging::exception("Exception in flush_handler");
                self.printer.invoke_shutdown("Exception in flush_handler", None);
                NEVER
            }
        }
    }

    fn flush_handler_inner(&self, eventtime: f64) -> KResult<f64> {
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        let aggr_sg_time = self.need_step_gen_time.get() - 2.0 * self.kin_flush_delay.get();
        let last_step_gen_time = self.last_step_gen_time.get();
        if last_step_gen_time < aggr_sg_time {
            // Actively stepping - want more aggressive flushing
            let mut want_sg_time = est_print_time + BGFLUSH_SG_HIGH_TIME;
            let batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME;
            let mut next_batch_time = last_step_gen_time + batch_time;
            if next_batch_time > est_print_time {
                // Improve run-to-run reproducibility by batching from last
                if next_batch_time > want_sg_time + 0.005 {
                    // Delay flushing until next wakeup
                    next_batch_time = last_step_gen_time;
                }
                want_sg_time = next_batch_time;
            }
            want_sg_time = py_min(want_sg_time, aggr_sg_time);
            // Flush motion queues (if needed)
            if want_sg_time > last_step_gen_time {
                self.advance_flush_time(0.0, want_sg_time)?;
            }
        } else {
            // Not stepping (or only step remnants) - use relaxed flushing
            let mut want_flush_time = est_print_time + BGFLUSH_HIGH_TIME;
            let max_flush_time = self.need_flush_time.get() + get_danger_options().bgflush_extra_time;
            want_flush_time = py_min(want_flush_time, max_flush_time);
            // Flush motion queues (if needed)
            if want_flush_time > self.last_flush_time.get() {
                self.advance_flush_time(want_flush_time, 0.0)?;
            }
        }
        // Reschedule timer
        let aggr_sg_time = self.need_step_gen_time.get() - 2.0 * self.kin_flush_delay.get();
        let waketime = if self.last_step_gen_time.get() < aggr_sg_time {
            self.last_step_gen_time.get() - BGFLUSH_SG_LOW_TIME
        } else {
            self.do_kick_flush_timer.set(true);
            let max_flush_time = self.need_flush_time.get() + get_danger_options().bgflush_extra_time;
            if self.last_flush_time.get() >= max_flush_time {
                return Ok(NEVER);
            }
            self.last_flush_time.get() - BGFLUSH_LOW_TIME
        };
        Ok(eventtime + waketime - est_print_time)
    }

    fn flush_handler_debug(&self, eventtime: f64) -> f64 {
        // Use custom flushing code when in batch output mode
        let res = (|| -> KResult<f64> {
            let faux_time = self.need_flush_time.get() - 1.5;
            let batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME;
            let mut flush_count = 0;
            while self.last_step_gen_time.get() < faux_time {
                let mut target = self.last_step_gen_time.get() + batch_time;
                if flush_count as f64 > 100.0 {
                    target = faux_time;
                }
                self.advance_flush_time(0.0, target)?;
                flush_count += 1;
            }
            if flush_count != 0 {
                return Ok(NOW);
            }
            self.advance_flush_time(self.need_flush_time.get() + get_danger_options().bgflush_extra_time, 0.0)?;
            self.do_kick_flush_timer.set(true);
            Ok(NEVER)
        })();
        let _ = eventtime;
        match res {
            Ok(w) => w,
            Err(_) => {
                logging::exception("Exception in flush_handler_debug");
                self.printer.invoke_shutdown("Exception in flush_handler_debug", None);
                NEVER
            }
        }
    }

    pub fn note_mcu_movequeue_activity(&self, mut mq_time: f64, is_step_gen: bool) {
        if is_step_gen {
            mq_time += self.kin_flush_delay.get();
            self.need_step_gen_time.set(py_max(self.need_step_gen_time.get(), mq_time));
        }
        self.need_flush_time.set(py_max(self.need_flush_time.get(), mq_time));
        if self.do_kick_flush_timer.get() {
            self.do_kick_flush_timer.set(false);
            if let Some(t) = self.flush_timer.borrow().as_ref() {
                self.reactor.update_timer(t, NOW);
            }
        }
    }

    // "Drip" timing (for homing and probing moves)
    pub fn drip_update_time(&self, start_time: f64, end_time: f64, drip_completion: &Rc<ReactorCompletion<bool>>) -> KResult<()> {
        self.drip_start_times.borrow_mut().push(start_time);
        self.await_flush_time(start_time)?;
        // Disable background flushing from timer
        if let Some(t) = self.flush_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NEVER);
        }
        self.do_kick_flush_timer.set(false);
        self.advance_flush_time(start_time, 0.0)?;
        // Flush in segments until drip_completion signal
        let mut flush_time = start_time;
        while flush_time < end_time {
            if drip_completion.test() {
                break;
            }
            let curtime = self.reactor.monotonic();
            let est_print_time = self.mcu.estimated_print_time(curtime);
            let wait_time = flush_time - est_print_time - DRIP_TIME;
            if wait_time > 0.0 && self.can_pause.get() {
                // Pause before sending more steps
                drip_completion.wait(curtime + wait_time, None)?;
                continue;
            }
            flush_time = py_min(flush_time + DRIP_SEGMENT_TIME, end_time);
            self.note_mcu_movequeue_activity(flush_time, true);
            self.advance_flush_time(flush_time, 0.0)?;
        }
        // Restore background flushing
        if let Some(t) = self.flush_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NOW);
        }
        self.advance_flush_time(flush_time + self.kin_flush_delay.get(), 0.0)?;
        let mut dst = self.drip_start_times.borrow_mut();
        // list.remove(): ValueError when missing
        let pos = dst.iter().position(|&t| t == start_time).ok_or_else(|| KError::Other("list.remove(x): x not in list".into()))?;
        dst.remove(pos);
        Ok(())
    }

    pub fn check_drip_timing(&self) -> Option<f64> {
        let dst = self.drip_start_times.borrow();
        dst.iter().copied().reduce(py_min)
    }
}

/// python max(a, b) / min(a, b) of floats (the first wins a tie or a nan)
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

fn py_max3(a: f64, b: f64, c: f64) -> f64 {
    py_max(py_max(a, b), c)
}

impl PrinterObject for PrinterMotionQueuing {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        // Globally calibrate mcu clocks (and step generation clocks)
        let sync_time = self.last_step_gen_time.get();
        let steppersyncs = self.steppersyncs.borrow().clone();
        for (mcu, ss) in steppersyncs {
            match mcu.calibrate_clock(sync_time, eventtime) {
                Ok((offset, freq)) => ss.borrow_mut().set_time(offset, freq),
                Err(e) => {
                    self.printer.unhandled_exception(&e);
                    return Some((false, String::new()));
                }
            }
        }
        // Calculate history expiration
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        self.clear_history_time.set(py_max(0.0, est_print_time - MOVE_HISTORY_EXPIRE));
        Some((false, String::new()))
    }
    fn teardown(&self) {
        self.flush_callbacks.borrow_mut().clear();
        self.steppersyncs.borrow_mut().clear();
        if let Some(t) = self.flush_timer.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterMotionQueuing::new(config)?)
}
