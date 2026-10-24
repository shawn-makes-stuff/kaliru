// Interface to Klipper micro-controller code (port of klippy/mcu.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The MCU helper classes (MCUConnectHelper, MCURestartHelper,
// MCUStatsHelper, MCUConfigHelper) keep their state in the structs below,
// which are fields of Mcu; their methods are Mcu methods prefixed with the
// helper name (conn_, restart_, stats_, cfg_). Event handlers are
// registered in python's order.
use crate::chelper::serialqueue::CommandQueue;
use crate::chelper::trdispatch::{Trdispatch, TrdispatchMcu};
use crate::clocksync::ClockSync;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::gcode::GCodeDispatch;
use crate::logging;
use crate::msgproto::{MessageFormat, MessageParser, Params, Val};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{ConfigFileObject, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_int, py_str, py_str_repr};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, ReactorCompletion, Timer, NEVER, NOW};
use crate::serialhdl::{params_time, SerialDev, SerialReader, SerialRetryCommand};
use crate::stepper::McuStepper;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// mcu.error
fn error(msg: impl Into<String>) -> KError {
    KError::Mcu(msg.into())
}

fn pins_error(msg: impl Into<String>) -> KError {
    KError::Pins(msg.into())
}

// Minimum time host needs to get scheduled events queued into mcu
pub const MIN_SCHEDULE_TIME: f64 = 0.100;
// Like MIN_SCHEDULE_TIME, but used during MCU initialization
pub const MIN_SCHEDULE_TIME_INIT: f64 = 0.200;
// The maximum number of clock cycles an MCU is expected
// to schedule into the future, due to the protocol and firmware.
pub const MAX_SCHEDULE_TICKS: i64 = (1 << 31) - 1;
// Maximum time all MCUs can internally schedule into the future.
// Directly caused by the limitation of MAX_SCHEDULE_TICKS.
pub const MAX_NOMINAL_DURATION: f64 = 3.0;

/// params[k] of an integer parameter
pub fn pint(params: &Params, k: &str) -> i64 {
    params.get(k).and_then(|v| v.as_int()).unwrap_or(0)
}

/// a command's argument list of integers
pub fn ints(data: &[i64]) -> Vec<Val> {
    data.iter().map(|&v| Val::Int(v)).collect()
}

// ----------------------------------------------------------------------
// Command transmit helper classes
// ----------------------------------------------------------------------

// Class to retry sending of a query command until a given response is received
pub struct RetryAsyncCommand {
    serial: Rc<SerialReader>,
    name: String,
    oid: Option<i64>,
    reactor: Rc<Reactor>,
    completion: Rc<ReactorCompletion<Params>>,
    min_query_time: Rc<Cell<f64>>,
}

impl RetryAsyncCommand {
    const TIMEOUT_TIME: f64 = 5.0;
    const RETRY_TIME: f64 = 0.500;

    pub fn new(serial: &Rc<SerialReader>, name: &str, oid: Option<i64>) -> Self {
        let reactor = serial.get_reactor();
        let completion = reactor.completion::<Params>();
        let min_query_time = Rc::new(Cell::new(reactor.monotonic()));
        let need_response = Cell::new(true);
        let (r, c, mqt) = (reactor.clone(), completion.clone(), min_query_time.clone());
        serial.register_response(
            Some(Rc::new(move |params: &Params| {
                // handle_callback
                if need_response.get() && params_time(params, "#sent_time") >= mqt.get() {
                    need_response.set(false);
                    r.async_complete(&c, params.clone());
                }
                Ok(())
            })),
            name,
            oid,
        );
        RetryAsyncCommand { serial: serial.clone(), name: name.to_string(), oid, reactor, completion, min_query_time }
    }

    pub fn get_response(&self, cmds: &[Vec<u8>], cmd_queue: &CommandQueue, minclock: u64, reqclock: u64, retry: bool) -> KResult<Params> {
        if cmds.len() != 1 {
            return Err(KError::Other(format!("too many values to unpack (expected 1, got {})", cmds.len())));
        }
        let cmd = &cmds[0];
        self.serial.raw_send_wait_ack(cmd, minclock, reqclock, cmd_queue)?;
        self.min_query_time.set(0.0);
        let mut query_time = self.reactor.monotonic();
        let mut timeout_time = query_time;
        if retry {
            timeout_time += Self::TIMEOUT_TIME;
        }
        loop {
            let params = self.completion.wait(query_time + Self::RETRY_TIME, None)?;
            if let Some(params) = params {
                self.serial.register_response(None, &self.name, self.oid);
                return Ok(params);
            }
            query_time = self.reactor.monotonic();
            if query_time > timeout_time {
                self.serial.register_response(None, &self.name, self.oid);
                return Err(KError::Serial(format!("Timeout on wait for '{}' response", self.name)));
            }
            self.serial.raw_send(cmd, minclock, minclock, cmd_queue)?;
        }
    }
}

// Wrapper around query commands
pub struct CommandQueryWrapper {
    serial: Rc<SerialReader>,
    cmd: Rc<MessageFormat>,
    response: String,
    oid: Option<i64>,
    /// the error class serialhdl.error is re-raised as (None: unchanged)
    error: Option<fn(String) -> KError>,
    is_async: bool,
    cmd_queue: CommandQueue,
}

impl CommandQueryWrapper {
    pub fn new(
        serial: &Rc<SerialReader>,
        msgformat: &str,
        respformat: &str,
        oid: Option<i64>,
        cmd_queue: Option<CommandQueue>,
        is_async: bool,
        error: Option<fn(String) -> KError>,
    ) -> KResult<Self> {
        let msgparser = serial.get_msgparser();
        let cmd = msgparser.lookup_command(msgformat)?;
        msgparser.lookup_command(respformat)?;
        let response = respformat.split_whitespace().next().unwrap_or("").to_string();
        let cmd_queue = cmd_queue.unwrap_or_else(|| serial.get_default_command_queue());
        Ok(CommandQueryWrapper { serial: serial.clone(), cmd, response, oid, error, is_async, cmd_queue })
    }

    fn do_send(&self, cmds: &[Vec<u8>], minclock: u64, reqclock: u64, retry: bool) -> KResult<Params> {
        let reqclock = minclock.max(reqclock);
        let res = if self.is_async {
            let xh = RetryAsyncCommand::new(&self.serial, &self.response, self.oid);
            xh.get_response(cmds, &self.cmd_queue, minclock, reqclock, retry)
        } else {
            let xh = SerialRetryCommand::new(&self.serial, &self.response, self.oid);
            xh.get_response(cmds, &self.cmd_queue, minclock, reqclock, retry)
        };
        match (res, self.error) {
            (Err(KError::Serial(e)), Some(error)) => Err(error(e)),
            (res, _) => res,
        }
    }

    pub fn send(&self, data: &[Val], minclock: u64, reqclock: u64, retry: bool) -> KResult<Params> {
        self.do_send(&[self.cmd.encode(data)?], minclock, reqclock, retry)
    }

    pub fn send_with_preface(
        &self,
        preface_cmd: &CommandWrapper,
        preface_data: &[Val],
        data: &[Val],
        minclock: u64,
        reqclock: u64,
        retry: bool,
    ) -> KResult<Params> {
        let cmds = [preface_cmd.cmd.encode(preface_data)?, self.cmd.encode(data)?];
        self.do_send(&cmds, minclock, reqclock, retry)
    }
}

// Wrapper around command sending
pub struct CommandWrapper {
    serial: Rc<SerialReader>,
    cmd: Rc<MessageFormat>,
    cmd_queue: CommandQueue,
    msgtag: u32,
    // Can't use send_wait_ack when in debugging mode
    debugoutput: bool,
}

impl CommandWrapper {
    pub fn new(serial: &Rc<SerialReader>, msgformat: &str, cmd_queue: Option<CommandQueue>, debugoutput: bool) -> KResult<Self> {
        let msgparser = serial.get_msgparser();
        let cmd = msgparser.lookup_command(msgformat)?;
        let cmd_queue = cmd_queue.unwrap_or_else(|| serial.get_default_command_queue());
        let msgtag = (msgparser.lookup_msgid(msgformat)? & 0xFFFFFFFF) as u32;
        Ok(CommandWrapper { serial: serial.clone(), cmd, cmd_queue, msgtag, debugoutput })
    }

    pub fn send(&self, data: &[Val], minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = self.cmd.encode(data)?;
        self.serial.raw_send(&cmd, minclock, reqclock, &self.cmd_queue)
    }

    pub fn send_wait_ack(&self, data: &[Val], minclock: u64, reqclock: u64) -> KResult<()> {
        if self.debugoutput {
            return self.send(data, minclock, reqclock);
        }
        let cmd = self.cmd.encode(data)?;
        self.serial.raw_send_wait_ack(&cmd, minclock, reqclock, &self.cmd_queue)?;
        Ok(())
    }

    pub fn get_command_tag(&self) -> u32 {
        self.msgtag
    }
}

// ----------------------------------------------------------------------
// Wrapper classes for MCU pins
// ----------------------------------------------------------------------

pub type StepperRef = Rc<McuStepper>;

fn same_stepper(a: &StepperRef, b: &StepperRef) -> bool {
    Rc::ptr_eq(a, b)
}

pub struct McuTrsync {
    mcu: Rc<Mcu>,
    trdispatch: Rc<Trdispatch>,
    reactor: Rc<Reactor>,
    steppers: RefCell<Vec<StepperRef>>,
    trdispatch_mcu: RefCell<Option<TrdispatchMcu>>,
    oid: i64,
    cmd_queue: CommandQueue,
    trsync_start_cmd: RefCell<Option<CommandWrapper>>,
    trsync_set_timeout_cmd: RefCell<Option<CommandWrapper>>,
    trsync_trigger_cmd: RefCell<Option<CommandWrapper>>,
    trsync_query_cmd: RefCell<Option<CommandQueryWrapper>>,
    stepper_stop_cmd: RefCell<Option<CommandWrapper>>,
    trigger_completion: RefCell<Option<Rc<ReactorCompletion<bool>>>>,
    home_end_clock: Cell<Option<i64>>,
}

impl McuTrsync {
    pub const REASON_ENDSTOP_HIT: i64 = 1;
    pub const REASON_HOST_REQUEST: i64 = 2;
    pub const REASON_PAST_END_TIME: i64 = 3;
    pub const REASON_COMMS_TIMEOUT: i64 = 4;

    pub fn new(mcu: &Rc<Mcu>, trdispatch: &Rc<Trdispatch>) -> Rc<Self> {
        let t = Rc::new(McuTrsync {
            mcu: mcu.clone(),
            trdispatch: trdispatch.clone(),
            reactor: mcu.get_printer().get_reactor(),
            steppers: RefCell::new(vec![]),
            trdispatch_mcu: RefCell::new(None),
            oid: mcu.create_oid(),
            cmd_queue: mcu.alloc_command_queue(),
            trsync_start_cmd: RefCell::new(None),
            trsync_set_timeout_cmd: RefCell::new(None),
            trsync_trigger_cmd: RefCell::new(None),
            trsync_query_cmd: RefCell::new(None),
            stepper_stop_cmd: RefCell::new(None),
            trigger_completion: RefCell::new(None),
            home_end_clock: Cell::new(None),
        });
        let tc = t.clone();
        mcu.register_config_callback(Rc::new(move || tc.build_config()));
        let tc = t.clone();
        mcu.get_printer().register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                tc.shutdown();
                Ok(())
            }),
        );
        t
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    pub fn get_command_queue(&self) -> CommandQueue {
        self.cmd_queue.clone()
    }

    pub fn add_stepper(&self, stepper: &StepperRef) {
        if self.steppers.borrow().iter().any(|s| same_stepper(s, stepper)) {
            return;
        }
        self.steppers.borrow_mut().push(stepper.clone());
    }

    pub fn get_steppers(&self) -> Vec<StepperRef> {
        self.steppers.borrow().clone()
    }

    fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;
        let oid = self.oid;
        let cq = Some(self.cmd_queue.clone());
        // Setup config
        mcu.add_config_cmd(&format!("config_trsync oid={}", oid), false, false);
        mcu.add_config_cmd(
            &format!("trsync_start oid={} report_clock=0 report_ticks=0 expire_reason=0", oid),
            false,
            true,
        );
        // Lookup commands
        *self.trsync_start_cmd.borrow_mut() = Some(mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c",
            cq.clone(),
        )?);
        *self.trsync_set_timeout_cmd.borrow_mut() =
            Some(mcu.lookup_command("trsync_set_timeout oid=%c clock=%u", cq.clone())?);
        *self.trsync_trigger_cmd.borrow_mut() = Some(mcu.lookup_command("trsync_trigger oid=%c reason=%c", cq.clone())?);
        *self.trsync_query_cmd.borrow_mut() = Some(mcu.lookup_query_command(
            "trsync_trigger oid=%c reason=%c",
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            Some(oid),
            cq.clone(),
            false,
        )?);
        *self.stepper_stop_cmd.borrow_mut() =
            Some(mcu.lookup_command("stepper_stop_on_trigger oid=%c trsync_oid=%c", cq)?);
        // Create trdispatch_mcu object
        let set_timeout_tag = mcu.lookup_command("trsync_set_timeout oid=%c clock=%u", None)?.get_command_tag();
        let trigger_tag = mcu.lookup_command("trsync_trigger oid=%c reason=%c", None)?.get_command_tag();
        let state_tag = mcu
            .lookup_command("trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u", None)?
            .get_command_tag();
        // build_config() runs again when a non-critical mcu reconnects;
        // free the previous object first so it is unlinked
        if let Some(tdm) = self.trdispatch_mcu.borrow_mut().take() {
            tdm.free();
        }
        let sq = mcu.serial.get_serialqueue().ok_or_else(|| KError::Other("serialqueue is not connected".into()))?;
        *self.trdispatch_mcu.borrow_mut() =
            Some(self.trdispatch.mcu_alloc(sq, self.cmd_queue.clone(), oid as u32, set_timeout_tag, trigger_tag, state_tag));
        Ok(())
    }

    fn shutdown(&self) {
        let tc = self.trigger_completion.borrow_mut().take();
        if let Some(tc) = tc {
            tc.complete(false);
        }
    }

    fn handle_trsync_state(&self, params: &Params) -> KResult<()> {
        if pint(params, "can_trigger") == 0 {
            let tc = self.trigger_completion.borrow_mut().take();
            if let Some(tc) = tc {
                let reason = pint(params, "trigger_reason");
                let is_failure = reason >= Self::REASON_COMMS_TIMEOUT;
                self.reactor.async_complete(&tc, is_failure);
            }
        } else if let Some(home_end_clock) = self.home_end_clock.get() {
            let clock = self.mcu.clock32_to_clock64(pint(params, "clock"));
            if clock >= home_end_clock {
                self.home_end_clock.set(None);
                if let Some(cmd) = self.trsync_trigger_cmd.borrow().as_ref() {
                    cmd.send(&ints(&[self.oid, Self::REASON_PAST_END_TIME]), 0, 0)?;
                }
            }
        }
        Ok(())
    }

    pub fn start(
        self: &Rc<Self>,
        print_time: f64,
        report_offset: f64,
        trigger_completion: &Rc<ReactorCompletion<bool>>,
        expire_timeout: f64,
    ) -> KResult<()> {
        *self.trigger_completion.borrow_mut() = Some(trigger_completion.clone());
        self.home_end_clock.set(None);
        let clock = self.mcu.print_time_to_clock(print_time);
        let expire_ticks = self.mcu.seconds_to_clock(expire_timeout);
        let expire_clock = clock + expire_ticks;
        let report_ticks = self.mcu.seconds_to_clock(expire_timeout * 0.3);
        let report_clock = clock + (report_ticks as f64 * report_offset + 0.5) as i64;
        let min_extend_ticks = (report_ticks as f64 * 0.8 + 0.5) as i64;
        if let Some(tdm) = self.trdispatch_mcu.borrow().as_ref() {
            tdm.setup(clock as u64, expire_clock as u64, expire_ticks as u64, min_extend_ticks as u64);
        }
        let t = self.clone();
        self.mcu.register_response(Some(Rc::new(move |p: &Params| t.handle_trsync_state(p))), "trsync_state", Some(self.oid));
        if let Some(cmd) = self.trsync_start_cmd.borrow().as_ref() {
            cmd.send(&ints(&[self.oid, report_clock, report_ticks, Self::REASON_COMMS_TIMEOUT]), 0, clock as u64)?;
        }
        for s in self.steppers.borrow().iter() {
            if let Some(cmd) = self.stepper_stop_cmd.borrow().as_ref() {
                cmd.send(&ints(&[s.get_oid(), self.oid]), 0, 0)?;
            }
        }
        if let Some(cmd) = self.trsync_set_timeout_cmd.borrow().as_ref() {
            cmd.send(&ints(&[self.oid, expire_clock]), 0, clock as u64)?;
        }
        Ok(())
    }

    pub fn set_home_end_time(&self, home_end_time: f64) {
        self.home_end_clock.set(Some(self.mcu.print_time_to_clock(home_end_time)));
    }

    pub fn stop(&self) -> KResult<i64> {
        self.mcu.register_response(None, "trsync_state", Some(self.oid));
        *self.trigger_completion.borrow_mut() = None;
        if self.mcu.is_fileoutput() {
            return Ok(Self::REASON_ENDSTOP_HIT);
        }
        let params = match self.trsync_query_cmd.borrow().as_ref() {
            Some(cmd) => cmd.send(&ints(&[self.oid, Self::REASON_HOST_REQUEST]), 0, 0, true)?,
            None => return Err(KError::Other("'NoneType' object has no attribute 'send'".into())),
        };
        for s in self.steppers.borrow().iter() {
            s.note_homing_end()?;
        }
        Ok(pint(&params, "trigger_reason"))
    }
}

pub struct TriggerDispatch {
    mcu: Rc<Mcu>,
    trigger_completion: RefCell<Option<Rc<ReactorCompletion<bool>>>>,
    trdispatch: Rc<Trdispatch>,
    trsyncs: RefCell<Vec<Rc<McuTrsync>>>,
}

impl TriggerDispatch {
    pub fn new(mcu: &Rc<Mcu>) -> Self {
        let trdispatch = Rc::new(Trdispatch::alloc());
        let trsyncs = vec![McuTrsync::new(mcu, &trdispatch)];
        TriggerDispatch { mcu: mcu.clone(), trigger_completion: RefCell::new(None), trdispatch, trsyncs: RefCell::new(trsyncs) }
    }

    pub fn get_oid(&self) -> i64 {
        self.trsyncs.borrow()[0].get_oid()
    }

    pub fn get_command_queue(&self) -> CommandQueue {
        self.trsyncs.borrow()[0].get_command_queue()
    }

    pub fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        let smcu = stepper.get_mcu();
        let found = self.trsyncs.borrow().iter().rev().find(|t| Rc::ptr_eq(&t.mcu, &smcu)).cloned();
        let trsync = match found {
            Some(t) => t,
            None => {
                let t = McuTrsync::new(&smcu, &self.trdispatch);
                self.trsyncs.borrow_mut().push(t.clone());
                t
            }
        };
        trsync.add_stepper(stepper);
        // Check for unsupported multi-mcu shared stepper rails
        let sname = stepper.get_name(false);
        if sname.starts_with("stepper_") {
            let prefix: String = sname.chars().take(9).collect();
            for ot in self.trsyncs.borrow().iter() {
                for s in ot.get_steppers() {
                    if !Rc::ptr_eq(ot, &trsync) && s.get_name(false).starts_with(&prefix) {
                        return Err(KError::config("Multi-mcu homing not supported on multi-mcu shared axis"));
                    }
                }
            }
        }
        Ok(())
    }

    pub fn get_steppers(&self) -> Vec<StepperRef> {
        self.trsyncs.borrow().iter().flat_map(|t| t.get_steppers()).collect()
    }

    pub fn start(&self, print_time: f64) -> KResult<Rc<ReactorCompletion<bool>>> {
        let reactor = self.mcu.get_printer().get_reactor();
        let tc = reactor.completion::<bool>();
        *self.trigger_completion.borrow_mut() = Some(tc.clone());
        let trsyncs = self.trsyncs.borrow().clone();
        let mut expire_timeout = get_danger_options().multi_mcu_trsync_timeout;
        if trsyncs.len() == 1 {
            expire_timeout = get_danger_options().single_mcu_trsync_timeout;
        }
        for (i, trsync) in trsyncs.iter().enumerate() {
            let report_offset = i as f64 / trsyncs.len() as f64;
            trsync.start(print_time, report_offset, &tc, expire_timeout)?;
        }
        self.trdispatch.start(McuTrsync::REASON_HOST_REQUEST as u32);
        Ok(tc)
    }

    pub fn wait_end(&self, end_time: f64) -> KResult<()> {
        let etrsync = self.trsyncs.borrow()[0].clone();
        etrsync.set_home_end_time(end_time);
        let tc = self.trigger_completion.borrow().clone();
        if let Some(tc) = tc {
            if self.mcu.is_fileoutput() {
                tc.complete(true);
            }
            tc.wait(NEVER, None)?;
        }
        Ok(())
    }

    pub fn stop(&self) -> KResult<i64> {
        self.trdispatch.stop();
        let trsyncs = self.trsyncs.borrow().clone();
        let mut res = vec![];
        for trsync in &trsyncs {
            res.push(trsync.stop()?);
        }
        if let Some(r) = res.iter().find(|&&r| r >= McuTrsync::REASON_COMMS_TIMEOUT) {
            return Ok(*r);
        }
        Ok(res[0])
    }
}

/// an endstop as homing.py and stepper.py use it (MCU_endstop, probe and
/// virtual endstops); python duck types these
pub trait Endstop {
    fn get_mcu(&self) -> Rc<Mcu>;
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()>;
    fn get_steppers(&self) -> Vec<StepperRef>;
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: bool,
    ) -> KResult<Rc<ReactorCompletion<bool>>>;
    fn home_wait(&self, home_end_time: f64) -> KResult<f64>;
    fn query_endstop(&self, print_time: f64) -> KResult<i64>;
    /// get_position_endstop(), None where python's hasattr() is false
    fn get_position_endstop(&self) -> Option<f64> {
        None
    }
    /// isinstance(endstop, mcu.MCU_endstop)
    fn is_mcu_endstop(&self) -> bool {
        false
    }
}

pub type EndstopRef = Rc<dyn Endstop>;

/// the object setup_pin("endstop", ...) returns, as an endstop: an
/// McuEndstop, or an EndstopRef boxed by other chips (virtual endstops)
pub fn any_to_endstop(obj: Rc<dyn Any>) -> Option<EndstopRef> {
    match obj.downcast::<McuEndstop>() {
        Ok(e) => Some(e),
        Err(obj) => obj.downcast::<EndstopRef>().ok().map(|e| (*e).clone()),
    }
}

impl Endstop for McuEndstop {
    fn is_mcu_endstop(&self) -> bool {
        true
    }
    fn get_mcu(&self) -> Rc<Mcu> {
        McuEndstop::get_mcu(self)
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        McuEndstop::add_stepper(self, stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        McuEndstop::get_steppers(self)
    }
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: bool,
    ) -> KResult<Rc<ReactorCompletion<bool>>> {
        McuEndstop::home_start(self, print_time, sample_time, sample_count, rest_time, triggered)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        McuEndstop::home_wait(self, home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        McuEndstop::query_endstop(self, print_time)
    }
}

pub struct McuEndstop {
    mcu: Rc<Mcu>,
    pin: String,
    pullup: i64,
    invert: i64,
    oid: i64,
    home_cmd: RefCell<Option<CommandWrapper>>,
    query_cmd: RefCell<Option<CommandQueryWrapper>>,
    rest_ticks: Cell<i64>,
    dispatch: TriggerDispatch,
}

impl McuEndstop {
    pub fn new(mcu: &Rc<Mcu>, pin_params: &SharedPinParams) -> Rc<Self> {
        let pp = pin_params.borrow();
        let oid = mcu.create_oid();
        let e = Rc::new_cyclic(|_| McuEndstop {
            mcu: mcu.clone(),
            pin: pp.pin.clone(),
            pullup: pp.pullup,
            invert: pp.invert,
            oid,
            home_cmd: RefCell::new(None),
            query_cmd: RefCell::new(None),
            rest_ticks: Cell::new(0),
            dispatch: TriggerDispatch::new(mcu),
        });
        // python registers build_config before creating TriggerDispatch;
        // reorder the callbacks to match
        let ec = e.clone();
        mcu.insert_config_callback_before_last(Rc::new(move || ec.build_config()));
        e
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.dispatch.add_stepper(stepper)
    }

    pub fn get_steppers(&self) -> Vec<StepperRef> {
        self.dispatch.get_steppers()
    }

    fn build_config(&self) -> KResult<()> {
        // Setup config
        self.mcu.add_config_cmd(&format!("config_endstop oid={} pin={} pull_up={}", self.oid, self.pin, self.pullup), false, false);
        self.mcu.add_config_cmd(
            &format!(
                "endstop_home oid={} clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0",
                self.oid
            ),
            false,
            true,
        );
        // Lookup commands
        let cmd_queue = Some(self.dispatch.get_command_queue());
        *self.home_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c",
            cmd_queue.clone(),
        )?);
        *self.query_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "endstop_query_state oid=%c",
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            Some(self.oid),
            cmd_queue,
            false,
        )?);
        Ok(())
    }

    fn check_connected(&self) -> KResult<()> {
        if self.mcu.non_critical_disconnected.get() {
            return Err(KError::command(format!("Cannot use endstop on disconnected MCU '{}'", self.mcu.get_name())));
        }
        Ok(())
    }

    pub fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: bool,
    ) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.check_connected()?;
        let clock = self.mcu.print_time_to_clock(print_time);
        let rest_ticks = self.mcu.print_time_to_clock(print_time + rest_time) - clock;
        self.rest_ticks.set(rest_ticks);
        let trigger_completion = self.dispatch.start(print_time)?;
        if let Some(cmd) = self.home_cmd.borrow().as_ref() {
            cmd.send(
                &ints(&[
                    self.oid,
                    clock,
                    self.mcu.seconds_to_clock(sample_time),
                    sample_count,
                    rest_ticks,
                    triggered as i64 ^ self.invert,
                    self.dispatch.get_oid(),
                    McuTrsync::REASON_ENDSTOP_HIT,
                ]),
                0,
                clock as u64,
            )?;
        }
        Ok(trigger_completion)
    }

    pub fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.dispatch.wait_end(home_end_time)?;
        if let Some(cmd) = self.home_cmd.borrow().as_ref() {
            cmd.send(&ints(&[self.oid, 0, 0, 0, 0, 0, 0, 0]), 0, 0)?;
        }
        let res = self.dispatch.stop()?;
        if res >= McuTrsync::REASON_COMMS_TIMEOUT {
            return Err(KError::command("Communication timeout during homing"));
        }
        if res != McuTrsync::REASON_ENDSTOP_HIT {
            return Ok(0.0);
        }
        if self.mcu.is_fileoutput() {
            return Ok(home_end_time);
        }
        let params = self.query(&[self.oid], 0)?;
        let next_clock = self.mcu.clock32_to_clock64(pint(&params, "next_clock"));
        Ok(self.mcu.clock_to_print_time((next_clock - self.rest_ticks.get()) as f64))
    }

    fn query(&self, data: &[i64], minclock: u64) -> KResult<Params> {
        match self.query_cmd.borrow().as_ref() {
            Some(cmd) => cmd.send(&ints(data), minclock, 0, true),
            None => Err(KError::Other("'NoneType' object has no attribute 'send'".into())),
        }
    }

    pub fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        self.check_connected()?;
        let clock = self.mcu.print_time_to_clock(print_time);
        if self.mcu.is_fileoutput() {
            return Ok(0);
        }
        let params = self.query(&[self.oid], clock as u64)?;
        Ok(pint(&params, "pin_value") ^ self.invert)
    }
}

pub struct McuDigitalOut {
    mcu: Rc<Mcu>,
    oid: Cell<Option<i64>>,
    pin: String,
    invert: i64,
    start_value: Cell<i64>,
    shutdown_value: Cell<i64>,
    max_duration: Cell<f64>,
    last_clock: Cell<i64>,
    set_cmd: RefCell<Option<CommandWrapper>>,
}

impl McuDigitalOut {
    pub fn new(mcu: &Rc<Mcu>, pin_params: &SharedPinParams) -> Rc<Self> {
        let pp = pin_params.borrow();
        let d = Rc::new(McuDigitalOut {
            mcu: mcu.clone(),
            oid: Cell::new(None),
            pin: pp.pin.clone(),
            invert: pp.invert,
            start_value: Cell::new(pp.invert),
            shutdown_value: Cell::new(pp.invert),
            max_duration: Cell::new(2.0),
            last_clock: Cell::new(0),
            set_cmd: RefCell::new(None),
        });
        let dc = d.clone();
        mcu.register_config_callback(Rc::new(move || dc.build_config()));
        d
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    pub fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        self.start_value.set(start_value as i64 ^ self.invert);
        self.shutdown_value.set(shutdown_value as i64 ^ self.invert);
    }

    fn build_config(&self) -> KResult<()> {
        if self.max_duration.get() != 0. && self.start_value.get() != self.shutdown_value.get() {
            return Err(pins_error("Pin with max duration must have start value equal to shutdown value"));
        }
        let mdur_ticks = self.mcu.seconds_to_clock(self.max_duration.get());
        if mdur_ticks > MAX_SCHEDULE_TICKS {
            return Err(pins_error("Digital pin max duration too large"));
        }
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid();
        self.oid.set(Some(oid));
        self.mcu.add_config_cmd(
            &format!(
                "config_digital_out oid={} pin={} value={} default_value={} max_duration={}",
                oid,
                self.pin,
                self.start_value.get(),
                self.shutdown_value.get(),
                mdur_ticks
            ),
            false,
            false,
        );
        self.mcu.add_config_cmd(&format!("update_digital_out oid={} value={}", oid, self.start_value.get()), false, true);
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.set_cmd.borrow_mut() = Some(self.mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?);
        Ok(())
    }

    pub fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        if self.mcu.non_critical_disconnected.get() {
            return Err(KError::command(format!("Cannot set pin on disconnected MCU '{}'", self.mcu.get_name())));
        }
        let clock = self.mcu.print_time_to_clock(print_time);
        if let Some(cmd) = self.set_cmd.borrow().as_ref() {
            cmd.send(
                &ints(&[self.oid.get().unwrap_or(0), clock, value as i64 ^ self.invert]),
                self.last_clock.get() as u64,
                clock as u64,
            )?;
        }
        self.last_clock.set(clock);
        Ok(())
    }
}

/// a "digital_out" pin as its users see it: an McuDigitalOut, or another
/// chip's pin (sx1509); python duck types these
pub trait DigitalOutPin {
    fn get_mcu(&self) -> Rc<Mcu>;
    fn setup_max_duration(&self, max_duration: f64);
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool);
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()>;
}

impl DigitalOutPin for McuDigitalOut {
    fn get_mcu(&self) -> Rc<Mcu> {
        McuDigitalOut::get_mcu(self)
    }
    fn setup_max_duration(&self, max_duration: f64) {
        McuDigitalOut::setup_max_duration(self, max_duration)
    }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        McuDigitalOut::setup_start_value(self, start_value, shutdown_value)
    }
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        McuDigitalOut::set_digital(self, print_time, value)
    }
}

/// a "pwm" pin as its users see it: an McuPwm, or another chip's pin
/// (sx1509); python duck types these
pub trait PwmPin {
    fn get_mcu(&self) -> Rc<Mcu>;
    fn setup_max_duration(&self, max_duration: f64);
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()>;
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64);
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()>;
}

impl PwmPin for McuPwm {
    fn get_mcu(&self) -> Rc<Mcu> {
        McuPwm::get_mcu(self)
    }
    fn setup_max_duration(&self, max_duration: f64) {
        McuPwm::setup_max_duration(self, max_duration)
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()> {
        McuPwm::setup_cycle_time(self, cycle_time, hardware_pwm);
        Ok(())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        McuPwm::setup_start_value(self, start_value, shutdown_value)
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        McuPwm::set_pwm(self, print_time, value)
    }
}

pub struct McuPwm {
    mcu: Rc<Mcu>,
    hardware_pwm: Cell<bool>,
    cycle_time: Cell<f64>,
    max_duration: Cell<f64>,
    oid: Cell<Option<i64>>,
    pin: String,
    invert: i64,
    start_value: Cell<f64>,
    shutdown_value: Cell<f64>,
    last_clock: Cell<i64>,
    pwm_max: Cell<f64>,
    set_cmd: RefCell<Option<CommandWrapper>>,
}

impl McuPwm {
    pub fn new(mcu: &Rc<Mcu>, pin_params: &SharedPinParams) -> Rc<Self> {
        let pp = pin_params.borrow();
        let p = Rc::new(McuPwm {
            mcu: mcu.clone(),
            hardware_pwm: Cell::new(false),
            cycle_time: Cell::new(0.100),
            max_duration: Cell::new(2.0),
            oid: Cell::new(None),
            pin: pp.pin.clone(),
            invert: pp.invert,
            start_value: Cell::new(pp.invert as f64),
            shutdown_value: Cell::new(pp.invert as f64),
            last_clock: Cell::new(0),
            pwm_max: Cell::new(0.0),
            set_cmd: RefCell::new(None),
        });
        let pc = p.clone();
        mcu.register_config_callback(Rc::new(move || pc.build_config()));
        p
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    pub fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
        self.cycle_time.set(cycle_time);
        self.hardware_pwm.set(hardware_pwm);
    }

    pub fn setup_start_value(&self, mut start_value: f64, mut shutdown_value: f64) {
        if self.invert != 0 {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        self.start_value.set(py_max(0.0, py_min(1.0, start_value)));
        self.shutdown_value.set(py_max(0.0, py_min(1.0, shutdown_value)));
    }

    fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;
        if self.max_duration.get() != 0. && self.start_value.get() != self.shutdown_value.get() {
            return Err(pins_error("Pin with max duration must have start value equal to shutdown value"));
        }
        let cmd_queue = mcu.alloc_command_queue();
        let curtime = mcu.get_printer().get_reactor().monotonic();
        let printtime = mcu.estimated_print_time(curtime);
        self.last_clock.set(mcu.print_time_to_clock(printtime + MIN_SCHEDULE_TIME_INIT));
        let cycle_ticks = mcu.seconds_to_clock(self.cycle_time.get());
        let mdur_ticks = mcu.seconds_to_clock(self.max_duration.get());
        if mdur_ticks > MAX_SCHEDULE_TICKS {
            return Err(pins_error("PWM pin max duration too large"));
        }
        let (start_value, shutdown_value) = (self.start_value.get(), self.shutdown_value.get());
        if self.hardware_pwm.get() {
            self.pwm_max.set(mcu.get_constant_float("PWM_MAX")?);
            let pwm_max = self.pwm_max.get();
            mcu.request_move_queue_slot();
            let oid = mcu.create_oid();
            self.oid.set(Some(oid));
            // "%d" of a float truncates
            mcu.add_config_cmd(
                &format!(
                    "config_pwm_out oid={} pin={} cycle_ticks={} value={} default_value={} max_duration={}",
                    oid,
                    self.pin,
                    cycle_ticks,
                    (start_value * pwm_max) as i64,
                    (shutdown_value * pwm_max) as i64,
                    mdur_ticks
                ),
                false,
                false,
            );
            let svalue = (start_value * pwm_max + 0.5) as i64;
            mcu.add_config_cmd(&format!("queue_pwm_out oid={} clock={} value={}", oid, self.last_clock.get(), svalue), false, true);
            *self.set_cmd.borrow_mut() = Some(mcu.lookup_command("queue_pwm_out oid=%c clock=%u value=%hu", Some(cmd_queue))?);
            return Ok(());
        }
        // Software PWM
        if shutdown_value != 0.0 && shutdown_value != 1.0 {
            return Err(pins_error("shutdown value must be 0.0 or 1.0 on soft pwm"));
        }
        if cycle_ticks > MAX_SCHEDULE_TICKS {
            return Err(pins_error("PWM pin cycle time too large"));
        }
        mcu.request_move_queue_slot();
        let oid = mcu.create_oid();
        self.oid.set(Some(oid));
        mcu.add_config_cmd(
            &format!(
                "config_digital_out oid={} pin={} value={} default_value={} max_duration={}",
                oid,
                self.pin,
                (start_value >= 1.0) as i64,
                (shutdown_value >= 0.5) as i64,
                mdur_ticks
            ),
            false,
            false,
        );
        mcu.add_config_cmd(&format!("set_digital_out_pwm_cycle oid={} cycle_ticks={}", oid, cycle_ticks), false, false);
        self.pwm_max.set(cycle_ticks as f64);
        let svalue = (start_value * cycle_ticks as f64 + 0.5) as i64;
        mcu.add_config_cmd(&format!("queue_digital_out oid={} clock={} on_ticks={}", oid, self.last_clock.get(), svalue), true, false);
        *self.set_cmd.borrow_mut() = Some(mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?);
        Ok(())
    }

    pub fn set_pwm(&self, print_time: f64, mut value: f64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        if clock < self.last_clock.get() && !self.mcu.is_fileoutput() {
            return Err(error("Tried to set PWM target before last update"));
        }
        if self.invert != 0 {
            value = 1.0 - value;
        }
        let v = (py_max(0.0, py_min(1.0, value)) * self.pwm_max.get() + 0.5) as i64;
        if let Some(cmd) = self.set_cmd.borrow().as_ref() {
            cmd.send(&ints(&[self.oid.get().unwrap_or(0), clock, v]), self.last_clock.get() as u64, clock as u64)?;
        }
        self.last_clock.set(clock);
        Ok(())
    }
}

/// python max(a, b) / min(a, b) of floats (nan handling follows python)
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

pub type AdcCallback = Rc<dyn Fn(f64, f64) -> KResult<()>>;

pub struct McuAdc {
    mcu: Rc<Mcu>,
    pin: String,
    min_sample: Cell<f64>,
    max_sample: Cell<f64>,
    sample_time: Cell<f64>,
    report_time: Cell<f64>,
    sample_count: Cell<i64>,
    range_check_count: Cell<i64>,
    report_clock: Cell<i64>,
    last_state: Cell<(f64, f64)>,
    oid: Cell<Option<i64>>,
    callback: RefCell<Option<AdcCallback>>,
    inv_max_adc: Cell<f64>,
}

/// an "adc" pin as its users see it: an McuAdc, or another chip's pin
/// (adc_scaled); python duck types these
pub trait AdcPin {
    fn get_mcu(&self) -> Rc<Mcu>;
    fn setup_adc_sample(&self, sample_time: f64, sample_count: i64, minval: f64, maxval: f64, range_check_count: i64);
    fn setup_adc_callback(&self, report_time: f64, callback: AdcCallback);
    fn get_last_value(&self) -> (f64, f64);
}

impl AdcPin for McuAdc {
    fn get_mcu(&self) -> Rc<Mcu> {
        McuAdc::get_mcu(self)
    }
    fn setup_adc_sample(&self, sample_time: f64, sample_count: i64, minval: f64, maxval: f64, range_check_count: i64) {
        McuAdc::setup_adc_sample(self, sample_time, sample_count, minval, maxval, range_check_count)
    }
    fn setup_adc_callback(&self, report_time: f64, callback: AdcCallback) {
        McuAdc::setup_adc_callback(self, report_time, callback)
    }
    fn get_last_value(&self) -> (f64, f64) {
        McuAdc::get_last_value(self)
    }
}

impl McuAdc {
    pub fn new(mcu: &Rc<Mcu>, pin_params: &SharedPinParams) -> Rc<Self> {
        let a = Rc::new(McuAdc {
            mcu: mcu.clone(),
            pin: pin_params.borrow().pin.clone(),
            min_sample: Cell::new(0.0),
            max_sample: Cell::new(0.0),
            sample_time: Cell::new(0.0),
            report_time: Cell::new(0.0),
            sample_count: Cell::new(0),
            range_check_count: Cell::new(0),
            report_clock: Cell::new(0),
            last_state: Cell::new((0.0, 0.0)),
            oid: Cell::new(None),
            callback: RefCell::new(None),
            inv_max_adc: Cell::new(0.0),
        });
        let ac = a.clone();
        mcu.register_config_callback(Rc::new(move || ac.build_config()));
        a
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn setup_adc_sample(&self, sample_time: f64, sample_count: i64, minval: f64, maxval: f64, range_check_count: i64) {
        self.sample_time.set(sample_time);
        self.sample_count.set(sample_count);
        self.min_sample.set(minval);
        self.max_sample.set(maxval);
        self.range_check_count.set(range_check_count);
    }

    pub fn setup_adc_callback(&self, report_time: f64, callback: AdcCallback) {
        self.report_time.set(report_time);
        *self.callback.borrow_mut() = Some(callback);
    }

    pub fn get_last_value(&self) -> (f64, f64) {
        self.last_state.get()
    }

    pub fn build_config(self: &Rc<Self>) -> KResult<()> {
        if self.sample_count.get() == 0 {
            return Ok(());
        }
        let mcu = &self.mcu;
        let oid = mcu.create_oid();
        self.oid.set(Some(oid));
        mcu.add_config_cmd(&format!("config_analog_in oid={} pin={}", oid, self.pin), false, false);
        let clock = mcu.get_query_slot(oid);
        let sample_ticks = mcu.seconds_to_clock(self.sample_time.get());
        let mcu_adc_max = mcu.get_constant_float("ADC_MAX")?;
        let max_adc = self.sample_count.get() as f64 * mcu_adc_max;
        self.inv_max_adc.set(1.0 / max_adc);
        self.report_clock.set(mcu.seconds_to_clock(self.report_time.get()));
        let min_sample = 0.max(0xFFFF.min((self.min_sample.get() * max_adc) as i64));
        let max_sample = 0.max(0xFFFF.min((self.max_sample.get() * max_adc).ceil() as i64));
        mcu.add_config_cmd(
            &format!(
                "query_analog_in oid={} clock={} sample_ticks={} sample_count={} rest_ticks={} min_value={} max_value={} range_check_count={}",
                oid,
                clock,
                sample_ticks,
                self.sample_count.get(),
                self.report_clock.get(),
                min_sample,
                max_sample,
                self.range_check_count.get()
            ),
            true,
            false,
        );
        let ac = self.clone();
        mcu.register_response(Some(Rc::new(move |p: &Params| ac.handle_analog_in_state(p))), "analog_in_state", Some(oid));
        Ok(())
    }

    fn handle_analog_in_state(&self, params: &Params) -> KResult<()> {
        let last_value = pint(params, "value") as f64 * self.inv_max_adc.get();
        let next_clock = self.mcu.clock32_to_clock64(pint(params, "next_clock"));
        let last_read_clock = next_clock - self.report_clock.get();
        let last_read_time = self.mcu.clock_to_print_time(last_read_clock as f64);
        self.last_state.set((last_value, last_read_time));
        let cb = self.callback.borrow().clone();
        if let Some(cb) = cb {
            cb(last_read_time, last_value)?;
        }
        Ok(())
    }
}

// ----------------------------------------------------------------------
// Main MCU class (and its helper classes)
// ----------------------------------------------------------------------

pub type ConfigCallback = Rc<dyn Fn() -> KResult<()>>;

// Support for restarting a micro-controller (MCURestartHelper)
struct RestartHelper {
    restart_method: RefCell<Option<String>>,
    reset_cmd: RefCell<Option<CommandWrapper>>,
    config_reset_cmd: RefCell<Option<CommandWrapper>>,
    is_mcu_bridge: Cell<bool>,
}

// Low-level mcu connection management helper (MCUConnectHelper)
struct ConnHelper {
    baud: Cell<i64>,
    serialport: RefCell<String>,
    canbus_iface: RefCell<Option<String>>,
    emergency_stop_cmd: RefCell<Option<CommandWrapper>>,
    is_shutdown: Cell<bool>,
    is_timeout: Cell<bool>,
    shutdown_msg: RefCell<String>,
}

// Handle statistics reporting (MCUStatsHelper)
struct StatsHelper {
    mcu_freq: Cell<f64>,
    get_status_info: RefCell<IndexMap<String, PyVal>>,
    stats_sumsq_base: Cell<f64>,
    mcu_tick_avg: Cell<f64>,
    mcu_tick_stddev: Cell<f64>,
    mcu_tick_awake: Cell<f64>,
}

// Handle process of configuring an mcu (MCUConfigHelper)
#[derive(Default)]
struct ConfigHelper {
    oid_count: Cell<i64>,
    config_callbacks: RefCell<Vec<ConfigCallback>>,
    config_cmds: RefCell<Vec<String>>,
    restart_cmds: RefCell<Vec<String>>,
    init_cmds: RefCell<Vec<String>>,
    mcu_freq: Cell<f64>,
    reserved_move_slots: Cell<i64>,
    config_crc: Cell<u32>,
    // State restored when a non-critical mcu reconnects
    cached_init_state: Cell<bool>,
    oid_count_post_inits: Cell<i64>,
    config_cmds_post_inits: RefCell<Vec<String>>,
    init_cmds_post_inits: RefCell<Vec<String>>,
    restart_cmds_post_inits: RefCell<Vec<String>>,
}

/// get_config's response (a dict in file output mode)
struct ConfigParams {
    is_config: i64,
    crc: i64,
    move_count: i64,
}

// Main MCU class
pub struct Mcu {
    me: Weak<Mcu>,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    clocksync: Rc<ClockSync>,
    reactor: Rc<Reactor>,
    name: String,
    serial: Rc<SerialReader>,
    conn: ConnHelper,
    restart: RestartHelper,
    cfg: ConfigHelper,
    stats_h: StatsHelper,
    // noncritical mcus
    pub is_non_critical: Cell<bool>,
    non_critical_recon_timer: RefCell<Option<Timer>>,
    pub non_critical_disconnected: Rc<Cell<bool>>,
    reconnecting: Cell<bool>,
    non_critical_reconnect_event_name: String,
    non_critical_disconnect_event_name: String,
    reconnect_interval: Cell<f64>,
}

/// msgparser.get_constant(name, None) (parser str)
fn constant_str_opt(mp: &MessageParser, name: &str) -> Option<String> {
    mp.config.get(name).map(py_str)
}

/// register a no-argument event handler calling `f` on the mcu
fn on_event(printer: &Rc<Printer>, event: &str, mcu: &Rc<Mcu>, f: fn(&Rc<Mcu>) -> KResult<()>) {
    let m = mcu.clone();
    printer.register_event_handler(event, Rc::new(move |_| f(&m)));
}

impl Mcu {
    pub fn new(config: &ConfigWrapper, clocksync: Rc<ClockSync>) -> KResult<Rc<Mcu>> {
        let printer = config.get_printer();
        // self.danger_options is looked up but not used
        printer.lookup_object("danger_options")?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let reactor = printer.get_reactor();
        let mut name = config.get_name().to_string();
        if let Some(n) = name.strip_prefix("mcu ") {
            name = n.to_string();
        }
        let non_critical_disconnected = Rc::new(Cell::new(false));
        // MCUConnectHelper: Serial port
        let wp = format!("mcu '{}': ", name);
        let serial = SerialReader::new(&reactor, &wp, &name, non_critical_disconnected.clone());
        let mcu = Rc::new_cyclic(|me| Mcu {
            me: me.clone(),
            printer: printer.clone(),
            gcode,
            clocksync,
            reactor: reactor.clone(),
            non_critical_reconnect_event_name: format!("danger:non_critical_mcu_{}:reconnected", name),
            non_critical_disconnect_event_name: format!("danger:non_critical_mcu_{}:disconnected", name),
            name,
            serial,
            conn: ConnHelper {
                baud: Cell::new(0),
                serialport: RefCell::new(String::new()),
                canbus_iface: RefCell::new(None),
                emergency_stop_cmd: RefCell::new(None),
                is_shutdown: Cell::new(false),
                is_timeout: Cell::new(false),
                shutdown_msg: RefCell::new(String::new()),
            },
            restart: RestartHelper {
                restart_method: RefCell::new(Some("command".into())),
                reset_cmd: RefCell::new(None),
                config_reset_cmd: RefCell::new(None),
                is_mcu_bridge: Cell::new(false),
            },
            cfg: ConfigHelper::default(),
            stats_h: StatsHelper {
                mcu_freq: Cell::new(0.0),
                get_status_info: RefCell::new(IndexMap::new()),
                stats_sumsq_base: Cell::new(0.0),
                mcu_tick_avg: Cell::new(0.0),
                mcu_tick_stddev: Cell::new(0.0),
                mcu_tick_awake: Cell::new(0.0),
            },
            is_non_critical: Cell::new(false),
            non_critical_recon_timer: RefCell::new(None),
            non_critical_disconnected,
            reconnecting: Cell::new(false),
            reconnect_interval: Cell::new(0.0),
        });
        // Check for disconnected non-critical mcus before the helpers
        on_event(&printer, "klippy:mcu_identify", &mcu, |m| m.mcu_identify().map(|_| ()));
        // Low-level connection and helpers
        mcu.conn_init(config)?;
        mcu.cfg_init()?;
        mcu.stats_init();
        printer.load_object(config, "error_mcu", true)?;

        // noncritical mcus
        mcu.is_non_critical.set(config.getboolean("is_non_critical", false)?);
        if mcu.is_non_critical.get() && mcu.get_name() == "mcu" {
            return Err(error("Primary MCU cannot be marked as non-critical!"));
        }
        if mcu.is_non_critical.get() {
            let w = Rc::downgrade(&mcu);
            let timer = reactor.register_timer(
                Box::new(move |eventtime| match w.upgrade() {
                    Some(m) => m.non_critical_recon_event(eventtime),
                    None => NEVER,
                }),
                NEVER,
            );
            *mcu.non_critical_recon_timer.borrow_mut() = Some(timer);
        }
        // add small change to not collide with other events
        mcu.reconnect_interval.set(config.getfloat("reconnect_interval", 2.0, Lim::new())? + 0.12);
        Ok(mcu)
    }

    // ------------------------------------------------ MCURestartHelper

    fn restart_init(self: &Rc<Self>, config: &ConfigWrapper) -> KResult<()> {
        if self.conn.baud.get() != 0 {
            let m = config.getchoice_opt("restart_method", &["arduino", "cheetah", "command", "rpi_usb"])?;
            *self.restart.restart_method.borrow_mut() = m;
        }
        // Register handlers
        on_event(&self.printer, "klippy:mcu_identify", self, |m| m.restart_mcu_identify());
        on_event(&self.printer, "klippy:firmware_restart", self, |m| m.restart_firmware_restart(false));
        on_event(&self.printer, "klippy:disconnect", self, |m| {
            m.restart_disconnect();
            Ok(())
        });
        Ok(())
    }

    // Connection phase
    fn restart_check_restart(&self, reason: &str) -> KResult<()> {
        let start_reason = self.printer.start_arg("start_reason");
        if start_reason.as_ref().and_then(|v| v.as_str()) == Some("firmware_restart") {
            return Ok(());
        }
        logging::info(&format!("Attempting automated MCU '{}' restart: {}", self.name, reason));
        self.printer.request_exit("firmware_restart");
        self.reactor.pause(self.reactor.monotonic() + 2.000)?;
        Err(error(format!("Attempt MCU '{}' restart failed", self.name)))
    }

    fn restart_check_restart_on_crc_mismatch(&self) -> KResult<()> {
        self.restart_check_restart("CRC mismatch")
    }

    fn restart_check_restart_on_send_config(&self) -> KResult<()> {
        if self.restart.restart_method.borrow().as_deref() == Some("rpi_usb") {
            // Only configure mcu after usb power reset
            self.restart_check_restart("full reset before config")?;
        }
        Ok(())
    }

    fn restart_check_restart_on_attach(&self) -> KResult<()> {
        let serialport = self.conn.serialport.borrow().clone();
        if self.restart.restart_method.borrow().as_deref() == Some("rpi_usb") && !std::path::Path::new(&serialport).exists() {
            // Try toggling usb power
            self.restart_check_restart("enable power")?;
        }
        Ok(())
    }

    fn restart_lookup_attach_uart_rts(&self) -> bool {
        // Cheetah boards require RTS to be deasserted
        // else a reset will trigger the built-in bootloader.
        self.restart.restart_method.borrow().as_deref() != Some("cheetah")
    }

    fn restart_mcu_identify(self: &Rc<Self>) -> KResult<()> {
        if self.non_critical_disconnected.get() {
            return Ok(());
        }
        *self.restart.reset_cmd.borrow_mut() = self.try_lookup_command("reset");
        *self.restart.config_reset_cmd.borrow_mut() = self.try_lookup_command("config_reset");
        let ext_only = self.restart.reset_cmd.borrow().is_none() && self.restart.config_reset_cmd.borrow().is_none();
        let msgparser = self.serial.get_msgparser();
        let mbaud = constant_str_opt(&msgparser, "SERIAL_BAUD");
        if self.restart.restart_method.borrow().is_none() && mbaud.is_none() && !ext_only {
            *self.restart.restart_method.borrow_mut() = Some("command".into());
        }
        // get_constant("CANBUS_BRIDGE", 0): any present value is a non-empty str
        if constant_str_opt(&msgparser, "CANBUS_BRIDGE").map(|s| !s.is_empty()).unwrap_or(false) {
            self.restart.is_mcu_bridge.set(true);
            on_event(&self.printer, "klippy:firmware_restart", self, |m| m.restart_firmware_restart(true));
        }
        Ok(())
    }

    fn restart_recon_handle_shutdown(&self) -> KResult<bool> {
        // A firmware shutdown survives the loss of the host connection.
        // Ask the mcu to reset so a later reconnect attempt starts from
        // a clean state.  Returns True if the mcu was shutdown.
        let get_config_cmd =
            self.lookup_query_command("get_config", "config is_config=%c crc=%u is_shutdown=%c move_count=%hu", None, None, false)?;
        if pint(&get_config_cmd.send(&[], 0, 0, true)?, "is_shutdown") == 0 {
            return Ok(false);
        }
        logging::info(&format!("MCU '{}' is in a shutdown state - attempting reset", self.name));
        if let Some(cmd) = self.restart.reset_cmd.borrow().as_ref() {
            cmd.send(&[], 0, 0)?;
        } else if let Some(cmd) = self.restart.config_reset_cmd.borrow().as_ref() {
            cmd.send(&[], 0, 0)?;
        } else {
            // No reset mechanism - clear the shutdown state and let the
            // next reconnect attempt reuse the retained config
            if let Some(cmd) = self.try_lookup_command("clear_shutdown") {
                cmd.send(&[], 0, 0)?;
            }
        }
        self.reactor.pause(self.reactor.monotonic() + 0.015)?;
        Ok(true)
    }

    fn restart_disconnect(&self) {
        self.serial.disconnect();
    }

    fn restart_arduino(&self) -> KResult<()> {
        logging::info(&format!("Attempting MCU '{}' reset", self.name));
        self.restart_disconnect();
        crate::serialhdl::arduino_reset(&self.conn.serialport.borrow(), &self.reactor)
    }

    fn restart_cheetah(&self) -> KResult<()> {
        logging::info(&format!("Attempting MCU '{}' Cheetah-style reset", self.name));
        self.restart_disconnect();
        crate::serialhdl::cheetah_reset(&self.conn.serialport.borrow(), &self.reactor)
    }

    fn restart_via_command(&self) -> KResult<()> {
        let no_cmds = self.restart.reset_cmd.borrow().is_none() && self.restart.config_reset_cmd.borrow().is_none();
        if no_cmds || !self.clocksync.is_active() {
            logging::info(&format!("Unable to issue reset command on MCU '{}'", self.name));
            return Ok(());
        }
        if self.restart.reset_cmd.borrow().is_none() {
            // Attempt reset via config_reset command
            logging::info(&format!("Attempting MCU '{}' config_reset command", self.name));
            self.conn_force_local_shutdown()?;
            self.reactor.pause(self.reactor.monotonic() + 0.015)?;
            if let Some(cmd) = self.restart.config_reset_cmd.borrow().as_ref() {
                cmd.send(&[], 0, 0)?;
            }
        } else {
            // Attempt reset via reset command
            logging::info(&format!("Attempting MCU '{}' reset command", self.name));
            if let Some(cmd) = self.restart.reset_cmd.borrow().as_ref() {
                cmd.send(&[], 0, 0)?;
            }
        }
        self.reactor.pause(self.reactor.monotonic() + 0.015)?;
        self.restart_disconnect();
        Ok(())
    }

    fn restart_rpi_usb(&self) -> KResult<()> {
        logging::info(&format!("Attempting MCU '{}' reset via rpi usb power", self.name));
        self.restart_disconnect();
        // ponytail: chelper.run_hub_ctrl (hub-ctrl.c) is not ported yet
        Err(KError::Other("chelper.run_hub_ctrl is not ported to Kaliru yet".into()))
    }

    fn restart_firmware_restart(&self, force: bool) -> KResult<()> {
        if (self.restart.is_mcu_bridge.get() && !force) || self.non_critical_disconnected.get() {
            return Ok(());
        }
        let method = self.restart.restart_method.borrow().clone();
        match method.as_deref() {
            Some("rpi_usb") => self.restart_rpi_usb(),
            Some("command") => self.restart_via_command(),
            Some("cheetah") => self.restart_cheetah(),
            _ => self.restart_arduino(),
        }
    }

    // ------------------------------------------------ MCUConnectHelper

    fn conn_init(self: &Rc<Self>, config: &ConfigWrapper) -> KResult<()> {
        let canbus_uuid = config.get_opt("canbus_uuid")?;
        if let Some(canbus_uuid) = canbus_uuid {
            *self.conn.serialport.borrow_mut() = canbus_uuid.clone();
            let canbus_iface = config.get("canbus_interface", "can0")?;
            *self.conn.canbus_iface.borrow_mut() = Some(canbus_iface.clone());
            let cbid = self.printer.load_as::<crate::extras::canbus_ids::PrinterCANBus>(config, "canbus_ids")?;
            cbid.add_uuid(config, &canbus_uuid, &canbus_iface)?;
            self.printer.load_object(config, &format!("canbus_stats {}", self.name), true)?;
        } else {
            let serialport = config.get("serial", crate::configfile::REQUIRED)?;
            if !(serialport.starts_with("/dev/rpmsg_") || serialport.starts_with("/tmp/klipper_host_")) {
                self.conn.baud.set(config.getint("baud", 250000, Lim::new().min(2400))?);
            }
            *self.conn.serialport.borrow_mut() = serialport;
        }
        // Register handlers
        on_event(&self.printer, "klippy:mcu_identify", self, |m| m.conn_mcu_identify());
        self.restart_init(config)?;
        on_event(&self.printer, "klippy:shutdown", self, |m| m.conn_shutdown(false));
        let m = self.clone();
        self.printer.register_event_handler(
            "klippy:analyze_shutdown",
            Rc::new(move |_| {
                m.conn_analyze_shutdown();
                Ok(())
            }),
        );
        Ok(())
    }

    fn conn_handle_shutdown(&self, params: &Params) -> KResult<()> {
        if self.conn.is_shutdown.get() {
            return Ok(());
        }
        self.conn.is_shutdown.set(true);
        let msg = params.get("static_string_id").map(|v| v.py_str()).unwrap_or_default();
        *self.conn.shutdown_msg.borrow_mut() = msg.clone();
        let shutdown_clock = match params.get("clock").and_then(|v| v.as_int()) {
            Some(c) => PyVal::Int(self.clock32_to_clock64(c)),
            None => PyVal::None,
        };
        let event_type = params.get("#name").map(|v| v.py_str()).unwrap_or_default();
        let mut details = IndexMap::new();
        details.insert("reason".to_string(), PyVal::Str(msg));
        details.insert("mcu".to_string(), PyVal::Str(self.name.clone()));
        details.insert("event_type".to_string(), PyVal::Str(event_type));
        details.insert("shutdown_clock".to_string(), shutdown_clock);
        self.printer.invoke_async_shutdown("MCU shutdown", Some(PyVal::Dict(details)));
        Ok(())
    }

    fn conn_handle_starting(&self, _params: &Params) -> KResult<()> {
        if !self.conn.is_shutdown.get() && !self.is_non_critical.get() {
            self.printer.invoke_async_shutdown(&format!("MCU '{}' spontaneous restart", self.name), None);
        }
        Ok(())
    }

    fn conn_log_info(&self) -> String {
        let msgparser = self.serial.get_msgparser();
        let app = py_str(msgparser.get_app_info());
        let message_count = msgparser.get_messages().len();
        let (version, build_versions) = msgparser.get_version_info();
        let constants: Vec<String> =
            msgparser.get_constants().iter().map(|(k, v)| format!("{}={}", k, py_str(v))).collect();
        [
            format!(
                "Loaded MCU '{}' {} commands ({} {} / {})",
                self.name,
                message_count,
                app,
                py_str(version),
                py_str(build_versions)
            ),
            format!("MCU '{}' config: {}", self.name, constants.join(" ")),
        ]
        .join("\n")
    }

    fn conn_attach_file(&self) -> KResult<()> {
        // In a debugging mode.  Open debug output file and read data dictionary
        let arg = |k: &str| self.printer.start_arg(k).map(|v| v.str());
        let (out_fname, dict_fname) = if self.name == "mcu" {
            (arg("debugoutput"), arg("dictionary"))
        } else {
            (arg("debugoutput").map(|d| format!("{}-{}", d, self.name)), arg(&format!("dictionary_{}", self.name)))
        };
        let path_type_error = || KError::Other("expected str, bytes or os.PathLike object, not NoneType".into());
        let out_fname = out_fname.ok_or_else(path_type_error)?;
        let outfile = std::fs::File::create(&out_fname)
            .map_err(|e| KError::os(&e, &out_fname))?;
        let dict_fname = dict_fname.ok_or_else(path_type_error)?;
        let dict_data =
            std::fs::read(&dict_fname).map_err(|e| KError::os(&e, &dict_fname))?;
        use std::os::fd::IntoRawFd;
        self.serial.connect_file(SerialDev { fd: outfile.into_raw_fd() }, &dict_data)?;
        self.clocksync.connect_file(&self.serial, false)
    }

    /// printer.lookup_object("canbus_ids").get_nodeid(canbus_uuid)
    fn canbus_nodeid(&self, canbus_uuid: &str) -> KResult<i64> {
        let cbid = self.printer.lookup::<crate::extras::canbus_ids::PrinterCANBus>("canbus_ids")?;
        cbid.get_nodeid(canbus_uuid)
    }

    pub fn conn_check_serial_exists(&self) -> KResult<bool> {
        let serialport = self.conn.serialport.borrow().clone();
        if let Some(iface) = self.conn.canbus_iface.borrow().clone() {
            let nodeid = self.canbus_nodeid(&serialport)?;
            return self.serial.check_canbus_connect(&serialport, nodeid, &iface);
        }
        let rts = self.restart_lookup_attach_uart_rts();
        Ok(self.serial.check_connect(&serialport, self.conn.baud.get() as u32, rts))
    }

    fn conn_attach(&self) -> KResult<()> {
        self.restart_check_restart_on_attach()?;
        let serialport = self.conn.serialport.borrow().clone();
        let res = (|| {
            if let Some(iface) = self.conn.canbus_iface.borrow().clone() {
                let nodeid = self.canbus_nodeid(&serialport)?;
                self.serial.connect_canbus(&serialport, nodeid, &iface)?;
            } else if self.conn.baud.get() != 0 {
                let rts = self.restart_lookup_attach_uart_rts();
                self.serial.connect_uart(&serialport, self.conn.baud.get() as u32, rts)?;
            } else {
                self.serial.connect_pipe(&serialport)?;
            }
            self.clocksync.connect(&self.serial)
        })();
        match res {
            Err(KError::Serial(e)) => Err(error(e)),
            r => r,
        }
    }

    fn conn_mcu_identify(self: &Rc<Self>) -> KResult<()> {
        if self.non_critical_disconnected.get() {
            return Ok(());
        }
        if self.is_fileoutput() {
            self.conn_attach_file()?;
        } else {
            self.conn_attach()?;
        }
        if get_danger_options().log_startup_info {
            logging::info(&self.conn_log_info());
        }
        // Setup shutdown handling
        *self.conn.emergency_stop_cmd.borrow_mut() = Some(self.lookup_command("emergency_stop", None)?);
        let m = self.clone();
        self.register_response(Some(Rc::new(move |p: &Params| m.conn_handle_shutdown(p))), "shutdown", None);
        let m = self.clone();
        self.register_response(Some(Rc::new(move |p: &Params| m.conn_handle_shutdown(p))), "is_shutdown", None);
        let m = self.clone();
        self.register_response(Some(Rc::new(move |p: &Params| m.conn_handle_starting(p))), "starting", None);
        Ok(())
    }

    fn conn_analyze_shutdown(&self) {
        if self.is_fileoutput() || self.non_critical_disconnected.get() || !get_danger_options().log_shutdown_info {
            return;
        }
        logging::info(&format!(
            "MCU '{}' shutdown: {}\n{}\n{}\n{}",
            self.name,
            self.conn.shutdown_msg.borrow(),
            self.dump_debug(),
            self.clocksync.dump_debug(),
            self.serial.dump_debug()
        ));
    }

    fn conn_shutdown(&self, force: bool) -> KResult<()> {
        let cmd = self.conn.emergency_stop_cmd.borrow();
        let Some(cmd) = cmd.as_ref() else { return Ok(()) };
        if self.non_critical_disconnected.get() || (self.conn.is_shutdown.get() && !force) {
            return Ok(());
        }
        cmd.send(&[], 0, 0)
    }

    fn conn_check_timeout(self: &Rc<Self>, eventtime: f64) -> KResult<()> {
        if self.clocksync.is_active() || self.is_fileoutput() || self.conn.is_timeout.get() {
            return Ok(());
        }
        if self.is_non_critical.get() {
            return self.handle_non_critical_disconnect();
        }
        self.conn.is_timeout.set(true);
        logging::info(&format!("Timeout with MCU '{}' (eventtime={:.6})", self.name, eventtime));
        if get_danger_options().log_shutdown_info {
            logging::info(&format!(
                "MCU '{}' disconnected: Timeout\n{}\n{}\n{}",
                self.name,
                self.dump_debug(),
                self.clocksync.dump_debug(),
                self.serial.dump_debug()
            ));
        }
        self.printer.invoke_shutdown(&format!("Lost communication with MCU '{}'", self.name), None);
        Ok(())
    }

    fn conn_force_local_shutdown(&self) -> KResult<()> {
        self.conn.is_shutdown.set(true);
        self.conn_shutdown(true)
    }

    fn conn_disconnect(&self) {
        self.clocksync.disconnect();
        self.serial.disconnect();
    }

    // ------------------------------------------------ MCUStatsHelper

    fn stats_init(self: &Rc<Self>) {
        on_event(&self.printer, "klippy:ready", self, |m| m.stats_ready());
        on_event(&self.printer, "klippy:mcu_identify", self, |m| m.stats_mcu_identify());
    }

    fn stats_handle_mcu_stats(&self, params: &Params) -> KResult<()> {
        let s = &self.stats_h;
        let count = pint(params, "count") as f64;
        let tick_sum = pint(params, "sum") as f64;
        let c = 1.0 / (count * s.mcu_freq.get());
        s.mcu_tick_avg.set(tick_sum * c);
        let tick_sumsq = pint(params, "sumsq") as f64 * s.stats_sumsq_base.get();
        let diff = count * tick_sumsq - tick_sum.powi(2);
        s.mcu_tick_stddev.set(c * py_max(0.0, diff).sqrt());
        s.mcu_tick_awake.set(tick_sum / s.mcu_freq.get());
        Ok(())
    }

    fn stats_mcu_identify(self: &Rc<Self>) -> KResult<()> {
        if self.non_critical_disconnected.get() {
            return Ok(());
        }
        self.stats_h.mcu_freq.set(self.get_constant_float("CLOCK_FREQ")?);
        self.stats_h.stats_sumsq_base.set(self.get_constant_float("STATS_SUMSQ_BASE")?);
        let msgparser = self.serial.get_msgparser();
        let app = msgparser.get_app_info();
        let (version, build_versions) = msgparser.get_version_info();
        {
            let mut info = self.stats_h.get_status_info.borrow_mut();
            info.insert("app".into(), PyVal::from(app));
            info.insert("mcu_version".into(), PyVal::from(version));
            info.insert("mcu_build_versions".into(), PyVal::from(build_versions));
            let constants = msgparser.get_constants().iter().map(|(k, v)| (k.clone(), PyVal::from(v))).collect();
            info.insert("mcu_constants".into(), PyVal::Dict(constants));
            info.insert("mcu_kconfig".into(), PyVal::from(msgparser.get_kconfig()));
        }
        if app.as_str() == Some("Klipper") || app.as_str() == Some("Danger-Klipper") {
            let pconfig = self.printer.lookup::<ConfigFileObject>("configfile")?;
            pconfig.pconfig.borrow_mut().runtime_warning(&format!(
                "MCU {} currently has firmware compiled for {} (version {}). It is recommended to re-flash for best compatiblity with Kalico",
                py_str_repr(&self.name),
                py_str(app),
                py_str(version)
            ));
        }
        let m = self.clone();
        self.register_response(Some(Rc::new(move |p: &Params| m.stats_handle_mcu_stats(p))), "stats", None);
        Ok(())
    }

    fn stats_ready(&self) -> KResult<()> {
        if self.is_fileoutput() {
            return Ok(());
        }
        // Check that reported mcu frequency is in range
        let mcu_freq = self.stats_h.mcu_freq.get();
        let systime = self.reactor.monotonic();
        let calc_freq = (self.clocksync.get_clock(systime + 1.) - self.clocksync.get_clock(systime)) as f64;
        let freq_diff = (mcu_freq - calc_freq).abs();
        let mcu_freq_mhz = (mcu_freq / 1000000.0 + 0.5) as i64;
        let calc_freq_mhz = (calc_freq / 1000000.0 + 0.5) as i64;
        if freq_diff > mcu_freq * 0.01 && mcu_freq_mhz != calc_freq_mhz {
            let pconfig = self.printer.lookup::<ConfigFileObject>("configfile")?;
            let msg = format!("MCU '{}' configured for {}Mhz but running at {}Mhz!", self.name, mcu_freq_mhz, calc_freq_mhz);
            pconfig.pconfig.borrow_mut().runtime_warning(&msg);
        }
        Ok(())
    }

    fn stats_set_status_info(&self, key: &str, value: PyVal) {
        self.stats_h.get_status_info.borrow_mut().insert(key.to_string(), value);
    }

    fn stats_stats(&self, eventtime: f64) -> (bool, String) {
        let s = &self.stats_h;
        let load = format!(
            "mcu_awake={:.3} mcu_task_avg={:.6} mcu_task_stddev={:.6}",
            s.mcu_tick_awake.get(),
            s.mcu_tick_avg.get(),
            s.mcu_tick_stddev.get()
        );
        let stats = [load, self.serial.stats(eventtime), self.clocksync.stats(eventtime)].join(" ");
        let mut last_stats = IndexMap::new();
        for part in stats.split_whitespace() {
            let (k, v) = part.split_once('=').unwrap_or((part, ""));
            let v = if v.contains('.') {
                PyVal::Float(py_float(v).unwrap_or(0.))
            } else {
                PyVal::Int(py_int(v).unwrap_or(0))
            };
            last_stats.insert(k.to_string(), v);
        }
        s.get_status_info.borrow_mut().insert("last_stats".into(), PyVal::Dict(last_stats));
        (false, format!("{}: {}", self.name, stats))
    }

    // ------------------------------------------------ MCUConfigHelper

    fn cfg_init(self: &Rc<Self>) -> KResult<()> {
        let ppins = self.printer.lookup::<PinsObject>("pins")?;
        let chip: Rc<dyn PinChip> = self.clone();
        ppins.0.borrow_mut().register_chip(&self.name, chip)?;
        on_event(&self.printer, "klippy:mcu_identify", self, |m| m.cfg_mcu_identify());
        on_event(&self.printer, "klippy:connect", self, |m| m.cfg_connect());
        Ok(())
    }

    fn cfg_send_config(&self, prev_crc: Option<i64>) -> KResult<()> {
        let c = &self.cfg;
        if !c.cached_init_state.get() {
            // first time config, we haven't created callback oids yet
            // so save the oid count for state reset later
            c.oid_count_post_inits.set(c.oid_count.get());
            *c.config_cmds_post_inits.borrow_mut() = c.config_cmds.borrow().clone();
            *c.init_cmds_post_inits.borrow_mut() = c.init_cmds.borrow().clone();
            *c.restart_cmds_post_inits.borrow_mut() = c.restart_cmds.borrow().clone();
            c.cached_init_state.set(true);
        }
        // Build config commands (callbacks may register more callbacks)
        let mut i = 0;
        loop {
            let cb = c.config_callbacks.borrow().get(i).cloned();
            let Some(cb) = cb else { break };
            cb()?;
            i += 1;
        }

        let mut local_config_cmds = c.config_cmds.borrow().clone();
        local_config_cmds.insert(0, format!("allocate_oids count={}", c.oid_count.get()));

        // Resolve pin names
        {
            let ppins = self.printer.lookup::<PinsObject>("pins")?;
            let mut ppins = ppins.0.borrow_mut();
            let pin_resolver = ppins.get_pin_resolver(&self.name)?;
            for cmd in local_config_cmds.iter_mut() {
                *cmd = pin_resolver.update_command(cmd)?;
            }
            for cmd in c.restart_cmds.borrow_mut().iter_mut() {
                *cmd = pin_resolver.update_command(cmd)?;
            }
            for cmd in c.init_cmds.borrow_mut().iter_mut() {
                *cmd = pin_resolver.update_command(cmd)?;
            }
        }
        // Calculate config CRC
        let encoded_config = local_config_cmds.join("\n");
        let mut crc = flate2::Crc::new();
        crc.update(encoded_config.as_bytes());
        c.config_crc.set(crc.sum());
        local_config_cmds.push(format!("finalize_config crc={}", c.config_crc.get()));
        if let Some(prev_crc) = prev_crc {
            if c.config_crc.get() as i64 != prev_crc {
                self.restart_check_restart_on_crc_mismatch()?;
                return Err(error(format!("MCU '{}' CRC does not match config", self.name)));
            }
        }
        // Transmit config messages (if needed)
        let res = (|| {
            if prev_crc.is_none() {
                logging::info(&format!("Sending MCU '{}' printer configuration...", self.name));
                for cmd in &local_config_cmds {
                    self.serial.send(cmd, 0, 0)?;
                }
            } else {
                let cmds = c.restart_cmds.borrow().clone();
                for cmd in &cmds {
                    self.serial.send(cmd, 0, 0)?;
                }
            }
            // Transmit init messages
            let cmds = c.init_cmds.borrow().clone();
            for cmd in &cmds {
                self.serial.send(cmd, 0, 0)?;
            }
            Ok(())
        })();
        match res {
            Err(e @ KError::Enumeration { .. }) => {
                let KError::Enumeration { enum_name, value, .. } = &e else { unreachable!() };
                if enum_name != "pin" {
                    return Err(e);
                }
                // Raise pin name errors as a config error (not a protocol error)
                let msg = format!("Pin '{}' is not a valid pin name on mcu '{}'", value, self.name);
                Err(KError::config(msg).with_context(e))
            }
            r => r,
        }
    }

    fn cfg_send_get_config(&self) -> KResult<ConfigParams> {
        let get_config_cmd =
            self.lookup_query_command("get_config", "config is_config=%c crc=%u is_shutdown=%c move_count=%hu", None, None, false)?;
        if self.is_fileoutput() {
            return Ok(ConfigParams { is_config: 0, move_count: 500, crc: 0 });
        }
        let config_params = get_config_cmd.send(&[], 0, 0, true)?;
        if self.conn.is_shutdown.get() {
            return Err(error(format!(
                "MCU '{}' error during config: {}",
                self.name,
                self.conn.shutdown_msg.borrow()
            )));
        }
        if pint(&config_params, "is_shutdown") != 0 {
            return Err(error(format!("Can not update MCU '{}' config as it is shutdown", self.name)));
        }
        Ok(ConfigParams {
            is_config: pint(&config_params, "is_config"),
            crc: pint(&config_params, "crc"),
            move_count: pint(&config_params, "move_count"),
        })
    }

    fn cfg_reset_to_initial_state(&self) {
        let c = &self.cfg;
        if c.cached_init_state.get() {
            c.oid_count.set(c.oid_count_post_inits.get());
            *c.config_cmds.borrow_mut() = c.config_cmds_post_inits.borrow().clone();
            *c.init_cmds.borrow_mut() = c.init_cmds_post_inits.borrow().clone();
            *c.restart_cmds.borrow_mut() = c.restart_cmds_post_inits.borrow().clone();
        }
        c.reserved_move_slots.set(0);
    }

    fn cfg_connect(&self) -> KResult<()> {
        if self.non_critical_disconnected.get() {
            self.schedule_non_critical_reconnect();
            return Ok(());
        }
        self.cfg_connect_config()
    }

    fn cfg_connect_config(&self) -> KResult<()> {
        let mut config_params = self.cfg_send_get_config()?;
        if config_params.is_config == 0 {
            self.restart_check_restart_on_send_config()?;
            // Not configured - send config and issue get_config again
            self.cfg_send_config(None)?;
            config_params = self.cfg_send_get_config()?;
            if config_params.is_config == 0 && !self.is_fileoutput() {
                return Err(error(format!("Unable to configure MCU '{}'", self.name)));
            }
        } else {
            // if the mcu crc match the initial crc, the mcu lost comms but not
            // power and is reconnecting
            if self.cfg.config_crc.get() as i64 != config_params.crc {
                let start_reason = self.printer.start_arg("start_reason");
                if start_reason.as_ref().and_then(|v| v.as_str()) == Some("firmware_restart") {
                    return Err(error(format!("Failed automated reset of MCU '{}'", self.name)));
                }
                // Already configured - send init commands
                self.cfg_send_config(Some(config_params.crc))?;
            }
        }
        // Setup steppersync with the move_count returned by get_config
        let move_count = config_params.move_count;
        if move_count < self.cfg.reserved_move_slots.get() {
            return Err(error(format!("Too few moves available on MCU '{}'", self.name)));
        }
        let ss_move_count = move_count - self.cfg.reserved_move_slots.get();
        let motion_queuing = self.printer.lookup::<crate::extras::motion_queuing::PrinterMotionQueuing>("motion_queuing")?;
        let mcu = self.me.upgrade().expect("mcu");
        motion_queuing.setup_mcu_movequeue(&mcu, self.serial.get_serialqueue(), ss_move_count)?;
        // Log config information
        let move_msg = format!("Configured MCU '{}' ({} moves)", self.name, move_count);
        logging::info(&move_msg);
        let log_info = format!("{}\n{}", self.conn_log_info(), move_msg);
        self.printer.set_rollover_info(&self.name, Some(&log_info), false);
        Ok(())
    }

    fn cfg_mcu_identify(&self) -> KResult<()> {
        if self.non_critical_disconnected.get() {
            return Ok(());
        }
        let mcu_freq = self.get_constant_float("CLOCK_FREQ")?;
        self.cfg.mcu_freq.set(mcu_freq);
        if MAX_NOMINAL_DURATION * mcu_freq > MAX_SCHEDULE_TICKS as f64 {
            let max_possible = MAX_SCHEDULE_TICKS as f64 / mcu_freq;
            return Err(error(format!(
                "Too high clock speed for MCU '{}' to be able to resolve a maximum nominal duration of {}s. Max possible duration: {}s",
                self.name, MAX_NOMINAL_DURATION as i64, max_possible as i64
            )));
        }
        let ppins = self.printer.lookup::<PinsObject>("pins")?;
        let mut ppins = ppins.0.borrow_mut();
        let pin_resolver = ppins.get_pin_resolver(&self.name)?;
        for (cname, value) in self.get_constants() {
            if let Some(reserve_name) = cname.strip_prefix("RESERVE_PINS_") {
                for pin in py_str(&value).split(',') {
                    pin_resolver.reserve_pin(pin, reserve_name)?;
                }
            }
        }
        Ok(())
    }

    // Config creation helpers
    fn cfg_setup_pin(self: &Rc<Self>, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        Ok(match pin_type {
            "endstop" => McuEndstop::new(self, pin_params) as Rc<dyn Any>,
            "digital_out" => McuDigitalOut::new(self, pin_params),
            "pwm" => McuPwm::new(self, pin_params),
            "adc" => McuAdc::new(self, pin_params),
            _ => return Err(format!("pin type {} not supported on mcu", pin_type)),
        })
    }

    pub fn create_oid(&self) -> i64 {
        self.cfg.oid_count.set(self.cfg.oid_count.get() + 1);
        self.cfg.oid_count.get() - 1
    }

    pub fn register_config_callback(&self, cb: ConfigCallback) {
        self.cfg.config_callbacks.borrow_mut().push(cb);
    }

    /// register_config_callback() for an object whose python constructor
    /// registers before creating a helper that registers its own
    fn insert_config_callback_before_last(&self, cb: ConfigCallback) {
        let mut cbs = self.cfg.config_callbacks.borrow_mut();
        let pos = cbs.len().saturating_sub(1);
        cbs.insert(pos, cb);
    }

    pub fn add_config_cmd(&self, cmd: &str, is_init: bool, on_restart: bool) {
        let c = &self.cfg;
        if is_init {
            c.init_cmds.borrow_mut().push(cmd.to_string());
        } else if on_restart {
            c.restart_cmds.borrow_mut().push(cmd.to_string());
        } else {
            c.config_cmds.borrow_mut().push(cmd.to_string());
        }
    }

    pub fn get_query_slot(&self, oid: i64) -> i64 {
        let slot = self.seconds_to_clock(oid as f64 * 0.01);
        let t = (self.estimated_print_time(self.reactor.monotonic()) + 1.5) as i64;
        self.print_time_to_clock(t as f64) + slot
    }

    pub fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * self.cfg.mcu_freq.get()) as i64
    }

    pub fn request_move_queue_slot(&self) {
        self.cfg.reserved_move_slots.set(self.cfg.reserved_move_slots.get() + 1);
    }

    pub fn dump_debug(&self) -> String {
        let cmds = self.cfg.config_cmds.borrow();
        let mut out = vec![format!("Dumping config commands, {} commands, {} oids", cmds.len(), self.cfg.oid_count.get())];
        for (idx, cmd) in cmds.iter().enumerate() {
            out.push(format!("Config {}: {}", idx, cmd));
        }
        out.join("\n")
    }

    // ------------------------------------------------ MCU

    // Non-critical mcu handling
    pub fn handle_non_critical_disconnect(&self) -> KResult<()> {
        if self.non_critical_disconnected.get() || self.reconnecting.get() {
            // motion_queuing.stats() keeps calling calibrate_clock() (and
            // thus check_timeout()) while the mcu is disconnected or still
            // reconnecting; only handle the first disconnect notification
            return Ok(());
        }
        self.non_critical_disconnected.set(true);
        self.conn_disconnect();
        if let Some(t) = self.non_critical_recon_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NOW);
        }
        self.printer.send_event(&self.non_critical_disconnect_event_name, &[])?;
        self.gcode.respond_info(&format!("mcu: '{}' disconnected!", self.name), true);
        Ok(())
    }

    pub fn schedule_non_critical_reconnect(&self) {
        if let Some(t) = self.non_critical_recon_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NOW + self.reconnect_interval.get());
        }
    }

    fn non_critical_recon_event(self: &Rc<Self>, eventtime: f64) -> f64 {
        match self.recon_mcu() {
            Ok(true) => {
                self.gcode.respond_info(&format!("mcu: '{}' reconnected!", self.name), true);
                NEVER
            }
            Ok(false) => eventtime + self.reconnect_interval.get(),
            Err(e) => {
                self.printer.unhandled_exception(&e);
                NEVER
            }
        }
    }

    fn recon_mcu(self: &Rc<Self>) -> KResult<bool> {
        self.reconnecting.set(true);
        let res = (|| -> KResult<bool> {
            if !self.mcu_identify()? {
                return Ok(false);
            }
            self.identify_helpers()?;
            if self.restart_recon_handle_shutdown()? {
                self.abort_recon_attempt();
                return Ok(false);
            }
            self.conn.is_shutdown.set(false);
            self.cfg_reset_to_initial_state();
            self.non_critical_disconnected.set(false);
            self.cfg_connect_config()?;
            Ok(true)
        })();
        self.reconnecting.set(false);
        match res {
            Ok(false) => return Ok(false),
            Ok(true) => {}
            Err(_) => {
                logging::exception(&format!("Reconnect attempt on MCU '{}' failed", self.name));
                self.abort_recon_attempt();
                return Ok(false);
            }
        }
        self.printer.send_event(&self.non_critical_reconnect_event_name, &[])?;
        Ok(true)
    }

    fn abort_recon_attempt(&self) {
        // Return to the disconnected state; the reconnect timer retries
        self.conn_disconnect();
        self.non_critical_disconnected.set(true);
        self.stats_set_status_info("non_critical_disconnected", PyVal::Bool(true));
    }

    fn mcu_identify(&self) -> KResult<bool> {
        if self.is_non_critical.get() && !self.conn_check_serial_exists()? {
            self.non_critical_disconnected.set(true);
            self.stats_set_status_info("non_critical_disconnected", PyVal::Bool(true));
            return Ok(false);
        }
        self.non_critical_disconnected.set(false);
        if self.is_non_critical.get() {
            self.stats_set_status_info("non_critical_disconnected", PyVal::Bool(false));
        }
        Ok(true)
    }

    fn identify_helpers(self: &Rc<Self>) -> KResult<()> {
        // Same order as the "klippy:mcu_identify" handler registration
        self.conn_mcu_identify()?;
        self.restart_mcu_identify()?;
        self.cfg_mcu_identify()?;
        self.stats_mcu_identify()
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }

    pub fn get_printer(&self) -> Rc<Printer> {
        self.printer.clone()
    }

    pub fn get_non_critical_reconnect_event_name(&self) -> &str {
        &self.non_critical_reconnect_event_name
    }

    pub fn get_non_critical_disconnect_event_name(&self) -> &str {
        &self.non_critical_disconnect_event_name
    }

    pub fn is_fileoutput(&self) -> bool {
        self.printer.start_arg("debugoutput").is_some()
    }

    // Command Handler helpers
    pub fn min_schedule_time(&self) -> f64 {
        MIN_SCHEDULE_TIME
    }

    pub fn max_nominal_duration(&self) -> f64 {
        MAX_NOMINAL_DURATION
    }

    pub fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        CommandWrapper::new(&self.serial, msgformat, cq, self.is_fileoutput())
    }

    pub fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        oid: Option<i64>,
        cq: Option<CommandQueue>,
        is_async: bool,
    ) -> KResult<CommandQueryWrapper> {
        CommandQueryWrapper::new(&self.serial, msgformat, respformat, oid, cq, is_async, Some(KError::Command))
    }

    pub fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.lookup_command(msgformat, None).ok()
    }

    // SerialHdl wrappers
    pub fn register_response(&self, cb: Option<crate::serialhdl::ResponseHandler>, msg: &str, oid: Option<i64>) {
        self.serial.register_response(cb, msg, oid);
    }

    pub fn alloc_command_queue(&self) -> CommandQueue {
        self.serial.alloc_command_queue()
    }

    pub fn get_serial(&self) -> Rc<SerialReader> {
        self.serial.clone()
    }

    // MsgParser wrappers
    pub fn get_enumerations(&self) -> crate::msgproto::Enumerations {
        self.serial.get_msgparser().get_enumerations()
    }

    pub fn get_constants(&self) -> serde_json::Map<String, serde_json::Value> {
        self.serial.get_msgparser().get_constants()
    }

    pub fn get_constant_float(&self, name: &str) -> KResult<f64> {
        Ok(self.serial.get_msgparser().get_constant_float(name, crate::msgproto::Default::Sentinel)?)
    }

    // ClockSync wrappers
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        self.clocksync.print_time_to_clock(print_time)
    }

    pub fn clock_to_print_time(&self, clock: f64) -> f64 {
        self.clocksync.clock_to_print_time(clock)
    }

    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        // Alter time reporting when debugging
        if self.is_fileoutput() {
            return 0.0;
        }
        self.clocksync.estimated_print_time(eventtime)
    }

    pub fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        self.clocksync.clock32_to_clock64(clock32)
    }

    pub fn calibrate_clock(self: &Rc<Self>, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        let (offset, freq) = self.clocksync.calibrate_clock(print_time, eventtime);
        self.conn_check_timeout(eventtime)?;
        Ok((offset, freq))
    }
}

impl PinChip for Mcu {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let me = self.me.upgrade().expect("mcu");
        me.cfg_setup_pin(pin_type, pin_params)
    }
}

impl PrinterObject for Mcu {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(self.stats_h.get_status_info.borrow().clone()))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(self.stats_stats(eventtime))
    }
    fn teardown(&self) {
        self.cfg.config_callbacks.borrow_mut().clear();
        *self.conn.emergency_stop_cmd.borrow_mut() = None;
        if let Some(t) = self.non_critical_recon_timer.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
        self.serial.teardown();
    }
}

pub fn add_printer_objects(printer: &Rc<Printer>, config: &ConfigWrapper) -> KResult<()> {
    let reactor = printer.get_reactor();
    let mainsync = ClockSync::new(&reactor);
    printer.add_object("mcu", Mcu::new(&config.getsection("mcu"), mainsync.clone())?)?;
    for s in config.get_prefix_sections("mcu ") {
        let mcu = Mcu::new(&s, ClockSync::new_secondary(&reactor, mainsync.clone()))?;
        printer.add_object(s.get_name(), mcu)?;
    }
    Ok(())
}

pub fn get_printer_mcu(printer: &Printer, name: &str) -> KResult<Rc<Mcu>> {
    if name == "mcu" {
        return printer.lookup::<Mcu>(name);
    }
    printer.lookup::<Mcu>(&format!("mcu {}", name))
}
