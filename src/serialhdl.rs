// Serial port management for firmware communication
// (port of klippy/serialhdl.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Threading difference: python runs the response handlers in the reader's
// background thread. Here the thread forwards raw messages and ack
// notifications, in pull order, to the reactor thread, which parses and
// dispatches them (handlers hold Rc state). serialqueue delivers a
// command's response before its ack notification, so waiters still see the
// response first.
// pyserial (uart open / baud / rts / dtr) and python-can (socketcan raw
// sockets) are replaced by direct termios / socket calls.
use crate::chelper::serialqueue::{self, CommandQueue, SerialQueue};
use crate::error::{KError, KResult};
use crate::logging;
use crate::msgproto::{MessageParser, Params, Val};
use crate::reactor::{Reactor, ReactorCompletion, NOW};
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::collections::{HashMap, VecDeque};
use std::os::fd::RawFd;
use std::rc::{Rc, Weak};
use std::sync::{Arc, Mutex};

/// serialhdl.error
fn error(msg: String) -> KError {
    KError::Serial(msg)
}

pub type ResponseHandler = Rc<dyn Fn(&Params) -> KResult<()>>;

/// what the background thread hands to the reactor thread
enum Incoming {
    Notify { id: u64, sent_time: f64, receive_time: f64 },
    Msg { msg: Vec<u8>, sent_time: f64, receive_time: f64 },
}

static NEXT_READER_ID: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(1);

thread_local! {
    // reactor-thread registry: the async callbacks find their reader here
    static READERS: RefCell<HashMap<usize, Weak<SerialReader>>> = RefCell::new(HashMap::new());
}

/// the serial device: a file (batch output), a tty / pipe, or a can socket
pub struct SerialDev {
    pub fd: RawFd,
}

impl Drop for SerialDev {
    fn drop(&mut self) {
        unsafe { libc::close(self.fd) };
    }
}

pub struct SerialReader {
    id: usize,
    reactor: Rc<Reactor>,
    warn_prefix: String,
    /// mcu.non_critical_disconnected
    pub non_critical_disconnected: Rc<Cell<bool>>,
    sq_name: String,
    serial_dev: RefCell<Option<SerialDev>>,
    pub msgparser: RefCell<Rc<MessageParser>>,
    serialqueue: RefCell<Option<Arc<SerialQueue>>>,
    default_cmd_queue: CommandQueue,
    background_thread: RefCell<Option<std::thread::JoinHandle<()>>>,
    incoming: Arc<Mutex<VecDeque<Incoming>>>,
    handlers: RefCell<HashMap<(String, Option<i64>), ResponseHandler>>,
    last_notify_id: Cell<u64>,
    pending_notifications: RefCell<IndexMap<u64, Rc<ReactorCompletion<Option<Params>>>>>,
}

fn params_get_int(params: &Params, k: &str) -> i64 {
    params.get(k).and_then(|v| v.as_int()).unwrap_or(0)
}

/// params["#sent_time"] and the like
pub fn params_time(params: &Params, k: &str) -> f64 {
    match params.get(k) {
        Some(Val::Float(f)) => *f,
        Some(Val::Int(i)) => *i as f64,
        _ => 0.,
    }
}

fn time_val(t: f64) -> Val {
    Val::Float(t)
}

impl SerialReader {
    /// SerialReader(reactor, warn_prefix, mcu); `mcu_name` "" without mcu
    pub fn new(reactor: &Rc<Reactor>, warn_prefix: &str, mcu_name: &str, non_critical_disconnected: Rc<Cell<bool>>) -> Rc<Self> {
        let mut sq_name = format!("serialq {}", mcu_name);
        sq_name.truncate(15);
        let sr = Rc::new(SerialReader {
            id: NEXT_READER_ID.fetch_add(1, std::sync::atomic::Ordering::SeqCst),
            reactor: reactor.clone(),
            warn_prefix: warn_prefix.to_string(),
            non_critical_disconnected,
            sq_name,
            serial_dev: RefCell::new(None),
            msgparser: RefCell::new(Rc::new(MessageParser::new(warn_prefix))),
            serialqueue: RefCell::new(None),
            default_cmd_queue: serialqueue::alloc_commandqueue(),
            background_thread: RefCell::new(None),
            incoming: Arc::new(Mutex::new(VecDeque::new())),
            handlers: RefCell::new(HashMap::new()),
            last_notify_id: Cell::new(0),
            pending_notifications: RefCell::new(IndexMap::new()),
        });
        READERS.with(|r| r.borrow_mut().insert(sr.id, Rc::downgrade(&sr)));
        let s = Rc::downgrade(&sr);
        sr.register_response(
            Some(Rc::new(move |p: &Params| {
                if let Some(s) = s.upgrade() {
                    s.handle_unknown_init(p);
                }
                Ok(())
            })),
            "#unknown",
            None,
        );
        let s = Rc::downgrade(&sr);
        sr.register_response(
            Some(Rc::new(move |p: &Params| {
                if let Some(s) = s.upgrade() {
                    s.handle_output(p);
                }
                Ok(())
            })),
            "#output",
            None,
        );
        sr
    }

    fn start_bg_thread(&self, sq: Arc<SerialQueue>) {
        let incoming = self.incoming.clone();
        let handle = self.reactor.async_handle();
        let id = self.id;
        let t = std::thread::spawn(move || loop {
            let response = sq.pull();
            if response.len < 0 {
                break;
            }
            let item = if response.notify_id != 0 {
                Incoming::Notify { id: response.notify_id, sent_time: response.sent_time, receive_time: response.receive_time }
            } else {
                Incoming::Msg { msg: response.msg, sent_time: response.sent_time, receive_time: response.receive_time }
            };
            incoming.lock().unwrap().push_back(item);
            handle.register_async_callback(
                Box::new(move |_| {
                    let reader = READERS.with(|r| r.borrow().get(&id).and_then(|w| w.upgrade()));
                    if let Some(reader) = reader {
                        reader.drain_incoming();
                    }
                }),
                NOW,
            );
        });
        *self.background_thread.borrow_mut() = Some(t);
    }

    /// the reactor-thread part of python's _bg_thread
    fn drain_incoming(&self) {
        loop {
            let item = self.incoming.lock().unwrap().pop_front();
            let Some(item) = item else { break };
            match item {
                Incoming::Notify { id, sent_time, receive_time } => {
                    let mut params = Params::new();
                    params.insert("#sent_time".into(), time_val(sent_time));
                    params.insert("#receive_time".into(), time_val(receive_time));
                    let completion = self.pending_notifications.borrow_mut().shift_remove(&id);
                    if let Some(c) = completion {
                        c.complete(Some(params));
                    }
                }
                Incoming::Msg { msg, sent_time, receive_time } => {
                    let msgparser = self.msgparser.borrow().clone();
                    let res = (|| -> KResult<()> {
                        let mut params = msgparser.parse(&msg)?;
                        params.insert("#sent_time".into(), time_val(sent_time));
                        params.insert("#receive_time".into(), time_val(receive_time));
                        let name = match params.get("#name") {
                            Some(Val::Str(s)) => s.clone(),
                            _ => String::new(),
                        };
                        let oid = params.get("oid").and_then(|v| v.as_int());
                        let hdl = self.handlers.borrow().get(&(name, oid)).cloned();
                        match hdl {
                            Some(h) => h(&params),
                            None => {
                                self.handle_default(&params);
                                Ok(())
                            }
                        }
                    })();
                    if res.is_err() {
                        logging::exception(&format!("{}Exception in serial callback", self.warn_prefix));
                    }
                }
            }
        }
    }

    fn get_identify_data(self: &Rc<Self>) -> Option<Vec<u8>> {
        // Query the "data dictionary" from the micro-controller
        let mut identify_data: Vec<u8> = vec![];
        loop {
            let msg = format!("identify offset={} count={}", identify_data.len(), 40);
            let params = match self.send_with_response(&msg, "identify_response") {
                Ok(p) => p,
                Err(_) => {
                    logging::exception(&format!("{}Wait for identify_response", self.warn_prefix));
                    return None;
                }
            };
            if params_get_int(&params, "offset") == identify_data.len() as i64 {
                let msgdata = match params.get("data") {
                    Some(Val::Bytes(b)) => b.clone(),
                    _ => vec![],
                };
                if msgdata.is_empty() {
                    // Done
                    return Some(identify_data);
                }
                identify_data.extend(msgdata);
            }
        }
    }

    fn start_session(self: &Rc<Self>, serial_dev: SerialDev, serial_fd_type: u8, client_id: u32) -> KResult<bool> {
        let fd = serial_dev.fd;
        *self.serial_dev.borrow_mut() = Some(serial_dev);
        let sq = SerialQueue::alloc(fd, serial_fd_type, client_id, &self.sq_name)
            .ok_or_else(|| error(format!("{}Unable to allocate serialqueue", self.warn_prefix)))?;
        *self.serialqueue.borrow_mut() = Some(sq.clone());
        self.start_bg_thread(sq.clone());
        // Obtain and load the data dictionary from the firmware
        let s = self.clone();
        let completion = self.reactor.register_callback(move |_| s.get_identify_data(), NOW);
        let identify_data = completion.wait(self.reactor.monotonic() + 5.0, None)?.flatten();
        let Some(identify_data) = identify_data else {
            logging::info(&format!("{}Timeout on connect", self.warn_prefix));
            self.disconnect();
            return Ok(false);
        };
        let mut msgparser = MessageParser::new(&self.warn_prefix);
        msgparser.process_identify(&identify_data, true)?;
        let msgparser = Rc::new(msgparser);
        *self.msgparser.borrow_mut() = msgparser.clone();
        let s = Rc::downgrade(self);
        self.register_response(
            Some(Rc::new(move |p: &Params| {
                if let Some(s) = s.upgrade() {
                    s.handle_unknown(p);
                }
                Ok(())
            })),
            "#unknown",
            None,
        );
        // Setup baud adjust
        let wire_name = if serial_fd_type == serialqueue::SQT_CAN { "CANBUS_FREQUENCY" } else { "SERIAL_BAUD" };
        if let Some(wire_freq) = constant_float_opt(&msgparser, wire_name)? {
            sq.set_wire_frequency(wire_freq);
        }
        if let Some(receive_window) = constant_int_opt(&msgparser, "RECEIVE_WINDOW")? {
            sq.set_receive_window(receive_window as i32);
        }
        Ok(true)
    }

    pub fn check_canbus_connect(self: &Rc<Self>, canbus_uuid: &str, _canbus_nodeid: i64, canbus_iface: &str) -> KResult<bool> {
        let uuid = crate::pyfmt::py_int_digits(canbus_uuid, 16)
            .filter(|(neg, _, d)| !neg && d.len() <= 16)
            .map(|(_, _, d)| d.iter().fold(0i64, |a, x| a.wrapping_mul(16) + *x as i64))
            .unwrap_or(-1);
        if !(0..=0xFFFFFFFFFFFF).contains(&uuid) {
            return Err(error(format!("{}Invalid CAN uuid", self.warn_prefix)));
        }
        const CANBUS_ID_ADMIN: u32 = 0x3F0;
        const CMD_QUERY_UNASSIGNED: u8 = 0x00;
        const CMD_QUERY_UNASSIGNED_EXTENDED: u8 = 0x01;
        const RESP_NEED_NODEID: u8 = 0x20;
        const RESP_HAVE_NODEID: u8 = 0x21;
        let bus = match CanSocket::open(canbus_iface, CANBUS_ID_ADMIN + 1, 0x7FF) {
            Ok(b) => b,
            Err(e) => {
                logging::warning(&format!("{}can issue: {}", self.warn_prefix, e));
                return Ok(false);
            }
        };
        if let Err(e) = bus.send(CANBUS_ID_ADMIN, &[CMD_QUERY_UNASSIGNED, CMD_QUERY_UNASSIGNED_EXTENDED]) {
            logging::warning(&format!("{}can issue: {}", self.warn_prefix, e));
            return Ok(false);
        }
        let start_time = self.reactor.monotonic();
        let mut curtime = start_time;
        loop {
            let tdiff = start_time + 1.0 - curtime;
            if tdiff <= 0.0 {
                break;
            }
            let msg = bus.recv(tdiff);
            curtime = self.reactor.monotonic();
            let Some((id, data)) = msg else { continue };
            if id != CANBUS_ID_ADMIN + 1 || data.len() < 7 || !(data[0] == RESP_NEED_NODEID || data[0] == RESP_HAVE_NODEID) {
                continue;
            }
            let found_uuid = data[1..7].iter().enumerate().fold(0i64, |a, (i, v)| a + ((*v as i64) << ((5 - i) * 8)));
            if found_uuid == uuid {
                self.disconnect();
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub fn connect_canbus(self: &Rc<Self>, canbus_uuid: &str, canbus_nodeid: i64, canbus_iface: &str) -> KResult<()> {
        let txid = (canbus_nodeid * 2 + 256) as u32;
        // Prep for SET_NODEID command
        let uuid = crate::pyfmt::py_int_digits(canbus_uuid, 16)
            .filter(|(neg, _, d)| !neg && d.len() <= 16)
            .map(|(_, _, d)| d.iter().fold(0i64, |a, x| a.wrapping_mul(16) + *x as i64))
            .unwrap_or(-1);
        if !(0..=0xFFFFFFFFFFFF).contains(&uuid) {
            return Err(error(format!("{}Invalid CAN uuid", self.warn_prefix)));
        }
        let uuid: Vec<u8> = (0..6).map(|i| ((uuid >> (40 - i * 8)) & 0xFF) as u8).collect();
        const CANBUS_ID_ADMIN: u32 = 0x3F0;
        const CMD_SET_NODEID: u8 = 0x01;
        let mut set_id_cmd = vec![CMD_SET_NODEID];
        set_id_cmd.extend(&uuid);
        set_id_cmd.push(canbus_nodeid as u8);
        // Start connection attempt
        logging::info(&format!("{}Starting CAN connect", self.warn_prefix));
        let start_time = self.reactor.monotonic();
        loop {
            if self.reactor.monotonic() > start_time + 90.0 {
                return Err(error(format!("{}Unable to connect", self.warn_prefix)));
            }
            let bus = CanSocket::open(canbus_iface, txid + 1, 0x7FF).and_then(|b| b.send(CANBUS_ID_ADMIN, &set_id_cmd).map(|_| b));
            let bus = match bus {
                Ok(b) => b,
                Err(e) => {
                    logging::warning(&format!("{}Unable to open CAN port: {}", self.warn_prefix, e));
                    self.reactor.pause(self.reactor.monotonic() + 5.0)?;
                    continue;
                }
            };
            let fd = bus.into_fd();
            if !self.start_session(SerialDev { fd }, serialqueue::SQT_CAN, txid)? {
                continue;
            }
            // Verify correct canbus_nodeid to canbus_uuid mapping
            match self.send_with_response("get_canbus_id", "canbus_id") {
                Ok(params) => {
                    if let Some(Val::Bytes(got)) = params.get("canbus_uuid") {
                        if *got == uuid {
                            break;
                        }
                    }
                }
                Err(_) => logging::exception(&format!("{}Error in canbus_uuid check", self.warn_prefix)),
            }
            logging::info(&format!("{}Failed to match canbus_uuid - retrying..", self.warn_prefix));
            self.disconnect();
        }
        Ok(())
    }

    pub fn connect_pipe(self: &Rc<Self>, filename: &str) -> KResult<()> {
        logging::info(&format!("{}Starting connect", self.warn_prefix));
        let start_time = self.reactor.monotonic();
        loop {
            if self.reactor.monotonic() > start_time + 90.0 {
                return Err(error(format!("{}Unable to connect", self.warn_prefix)));
            }
            let c = std::ffi::CString::new(filename).unwrap();
            let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY) };
            if fd < 0 {
                let e = std::io::Error::last_os_error();
                logging::warning(&format!("{}Unable to open port: {}", self.warn_prefix, crate::pyfmt::py_oserror(&e, filename)));
                self.reactor.pause(self.reactor.monotonic() + 5.0)?;
                continue;
            }
            if self.start_session(SerialDev { fd }, serialqueue::SQT_UART, 0)? {
                break;
            }
        }
        Ok(())
    }

    pub fn connect_uart(self: &Rc<Self>, serialport: &str, baud: u32, rts: bool) -> KResult<()> {
        // Initial connection
        logging::info(&format!("{}Starting serial connect", self.warn_prefix));
        let start_time = self.reactor.monotonic();
        loop {
            if self.serialqueue.borrow().is_some() {
                // if we're already connected, don't recon
                break;
            }
            if self.reactor.monotonic() > start_time + 90.0 {
                return Err(error(format!("{}Unable to connect", self.warn_prefix)));
            }
            let dev = match Uart::open(serialport, baud, Some(true), Some(rts)) {
                Ok(d) => d,
                Err(e) => {
                    logging::warning(&format!("{}Unable to open serial port: {}", self.warn_prefix, e));
                    self.reactor.pause(self.reactor.monotonic() + 5.0)?;
                    continue;
                }
            };
            stk500v2_leave(&dev, &self.reactor)?;
            if self.start_session(SerialDev { fd: dev.into_fd() }, serialqueue::SQT_UART, 0)? {
                break;
            }
        }
        Ok(())
    }

    pub fn check_connect(&self, serialport: &str, baud: u32, rts: bool) -> bool {
        Uart::open(serialport, baud, Some(false), Some(rts)).is_ok()
    }

    /// connect_file(debugoutput, dictionary, pace)
    pub fn connect_file(&self, debugoutput: SerialDev, dictionary: &[u8]) -> KResult<()> {
        let fd = debugoutput.fd;
        *self.serial_dev.borrow_mut() = Some(debugoutput);
        let mut msgparser = MessageParser::new(&self.warn_prefix);
        msgparser.process_identify(dictionary, false)?;
        *self.msgparser.borrow_mut() = Rc::new(msgparser);
        let sq = SerialQueue::alloc(fd, serialqueue::SQT_DEBUGFILE, 0, &self.sq_name)
            .ok_or_else(|| error(format!("{}Unable to allocate serialqueue", self.warn_prefix)))?;
        *self.serialqueue.borrow_mut() = Some(sq);
        Ok(())
    }

    pub fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: u64, last_clock: u64) {
        if let Some(sq) = self.serialqueue.borrow().as_ref() {
            sq.set_clock_est(freq, conv_time, conv_clock, last_clock);
        }
    }

    pub fn disconnect(&self) {
        let sq = self.serialqueue.borrow_mut().take();
        if let Some(sq) = sq {
            sq.exit();
            if let Some(t) = self.background_thread.borrow_mut().take() {
                let _ = t.join();
            }
            drop(sq);
        }
        self.serial_dev.borrow_mut().take();
        let pending: Vec<_> = self.pending_notifications.borrow_mut().drain(..).map(|(_, c)| c).collect();
        for pn in pending {
            pn.complete(None);
        }
    }

    pub fn stats(&self, _eventtime: f64) -> String {
        match self.serialqueue.borrow().as_ref() {
            None => String::new(),
            Some(sq) => sq.get_stats(),
        }
    }

    pub fn get_reactor(&self) -> Rc<Reactor> {
        self.reactor.clone()
    }

    pub fn get_msgparser(&self) -> Rc<MessageParser> {
        self.msgparser.borrow().clone()
    }

    pub fn get_serialqueue(&self) -> Option<Arc<SerialQueue>> {
        self.serialqueue.borrow().clone()
    }

    pub fn get_default_command_queue(&self) -> CommandQueue {
        self.default_cmd_queue.clone()
    }

    /// register_response(callback, name, oid); None removes
    pub fn register_response(&self, callback: Option<ResponseHandler>, name: &str, oid: Option<i64>) {
        let key = (name.to_string(), oid);
        match callback {
            None => {
                self.handlers.borrow_mut().remove(&key);
            }
            Some(cb) => {
                self.handlers.borrow_mut().insert(key, cb);
            }
        }
    }

    fn check_noncritical_disconnected(&self) -> KResult<()> {
        if self.non_critical_disconnected.get() {
            return Err(error(format!("{}non-critical MCU is disconnected", self.warn_prefix)));
        }
        Ok(())
    }

    // Command sending
    pub fn raw_send(&self, cmd: &[u8], minclock: u64, reqclock: u64, cmd_queue: &CommandQueue) -> KResult<()> {
        self.check_noncritical_disconnected()?;
        if let Some(sq) = self.serialqueue.borrow().as_ref() {
            sq.send(cmd_queue, cmd, minclock, reqclock, 0);
        }
        Ok(())
    }

    pub fn raw_send_wait_ack(&self, cmd: &[u8], minclock: u64, reqclock: u64, cmd_queue: &CommandQueue) -> KResult<Option<Params>> {
        self.check_noncritical_disconnected()?;
        let Some(sq) = self.serialqueue.borrow().clone() else { return Ok(None) };
        self.last_notify_id.set(self.last_notify_id.get() + 1);
        let nid = self.last_notify_id.get();
        let completion = self.reactor.completion::<Option<Params>>();
        self.pending_notifications.borrow_mut().insert(nid, completion.clone());
        sq.send(cmd_queue, cmd, minclock, reqclock, nid);
        let params = completion.wait(crate::reactor::NEVER, None)?.flatten();
        match params {
            None => Err(error(format!("{}Serial connection closed", self.warn_prefix))),
            Some(p) => Ok(Some(p)),
        }
    }

    pub fn send(&self, msg: &str, minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = self.msgparser.borrow().create_command(msg)?;
        self.raw_send(&cmd, minclock, reqclock, &self.default_cmd_queue)
    }

    pub fn send_with_response(self: &Rc<Self>, msg: &str, response: &str) -> KResult<Params> {
        let cmd = self.msgparser.borrow().create_command(msg)?;
        let src = SerialRetryCommand::new(self, response, None);
        src.get_response(&[cmd], &self.default_cmd_queue, 0, 0, true)
    }

    /// not in python: drop the response handlers (they hold Rc state that
    /// refers back to the reader's owner) when the printer instance ends
    pub fn teardown(&self) {
        self.handlers.borrow_mut().clear();
    }

    pub fn alloc_command_queue(&self) -> CommandQueue {
        serialqueue::alloc_commandqueue()
    }

    // Dumping debug lists
    pub fn dump_debug(&self) -> String {
        let mut out = vec![format!("Dumping serial stats: {}", self.stats(self.reactor.monotonic()))];
        let Some(sq) = self.serialqueue.borrow().clone() else { return out.join("\n") };
        let sdata = sq.extract_old(true, 1024);
        let rdata = sq.extract_old(false, 1024);
        let msgparser = self.msgparser.borrow().clone();
        let dump = |m: &[u8]| msgparser.dump(m).map(|c| c.join(", ")).unwrap_or_default();
        out.push(format!("Dumping send queue {} messages", sdata.len()));
        for (i, msg) in sdata.iter().enumerate() {
            out.push(format!("Sent {} {:.6} {:.6} {}: {}", i, msg.receive_time, msg.sent_time, msg.len, dump(&msg.msg)));
        }
        out.push(format!("Dumping receive queue {} messages", rdata.len()));
        for (i, msg) in rdata.iter().enumerate() {
            out.push(format!("Receive: {} {:.6} {:.6} {}: {}", i, msg.receive_time, msg.sent_time, msg.len, dump(&msg.msg)));
        }
        out.join("\n")
    }

    // Default message handlers
    fn handle_unknown_init(&self, params: &Params) {
        let msglen = match params.get("#msg") {
            Some(Val::Bytes(b)) => b.len(),
            _ => 0,
        };
        logging::debug(&format!(
            "{}Unknown message {} (len {}) while identifying",
            self.warn_prefix,
            params_get_int(params, "#msgid"),
            msglen
        ));
    }

    fn handle_unknown(&self, params: &Params) {
        logging::warning(&format!(
            "{}Unknown message type {}: {}",
            self.warn_prefix,
            params_get_int(params, "#msgid"),
            params.get("#msg").map(|v| v.py_repr()).unwrap_or_default()
        ));
    }

    fn handle_output(&self, params: &Params) {
        logging::info(&format!(
            "{}{}: {}",
            self.warn_prefix,
            params.get("#name").map(|v| v.py_str()).unwrap_or_default(),
            params.get("#msg").map(|v| v.py_str()).unwrap_or_default()
        ));
    }

    fn handle_default(&self, params: &Params) {
        if crate::extras::danger_options::get_danger_options().log_serial_reader_warnings {
            logging::warning(&format!("{} got {}", self.warn_prefix, params_repr_py(params)));
        }
    }
}

impl Drop for SerialReader {
    fn drop(&mut self) {
        let id = self.id;
        let _ = READERS.try_with(|r| r.borrow_mut().remove(&id));
    }
}

/// repr() of a params dict
pub fn params_repr_py(params: &Params) -> String {
    crate::msgproto::params_repr(params)
}

fn constant_float_opt(mp: &MessageParser, name: &str) -> KResult<Option<f64>> {
    if !mp.config.contains_key(name) {
        return Ok(None);
    }
    Ok(Some(mp.get_constant_float(name, crate::msgproto::Default::Sentinel)?))
}

fn constant_int_opt(mp: &MessageParser, name: &str) -> KResult<Option<i64>> {
    if !mp.config.contains_key(name) {
        return Ok(None);
    }
    Ok(Some(mp.get_constant_int(name, crate::msgproto::Default::Sentinel)?))
}

// Class to send a query command and return the received response
pub struct SerialRetryCommand {
    serial: Rc<SerialReader>,
    name: String,
    oid: Option<i64>,
    last_params: Rc<RefCell<Option<Params>>>,
}

impl SerialRetryCommand {
    pub fn new(serial: &Rc<SerialReader>, name: &str, oid: Option<i64>) -> Self {
        let last_params: Rc<RefCell<Option<Params>>> = Rc::new(RefCell::new(None));
        let lp = last_params.clone();
        serial.register_response(
            Some(Rc::new(move |p: &Params| {
                *lp.borrow_mut() = Some(p.clone());
                Ok(())
            })),
            name,
            oid,
        );
        SerialRetryCommand { serial: serial.clone(), name: name.to_string(), oid, last_params }
    }

    pub fn get_response(&self, cmds: &[Vec<u8>], cmd_queue: &CommandQueue, minclock: u64, reqclock: u64, retry: bool) -> KResult<Params> {
        let mut retries = if retry { 5 } else { 0 };
        let mut retry_delay = 0.010;
        loop {
            for cmd in &cmds[..cmds.len() - 1] {
                self.serial.raw_send(cmd, minclock, reqclock, cmd_queue)?;
            }
            self.serial.raw_send_wait_ack(&cmds[cmds.len() - 1], minclock, reqclock, cmd_queue)?;
            let params = self.last_params.borrow().clone();
            if let Some(params) = params {
                self.serial.register_response(None, &self.name, self.oid);
                return Ok(params);
            }
            if retries <= 0 {
                self.serial.register_response(None, &self.name, self.oid);
                return Err(error(format!("Unable to obtain '{}' response", self.name)));
            }
            let reactor = self.serial.reactor.clone();
            reactor.pause(reactor.monotonic() + retry_delay)?;
            retries -= 1;
            retry_delay *= 2.0;
        }
    }
}

// ----------------------------------------------------------------------
// pyserial replacement: Serial(port, baudrate, timeout=0, exclusive=...)
// ----------------------------------------------------------------------

pub struct Uart {
    fd: RawFd,
}

/// termios2 / BOTHER (asm-generic layout; pyserial's custom baud path)
#[repr(C)]
#[derive(Clone, Copy)]
struct Termios2 {
    c_iflag: u32,
    c_oflag: u32,
    c_cflag: u32,
    c_lflag: u32,
    c_line: u8,
    c_cc: [u8; 19],
    c_ispeed: u32,
    c_ospeed: u32,
}

// (the ioctl request type: c_ulong on glibc, c_int on musl; the same bits)
const TCGETS2: libc::Ioctl = 0x802C542Au32 as libc::Ioctl;
const TCSETS2: libc::Ioctl = 0x402C542Bu32 as libc::Ioctl;
const BOTHER: u32 = 0o010000;
const CBAUD: u32 = 0o010017;

fn std_baud(baud: u32) -> Option<libc::speed_t> {
    Some(match baud {
        50 => libc::B50,
        75 => libc::B75,
        110 => libc::B110,
        134 => libc::B134,
        150 => libc::B150,
        200 => libc::B200,
        300 => libc::B300,
        600 => libc::B600,
        1200 => libc::B1200,
        1800 => libc::B1800,
        2400 => libc::B2400,
        4800 => libc::B4800,
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        576000 => libc::B576000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        1152000 => libc::B1152000,
        1500000 => libc::B1500000,
        2000000 => libc::B2000000,
        2500000 => libc::B2500000,
        3000000 => libc::B3000000,
        3500000 => libc::B3500000,
        4000000 => libc::B4000000,
        _ => return None,
    })
}

// pyserial 3.4 serialposix errors: SerialException raised while handling
// the OSError / termios.error underneath
fn serial_exception(msg: String, context: KError) -> KError {
    KError::Py("serial.serialutil.SerialException".into(), msg).with_context(context)
}

/// str() of an OSError without a filename: "[Errno N] message"
fn errno_str(e: &std::io::Error) -> String {
    let errno = e.raw_os_error().unwrap_or(0);
    let msg = unsafe { std::ffi::CStr::from_ptr(libc::strerror(errno)) }.to_string_lossy().into_owned();
    format!("[Errno {}] {}", errno, msg)
}

fn os_error(e: &std::io::Error) -> KError {
    KError::Py(crate::pyfmt::py_oserror_class(e).into(), errno_str(e))
}

/// termios.error: str() is the (errno, message) tuple
fn termios_error(e: &std::io::Error) -> KError {
    let errno = e.raw_os_error().unwrap_or(0);
    let msg = unsafe { std::ffi::CStr::from_ptr(libc::strerror(errno)) }.to_string_lossy().into_owned();
    KError::Py("termios.error".into(), format!("({}, {})", errno, crate::pyfmt::py_str_repr(&msg)))
}

/// _set_special_baudrate: ValueError raised while handling the ioctl's OSError
fn custom_baud_error(baud: u32) -> KError {
    let e = std::io::Error::last_os_error();
    KError::Py("ValueError".into(), format!("Failed to set custom baud rate ({}): {}", baud, errno_str(&e))).with_context(os_error(&e))
}

impl Uart {
    /// pyserial Serial(baudrate=baud, timeout=0, exclusive=...) + port +
    /// rts + open(); dtr is asserted on open like pyserial
    pub fn open(port: &str, baud: u32, exclusive: Option<bool>, rts: Option<bool>) -> KResult<Uart> {
        let c = std::ffi::CString::new(port).map_err(|e| KError::Py("ValueError".into(), e.to_string()))?;
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK) };
        if fd < 0 {
            let e = std::io::Error::last_os_error();
            let ctx = KError::Py(crate::pyfmt::py_oserror_class(&e).into(), crate::pyfmt::py_oserror(&e, port));
            return Err(serial_exception(
                format!("[Errno {}] could not open port {}: {}", e.raw_os_error().unwrap_or(0), port, crate::pyfmt::py_oserror(&e, port)),
                ctx,
            ));
        }
        // (dropped, so closed, when the configuration fails)
        let uart = Uart { fd };
        // _reconfigure_port: the exclusive lock comes before anything else
        // (exclusive=None: no flock at all)
        if let Some(exclusive) = exclusive {
            let lock = if exclusive { libc::LOCK_EX | libc::LOCK_NB } else { libc::LOCK_UN };
            if unsafe { libc::flock(fd, lock) } != 0 {
                let e = std::io::Error::last_os_error();
                if !exclusive {
                    return Err(os_error(&e));
                }
                return Err(serial_exception(
                    format!("[Errno {}] Could not exclusively lock port {}: {}", e.raw_os_error().unwrap_or(0), port, errno_str(&e)),
                    os_error(&e),
                ));
            }
        }
        uart.configure(baud)?;
        uart.set_modem_line(libc::TIOCM_DTR, true);
        if let Some(r) = rts {
            uart.set_modem_line(libc::TIOCM_RTS, r);
        }
        Ok(uart)
    }

    fn configure(&self, baud: u32) -> KResult<()> {
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(self.fd, &mut t) != 0 {
                // a port that has a /dev file but is no tty fails here
                let e = std::io::Error::last_os_error();
                let te = termios_error(&e);
                return Err(serial_exception(format!("Could not configure port: {}", te.message()), te));
            }
            // pyserial: raw mode, 8N1, CLOCAL | CREAD, no flow control, VMIN 0 VTIME 0
            t.c_cflag |= libc::CLOCAL | libc::CREAD;
            t.c_lflag &= !(libc::ICANON | libc::ECHO | libc::ECHOE | libc::ECHOK | libc::ECHONL | libc::ISIG | libc::IEXTEN | libc::ECHOCTL | libc::ECHOKE);
            t.c_oflag &= !(libc::OPOST | libc::ONLCR | libc::OCRNL);
            t.c_iflag &= !(libc::INLCR | libc::IGNCR | libc::ICRNL | libc::IGNBRK | libc::INPCK | libc::ISTRIP | libc::IXON | libc::IXOFF | libc::IXANY | libc::PARMRK);
            t.c_cflag &= !(libc::CSIZE | libc::CSTOPB | libc::PARENB | libc::PARODD | libc::CRTSCTS);
            t.c_cflag |= libc::CS8;
            t.c_cc[libc::VMIN] = 0;
            t.c_cc[libc::VTIME] = 0;
            let std = std_baud(baud);
            let speed = std.unwrap_or(libc::B38400);
            libc::cfsetispeed(&mut t, speed);
            libc::cfsetospeed(&mut t, speed);
            if libc::tcsetattr(self.fd, libc::TCSANOW, &t) != 0 {
                return Err(termios_error(&std::io::Error::last_os_error()));
            }
            if std.is_none() {
                // custom baud rate via termios2 / BOTHER
                let mut t2: Termios2 = std::mem::zeroed();
                if libc::ioctl(self.fd, TCGETS2, &mut t2) != 0 {
                    return Err(custom_baud_error(baud));
                }
                t2.c_cflag &= !CBAUD;
                t2.c_cflag |= BOTHER;
                t2.c_ispeed = baud;
                t2.c_ospeed = baud;
                if libc::ioctl(self.fd, TCSETS2, &t2) != 0 {
                    return Err(custom_baud_error(baud));
                }
            }
        }
        Ok(())
    }

    fn set_modem_line(&self, bit: libc::c_int, on: bool) {
        let req = if on { libc::TIOCMBIS } else { libc::TIOCMBIC };
        unsafe { libc::ioctl(self.fd, req, &bit) };
    }

    pub fn set_baudrate(&self, baud: u32) -> KResult<()> {
        self.configure(baud)
    }

    pub fn set_dtr(&self, on: bool) {
        self.set_modem_line(libc::TIOCM_DTR, on);
    }

    pub fn set_rts(&self, on: bool) {
        self.set_modem_line(libc::TIOCM_RTS, on);
    }

    /// read(n) with timeout=0: whatever is available
    pub fn read(&self, n: usize) -> Vec<u8> {
        let mut buf = vec![0u8; n];
        let r = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, n) };
        buf.truncate(r.max(0) as usize);
        buf
    }

    pub fn write(&self, data: &[u8]) {
        unsafe { libc::write(self.fd, data.as_ptr() as *const libc::c_void, data.len()) };
    }

    /// pyserial read(n) with timeout=0, raising like it: a readable port
    /// that returns no data is a disconnected device
    pub fn read_checked(&self, n: usize) -> Result<Vec<u8>, String> {
        let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLIN, revents: 0 };
        if unsafe { libc::poll(&mut pfd, 1, 0) } <= 0 {
            return Ok(vec![]);
        }
        let mut buf = vec![0u8; n];
        let r = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, n) };
        if r == 0 {
            return Err("device reports readiness to read but returned no data (device disconnected or multiple access on port?)".into());
        }
        if r < 0 {
            let e = std::io::Error::last_os_error();
            return match e.raw_os_error() {
                // EWOULDBLOCK is EAGAIN on linux
                Some(libc::EAGAIN) | Some(libc::EALREADY) | Some(libc::EINPROGRESS) | Some(libc::EINTR) => Ok(vec![]),
                _ => Err(format!("read failed: {}", e)),
            };
        }
        buf.truncate(r as usize);
        Ok(buf)
    }

    /// pyserial write(data) with write_timeout=0: anything not written at
    /// once is a write timeout
    pub fn write_checked(&self, data: &[u8]) -> Result<(), String> {
        let r = unsafe { libc::write(self.fd, data.as_ptr() as *const libc::c_void, data.len()) };
        if r < 0 {
            let e = std::io::Error::last_os_error();
            if e.raw_os_error() != Some(libc::EAGAIN) {
                return Err(format!("write failed: {}", e));
            }
        }
        if r < data.len() as isize {
            return Err("Write timeout".into());
        }
        Ok(())
    }


    pub fn into_fd(self) -> RawFd {
        let fd = self.fd;
        std::mem::forget(self);
        fd
    }
}

impl Drop for Uart {
    fn drop(&mut self) {
        unsafe { libc::close(self.fd) };
    }
}

// Attempt to place an AVR stk500v2 style programmer into normal mode
pub fn stk500v2_leave(ser: &Uart, reactor: &Rc<Reactor>) -> KResult<()> {
    logging::debug("Starting stk500v2 leave programmer sequence");
    crate::util::clear_hupcl(ser.fd);
    // pyserial keeps its configured baud; origbaud = ser.baudrate
    let mut t: libc::termios = unsafe { std::mem::zeroed() };
    unsafe { libc::tcgetattr(ser.fd, &mut t) };
    // Request a dummy speed first as this seems to help reset the port
    let _ = ser.set_baudrate(2400);
    ser.read(1);
    // Send stk500v2 leave programmer sequence
    let _ = ser.set_baudrate(115200);
    reactor.pause(reactor.monotonic() + 0.100)?;
    ser.read(4096);
    ser.write(b"\x1b\x01\x00\x01\x0e\x11\x04");
    reactor.pause(reactor.monotonic() + 0.050)?;
    let res = ser.read(4096);
    logging::debug(&format!("Got {} from stk500v2", crate::pyfmt::py_bytes_repr(&res)));
    // restore the original settings (custom bauds included)
    unsafe { libc::tcsetattr(ser.fd, libc::TCSANOW, &t) };
    Ok(())
}

pub fn cheetah_reset(serialport: &str, reactor: &Rc<Reactor>) -> KResult<()> {
    // Fysetc Cheetah v1.2 boards have a weird stateful circuitry for
    // configuring the bootloader. This sequence takes care of disabling it for
    // sure.
    // Open the serial port with RTS asserted
    let ser = Uart::open(serialport, 2400, Some(true), Some(true))?;
    ser.read(1);
    reactor.pause(reactor.monotonic() + 0.100)?;
    // Toggle DTR
    ser.set_dtr(true);
    reactor.pause(reactor.monotonic() + 0.100)?;
    ser.set_dtr(false);
    // Deassert RTS
    reactor.pause(reactor.monotonic() + 0.100)?;
    ser.set_rts(false);
    reactor.pause(reactor.monotonic() + 0.100)?;
    // Toggle DTR again
    ser.set_dtr(true);
    reactor.pause(reactor.monotonic() + 0.100)?;
    ser.set_dtr(false);
    reactor.pause(reactor.monotonic() + 0.100)?;
    Ok(())
}

// Attempt an arduino style reset on a serial port
pub fn arduino_reset(serialport: &str, reactor: &Rc<Reactor>) -> KResult<()> {
    // First try opening the port at a different baud
    let ser = Uart::open(serialport, 2400, Some(true), None)?;
    ser.read(1);
    reactor.pause(reactor.monotonic() + 0.100)?;
    // Then toggle DTR
    ser.set_dtr(true);
    reactor.pause(reactor.monotonic() + 0.100)?;
    ser.set_dtr(false);
    reactor.pause(reactor.monotonic() + 0.100)?;
    Ok(())
}

// ----------------------------------------------------------------------
// python-can replacement: a raw socketcan socket with one filter
// ----------------------------------------------------------------------

pub struct CanSocket {
    fd: RawFd,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct CanFrame {
    can_id: u32,
    can_dlc: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    data: [u8; 8],
}

#[repr(C)]
struct SockaddrCan {
    can_family: libc::sa_family_t,
    can_ifindex: libc::c_int,
    rx_id: u32,
    tx_id: u32,
}

#[repr(C)]
struct CanFilter {
    can_id: u32,
    can_mask: u32,
}

impl CanSocket {
    pub fn open(iface: &str, filter_id: u32, filter_mask: u32) -> Result<CanSocket, String> {
        unsafe {
            let fd = libc::socket(libc::PF_CAN, libc::SOCK_RAW, libc::CAN_RAW);
            if fd < 0 {
                return Err(std::io::Error::last_os_error().to_string());
            }
            let sock = CanSocket { fd };
            // python-can: standard (11 bit) filter -> mask includes the EFF flag
            let filter = CanFilter { can_id: filter_id, can_mask: filter_mask | libc::CAN_EFF_FLAG };
            libc::setsockopt(
                fd,
                libc::SOL_CAN_RAW,
                libc::CAN_RAW_FILTER,
                &filter as *const CanFilter as *const libc::c_void,
                std::mem::size_of::<CanFilter>() as libc::socklen_t,
            );
            let ciface = std::ffi::CString::new(iface).map_err(|e| e.to_string())?;
            let ifindex = libc::if_nametoindex(ciface.as_ptr());
            if ifindex == 0 {
                return Err(format!("Failed to bind to interface {}: {}", iface, std::io::Error::last_os_error()));
            }
            let addr = SockaddrCan { can_family: libc::AF_CAN as libc::sa_family_t, can_ifindex: ifindex as libc::c_int, rx_id: 0, tx_id: 0 };
            if libc::bind(fd, &addr as *const SockaddrCan as *const libc::sockaddr, std::mem::size_of::<SockaddrCan>() as libc::socklen_t) != 0 {
                return Err(format!("Failed to bind to interface {}: {}", iface, std::io::Error::last_os_error()));
            }
            Ok(sock)
        }
    }

    pub fn send(&self, can_id: u32, data: &[u8]) -> Result<(), String> {
        let mut cf = CanFrame { can_id, can_dlc: data.len() as u8, ..Default::default() };
        cf.data[..data.len()].copy_from_slice(data);
        let r = unsafe { libc::write(self.fd, &cf as *const CanFrame as *const libc::c_void, std::mem::size_of::<CanFrame>()) };
        if r < 0 {
            return Err(std::io::Error::last_os_error().to_string());
        }
        Ok(())
    }

    /// recv(timeout): (arbitration id, data)
    pub fn recv(&self, timeout: f64) -> Option<(u32, Vec<u8>)> {
        let mut p = libc::pollfd { fd: self.fd, events: libc::POLLIN, revents: 0 };
        if unsafe { libc::poll(&mut p, 1, (timeout * 1000.0).ceil() as i32) } <= 0 {
            return None;
        }
        let mut cf = CanFrame::default();
        let r = unsafe { libc::read(self.fd, &mut cf as *mut CanFrame as *mut libc::c_void, std::mem::size_of::<CanFrame>()) };
        if r <= 0 {
            return None;
        }
        Some((cf.can_id & libc::CAN_EFF_MASK, cf.data[..cf.can_dlc.min(8) as usize].to_vec()))
    }

    pub fn into_fd(self) -> RawFd {
        let fd = self.fd;
        std::mem::forget(self);
        fd
    }
}

impl Drop for CanSocket {
    fn drop(&mut self) {
        unsafe { libc::close(self.fd) };
    }
}
