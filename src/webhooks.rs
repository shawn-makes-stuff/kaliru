// Klippy WebHooks registration and server connection
// (port of klippy/webhooks.py)
//
// Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license
//
// Differences: "klipper_path" / "python_path" in the info response are the
// directory above the kaliru binary / the binary itself; client ids are rust
// pointer values like python's id(); numpy conversion in send() is not
// needed (PyVal is always serializable).
use crate::error::{KError, KResult};
use crate::gcode::GCodeDispatch;
use crate::logging;
use crate::printer::{Printer, PrinterObject, APP_NAME};
use crate::pyfmt::py_bytes_repr;
use crate::pyval::PyVal;
use crate::reactor::{FileHandler, ReactorCompletion, Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::os::fd::RawFd;
use std::rc::Rc;

pub const REQUEST_LOG_SIZE: usize = 20;

/// WebRequestError (a gcode.CommandError)
fn web_error(msg: impl Into<String>) -> KError {
    KError::Command(msg.into())
}

fn error_dict(message: &str) -> PyVal {
    let mut d = IndexMap::new();
    d.insert("error".to_string(), PyVal::Str("WebRequestError".into()));
    d.insert("message".to_string(), PyVal::Str(message.to_string()));
    PyVal::Dict(d)
}

pub type Endpoint = Rc<dyn Fn(&WebRequest) -> KResult<()>>;

pub struct WebRequest {
    client_conn: Option<Rc<ClientConnection>>,
    pub id: PyVal,
    pub method: String,
    pub params: IndexMap<String, PyVal>,
    response: RefCell<Option<PyVal>>,
    is_error: Cell<bool>,
}

/// expected python types for WebRequest.get(..., types=...)
#[derive(Clone, Copy)]
pub enum Ty {
    Str,
    Int,
    Float,
    Dict,
}

fn type_ok(v: &PyVal, types: &[Ty]) -> bool {
    types.iter().any(|t| matches!((t, v), (Ty::Str, PyVal::Str(_)) | (Ty::Int, PyVal::Int(_)) | (Ty::Float, PyVal::Float(_)) | (Ty::Dict, PyVal::Dict(_))))
}

impl WebRequest {
    /// WebRequest(client_conn, request); Err is the python exception text
    pub fn new(client_conn: Option<Rc<ClientConnection>>, request: &str) -> Result<WebRequest, String> {
        let base_request = PyVal::json_loads(request)?;
        let PyVal::Dict(base) = base_request else {
            return Err("Not a top-level dictionary".into());
        };
        let id = base.get("id").cloned().unwrap_or(PyVal::None);
        let method = base.get("method").cloned().unwrap_or(PyVal::None);
        let params = base.get("params").cloned().unwrap_or_else(PyVal::dict);
        let (PyVal::Str(method), PyVal::Dict(params)) = (method, params) else {
            return Err("Invalid request type".into());
        };
        Ok(WebRequest { client_conn, id, method, params, response: RefCell::new(None), is_error: Cell::new(false) })
    }

    pub fn get_client_connection(&self) -> Option<Rc<ClientConnection>> {
        self.client_conn.clone()
    }

    /// get(item, default=Sentinel, types=None); default None = required
    pub fn get(&self, item: &str, default: Option<PyVal>, types: Option<&[Ty]>) -> KResult<PyVal> {
        let value = match (self.params.get(item), default) {
            (Some(v), _) => v.clone(),
            (None, Some(d)) => return Ok(d),
            (None, None) => return Err(web_error(format!("Missing Argument [{}]", item))),
        };
        if let Some(types) = types {
            if !type_ok(&value, types) {
                return Err(web_error(format!("Invalid Argument Type [{}]", item)));
            }
        }
        Ok(value)
    }

    pub fn get_str(&self, item: &str, default: Option<PyVal>) -> KResult<PyVal> {
        self.get(item, default, Some(&[Ty::Str]))
    }

    pub fn get_int(&self, item: &str, default: Option<PyVal>) -> KResult<PyVal> {
        self.get(item, default, Some(&[Ty::Int]))
    }

    /// float(get(item, default, types=(int, float)))
    pub fn get_float(&self, item: &str, default: Option<PyVal>) -> KResult<f64> {
        let v = self.get(item, default, Some(&[Ty::Int, Ty::Float]))?;
        v.as_f64().ok_or_else(|| KError::Other(format!("float() argument must be a string or a real number, not '{}'", v.repr())))
    }

    pub fn get_dict(&self, item: &str, default: Option<PyVal>) -> KResult<PyVal> {
        self.get(item, default, Some(&[Ty::Dict]))
    }

    pub fn get_method(&self) -> &str {
        &self.method
    }

    pub fn set_error(&self, message: &str) {
        self.is_error.set(true);
        *self.response.borrow_mut() = Some(error_dict(message));
    }

    pub fn send(&self, data: PyVal) -> KResult<()> {
        if self.response.borrow().is_some() {
            return Err(web_error("Multiple calls to send not allowed"));
        }
        *self.response.borrow_mut() = Some(data);
        Ok(())
    }

    pub fn finish(&self) -> Option<PyVal> {
        if self.id == PyVal::None {
            return None;
        }
        let rtype = if self.is_error.get() { "error" } else { "result" };
        // No error was set and the user never executed send, default
        // response is {}
        let response = self.response.borrow().clone().unwrap_or_else(PyVal::dict);
        let mut d = IndexMap::new();
        d.insert("id".to_string(), self.id.clone());
        d.insert(rtype.to_string(), response);
        Some(PyVal::Dict(d))
    }
}

pub struct ServerSocket {
    printer: Rc<Printer>,
    sock: Cell<RawFd>,
    fd_handle: RefCell<Option<FileHandler>>,
    clients: RefCell<IndexMap<usize, Rc<ClientConnection>>>,
}

impl ServerSocket {
    fn new(webhooks: &Rc<WebHooks>, printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let ss = Rc::new(ServerSocket { printer: printer.clone(), sock: Cell::new(-1), fd_handle: RefCell::new(None), clients: RefCell::new(IndexMap::new()) });
        let server_address = printer.start_arg("apiserver_file").map(|v| v.str());
        let is_fileinput = printer.start_arg("debuginput").is_some();
        let Some(server_address) = server_address.filter(|s| !s.is_empty() && !is_fileinput) else {
            // Do not enable server
            return Ok(ss);
        };
        ss.remove_socket_file(&server_address)?;
        let sock = unsafe { libc::socket(libc::AF_UNIX, libc::SOCK_STREAM, 0) };
        if sock < 0 {
            return Err(KError::Other(std::io::Error::last_os_error().to_string()));
        }
        crate::util::set_nonblock(sock);
        let mut addr: libc::sockaddr_un = unsafe { std::mem::zeroed() };
        addr.sun_family = libc::AF_UNIX as libc::sa_family_t;
        let bytes = server_address.as_bytes();
        if bytes.len() >= addr.sun_path.len() {
            return Err(KError::Other("AF_UNIX path too long".into()));
        }
        for (i, b) in bytes.iter().enumerate() {
            addr.sun_path[i] = *b as libc::c_char;
        }
        let r = unsafe {
            libc::bind(sock, &addr as *const libc::sockaddr_un as *const libc::sockaddr, std::mem::size_of::<libc::sockaddr_un>() as libc::socklen_t)
        };
        if r != 0 {
            return Err(KError::Other(crate::pyfmt::py_oserror(&std::io::Error::last_os_error(), &server_address)));
        }
        apply_permission_options_to_socket(&server_address, printer)?;
        unsafe { libc::listen(sock, 1) };
        ss.sock.set(sock);
        let (s, w) = (ss.clone(), Rc::downgrade(webhooks));
        let h = printer.reactor.register_fd(
            sock,
            Box::new(move |_| {
                if let Some(wh) = w.upgrade() {
                    s.handle_accept(&wh);
                }
            }),
            None,
        );
        *ss.fd_handle.borrow_mut() = Some(h);
        let s = ss.clone();
        printer.register_event_handler("klippy:disconnect", Rc::new(move |_| {
            s.handle_disconnect();
            Ok(())
        }));
        let s = ss.clone();
        printer.register_event_handler("klippy:analyze_shutdown", Rc::new(move |_| {
            for client in s.clients.borrow().values() {
                client.dump_request_log();
            }
            Ok(())
        }));
        Ok(ss)
    }

    fn handle_accept(self: &Rc<Self>, webhooks: &Rc<WebHooks>) {
        let fd = unsafe { libc::accept(self.sock.get(), std::ptr::null_mut(), std::ptr::null_mut()) };
        if fd < 0 {
            return;
        }
        crate::util::set_nonblock(fd);
        let client = ClientConnection::new(self, webhooks, fd);
        self.clients.borrow_mut().insert(client.uid, client);
    }

    fn handle_disconnect(&self) {
        let clients: Vec<_> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            client.close();
        }
        if self.sock.get() >= 0 {
            if let Some(h) = self.fd_handle.borrow_mut().take() {
                self.printer.reactor.unregister_fd(&h);
            }
            unsafe { libc::close(self.sock.get()) };
        }
    }

    fn remove_socket_file(&self, file_path: &str) -> KResult<()> {
        if let Err(e) = std::fs::remove_file(file_path) {
            if std::path::Path::new(file_path).exists() {
                logging::exception(&format!("webhooks: Unable to delete socket file '{}'", file_path));
                return Err(KError::Other(crate::pyfmt::py_oserror(&e, file_path)));
            }
        }
        Ok(())
    }

    fn pop_client(&self, client_id: usize) {
        self.clients.borrow_mut().shift_remove(&client_id);
    }

    pub fn stats(&self, _eventtime: f64) -> (bool, String) {
        // Called once per second - check for idle clients
        let clients: Vec<_> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            if client.is_blocking.get() {
                client.blocking_count.set(client.blocking_count.get() - 1);
                if client.blocking_count.get() < 0 {
                    logging::info(&format!("Closing unresponsive client {}", client.uid));
                    client.close();
                }
            }
        }
        (false, String::new())
    }
}

fn apply_permission_options_to_socket(socket_path: &str, printer: &Printer) -> KResult<()> {
    let user = printer.start_arg("apiserver_user").map(|v| v.str());
    let group = printer.start_arg("apiserver_group").map(|v| v.str());
    let file_mode = printer.start_arg("apiserver_file_mode").map(|v| v.str());
    let cpath = std::ffi::CString::new(socket_path).unwrap();
    if user.is_some() || group.is_some() {
        let uid = match &user {
            None => u32::MAX,
            Some(u) => {
                let c = std::ffi::CString::new(u.as_str()).unwrap();
                let pw = unsafe { libc::getpwnam(c.as_ptr()) };
                if pw.is_null() {
                    return Err(KError::Other(format!("getpwnam(): name not found: {}", crate::pyfmt::py_str_repr(u))));
                }
                unsafe { (*pw).pw_uid }
            }
        };
        let gid = match &group {
            None => u32::MAX,
            Some(g) => {
                let c = std::ffi::CString::new(g.as_str()).unwrap();
                let gr = unsafe { libc::getgrnam(c.as_ptr()) };
                if gr.is_null() {
                    return Err(KError::Other(format!("getgrnam(): name not found: {}", crate::pyfmt::py_str_repr(g))));
                }
                unsafe { (*gr).gr_gid }
            }
        };
        unsafe { libc::chown(cpath.as_ptr(), uid, gid) };
    }
    if let Some(mode) = file_mode {
        let mode_int = crate::pyfmt::py_int_digits(&mode, 8)
            .and_then(|(neg, _, d)| if neg { None } else { Some(d.iter().fold(0u32, |a, x| a * 8 + *x as u32)) })
            .ok_or_else(|| KError::Other(format!("invalid literal for int() with base 8: {}", crate::pyfmt::py_str_repr(&mode))))?;
        unsafe { libc::chmod(cpath.as_ptr(), mode_int as libc::mode_t) };
    }
    Ok(())
}

pub struct ClientConnection {
    printer: Rc<Printer>,
    webhooks: std::rc::Weak<WebHooks>,
    server: std::rc::Weak<ServerSocket>,
    pub uid: usize,
    sock: RawFd,
    fd_handle: RefCell<Option<FileHandler>>,
    partial_data: RefCell<Vec<u8>>,
    send_buffer: RefCell<Vec<u8>>,
    is_blocking: Cell<bool>,
    blocking_count: Cell<i64>,
    request_log: RefCell<VecDeque<(f64, Vec<u8>)>>,
}

impl ClientConnection {
    fn new(server: &Rc<ServerSocket>, webhooks: &Rc<WebHooks>, sock: RawFd) -> Rc<Self> {
        let c = Rc::new_cyclic(|me: &std::rc::Weak<ClientConnection>| ClientConnection {
            printer: server.printer.clone(),
            webhooks: Rc::downgrade(webhooks),
            server: Rc::downgrade(server),
            // id(self)
            uid: me.as_ptr() as usize,
            sock,
            fd_handle: RefCell::new(None),
            partial_data: RefCell::new(vec![]),
            send_buffer: RefCell::new(vec![]),
            is_blocking: Cell::new(false),
            blocking_count: Cell::new(0),
            request_log: RefCell::new(VecDeque::new()),
        });
        let (r, w) = (c.clone(), c.clone());
        let h = c.printer.reactor.register_fd(sock, Box::new(move |e| r.process_received(e)), Some(Box::new(move |_| w.do_send())));
        *c.fd_handle.borrow_mut() = Some(h);
        c.set_client_info(Some(&PyVal::Str("?".into())), Some("New connection"));
        c
    }

    fn dump_request_log(&self) {
        let log = self.request_log.borrow();
        let mut out = vec![format!("Dumping {} requests for client {}", log.len(), self.uid)];
        for (eventtime, request) in log.iter() {
            out.push(format!("Received {:.6}: {}", eventtime, py_bytes_repr(request)));
        }
        logging::info(&out.join("\n"));
    }

    pub fn set_client_info(&self, client_info: Option<&PyVal>, state_msg: Option<&str>) {
        let state_msg = match state_msg {
            Some(s) => s.to_string(),
            None => format!("Client info {}", client_info.map(|c| c.repr()).unwrap_or_else(|| "None".into())),
        };
        logging::info(&format!("webhooks client {}: {}", self.uid, state_msg));
        let log_id = format!("webhooks {}", self.uid);
        let Some(client_info) = client_info else {
            self.printer.set_rollover_info(&log_id, None, false);
            return;
        };
        let rollover_msg = format!("webhooks client {}: {}", self.uid, client_info.repr());
        self.printer.set_rollover_info(&log_id, Some(&rollover_msg), false);
    }

    pub fn close(&self) {
        let Some(h) = self.fd_handle.borrow_mut().take() else { return };
        self.set_client_info(None, Some("Disconnected"));
        self.printer.reactor.unregister_fd(&h);
        unsafe { libc::close(self.sock) };
        if let Some(s) = self.server.upgrade() {
            s.pop_client(self.uid);
        }
    }

    pub fn is_closed(&self) -> bool {
        self.fd_handle.borrow().is_none()
    }

    fn process_received(self: &Rc<Self>, eventtime: f64) {
        let mut buf = [0u8; 4096];
        let n = unsafe { libc::recv(self.sock, buf.as_mut_ptr() as *mut libc::c_void, buf.len(), 0) };
        let data: &[u8] = if n < 0 {
            // If bad file descriptor allow connection to be closed by the
            // data check
            if std::io::Error::last_os_error().raw_os_error() == Some(libc::EBADF) {
                &[]
            } else {
                return;
            }
        } else {
            &buf[..n as usize]
        };
        if data.is_empty() {
            // Socket Closed
            self.close();
            return;
        }
        let mut requests: Vec<Vec<u8>> = data.split(|b| *b == 0x03).map(|r| r.to_vec()).collect();
        let first = [self.partial_data.borrow().as_slice(), requests[0].as_slice()].concat();
        requests[0] = first;
        *self.partial_data.borrow_mut() = requests.pop().unwrap();
        for req in requests {
            {
                let mut log = self.request_log.borrow_mut();
                log.push_back((eventtime, req.clone()));
                if log.len() > REQUEST_LOG_SIZE {
                    log.pop_front();
                }
            }
            let web_request = std::str::from_utf8(&req)
                .map_err(|e| e.to_string())
                .and_then(|s| WebRequest::new(Some(self.clone()), s));
            let web_request = match web_request {
                Ok(w) => w,
                Err(_) => {
                    logging::exception(&format!("webhooks: Error decoding Server Request {}", py_bytes_repr(&req)));
                    continue;
                }
            };
            let s = self.clone();
            self.printer.reactor.register_callback(move |_| s.process_request(web_request), NOW);
        }
    }

    fn process_request(self: &Rc<Self>, web_request: WebRequest) {
        let web_request = Rc::new(web_request);
        let res = (|| -> KResult<()> {
            let wh = self.webhooks.upgrade().ok_or_else(|| KError::Other("webhooks gone".into()))?;
            let func = wh.get_callback(web_request.get_method())?;
            crate::running::run(&web_request, |w| func(w))
        })();
        if let Err(e) = res {
            if e.is_command_error() {
                web_request.set_error(&e.to_string());
            } else {
                let msg = format!("Internal Error on WebRequest: {}", web_request.get_method());
                logging::exception(&msg);
                web_request.set_error(&e.to_string());
                self.printer.invoke_shutdown(&msg, None);
            }
        }
        let Some(result) = web_request.finish() else { return };
        self.send(&result);
    }

    pub fn send(&self, data: &PyVal) {
        let jmsg = data.json_compact();
        {
            let mut buf = self.send_buffer.borrow_mut();
            buf.extend_from_slice(jmsg.as_bytes());
            buf.push(0x03);
        }
        if !self.is_blocking.get() {
            self.do_send();
        }
    }

    fn do_send(&self) {
        let Some(h) = self.fd_handle.borrow().clone() else { return };
        let buf = self.send_buffer.borrow().clone();
        let sent = unsafe { libc::send(self.sock, buf.as_ptr() as *const libc::c_void, buf.len(), libc::MSG_NOSIGNAL) };
        let sent = if sent < 0 {
            let err = std::io::Error::last_os_error().raw_os_error();
            if err != Some(libc::EAGAIN) && err != Some(libc::EWOULDBLOCK) {
                logging::info(&format!("webhooks: socket write error {}", self.uid));
                self.close();
                return;
            }
            0
        } else {
            sent as usize
        };
        if sent < buf.len() {
            if !self.is_blocking.get() {
                self.printer.reactor.set_fd_wake(&h, false, true);
                self.is_blocking.set(true);
                self.blocking_count.set(5);
            }
        } else if self.is_blocking.get() {
            self.printer.reactor.set_fd_wake(&h, true, false);
            self.is_blocking.set(false);
        }
        self.send_buffer.borrow_mut().drain(..sent);
    }
}

type MuxValues = IndexMap<Option<String>, Endpoint>;

pub struct WebHooks {
    printer: Rc<Printer>,
    endpoints: RefCell<IndexMap<String, Endpoint>>,
    remote_methods: RefCell<IndexMap<String, IndexMap<usize, (Rc<ClientConnection>, PyVal)>>>,
    mux_endpoints: RefCell<IndexMap<String, (String, MuxValues)>>,
    sconn: RefCell<Option<Rc<ServerSocket>>>,
}

impl PrinterObject for WebHooks {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let (state_message, state) = self.printer.get_state_message();
        let mut d = IndexMap::new();
        d.insert("state".to_string(), PyVal::Str(state.into()));
        d.insert("state_message".to_string(), PyVal::Str(state_message));
        Some(PyVal::Dict(d))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        self.sconn.borrow().as_ref().map(|s| s.stats(eventtime))
    }
    fn teardown(&self) {
        self.endpoints.borrow_mut().clear();
        self.mux_endpoints.borrow_mut().clear();
        self.remote_methods.borrow_mut().clear();
        if let Some(s) = self.sconn.borrow_mut().take() {
            s.clients.borrow_mut().clear();
            s.fd_handle.borrow_mut().take();
        }
    }
}

impl WebHooks {
    fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let wh = Rc::new(WebHooks {
            printer: printer.clone(),
            endpoints: RefCell::new(IndexMap::new()),
            remote_methods: RefCell::new(IndexMap::new()),
            mux_endpoints: RefCell::new(IndexMap::new()),
            sconn: RefCell::new(None),
        });
        let w = Rc::downgrade(&wh);
        wh.endpoints.borrow_mut().insert(
            "list_endpoints".into(),
            Rc::new(move |wr: &WebRequest| {
                let wh = w.upgrade().unwrap();
                let names: Vec<PyVal> = wh.endpoints.borrow().keys().map(|k| PyVal::Str(k.clone())).collect();
                let mut d = IndexMap::new();
                d.insert("endpoints".to_string(), PyVal::List(names));
                wr.send(PyVal::Dict(d))
            }),
        );
        let w = Rc::downgrade(&wh);
        wh.register_endpoint("info", Rc::new(move |wr| w.upgrade().unwrap().handle_info_request(wr)))?;
        let w = Rc::downgrade(&wh);
        wh.register_endpoint(
            "emergency_stop",
            Rc::new(move |_| {
                w.upgrade().unwrap().printer.invoke_shutdown("Shutdown due to webhooks request", None);
                Ok(())
            }),
        )?;
        let w = Rc::downgrade(&wh);
        wh.register_endpoint("register_remote_method", Rc::new(move |wr| w.upgrade().unwrap().handle_rpc_registration(wr)))?;
        let sconn = ServerSocket::new(&wh, printer)?;
        *wh.sconn.borrow_mut() = Some(sconn);
        Ok(wh)
    }

    pub fn register_endpoint(&self, path: &str, callback: Endpoint) -> KResult<()> {
        if self.endpoints.borrow().contains_key(path) {
            return Err(web_error("Path already registered to an endpoint"));
        }
        self.endpoints.borrow_mut().insert(path.to_string(), callback);
        Ok(())
    }

    /// register_mux_endpoint(path, key, value, callback); value None is
    /// python's None key
    pub fn register_mux_endpoint(self: &Rc<Self>, path: &str, key: &str, value: Option<&str>, callback: Endpoint) -> KResult<()> {
        if !self.mux_endpoints.borrow().contains_key(path) {
            let w = Rc::downgrade(self);
            self.register_endpoint(path, Rc::new(move |wr| w.upgrade().unwrap().handle_mux(wr)))?;
            self.mux_endpoints.borrow_mut().insert(path.to_string(), (key.to_string(), IndexMap::new()));
        }
        let mut mux = self.mux_endpoints.borrow_mut();
        let (prev_key, prev_values) = mux.get_mut(path).unwrap();
        let show = |v: Option<&str>| v.map(|s| s.to_string()).unwrap_or_else(|| "None".into());
        if prev_key != key {
            return Err(KError::config(format!("mux endpoint {} {} {} may have only one key ({})", path, key, show(value), prev_key)));
        }
        let k = value.map(|s| s.to_string());
        if prev_values.contains_key(&k) {
            let keys: Vec<String> = prev_values.keys().map(|k| k.as_deref().map(crate::pyfmt::py_str_repr).unwrap_or_else(|| "None".into())).collect();
            return Err(KError::config(format!("mux endpoint {} {} {} already registered ({{{}}})", path, key, show(value), keys.join(", "))));
        }
        prev_values.insert(k, callback);
        Ok(())
    }

    fn handle_mux(&self, web_request: &WebRequest) -> KResult<()> {
        let func = {
            let mux = self.mux_endpoints.borrow();
            let (key, values) = mux.get(web_request.get_method()).unwrap();
            let key_param = if values.contains_key(&None) {
                web_request.get(key, Some(PyVal::None), None)?
            } else {
                web_request.get(key, None, None)?
            };
            let lookup = match &key_param {
                PyVal::None => Some(None),
                PyVal::Str(s) => Some(Some(s.clone())),
                _ => None,
            };
            match lookup.and_then(|k| values.get(&k).cloned()) {
                Some(f) => f,
                None => return Err(web_error(format!("The value '{}' is not valid for {}", key_param.str(), key))),
            }
        };
        func(web_request)
    }

    fn handle_info_request(&self, web_request: &WebRequest) -> KResult<()> {
        let client_info = web_request.get_dict("client_info", Some(PyVal::None))?;
        if client_info != PyVal::None {
            if let Some(c) = web_request.get_client_connection() {
                c.set_client_info(Some(&client_info), None);
            }
        }
        let (state_message, state) = self.printer.get_state_message();
        let exe = std::env::current_exe().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
        let klipper_path = crate::pyglob::normpath(&crate::pyglob::join(&crate::pyglob::dirname(&exe), &[".."]));
        let mut hostname = [0u8; 256];
        unsafe { libc::gethostname(hostname.as_mut_ptr() as *mut libc::c_char, hostname.len()) };
        let hostname = String::from_utf8_lossy(&hostname[..hostname.iter().position(|b| *b == 0).unwrap_or(0)]).into_owned();
        let mut response = IndexMap::new();
        response.insert("app".to_string(), PyVal::Str(APP_NAME.into()));
        response.insert("state".to_string(), PyVal::Str(state.into()));
        response.insert("state_message".to_string(), PyVal::Str(state_message));
        response.insert("hostname".to_string(), PyVal::Str(hostname));
        response.insert("klipper_path".to_string(), PyVal::Str(klipper_path));
        response.insert("python_path".to_string(), PyVal::Str(exe));
        response.insert("process_id".to_string(), PyVal::Int(std::process::id() as i64));
        response.insert("user_id".to_string(), PyVal::Int(unsafe { libc::getuid() } as i64));
        response.insert("group_id".to_string(), PyVal::Int(unsafe { libc::getgid() } as i64));
        for sa in ["log_file", "config_file", "software_version", "cpu_info"] {
            response.insert(sa.to_string(), self.printer.start_arg(sa).unwrap_or(PyVal::None));
        }
        web_request.send(PyVal::Dict(response))
    }

    fn handle_rpc_registration(&self, web_request: &WebRequest) -> KResult<()> {
        let template = web_request.get_dict("response_template", None)?;
        let method = web_request.get_str("remote_method", None)?.str();
        let Some(new_conn) = web_request.get_client_connection() else { return Ok(()) };
        if crate::extras::danger_options::get_danger_options().log_webhook_method_register_messages {
            logging::info(&format!("webhooks: registering remote method '{}' for connection id: {}", method, new_conn.uid));
        }
        self.remote_methods.borrow_mut().entry(method).or_default().insert(new_conn.uid, (new_conn, template));
        Ok(())
    }

    pub fn get_connection(&self) -> Option<Rc<ServerSocket>> {
        self.sconn.borrow().clone()
    }

    /// the methods clients registered (python plugins read `_remote_methods`)
    pub fn remote_method_names(&self) -> Vec<String> {
        self.remote_methods.borrow().keys().cloned().collect()
    }

    pub fn get_callback(&self, path: &str) -> KResult<Endpoint> {
        match self.endpoints.borrow().get(path) {
            Some(cb) => Ok(cb.clone()),
            None => {
                let msg = format!("webhooks: No registered callback for path '{}'", path);
                logging::info(&msg);
                Err(web_error(msg))
            }
        }
    }

    /// call_remote_method(method, **kwargs)
    pub fn call_remote_method(&self, method: &str, kwargs: IndexMap<String, PyVal>) -> KResult<()> {
        let conn_map = self
            .remote_methods
            .borrow()
            .get(method)
            .cloned()
            .ok_or_else(|| KError::command(format!("Remote method '{}' not registered", method)))?;
        let mut valid_conns = IndexMap::new();
        for (uid, (conn, template)) in conn_map {
            if !conn.is_closed() {
                let mut out = IndexMap::new();
                out.insert("params".to_string(), PyVal::Dict(kwargs.clone()));
                if let PyVal::Dict(t) = &template {
                    for (k, v) in t {
                        out.insert(k.clone(), v.clone());
                    }
                }
                conn.send(&PyVal::Dict(out));
                valid_conns.insert(uid, (conn, template));
            }
        }
        if valid_conns.is_empty() {
            self.remote_methods.borrow_mut().shift_remove(method);
            return Err(KError::command(format!("No active connections for method '{}'", method)));
        }
        self.remote_methods.borrow_mut().insert(method.to_string(), valid_conns);
        Ok(())
    }
}

struct GCodeHelper {
    gcode: Rc<GCodeDispatch>,
    is_output_registered: Cell<bool>,
    clients: RefCell<IndexMap<usize, (Rc<ClientConnection>, PyVal)>>,
}

impl GCodeHelper {
    fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let gh = Rc::new(GCodeHelper {
            gcode: printer.lookup::<GCodeDispatch>("gcode")?,
            is_output_registered: Cell::new(false),
            clients: RefCell::new(IndexMap::new()),
        });
        // Register webhooks
        let wh = printer.lookup::<WebHooks>("webhooks")?;
        let g = gh.clone();
        wh.register_endpoint(
            "gcode/help",
            Rc::new(move |wr| {
                let help: IndexMap<String, PyVal> = g.gcode.get_command_help().into_iter().map(|(k, v)| (k, PyVal::Str(v))).collect();
                wr.send(PyVal::Dict(help))
            }),
        )?;
        let g = gh.clone();
        wh.register_endpoint("gcode/script", Rc::new(move |wr| g.gcode.run_script(&wr.get_str("script", None)?.str())))?;
        let g = gh.clone();
        wh.register_endpoint("gcode/restart", Rc::new(move |_| g.gcode.run_script("restart")))?;
        let g = gh.clone();
        wh.register_endpoint("gcode/firmware_restart", Rc::new(move |_| g.gcode.run_script("firmware_restart")))?;
        let g = gh.clone();
        wh.register_endpoint("gcode/subscribe_output", Rc::new(move |wr| g.handle_subscribe_output(wr)))?;
        Ok(gh)
    }

    fn output_callback(&self, msg: &str) {
        let clients: Vec<_> = self.clients.borrow().iter().map(|(k, v)| (*k, v.clone())).collect();
        for (uid, (cconn, template)) in clients {
            if cconn.is_closed() {
                self.clients.borrow_mut().shift_remove(&uid);
                continue;
            }
            let mut tmp = match template {
                PyVal::Dict(d) => d,
                _ => IndexMap::new(),
            };
            let mut params = IndexMap::new();
            params.insert("response".to_string(), PyVal::Str(msg.to_string()));
            tmp.insert("params".to_string(), PyVal::Dict(params));
            cconn.send(&PyVal::Dict(tmp));
        }
    }

    fn handle_subscribe_output(self: &Rc<Self>, web_request: &WebRequest) -> KResult<()> {
        let template = web_request.get_dict("response_template", Some(PyVal::dict()))?;
        if let Some(cconn) = web_request.get_client_connection() {
            self.clients.borrow_mut().insert(cconn.uid, (cconn, template));
        }
        if !self.is_output_registered.get() {
            let g = self.clone();
            let origin: crate::gcode::CallbackOrigin = (self.clone(), std::any::type_name::<GCodeHelper>());
            self.gcode.register_output_handler_from(Rc::new(move |msg| g.output_callback(msg)), origin);
            self.is_output_registered.set(true);
        }
        Ok(())
    }
}

pub const SUBSCRIPTION_REFRESH_TIME: f64 = 0.25;

/// a subscription dict (shared: python keeps the same dict in the pending
/// query and in the client subscription)
type Subscription = Rc<RefCell<IndexMap<String, PyVal>>>;

enum SendFunc {
    Complete(Rc<ReactorCompletion<PyVal>>),
    Conn(Rc<ClientConnection>),
}

struct QueryStatusHelper {
    printer: Rc<Printer>,
    clients: RefCell<IndexMap<usize, (Rc<ClientConnection>, Subscription, PyVal)>>,
    pending_queries: RefCell<Vec<(Option<Rc<ClientConnection>>, Subscription, SendFunc, PyVal)>>,
    query_timer: RefCell<Option<Timer>>,
    last_query: RefCell<IndexMap<String, PyVal>>,
}

impl QueryStatusHelper {
    fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let q = Rc::new(QueryStatusHelper {
            printer: printer.clone(),
            clients: RefCell::new(IndexMap::new()),
            pending_queries: RefCell::new(vec![]),
            query_timer: RefCell::new(None),
            last_query: RefCell::new(IndexMap::new()),
        });
        // Register webhooks
        let wh = printer.lookup::<WebHooks>("webhooks")?;
        let qq = q.clone();
        wh.register_endpoint(
            "objects/list",
            Rc::new(move |wr| {
                let objects: Vec<PyVal> = qq
                    .printer
                    .lookup_objects(None)
                    .into_iter()
                    .filter(|(_, o)| o.has_status())
                    .map(|(n, _)| PyVal::Str(n))
                    .collect();
                let mut d = IndexMap::new();
                d.insert("objects".to_string(), PyVal::List(objects));
                wr.send(PyVal::Dict(d))
            }),
        )?;
        let qq = q.clone();
        wh.register_endpoint("objects/query", Rc::new(move |wr| qq.handle_query(wr, false)))?;
        let qq = q.clone();
        wh.register_endpoint("objects/subscribe", Rc::new(move |wr| qq.handle_query(wr, true)))?;
        Ok(q)
    }

    fn do_query(self: &Rc<Self>, eventtime: f64) -> f64 {
        let last_query = std::mem::take(&mut *self.last_query.borrow_mut());
        let mut query: IndexMap<String, PyVal> = IndexMap::new();
        let mut msglist = std::mem::take(&mut *self.pending_queries.borrow_mut());
        for (_, (cconn, sub, template)) in self.clients.borrow().iter() {
            msglist.push((Some(cconn.clone()), sub.clone(), SendFunc::Conn(cconn.clone()), template.clone()));
        }
        // Generate get_status() info for each client
        for (cconn, subscription, send_func, template) in msglist {
            let is_query = cconn.is_none();
            if let Some(c) = &cconn {
                if c.is_closed() {
                    self.clients.borrow_mut().shift_remove(&c.uid);
                    continue;
                }
            }
            // Query each requested printer object
            let mut cquery = IndexMap::new();
            let names: Vec<String> = subscription.borrow().keys().cloned().collect();
            for obj_name in names {
                if !query.contains_key(&obj_name) {
                    let res = match self.printer.lookup_object_opt(&obj_name) {
                        Some(po) if po.has_status() => po.get_status(eventtime).unwrap_or_else(PyVal::dict),
                        _ => PyVal::dict(),
                    };
                    query.insert(obj_name.clone(), res);
                }
                let res = query[&obj_name].clone();
                let res_keys: Vec<String> = match &res {
                    PyVal::Dict(d) => d.keys().cloned().collect(),
                    _ => vec![],
                };
                let mut req_items = subscription.borrow()[&obj_name].clone();
                if req_items == PyVal::None {
                    req_items = PyVal::List(res_keys.iter().map(|k| PyVal::Str(k.clone())).collect());
                    if !res_keys.is_empty() {
                        subscription.borrow_mut().insert(obj_name.clone(), req_items.clone());
                    }
                }
                let empty = IndexMap::new();
                let lres = match last_query.get(&obj_name) {
                    Some(PyVal::Dict(d)) => d,
                    _ => &empty,
                };
                let mut cres = IndexMap::new();
                for ri in req_items.items().unwrap_or(&[]) {
                    let ri = ri.str();
                    let rd = match &res {
                        PyVal::Dict(d) => d.get(&ri).cloned().unwrap_or(PyVal::None),
                        _ => PyVal::None,
                    };
                    if is_query || !rd.py_eq(lres.get(&ri).unwrap_or(&PyVal::None)) {
                        cres.insert(ri, rd);
                    }
                }
                if !cres.is_empty() || is_query {
                    cquery.insert(obj_name.clone(), PyVal::Dict(cres));
                }
            }
            // Send data
            if !cquery.is_empty() || is_query {
                let mut tmp = match template {
                    PyVal::Dict(d) => d,
                    _ => IndexMap::new(),
                };
                let mut params = IndexMap::new();
                params.insert("eventtime".to_string(), PyVal::Float(eventtime));
                params.insert("status".to_string(), PyVal::Dict(cquery));
                tmp.insert("params".to_string(), PyVal::Dict(params));
                let msg = PyVal::Dict(tmp);
                match send_func {
                    SendFunc::Complete(c) => c.complete(msg),
                    SendFunc::Conn(c) => c.send(&msg),
                }
            }
        }
        let empty = query.is_empty();
        *self.last_query.borrow_mut() = query;
        if empty {
            // Unregister timer if there are no longer any subscriptions
            if let Some(t) = self.query_timer.borrow_mut().take() {
                self.printer.reactor.unregister_timer(&t);
            }
            return NEVER;
        }
        eventtime + SUBSCRIPTION_REFRESH_TIME
    }

    fn handle_query(self: &Rc<Self>, web_request: &WebRequest, is_subscribe: bool) -> KResult<()> {
        let PyVal::Dict(objects) = web_request.get_dict("objects", None)? else { unreachable!() };
        // Validate subscription format
        for v in objects.values() {
            match v {
                PyVal::None => {}
                PyVal::List(items) => {
                    if items.iter().any(|ri| !matches!(ri, PyVal::Str(_))) {
                        return Err(web_error("Invalid argument"));
                    }
                }
                _ => return Err(web_error("Invalid argument")),
            }
        }
        let objects: Subscription = Rc::new(RefCell::new(objects));
        // Add to pending queries
        let cconn = web_request.get_client_connection();
        let template = web_request.get_dict("response_template", Some(PyVal::dict()))?;
        if is_subscribe {
            if let Some(c) = &cconn {
                self.clients.borrow_mut().shift_remove(&c.uid);
            }
        }
        let reactor = self.printer.get_reactor();
        let complete = reactor.completion::<PyVal>();
        self.pending_queries.borrow_mut().push((None, objects.clone(), SendFunc::Complete(complete.clone()), PyVal::dict()));
        // Start timer if needed
        if self.query_timer.borrow().is_none() {
            let q = self.clone();
            let qt = reactor.register_timer(Box::new(move |e| q.do_query(e)), NOW);
            *self.query_timer.borrow_mut() = Some(qt);
        }
        // Wait for data to be queried
        let msg = complete.wait(NEVER, None)?.unwrap_or(PyVal::None);
        let params = match &msg {
            PyVal::Dict(d) => d.get("params").cloned().unwrap_or(PyVal::None),
            _ => PyVal::None,
        };
        web_request.send(params)?;
        if is_subscribe {
            if let Some(c) = cconn {
                self.clients.borrow_mut().insert(c.uid, (c, objects, template));
            }
        }
        Ok(())
    }
}

pub fn add_early_printer_objects(printer: &Rc<Printer>) -> KResult<()> {
    printer.add_object("webhooks", WebHooks::new(printer)?)?;
    GCodeHelper::new(printer)?;
    QueryStatusHelper::new(printer)?;
    Ok(())
}
