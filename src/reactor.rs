// File descriptor and timer event helper (port of klippy/reactor.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Ported: SelectReactor's timer / greenlet / completion / mutex logic with
// PollReactor's fd handling (what `Reactor` is on Linux). Greenlets are
// crate::greenlet (corosensei). Python's gc checking has no rust equivalent:
// get_gc_stats() stays (0, 0, 0).
// Not ported yet: async_complete() (cross-thread completion; first user is
// serialhdl).
use crate::chelper::pyhelper;
use crate::greenlet::{self, Greenlet, Target, G};
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::os::fd::RawFd;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

pub const NOW: f64 = 0.0;
pub const NEVER: f64 = 9999999999999999.0;

/// ReactorError
#[derive(Clone, Debug)]
pub struct ReactorError(pub String);

impl std::fmt::Display for ReactorError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

pub type TimerCallback = Box<dyn FnMut(f64) -> f64>;

pub struct ReactorTimer {
    callback: RefCell<TimerCallback>,
    pub waketime: Cell<f64>,
    pub timer_is_running: Cell<bool>,
}

pub type Timer = Rc<ReactorTimer>;

pub struct ReactorCompletion<T: Clone> {
    reactor: Rc<Reactor>,
    result: RefCell<Option<T>>,
    waiting: RefCell<Vec<Option<Rc<ReactorGreenlet>>>>,
}

impl<T: Clone> ReactorCompletion<T> {
    pub fn test(&self) -> bool {
        self.result.borrow().is_some()
    }

    pub fn complete(&self, result: T) {
        *self.result.borrow_mut() = Some(result);
        let waiting = self.waiting.borrow().clone();
        for wait in waiting.iter().flatten() {
            if let Some(t) = wait.timer.borrow().clone() {
                self.reactor.update_timer(&t, NOW);
            }
        }
    }

    /// wait(waketime=NEVER, waketime_result=None)
    pub fn wait(&self, waketime: f64, waketime_result: Option<T>) -> Result<Option<T>, ReactorError> {
        if self.result.borrow().is_none() {
            let wait = self.reactor.current_greenlet();
            self.waiting.borrow_mut().push(wait.clone());
            self.reactor.pause(waketime)?;
            let mut waiting = self.waiting.borrow_mut();
            // list.remove(): the first entry that is this greenlet
            if let Some(pos) = waiting.iter().position(|w| match (w, &wait) {
                (Some(a), Some(b)) => Rc::ptr_eq(a, b),
                (None, None) => true,
                _ => false,
            }) {
                waiting.remove(pos);
            }
            if self.result.borrow().is_none() {
                return Ok(waketime_result);
            }
        }
        Ok(self.result.borrow().clone())
    }
}

pub struct ReactorFileHandler {
    pub fd: RawFd,
    // Rc so a callback can run re-entrantly (a pause inside it lets
    // another greenlet dispatch the same fd, as python allows)
    read_callback: Rc<dyn Fn(f64)>,
    write_callback: Option<Rc<dyn Fn(f64)>>,
}

pub type FileHandler = Rc<ReactorFileHandler>;

pub struct ReactorGreenlet {
    pub g: G,
    pub timer: RefCell<Option<Timer>>,
}

pub struct ReactorMutex {
    reactor: Rc<Reactor>,
    is_locked: Cell<bool>,
    next_pending: Cell<bool>,
    queue: RefCell<Vec<Option<Rc<ReactorGreenlet>>>>,
}

fn same(a: &Option<Rc<ReactorGreenlet>>, b: &Option<Rc<ReactorGreenlet>>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => Rc::ptr_eq(a, b),
        (None, None) => true,
        _ => false,
    }
}

impl ReactorMutex {
    pub fn test(&self) -> bool {
        self.is_locked.get()
    }

    /// __enter__ / lock()
    pub fn lock(&self) -> Result<(), ReactorError> {
        if !self.is_locked.get() {
            self.is_locked.set(true);
            return Ok(());
        }
        let g = self.reactor.current_greenlet();
        self.queue.borrow_mut().push(g.clone());
        loop {
            self.reactor.pause(NEVER)?;
            if self.next_pending.get() && same(&self.queue.borrow()[0], &g) {
                self.next_pending.set(false);
                self.queue.borrow_mut().remove(0);
                return Ok(());
            }
        }
    }

    /// __exit__ / unlock()
    pub fn unlock(&self) {
        let first = self.queue.borrow().first().cloned();
        let Some(first) = first else {
            self.is_locked.set(false);
            return;
        };
        self.next_pending.set(true);
        if let Some(t) = first.and_then(|g| g.timer.borrow().clone()) {
            self.reactor.update_timer(&t, NOW);
        }
    }
}

/// assert_no_pause(): pauses are disallowed while this lives
pub struct ReactorPreventPause {
    reactor: Rc<Reactor>,
}

impl Drop for ReactorPreventPause {
    fn drop(&mut self) {
        let c = &self.reactor.prevent_pause_count;
        c.set(c.get() - 1);
    }
}

/// A callback queued from another thread; runs on the reactor thread
pub type AsyncCallback = Box<dyn FnOnce(f64) + Send>;

/// The thread-safe part of the reactor (register_async_callback)
#[derive(Clone)]
pub struct AsyncHandle {
    queue: Arc<Mutex<VecDeque<(AsyncCallback, f64)>>>,
    pipe_w: Arc<Mutex<Option<RawFd>>>,
}

impl AsyncHandle {
    /// register_async_callback(callback, waketime)
    pub fn register_async_callback(&self, callback: AsyncCallback, waketime: f64) {
        self.queue.lock().unwrap().push_back((callback, waketime));
        if let Some(fd) = *self.pipe_w.lock().unwrap() {
            // errors (eg EAGAIN with a full pipe) are ignored like python
            unsafe { libc::write(fd, b".".as_ptr() as *const libc::c_void, 1) };
        }
    }
}

pub struct Reactor {
    process: Cell<bool>,
    // Timers
    timers: RefCell<Vec<Timer>>,
    next_timer: Cell<f64>,
    // Callbacks
    pipe_fds: Cell<Option<(RawFd, RawFd)>>,
    async_handle: AsyncHandle,
    // File descriptors (insertion ordered like CPython's poll object)
    dummy_fd_hdl: FileHandler,
    fds: RefCell<IndexMap<RawFd, FileHandler>>,
    poll_fds: RefCell<IndexMap<RawFd, i16>>,
    read_mask: i16,
    write_mask: i16,
    // Greenlets
    g_dispatch: RefCell<Option<Rc<ReactorGreenlet>>>,
    greenlets: RefCell<Vec<Rc<ReactorGreenlet>>>,
    all_greenlets: RefCell<Vec<Rc<ReactorGreenlet>>>,
    prevent_pause_count: Cell<i32>,
}

impl Reactor {
    /// Reactor(gc_checking=...) (gc checking has no rust equivalent)
    pub fn new(_gc_checking: bool) -> Rc<Self> {
        Rc::new(Reactor {
            process: Cell::new(false),
            timers: RefCell::new(vec![]),
            next_timer: Cell::new(NEVER),
            pipe_fds: Cell::new(None),
            async_handle: AsyncHandle {
                queue: Arc::new(Mutex::new(VecDeque::new())),
                pipe_w: Arc::new(Mutex::new(None)),
            },
            dummy_fd_hdl: Rc::new(ReactorFileHandler {
                fd: -1,
                read_callback: Rc::new(|_| {}),
                write_callback: Some(Rc::new(|_| {})),
            }),
            fds: RefCell::new(IndexMap::new()),
            poll_fds: RefCell::new(IndexMap::new()),
            read_mask: libc::POLLIN | libc::POLLHUP,
            write_mask: libc::POLLOUT,
            g_dispatch: RefCell::new(None),
            greenlets: RefCell::new(vec![]),
            all_greenlets: RefCell::new(vec![]),
            prevent_pause_count: Cell::new(0),
        })
    }

    pub fn monotonic(&self) -> f64 {
        pyhelper::get_monotonic()
    }

    pub fn get_gc_stats(&self) -> (f64, f64, f64) {
        (0., 0., 0.)
    }

    /// len(_all_greenlets), for tests
    pub fn greenlet_count(&self) -> usize {
        self.all_greenlets.borrow().len()
    }

    // Timers
    pub fn update_timer(&self, timer_handler: &Timer, waketime: f64) {
        if timer_handler.timer_is_running.get() {
            return;
        }
        timer_handler.waketime.set(waketime);
        self.next_timer.set(self.next_timer.get().min(waketime));
    }

    pub fn register_timer(&self, callback: TimerCallback, waketime: f64) -> Timer {
        let timer_handler = Rc::new(ReactorTimer {
            callback: RefCell::new(callback),
            waketime: Cell::new(waketime),
            timer_is_running: Cell::new(false),
        });
        // python replaces the list (a running _check_timers loop keeps
        // iterating its old copy)
        let mut timers = self.timers.borrow().clone();
        timers.push(timer_handler.clone());
        *self.timers.borrow_mut() = timers;
        self.next_timer.set(self.next_timer.get().min(waketime));
        timer_handler
    }

    pub fn unregister_timer(&self, timer_handler: &Timer) {
        timer_handler.waketime.set(NEVER);
        let mut timers = self.timers.borrow().clone();
        // python: timers.pop(timers.index(timer_handler)) (ValueError if absent)
        let pos = timers.iter().position(|t| Rc::ptr_eq(t, timer_handler)).expect("unregister_timer: unknown timer");
        timers.remove(pos);
        *self.timers.borrow_mut() = timers;
    }

    fn check_timers(&self, eventtime: f64, busy: bool) -> f64 {
        if eventtime < self.next_timer.get() {
            if busy {
                return 0.0;
            }
            return 1.0f64.min(0.001f64.max(self.next_timer.get() - eventtime));
        }
        self.next_timer.set(NEVER);
        let g_dispatch = self.g_dispatch.borrow().clone();
        let timers = self.timers.borrow().clone();
        for t in timers.iter() {
            let mut waketime = t.waketime.get();
            if eventtime >= waketime {
                t.waketime.set(NEVER);
                t.timer_is_running.set(true);
                waketime = (t.callback.borrow_mut())(eventtime);
                t.waketime.set(waketime);
                t.timer_is_running.set(false);
                if !same(&g_dispatch, &self.g_dispatch.borrow()) {
                    self.next_timer.set(self.next_timer.get().min(waketime));
                    self.end_greenlet(g_dispatch.unwrap());
                    return 0.0;
                }
            }
            self.next_timer.set(self.next_timer.get().min(waketime));
        }
        0.0
    }

    // Callbacks and Completions
    pub fn completion<T: Clone + 'static>(self: &Rc<Self>) -> Rc<ReactorCompletion<T>> {
        Rc::new(ReactorCompletion { reactor: self.clone(), result: RefCell::new(None), waiting: RefCell::new(vec![]) })
    }

    /// register_callback(callback, waketime): runs callback(eventtime) from
    /// a timer; the returned completion gets its result
    pub fn register_callback<T: Clone + 'static>(
        self: &Rc<Self>,
        callback: impl FnOnce(f64) -> T + 'static,
        waketime: f64,
    ) -> Rc<ReactorCompletion<T>> {
        // ReactorCallback: the timer slot is emptied on invoke, which also
        // breaks the timer -> closure -> timer cycle
        let completion = self.completion::<T>();
        let timer_slot: Rc<RefCell<Option<Timer>>> = Rc::new(RefCell::new(None));
        let (reactor, slot, comp) = (self.clone(), timer_slot.clone(), completion.clone());
        let mut callback = Some(callback);
        let timer = self.register_timer(
            Box::new(move |eventtime| {
                if let Some(t) = slot.borrow_mut().take() {
                    reactor.unregister_timer(&t);
                }
                if let Some(cb) = callback.take() {
                    let res = cb(eventtime);
                    comp.complete(res);
                }
                NEVER
            }),
            waketime,
        );
        *timer_slot.borrow_mut() = Some(timer);
        completion
    }

    /// the handle other threads use for register_async_callback()
    pub fn async_handle(&self) -> AsyncHandle {
        self.async_handle.clone()
    }

    pub fn register_async_callback(&self, callback: AsyncCallback, waketime: f64) {
        self.async_handle.register_async_callback(callback, waketime)
    }

    /// async_complete(completion, result). Kaliru calls it only from the
    /// reactor thread (serial responses are dispatched there), but it keeps
    /// python's ordering by going through the same async queue.
    pub fn async_complete<T: Clone + 'static>(&self, completion: &Rc<ReactorCompletion<T>>, result: T) {
        let completion = completion.clone();
        self.register_async_callback_local(move |_| completion.complete(result), NOW);
    }

    /// register_async_callback() for a callback queued on the reactor
    /// thread itself (it may hold Rc state)
    pub fn register_async_callback_local(&self, callback: impl FnOnce(f64) + 'static, waketime: f64) {
        struct ReactorThreadOnly<F>(F);
        // SAFETY: queued and run on the reactor thread only
        unsafe impl<F> Send for ReactorThreadOnly<F> {}
        let cb = ReactorThreadOnly(callback);
        self.register_async_callback(
            Box::new(move |e| {
                let cb = cb;
                (cb.0)(e)
            }),
            waketime,
        );
    }

    fn got_pipe_signal(self: &Rc<Self>, _eventtime: f64) {
        if let Some((r, _)) = self.pipe_fds.get() {
            let mut buf = [0u8; 4096];
            unsafe { libc::read(r, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        }
        loop {
            let item = self.async_handle.queue.lock().unwrap().pop_front();
            let Some((callback, waketime)) = item else { break };
            // ReactorCallback(self, callback, waketime)
            self.register_callback(move |e| callback(e), waketime);
        }
    }

    fn setup_async_callbacks(self: &Rc<Self>) {
        let mut fds = [0 as RawFd; 2];
        unsafe { libc::pipe(fds.as_mut_ptr()) };
        crate::util::set_nonblock(fds[0]);
        crate::util::set_nonblock(fds[1]);
        self.pipe_fds.set(Some((fds[0], fds[1])));
        *self.async_handle.pipe_w.lock().unwrap() = Some(fds[1]);
        let r = self.clone();
        self.register_fd(fds[0], Box::new(move |e| r.got_pipe_signal(e)), None);
    }

    // Greenlets
    fn sys_pause(&self, waketime: f64) -> f64 {
        // Pause using system sleep for when reactor not running
        let delay = waketime - self.monotonic();
        if delay > 0.0 {
            std::thread::sleep(std::time::Duration::from_secs_f64(delay));
        }
        self.monotonic()
    }

    /// the ReactorGreenlet running now (None: the main context)
    pub fn current_greenlet(&self) -> Option<Rc<ReactorGreenlet>> {
        self.all_greenlets.borrow().iter().find(|rg| greenlet::is_current(&rg.g)).cloned()
    }

    pub fn pause(&self, waketime: f64) -> Result<f64, ReactorError> {
        let g_dispatch = self.g_dispatch.borrow().clone();
        let Some(g_dispatch) = g_dispatch else {
            // The reactor is not running - use a system pause instead
            return Ok(self.sys_pause(waketime));
        };
        if self.prevent_pause_count.get() != 0 {
            self.verify_can_pause()?;
        }
        // Determine if this greenlet is the main dispatch greenlet
        let g = self.current_greenlet();
        if !same(&g, &Some(g_dispatch.clone())) {
            // This greenlet has called pause() before and has a timer setup,
            // so switch to _check_timers (via g.timer.callback return)
            return Ok(greenlet::switch(Target::G(g_dispatch.g.clone()), waketime));
        }
        // Pausing the dispatch greenlet - setup timer to resume this greenlet
        let gg = g_dispatch.g.clone();
        let timer = self.register_timer(Box::new(move |e| greenlet::switch(Target::G(gg.clone()), e)), waketime);
        *g_dispatch.timer.borrow_mut() = Some(timer);
        self.next_timer.set(NOW);
        let g_next = self.greenlets.borrow_mut().pop();
        let eventtime = match g_next {
            // Switch to _end_greenlet to activate cached dispatch greenlet
            Some(g_next) => greenlet::switch(Target::G(g_next.g.clone()), f64::NAN),
            // No cached greenlets, switch to run() to create new dispatcher
            None => greenlet::switch(Target::Main, f64::NAN),
        };
        // This greenlet activated from g.timer.callback (via _check_timers)
        Ok(eventtime)
    }

    pub fn delay_for_ms(&self, milliseconds: f64) -> Result<f64, ReactorError> {
        self.pause(self.monotonic() + milliseconds / 1000.0)
    }

    fn end_greenlet(&self, g_old: Rc<ReactorGreenlet>) {
        // Cache this greenlet for later use
        self.greenlets.borrow_mut().push(g_old.clone());
        let timer = g_old.timer.borrow_mut().take();
        if let Some(t) = timer {
            self.unregister_timer(&t);
        }
        // Switch to _check_timers (via g_old.timer.callback return)
        let g_dispatch = self.g_dispatch.borrow().clone().unwrap();
        greenlet::switch(Target::G(g_dispatch.g.clone()), NEVER);
        // This greenlet reactivated from pause() - return to main dispatch loop
        *self.g_dispatch.borrow_mut() = Some(g_old);
    }

    // Support for temporarily disabling pauses
    pub fn assert_no_pause(self: &Rc<Self>) -> ReactorPreventPause {
        self.prevent_pause_count.set(self.prevent_pause_count.get() + 1);
        ReactorPreventPause { reactor: self.clone() }
    }

    pub fn verify_can_pause(&self) -> Result<(), ReactorError> {
        if self.prevent_pause_count.get() != 0 {
            return Err(ReactorError("Internal error - reactor pause disabled".into()));
        }
        Ok(())
    }

    // Mutexes
    pub fn mutex(self: &Rc<Self>, is_locked: bool) -> Rc<ReactorMutex> {
        Rc::new(ReactorMutex {
            reactor: self.clone(),
            is_locked: Cell::new(is_locked),
            next_pending: Cell::new(false),
            queue: RefCell::new(vec![]),
        })
    }

    // File descriptors (PollReactor)
    pub fn register_fd(&self, fd: RawFd, read_callback: Box<dyn Fn(f64)>, write_callback: Option<Box<dyn Fn(f64)>>) -> FileHandler {
        let file_handler = Rc::new(ReactorFileHandler {
            fd,
            read_callback: Rc::from(read_callback),
            write_callback: write_callback.map(Rc::from),
        });
        self.fds.borrow_mut().insert(fd, file_handler.clone());
        self.poll_fds.borrow_mut().insert(fd, libc::POLLIN | libc::POLLHUP);
        file_handler
    }

    pub fn unregister_fd(&self, file_handler: &FileHandler) {
        self.poll_fds.borrow_mut().shift_remove(&file_handler.fd);
        self.fds.borrow_mut().shift_remove(&file_handler.fd);
    }

    pub fn set_fd_wake(&self, file_handler: &FileHandler, is_readable: bool, is_writeable: bool) {
        let mut flags = libc::POLLHUP;
        if is_readable {
            flags |= libc::POLLIN;
        }
        if is_writeable {
            flags |= libc::POLLOUT;
        }
        // poll.modify() keeps the fd's position
        if let Some(f) = self.poll_fds.borrow_mut().get_mut(&file_handler.fd) {
            *f = flags;
        }
    }

    fn check_fds(&self, eventtime: f64, hdls: &[(RawFd, i16)]) -> f64 {
        let g_dispatch = self.g_dispatch.borrow().clone();
        for &(fd, event) in hdls {
            let hdl = self.fds.borrow().get(&fd).cloned().unwrap_or_else(|| self.dummy_fd_hdl.clone());
            if event & self.read_mask != 0 {
                let cb = hdl.read_callback.clone();
                cb(eventtime);
                if !same(&g_dispatch, &self.g_dispatch.borrow()) {
                    self.end_greenlet(g_dispatch.unwrap());
                    return self.monotonic();
                }
            }
            if event & self.write_mask != 0 {
                if let Some(cb) = hdl.write_callback.clone() {
                    cb(eventtime);
                }
                if !same(&g_dispatch, &self.g_dispatch.borrow()) {
                    self.end_greenlet(g_dispatch.unwrap());
                    return self.monotonic();
                }
            }
        }
        eventtime
    }

    fn poll(&self, timeout_ms: i32) -> Vec<(RawFd, i16)> {
        let mut pfds: Vec<libc::pollfd> = self
            .poll_fds
            .borrow()
            .iter()
            .map(|(&fd, &events)| libc::pollfd { fd, events, revents: 0 })
            .collect();
        // python plugins: other python threads run while the reactor waits
        #[cfg(feature = "plugins")]
        let n = crate::plugins::allow_threads(|| unsafe { libc::poll(pfds.as_mut_ptr(), pfds.len() as libc::nfds_t, timeout_ms) });
        #[cfg(not(feature = "plugins"))]
        let n = unsafe { libc::poll(pfds.as_mut_ptr(), pfds.len() as libc::nfds_t, timeout_ms) };
        if n <= 0 {
            // ponytail: python raises on poll errors other than EINTR (which it retries)
            return vec![];
        }
        pfds.iter().filter(|p| p.revents != 0).map(|p| (p.fd, p.revents)).collect()
    }

    // Main loop
    fn dispatch_loop(&self) {
        let mut busy = true;
        let mut eventtime = self.monotonic();
        while self.process.get() {
            let timeout = self.check_timers(eventtime, busy);
            busy = false;
            let res = self.poll((timeout * 1000.0).ceil() as i32);
            eventtime = self.monotonic();
            if !res.is_empty() {
                busy = true;
                eventtime = self.check_fds(eventtime, &res);
            }
        }
    }

    pub fn run(self: &Rc<Self>) {
        if self.pipe_fds.get().is_none() {
            self.setup_async_callbacks();
        }
        self.process.set(true);
        self.prevent_pause_count.set(0);
        while self.process.get() {
            // Create new greenlet to dispatch timers and events
            let r = self.clone();
            let g_next = Rc::new(ReactorGreenlet { g: Greenlet::new(move || r.dispatch_loop()), timer: RefCell::new(None) });
            self.all_greenlets.borrow_mut().push(g_next.clone());
            *self.g_dispatch.borrow_mut() = Some(g_next.clone());
            greenlet::switch(Target::G(g_next.g.clone()), f64::NAN);
            // Control returns here on end() request or switch from pause()
        }
        *self.g_dispatch.borrow_mut() = None;
    }

    pub fn end(&self) {
        self.process.set(false);
    }

    pub fn finalize(&self) {
        *self.g_dispatch.borrow_mut() = None;
        self.greenlets.borrow_mut().clear();
        let all: Vec<_> = self.all_greenlets.borrow_mut().drain(..).collect();
        for g in all {
            // g.throw(): unwinds a suspended greenlet; dead ones are a no-op
            g.g.kill();
            g.timer.borrow_mut().take();
        }
        if let Some((r, w)) = self.pipe_fds.take() {
            *self.async_handle.pipe_w.lock().unwrap() = None;
            unsafe {
                libc::close(r);
                libc::close(w);
            }
        }
        // not in python (the reactor is discarded after finalize): the timer and
        // fd closures hold Rc<Reactor>, clearing them breaks those cycles
        self.timers.borrow_mut().clear();
        self.fds.borrow_mut().clear();
        self.poll_fds.borrow_mut().clear();
    }
}
