// klippy/reactor.py: PollReactor (the class Kalico's `reactor.Reactor` is on
// Linux), ReactorTimer, ReactorCompletion, ReactorMutex, ReactorFileHandler.
// Completions python makes carry python objects (ReactorCompletion<PyRef>).
use super::*;
use crate::method;
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;
use crate::plugins::native::Args;
use crate::reactor::{FileHandler, Reactor, ReactorCompletion, ReactorMutex, ReactorTimer, NEVER};

type PyCompletion = ReactorCompletion<PyRef>;

pub fn register(r: &mut Registry) {
    r.method("klippy.reactor.SelectReactor.register_timer", register_timer);
    method!(r, "klippy.reactor.SelectReactor", Reactor, update_timer(re, t: Rc<ReactorTimer>, waketime: f64) => re.update_timer(&t, waketime));
    method!(r, "klippy.reactor.SelectReactor", Reactor, unregister_timer(re, t: Rc<ReactorTimer>) => re.unregister_timer(&t));
    method!(r, "klippy.reactor.SelectReactor", Reactor, pause(re, waketime: f64) => re.pause(waketime).map_err(|e| KError::Reactor(e.0)));
    method!(r, "klippy.reactor.SelectReactor", Reactor, delay_for_ms(re, ms: f64) => re.delay_for_ms(ms).map_err(|e| KError::Reactor(e.0)));
    method!(r, "klippy.reactor.SelectReactor", Reactor, get_gc_stats(re) => re.get_gc_stats());
    method!(r, "klippy.reactor.SelectReactor", Reactor, completion(re) => re.completion::<PyRef>());
    r.method("klippy.reactor.SelectReactor.register_callback", register_callback);
    r.method("klippy.reactor.SelectReactor.register_async_callback", register_async_callback);
    method!(r, "klippy.reactor.SelectReactor", Reactor, async_complete(re, c: Rc<PyCompletion>, result: PyRef) => re.async_complete(&c, result));
    method!(r, "klippy.reactor.SelectReactor", Reactor, mutex(re, is_locked: bool) => re.mutex(is_locked));
    r.method("klippy.reactor.SelectReactor.register_fd", register_fd);
    method!(r, "klippy.reactor.SelectReactor", Reactor, unregister_fd(re, h: Rc<crate::reactor::ReactorFileHandler>) => re.unregister_fd(&h));
    method!(r, "klippy.reactor.SelectReactor", Reactor, set_fd_wake(re, h: Rc<crate::reactor::ReactorFileHandler>, readable: bool, writeable: bool) =>
        re.set_fd_wake(&h, readable, writeable));
    method!(r, "klippy.reactor.SelectReactor", Reactor, end(re) => re.end());
    // self.monotonic = chelper...get_monotonic (a function attribute)
    r.attr("klippy.reactor.SelectReactor.monotonic", |o| {
        let re = objects::unwrap_as::<Reactor>(o)?;
        py_function(Rc::new(move |_args: &[PyRef]| Ok(py_float(re.monotonic()))))
    }, None);

    r.attr("klippy.reactor.ReactorTimer.waketime", |o| Ok(py_float(objects::unwrap_as::<ReactorTimer>(o)?.waketime.get())), None);

    method!(r, "klippy.reactor.ReactorCompletion", PyCompletion, test(c) => c.test());
    method!(r, "klippy.reactor.ReactorCompletion", PyCompletion, complete(c, result: PyRef) => c.complete(result));
    method!(r, "klippy.reactor.ReactorCompletion", PyCompletion, wait(c, waketime: f64, waketime_result: PyRef) =>
        c.wait(waketime, Some(waketime_result)).map_err(|e| KError::Reactor(e.0)));

    method!(r, "klippy.reactor.ReactorMutex", ReactorMutex, test(m) => m.test());
    method!(r, "klippy.reactor.ReactorMutex", ReactorMutex, __enter__(m) => m.lock().map_err(|e| KError::Reactor(e.0)));
    method!(r, "klippy.reactor.ReactorMutex", ReactorMutex, __exit__(m, _t: PyRef, _v: PyRef, _tb: PyRef) => m.unlock());
    // self.lock = self.__enter__; self.unlock = self.__exit__
    r.attr("klippy.reactor.ReactorMutex.lock", |o| {
        let m = objects::unwrap_as::<ReactorMutex>(o)?;
        py_function(Rc::new(move |_a: &[PyRef]| m.lock().map_err(|e| KError::Reactor(e.0)).and_then(|_| Ok(py_none()))))
    }, None);
    r.attr("klippy.reactor.ReactorMutex.unlock", |o| {
        let m = objects::unwrap_as::<ReactorMutex>(o)?;
        py_function(Rc::new(move |_a: &[PyRef]| {
            m.unlock();
            Ok(py_none())
        }))
    }, None);

    r.attr("klippy.reactor.ReactorFileHandler.fd", |o| Ok(py_int(objects::unwrap_as::<crate::reactor::ReactorFileHandler>(o)?.fd as i64)), None);
}

/// A python callable called with eventtime; a raise is reactor.run()'s
/// "Unhandled exception during run"
fn call_reporting(cb: &PyRef, eventtime: f64) -> Option<PyRef> {
    match cb.call(&[&py_float(eventtime)]) {
        Ok(v) => Some(v),
        Err(e) => {
            report_unhandled(&e);
            None
        }
    }
}

// register_timer(callback, waketime=NEVER)
fn register_timer(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = a.arg(0).clone();
    let timer = reactor.register_timer(
        Box::new(move |eventtime| match call_reporting(&cb, eventtime).map(|r| r.as_f64()) {
            Some(Ok(w)) => w,
            Some(Err(e)) => {
                report_unhandled(&e);
                NEVER
            }
            // python leaves reactor.run(); the timer does not run again
            None => NEVER,
        }),
        a.f64(1)?,
    );
    timer.to_py()
}

// register_callback(callback, waketime=NOW) -> completion of its result
fn register_callback(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = a.arg(0).clone();
    let completion: Rc<PyCompletion> = reactor.register_callback(
        move |eventtime| call_reporting(&cb, eventtime).unwrap_or_else(py_none),
        a.f64(1)?,
    );
    completion.to_py()
}

/// A python object carried to the reactor thread (it runs there, holding
/// the GIL: docs/PLUGINS.md "GIL and threads")
struct ToReactor(PyRef);
// SAFETY: created and dropped with the GIL held; used on the reactor thread
unsafe impl Send for ToReactor {}

// register_async_callback(callback, waketime=NOW): from any thread
fn register_async_callback(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = ToReactor(a.arg(0).clone());
    reactor.register_async_callback(
        Box::new(move |eventtime| {
            let cb = cb;
            call_reporting(&cb.0, eventtime);
        }),
        a.f64(1)?,
    );
    Ok(py_none())
}

// register_fd(fd, read_callback, write_callback=None)
fn register_fd(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let fd = a.arg(0).as_i64()? as std::os::fd::RawFd;
    let read = a.arg(1).clone();
    let write = if a.arg(2).is_none() { None } else { Some(a.arg(2).clone()) };
    let h: FileHandler = reactor.register_fd(
        fd,
        Box::new(move |et| {
            call_reporting(&read, et);
        }),
        write.map(|w| -> Box<dyn Fn(f64)> {
            Box::new(move |et| {
                call_reporting(&w, et);
            })
        }),
    );
    h.to_py()
}
