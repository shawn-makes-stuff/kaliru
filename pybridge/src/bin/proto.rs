// Prototype check for docs/PLUGINS.md step 1: python frames live on two
// kaliru greenlets at once, switched from inside python; per-greenlet
// exception state; C stack protection on greenlet stacks; a python thread
// that only runs while the reactor thread has released the GIL.
//
// usage: proto [noswap]   (noswap: no switch hook, the control run)
use kaliru::greenlet::{switch, Greenlet, Target, G};
use kaliru_pybridge::ffi::*;
use kaliru_pybridge::greenlets;
use std::cell::RefCell;
use std::ffi::CString;

thread_local! {
    static GS: RefCell<Vec<G>> = const { RefCell::new(Vec::new()) };
}

// _k.switch(target, value): 0 = main context, n = greenlet n
unsafe extern "C" fn k_switch(_slf: *mut PyObject, args: *const *mut PyObject, nargs: Py_ssize_t) -> *mut PyObject {
    assert_eq!(nargs, 2);
    let t = PyLong_AsLong(*args);
    let v = PyFloat_AsDouble(*args.add(1));
    if !PyErr_Occurred().is_null() {
        return std::ptr::null_mut();
    }
    let target = if t == 0 { Target::Main } else { Target::G(GS.with(|g| g.borrow()[t as usize - 1].clone())) };
    PyFloat_FromDouble(switch(target, v))
}

// _k.noop(): the cheapest python -> rust call
unsafe extern "C" fn k_noop(_slf: *mut PyObject, _args: *const *mut PyObject, _nargs: Py_ssize_t) -> *mut PyObject {
    PyFloat_FromDouble(0.0)
}

static NOOP_DEF: PyMethodDef = PyMethodDef {
    ml_name: c"noop".as_ptr(),
    ml_meth: k_noop,
    ml_flags: METH_FASTCALL,
    ml_doc: std::ptr::null(),
};

static SWITCH_DEF: PyMethodDef = PyMethodDef {
    ml_name: c"switch".as_ptr(),
    ml_meth: k_switch,
    ml_flags: METH_FASTCALL,
    ml_doc: std::ptr::null(),
};

fn run(code: &str) {
    let c = CString::new(code).unwrap();
    if unsafe { PyRun_SimpleString(c.as_ptr()) } != 0 {
        eprintln!("python error (traceback above)");
        std::process::exit(1);
    }
}

/// CLOCK_MONOTONIC in seconds, as python's time.monotonic()
fn mono() -> f64 {
    let mut t = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut t) };
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}

fn rss(tag: &str) {
    let s = std::fs::read_to_string("/proc/self/status").unwrap();
    let get = |k: &str| s.lines().find(|l| l.starts_with(k)).unwrap_or("").split_whitespace().nth(1).unwrap_or("?").to_string();
    println!("{tag:24} RssAnon {:>7} kB  RssFile {:>7} kB", get("RssAnon:"), get("RssFile:"));
}

const G1: &str = r#"
import sys, _k, traceback
def rec(n):
    if n == 0:
        try:
            raise ValueError("g1 exc")
        except ValueError:
            v = _k.switch(2, 0.0)
            # still inside the except block of this greenlet
            assert repr(sys.exception()) == "ValueError('g1 exc')", sys.exception()
            results.append(("g1 frames", len(traceback.extract_stack()) >= 52))
            return v
    return rec(n - 1) + 1
results.append(("g1", rec(50)))
"#;

const G2: &str = r#"
import sys, _k
# g1 is suspended inside an except block: not visible here
results.append(("g2 exception isolated", sys.exception() is None))
# C recursion (map/list) until the greenlet's 1 MiB stack runs out
old = sys.getrecursionlimit()
sys.setrecursionlimit(10**7)
def f(n):
    return list(map(f, [n + 1]))
try:
    f(0)
    results.append(("g2 C stack", "no error"))
except RecursionError as e:
    results.append(("g2 C stack", "RecursionError"))
sys.setrecursionlimit(old)
def rec2(n):
    if n == 0:
        return _k.switch(1, 5.0)
    return rec2(n - 1) + 10
results.append(("g2", rec2(30)))
"#;

fn main() {
    let noswap = std::env::args().nth(1).as_deref() == Some("noswap");
    rss("start");
    unsafe {
        Py_InitializeEx(0);
        let m = PyImport_AddModuleRef(c"_k".as_ptr());
        let f = PyCMethod_New(&SWITCH_DEF, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut());
        PyObject_SetAttrString(m, c"switch".as_ptr(), f);
        Py_DecRef(f);
        let f = PyCMethod_New(&NOOP_DEF, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut());
        PyObject_SetAttrString(m, c"noop".as_ptr(), f);
        Py_DecRef(f);
        Py_DecRef(m);
    }
    rss("python initialized");
    if !noswap {
        greenlets::install();
    }
    run("import builtins; builtins.results = []");

    let g1 = Greenlet::new(|| run(G1));
    let g2 = Greenlet::new(|| run(G2));
    GS.with(|g| *g.borrow_mut() = vec![g1.clone(), g2.clone()]);
    switch(Target::G(g1.clone()), 0.); // g1 -> g2 -> g1 (finishes)
    switch(Target::G(g2.clone()), 7.0); // g2 resumes and finishes
    assert!(g1.is_dead() && g2.is_dead());
    run(r#"
import sys
results.append(("main exception clean", sys.exception() is None))
expect = [("g2 exception isolated", True), ("g2 C stack", "RecursionError"),
          ("g1 frames", True), ("g1", 55.0), ("g2", 307.0), ("main exception clean", True)]
print("greenlets:", "OK" if results == expect else "FAIL %r" % (results,), flush=True)
"#);
    rss("after greenlet test");

    // thread: never runs while kaliru works with the GIL held, does run while
    // it is released (the reactor's poll). Its steps are stamped with
    // time.monotonic(), the clock mono() reads.
    run(r#"
import threading, time
stamps = []
def work():
    for i in range(5):
        stamps.append(time.monotonic())
        time.sleep(0.001)
th = threading.Thread(target=work)
th.start()
"#);
    let busy_start = mono();
    while mono() - busy_start < 0.05 {} // kaliru work with the GIL held
    let busy_end = mono();
    unsafe {
        let ts = PyEval_SaveThread(); // poll
        std::thread::sleep(std::time::Duration::from_millis(100));
        PyEval_RestoreThread(ts);
    }
    run(&format!(r#"
th.join()
during = [t for t in stamps if {busy_start} < t < {busy_end}]
print("thread:", "OK" if not during and len(stamps) == 5 else "FAIL %r" % ((during, stamps),), flush=True)
"#));
    rss("after thread test");

    // crossing costs
    const N: usize = 200_000;
    run(&format!(r#"
import time, _k
def cb():
    return None
t = time.perf_counter()
for i in range({N}):
    _k.noop()
py_to_rust = (time.perf_counter() - t) / {N}
t = time.perf_counter()
for i in range({N}):
    cb()
py_to_py = (time.perf_counter() - t) / {N}
print("python->rust call: %.3f us (python->python call: %.3f us)" % (py_to_rust * 1e6, py_to_py * 1e6), flush=True)
"#));
    unsafe {
        let main = PyImport_AddModuleRef(c"__main__".as_ptr());
        let cb = PyObject_GetAttrString(main, c"cb".as_ptr());
        let t0 = mono();
        for _ in 0..N {
            let r = PyObject_CallNoArgs(cb);
            Py_DecRef(r);
        }
        let dt = (mono() - t0) / N as f64;
        println!("rust->python call: {:.3} us", dt * 1e6);
        let g = Greenlet::new(|| {});
        let t0 = mono();
        for _ in 0..N {
            greenlets::switch_hook_cost(&g);
        }
        println!("thread state swap per greenlet switch: {:.3} us", (mono() - t0) / N as f64 * 1e6);
        Py_DecRef(cb);
        Py_DecRef(main);
    }
}
