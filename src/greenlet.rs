// The subset of python's greenlet that klippy's reactor uses, on corosensei
// stackful coroutines: create a greenlet, switch(target, value) between any
// two contexts (symmetric, like greenlet.switch), getcurrent(), and kill a
// suspended greenlet (greenlet.throw() in reactor.finalize()).
//
// corosensei coroutines are asymmetric (resume / yield), so switching goes
// through a trampoline on the main stack: a greenlet yields (target, value)
// and the trampoline resumes the target with value. Only the main context
// (the code calling the reactor's run()) runs the trampoline.
use corosensei::stack::{DefaultStack, Stack};
use corosensei::{Coroutine, CoroutineResult, Yielder};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// Stack per greenlet. Virtual size: pages are only committed when touched,
/// with a guard page below.
// ponytail: fixed size (python greenlets grow); raise it if deep macro or
// callback chains overflow (that shows as a SIGSEGV on the guard page)
pub const STACK_SIZE: usize = 1 << 20;

type Yld = Yielder<f64, (Target, f64)>;

pub struct Greenlet {
    coro: RefCell<Option<Coroutine<f64, (Target, f64), (), DefaultStack>>>,
    yielder: Rc<Cell<*const Yld>>,
    dead: Cell<bool>,
    /// usable stack (lowest address, size), guard page excluded
    stack: (usize, usize),
}

/// What a switch hook is told, on the main stack, around every switch
pub enum SwitchEvent<'a> {
    /// this greenlet is about to run
    Enter(&'a Greenlet),
    /// the main context runs again
    Main,
    /// this greenlet finished or was killed
    Dead(&'a Greenlet),
}

pub type SwitchHook = fn(SwitchEvent);

pub type G = Rc<Greenlet>;

/// A switch target: the main context or a greenlet
#[derive(Clone)]
pub enum Target {
    Main,
    G(G),
}

thread_local! {
    // the running greenlet; None while the main context runs
    static CURRENT: RefCell<Option<G>> = const { RefCell::new(None) };
    // per-context state that follows the running greenlet (python plugins:
    // one python thread state per greenlet)
    static HOOK: Cell<Option<SwitchHook>> = const { Cell::new(None) };
}

/// Install the switch hook of this thread (None removes it)
pub fn set_switch_hook(hook: Option<SwitchHook>) {
    HOOK.with(|h| h.set(hook));
}

fn notify(ev: SwitchEvent) {
    if let Some(hook) = HOOK.with(|h| h.get()) {
        hook(ev);
    }
}

impl Greenlet {
    /// greenlet(run=...): `run` starts on the first switch to it; when it
    /// returns the greenlet is dead and control goes to the main context
    pub fn new(run: impl FnOnce() + 'static) -> G {
        let yielder: Rc<Cell<*const Yld>> = Rc::new(Cell::new(std::ptr::null()));
        let y = yielder.clone();
        let stack = DefaultStack::new(STACK_SIZE).expect("greenlet stack");
        // limit() includes the guard page
        let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) } as usize;
        let lo = stack.limit().get() + page;
        let bounds = (lo, stack.base().get() - lo);
        let coro = Coroutine::with_stack(stack, move |yielder: &Yld, _first: f64| {
            y.set(yielder as *const Yld);
            run();
        });
        Rc::new(Greenlet { coro: RefCell::new(Some(coro)), yielder, dead: Cell::new(false), stack: bounds })
    }

    pub fn is_dead(&self) -> bool {
        self.dead.get()
    }

    /// The usable stack: (lowest address, size in bytes)
    pub fn stack(&self) -> (usize, usize) {
        self.stack
    }

    /// greenlet.throw() on a suspended greenlet: unwinds its stack (running
    /// drops) and marks it dead. Must not be called on the running greenlet.
    pub fn kill(&self) {
        let coro = self.coro.borrow_mut().take();
        // the unwind runs on this greenlet's stack
        notify(SwitchEvent::Enter(self));
        drop(coro);
        notify(SwitchEvent::Main);
        self.dead.set(true);
        notify(SwitchEvent::Dead(self));
    }
}

/// greenlet.getcurrent(): None for the main context
pub fn getcurrent() -> Option<G> {
    CURRENT.with(|c| c.borrow().clone())
}

pub fn is_current(g: &G) -> bool {
    CURRENT.with(|c| c.borrow().as_ref().map(|cur| Rc::ptr_eq(cur, g)).unwrap_or(false))
}

/// target.switch(value): returns the value this context is later switched
/// back with (NaN when python would return () or None)
pub fn switch(target: Target, value: f64) -> f64 {
    match getcurrent() {
        Some(cur) => {
            // inside a greenlet: hand (target, value) to the trampoline
            let y = cur.yielder.get();
            drop(cur);
            unsafe { (*y).suspend((target, value)) }
        }
        None => trampoline(target, value),
    }
}

fn trampoline(mut target: Target, mut value: f64) -> f64 {
    loop {
        let g = match target {
            Target::Main => return value,
            Target::G(g) => g,
        };
        CURRENT.with(|c| *c.borrow_mut() = Some(g.clone()));
        notify(SwitchEvent::Enter(&g));
        let res = {
            let mut coro = g.coro.borrow_mut();
            let coro = coro.as_mut().expect("switch to a dead greenlet");
            coro.resume(value)
        };
        CURRENT.with(|c| *c.borrow_mut() = None);
        notify(SwitchEvent::Main);
        match res {
            CoroutineResult::Yield((t, v)) => {
                target = t;
                value = v;
            }
            CoroutineResult::Return(()) => {
                // a finished greenlet switches to its parent (the main context)
                g.dead.set(true);
                g.coro.borrow_mut().take();
                notify(SwitchEvent::Dead(&g));
                target = Target::Main;
                value = f64::NAN;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symmetric_switching() {
        let log = Rc::new(RefCell::new(vec![]));
        let g2_slot: Rc<RefCell<Option<G>>> = Rc::new(RefCell::new(None));
        let (l1, s1) = (log.clone(), g2_slot.clone());
        let g1 = Greenlet::new(move || {
            l1.borrow_mut().push("g1 start".to_string());
            let g2 = s1.borrow().clone().unwrap();
            // greenlet to greenlet with a value, like a timer callback g.switch(eventtime)
            let back = switch(Target::G(g2), 1.5);
            l1.borrow_mut().push(format!("g1 got {}", back));
            let back = switch(Target::Main, 3.0);
            l1.borrow_mut().push(format!("g1 resumed {}", back));
        });
        let (l2, g1c) = (log.clone(), g1.clone());
        let g2 = Greenlet::new(move || {
            l2.borrow_mut().push("g2 start".to_string());
            assert!(getcurrent().is_some());
            switch(Target::G(g1c.clone()), 2.5);
            l2.borrow_mut().push("g2 end".to_string());
        });
        *g2_slot.borrow_mut() = Some(g2.clone());
        let v = switch(Target::G(g1.clone()), 0.);
        assert_eq!(v, 3.0);
        assert!(getcurrent().is_none());
        let v = switch(Target::G(g1.clone()), 4.0);
        assert!(v.is_nan() && g1.is_dead());
        let v = switch(Target::G(g2.clone()), 0.);
        assert!(v.is_nan() && g2.is_dead());
        assert_eq!(
            *log.borrow(),
            ["g1 start", "g2 start", "g1 got 2.5", "g1 resumed 4", "g2 end"]
        );
    }

    #[test]
    fn kill_unwinds_suspended() {
        struct Guard(Rc<Cell<u32>>);
        impl Drop for Guard {
            fn drop(&mut self) {
                self.0.set(self.0.get() + 1);
            }
        }
        let drops = Rc::new(Cell::new(0));
        let d = drops.clone();
        let g = Greenlet::new(move || {
            let _guard = Guard(d);
            switch(Target::Main, 0.);
            unreachable!();
        });
        switch(Target::G(g.clone()), 0.);
        assert_eq!(drops.get(), 0);
        g.kill();
        assert_eq!(drops.get(), 1);
        assert!(g.is_dead());
    }

    #[test]
    fn deep_stack() {
        fn rec(n: u32) -> u64 {
            let buf = [n as u8; 512];
            if n == 0 {
                return buf[0] as u64;
            }
            rec(n - 1) + buf[(n % 512) as usize] as u64
        }
        let out = Rc::new(Cell::new(0));
        let o = out.clone();
        // ~600 KB of stack frames
        let g = Greenlet::new(move || o.set(rec(1000)));
        switch(Target::G(g), 0.);
        assert!(out.get() > 0);
    }
}
