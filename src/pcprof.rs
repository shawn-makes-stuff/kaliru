// Development profiler for hosts without perf (the CC1): with
// KALIRU_PCPROF=FILE, SIGPROF every 2 ms of process CPU time records the
// interrupted thread's id, pc and link register. After KALIRU_PCPROF_SECS
// (default 60) a thread writes "tid pc lr count" lines, plus the binary's
// load address from /proc/self/maps, to FILE; map the addresses with
// addr2line on an unstripped build of the same code
// (tools/pcprof_report.py). Not in python; off unless the variable is set.
use std::sync::atomic::{AtomicUsize, Ordering};

const SLOTS: usize = 1 << 16;
static SAMPLES: [[AtomicUsize; 3]; SLOTS] = [const { [const { AtomicUsize::new(0) }; 3] }; SLOTS];
static COUNT: AtomicUsize = AtomicUsize::new(0);

#[cfg(target_arch = "arm")]
unsafe fn pc_lr(uc: *const libc::ucontext_t) -> (usize, usize) {
    ((*uc).uc_mcontext.arm_pc as usize, (*uc).uc_mcontext.arm_lr as usize)
}

#[cfg(target_arch = "x86_64")]
unsafe fn pc_lr(uc: *const libc::ucontext_t) -> (usize, usize) {
    // no link register: the word at the stack pointer (a return address in
    // a leaf before it sets up its frame, else a guess)
    let g = &(*uc).uc_mcontext.gregs;
    let sp = g[libc::REG_RSP as usize] as *const usize;
    (g[libc::REG_RIP as usize] as usize, *sp)
}

#[cfg(not(any(target_arch = "arm", target_arch = "x86_64")))]
unsafe fn pc_lr(_uc: *const libc::ucontext_t) -> (usize, usize) {
    (0, 0)
}

extern "C" fn on_prof(_sig: libc::c_int, _info: *mut libc::siginfo_t, ctx: *mut libc::c_void) {
    let i = COUNT.fetch_add(1, Ordering::Relaxed);
    if i >= SLOTS {
        return;
    }
    let (pc, lr) = unsafe { pc_lr(ctx as *const libc::ucontext_t) };
    let tid = unsafe { libc::syscall(libc::SYS_gettid) } as usize;
    SAMPLES[i][0].store(tid, Ordering::Relaxed);
    SAMPLES[i][1].store(pc, Ordering::Relaxed);
    SAMPLES[i][2].store(lr, Ordering::Relaxed);
}

/// Start sampling if KALIRU_PCPROF is set
pub fn start() {
    let Some(path) = std::env::var_os("KALIRU_PCPROF") else { return };
    let secs: u64 = std::env::var("KALIRU_PCPROF_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(60);
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_prof as usize;
        sa.sa_flags = libc::SA_SIGINFO | libc::SA_RESTART;
        libc::sigaction(libc::SIGPROF, &sa, std::ptr::null_mut());
        let tv = libc::timeval { tv_sec: 0, tv_usec: 2000 };
        let it = libc::itimerval { it_interval: tv, it_value: tv };
        libc::setitimer(libc::ITIMER_PROF, &it, std::ptr::null_mut());
    }
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_secs(secs));
        let n = COUNT.load(Ordering::Relaxed).min(SLOTS);
        let mut hist: std::collections::HashMap<(usize, usize, usize), usize> = std::collections::HashMap::new();
        for s in SAMPLES.iter().take(n) {
            let key = (s[0].load(Ordering::Relaxed), s[1].load(Ordering::Relaxed), s[2].load(Ordering::Relaxed));
            *hist.entry(key).or_default() += 1;
        }
        let mut out = String::new();
        // executable mappings (the binary, libc, libpython, ...)
        for l in std::fs::read_to_string("/proc/self/maps").unwrap_or_default().lines() {
            if l.split_whitespace().nth(1).is_some_and(|p| p.contains('x')) && l.contains('/') {
                out += &format!("map {}\n", l);
            }
        }
        out += &format!("samples {} interval_us 2000 secs {}\n", n, secs);
        let mut rows: Vec<_> = hist.into_iter().collect();
        rows.sort_by(|a, b| b.1.cmp(&a.1));
        for ((tid, pc, lr), c) in rows {
            out += &format!("{} {:x} {:x} {}\n", tid, pc, lr, c);
        }
        let _ = std::fs::write(&path, out);
    });
}
