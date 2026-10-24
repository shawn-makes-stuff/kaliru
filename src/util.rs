// Low level unix utility functions (port of klippy/util.py)
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Rusticc: get_git_version() runs git in the install directory as Kalico
// does, then reads klippy/.version (the klippy directory is
// plugins::klippy_dir()).
// Deliberate difference (no python source tree next to the binary):
// - dump_mcu_build() looks for the mcu build files one directory above the
//   binary (python: one directory above klippy/, the source tree root)
use crate::logging;
use crate::pyfmt::py_strip;
use std::ffi::CString;
use std::os::fd::RawFd;

// Return the SIGINT interrupt handler back to the OS default
pub fn fix_sigint() {
    unsafe { libc::signal(libc::SIGINT, libc::SIG_DFL) };
}

// Set a file-descriptor as non-blocking
pub fn set_nonblock(fd: RawFd) {
    unsafe {
        let flags = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
    }
}

// Clear HUPCL flag
pub fn clear_hupcl(fd: RawFd) {
    unsafe {
        let mut attrs: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut attrs) != 0 {
            // python: termios.tcgetattr raises (not caught here)
            return;
        }
        attrs.c_cflag &= !libc::HUPCL;
        // termios.error is ignored
        libc::tcsetattr(fd, libc::TCSADRAIN, &attrs);
    }
}

// Support for creating a pseudo-tty for emulating a serial port
pub fn create_pty(ptyname: &str) -> std::io::Result<RawFd> {
    let (mut mfd, mut sfd) = (0, 0);
    let r = unsafe { libc::openpty(&mut mfd, &mut sfd, std::ptr::null_mut(), std::ptr::null(), std::ptr::null()) };
    if r != 0 {
        return Err(std::io::Error::last_os_error());
    }
    let _ = std::fs::remove_file(ptyname);
    let filename = unsafe {
        let p = libc::ttyname(sfd);
        if p.is_null() {
            return Err(std::io::Error::last_os_error());
        }
        std::ffi::CStr::from_ptr(p).to_string_lossy().into_owned()
    };
    let cf = CString::new(filename.clone()).unwrap();
    unsafe { libc::chmod(cf.as_ptr(), 0o660) };
    std::os::unix::fs::symlink(&filename, ptyname)?;
    set_nonblock(mfd);
    unsafe {
        let mut old: libc::termios = std::mem::zeroed();
        libc::tcgetattr(mfd, &mut old);
        old.c_lflag &= !libc::ECHO;
        libc::tcsetattr(mfd, libc::TCSADRAIN, &old);
    }
    Ok(mfd)
}

// ----------------------------------------------------------------------
// Helper code for extracting mcu build info
// ----------------------------------------------------------------------

/// time.asctime(time.localtime(t))
pub fn asctime_local(t: f64) -> String {
    let fmt = CString::new("%a %b %e %H:%M:%S %Y").unwrap();
    let mut buf = [0u8; 64];
    unsafe {
        let tt = t as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&tt, &mut tm);
        let n = libc::strftime(buf.as_mut_ptr() as *mut libc::c_char, buf.len(), fmt.as_ptr(), &tm);
        String::from_utf8_lossy(&buf[..n]).into_owned()
    }
}

pub fn dump_file_stats(build_dir: &str, filename: &str) {
    let fname = crate::pyglob::join(build_dir, &[filename]);
    match std::fs::metadata(&fname) {
        Ok(m) => {
            use std::os::unix::fs::MetadataExt;
            let timestr = asctime_local(m.mtime() as f64);
            logging::info(&format!("Build file {}({}): {}", fname, m.size(), timestr));
        }
        Err(_) => logging::info(&format!("No build file {}", fname)),
    }
}

fn build_dir() -> String {
    let exe = std::env::current_exe().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
    crate::pyglob::join(&crate::pyglob::dirname(&exe), &[".."])
}

/// f.read(32 * 1024) of a text file
fn read_32k(path: &str) -> Option<String> {
    let data = std::fs::read(path).ok()?;
    let s = String::from_utf8(data).ok()?;
    Some(s.chars().take(32 * 1024).collect())
}

// Try to log information on the last mcu build
pub fn dump_mcu_build() {
    let build_dir = build_dir();
    // Try to log last mcu config
    dump_file_stats(&build_dir, ".config");
    if let Some(data) = read_32k(&crate::pyglob::join(&build_dir, &[".config"])) {
        logging::info(&format!("========= Last MCU build config =========\n{}=======================", data));
    }
    // Try to log last mcu build version
    dump_file_stats(&build_dir, "out/klipper.dict");
    if let Some(data) = read_32k(&crate::pyglob::join(&build_dir, &["out/klipper.dict"])) {
        if let Ok(data) = serde_json::from_str::<serde_json::Value>(&data) {
            let get = |k: &str| data.get(k).map(crate::pyfmt::py_str).unwrap_or_default();
            logging::info(&format!("Last MCU build version: {}", get("version")));
            logging::info(&format!("Last MCU build tools: {}", get("build_versions")));
            let cparts: Vec<String> = data
                .get("config")
                .and_then(|c| c.as_object())
                .map(|c| c.iter().map(|(k, v)| format!("{}={}", k, crate::pyfmt::py_str(v))).collect())
                .unwrap_or_default();
            logging::info(&format!("Last MCU build config: {}", cparts.join(" ")));
        }
    }
    dump_file_stats(&build_dir, "out/klipper.elf");
}

// ----------------------------------------------------------------------
// General system and software information
// ----------------------------------------------------------------------

pub fn get_cpu_info() -> String {
    let data = match std::fs::read_to_string("/proc/cpuinfo") {
        Ok(d) => d,
        Err(e) => {
            logging::debug(&format!("Exception on read /proc/cpuinfo: {}", e));
            return "?".into();
        }
    };
    let lines: Vec<(String, String)> = data
        .split('\n')
        .filter_map(|l| l.split_once(':'))
        .map(|(k, v)| (py_strip(k).to_string(), py_strip(v).to_string()))
        .collect();
    let core_count = lines.iter().filter(|(k, _)| k == "processor").count();
    // dict(lines): the last "model name" wins
    let model_name = lines.iter().rev().find(|(k, _)| k == "model name").map(|(_, v)| v.clone()).unwrap_or_else(|| "?".into());
    format!("{} core {}", core_count, model_name)
}

pub struct GitInfo {
    pub version: String,
    pub file_status: Vec<Vec<String>>,
    pub branch: String,
    pub remote: String,
    pub url: String,
}

/// Rusticc: get_git_version(from_file=True) as Kalico runs it from an
/// install directory (COSMOS: /usr/share/klipper, no git repository): git
/// describe in klippy/.., else klippy/.version ("Release - <version>")
pub fn get_git_version() -> GitInfo {
    let klippy_src = crate::plugins::klippy_dir();
    let gitdir = klippy_src.join("..").to_string_lossy().into_owned();
    let mut info = GitInfo { version: "?".into(), file_status: vec![], branch: "?".into(), remote: "?".into(), url: "?".into() };
    match git(&gitdir, &["describe", "--always", "--tags", "--long", "--dirty"]) {
        Ok((0, ver, _)) => {
            info.version = bytes_strip(&ver);
            match git(&gitdir, &["status", "--porcelain", "--ignored"]) {
                Ok((0, stat, _)) => {
                    info.file_status = bytes_strip(&stat).split('\n').map(|l| py_split_maxsplit1(l)).collect();
                }
                Ok((_, _, err)) => logging::debug(&format!("Error getting git status: {}", crate::pyfmt::py_bytes_repr(&err))),
                Err(e) => logging::debug(&format!("Exception on run: {}", e)),
            }
            get_repo_info(&gitdir, &mut info);
            return info;
        }
        Ok((_, _, err)) => logging::debug(&format!("Error getting git version: {}", crate::pyfmt::py_bytes_repr(&err))),
        Err(e) => logging::debug(&format!("Exception on run: {}", e)),
    }
    info.version = get_version_from_file(&klippy_src);
    info
}

/// get_version_from_file(klippy_src)
pub fn get_version_from_file(klippy_src: &std::path::Path) -> String {
    match std::fs::read_to_string(klippy_src.join(".version")) {
        Ok(s) => s.trim_end().to_string(),
        Err(_) => "?".into(),
    }
}

/// subprocess.Popen(("git", "-C", gitdir, ...)): (returncode, stdout,
/// stderr), or the traceback.format_exc() text when git cannot run
fn git(gitdir: &str, args: &[&str]) -> Result<(i32, Vec<u8>, Vec<u8>), String> {
    match std::process::Command::new("git").arg("-C").arg(gitdir).args(args).output() {
        Ok(o) => Ok((o.status.code().unwrap_or(-1), o.stdout, o.stderr)),
        Err(e) => Err(format!(
            "Traceback (most recent call last):\n  File \"<kaliru>\"\n{}: {}\n",
            crate::pyfmt::py_oserror_class(&e),
            crate::pyfmt::py_oserror(&e, "git")
        )),
    }
}

/// str(b.strip().decode())
fn bytes_strip(b: &[u8]) -> String {
    py_strip(&String::from_utf8_lossy(b)).to_string()
}

/// str.split(None, 1)
fn py_split_maxsplit1(l: &str) -> Vec<String> {
    let t = l.trim_start();
    if t.is_empty() {
        return vec![];
    }
    match t.find(char::is_whitespace) {
        Some(i) => vec![t[..i].to_string(), t[i..].trim_start().to_string()],
        None => vec![t.to_string()],
    }
}

/// _get_repo_info(gitdir)
fn get_repo_info(gitdir: &str, info: &mut GitInfo) {
    let res = (|| -> Result<(), String> {
        let (rc, branch_list, err) = git(gitdir, &["branch", "--no-color"])?;
        if rc != 0 {
            logging::debug(&format!("Error running git branch: {}", crate::pyfmt::py_bytes_repr(&err)));
            return Ok(());
        }
        let text = bytes_strip(&branch_list);
        match text.split('\n').find(|l| l.starts_with('*')) {
            Some(l) => info.branch = py_strip(&l[1..]).to_string(),
            None => {
                logging::debug(&format!("Unable to find current branch:\n{}", crate::pyfmt::py_bytes_repr(&branch_list)));
                return Ok(());
            }
        }
        if info.branch.starts_with("(HEAD detached") {
            let inner = info.branch.trim_matches(|c| c == '(' || c == ')');
            let last = inner.split_whitespace().last().unwrap_or("");
            let parts: Vec<&str> = last.splitn(2, '/').collect();
            if parts.len() != 2 {
                return Ok(());
            }
            info.remote = parts[0].to_string();
        } else {
            let key = format!("branch.{}.remote", info.branch);
            let (rc, remote, err) = git(gitdir, &["config", "--get", &key])?;
            if rc != 0 {
                logging::debug(&format!("Error running git config: {}", crate::pyfmt::py_bytes_repr(&err)));
                return Ok(());
            }
            info.remote = bytes_strip(&remote);
        }
        let (rc, url, err) = git(gitdir, &["remote", "get-url", &info.remote.clone()])?;
        if rc != 0 {
            logging::debug(&format!("Error running git remote get-url: {}", crate::pyfmt::py_bytes_repr(&err)));
            return Ok(());
        }
        info.url = bytes_strip(&url);
        Ok(())
    })();
    if let Err(tb) = res {
        logging::debug(&format!("Error fetching repo info: {}", tb));
    }
}
