// Code to implement asynchronous logging from a background thread
// (port of klippy/queuelogger.py, with the parts of python 3.13's
// logging.handlers.TimedRotatingFileHandler it relies on; sources in ref/).
// Rusticc: COSMOS's 0001-Reduce-log-rotate-threshold makes the listener a
// RotatingFileHandler (5 MiB, 2 backups): SizeRotatingFile.
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::logging;
use std::collections::BTreeMap;
use std::fs::{File, OpenOptions};
use std::io::{Seek, SeekFrom, Write};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};

const MIDNIGHT: i64 = 24 * 60 * 60;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum When {
    /// when="S", interval=60*60*24 (rotate_log_at_restart)
    Seconds,
    /// when="midnight"
    Midnight,
}

struct LocalTm {
    hour: i64,
    min: i64,
    sec: i64,
    isdst: i32,
}

fn localtime(t: i64) -> LocalTm {
    unsafe {
        let tt = t as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&tt, &mut tm);
        LocalTm { hour: tm.tm_hour as i64, min: tm.tm_min as i64, sec: tm.tm_sec as i64, isdst: tm.tm_isdst }
    }
}

fn strftime_local(fmt: &str, t: i64) -> String {
    let f = std::ffi::CString::new(fmt).unwrap();
    let mut buf = [0u8; 64];
    unsafe {
        let tt = t as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&tt, &mut tm);
        let n = libc::strftime(buf.as_mut_ptr() as *mut libc::c_char, buf.len(), f.as_ptr(), &tm);
        String::from_utf8_lossy(&buf[..n]).into_owned()
    }
}

fn now() -> i64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

/// TimedRotatingFileHandler (backupCount=5, local time, no namer/rotator)
pub struct RotatingFile {
    base_filename: String,
    stream: Option<File>,
    when: When,
    interval: i64,
    suffix: &'static str,
    backup_count: usize,
    pub rollover_at: i64,
}

impl RotatingFile {
    pub fn new(filename: &str, when: When) -> std::io::Result<Self> {
        let base_filename = crate::pyglob::abspath(filename);
        let stream = Some(OpenOptions::new().create(true).append(true).open(&base_filename)?);
        let (interval, suffix) = match when {
            When::Seconds => (60 * 60 * 24, "%Y-%m-%d_%H-%M-%S"),
            When::Midnight => (60 * 60 * 24, "%Y-%m-%d"),
        };
        let t = std::fs::metadata(&base_filename)
            .ok()
            .and_then(|m| m.modified().ok())
            .and_then(|m| m.duration_since(std::time::UNIX_EPOCH).ok())
            .map(|d| d.as_secs() as i64)
            .unwrap_or_else(now);
        let mut f = RotatingFile { base_filename, stream, when, interval, suffix, backup_count: 5, rollover_at: 0 };
        f.rollover_at = f.compute_rollover(t);
        Ok(f)
    }

    /// computeRollover(currentTime)
    pub fn compute_rollover(&self, current_time: i64) -> i64 {
        let mut result = current_time + self.interval;
        if self.when == When::Midnight {
            let t = localtime(current_time);
            let mut r = MIDNIGHT - ((t.hour * 60 + t.min) * 60 + t.sec);
            if r <= 0 {
                r += MIDNIGHT;
            }
            result = current_time + r;
            result += self.interval - MIDNIGHT;
            let dst_now = t.isdst;
            let dst_at_rollover = localtime(result).isdst;
            if dst_now != dst_at_rollover {
                let addend = if dst_now == 0 {
                    // DST kicks in before next rollover, so we need to deduct an hour
                    if localtime(result - 3600).isdst == 0 {
                        0
                    } else {
                        -3600
                    }
                } else {
                    // DST bows out before next rollover, so we need to add an hour
                    3600
                };
                result += addend;
            }
        }
        result
    }

    fn should_rollover(&mut self) -> bool {
        let t = now();
        if t >= self.rollover_at {
            let p = std::path::Path::new(&self.base_filename);
            if p.exists() && !p.is_file() {
                self.rollover_at = self.compute_rollover(t);
                return false;
            }
            return true;
        }
        false
    }

    // extMatch fullmatch of the suffix
    fn ext_match(&self, s: &str) -> bool {
        let pat: &[u8] = match self.when {
            When::Seconds => b"dddd-dd-dd_dd-dd-dd",
            When::Midnight => b"dddd-dd-dd",
        };
        let b = s.as_bytes();
        b.len() == pat.len() && b.iter().zip(pat).all(|(c, p)| if *p == b'd' { c.is_ascii_digit() } else { c == p })
    }

    fn get_files_to_delete(&self) -> Vec<String> {
        let (dir_name, base_name) = crate::pyglob::split(&self.base_filename);
        let prefix = format!("{}.", base_name);
        let mut result: Vec<String> = std::fs::read_dir(&dir_name)
            .map(|rd| {
                rd.flatten()
                    .map(|e| e.file_name().to_string_lossy().into_owned())
                    .filter(|f| f.starts_with(&prefix) && self.ext_match(&f[prefix.len()..]))
                    .map(|f| crate::pyglob::join(&dir_name, &[&f]))
                    .collect()
            })
            .unwrap_or_default();
        if result.len() < self.backup_count {
            return vec![];
        }
        result.sort();
        result.truncate(result.len() - self.backup_count);
        result
    }

    /// doRollover(); returns whether rollover_at moved
    pub fn do_rollover(&mut self) -> bool {
        let before = self.rollover_at;
        let current_time = now();
        let mut t = self.rollover_at - self.interval;
        let dst_now = localtime(current_time).isdst;
        let dst_then = localtime(t).isdst;
        if dst_now != dst_then {
            t += if dst_now != 0 { 3600 } else { -3600 };
        }
        let dfn = format!("{}.{}", self.base_filename, strftime_local(self.suffix, t));
        if std::path::Path::new(&dfn).exists() {
            return false;
        }
        self.stream = None;
        if std::path::Path::new(&self.base_filename).exists() {
            let _ = std::fs::rename(&self.base_filename, &dfn);
        }
        for s in self.get_files_to_delete() {
            let _ = std::fs::remove_file(s);
        }
        self.stream = OpenOptions::new().create(true).append(true).open(&self.base_filename).ok();
        self.rollover_at = self.compute_rollover(current_time);
        self.rollover_at > before
    }

    fn write_line(&mut self, msg: &str) {
        if let Some(f) = self.stream.as_mut() {
            let _ = f.write_all(format!("{}\n", msg).as_bytes());
            let _ = f.flush();
        }
    }
}

/// python 3.15 logging.handlers.RotatingFileHandler (mode 'a', no
/// namer/rotator)
pub struct SizeRotatingFile {
    base_filename: String,
    stream: Option<File>,
    max_bytes: u64,
    backup_count: usize,
}

impl SizeRotatingFile {
    pub fn new(filename: &str, max_bytes: u64, backup_count: usize) -> std::io::Result<Self> {
        let base_filename = crate::pyglob::abspath(filename);
        let stream = Some(Self::open(&base_filename)?);
        Ok(SizeRotatingFile { base_filename, stream, max_bytes, backup_count })
    }

    // open(filename, 'a'): positioned at the end, as tell() reports it
    fn open(path: &str) -> std::io::Result<File> {
        let mut f = OpenOptions::new().create(true).append(true).open(path)?;
        f.seek(SeekFrom::End(0))?;
        Ok(f)
    }

    /// shouldRollover(record) for the formatted message `msg`
    fn should_rollover(&mut self, msg: &str) -> bool {
        if self.stream.is_none() {
            self.stream = Self::open(&self.base_filename).ok();
        }
        if self.max_bytes > 0 {
            let Some(pos) = self.stream.as_mut().and_then(|f| f.stream_position().ok()) else { return false };
            // Never rollover an empty file
            if pos == 0 {
                return false;
            }
            // (len() of the text: characters, not bytes)
            if pos + msg.chars().count() as u64 + 1 >= self.max_bytes {
                // Never rollover anything other than regular files
                let p = std::path::Path::new(&self.base_filename);
                return !(p.exists() && !p.is_file());
            }
        }
        false
    }

    /// doRollover()
    pub fn do_rollover(&mut self) {
        let exists = |p: &str| std::path::Path::new(p).exists();
        self.stream = None;
        if self.backup_count > 0 {
            for i in (1..self.backup_count).rev() {
                let sfn = format!("{}.{}", self.base_filename, i);
                let dfn = format!("{}.{}", self.base_filename, i + 1);
                if exists(&sfn) {
                    if exists(&dfn) {
                        let _ = std::fs::remove_file(&dfn);
                    }
                    let _ = std::fs::rename(&sfn, &dfn);
                }
            }
            let dfn = format!("{}.1", self.base_filename);
            if exists(&dfn) {
                let _ = std::fs::remove_file(&dfn);
            }
            if exists(&self.base_filename) {
                let _ = std::fs::rename(&self.base_filename, &dfn);
            }
        }
        self.stream = Self::open(&self.base_filename).ok();
    }

    fn write_line(&mut self, msg: &str) {
        if let Some(f) = self.stream.as_mut() {
            let _ = f.write_all(format!("{}
", msg).as_bytes());
            let _ = f.flush();
        }
    }
}

struct Inner {
    file: SizeRotatingFile,
    rollover_info: BTreeMap<String, String>,
}

impl Inner {
    /// QueueListener.doRollover
    fn do_rollover(&mut self) {
        self.file.do_rollover();
        let mut lines: Vec<String> = self.rollover_info.values().cloned().collect();
        lines.push(format!("=============== Log rollover at {} ===============", crate::util::asctime_local(now() as f64)));
        self.emit(&lines.join("\n"));
    }

    /// BaseRotatingHandler.emit
    fn emit(&mut self, msg: &str) {
        if self.file.should_rollover(msg) {
            self.do_rollover();
        }
        self.file.write_line(msg);
    }
}

/// QueueListener: writes log records from a background thread
pub struct QueueListener {
    inner: Arc<Mutex<Inner>>,
    tx: mpsc::Sender<Option<String>>,
    thread: Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl QueueListener {
    /// (rotate_log_at_restart: unused here, as in COSMOS's queuelogger)
    pub fn new(filename: &str, _rotate_log_at_restart: bool) -> std::io::Result<Self> {
        let file = SizeRotatingFile::new(filename, 5 * 1024 * 1024, 2)?;
        let inner = Arc::new(Mutex::new(Inner { file, rollover_info: BTreeMap::new() }));
        let (tx, rx) = mpsc::channel::<Option<String>>();
        let i = inner.clone();
        let thread = std::thread::spawn(move || {
            while let Ok(Some(msg)) = rx.recv() {
                i.lock().unwrap().emit(&msg);
            }
        });
        Ok(QueueListener { inner, tx, thread: Mutex::new(Some(thread)) })
    }

    pub fn stop(&self) {
        let _ = self.tx.send(None);
        if let Some(t) = self.thread.lock().unwrap().take() {
            let _ = t.join();
        }
    }

    pub fn set_rollover_info(&self, name: &str, info: Option<&str>) {
        let mut i = self.inner.lock().unwrap();
        match info {
            None => {
                i.rollover_info.remove(name);
            }
            Some(info) => {
                i.rollover_info.insert(name.to_string(), info.to_string());
            }
        }
    }

    pub fn clear_rollover_info(&self) {
        self.inner.lock().unwrap().rollover_info.clear();
    }

    pub fn do_rollover(&self) {
        self.inner.lock().unwrap().do_rollover();
    }

    fn sender(&self) -> mpsc::Sender<Option<String>> {
        self.tx.clone()
    }
}

/// setup_bg_logging(filename, debuglevel, rotate_log_at_restart)
pub fn setup_bg_logging(filename: &str, debuglevel: i32, rotate_log_at_restart: bool) -> std::io::Result<QueueListener> {
    let ql = QueueListener::new(filename, rotate_log_at_restart)?;
    // QueueHandler: the formatted message ("%(message)s") goes to the queue
    let tx = Mutex::new(ql.sender());
    logging::set_handler(Some(Box::new(move |_level, msg| {
        let _ = tx.lock().unwrap().send(Some(msg.to_string()));
    })));
    logging::set_level(debuglevel);
    Ok(ql)
}

pub fn clear_bg_logging() {
    logging::set_handler(None);
    logging::set_level(logging::WARNING);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rollover_files() {
        let dir = std::env::temp_dir().join(format!("kaliru_ql_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("klippy.log").to_string_lossy().into_owned();
        // six old backups and an unrelated file
        for d in ["2026-01-01", "2026-01-02", "2026-01-03", "2026-01-04", "2026-01-05", "2026-01-06"] {
            std::fs::write(format!("{}.{}", log, d), "old").unwrap();
        }
        std::fs::write(format!("{}.notadate", log), "x").unwrap();
        let mut f = RotatingFile::new(&log, When::Midnight).unwrap();
        f.write_line("hello");
        // force a rollover: as if the next midnight had passed
        f.rollover_at = now() - 1;
        f.interval = 60 * 60 * 24;
        assert!(f.do_rollover());
        let mut names: Vec<String> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
        names.sort();
        // 5 backups kept (oldest two deleted), the unrelated file stays
        assert_eq!(names.len(), 7, "{:?}", names);
        assert!(!names.contains(&"klippy.log.2026-01-01".to_string()));
        assert!(names.contains(&"klippy.log.notadate".to_string()));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn size_rollover() {
        let dir = std::env::temp_dir().join(format!("kaliru_qls_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("klippy.log").to_string_lossy().into_owned();
        std::fs::write(format!("{}.1", log), "one").unwrap();
        std::fs::write(format!("{}.2", log), "two").unwrap();
        let mut f = SizeRotatingFile::new(&log, 11, 2).unwrap();
        // an empty file never rolls over
        assert!(!f.should_rollover("0123456789abc"));
        f.write_line("abcd");
        // pos 5 + the message and its newline, in characters ("é" is one)
        assert!(!f.should_rollover("123é"));
        assert!(f.should_rollover("12345"));
        f.do_rollover();
        let read = |n: &str| std::fs::read_to_string(dir.join(n)).unwrap();
        assert_eq!(read("klippy.log.1"), "abcd
");
        assert_eq!(read("klippy.log.2"), "one");
        assert_eq!(read("klippy.log"), "");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
