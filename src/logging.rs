// The parts of python's `logging` module klippy uses: the root logger with
// module level functions. Like python, the first message logged without a
// handler installs basicConfig()'s stderr handler, format
// "%(levelname)s:%(name)s:%(message)s". queuelogger installs its own handler.
use std::sync::Mutex;

pub const DEBUG: i32 = 10;
pub const INFO: i32 = 20;
pub const WARNING: i32 = 30;
pub const ERROR: i32 = 40;

pub type Handler = Box<dyn Fn(i32, &str) + Send>;

struct Root {
    level: i32,
    handler: Option<Handler>,
}

static ROOT: Mutex<Root> = Mutex::new(Root { level: WARNING, handler: None });

pub fn level_name(level: i32) -> &'static str {
    match level {
        DEBUG => "DEBUG",
        INFO => "INFO",
        WARNING => "WARNING",
        ERROR => "ERROR",
        _ => "Level",
    }
}

fn basic_config_handler() -> Handler {
    Box::new(|level, msg| eprintln!("{}:root:{}", level_name(level), msg))
}

/// logging.getLogger().setLevel()
pub fn set_level(level: i32) {
    ROOT.lock().unwrap().level = level;
}

pub fn get_level() -> i32 {
    ROOT.lock().unwrap().level
}

/// root.addHandler() (klippy only ever installs one)
pub fn set_handler(handler: Option<Handler>) {
    ROOT.lock().unwrap().handler = handler;
}

pub fn log(level: i32, msg: &str) {
    let mut root = ROOT.lock().unwrap();
    if root.handler.is_none() {
        root.handler = Some(basic_config_handler());
    }
    if level >= root.level {
        (root.handler.as_ref().unwrap())(level, msg);
    }
}

pub fn debug(msg: &str) {
    log(DEBUG, msg)
}
pub fn info(msg: &str) {
    log(INFO, msg)
}
pub fn warning(msg: &str) {
    log(WARNING, msg)
}
pub fn error(msg: &str) {
    log(ERROR, msg)
}

/// logging.exception(); python appends the active traceback, which has no
/// rust equivalent: only the message is logged
// ponytail: no traceback text; mask "Traceback" blocks when diffing logs
pub fn exception(msg: &str) {
    log(ERROR, msg)
}

/// logging.exception(msg) while handling a python exception: the message
/// plus a traceback whose last line is "<class>: <str(e)>". Kaliru has no
/// python frames: one placeholder frame line is written (refdiff masks the
/// frame lines of both).
pub fn exception_tb(msg: &str, cls: &str, emsg: &str) {
    log(ERROR, &with_exc(msg, &traceback(cls, emsg)))
}

/// Formatter.format: the traceback goes on a new line after the message
fn with_exc(msg: &str, exc_text: &str) -> String {
    let sep = if msg.ends_with('\n') { "" } else { "\n" };
    format!("{}{}{}", msg, sep, exc_text)
}

fn traceback(cls: &str, emsg: &str) -> String {
    // a SyntaxError's last line is its msg: the " (file, line N)" of str(e)
    // is shown as frame lines instead
    let mut emsg = emsg;
    if matches!(cls, "SyntaxError" | "IndentationError" | "TabError") && emsg.ends_with(')') {
        if let Some(i) = emsg.rfind(" (") {
            if emsg[i..].contains(", line ") {
                emsg = &emsg[..i];
            }
        }
    }
    let last = if emsg.is_empty() { cls.to_string() } else { format!("{}: {}", cls, emsg) };
    format!("Traceback (most recent call last):\n  File \"<kaliru>\"\n{}", last)
}

/// traceback.format_exception() of `e`, its __context__ chain first
fn format_exception(e: &crate::error::KError) -> String {
    match e {
        crate::error::KError::Chained(err, context) => format!(
            "{}\n\nDuring handling of the above exception, another exception occurred:\n\n{}",
            format_exception(context),
            format_exception(err)
        ),
        crate::error::KError::Foreign(f) if f.traceback.is_some() => f.traceback.as_deref().unwrap().trim_end().to_string(),
        e => traceback(e.py_class(), &e.to_string()),
    }
}

/// logging.exception(msg) while handling `e`
pub fn exception_err(msg: &str, e: &crate::error::KError) {
    log(ERROR, &with_exc(msg, &format_exception(e)))
}

/// logging.warning(msg, exc_info=True) while handling `e`
pub fn warning_err(msg: &str, e: &crate::error::KError) {
    log(WARNING, &with_exc(msg, &format_exception(e)))
}
