// The python exception classes klippy code raises and catches by type, as
// one error enum (no python counterpart file). Display is python's str(e).
//
//   Config     configfile.error (configparser.Error)  printer.config_error
//   Pins       pins.error
//   Protocol   msgproto.error
//   Enumeration msgproto.enumeration_error (a msgproto.error subclass)
//   Serial     serialhdl.error
//   Mcu        mcu.error
//   Command    gcode.CommandError                     printer.command_error
//              (and its subclasses that are caught as CommandError:
//               WebRequestError, homing.CommandError, ...)
//   WaitInterruption  printer.WaitInterruption (a CommandError subclass)
//   CommandSub other CommandError subclasses that are caught by class
//              (trad_rack's TradRackLoadError, SelectorNotHomedError)
//   Reactor    reactor.ReactorError
//   Other      any other exception (str(e) only)
//   Foreign    an exception object from a python plugin (src/plugins/): it
//              behaves as its nearest kind above and re-raises as itself
use std::fmt;
use std::rc::Rc;

/// A python plugin's exception crossing kaliru code
pub struct ForeignError {
    /// what kaliru code sees: the nearest klippy class above, with str(e)
    pub kind: KError,
    /// the qualified class name tracebacks print
    pub class: String,
    /// the exception object itself (src/plugins keeps the python reference)
    pub obj: Box<dyn std::any::Any>,
    /// traceback.format_exception() of it, for the log
    pub traceback: Option<String>,
}

impl fmt::Debug for ForeignError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "ForeignError({}: {:?})", self.class, self.kind)
    }
}

#[derive(Clone, Debug)]
pub enum KError {
    Config(String),
    Pins(String),
    Protocol(String),
    Enumeration { msg: String, enum_name: String, value: String },
    Serial(String),
    Mcu(String),
    Command(String),
    WaitInterruption(String),
    /// (python class name, message)
    CommandSub(&'static str, String),
    Reactor(String),
    /// a builtin python exception that is not a klippy error: (class
    /// name, message), e.g. ("ValueError", "could not convert ...")
    Py(String, String),
    Other(String),
    /// an error raised while handling another: (error, its __context__).
    /// It behaves as the error; tracebacks print the context first
    Chained(Box<KError>, Box<KError>),
    Foreign(Rc<ForeignError>),
}

impl KError {
    pub fn message(&self) -> &str {
        match self {
            KError::Chained(e, _) => e.message(),
            KError::Foreign(f) => f.kind.message(),
            KError::Config(m)
            | KError::Pins(m)
            | KError::Protocol(m)
            | KError::Enumeration { msg: m, .. }
            | KError::Serial(m)
            | KError::Mcu(m)
            | KError::Command(m)
            | KError::WaitInterruption(m)
            | KError::CommandSub(_, m)
            | KError::Reactor(m)
            | KError::Py(_, m)
            | KError::Other(m) => m,
        }
    }

    /// isinstance(e, gcode.CommandError)
    pub fn is_command_error(&self) -> bool {
        matches!(self.base(), KError::Command(_) | KError::WaitInterruption(_) | KError::CommandSub(..))
    }

    /// raise self while handling `context` (python's implicit __context__)
    pub fn with_context(self, context: KError) -> KError {
        KError::Chained(Box::new(self), Box::new(context))
    }

    /// the error itself, without the exceptions it was raised while handling
    pub fn base(&self) -> &KError {
        match self {
            KError::Chained(e, _) => e.base(),
            KError::Foreign(f) => f.kind.base(),
            e => e,
        }
    }

    /// the python exception class as traceback.format_exception_only()
    /// names it ("Other" errors carry no class: "Exception")
    pub fn py_class(&self) -> &str {
        match self {
            KError::Config(_) => "configparser.Error",
            KError::Pins(_) => "klippy.pins.error",
            KError::Protocol(_) => "klippy.msgproto.error",
            KError::Enumeration { .. } => "klippy.msgproto.enumeration_error",
            KError::Serial(_) => "klippy.serialhdl.error",
            KError::Mcu(_) => "klippy.mcu.error",
            KError::Command(_) => "klippy.gcode.CommandError",
            KError::WaitInterruption(_) => "klippy.printer.WaitInterruption",
            KError::CommandSub(class, _) => class,
            KError::Reactor(_) => "klippy.reactor.ReactorError",
            KError::Py(class, _) => class,
            KError::Other(_) => "Exception",
            KError::Chained(e, _) => e.py_class(),
            KError::Foreign(f) => &f.class,
        }
    }

    /// isinstance(e, msgproto.error)
    pub fn is_protocol_error(&self) -> bool {
        matches!(self.base(), KError::Protocol(_) | KError::Enumeration { .. })
    }

    /// the OSError python raises for an io failure on `path`
    pub fn os(e: &std::io::Error, path: &str) -> Self {
        KError::Py(crate::pyfmt::py_oserror_class(e).into(), crate::pyfmt::py_oserror(e, path))
    }

    pub fn config(msg: impl Into<String>) -> Self {
        KError::Config(msg.into())
    }

    pub fn command(msg: impl Into<String>) -> Self {
        KError::Command(msg.into())
    }
}

impl fmt::Display for KError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(self.message())
    }
}

impl From<crate::configparser::Error> for KError {
    fn from(e: crate::configparser::Error) -> Self {
        match e.context {
            Some(c) => KError::Config(e.message).with_context(KError::Py(c.0, c.1)),
            None => KError::Config(e.message),
        }
    }
}

impl From<crate::pins::Error> for KError {
    fn from(e: crate::pins::Error) -> Self {
        KError::Pins(e.0)
    }
}

impl From<crate::msgproto::Error> for KError {
    fn from(e: crate::msgproto::Error) -> Self {
        match e.enum_params {
            Some((enum_name, value)) => KError::Enumeration { msg: e.msg, enum_name, value },
            None => KError::Protocol(e.msg),
        }
    }
}

impl From<crate::reactor::ReactorError> for KError {
    fn from(e: crate::reactor::ReactorError) -> Self {
        KError::Reactor(e.0)
    }
}

pub type KResult<T> = Result<T, KError>;
