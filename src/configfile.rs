// Code for reading and writing the Klipper config file
// (port of klippy/configfile.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// printer.rs wires the parts that need the printer: SAVE_CONFIG
// (ConfigFileObject::cmd_save_config), the object names for
// check_unused_options, set_rollover_info for log_config; gcode errors are
// returned as Err(String).
use crate::configparser::{self, Error, Interpolation, RawConfigParser};
use crate::logging;
use crate::mathutil;
use crate::pyfmt::{py_int, py_rstrip, py_str_repr, py_strip};
use crate::pyglob;
use indexmap::IndexMap;
use crate::pyval::PyVal;
use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

pub type Result<T> = std::result::Result<T, Error>;

pub const PYTHON_SCRIPT_PREFIX: &str = "!";

fn error(msg: impl Into<String>) -> Error {
    Error::new(msg)
}

/// pathlib.Path(p) rendered with str(): '.' parts and repeated or trailing
/// slashes dropped, '..' kept
fn pathlib_str(p: &str) -> String {
    let parts: Vec<&str> = p.split('/').filter(|s| !s.is_empty() && *s != ".").collect();
    let root = if p.starts_with('/') { "/" } else { "" };
    let out = format!("{}{}", root, parts.join("/"));
    if out.is_empty() {
        ".".into()
    } else {
        out
    }
}

/// pathlib.Path(p).parent
fn pathlib_parent(p: &str) -> String {
    let s = pathlib_str(p);
    match s.rfind('/') {
        Some(0) => "/".into(),
        Some(i) => s[..i].to_string(),
        None => ".".into(),
    }
}

/// pathlib.Path(p).absolute()
fn pathlib_absolute(p: &str) -> String {
    if p.starts_with('/') {
        pathlib_str(p)
    } else {
        pathlib_str(&format!("{}/{}", pyglob::getcwd(), p))
    }
}

/// pathlib `a / b`
fn pathlib_join(a: &str, b: &str) -> String {
    if b.starts_with('/') {
        pathlib_str(b)
    } else {
        pathlib_str(&format!("{}/{}", a, b))
    }
}

// _INCLUDERE = re.compile(r"!!include (?P<file>.*)"): search() / sub()
const INCLUDE_TAG: &str = "!!include ";

fn fix_include_path(source_file: &str, file: &str) -> Result<String> {
    let new_path = pathlib_join(&pathlib_absolute(&pathlib_parent(source_file)), file);
    if !std::path::Path::new(&new_path).is_file() {
        return Err(error(format!("Attempted to include non-existent file {}", new_path)));
    }
    Ok(format!("!!include {}", new_path))
}

pub type AccessTracking = IndexMap<(String, String), PyVal>;

/// variable interpolation replacing ${[section.]option}
pub struct SectionInterpolation {
    pub access_tracking: Rc<RefCell<AccessTracking>>,
}

/// _KEYCRE.search(): (start, end, section, option). The `(?<!\\)?` in the
/// python pattern is an optional lookbehind, so a backslash does not escape
/// (verified against python).
fn keycre_search(value: &str) -> Option<(usize, usize, Option<String>, String)> {
    let not_sect = |c: char| matches!(c, '.' | ':' | '$' | '{' | '}');
    let not_opt = |c: char| matches!(c, '$' | '{' | '}');
    let mut start = 0;
    while let Some(off) = value[start..].find("${") {
        let p = start + off;
        let body = &value[p + 2..];
        // with a section: [^.:${}]+ then [.:] then [^${}]+ then }
        let sect_len = body.find(not_sect).unwrap_or(body.len());
        if sect_len > 0 && body[sect_len..].starts_with(['.', ':']) {
            let rest = &body[sect_len + 1..];
            let opt_len = rest.find(not_opt).unwrap_or(rest.len());
            if opt_len > 0 && rest[opt_len..].starts_with('}') {
                let end = p + 2 + sect_len + 1 + opt_len + 1;
                return Some((p, end, Some(body[..sect_len].to_string()), rest[..opt_len].to_string()));
            }
        }
        // without a section: [^${}]+ then }
        let opt_len = body.find(not_opt).unwrap_or(body.len());
        if opt_len > 0 && body[opt_len..].starts_with('}') {
            return Some((p, p + 2 + opt_len + 1, None, body[..opt_len].to_string()));
        }
        start = p + 1;
    }
    None
}

impl Interpolation for SectionInterpolation {
    fn before_get(&self, parser: &RawConfigParser, section: &str, _option: &str, value: &str) -> Result<String> {
        let mut value = value.to_string();
        let mut depth = configparser::MAX_INTERPOLATION_DEPTH;
        while depth > 0 {
            depth -= 1;
            let Some((start, end, sect, opt)) = keycre_search(&value) else { break };
            let sect = sect.filter(|s| !s.is_empty()).unwrap_or_else(|| section.to_string());
            let constant = parser.get(&sect, &opt)?;
            self.access_tracking.borrow_mut().entry((sect, opt)).or_insert_with(|| PyVal::Str(constant.clone()));
            value = format!("{}{}{}", &value[..start], constant, &value[end..]);
        }
        Ok(value.replace("\\${", "${"))
    }
}

/// The default of a ConfigWrapper getter: required (python's sentinel),
/// None, or a value (kept with its python type for access tracking)
#[derive(Clone, Debug)]
pub enum Dflt {
    Required,
    None,
    Val(PyVal),
}

pub const REQUIRED: Dflt = Dflt::Required;

impl From<f64> for Dflt {
    fn from(v: f64) -> Self {
        Dflt::Val(PyVal::from(v))
    }
}
impl From<i64> for Dflt {
    fn from(v: i64) -> Self {
        Dflt::Val(PyVal::from(v))
    }
}
impl From<i32> for Dflt {
    fn from(v: i32) -> Self {
        Dflt::Val(PyVal::from(v))
    }
}
impl From<bool> for Dflt {
    fn from(v: bool) -> Self {
        Dflt::Val(PyVal::from(v))
    }
}
impl From<&str> for Dflt {
    fn from(v: &str) -> Self {
        Dflt::Val(PyVal::from(v))
    }
}
impl From<String> for Dflt {
    fn from(v: String) -> Self {
        Dflt::Val(PyVal::from(v))
    }
}
impl From<PyVal> for Dflt {
    fn from(v: PyVal) -> Self {
        Dflt::Val(v)
    }
}

/// minval / maxval / above / below / note_valid of the getters
#[derive(Clone, Debug)]
pub struct Lim {
    pub minval: Option<PyVal>,
    pub maxval: Option<PyVal>,
    pub above: Option<PyVal>,
    pub below: Option<PyVal>,
    pub note_valid: bool,
}

impl Default for Lim {
    fn default() -> Self {
        Lim { minval: None, maxval: None, above: None, below: None, note_valid: true }
    }
}

impl Lim {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn min(mut self, v: impl Into<PyVal>) -> Self {
        self.minval = Some(v.into());
        self
    }
    pub fn max(mut self, v: impl Into<PyVal>) -> Self {
        self.maxval = Some(v.into());
        self
    }
    pub fn above(mut self, v: impl Into<PyVal>) -> Self {
        self.above = Some(v.into());
        self
    }
    pub fn below(mut self, v: impl Into<PyVal>) -> Self {
        self.below = Some(v.into());
        self
    }
    pub fn no_note(mut self) -> Self {
        self.note_valid = false;
        self
    }
}

/// value parsers of getlists()
#[derive(Clone, Copy, Debug)]
pub enum ListParser {
    Str,
    Int,
    SafeFloat,
    /// python float()
    Float,
}

// a getter's parse failure: a configparser error is re-raised as is, any
// other exception (class, message) becomes "Unable to parse option"
enum ParseErr {
    Config(Error),
    Other(&'static str, String),
}

impl From<std::result::Result<Error, String>> for ParseErr {
    fn from(e: std::result::Result<Error, String>) -> Self {
        match e {
            Ok(e) => ParseErr::Config(e),
            // the conversions raise ValueError
            Err(msg) => ParseErr::Other("ValueError", msg),
        }
    }
}

fn value_error(msg: String) -> ParseErr {
    ParseErr::Other("ValueError", msg)
}

type Deprecated = IndexMap<(String, String, Option<String>), String>;

#[derive(Clone)]
pub struct ConfigWrapper {
    pub fileconfig: Rc<RefCell<RawConfigParser>>,
    pub access_tracking: Rc<RefCell<AccessTracking>>,
    pub section: String,
    // PrinterConfig.deprecated (python reaches it via the printer)
    deprecated: Rc<RefCell<Deprecated>>,
    // weak: the printer owns the configfile object holding wrappers
    printer: std::rc::Weak<crate::printer::Printer>,
}

fn num(v: &PyVal) -> f64 {
    v.as_f64().unwrap_or(f64::NAN)
}

impl ConfigWrapper {
    pub fn get_name(&self) -> &str {
        &self.section
    }

    /// config.fileconfig.has_option(config.section, option)
    pub fn fileconfig_has_option(&self, option: &str) -> bool {
        self.fileconfig.borrow().has_option(&self.section, option)
    }

    fn get_wrapper(
        &self,
        parser: impl FnOnce(&RawConfigParser) -> std::result::Result<PyVal, ParseErr>,
        option: &str,
        default: &Dflt,
        lim: &Lim,
    ) -> Result<Option<PyVal>> {
        if !self.fileconfig.borrow().has_option(&self.section, option) {
            return match default {
                Dflt::Required => Err(error(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Dflt::None => Ok(None),
                Dflt::Val(v) => {
                    if lim.note_valid {
                        let acc_id = (self.section.to_lowercase(), option.to_lowercase());
                        self.access_tracking.borrow_mut().insert(acc_id, v.clone());
                    }
                    Ok(Some(v.clone()))
                }
            };
        }
        let v = match parser(&self.fileconfig.borrow()) {
            Ok(v) => v,
            Err(ParseErr::Config(e)) => return Err(e),
            Err(ParseErr::Other(cls, msg)) => {
                return Err(error(format!("Unable to parse option '{}' in section '{}'", option, self.section)).with_context(cls, &msg))
            }
        };
        if lim.note_valid {
            self.access_tracking
                .borrow_mut()
                .insert((self.section.to_lowercase(), option.to_lowercase()), v.clone());
        }
        let f = num(&v);
        let check = |bound: &Option<PyVal>, bad: &dyn Fn(f64, f64) -> bool, what: &str| -> Result<()> {
            if let Some(b) = bound {
                if bad(f, num(b)) {
                    return Err(error(format!(
                        "Option '{}' in section '{}' must {} {}",
                        option,
                        self.section,
                        what,
                        b.str()
                    )));
                }
            }
            Ok(())
        };
        check(&lim.minval, &|v, b| v < b, "have minimum of")?;
        check(&lim.maxval, &|v, b| v > b, "have maximum of")?;
        check(&lim.above, &|v, b| v <= b, "be above")?;
        check(&lim.below, &|v, b| v >= b, "be below")?;
        Ok(Some(v))
    }

    fn get_str_value(&self, option: &str, default: &Dflt, note_valid: bool) -> Result<Option<PyVal>> {
        let lim = Lim { note_valid, ..Lim::default() };
        let section = self.section.clone();
        self.get_wrapper(|fc| Ok(PyVal::Str(fc.get(&section, option).map_err(ParseErr::Config)?)), option, default, &lim)
    }

    // ------------------------------------------------------------ getters

    /// config.get(option, default); default must not be None (see get_opt)
    pub fn get(&self, option: &str, default: impl Into<Dflt>) -> Result<String> {
        let v = self.get_str_value(option, &default.into(), true)?;
        Ok(v.map(|v| v.str()).unwrap_or_default())
    }

    /// config.get(option, None)
    pub fn get_opt(&self, option: &str) -> Result<Option<String>> {
        Ok(self.get_str_value(option, &Dflt::None, true)?.map(|v| v.str()))
    }

    /// config.get(option, default, note_valid=...) returning the raw value
    /// (a non-string default comes back as given)
    pub fn get_value(&self, option: &str, default: impl Into<Dflt>, note_valid: bool) -> Result<Option<PyVal>> {
        self.get_str_value(option, &default.into(), note_valid)
    }

    pub fn getscript(&self, option: &str, default: impl Into<Dflt>) -> Result<(&'static str, String)> {
        let value = self.get(option, default)?;
        let value = py_strip(&value);
        if let Some(pos) = value.find(INCLUDE_TAG) {
            let file_path = &value[pos + INCLUDE_TAG.len()..];
            let text = std::fs::read_to_string(file_path)
                .map_err(|e| error(crate::pyfmt::py_oserror(&e, file_path)))?;
            let is_py = std::path::Path::new(file_path)
                .extension()
                .map(|e| e.to_string_lossy().to_lowercase() == "py")
                .unwrap_or(false);
            return Ok((if is_py { "python" } else { "gcode" }, text));
        }
        if value.starts_with(PYTHON_SCRIPT_PREFIX) {
            let lines: Vec<&str> = py_splitlines(value).into_iter().map(|l| l.strip_prefix(PYTHON_SCRIPT_PREFIX).unwrap_or(l)).collect();
            return Ok(("python", lines.join("\n")));
        }
        Ok(("gcode", value.to_string()))
    }

    pub fn getint_value(&self, option: &str, default: &Dflt, lim: &Lim) -> Result<Option<PyVal>> {
        let section = self.section.clone();
        self.get_wrapper(|fc| Ok(PyVal::from(fc.getint(&section, option)?)), option, default, lim)
    }

    pub fn getint(&self, option: &str, default: impl Into<Dflt>, lim: Lim) -> Result<i64> {
        let v = self.getint_value(option, &default.into(), &lim)?;
        Ok(v.map(|v| int_of(&v)).unwrap_or(0))
    }

    pub fn getint_opt(&self, option: &str, lim: Lim) -> Result<Option<i64>> {
        Ok(self.getint_value(option, &Dflt::None, &lim)?.map(|v| int_of(&v)))
    }

    /// getfloat() returning the python object: a default comes back unchanged,
    /// so an int default stays an int (matters where the value is printed)
    pub fn getfloat_value(&self, option: &str, default: &Dflt, lim: &Lim) -> Result<Option<PyVal>> {
        let section = self.section.clone();
        // fileconfig.getfloat is python float(): "inf" and "nan" are accepted
        self.get_wrapper(|fc| Ok(float_value(fc.getfloat(&section, option)?)), option, default, lim)
    }

    pub fn getfloat(&self, option: &str, default: impl Into<Dflt>, lim: Lim) -> Result<f64> {
        let v = self.getfloat_value(option, &default.into(), &lim)?;
        Ok(v.map(|v| num(&v)).unwrap_or(0.))
    }

    pub fn getfloat_opt(&self, option: &str, lim: Lim) -> Result<Option<f64>> {
        Ok(self.getfloat_value(option, &Dflt::None, &lim)?.map(|v| num(&v)))
    }

    pub fn getboolean_value(&self, option: &str, default: &Dflt, note_valid: bool) -> Result<Option<PyVal>> {
        let lim = Lim { note_valid, ..Lim::default() };
        let section = self.section.clone();
        self.get_wrapper(|fc| Ok(PyVal::from(fc.getboolean(&section, option)?)), option, default, &lim)
    }

    pub fn getboolean(&self, option: &str, default: impl Into<Dflt>) -> Result<bool> {
        let v = self.getboolean_value(option, &default.into(), true)?;
        Ok(v.map(|v| truthy(&v)).unwrap_or(false))
    }

    /// getboolean(option, default, note_valid=False)
    pub fn getboolean_value_nv(&self, option: &str, default: impl Into<Dflt>) -> Result<bool> {
        let v = self.getboolean_value(option, &default.into(), false)?;
        Ok(v.map(|v| truthy(&v)).unwrap_or(false))
    }

    pub fn getboolean_opt(&self, option: &str) -> Result<Option<bool>> {
        Ok(self.getboolean_value(option, &Dflt::None, true)?.map(|v| truthy(&v)))
    }

    /// getchoice() with string keys (python list or str-keyed dict)
    pub fn getchoice<T: Clone>(&self, option: &str, choices: &[(&str, T)], default: impl Into<Dflt>) -> Result<T> {
        let c = self.get_str_value(option, &default.into(), true)?;
        let key = c.as_ref().map(|v| v.str()).unwrap_or_else(|| "None".into());
        choices.iter().find(|(k, _)| c.is_some() && *k == key).map(|(_, v)| v.clone()).ok_or_else(|| {
            error(format!("Choice '{}' for option '{}' in section '{}' is not a valid choice", key, option, self.section))
        })
    }

    /// getchoice(option, [None, "a", ...], None): a missing option is None
    pub fn getchoice_opt(&self, option: &str, choices: &[&str]) -> Result<Option<String>> {
        let Some(c) = self.get_opt(option)? else { return Ok(None) };
        if !choices.contains(&c.as_str()) {
            return Err(error(format!(
                "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                c, option, self.section
            )));
        }
        Ok(Some(c))
    }

    /// getchoice() with int keys
    pub fn getchoice_int<T: Clone>(&self, option: &str, choices: &[(i64, T)], default: impl Into<Dflt>) -> Result<T> {
        let c = self.getint_value(option, &default.into(), &Lim::default())?;
        let key = c.as_ref().map(|v| num(v) as i64);
        choices.iter().find(|(k, _)| Some(*k) == key).map(|(_, v)| v.clone()).ok_or_else(|| {
            let shown = c.as_ref().map(|v| v.str()).unwrap_or_else(|| "None".into());
            error(format!("Choice '{}' for option '{}' in section '{}' is not a valid choice", shown, option, self.section))
        })
    }

    /// getlists(): nested tuples (json arrays) of parsed values
    pub fn getlists(
        &self,
        option: &str,
        default: impl Into<Dflt>,
        seps: &[&str],
        count: Option<usize>,
        parser: ListParser,
        note_valid: bool,
    ) -> Result<Option<PyVal>> {
        let section = self.section.clone();
        let lim = Lim { note_valid, ..Lim::default() };
        let lparse_err = |what: ParseErr| what;
        self.get_wrapper(
            |fc| {
                let value = fc.get(&section, option).map_err(ParseErr::Config)?;
                lparser(&value, seps.len() - 1, seps, count, parser, option, &section).map_err(lparse_err)
            },
            option,
            &default.into(),
            &lim,
        )
    }

    pub fn getlist(&self, option: &str, default: impl Into<Dflt>, sep: &str, count: Option<usize>) -> Result<Option<Vec<String>>> {
        let v = self.getlists(option, default, &[sep], count, ListParser::Str, true)?;
        Ok(v.map(|v| v.items().map(|a| a.iter().map(|v| v.str()).collect()).unwrap_or_default()))
    }

    pub fn getintlist(&self, option: &str, default: impl Into<Dflt>, sep: &str, count: Option<usize>) -> Result<Option<Vec<i64>>> {
        let v = self.getlists(option, default, &[sep], count, ListParser::Int, true)?;
        Ok(v.map(|v| v.items().map(|a| a.iter().map(int_of).collect()).unwrap_or_default()))
    }

    pub fn getfloatlist(&self, option: &str, default: impl Into<Dflt>, sep: &str, count: Option<usize>) -> Result<Option<Vec<f64>>> {
        let v = self.getlists(option, default, &[sep], count, ListParser::SafeFloat, true)?;
        Ok(v.map(|v| v.items().map(|a| a.iter().map(num).collect()).unwrap_or_default()))
    }

    // ------------------------------------------------------------ sections

    pub fn getsection(&self, section: &str) -> ConfigWrapper {
        ConfigWrapper {
            fileconfig: self.fileconfig.clone(),
            access_tracking: self.access_tracking.clone(),
            section: section.to_string(),
            deprecated: self.deprecated.clone(),
            printer: self.printer.clone(),
        }
    }

    /// a wrapper of one section whose options are `contents` (the indx
    /// compat.ConfigWrapper): own access tracking, same printer
/// ConfigWrapper(printer, fileconfig, access_tracking, section) built by a
    /// python plugin (src/plugins): the deprecation list is the printer's
    pub fn from_parts(
        printer: &Rc<crate::printer::Printer>,
        fileconfig: Rc<RefCell<RawConfigParser>>,
        access_tracking: Rc<RefCell<AccessTracking>>,
        section: &str,
    ) -> ConfigWrapper {
        let deprecated = printer
            .pconfig
            .borrow()
            .as_ref()
            .map(|c| c.pconfig.borrow().deprecated.clone())
            .unwrap_or_default();
        ConfigWrapper { fileconfig, access_tracking, section: section.to_string(), deprecated, printer: Rc::downgrade(printer) }
    }

    /// An empty parser like python's RawConfigParser() (plugins build their own)
    pub fn empty_fileconfig() -> RawConfigParser {
        RawConfigParser::new(true, &[], Box::new(crate::configparser::NoInterpolation))
    }

    pub fn synthetic(&self, section: &str, contents: &[(&str, String)]) -> ConfigWrapper {
        let mut fc = RawConfigParser::new(true, &[], Box::new(crate::configparser::NoInterpolation));
        let _ = fc.add_section(section);
        for (option, value) in contents {
            let _ = fc.set(section, option, value);
        }
        ConfigWrapper {
            fileconfig: Rc::new(RefCell::new(fc)),
            access_tracking: Rc::new(RefCell::new(AccessTracking::new())),
            section: section.to_string(),
            deprecated: self.deprecated.clone(),
            printer: self.printer.clone(),
        }
    }

    /// this wrapper (and the sections taken from it) with get_printer()
    pub fn with_printer(mut self, printer: &Rc<crate::printer::Printer>) -> Self {
        self.printer = Rc::downgrade(printer);
        self
    }

    pub fn get_printer(&self) -> Rc<crate::printer::Printer> {
        self.printer.upgrade().expect("config wrapper without printer")
    }

    pub fn has_section(&self, section: &str) -> bool {
        self.fileconfig.borrow().has_section(section)
    }

    pub fn get_prefix_sections(&self, prefix: &str) -> Vec<ConfigWrapper> {
        let sections = self.fileconfig.borrow().sections();
        sections.iter().filter(|s| s.starts_with(prefix)).map(|s| self.getsection(s)).collect()
    }

    pub fn get_prefix_options(&self, prefix: &str) -> Result<Vec<String>> {
        let opts = self.fileconfig.borrow().options(&self.section)?;
        Ok(opts.into_iter().filter(|o| o.starts_with(prefix)).collect())
    }

    pub fn deprecate(&self, option: &str, value: Option<&str>) {
        if !self.fileconfig.borrow().has_option(&self.section, option) {
            return;
        }
        let msg = match value {
            None => format!("Option '{}' in section '{}' is deprecated.", option, self.section),
            Some(v) => format!("PyVal '{}' in option '{}' in section '{}' is deprecated.", v, option, self.section),
        };
        self.deprecated
            .borrow_mut()
            .insert((self.section.clone(), option.to_string(), value.map(|v| v.to_string())), msg);
    }
}

fn int_of(v: &PyVal) -> i64 {
    v.as_i64().unwrap_or(0)
}

fn truthy(v: &PyVal) -> bool {
    v.truthy()
}

fn float_value(f: f64) -> PyVal {
    PyVal::Float(f)
}

/// str.splitlines() for the line breaks config text can hold
fn py_splitlines(s: &str) -> Vec<&str> {
    let mut out = vec![];
    let mut rest = s;
    while !rest.is_empty() {
        match rest.find(['\n', '\r', '\x0b', '\x0c', '\x1c', '\x1d', '\x1e', '\u{85}', '\u{2028}', '\u{2029}']) {
            None => {
                out.push(rest);
                break;
            }
            Some(i) => {
                out.push(&rest[..i]);
                let c = rest[i..].chars().next().unwrap();
                let mut n = c.len_utf8();
                if c == '\r' && rest[i + 1..].starts_with('\n') {
                    n += 1;
                }
                rest = &rest[i + n..];
            }
        }
    }
    out
}

fn lparser(
    value: &str,
    pos: usize,
    seps: &[&str],
    count: Option<usize>,
    parser: ListParser,
    option: &str,
    section: &str,
) -> std::result::Result<PyVal, ParseErr> {
    let parts: Vec<&str> = if py_strip(value).is_empty() {
        // Return an empty list instead of [''] for empty string
        vec![]
    } else {
        value.split(seps[pos]).map(py_strip).collect()
    };
    if pos > 0 {
        // Nested list
        let mut out = vec![];
        for p in parts.into_iter().filter(|p| !p.is_empty()) {
            out.push(lparser(p, pos - 1, seps, count, parser, option, section)?);
        }
        return Ok(PyVal::Tuple(out));
    }
    let mut res = vec![];
    for p in &parts {
        res.push(match parser {
            ListParser::Str => PyVal::Str(p.to_string()),
            ListParser::Int => PyVal::from(py_int(p).ok_or_else(|| value_error(format!("invalid literal for int() with base 10: {}", py_str_repr(p))))?),
            ListParser::SafeFloat => PyVal::from(mathutil::safe_float(p).map_err(value_error)?),
            ListParser::Float => PyVal::from(crate::pyfmt::py_float(p).ok_or_else(|| value_error(format!("could not convert string to float: {}", py_str_repr(p))))?),
        });
    }
    if let Some(count) = count {
        if res.len() != count {
            return Err(ParseErr::Config(error(format!(
                "Option '{}' in section '{}' must have {} elements",
                option, section, count
            ))));
        }
    }
    Ok(PyVal::Tuple(res))
}

pub const AUTOSAVE_HEADER: &str = "
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
";

pub struct PrinterConfig {
    pub config_file: String,
    pub autosave: Option<ConfigWrapper>,
    pub deprecated: Rc<RefCell<Deprecated>>,
    pub runtime_warnings: Vec<PyVal>,
    pub deprecate_warnings: Vec<PyVal>,
    pub status_raw_config: IndexMap<String, PyVal>,
    pub status_save_pending: IndexMap<String, PyVal>,
    pub status_settings: IndexMap<String, PyVal>,
    pub status_warnings: Vec<PyVal>,
    pub unused_sections: Vec<String>,
    pub unused_options: Vec<(String, String)>,
    pub save_config_pending: bool,
}

impl PrinterConfig {
    /// `config_file` is start_args["config_file"]
    pub fn new(config_file: &str) -> Self {
        PrinterConfig {
            config_file: config_file.to_string(),
            autosave: None,
            deprecated: Rc::new(RefCell::new(IndexMap::new())),
            runtime_warnings: vec![],
            deprecate_warnings: vec![],
            status_raw_config: IndexMap::new(),
            status_save_pending: IndexMap::new(),
            status_settings: IndexMap::new(),
            status_warnings: vec![],
            unused_sections: vec![],
            unused_options: vec![],
            save_config_pending: false,
        }
    }

    fn read_config_file(&self, filename: &str) -> Result<String> {
        // open(filename, "r"): utf-8, universal newlines
        let res = match std::fs::read(filename) {
            Ok(b) => String::from_utf8(b).map_err(|e| ("UnicodeDecodeError", crate::pyfmt::py_utf8_error(e.as_bytes(), &e.utf8_error()))),
            Err(e) => Err((crate::pyfmt::py_oserror_class(&e), crate::pyfmt::py_oserror(&e, filename))),
        };
        match res {
            Ok(data) => Ok(data.replace("\r\n", "\n").replace('\r', "\n")),
            Err((cls, emsg)) => {
                let msg = format!("Unable to open config file {}", filename);
                logging::exception_tb(&msg, cls, &emsg);
                Err(error(msg).with_context(cls, &emsg))
            }
        }
    }

    pub fn find_autosave_data(&self, data: &str) -> (String, String) {
        let mut regular_data = data.to_string();
        let mut autosave_data = String::new();
        if let Some(pos) = data.find(AUTOSAVE_HEADER) {
            regular_data = data[..pos].to_string();
            autosave_data = py_strip(&data[pos + AUTOSAVE_HEADER.len()..]).to_string();
        }
        // Check for errors and strip line prefixes
        if regular_data.contains("\n#*# ") {
            logging::warning("Can't read autosave from config file - autosave state corrupted");
            return (data.to_string(), String::new());
        }
        let mut out = vec![String::new()];
        for line in autosave_data.split('\n') {
            if (!line.starts_with("#*#") || (line.chars().count() >= 4 && !line.starts_with("#*# "))) && !autosave_data.is_empty() {
                logging::warning("Can't read autosave from config file - modifications after header");
                return (data.to_string(), String::new());
            }
            out.push(line.chars().skip(4).collect());
        }
        out.push(String::new());
        (regular_data, out.join("\n"))
    }

    // comment_r = "[#;].*$", value_r = "[^A-Za-z0-9_].*$"
    pub fn strip_duplicates(&self, data: &str, config: &ConfigWrapper) -> String {
        let mut lines: Vec<String> = data.split('\n').map(|s| s.to_string()).collect();
        let mut section: Option<String> = None;
        let mut is_dup_field = false;
        for line in lines.iter_mut() {
            let pruned = match line.find(['#', ';']) {
                Some(p) => &line[..p],
                None => &line[..],
            };
            let pruned_line = py_rstrip(pruned).to_string();
            let Some(first) = pruned_line.chars().next() else { continue };
            if crate::pyfmt::py_isspace(first) {
                if is_dup_field {
                    line.insert(0, '#');
                }
                continue;
            }
            is_dup_field = false;
            if first == '[' {
                let inner: String = pruned_line.chars().skip(1).take(pruned_line.chars().count().saturating_sub(2)).collect();
                section = Some(py_strip(&inner).to_string());
                continue;
            }
            let field = match pruned_line.find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')) {
                Some(p) => &pruned_line[..p],
                None => &pruned_line[..],
            };
            if config.fileconfig.borrow().has_option(section.as_deref().unwrap_or(""), field) {
                is_dup_field = true;
                line.insert(0, '#');
            }
        }
        lines.join("\n")
    }

    fn parse_config_buffer(&self, buffer: &mut Vec<String>, filename: &str, fileconfig: &RefCell<RawConfigParser>) -> Result<()> {
        if buffer.is_empty() {
            return Ok(());
        }
        let data = buffer.join("\n");
        buffer.clear();
        fileconfig.borrow_mut().read_string(&data, filename)
    }

    fn resolve_include(
        &self,
        source_filename: &str,
        include_spec: &str,
        fileconfig: &RefCell<RawConfigParser>,
        visited: &mut HashSet<String>,
        include_vars: &IndexMap<String, String>,
    ) -> Result<Vec<String>> {
        let dirname = pyglob::dirname(source_filename);
        let include_spec = py_strip(include_spec);
        let include_glob = pyglob::join(&dirname, &[include_spec]);
        let mut include_filenames = pyglob::glob(&include_glob, true);
        if include_filenames.is_empty() && !pyglob::has_magic(&include_glob) {
            // Empty set is OK if wildcard but not for direct file reference
            return Err(error(format!("Include file '{}' does not exist", include_glob)));
        }
        include_filenames.sort();
        for include_filename in &include_filenames {
            let mut include_data = self.read_config_file(include_filename)?;
            if !include_vars.is_empty() {
                include_data = render_include(&include_data, include_filename, include_vars)?;
            }
            self.parse_config(&include_data, include_filename, fileconfig, visited, include_vars)?;
        }
        Ok(include_filenames)
    }

    fn parse_config(
        &self,
        data: &str,
        filename: &str,
        fileconfig: &RefCell<RawConfigParser>,
        visited: &mut HashSet<String>,
        include_vars: &IndexMap<String, String>,
    ) -> Result<()> {
        let path = pyglob::abspath(filename);
        if visited.contains(&path) {
            return Err(error(format!("Recursive include of config file '{}'", filename)));
        }
        visited.insert(path.clone());
        // Buffer lines between includes and parse as a unit so that overrides
        // in includes apply linearly as they do within a single file
        let mut buffer: Vec<String> = vec![];
        let mut pending_include: Option<String> = None;
        let mut pending_include_vars = include_vars.clone();
        for line in data.split('\n') {
            // Strip trailing comment
            let line = match line.find('#') {
                Some(pos) => &line[..pos],
                None => line,
            };
            // Process include or buffer line
            let header = configparser::match_sectcre(line);
            if let Some(spec) = pending_include.clone() {
                // if we hit another header, then process the include
                if header.is_some() {
                    self.parse_config_buffer(&mut buffer, filename, fileconfig)?;
                    self.resolve_include(filename, &spec, fileconfig, visited, &pending_include_vars)?;
                    pending_include = None;
                    pending_include_vars = include_vars.clone();
                } else {
                    // parse variable line
                    if let Some((opt, val)) = configparser::match_optcre(line) {
                        pending_include_vars.insert(opt, py_strip(&val).to_string());
                    } else if !py_strip(line).is_empty() {
                        return Err(error(format!("Invalid include variable '{}' in '{}'", py_strip(line), filename)));
                    }
                    continue;
                }
            }
            match header.as_deref().and_then(|h| h.strip_prefix("include ")) {
                Some(spec) => pending_include = Some(py_strip(spec).to_string()),
                None => {
                    let line = match line.find(INCLUDE_TAG) {
                        Some(p) => format!("{}{}", &line[..p], fix_include_path(filename, &line[p + INCLUDE_TAG.len()..])?),
                        None => line.to_string(),
                    };
                    buffer.push(line);
                }
            }
        }
        self.parse_config_buffer(&mut buffer, filename, fileconfig)?;
        if let Some(spec) = pending_include {
            self.resolve_include(filename, &spec, fileconfig, visited, &pending_include_vars)?;
        }
        visited.remove(&path);
        Ok(())
    }

    pub fn build_config_wrapper(&self, data: &str, filename: &str) -> Result<ConfigWrapper> {
        let access_tracking = Rc::new(RefCell::new(AccessTracking::new()));
        let interp = SectionInterpolation { access_tracking: access_tracking.clone() };
        let fileconfig = RefCell::new(RawConfigParser::new(false, &[";", "#"], Box::new(interp)));
        self.parse_config(data, filename, &fileconfig, &mut HashSet::new(), &IndexMap::new())?;
        Ok(ConfigWrapper {
            fileconfig: Rc::new(fileconfig),
            access_tracking,
            section: "printer".into(),
            deprecated: self.deprecated.clone(),
            printer: std::rc::Weak::new(),
        })
    }

    pub fn build_config_string(&self, config: &ConfigWrapper) -> String {
        py_strip(&config.fileconfig.borrow().write()).to_string()
    }

    pub fn read_config(&self, filename: &str) -> Result<ConfigWrapper> {
        let data = self.read_config_file(filename)?;
        self.build_config_wrapper(&data, filename)
    }

    pub fn read_main_config(&mut self) -> Result<ConfigWrapper> {
        let filename = self.config_file.clone();
        let data = self.read_config_file(&filename)?;
        let (regular_data, autosave_data) = self.find_autosave_data(&data);
        let regular_config = self.build_config_wrapper(&regular_data, &filename)?;
        let autosave_data = self.strip_duplicates(&autosave_data, &regular_config);
        self.autosave = Some(self.build_config_wrapper(&autosave_data, &filename)?);
        self.build_config_wrapper(&format!("{}{}", regular_data, autosave_data), &filename)
    }

    /// `objects`: the names from printer.lookup_objects()
    pub fn check_unused_options(&mut self, config: &ConfigWrapper, error_on_unused: bool, objects: &[String]) -> Result<()> {
        let fileconfig = config.fileconfig.borrow();
        // Determine all the fields that have been accessed
        let mut access_tracking = config.access_tracking.borrow().clone();
        let autosave = self.autosave.as_ref().expect("read_main_config first").fileconfig.clone();
        for section in autosave.borrow().sections() {
            for option in autosave.borrow().options(&section)? {
                access_tracking.insert((section.to_lowercase(), option.to_lowercase()), PyVal::from(1));
            }
        }
        // Validate that there are no undefined parameters in the config file
        let valid_sections: HashSet<&String> = access_tracking.keys().map(|(s, _)| s).collect();
        for section_name in fileconfig.sections() {
            let section = section_name.to_lowercase();
            if !valid_sections.contains(&section) && !objects.contains(&section) {
                if error_on_unused {
                    return Err(error(format!("Section '{}' is not a valid config section", section)));
                }
                self.unused_sections.push(section.clone());
            }
            for option in fileconfig.options(&section_name)? {
                let option = option.to_lowercase();
                if !access_tracking.contains_key(&(section.clone(), option.clone())) {
                    if error_on_unused && section != "constants" {
                        return Err(error(format!("Option '{}' is not valid in section '{}'", option, section)));
                    }
                    self.unused_options.push((section.clone(), option));
                }
            }
        }
        drop(fileconfig);
        // Setup get_status()
        self.build_status(config)
    }

    /// the text python hands to set_rollover_info("config", ...)
    pub fn log_config(&self, config: &ConfigWrapper) -> String {
        ["===== Config file =====", &self.build_config_string(config), "======================="].join("\n")
    }

    // Status reporting
    pub fn runtime_warning(&mut self, msg: &str) {
        logging::warning(msg);
        let mut res = IndexMap::new();
        res.insert("type".to_string(), PyVal::from("runtime_warning"));
        res.insert("message".to_string(), PyVal::from(msg));
        self.runtime_warnings.push(PyVal::Dict(res));
        self.status_warnings = self.runtime_warnings.iter().chain(&self.deprecate_warnings).cloned().collect();
    }

    pub fn deprecate(&self, section: &str, option: &str, value: Option<&str>, msg: &str) {
        self.deprecated
            .borrow_mut()
            .insert((section.to_string(), option.to_string(), value.map(|v| v.to_string())), msg.to_string());
    }

    pub fn warn(&mut self, type_: &str, msg: &str, section: Option<&str>, option: Option<&str>, value: Option<&str>) {
        let mut res = IndexMap::new();
        res.insert("type".to_string(), PyVal::from(type_));
        res.insert("message".to_string(), PyVal::from(msg));
        if let Some(s) = section {
            res.insert("section".to_string(), PyVal::from(s));
        }
        if let Some(o) = option {
            res.insert("option".to_string(), PyVal::from(o));
        }
        if let Some(v) = value {
            res.insert("value".to_string(), PyVal::from(v));
        }
        self.status_warnings.push(PyVal::Dict(res));
    }

    fn build_status(&mut self, config: &ConfigWrapper) -> Result<()> {
        self.status_raw_config.clear();
        for section in config.get_prefix_sections("") {
            let mut section_status = IndexMap::new();
            for option in section.get_prefix_options("")? {
                let v = section.get_value(&option, REQUIRED, false)?.unwrap_or(PyVal::None);
                section_status.insert(option, v);
            }
            self.status_raw_config.insert(section.get_name().to_string(), PyVal::Dict(section_status));
        }
        self.status_settings = IndexMap::new();
        for ((section, option), value) in config.access_tracking.borrow().iter() {
            if let PyVal::Dict(d) = self.status_settings.entry(section.clone()).or_insert_with(PyVal::dict) {
                d.insert(option.clone(), value.clone());
            }
        }
        let deprecated: Vec<_> = self.deprecated.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        for ((section, option, value), msg) in deprecated {
            self.warn("deprecated_value", &msg, Some(&section), Some(&option), value.as_deref());
        }
        for (section, option) in self.unused_options.clone() {
            let msg = if section == "constants" {
                format!("Constant '{}' is unused", option)
            } else {
                format!("Option '{}' in section '{}' is invalid", option, section)
            };
            self.warn("unused_option", &msg, Some(&section), Some(&option), None);
        }
        for section in self.unused_sections.clone() {
            self.warn("unused_section", &format!("Section '{}' is invalid", section), Some(&section), None, None);
        }
        Ok(())
    }

    pub fn get_status(&self, _eventtime: f64) -> PyVal {
        let mut d = IndexMap::new();
        d.insert("config".to_string(), PyVal::Dict(self.status_raw_config.clone()));
        d.insert("settings".to_string(), PyVal::Dict(self.status_settings.clone()));
        d.insert("warnings".to_string(), PyVal::List(self.status_warnings.clone()));
        d.insert("save_config_pending".to_string(), PyVal::Bool(self.save_config_pending));
        d.insert("save_config_pending_items".to_string(), PyVal::Dict(self.status_save_pending.clone()));
        PyVal::Dict(d)
    }

    // Autosave functions
    /// set(section, option, value): `value` is python str(value)
    pub fn set(&mut self, section: &str, option: &str, svalue: &str) -> Result<()> {
        let autosave = self.autosave.as_ref().expect("read_main_config first").fileconfig.clone();
        if !autosave.borrow().has_section(section) {
            autosave.borrow_mut().add_section(section)?;
        }
        autosave.borrow_mut().set(section, option, svalue)?;
        let mut pending = self.status_save_pending.clone();
        let mut sect = match pending.get(section) {
            Some(PyVal::Dict(m)) => m.clone(),
            _ => IndexMap::new(),
        };
        sect.insert(option.to_string(), PyVal::from(svalue));
        pending.insert(section.to_string(), PyVal::Dict(sect));
        self.status_save_pending = pending;
        self.save_config_pending = true;
        logging::info(&format!("save_config: set [{}] {} = {}", section, option, svalue));
        Ok(())
    }

    pub fn remove_section(&mut self, section: &str) {
        let autosave = self.autosave.as_ref().expect("read_main_config first").fileconfig.clone();
        if autosave.borrow().has_section(section) {
            autosave.borrow_mut().remove_section(section);
            let mut pending = self.status_save_pending.clone();
            pending.insert(section.to_string(), PyVal::None);
            self.status_save_pending = pending;
            self.save_config_pending = true;
        } else if matches!(self.status_save_pending.get(section), Some(v) if *v != PyVal::None) {
            let mut pending = self.status_save_pending.clone();
            pending.shift_remove(section);
            self.status_save_pending = pending;
            self.save_config_pending = true;
        }
    }

    /// Err(msg) where python raises gcode.error(msg)
    fn disallow_include_conflicts(&self, regular_data: &str, cfgname: &str) -> std::result::Result<(), String> {
        let config = self.build_config_wrapper(regular_data, cfgname).map_err(|e| e.message)?;
        let autosave = self.autosave.as_ref().unwrap().fileconfig.borrow();
        let fc = config.fileconfig.borrow();
        for section in autosave.sections() {
            for option in autosave.options(&section).map_err(|e| e.message)? {
                if fc.has_option(&section, &option) {
                    // They conflict only if they are not the same value
                    let included_value = fc.get(&section, &option).map_err(|e| e.message)?;
                    let autosave_value = autosave.get(&section, &option).map_err(|e| e.message)?;
                    if included_value != autosave_value {
                        return Err(format!(
                            "SAVE_CONFIG section '{}' option '{}' value '{}' conflicts with included value '{}' ",
                            section, option, autosave_value, included_value
                        ));
                    }
                }
            }
        }
        Ok(())
    }

    fn write_backup(&self, cfgpath: &str, cfgdata: &str) -> std::result::Result<(), String> {
        let printercfg = &self.config_file;
        let configdir = pyglob::dirname(printercfg);
        // Define a directory for configuration backups so that include blocks
        // using a wildcard to reference all files in a directory don't throw
        // errors
        let backupdir = pyglob::join(&configdir, &["config_backups"]);
        // Create the backup directory if it doesn't already exist
        if !std::path::Path::new(&backupdir).exists() {
            // ponytail: python's os.mkdir failure is an uncaught exception
            // (gcode "Internal error"); reported as a gcode error here
            std::fs::create_dir(&backupdir).map_err(|e| e.to_string())?;
        }
        // Generate the name of the backup file by stripping the leading path in
        // `cfgpath` and appending to it. Then add it to the config_backups dir
        let datestr = strftime_local("-%Y%m%d_%H%M%S");
        let cfgname = pyglob::split(cfgpath).1;
        let mut backup_path = format!("{}/{}{}", backupdir, cfgname, datestr);
        if cfgpath.ends_with(".cfg") {
            backup_path = format!("{}/{}{}.cfg", backupdir, &cfgname[..cfgname.len() - 4], datestr);
        }
        logging::info(&format!("SAVE_CONFIG to '{}' (backup in '{}')", cfgpath, backup_path));
        let res = (|| -> std::io::Result<()> {
            // Read the current config into the backup before making changes to
            // the original file
            let current = std::fs::read_to_string(cfgpath)?;
            std::fs::write(&backup_path, current)?;
            // With the backup created, write the new data to the original file
            std::fs::write(cfgpath, cfgdata)
        })();
        if res.is_err() {
            let msg = "Unable to write config file during SAVE_CONFIG";
            logging::exception(msg);
            return Err(msg.into());
        }
        Ok(())
    }

    fn save_includes(&self, cfgpath: &str, data: &str, visitedpaths: &mut HashSet<String>) -> std::result::Result<(), String> {
        // Prevent an infinite loop in the event of configs circularly
        // referencing each other
        if visitedpaths.contains(cfgpath) {
            return Ok(());
        }
        visitedpaths.insert(cfgpath.to_string());
        let dirname = pyglob::dirname(cfgpath);
        // Read the data as individual lines so we can find include blocks
        for line in data.split('\n') {
            // Strip trailing comment
            let line = match line.find('#') {
                Some(pos) => &line[..pos],
                None => line,
            };
            let Some(spec) = configparser::match_sectcre(line).and_then(|h| h.strip_prefix("include ").map(|s| s.to_string()))
            else {
                continue;
            };
            let include_glob = pyglob::join(&dirname, &[py_strip(&spec)]);
            // retrieve all filenames associated with the absolute path of
            // the include header
            let mut include_filenames = pyglob::glob(&include_glob, false);
            if include_filenames.is_empty() && !pyglob::has_magic(&include_glob) {
                // ponytail: python raises configfile.error here (gcode turns it
                // into an internal error); reported as a gcode error
                return Err(format!("Include file '{}' does not exist", include_glob));
            }
            include_filenames.sort();
            // Read the include files and check them against autosave data.
            // If autosave data overwites anything we'll update the file
            // and create a backup.
            for include_filename in include_filenames {
                // Recursively check for includes. No need to check for looping
                // includes as klipper checks this at startup.
                let include_predata = self.read_config_file(&include_filename).map_err(|e| e.message)?;
                self.save_includes(&include_filename, &include_predata, visitedpaths)?;
                let include_postdata = self.strip_duplicates(&include_predata, self.autosave.as_ref().unwrap());
                // Only write and backup data that's been changed
                if include_predata != include_postdata {
                    self.write_backup(&include_filename, &include_postdata)?;
                }
            }
        }
        Ok(())
    }

    /// cmd_SAVE_CONFIG up to the restart handling: Ok(false) when there is
    /// nothing to save, Err(msg) where python raises gcode.error(msg). The
    /// caller does RESTART=... / request_restart / respond_info.
    pub fn save_config(&mut self, autosave_includes: bool) -> std::result::Result<bool, String> {
        if self.autosave.as_ref().map(|a| a.fileconfig.borrow().sections().is_empty()).unwrap_or(true) {
            return Ok(false);
        }
        // Create string containing autosave data
        let autosave_data = self.build_config_string(self.autosave.as_ref().unwrap());
        let mut lines: Vec<String> = autosave_data.split('\n').map(|l| py_strip(&format!("#*# {}", l)).to_string()).collect();
        lines.insert(0, format!("\n{}", py_rstrip(AUTOSAVE_HEADER)));
        lines.push(String::new());
        let autosave_data = lines.join("\n");
        // Read in and validate current config file
        let cfgname = self.config_file.clone();
        let parsed = self.read_config_file(&cfgname).and_then(|data| {
            let (regular_data, _old_autosave_data) = self.find_autosave_data(&data);
            self.build_config_wrapper(&regular_data, &cfgname).map(|_| (data, regular_data))
        });
        let (data, regular_data) = match parsed {
            Ok(v) => v,
            Err(_) => {
                let msg = "Unable to parse existing config on SAVE_CONFIG";
                logging::exception(msg);
                return Err(msg.into());
            }
        };
        let regular_data = self.strip_duplicates(&regular_data, self.autosave.as_ref().unwrap());
        if autosave_includes {
            self.save_includes(&cfgname, &data, &mut HashSet::new())?;
        }
        // NOW we're safe to check for conflicts
        self.disallow_include_conflicts(&regular_data, &cfgname)?;
        let data = format!("{}{}", py_rstrip(&regular_data), autosave_data);
        self.write_backup(&cfgname, &data)?;
        Ok(true)
    }
}

/// time.strftime(fmt) in local time
pub fn strftime_local(fmt: &str) -> String {
    let fmt = std::ffi::CString::new(fmt).unwrap();
    let mut buf = [0u8; 128];
    unsafe {
        let t = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        let n = libc::strftime(buf.as_mut_ptr() as *mut libc::c_char, buf.len(), fmt.as_ptr(), &tm);
        String::from_utf8_lossy(&buf[..n]).into_owned()
    }
}

// ------------------------------------------------------------ include templates

/// jinja2 Environment of PrinterConfig: {!% %!} blocks, {! !} variables,
/// {!# #!} comments, StrictUndefined, keep_trailing_newline
fn render_include(data: &str, filename: &str, include_vars: &IndexMap<String, String>) -> Result<String> {
    use minijinja::syntax::SyntaxConfig;
    use minijinja::value::{Enumerator, Object, Value as JValue};
    use std::sync::{Arc, Mutex};

    // context that records a missing name, for jinja2's "'x' is undefined"
    #[derive(Debug)]
    struct Ctx {
        vars: IndexMap<String, String>,
        missing: Mutex<Option<String>>,
    }
    impl Object for Ctx {
        fn get_value(self: &Arc<Self>, key: &JValue) -> Option<JValue> {
            let k = key.as_str()?;
            match self.vars.get(k) {
                Some(v) => Some(JValue::from(v.clone())),
                None => {
                    // the last miss before an error is the culprit (`is defined` tests miss too)
                    *self.missing.lock().unwrap() = Some(k.to_string());
                    None
                }
            }
        }
        fn enumerate(self: &Arc<Self>) -> Enumerator {
            Enumerator::Values(self.vars.keys().map(|k| JValue::from(k.clone())).collect())
        }
    }

    // raised while handling the jinja2 exception (class, message)
    let render_err = |cls: &str, e: String| error(format!("Error rendering include file '{}': {}", filename, e)).with_context(cls, &e);
    const SYNTAX: &str = "jinja2.exceptions.TemplateSyntaxError";
    let syntax = SyntaxConfig::builder()
        .block_delimiters("{!%", "%!}")
        .variable_delimiters("{!", "!}")
        .comment_delimiters("{!#", "#!}")
        .build()
        .map_err(|e| render_err(SYNTAX, e.to_string()))?;
    let mut env = minijinja::Environment::new();
    env.set_syntax(syntax);
    env.set_undefined_behavior(minijinja::UndefinedBehavior::Strict);
    env.set_keep_trailing_newline(true);
    // ponytail: jinja2 error texts are only reproduced for undefined names
    let tmpl = env.template_from_str(data).map_err(|e| render_err(SYNTAX, e.to_string()))?;
    let ctx = Arc::new(Ctx { vars: include_vars.clone(), missing: Mutex::new(None) });
    tmpl.render(JValue::from_dyn_object(ctx.clone())).map_err(|e| {
        if e.kind() == minijinja::ErrorKind::UndefinedError {
            if let Some(name) = ctx.missing.lock().unwrap().clone() {
                return render_err("jinja2.exceptions.UndefinedError", format!("'{}' is undefined", name));
            }
        }
        render_err("jinja2.exceptions.TemplateError", e.to_string())
    })
}
