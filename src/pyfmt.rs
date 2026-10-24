// Python compatible string formatting and conversion helpers (from
// Moonruster/src/pyfmt.rs). klippy embeds str()/repr() output of python
// objects in messages and logs; these reproduce it.

use serde_json::Value;

/// Python's repr() for a float
pub fn py_float_repr(f: f64) -> String {
    if f.is_nan() {
        return "nan".into();
    }
    if f.is_infinite() {
        return if f > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if f == 0.0 {
        return if f.is_sign_negative() { "-0.0".into() } else { "0.0".into() };
    }
    // Rust's {:e} produces the shortest round trip digits
    let sci = format!("{:e}", f);
    let (mantissa, exp) = sci.split_once('e').unwrap();
    let exp: i32 = exp.parse().unwrap();
    let neg = mantissa.starts_with('-');
    let digits: String = mantissa.chars().filter(|c| c.is_ascii_digit()).collect();
    let sign = if neg { "-" } else { "" };
    if (-4..16).contains(&exp) {
        // fixed notation
        let ndigits = digits.len() as i32;
        let s = if exp >= 0 {
            let int_len = exp + 1;
            if ndigits <= int_len {
                format!("{}{}.0", digits, "0".repeat((int_len - ndigits) as usize))
            } else {
                format!("{}.{}", &digits[..int_len as usize], &digits[int_len as usize..])
            }
        } else {
            format!("0.{}{}", "0".repeat((-exp - 1) as usize), digits)
        };
        format!("{}{}", sign, s)
    } else {
        let m = if digits.len() > 1 { format!("{}.{}", &digits[..1], &digits[1..]) } else { digits.clone() };
        let esign = if exp < 0 { "-" } else { "+" };
        format!("{}{}e{}{:02}", sign, m, esign, exp.abs())
    }
}

pub fn py_str_repr(s: &str) -> String {
    // Python prefers single quotes unless the string contains single quotes
    // and no double quotes
    let quote = if s.contains('\'') && !s.contains('"') { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c == quote => {
                out.push('\\');
                out.push(c);
            }
            c if (c as u32) < 0x20 || c as u32 == 0x7f => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push(quote);
    out
}

/// Python's repr() for a json value
pub fn py_repr(v: &Value) -> String {
    match v {
        Value::Null => "None".into(),
        Value::Bool(b) => if *b { "True".into() } else { "False".into() },
        Value::Number(n) => {
            if n.is_f64() {
                py_float_repr(n.as_f64().unwrap())
            } else {
                n.to_string()
            }
        }
        Value::String(s) => py_str_repr(s),
        Value::Array(a) => format!("[{}]", a.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Value::Object(o) => format!(
            "{{{}}}",
            o.iter().map(|(k, v)| format!("{}: {}", py_str_repr(k), py_repr(v))).collect::<Vec<_>>().join(", ")
        ),
    }
}

/// Python's str() for a json value
pub fn py_str(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        other => py_repr(other),
    }
}

pub fn py_bool(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

/// Python type name of a json value
pub fn py_type_name(v: &Value) -> &'static str {
    match v {
        Value::Null => "NoneType",
        Value::Bool(_) => "bool",
        Value::Number(n) => if n.is_f64() { "float" } else { "int" },
        Value::String(_) => "str",
        Value::Array(_) => "list",
        Value::Object(_) => "dict",
    }
}

/// Python's min(a, b) for floats (a unless b < a: NaN handling included)
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's max(a, b) for floats (a unless b > a)
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python 3.12+ builtin sum() of floats (start 0): Neumaier compensated
/// summation (Python/bltinmodule.c builtin_sum_impl)
pub fn py_sum(items: impl IntoIterator<Item = f64>) -> f64 {
    let mut f_result = 0.0f64;
    let mut c = 0.0f64;
    for x in items {
        // Improved Kahan-Babuska algorithm by Arnold Neumaier
        let t = f_result + x;
        if f_result.abs() >= x.abs() {
            c += (f_result - t) + x;
        } else {
            c += (x - t) + f_result;
        }
        f_result = t;
    }
    // Avoid losing the sign on a negative result, and don't let adding the
    // compensation convert an infinite or overflowed sum to a NaN.
    if c != 0.0 && c.is_finite() {
        f_result += c;
    }
    f_result
}

/// Python's round(x, ndigits) (correctly rounded, ties to even)
pub fn py_round(x: f64, ndigits: usize) -> f64 {
    if !x.is_finite() {
        return x;
    }
    format!("{:.*}", ndigits, x).parse().unwrap_or(x)
}

const SUGGEST_MOVE_COST: usize = 2;
const SUGGEST_CASE_COST: usize = 1;
const SUGGEST_MAX_STRING_SIZE: usize = 40;
const SUGGEST_MAX_CANDIDATE_ITEMS: usize = 750;

// Python/suggestions.c levenshtein_distance (on the utf-8 bytes)
fn levenshtein_distance<'a>(mut a: &'a [u8], mut b: &'a [u8], max_cost: usize) -> usize {
    if a == b {
        return 0;
    }
    while !a.is_empty() && !b.is_empty() && a[0] == b[0] {
        a = &a[1..];
        b = &b[1..];
    }
    while !a.is_empty() && !b.is_empty() && a[a.len() - 1] == b[b.len() - 1] {
        a = &a[..a.len() - 1];
        b = &b[..b.len() - 1];
    }
    if a.is_empty() || b.is_empty() {
        return (a.len() + b.len()) * SUGGEST_MOVE_COST;
    }
    if a.len() > SUGGEST_MAX_STRING_SIZE || b.len() > SUGGEST_MAX_STRING_SIZE {
        return max_cost + 1;
    }
    if b.len() < a.len() {
        std::mem::swap(&mut a, &mut b);
    }
    if (b.len() - a.len()) * SUGGEST_MOVE_COST > max_cost {
        return max_cost + 1;
    }
    let cost = |x: u8, y: u8| {
        if x == y {
            0
        } else if x.is_ascii_alphabetic() && y.is_ascii_alphabetic() && x.eq_ignore_ascii_case(&y) {
            SUGGEST_CASE_COST
        } else {
            SUGGEST_MOVE_COST
        }
    };
    let mut buffer: Vec<usize> = (1..=a.len()).map(|i| i * SUGGEST_MOVE_COST).collect();
    let mut result = 0;
    for (b_index, &code) in b.iter().enumerate() {
        result = b_index * SUGGEST_MOVE_COST;
        let mut distance = result;
        let mut minimum = usize::MAX;
        for (index, &ch) in a.iter().enumerate() {
            let substitute = distance + cost(code, ch);
            distance = buffer[index];
            let insert_delete = result.min(distance) + SUGGEST_MOVE_COST;
            result = insert_delete.min(substitute);
            buffer[index] = result;
            minimum = minimum.min(result);
        }
        if minimum > max_cost {
            return max_cost + 1;
        }
    }
    result
}

/// The "Did you mean" name of a python 3.15 traceback (traceback.py
/// _compute_suggestion_error via Python/suggestions.c
/// _Py_CalculateSuggestions): the closest candidate, the first on ties
pub fn py_suggestion<'a>(candidates: &'a [String], name: &str) -> Option<&'a str> {
    if candidates.len() >= SUGGEST_MAX_CANDIDATE_ITEMS || name.len() > SUGGEST_MAX_STRING_SIZE {
        return None;
    }
    let mut best: Option<(&str, usize)> = None;
    for item in candidates {
        if item == name {
            continue;
        }
        // No more than 1/3 of the involved characters should need changed,
        // and don't take matches already beaten
        let mut max_distance = (name.len() + item.len() + 3) * SUGGEST_MOVE_COST / 6;
        if let Some((_, d)) = best {
            max_distance = max_distance.min(d - 1);
        }
        let d = levenshtein_distance(name.as_bytes(), item.as_bytes(), max_distance);
        if d > max_distance {
            continue;
        }
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((item, d));
        }
    }
    best.map(|(s, _)| s)
}

/// Python's str.isspace() for one char (also the `\s` class of `re`):
/// Rust's char::is_whitespace() plus the \x1c-\x1f separators
pub fn py_isspace(c: char) -> bool {
    c.is_whitespace() || ('\x1c'..='\x1f').contains(&c)
}

/// Python's str.strip() / lstrip() / rstrip() without arguments
pub fn py_strip(s: &str) -> &str {
    s.trim_matches(py_isspace)
}
pub fn py_lstrip(s: &str) -> &str {
    s.trim_start_matches(py_isspace)
}
pub fn py_rstrip(s: &str) -> &str {
    s.trim_end_matches(py_isspace)
}

// underscores in python number literals must sit between two digits
fn underscores_ok(s: &str) -> bool {
    let b = s.as_bytes();
    (0..b.len()).all(|i| b[i] != b'_' || (i > 0 && i + 1 < b.len() && b[i - 1].is_ascii_digit() && b[i + 1].is_ascii_digit()))
}

/// Python's int(str) conversion
pub fn py_int(s: &str) -> Option<i64> {
    let t = py_strip(s);
    if t.is_empty() {
        return None;
    }
    let (sign, body) = match t.as_bytes()[0] {
        b'+' => (1, &t[1..]),
        b'-' => (-1, &t[1..]),
        _ => (1, t),
    };
    if body.is_empty() || body.starts_with('_') || body.ends_with('_') || body.contains("__") {
        return None;
    }
    let clean: String = body.chars().filter(|c| *c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    clean.parse::<i64>().ok().map(|v| v * sign)
}

/// Python's int(str, 0): a 0x / 0o / 0b prefix picks the base, and a
/// decimal may not have leading zeros
pub fn py_int_base0(s: &str) -> Option<i64> {
    let t = py_strip(s);
    let (sign, body) = match t.as_bytes().first()? {
        b'+' => (1, &t[1..]),
        b'-' => (-1, &t[1..]),
        _ => (1, t),
    };
    let lower = body.to_ascii_lowercase();
    let (radix, digits) = match lower.get(..2) {
        Some("0x") => (16, &lower[2..]),
        Some("0o") => (8, &lower[2..]),
        Some("0b") => (2, &lower[2..]),
        _ => (10, lower.as_str()),
    };
    // an underscore may follow the prefix or separate digits
    let digits = if radix != 10 { digits.strip_prefix('_').unwrap_or(digits) } else { digits };
    if digits.is_empty() || digits.starts_with('_') || digits.ends_with('_') || digits.contains("__") {
        return None;
    }
    let clean: String = digits.chars().filter(|c| *c != '_').collect();
    if radix == 10 && clean.len() > 1 && clean.starts_with('0') && clean.chars().any(|c| c != '0') {
        return None;
    }
    i64::from_str_radix(&clean, radix).ok().filter(|_| clean.chars().all(|c| c.is_digit(radix))).map(|v| v * sign)
}

/// Python's int(str, 16): an optional 0x prefix, then hex digits
pub fn py_int_base16(s: &str) -> Option<i64> {
    let t = py_strip(s);
    let (sign, body) = match t.as_bytes().first()? {
        b'+' => (1, &t[1..]),
        b'-' => (-1, &t[1..]),
        _ => (1, t),
    };
    let lower = body.to_ascii_lowercase();
    let digits = match lower.strip_prefix("0x") {
        // an underscore may follow the prefix
        Some(d) => d.strip_prefix('_').unwrap_or(d),
        None => lower.as_str(),
    };
    if digits.is_empty() || digits.starts_with('_') || digits.ends_with('_') || digits.contains("__") {
        return None;
    }
    let clean: String = digits.chars().filter(|c| *c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    i64::from_str_radix(&clean, 16).ok().map(|v| v * sign)
}

/// Python's float(str) conversion
pub fn py_float(s: &str) -> Option<f64> {
    let t = py_strip(s);
    if t.is_empty() {
        return None;
    }
    let lower = t.to_lowercase();
    let (sign, body) = match lower.as_bytes()[0] {
        b'+' => (1.0, &lower[1..]),
        b'-' => (-1.0, &lower[1..]),
        _ => (1.0, lower.as_str()),
    };
    match body {
        "inf" | "infinity" => return Some(sign * f64::INFINITY),
        "nan" => return Some(f64::NAN),
        _ => {}
    }
    if !underscores_ok(body) {
        return None;
    }
    let clean: String = body.chars().filter(|c| *c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_digit() || c == '.' || c == 'e' || c == '+' || c == '-') {
        return None;
    }
    // rust also takes "inf"/"nan" spellings, handled above; signs only
    // directly after 'e' are valid
    if clean.starts_with(['+', '-']) {
        return None;
    }
    clean.parse::<f64>().ok().map(|v| v * sign)
}

/// Python bool(value)
pub fn py_truthy(v: &Value) -> bool {
    match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().map(|f| f != 0.0).unwrap_or(true),
        Value::String(s) => !s.is_empty(),
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
    }
}

/// list(shlex.shlex(s, posix=True)) with whitespace_split=True and the given
/// commenters (port of the python 3.13 state machine for that configuration:
/// no punctuation_chars, no source inclusion). Err is the ValueError text.
pub fn py_shlex_split(s: &str, commenters: &str) -> Result<Vec<String>, String> {
    const WHITESPACE: &str = " \t\r\n";
    const QUOTES: &str = "'\"";
    const ESCAPE: char = '\\';
    const ESCAPEDQUOTES: &str = "\"";
    let chars: Vec<char> = s.chars().collect();
    let mut pos = 0;
    let mut out = vec![];
    // state: ' ' (whitespace), 'a' (word), a quote char, the escape char, None (eof)
    let mut state: Option<char> = Some(' ');
    loop {
        // read_token()
        let mut quoted = false;
        let mut escapedstate = ' ';
        let mut token = String::new();
        loop {
            let nextchar = chars.get(pos).copied();
            if nextchar.is_some() {
                pos += 1;
            }
            let Some(st) = state else {
                token.clear(); // past end of file
                break;
            };
            if st == ' ' {
                match nextchar {
                    None => {
                        state = None; // end of file
                        break;
                    }
                    Some(c) if WHITESPACE.contains(c) => {
                        if !token.is_empty() || quoted {
                            break; // emit current token
                        }
                        continue;
                    }
                    Some(c) if commenters.contains(c) => {
                        // instream.readline()
                        while let Some(&c) = chars.get(pos) {
                            pos += 1;
                            if c == '\n' {
                                break;
                            }
                        }
                    }
                    Some(c) if c == ESCAPE => {
                        escapedstate = 'a';
                        state = Some(c);
                    }
                    Some(c) if QUOTES.contains(c) => state = Some(c),
                    Some(c) => {
                        // wordchars or whitespace_split
                        token.push(c);
                        state = Some('a');
                    }
                }
            } else if QUOTES.contains(st) {
                quoted = true;
                match nextchar {
                    None => return Err("No closing quotation".into()),
                    Some(c) if c == st => state = Some('a'),
                    Some(c) if c == ESCAPE && ESCAPEDQUOTES.contains(st) => {
                        escapedstate = st;
                        state = Some(c);
                    }
                    Some(c) => token.push(c),
                }
            } else if st == ESCAPE {
                let Some(c) = nextchar else {
                    return Err("No escaped character".into());
                };
                // In posix shells, only the quote itself or the escape
                // character may be escaped within quotes.
                if QUOTES.contains(escapedstate) && c != st && c != escapedstate {
                    token.push(st);
                }
                token.push(c);
                state = Some(escapedstate);
            } else {
                // state 'a'
                match nextchar {
                    None => {
                        state = None; // end of file
                        break;
                    }
                    Some(c) if WHITESPACE.contains(c) => {
                        state = Some(' ');
                        if !token.is_empty() || quoted {
                            break; // emit current token
                        }
                        continue;
                    }
                    Some(c) if commenters.contains(c) => {
                        while let Some(&c) = chars.get(pos) {
                            pos += 1;
                            if c == '\n' {
                                break;
                            }
                        }
                        state = Some(' ');
                        if !token.is_empty() || quoted {
                            break; // emit current token
                        }
                        continue;
                    }
                    Some(c) if QUOTES.contains(c) => state = Some(c),
                    Some(c) if c == ESCAPE => {
                        escapedstate = 'a';
                        state = Some(c);
                    }
                    Some(c) => token.push(c),
                }
            }
        }
        // posix: an unquoted empty token is EOF
        if !quoted && token.is_empty() {
            return Ok(out);
        }
        out.push(token);
    }
}

/// str() of the python OSError an io failure on `path` raises:
/// "[Errno 2] No such file or directory: 'path'"
pub fn py_oserror(e: &std::io::Error, path: &str) -> String {
    let errno = e.raw_os_error().unwrap_or(0);
    let msg = unsafe { std::ffi::CStr::from_ptr(libc::strerror(errno)) }.to_string_lossy().into_owned();
    format!("[Errno {}] {}: {}", errno, msg, py_str_repr(path))
}

/// the OSError subclass python raises for an io failure's errno
pub fn py_oserror_class(e: &std::io::Error) -> &'static str {
    match e.raw_os_error().unwrap_or(0) {
        libc::ENOENT => "FileNotFoundError",
        libc::EACCES | libc::EPERM => "PermissionError",
        libc::EISDIR => "IsADirectoryError",
        libc::ENOTDIR => "NotADirectoryError",
        libc::EEXIST => "FileExistsError",
        libc::EINTR => "InterruptedError",
        libc::EAGAIN | libc::EALREADY | libc::EINPROGRESS => "BlockingIOError",
        libc::ECHILD => "ChildProcessError",
        libc::EPIPE | libc::ESHUTDOWN => "BrokenPipeError",
        libc::ECONNABORTED => "ConnectionAbortedError",
        libc::ECONNREFUSED => "ConnectionRefusedError",
        libc::ECONNRESET => "ConnectionResetError",
        libc::ESRCH => "ProcessLookupError",
        libc::ETIMEDOUT => "TimeoutError",
        _ => "OSError",
    }
}

/// str() of the UnicodeDecodeError of decoding `data` as utf-8
pub fn py_utf8_error(data: &[u8], e: &std::str::Utf8Error) -> String {
    let pos = e.valid_up_to();
    let reason = match e.error_len() {
        None => "unexpected end of data",
        Some(_) if matches!(data[pos], 0x80..=0xC1 | 0xF5..=0xFF) => "invalid start byte",
        Some(_) => "invalid continuation byte",
    };
    format!("'utf-8' codec can't decode byte 0x{:02x} in position {}: {}", data[pos], pos, reason)
}

/// the ValueError of unpacking a list of `got` items into `expected`
/// names, None when they match
pub fn py_unpack_error(expected: usize, got: usize) -> Option<String> {
    if got < expected {
        Some(format!("not enough values to unpack (expected {}, got {})", expected, got))
    } else if got > expected {
        Some(format!("too many values to unpack (expected {})", expected))
    } else {
        None
    }
}

/// the ValueError of float(s)
pub fn py_float_error(s: &str) -> String {
    format!("could not convert string to float: {}", py_str_repr(s))
}

/// json.dumps() of a python float
pub fn py_json_float(f: f64) -> String {
    if f.is_nan() {
        "NaN".into()
    } else if f.is_infinite() {
        if f > 0. { "Infinity".into() } else { "-Infinity".into() }
    } else {
        py_float_repr(f)
    }
}

/// Python's repr() for bytes
pub fn py_bytes_repr(b: &[u8]) -> String {
    let quote = if b.contains(&b'\'') && !b.contains(&b'"') { b'"' } else { b'\'' };
    let mut out = String::with_capacity(b.len() + 3);
    out.push('b');
    out.push(quote as char);
    for &c in b {
        match c {
            b'\\' => out.push_str("\\\\"),
            b'\t' => out.push_str("\\t"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            c if c == quote => {
                out.push('\\');
                out.push(c as char);
            }
            c if c < 0x20 || c >= 0x7f => out.push_str(&format!("\\x{:02x}", c)),
            c => out.push(c as char),
        }
    }
    out.push(quote as char);
    out
}

/// The digits of Python's int(s, base) for base 0 or 16: (negative, base,
/// digit values most significant first), None where python raises ValueError
pub fn py_int_digits(s: &str, base: u32) -> Option<(bool, u32, Vec<u8>)> {
    let t = s.trim();
    let (neg, mut body) = match t.as_bytes().first() {
        Some(b'+') => (false, &t[1..]),
        Some(b'-') => (true, &t[1..]),
        _ => (false, t),
    };
    let lower = body.to_ascii_lowercase();
    let auto = base == 0;
    let mut base = base;
    let mut prefixed = false;
    for (p, b) in [("0x", 16), ("0o", 8), ("0b", 2)] {
        if lower.starts_with(p) && (base == 0 || base == b) {
            base = b;
            body = &body[2..];
            prefixed = true;
            break;
        }
    }
    if base == 0 {
        base = 10;
    }
    // an underscore may follow the prefix, otherwise only sit between digits
    if prefixed && body.starts_with('_') {
        body = &body[1..];
    }
    if body.is_empty() || body.starts_with('_') || body.ends_with('_') || body.contains("__") {
        return None;
    }
    let digits: Vec<u8> = body
        .chars()
        .filter(|c| *c != '_')
        .map(|c| c.to_digit(base).map(|d| d as u8))
        .collect::<Option<_>>()?;
    // base 0 rejects leading zeros on non-zero decimals ("010")
    if auto && !prefixed && digits.len() > 1 && digits[0] == 0
        && digits.iter().any(|d| *d != 0)
    {
        return None;
    }
    Some((neg, base, digits))
}

/// Python's int(s, base) for base 0 or 16
// ponytail: limited to i64, python ints are unbounded; None on overflow
pub fn py_int_base(s: &str, base: u32) -> Option<i64> {
    let (neg, base, digits) = py_int_digits(s, base)?;
    let mut v: i128 = 0;
    for d in digits {
        v = v.checked_mul(base as i128)?.checked_add(d as i128)?;
        if v > i64::MAX as i128 + 1 {
            return None;
        }
    }
    let v = if neg { -v } else { v };
    i64::try_from(v).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn bytes_repr_and_int_base() {
        assert_eq!(py_bytes_repr(b"a'\x00\x7f\xff\"\\\n"), "b'a\\'\\x00\\x7f\\xff\"\\\\\\n'");
        assert_eq!(py_bytes_repr(b"it's"), "b\"it's\"");
        assert_eq!(py_int_base("0x1F", 0), Some(31));
        assert_eq!(py_int_base(" -0b101 ", 0), Some(-5));
        assert_eq!(py_int_base("0o17", 0), Some(15));
        assert_eq!(py_int_base("0x_ff", 0), Some(255));
        assert_eq!(py_int_base("010", 0), None);
        assert_eq!(py_int_base("000", 0), Some(0));
        assert_eq!(py_int_base("1_000", 0), Some(1000));
        assert_eq!(py_int_base("ff", 0), None);
        assert_eq!(py_int_base("ff", 16), Some(255));
        assert_eq!(py_int_base("0xff", 16), Some(255));
        assert_eq!(py_int_base("", 0), None);
        // python float() / int()
        assert_eq!(py_float(" 1_0.5 "), Some(10.5));
        assert_eq!(py_float("1_.5"), None);
        assert_eq!(py_float("+.5"), Some(0.5));
        assert_eq!(py_float("5."), Some(5.0));
        assert_eq!(py_float("1e-3"), Some(0.001));
        assert_eq!(py_float("--1"), None);
        assert_eq!(py_float("1e"), None);
        assert_eq!(py_float("\x1c2\x1f"), Some(2.0));
        assert_eq!(py_int("_1"), None);
        assert_eq!(py_int("-0"), Some(0));
        assert_eq!(py_strip("\x1c a \t\n"), "a");
    }

    #[test]
    fn float_repr() {
        assert_eq!(py_float_repr(1.0), "1.0");
        assert_eq!(py_float_repr(0.1), "0.1");
        assert_eq!(py_float_repr(1e-5), "1e-05");
        assert_eq!(py_float_repr(1e16), "1e+16");
        assert_eq!(py_float_repr(123456.789), "123456.789");
        assert_eq!(py_float_repr(-0.0001), "-0.0001");
        assert_eq!(py_float_repr(1.5e300), "1.5e+300");
    }

    #[test]
    fn repr() {
        assert_eq!(py_repr(&json!({"a": [1, 2.5, null, true], "b": "it's"})), "{'a': [1, 2.5, None, True], 'b': \"it's\"}");
        assert_eq!(py_int(" 1_000 "), Some(1000));
        assert_eq!(py_int("1.5"), None);
        assert_eq!(py_float("1e3"), Some(1000.0));
    }
}
