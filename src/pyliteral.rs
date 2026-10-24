// python's ast.literal_eval() (CPython 3.13 Lib/ast.py over the python
// expression grammar), with python's error messages. Values come back as
// template values (tuples stay tuples, sets / bytes / complex numbers are
// kept only far enough to fail the json check gcode_macro applies).
//
// The parser covers python's expression syntax far enough to tell a
// SyntaxError from a valid non-literal expression ("malformed node").
use crate::jinja::value::{Key, Value};
use crate::pyfmt::py_isspace;
use indexmap::IndexMap;

/// literal_eval failure: (exception class, str(e))
pub type LitError = (&'static str, String);

#[derive(Debug)]
enum Node {
    Const(Value, usize),
    /// a complex / bytes / set constant (not json serializable)
    Special(&'static str, usize),
    Tuple(Vec<Node>, usize),
    List(Vec<Node>, usize),
    Set(Vec<Node>, usize),
    /// keys None for **mapping entries
    Dict(Vec<(Option<Node>, Node)>, usize),
    UnaryOp(char, Box<Node>, usize),
    BinOp(char, Box<Node>, Box<Node>, usize),
    /// any other valid expression node: (ast class name, lineno)
    Other(&'static str, usize),
    /// set() with no arguments
    SetCall(usize),
}

#[derive(Clone, Debug)]
enum Tok {
    Num(Value),
    Imag,
    Str(String),
    Bytes,
    FStr,
    Name(String),
    Op(String),
    End,
}

struct Lexer {
    toks: Vec<(Tok, usize)>,
}

fn syntax(msg: impl Into<String>, line: usize) -> LitError {
    ("SyntaxError", format!("{} (<unknown>, line {})", msg.into(), line))
}

fn lex(src: &str) -> Result<Lexer, LitError> {
    let c: Vec<char> = src.chars().collect();
    let mut i = 0;
    let mut line = 1;
    let mut toks = vec![];
    let mut depth: Vec<(char, usize)> = vec![];
    while i < c.len() {
        let ch = c[i];
        if ch == '\n' {
            if depth.is_empty() {
                toks.push((Tok::Op("NEWLINE".into()), line));
            }
            line += 1;
            i += 1;
            continue;
        }
        if ch == '\\' && c.get(i + 1) == Some(&'\n') {
            line += 1;
            i += 2;
            continue;
        }
        if py_isspace(ch) {
            i += 1;
            continue;
        }
        if ch == '#' {
            while i < c.len() && c[i] != '\n' {
                i += 1;
            }
            continue;
        }
        // strings (with prefixes)
        let mut j = i;
        while j < c.len() && c[j].is_ascii_alphabetic() && j - i < 2 {
            j += 1;
        }
        let prefix: String = c[i..j].iter().collect::<String>().to_lowercase();
        let is_prefix = matches!(prefix.as_str(), "" | "r" | "u" | "b" | "br" | "rb" | "f" | "fr" | "rf");
        if is_prefix && j < c.len() && (c[j] == '\'' || c[j] == '"') {
            let q = c[j];
            let triple = c.get(j + 1) == Some(&q) && c.get(j + 2) == Some(&q);
            let start_line = line;
            let mut k = if triple { j + 3 } else { j + 1 };
            let body_start = k;
            let raw = prefix.contains('r');
            loop {
                if k >= c.len() || (!triple && c[k] == '\n') {
                    let msg = if triple {
                        format!("unterminated triple-quoted string literal (detected at line {})", line)
                    } else {
                        format!("unterminated string literal (detected at line {})", start_line)
                    };
                    return Err(syntax(msg, start_line));
                }
                if c[k] == '\\' {
                    if c.get(k + 1) == Some(&'\n') {
                        line += 1;
                    }
                    k += 2;
                    continue;
                }
                if c[k] == '\n' {
                    line += 1;
                }
                if c[k] == q && (!triple || (c.get(k + 1) == Some(&q) && c.get(k + 2) == Some(&q))) {
                    break;
                }
                k += 1;
            }
            let body: String = c[body_start..k].iter().collect();
            let tok = if prefix.contains('b') {
                Tok::Bytes
            } else if prefix.contains('f') {
                Tok::FStr
            } else if raw {
                Tok::Str(body)
            } else {
                Tok::Str(unescape(&body).map_err(|m| syntax(m, start_line))?)
            };
            toks.push((tok, start_line));
            i = if triple { k + 3 } else { k + 1 };
            continue;
        }
        // numbers
        if ch.is_ascii_digit() || (ch == '.' && c.get(i + 1).map(|d| d.is_ascii_digit()).unwrap_or(false)) {
            let (tok, end) = lex_number(&c, i, line)?;
            toks.push((tok, line));
            i = end;
            continue;
        }
        // names
        if ch == '_' || ch.is_alphabetic() {
            let mut k = i;
            while k < c.len() && (c[k] == '_' || c[k].is_alphanumeric()) {
                k += 1;
            }
            toks.push((Tok::Name(c[i..k].iter().collect()), line));
            i = k;
            continue;
        }
        // operators
        let three: String = c[i..(i + 3).min(c.len())].iter().collect();
        let two: String = c[i..(i + 2).min(c.len())].iter().collect();
        let op = if ["**=", "//=", ">>=", "<<=", "..."].contains(&three.as_str()) {
            three
        } else if ["**", "//", "==", "!=", "<=", ">=", "<<", ">>", "->", ":=", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "@="].contains(&two.as_str()) {
            two
        } else if "+-*/%@&|^~<>()[]{},:;.=".contains(ch) {
            ch.to_string()
        } else {
            return Err(syntax(format!("invalid character '{}' (U+{:04X})", ch, ch as u32), line));
        };
        match op.as_str() {
            "(" | "[" | "{" => depth.push((op.chars().next().unwrap(), line)),
            ")" | "]" | "}" => {
                let close = op.chars().next().unwrap();
                match depth.pop() {
                    None => return Err(syntax(format!("unmatched '{}'", close), line)),
                    Some((open, _)) => {
                        let expected = match open {
                            '(' => ')',
                            '[' => ']',
                            _ => '}',
                        };
                        if expected != close {
                            return Err(syntax(
                                format!("closing parenthesis '{}' does not match opening parenthesis '{}'", close, open),
                                line,
                            ));
                        }
                    }
                }
            }
            _ => {}
        }
        i += op.chars().count();
        toks.push((Tok::Op(op), line));
    }
    if let Some((open, l)) = depth.last() {
        return Err(syntax(format!("'{}' was never closed", open), *l));
    }
    toks.push((Tok::End, line));
    Ok(Lexer { toks })
}

fn lex_number(c: &[char], start: usize, line: usize) -> Result<(Tok, usize), LitError> {
    let mut i = start;
    let invalid = |kind: &str| syntax(format!("invalid {} literal", kind), line);
    let digits = |i: &mut usize, ok: &dyn Fn(char) -> bool| -> Result<String, LitError> {
        let mut s = String::new();
        loop {
            if *i < c.len() && ok(c[*i]) {
                s.push(c[*i]);
                *i += 1;
            } else if *i < c.len() && c[*i] == '_' {
                if *i + 1 < c.len() && ok(c[*i + 1]) && !s.is_empty() {
                    *i += 1;
                } else {
                    return Err(syntax("invalid decimal literal", line));
                }
            } else {
                return Ok(s);
            }
        }
    };
    if c[i] == '0' && i + 1 < c.len() && matches!(c[i + 1].to_ascii_lowercase(), 'x' | 'o' | 'b') {
        let kind = c[i + 1].to_ascii_lowercase();
        i += 2;
        let (radix, name) = match kind {
            'x' => (16, "hexadecimal"),
            'o' => (8, "octal"),
            _ => (2, "binary"),
        };
        if i < c.len() && c[i] == '_' {
            i += 1;
        }
        let s = digits(&mut i, &|ch: char| ch.is_digit(radix))?;
        if s.is_empty() || (i < c.len() && c[i].is_alphanumeric()) {
            return Err(invalid(name));
        }
        let v = i64::from_str_radix(&s, radix).map(Value::Int).unwrap_or(Value::Float(f64::INFINITY));
        return Ok((Tok::Num(v), i));
    }
    let int_part = digits(&mut i, &|ch: char| ch.is_ascii_digit())?;
    let mut is_float = false;
    let mut text = int_part.clone();
    if i < c.len() && c[i] == '.' {
        is_float = true;
        i += 1;
        text.push('.');
        text.push_str(&digits(&mut i, &|ch: char| ch.is_ascii_digit())?);
    }
    if i < c.len() && (c[i] == 'e' || c[i] == 'E') {
        is_float = true;
        i += 1;
        text.push('e');
        if i < c.len() && (c[i] == '+' || c[i] == '-') {
            text.push(c[i]);
            i += 1;
        }
        let e = digits(&mut i, &|ch: char| ch.is_ascii_digit())?;
        if e.is_empty() {
            return Err(invalid("decimal"));
        }
        text.push_str(&e);
    }
    if i < c.len() && (c[i] == 'j' || c[i] == 'J') {
        i += 1;
        return Ok((Tok::Imag, i));
    }
    if i < c.len() && (c[i].is_alphanumeric() || c[i] == '_') {
        return Err(invalid("decimal"));
    }
    if !is_float && int_part.len() > 1 && int_part.starts_with('0') && int_part.chars().any(|d| d != '0') {
        return Err(syntax("leading zeros in decimal integer literals are not permitted; use an 0o prefix for octal integers", line));
    }
    let v = if is_float {
        Value::Float(text.parse::<f64>().unwrap_or(f64::INFINITY))
    } else {
        match text.parse::<i64>() {
            Ok(v) => Value::Int(v),
            Err(_) => Value::Float(text.parse::<f64>().unwrap_or(f64::INFINITY)),
        }
    };
    Ok((Tok::Num(v), i))
}

/// the "unicode_escape" decoding of a non-raw string literal
fn unescape(s: &str) -> Result<String, String> {
    let chars: Vec<char> = s.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] != '\\' {
            out.push(chars[i]);
            i += 1;
            continue;
        }
        let pos = i;
        i += 1;
        let Some(&e) = chars.get(i) else {
            out.push('\\');
            break;
        };
        i += 1;
        match e {
            '\n' => {}
            '\\' => out.push('\\'),
            '\'' => out.push('\''),
            '"' => out.push('"'),
            'a' => out.push('\x07'),
            'b' => out.push('\x08'),
            'f' => out.push('\x0c'),
            'n' => out.push('\n'),
            'r' => out.push('\r'),
            't' => out.push('\t'),
            'v' => out.push('\x0b'),
            '0'..='7' => {
                let mut v = e.to_digit(8).unwrap();
                for _ in 0..2 {
                    match chars.get(i).and_then(|c| c.to_digit(8)) {
                        Some(d) => {
                            v = v * 8 + d;
                            i += 1;
                        }
                        None => break,
                    }
                }
                out.push(char::from_u32(v).unwrap_or('\u{fffd}'));
            }
            'x' | 'u' | 'U' => {
                let n = match e {
                    'x' => 2,
                    'u' => 4,
                    _ => 8,
                };
                let hex: String = chars[i..chars.len().min(i + n)].iter().collect();
                if hex.chars().count() < n || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
                    let what = match e {
                        'x' => "truncated \\xXX escape",
                        'u' => "truncated \\uXXXX escape",
                        _ => "truncated \\UXXXXXXXX escape",
                    };
                    let end = pos + 1 + hex.chars().take_while(|c| c.is_ascii_hexdigit()).count();
                    return Err(format!(
                        "(unicode error) 'unicodeescape' codec can't decode bytes in position {}-{}: {}",
                        pos,
                        end,
                        what
                    ));
                }
                i += n;
                out.push(char::from_u32(u32::from_str_radix(&hex, 16).unwrap()).unwrap_or('\u{fffd}'));
            }
            other => {
                out.push('\\');
                out.push(other);
            }
        }
    }
    Ok(out)
}

struct Parser {
    toks: Vec<(Tok, usize)>,
    pos: usize,
}

type PR<T> = Result<T, LitError>;

impl Parser {
    fn peek(&self) -> &Tok {
        &self.toks[self.pos].0
    }
    fn line(&self) -> usize {
        self.toks[self.pos].1
    }
    fn is_op(&self, op: &str) -> bool {
        matches!(self.peek(), Tok::Op(o) if o == op)
    }
    fn is_name(&self, n: &str) -> bool {
        matches!(self.peek(), Tok::Name(o) if o == n)
    }
    fn next(&mut self) -> (Tok, usize) {
        let t = self.toks[self.pos].clone();
        if self.pos + 1 < self.toks.len() {
            self.pos += 1;
        }
        t
    }
    fn fail<T>(&self) -> PR<T> {
        Err(syntax("invalid syntax", self.line()))
    }
    fn expect_op(&mut self, op: &str) -> PR<()> {
        if self.is_op(op) {
            self.next();
            return Ok(());
        }
        self.fail()
    }

    // expression: lambda / ifexp / or-test
    fn expr(&mut self) -> PR<Node> {
        let line = self.line();
        if self.is_name("lambda") {
            self.next();
            while !self.is_op(":") {
                if matches!(self.peek(), Tok::End) {
                    return self.fail();
                }
                self.next();
            }
            self.next();
            self.expr()?;
            return Ok(Node::Other("Lambda", line));
        }
        let body = self.or_test()?;
        if self.is_name("if") {
            self.next();
            self.or_test()?;
            if !self.is_name("else") {
                return self.fail();
            }
            self.next();
            self.expr()?;
            return Ok(Node::Other("IfExp", line));
        }
        Ok(body)
    }

    fn or_test(&mut self) -> PR<Node> {
        let line = self.line();
        let mut n = self.and_test()?;
        while self.is_name("or") {
            self.next();
            self.and_test()?;
            n = Node::Other("BoolOp", line);
        }
        Ok(n)
    }

    fn and_test(&mut self) -> PR<Node> {
        let line = self.line();
        let mut n = self.not_test()?;
        while self.is_name("and") {
            self.next();
            self.not_test()?;
            n = Node::Other("BoolOp", line);
        }
        Ok(n)
    }

    fn not_test(&mut self) -> PR<Node> {
        let line = self.line();
        if self.is_name("not") {
            self.next();
            self.not_test()?;
            return Ok(Node::Other("UnaryOp", line));
        }
        self.comparison()
    }

    fn comparison(&mut self) -> PR<Node> {
        let line = self.line();
        let mut n = self.arith()?;
        loop {
            let is_cmp = ["<", ">", "==", ">=", "<=", "!="].iter().any(|o| self.is_op(o)) || self.is_name("in") || self.is_name("is")
                || (self.is_name("not") && matches!(self.toks.get(self.pos + 1), Some((Tok::Name(n), _)) if n == "in"));
            if !is_cmp {
                break;
            }
            if self.is_name("not") || self.is_name("is") {
                self.next();
                if self.is_name("not") || self.is_name("in") {
                    self.next();
                }
            } else {
                self.next();
            }
            self.arith()?;
            n = Node::Other("Compare", line);
        }
        Ok(n)
    }

    fn arith(&mut self) -> PR<Node> {
        let line = self.line();
        let mut n = self.term()?;
        loop {
            let op = if self.is_op("+") {
                '+'
            } else if self.is_op("-") {
                '-'
            } else if ["|", "^", "&", "<<", ">>"].iter().any(|o| self.is_op(o)) {
                'x'
            } else {
                break;
            };
            self.next();
            let r = self.term()?;
            n = Node::BinOp(op, Box::new(n), Box::new(r), line);
        }
        Ok(n)
    }

    fn term(&mut self) -> PR<Node> {
        let line = self.line();
        let mut n = self.factor()?;
        while ["*", "/", "//", "%", "@"].iter().any(|o| self.is_op(o)) {
            self.next();
            let r = self.factor()?;
            n = Node::BinOp('x', Box::new(n), Box::new(r), line);
        }
        Ok(n)
    }

    fn factor(&mut self) -> PR<Node> {
        let line = self.line();
        if self.is_op("-") || self.is_op("+") || self.is_op("~") {
            let op = match self.next().0 {
                Tok::Op(o) => o.chars().next().unwrap(),
                _ => unreachable!(),
            };
            let operand = self.factor()?;
            return Ok(Node::UnaryOp(op, Box::new(operand), line));
        }
        self.power()
    }

    fn power(&mut self) -> PR<Node> {
        let line = self.line();
        let n = self.atom_expr()?;
        if self.is_op("**") {
            self.next();
            let r = self.factor()?;
            return Ok(Node::BinOp('x', Box::new(n), Box::new(r), line));
        }
        Ok(n)
    }

    fn atom_expr(&mut self) -> PR<Node> {
        let line = self.line();
        let mut n = self.atom()?;
        loop {
            if self.is_op("(") {
                self.next();
                let is_set_call = matches!(&n, Node::Other("Name:set", _)) && self.is_op(")");
                self.call_args()?;
                n = if is_set_call { Node::SetCall(line) } else { Node::Other("Call", line) };
            } else if self.is_op("[") {
                self.next();
                self.subscript_list()?;
                self.expect_op("]")?;
                n = Node::Other("Subscript", line);
            } else if self.is_op(".") {
                self.next();
                if !matches!(self.peek(), Tok::Name(_)) {
                    return self.fail();
                }
                self.next();
                n = Node::Other("Attribute", line);
            } else {
                break;
            }
        }
        if let Node::Other("Name:set", l) = n {
            n = Node::Other("Name", l);
        }
        Ok(n)
    }

    fn call_args(&mut self) -> PR<()> {
        while !self.is_op(")") {
            if self.is_op("*") || self.is_op("**") {
                self.next();
            }
            self.expr()?;
            if self.is_op("=") {
                self.next();
                self.expr()?;
            }
            if self.is_name("for") {
                self.comprehension()?;
            }
            if !self.is_op(",") {
                break;
            }
            self.next();
        }
        self.expect_op(")")
    }

    fn subscript_list(&mut self) -> PR<()> {
        while !self.is_op("]") {
            if !self.is_op(":") {
                self.expr()?;
            }
            while self.is_op(":") {
                self.next();
                if !self.is_op(":") && !self.is_op("]") && !self.is_op(",") {
                    self.expr()?;
                }
            }
            if !self.is_op(",") {
                break;
            }
            self.next();
        }
        Ok(())
    }

    fn comprehension(&mut self) -> PR<()> {
        while self.is_name("for") || self.is_name("if") || self.is_name("async") {
            let kw = self.next().0;
            let is_async = matches!(&kw, Tok::Name(n) if n == "async");
            if is_async || matches!(&kw, Tok::Name(n) if n == "for") {
                if is_async {
                    self.next();
                }
                self.or_test()?;
                while self.is_op(",") {
                    self.next();
                    self.or_test()?;
                }
                if !self.is_name("in") {
                    return self.fail();
                }
                self.next();
            }
            self.or_test()?;
        }
        Ok(())
    }

    fn atom(&mut self) -> PR<Node> {
        let line = self.line();
        let (tok, _) = self.toks[self.pos].clone();
        match tok {
            Tok::Num(v) => {
                self.next();
                Ok(Node::Const(v, line))
            }
            Tok::Imag => {
                self.next();
                Ok(Node::Special("complex", line))
            }
            Tok::Str(_) | Tok::Bytes | Tok::FStr => {
                // implicit concatenation
                let mut s = String::new();
                let mut kind = "str";
                while let Tok::Str(_) | Tok::Bytes | Tok::FStr = self.peek() {
                    match self.next().0 {
                        Tok::Str(x) => s.push_str(&x),
                        Tok::Bytes => kind = "bytes",
                        _ => {
                            if kind == "str" {
                                kind = "fstr"
                            }
                        }
                    }
                }
                Ok(match kind {
                    "str" => Node::Const(Value::from(s), line),
                    "bytes" => Node::Special("bytes", line),
                    _ => Node::Other("JoinedStr", line),
                })
            }
            Tok::Name(n) => {
                self.next();
                match n.as_str() {
                    "None" => Ok(Node::Const(Value::None, line)),
                    "True" => Ok(Node::Const(Value::Bool(true), line)),
                    "False" => Ok(Node::Const(Value::Bool(false), line)),
                    "set" => Ok(Node::Other("Name:set", line)),
                    kw if ["and", "or", "not", "if", "else", "for", "in", "is", "lambda", "def", "class", "return", "import", "from", "while", "with", "yield", "del", "pass", "raise", "try", "except", "finally", "global", "nonlocal", "assert", "break", "continue", "elif", "as", "await", "async"].contains(&kw) => {
                        self.pos -= 1;
                        self.fail()
                    }
                    _ => Ok(Node::Other("Name", line)),
                }
            }
            Tok::Op(o) if o == "(" => {
                self.next();
                if self.is_op(")") {
                    self.next();
                    return Ok(Node::Tuple(vec![], line));
                }
                let first = self.star_expr()?;
                if self.is_name("for") {
                    self.comprehension()?;
                    self.expect_op(")")?;
                    return Ok(Node::Other("GeneratorExp", line));
                }
                if self.is_op(")") {
                    self.next();
                    return Ok(first);
                }
                let mut items = vec![first];
                while self.is_op(",") {
                    self.next();
                    if self.is_op(")") {
                        break;
                    }
                    items.push(self.star_expr()?);
                }
                self.expect_op(")")?;
                Ok(Node::Tuple(items, line))
            }
            Tok::Op(o) if o == "[" => {
                self.next();
                let mut items = vec![];
                while !self.is_op("]") {
                    items.push(self.star_expr()?);
                    if self.is_name("for") {
                        self.comprehension()?;
                        self.expect_op("]")?;
                        return Ok(Node::Other("ListComp", line));
                    }
                    if !self.is_op(",") {
                        break;
                    }
                    self.next();
                }
                self.expect_op("]")?;
                Ok(Node::List(items, line))
            }
            Tok::Op(o) if o == "{" => {
                self.next();
                if self.is_op("}") {
                    self.next();
                    return Ok(Node::Dict(vec![], line));
                }
                // dict or set
                let mut dict: Vec<(Option<Node>, Node)> = vec![];
                let mut set = vec![];
                let mut is_dict = None;
                loop {
                    if self.is_op("**") {
                        self.next();
                        let v = self.arith()?;
                        dict.push((None, v));
                        is_dict = Some(true);
                    } else {
                        let k = self.star_expr()?;
                        if self.is_op(":") && is_dict != Some(false) {
                            self.next();
                            let v = self.expr()?;
                            if self.is_name("for") {
                                self.comprehension()?;
                                self.expect_op("}")?;
                                return Ok(Node::Other("DictComp", line));
                            }
                            dict.push((Some(k), v));
                            is_dict = Some(true);
                        } else if is_dict == Some(true) {
                            return self.fail();
                        } else {
                            if self.is_name("for") {
                                self.comprehension()?;
                                self.expect_op("}")?;
                                return Ok(Node::Other("SetComp", line));
                            }
                            set.push(k);
                            is_dict = Some(false);
                        }
                    }
                    if !self.is_op(",") {
                        break;
                    }
                    self.next();
                    if self.is_op("}") {
                        break;
                    }
                }
                self.expect_op("}")?;
                if is_dict == Some(true) {
                    Ok(Node::Dict(dict, line))
                } else {
                    Ok(Node::Set(set, line))
                }
            }
            Tok::Op(o) if o == "..." => {
                self.next();
                Ok(Node::Const(Value::str("Ellipsis"), line))
            }
            _ => self.fail(),
        }
    }

    fn star_expr(&mut self) -> PR<Node> {
        let line = self.line();
        if self.is_op("*") {
            self.next();
            self.arith()?;
            return Ok(Node::Other("Starred", line));
        }
        self.expr()
    }
}

fn node_repr(n: &Node) -> (String, Option<usize>) {
    let (name, line) = match n {
        Node::Const(_, l) | Node::Special(_, l) => ("Constant", *l),
        Node::Tuple(_, l) => ("Tuple", *l),
        Node::List(_, l) => ("List", *l),
        Node::Set(_, l) => ("Set", *l),
        Node::Dict(_, l) => ("Dict", *l),
        Node::UnaryOp(_, _, l) => ("UnaryOp", *l),
        Node::BinOp(_, _, _, l) => ("BinOp", *l),
        Node::Other(n, l) => (*n, *l),
        Node::SetCall(l) => ("Call", *l),
    };
    (format!("<ast.{} object at 0x{:012x}>", name, n as *const Node as usize), Some(line))
}

fn malformed(n: Option<&Node>) -> LitError {
    match n {
        None => ("ValueError", "malformed node or string: None".into()),
        Some(n) => {
            let (r, line) = node_repr(n);
            let mut msg = "malformed node or string".to_string();
            if let Some(l) = line {
                msg.push_str(&format!(" on line {}", l));
            }
            ("ValueError", format!("{}: {}", msg, r))
        }
    }
}

/// What ast.literal_eval made: scalars as values, containers kept apart
/// so json.dumps can be walked like python's encoder (Opaque: a set /
/// bytes / complex, which json cannot serialize)
enum Lit {
    V(Value),
    Opaque(&'static str),
    List(Vec<Lit>),
    Tuple(Vec<Lit>),
    Dict(Vec<(DictKey, Lit)>),
}

/// A dict key: a value, or a hashable key that is no value (a bytes /
/// complex, or a tuple holding one: its type name)
enum DictKey {
    K(Key),
    Other(String),
}

impl Lit {
    fn type_name(&self) -> String {
        match self {
            Lit::V(v) => v.type_name(),
            Lit::Opaque(t) => t.to_string(),
            Lit::List(_) => "list".into(),
            Lit::Tuple(_) => "tuple".into(),
            Lit::Dict(_) => "dict".into(),
        }
    }

    /// hash(self) fails: the type that is not hashable
    fn unhashable(&self) -> Option<String> {
        match self {
            Lit::Opaque("set") | Lit::List(_) | Lit::Dict(_) => Some(self.type_name()),
            Lit::Tuple(items) => items.iter().find_map(Lit::unhashable),
            _ => None,
        }
    }

    fn as_value(&self) -> Option<Value> {
        match self {
            Lit::V(v) => Some(v.clone()),
            Lit::Tuple(items) => items.iter().map(Lit::as_value).collect::<Option<Vec<_>>>().map(Value::tuple),
            _ => None,
        }
    }

    /// json.dumps(self): the value, else the TypeError message and the
    /// notes python 3.15's encoder adds on the way out (innermost first)
    fn to_json(&self) -> Result<Value, (String, Vec<String>)> {
        let item = |kind: &str, i: String, l: &Lit| {
            l.to_json().map_err(|(m, mut notes)| {
                notes.push(format!("when serializing {} item {}", kind, i));
                (m, notes)
            })
        };
        let key_error = |t: &str| (format!("keys must be str, int, float, bool or None, not {}", t), vec![]);
        match self {
            Lit::V(v) => Ok(v.clone()),
            Lit::Opaque(t) => Err((format!("Object of type {} is not JSON serializable", t), vec![])),
            Lit::List(items) => {
                let v = items.iter().enumerate().map(|(i, l)| item("list", i.to_string(), l)).collect::<Result<_, _>>()?;
                Ok(Value::list(v))
            }
            Lit::Tuple(items) => {
                let v = items.iter().enumerate().map(|(i, l)| item("tuple", i.to_string(), l)).collect::<Result<_, _>>()?;
                Ok(Value::tuple(v))
            }
            Lit::Dict(entries) => {
                let mut d = IndexMap::new();
                for (k, l) in entries {
                    let k = match k {
                        DictKey::K(k) if matches!(k.0, Value::Tuple(_)) => return Err(key_error("tuple")),
                        DictKey::K(k) => k,
                        DictKey::Other(t) => return Err(key_error(t)),
                    };
                    d.insert(k.clone(), item("dict", k.0.repr(), l)?);
                }
                Ok(Value::dict(d))
            }
        }
    }
}

fn convert_num(n: &Node) -> Result<Lit, LitError> {
    match n {
        Node::Const(v @ (Value::Int(_) | Value::Float(_)), _) => Ok(Lit::V(v.clone())),
        Node::Special("complex", _) => Ok(Lit::Opaque("complex")),
        other => Err(malformed(Some(other))),
    }
}

fn convert_signed_num(n: &Node) -> Result<Lit, LitError> {
    if let Node::UnaryOp(op @ ('+' | '-'), operand, _) = n {
        let v = convert_num(operand)?;
        return Ok(match (op, v) {
            ('-', Lit::V(v)) => Lit::V(v.neg().map_err(|_| malformed(Some(n)))?),
            (_, v) => v,
        });
    }
    convert_num(n)
}

fn convert(n: Option<&Node>) -> Result<Lit, LitError> {
    let Some(n) = n else { return Err(malformed(None)) };
    let seq = |items: &[Node]| items.iter().map(|i| convert(Some(i))).collect::<Result<Vec<_>, _>>();
    match n {
        Node::Const(v, _) => Ok(Lit::V(v.clone())),
        Node::Special(t, _) => Ok(Lit::Opaque(t)),
        Node::Tuple(items, _) => Ok(Lit::Tuple(seq(items)?)),
        Node::List(items, _) => Ok(Lit::List(seq(items)?)),
        Node::Set(items, _) => {
            seq(items)?;
            Ok(Lit::Opaque("set"))
        }
        Node::SetCall(_) => Ok(Lit::Opaque("set")),
        Node::Dict(items, _) => {
            // dict(zip(map(_convert, keys), map(_convert, values))): pair by
            // pair, a repeated key keeps its place and takes the new value
            let mut entries: Vec<(DictKey, Lit)> = Vec::new();
            for (k, v) in items {
                let k = convert(k.as_ref())?;
                let v = convert(Some(v))?;
                if let Some(t) = k.unhashable() {
                    return Err(("TypeError", format!("cannot use '{}' as a dict key (unhashable type: '{}')", k.type_name(), t)));
                }
                let key = match k.as_value() {
                    Some(kv) => DictKey::K(Key::hashable(&kv).map_err(|e| ("TypeError", e.msg))?),
                    None => DictKey::Other(k.type_name()),
                };
                if let DictKey::K(key) = &key {
                    if let Some(e) = entries.iter_mut().find(|(x, _)| matches!(x, DictKey::K(x) if x == key)) {
                        e.1 = v;
                        continue;
                    }
                }
                entries.push((key, v));
            }
            Ok(Lit::Dict(entries))
        }
        Node::BinOp('+' | '-', left, right, _) => {
            let l = convert_signed_num(left)?;
            let r = convert_num(right)?;
            match (l, r) {
                (Lit::V(_), Lit::Opaque("complex")) => Ok(Lit::Opaque("complex")),
                _ => convert_signed_num(n),
            }
        }
        _ => convert_signed_num(n),
    }
}

/// ast.literal_eval(s) followed by json.dumps(literal) (the gcode_macro
/// variable check); Err is (exception class, message)
pub fn literal_eval_json(src: &str) -> Result<Value, LitError> {
    literal_eval_json_noted(src).map_err(|(e, _)| e)
}

/// literal_eval_json, with the notes json.dumps added to the exception
/// (traceback lines below the message)
pub fn literal_eval_json_noted(src: &str) -> Result<Value, (LitError, Vec<String>)> {
    let lit = literal_eval(src).map_err(|e| (e, vec![]))?;
    lit.to_json().map_err(|(m, notes)| (("TypeError", m), notes))
}

fn literal_eval(src: &str) -> Result<Lit, LitError> {
    let src = src.trim_start_matches([' ', '\t']);
    let lx = lex(src)?;
    let mut toks = lx.toks;
    // trailing NEWLINE tokens are fine
    while toks.len() >= 2 && matches!(&toks[toks.len() - 2].0, Tok::Op(o) if o == "NEWLINE") {
        toks.remove(toks.len() - 2);
    }
    if toks.len() == 1 {
        return Err(syntax("invalid syntax", 0));
    }
    let mut p = Parser { toks, pos: 0 };
    let first = p.star_expr()?;
    let node = if p.is_op(",") {
        let line = match &first {
            Node::Const(_, l) => *l,
            _ => 1,
        };
        let mut items = vec![first];
        while p.is_op(",") {
            p.next();
            if matches!(p.peek(), Tok::End) {
                break;
            }
            items.push(p.star_expr()?);
        }
        Node::Tuple(items, line)
    } else {
        first
    };
    if p.is_op("NEWLINE") {
        p.next();
    }
    if !matches!(p.peek(), Tok::End) {
        return p.fail();
    }
    convert(Some(&node))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ast.literal_eval + json.dumps errors and notes, as CPython 3.15 gives them
    #[test]
    fn json_notes() {
        let cases: &[(&str, Option<(&str, &str, &[&str])>)] = &[
            ("{1:{2}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item 1"]))),
            ("[1,{\"a\":(1,{3})}]", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing tuple item 1", "when serializing dict item 'a'", "when serializing list item 1"]))),
            ("{(1,2):3}", Some(("TypeError", "keys must be str, int, float, bool or None, not tuple", &[]))),
            ("{1:{(1,2):3}}", Some(("TypeError", "keys must be str, int, float, bool or None, not tuple", &["when serializing dict item 1"]))),
            ("({2},)", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing tuple item 0"]))),
            ("[{2},{3j}]", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing list item 0"]))),
            ("{2:3j}", Some(("TypeError", "Object of type complex is not JSON serializable", &["when serializing dict item 2"]))),
            ("{\"a\":1, \"b\":{2}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item 'b'"]))),
            ("b\"x\"", Some(("TypeError", "Object of type bytes is not JSON serializable", &[]))),
            ("[b\"x\"]", Some(("TypeError", "Object of type bytes is not JSON serializable", &["when serializing list item 0"]))),
            ("{1:2, (1,2):{3}}", Some(("TypeError", "keys must be str, int, float, bool or None, not tuple", &[]))),
            ("{1:{2}, 1:3}", None),
            ("{1:{2}, 1.0:3}", None),
            ("{{1}:2}", Some(("TypeError", "cannot use 'set' as a dict key (unhashable type: 'set')", &[]))),
            ("{[1]:2}", Some(("TypeError", "cannot use 'list' as a dict key (unhashable type: 'list')", &[]))),
            ("{(1,[2]):3}", Some(("TypeError", "cannot use 'tuple' as a dict key (unhashable type: 'list')", &[]))),
            ("{(1,{2}):3}", Some(("TypeError", "cannot use 'tuple' as a dict key (unhashable type: 'set')", &[]))),
            ("{b\"x\":1}", Some(("TypeError", "keys must be str, int, float, bool or None, not bytes", &[]))),
            ("{1j:1}", Some(("TypeError", "keys must be str, int, float, bool or None, not complex", &[]))),
            ("{(1,b\"x\"):1}", Some(("TypeError", "keys must be str, int, float, bool or None, not tuple", &[]))),
            ("{(1,2):{3}}", Some(("TypeError", "keys must be str, int, float, bool or None, not tuple", &[]))),
            ("{\"a\":{1:{2}}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item 1", "when serializing dict item 'a'"]))),
            ("[1, 2, 3j]", Some(("TypeError", "Object of type complex is not JSON serializable", &["when serializing list item 2"]))),
            ("1+2j", Some(("TypeError", "Object of type complex is not JSON serializable", &[]))),
            ("{True:{2}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item True"]))),
            ("{None:{2}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item None"]))),
            ("{1.5:{2}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item 1.5"]))),
            ("{\"a'b\":{2}}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing dict item \"a'b\""]))),
            ("{1:[2,(3,{4})]}", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing tuple item 1", "when serializing list item 1", "when serializing dict item 1"]))),
            ("{1:2, {1}:3}", Some(("TypeError", "cannot use 'set' as a dict key (unhashable type: 'set')", &[]))),
            ("{{1}:{2}}", Some(("TypeError", "cannot use 'set' as a dict key (unhashable type: 'set')", &[]))),
            ("set()", Some(("TypeError", "Object of type set is not JSON serializable", &[]))),
            ("[set()]", Some(("TypeError", "Object of type set is not JSON serializable", &["when serializing list item 0"]))),
        ];
        for (src, want) in cases {
            let got = literal_eval_json_noted(src).err();
            let got = got.as_ref().map(|((c, m), n)| (*c, m.as_str(), n.iter().map(String::as_str).collect::<Vec<_>>()));
            let want = want.map(|(c, m, n)| (c, m, n.to_vec()));
            assert_eq!(got, want, "{}", src);
        }
    }
}
