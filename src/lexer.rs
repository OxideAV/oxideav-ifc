//! Tokeniser for the ISO 10303-21 clear-text encoding.
//!
//! Token inventory per the Part 21 lexical grammar: keywords (entity
//! names + section sentinels), integer / real literals, `'...'`
//! strings with the §6.4.3 escape directives, `.ENUM.` literals,
//! `"hex"` binaries, `#id` references, and the punctuation set
//! `( ) , ; = $ *`. Whitespace and `/* ... */` comments separate
//! tokens and are otherwise insignificant.

use crate::error::{Error, Result};

/// One lexical token.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Token {
    /// Entity name / section sentinel, upper-cased. Hyphens are
    /// accepted inside keywords so the `ISO-10303-21` /
    /// `END-ISO-10303-21` sentinels lex as single tokens.
    Keyword(String),
    Integer(i64),
    Real(f64),
    Str(String),
    /// Enumeration literal without the delimiting dots, upper-cased.
    Enum(String),
    /// Binary literal: raw hex digit string, upper-cased.
    Binary(String),
    /// `#id` instance reference.
    Reference(u64),
    LParen,
    RParen,
    Comma,
    Semicolon,
    Equals,
    Dollar,
    Star,
    Eof,
}

impl Token {
    /// Short grammar-level description for error messages.
    pub(crate) fn describe(&self) -> String {
        match self {
            Self::Keyword(k) => format!("keyword `{k}`"),
            Self::Integer(v) => format!("integer `{v}`"),
            Self::Real(v) => format!("real `{v}`"),
            Self::Str(_) => "string literal".into(),
            Self::Enum(e) => format!("enumeration `.{e}.`"),
            Self::Binary(_) => "binary literal".into(),
            Self::Reference(id) => format!("reference `#{id}`"),
            Self::LParen => "`(`".into(),
            Self::RParen => "`)`".into(),
            Self::Comma => "`,`".into(),
            Self::Semicolon => "`;`".into(),
            Self::Equals => "`=`".into(),
            Self::Dollar => "`$`".into(),
            Self::Star => "`*`".into(),
            Self::Eof => "end of input".into(),
        }
    }
}

pub(crate) struct Lexer<'a> {
    input: &'a [u8],
    pos: usize,
    line: usize,
    col: usize,
    /// Line/column of the start of the most recently lexed token.
    pub(crate) tok_line: usize,
    pub(crate) tok_col: usize,
    max_string_len: usize,
}

impl<'a> Lexer<'a> {
    pub(crate) fn new(input: &'a [u8], max_string_len: usize) -> Self {
        let mut lex = Self {
            input,
            pos: 0,
            line: 1,
            col: 1,
            tok_line: 1,
            tok_col: 1,
            max_string_len,
        };
        // Tolerate a UTF-8 byte-order mark before the `ISO-10303-21;`
        // magic (some authoring tools emit one).
        if input.starts_with(&[0xEF, 0xBB, 0xBF]) {
            lex.pos = 3;
        }
        lex
    }

    fn err(&self, message: impl Into<String>) -> Error {
        Error::Syntax {
            line: self.line,
            column: self.col,
            message: message.into(),
        }
    }

    fn peek(&self) -> Option<u8> {
        self.input.get(self.pos).copied()
    }

    fn peek_at(&self, off: usize) -> Option<u8> {
        self.input.get(self.pos + off).copied()
    }

    fn bump(&mut self) -> Option<u8> {
        let b = self.peek()?;
        self.pos += 1;
        if b == b'\n' {
            self.line += 1;
            self.col = 1;
        } else {
            self.col += 1;
        }
        Some(b)
    }

    /// Skip whitespace and `/* ... */` comments (non-nesting).
    fn skip_trivia(&mut self) -> Result<()> {
        loop {
            match self.peek() {
                Some(b' ' | b'\t' | b'\r' | b'\n') => {
                    self.bump();
                }
                Some(b'/') if self.peek_at(1) == Some(b'*') => {
                    let (line, col) = (self.line, self.col);
                    self.bump();
                    self.bump();
                    loop {
                        match self.bump() {
                            Some(b'*') if self.peek() == Some(b'/') => {
                                self.bump();
                                break;
                            }
                            Some(_) => {}
                            None => {
                                return Err(Error::Syntax {
                                    line,
                                    column: col,
                                    message: "unterminated comment".into(),
                                });
                            }
                        }
                    }
                }
                _ => return Ok(()),
            }
        }
    }

    /// Lex the next token.
    pub(crate) fn next_token(&mut self) -> Result<Token> {
        self.skip_trivia()?;
        self.tok_line = self.line;
        self.tok_col = self.col;
        let Some(b) = self.peek() else {
            return Ok(Token::Eof);
        };
        match b {
            b'(' => {
                self.bump();
                Ok(Token::LParen)
            }
            b')' => {
                self.bump();
                Ok(Token::RParen)
            }
            b',' => {
                self.bump();
                Ok(Token::Comma)
            }
            b';' => {
                self.bump();
                Ok(Token::Semicolon)
            }
            b'=' => {
                self.bump();
                Ok(Token::Equals)
            }
            b'$' => {
                self.bump();
                Ok(Token::Dollar)
            }
            b'*' => {
                self.bump();
                Ok(Token::Star)
            }
            b'\'' => self.lex_string(),
            b'"' => self.lex_binary(),
            b'#' => self.lex_reference(),
            b'.' => {
                // `.5` real vs `.ENUM.` literal — disambiguate on the
                // byte after the dot.
                if self.peek_at(1).is_some_and(|c| c.is_ascii_digit()) {
                    self.lex_number()
                } else {
                    self.lex_enum()
                }
            }
            b'+' | b'-' => self.lex_number(),
            b'0'..=b'9' => self.lex_number(),
            b'A'..=b'Z' | b'a'..=b'z' | b'_' => self.lex_keyword(),
            other => Err(self.err(format!("unexpected byte 0x{other:02X}"))),
        }
    }

    fn lex_keyword(&mut self) -> Result<Token> {
        let mut out = String::new();
        while let Some(b) = self.peek() {
            if b.is_ascii_alphanumeric() || b == b'_' || b == b'-' {
                out.push(b.to_ascii_uppercase() as char);
                self.bump();
            } else {
                break;
            }
        }
        Ok(Token::Keyword(out))
    }

    fn lex_reference(&mut self) -> Result<Token> {
        self.bump(); // '#'
        let start = self.pos;
        while self.peek().is_some_and(|b| b.is_ascii_digit()) {
            self.bump();
        }
        if self.pos == start {
            return Err(self.err("expected digits after `#`"));
        }
        // The digit run is pure ASCII by construction.
        let digits = std::str::from_utf8(&self.input[start..self.pos]).unwrap();
        let id: u64 = digits
            .parse()
            .map_err(|_| self.err(format!("instance id `#{digits}` out of range")))?;
        if id == 0 {
            return Err(self.err("instance id `#0` is not valid (ids start at #1)"));
        }
        Ok(Token::Reference(id))
    }

    fn lex_enum(&mut self) -> Result<Token> {
        self.bump(); // '.'
        let mut name = String::new();
        loop {
            match self.peek() {
                Some(b'.') => {
                    self.bump();
                    break;
                }
                Some(b) if b.is_ascii_alphanumeric() || b == b'_' => {
                    name.push(b.to_ascii_uppercase() as char);
                    self.bump();
                }
                Some(other) => {
                    return Err(
                        self.err(format!("invalid byte 0x{other:02X} in enumeration literal"))
                    );
                }
                None => return Err(self.err("unterminated enumeration literal")),
            }
        }
        if name.is_empty() {
            return Err(self.err("empty enumeration literal `..`"));
        }
        Ok(Token::Enum(name))
    }

    fn lex_binary(&mut self) -> Result<Token> {
        self.bump(); // '"'
        let mut hex = String::new();
        loop {
            match self.peek() {
                Some(b'"') => {
                    self.bump();
                    break;
                }
                Some(b) if b.is_ascii_hexdigit() => {
                    if hex.len() >= self.max_string_len {
                        return Err(Error::LimitExceeded(format!(
                            "binary literal longer than {} digits",
                            self.max_string_len
                        )));
                    }
                    hex.push(b.to_ascii_uppercase() as char);
                    self.bump();
                }
                Some(other) => {
                    return Err(self.err(format!("invalid byte 0x{other:02X} in binary literal")));
                }
                None => return Err(self.err("unterminated binary literal")),
            }
        }
        Ok(Token::Binary(hex))
    }

    fn lex_number(&mut self) -> Result<Token> {
        let start = self.pos;
        if matches!(self.peek(), Some(b'+' | b'-')) {
            self.bump();
        }
        let mut saw_digit = false;
        let mut is_real = false;
        while self.peek().is_some_and(|b| b.is_ascii_digit()) {
            saw_digit = true;
            self.bump();
        }
        if self.peek() == Some(b'.') {
            is_real = true;
            self.bump();
            while self.peek().is_some_and(|b| b.is_ascii_digit()) {
                saw_digit = true;
                self.bump();
            }
        }
        if !saw_digit {
            return Err(self.err("expected digits in numeric literal"));
        }
        if matches!(self.peek(), Some(b'E' | b'e')) {
            // Exponent. The strict grammar requires a decimal point
            // before the exponent; `1E6` is accepted tolerantly as a
            // real (a known real-world deviation).
            is_real = true;
            self.bump();
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.bump();
            }
            let mut exp_digits = false;
            while self.peek().is_some_and(|b| b.is_ascii_digit()) {
                exp_digits = true;
                self.bump();
            }
            if !exp_digits {
                return Err(self.err("exponent must have at least one digit"));
            }
        }
        // The scanned run is pure ASCII by construction.
        let text = std::str::from_utf8(&self.input[start..self.pos]).unwrap();
        if is_real {
            let v: f64 = text
                .parse()
                .map_err(|_| self.err(format!("invalid real literal `{text}`")))?;
            Ok(Token::Real(v))
        } else {
            let v: i64 = text
                .parse()
                .map_err(|_| self.err(format!("integer literal `{text}` out of range")))?;
            Ok(Token::Integer(v))
        }
    }

    /// Decode a `'...'` string literal with the ISO 10303-21 §6.4.3
    /// escape directives.
    fn lex_string(&mut self) -> Result<Token> {
        self.bump(); // opening '\''
        let mut out = String::new();
        // Pending run of bytes >= 0x80 (raw non-ASCII passthrough —
        // tolerated per common practice even though strict Part 21
        // requires the \X escapes). Flushed as UTF-8 when the run is
        // well-formed, else as Latin-1, preserving byte order.
        let mut raw: Vec<u8> = Vec::new();
        // Active code page selected by a `\P?\` directive; page A is
        // ISO 8859-1 (the default).
        let mut code_page = b'A';
        loop {
            if out.len() + raw.len() > self.max_string_len {
                return Err(Error::LimitExceeded(format!(
                    "string literal longer than {} bytes",
                    self.max_string_len
                )));
            }
            let Some(b) = self.peek() else {
                return Err(self.err("unterminated string literal"));
            };
            match b {
                b'\'' => {
                    if self.peek_at(1) == Some(b'\'') {
                        // `''` quote doubling → one literal apostrophe.
                        Self::flush_raw(&mut out, &mut raw);
                        out.push('\'');
                        self.bump();
                        self.bump();
                    } else {
                        self.bump();
                        Self::flush_raw(&mut out, &mut raw);
                        return Ok(Token::Str(out));
                    }
                }
                b'\\' => {
                    Self::flush_raw(&mut out, &mut raw);
                    self.lex_string_directive(&mut out, &mut code_page)?;
                }
                b'\n' | b'\r' => {
                    return Err(self.err("raw newline inside string literal"));
                }
                0x20..=0x7E => {
                    Self::flush_raw(&mut out, &mut raw);
                    out.push(b as char);
                    self.bump();
                }
                0x80..=0xFF => {
                    raw.push(b);
                    self.bump();
                }
                other => {
                    return Err(
                        self.err(format!("control byte 0x{other:02X} inside string literal"))
                    );
                }
            }
        }
    }

    /// Flush a pending raw byte run into `out` — UTF-8 if well-formed,
    /// else each byte as its Latin-1 codepoint.
    fn flush_raw(out: &mut String, raw: &mut Vec<u8>) {
        if raw.is_empty() {
            return;
        }
        match std::str::from_utf8(raw) {
            Ok(s) => out.push_str(s),
            Err(_) => {
                for &b in raw.iter() {
                    out.push(b as char);
                }
            }
        }
        raw.clear();
    }

    /// Handle one `\...\` directive; the cursor sits on the leading
    /// backslash.
    fn lex_string_directive(&mut self, out: &mut String, code_page: &mut u8) -> Result<()> {
        self.bump(); // '\\'
        match self.peek() {
            Some(b'\\') => {
                // `\\` → one literal backslash.
                self.bump();
                out.push('\\');
                Ok(())
            }
            Some(b'X') => {
                self.bump();
                match self.peek() {
                    Some(b'\\') => {
                        // `\X\HH` — one ISO 8859-1 codepoint.
                        self.bump();
                        let cp = self.read_hex(2)?;
                        // 0x00..=0xFF always maps to a valid char.
                        out.push(char::from_u32(cp).unwrap());
                        Ok(())
                    }
                    Some(b'2') => {
                        self.bump();
                        self.expect_byte(b'\\')?;
                        self.read_hex_run(out, 4)
                    }
                    Some(b'4') => {
                        self.bump();
                        self.expect_byte(b'\\')?;
                        self.read_hex_run(out, 8)
                    }
                    Some(b'0') => {
                        // Stray `\X0\` outside a run — tolerated no-op.
                        self.bump();
                        self.expect_byte(b'\\')?;
                        Ok(())
                    }
                    _ => Err(self.err("malformed \\X string directive")),
                }
            }
            Some(b'S') => {
                // `\S\c` — codepoint c + 0x80 in the active code page.
                self.bump();
                self.expect_byte(b'\\')?;
                let Some(c) = self.peek() else {
                    return Err(self.err("unterminated \\S\\ directive"));
                };
                if !(0x20..=0x7E).contains(&c) {
                    return Err(self.err("\\S\\ directive operand must be a printable ASCII char"));
                }
                self.bump();
                if *code_page != b'A' {
                    return Err(self.err(format!(
                        "\\S\\ decoding for code page {} is not supported (only page A / ISO 8859-1)",
                        *code_page as char
                    )));
                }
                // Page A: ISO 8859-1 upper half.
                out.push(char::from_u32(c as u32 + 0x80).unwrap());
                Ok(())
            }
            Some(b'P') => {
                // `\P?\` — select the code page for following \S\
                // directives (legacy alphabet escape).
                self.bump();
                let Some(p) = self.peek() else {
                    return Err(self.err("unterminated \\P directive"));
                };
                if !p.is_ascii_uppercase() {
                    return Err(self.err("\\P directive operand must be an upper-case letter"));
                }
                self.bump();
                self.expect_byte(b'\\')?;
                *code_page = p;
                Ok(())
            }
            _ => Err(self.err("unknown string escape directive")),
        }
    }

    fn expect_byte(&mut self, want: u8) -> Result<()> {
        if self.peek() == Some(want) {
            self.bump();
            Ok(())
        } else {
            Err(self.err(format!("expected `{}` in string directive", want as char)))
        }
    }

    /// Read exactly `n` hex digits and return their value.
    fn read_hex(&mut self, n: usize) -> Result<u32> {
        let mut v: u32 = 0;
        for _ in 0..n {
            let Some(b) = self.peek() else {
                return Err(self.err("unterminated hex escape in string literal"));
            };
            let d = (b as char)
                .to_digit(16)
                .ok_or_else(|| self.err(format!("invalid hex digit 0x{b:02X} in string escape")))?;
            v = (v << 4) | d;
            self.bump();
        }
        Ok(v)
    }

    /// Read a run of `width`-digit hex codepoints started by `\X2\` or
    /// `\X4\`. The run ends at an explicit `\X0\` terminator, or
    /// implicitly at the closing quote (the terminator may be omitted
    /// there per ISO 10303-21 §6.4.3).
    fn read_hex_run(&mut self, out: &mut String, width: usize) -> Result<()> {
        loop {
            match self.peek() {
                Some(b'\\') => {
                    // Expect the `\X0\` terminator.
                    self.bump();
                    self.expect_byte(b'X')?;
                    self.expect_byte(b'0')?;
                    self.expect_byte(b'\\')?;
                    return Ok(());
                }
                Some(b'\'') => {
                    // Implicit termination at the closing quote; leave
                    // the quote for the string scanner.
                    return Ok(());
                }
                Some(_) => {
                    let cp = self.read_hex(width)?;
                    let ch = char::from_u32(cp).ok_or_else(|| {
                        self.err(format!(
                            "escape codepoint U+{cp:04X} is not a valid character"
                        ))
                    })?;
                    out.push(ch);
                    if out.len() > self.max_string_len {
                        return Err(Error::LimitExceeded(format!(
                            "string literal longer than {} bytes",
                            self.max_string_len
                        )));
                    }
                }
                None => return Err(self.err("unterminated \\X2\\/\\X4\\ escape run")),
            }
        }
    }
}
