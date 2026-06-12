//! Recursive-descent parser for the ISO 10303-21 exchange structure.
//!
//! Top-level shape (ISO 10303-21 §7):
//!
//! ```text
//! ISO-10303-21;
//! HEADER;   <header records>   ENDSEC;
//! DATA;     <instance records> ENDSEC;     (repeatable)
//! END-ISO-10303-21;
//! ```
//!
//! Entry points: [`parse_step`] / [`parse_step_with_limits`] /
//! [`probe_step`].

use std::collections::{BTreeMap, BTreeSet};

use crate::error::{Error, Result};
use crate::header::{Header, HeaderRecord};
use crate::lexer::{Lexer, Token};
use crate::value::Value;

/// DoS-hardening caps applied while parsing. All caps reject the
/// input with [`Error::LimitExceeded`] when crossed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepLimits {
    /// Maximum accepted input size in bytes.
    pub max_input_len: usize,
    /// Maximum number of instance records across all DATA sections.
    pub max_instances: usize,
    /// Maximum nesting depth of aggregates / typed parameters inside
    /// one parameter list.
    pub max_depth: usize,
    /// Maximum decoded length of a single string or binary literal.
    pub max_string_len: usize,
}

impl Default for StepLimits {
    fn default() -> Self {
        Self {
            // 256 MiB of text is far beyond any sane building model
            // delivered as clear text (.ifcZIP exists for a reason).
            max_input_len: 256 * 1024 * 1024,
            max_instances: 8_000_000,
            max_depth: 64,
            max_string_len: 16 * 1024 * 1024,
        }
    }
}

/// One DATA-section instance record: `#id = KEYWORD(args);`.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedInstance {
    /// The `#id` instance name (unique per file).
    pub id: u64,
    /// Upper-cased entity keyword, e.g. `IFCWALL`.
    pub keyword: String,
    /// The parameter list, in serialisation order.
    pub args: Vec<Value>,
}

/// A fully parsed STEP physical file.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StepFile {
    /// Typed + raw HEADER section.
    pub header: Header,
    /// Instance graph, keyed by `#id` (sorted map so iteration order
    /// is deterministic).
    pub instances: BTreeMap<u64, ParsedInstance>,
}

impl StepFile {
    /// Number of instance records.
    pub fn len(&self) -> usize {
        self.instances.len()
    }

    /// True when the DATA section holds no instances.
    pub fn is_empty(&self) -> bool {
        self.instances.is_empty()
    }

    /// Look up an instance by id.
    pub fn get(&self, id: u64) -> Option<&ParsedInstance> {
        self.instances.get(&id)
    }

    /// Resolve a [`Value::Reference`] to its target instance. Returns
    /// `None` for non-reference values and for dangling references.
    pub fn resolve<'a>(&'a self, value: &Value) -> Option<&'a ParsedInstance> {
        self.instances.get(&value.as_reference()?)
    }

    /// Iterate all instances of one entity keyword
    /// (case-insensitive: `"IfcWall"` matches `IFCWALL` records).
    pub fn instances_of<'a>(&'a self, keyword: &str) -> impl Iterator<Item = &'a ParsedInstance> {
        let want = keyword.to_ascii_uppercase();
        self.instances
            .values()
            .filter(move |inst| inst.keyword == want)
    }

    /// Every `#id` referenced (at any nesting depth) by the instance
    /// with id `id`. Empty when the instance does not exist.
    pub fn references_of(&self, id: u64) -> Vec<u64> {
        let mut out = Vec::new();
        if let Some(inst) = self.instances.get(&id) {
            for arg in &inst.args {
                arg.collect_references(&mut out);
            }
        }
        out
    }

    /// Transitive closure of instances reachable from `id` by
    /// following entity references. Cycle-safe (visited-set walk —
    /// reference cycles are normal in IFC instance graphs). The
    /// starting instance is included when it exists.
    pub fn reachable_from(&self, id: u64) -> BTreeSet<u64> {
        let mut seen = BTreeSet::new();
        let mut stack = Vec::new();
        if self.instances.contains_key(&id) {
            seen.insert(id);
            stack.push(id);
        }
        while let Some(cur) = stack.pop() {
            for next in self.references_of(cur) {
                if self.instances.contains_key(&next) && seen.insert(next) {
                    stack.push(next);
                }
            }
        }
        seen
    }

    /// All `(referencing_instance, missing_target)` pairs where an
    /// instance references an id that does not exist in the file.
    pub fn dangling_references(&self) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        for (&id, _) in self.instances.iter() {
            for target in self.references_of(id) {
                if !self.instances.contains_key(&target) {
                    out.push((id, target));
                }
            }
        }
        out
    }
}

/// Cheap magic probe: true when the first token of `bytes` is the
/// `ISO-10303-21;` exchange-structure sentinel (leading whitespace,
/// comments, and a UTF-8 BOM are tolerated).
pub fn probe_step(bytes: &[u8]) -> bool {
    let mut lexer = Lexer::new(bytes, 4096);
    matches!(lexer.next_token(), Ok(Token::Keyword(kw)) if kw == "ISO-10303-21")
        && matches!(lexer.next_token(), Ok(Token::Semicolon))
}

/// Parse a STEP physical file with default [`StepLimits`].
pub fn parse_step(bytes: &[u8]) -> Result<StepFile> {
    parse_step_with_limits(bytes, &StepLimits::default())
}

/// Parse a STEP physical file with caller-supplied limits.
pub fn parse_step_with_limits(bytes: &[u8], limits: &StepLimits) -> Result<StepFile> {
    if bytes.len() > limits.max_input_len {
        return Err(Error::LimitExceeded(format!(
            "input is {} bytes, cap is {}",
            bytes.len(),
            limits.max_input_len
        )));
    }
    Parser::new(bytes, limits).parse_file()
}

struct Parser<'a> {
    lexer: Lexer<'a>,
    /// One-token lookahead plus its source position.
    tok: Token,
    tok_line: usize,
    tok_col: usize,
    limits: &'a StepLimits,
}

impl<'a> Parser<'a> {
    fn new(bytes: &'a [u8], limits: &'a StepLimits) -> Self {
        Self {
            lexer: Lexer::new(bytes, limits.max_string_len),
            tok: Token::Eof,
            tok_line: 1,
            tok_col: 1,
            limits,
        }
    }

    fn err_here(&self, message: impl Into<String>) -> Error {
        Error::Syntax {
            line: self.tok_line,
            column: self.tok_col,
            message: message.into(),
        }
    }

    /// Advance to the next token, returning the previous one.
    fn bump(&mut self) -> Result<Token> {
        let next = self.lexer.next_token()?;
        let prev = std::mem::replace(&mut self.tok, next);
        self.tok_line = self.lexer.tok_line;
        self.tok_col = self.lexer.tok_col;
        Ok(prev)
    }

    fn expect(&mut self, want: &Token, ctx: &str) -> Result<()> {
        if &self.tok == want {
            self.bump()?;
            Ok(())
        } else {
            Err(self.err_here(format!(
                "expected {} {ctx}, found {}",
                want.describe(),
                self.tok.describe()
            )))
        }
    }

    fn expect_keyword(&mut self, want: &str, ctx: &str) -> Result<()> {
        match &self.tok {
            Token::Keyword(kw) if kw == want => {
                self.bump()?;
                Ok(())
            }
            other => Err(self.err_here(format!(
                "expected `{want}` {ctx}, found {}",
                other.describe()
            ))),
        }
    }

    fn at_keyword(&self, want: &str) -> bool {
        matches!(&self.tok, Token::Keyword(kw) if kw == want)
    }

    fn parse_file(&mut self) -> Result<StepFile> {
        // Prime the lookahead.
        self.bump()?;

        self.expect_keyword("ISO-10303-21", "at start of file")?;
        self.expect(&Token::Semicolon, "after `ISO-10303-21`")?;

        // HEADER section.
        self.expect_keyword("HEADER", "to open the header section")?;
        self.expect(&Token::Semicolon, "after `HEADER`")?;
        let mut records = Vec::new();
        while !self.at_keyword("ENDSEC") {
            let keyword = match self.bump()? {
                Token::Keyword(kw) => kw,
                other => {
                    return Err(self.err_here(format!(
                        "expected a header record keyword, found {}",
                        other.describe()
                    )));
                }
            };
            let args = self.parse_paren_args(0)?;
            self.expect(&Token::Semicolon, "after header record")?;
            records.push(HeaderRecord { keyword, args });
        }
        self.expect_keyword("ENDSEC", "to close the header section")?;
        self.expect(&Token::Semicolon, "after `ENDSEC`")?;
        let header = Header::from_records(records)?;

        // One or more DATA sections (multi-section files are legal,
        // single-section is the IFC norm).
        let mut instances: BTreeMap<u64, ParsedInstance> = BTreeMap::new();
        let mut saw_data = false;
        while self.at_keyword("DATA") {
            saw_data = true;
            self.bump()?;
            if self.tok == Token::LParen {
                // Optional section parameters (section name + schema
                // list in the multi-section form) — parsed and
                // discarded.
                self.parse_paren_args(0)?;
            }
            self.expect(&Token::Semicolon, "after `DATA`")?;
            while !self.at_keyword("ENDSEC") {
                let id = match self.bump()? {
                    Token::Reference(id) => id,
                    other => {
                        return Err(self.err_here(format!(
                            "expected `#id` to start an instance record, found {}",
                            other.describe()
                        )));
                    }
                };
                self.expect(&Token::Equals, "after instance id")?;
                let keyword = match self.bump()? {
                    Token::Keyword(kw) => kw,
                    Token::LParen => {
                        // External-mapping (multi-keyword complex
                        // entity) records are not used by IFC writers.
                        return Err(self.err_here(
                            "external-mapping complex entity records are not supported",
                        ));
                    }
                    other => {
                        return Err(self.err_here(format!(
                            "expected an entity keyword, found {}",
                            other.describe()
                        )));
                    }
                };
                let args = self.parse_paren_args(0)?;
                self.expect(&Token::Semicolon, "after instance record")?;
                if instances.len() >= self.limits.max_instances {
                    return Err(Error::LimitExceeded(format!(
                        "more than {} instance records",
                        self.limits.max_instances
                    )));
                }
                if instances
                    .insert(id, ParsedInstance { id, keyword, args })
                    .is_some()
                {
                    return Err(Error::DuplicateId(id));
                }
            }
            self.expect_keyword("ENDSEC", "to close the data section")?;
            self.expect(&Token::Semicolon, "after `ENDSEC`")?;
        }
        if !saw_data {
            return Err(self.err_here("expected a `DATA` section"));
        }

        self.expect_keyword("END-ISO-10303-21", "at end of file")?;
        self.expect(&Token::Semicolon, "after `END-ISO-10303-21`")?;
        if self.tok != Token::Eof {
            return Err(self.err_here(format!(
                "trailing content after `END-ISO-10303-21;`: {}",
                self.tok.describe()
            )));
        }

        Ok(StepFile { header, instances })
    }

    /// Parse `( arg, arg, ... )`. A trailing comma before `)` is
    /// tolerated (known writer deviation).
    fn parse_paren_args(&mut self, depth: usize) -> Result<Vec<Value>> {
        if depth > self.limits.max_depth {
            return Err(Error::LimitExceeded(format!(
                "parameter nesting deeper than {}",
                self.limits.max_depth
            )));
        }
        self.expect(&Token::LParen, "to open a parameter list")?;
        let mut args = Vec::new();
        if self.tok == Token::RParen {
            self.bump()?;
            return Ok(args);
        }
        loop {
            args.push(self.parse_argument(depth)?);
            match &self.tok {
                Token::Comma => {
                    self.bump()?;
                    if self.tok == Token::RParen {
                        // Trailing comma tolerance.
                        self.bump()?;
                        return Ok(args);
                    }
                }
                Token::RParen => {
                    self.bump()?;
                    return Ok(args);
                }
                other => {
                    return Err(self.err_here(format!(
                        "expected `,` or `)` in parameter list, found {}",
                        other.describe()
                    )));
                }
            }
        }
    }

    fn parse_argument(&mut self, depth: usize) -> Result<Value> {
        match &self.tok {
            Token::Dollar => {
                self.bump()?;
                Ok(Value::Unset)
            }
            Token::Star => {
                self.bump()?;
                Ok(Value::Derived)
            }
            Token::Integer(_) => match self.bump()? {
                Token::Integer(v) => Ok(Value::Integer(v)),
                _ => unreachable!(),
            },
            Token::Real(_) => match self.bump()? {
                Token::Real(v) => Ok(Value::Real(v)),
                _ => unreachable!(),
            },
            Token::Str(_) => match self.bump()? {
                Token::Str(s) => Ok(Value::String(s)),
                _ => unreachable!(),
            },
            Token::Enum(_) => match self.bump()? {
                Token::Enum(e) => Ok(Value::Enum(e)),
                _ => unreachable!(),
            },
            Token::Binary(_) => match self.bump()? {
                Token::Binary(b) => Ok(Value::Binary(b)),
                _ => unreachable!(),
            },
            Token::Reference(_) => match self.bump()? {
                Token::Reference(id) => Ok(Value::Reference(id)),
                _ => unreachable!(),
            },
            Token::LParen => {
                if depth + 1 > self.limits.max_depth {
                    return Err(Error::LimitExceeded(format!(
                        "parameter nesting deeper than {}",
                        self.limits.max_depth
                    )));
                }
                let items = self.parse_paren_args(depth + 1)?;
                Ok(Value::List(items))
            }
            Token::Keyword(_) => {
                let keyword = match self.bump()? {
                    Token::Keyword(kw) => kw,
                    _ => unreachable!(),
                };
                let args = self.parse_paren_args(depth + 1)?;
                Ok(Value::Typed { keyword, args })
            }
            other => Err(self.err_here(format!(
                "expected a parameter value, found {}",
                other.describe()
            ))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Wrap a DATA-section body in a minimal valid exchange structure.
    fn wrap(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t.ifc','2026-06-12T00:00:00',('a'),('o'),'p','s','auth');\n\
             FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    fn parse(data: &str) -> StepFile {
        parse_step(wrap(data).as_bytes()).expect("parse failed")
    }

    fn arg(file: &StepFile, id: u64, idx: usize) -> Value {
        file.get(id).expect("instance missing").args[idx].clone()
    }

    #[test]
    fn minimal_file_and_header() {
        let f = parse("#1=IFCWALL('x',$,*,1,2.5,.ADDED.,(#1),#1);");
        assert_eq!(f.len(), 1);
        assert_eq!(f.header.file_schema, ["IFC4"]);
        assert_eq!(f.header.file_description.implementation_level, "2;1");
        assert_eq!(f.header.file_name.name, "t.ifc");
        assert_eq!(f.header.file_name.author, ["a"]);
        assert_eq!(f.header.file_name.authorization, "auth");
        assert_eq!(f.header.records.len(), 3);
    }

    #[test]
    fn all_parameter_forms() {
        let f = parse(
            "#7=IFCTHING($,*,42,-7,1.5,-2.7E-3,0.,.5,'s',.NOTDEFINED.,#9,\
             (#9,1,()),IFCLABEL('w'),\"0AFF\");\n#9=IFCOTHER();",
        );
        let inst = f.get(7).unwrap();
        assert_eq!(inst.keyword, "IFCTHING");
        assert_eq!(inst.args[0], Value::Unset);
        assert_eq!(inst.args[1], Value::Derived);
        assert_eq!(inst.args[2], Value::Integer(42));
        assert_eq!(inst.args[3], Value::Integer(-7));
        assert_eq!(inst.args[4], Value::Real(1.5));
        assert_eq!(inst.args[5], Value::Real(-2.7e-3));
        assert_eq!(inst.args[6], Value::Real(0.0));
        assert_eq!(inst.args[7], Value::Real(0.5));
        assert_eq!(inst.args[8], Value::String("s".into()));
        assert_eq!(inst.args[9], Value::Enum("NOTDEFINED".into()));
        assert_eq!(inst.args[10], Value::Reference(9));
        assert_eq!(
            inst.args[11],
            Value::List(vec![
                Value::Reference(9),
                Value::Integer(1),
                Value::List(vec![]),
            ])
        );
        assert_eq!(
            inst.args[12],
            Value::Typed {
                keyword: "IFCLABEL".into(),
                args: vec![Value::String("w".into())],
            }
        );
        assert_eq!(inst.args[13], Value::Binary("0AFF".into()));
    }

    #[test]
    fn tolerant_real_exponent_without_dot() {
        let f = parse("#1=IFCA(1E6);");
        assert_eq!(arg(&f, 1, 0), Value::Real(1.0e6));
    }

    #[test]
    fn string_quote_doubling_and_backslash() {
        let f = parse(r"#1=IFCA('it''s a \\ test','');");
        assert_eq!(arg(&f, 1, 0), Value::String("it's a \\ test".into()));
        assert_eq!(arg(&f, 1, 1), Value::String(String::new()));
    }

    #[test]
    fn string_x_escape_latin1() {
        let f = parse(r"#1=IFCA('caf\X\E9');");
        assert_eq!(arg(&f, 1, 0), Value::String("café".into()));
    }

    #[test]
    fn string_x2_escape_run_with_terminator() {
        // Two BMP codepoints in one \X2\ run with explicit \X0\.
        let f = parse(r"#1=IFCA('a\X2\30C630B9\X0\b');");
        assert_eq!(arg(&f, 1, 0), Value::String("aテスb".into()));
    }

    #[test]
    fn string_x2_escape_terminator_omitted_at_quote() {
        let f = parse(r"#1=IFCA('\X2\00E9');");
        assert_eq!(arg(&f, 1, 0), Value::String("é".into()));
    }

    #[test]
    fn string_x4_escape() {
        let f = parse(r"#1=IFCA('\X4\0001F600\X0\!');");
        assert_eq!(arg(&f, 1, 0), Value::String("😀!".into()));
    }

    #[test]
    fn string_s_escape_page_a() {
        // \S\c maps to c + 0x80 in ISO 8859-1: 'i' (0x69) → 0xE9 'é'.
        let f = parse(r"#1=IFCA('caf\S\i');");
        assert_eq!(arg(&f, 1, 0), Value::String("café".into()));
    }

    #[test]
    fn string_s_escape_non_default_page_rejected() {
        let res = parse_step(wrap(r"#1=IFCA('\PB\\S\i');").as_bytes());
        assert!(matches!(res, Err(Error::Syntax { .. })), "{res:?}");
    }

    #[test]
    fn string_raw_utf8_passthrough_and_latin1_fallback() {
        // Raw UTF-8 bytes (tolerated writer deviation).
        let f = parse_step(wrap("#1=IFCA('caf\u{e9}');").as_bytes()).unwrap();
        assert_eq!(arg(&f, 1, 0), Value::String("café".into()));
        // A lone 0xE9 byte is not valid UTF-8 → Latin-1 fallback.
        let mut bytes = wrap("").into_bytes();
        let needle = b"DATA;\n".as_slice();
        let at = bytes
            .windows(needle.len())
            .position(|w| w == needle)
            .unwrap()
            + needle.len();
        bytes.splice(at..at, b"#1=IFCA('caf\xE9');\n".iter().copied());
        let f = parse_step(&bytes).unwrap();
        assert_eq!(arg(&f, 1, 0), Value::String("café".into()));
    }

    #[test]
    fn string_raw_newline_rejected() {
        let res = parse_step(wrap("#1=IFCA('a\nb');").as_bytes());
        assert!(matches!(res, Err(Error::Syntax { .. })), "{res:?}");
    }

    #[test]
    fn comments_and_multiline_records() {
        let f = parse(
            "/* leading */ #1 = IFCA( /* inner */ 'x',\n   #2 /* trailing arg */ );\n\
             #2=IFCB();",
        );
        assert_eq!(f.len(), 2);
        assert_eq!(arg(&f, 1, 1), Value::Reference(2));
    }

    #[test]
    fn unterminated_comment_rejected() {
        let res = parse_step(b"ISO-10303-21; /* oops");
        assert!(matches!(res, Err(Error::Syntax { .. })), "{res:?}");
    }

    #[test]
    fn trailing_comma_tolerated() {
        let f = parse("#1=IFCA((1,2,),3,);");
        assert_eq!(
            arg(&f, 1, 0),
            Value::List(vec![Value::Integer(1), Value::Integer(2)])
        );
        assert_eq!(arg(&f, 1, 1), Value::Integer(3));
    }

    #[test]
    fn forward_references_resolve() {
        let f = parse("#1=IFCA(#5);\n#5=IFCB('later');");
        let target = f.resolve(&arg(&f, 1, 0)).unwrap();
        assert_eq!(target.keyword, "IFCB");
        assert!(f.dangling_references().is_empty());
    }

    #[test]
    fn dangling_reference_detected() {
        let f = parse("#1=IFCA(#99);");
        assert_eq!(f.dangling_references(), vec![(1, 99)]);
    }

    #[test]
    fn reachability_is_cycle_safe() {
        let f = parse("#1=IFCA(#2);\n#2=IFCB(#1,(#3));\n#3=IFCC();\n#4=IFCD();");
        let reach = f.reachable_from(1);
        assert_eq!(reach.into_iter().collect::<Vec<_>>(), vec![1, 2, 3]);
    }

    #[test]
    fn duplicate_id_rejected() {
        let res = parse_step(wrap("#1=IFCA();\n#1=IFCB();").as_bytes());
        assert_eq!(res.unwrap_err(), Error::DuplicateId(1));
    }

    #[test]
    fn zero_id_rejected() {
        let res = parse_step(wrap("#0=IFCA();").as_bytes());
        assert!(matches!(res, Err(Error::Syntax { .. })), "{res:?}");
    }

    #[test]
    fn missing_header_record_rejected() {
        let res = parse_step(
            b"ISO-10303-21;HEADER;FILE_DESCRIPTION((''),'2;1');\
              FILE_SCHEMA(('IFC4'));ENDSEC;DATA;ENDSEC;END-ISO-10303-21;",
        );
        assert!(matches!(res, Err(Error::Header(_))), "{res:?}");
    }

    #[test]
    fn missing_magic_rejected_and_probe() {
        assert!(matches!(
            parse_step(b"HELLO;"),
            Err(Error::Syntax { line: 1, .. })
        ));
        assert!(!probe_step(b"HELLO;"));
        assert!(!probe_step(b""));
        assert!(probe_step(b"  /* c */ ISO-10303-21;"));
        assert!(probe_step(&{
            let mut v = vec![0xEF, 0xBB, 0xBF];
            v.extend_from_slice(b"ISO-10303-21;");
            v
        }));
    }

    #[test]
    fn trailing_garbage_rejected() {
        let mut text = wrap("#1=IFCA();");
        text.push_str("EXTRA;");
        let res = parse_step(text.as_bytes());
        assert!(matches!(res, Err(Error::Syntax { .. })), "{res:?}");
    }

    #[test]
    fn multiple_data_sections_merge() {
        let f = parse_step(
            b"ISO-10303-21;HEADER;FILE_DESCRIPTION((''),'2;1');\
              FILE_NAME('','',(),(),'','','');FILE_SCHEMA(('IFC4'));ENDSEC;\
              DATA('one',('IFC4'));#1=IFCA();ENDSEC;\
              DATA;#2=IFCB();ENDSEC;END-ISO-10303-21;",
        )
        .unwrap();
        assert_eq!(f.len(), 2);
        assert!(f.get(1).is_some() && f.get(2).is_some());
    }

    #[test]
    fn depth_cap_fires() {
        let mut deep = String::from("#1=IFCA(");
        deep.push_str(&"(".repeat(80));
        deep.push('1');
        deep.push_str(&")".repeat(80));
        deep.push_str(");");
        let res = parse_step(wrap(&deep).as_bytes());
        assert!(matches!(res, Err(Error::LimitExceeded(_))), "{res:?}");
        // Within the cap parses fine.
        let limits = StepLimits {
            max_depth: 100,
            ..StepLimits::default()
        };
        assert!(parse_step_with_limits(wrap(&deep).as_bytes(), &limits).is_ok());
    }

    #[test]
    fn instance_count_cap_fires() {
        let limits = StepLimits {
            max_instances: 2,
            ..StepLimits::default()
        };
        let res =
            parse_step_with_limits(wrap("#1=IFCA();#2=IFCA();#3=IFCA();").as_bytes(), &limits);
        assert!(matches!(res, Err(Error::LimitExceeded(_))), "{res:?}");
    }

    #[test]
    fn input_len_cap_fires() {
        let limits = StepLimits {
            max_input_len: 16,
            ..StepLimits::default()
        };
        let res = parse_step_with_limits(wrap("#1=IFCA();").as_bytes(), &limits);
        assert!(matches!(res, Err(Error::LimitExceeded(_))), "{res:?}");
    }

    #[test]
    fn string_len_cap_fires() {
        let limits = StepLimits {
            max_string_len: 4,
            ..StepLimits::default()
        };
        let res = parse_step_with_limits(wrap("#1=IFCA('longer than four');").as_bytes(), &limits);
        assert!(matches!(res, Err(Error::LimitExceeded(_))), "{res:?}");
    }

    #[test]
    fn external_mapping_records_rejected_clearly() {
        let res = parse_step(wrap("#1=(IFCA()IFCB());").as_bytes());
        match res {
            Err(Error::Syntax { message, .. }) => {
                assert!(message.contains("external-mapping"), "{message}");
            }
            other => panic!("expected syntax error, got {other:?}"),
        }
    }

    #[test]
    fn instances_of_is_case_insensitive() {
        let f = parse("#1=IFCWALL();#2=IFCWALLTYPE();#3=IFCWALL();");
        assert_eq!(f.instances_of("IfcWall").count(), 2);
        assert_eq!(f.instances_of("IFCWALLTYPE").count(), 1);
    }

    #[test]
    fn lowercase_keywords_and_enums_normalised() {
        let f = parse("#1=ifcwall(.added.);");
        let inst = f.get(1).unwrap();
        assert_eq!(inst.keyword, "IFCWALL");
        assert_eq!(inst.args[0], Value::Enum("ADDED".into()));
    }
}
