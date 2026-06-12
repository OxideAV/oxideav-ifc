//! Crate-local error type for the ISO 10303-21 parser.
//!
//! The standalone (`--no-default-features`) build exposes exactly this
//! type; the `registry`-feature decoder maps it onto the framework
//! error vocabulary at the trait boundary.

use std::fmt;

/// Errors produced while parsing a STEP physical file (ISO 10303-21).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Lexical or syntactic violation of the ISO 10303-21 grammar.
    /// `line` / `column` are 1-based and point at the offending token.
    Syntax {
        /// 1-based line of the offending byte/token.
        line: usize,
        /// 1-based column of the offending byte/token.
        column: usize,
        /// Human-readable description of the violation.
        message: String,
    },
    /// A configured [`StepLimits`](crate::StepLimits) cap was exceeded
    /// — the canonical "DoS protection fired" rejection.
    LimitExceeded(String),
    /// The HEADER section is missing a mandatory record
    /// (`FILE_DESCRIPTION` / `FILE_NAME` / `FILE_SCHEMA`, ISO 10303-21
    /// §8) or a mandatory record is malformed.
    Header(String),
    /// Two instance records in the DATA section share the same `#id`
    /// (instance names must be unique per ISO 10303-21 §9).
    DuplicateId(u64),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Syntax {
                line,
                column,
                message,
            } => write!(f, "syntax error at {line}:{column}: {message}"),
            Self::LimitExceeded(msg) => write!(f, "limit exceeded: {msg}"),
            Self::Header(msg) => write!(f, "invalid HEADER section: {msg}"),
            Self::DuplicateId(id) => write!(f, "duplicate instance id #{id}"),
        }
    }
}

impl std::error::Error for Error {}

/// Convenience alias used throughout the crate.
pub type Result<T> = std::result::Result<T, Error>;
