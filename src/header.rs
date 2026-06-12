//! Typed HEADER section (ISO 10303-21 §8).
//!
//! Three header entities are mandatory: `FILE_DESCRIPTION`,
//! `FILE_NAME`, `FILE_SCHEMA`. They are surfaced both typed (the
//! structs below) and raw (every header record, including optional
//! ones such as `FILE_POPULATION`, is kept in [`Header::records`]).

use crate::error::{Error, Result};
use crate::value::Value;

/// One raw header record (`KEYWORD(args);` — no `#id`).
#[derive(Debug, Clone, PartialEq)]
pub struct HeaderRecord {
    /// Upper-cased keyword, e.g. `FILE_NAME`.
    pub keyword: String,
    /// The record's parameter list.
    pub args: Vec<Value>,
}

/// `FILE_DESCRIPTION` — informal description + implementation level.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FileDescription {
    /// Free-text description lines (often the ViewDefinition tag).
    pub description: Vec<String>,
    /// Implementation level, `'2;1'` for the Part 21 edition-2
    /// conformance-class-1 files that IFC produces.
    pub implementation_level: String,
}

/// `FILE_NAME` — provenance of the exchange structure.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FileName {
    /// Name of the exchange structure (usually the file name).
    pub name: String,
    /// ISO 8601 timestamp string.
    pub time_stamp: String,
    /// Author name(s).
    pub author: Vec<String>,
    /// Authoring organization(s).
    pub organization: Vec<String>,
    /// Preprocessor (writer library) version string.
    pub preprocessor_version: String,
    /// Originating system (authoring application) string.
    pub originating_system: String,
    /// Authorisation string.
    pub authorization: String,
}

/// Parsed HEADER section.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Header {
    /// Typed `FILE_DESCRIPTION` record.
    pub file_description: FileDescription,
    /// Typed `FILE_NAME` record.
    pub file_name: FileName,
    /// Schema identifiers from `FILE_SCHEMA`, e.g. `["IFC4"]`.
    pub file_schema: Vec<String>,
    /// Every raw header record in file order (the three mandatory
    /// ones included), so optional records stay accessible.
    pub records: Vec<HeaderRecord>,
}

/// `$`-tolerant string extraction (`$` ↦ empty string).
fn arg_string(args: &[Value], idx: usize, record: &str) -> Result<String> {
    match args.get(idx) {
        None | Some(Value::Unset) => Ok(String::new()),
        Some(Value::String(s)) => Ok(s.clone()),
        Some(other) => Err(Error::Header(format!(
            "{record} argument {idx} must be a string, found {other:?}"
        ))),
    }
}

/// `$`-tolerant string-list extraction (`$` ↦ empty list).
fn arg_string_list(args: &[Value], idx: usize, record: &str) -> Result<Vec<String>> {
    match args.get(idx) {
        None | Some(Value::Unset) => Ok(Vec::new()),
        Some(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::String(s) => Ok(s.clone()),
                other => Err(Error::Header(format!(
                    "{record} argument {idx} must be a list of strings, found {other:?}"
                ))),
            })
            .collect(),
        Some(other) => Err(Error::Header(format!(
            "{record} argument {idx} must be a list, found {other:?}"
        ))),
    }
}

impl Header {
    /// Build the typed header from the raw record list, enforcing the
    /// three mandatory entities.
    pub(crate) fn from_records(records: Vec<HeaderRecord>) -> Result<Self> {
        let find = |kw: &str| records.iter().find(|r| r.keyword == kw);

        let fd = find("FILE_DESCRIPTION")
            .ok_or_else(|| Error::Header("missing mandatory FILE_DESCRIPTION record".into()))?;
        let file_description = FileDescription {
            description: arg_string_list(&fd.args, 0, "FILE_DESCRIPTION")?,
            implementation_level: arg_string(&fd.args, 1, "FILE_DESCRIPTION")?,
        };

        let fnm = find("FILE_NAME")
            .ok_or_else(|| Error::Header("missing mandatory FILE_NAME record".into()))?;
        let file_name = FileName {
            name: arg_string(&fnm.args, 0, "FILE_NAME")?,
            time_stamp: arg_string(&fnm.args, 1, "FILE_NAME")?,
            author: arg_string_list(&fnm.args, 2, "FILE_NAME")?,
            organization: arg_string_list(&fnm.args, 3, "FILE_NAME")?,
            preprocessor_version: arg_string(&fnm.args, 4, "FILE_NAME")?,
            originating_system: arg_string(&fnm.args, 5, "FILE_NAME")?,
            authorization: arg_string(&fnm.args, 6, "FILE_NAME")?,
        };

        let fs = find("FILE_SCHEMA")
            .ok_or_else(|| Error::Header("missing mandatory FILE_SCHEMA record".into()))?;
        let file_schema = arg_string_list(&fs.args, 0, "FILE_SCHEMA")?;
        if file_schema.is_empty() {
            return Err(Error::Header(
                "FILE_SCHEMA must name at least one schema".into(),
            ));
        }

        Ok(Self {
            file_description,
            file_name,
            file_schema,
            records,
        })
    }
}
