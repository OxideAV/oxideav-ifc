//! [`Value`] — one parsed STEP parameter (ISO 10303-21 §10 parameter
//! grammar).
//!
//! Every argument slot of an instance record (`#id = ENTITY(args);`)
//! parses into exactly one `Value`. Aggregates and typed parameters
//! nest recursively (bounded by
//! [`StepLimits::max_depth`](crate::StepLimits)).

/// A single STEP parameter value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// `$` — optional attribute not provided.
    Unset,
    /// `*` — attribute derived by a schema rule, not serialised.
    Derived,
    /// Signed decimal integer, e.g. `42`, `-7`.
    Integer(i64),
    /// Real literal, e.g. `1.5`, `-2.7E-3`, `0.`, `.5`.
    Real(f64),
    /// `'...'` string after escape decoding (quote doubling, `\\`,
    /// `\X\` / `\X2\` / `\X4\` / `\S\` directives).
    String(String),
    /// `.NAME.` enumeration literal, stored without the delimiting
    /// dots and upper-cased (`.ADDED.` → `ADDED`). The logical
    /// literals `.T.` / `.F.` / `.U.` arrive here as `T` / `F` / `U`.
    Enum(String),
    /// `"<hex>"` binary literal, stored as the raw upper-cased hex
    /// digit string (the first digit is the unused-bit count per
    /// ISO 10303-21 §10).
    Binary(String),
    /// `#id` reference to another instance in the DATA section.
    Reference(u64),
    /// Typed (constructor-wrapped) parameter, e.g. `IFCLABEL('x')` —
    /// the explicit-variant form used for SELECT-typed attributes.
    Typed {
        /// Upper-cased type keyword (`IFCLABEL`, …).
        keyword: String,
        /// The wrapped parameter list (almost always exactly one).
        args: Vec<Value>,
    },
    /// `( ... )` aggregate (LIST / SET / BAG / ARRAY all serialise
    /// identically).
    List(Vec<Value>),
}

impl Value {
    /// True for the `$` placeholder.
    pub fn is_unset(&self) -> bool {
        matches!(self, Self::Unset)
    }

    /// Integer payload, if this is an `Integer`.
    pub fn as_integer(&self) -> Option<i64> {
        match self {
            Self::Integer(v) => Some(*v),
            _ => None,
        }
    }

    /// Numeric payload widened to `f64` (`Real` as-is, `Integer`
    /// converted) — convenient because writers sometimes emit `0`
    /// where the schema says REAL.
    pub fn as_number(&self) -> Option<f64> {
        match self {
            Self::Real(v) => Some(*v),
            Self::Integer(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Real payload, if this is a `Real`.
    pub fn as_real(&self) -> Option<f64> {
        match self {
            Self::Real(v) => Some(*v),
            _ => None,
        }
    }

    /// String payload, if this is a `String`.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Self::String(s) => Some(s),
            _ => None,
        }
    }

    /// Enumeration payload (without dots), if this is an `Enum`.
    pub fn as_enum(&self) -> Option<&str> {
        match self {
            Self::Enum(s) => Some(s),
            _ => None,
        }
    }

    /// Referenced instance id, if this is a `Reference`.
    pub fn as_reference(&self) -> Option<u64> {
        match self {
            Self::Reference(id) => Some(*id),
            _ => None,
        }
    }

    /// Aggregate items, if this is a `List`.
    pub fn as_list(&self) -> Option<&[Value]> {
        match self {
            Self::List(items) => Some(items),
            _ => None,
        }
    }

    /// Typed-parameter parts, if this is a `Typed` value.
    pub fn as_typed(&self) -> Option<(&str, &[Value])> {
        match self {
            Self::Typed { keyword, args } => Some((keyword, args)),
            _ => None,
        }
    }

    /// Append every `#id` referenced anywhere inside this value
    /// (recursing through aggregates and typed parameters) to `out`.
    pub fn collect_references(&self, out: &mut Vec<u64>) {
        match self {
            Self::Reference(id) => out.push(*id),
            Self::List(items) => {
                for item in items {
                    item.collect_references(out);
                }
            }
            Self::Typed { args, .. } => {
                for arg in args {
                    arg.collect_references(out);
                }
            }
            _ => {}
        }
    }
}
