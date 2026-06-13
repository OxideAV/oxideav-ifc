//! Pure-Rust IFC (Industry Foundation Classes, ISO 16739) reader.
//!
//! IFC is the open BIM interchange format: a complete building model
//! (spatial hierarchy, classified elements, property sets, geometry)
//! serialised by default as a **STEP physical file** (ISO 10303-21
//! clear-text encoding) with the `.ifc` extension.
//!
//! ## Phase 1 (this release): STEP physical-file parser
//!
//! [`parse_step`] turns the bytes of a `.ifc` file into a typed
//! [`StepFile`]:
//!
//! * **HEADER section** — typed `FILE_DESCRIPTION` / `FILE_NAME` /
//!   `FILE_SCHEMA` records ([`Header`]), with optional header records
//!   kept raw.
//! * **DATA section** — the full instance graph: every
//!   `#id = ENTITY(args);` record becomes a [`ParsedInstance`] whose
//!   arguments are typed [`Value`]s covering the complete Part 21
//!   parameter grammar (`$` unset, `*` derived, integers, reals,
//!   strings with the §6.4.3 escape directives, `.ENUM.` literals,
//!   `"hex"` binaries, typed/SELECT parameters, nested aggregates,
//!   and `#id` entity references — forward references included).
//! * **Graph utilities** — cycle-safe reference resolution
//!   ([`StepFile::resolve`], [`StepFile::reachable_from`]) and
//!   dangling-reference detection.
//! * **DoS hardening** — input-size / instance-count / nesting-depth /
//!   string-length caps via [`StepLimits`].
//!
//! ## Phase 2 (this release): EXPRESS schema typing
//!
//! [`schema`] layers the IFC 4 EXPRESS schema over the positional
//! instance graph for the core entity slice (spatial structure +
//! common building elements + placements + representation refs):
//!
//! * [`TypedEntity`] — names each positional argument per the entity's
//!   inheritance-resolved attribute order, with typed accessors
//!   (`global_id`, `name`, `object_placement`, …).
//! * [`Model`] — folds `IfcRelAggregates` +
//!   `IfcRelContainedInSpatialStructure` into a navigable
//!   project → site → building → storey → space → element tree.
//!
//! Geometry extraction into `Scene3D` is Phase 3.
//!
//! ## Standalone build
//!
//! The framework deps (`oxideav-core`, `oxideav-mesh3d`) sit behind
//! the default-on `registry` cargo feature:
//!
//! ```toml
//! oxideav-ifc = { version = "0.0", default-features = false }
//! ```
//!
//! leaves only the std-typed parser surface (`parse_step`,
//! [`StepFile`], crate-local [`Error`]). With the feature on, the
//! crate additionally exposes [`IfcDecoder`] (a `Mesh3DDecoder`),
//! [`make_decoder`], and [`register_mesh3d`].

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(missing_debug_implementations)]

pub mod error;
pub mod header;
mod lexer;
pub mod parser;
pub mod schema;
pub mod value;

#[cfg(feature = "registry")]
pub mod decoder;

pub use error::{Error, Result};
pub use header::{FileDescription, FileName, Header, HeaderRecord};
pub use parser::{
    parse_step, parse_step_with_limits, probe_step, ParsedInstance, StepFile, StepLimits,
};
pub use schema::{schema_of, EntityKind, EntitySchema, Model, SpatialKind, TypedEntity, SCHEMA};
pub use value::Value;

#[cfg(feature = "registry")]
pub use decoder::{make_decoder, register_mesh3d, IfcDecoder};
