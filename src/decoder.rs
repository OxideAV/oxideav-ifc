//! Framework integration (`registry` feature): a [`Mesh3DDecoder`]
//! implementation plus the [`Mesh3DRegistry`] registration helper.
//!
//! Phase 1 scope: the decoder probes the `ISO-10303-21;` magic and
//! fully parses + validates the exchange structure, then reports the
//! geometry stage as unsupported — IFC tessellation (extracting
//! `IFCTRIANGULATEDFACESET` / `IFCPOLYGONALFACESET` meshes into a
//! `Scene3D`) lands in Phase 3.

use oxideav_core::Error as CoreError;
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DRegistry, Scene3D};

use crate::error::Error;
use crate::parser::{parse_step_with_limits, probe_step, StepLimits};

/// IFC decoder front-end for the OxideAV 3D-format registry.
#[derive(Debug, Clone, Default)]
pub struct IfcDecoder {
    limits: StepLimits,
}

impl IfcDecoder {
    /// Decoder with default [`StepLimits`].
    pub fn new() -> Self {
        Self::default()
    }

    /// Decoder with caller-supplied DoS caps.
    pub fn with_limits(limits: StepLimits) -> Self {
        Self { limits }
    }
}

impl Mesh3DDecoder for IfcDecoder {
    fn decode(&mut self, bytes: &[u8]) -> oxideav_mesh3d::Result<Scene3D> {
        if !probe_step(bytes) {
            return Err(CoreError::InvalidData(
                "not an ISO 10303-21 exchange structure (missing `ISO-10303-21;` magic)".into(),
            ));
        }
        let step = parse_step_with_limits(bytes, &self.limits).map_err(|e| match e {
            Error::LimitExceeded(msg) => CoreError::ResourceExhausted(msg),
            other => CoreError::InvalidData(other.to_string()),
        })?;
        Err(CoreError::Unsupported(format!(
            "IFC geometry extraction is not implemented yet (Phase 3): \
             exchange structure parsed OK ({} instances, schema {})",
            step.len(),
            step.header.file_schema.join("+"),
        )))
    }
}

/// Direct (registry-free) constructor — the conventional `make_`
/// entry point alongside [`register_mesh3d`].
pub fn make_decoder() -> IfcDecoder {
    IfcDecoder::new()
}

/// Register the IFC decoder into a [`Mesh3DRegistry`] under format id
/// `"ifc"` with the `.ifc` file extension.
pub fn register_mesh3d(registry: &mut Mesh3DRegistry) {
    registry.register_decoder("ifc", &["ifc"], Box::new(|| Box::new(IfcDecoder::new())));
}
