//! Analytic surfaces — the `IfcElementarySurface` family
//! (`IfcPlane`, `IfcCylindricalSurface`, `IfcSphericalSurface`,
//! `IfcToroidalSurface`), resolved from their `Position`
//! `IfcAxis2Placement3D` frame and radii (`IFC4X3_ADD2.exp`
//! `IfcElementarySurface.Position`, `IfcCylindricalSurface.Radius`,
//! `IfcSphericalSurface.Radius`, `IfcToroidalSurface.MajorRadius` /
//! `MinorRadius` with the `MajorLargerMinor` WHERE rule).
//!
//! The surface *normal* at a point near the surface is what the
//! surface-curve swept solid needs (the reference direction its profile
//! frame follows); the natural normal is the one implied by the
//! placement: the plane's `Axis` (local +z), the cylinder's outward
//! radial from its axis line, the sphere's outward radial from its
//! centre, and the torus's outward radial from the tube centre circle.

use super::{axis2_placement_3d, dot_raw, normalise, GeometryError, Transform};
use crate::parser::StepFile;
use crate::value::Value;

/// One resolved elementary surface: its placement frame plus the
/// subtype's radii.
#[derive(Debug, Clone)]
pub(super) struct ElementarySurface {
    /// The `Position` frame: origin + orthonormal (x, y, z) columns.
    pub(super) frame: Transform,
    pub(super) kind: SurfaceKind,
}

/// The elementary surface subtypes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum SurfaceKind {
    /// `IfcPlane`: the local xy-plane, normal local +z.
    Plane,
    /// `IfcCylindricalSurface(Radius)`: axis along local z.
    Cylinder { radius: f64 },
    /// `IfcSphericalSurface(Radius)`: centred on the origin.
    Sphere { radius: f64 },
    /// `IfcToroidalSurface(MajorRadius, MinorRadius)`: the tube centre
    /// circle of `major` radius lies in the local xy-plane.
    Torus { major: f64, minor: f64 },
}

impl ElementarySurface {
    /// Resolve `IfcPlane` / `IfcCylindricalSurface` /
    /// `IfcSphericalSurface` / `IfcToroidalSurface` by instance id.
    /// Any other keyword is `Unsupported`.
    pub(super) fn from_id(step: &StepFile, id: u64) -> Result<Self, GeometryError> {
        let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
        let positive = |i: usize| -> Result<f64, GeometryError> {
            let v = match inst.args.get(i) {
                Some(Value::Typed { args, .. }) => args.first().and_then(Value::as_number),
                Some(v) => v.as_number(),
                None => None,
            }
            .ok_or(GeometryError::BadProfile)?;
            if v > 0.0 && v.is_finite() {
                Ok(v)
            } else {
                Err(GeometryError::BadProfile)
            }
        };
        let kind = match inst.keyword.as_str() {
            "IFCPLANE" => SurfaceKind::Plane,
            "IFCCYLINDRICALSURFACE" => SurfaceKind::Cylinder {
                radius: positive(1)?,
            },
            "IFCSPHERICALSURFACE" => SurfaceKind::Sphere {
                radius: positive(1)?,
            },
            "IFCTOROIDALSURFACE" => {
                let (major, minor) = (positive(1)?, positive(2)?);
                // WHERE MajorLargerMinor: MinorRadius < MajorRadius.
                if minor >= major {
                    return Err(GeometryError::BadProfile);
                }
                SurfaceKind::Torus { major, minor }
            }
            other => return Err(GeometryError::Unsupported(other.to_string())),
        };
        // Position : IfcAxis2Placement3D (index 0).
        let pos_id = inst
            .args
            .first()
            .and_then(Value::as_reference)
            .ok_or(GeometryError::BadCoordinates)?;
        let frame = axis2_placement_3d(step, pos_id)?;
        Ok(Self { frame, kind })
    }

    /// Map a world point into the placement's local frame.
    pub(super) fn to_local(&self, p: [f64; 3]) -> [f64; 3] {
        let d = [
            p[0] - self.frame.translation[0],
            p[1] - self.frame.translation[1],
            p[2] - self.frame.translation[2],
        ];
        [
            dot_raw(d, self.frame.cols[0]),
            dot_raw(d, self.frame.cols[1]),
            dot_raw(d, self.frame.cols[2]),
        ]
    }

    /// Map a local direction into world space (rotation only).
    pub(super) fn dir_to_world(&self, v: [f64; 3]) -> [f64; 3] {
        let c = &self.frame.cols;
        [
            c[0][0] * v[0] + c[1][0] * v[1] + c[2][0] * v[2],
            c[0][1] * v[0] + c[1][1] * v[1] + c[2][1] * v[2],
            c[0][2] * v[0] + c[1][2] * v[1] + c[2][2] * v[2],
        ]
    }

    /// The unit surface normal (in world space) at — or nearest to —
    /// the world point `p`. `None` when the point sits on a degenerate
    /// locus (a cylinder axis, a sphere centre, the torus axis / tube
    /// centre circle) where the normal is undefined.
    pub(super) fn normal_at(&self, p: [f64; 3]) -> Option<[f64; 3]> {
        let l = self.to_local(p);
        let local = match self.kind {
            SurfaceKind::Plane => [0.0, 0.0, 1.0],
            SurfaceKind::Cylinder { .. } => normalise([l[0], l[1], 0.0])?,
            SurfaceKind::Sphere { .. } => normalise(l)?,
            SurfaceKind::Torus { major, .. } => {
                let radial = normalise([l[0], l[1], 0.0])?;
                let centre = [radial[0] * major, radial[1] * major, 0.0];
                normalise([l[0] - centre[0], l[1] - centre[1], l[2] - centre[2]])?
            }
        };
        normalise(self.dir_to_world(local))
    }
}
