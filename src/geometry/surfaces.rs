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

use super::{
    axis1_placement, axis2_placement_3d, cross_raw, curve_points_2d, dot_raw, normalise,
    profile_ring, GeometryError, Transform,
};
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

/// A surface with an explicit `(u, v)` parameterisation and its inverse —
/// what the parameter-space face trimmer ([`super::trim`]) works on.
///
/// Parameterisations (local frame of the `Position` placement, `X`,
/// `Y`, `Z` its axes, `O` its origin):
///
/// * cylinder: `S(u, v) = O + R(cos u X + sin u Y) + v Z` — `u` periodic;
/// * sphere: `S(u, v) = O + R(cos v cos u X + cos v sin u Y + sin v Z)` —
///   `u` periodic, `v ∈ [−π/2, π/2]` with the poles degenerate in `u`;
/// * torus: `S(u, v) = O + (R + r cos v)(cos u X + sin u Y) + r sin v Z` —
///   both periodic;
/// * B-spline surface: de Boor over the knot domains; a `UClosed` /
///   `VClosed` surface is treated as periodic over its domain length.
///
/// These are the natural parameterisations implied by the placement
/// (the angular parameter starts on the local +x axis and increases
/// towards +y); only the trimmer's internal coordinates depend on them
/// — every face boundary is given as 3-D edge curves and inverted, so
/// the choice never changes the tessellated geometry.
#[derive(Debug, Clone)]
pub(super) enum ParamSurface {
    Elementary(ElementarySurface),
    BSpline {
        surface: super::bspline::BSplineSurface,
        /// A coarse sample grid `(u, v, point)` for the inverse's
        /// initial guess.
        samples: Vec<(f64, f64, [f64; 3])>,
        /// Control-net extent (for scale-relative tolerances).
        size: f64,
    },
    /// `IfcSurfaceOfRevolution(SweptCurve, Position, AxisPosition)`: the
    /// sampled 2-D profile (in the `Position` xy-plane) revolved about
    /// the axis line — `u` the revolution angle (periodic), `v` the
    /// fractional profile-sample index. The profile point `P(v)` is
    /// decomposed against the axis into an axial offset and a signed
    /// distance along `e1 = z × axis` (which lies in the xy-plane); the
    /// revolution carries `e1` to `cos u · e1 + sin u · e2` with
    /// `e2 = axis × e1 = z`.
    Revolution {
        frame: Transform,
        profile: Vec<[f64; 2]>,
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        e1: [f64; 3],
        e2: [f64; 3],
    },
    /// `IfcSurfaceOfLinearExtrusion(SweptCurve, Position,
    /// ExtrudedDirection, Depth)`: the sampled 2-D profile (in the
    /// `Position` xy-plane) translated along the extrusion direction —
    /// `u` the fractional profile-sample index (periodic over the
    /// sample count for a closed profile), `v` the distance along the
    /// unit direction (`Depth` only bounds the authored surface; the
    /// face loops bound the region).
    Extrusion {
        frame: Transform,
        profile: Vec<[f64; 2]>,
        closed: bool,
        dir: [f64; 3],
    },
}

/// A parameter-space point.
pub(super) type Uv = [f64; 2];

impl ParamSurface {
    /// Resolve a surface instance usable as an `IfcAdvancedFace.FaceSurface`.
    pub(super) fn from_id(step: &StepFile, id: u64) -> Result<Self, GeometryError> {
        let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
        match inst.keyword.as_str() {
            "IFCCYLINDRICALSURFACE" | "IFCSPHERICALSURFACE" | "IFCTOROIDALSURFACE" => {
                Ok(Self::Elementary(ElementarySurface::from_id(step, id)?))
            }
            "IFCSURFACEOFREVOLUTION" | "IFCSURFACEOFLINEAREXTRUSION" => {
                // IfcSweptSurface(SweptCurve, Position) + own attributes.
                let profile_id = inst
                    .args
                    .first()
                    .and_then(Value::as_reference)
                    .ok_or(GeometryError::BadProfile)?;
                let (profile, closed) = profile_curve(step, profile_id)?;
                let frame = match inst.args.get(1).and_then(Value::as_reference) {
                    Some(pid) => axis2_placement_3d(step, pid)?,
                    None => Transform::IDENTITY,
                };
                if inst.keyword == "IFCSURFACEOFREVOLUTION" {
                    // AxisPosition : IfcAxis1Placement (index 2), in the
                    // Position frame.
                    let axis_id = inst
                        .args
                        .get(2)
                        .and_then(Value::as_reference)
                        .ok_or(GeometryError::BadCoordinates)?;
                    let (axis_origin, axis_dir) = axis1_placement(step, axis_id)?;
                    let z = [0.0, 0.0, 1.0];
                    // e1 ⟂ axis in the profile plane; a vertical axis
                    // (the profile plane is then a plane of revolution
                    // cross-sections, not a meridian) has no profile
                    // side.
                    let e1 = normalise(cross_raw(z, axis_dir)).ok_or(GeometryError::BadProfile)?;
                    let e2 = cross_raw(axis_dir, e1);
                    Ok(Self::Revolution {
                        frame,
                        profile,
                        axis_origin,
                        axis_dir,
                        e1,
                        e2,
                    })
                } else {
                    // ExtrudedDirection (2), Depth (3).
                    let dir = super::direction(step, inst.args.get(2))?
                        .and_then(normalise)
                        .ok_or(GeometryError::BadCoordinates)?;
                    let depth = match inst.args.get(3) {
                        Some(Value::Typed { args, .. }) => args.first().and_then(Value::as_number),
                        Some(v) => v.as_number(),
                        None => None,
                    }
                    .ok_or(GeometryError::BadCoordinate)?;
                    // A direction in the profile plane sweeps no surface
                    // graph over the plane; the inverse needs dir.z ≠ 0.
                    if depth <= 0.0 || dir[2].abs() < 1e-9 {
                        return Err(GeometryError::BadProfile);
                    }
                    Ok(Self::Extrusion {
                        frame,
                        profile,
                        closed,
                        dir,
                    })
                }
            }
            "IFCBSPLINESURFACEWITHKNOTS" | "IFCRATIONALBSPLINESURFACEWITHKNOTS" => {
                let surface =
                    super::bspline::BSplineSurface::from_instance(step, &inst.keyword, &inst.args)?;
                let us = surface.u_samples(8);
                let vs = surface.v_samples(8);
                let mut samples = Vec::with_capacity(us.len() * vs.len());
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for &u in &us {
                    for &v in &vs {
                        let p = surface.point_at(u, v);
                        for k in 0..3 {
                            lo[k] = lo[k].min(p[k]);
                            hi[k] = hi[k].max(p[k]);
                        }
                        samples.push((u, v, p));
                    }
                }
                let size =
                    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2))
                        .sqrt()
                        .max(f64::MIN_POSITIVE);
                Ok(Self::BSpline {
                    surface,
                    samples,
                    size,
                })
            }
            other => Err(GeometryError::Unsupported(other.to_string())),
        }
    }

    /// The surface point at `(u, v)` in world space.
    pub(super) fn eval(&self, uv: Uv) -> [f64; 3] {
        let (u, v) = (uv[0], uv[1]);
        match self {
            Self::Elementary(e) => {
                let local = match e.kind {
                    SurfaceKind::Plane => [u, v, 0.0],
                    SurfaceKind::Cylinder { radius } => [radius * u.cos(), radius * u.sin(), v],
                    SurfaceKind::Sphere { radius } => [
                        radius * v.cos() * u.cos(),
                        radius * v.cos() * u.sin(),
                        radius * v.sin(),
                    ],
                    SurfaceKind::Torus { major, minor } => {
                        let rho = major + minor * v.cos();
                        [rho * u.cos(), rho * u.sin(), minor * v.sin()]
                    }
                };
                let d = e.dir_to_world(local);
                let t = e.frame.translation;
                [d[0] + t[0], d[1] + t[1], d[2] + t[2]]
            }
            Self::BSpline { surface, .. } => surface.point_at(u, v),
            Self::Revolution {
                frame,
                profile,
                axis_origin,
                axis_dir,
                e1,
                e2,
            } => {
                let p = polyline_point(profile, false, v);
                let q = [
                    p[0] - axis_origin[0],
                    p[1] - axis_origin[1],
                    -axis_origin[2],
                ];
                let a = dot_raw(q, *axis_dir);
                let r = dot_raw(q, *e1);
                let (c, s) = (u.cos(), u.sin());
                let local = [
                    axis_origin[0] + a * axis_dir[0] + r * (c * e1[0] + s * e2[0]),
                    axis_origin[1] + a * axis_dir[1] + r * (c * e1[1] + s * e2[1]),
                    axis_origin[2] + a * axis_dir[2] + r * (c * e1[2] + s * e2[2]),
                ];
                frame.apply(local)
            }
            Self::Extrusion {
                frame,
                profile,
                closed,
                dir,
            } => {
                let p = polyline_point(profile, *closed, u);
                frame.apply([p[0] + v * dir[0], p[1] + v * dir[1], v * dir[2]])
            }
        }
    }

    /// The parameters of (the surface point nearest to) `p`. The flag
    /// is `true` when `u` is undefined there (a sphere pole): the
    /// returned `u` is then arbitrary and the caller fills it from the
    /// loop neighbours.
    pub(super) fn inverse(&self, p: [f64; 3]) -> (Uv, bool) {
        match self {
            Self::Elementary(e) => {
                let l = e.to_local(p);
                match e.kind {
                    SurfaceKind::Plane => ([l[0], l[1]], false),
                    SurfaceKind::Cylinder { .. } => {
                        let rho = l[0].hypot(l[1]);
                        ([l[1].atan2(l[0]), l[2]], rho <= 0.0)
                    }
                    SurfaceKind::Sphere { radius } => {
                        let rho = l[0].hypot(l[1]);
                        let v = (l[2] / radius).clamp(-1.0, 1.0).asin();
                        let degenerate = rho <= 1e-9 * radius;
                        ([l[1].atan2(l[0]), v], degenerate)
                    }
                    SurfaceKind::Torus { major, .. } => {
                        let rho = l[0].hypot(l[1]);
                        ([l[1].atan2(l[0]), l[2].atan2(rho - major)], rho <= 0.0)
                    }
                }
            }
            Self::BSpline {
                surface,
                samples,
                size,
            } => {
                // Nearest coarse sample, then Gauss–Newton on the
                // squared distance.
                let mut best = (f64::INFINITY, 0.0, 0.0);
                for &(u, v, q) in samples {
                    let d = dist2(p, q);
                    if d < best.0 {
                        best = (d, u, v);
                    }
                }
                let (u0, u1) = surface.u_domain();
                let (v0, v1) = surface.v_domain();
                let (mut u, mut v) = (best.1, best.2);
                for _ in 0..24 {
                    let s = surface.point_at(u, v);
                    let r = [p[0] - s[0], p[1] - s[1], p[2] - s[2]];
                    if dot_raw(r, r) <= (1e-12 * size).powi(2) {
                        break;
                    }
                    let (su, sv) = surface.partials(u, v);
                    let (a, b, c) = (dot_raw(su, su), dot_raw(su, sv), dot_raw(sv, sv));
                    let (e, f) = (dot_raw(su, r), dot_raw(sv, r));
                    let det = a * c - b * b;
                    if det.abs() <= f64::MIN_POSITIVE {
                        break;
                    }
                    let du = (e * c - b * f) / det;
                    let dv = (a * f - b * e) / det;
                    let (nu, nv) = ((u + du).clamp(u0, u1), (v + dv).clamp(v0, v1));
                    let moved = (nu - u).abs() + (nv - v).abs();
                    u = nu;
                    v = nv;
                    if moved <= 1e-14 * ((u1 - u0).abs() + (v1 - v0).abs()) {
                        break;
                    }
                }
                ([u, v], false)
            }
            Self::Revolution {
                frame,
                profile,
                axis_origin,
                axis_dir,
                e1,
                e2,
            } => {
                let l = frame.invert_point(p);
                let q = [
                    l[0] - axis_origin[0],
                    l[1] - axis_origin[1],
                    l[2] - axis_origin[2],
                ];
                let a = dot_raw(q, *axis_dir);
                let (p1, p2) = (dot_raw(q, *e1), dot_raw(q, *e2));
                let rho = p1.hypot(p2);
                let scale = profile
                    .iter()
                    .map(|p| p[0].abs().max(p[1].abs()))
                    .fold(1.0, f64::max);
                if rho <= 1e-9 * scale {
                    // On the axis: u is undefined; v from the axial offset.
                    let base = [
                        axis_origin[0] + a * axis_dir[0],
                        axis_origin[1] + a * axis_dir[1],
                    ];
                    let (v, _) = nearest_on_polyline(profile, false, base);
                    return ([0.0, v], true);
                }
                let u = p2.atan2(p1);
                // The profile may lie on either side of the axis: try the
                // unrotated point at +rho (angle u) and at −rho (angle
                // u + π), keep the closer one.
                let candidate = |sign: f64| -> (f64, f64) {
                    let r = sign * rho;
                    let pt = [
                        axis_origin[0] + a * axis_dir[0] + r * e1[0],
                        axis_origin[1] + a * axis_dir[1] + r * e1[1],
                    ];
                    nearest_on_polyline(profile, false, pt)
                };
                let (vp, dp) = candidate(1.0);
                let (vn, dn) = candidate(-1.0);
                if dp <= dn {
                    ([u, vp], false)
                } else {
                    ([u + core::f64::consts::PI, vn], false)
                }
            }
            Self::Extrusion {
                frame,
                profile,
                closed,
                dir,
            } => {
                let l = frame.invert_point(p);
                let s = l[2] / dir[2];
                let c = [l[0] - s * dir[0], l[1] - s * dir[1]];
                let (u, _) = nearest_on_polyline(profile, *closed, c);
                ([u, s], false)
            }
        }
    }

    /// Where to split the parameter edge `a → b` (a fraction in
    /// `(0, 1)`): the midpoint, unless the direction parameterised by
    /// profile-sample index contains a sample between the ends — then
    /// the sample nearest the middle, so refined edges follow the
    /// sampled profile instead of cutting its corners.
    pub(super) fn split_fraction(&self, a: Uv, b: Uv) -> f64 {
        let index_axis = match self {
            Self::Revolution { .. } => Some(1),
            Self::Extrusion { .. } => Some(0),
            _ => None,
        };
        if let Some(k) = index_axis {
            let (lo, hi) = (a[k].min(b[k]), a[k].max(b[k]));
            let mid = 0.5 * (lo + hi);
            let candidates = [mid.floor(), mid.ceil()];
            let mut best: Option<f64> = None;
            for c in candidates {
                let inside = c > lo + 1e-9 && c < hi - 1e-9;
                let closer = match best {
                    Some(b) => (c - mid).abs() < (b - mid).abs(),
                    None => true,
                };
                if inside && closer {
                    best = Some(c);
                }
            }
            if let Some(c) = best {
                return ((c - a[k]) / (b[k] - a[k])).clamp(1e-6, 1.0 - 1e-6);
            }
        }
        0.5
    }

    /// The `u` period (the parameter wraps), if any.
    pub(super) fn period_u(&self) -> Option<f64> {
        match self {
            Self::Elementary(e) => match e.kind {
                SurfaceKind::Plane => None,
                _ => Some(2.0 * core::f64::consts::PI),
            },
            Self::Revolution { .. } => Some(2.0 * core::f64::consts::PI),
            Self::Extrusion {
                profile, closed, ..
            } => {
                if *closed {
                    Some(profile.len() as f64)
                } else {
                    None
                }
            }
            Self::BSpline { surface, .. } => {
                if surface.u_closed == Some(true) {
                    let (a, b) = surface.u_domain();
                    Some(b - a)
                } else {
                    None
                }
            }
        }
    }

    /// The `v` period, if any.
    pub(super) fn period_v(&self) -> Option<f64> {
        match self {
            Self::Elementary(e) => match e.kind {
                SurfaceKind::Torus { .. } => Some(2.0 * core::f64::consts::PI),
                _ => None,
            },
            Self::Revolution { .. } | Self::Extrusion { .. } => None,
            Self::BSpline { surface, .. } => {
                if surface.v_closed == Some(true) {
                    let (a, b) = surface.v_domain();
                    Some(b - a)
                } else {
                    None
                }
            }
        }
    }

    /// The fixed `u` extent of the parameter domain, if the surface has
    /// one (the periodic range, or a B-spline knot domain); `None` lets
    /// the loops decide.
    pub(super) fn u_extent(&self) -> Option<(f64, f64)> {
        match self {
            Self::Elementary(_) | Self::Revolution { .. } => self.period_u().map(|p| (0.0, p)),
            Self::BSpline { surface, .. } => Some(surface.u_domain()),
            Self::Extrusion {
                profile, closed, ..
            } => Some((
                0.0,
                if *closed {
                    profile.len() as f64
                } else {
                    (profile.len() - 1) as f64
                },
            )),
        }
    }

    /// The fixed `v` extent (sphere latitudes, the torus period, a
    /// B-spline domain).
    pub(super) fn v_extent(&self) -> Option<(f64, f64)> {
        match self {
            Self::Elementary(e) => match e.kind {
                SurfaceKind::Sphere { .. } => {
                    Some((-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2))
                }
                SurfaceKind::Torus { .. } => Some((0.0, 2.0 * core::f64::consts::PI)),
                _ => None,
            },
            Self::BSpline { surface, .. } => Some(surface.v_domain()),
            Self::Revolution { profile, .. } => Some((0.0, (profile.len() - 1) as f64)),
            Self::Extrusion { .. } => None,
        }
    }

    /// The largest parameter span a mesh edge may cover in `u` / `v`
    /// before it is subdivided (`None` = the surface is straight in that
    /// direction, never subdivide): the circle density for angular
    /// parameters, a fixed fraction of the domain for B-spline patches.
    pub(super) fn step(&self) -> (Option<f64>, Option<f64>) {
        let angular = 2.0 * core::f64::consts::PI / (super::CIRCLE_SEGMENTS as f64);
        match self {
            Self::Elementary(e) => match e.kind {
                SurfaceKind::Plane => (None, None),
                SurfaceKind::Cylinder { .. } => (Some(angular), None),
                SurfaceKind::Sphere { .. } | SurfaceKind::Torus { .. } => {
                    (Some(angular), Some(angular))
                }
            },
            Self::BSpline { surface, .. } => {
                let (u0, u1) = surface.u_domain();
                let (v0, v1) = surface.v_domain();
                (Some((u1 - u0) / 24.0), Some((v1 - v0) / 24.0))
            }
            // One profile sample per edge at most (the split lands on
            // the samples); straight along the sweep.
            Self::Revolution { .. } => (Some(angular), Some(1.0)),
            Self::Extrusion { .. } => (Some(1.0), None),
        }
    }

    /// Scale factors turning parameter differences into (approximate)
    /// world lengths, so shape decisions in parameter space are not
    /// skewed by an anisotropic parameterisation.
    pub(super) fn metric(&self) -> (f64, f64) {
        match self {
            Self::Elementary(e) => match e.kind {
                SurfaceKind::Plane => (1.0, 1.0),
                SurfaceKind::Cylinder { radius } => (radius, 1.0),
                SurfaceKind::Sphere { radius } => (radius, radius),
                SurfaceKind::Torus { major, minor } => (major + minor, minor),
            },
            Self::BSpline { surface, size, .. } => {
                let (u0, u1) = surface.u_domain();
                let (v0, v1) = surface.v_domain();
                (size / (u1 - u0), size / (v1 - v0))
            }
            Self::Revolution {
                profile,
                axis_origin,
                e1,
                ..
            } => {
                let radius = profile
                    .iter()
                    .map(|p| {
                        ((p[0] - axis_origin[0]) * e1[0] + (p[1] - axis_origin[1]) * e1[1]).abs()
                    })
                    .fold(1.0, f64::max);
                (radius, polyline_mean_segment(profile))
            }
            Self::Extrusion { profile, .. } => (polyline_mean_segment(profile), 1.0),
        }
    }

    /// A welding key for a parameter point: two points with equal keys
    /// are the same surface point (periodic images, the sphere poles).
    pub(super) fn weld_key(&self, uv: Uv) -> (i64, i64) {
        let q = |x: f64, period: Option<f64>| -> i64 {
            let x = match period {
                Some(p) => {
                    let r = x.rem_euclid(p);
                    // A value within tolerance of the period wraps to 0.
                    if (p - r).abs() < 1e-9 * p {
                        0.0
                    } else {
                        r
                    }
                }
                None => x,
            };
            (x * 1e9).round() as i64
        };
        let pole = matches!(
            self,
            Self::Elementary(ElementarySurface {
                kind: SurfaceKind::Sphere { .. },
                ..
            })
        ) && (uv[1].abs() - core::f64::consts::FRAC_PI_2).abs() < 1e-9;
        if pole {
            return (0, if uv[1] > 0.0 { i64::MAX } else { i64::MIN });
        }
        if let Self::Revolution {
            profile,
            axis_origin,
            e1,
            ..
        } = self
        {
            // A profile point on the axis is one surface point for
            // every u.
            let p = polyline_point(profile, false, uv[1]);
            let r = (p[0] - axis_origin[0]) * e1[0] + (p[1] - axis_origin[1]) * e1[1];
            let scale = profile
                .iter()
                .map(|p| p[0].abs().max(p[1].abs()))
                .fold(1.0, f64::max);
            if r.abs() <= 1e-9 * scale {
                return (i64::MAX, q(uv[1], None));
            }
        }
        (q(uv[0], self.period_u()), q(uv[1], self.period_v()))
    }
}

/// The `SweptCurve` profile of a swept surface (`ProfileType` CURVE):
/// an `IfcArbitraryOpenProfileDef(…, Curve)` samples its curve (open);
/// any closed profile kind resolves through the shared ring path
/// (closed). Returns the 2-D samples and whether they close.
fn profile_curve(step: &StepFile, profile_id: u64) -> Result<(Vec<[f64; 2]>, bool), GeometryError> {
    let inst = step
        .get(profile_id)
        .ok_or(GeometryError::MissingInstance(profile_id))?;
    let (pts, closed) = match inst.keyword.as_str() {
        "IFCARBITRARYOPENPROFILEDEF" => {
            let curve_id = inst
                .args
                .get(2)
                .and_then(Value::as_reference)
                .ok_or(GeometryError::BadProfile)?;
            (curve_points_2d(step, curve_id)?, false)
        }
        _ => (profile_ring(step, profile_id)?, true),
    };
    if pts.len() < 2 || (closed && pts.len() < 3) {
        return Err(GeometryError::BadProfile);
    }
    Ok((pts, closed))
}

/// The point at fractional sample index `t` of a polyline (linear
/// between samples; a closed polyline wraps).
fn polyline_point(pts: &[[f64; 2]], closed: bool, t: f64) -> [f64; 2] {
    let n = pts.len();
    let t = if closed {
        t.rem_euclid(n as f64)
    } else {
        t.clamp(0.0, (n - 1) as f64)
    };
    let i = (t.floor() as usize).min(n - 1);
    let f = t - i as f64;
    let j = if closed {
        (i + 1) % n
    } else {
        (i + 1).min(n - 1)
    };
    let (a, b) = (pts[i], pts[j]);
    [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]
}

/// The fractional sample index of the polyline point nearest to `p`,
/// and that distance.
fn nearest_on_polyline(pts: &[[f64; 2]], closed: bool, p: [f64; 2]) -> (f64, f64) {
    let n = pts.len();
    let segments = if closed { n } else { n - 1 };
    let mut best = (0.0, f64::INFINITY);
    for i in 0..segments {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let len2 = d[0] * d[0] + d[1] * d[1];
        let f = if len2 > 0.0 {
            (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = [a[0] + d[0] * f, a[1] + d[1] * f];
        let dist = (p[0] - q[0]).hypot(p[1] - q[1]);
        if dist < best.1 {
            best = (i as f64 + f, dist);
        }
    }
    best
}

/// Mean segment length of a polyline (a metric scale for its index
/// parameter).
fn polyline_mean_segment(pts: &[[f64; 2]]) -> f64 {
    let total: f64 = pts
        .windows(2)
        .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
        .sum();
    (total / (pts.len().saturating_sub(1).max(1) as f64)).max(f64::MIN_POSITIVE)
}

fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
}
