//! Neutral ISO 10303-42 geometry kernel.
//!
//! The IFC geometry resource is a profile of ISO 10303-42 (geometric
//! and topological representation): the curve / surface / topology
//! evaluators behind [`super::tessellate_item`] are not IFC-specific,
//! only their entity resolution is. This module exposes those
//! evaluators over plain data so another ISO 10303 application-protocol
//! reader (the STEP AP203 / AP214 / AP242 reader `oxideav-step`) can
//! resolve its own entity layouts and share one tessellation engine:
//!
//! * [`BSplineCurve`] / [`BSplineSurface`] — (rational) B-splines from
//!   control points + distinct knots + multiplicities, evaluated by de
//!   Boor's recurrence in homogeneous coordinates.
//! * [`Surface`] — a face surface with an explicit `(u, v)`
//!   parameterisation and inverse: plane, cylinder, cone, sphere, torus,
//!   B-spline, surface of revolution / linear extrusion of a sampled
//!   curve, offset of any of these. The parameterisations follow ISO
//!   10303-42 (`conical_surface`: `C + (R + v·tan α)(cos u·x + sin u·y)
//!   + v·z`, …).
//! * [`FaceMesher`] — the shared vertex pool + triangle list a shell is
//!   meshed into: planar faces (hole-aware ear clipping), trimmed curved
//!   faces (boundary loops inverted into parameter space, clipped to the
//!   fundamental domain, triangulated and refined — see the `trim`
//!   module), and faces bounded by parameter-space loops. Faces that
//!   share boundary vertices (the caller samples each edge once and
//!   reuses the run from both sides) come out watertight; the final
//!   [`FaceMesher::finish`] splits the T-junctions refinement left on
//!   shared boundary chords.
//!
//! Nothing here reads a [`StepFile`](crate::StepFile); every input is a
//! number, a [`Transform`] frame or a vertex id from the mesher.

use super::bspline;
use super::surfaces::{ElementarySurface, ParamSurface, SurfaceKind};
use super::{
    cross_raw, dot_raw, normalise, triangulate_face_3d, triangulate_profile, trim, GeometryError,
    ProfileArea, Transform, TriMesh, VertexPool,
};

/// Default angular density: the circle segment count the IFC
/// tessellator uses.
const DEFAULT_SEGMENTS: f64 = super::CIRCLE_SEGMENTS as f64;

/// A (rational) B-spline curve.
#[derive(Debug, Clone)]
pub struct BSplineCurve(bspline::BSplineCurve);

impl BSplineCurve {
    /// Build from `degree`, the control points, optional positive
    /// weights (one per control point — a rational curve), and the
    /// distinct knot values with their multiplicities (ISO 10303-42
    /// `b_spline_curve_with_knots`: multiplicities sum to
    /// `degree + control points + 1`, knots strictly increasing; equal
    /// adjacent knots are merged leniently).
    pub fn new(
        degree: usize,
        control: &[[f64; 3]],
        weights: Option<&[f64]>,
        knots: &[f64],
        multiplicities: &[usize],
    ) -> Result<Self, GeometryError> {
        bspline::BSplineCurve::from_data(degree, control, weights, knots, multiplicities).map(Self)
    }

    /// A piecewise Bézier curve (ISO 10303-42 `bezier_curve`): the
    /// control count minus one must be a multiple of `degree`.
    pub fn bezier(
        degree: usize,
        control: &[[f64; 3]],
        weights: Option<&[f64]>,
    ) -> Result<Self, GeometryError> {
        bspline::BSplineCurve::bezier_data(degree, control, weights).map(Self)
    }

    /// The curve degree.
    pub fn degree(&self) -> usize {
        self.0.degree()
    }

    /// The parameter domain `[t0, t1]`.
    pub fn domain(&self) -> (f64, f64) {
        self.0.domain()
    }

    /// The point at parameter `t` (clamped to the domain).
    pub fn point_at(&self, t: f64) -> [f64; 3] {
        self.0.point_at(t)
    }

    /// The distinct knot values in the domain (span boundaries, domain
    /// ends included) — where the curve may lose smoothness.
    pub fn breaks(&self) -> Vec<f64> {
        self.0.breaks()
    }
}

/// A (rational) B-spline surface.
#[derive(Debug, Clone)]
pub struct BSplineSurface(bspline::BSplineSurface);

impl BSplineSurface {
    /// Build from the `(u, v)` degrees, the control net (`control[i][j]`
    /// with `i` along `u`), optional weights shaped like the net, the
    /// distinct knots + multiplicities per direction, and the authored
    /// `u_closed` / `v_closed` flags (a closed direction is treated as
    /// periodic over its domain).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        u_degree: usize,
        v_degree: usize,
        control: &[Vec<[f64; 3]>],
        weights: Option<&[Vec<f64>]>,
        u_knots: (&[f64], &[usize]),
        v_knots: (&[f64], &[usize]),
        u_closed: Option<bool>,
        v_closed: Option<bool>,
    ) -> Result<Self, GeometryError> {
        bspline::BSplineSurface::from_data(
            u_degree, v_degree, control, weights, u_knots, v_knots, u_closed, v_closed,
        )
        .map(Self)
    }

    /// The `u` parameter domain.
    pub fn u_domain(&self) -> (f64, f64) {
        self.0.u_domain()
    }

    /// The `v` parameter domain.
    pub fn v_domain(&self) -> (f64, f64) {
        self.0.v_domain()
    }

    /// The surface point at `(u, v)` (clamped to the domain).
    pub fn point_at(&self, u: f64, v: f64) -> [f64; 3] {
        self.0.point_at(u, v)
    }

    /// The `(u, v)` degrees.
    pub fn degrees(&self) -> (usize, usize) {
        self.0.degrees()
    }

    /// The distinct `u` knots in the domain (span boundaries).
    pub fn u_breaks(&self) -> Vec<f64> {
        self.0.u_breaks()
    }

    /// The distinct `v` knots in the domain (span boundaries).
    pub fn v_breaks(&self) -> Vec<f64> {
        self.0.v_breaks()
    }
}

/// A face surface with an explicit `(u, v)` parameterisation, its
/// inverse, and the mesh density the trimmed-face tessellator refines
/// it to.
#[derive(Debug, Clone)]
pub struct Surface {
    inner: ParamSurface,
    steps: (Option<f64>, Option<f64>),
}

fn positive(x: f64) -> Result<f64, GeometryError> {
    if x > 0.0 && x.is_finite() {
        Ok(x)
    } else {
        Err(GeometryError::BadProfile)
    }
}

fn finite3(p: [f64; 3]) -> Result<[f64; 3], GeometryError> {
    if p.iter().all(|c| c.is_finite()) {
        Ok(p)
    } else {
        Err(GeometryError::BadCoordinate)
    }
}

/// The parameter step that keeps the chord of a circle of `radius`
/// within `tolerance` of the arc, capped at `max_angle`.
fn angular_step(radius: f64, tolerance: f64, max_angle: f64) -> f64 {
    let r = radius.abs();
    if r <= tolerance || r <= 0.0 {
        return max_angle;
    }
    let a = 2.0 * (1.0 - tolerance / r).clamp(-1.0, 1.0).acos();
    // Never finer than 1024 segments per turn.
    a.clamp(2.0 * core::f64::consts::PI / 1024.0, max_angle)
}

impl Surface {
    fn wrap(inner: ParamSurface) -> Self {
        let steps = inner.step();
        Self { inner, steps }
    }

    fn elementary(frame: Transform, kind: SurfaceKind) -> Self {
        Self::wrap(ParamSurface::Elementary(ElementarySurface { frame, kind }))
    }

    /// A plane: the `frame`'s local xy-plane, `S(u, v) = O + u·x + v·y`.
    pub fn plane(frame: Transform) -> Self {
        Self::elementary(frame, SurfaceKind::Plane)
    }

    /// A cylinder about the `frame`'s z axis:
    /// `S(u, v) = O + R(cos u·x + sin u·y) + v·z`.
    pub fn cylinder(frame: Transform, radius: f64) -> Result<Self, GeometryError> {
        Ok(Self::elementary(
            frame,
            SurfaceKind::Cylinder {
                radius: positive(radius)?,
            },
        ))
    }

    /// A cone about the `frame`'s z axis (ISO 10303-42
    /// `conical_surface`): `S(u, v) = O + (R + v·tan α)(cos u·x +
    /// sin u·y) + v·z`, `radius` ≥ 0 at the frame origin,
    /// `semi_angle` (radians) in `(0, π/2)`.
    pub fn cone(frame: Transform, radius: f64, semi_angle: f64) -> Result<Self, GeometryError> {
        if !(radius >= 0.0 && radius.is_finite())
            || !(semi_angle > 0.0 && semi_angle < core::f64::consts::FRAC_PI_2)
        {
            return Err(GeometryError::BadProfile);
        }
        Ok(Self::elementary(
            frame,
            SurfaceKind::Cone {
                radius,
                tan: semi_angle.tan(),
            },
        ))
    }

    /// A sphere centred on the `frame` origin:
    /// `S(u, v) = O + R(cos v cos u·x + cos v sin u·y + sin v·z)`.
    pub fn sphere(frame: Transform, radius: f64) -> Result<Self, GeometryError> {
        Ok(Self::elementary(
            frame,
            SurfaceKind::Sphere {
                radius: positive(radius)?,
            },
        ))
    }

    /// A ring torus about the `frame`'s z axis (`minor < major`):
    /// `S(u, v) = O + (R + r cos v)(cos u·x + sin u·y) + r sin v·z`.
    pub fn torus(frame: Transform, major: f64, minor: f64) -> Result<Self, GeometryError> {
        let (major, minor) = (positive(major)?, positive(minor)?);
        if minor >= major {
            return Err(GeometryError::BadProfile);
        }
        Ok(Self::elementary(frame, SurfaceKind::Torus { major, minor }))
    }

    /// A B-spline surface over its knot domain.
    pub fn bspline(surface: BSplineSurface) -> Self {
        Self::wrap(ParamSurface::from_bspline(surface.0))
    }

    /// The surface swept by revolving the sampled curve `curve` about
    /// the axis line through `axis_origin` along `axis_dir` (ISO
    /// 10303-42 `surface_of_revolution`). Only each sample's axial
    /// offset and distance from the axis matter (any curve sweeps the
    /// same surface as its meridian image), so the surface is built
    /// over the meridian polyline: `u` the revolution angle, `v` the
    /// fractional sample index.
    pub fn revolution(
        curve: &[[f64; 3]],
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
    ) -> Result<Self, GeometryError> {
        if curve.len() < 2 {
            return Err(GeometryError::BadProfile);
        }
        let o = finite3(axis_origin)?;
        let a = normalise(finite3(axis_dir)?).ok_or(GeometryError::BadCoordinates)?;
        let mut meridian: Vec<[f64; 2]> = Vec::with_capacity(curve.len());
        let mut radial: Option<[f64; 3]> = None;
        for &p in curve {
            let p = finite3(p)?;
            let q = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
            let h = dot_raw(q, a);
            let w = [q[0] - h * a[0], q[1] - h * a[1], q[2] - h * a[2]];
            let r = dot_raw(w, w).sqrt();
            if radial.is_none() && r > 0.0 {
                radial = normalise(w);
            }
            meridian.push([h, r]);
        }
        // Frame: x along the axis, y the first radial direction — the
        // meridian half-plane is the frame's xy-plane with the axis on
        // its x axis.
        let y = match radial {
            Some(y) => y,
            None => return Err(GeometryError::BadProfile),
        };
        let z = cross_raw(a, y);
        let frame = Transform {
            cols: [a, y, z],
            translation: o,
        };
        let axis = [1.0, 0.0, 0.0];
        let e1 = [0.0, 1.0, 0.0];
        let e2 = cross_raw(axis, e1);
        Ok(Self::wrap(ParamSurface::Revolution {
            frame,
            profile: meridian,
            axis_origin: [0.0, 0.0, 0.0],
            axis_dir: axis,
            e1,
            e2,
        }))
    }

    /// The surface swept by translating the sampled curve `curve` along
    /// `dir` (ISO 10303-42 `surface_of_linear_extrusion`): `u` the
    /// fractional sample index (periodic when `closed`), `v` the distance
    /// along the unit direction. The curve is projected along `dir` onto
    /// the plane through its first sample perpendicular to `dir` (the
    /// swept surface is unchanged by that projection).
    pub fn extrusion(
        curve: &[[f64; 3]],
        closed: bool,
        dir: [f64; 3],
    ) -> Result<Self, GeometryError> {
        if curve.len() < 2 {
            return Err(GeometryError::BadProfile);
        }
        let d = normalise(finite3(dir)?).ok_or(GeometryError::BadCoordinates)?;
        let seed = if d[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let k = dot_raw(seed, d);
        let x = normalise([seed[0] - k * d[0], seed[1] - k * d[1], seed[2] - k * d[2]])
            .ok_or(GeometryError::BadCoordinates)?;
        let y = cross_raw(d, x);
        let origin = finite3(curve[0])?;
        let mut profile: Vec<[f64; 2]> = Vec::with_capacity(curve.len());
        for &p in curve {
            let p = finite3(p)?;
            let q = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
            let pt = [dot_raw(q, x), dot_raw(q, y)];
            if profile.last() != Some(&pt) {
                profile.push(pt);
            }
        }
        if closed && profile.len() > 2 && profile.first() == profile.last() {
            profile.pop();
        }
        if profile.len() < 2 || (closed && profile.len() < 3) {
            return Err(GeometryError::BadProfile);
        }
        let frame = Transform {
            cols: [x, y, d],
            translation: origin,
        };
        Ok(Self::wrap(ParamSurface::Extrusion {
            frame,
            profile,
            closed,
            dir: [0.0, 0.0, 1.0],
        }))
    }

    /// `base` offset by `distance` along its unit normal
    /// `∂S/∂u × ∂S/∂v` (ISO 10303-42 `offset_surface`), sharing its
    /// parameterisation. Elementary bases are better expressed as the
    /// equivalent elementary surface; this general form is exact up to
    /// the finite-difference normal.
    pub fn offset(base: Surface, distance: f64) -> Result<Self, GeometryError> {
        if !distance.is_finite() {
            return Err(GeometryError::BadCoordinate);
        }
        let steps = base.steps;
        Ok(Self {
            inner: ParamSurface::Offset {
                base: Box::new(base.inner),
                distance,
            },
            steps,
        })
    }

    /// The surface point at `uv`.
    pub fn point_at(&self, uv: [f64; 2]) -> [f64; 3] {
        self.inner.eval(uv)
    }

    /// The parameters of (the surface point nearest to) `p`, and whether
    /// `u` is undefined there (a pole / apex).
    pub fn inverse(&self, p: [f64; 3]) -> ([f64; 2], bool) {
        self.inner.inverse(p)
    }

    /// The unit normal `∂S/∂u × ∂S/∂v` at `uv` (`None` at a degenerate
    /// point).
    pub fn normal_at(&self, uv: [f64; 2]) -> Option<[f64; 3]> {
        self.inner.unit_normal(uv)
    }

    /// The `(u, v)` periods of a periodic parameterisation.
    pub fn periods(&self) -> (Option<f64>, Option<f64>) {
        (self.inner.period_u(), self.inner.period_v())
    }

    /// True for a plane (faces on it need no interior refinement).
    pub fn is_plane(&self) -> bool {
        matches!(
            self.inner,
            ParamSurface::Elementary(ElementarySurface {
                kind: SurfaceKind::Plane,
                ..
            })
        )
    }

    /// The largest parameter span a mesh edge may cover in `u` / `v`
    /// (`None` = straight in that direction).
    pub fn param_steps(&self) -> (Option<f64>, Option<f64>) {
        self.steps
    }

    /// Override the refinement density (`None` = never subdivide along
    /// that parameter). Non-positive / non-finite steps are ignored.
    pub fn with_param_steps(mut self, u: Option<f64>, v: Option<f64>) -> Self {
        let ok = |s: Option<f64>| s.filter(|s| *s > 0.0 && s.is_finite());
        self.steps = (ok(u), ok(v));
        self
    }

    /// Derive the refinement density from a chordal tolerance (largest
    /// distance between a mesh edge and the surface, in model units)
    /// and an angular cap (largest turn of an angular parameter per mesh
    /// edge, radians). `size_hint` bounds the extent of the faces on the
    /// surface (used where the curvature varies, e.g. along a cone).
    pub fn with_chordal_tolerance(self, tolerance: f64, max_angle: f64, size_hint: f64) -> Self {
        if !(tolerance > 0.0 && tolerance.is_finite()) {
            return self;
        }
        let max_angle = if max_angle > 0.0 && max_angle.is_finite() {
            max_angle.min(core::f64::consts::FRAC_PI_2)
        } else {
            2.0 * core::f64::consts::PI / DEFAULT_SEGMENTS
        };
        let size_hint = if size_hint.is_finite() {
            size_hint.abs()
        } else {
            0.0
        };
        let steps = chordal_steps(&self.inner, tolerance, max_angle, size_hint);
        Self { steps, ..self }
    }
}

/// Tolerance-driven refinement steps for one parameterisation.
fn chordal_steps(
    s: &ParamSurface,
    tol: f64,
    max_angle: f64,
    size_hint: f64,
) -> (Option<f64>, Option<f64>) {
    let ang = |r: f64| Some(angular_step(r, tol, max_angle));
    match s {
        ParamSurface::Elementary(e) => match e.kind {
            SurfaceKind::Plane => (None, None),
            SurfaceKind::Cylinder { radius } => (ang(radius), None),
            SurfaceKind::Cone { radius, tan } => (ang(radius + size_hint * tan.abs()), None),
            SurfaceKind::Sphere { radius } => (ang(radius), ang(radius)),
            SurfaceKind::Torus { major, minor } => (ang(major + minor), ang(minor)),
        },
        ParamSurface::BSpline { surface, .. } => bspline_steps(surface, tol),
        ParamSurface::Revolution { profile, .. } => {
            let r = profile.iter().map(|p| p[1].abs()).fold(0.0, f64::max);
            (ang(r), Some(1.0))
        }
        ParamSurface::Extrusion { .. } => (Some(1.0), None),
        ParamSurface::Offset { base, distance } => {
            chordal_steps(base, tol, max_angle, size_hint + distance.abs())
        }
    }
}

/// Per-direction parameter steps for a B-spline patch: the linear-
/// interpolation error over a step `h` is about `h²/8 · |S''|`, with
/// the second derivative estimated by differences on a sample grid.
fn bspline_steps(surface: &bspline::BSplineSurface, tol: f64) -> (Option<f64>, Option<f64>) {
    let (u0, u1) = surface.u_domain();
    let (v0, v1) = surface.v_domain();
    let (pu, pv) = surface.degrees();
    let n = 16usize;
    let (du, dv) = ((u1 - u0) / n as f64, (v1 - v0) / n as f64);
    let mut max_uu: f64 = 0.0;
    let mut max_vv: f64 = 0.0;
    let second = |a: [f64; 3], b: [f64; 3], c: [f64; 3], h: f64| -> f64 {
        let d = [
            a[0] - 2.0 * b[0] + c[0],
            a[1] - 2.0 * b[1] + c[1],
            a[2] - 2.0 * b[2] + c[2],
        ];
        dot_raw(d, d).sqrt() / (h * h)
    };
    if du > 0.0 && dv > 0.0 {
        for i in 0..=n {
            for j in 0..=n {
                let (u, v) = (u0 + du * i as f64, v0 + dv * j as f64);
                let p = surface.point_at(u, v);
                if i > 0 && i < n {
                    let a = surface.point_at(u - du, v);
                    let c = surface.point_at(u + du, v);
                    max_uu = max_uu.max(second(a, p, c, du));
                }
                if j > 0 && j < n {
                    let a = surface.point_at(u, v - dv);
                    let c = surface.point_at(u, v + dv);
                    max_vv = max_vv.max(second(a, p, c, dv));
                }
            }
        }
    }
    let step = |dd: f64, lo: f64, hi: f64, degree: usize| -> Option<f64> {
        let span = hi - lo;
        if span.is_nan() || span <= 0.0 {
            return None;
        }
        if degree == 1 && dd <= 0.0 {
            // Bilinear along this direction between knots: still split at
            // a modest density so the twist is followed.
            return Some(span / 8.0);
        }
        let h = if dd > 0.0 {
            (8.0 * tol / dd).sqrt()
        } else {
            span
        };
        Some(h.clamp(span / 256.0, span / 2.0))
    };
    (step(max_uu, u0, u1, pu), step(max_vv, v0, v1, pv))
}

/// The vertex pool + triangle list a shell (or a set of faces) is
/// meshed into.
#[derive(Debug)]
pub struct FaceMesher {
    pool: VertexPool,
    triangles: Vec<[u32; 3]>,
    /// Per-triangle tag (the caller's face id, see
    /// [`FaceMesher::set_tag`]).
    tags: Vec<u32>,
    tag: u32,
}

impl Default for FaceMesher {
    fn default() -> Self {
        Self::new()
    }
}

impl FaceMesher {
    /// An empty mesher.
    pub fn new() -> Self {
        Self {
            pool: VertexPool::new(),
            triangles: Vec::new(),
            tags: Vec::new(),
            tag: 0,
        }
    }

    /// Set the tag every triangle added from now on carries (e.g. a
    /// face id for per-face styling); [`FaceMesher::finish_tagged`]
    /// returns it per output triangle, T-junction splits included.
    pub fn set_tag(&mut self, tag: u32) {
        self.tag = tag;
    }

    /// Tag the triangles appended since the last sync.
    fn sync_tags(&mut self) {
        let n = self.triangles.len();
        if self.tags.len() < n {
            self.tags.resize(n, self.tag);
        } else {
            self.tags.truncate(n);
        }
    }

    /// Append a vertex, returning its id. Boundary vertices shared by
    /// several faces must be added once and referenced by id from every
    /// face's loops — that is what makes the result watertight.
    pub fn add_vertex(&mut self, p: [f64; 3]) -> u32 {
        self.pool.push_raw(p)
    }

    /// The position of vertex `id`.
    pub fn position(&self, id: u32) -> Option<[f64; 3]> {
        self.pool.positions.get(id as usize).copied()
    }

    /// Number of vertices so far.
    pub fn vertex_count(&self) -> usize {
        self.pool.positions.len()
    }

    /// Number of triangles so far.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// The vertex positions so far.
    pub fn positions(&self) -> &[[f64; 3]] {
        &self.pool.positions
    }

    /// The triangles so far (before [`FaceMesher::finish`]'s T-junction
    /// repair).
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    fn check_ids(&self, ids: &[u32]) -> Result<(), GeometryError> {
        let n = self.pool.positions.len();
        if ids.iter().any(|&i| i as usize >= n) {
            return Err(GeometryError::IndexOutOfRange);
        }
        Ok(())
    }

    /// Append one triangle over existing vertex ids.
    pub fn add_triangle(&mut self, t: [u32; 3]) -> Result<(), GeometryError> {
        self.check_ids(&t)?;
        self.sync_tags();
        self.triangles.push(t);
        self.tags.push(self.tag);
        Ok(())
    }

    /// Triangulate a planar polygon-with-holes given by vertex ids
    /// (`outer` counter-clockwise about the face normal; holes either
    /// way). Slightly non-planar loops are projected on their Newell
    /// plane.
    pub fn add_planar_face(
        &mut self,
        outer: &[u32],
        holes: &[Vec<u32>],
    ) -> Result<(), GeometryError> {
        self.check_ids(outer)?;
        for h in holes {
            self.check_ids(h)?;
        }
        let ring = |ids: &[u32]| -> Vec<(u32, [f64; 3])> {
            ids.iter()
                .map(|&i| (i, self.pool.positions[i as usize]))
                .collect()
        };
        let outer = dedup_ring(ring(outer));
        if outer.len() < 3 {
            return Err(GeometryError::IndexOutOfRange);
        }
        let holes: Vec<Vec<(u32, [f64; 3])>> = holes
            .iter()
            .map(|h| dedup_ring(ring(h)))
            .filter(|h| h.len() >= 3)
            .collect();
        self.sync_tags();
        let r = triangulate_face_3d(&outer, &holes, &mut self.triangles);
        self.sync_tags();
        r
    }

    /// Mesh the region of `surface` bounded by `loops` (vertex ids;
    /// `loops[0]` the outer bound, each loop in its effective direction
    /// — counter-clockwise about the face's outward normal for the outer
    /// bound). `same_sense` is FALSE when the face normal opposes the
    /// surface normal `∂S/∂u × ∂S/∂v`. `surface_key` identifies the
    /// surface: faces on one surface with equal keys weld their interior
    /// grid vertices.
    pub fn add_surface_face(
        &mut self,
        surface: &Surface,
        surface_key: u64,
        loops: &[Vec<u32>],
        same_sense: bool,
    ) -> Result<(), GeometryError> {
        if loops.is_empty() {
            return Err(GeometryError::BadCoordinates);
        }
        let mut lv: Vec<Vec<trim::LoopVertex>> = Vec::with_capacity(loops.len());
        for l in loops {
            self.check_ids(l)?;
            let ring = dedup_ring(
                l.iter()
                    .map(|&i| (i, self.pool.positions[i as usize]))
                    .collect(),
            );
            lv.push(
                ring.into_iter()
                    .map(|(id, p)| trim::LoopVertex { id, p })
                    .collect(),
            );
        }
        self.sync_tags();
        let r = trim::tessellate_curved_face(
            &surface.inner,
            surface.steps,
            surface_key,
            &lv,
            same_sense,
            &mut self.pool,
            &mut self.triangles,
        );
        self.sync_tags();
        r
    }

    /// Mesh the region of `surface` bounded by loops given directly in
    /// its parameter space (outer loop counter-clockwise, holes
    /// clockwise; the normal is `∂S/∂u × ∂S/∂v`).
    pub fn add_parameter_face(
        &mut self,
        surface: &Surface,
        surface_key: u64,
        loops: &[Vec<[f64; 2]>],
    ) -> Result<(), GeometryError> {
        self.sync_tags();
        let r = trim::tessellate_parameter_face(
            &surface.inner,
            surface.steps,
            surface_key,
            loops,
            &mut self.pool,
            &mut self.triangles,
        );
        self.sync_tags();
        r
    }

    /// Reverse the winding of every triangle from index `start` on.
    pub fn reverse_from(&mut self, start: usize) {
        for t in self.triangles.iter_mut().skip(start) {
            t.swap(1, 2);
        }
    }

    /// Drop every triangle from index `start` on (rolls back a failed
    /// face that emitted partial output).
    pub fn truncate(&mut self, start: usize) {
        self.sync_tags();
        self.triangles.truncate(start);
        self.tags.truncate(start);
    }

    /// The signed volume (divergence theorem) of the triangles from
    /// index `start` on — positive for an outward-wound closed shell.
    pub fn signed_volume_from(&self, start: usize) -> f64 {
        let mut six_v = 0.0;
        for t in self.triangles.iter().skip(start) {
            let a = self.pool.positions[t[0] as usize];
            let b = self.pool.positions[t[1] as usize];
            let c = self.pool.positions[t[2] as usize];
            six_v += dot_raw(a, cross_raw(b, c));
        }
        six_v / 6.0
    }

    /// Finish: split the T-junctions trimmed-face refinement left on
    /// shared boundary chords and return the mesh.
    pub fn finish(self) -> TriMesh {
        self.finish_tagged().0
    }

    /// [`FaceMesher::finish`], also returning each output triangle's
    /// tag (see [`FaceMesher::set_tag`]).
    pub fn finish_tagged(mut self) -> (TriMesh, Vec<u32>) {
        self.sync_tags();
        let mut tagged: Vec<([u32; 3], u32)> = self
            .triangles
            .iter()
            .copied()
            .zip(self.tags.iter().copied())
            .collect();
        trim::repair_t_junctions_tagged(&mut tagged, &self.pool);
        let (triangles, tags) = tagged.into_iter().unzip();
        (
            TriMesh {
                positions: self.pool.positions,
                triangles,
            },
            tags,
        )
    }
}

/// Drop consecutive repeated ids (and a closing repeat of the first).
fn dedup_ring(mut ring: Vec<(u32, [f64; 3])>) -> Vec<(u32, [f64; 3])> {
    ring.dedup_by(|a, b| a.0 == b.0);
    while ring.len() > 1 && ring.first().map(|f| f.0) == ring.last().map(|l| l.0) {
        ring.pop();
    }
    ring
}

/// Triangulate a 2-D polygon-with-holes (outer ring counter-clockwise,
/// holes counter-clockwise too — they are reversed when merged). The
/// returned triangles index the concatenation outer ++ holes…
pub fn triangulate_polygon(
    outer: &[[f64; 2]],
    holes: &[Vec<[f64; 2]>],
) -> Result<Vec<[u32; 3]>, GeometryError> {
    if outer.len() < 3
        || outer
            .iter()
            .chain(holes.iter().flatten())
            .any(|p| !p[0].is_finite() || !p[1].is_finite())
    {
        return Err(GeometryError::BadProfile);
    }
    triangulate_profile(&ProfileArea {
        outer: outer.to_vec(),
        holes: holes.to_vec(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn closed(m: &TriMesh) -> bool {
        super::super::csg::is_closed(m)
    }

    #[test]
    fn cone_round_trips_and_is_angular() {
        let s = Surface::cone(Transform::IDENTITY, 1.0, 0.25).unwrap();
        for &(u, v) in &[(0.3, 0.5), (2.0, -1.0), (-1.2, 2.5)] {
            let p = s.point_at([u, v]);
            let (uv, deg) = s.inverse(p);
            assert!(!deg);
            let q = s.point_at(uv);
            assert!((0..3).all(|k| (p[k] - q[k]).abs() < 1e-9), "{p:?} {q:?}");
        }
        assert_eq!(s.periods().0, Some(2.0 * core::f64::consts::PI));
        // Apex (v = −R / tan α) is degenerate.
        let apex = s.point_at([0.0, -1.0 / 0.25f64.tan()]);
        assert!(s.inverse(apex).1);
    }

    #[test]
    fn revolution_of_a_line_is_a_cylinder() {
        // Line parallel to the z axis at distance 2, revolved about z.
        let curve: Vec<[f64; 3]> = vec![[2.0, 0.0, 0.0], [2.0, 0.0, 3.0]];
        let s = Surface::revolution(&curve, [0.0; 3], [0.0, 0.0, 1.0]).unwrap();
        for &(u, v) in &[(0.1, 0.0), (1.0, 0.5), (4.0, 1.0)] {
            let p = s.point_at([u, v]);
            assert!(((p[0] * p[0] + p[1] * p[1]).sqrt() - 2.0).abs() < 1e-9);
            let (uv, _) = s.inverse(p);
            let q = s.point_at(uv);
            assert!((0..3).all(|k| (p[k] - q[k]).abs() < 1e-9));
        }
    }

    #[test]
    fn extrusion_of_an_oblique_curve() {
        let curve: Vec<[f64; 3]> = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 1.0], [2.0, 0.0, 0.0]];
        let s = Surface::extrusion(&curve, false, [0.0, 1.0, 0.0]).unwrap();
        let p = s.point_at([1.0, 2.0]);
        assert!(
            (p[0] - 1.0).abs() < 1e-12 && (p[1] - 2.0).abs() < 1e-12 && (p[2] - 1.0).abs() < 1e-12
        );
        let (uv, _) = s.inverse(p);
        assert!((uv[0] - 1.0).abs() < 1e-9 && (uv[1] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn offset_plane_shifts_along_normal() {
        let s = Surface::offset(Surface::plane(Transform::IDENTITY), 0.5).unwrap();
        let p = s.point_at([1.0, 2.0]);
        assert!((p[2] - 0.5).abs() < 1e-6, "{p:?}");
        let n = s.normal_at([1.0, 2.0]).unwrap();
        assert!((n[2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn chordal_tolerance_sets_angular_density() {
        let s = Surface::cylinder(Transform::IDENTITY, 10.0)
            .unwrap()
            .with_chordal_tolerance(0.01, 1.0, 0.0);
        let (su, sv) = s.param_steps();
        assert!(sv.is_none());
        let su = su.unwrap();
        // Sagitta of the step stays within tolerance.
        assert!(10.0 * (1.0 - (su / 2.0).cos()) <= 0.01 + 1e-12);
        assert!(su > 0.05);
    }

    #[test]
    fn bspline_curve_neutral_constructor() {
        let c = BSplineCurve::new(
            2,
            &[[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            Some(&[1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
            &[0.0, 1.0],
            &[3, 3],
        )
        .unwrap();
        let p = c.point_at(0.5);
        assert!(((p[0] * p[0] + p[1] * p[1]).sqrt() - 1.0).abs() < 1e-12);
        assert_eq!(c.breaks(), [0.0, 1.0]);
        assert!(BSplineCurve::new(2, &[[0.0; 3]; 3], None, &[0.0, 1.0], &[2, 2]).is_err());
    }

    /// A closed cylinder (radius 1, height 2) assembled the way a STEP
    /// B-rep reader would: two circle edges sampled once, shared by the
    /// lateral face and the caps.
    #[test]
    fn shared_edges_make_a_watertight_cylinder() {
        let mut m = FaceMesher::new();
        let n = 32;
        let ring = |m: &mut FaceMesher, z: f64| -> Vec<u32> {
            (0..n)
                .map(|i| {
                    let a = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
                    m.add_vertex([a.cos(), a.sin(), z])
                })
                .collect()
        };
        let bottom = ring(&mut m, 0.0);
        let top = ring(&mut m, 2.0);
        // Top cap: counter-clockwise seen from +z.
        m.add_planar_face(&top, &[]).unwrap();
        // Bottom cap: clockwise seen from +z (outward −z).
        let rev: Vec<u32> = bottom.iter().rev().copied().collect();
        m.add_planar_face(&rev, &[]).unwrap();
        // Lateral face: two loops (bottom ccw, top cw seen from +z) —
        // region between them on the cylinder.
        let cyl = Surface::cylinder(Transform::IDENTITY, 1.0).unwrap();
        let top_rev: Vec<u32> = top.iter().rev().copied().collect();
        m.add_surface_face(&cyl, 1, &[bottom.clone(), top_rev], true)
            .unwrap();
        let mesh = m.finish();
        assert!(closed(&mesh), "not watertight");
        let vol = mesh.signed_volume();
        let exact = core::f64::consts::PI * 2.0;
        assert!((vol - exact).abs() / exact < 0.02, "{vol}");
    }

    /// A solid cone (apex up): the lateral face is bounded by a single
    /// circle loop (the apex is a pole of the parameterisation), closed
    /// by a planar disk; also a frustum between two circles.
    #[test]
    fn cone_to_apex_and_frustum_are_watertight() {
        let n = 48;
        let ring = |m: &mut FaceMesher, r: f64, z: f64| -> Vec<u32> {
            (0..n)
                .map(|i| {
                    let a = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
                    m.add_vertex([r * a.cos(), r * a.sin(), z])
                })
                .collect()
        };
        // Cone: radius 1 at z = 0, apex at z = 2 (semi-angle atan(1/2)),
        // axis pointing down so radius grows... use axis +z with the
        // frame at the base: radius shrinks with v, i.e. a negative
        // semi-angle is not allowed — flip the frame instead (z down,
        // origin at the apex level).
        let half = (0.5f64).atan();
        let frame = Transform {
            cols: [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            translation: [0.0, 0.0, 2.0],
        };
        let cone = Surface::cone(frame, 0.0, half).unwrap();
        let mut m = FaceMesher::new();
        let base = ring(&mut m, 1.0, 0.0);
        let rev: Vec<u32> = base.iter().rev().copied().collect();
        m.add_planar_face(&rev, &[]).unwrap();
        // Lateral loop counter-clockwise seen from outside-above: the
        // base circle traversed counter-clockwise about +z.
        m.add_surface_face(&cone, 1, std::slice::from_ref(&base), true)
            .unwrap();
        let mesh = m.finish();
        assert!(closed(&mesh), "cone not watertight");
        let exact = core::f64::consts::PI * 2.0 / 3.0;
        let vol = mesh.signed_volume();
        assert!((vol - exact).abs() / exact < 0.02, "{vol} vs {exact}");

        // Frustum: radius 2 at z = 0 to radius 1 at z = 1 (apex at z = 2).
        let frame = Transform {
            cols: [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            translation: [0.0, 0.0, 0.0],
        };
        let cone = Surface::cone(frame, 2.0, core::f64::consts::FRAC_PI_4).unwrap();
        let mut m = FaceMesher::new();
        let bottom = ring(&mut m, 2.0, 0.0);
        let top = ring(&mut m, 1.0, 1.0);
        m.add_planar_face(&top, &[]).unwrap();
        let rev: Vec<u32> = bottom.iter().rev().copied().collect();
        m.add_planar_face(&rev, &[]).unwrap();
        let top_rev: Vec<u32> = top.iter().rev().copied().collect();
        m.add_surface_face(&cone, 2, &[bottom.clone(), top_rev], true)
            .unwrap();
        let mesh = m.finish();
        assert!(closed(&mesh), "frustum not watertight");
        let exact = core::f64::consts::PI / 3.0 * (4.0 + 2.0 + 1.0);
        let vol = mesh.signed_volume();
        assert!((vol - exact).abs() / exact < 0.02, "{vol} vs {exact}");
    }

    /// A hemisphere closed by a single equator loop plus a disk.
    #[test]
    fn hemisphere_single_loop_is_watertight() {
        let n = 48;
        let mut m = FaceMesher::new();
        let base: Vec<u32> = (0..n)
            .map(|i| {
                let a = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
                m.add_vertex([a.cos(), a.sin(), 0.0])
            })
            .collect();
        let rev: Vec<u32> = base.iter().rev().copied().collect();
        m.add_planar_face(&rev, &[]).unwrap();
        let s = Surface::sphere(Transform::IDENTITY, 1.0).unwrap();
        m.add_surface_face(&s, 1, std::slice::from_ref(&base), true)
            .unwrap();
        let mesh = m.finish();
        assert!(closed(&mesh));
        let exact = 2.0 / 3.0 * core::f64::consts::PI;
        assert!((mesh.signed_volume() - exact).abs() / exact < 0.02);
    }

    #[test]
    fn tags_follow_triangles_through_repair() {
        let mut m = FaceMesher::new();
        let a = m.add_vertex([0.0, 0.0, 0.0]);
        let b = m.add_vertex([1.0, 0.0, 0.0]);
        let c = m.add_vertex([0.0, 1.0, 0.0]);
        let d = m.add_vertex([0.0, 0.0, 1.0]);
        m.set_tag(7);
        m.add_planar_face(&[a, c, b], &[]).unwrap();
        m.set_tag(9);
        m.add_triangle([a, b, d]).unwrap();
        m.add_triangle([b, c, d]).unwrap();
        m.add_triangle([c, a, d]).unwrap();
        let (mesh, tags) = m.finish_tagged();
        assert_eq!(mesh.triangles.len(), tags.len());
        assert_eq!(tags.iter().filter(|&&t| t == 7).count(), 1);
        assert_eq!(tags.iter().filter(|&&t| t == 9).count(), 3);
    }

    /// Several holes whose nearest outer vertex is the same corner: each
    /// later bridge must leave that corner through the copy whose
    /// sector opens towards its hole (else the bridged polygon
    /// self-intersects and ear clipping stalls).
    #[test]
    fn holes_bridged_to_one_corner() {
        let outer = [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]];
        let sq = |x: f64, y: f64| vec![[x, y], [x + 4.0, y], [x + 4.0, y + 4.0], [x, y + 4.0]];
        let holes = vec![
            sq(2.0, 90.0),
            sq(8.0, 94.0),
            sq(3.0, 82.0),
            sq(12.0, 86.0),
            sq(20.0, 93.0),
        ];
        let t = triangulate_polygon(&outer, &holes).unwrap();
        // n vertices + 2 per bridge, triangles = n + 2h − 2.
        assert_eq!(t.len(), 4 + 20 + 2 * 5 - 2);
        let all: Vec<[f64; 2]> = outer
            .iter()
            .copied()
            .chain(holes.iter().flatten().copied())
            .collect();
        let area: f64 = t
            .iter()
            .map(|&[a, b, c]| {
                let (a, b, c) = (all[a as usize], all[b as usize], all[c as usize]);
                0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            })
            .sum();
        assert!((area - (10_000.0 - 5.0 * 16.0)).abs() < 1e-6, "{area}");
    }

    /// A cone split into two half faces that both pass through the apex
    /// (the apex is a loop vertex on a pole of the parameterisation):
    /// the halves' u ranges span exactly half a period, the pole is
    /// crossed freely, and refinement on the pole line reuses the loop's
    /// apex vertex and borders no boundary chord — watertight, with the
    /// exact area and volume up to the chord error.
    #[test]
    fn cone_halves_through_the_apex() {
        let n = 24;
        let frame = Transform {
            cols: [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            translation: [0.0, 0.0, 4.0],
        };
        let cone = Surface::cone(frame, 0.0, (0.5f64).atan())
            .unwrap()
            .with_chordal_tolerance(0.001, 0.2, 4.0);
        let mut m = FaceMesher::new();
        let apex = m.add_vertex([0.0, 0.0, 4.0]);
        let rim: Vec<u32> = (0..2 * n)
            .map(|i| {
                let a = core::f64::consts::PI * i as f64 / n as f64;
                m.add_vertex([2.0 * a.cos(), 2.0 * a.sin(), 0.0])
            })
            .collect();
        let half = |k: usize| -> Vec<u32> {
            let mut l: Vec<u32> = (0..=n).map(|i| rim[(k * n + i) % (2 * n)]).collect();
            l.push(apex);
            l
        };
        m.set_tag(1);
        m.add_surface_face(&cone, 1, &[half(0)], true).unwrap();
        m.set_tag(2);
        m.add_surface_face(&cone, 1, &[half(1)], true).unwrap();
        let rev: Vec<u32> = rim.iter().rev().copied().collect();
        m.set_tag(3);
        m.add_planar_face(&rev, &[]).unwrap();
        let (mesh, tags) = m.finish_tagged();
        assert!(closed(&mesh), "not watertight");
        // Lateral area π r l per half: π · 2 · √20 / 2.
        let want = core::f64::consts::PI * 20f64.sqrt();
        for tag in [1, 2] {
            let area: f64 = mesh
                .triangles
                .iter()
                .zip(&tags)
                .filter(|(_, &g)| g == tag)
                .map(|(t, _)| {
                    let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
                    let n = cross_raw(
                        [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                        [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
                    );
                    0.5 * dot_raw(n, n).sqrt()
                })
                .sum();
            assert!(
                (area - want).abs() / want < 0.005,
                "half {tag}: {area} vs {want}"
            );
        }
        // π r² h / 3; the 48-gon rim loses ~0.3 %.
        let exact = core::f64::consts::PI * 4.0 * 4.0 / 3.0;
        let vol = mesh.signed_volume();
        assert!((vol - exact).abs() / exact < 0.005, "{vol} vs {exact}");
    }

    #[test]
    fn polygon_triangulation_with_hole() {
        let outer = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        let hole = vec![[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]];
        let t = triangulate_polygon(&outer, &[hole]).unwrap();
        assert_eq!(t.len(), 8);
    }
}
