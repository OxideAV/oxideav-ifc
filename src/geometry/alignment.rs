//! IFC 4.3 alignment curves: `IfcCurveSegment` chains, the spiral
//! parent curves, `IfcGradientCurve` and `IfcOffsetCurveByDistances`.
//!
//! **Curve segments.** `IfcCurveSegment(Transition, Placement,
//! SegmentStart, SegmentLength, ParentCurve)` cuts a run out of its
//! parent curve — from `SegmentStart`, over `SegmentLength`, both
//! lengths along the parent, a negative length running against the
//! parent's sense — and inserts it with its start point on
//! `Placement.Location` and its direction of travel there along
//! `Placement.RefDirection` (IfcCurveSegment page: "As insertion point
//! SegmentStart is the reference point of Placement. RefDirection of
//! Placement also specifies the sense of the trimmed segment").
//!
//! A planar curve is fixed up to a rigid motion by its curvature as a
//! function of arc length, and the placement supplies exactly that
//! motion. So a segment is evaluated by integrating the parent's
//! curvature `κ(s)` over `[SegmentStart, SegmentStart + SegmentLength]`
//! from the placement point and heading (a run against the parent's
//! sense sees `−κ`); the parent's own `Position` never enters.
//!
//! | parent | curvature at arc length `s` |
//! |---|---|
//! | `IfcLine` | `0` |
//! | `IfcCircle(Radius)` | `1 / Radius` (the circle's sense is anti-clockwise) |
//! | `IfcClothoid(ClothoidConstant A)` | `s / (A·|A|)` — linear in `s`, zero at the spiral's origin, `A² = R·L`; a negative constant turns right |
//! | `IfcCosineSpiral(CosineTerm, ConstantTerm)` | `1/ConstantTerm + cos(π·s / L) / CosineTerm`, `L` the segment length |
//!
//! The clothoid and cosine rows are this crate's reading of the two
//! attribute lists against the Annex E alignment listings, which they
//! reproduce (the transition ends at the listed point and heading with
//! the listed radius; the cosine segment starts and ends at the listed
//! radii). The remaining spirals (`IfcSineSpiral`, the second / third /
//! seventh order polynomial spirals) and `IfcPolynomialCurve` carry
//! term lists whose normalisation the staged pages do not give: they
//! are refused by keyword.
//!
//! **Stations.** Every evaluator returns, with each point, its
//! distance along the curve — exact arc length for curve segments
//! (not the chord sum of the samples), which is what linear placements
//! and directrix trims measure.
//!
//! **Gradient curve.** `IfcGradientCurve(Segments, SelfIntersect,
//! BaseCurve, EndPoint)`: `BaseCurve` is the horizontal projection and
//! `Segments` describe the elevation in the plane (distance along the
//! base curve, height). The 3-D point at distance `d` is the base
//! curve point at `d` lifted to the profile's height at `d`; its
//! parameter is the base curve's ("The value of the parameter equals
//! the parameter value of BaseCurve").
//!
//! **Offset curve by distances.** `IfcOffsetCurveByDistances(BasisCurve,
//! OffsetValues, Tag)`: each `IfcPointByDistanceExpression` gives the
//! lateral offset (positive to the left of travel) and vertical offset
//! (perpendicular to the tangent in the vertical plane through it) at
//! a distance along the basis curve; offsets vary linearly between
//! consecutive values. One value offsets the whole basis curve; several
//! span the curve between the first and last distance.

use super::{
    GeometryError, MAX_CURVE_DEPTH, axis2_placement_2d, cross_raw, curve_dimension,
    curve_points_2d_depth, curve_points_3d, normalise,
};
use crate::parser::{ParsedInstance, StepFile};
use crate::value::Value;

/// A sampled curve with the distance along it at every sample.
#[derive(Debug, Clone, Default)]
pub(super) struct Stationed {
    pub(super) points: Vec<[f64; 3]>,
    pub(super) stations: Vec<f64>,
}

impl Stationed {
    fn push(&mut self, p: [f64; 3], station: f64) {
        if let (Some(last), Some(&at)) = (self.points.last(), self.stations.last()) {
            let scale = last.iter().fold(1.0f64, |m, c| m.max(c.abs()));
            let same = (0..3).all(|k| (last[k] - p[k]).abs() <= 1e-9 * scale);
            if same && (station - at).abs() <= 1e-9 * at.abs().max(1.0) {
                return;
            }
        }
        self.points.push(p);
        self.stations.push(station);
    }

    /// The point at distance `d` (clamped to the curve).
    fn at(&self, d: f64) -> [f64; 3] {
        let n = self.points.len();
        let i = self.stations.partition_point(|&s| s < d).clamp(1, n - 1);
        let (s0, s1) = (self.stations[i - 1], self.stations[i]);
        let t = if s1 > s0 {
            ((d - s0) / (s1 - s0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (a, b) = (self.points[i - 1], self.points[i]);
        [
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        ]
    }

    /// The unit direction of travel around distance `d`.
    fn tangent_at(&self, d: f64) -> Option<[f64; 3]> {
        let n = self.points.len();
        let i = self.stations.partition_point(|&s| s < d).clamp(1, n - 1);
        let (a, b) = (self.points[i - 1], self.points[i]);
        normalise([b[0] - a[0], b[1] - a[1], b[2] - a[2]])
    }
}

/// A length-valued `IfcCurveMeasureSelect` (typed or bare).
fn length_measure(v: Option<&Value>, what: &str) -> Result<f64, GeometryError> {
    match v {
        Some(v @ (Value::Real(_) | Value::Integer(_))) => v.as_number(),
        Some(Value::Typed { keyword, args })
            if keyword == "IFCLENGTHMEASURE" || keyword == "IFCNONNEGATIVELENGTHMEASURE" =>
        {
            args.first().and_then(Value::as_number)
        }
        Some(Value::Typed { keyword, .. }) => {
            return Err(GeometryError::Unsupported(format!("{what}({keyword})")));
        }
        _ => None,
    }
    .filter(|v| v.is_finite())
    .ok_or(GeometryError::BadCoordinate)
}

/// The curvature of a curve-segment parent as a function of arc
/// length, for a segment of length `span`.
fn parent_curvature(
    parent: &ParsedInstance,
    span: f64,
) -> Result<Box<dyn Fn(f64) -> f64>, GeometryError> {
    let term = |i: usize| -> Result<Option<f64>, GeometryError> {
        match parent.args.get(i) {
            None | Some(Value::Unset) => Ok(None),
            Some(v) => match v.as_number().filter(|x| x.is_finite() && *x != 0.0) {
                Some(x) => Ok(Some(x)),
                None => Err(GeometryError::BadProfile),
            },
        }
    };
    match parent.keyword.as_str() {
        "IFCLINE" => Ok(Box::new(|_| 0.0)),
        // IfcCircle(Position, Radius).
        "IFCCIRCLE" => {
            let r = term(1)?
                .filter(|r| *r > 0.0)
                .ok_or(GeometryError::BadProfile)?;
            Ok(Box::new(move |_| 1.0 / r))
        }
        // IfcClothoid(Position, ClothoidConstant).
        "IFCCLOTHOID" => {
            let a = term(1)?.ok_or(GeometryError::BadProfile)?;
            Ok(Box::new(move |s| s / (a * a.abs())))
        }
        // IfcCosineSpiral(Position, CosineTerm, ConstantTerm).
        "IFCCOSINESPIRAL" => {
            let cosine = term(1)?.ok_or(GeometryError::BadProfile)?;
            let constant = term(2)?.map_or(0.0, |c| 1.0 / c);
            if span <= 0.0 {
                return Ok(Box::new(move |_| constant + 1.0 / cosine));
            }
            Ok(Box::new(move |s| {
                constant + (core::f64::consts::PI * s / span).cos() / cosine
            }))
        }
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
}

/// Internal integration steps per emitted sample.
const SUBSTEPS: usize = 8;

/// Most samples one curve segment may emit.
const MAX_SEGMENT_SAMPLES: usize = 4096;

/// Evaluate one `IfcCurveSegment` in the plane of its 2-D placement:
/// the sampled points with their distance from the segment start.
fn curve_segment(
    step: &StepFile,
    seg: &ParsedInstance,
) -> Result<Vec<([f64; 2], f64)>, GeometryError> {
    let placement_id = seg
        .args
        .get(1)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let placement = step
        .get(placement_id)
        .ok_or(GeometryError::MissingInstance(placement_id))?;
    if placement.keyword != "IFCAXIS2PLACEMENT2D" {
        // A 3-D or linear segment placement positions a segment of a
        // cant / superelevation curve.
        return Err(GeometryError::Unsupported(format!(
            "IFCCURVESEGMENT({})",
            placement.keyword
        )));
    }
    let frame = axis2_placement_2d(step, placement_id)?;
    let start = length_measure(seg.args.get(2), "IFCCURVESEGMENT")?;
    let length = length_measure(seg.args.get(3), "IFCCURVESEGMENT")?;
    let parent_id = seg
        .args
        .get(4)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let parent = step
        .get(parent_id)
        .ok_or(GeometryError::MissingInstance(parent_id))?;
    let span = length.abs();
    let curvature = parent_curvature(parent, span)?;
    let mut out = vec![(frame.origin, 0.0)];
    if span <= 0.0 {
        return Ok(out);
    }
    let sense = if length < 0.0 { -1.0 } else { 1.0 };
    // Total turning decides the sample count.
    let probes = 64;
    let mut turn = 0.0;
    let mut varies = false;
    let k0 = curvature(start);
    for i in 0..probes {
        let k = curvature(start + sense * span * (i as f64 + 0.5) / probes as f64);
        turn += k.abs() * span / probes as f64;
        varies |= (k - k0).abs() > 1e-12 * k0.abs().max(1e-12);
    }
    if !turn.is_finite() {
        return Err(GeometryError::BadProfile);
    }
    if !varies && k0 == 0.0 {
        // A straight run: its end point, exactly.
        let end = [
            frame.origin[0] + span * frame.x_axis[0],
            frame.origin[1] + span * frame.x_axis[1],
        ];
        out.push((end, span));
        return Ok(out);
    }
    // Alignment radii are large against the lengths that matter across
    // them: a quarter of the circle step per sample, and at least
    // sixteen samples on any curved run (a sag curve turning a
    // milliradian still follows its arc).
    let step_angle = 0.5 * core::f64::consts::PI / super::CIRCLE_SEGMENTS as f64;
    let samples = ((turn / step_angle).ceil() as usize).clamp(16, MAX_SEGMENT_SAMPLES);
    let h = span / (samples * SUBSTEPS) as f64;
    let mut heading = frame.x_axis[1].atan2(frame.x_axis[0]);
    let mut p = frame.origin;
    // The heading change over [t, t + w] of the run: Simpson's rule on
    // the curvature seen in the direction of travel.
    let bend = |t: f64, w: f64| -> f64 {
        let k = |x: f64| sense * curvature(start + sense * x);
        w / 6.0 * (k(t) + 4.0 * k(t + 0.5 * w) + k(t + w))
    };
    // Three-point Gauss–Legendre nodes and weights on [0, 1].
    let offset = 0.5 * (0.6f64).sqrt();
    let nodes = [
        (0.5 - offset, 5.0 / 18.0),
        (0.5, 8.0 / 18.0),
        (0.5 + offset, 5.0 / 18.0),
    ];
    for i in 0..samples {
        for j in 0..SUBSTEPS {
            let t = (i * SUBSTEPS + j) as f64 * h;
            for (node, weight) in nodes {
                let dir = heading + bend(t, node * h);
                p = [p[0] + weight * h * dir.cos(), p[1] + weight * h * dir.sin()];
            }
            heading += bend(t, h);
        }
        if !(p[0].is_finite() && p[1].is_finite()) {
            return Err(GeometryError::BadCoordinate);
        }
        out.push((p, span * (i + 1) as f64 / samples as f64));
    }
    Ok(out)
}

/// Whether a composite curve's segments are `IfcCurveSegment`s.
pub(super) fn has_curve_segments(step: &StepFile, args: &[Value]) -> bool {
    args.first()
        .and_then(Value::as_list)
        .and_then(<[Value]>::first)
        .and_then(|s| step.resolve(s))
        .is_some_and(|s| s.keyword == "IFCCURVESEGMENT")
}

/// A chain of 2-D curve segments (`Segments` at argument 0): the
/// plane points with their distance from the chain start.
fn segment_chain(step: &StepFile, args: &[Value]) -> Result<Vec<([f64; 2], f64)>, GeometryError> {
    let segments = args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadProfile)?;
    let mut out: Vec<([f64; 2], f64)> = Vec::new();
    let mut travelled = 0.0;
    for seg in segments {
        let sid = seg.as_reference().ok_or(GeometryError::BadProfile)?;
        let sinst = step.get(sid).ok_or(GeometryError::MissingInstance(sid))?;
        if sinst.keyword != "IFCCURVESEGMENT" {
            return Err(GeometryError::Unsupported(sinst.keyword.clone()));
        }
        let run = curve_segment(step, sinst)?;
        let length = run.last().map_or(0.0, |r| r.1);
        for (p, t) in run {
            let station = travelled + t;
            if let Some(&(q, at)) = out.last() {
                let scale = q[0].abs().max(q[1].abs()).max(1.0);
                if (q[0] - p[0]).abs() <= 1e-9 * scale
                    && (q[1] - p[1]).abs() <= 1e-9 * scale
                    && (station - at).abs() <= 1e-9 * at.abs().max(1.0)
                {
                    continue;
                }
            }
            out.push((p, station));
        }
        travelled += length;
    }
    if out.len() < 2 {
        return Err(GeometryError::BadProfile);
    }
    Ok(out)
}

/// The 2-D points of an `IfcCompositeCurve` of curve segments.
pub(super) fn composite_points_2d(
    step: &StepFile,
    args: &[Value],
) -> Result<Vec<[f64; 2]>, GeometryError> {
    Ok(segment_chain(step, args)?
        .into_iter()
        .map(|(p, _)| p)
        .collect())
}

/// Any curve as a stationed 3-D run: an alignment curve with its true
/// distances, anything else by the chord lengths of its samples (a
/// 2-D curve lies in z = 0).
fn stationed_any(step: &StepFile, id: u64, depth: usize) -> Result<Stationed, GeometryError> {
    if let Some(run) = stationed(step, id, depth)? {
        return Ok(run);
    }
    let points: Vec<[f64; 3]> = if curve_dimension(step, id, depth) == Some(2) {
        curve_points_2d_depth(step, id, depth + 1)?
            .into_iter()
            .map(|p| [p[0], p[1], 0.0])
            .collect()
    } else {
        curve_points_3d(step, id, depth + 1)?
    };
    let mut run = Stationed::default();
    let mut travelled = 0.0;
    for (i, &p) in points.iter().enumerate() {
        if i > 0 {
            travelled += super::dist(points[i - 1], p);
        }
        run.push(p, travelled);
    }
    if run.points.len() < 2 {
        return Err(GeometryError::BadProfile);
    }
    Ok(run)
}

/// The stationed 3-D run of an alignment curve — a composite curve of
/// curve segments, a gradient curve, an offset curve by distances —
/// or `None` for any other curve kind.
pub(super) fn stationed(
    step: &StepFile,
    id: u64,
    depth: usize,
) -> Result<Option<Stationed>, GeometryError> {
    if depth >= MAX_CURVE_DEPTH {
        return Err(GeometryError::BadProfile);
    }
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    match inst.keyword.as_str() {
        "IFCCOMPOSITECURVE" if has_curve_segments(step, &inst.args) => {
            let mut run = Stationed::default();
            for (p, station) in segment_chain(step, &inst.args)? {
                run.push([p[0], p[1], 0.0], station);
            }
            Ok(Some(run))
        }
        "IFCGRADIENTCURVE" => gradient_curve(step, &inst.args, depth).map(Some),
        "IFCOFFSETCURVEBYDISTANCES" => offset_by_distances(step, &inst.args, depth).map(Some),
        // The cant curve displaces and banks its base curve by rules
        // the staged pages only outline.
        "IFCSEGMENTEDREFERENCECURVE" => Err(GeometryError::Unsupported(inst.keyword.clone())),
        _ => Ok(None),
    }
}

/// The distances along curve `id` at the samples [`curve_points_3d`]
/// returns for it: true stations for an alignment curve (`None` for
/// any other kind — the caller then measures chords).
pub(super) fn stations(step: &StepFile, id: u64, samples: usize) -> Option<Vec<f64>> {
    let run = stationed(step, id, 0).ok()??;
    (run.stations.len() == samples).then_some(run.stations)
}

fn gradient_curve(
    step: &StepFile,
    args: &[Value],
    depth: usize,
) -> Result<Stationed, GeometryError> {
    let base_id = args
        .get(2)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let base = stationed_any(step, base_id, depth + 1)?;
    // The elevation profile: (distance along the base curve, height).
    let profile = segment_chain(step, args)?;
    let mut profile: Vec<[f64; 2]> = profile.into_iter().map(|(p, _)| p).collect();
    // The profile advances along the base curve; segment joints may
    // step back by the rounding of their authored coordinates only.
    let slack = 1e-6 * profile.iter().fold(1.0f64, |m, p| m.max(p[0].abs()));
    for i in 1..profile.len() {
        let before = profile[i - 1][0];
        if profile[i][0] < before - slack {
            return Err(GeometryError::BadProfile);
        }
        profile[i][0] = profile[i][0].max(before);
    }
    let height_at = |d: f64| -> f64 {
        let i = profile
            .partition_point(|p| p[0] < d)
            .clamp(1, profile.len() - 1);
        let (a, b) = (profile[i - 1], profile[i]);
        let t = if b[0] > a[0] {
            ((d - a[0]) / (b[0] - a[0])).clamp(0.0, 1.0)
        } else {
            0.0
        };
        a[1] + (b[1] - a[1]) * t
    };
    let (first, last) = (base.stations[0], base.stations[base.stations.len() - 1]);
    let lo = first.max(profile[0][0]);
    let hi = last.min(profile[profile.len() - 1][0]);
    if hi <= lo {
        return Err(GeometryError::BadProfile);
    }
    // Every horizontal sample and every profile break is a vertex.
    let mut cuts: Vec<f64> = base
        .stations
        .iter()
        .copied()
        .chain(profile.iter().map(|p| p[0]))
        .filter(|&d| d > lo && d < hi)
        .collect();
    cuts.push(lo);
    cuts.push(hi);
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let mut run = Stationed::default();
    for d in cuts {
        let p = base.at(d);
        run.push([p[0], p[1], p[2] + height_at(d)], d);
    }
    if run.points.len() < 2 {
        return Err(GeometryError::BadProfile);
    }
    Ok(run)
}

fn offset_by_distances(
    step: &StepFile,
    args: &[Value],
    depth: usize,
) -> Result<Stationed, GeometryError> {
    let basis_id = args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let basis = stationed_any(step, basis_id, depth + 1)?;
    let values = args
        .get(1)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    // (distance, lateral, vertical) per IfcPointByDistanceExpression
    // (DistanceAlong, OffsetLateral, OffsetVertical, OffsetLongitudinal,
    // BasisCurve).
    let mut offsets: Vec<[f64; 3]> = Vec::with_capacity(values.len());
    for v in values {
        let inst = step.resolve(v).ok_or(GeometryError::BadCoordinates)?;
        if inst.keyword != "IFCPOINTBYDISTANCEEXPRESSION" {
            return Err(GeometryError::Unsupported(inst.keyword.clone()));
        }
        let optional = |i: usize| -> Result<f64, GeometryError> {
            match inst.args.get(i) {
                None | Some(Value::Unset) => Ok(0.0),
                Some(v) => v
                    .as_number()
                    .filter(|x| x.is_finite())
                    .ok_or(GeometryError::BadCoordinate),
            }
        };
        if optional(3)? != 0.0 {
            // A longitudinal offset moves the point along the curve by
            // a rule the staged pages do not give.
            return Err(GeometryError::Unsupported(
                "IFCPOINTBYDISTANCEEXPRESSION(OffsetLongitudinal)".to_string(),
            ));
        }
        offsets.push([
            length_measure(inst.args.first(), "IFCPOINTBYDISTANCEEXPRESSION")?,
            optional(1)?,
            optional(2)?,
        ]);
    }
    if offsets.is_empty() {
        return Err(GeometryError::BadCoordinates);
    }
    offsets.sort_by(|a, b| {
        a[0].partial_cmp(&b[0])
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let (first, last) = (basis.stations[0], basis.stations[basis.stations.len() - 1]);
    let (lo, hi) = if offsets.len() == 1 {
        (first, last)
    } else {
        (
            first.max(offsets[0][0]),
            last.min(offsets[offsets.len() - 1][0]),
        )
    };
    if hi <= lo {
        return Err(GeometryError::BadProfile);
    }
    let offset_at = |d: f64| -> (f64, f64) {
        let i = offsets
            .partition_point(|o| o[0] < d)
            .clamp(1, offsets.len().max(2) - 1);
        let (a, b) = (offsets[i - 1], offsets[i.min(offsets.len() - 1)]);
        let t = if b[0] > a[0] {
            ((d - a[0]) / (b[0] - a[0])).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)
    };
    let mut cuts: Vec<f64> = basis
        .stations
        .iter()
        .copied()
        .chain(offsets.iter().map(|o| o[0]))
        .filter(|&d| d > lo && d < hi)
        .collect();
    cuts.push(lo);
    cuts.push(hi);
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let mut run = Stationed::default();
    for d in cuts {
        let p = basis.at(d);
        let tangent = basis.tangent_at(d).ok_or(GeometryError::BadProfile)?;
        // Left of travel in the horizontal plane; up perpendicular to
        // the tangent in the vertical plane through it.
        let left = normalise(cross_raw([0.0, 0.0, 1.0], tangent)).unwrap_or([0.0, 1.0, 0.0]);
        let up = normalise(cross_raw(tangent, left)).unwrap_or([0.0, 0.0, 1.0]);
        let (lateral, vertical) = offset_at(d);
        run.push(
            [
                p[0] + lateral * left[0] + vertical * up[0],
                p[1] + lateral * left[1] + vertical * up[1],
                p[2] + lateral * left[2] + vertical * up[2],
            ],
            d,
        );
    }
    if run.points.len() < 2 {
        return Err(GeometryError::BadProfile);
    }
    Ok(run)
}
