//! B-spline curves — the `IfcBSplineCurve` family evaluated by de
//! Boor's recurrence.
//!
//! The EXPRESS text (`IFC4X3_ADD2.exp`, identical in `IFC4_ADD2.exp`)
//! carries the data: `Degree`, the control-point list, the distinct
//! `Knots` with their `KnotMultiplicities` (curves) / `UKnots` +
//! `UMultiplicities` and `VKnots` + `VMultiplicities` (surfaces), and
//! for the `IfcRational…` subtypes a parallel `WeightsData` list. The
//! `IfcConstraintsParamBSpline` schema function pins the consistency
//! rules this module enforces before evaluating: degree ≥ 1, at least
//! two distinct knots, at least `Degree + 1` control points, the
//! multiplicities summing to `Degree + (control points) + 1`, end
//! multiplicities in `1..=Degree + 1`, interior ones in `1..=Degree`,
//! and strictly increasing knot values. The IFC 2x3 `IfcBezierCurve` /
//! `IfcRationalBezierCurve` carry no knot list: they are the piecewise
//! Bézier special case, so the knot vector is synthesised as
//! `0 (×Degree+1), 1 (×Degree), …, k (×Degree+1)`.
//!
//! Evaluation is the textbook de Boor recurrence on the expanded knot
//! vector, carried out in homogeneous coordinates so rational (NURBS)
//! and polynomial curves share one path; the curve parameter domain is
//! `[knot[Degree], knot[n + 1]]` of the expanded vector (which allows
//! the unclamped vectors the schema permits). Sampling is per non-empty
//! knot span, dense enough that a quadratic-rational circle meshes at
//! the same density as an `IfcCircle`, and capped in total so a hostile
//! control net cannot demand an unbounded point run.

use super::{cartesian_point, GeometryError, CIRCLE_SEGMENTS};
use crate::parser::StepFile;
use crate::value::Value;

/// Upper bound on the samples one curve run may produce (a hostile knot
/// list with thousands of spans is clamped to this density).
const MAX_CURVE_SAMPLES: usize = 4096;

/// Upper bound on the control points (or knots / weights) one curve or
/// surface may carry — a hostile list past this is rejected outright.
const MAX_CONTROL_POINTS: usize = 65_536;

/// Numeric attribute, unwrapping a typed measure wrapper
/// (`IFCPARAMETERVALUE(0.5)`, `IFCINTEGER(3)`) when the writer used one.
fn number(v: &Value) -> Option<f64> {
    match v {
        Value::Typed { args, .. } => args.first().and_then(number),
        other => other.as_number(),
    }
}

/// Read a list attribute of numbers.
fn number_list(v: Option<&Value>) -> Result<Vec<f64>, GeometryError> {
    let items = v
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadProfile)?;
    if items.len() > MAX_CONTROL_POINTS {
        return Err(GeometryError::BadProfile);
    }
    items
        .iter()
        .map(|it| number(it).ok_or(GeometryError::BadProfile))
        .collect()
}

/// Read a list attribute of integers (knot multiplicities).
fn integer_list(v: Option<&Value>) -> Result<Vec<usize>, GeometryError> {
    let items = v
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadProfile)?;
    if items.len() > MAX_CONTROL_POINTS {
        return Err(GeometryError::BadProfile);
    }
    items
        .iter()
        .map(|it| {
            let n = match it {
                Value::Typed { args, .. } => args.first().and_then(Value::as_integer),
                other => other.as_integer(),
            }
            .ok_or(GeometryError::BadProfile)?;
            if n < 0 {
                return Err(GeometryError::BadProfile);
            }
            Ok(n as usize)
        })
        .collect()
}

/// The EXPRESS `IfcConstraintsParamBSpline(Degree, UpKnots, UpCp,
/// KnotMult, Knots)` schema function, transcribed for the WHERE-rule
/// checker: `up_knots` is `SIZEOF(Knots)`, `up_cp` the upper control
/// point index (`SIZEOF(ControlPointsList) − 1`).
pub(crate) fn constraints_param_bspline(
    degree: i64,
    up_knots: usize,
    up_cp: i64,
    knot_mult: &[i64],
    knots: &[f64],
) -> bool {
    if knot_mult.len() < up_knots || knots.len() < up_knots || up_knots == 0 {
        return false;
    }
    let sum: i64 = knot_mult[..up_knots].iter().sum();
    // Limits holding for all B-spline parametrisations.
    if degree < 1 || up_knots < 2 || up_cp < degree || sum != degree + up_cp + 2 {
        return false;
    }
    let k = knot_mult[0];
    if k < 1 || k > degree + 1 {
        return false;
    }
    for i in 1..up_knots {
        if knot_mult[i] < 1 || knots[i] <= knots[i - 1] {
            return false;
        }
        let k = knot_mult[i];
        if i + 1 < up_knots && k > degree {
            return false;
        }
        if i + 1 == up_knots && k > degree + 1 {
            return false;
        }
    }
    true
}

/// Expand `(knots, multiplicities)` into the full knot vector, checking
/// the `IfcConstraintsParamBSpline` invariants for `degree` and
/// `control_count` control points. Returns the expanded vector.
fn expand_knots(
    degree: usize,
    control_count: usize,
    knots: &[f64],
    mults: &[usize],
) -> Result<Vec<f64>, GeometryError> {
    // IfcConstraintsParamBSpline(Degree, UpKnots, UpCp, KnotMult, Knots):
    // UpKnots = SIZEOF(Knots), UpCp = control_count − 1.
    if degree < 1 || knots.len() < 2 || knots.len() != mults.len() || control_count < degree + 1 {
        return Err(GeometryError::BadProfile);
    }
    let sum: usize = mults.iter().sum();
    // Sum <> (Degree + UpCp + 2)  ⇔  sum ≠ degree + control_count + 1.
    if sum != degree + control_count + 1 {
        return Err(GeometryError::BadProfile);
    }
    let last = knots.len() - 1;
    for (i, (&k, &m)) in knots.iter().zip(mults).enumerate() {
        if !k.is_finite() || m < 1 {
            return Err(GeometryError::BadProfile);
        }
        let cap = if i == 0 || i == last {
            degree + 1
        } else {
            degree
        };
        if m > cap {
            return Err(GeometryError::BadProfile);
        }
        if i > 0 && k <= knots[i - 1] {
            return Err(GeometryError::BadProfile);
        }
    }
    let mut out = Vec::with_capacity(sum);
    for (&k, &m) in knots.iter().zip(mults) {
        out.extend(core::iter::repeat(k).take(m));
    }
    Ok(out)
}

/// The synthesised knot vector of a piecewise Bézier curve of `degree`
/// over `control_count` control points (IFC 2x3 `IfcBezierCurve`): the
/// control count minus one must be a multiple of the degree.
fn bezier_knots(degree: usize, control_count: usize) -> Result<Vec<f64>, GeometryError> {
    if degree < 1 || control_count < degree + 1 || (control_count - 1) % degree != 0 {
        return Err(GeometryError::BadProfile);
    }
    let spans = (control_count - 1) / degree;
    let mut knots = Vec::with_capacity(spans + 1);
    let mut mults = Vec::with_capacity(spans + 1);
    for i in 0..=spans {
        knots.push(i as f64);
        mults.push(if i == 0 || i == spans {
            degree + 1
        } else {
            degree
        });
    }
    expand_knots(degree, control_count, &knots, &mults)
}

/// Locate the knot span index `k` with `knots[k] <= t < knots[k + 1]`
/// within the valid domain `[knots[degree], knots[n + 1]]` (the last
/// non-empty span is returned for `t` at the domain end).
fn find_span(knots: &[f64], degree: usize, control_count: usize, t: f64) -> usize {
    let n = control_count - 1;
    let (lo, hi) = (degree, n + 1);
    if t >= knots[hi] {
        // Walk back to the last span with positive width.
        let mut k = hi - 1;
        while k > lo && knots[k] >= knots[k + 1] {
            k -= 1;
        }
        return k;
    }
    let mut k = lo;
    while k + 1 < hi && t >= knots[k + 1] {
        k += 1;
    }
    k
}

/// De Boor's recurrence on homogeneous 4-vectors: `points[i]` is
/// `(w·x, w·y, w·z, w)`; returns the homogeneous result at `t`.
fn de_boor(knots: &[f64], degree: usize, points: &[[f64; 4]], t: f64) -> [f64; 4] {
    let k = find_span(knots, degree, points.len(), t);
    let mut d: Vec<[f64; 4]> = (0..=degree).map(|j| points[k + j - degree]).collect();
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = j + k - degree;
            let denom = knots[i + degree + 1 - r] - knots[i];
            let alpha = if denom.abs() > 0.0 {
                (t - knots[i]) / denom
            } else {
                0.0
            };
            let (a, b) = (d[j - 1], d[j]);
            d[j] = [
                (1.0 - alpha) * a[0] + alpha * b[0],
                (1.0 - alpha) * a[1] + alpha * b[1],
                (1.0 - alpha) * a[2] + alpha * b[2],
                (1.0 - alpha) * a[3] + alpha * b[3],
            ];
        }
    }
    d[degree]
}

/// Project a homogeneous point.
fn dehomogenise(h: [f64; 4]) -> [f64; 3] {
    if h[3].abs() > 0.0 {
        [h[0] / h[3], h[1] / h[3], h[2] / h[3]]
    } else {
        [h[0], h[1], h[2]]
    }
}

/// A B-spline (or NURBS) curve ready for evaluation.
#[derive(Debug, Clone)]
pub(super) struct BSplineCurve {
    degree: usize,
    /// Homogeneous control points `(w·x, w·y, w·z, w)`.
    points: Vec<[f64; 4]>,
    /// The expanded knot vector (`points.len() + degree + 1` values).
    knots: Vec<f64>,
}

impl BSplineCurve {
    /// Build from an `IfcBSplineCurveWithKnots` /
    /// `IfcRationalBSplineCurveWithKnots` (IFC4+) or an `IfcBezierCurve`
    /// / `IfcRationalBezierCurve` (IFC 2x3) instance.
    ///
    /// Attribute order (EXPRESS): `IfcBSplineCurve(Degree,
    /// ControlPointsList, CurveForm, ClosedCurve, SelfIntersect)`, then
    /// `…WithKnots(KnotMultiplicities, Knots, KnotSpec)`, then
    /// `IfcRational…(WeightsData)`. The 2x3 rational Bézier appends
    /// `WeightsData` directly after the five base attributes.
    pub(super) fn from_instance(
        step: &StepFile,
        keyword: &str,
        args: &[Value],
    ) -> Result<Self, GeometryError> {
        let degree = args
            .first()
            .and_then(|v| match v {
                Value::Typed { args, .. } => args.first().and_then(Value::as_integer),
                other => other.as_integer(),
            })
            .ok_or(GeometryError::BadProfile)?;
        if !(1..=32).contains(&degree) {
            return Err(GeometryError::BadProfile);
        }
        let degree = degree as usize;
        let cps = args
            .get(1)
            .and_then(Value::as_list)
            .ok_or(GeometryError::BadProfile)?;
        if cps.len() < 2 || cps.len() > MAX_CONTROL_POINTS {
            return Err(GeometryError::BadProfile);
        }
        let mut control: Vec<[f64; 3]> = Vec::with_capacity(cps.len());
        for p in cps {
            control.push(cartesian_point(step, Some(p))?);
        }
        let (knots, weights) = match keyword {
            "IFCBSPLINECURVEWITHKNOTS" | "IFCRATIONALBSPLINECURVEWITHKNOTS" => {
                let mults = integer_list(args.get(5))?;
                let knot_values = number_list(args.get(6))?;
                let knots = expand_knots(degree, control.len(), &knot_values, &mults)?;
                let weights = if keyword == "IFCRATIONALBSPLINECURVEWITHKNOTS" {
                    Some(number_list(args.get(8))?)
                } else {
                    None
                };
                (knots, weights)
            }
            "IFCBEZIERCURVE" | "IFCRATIONALBEZIERCURVE" => {
                let knots = bezier_knots(degree, control.len())?;
                let weights = if keyword == "IFCRATIONALBEZIERCURVE" {
                    Some(number_list(args.get(5))?)
                } else {
                    None
                };
                (knots, weights)
            }
            other => return Err(GeometryError::Unsupported(other.to_string())),
        };
        let points = homogenise(&control, weights.as_deref())?;
        Ok(Self {
            degree,
            points,
            knots,
        })
    }

    /// The parameter domain `[knot[Degree], knot[n + 1]]`.
    pub(super) fn domain(&self) -> (f64, f64) {
        let n = self.points.len() - 1;
        (self.knots[self.degree], self.knots[n + 1])
    }

    /// The curve point at parameter `t` (clamped to the domain).
    pub(super) fn point_at(&self, t: f64) -> [f64; 3] {
        let (t0, t1) = self.domain();
        dehomogenise(de_boor(
            &self.knots,
            self.degree,
            &self.points,
            t.clamp(t0, t1),
        ))
    }

    /// The parameter values of the sampling run over `[t0, t1]`
    /// (sub-range of the domain): every distinct knot inside the range
    /// plus evenly spaced interior samples per span, so the run
    /// resolves each span with at least [`CIRCLE_SEGMENTS`]`/spans`
    /// points and never more than [`MAX_CURVE_SAMPLES`] in total. A
    /// degree-1 curve (the `POLYLINE_FORM`) is exactly its control
    /// polygon, so only the knots are emitted.
    pub(super) fn sample_params(&self, t0: f64, t1: f64) -> Vec<f64> {
        let (d0, d1) = self.domain();
        let (t0, t1) = (t0.clamp(d0, d1), t1.clamp(d0, d1));
        let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        // Distinct knots strictly inside (lo, hi).
        let n = self.points.len() - 1;
        let mut breaks: Vec<f64> = vec![lo];
        for &k in &self.knots[self.degree..=n + 1] {
            if k > lo && k < hi && breaks.last().is_some_and(|&b| k > b) {
                breaks.push(k);
            }
        }
        if hi > lo {
            breaks.push(hi);
        }
        let spans = breaks.len().saturating_sub(1).max(1);
        let per_span = if self.degree == 1 {
            1
        } else {
            (CIRCLE_SEGMENTS.div_ceil(spans))
                .max(4)
                .min(MAX_CURVE_SAMPLES.div_ceil(spans).max(1))
        };
        let mut out = Vec::with_capacity(spans * per_span + 1);
        for w in breaks.windows(2) {
            let (a, b) = (w[0], w[1]);
            for i in 0..per_span {
                out.push(a + (b - a) * (i as f64) / (per_span as f64));
            }
        }
        out.push(*breaks.last().unwrap_or(&lo));
        if t0 > t1 {
            out.reverse();
        }
        out
    }

    /// The sampled point run over the whole domain.
    pub(super) fn sample(&self) -> Vec<[f64; 3]> {
        let (t0, t1) = self.domain();
        self.sample_range(t0, t1)
    }

    /// The sampled point run over `[t0, t1]` (reversed when `t0 > t1`).
    pub(super) fn sample_range(&self, t0: f64, t1: f64) -> Vec<[f64; 3]> {
        self.sample_params(t0, t1)
            .into_iter()
            .map(|t| self.point_at(t))
            .collect()
    }
}

/// Lift control points to homogeneous coordinates with the optional
/// weights (`IfcCurveWeightsPositive`: every weight > 0).
fn homogenise(
    control: &[[f64; 3]],
    weights: Option<&[f64]>,
) -> Result<Vec<[f64; 4]>, GeometryError> {
    match weights {
        Some(w) => {
            if w.len() != control.len() {
                return Err(GeometryError::BadProfile);
            }
            control
                .iter()
                .zip(w)
                .map(|(p, &w)| {
                    if w <= 0.0 || !w.is_finite() {
                        return Err(GeometryError::BadProfile);
                    }
                    Ok([p[0] * w, p[1] * w, p[2] * w, w])
                })
                .collect()
        }
        None => Ok(control.iter().map(|p| [p[0], p[1], p[2], 1.0]).collect()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line_curve(
        degree: usize,
        control: &[[f64; 3]],
        knots: &[f64],
        mults: &[usize],
    ) -> BSplineCurve {
        let expanded = expand_knots(degree, control.len(), knots, mults).unwrap();
        BSplineCurve {
            degree,
            points: homogenise(control, None).unwrap(),
            knots: expanded,
        }
    }

    #[test]
    fn knot_expansion_enforces_schema_constraints() {
        // 4 control points, degree 3: sum of multiplicities must be 8.
        assert!(expand_knots(3, 4, &[0.0, 1.0], &[4, 4]).is_ok());
        assert!(expand_knots(3, 4, &[0.0, 1.0], &[4, 3]).is_err());
        // Interior multiplicity above the degree is rejected.
        assert!(expand_knots(2, 5, &[0.0, 1.0, 2.0], &[3, 3, 2]).is_err());
        assert!(expand_knots(2, 5, &[0.0, 1.0, 2.0], &[3, 2, 3]).is_ok());
        // Knots must strictly increase.
        assert!(expand_knots(2, 5, &[0.0, 0.0, 2.0], &[3, 2, 3]).is_err());
        // Degree needs Degree + 1 control points.
        assert!(expand_knots(3, 3, &[0.0, 1.0], &[4, 3]).is_err());
    }

    #[test]
    fn degree_one_is_the_control_polygon() {
        let c = line_curve(
            1,
            &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
            &[0.0, 1.0, 2.0],
            &[2, 1, 2],
        );
        let pts = c.sample();
        assert_eq!(pts.len(), 3);
        assert_eq!(pts[1], [1.0, 0.0, 0.0]);
        let mid = c.point_at(1.5);
        assert!((mid[0] - 1.0).abs() < 1e-12 && (mid[1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn cubic_bezier_endpoints_and_midpoint() {
        // Clamped cubic with one span is a Bézier: B(0.5) = (P0 + 3P1 + 3P2 + P3)/8.
        let control = [
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 0.0],
            [3.0, 2.0, 0.0],
            [4.0, 0.0, 0.0],
        ];
        let c = line_curve(3, &control, &[0.0, 1.0], &[4, 4]);
        assert_eq!(c.point_at(0.0), control[0]);
        assert_eq!(c.point_at(1.0), control[3]);
        let m = c.point_at(0.5);
        assert!((m[0] - 2.0).abs() < 1e-12);
        assert!((m[1] - 1.5).abs() < 1e-12);
        // Unclamped domain: the sampled run starts and ends on the ends.
        let s = c.sample();
        assert_eq!(s[0], control[0]);
        assert_eq!(*s.last().unwrap(), control[3]);
    }

    #[test]
    fn rational_quadratic_quarter_circle_is_exact() {
        // Weights (1, √2/2, 1) on (1,0),(1,1),(0,1) trace a unit quarter circle.
        let control = [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]];
        let w = [1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0];
        let c = BSplineCurve {
            degree: 2,
            points: homogenise(&control, Some(&w)).unwrap(),
            knots: expand_knots(2, 3, &[0.0, 1.0], &[3, 3]).unwrap(),
        };
        for p in c.sample() {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!((r - 1.0).abs() < 1e-12, "{p:?}");
        }
        assert!(c.sample().len() >= 5);
    }

    #[test]
    fn bezier_knots_need_a_span_multiple() {
        assert!(bezier_knots(3, 4).is_ok());
        assert!(bezier_knots(3, 7).is_ok());
        assert!(bezier_knots(3, 5).is_err());
        assert!(bezier_knots(2, 5).is_ok());
    }
}
