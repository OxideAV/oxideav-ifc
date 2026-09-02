//! Named parameterised profile contours — the `IfcParameterizedProfileDef`
//! family whose boundary is implicit in a handful of dimensions rather
//! than an explicit curve: I / L / T / U / Z / C sections, the rounded
//! rectangle and the (deprecated) trapezium.
//!
//! Conventions (parameterised-profiles digest §0): every contour is built
//! in a local frame whose origin is the **centre of the profile's
//! bounding box** (the trapezium centres on its `BottomXDim × YDim`
//! rectangle, §8), traversed **counter-clockwise**, then mapped by the
//! optional 2-D `Position` (applied by the caller). Optional radii keep
//! the schema's tri-state: an omitted (`$`) radius is *unknown* rather
//! than zero; this slice resolves it as a sharp corner, which the digest
//! lists as a permitted receiver choice ("simply assume zero values").
//!
//! Corner rounding is the standard tangent–tangent fillet (digest §0):
//! the arc is tangent to both adjoining faces, whether the corner is
//! re-entrant (a *fillet* radius: web-to-flange, leg-to-leg) or convex
//! (an *edge* radius: flange toes, web ends). Flange / web / leg slopes
//! tilt the inner faces; the thickness is taken at the reference station
//! the digest pins for the T-shape (`FlangeWidth/4` from the web
//! centre-line, §3) and, by the digest's stated inference, at the same
//! station for the I and U shapes. Where no station is documented (the
//! T web, the L legs) the thickness is taken at the mid-length of the
//! tapered face and the choice is recorded in the function docs.

use super::{GeometryError, CIRCLE_SEGMENTS};
use crate::parser::StepFile;
use crate::value::Value;

/// A polygon vertex with the radius its corner is rounded by (`0.0` =
/// sharp).
#[derive(Clone, Copy, Debug)]
pub(super) struct Corner {
    pub(super) p: [f64; 2],
    pub(super) r: f64,
}

pub(super) fn corner(x: f64, y: f64, r: f64) -> Corner {
    Corner { p: [x, y], r }
}

/// Numeric attribute, unwrapping a typed measure wrapper
/// (`IFCPOSITIVELENGTHMEASURE(5.)`) when the writer used one.
fn number(v: &Value) -> Option<f64> {
    match v {
        Value::Typed { args, .. } => args.first().and_then(Value::as_number),
        other => other.as_number(),
    }
}

/// Required positive length at attribute `index`.
fn positive(args: &[Value], index: usize) -> Result<f64, GeometryError> {
    let v = args
        .get(index)
        .and_then(number)
        .ok_or(GeometryError::BadProfile)?;
    if v > 0.0 && v.is_finite() {
        Ok(v)
    } else {
        Err(GeometryError::BadProfile)
    }
}

/// Required signed length at attribute `index`.
fn signed(args: &[Value], index: usize) -> Result<f64, GeometryError> {
    let v = args
        .get(index)
        .and_then(number)
        .ok_or(GeometryError::BadProfile)?;
    if v.is_finite() {
        Ok(v)
    } else {
        Err(GeometryError::BadProfile)
    }
}

/// Optional non-negative length at attribute `index`: `$` / missing →
/// `0.0` (the "assume zero" receiver choice); a negative value is
/// malformed.
fn radius(args: &[Value], index: usize) -> Result<f64, GeometryError> {
    match args.get(index).and_then(number) {
        None => Ok(0.0),
        Some(r) if r >= 0.0 && r.is_finite() => Ok(r),
        Some(_) => Err(GeometryError::BadProfile),
    }
}

/// Optional slope angle at attribute `index`, converted to radians by
/// the model's plane-angle unit, returned as `tan(angle)` (`0.0` when
/// unset). Slopes at or beyond 90° are malformed.
fn slope_tan(step: &StepFile, args: &[Value], index: usize) -> Result<f64, GeometryError> {
    match args.get(index).and_then(number) {
        None => Ok(0.0),
        Some(a) => {
            let rad = a * crate::schema::plane_angle_unit_scale(step).unwrap_or(1.0);
            if !rad.is_finite() || rad.abs() >= core::f64::consts::FRAC_PI_2 {
                return Err(GeometryError::BadProfile);
            }
            Ok(rad.tan())
        }
    }
}

/// Intersection of the lines `a + s·da` and `b + t·db`.
fn intersect_lines(
    a: [f64; 2],
    da: [f64; 2],
    b: [f64; 2],
    db: [f64; 2],
) -> Result<[f64; 2], GeometryError> {
    let det = da[0] * db[1] - da[1] * db[0];
    if det.abs() < 1e-15 {
        return Err(GeometryError::BadProfile);
    }
    let s = ((b[0] - a[0]) * db[1] - (b[1] - a[1]) * db[0]) / det;
    Ok([a[0] + s * da[0], a[1] + s * da[1]])
}

/// Replace every rounded corner of a simple polygon by its tangent arc
/// (digest §0 "Reading the fillet/edge-radius attributes"): for the two
/// edges meeting at the corner with interior angle `φ`, the tangent
/// points lie `r / tan(φ/2)` along each edge and the arc centre
/// `r / sin(φ/2)` along the bisector. Arcs are sampled at the
/// [`CIRCLE_SEGMENTS`] density; a corner whose tangent lengths would
/// overrun an adjoining edge (a radius too large for its corner) is
/// malformed.
pub(super) fn round_corners(corners: &[Corner]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let n = corners.len();
    if n < 3 {
        return Err(GeometryError::BadProfile);
    }
    // Tangent distance per corner (0 for sharp corners).
    let mut tangent = vec![0.0; n];
    for i in 0..n {
        let c = corners[i];
        if c.r <= 0.0 {
            continue;
        }
        let prev = corners[(i + n - 1) % n].p;
        let next = corners[(i + 1) % n].p;
        let u1 = unit([prev[0] - c.p[0], prev[1] - c.p[1]])?;
        let u2 = unit([next[0] - c.p[0], next[1] - c.p[1]])?;
        let phi = (u1[0] * u2[0] + u1[1] * u2[1]).clamp(-1.0, 1.0).acos();
        if phi <= 1e-9 || phi >= core::f64::consts::PI - 1e-9 {
            return Err(GeometryError::BadProfile); // spike or straight
        }
        tangent[i] = c.r / (phi / 2.0).tan();
    }
    // Tangent lengths must fit on every edge.
    for i in 0..n {
        let j = (i + 1) % n;
        let d = [
            corners[j].p[0] - corners[i].p[0],
            corners[j].p[1] - corners[i].p[1],
        ];
        let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
        if tangent[i] + tangent[j] > len * (1.0 + 1e-9) + 1e-12 {
            return Err(GeometryError::BadProfile);
        }
    }
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(n * 4);
    for i in 0..n {
        let c = corners[i];
        if c.r <= 0.0 {
            push_unique(&mut out, c.p);
            continue;
        }
        let prev = corners[(i + n - 1) % n].p;
        let next = corners[(i + 1) % n].p;
        let u1 = unit([prev[0] - c.p[0], prev[1] - c.p[1]])?;
        let u2 = unit([next[0] - c.p[0], next[1] - c.p[1]])?;
        let phi = (u1[0] * u2[0] + u1[1] * u2[1]).clamp(-1.0, 1.0).acos();
        let d = tangent[i];
        let t1 = [c.p[0] + u1[0] * d, c.p[1] + u1[1] * d];
        let t2 = [c.p[0] + u2[0] * d, c.p[1] + u2[1] * d];
        let bis = unit([u1[0] + u2[0], u1[1] + u2[1]])?;
        let h = c.r / (phi / 2.0).sin();
        let centre = [c.p[0] + bis[0] * h, c.p[1] + bis[1] * h];
        // Sweep from t1 to t2 the short way round (the arc spans π − φ).
        let a0 = (t1[1] - centre[1]).atan2(t1[0] - centre[0]);
        let a1 = (t2[1] - centre[1]).atan2(t2[0] - centre[0]);
        let mut delta = a1 - a0;
        while delta > core::f64::consts::PI {
            delta -= 2.0 * core::f64::consts::PI;
        }
        while delta < -core::f64::consts::PI {
            delta += 2.0 * core::f64::consts::PI;
        }
        // Segment count from the swept fraction of a turn. The fraction
        // is nudged below the exact value before ceil() so a right
        // angle is exactly CIRCLE_SEGMENTS / 4 on every platform (a
        // last-ulp difference in atan2 must not change the point count
        // — a tapered loft pairs rings by count).
        let fraction = delta.abs() / (2.0 * core::f64::consts::PI);
        let segs = ((CIRCLE_SEGMENTS as f64 * fraction - 1e-6).ceil() as usize).max(2);
        for k in 0..=segs {
            let a = a0 + delta * (k as f64) / (segs as f64);
            push_unique(
                &mut out,
                [centre[0] + c.r * a.cos(), centre[1] + c.r * a.sin()],
            );
        }
    }
    // A rounding that consumed a whole edge (r = XDim/2) makes the last
    // arc end where the first began.
    if out.len() > 1 {
        let (f, l) = (out[0], out[out.len() - 1]);
        if (f[0] - l[0]).abs() < 1e-9 && (f[1] - l[1]).abs() < 1e-9 {
            out.pop();
        }
    }
    if out.len() < 3 {
        return Err(GeometryError::BadProfile);
    }
    Ok(out)
}

fn unit(v: [f64; 2]) -> Result<[f64; 2], GeometryError> {
    let m = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if m < 1e-15 {
        return Err(GeometryError::BadProfile);
    }
    Ok([v[0] / m, v[1] / m])
}

fn push_unique(out: &mut Vec<[f64; 2]>, p: [f64; 2]) {
    if let Some(last) = out.last() {
        if (last[0] - p[0]).abs() < 1e-9 && (last[1] - p[1]).abs() < 1e-9 {
            return;
        }
    }
    out.push(p);
}

/// The local-frame contour of a named parameterised profile, or `None`
/// when `keyword` is not one of this module's profile kinds. `args` are
/// the instance's positional attributes (`ProfileType`, `ProfileName`,
/// `Position`, then the profile's own dimensions from index 3).
pub(super) fn parameterised_ring(
    step: &StepFile,
    keyword: &str,
    args: &[Value],
) -> Result<Option<Vec<[f64; 2]>>, GeometryError> {
    let ring = match keyword {
        "IFCISHAPEPROFILEDEF" => i_shape(step, args)?,
        "IFCASYMMETRICISHAPEPROFILEDEF" => asymmetric_i_shape(step, args)?,
        "IFCLSHAPEPROFILEDEF" => l_shape(step, args)?,
        "IFCTSHAPEPROFILEDEF" => t_shape(step, args)?,
        "IFCUSHAPEPROFILEDEF" => u_shape(step, args)?,
        "IFCZSHAPEPROFILEDEF" => z_shape(args)?,
        "IFCCSHAPEPROFILEDEF" => c_shape(args)?,
        "IFCROUNDEDRECTANGLEPROFILEDEF" => rounded_rectangle(args)?,
        "IFCTRAPEZIUMPROFILEDEF" => trapezium(args)?,
        _ => return Ok(None),
    };
    Ok(Some(ring))
}

/// `IfcIShapeProfileDef(…, OverallWidth, OverallDepth, WebThickness,
/// FlangeThickness, FilletRadius, FlangeEdgeRadius, FlangeSlope)` —
/// digest §1. Doubly symmetric: web centred on both axes, equal
/// flanges. `FilletRadius` rounds the four web/flange corners,
/// `FlangeEdgeRadius` the four flange-tip inner corners. With a
/// `FlangeSlope` the flange inner faces tilt about the `b/4` station
/// (thicker toward the web).
fn i_shape(step: &StepFile, args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let b = positive(args, 3)?;
    let h = positive(args, 4)?;
    let tw = positive(args, 5)?;
    let tf = positive(args, 6)?;
    let r1 = radius(args, 7)?;
    let r2 = radius(args, 8)?;
    let k = slope_tan(step, args, 9)?;
    // WHERE ValidWebThickness / ValidFlangeThickness / ValidFilletRadius.
    if tw >= b || 2.0 * tf >= h || r1 > (b - tw) / 2.0 || r1 > (h - 2.0 * tf) / 2.0 {
        return Err(GeometryError::BadProfile);
    }
    // Inner-face height at the web (x = tw/2) and at the tip (x = b/2):
    // thickness t(x) = tf + (b/4 − x)·tan α.
    let y_w = h / 2.0 - (tf + (b / 4.0 - tw / 2.0) * k);
    let y_t = h / 2.0 - (tf + (b / 4.0 - b / 2.0) * k);
    if y_w <= 0.0 || y_t >= h / 2.0 || y_t <= 0.0 {
        return Err(GeometryError::BadProfile);
    }
    let (hb, hh, hw) = (b / 2.0, h / 2.0, tw / 2.0);
    round_corners(&[
        corner(-hb, -hh, 0.0),
        corner(hb, -hh, 0.0),
        corner(hb, -y_t, r2),
        corner(hw, -y_w, r1),
        corner(hw, y_w, r1),
        corner(hb, y_t, r2),
        corner(hb, hh, 0.0),
        corner(-hb, hh, 0.0),
        corner(-hb, y_t, r2),
        corner(-hw, y_w, r1),
        corner(-hw, -y_w, r1),
        corner(-hb, -y_t, r2),
    ])
}

/// `IfcAsymmetricIShapeProfileDef(…, BottomFlangeWidth, OverallDepth,
/// WebThickness, BottomFlangeThickness, BottomFlangeFilletRadius,
/// TopFlangeWidth, TopFlangeThickness, TopFlangeFilletRadius,
/// BottomFlangeEdgeRadius, BottomFlangeSlope, TopFlangeEdgeRadius,
/// TopFlangeSlope)` (IFC4 / 4.3 attribute order) — the I section with
/// unequal flanges. Both flanges are centred on the web (the profile
/// is symmetric about the y axis) and the contour is built in the
/// bounding-box-centred frame of digest §0: width
/// `max(BottomFlangeWidth, TopFlangeWidth)`, height `OverallDepth`.
/// An omitted `TopFlangeThickness` reads as the bottom thickness
/// (the entity page is not staged; this is the natural "equal unless
/// stated" reading and is recorded here). Flange slopes tilt each
/// flange's inner face about its own `width/4` station as for the
/// symmetric I. The IFC 2x3 form (a subtype of `IfcIShapeProfileDef`
/// carrying `OverallWidth, OverallDepth, WebThickness, FlangeThickness,
/// FilletRadius, TopFlangeWidth, TopFlangeThickness,
/// TopFlangeFilletRadius, CentreOfGravityInY`) is recognised by its
/// 12-attribute layout: the base I attributes describe the bottom
/// flange, `CentreOfGravityInY` is informational and ignored.
fn asymmetric_i_shape(step: &StepFile, args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let ifc2x3 = args.len() == 12;
    let bb = positive(args, 3)?;
    let h = positive(args, 4)?;
    let tw = positive(args, 5)?;
    let tb = positive(args, 6)?;
    let r_bf = radius(args, 7)?;
    let bt = positive(args, 8)?;
    let tt = match args.get(9).and_then(number) {
        None => tb,
        Some(v) if v > 0.0 && v.is_finite() => v,
        Some(_) => return Err(GeometryError::BadProfile),
    };
    let r_tf = radius(args, 10)?;
    let (r_be, kb, r_te, kt) = if ifc2x3 {
        (0.0, 0.0, 0.0, 0.0)
    } else {
        (
            radius(args, 11)?,
            slope_tan(step, args, 12)?,
            radius(args, 13)?,
            slope_tan(step, args, 14)?,
        )
    };
    // WHERE ValidWebThickness / ValidFlangeThickness /
    // ValidBottomFilletRadius / ValidTopFilletRadius.
    if tw >= bb || tw >= bt || tb + tt >= h || r_bf > (bb - tw) / 2.0 || r_tf > (bt - tw) / 2.0 {
        return Err(GeometryError::BadProfile);
    }
    let hh = h / 2.0;
    // Inner-face heights at the web and at each flange tip: thickness
    // t(x) = t + (width/4 − x)·tan α measured from the outer face.
    let yb_web = -hh + (tb + (bb / 4.0 - tw / 2.0) * kb);
    let yb_tip = -hh + (tb + (bb / 4.0 - bb / 2.0) * kb);
    let yt_web = hh - (tt + (bt / 4.0 - tw / 2.0) * kt);
    let yt_tip = hh - (tt + (bt / 4.0 - bt / 2.0) * kt);
    if yb_web >= yt_web || yb_tip <= -hh || yt_tip >= hh || yb_tip >= yt_tip {
        return Err(GeometryError::BadProfile);
    }
    let (hb, ht, hw) = (bb / 2.0, bt / 2.0, tw / 2.0);
    round_corners(&[
        corner(-hb, -hh, 0.0),
        corner(hb, -hh, 0.0),
        corner(hb, yb_tip, r_be),
        corner(hw, yb_web, r_bf),
        corner(hw, yt_web, r_tf),
        corner(ht, yt_tip, r_te),
        corner(ht, hh, 0.0),
        corner(-ht, hh, 0.0),
        corner(-ht, yt_tip, r_te),
        corner(-hw, yt_web, r_tf),
        corner(-hw, yb_web, r_bf),
        corner(-hb, yb_tip, r_be),
    ])
}

/// `IfcLShapeProfileDef(…, Depth, Width, Thickness, FilletRadius,
/// EdgeRadius, LegSlope)` — digest §2. The vertical leg (length `Depth`)
/// lies along −x, the horizontal leg (length `Width`, defaulting to
/// `Depth` when the optional attribute is omitted — the legs are then
/// equal) along −y, so the outer corner is at `(−b/2, −h/2)`.
/// `FilletRadius` rounds the single re-entrant corner, `EdgeRadius` the
/// two toes. With a `LegSlope` each leg's inner face tilts (thicker at
/// the root); the documentation pins no reference station for the
/// legs, so `Thickness` is taken at the **mid-length of each leg's
/// inner face** — an assumption, moot for the common `LegSlope = 0`.
fn l_shape(step: &StepFile, args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let h = positive(args, 3)?;
    let b = match args.get(4).and_then(number) {
        None => h,
        Some(w) if w > 0.0 && w.is_finite() => w,
        Some(_) => return Err(GeometryError::BadProfile),
    };
    let t = positive(args, 5)?;
    let r1 = radius(args, 6)?;
    let r2 = radius(args, 7)?;
    let k = slope_tan(step, args, 8)?;
    // WHERE ValidThickness.
    if t >= h || t >= b {
        return Err(GeometryError::BadProfile);
    }
    let (hb, hh) = (b / 2.0, h / 2.0);
    // Horizontal leg inner face: y(x) = −h/2 + t + (x_s − x)·k, station
    // x_s at the leg's mid-length; vertical leg: x(y) = −b/2 + t +
    // (y_s − y)·k.
    let x_s = (-hb + t + hb) / 2.0;
    let y_s = (-hh + t + hh) / 2.0;
    let horiz = ([x_s, -hh + t], [1.0, -k]);
    let vert = ([-hb + t, y_s], [-k, 1.0]);
    let inner = intersect_lines(horiz.0, horiz.1, vert.0, vert.1)?;
    let toe_h = [hb, -hh + t + (x_s - hb) * k];
    let toe_v = [-hb + t + (y_s - hh) * k, hh];
    if toe_h[1] <= -hh || toe_v[0] <= -hb || inner[0] >= hb || inner[1] >= hh {
        return Err(GeometryError::BadProfile);
    }
    round_corners(&[
        corner(-hb, -hh, 0.0),
        corner(hb, -hh, 0.0),
        corner(toe_h[0], toe_h[1], r2),
        corner(inner[0], inner[1], r1),
        corner(toe_v[0], toe_v[1], r2),
        corner(-hb, hh, 0.0),
    ])
}

/// `IfcTShapeProfileDef(…, Depth, FlangeWidth, WebThickness,
/// FlangeThickness, FilletRadius, FlangeEdgeRadius, WebEdgeRadius,
/// WebSlope, FlangeSlope)` — digest §3. Flange on top, web hanging down
/// centred on the y-axis; `FlangeWidth` is the full bounding-box width.
/// The two slope attributes are read by their **names** (the published
/// descriptions are transposed, digest §3): `FlangeSlope` tilts the
/// flange underside about the `b/4` station, `WebSlope` tilts the web
/// faces (thicker toward the flange) about the web's mid-length — the
/// latter station is not documented and is an assumption.
fn t_shape(step: &StepFile, args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let h = positive(args, 3)?;
    let b = positive(args, 4)?;
    let tw = positive(args, 5)?;
    let tf = positive(args, 6)?;
    let r1 = radius(args, 7)?;
    let r2 = radius(args, 8)?;
    let r3 = radius(args, 9)?;
    let kw = slope_tan(step, args, 10)?;
    let kf = slope_tan(step, args, 11)?;
    // WHERE ValidFlangeThickness / ValidWebThickness.
    if tf >= h || tw >= b {
        return Err(GeometryError::BadProfile);
    }
    let (hb, hh, hw) = (b / 2.0, h / 2.0, tw / 2.0);
    // Flange underside (right half): y(x) = h/2 − tf − (b/4 − x)·kf.
    let y_tip = hh - tf - (hb / 2.0 - hb) * kf;
    // Web right face: x(y) = tw/2 + (y − y_s)·kw (thicker toward the
    // flange), y_s at the web's mid-length between its free end and
    // the flange underside.
    let y_s = (-hh + (hh - tf)) / 2.0;
    let x_end = hw + (-hh - y_s) * kw;
    let flange = ([hb / 2.0, hh - tf], [1.0, kf]);
    let web = ([hw, y_s], [kw, 1.0]);
    let inner = intersect_lines(flange.0, flange.1, web.0, web.1)?;
    if y_tip >= hh || y_tip <= -hh || x_end <= 0.0 || inner[0] >= hb || inner[1] <= -hh {
        return Err(GeometryError::BadProfile);
    }
    round_corners(&[
        corner(-x_end, -hh, r3),
        corner(x_end, -hh, r3),
        corner(inner[0], inner[1], r1),
        corner(hb, y_tip, r2),
        corner(hb, hh, 0.0),
        corner(-hb, hh, 0.0),
        corner(-hb, y_tip, r2),
        corner(-inner[0], inner[1], r1),
    ])
}

/// `IfcUShapeProfileDef(…, Depth, FlangeWidth, WebThickness,
/// FlangeThickness, FilletRadius, EdgeRadius, FlangeSlope)` — digest §4.
/// Web on the left, both flanges extending to +x; `FlangeWidth` is the
/// full bounding-box width. With a `FlangeSlope` the flange inner faces
/// tilt (thicker toward the web) about the station `FlangeWidth/4` from
/// the web's outer face — the digest's inference from the T-shape.
fn u_shape(step: &StepFile, args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let h = positive(args, 3)?;
    let b = positive(args, 4)?;
    let tw = positive(args, 5)?;
    let tf = positive(args, 6)?;
    let r1 = radius(args, 7)?;
    let r2 = radius(args, 8)?;
    let k = slope_tan(step, args, 9)?;
    // WHERE ValidFlangeThickness / ValidWebThickness.
    if tf >= h / 2.0 || tw >= b {
        return Err(GeometryError::BadProfile);
    }
    let (hb, hh) = (b / 2.0, h / 2.0);
    let x_s = -hb + b / 4.0;
    // t(x) = tf + (x_s − x)·k.
    let y_w = hh - (tf + (x_s - (-hb + tw)) * k);
    let y_t = hh - (tf + (x_s - hb) * k);
    if y_w <= 0.0 || y_t >= hh || y_t <= 0.0 {
        return Err(GeometryError::BadProfile);
    }
    round_corners(&[
        corner(-hb, -hh, 0.0),
        corner(hb, -hh, 0.0),
        corner(hb, -y_t, r2),
        corner(-hb + tw, -y_w, r1),
        corner(-hb + tw, y_w, r1),
        corner(hb, y_t, r2),
        corner(hb, hh, 0.0),
        corner(-hb, hh, 0.0),
    ])
}

/// `IfcZShapeProfileDef(…, Depth, FlangeWidth, WebThickness,
/// FlangeThickness, FilletRadius, EdgeRadius)` — digest §5.
/// Point-symmetric about the origin; `FlangeWidth` runs from a flange
/// tip to the far face of the web, so the overall width is
/// `2·FlangeWidth − WebThickness`. The lower flange runs to +x, the
/// upper to −x. A `WebThickness ≥ FlangeWidth` (no WHERE rule forbids
/// it) would invert the flanges and is rejected.
fn z_shape(args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let h = positive(args, 3)?;
    let b = positive(args, 4)?;
    let tw = positive(args, 5)?;
    let tf = positive(args, 6)?;
    let r1 = radius(args, 7)?;
    let r2 = radius(args, 8)?;
    if tf >= h / 2.0 || tw >= b {
        return Err(GeometryError::BadProfile);
    }
    let w = 2.0 * b - tw;
    let (x_l, x_r, hh, hw) = (-w / 2.0, w / 2.0, h / 2.0, tw / 2.0);
    round_corners(&[
        corner(-hw, -hh, 0.0),
        corner(x_r, -hh, 0.0),
        corner(x_r, -hh + tf, r2),
        corner(hw, -hh + tf, r1),
        corner(hw, hh, 0.0),
        corner(x_l, hh, 0.0),
        corner(x_l, hh - tf, r2),
        corner(-hw, hh - tf, r1),
    ])
}

/// `IfcCShapeProfileDef(…, Depth, Width, WallThickness, Girth,
/// InternalFilletRadius)` — digest §6. A constant-thickness cold-formed
/// channel: web on the left, top and bottom walls to +x, each ending in
/// a lip returning inward of length `Girth` (measured from the wall's
/// outer face). `InternalFilletRadius` rounds the four internal corners
/// of the cavity; the outer corners stay sharp (the outer bend radius is
/// not an attribute and the page does not state it, digest §6).
fn c_shape(args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let h = positive(args, 3)?;
    let b = positive(args, 4)?;
    let t = positive(args, 5)?;
    let c = positive(args, 6)?;
    let r1 = radius(args, 7)?;
    // WHERE ValidGirth / ValidWallThickness / ValidInternalFilletRadius.
    if c >= h / 2.0 || t >= b / 2.0 || t >= h / 2.0 || r1 > b / 2.0 - t || r1 > h / 2.0 - t {
        return Err(GeometryError::BadProfile);
    }
    // The lip must reach below the wall it returns from.
    if c <= t {
        return Err(GeometryError::BadProfile);
    }
    let (hb, hh) = (b / 2.0, h / 2.0);
    round_corners(&[
        corner(-hb, -hh, 0.0),
        corner(hb, -hh, 0.0),
        corner(hb, -hh + c, 0.0),
        corner(hb - t, -hh + c, 0.0),
        corner(hb - t, -hh + t, r1),
        corner(-hb + t, -hh + t, r1),
        corner(-hb + t, hh - t, r1),
        corner(hb - t, hh - t, r1),
        corner(hb - t, hh - c, 0.0),
        corner(hb, hh - c, 0.0),
        corner(hb, hh, 0.0),
        corner(-hb, hh, 0.0),
    ])
}

/// `IfcRoundedRectangleProfileDef(…, XDim, YDim, RoundingRadius)` —
/// digest §7. All four corners rounded by the mandatory, strictly
/// positive `RoundingRadius` (WHERE `ValidRadius`: at most half of
/// either dimension — equality is legal and degenerates to a stadium or
/// a circle).
fn rounded_rectangle(args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let x = positive(args, 3)?;
    let y = positive(args, 4)?;
    let r = positive(args, 5)?;
    if r > x / 2.0 * (1.0 + 1e-12) || r > y / 2.0 * (1.0 + 1e-12) {
        return Err(GeometryError::BadProfile);
    }
    let r = r.min(x / 2.0).min(y / 2.0);
    let (hx, hy) = (x / 2.0, y / 2.0);
    round_corners(&[
        corner(-hx, -hy, r),
        corner(hx, -hy, r),
        corner(hx, hy, r),
        corner(-hx, hy, r),
    ])
}

/// `IfcTrapeziumProfileDef(…, BottomXDim, TopXDim, YDim, TopXOffset)` —
/// digest §8 (deprecated in IFC 4.3, still read). The bottom edge is
/// centred on the origin; the top edge starts `TopXOffset` (signed)
/// right of the bottom-left corner and runs `TopXDim` along +x. The
/// origin is the centre of the `BottomXDim × YDim` rectangle even when
/// the top overhangs it.
fn trapezium(args: &[Value]) -> Result<Vec<[f64; 2]>, GeometryError> {
    let bottom = positive(args, 3)?;
    let top = positive(args, 4)?;
    let y = positive(args, 5)?;
    let off = signed(args, 6)?;
    let (hb, hy) = (bottom / 2.0, y / 2.0);
    Ok(vec![
        [-hb, -hy],
        [hb, -hy],
        [-hb + off + top, hy],
        [-hb + off, hy],
    ])
}

#[cfg(test)]
mod tests {
    use super::super::{tessellate_item, GeometryError, TriMesh};
    use crate::parser::{parse_step, StepFile};

    fn parse(data: &str) -> StepFile {
        let text = format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t.ifc','2026-08-30T00:00:00',('a'),('o'),'p','s','auth');\n\
             FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        );
        parse_step(text.as_bytes()).expect("parse failed")
    }

    /// Extrude `profile` (already declared as #1) by 1 along +z and
    /// return the mesh; its volume equals the profile area.
    fn extrude(profile: &str) -> TriMesh {
        let f = parse(&format!(
            "{profile}\n#2=IFCDIRECTION((0.,0.,1.));\n#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);"
        ));
        tessellate_item(&f, 3).expect("tessellate")
    }

    fn bbox(m: &TriMesh) -> ([f64; 2], [f64; 2]) {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for p in &m.positions {
            for a in 0..2 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
        (lo, hi)
    }

    fn assert_close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} vs {b} (tol {tol})");
    }

    fn assert_centred(m: &TriMesh, w: f64, h: f64) {
        let (lo, hi) = bbox(m);
        assert_close(lo[0], -w / 2.0, 1e-9);
        assert_close(hi[0], w / 2.0, 1e-9);
        assert_close(lo[1], -h / 2.0, 1e-9);
        assert_close(hi[1], h / 2.0, 1e-9);
    }

    #[test]
    fn i_shape_sharp_area_and_bbox() {
        // b=100, h=200, tw=10, tf=15: area = 2·100·15 + 170·10 = 4700.
        let m = extrude("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,$,$,$);");
        assert_close(m.signed_volume(), 4700.0, 1e-9);
        assert_centred(&m, 100.0, 200.0);
        // 12 ring points, no arcs.
        assert_eq!(m.vertex_count(), 24);
    }

    #[test]
    fn i_shape_fillets_add_area_edge_radii_remove_it() {
        // Fillet r1 = 5 at four re-entrant corners each ADDS
        // (1 − π/4)·r² of material; edge radius r2 = 3 at four convex
        // corners each REMOVES (1 − π/4)·r².
        let m = extrude("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,5.,3.,$);");
        let k = 1.0 - core::f64::consts::FRAC_PI_4;
        let expect = 4700.0 + 4.0 * k * 25.0 - 4.0 * k * 9.0;
        // Arcs are polygonal (48 segments per turn): tolerance ~0.1%.
        assert_close(m.signed_volume(), expect, expect * 2e-3);
        assert_centred(&m, 100.0, 200.0);
    }

    #[test]
    fn i_shape_zero_radii_equal_omitted() {
        let a = extrude("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,0.,0.,0.);");
        let b = extrude("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,$,$,$);");
        assert_eq!(a.positions, b.positions);
    }

    #[test]
    fn i_shape_flange_slope_keeps_area_at_quarter_station() {
        // With the thickness pinned at b/4 the sloped flange's mean
        // thickness over the outstand (tw/2 … b/2) differs from tf by
        // (b/4 − (b/2 + tw/2)/2)·tan α per side. Slope 0.1 rad, b=100,
        // tw=10: station offset = 25 − 27.5 = −2.5 → mean thickness =
        // 15 − 0.2508; outstand width 45, 4 outstands.
        let m = extrude("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,$,$,0.1);");
        let k = (0.1f64).tan();
        let outstand = 4.0 * 45.0 * (15.0 - 2.5 * k);
        let web = 10.0 * 200.0;
        assert_close(m.signed_volume(), outstand + web, 1e-9);
        assert_centred(&m, 100.0, 200.0);
    }

    #[test]
    fn i_shape_where_rules_reject() {
        // Web wider than the profile; flanges thicker than the depth;
        // fillet radius over the web/flange clearance.
        for bad in [
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,100.,15.,$,$,$);",
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,100.,$,$,$);",
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,50.,$,$);",
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,-1.,$,$);",
        ] {
            let f = parse(&format!(
                "{bad}\n#2=IFCDIRECTION((0.,0.,1.));\n#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);"
            ));
            assert_eq!(
                tessellate_item(&f, 3).unwrap_err(),
                GeometryError::BadProfile
            );
        }
    }

    #[test]
    fn asymmetric_i_shape_area_bbox_and_2x3_form() {
        // Bottom flange 120 × 15, top flange 80 × 10, web 8 thick,
        // depth 200: A = 120·15 + 80·10 + 8·175 = 4000, bbox 120 × 200
        // centred.
        let m = extrude(
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,$,80.,10.,$,$,$,$,$);",
        );
        assert_close(m.signed_volume(), 4000.0, 1e-9);
        assert_centred(&m, 120.0, 200.0);
        // Omitted top thickness reads as the bottom thickness.
        let eq = extrude(
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,$,80.,$,$,$,$,$,$);",
        );
        assert_close(
            eq.signed_volume(),
            120.0 * 15.0 + 80.0 * 15.0 + 8.0 * 170.0,
            1e-9,
        );
        // A wider top flange widens the bounding box.
        let top = extrude(
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,80.,200.,8.,10.,$,120.,15.,$,$,$,$,$);",
        );
        assert_close(top.signed_volume(), 4000.0, 1e-9);
        assert_centred(&top, 120.0, 200.0);
        // Fillets add material, edge radii remove it.
        let f = extrude(
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,6.,80.,10.,4.,$,$,$,$);",
        );
        assert!(f.signed_volume() > 4000.0);
        let e = extrude(
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,$,80.,10.,$,3.,$,3.,$);",
        );
        assert!(e.signed_volume() < 4000.0);
        // IFC 2x3 layout (12 attributes): the I base is the bottom flange.
        let old =
            extrude("#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,$,80.,10.,$,$);");
        assert_close(old.signed_volume(), 4000.0, 1e-9);
        assert_centred(&old, 120.0, 200.0);
    }

    #[test]
    fn asymmetric_i_shape_where_rules_reject() {
        for src in [
            // Web wider than the top flange.
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,90.,15.,$,80.,10.,$,$,$,$,$);",
            // Flanges thicker than the depth.
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,150.,$,80.,60.,$,$,$,$,$);",
            // Bottom fillet past the half flange overhang.
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,60.,80.,10.,$,$,$,$,$);",
            // Top fillet past the half flange overhang.
            "#1=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,8.,15.,$,80.,10.,40.,$,$,$,$);",
        ] {
            let f = parse(&format!(
                "{src}\n#2=IFCDIRECTION((0.,0.,1.));\n#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);"
            ));
            assert_eq!(
                crate::geometry::tessellate_item(&f, 3).unwrap_err(),
                GeometryError::BadProfile,
                "{src}"
            );
        }
    }

    #[test]
    fn l_shape_layout_and_area() {
        // h=100 (y), b=60 (x), t=8: area = 60·8 + (100−8)·8 = 1216;
        // outer corner at (−30, −50).
        let m = extrude("#1=IFCLSHAPEPROFILEDEF(.AREA.,$,$,100.,60.,8.,$,$,$);");
        assert_close(m.signed_volume(), 1216.0, 1e-9);
        assert_centred(&m, 60.0, 100.0);
        // The vertical leg is on the LEFT: material at x=−30..−22 for all y.
        assert!(m
            .positions
            .iter()
            .any(|p| (p[0] + 22.0).abs() < 1e-9 && (p[1] - 50.0).abs() < 1e-9));
        // Width omitted → equal legs of Depth.
        let e = extrude("#1=IFCLSHAPEPROFILEDEF(.AREA.,$,$,100.,$,8.,$,$,$);");
        assert_close(e.signed_volume(), 100.0 * 8.0 + 92.0 * 8.0, 1e-9);
        assert_centred(&e, 100.0, 100.0);
    }

    #[test]
    fn l_shape_fillet_and_edge_radii() {
        let m = extrude("#1=IFCLSHAPEPROFILEDEF(.AREA.,$,$,100.,60.,8.,6.,4.,$);");
        let k = 1.0 - core::f64::consts::FRAC_PI_4;
        let expect = 1216.0 + k * 36.0 - 2.0 * k * 16.0;
        assert_close(m.signed_volume(), expect, expect * 2e-3);
    }

    #[test]
    fn t_shape_flange_on_top_web_down() {
        // h=120, b=80, tw=10, tf=12: area = 80·12 + 108·10 = 2040.
        let m = extrude("#1=IFCTSHAPEPROFILEDEF(.AREA.,$,$,120.,80.,10.,12.,$,$,$,$,$);");
        assert_close(m.signed_volume(), 2040.0, 1e-9);
        assert_centred(&m, 80.0, 120.0);
        // Flange occupies y ∈ [48, 60] across the full width.
        assert!(m
            .positions
            .iter()
            .any(|p| (p[0] - 40.0).abs() < 1e-9 && (p[1] - 48.0).abs() < 1e-9));
        // Web end at y = −60 with x = ±5.
        assert!(m
            .positions
            .iter()
            .any(|p| (p[0] - 5.0).abs() < 1e-9 && (p[1] + 60.0).abs() < 1e-9));
    }

    #[test]
    fn t_shape_all_three_radii() {
        let m = extrude("#1=IFCTSHAPEPROFILEDEF(.AREA.,$,$,120.,80.,10.,12.,4.,3.,2.,$,$);");
        let k = 1.0 - core::f64::consts::FRAC_PI_4;
        let expect = 2040.0 + 2.0 * k * 16.0 - 2.0 * k * 9.0 - 2.0 * k * 4.0;
        assert_close(m.signed_volume(), expect, expect * 2e-3);
    }

    #[test]
    fn t_shape_slopes_read_by_name() {
        // FlangeSlope (index 11) tilts the flange underside: the area
        // changes by the b/4-station rule; WebSlope (index 10) tilts the
        // web faces about the web mid-length, so the web area is
        // unchanged to first order and the profile stays within bbox.
        let flange = extrude("#1=IFCTSHAPEPROFILEDEF(.AREA.,$,$,120.,80.,10.,12.,$,$,$,$,0.1);");
        let k = (0.1f64).tan();
        // Right outstand: x from 5 to 40, thickness 12 + (20 − x)k;
        // mean thickness = 12 + (20 − 22.5)k over width 35, two sides.
        let expect = 2.0 * 35.0 * (12.0 - 2.5 * k) + 10.0 * 120.0;
        assert_close(flange.signed_volume(), expect, 1e-9);
        assert_centred(&flange, 80.0, 120.0);

        let web = extrude("#1=IFCTSHAPEPROFILEDEF(.AREA.,$,$,120.,80.,10.,12.,$,$,$,0.02,$);");
        assert_centred(&web, 80.0, 120.0);
        // Web thicker at the flange than at the free end.
        let (lo, _) = bbox(&web);
        let end_x = web
            .positions
            .iter()
            .filter(|p| (p[1] - lo[1]).abs() < 1e-9 && p[2] == 0.0)
            .map(|p| p[0])
            .fold(0.0f64, f64::max);
        assert!(end_x < 5.0, "web end half-width {end_x} should be under 5");
    }

    #[test]
    fn u_shape_web_left_flanges_right() {
        // h=100, b=50, tw=6, tf=8: area = 2·50·8 + 84·6 = 1304.
        let m = extrude("#1=IFCUSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,6.,8.,$,$,$);");
        assert_close(m.signed_volume(), 1304.0, 1e-9);
        assert_centred(&m, 50.0, 100.0);
        // Inner web face at x = −25 + 6 = −19.
        assert!(m
            .positions
            .iter()
            .any(|p| (p[0] + 19.0).abs() < 1e-9 && (p[1] - 42.0).abs() < 1e-9));
        let r = extrude("#1=IFCUSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,6.,8.,3.,2.,$);");
        let k = 1.0 - core::f64::consts::FRAC_PI_4;
        let expect = 1304.0 + 2.0 * k * 9.0 - 2.0 * k * 4.0;
        assert_close(r.signed_volume(), expect, expect * 2e-3);
    }

    #[test]
    fn z_shape_overall_width_and_point_symmetry() {
        // h=100, b=40, tw=6, tf=8: overall width 2·40 − 6 = 74;
        // area = 2·40·8 + (100 − 16)·6 = 1144.
        let m = extrude("#1=IFCZSHAPEPROFILEDEF(.AREA.,$,$,100.,40.,6.,8.,$,$);");
        assert_close(m.signed_volume(), 1144.0, 1e-9);
        assert_centred(&m, 74.0, 100.0);
        // Every bottom-ring point maps to a point under (x,y) → (−x,−y).
        let ring: Vec<[f64; 3]> = m
            .positions
            .iter()
            .copied()
            .filter(|p| p[2] == 0.0)
            .collect();
        for p in &ring {
            assert!(ring
                .iter()
                .any(|q| (q[0] + p[0]).abs() < 1e-9 && (q[1] + p[1]).abs() < 1e-9));
        }
        // Lower flange to +x: (37, −50) is a vertex; upper flange to −x.
        assert!(ring
            .iter()
            .any(|p| (p[0] - 37.0).abs() < 1e-9 && (p[1] + 50.0).abs() < 1e-9));
        assert!(ring
            .iter()
            .any(|p| (p[0] + 37.0).abs() < 1e-9 && (p[1] - 50.0).abs() < 1e-9));
        // tw ≥ b inverts the flanges → rejected.
        let f = parse(
            "#1=IFCZSHAPEPROFILEDEF(.AREA.,$,$,100.,40.,40.,8.,$,$);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);",
        );
        assert_eq!(
            tessellate_item(&f, 3).unwrap_err(),
            GeometryError::BadProfile
        );
    }

    #[test]
    fn c_shape_lips_return_inward() {
        // h=100, b=50, t=2, c=15: web 100·2 + two walls 48·2 + two lips
        // (15 − 2)·2 = 200 + 192 + 52 = 444.
        let m = extrude("#1=IFCCSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,2.,15.,$);");
        assert_close(m.signed_volume(), 444.0, 1e-9);
        assert_centred(&m, 50.0, 100.0);
        // Lip free end at y = −50 + 15 = −35 on x ∈ [48 − 2, 25]... i.e.
        // vertices (25, −35) and (23, −35).
        assert!(m
            .positions
            .iter()
            .any(|p| (p[0] - 23.0).abs() < 1e-9 && (p[1] + 35.0).abs() < 1e-9));
        // Internal fillet adds material at four re-entrant corners.
        let r = extrude("#1=IFCCSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,2.,15.,3.);");
        let k = 1.0 - core::f64::consts::FRAC_PI_4;
        let expect = 444.0 + 4.0 * k * 9.0;
        assert_close(r.signed_volume(), expect, expect * 2e-3);
    }

    #[test]
    fn rounded_rectangle_area_stadium_and_circle() {
        let k = 1.0 - core::f64::consts::FRAC_PI_4;
        let m = extrude("#1=IFCROUNDEDRECTANGLEPROFILEDEF(.AREA.,$,$,40.,20.,5.);");
        let expect = 800.0 - 4.0 * k * 25.0;
        assert_close(m.signed_volume(), expect, expect * 2e-3);
        assert_centred(&m, 40.0, 20.0);
        // r = YDim/2 → stadium: 20·20 + π·10².
        let s = extrude("#1=IFCROUNDEDRECTANGLEPROFILEDEF(.AREA.,$,$,40.,20.,10.);");
        let expect = 400.0 + core::f64::consts::PI * 100.0;
        assert_close(s.signed_volume(), expect, expect * 3e-3);
        // r = XDim/2 = YDim/2 → a circle of radius 10.
        let c = extrude("#1=IFCROUNDEDRECTANGLEPROFILEDEF(.AREA.,$,$,20.,20.,10.);");
        let expect = core::f64::consts::PI * 100.0;
        assert_close(c.signed_volume(), expect, expect * 3e-3);
        assert_centred(&c, 20.0, 20.0);
        // Over-large radius → rejected.
        let f = parse(
            "#1=IFCROUNDEDRECTANGLEPROFILEDEF(.AREA.,$,$,40.,20.,11.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);",
        );
        assert_eq!(
            tessellate_item(&f, 3).unwrap_err(),
            GeometryError::BadProfile
        );
    }

    #[test]
    fn trapezium_offset_and_overhang() {
        // Bottom 3, top 2, height 1, offset 0.5: area = (3+2)/2·1 = 2.5,
        // top edge x ∈ [−1.5+0.5, −1.5+0.5+2] = [−1, 1].
        let m = extrude("#1=IFCTRAPEZIUMPROFILEDEF(.AREA.,$,$,3.,2.,1.,0.5);");
        assert_close(m.signed_volume(), 2.5, 1e-9);
        assert_centred(&m, 3.0, 1.0);
        assert!(m
            .positions
            .iter()
            .any(|p| (p[0] - 1.0).abs() < 1e-9 && (p[1] - 0.5).abs() < 1e-9));
        // Negative offset overhangs the bottom; the origin stays on the
        // BottomXDim × YDim rectangle (bbox is NOT re-centred).
        let o = extrude("#1=IFCTRAPEZIUMPROFILEDEF(.AREA.,$,$,3.,2.,1.,-1.);");
        assert_close(o.signed_volume(), 2.5, 1e-9);
        let (lo, hi) = bbox(&o);
        assert_close(lo[0], -2.5, 1e-9);
        assert_close(hi[0], 1.5, 1e-9);
    }

    #[test]
    fn named_profile_honours_position() {
        // Position moves the I-section's bbox centre to (100, 50).
        let f = parse(
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,#4,100.,200.,10.,15.,$,$,$);\n\
             #4=IFCAXIS2PLACEMENT2D(#5,$);\n#5=IFCCARTESIANPOINT((100.,50.));\n\
             #2=IFCDIRECTION((0.,0.,1.));\n#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        let (lo, hi) = bbox(&m);
        assert_close((lo[0] + hi[0]) / 2.0, 100.0, 1e-9);
        assert_close((lo[1] + hi[1]) / 2.0, 50.0, 1e-9);
        assert_close(m.signed_volume(), 4700.0, 1e-9);
    }

    #[test]
    fn typed_measure_wrappers_accepted() {
        let m = extrude(
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,IFCPOSITIVELENGTHMEASURE(100.),\
             IFCPOSITIVELENGTHMEASURE(200.),10.,15.,$,$,$);",
        );
        assert_close(m.signed_volume(), 4700.0, 1e-9);
    }

    #[test]
    fn named_profile_revolves() {
        // A U-section revolved a full turn about a y-parallel axis 100
        // away: volume = area × 2π × (distance of centroid) — Pappus.
        // Area 1304, centroid x for the U (web left): compute from parts:
        // web 6×100 at x = −22; flanges 2×(44×8) at x = 3.
        let f = parse(
            "#1=IFCUSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,6.,8.,$,$,$);\n\
             #2=IFCAXIS1PLACEMENT(#4,#5);\n#4=IFCCARTESIANPOINT((-100.,0.,0.));\n\
             #5=IFCDIRECTION((0.,1.,0.));\n\
             #3=IFCREVOLVEDAREASOLID(#1,$,#2,6.283185307179586);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        let area = 1304.0;
        let cx = (600.0 * -22.0 + 704.0 * 3.0) / area;
        let expect = area * 2.0 * core::f64::consts::PI * (cx + 100.0);
        assert_close(m.signed_volume().abs(), expect, expect * 5e-3);
    }
}
