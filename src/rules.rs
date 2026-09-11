//! EXPRESS WHERE-rule validation for the geometry entities the swept
//! solid / profile / georeferencing slices consume — each rule is a
//! transcription of the staged `IFC4X3_ADD2.exp` declaration (the IFC4
//! text is identical for these entities), evaluated on the positional
//! attributes so it works with or without the typed schema slice.
//!
//! The geometry extractor already rejects rule-violating profiles with
//! `BadProfile`; this module names *which* rule failed so a caller can
//! report it, and covers rules the extractor tolerates (a `CURVE`-typed
//! profile under a swept solid, a composite of mixed profile types, a
//! revolution axis off the XY plane).

use crate::parser::StepFile;
use crate::value::Value;

/// One failed WHERE rule on one instance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuleViolation {
    /// The `#id` of the instance.
    pub id: u64,
    /// The instance's entity keyword (`IFCISHAPEPROFILEDEF`).
    pub entity: String,
    /// The EXPRESS rule label (`ValidFilletRadius`).
    pub rule: &'static str,
}

/// Numeric attribute, unwrapping a typed measure wrapper.
fn num(args: &[Value], index: usize) -> Option<f64> {
    match args.get(index)? {
        Value::Typed { args, .. } => args.first().and_then(Value::as_number),
        other => other.as_number(),
    }
}

/// Integer attribute value, unwrapping a typed wrapper.
fn int(v: &Value) -> Option<i64> {
    match v {
        Value::Typed { args, .. } => args.first().and_then(Value::as_integer),
        other => other.as_integer(),
    }
}

/// Run one rule body in its own `?` scope.
fn evaluate(body: impl FnOnce() -> Option<bool>) -> Option<bool> {
    body()
}

fn keyword_of(step: &StepFile, id: Option<u64>) -> Option<&str> {
    step.get(id?).map(|inst| inst.keyword.as_str())
}

/// Evaluate the WHERE rules transcribed for `id`'s entity.
///
/// Returns `None` when the entity carries no transcribed rules (not
/// "valid" — merely unchecked), else the (possibly empty) list of
/// violated rules. Attributes that are missing or non-numeric where a
/// rule needs them count as violations of that rule.
pub fn where_rule_violations(step: &StepFile, id: u64) -> Option<Vec<RuleViolation>> {
    let inst = step.get(id)?;
    let a = &inst.args;
    let mut failed: Vec<&'static str> = Vec::new();
    let mut check = |rule: &'static str, ok: Option<bool>| {
        if ok != Some(true) {
            failed.push(rule);
        }
    };
    // `rule!(label, expr)` evaluates `expr` (which may use `?` on the
    // Option-valued attributes) inside its own closure, so a missing
    // attribute fails just that rule instead of aborting the walk.
    macro_rules! rule {
        ($label:expr, $body:expr $(,)?) => {
            check($label, evaluate(|| Some($body)))
        };
    }
    match inst.keyword.as_str() {
        // ---- Parameterised profiles (attrs from index 3) ----
        "IFCISHAPEPROFILEDEF" => {
            let (b, h, tw, tf) = (num(a, 3), num(a, 4), num(a, 5), num(a, 6));
            rule!(
                "ValidFilletRadius",
                match num(a, 7) {
                    None => true,
                    Some(r) => r <= (b? - tw?) / 2.0 && r <= (h? - 2.0 * tf?) / 2.0,
                },
            );
            rule!("ValidFlangeThickness", 2.0 * tf? < h?);
            rule!("ValidWebThickness", tw? < b?);
        }
        "IFCASYMMETRICISHAPEPROFILEDEF" if a.len() >= 15 => {
            // (…, BottomFlangeWidth, OverallDepth, WebThickness,
            // BottomFlangeThickness, BottomFlangeFilletRadius,
            // TopFlangeWidth, TopFlangeThickness, TopFlangeFilletRadius, …).
            let (bb, h, tw, tb, bt) = (num(a, 3), num(a, 4), num(a, 5), num(a, 6), num(a, 8));
            rule!(
                "ValidBottomFilletRadius",
                match num(a, 7) {
                    None => true,
                    Some(r) => r <= (bb? - tw?) / 2.0,
                },
            );
            rule!(
                "ValidFlangeThickness",
                match num(a, 9) {
                    None => true,
                    Some(tt) => tb? + tt < h?,
                },
            );
            rule!(
                "ValidTopFilletRadius",
                match num(a, 10) {
                    None => true,
                    Some(r) => r <= (bt? - tw?) / 2.0,
                },
            );
            rule!("ValidWebThickness", tw? < bb? && tw? < bt?);
        }
        "IFCLSHAPEPROFILEDEF" => {
            let (h, w, t) = (num(a, 3), num(a, 4), num(a, 5));
            rule!(
                "ValidThickness",
                t? < h?
                    && match w {
                        None => true,
                        Some(w) => t? < w,
                    }
            );
        }
        "IFCTSHAPEPROFILEDEF" => {
            let (h, b, tw, tf) = (num(a, 3), num(a, 4), num(a, 5), num(a, 6));
            rule!("ValidFlangeThickness", tf? < h?);
            rule!("ValidWebThickness", tw? < b?);
        }
        "IFCUSHAPEPROFILEDEF" => {
            let (h, b, tw, tf) = (num(a, 3), num(a, 4), num(a, 5), num(a, 6));
            rule!("ValidFlangeThickness", tf? < h? / 2.0);
            rule!("ValidWebThickness", tw? < b?);
        }
        "IFCZSHAPEPROFILEDEF" => {
            let (h, tf) = (num(a, 3), num(a, 6));
            rule!("ValidFlangeThickness", tf? < h? / 2.0);
        }
        "IFCCSHAPEPROFILEDEF" => {
            let (h, b, t, c) = (num(a, 3), num(a, 4), num(a, 5), num(a, 6));
            rule!("ValidGirth", c? < h? / 2.0);
            rule!(
                "ValidInternalFilletRadius",
                match num(a, 7) {
                    None => true,
                    Some(r) => r <= b? / 2.0 - t? && r <= h? / 2.0 - t?,
                },
            );
            rule!("ValidWallThickness", t? < b? / 2.0 && t? < h? / 2.0);
        }
        "IFCROUNDEDRECTANGLEPROFILEDEF" => {
            let (x, y, r) = (num(a, 3), num(a, 4), num(a, 5));
            rule!("ValidRadius", r? <= x? / 2.0 && r? <= y? / 2.0);
        }
        "IFCRECTANGLEHOLLOWPROFILEDEF" => {
            let (x, y, t) = (num(a, 3), num(a, 4), num(a, 5));
            rule!(
                "ValidInnerRadius",
                match num(a, 6) {
                    None => true,
                    Some(r) => r <= x? / 2.0 - t? && r <= y? / 2.0 - t?,
                },
            );
            rule!(
                "ValidOuterRadius",
                match num(a, 7) {
                    None => true,
                    Some(r) => r <= x? / 2.0 && r <= y? / 2.0,
                },
            );
            rule!("ValidWallThickness", t? < x? / 2.0 && t? < y? / 2.0);
        }
        "IFCCIRCLEHOLLOWPROFILEDEF" => {
            rule!("WR1", num(a, 4)? < num(a, 3)?);
        }
        "IFCCOMPOSITEPROFILEDEF" => {
            // Profiles : SET [2:?] OF IfcProfileDef (index 2).
            let profiles = a.get(2).and_then(Value::as_list);
            let types: Option<Vec<&str>> = profiles.map(|ps| {
                ps.iter()
                    .filter_map(|p| step.get(p.as_reference()?))
                    .map(|i| i.keyword.as_str())
                    .collect()
            });
            let first_type = profiles.and_then(|ps| {
                let p = step.get(ps.first()?.as_reference()?)?;
                p.args.first().and_then(Value::as_enum)
            });
            check(
                "InvariantProfileType",
                profiles.map(|ps| {
                    ps.iter().all(|p| {
                        p.as_reference()
                            .and_then(|pid| step.get(pid))
                            .and_then(|i| i.args.first().and_then(Value::as_enum))
                            == first_type
                    })
                }),
            );
            check(
                "NoRecursion",
                types.map(|ts| ts.iter().all(|t| *t != "IFCCOMPOSITEPROFILEDEF")),
            );
        }
        // ---- Swept solids ----
        "IFCEXTRUDEDAREASOLID"
        | "IFCEXTRUDEDAREASOLIDTAPERED"
        | "IFCREVOLVEDAREASOLID"
        | "IFCREVOLVEDAREASOLIDTAPERED"
        | "IFCSURFACECURVESWEPTAREASOLID"
        | "IFCFIXEDREFERENCESWEPTAREASOLID"
        | "IFCDIRECTRIXDERIVEDREFERENCESWEPTAREASOLID" => {
            // IfcSweptAreaSolid.SweptAreaType: SweptArea.ProfileType = AREA.
            let swept = a.first().and_then(Value::as_reference);
            check(
                "SweptAreaType",
                swept
                    .and_then(|pid| step.get(pid))
                    .map(|p| p.args.first().and_then(Value::as_enum) == Some("AREA")),
            );
            match inst.keyword.as_str() {
                "IFCEXTRUDEDAREASOLID" | "IFCEXTRUDEDAREASOLIDTAPERED" => {
                    // ValidExtrusionDirection: dot((0,0,1), ExtrudedDirection) ≠ 0.
                    let dir = a
                        .get(2)
                        .and_then(Value::as_reference)
                        .and_then(|d| step.get(d))
                        .filter(|d| d.keyword == "IFCDIRECTION")
                        .and_then(|d| d.args.first().and_then(Value::as_list))
                        .and_then(|ratios| ratios.get(2).and_then(Value::as_number));
                    check("ValidExtrusionDirection", dir.map(|z| z != 0.0));
                }
                "IFCREVOLVEDAREASOLID" | "IFCREVOLVEDAREASOLIDTAPERED" => {
                    // Axis : IfcAxis1Placement(Location, Axis) (index 2).
                    let axis = a
                        .get(2)
                        .and_then(Value::as_reference)
                        .and_then(|x| step.get(x))
                        .filter(|x| x.keyword == "IFCAXIS1PLACEMENT");
                    let loc_z = axis
                        .and_then(|x| x.args.first().and_then(Value::as_reference))
                        .and_then(|l| step.get(l))
                        .filter(|l| l.keyword == "IFCCARTESIANPOINT")
                        .and_then(|l| l.args.first().and_then(Value::as_list))
                        .map(|c| c.get(2).and_then(Value::as_number).unwrap_or(0.0));
                    check("AxisStartInXY", loc_z.map(|z| z == 0.0));
                    // AxisDirectionInXY: Axis.Z.DirectionRatios[3] = 0 —
                    // Axis.Z derives from the optional Axis direction,
                    // defaulting to (0, 0, 1) which fails the rule.
                    let dir_z = axis.map(|x| {
                        x.args
                            .get(1)
                            .and_then(Value::as_reference)
                            .and_then(|d| step.get(d))
                            .and_then(|d| d.args.first().and_then(Value::as_list))
                            .and_then(|r| r.get(2).and_then(Value::as_number))
                            .unwrap_or(1.0)
                    });
                    check("AxisDirectionInXY", dir_z.map(|z| z == 0.0));
                }
                "IFCSURFACECURVESWEPTAREASOLID"
                | "IFCFIXEDREFERENCESWEPTAREASOLID"
                | "IFCDIRECTRIXDERIVEDREFERENCESWEPTAREASOLID" => {
                    // IfcDirectrixCurveSweptAreaSolid.DirectrixBounded:
                    // both StartParam / EndParam (indices 3, 4), or a
                    // conic / bounded Directrix (index 2).
                    let directrix = a
                        .get(2)
                        .and_then(Value::as_reference)
                        .and_then(|d| step.get(d));
                    let bounded = a.get(3).is_some_and(|v| !v.is_unset())
                        && a.get(4).is_some_and(|v| !v.is_unset());
                    check(
                        "DirectrixBounded",
                        directrix.map(|d| bounded || is_conic_or_bounded(&d.keyword)),
                    );
                }
                _ => {}
            }
            if matches!(
                inst.keyword.as_str(),
                "IFCEXTRUDEDAREASOLIDTAPERED" | "IFCREVOLVEDAREASOLIDTAPERED"
            ) {
                // CorrectProfileAssignment (IfcTaperedSweptAreaProfiles):
                // a parameterised start with an end of the same type, or
                // an IfcDerivedProfileDef whose ParentProfile is the
                // start.
                let end = a.get(4).and_then(Value::as_reference);
                let start_kw = keyword_of(step, swept);
                let end_inst = end.and_then(|e| step.get(e));
                let ok = match (start_kw, end_inst) {
                    (Some(sk), Some(ei)) => {
                        if ei.keyword == "IFCDERIVEDPROFILEDEF"
                            || ei.keyword == "IFCMIRROREDPROFILEDEF"
                        {
                            ei.args.get(2).and_then(Value::as_reference) == swept
                        } else {
                            is_parameterised(sk) && sk == ei.keyword
                        }
                    }
                    _ => false,
                };
                rule!("CorrectProfileAssignment", ok);
            }
        }
        "IFCSWEPTDISKSOLID" | "IFCSWEPTDISKSOLIDPOLYGONAL" => {
            // (Directrix, Radius, InnerRadius, StartParam, EndParam[, FilletRadius]).
            rule!(
                "InnerRadiusSize",
                match num(a, 2) {
                    None => true,
                    Some(inner) => num(a, 1)? > inner,
                },
            );
            let directrix = a
                .first()
                .and_then(Value::as_reference)
                .and_then(|d| step.get(d));
            let bounded =
                a.get(3).is_some_and(|v| !v.is_unset()) && a.get(4).is_some_and(|v| !v.is_unset());
            check(
                "DirectrixBounded",
                directrix.map(|d| bounded || is_conic_or_bounded(&d.keyword)),
            );
            if inst.keyword == "IFCSWEPTDISKSOLIDPOLYGONAL" {
                // FilletRadius (index 5) ≥ Radius when present; the
                // directrix is a polyline or a segment-less indexed
                // poly-curve.
                rule!(
                    "CorrectRadii",
                    match num(a, 5) {
                        None => true,
                        Some(fillet) => fillet >= num(a, 1)?,
                    }
                );
                check(
                    "DirectrixIsPolyline",
                    directrix.map(|d| {
                        d.keyword == "IFCPOLYLINE"
                            || (d.keyword == "IFCINDEXEDPOLYCURVE"
                                && d.args.get(1).map_or(true, Value::is_unset))
                    }),
                );
            }
        }
        // ---- Advanced Breps, faces, edges ----
        "IFCADVANCEDBREP" | "IFCADVANCEDBREPWITHVOIDS" => {
            // (Outer[, Voids]); every face of every shell an IfcAdvancedFace.
            let shell_faces_advanced = |shell: Option<u64>| -> Option<bool> {
                let faces = step.get(shell?)?.args.first()?.as_list()?;
                Some(
                    faces
                        .iter()
                        .all(|f| keyword_of(step, f.as_reference()) == Some("IFCADVANCEDFACE")),
                )
            };
            rule!(
                "HasAdvancedFaces",
                shell_faces_advanced(a.first().and_then(Value::as_reference))?
            );
            if inst.keyword == "IFCADVANCEDBREPWITHVOIDS" {
                rule!(
                    "VoidsHaveAdvancedFaces",
                    a.get(1)
                        .and_then(Value::as_list)?
                        .iter()
                        .all(|v| shell_faces_advanced(v.as_reference()) == Some(true))
                );
            }
        }
        "IFCADVANCEDFACE" => {
            // (Bounds, FaceSurface, SameSense).
            let bounds = a.first().and_then(Value::as_list);
            // Every IfcEdgeLoop bound's oriented edges, as (element keyword,
            // edge-geometry keyword) pairs.
            let loop_edges = |only_loops: bool| -> Option<Vec<(Option<&str>, Option<&str>)>> {
                let mut out = Vec::new();
                for b in bounds? {
                    let bound = step.get(b.as_reference()?)?;
                    let lp = step.get(bound.args.first()?.as_reference()?)?;
                    if lp.keyword != "IFCEDGELOOP" {
                        if only_loops {
                            continue;
                        }
                        continue;
                    }
                    for oe in lp.args.first()?.as_list()? {
                        let oe = step.get(oe.as_reference()?)?;
                        let element = oe.args.get(2).and_then(Value::as_reference);
                        let el_kw = keyword_of(step, element);
                        let geom_kw = element
                            .and_then(|e| step.get(e))
                            .filter(|e| e.keyword == "IFCEDGECURVE")
                            .and_then(|e| {
                                keyword_of(step, e.args.get(2).and_then(Value::as_reference))
                            });
                        out.push((el_kw, geom_kw));
                    }
                }
                Some(out)
            };
            rule!(
                "ApplicableEdgeCurves",
                loop_edges(true)?.iter().all(|(_, g)| {
                    matches!(
                        g,
                        Some(
                            "IFCLINE"
                                | "IFCCIRCLE"
                                | "IFCELLIPSE"
                                | "IFCPOLYLINE"
                                | "IFCBSPLINECURVEWITHKNOTS"
                                | "IFCRATIONALBSPLINECURVEWITHKNOTS"
                        )
                    )
                })
            );
            rule!(
                "ApplicableSurface",
                matches!(
                    keyword_of(step, a.get(1).and_then(Value::as_reference)),
                    Some(
                        "IFCPLANE"
                            | "IFCCYLINDRICALSURFACE"
                            | "IFCSPHERICALSURFACE"
                            | "IFCTOROIDALSURFACE"
                            | "IFCSURFACEOFREVOLUTION"
                            | "IFCSURFACEOFLINEAREXTRUSION"
                            | "IFCBSPLINESURFACEWITHKNOTS"
                            | "IFCRATIONALBSPLINESURFACEWITHKNOTS"
                    )
                )
            );
            rule!(
                "RequiresEdgeCurve",
                loop_edges(true)?
                    .iter()
                    .all(|(e, _)| *e == Some("IFCEDGECURVE"))
            );
        }
        "IFCEDGELOOP" => {
            // (EdgeList): IsClosed — first start :=: last end;
            // IsContinuous (IfcLoopHeadToTail) — each end :=: next start.
            // Oriented edges derive their ends from EdgeElement +
            // Orientation.
            let ends = |oe: &Value| -> Option<(u64, u64)> {
                let oe = step.get(oe.as_reference()?)?;
                let (el, forward) = if oe.keyword == "IFCORIENTEDEDGE" {
                    (
                        step.get(oe.args.get(2)?.as_reference()?)?,
                        oe.args.get(3).and_then(Value::as_enum) != Some("F"),
                    )
                } else {
                    (oe, true)
                };
                let s = el.args.first()?.as_reference()?;
                let e = el.args.get(1)?.as_reference()?;
                Some(if forward { (s, e) } else { (e, s) })
            };
            let list = a.first().and_then(Value::as_list);
            rule!(
                "IsClosed",
                ends(list?.first()?)?.0 == ends(list?.last()?)?.1
            );
            rule!(
                "IsContinuous",
                list?.windows(2).all(|w| match (ends(&w[0]), ends(&w[1])) {
                    (Some(p), Some(q)) => p.1 == q.0,
                    _ => false,
                })
            );
        }
        "IFCORIENTEDEDGE" => {
            rule!(
                "EdgeElementNotOriented",
                keyword_of(step, a.get(2).and_then(Value::as_reference))? != "IFCORIENTEDEDGE"
            );
        }
        // ---- Surfaces ----
        "IFCTOROIDALSURFACE" => {
            rule!("MajorLargerMinor", num(a, 2)? < num(a, 1)?);
        }
        "IFCSURFACEOFREVOLUTION" | "IFCSURFACEOFLINEAREXTRUSION" => {
            // IfcSweptSurface.SweptCurveType: SweptCurve.ProfileType = CURVE.
            rule!(
                "SweptCurveType",
                step.get(a.first()?.as_reference()?)?
                    .args
                    .first()
                    .and_then(Value::as_enum)?
                    == "CURVE"
            );
            if inst.keyword == "IFCSURFACEOFLINEAREXTRUSION" {
                rule!("DepthGreaterZero", num(a, 3)? > 0.0);
            }
        }
        "IFCBSPLINESURFACEWITHKNOTS" | "IFCRATIONALBSPLINESURFACEWITHKNOTS" => {
            // (UDegree, VDegree, ControlPointsList, SurfaceForm, UClosed,
            // VClosed, SelfIntersect, UMultiplicities, VMultiplicities,
            // UKnots, VKnots, KnotSpec[, WeightsData]).
            let rows = a.get(2).and_then(Value::as_list);
            let u_upper = rows.map(|r| r.len() as i64 - 1);
            let v_upper = rows
                .and_then(|r| r.first())
                .and_then(Value::as_list)
                .map(|r| r.len() as i64 - 1);
            let ints = |i: usize| -> Option<Vec<i64>> {
                a.get(i)
                    .and_then(Value::as_list)
                    .map(|l| l.iter().filter_map(int).collect())
            };
            let reals = |i: usize| -> Option<Vec<f64>> {
                a.get(i).and_then(Value::as_list).map(|l| {
                    l.iter()
                        .filter_map(|v| num(core::slice::from_ref(v), 0))
                        .collect()
                })
            };
            let (um, vm, uk, vk) = (ints(7), ints(8), reals(9), reals(10));
            rule!(
                "CorrespondingULists",
                um.as_ref()?.len() == uk.as_ref()?.len()
            );
            rule!(
                "CorrespondingVLists",
                vm.as_ref()?.len() == vk.as_ref()?.len()
            );
            rule!(
                "UDirectionConstraints",
                crate::geometry::bspline::constraints_param_bspline(
                    a.first().and_then(int)?,
                    uk.as_ref()?.len(),
                    u_upper?,
                    um.as_ref()?,
                    uk.as_ref()?,
                )
            );
            rule!(
                "VDirectionConstraints",
                crate::geometry::bspline::constraints_param_bspline(
                    a.get(1).and_then(int)?,
                    vk.as_ref()?.len(),
                    v_upper?,
                    vm.as_ref()?,
                    vk.as_ref()?,
                )
            );
            if inst.keyword == "IFCRATIONALBSPLINESURFACEWITHKNOTS" {
                let weights = a.get(12).and_then(Value::as_list);
                rule!(
                    "CorrespondingWeightsDataLists",
                    weights?.len() == rows?.len()
                        && weights?.first()?.as_list()?.len() == rows?.first()?.as_list()?.len()
                );
                // IfcSurfaceWeightsPositive: every weight > 0.
                rule!(
                    "WeightValuesGreaterZero",
                    weights?.iter().all(|row| {
                        row.as_list().is_some_and(|r| {
                            r.iter()
                                .all(|w| num(core::slice::from_ref(w), 0).is_some_and(|w| w > 0.0))
                        })
                    })
                );
            }
        }
        "IFCRECTANGULARTRIMMEDSURFACE" => {
            // (BasisSurface, U1, V1, U2, V2, Usense, Vsense).
            let (u1, v1, u2, v2) = (num(a, 1), num(a, 2), num(a, 3), num(a, 4));
            let sense = |i: usize| match a.get(i).and_then(Value::as_enum) {
                Some("T") => Some(true),
                Some("F") => Some(false),
                _ => None,
            };
            let basis = keyword_of(step, a.first().and_then(Value::as_reference));
            rule!("U1AndU2Different", u1? != u2?);
            // Elementary non-plane bases and surfaces of revolution
            // wrap, so any sense is compatible there.
            let wraps = matches!(
                basis,
                Some(
                    "IFCCYLINDRICALSURFACE"
                        | "IFCSPHERICALSURFACE"
                        | "IFCTOROIDALSURFACE"
                        | "IFCSURFACEOFREVOLUTION"
                )
            );
            rule!("UsenseCompatible", wraps || sense(5)? == (u2? > u1?));
            rule!("V1AndV2Different", v1? != v2?);
            rule!("VsenseCompatible", sense(6)? == (v2? > v1?));
        }
        // ---- Curves on surfaces ----
        "IFCPCURVE" => {
            // (BasisSurface, ReferenceCurve).
            rule!(
                "DimIs2D",
                crate::geometry::curve_dimension(step, a.get(1)?.as_reference()?, 0)? == 2
            );
        }
        "IFCSURFACECURVE" | "IFCINTERSECTIONCURVE" | "IFCSEAMCURVE" => {
            // (Curve3D, AssociatedGeometry, MasterRepresentation).
            let curve = a.first().and_then(Value::as_reference);
            rule!(
                "CurveIs3D",
                crate::geometry::curve_dimension(step, curve?, 0)? == 3
            );
            rule!("CurveIsNotPcurve", keyword_of(step, curve)? != "IFCPCURVE");
            if inst.keyword != "IFCSURFACECURVE" {
                let pcurves = a.get(1).and_then(Value::as_list);
                rule!("TwoPCurves", pcurves?.len() == 2);
                let bases: Option<Vec<u64>> = pcurves.map(|ps| {
                    ps.iter()
                        .filter_map(|p| pcurve_basis(step, p.as_reference()?))
                        .collect()
                });
                if inst.keyword == "IFCSEAMCURVE" {
                    rule!("SameSurface", {
                        let b = bases?;
                        b.len() == 2 && b[0] == b[1]
                    });
                } else {
                    rule!("DistinctSurfaces", {
                        let b = bases?;
                        b.len() == 2 && b[0] != b[1]
                    });
                }
            }
        }
        "IFCCOMPOSITECURVEONSURFACE" | "IFCBOUNDARYCURVE" | "IFCOUTERBOUNDARYCURVE" => {
            // (Segments, SelfIntersect). SameSurface: the segments'
            // parent curves share a basis surface (the transcribed
            // IfcGetBasisSurface intersection); IsClosed (boundary
            // curves): the last segment's Transition is not
            // DISCONTINUOUS (the derived ClosedCurve).
            let segments = a.first().and_then(Value::as_list);
            rule!("SameSurface", {
                let segs = segments?;
                let mut common: Option<Vec<u64>> = None;
                for seg in segs {
                    let parent = step.get(seg.as_reference()?)?.args.get(2)?.as_reference()?;
                    let bases = basis_surfaces(step, parent, 0);
                    common = Some(match common {
                        None => bases,
                        Some(c) => c.into_iter().filter(|b| bases.contains(b)).collect(),
                    });
                }
                !common?.is_empty()
            });
            if inst.keyword != "IFCCOMPOSITECURVEONSURFACE" {
                rule!("IsClosed", {
                    let last = step.get(segments?.last()?.as_reference()?)?;
                    last.args.first()?.as_enum()? != "DISCONTINUOUS"
                });
            }
        }
        "IFCSECTIONEDSOLIDHORIZONTAL" | "IFCSECTIONEDSURFACE" => {
            // IfcSectionedSolidHorizontal(Directrix, CrossSections,
            // CrossSectionPositions); IfcSectionedSurface(Directrix,
            // CrossSectionPositions, CrossSections).
            let surface = inst.keyword == "IFCSECTIONEDSURFACE";
            let (sections, positions) = if surface {
                (a.get(2), a.get(1))
            } else {
                (a.get(1), a.get(2))
            };
            let sections = sections.and_then(Value::as_list);
            let positions = positions.and_then(Value::as_list);
            let profile = |p: &Value| p.as_reference().and_then(|pid| step.get(pid));
            let profile_type = |p: &Value| profile(p).and_then(|i| i.args.first()?.as_enum());
            rule!(
                "DirectrixIs3D",
                crate::geometry::curve_dimension(step, a.first()?.as_reference()?, 0)? == 3
            );
            if surface {
                rule!(
                    "AreaProfileTypes",
                    sections?.iter().any(|p| profile_type(p) == Some("CURVE"))
                );
            } else {
                rule!("ConsistentProfileTypes", {
                    let first = profile_type(sections?.first()?);
                    sections?.iter().all(|p| profile_type(p) == first)
                });
            }
            rule!("SectionsSameType", {
                let first = profile(sections?.first()?)?.keyword.as_str();
                sections?
                    .iter()
                    .all(|p| profile(p).is_some_and(|i| i.keyword == first))
            });
            rule!(
                "CorrespondingSectionPositions",
                sections?.len() == positions?.len()
            );
            // Offsets live on the placement's IfcPointByDistanceExpression
            // Location: (DistanceAlong, OffsetLateral, OffsetVertical,
            // OffsetLongitudinal, BasisCurve).
            let offset_present = |p: &Value, index: usize| -> Option<bool> {
                let placement = step.get(p.as_reference()?)?;
                let loc = step.get(placement.args.first()?.as_reference()?)?;
                Some(loc.args.get(index).is_some_and(|v| !v.is_unset()))
            };
            if surface {
                rule!("NoOffsets", {
                    let ps = positions?;
                    let mut any = false;
                    for p in ps {
                        for i in 1..=3 {
                            any |= offset_present(p, i)?;
                        }
                    }
                    !any
                });
            } else {
                rule!("NoLongitudinalOffsets", {
                    let ps = positions?;
                    let mut any = false;
                    for p in ps {
                        any |= offset_present(p, 3)?;
                    }
                    !any
                });
            }
        }
        // ---- Boolean results and half-spaces ----
        "IFCBOOLEANRESULT" | "IFCBOOLEANCLIPPINGRESULT" => {
            // (Operator, FirstOperand, SecondOperand).
            let operand = |i: usize| {
                a.get(i)
                    .and_then(Value::as_reference)
                    .and_then(|id| step.get(id))
            };
            // A tessellated-face-set operand must be Closed = TRUE
            // (Closed is attribute 2 of a triangulated set, 1 of a
            // polygonal one); every other operand kind passes.
            let closed_ok = |i: usize| -> Option<bool> {
                let inst = operand(i)?;
                let closed = match inst.keyword.as_str() {
                    "IFCTRIANGULATEDFACESET" | "IFCTRIANGULATEDIRREGULARNETWORK" => {
                        inst.args.get(2)
                    }
                    "IFCPOLYGONALFACESET" => inst.args.get(1),
                    _ => return Some(true),
                };
                Some(closed.and_then(Value::as_enum) == Some("T"))
            };
            check("FirstOperandClosed", closed_ok(1));
            check("SecondOperandClosed", closed_ok(2));
            // Every IfcBooleanOperand member is three-dimensional.
            rule!("SameDim", {
                operand(1)?;
                operand(2)?;
                true
            });
            if inst.keyword == "IFCBOOLEANCLIPPINGRESULT" {
                rule!("OperatorType", a.first()?.as_enum()? == "DIFFERENCE");
                // The schema literal reads IFCSWEPTDISCSOLID (sic); the
                // intended IfcSweptDiskSolid is accepted (half-space
                // digest §5.3).
                rule!(
                    "FirstOperandType",
                    matches!(
                        operand(1)?.keyword.as_str(),
                        "IFCEXTRUDEDAREASOLID"
                            | "IFCEXTRUDEDAREASOLIDTAPERED"
                            | "IFCREVOLVEDAREASOLID"
                            | "IFCREVOLVEDAREASOLIDTAPERED"
                            | "IFCFIXEDREFERENCESWEPTAREASOLID"
                            | "IFCDIRECTRIXDERIVEDREFERENCESWEPTAREASOLID"
                            | "IFCSURFACECURVESWEPTAREASOLID"
                            | "IFCSWEPTDISKSOLID"
                            | "IFCSWEPTDISKSOLIDPOLYGONAL"
                            | "IFCBOOLEANCLIPPINGRESULT"
                    )
                );
                rule!(
                    "SecondOperandType",
                    matches!(
                        operand(2)?.keyword.as_str(),
                        "IFCHALFSPACESOLID" | "IFCPOLYGONALBOUNDEDHALFSPACE" | "IFCBOXEDHALFSPACE"
                    )
                );
            }
        }
        "IFCPOLYGONALBOUNDEDHALFSPACE" => {
            // (BaseSurface, AgreementFlag, Position, PolygonalBoundary).
            let boundary = a.get(3).and_then(Value::as_reference);
            rule!(
                "BoundaryDim",
                crate::geometry::curve_dimension(step, boundary?, 0)? == 2
            );
            rule!(
                "BoundaryType",
                matches!(
                    keyword_of(step, boundary)?,
                    "IFCPOLYLINE" | "IFCCOMPOSITECURVE" | "IFCINDEXEDPOLYCURVE"
                )
            );
        }
        "IFCBOXEDHALFSPACE" => {
            // (BaseSurface, AgreementFlag, Enclosure).
            rule!(
                "UnboundedSurface",
                keyword_of(step, a.first()?.as_reference())? != "IFCCURVEBOUNDEDPLANE"
            );
        }
        "IFCTRIANGULATEDIRREGULARNETWORK" => {
            // (Coordinates, Normals, Closed, CoordIndex, PnIndex, Flags):
            // NotClosed — Closed = FALSE (an unset flag is not TRUE).
            rule!("NotClosed", a.get(2).and_then(Value::as_enum) != Some("T"));
        }
        "IFCSECTIONEDSPINE" => {
            // (SpineCurve, CrossSections, CrossSectionPositions).
            let sections = a.get(1).and_then(Value::as_list);
            let positions = a.get(2).and_then(Value::as_list);
            rule!(
                "CorrespondingSectionPositions",
                sections?.len() == positions?.len()
            );
            let first_type = sections.and_then(|ps| {
                let p = step.get(ps.first()?.as_reference()?)?;
                p.args.first().and_then(Value::as_enum)
            });
            check(
                "ConsistentProfileTypes",
                sections.map(|ps| {
                    ps.iter().all(|p| {
                        p.as_reference()
                            .and_then(|pid| step.get(pid))
                            .and_then(|i| i.args.first().and_then(Value::as_enum))
                            == first_type
                    })
                }),
            );
        }
        // ---- B-spline curves ----
        "IFCBSPLINECURVEWITHKNOTS" | "IFCRATIONALBSPLINECURVEWITHKNOTS" => {
            // (Degree, ControlPointsList, CurveForm, ClosedCurve,
            // SelfIntersect, KnotMultiplicities, Knots, KnotSpec
            // [, WeightsData]).
            let degree = a.first().and_then(int);
            let cp_count = a.get(1).and_then(Value::as_list).map(<[Value]>::len);
            let mults: Option<Vec<i64>> = a
                .get(5)
                .and_then(Value::as_list)
                .map(|l| l.iter().filter_map(int).collect());
            let knots: Option<Vec<f64>> = a.get(6).and_then(Value::as_list).map(|l| {
                l.iter()
                    .filter_map(|v| num(core::slice::from_ref(v), 0))
                    .collect()
            });
            rule!(
                "ConsistentBSpline",
                crate::geometry::bspline::constraints_param_bspline(
                    degree?,
                    knots.as_ref()?.len(),
                    cp_count? as i64 - 1,
                    mults.as_ref()?,
                    knots.as_ref()?,
                )
            );
            rule!(
                "CorrespondingKnotLists",
                mults.as_ref()?.len() == knots.as_ref()?.len()
                    && a.get(5).and_then(Value::as_list)?.len() == mults.as_ref()?.len()
            );
            if inst.keyword == "IFCRATIONALBSPLINECURVEWITHKNOTS" {
                let weights: Option<Vec<f64>> = a.get(8).and_then(Value::as_list).map(|l| {
                    l.iter()
                        .filter_map(|v| num(core::slice::from_ref(v), 0))
                        .collect()
                });
                rule!(
                    "SameNumOfWeightsAndPoints",
                    a.get(8).and_then(Value::as_list)?.len() == cp_count?
                );
                // IfcCurveWeightsPositive: every weight > 0.
                rule!(
                    "WeightsGreaterZero",
                    weights.as_ref()?.len() == a.get(8).and_then(Value::as_list)?.len()
                        && weights.as_ref()?.iter().all(|w| *w > 0.0)
                );
            }
        }
        // ---- Georeferencing ----
        "IFCMAPCONVERSION" | "IFCMAPCONVERSIONSCALED" => {
            rule!(
                "TargetCRSOnlyProjected",
                keyword_of(step, a.get(1).and_then(Value::as_reference)) == Some("IFCPROJECTEDCRS")
            );
        }
        "IFCRIGIDOPERATION" => {
            let kind = |i: usize| a.get(i).and_then(Value::as_typed).map(|(k, _)| k);
            let (f, s) = (kind(2), kind(3));
            rule!(
                "SameCoordinateType",
                (f == Some("IFCLENGTHMEASURE") && s == Some("IFCLENGTHMEASURE"))
                    || (f == Some("IFCPLANEANGLEMEASURE") && s == Some("IFCPLANEANGLEMEASURE"))
            );
        }
        _ => return None,
    }
    Some(
        failed
            .into_iter()
            .map(|rule| RuleViolation {
                id,
                entity: inst.keyword.clone(),
                rule,
            })
            .collect(),
    )
}

/// Every violation across the model's instances that carry transcribed
/// rules, in ascending id order.
pub fn model_where_rule_violations(step: &StepFile) -> Vec<RuleViolation> {
    step.instances
        .values()
        .flat_map(|inst| where_rule_violations(step, inst.id).unwrap_or_default())
        .collect()
}

/// The `IfcParameterizedProfileDef` subtypes (IFC 4.3 ONEOF list).
fn is_parameterised(keyword: &str) -> bool {
    matches!(
        keyword,
        "IFCASYMMETRICISHAPEPROFILEDEF"
            | "IFCCSHAPEPROFILEDEF"
            | "IFCCIRCLEPROFILEDEF"
            | "IFCCIRCLEHOLLOWPROFILEDEF"
            | "IFCELLIPSEPROFILEDEF"
            | "IFCISHAPEPROFILEDEF"
            | "IFCLSHAPEPROFILEDEF"
            | "IFCRECTANGLEPROFILEDEF"
            | "IFCRECTANGLEHOLLOWPROFILEDEF"
            | "IFCROUNDEDRECTANGLEPROFILEDEF"
            | "IFCTSHAPEPROFILEDEF"
            | "IFCTRAPEZIUMPROFILEDEF"
            | "IFCUSHAPEPROFILEDEF"
            | "IFCZSHAPEPROFILEDEF"
    )
}

/// The `BasisSurface` of an `IfcPcurve`.
fn pcurve_basis(step: &StepFile, id: u64) -> Option<u64> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCPCURVE" {
        return None;
    }
    inst.args.first()?.as_reference()
}

/// The transcribed `IfcGetBasisSurface`: the surfaces a curve-on-
/// surface lies on — a p-curve's basis, a surface curve's associated
/// p-curve bases, the intersection over a composite's segments.
fn basis_surfaces(step: &StepFile, id: u64, depth: usize) -> Vec<u64> {
    if depth > 32 {
        return Vec::new();
    }
    let Some(inst) = step.get(id) else {
        return Vec::new();
    };
    match inst.keyword.as_str() {
        "IFCPCURVE" => pcurve_basis(step, id).into_iter().collect(),
        "IFCSURFACECURVE" | "IFCINTERSECTIONCURVE" | "IFCSEAMCURVE" => inst
            .args
            .get(1)
            .and_then(Value::as_list)
            .map(|ps| {
                ps.iter()
                    .filter_map(|p| pcurve_basis(step, p.as_reference()?))
                    .collect()
            })
            .unwrap_or_default(),
        "IFCCOMPOSITECURVEONSURFACE" | "IFCBOUNDARYCURVE" | "IFCOUTERBOUNDARYCURVE" => {
            let mut common: Option<Vec<u64>> = None;
            for seg in inst.args.first().and_then(Value::as_list).unwrap_or(&[]) {
                let parent = seg
                    .as_reference()
                    .and_then(|sid| step.get(sid))
                    .and_then(|s| s.args.get(2).and_then(Value::as_reference));
                let bases = parent.map_or_else(Vec::new, |p| basis_surfaces(step, p, depth + 1));
                common = Some(match common {
                    None => bases,
                    Some(c) => c.into_iter().filter(|b| bases.contains(b)).collect(),
                });
            }
            common.unwrap_or_default()
        }
        _ => Vec::new(),
    }
}

/// `IfcConic` or `IfcBoundedCurve` subtypes — the curves whose extent
/// bounds a directrix without explicit parameters.
fn is_conic_or_bounded(keyword: &str) -> bool {
    matches!(
        keyword,
        "IFCCIRCLE"
            | "IFCELLIPSE"
            | "IFCPOLYLINE"
            | "IFCTRIMMEDCURVE"
            | "IFCCOMPOSITECURVE"
            | "IFCCOMPOSITECURVEONSURFACE"
            | "IFCBOUNDARYCURVE"
            | "IFCOUTERBOUNDARYCURVE"
            | "IFCINDEXEDPOLYCURVE"
            | "IFCBSPLINECURVE"
            | "IFCBSPLINECURVEWITHKNOTS"
            | "IFCRATIONALBSPLINECURVEWITHKNOTS"
            | "IFCSEGMENTEDREFERENCECURVE"
            | "IFCGRADIENTCURVE"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_step;

    fn parse(data: &str) -> StepFile {
        let text = format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t.ifc','2026-08-30T00:00:00',('a'),('o'),'p','s','auth');\n\
             FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        );
        parse_step(text.as_bytes()).expect("parse failed")
    }

    fn rules(step: &StepFile, id: u64) -> Vec<&'static str> {
        where_rule_violations(step, id)
            .expect("entity has rules")
            .into_iter()
            .map(|v| v.rule)
            .collect()
    }

    #[test]
    fn valid_profiles_pass_and_unruled_entities_are_none() {
        let f = parse(
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,10.,15.,5.,3.,$);\n\
             #2=IFCLSHAPEPROFILEDEF(.AREA.,$,$,100.,$,8.,$,$,$);\n\
             #3=IFCCSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,2.,15.,3.);\n\
             #4=IFCRECTANGLEHOLLOWPROFILEDEF(.AREA.,$,$,40.,20.,4.,2.,6.);\n\
             #5=IFCCARTESIANPOINT((0.,0.));",
        );
        for id in 1..=4 {
            assert_eq!(rules(&f, id), Vec::<&str>::new(), "#{id}");
        }
        assert_eq!(where_rule_violations(&f, 5), None);
        assert_eq!(where_rule_violations(&f, 99), None);
        assert!(model_where_rule_violations(&f).is_empty());
    }

    #[test]
    fn profile_rules_name_the_failure() {
        let f = parse(
            "#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,200.,100.,150.,60.,$,$);\n\
             #2=IFCLSHAPEPROFILEDEF(.AREA.,$,$,100.,60.,80.,$,$,$);\n\
             #10=IFCASYMMETRICISHAPEPROFILEDEF(.AREA.,$,$,120.,200.,130.,150.,60.,80.,60.,40.,$,$,$,$);\n\
             #3=IFCTSHAPEPROFILEDEF(.AREA.,$,$,120.,80.,90.,130.,$,$,$,$,$);\n\
             #4=IFCUSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,60.,50.,$,$,$);\n\
             #5=IFCZSHAPEPROFILEDEF(.AREA.,$,$,100.,40.,6.,50.,$,$);\n\
             #6=IFCCSHAPEPROFILEDEF(.AREA.,$,$,100.,50.,30.,60.,40.);\n\
             #7=IFCROUNDEDRECTANGLEPROFILEDEF(.AREA.,$,$,40.,20.,11.);\n\
             #8=IFCRECTANGLEHOLLOWPROFILEDEF(.AREA.,$,$,40.,20.,10.,7.,11.);\n\
             #9=IFCCIRCLEHOLLOWPROFILEDEF(.AREA.,$,$,5.,5.);",
        );
        assert_eq!(
            rules(&f, 1),
            [
                "ValidFilletRadius",
                "ValidFlangeThickness",
                "ValidWebThickness"
            ]
        );
        assert_eq!(rules(&f, 2), ["ValidThickness"]);
        assert_eq!(rules(&f, 3), ["ValidFlangeThickness", "ValidWebThickness"]);
        assert_eq!(rules(&f, 4), ["ValidFlangeThickness", "ValidWebThickness"]);
        assert_eq!(rules(&f, 5), ["ValidFlangeThickness"]);
        assert_eq!(
            rules(&f, 6),
            [
                "ValidGirth",
                "ValidInternalFilletRadius",
                "ValidWallThickness"
            ]
        );
        assert_eq!(rules(&f, 7), ["ValidRadius"]);
        assert_eq!(
            rules(&f, 8),
            ["ValidInnerRadius", "ValidOuterRadius", "ValidWallThickness"]
        );
        assert_eq!(rules(&f, 9), ["WR1"]);
        assert_eq!(
            rules(&f, 10),
            [
                "ValidBottomFilletRadius",
                "ValidFlangeThickness",
                "ValidTopFilletRadius",
                "ValidWebThickness"
            ]
        );
        let all = model_where_rule_violations(&f);
        assert_eq!(all.len(), 21);
        assert_eq!(all[0].id, 1);
        assert_eq!(all[0].entity, "IFCISHAPEPROFILEDEF");
    }

    #[test]
    fn missing_attributes_fail_the_rule_that_needs_them() {
        // OverallDepth unset: only the rule that reads it fails (the
        // fillet rule is vacuously true with the radius omitted, the
        // web rule reads width only).
        let f = parse("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,$,10.,15.,$,$,$);");
        assert_eq!(rules(&f, 1), ["ValidFlangeThickness"]);
        // …and with a fillet radius given, that rule needs the depth too.
        let f = parse("#1=IFCISHAPEPROFILEDEF(.AREA.,$,$,100.,$,10.,15.,1.,$,$);");
        assert_eq!(rules(&f, 1), ["ValidFilletRadius", "ValidFlangeThickness"]);
    }

    #[test]
    fn composite_profile_rules() {
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,4.);\n\
             #2=IFCRECTANGLEPROFILEDEF(.CURVE.,$,$,2.,4.);\n\
             #3=IFCCOMPOSITEPROFILEDEF(.AREA.,$,(#1,#2),$);\n\
             #4=IFCCOMPOSITEPROFILEDEF(.AREA.,$,(#1,#3),$);\n\
             #5=IFCCOMPOSITEPROFILEDEF(.AREA.,$,(#1,#1),$);",
        );
        assert_eq!(rules(&f, 3), ["InvariantProfileType"]);
        assert_eq!(rules(&f, 4), ["NoRecursion"]);
        assert!(rules(&f, 5).is_empty());
    }

    #[test]
    fn swept_solid_rules() {
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,4.);\n\
             #2=IFCRECTANGLEPROFILEDEF(.CURVE.,$,$,2.,4.);\n\
             #3=IFCDIRECTION((0.,0.,1.));\n\
             #4=IFCDIRECTION((1.,0.,0.));\n\
             #10=IFCEXTRUDEDAREASOLID(#1,$,#3,3.);\n\
             #11=IFCEXTRUDEDAREASOLID(#2,$,#4,3.);\n\
             #12=IFCEXTRUDEDAREASOLIDTAPERED(#1,$,#3,3.,#1);\n\
             #13=IFCEXTRUDEDAREASOLIDTAPERED(#1,$,#3,3.,#20);\n\
             #14=IFCEXTRUDEDAREASOLIDTAPERED(#1,$,#3,3.,#21);\n\
             #15=IFCEXTRUDEDAREASOLIDTAPERED(#1,$,#3,3.,#22);\n\
             #20=IFCCIRCLEPROFILEDEF(.AREA.,$,$,1.);\n\
             #21=IFCDERIVEDPROFILEDEF(.AREA.,$,#1,#23,$);\n\
             #22=IFCDERIVEDPROFILEDEF(.AREA.,$,#20,#23,$);\n\
             #23=IFCCARTESIANTRANSFORMATIONOPERATOR2D($,$,#24,0.5);\n\
             #24=IFCCARTESIANPOINT((0.,0.));\n\
             #30=IFCAXIS1PLACEMENT(#31,#32);\n#31=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #32=IFCDIRECTION((0.,1.,0.));\n\
             #33=IFCAXIS1PLACEMENT(#34,$);\n#34=IFCCARTESIANPOINT((0.,0.,5.));\n\
             #40=IFCREVOLVEDAREASOLID(#1,$,#30,1.);\n\
             #41=IFCREVOLVEDAREASOLID(#1,$,#33,1.);\n\
             #50=IFCCARTESIANPOINT((0.,0.,0.));\n#51=IFCLINE(#50,#52);\n\
             #52=IFCVECTOR(#3,1.);\n#53=IFCPOLYLINE((#50,#50));\n\
             #60=IFCFIXEDREFERENCESWEPTAREASOLID(#1,$,#51,$,$,#4);\n\
             #61=IFCFIXEDREFERENCESWEPTAREASOLID(#1,$,#51,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(2.),#4);\n\
             #62=IFCSURFACECURVESWEPTAREASOLID(#2,$,#53,$,$,#4);",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["SweptAreaType", "ValidExtrusionDirection"]);
        assert!(rules(&f, 12).is_empty());
        assert_eq!(rules(&f, 13), ["CorrectProfileAssignment"]);
        assert!(rules(&f, 14).is_empty());
        assert_eq!(rules(&f, 15), ["CorrectProfileAssignment"]);
        assert!(rules(&f, 40).is_empty());
        assert_eq!(rules(&f, 41), ["AxisStartInXY", "AxisDirectionInXY"]);
        assert_eq!(rules(&f, 60), ["DirectrixBounded"]);
        assert!(rules(&f, 61).is_empty());
        assert_eq!(rules(&f, 62), ["SweptAreaType"]);
    }

    #[test]
    fn swept_disk_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCCARTESIANPOINT((0.,0.,5.));\n\
             #3=IFCPOLYLINE((#1,#2));\n\
             #4=IFCLINE(#1,#5);\n#5=IFCVECTOR(#6,1.);\n#6=IFCDIRECTION((0.,0.,1.));\n\
             #10=IFCSWEPTDISKSOLID(#3,2.,1.,$,$);\n\
             #11=IFCSWEPTDISKSOLID(#3,1.,2.,$,$);\n\
             #12=IFCSWEPTDISKSOLID(#4,2.,$,$,$);\n\
             #13=IFCSWEPTDISKSOLID(#4,2.,$,0.,5.);",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["InnerRadiusSize"]);
        assert_eq!(rules(&f, 12), ["DirectrixBounded"]);
        assert!(rules(&f, 13).is_empty());
    }

    #[test]
    fn advanced_brep_face_and_edge_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((1.,1.,0.));\n\
             #11=IFCVERTEXPOINT(#1);\n#12=IFCVERTEXPOINT(#2);\n#13=IFCVERTEXPOINT(#3);\n\
             #20=IFCDIRECTION((1.,0.,0.));\n#21=IFCVECTOR(#20,1.);\n#22=IFCLINE(#1,#21);\n\
             #23=IFCPOLYLINE((#1,#2));\n\
             #30=IFCEDGECURVE(#11,#12,#22,.T.);\n#31=IFCEDGECURVE(#12,#13,#23,.T.);\n\
             #32=IFCEDGECURVE(#13,#11,#22,.T.);\n#33=IFCEDGE(#13,#11);\n\
             #34=IFCEDGECURVE(#13,#11,#5,.T.);\n#5=IFCTRIMMEDCURVE(#22,(#1),(#2),.T.,.CARTESIAN.);\n\
             #40=IFCORIENTEDEDGE(*,*,#30,.T.);\n#41=IFCORIENTEDEDGE(*,*,#31,.T.);\n\
             #42=IFCORIENTEDEDGE(*,*,#32,.T.);\n#43=IFCORIENTEDEDGE(*,*,#33,.T.);\n\
             #44=IFCORIENTEDEDGE(*,*,#34,.T.);\n#45=IFCORIENTEDEDGE(*,*,#40,.T.);\n\
             #50=IFCEDGELOOP((#40,#41,#42));\n\
             #51=IFCEDGELOOP((#40,#41));\n\
             #52=IFCEDGELOOP((#40,#42,#41));\n\
             #53=IFCEDGELOOP((#40,#41,#43));\n\
             #54=IFCEDGELOOP((#40,#41,#44));\n\
             #60=IFCFACEOUTERBOUND(#50,.T.);\n#61=IFCFACEOUTERBOUND(#53,.T.);\n\
             #62=IFCFACEOUTERBOUND(#54,.T.);\n\
             #70=IFCAXIS2PLACEMENT3D(#1,$,$);\n#71=IFCPLANE(#70);\n\
             #72=IFCCURVEBOUNDEDPLANE(#71,#23,$);\n\
             #80=IFCADVANCEDFACE((#60),#71,.T.);\n\
             #81=IFCADVANCEDFACE((#61),#71,.T.);\n\
             #82=IFCADVANCEDFACE((#62),#72,.T.);\n\
             #83=IFCFACE((#60));\n\
             #90=IFCCLOSEDSHELL((#80));\n#91=IFCCLOSEDSHELL((#80,#83));\n\
             #92=IFCADVANCEDBREP(#90);\n#93=IFCADVANCEDBREP(#91);\n\
             #94=IFCADVANCEDBREPWITHVOIDS(#90,(#91));",
        );
        assert!(rules(&f, 50).is_empty());
        assert_eq!(rules(&f, 51), ["IsClosed"]);
        // Out of order: neither continuous nor (first start = last end) closed.
        assert_eq!(rules(&f, 52), ["IsClosed", "IsContinuous"]);
        assert!(rules(&f, 40).is_empty());
        assert_eq!(rules(&f, 45), ["EdgeElementNotOriented"]);
        assert!(rules(&f, 80).is_empty());
        // A bare IfcEdge in the loop fails both edge rules.
        assert_eq!(rules(&f, 81), ["ApplicableEdgeCurves", "RequiresEdgeCurve"]);
        // A trimmed-curve edge geometry and a bounded-surface face surface.
        assert_eq!(rules(&f, 82), ["ApplicableEdgeCurves", "ApplicableSurface"]);
        assert!(rules(&f, 92).is_empty());
        assert_eq!(rules(&f, 93), ["HasAdvancedFaces"]);
        assert_eq!(rules(&f, 94), ["VoidsHaveAdvancedFaces"]);
    }

    #[test]
    fn surface_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCAXIS2PLACEMENT3D(#1,$,$);\n\
             #3=IFCTOROIDALSURFACE(#2,3.,1.);\n#4=IFCTOROIDALSURFACE(#2,1.,3.);\n\
             #10=IFCCARTESIANPOINT((1.,0.));\n#11=IFCCARTESIANPOINT((1.,2.));\n\
             #12=IFCPOLYLINE((#10,#11));\n#13=IFCARBITRARYOPENPROFILEDEF(.CURVE.,$,#12);\n\
             #14=IFCARBITRARYOPENPROFILEDEF(.AREA.,$,#12);\n\
             #15=IFCDIRECTION((0.,0.,1.));\n\
             #20=IFCSURFACEOFLINEAREXTRUSION(#13,#2,#15,2.);\n\
             #21=IFCSURFACEOFLINEAREXTRUSION(#14,#2,#15,0.);\n\
             #22=IFCAXIS1PLACEMENT(#1,$);\n#23=IFCSURFACEOFREVOLUTION(#13,$,#22);\n\
             #30=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #31=IFCBSPLINESURFACEWITHKNOTS(1,1,((#30,#30),(#30,#30)),.PLANE_SURF.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n\
             #32=IFCBSPLINESURFACEWITHKNOTS(1,1,((#30,#30),(#30,#30)),.PLANE_SURF.,.F.,.F.,.F.,(2,1),(2,2),(0.,1.),(0.,1.,2.),.UNSPECIFIED.);\n\
             #33=IFCRATIONALBSPLINESURFACEWITHKNOTS(1,1,((#30,#30),(#30,#30)),.PLANE_SURF.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.,((1.,1.),(1.,1.)));\n\
             #34=IFCRATIONALBSPLINESURFACEWITHKNOTS(1,1,((#30,#30),(#30,#30)),.PLANE_SURF.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.,((1.),(1.,0.)));",
        );
        assert!(rules(&f, 3).is_empty());
        assert_eq!(rules(&f, 4), ["MajorLargerMinor"]);
        assert!(rules(&f, 20).is_empty());
        assert_eq!(rules(&f, 21), ["SweptCurveType", "DepthGreaterZero"]);
        assert!(rules(&f, 23).is_empty());
        assert!(rules(&f, 31).is_empty());
        assert_eq!(
            rules(&f, 32),
            [
                "CorrespondingVLists",
                "UDirectionConstraints",
                "VDirectionConstraints"
            ]
        );
        assert!(rules(&f, 33).is_empty());
        assert_eq!(
            rules(&f, 34),
            ["CorrespondingWeightsDataLists", "WeightValuesGreaterZero"]
        );
    }

    #[test]
    fn rectangular_trimmed_surface_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCAXIS2PLACEMENT3D(#1,$,$);\n\
             #3=IFCPLANE(#2);\n#4=IFCCYLINDRICALSURFACE(#2,1.);\n\
             #10=IFCRECTANGULARTRIMMEDSURFACE(#3,0.,0.,3.,2.,.T.,.T.);\n\
             #11=IFCRECTANGULARTRIMMEDSURFACE(#3,3.,0.,0.,2.,.T.,.T.);\n\
             #12=IFCRECTANGULARTRIMMEDSURFACE(#4,3.,2.,0.,2.,.T.,.T.);\n\
             #13=IFCRECTANGULARTRIMMEDSURFACE(#3,1.,0.,1.,2.,.T.,.T.);",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["UsenseCompatible"]);
        assert_eq!(rules(&f, 12), ["V1AndV2Different", "VsenseCompatible"]);
        assert_eq!(rules(&f, 13), ["U1AndU2Different", "UsenseCompatible"]);
    }

    #[test]
    fn curve_on_surface_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCAXIS2PLACEMENT3D(#1,$,$);\n\
             #3=IFCPLANE(#2);\n#4=IFCCYLINDRICALSURFACE(#2,1.);\n\
             #5=IFCCARTESIANPOINT((0.,0.));\n#6=IFCCARTESIANPOINT((1.,0.));\n\
             #7=IFCPOLYLINE((#5,#6));\n#8=IFCPOLYLINE((#1,#1));\n\
             #10=IFCPCURVE(#3,#7);\n#11=IFCPCURVE(#3,#8);\n#12=IFCPCURVE(#4,#7);\n\
             #20=IFCCIRCLE(#2,1.);\n\
             #21=IFCSURFACECURVE(#20,(#10),.CURVE3D.);\n\
             #22=IFCSURFACECURVE(#7,(#10),.CURVE3D.);\n\
             #23=IFCSURFACECURVE(#10,(#10),.CURVE3D.);\n\
             #24=IFCSEAMCURVE(#20,(#10,#11),.CURVE3D.);\n\
             #25=IFCSEAMCURVE(#20,(#10,#12),.CURVE3D.);\n\
             #26=IFCINTERSECTIONCURVE(#20,(#10,#12),.CURVE3D.);\n\
             #27=IFCINTERSECTIONCURVE(#20,(#10),.CURVE3D.);\n\
             #30=IFCCOMPOSITECURVESEGMENT(.CONTINUOUS.,.T.,#10);\n\
             #31=IFCCOMPOSITECURVESEGMENT(.CONTINUOUS.,.T.,#12);\n\
             #32=IFCCOMPOSITECURVESEGMENT(.DISCONTINUOUS.,.T.,#10);\n\
             #40=IFCOUTERBOUNDARYCURVE((#30,#30),.F.);\n\
             #41=IFCBOUNDARYCURVE((#30,#31),.F.);\n\
             #42=IFCBOUNDARYCURVE((#30,#32),.F.);\n\
             #43=IFCCOMPOSITECURVEONSURFACE((#30,#32),.F.);",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["DimIs2D"]);
        assert!(rules(&f, 21).is_empty());
        assert_eq!(rules(&f, 22), ["CurveIs3D"]);
        // A p-curve's Dim is its basis surface's (3): only the type rule.
        assert_eq!(rules(&f, 23), ["CurveIsNotPcurve"]);
        assert!(rules(&f, 24).is_empty());
        assert_eq!(rules(&f, 25), ["SameSurface"]);
        assert!(rules(&f, 26).is_empty());
        assert_eq!(rules(&f, 27), ["TwoPCurves", "DistinctSurfaces"]);
        assert!(rules(&f, 40).is_empty());
        assert_eq!(rules(&f, 41), ["SameSurface"]);
        assert_eq!(rules(&f, 42), ["IsClosed"]);
        assert!(rules(&f, 43).is_empty());
    }

    #[test]
    fn boolean_and_half_space_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCTRIANGULATEDFACESET(#1,$,$,((1,2,3)),$);\n\
             #4=IFCPOLYGONALFACESET(#1,.F.,(#9),$);\n\
             #5=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,1.,1.);\n#6=IFCDIRECTION((0.,0.,1.));\n\
             #7=IFCEXTRUDEDAREASOLID(#5,$,#6,1.);\n\
             #8=IFCCARTESIANPOINT((0.,0.,0.));\n#9=IFCAXIS2PLACEMENT3D(#8,$,$);\n\
             #10=IFCPLANE(#9);\n#11=IFCHALFSPACESOLID(#10,.T.);\n\
             #12=IFCCARTESIANPOINT((0.,0.));\n#13=IFCCARTESIANPOINT((1.,0.));\n\
             #14=IFCPOLYLINE((#12,#13));\n#15=IFCPOLYLINE((#8,#8));\n\
             #16=IFCCIRCLE(#9,1.);\n\
             #20=IFCBOOLEANRESULT(.UNION.,#2,#7);\n\
             #21=IFCBOOLEANRESULT(.UNION.,#3,#4);\n\
             #22=IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.,#7,#11);\n\
             #23=IFCBOOLEANCLIPPINGRESULT(.UNION.,#2,#7);\n\
             #30=IFCPOLYGONALBOUNDEDHALFSPACE(#10,.T.,#9,#14);\n\
             #31=IFCPOLYGONALBOUNDEDHALFSPACE(#10,.T.,#9,#15);\n\
             #32=IFCPOLYGONALBOUNDEDHALFSPACE(#10,.T.,#9,#16);\n\
             #40=IFCBOUNDINGBOX(#8,1.,1.,1.);\n\
             #41=IFCBOXEDHALFSPACE(#10,.T.,#40);\n\
             #42=IFCCURVEBOUNDEDPLANE(#10,#14,$);\n\
             #43=IFCBOXEDHALFSPACE(#42,.T.,#40);",
        );
        assert!(rules(&f, 20).is_empty());
        assert_eq!(rules(&f, 21), ["FirstOperandClosed", "SecondOperandClosed"]);
        assert!(rules(&f, 22).is_empty());
        assert_eq!(
            rules(&f, 23),
            ["OperatorType", "FirstOperandType", "SecondOperandType"]
        );
        assert!(rules(&f, 30).is_empty());
        assert_eq!(rules(&f, 31), ["BoundaryDim"]);
        // A 3-D circle fails both the dimension and the type rule.
        assert_eq!(rules(&f, 32), ["BoundaryDim", "BoundaryType"]);
        assert!(rules(&f, 41).is_empty());
        assert_eq!(rules(&f, 43), ["UnboundedSurface"]);
    }

    #[test]
    fn swept_disk_polygonal_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCCARTESIANPOINT((10.,0.,0.));\n\
             #3=IFCPOLYLINE((#1,#2));\n#4=IFCCIRCLE(#9,1.);\n\
             #10=IFCSWEPTDISKSOLIDPOLYGONAL(#3,1.,$,$,$,3.);\n\
             #11=IFCSWEPTDISKSOLIDPOLYGONAL(#3,2.,$,$,$,1.);\n\
             #12=IFCSWEPTDISKSOLIDPOLYGONAL(#4,1.,$,$,$,$);",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["CorrectRadii"]);
        assert_eq!(rules(&f, 12), ["DirectrixIsPolyline"]);
    }

    #[test]
    fn sectioned_solid_and_surface_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCCARTESIANPOINT((10.,0.,0.));\n\
             #3=IFCPOLYLINE((#1,#2));\n\
             #4=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,1.);\n\
             #5=IFCRECTANGLEPROFILEDEF(.CURVE.,$,$,2.,1.);\n\
             #6=IFCCIRCLEPROFILEDEF(.AREA.,$,$,1.);\n\
             #7=IFCCARTESIANPOINT((0.,0.));\n#8=IFCCARTESIANPOINT((1.,0.));\n#9=IFCPOLYLINE((#7,#8));\n\
             #10=IFCPOINTBYDISTANCEEXPRESSION(IFCNONNEGATIVELENGTHMEASURE(0.),$,$,$,#3);\n\
             #11=IFCAXIS2PLACEMENTLINEAR(#10,$,$);\n\
             #12=IFCPOINTBYDISTANCEEXPRESSION(IFCNONNEGATIVELENGTHMEASURE(10.),1.,$,2.,#3);\n\
             #13=IFCAXIS2PLACEMENTLINEAR(#12,$,$);\n\
             #20=IFCSECTIONEDSOLIDHORIZONTAL(#3,(#4,#4),(#11,#11));\n\
             #21=IFCSECTIONEDSOLIDHORIZONTAL(#3,(#4,#5),(#11,#13));\n\
             #22=IFCSECTIONEDSOLIDHORIZONTAL(#9,(#4,#6),(#11));\n\
             #30=IFCSECTIONEDSURFACE(#3,(#11,#11),(#5,#5));\n\
             #31=IFCSECTIONEDSURFACE(#3,(#11,#13),(#4,#4));\n\
             #40=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(1.,1.,0.)));\n\
             #41=IFCTRIANGULATEDIRREGULARNETWORK(#40,$,.F.,((1,2,3)),$,(0));\n\
             #42=IFCTRIANGULATEDIRREGULARNETWORK(#40,$,.T.,((1,2,3)),$,(0));",
        );
        assert!(rules(&f, 20).is_empty());
        assert_eq!(
            rules(&f, 21),
            ["ConsistentProfileTypes", "NoLongitudinalOffsets"]
        );
        assert_eq!(
            rules(&f, 22),
            [
                "DirectrixIs3D",
                "SectionsSameType",
                "CorrespondingSectionPositions"
            ]
        );
        assert!(rules(&f, 30).is_empty());
        assert_eq!(rules(&f, 31), ["AreaProfileTypes", "NoOffsets"]);
        assert!(rules(&f, 41).is_empty());
        assert_eq!(rules(&f, 42), ["NotClosed"]);
    }

    #[test]
    fn sectioned_spine_rules() {
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,4.);\n\
             #2=IFCRECTANGLEPROFILEDEF(.CURVE.,$,$,2.,4.);\n\
             #3=IFCCARTESIANPOINT((0.,0.,0.));\n#4=IFCAXIS2PLACEMENT3D(#3,$,$);\n\
             #10=IFCSECTIONEDSPINE(#9,(#1,#1),(#4,#4));\n\
             #11=IFCSECTIONEDSPINE(#9,(#1,#1),(#4));\n\
             #12=IFCSECTIONEDSPINE(#9,(#1,#2),(#4,#4));",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["CorrespondingSectionPositions"]);
        assert_eq!(rules(&f, 12), ["ConsistentProfileTypes"]);
    }

    #[test]
    fn bspline_curve_rules() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.));\n#2=IFCCARTESIANPOINT((1.,0.));\n\
             #3=IFCCARTESIANPOINT((1.,1.));\n\
             #10=IFCBSPLINECURVEWITHKNOTS(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);\n\
             #11=IFCBSPLINECURVEWITHKNOTS(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,2),(0.,1.),.UNSPECIFIED.);\n\
             #12=IFCBSPLINECURVEWITHKNOTS(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.,2.),.UNSPECIFIED.);\n\
             #13=IFCRATIONALBSPLINECURVEWITHKNOTS(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.,(1.,0.5,1.));\n\
             #14=IFCRATIONALBSPLINECURVEWITHKNOTS(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.,(1.,-0.5));",
        );
        assert!(rules(&f, 10).is_empty());
        assert_eq!(rules(&f, 11), ["ConsistentBSpline"]);
        assert_eq!(
            rules(&f, 12),
            ["ConsistentBSpline", "CorrespondingKnotLists"]
        );
        assert!(rules(&f, 13).is_empty());
        assert_eq!(
            rules(&f, 14),
            ["SameNumOfWeightsAndPoints", "WeightsGreaterZero"]
        );
    }

    #[test]
    fn georeferencing_rules() {
        let f = parse(
            "#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);\n\
             #20=IFCPROJECTEDCRS('EPSG:25832',$,$,$,$,$,$);\n\
             #21=IFCGEOGRAPHICCRS('EPSG:4326',$,$,$,$);\n\
             #30=IFCMAPCONVERSION(#10,#20,0.,0.,0.,$,$,$);\n\
             #31=IFCMAPCONVERSION(#10,#21,0.,0.,0.,$,$,$);\n\
             #32=IFCRIGIDOPERATION(#20,#21,IFCLENGTHMEASURE(1.),IFCLENGTHMEASURE(2.),$);\n\
             #33=IFCRIGIDOPERATION(#20,#21,IFCLENGTHMEASURE(1.),IFCPLANEANGLEMEASURE(2.),$);\n\
             #34=IFCRIGIDOPERATION(#20,#21,1.,2.,$);",
        );
        assert!(rules(&f, 30).is_empty());
        assert_eq!(rules(&f, 31), ["TargetCRSOnlyProjected"]);
        assert!(rules(&f, 32).is_empty());
        assert_eq!(rules(&f, 33), ["SameCoordinateType"]);
        assert_eq!(rules(&f, 34), ["SameCoordinateType"]);
    }
}
