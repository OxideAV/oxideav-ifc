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
