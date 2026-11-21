//! IFC 4.3 alignment curves: curve segments over lines, circles,
//! clothoids and cosine spirals, gradient curves, offset curves by
//! distances, and the solids swept along them.
//!
//! The reference numbers (segment end points, headings, radii and
//! elevations) are the values of the Annex E alignment listings of the
//! IFC 4.3 specification; the models themselves are authored here.

use oxideav_ifc::{curve_points, parse_step, tessellate_item};

fn parse(data: &str) -> oxideav_ifc::StepFile {
    parse_step(
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('IFC4X3_ADD2'));\nENDSEC;\n\
             DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
        .as_bytes(),
    )
    .expect("parses")
}

/// A horizontal alignment — 400 straight, a 150 clothoid easing into a
/// right-hand radius of 500, then 400 of that arc — as #54, and its
/// gradient curve (a fall of 1 ‰, a sag curve of radius 69 230.8, a
/// rise of 0.444 ‰) as #79.
const ALIGNMENT: &str = "\
#56=IFCCARTESIANPOINT((0.,0.));\n#57=IFCDIRECTION((1.,0.));\n#58=IFCAXIS2PLACEMENT2D(#56,#57);\n\
#61=IFCVECTOR(#57,1.);\n#59=IFCLINE(#56,#61);\n\
#55=IFCCURVESEGMENT(.CONTSAMEGRADIENTSAMECURVATURE.,#58,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(400.),#59);\n\
#64=IFCCARTESIANPOINT((400.,0.));\n#66=IFCAXIS2PLACEMENT2D(#64,#57);\n\
#67=IFCCLOTHOID(#58,-273.861278752584);\n\
#63=IFCCURVESEGMENT(.CONTSAMEGRADIENTSAMECURVATURE.,#66,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(150.),#67);\n\
#72=IFCCARTESIANPOINT((549.662851380011,-7.48795505445));\n\
#73=IFCDIRECTION((0.988771077936042,-0.149438132473604));\n#74=IFCAXIS2PLACEMENT2D(#72,#73);\n\
#75=IFCCIRCLE(#58,500.000000000002);\n\
#71=IFCCURVESEGMENT(.DISCONTINUOUS.,#74,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(-400.),#75);\n\
#54=IFCCOMPOSITECURVE((#55,#63,#71),.U.);\n\
#81=IFCCARTESIANPOINT((0.,150.));\n#82=IFCDIRECTION((0.999999500000375,-0.000999999499995919));\n\
#83=IFCAXIS2PLACEMENT2D(#81,#82);\n\
#80=IFCCURVESEGMENT(.CONTSAMEGRADIENTSAMECURVATURE.,#83,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(450.000218741065),#59);\n\
#89=IFCCARTESIANPOINT((449.999993741124,149.550000006261));\n#91=IFCAXIS2PLACEMENT2D(#89,#82);\n\
#92=IFCCIRCLE(#58,69230.7996321627);\n\
#88=IFCCURVESEGMENT(.CONTSAMEGRADIENTSAMECURVATURE.,#91,IFCLENGTHMEASURE(4.71138898071803),IFCLENGTHMEASURE(100.00001881),#92);\n\
#97=IFCCARTESIANPOINT((550.,149.522222225005));\n#98=IFCDIRECTION((0.999999901234583,0.000444444400554072));\n\
#99=IFCAXIS2PLACEMENT2D(#97,#98);\n\
#96=IFCCURVESEGMENT(.DISCONTINUOUS.,#99,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(400.000039506171),#59);\n\
#79=IFCGRADIENTCURVE((#80,#88,#96),.U.,#54,$);\n";

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn clothoid_transition_reaches_the_listed_point_and_radius() {
    let f = parse(ALIGNMENT);
    let pts = curve_points(&f, 54).unwrap();
    assert!(pts.iter().all(|p| p[2] == 0.0));
    // The straight ends at (400, 0); the clothoid at the listed start
    // of the arc.
    assert!(
        pts.iter()
            .any(|p| near(p[0], 400.0, 1e-9) && near(p[1], 0.0, 1e-9))
    );
    let join = pts
        .iter()
        .find(|p| near(p[0], 549.662851380011, 1e-6))
        .expect("the clothoid's end point");
    assert!(near(join[1], -7.48795505445, 1e-6), "{join:?}");
    // Every later point lies on the right-hand circle of radius 500
    // tangent there: centre = join + 500 · (right of the heading).
    let (hx, hy) = (0.988771077936042, -0.149438132473604);
    let centre = [join[0] + 500.0 * hy, join[1] - 500.0 * hx];
    let on_arc: Vec<_> = pts.iter().filter(|p| p[0] > join[0] + 1e-6).collect();
    assert!(on_arc.len() >= 16);
    for p in &on_arc {
        let r = (p[0] - centre[0]).hypot(p[1] - centre[1]);
        assert!(near(r, 500.0, 1e-6), "{p:?}: {r}");
    }
    // 400 of arc at radius 500 turn the heading by 0.8 rad to the right.
    let last = pts[pts.len() - 1];
    let end_angle = (last[1] - centre[1]).atan2(last[0] - centre[0]);
    let start_angle = (join[1] - centre[1]).atan2(join[0] - centre[0]);
    assert!(near(start_angle - end_angle, 0.8, 1e-9));
}

#[test]
fn gradient_curve_lifts_the_base_curve_to_the_profile() {
    let f = parse(ALIGNMENT);
    let pts = curve_points(&f, 79).unwrap();
    // Starts 150 above the base curve's origin.
    assert!(near(pts[0][0], 0.0, 1e-9) && near(pts[0][2], 150.0, 1e-9));
    // The straight grade, the start of the sag curve and its end sit
    // at the listed elevations; the plan position is the base curve's.
    let height_at_x = |x: f64| {
        pts.iter()
            .find(|p| near(p[0], x, 1e-6))
            .unwrap_or_else(|| panic!("no vertex at x = {x}"))[2]
    };
    assert!(near(height_at_x(400.0), 150.0 - 0.4, 1e-6));
    let plan = curve_points(&f, 54).unwrap();
    // Distance 450 lies 50 into the clothoid; 550 is its end.
    let at_550 = pts
        .iter()
        .find(|p| near(p[0], 549.662851380011, 1e-6))
        .expect("vertex at the clothoid end");
    assert!(near(at_550[2], 149.522222225005, 1e-6), "{at_550:?}");
    assert!(near(at_550[1], -7.48795505445, 1e-6));
    // The lowest point of the sag is below both of its ends.
    let low = pts.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
    assert!(low < 149.522222225005 && low > 149.4);
    // The curve ends where the base curve does (distance 950).
    let (a, b) = (pts[pts.len() - 1], plan[plan.len() - 1]);
    assert!(near(a[0], b[0], 1e-9) && near(a[1], b[1], 1e-9));
    assert!(near(
        a[2],
        149.522222225005 + 400.0 * 0.000444444444449813,
        1e-5
    ));
}

#[test]
fn cosine_spiral_runs_between_the_listed_radii() {
    // Radius 300 easing to 1000 over 100: the listed end point, and a
    // heading change of 100 / 461.538… = 0.21667 rad (the start
    // direction of the following tangent).
    let f = parse(
        "#1=IFCCARTESIANPOINT((0.,0.));\n#2=IFCDIRECTION((1.,0.));\n#3=IFCAXIS2PLACEMENT2D(#1,#2);\n\
         #4=IFCCOSINESPIRAL(#3,857.142857142857,461.538461538462);\n\
         #5=IFCCURVESEGMENT(.CONTINUOUS.,#3,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(100.),#4);\n\
         #6=IFCCOMPOSITECURVE((#5),.F.);\n\
         #7=IFCCURVESEGMENT(.CONTINUOUS.,#3,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(-100.),#4);\n\
         #8=IFCCOMPOSITECURVE((#7),.F.);",
    );
    let pts = curve_points(&f, 6).unwrap();
    let end = pts[pts.len() - 1];
    assert!(near(end[0], 98.9298874325988, 1e-5), "{end:?}");
    assert!(near(end[1], 13.1346246945757, 1e-5), "{end:?}");
    let before = pts[pts.len() - 2];
    let heading = (end[1] - before[1]).atan2(end[0] - before[0]);
    // The last chord's direction trails the end heading by half a step.
    assert!(heading > 0.19 && heading < 0.216666666666666, "{heading}");
    // The segment on its own is the same run.
    assert_eq!(curve_points(&f, 5).unwrap(), pts);
    // Run against the spiral's sense, the same shape bends the other way.
    let back = curve_points(&f, 8).unwrap();
    let end_back = back[back.len() - 1];
    assert!(near(end_back[0], end[0], 1e-9) && near(end_back[1], -end[1], 1e-9));
}

#[test]
fn spirals_without_a_staged_definition_are_refused_by_name() {
    let f = parse(
        "#1=IFCCARTESIANPOINT((0.,0.));\n#2=IFCDIRECTION((1.,0.));\n#3=IFCAXIS2PLACEMENT2D(#1,#2);\n\
         #4=IFCSINESPIRAL(#3,300.,$,$);\n\
         #5=IFCCURVESEGMENT(.CONTINUOUS.,#3,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(100.),#4);\n\
         #6=IFCCOMPOSITECURVE((#5),.F.);\n\
         #7=IFCSEGMENTEDREFERENCECURVE((#5),.F.,#6,$);",
    );
    for (id, keyword) in [(6, "IFCSINESPIRAL"), (7, "IFCSEGMENTEDREFERENCECURVE")] {
        assert_eq!(
            curve_points(&f, id).unwrap_err(),
            oxideav_ifc::GeometryError::Unsupported(keyword.to_string())
        );
    }
}

#[test]
fn offset_curve_by_distances_moves_left_and_up() {
    // A straight basis along +x (a gradient curve at height 5), offset
    // 2 to the left at distance 10, widening to 4 at distance 30, and
    // raised by 1 throughout.
    let f = parse(
        "#1=IFCCARTESIANPOINT((0.,0.));\n#2=IFCDIRECTION((1.,0.));\n#3=IFCAXIS2PLACEMENT2D(#1,#2);\n\
         #4=IFCVECTOR(#2,1.);\n#5=IFCLINE(#1,#4);\n\
         #6=IFCCURVESEGMENT(.DISCONTINUOUS.,#3,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(40.),#5);\n\
         #7=IFCCOMPOSITECURVE((#6),.F.);\n\
         #8=IFCCARTESIANPOINT((0.,5.));\n#9=IFCAXIS2PLACEMENT2D(#8,#2);\n\
         #10=IFCCURVESEGMENT(.DISCONTINUOUS.,#9,IFCLENGTHMEASURE(0.),IFCLENGTHMEASURE(40.),#5);\n\
         #11=IFCGRADIENTCURVE((#10),.F.,#7,$);\n\
         #12=IFCPOINTBYDISTANCEEXPRESSION(IFCLENGTHMEASURE(10.),2.,1.,$,#11);\n\
         #13=IFCPOINTBYDISTANCEEXPRESSION(IFCLENGTHMEASURE(30.),4.,1.,$,#11);\n\
         #14=IFCOFFSETCURVEBYDISTANCES(#11,(#12,#13),$);\n\
         #15=IFCOFFSETCURVEBYDISTANCES(#11,(#12),$);",
    );
    let pts = curve_points(&f, 14).unwrap();
    assert_eq!(pts.first().copied(), Some([10.0, 2.0, 6.0]));
    assert_eq!(pts.last().copied(), Some([30.0, 4.0, 6.0]));
    // One value offsets the whole basis curve.
    let whole = curve_points(&f, 15).unwrap();
    assert_eq!(whole.first().copied(), Some([0.0, 2.0, 6.0]));
    assert_eq!(whole.last().copied(), Some([40.0, 2.0, 6.0]));
}

#[test]
fn solids_follow_an_alignment_by_its_stations() {
    // A 2 × 1 rectangle swept between distances 300 and 600 of the
    // gradient curve: across the straight, the whole clothoid and 50
    // of the arc. The swept volume is area × length up to the chord
    // error of the arc (the gradient is 1 ‰ at most).
    let src = format!(
        "{ALIGNMENT}\
         #200=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,1.);\n#201=IFCDIRECTION((0.,0.,1.));\n\
         #202=IFCFIXEDREFERENCESWEPTAREASOLID(#200,$,#79,IFCLENGTHMEASURE(300.),IFCLENGTHMEASURE(600.),#201);\n\
         #210=IFCPOINTBYDISTANCEEXPRESSION(IFCLENGTHMEASURE(100.),$,$,$,#79);\n\
         #211=IFCAXIS2PLACEMENTLINEAR(#210,$,$);\n\
         #212=IFCPOINTBYDISTANCEEXPRESSION(IFCLENGTHMEASURE(500.),$,$,$,#79);\n\
         #213=IFCAXIS2PLACEMENTLINEAR(#212,$,$);\n\
         #214=IFCSECTIONEDSOLIDHORIZONTAL(#79,(#200,#200),(#211,#213));"
    );
    let f = parse(&src);
    let sweep = tessellate_item(&f, 202).unwrap();
    let v = sweep.signed_volume();
    assert!(v > 0.0 && (v - 600.0).abs() < 0.5, "{v}");
    let (lo, hi) = sweep
        .positions
        .iter()
        .fold((f64::INFINITY, 0.0f64), |(lo, hi), p| {
            (lo.min(p[0]), hi.max(p[0]))
        });
    // Starts on the straight at x = 300, ends 50 into the arc.
    // (The section is square to the 1 ‰ grade, so its corners lean by
    // a millimetre.)
    assert!((lo - 300.0).abs() < 2e-3, "{lo}");
    assert!(hi > 598.0 && hi < 600.0, "{hi}");
    // A sectioned solid between distances 100 and 500: 400 long, its
    // far end 100 into the clothoid.
    let loft = tessellate_item(&f, 214).unwrap();
    let v = loft.signed_volume();
    assert!((v.abs() - 800.0).abs() < 0.5, "{v}");
    let far = loft.positions.iter().map(|p| p[0]).fold(0.0f64, f64::max);
    assert!(far > 499.5 && far < 501.5, "{far}");
}

#[test]
fn fill_areas_geometric_sets_and_trimmed_lines() {
    let f = parse(
        "#1=IFCCARTESIANPOINT((0.,0.));\n#2=IFCCARTESIANPOINT((4.,0.));\n\
         #3=IFCCARTESIANPOINT((4.,3.));\n#4=IFCCARTESIANPOINT((0.,3.));\n\
         #5=IFCPOLYLINE((#1,#2,#3,#4,#1));\n\
         #6=IFCCARTESIANPOINT((1.,1.));\n#7=IFCCARTESIANPOINT((2.,1.));\n\
         #8=IFCCARTESIANPOINT((2.,2.));\n#9=IFCCARTESIANPOINT((1.,2.));\n\
         #10=IFCPOLYLINE((#6,#7,#8,#9,#6));\n\
         #11=IFCANNOTATIONFILLAREA(#5,(#10));\n\
         #12=IFCGEOMETRICSET((#5,#11));\n#13=IFCGEOMETRICCURVESET((#5));\n\
         #20=IFCCARTESIANPOINT((0.,0.,0.));\n#21=IFCDIRECTION((0.,0.,1.));\n\
         #22=IFCVECTOR(#21,2.);\n#23=IFCLINE(#20,#22);\n\
         #24=IFCTRIMMEDCURVE(#23,(IFCPARAMETERVALUE(1.)),(IFCPARAMETERVALUE(4.)),.T.,.PARAMETER.);\n\
         #25=IFCSWEPTDISKSOLID(#24,0.5,$,$,$);",
    );
    // A 4 × 3 fill with a unit hole, facing +z.
    let fill = tessellate_item(&f, 11).unwrap();
    let area: f64 = fill
        .triangles
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| fill.positions[i as usize]);
            0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        })
        .sum();
    assert!((area - 11.0).abs() < 1e-12, "{area}");
    // The set meshes its fill area; a set of curves has no surface.
    assert_eq!(
        tessellate_item(&f, 12).unwrap().triangle_count(),
        fill.triangle_count()
    );
    assert_eq!(
        tessellate_item(&f, 13).unwrap_err(),
        oxideav_ifc::GeometryError::Unsupported("IFCPOLYLINE".to_string())
    );
    assert_eq!(curve_points(&f, 5).unwrap().len(), 5);
    // A line trimmed by parameters 1 … 4 of a magnitude-2 vector runs
    // from z = 2 to z = 8.
    assert_eq!(
        curve_points(&f, 24).unwrap(),
        vec![[0.0, 0.0, 2.0], [0.0, 0.0, 8.0]]
    );
    let rod = tessellate_item(&f, 25).unwrap();
    let (lo, hi) = rod
        .positions
        .iter()
        .fold((f64::INFINITY, 0.0f64), |(lo, hi), p| {
            (lo.min(p[2]), hi.max(p[2]))
        });
    assert_eq!((lo, hi), (2.0, 8.0));
}

#[test]
fn two_dimensional_operator_axes_follow_ifc_base_axis() {
    // A derived profile turned by an operator that gives Axis1 only:
    // (0, −1) maps profile x to −y, and U2 is its orthogonal
    // complement (1, 0) — a quarter turn clockwise, no mirror. The
    // 4 × 2 rectangle offset to x ∈ [1, 5] lands on y ∈ [−5, −1].
    let f = parse(
        "#1=IFCCARTESIANPOINT((3.,0.));\n#2=IFCAXIS2PLACEMENT2D(#1,$);\n\
         #3=IFCRECTANGLEPROFILEDEF(.AREA.,$,#2,4.,2.);\n\
         #4=IFCDIRECTION((0.,-1.));\n#5=IFCCARTESIANPOINT((0.,0.));\n\
         #6=IFCCARTESIANTRANSFORMATIONOPERATOR2D(#4,$,#5,$);\n\
         #7=IFCDERIVEDPROFILEDEF(.AREA.,$,#3,#6,$);\n\
         #8=IFCDIRECTION((0.,0.,1.));\n#9=IFCEXTRUDEDAREASOLID(#7,$,#8,1.);\n\
         #10=IFCDIRECTION((0.,1.));\n\
         #11=IFCCARTESIANTRANSFORMATIONOPERATOR2D($,#10,#5,$);\n\
         #12=IFCDERIVEDPROFILEDEF(.AREA.,$,#3,#11,$);\n#13=IFCEXTRUDEDAREASOLID(#12,$,#8,1.);",
    );
    let bounds = |id: u64| {
        let m = tessellate_item(&f, id).unwrap();
        assert!((m.signed_volume() - 8.0).abs() < 1e-12);
        m.positions.iter().fold(
            ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
            |(lo, hi), p| {
                (
                    [lo[0].min(p[0]), lo[1].min(p[1])],
                    [hi[0].max(p[0]), hi[1].max(p[1])],
                )
            },
        )
    };
    assert_eq!(bounds(9), ([-1.0, -5.0], [1.0, -1.0]));
    // Axis2 only, along +y: the identity.
    assert_eq!(bounds(13), ([1.0, -1.0], [5.0, 1.0]));
}

#[test]
fn hostile_reference_lattices_are_not_walked_path_by_path() {
    // Forty levels, each referencing the next four times and never
    // reaching a point: a per-path search would visit 4^32 instances.
    let mut src = String::new();
    for level in 1..=40u64 {
        let next = level + 1;
        src.push_str(&format!(
            "#{level}=IFCCOMPOSITECURVE((#{next},#{next},#{next},#{next}),.F.);\n"
        ));
    }
    src.push_str("#41=IFCDIRECTION((1.,0.));");
    let f = parse(&src);
    for id in 1..=41 {
        assert!(curve_points(&f, id).is_err());
        assert!(tessellate_item(&f, id).is_err());
    }
}
