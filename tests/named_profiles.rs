//! Named parameterised profiles, tapered sweeps and georeferencing over
//! a synthetic IFC 4 model authored from the staged schema: an
//! `IfcBeam` whose body is an `IfcIShapeProfileDef` extrusion laid
//! along +x, and an `IfcColumn` whose body is an
//! `IfcExtrudedAreaSolidTapered` over an `IfcUShapeProfileDef`, in a
//! millimetre project georeferenced by an `IfcMapConversion`.
//! Exercises the std-only API (no `registry` feature needed) plus one
//! registry decode.

use oxideav_ifc::{
    map_conversion, mesh_from_product_shape, parse_step, placement_transform, TriMesh,
};

const MODEL: &str = "ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
FILE_NAME('named-profiles.ifc','2026-08-30T00:00:00',('oxideav'),('oxideav'),'','','');
FILE_SCHEMA(('IFC4'));
ENDSEC;
DATA;
#1=IFCPROJECT('0000000000000000000001',$,'Steelwork',$,$,$,$,(#10),#20);
#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,#11,$);
#11=IFCAXIS2PLACEMENT3D(#12,$,$);
#12=IFCCARTESIANPOINT((0.,0.,0.));
#20=IFCUNITASSIGNMENT((#21,#22));
#21=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);
#22=IFCSIUNIT(*,.PLANEANGLEUNIT.,$,.RADIAN.);
#30=IFCPROJECTEDCRS('EPSG:25832','ETRS89 / UTM zone 32N','ETRS89','DHHN2016','UTM','32N',#31);
#31=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);
#32=IFCMAPCONVERSION(#10,#30,400000.,5600000.,110.,0.8,0.6,0.001);
#100=IFCLOCALPLACEMENT($,#101);
#101=IFCAXIS2PLACEMENT3D(#102,$,$);
#102=IFCCARTESIANPOINT((1000.,2000.,0.));
#110=IFCBEAM('0000000000000000000002',$,'Beam-1',$,$,#100,#111,$,.BEAM.);
#111=IFCPRODUCTDEFINITIONSHAPE($,$,(#112));
#112=IFCSHAPEREPRESENTATION(#10,'Body','SweptSolid',(#113));
#113=IFCEXTRUDEDAREASOLID(#114,#115,#118,3000.);
#114=IFCISHAPEPROFILEDEF(.AREA.,'IPE200',$,100.,200.,5.6,8.5,12.,$,$);
#115=IFCAXIS2PLACEMENT3D(#116,#117,#119);
#116=IFCCARTESIANPOINT((0.,0.,0.));
#117=IFCDIRECTION((1.,0.,0.));
#118=IFCDIRECTION((0.,0.,1.));
#119=IFCDIRECTION((0.,0.,1.));
#200=IFCLOCALPLACEMENT($,#201);
#201=IFCAXIS2PLACEMENT3D(#202,$,$);
#202=IFCCARTESIANPOINT((5000.,0.,0.));
#210=IFCCOLUMN('0000000000000000000003',$,'Column-1',$,$,#200,#211,$,.COLUMN.);
#211=IFCPRODUCTDEFINITIONSHAPE($,$,(#212));
#212=IFCSHAPEREPRESENTATION(#10,'Body','AdvancedSweptSolid',(#213));
#213=IFCEXTRUDEDAREASOLIDTAPERED(#214,$,#118,4000.,#215);
#214=IFCUSHAPEPROFILEDEF(.AREA.,'UPE300',$,300.,100.,9.5,15.,15.,$,$);
#215=IFCUSHAPEPROFILEDEF(.AREA.,'UPE200',$,200.,80.,6.,11.,13.,$,$);
ENDSEC;
END-ISO-10303-21;
";

fn bbox(m: &TriMesh) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &m.positions {
        for a in 0..3 {
            lo[a] = lo[a].min(p[a]);
            hi[a] = hi[a].max(p[a]);
        }
    }
    (lo, hi)
}

#[test]
fn beam_with_i_profile_extrudes_along_x() {
    let f = parse_step(MODEL.as_bytes()).expect("parse");
    let m = mesh_from_product_shape(&f, 111).expect("beam body");
    // The solid Position maps profile z → world x, so the beam runs
    // 3000 along +x; the profile (100 wide × 200 deep) spans y / z.
    let (lo, hi) = bbox(&m);
    assert!(lo[0].abs() < 1e-9 && (hi[0] - 3000.0).abs() < 1e-9);
    assert!((hi[1] - lo[1] - 100.0).abs() < 1e-9 || (hi[2] - lo[2] - 100.0).abs() < 1e-9);
    assert!((hi[1] - lo[1] - 200.0).abs() < 1e-9 || (hi[2] - lo[2] - 200.0).abs() < 1e-9);
    // Volume = section area × length; the r1 = 12 fillets add
    // 4·(1 − π/4)·144 to the sharp area 2·100·8.5 + 183·5.6.
    let sharp = 2.0 * 100.0 * 8.5 + 183.0 * 5.6;
    let area = sharp + 4.0 * (1.0 - core::f64::consts::FRAC_PI_4) * 144.0;
    let expect = area * 3000.0;
    assert!(
        (m.signed_volume().abs() - expect).abs() < expect * 2e-3,
        "beam volume {} vs {expect}",
        m.signed_volume()
    );
}

#[test]
fn column_with_tapered_u_profile() {
    let f = parse_step(MODEL.as_bytes()).expect("parse");
    let m = mesh_from_product_shape(&f, 211).expect("column body");
    let (lo, hi) = bbox(&m);
    assert!(lo[2].abs() < 1e-9 && (hi[2] - 4000.0).abs() < 1e-9);
    // Base section is 100 × 300, top section 80 × 200: the bbox is the
    // base's, the top ring is the smaller one.
    assert!((hi[0] - lo[0] - 100.0).abs() < 1e-9 && (hi[1] - lo[1] - 300.0).abs() < 1e-9);
    let top: Vec<&[f64; 3]> = m
        .positions
        .iter()
        .filter(|p| (p[2] - 4000.0).abs() < 1e-9)
        .collect();
    assert!(!top.is_empty());
    assert!(top
        .iter()
        .all(|p| p[0].abs() <= 40.0 + 1e-9 && p[1].abs() <= 100.0 + 1e-9));
    // The loft's volume lies strictly between the two prisms.
    let k = 1.0 - core::f64::consts::FRAC_PI_4;
    let base = 2.0 * 100.0 * 15.0 + 270.0 * 9.5 + 2.0 * k * 225.0;
    let top_a = 2.0 * 80.0 * 11.0 + 178.0 * 6.0 + 2.0 * k * 169.0;
    let v = m.signed_volume().abs();
    assert!(v < base * 4000.0 && v > top_a * 4000.0, "{v}");
}

#[test]
fn placed_bodies_reach_map_coordinates() {
    let f = parse_step(MODEL.as_bytes()).expect("parse");
    assert_eq!(oxideav_ifc::length_unit_scale(&f), Some(1e-3));
    let conv = map_conversion(&f).expect("georeferenced");
    assert_eq!(conv.target_crs.as_ref().unwrap().name, Some("EPSG:25832"));
    // Beam: world placement (1000, 2000, 0); its local origin maps
    // through scale 0.001 (mm → m) and the 3-4-5 rotation.
    let world = placement_transform(&f, 100).expect("placement");
    let body = mesh_from_product_shape(&f, 111)
        .unwrap()
        .transformed(&world)
        .transformed(&conv.transform());
    let origin = conv.to_map([1000.0, 2000.0, 0.0]);
    assert!((origin[0] - (400000.0 + 0.8 - 1.2)).abs() < 1e-9);
    assert!((origin[1] - (5600000.0 + 0.6 + 1.6)).abs() < 1e-9);
    assert!((origin[2] - 110.0).abs() < 1e-9);
    // Every mapped vertex lies within 3.5 m of the mapped origin (the
    // beam is 3 m long, 0.2 m deep) and heights are scaled to metres.
    for p in &body.positions {
        let d = ((p[0] - origin[0]).powi(2) + (p[1] - origin[1]).powi(2)).sqrt();
        assert!(d <= 3.5, "{d}");
        assert!((109.8..=110.2).contains(&p[2]), "{}", p[2]);
    }
}

#[cfg(feature = "registry")]
#[test]
fn registry_decodes_named_profile_products() {
    use oxideav_mesh3d::Mesh3DDecoder;
    let mut decoder = oxideav_ifc::make_decoder();
    let scene = decoder.decode(MODEL.as_bytes()).expect("decodes");
    assert_eq!(scene.meshes.len(), 2, "beam + column bodies");
    let names: Vec<String> = scene.nodes.iter().filter_map(|n| n.name.clone()).collect();
    assert!(names.iter().any(|n| n == "Beam-1"), "{names:?}");
    assert!(names.iter().any(|n| n == "Column-1"), "{names:?}");
    assert!(scene.triangle_count() > 40);
}
