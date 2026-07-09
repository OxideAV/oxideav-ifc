//! Phase-2 typed-model suite: the EXPRESS schema layer
//! ([`oxideav_ifc::schema`]) resolves attribute names and the
//! spatial-structure graph over the five staged IFC 4 fixtures
//! (buildingSMART Sample-Test-Files, CC-BY 4.0 — see README "Fixtures").

use oxideav_ifc::{parse_step, EntityKind, Model, SpatialKind, TypedEntity};

const COLUMN: &[u8] = include_bytes!("fixtures/ifc4-column-straight-rectangle-tessellation.ifc");
const ITEM: &[u8] = include_bytes!("fixtures/ifc4-tessellated-item.ifc");
const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");

#[test]
fn wall_fixture_full_spatial_hierarchy() {
    let f = parse_step(WALL).expect("parse");
    let m = Model::from_step(&f);

    // Single project root.
    let project = m.project().expect("one IfcProject");
    assert_eq!(project.id(), 1);
    assert_eq!(project.kind(), EntityKind::Project);
    assert_eq!(project.global_id(), Some("28hypXUBvBefc20SI8kfA$"));
    assert_eq!(project.name(), Some("Default Project"));
    assert_eq!(
        project.description(),
        Some("Description of Default Project")
    );

    // project(#1) → site(#31) → building(#34) → storey(#38), via
    // IfcRelAggregates.
    assert_eq!(m.aggregated_children(1), &[31]);
    assert_eq!(m.aggregated_children(31), &[34]);
    assert_eq!(m.aggregated_children(34), &[38]);

    let site = m.typed(31).unwrap();
    assert_eq!(site.kind(), EntityKind::Spatial(SpatialKind::Site));
    assert_eq!(site.name(), Some("Default Site"));
    // IfcSite.RefElevation = 10.
    assert_eq!(site.attr("RefElevation").unwrap().as_number(), Some(10.0));
    // CompositionType .ELEMENT.
    assert_eq!(
        site.attr("CompositionType").unwrap().as_enum(),
        Some("ELEMENT")
    );

    let building = m.typed(34).unwrap();
    assert_eq!(building.kind(), EntityKind::Spatial(SpatialKind::Building));
    assert_eq!(building.name(), Some("Default Building"));
    assert_eq!(building.object_placement(), Some(35));

    let storey = m.typed(38).unwrap();
    assert_eq!(storey.kind(), EntityKind::Spatial(SpatialKind::Storey));
    assert_eq!(storey.attr("Elevation").unwrap().as_number(), Some(0.0));

    // Storey contains the wall (#45) and the window (#102) via
    // IfcRelContainedInSpatialStructure (#44).
    let contained = m.contained_elements(38);
    assert!(contained.contains(&45), "wall contained in storey");
    assert!(contained.contains(&102), "window contained in storey");

    // Typed wall: placement + representation references, predefined type.
    let wall = m.typed(45).unwrap();
    assert_eq!(wall.keyword(), "IFCWALL");
    assert_eq!(wall.kind(), EntityKind::Product);
    assert_eq!(wall.global_id(), Some("3ZYW59sxj8lei475l7EhLU"));
    assert_eq!(wall.name(), Some("Wall for Test Example"));
    assert_eq!(wall.object_placement(), Some(46));
    assert_eq!(wall.representation(), Some(48));

    // Typed window: OverallHeight / OverallWidth measures.
    let window = m.typed(102).unwrap();
    assert_eq!(window.keyword(), "IFCWINDOW");
    assert_eq!(
        window.attr("OverallHeight").unwrap().as_number(),
        Some(1000.0)
    );
    assert_eq!(
        window.attr("OverallWidth").unwrap().as_number(),
        Some(1000.0)
    );
    assert_eq!(window.object_placement(), Some(103));

    // The opening element is typed too (subtraction feature) and carries
    // its predefined type .OPENING.
    let opening = m.typed(80).unwrap();
    assert_eq!(opening.keyword(), "IFCOPENINGELEMENT");
    assert_eq!(opening.kind(), EntityKind::Product);
    assert_eq!(opening.predefined_type(), Some("OPENING"));

    // Spatial enumeration: exactly site + building + storey (three).
    assert_eq!(m.spatial_elements().count(), 3);
}

#[test]
fn column_fixture_contained_directly_in_site() {
    let f = parse_step(COLUMN).expect("parse");
    let m = Model::from_step(&f);

    let project = m.project().expect("one IfcProject");
    assert_eq!(project.id(), 37);
    assert_eq!(project.name(), Some("Project"));

    // project(#37) → site(#44); the column (#71) is contained directly
    // in the site, not under a storey.
    assert_eq!(m.aggregated_children(37), &[44]);
    let site = m.typed(44).unwrap();
    assert_eq!(site.kind(), EntityKind::Spatial(SpatialKind::Site));
    assert_eq!(site.name(), Some("Site #1"));

    assert_eq!(m.contained_elements(44), &[71]);
    let column = m.typed(71).unwrap();
    assert_eq!(column.keyword(), "IFCCOLUMN");
    assert_eq!(column.kind(), EntityKind::Product);
    assert_eq!(column.name(), Some("Column #1"));
    assert_eq!(column.predefined_type(), Some("COLUMN"));
    assert_eq!(column.object_placement(), Some(121));
    assert_eq!(column.representation(), Some(111));

    assert_eq!(m.products().count(), 1);
}

#[test]
fn tessellated_item_fixture_containment() {
    let f = parse_step(ITEM).expect("parse");
    let m = Model::from_step(&f);

    // #10000 IFCRELCONTAINEDINSPATIALSTRUCTURE relates (#1000) to #500.
    let rel = TypedEntity::new(f.get(10000).unwrap()).unwrap();
    assert_eq!(rel.kind(), EntityKind::RelContained);
    assert_eq!(
        rel.attr("RelatingStructure").unwrap().as_reference(),
        Some(500)
    );
    // The model exposes the same edge.
    assert_eq!(m.contained_elements(500), &[1000]);

    // #500 is the spatial-structure host (an IfcBuilding here).
    let host = m.typed(500).unwrap();
    assert!(matches!(host.kind(), EntityKind::Spatial(_)));
}

#[test]
fn column_geometry_primitive_chain_is_typed() {
    // Walk the column's product → shape → placement geometry entirely
    // through the typed schema layer (no positional indexing).
    let f = parse_step(COLUMN).expect("parse");
    let m = Model::from_step(&f);

    let column = m.typed(71).unwrap();
    let shape = m.typed(column.representation().unwrap()).unwrap(); // #111
    assert_eq!(shape.keyword(), "IFCPRODUCTDEFINITIONSHAPE");
    // IfcProductDefinitionShape.Representations → the shape reps.
    let reps = shape.attr("Representations").unwrap().as_list().unwrap();
    let rep_id = reps[0].as_reference().unwrap(); // #154

    let rep = m.typed(rep_id).unwrap();
    assert_eq!(rep.keyword(), "IFCSHAPEREPRESENTATION");
    assert_eq!(rep.kind(), EntityKind::Representation);
    assert_eq!(rep.representation_identifier(), Some("Body"));
    assert_eq!(rep.representation_type(), Some("Tessellation"));
    // #41 is the (sub)context this representation sits in.
    assert_eq!(rep.context_of_items(), Some(41));
    // One tessellated item.
    assert_eq!(rep.items().unwrap().len(), 1);

    // The placement chain: IfcLocalPlacement(#121) → RelativePlacement
    // #126 IfcAxis2Placement3D(Location #125, Axis #119, RefDirection #120).
    let lp = m.typed(column.object_placement().unwrap()).unwrap(); // #121
    assert_eq!(lp.keyword(), "IFCLOCALPLACEMENT");
    let a2p_id = lp
        .attr("RelativePlacement")
        .unwrap()
        .as_reference()
        .unwrap();
    let a2p = m.typed(a2p_id).unwrap(); // #126
    assert_eq!(a2p.keyword(), "IFCAXIS2PLACEMENT3D");
    assert_eq!(a2p.kind(), EntityKind::Geometry);

    // Location is the column's placed origin (432, 288, 48).
    let loc = m.typed(a2p.location().unwrap()).unwrap(); // #125
    assert_eq!(loc.keyword(), "IFCCARTESIANPOINT");
    assert_eq!(loc.coordinates(), Some(vec![432.0, 288.0, 48.0]));

    // Axis = +Z, RefDirection = +X.
    let axis = m.typed(a2p.axis().unwrap()).unwrap(); // #119
    assert_eq!(axis.direction_ratios(), Some(vec![0.0, 0.0, 1.0]));
    let refdir = m.typed(a2p.ref_direction().unwrap()).unwrap(); // #120
    assert_eq!(refdir.direction_ratios(), Some(vec![1.0, 0.0, 0.0]));
}

#[test]
fn wall_axis_polyline_is_typed() {
    // The wall body carries an "Axis" curve representation whose item is
    // an IfcPolyline of two IfcCartesianPoints — fully typed here.
    let f = parse_step(WALL).expect("parse");
    let m = Model::from_step(&f);

    // Find any IfcPolyline in the model via the typed layer and verify
    // its points resolve to typed cartesian points.
    let polyline = f
        .instances
        .values()
        .filter_map(TypedEntity::new)
        .find(|e| e.keyword() == "IFCPOLYLINE")
        .expect("a polyline in the wall fixture");
    assert_eq!(polyline.kind(), EntityKind::Geometry);
    let pts = polyline.points().expect("polyline points");
    assert!(pts.len() >= 2, "a polyline has at least two points");
    for pid in pts {
        let p = m.typed(pid).expect("point in typed slice");
        assert_eq!(p.keyword(), "IFCCARTESIANPOINT");
        let coords = p.coordinates().expect("coordinates");
        assert!(
            coords.len() == 2 || coords.len() == 3,
            "cartesian point is 2-D or 3-D"
        );
    }
}

#[test]
fn wall_fixture_length_unit_is_millimetres() {
    // The wall fixture's project assigns IFCSIUNIT(*, .LENGTHUNIT.,
    // .MILLI., .METRE.) — 10⁻³ metres per model unit.
    let f = parse_step(WALL).expect("parse");
    assert_eq!(oxideav_ifc::length_unit_scale(&f), Some(1e-3));
}

#[test]
fn column_fixture_length_unit_is_inches() {
    // The column fixture assigns IFCCONVERSIONBASEDUNIT('inch') whose
    // ConversionFactor is IFCMEASUREWITHUNIT(IFCLENGTHMEASURE(0.0254),
    // <SI metre>) — the real-fixture conversion-based path.
    let f = parse_step(COLUMN).expect("parse");
    let s = oxideav_ifc::length_unit_scale(&f).expect("scale");
    assert!((s - 0.0254).abs() < 1e-12);
}

#[test]
fn conversion_based_length_unit_resolves_through_si_base() {
    // A conversion-based unit (0.3048 of a metre) expressed through
    // IfcMeasureWithUnit over an SI metre.
    let text = b"ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('u.ifc','2026-07-09T00:00:00',('a'),('o'),'p','s','auth');
FILE_SCHEMA(('IFC4'));
ENDSEC;
DATA;
#1=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);
#2=IFCMEASUREWITHUNIT(IFCRATIOMEASURE(0.3048),#1);
#3=IFCCONVERSIONBASEDUNIT(*,.LENGTHUNIT.,'FOOT',#2);
#4=IFCUNITASSIGNMENT((#3));
#5=IFCPROJECT('x',$,$,$,$,$,$,$,#4);
ENDSEC;
END-ISO-10303-21;
";
    let f = parse_step(text).expect("parse");
    let s = oxideav_ifc::length_unit_scale(&f).expect("scale");
    assert!((s - 0.3048).abs() < 1e-12);
}

#[test]
fn model_without_units_has_no_length_scale() {
    let text = b"ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('u.ifc','2026-07-09T00:00:00',('a'),('o'),'p','s','auth');
FILE_SCHEMA(('IFC4'));
ENDSEC;
DATA;
#5=IFCPROJECT('x',$,$,$,$,$,$,$,$);
ENDSEC;
END-ISO-10303-21;
";
    let f = parse_step(text).expect("parse");
    assert_eq!(oxideav_ifc::length_unit_scale(&f), None);
}
