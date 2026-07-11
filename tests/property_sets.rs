//! Phase-4 property-set suite: `IfcRelDefinesByProperties` /
//! `IfcRelDefinesByType` extraction ([`oxideav_ifc::props`]) over the
//! staged IFC 4 fixtures (buildingSMART Sample-Test-Files, CC-BY 4.0 —
//! see README "Fixtures").

use oxideav_ifc::{parse_step, Model, PropertyValue};

const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");
const BASIN: &[u8] = include_bytes!("fixtures/ifc4-basin-tessellation.ifc");

#[test]
fn wall_fixture_pset_wallcommon_resolves() {
    let f = parse_step(WALL).expect("parse");
    let m = Model::from_step(&f);

    // #60 = IFCRELDEFINESBYPROPERTIES(..., (#45), #49) hands the wall
    // its Pset_WallCommon.
    assert_eq!(m.defined_property_sets(45), &[49]);
    let psets = m.property_sets(45);
    assert_eq!(psets.len(), 1);
    let pset = &psets[0];
    assert_eq!(pset.id, 49);
    assert_eq!(pset.name, Some("Pset_WallCommon"));
    assert_eq!(pset.global_id, Some("3nMqHLyZHAegWs5Yyxh1ry"));
    assert_eq!(pset.properties.len(), 10);

    // Booleans decode through the IFCBOOLEAN wrapper.
    let is_external = pset.property("IsExternal").unwrap();
    assert_eq!(is_external.nominal().unwrap().as_bool(), Some(true));
    for name in ["Combustible", "ExtendToStructure", "LoadBearing"] {
        assert_eq!(
            pset.property(name).unwrap().nominal().unwrap().as_bool(),
            Some(false),
            "{name}"
        );
    }

    // A real measure keeps its defined-type wrapper.
    let tt = pset.property("ThermalTransmittance").unwrap();
    let v = tt.nominal().unwrap();
    assert_eq!(v.type_name(), Some("IFCTHERMALTRANSMITTANCEMEASURE"));
    assert_eq!(v.as_number(), Some(0.24));

    // The empty-label pattern the fixture writer uses.
    let fire = pset.property("FireRating").unwrap();
    assert_eq!(fire.nominal().unwrap().as_str(), Some(""));
    assert!(matches!(
        fire.value,
        PropertyValue::Single { unit: None, .. }
    ));

    // Property names are unique and all resolve.
    let names: Vec<_> = pset.properties.iter().filter_map(|p| p.name).collect();
    assert_eq!(names.len(), 10);
    assert!(names.contains(&"AcousticRating"));
    assert!(names.contains(&"SurfaceSpreadOfFlame"));
    assert!(names.contains(&"Compartmentation"));
}

#[test]
fn wall_fixture_window_pset_and_type_link() {
    let f = parse_step(WALL).expect("parse");
    let m = Model::from_step(&f);

    // The window (#102) carries Pset_WindowCommon (#113) and is typed
    // by the IfcWindowType (#107) — whose HasPropertySets is unset, so
    // no extra sets are inherited.
    assert_eq!(m.type_of(102), Some(107));
    assert_eq!(m.property_set_ids(102), vec![113]);
    let psets = m.property_sets(102);
    assert_eq!(psets.len(), 1);
    let pset = &psets[0];
    assert_eq!(pset.name, Some("Pset_WindowCommon"));
    assert_eq!(pset.properties.len(), 9);
    assert_eq!(
        pset.property("IsExternal")
            .unwrap()
            .nominal()
            .unwrap()
            .as_bool(),
        Some(true)
    );
    let infiltration = pset.property("Infiltration").unwrap();
    assert_eq!(
        infiltration.nominal().unwrap().type_name(),
        Some("IFCVOLUMETRICFLOWRATEMEASURE")
    );
    assert_eq!(infiltration.nominal().unwrap().as_number(), Some(0.3));
    assert_eq!(
        pset.property("GlazingAreaFraction")
            .unwrap()
            .nominal()
            .unwrap()
            .as_number(),
        Some(0.7)
    );

    // The wall has no type object.
    assert_eq!(m.type_of(45), None);

    // The window type is recognised as a type object; its
    // HasPropertySets is unset, so it owns no sets either.
    assert!(m.is_type_object(107));
    assert!(!m.is_type_object(102));
    assert_eq!(m.property_set_ids(107), Vec::<u64>::new());
}

#[test]
fn basin_fixture_type_link_resolves() {
    let f = parse_step(BASIN).expect("parse");
    let m = Model::from_step(&f);

    // #210 = IFCRELDEFINESBYTYPE(..., (#217), #209): the sanitary
    // terminal is typed by the IfcSanitaryTerminalType. The type's
    // HasPropertySets is $ (its (#202) aggregate is RepresentationMaps
    // at index 6), so nothing is inherited.
    assert_eq!(m.type_of(217), Some(209));
    assert_eq!(m.property_set_ids(217), Vec::<u64>::new());
    assert!(m.property_sets(217).is_empty());
    assert!(m.element_quantities(217).is_empty());
}

#[test]
fn wall_fixture_void_and_fill_graph() {
    let f = parse_step(WALL).expect("parse");
    let m = Model::from_step(&f);

    // #85 = IFCRELVOIDSELEMENT(..., #45, #80): the wall is voided by
    // the opening; #112 = IFCRELFILLSELEMENT(..., #80, #102): the
    // window fills it.
    assert_eq!(m.openings_of(45), &[80]);
    assert_eq!(m.voided_element_of(80), Some(45));
    assert_eq!(m.fillers_of(80), &[102]);
    assert_eq!(m.filled_opening_of(102), Some(80));
    // Wall → openings → fillers: the wall hosts exactly the window.
    assert_eq!(m.hosted_fillers(45), vec![102]);
    // The window hosts nothing and is voided by nothing.
    assert!(m.openings_of(102).is_empty());
    assert_eq!(m.voided_element_of(102), None);
    assert!(m.fillers_of(45).is_empty());
    assert_eq!(m.filled_opening_of(45), None);
}

#[test]
fn basin_fixture_material_falls_back_to_type() {
    let f = parse_step(BASIN).expect("parse");
    let m = Model::from_step(&f);

    // #207 = IFCRELASSOCIATESMATERIAL(..., (#209), #206) associates
    // Ceramic with the *type*; the occurrence (#217) has no direct
    // association and inherits it.
    assert_eq!(m.material_of(209), Some(206));
    assert_eq!(m.material_of(217), Some(206));
    let assignment = m.material_assignment(217).expect("material");
    assert_eq!(assignment.name(), Some("Ceramic"));
    let oxideav_ifc::MaterialAssignment::Material(mat) = assignment else {
        panic!("expected a plain material");
    };
    assert_eq!(mat.id, 206);
    assert_eq!(mat.description, None);
}
