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
