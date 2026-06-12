//! Fixture suite: the five small IFC 4 sample models under
//! `tests/fixtures/` (buildingSMART Sample-Test-Files, CC-BY 4.0 —
//! see README "Fixtures" for attribution) must parse completely, with
//! exact instance counts, schema identification, and spot-checked
//! entities.

use oxideav_ifc::{parse_step, probe_step, StepFile, Value};

const BASIN: &[u8] = include_bytes!("fixtures/ifc4-basin-tessellation.ifc");
const COLUMN: &[u8] = include_bytes!("fixtures/ifc4-column-straight-rectangle-tessellation.ifc");
const ITEM: &[u8] = include_bytes!("fixtures/ifc4-tessellated-item.ifc");
const COLORS: &[u8] = include_bytes!("fixtures/ifc4-tessellation-with-individual-colors.ifc");
const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");

fn parse_checked(bytes: &[u8], expected_instances: usize) -> StepFile {
    assert!(probe_step(bytes));
    let file = parse_step(bytes).expect("fixture must parse");
    assert_eq!(file.len(), expected_instances, "instance count");
    assert_eq!(file.header.file_schema, ["IFC4"], "FILE_SCHEMA");
    assert!(
        file.dangling_references().is_empty(),
        "fixtures are self-contained"
    );
    file
}

#[test]
fn basin_tessellation() {
    let f = parse_checked(BASIN, 44);
    // #201 is the basin mesh: IFCTRIANGULATEDFACESET(#200,$,.T.,((28,2,29),...
    let mesh = f.get(201).expect("#201");
    assert_eq!(mesh.keyword, "IFCTRIANGULATEDFACESET");
    assert_eq!(mesh.args[0], Value::Reference(200));
    assert_eq!(mesh.args[1], Value::Unset);
    assert_eq!(mesh.args[2], Value::Enum("T".into()));
    let coord_index = mesh.args[3].as_list().expect("CoordIndex aggregate");
    assert_eq!(
        coord_index[0],
        Value::List(vec![
            Value::Integer(28),
            Value::Integer(2),
            Value::Integer(29),
        ])
    );
    assert_eq!(f.instances_of("IfcTriangulatedFaceSet").count(), 1);
}

#[test]
fn column_straight_rectangle() {
    let f = parse_checked(COLUMN, 26);
    assert_eq!(f.header.file_name.author, ["Tim Chipman"]);
    assert_eq!(f.header.file_name.organization, [""]);
    assert_eq!(f.instances_of("IfcColumn").count(), 1);
    assert_eq!(f.instances_of("IfcTriangulatedFaceSet").count(), 1);
}

#[test]
fn tessellated_item() {
    let f = parse_checked(ITEM, 29);
    // Last record: spatial containment relationship.
    let rel = f.get(10000).expect("#10000");
    assert_eq!(rel.keyword, "IFCRELCONTAINEDINSPATIALSTRUCTURE");
    assert_eq!(rel.args[0], Value::String("2TnxZkTXT08eDuMuhUUFNy".into()));
    assert_eq!(rel.args[4], Value::List(vec![Value::Reference(1000)]));
    assert_eq!(rel.args[5], Value::Reference(500));
    // The relationship's element is reachable, cycle-safely.
    assert!(f.reachable_from(10000).contains(&1000));
}

#[test]
fn tessellation_with_individual_colors() {
    let f = parse_checked(COLORS, 32);
    // #9 = IFCCARTESIANPOINT((0.0,0.0,0.0));
    let origin = f.get(9).expect("#9");
    assert_eq!(origin.keyword, "IFCCARTESIANPOINT");
    assert_eq!(
        origin.args[0],
        Value::List(vec![Value::Real(0.0), Value::Real(0.0), Value::Real(0.0)])
    );
    // #102 = IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.); — `*` derived
    // placeholder in the first slot.
    let unit = f.get(102).expect("#102");
    assert_eq!(unit.keyword, "IFCSIUNIT");
    assert_eq!(
        unit.args,
        vec![
            Value::Derived,
            Value::Enum("LENGTHUNIT".into()),
            Value::Enum("MILLI".into()),
            Value::Enum("METRE".into()),
        ]
    );
}

#[test]
fn wall_with_opening_and_window() {
    let f = parse_checked(WALL, 127);
    assert_eq!(
        f.header.file_name.name,
        "building_element_configuration_wall.ifc"
    );
    assert_eq!(
        f.header.file_description.description,
        ["ViewDefinition [ReferenceView_V1.2]"]
    );
    assert_eq!(f.header.file_description.implementation_level, "2;1");
    assert_eq!(f.header.file_name.authorization, "The authorising person");

    // Exactly one window instance — and the type object must not be
    // conflated with it.
    assert_eq!(f.instances_of("IfcWindow").count(), 1);
    assert_eq!(f.instances_of("IfcWindowType").count(), 1);
    assert_eq!(f.instances_of("IfcOpeningElement").count(), 1);

    // #2 = IFCOWNERHISTORY(..., .NOTDEFINED., $, $, $, 1323724715);
    let oh = f.get(2).expect("#2");
    assert_eq!(oh.args[3], Value::Enum("NOTDEFINED".into()));
    assert_eq!(oh.args[7], Value::Integer(1323724715));

    // #102 IFCWINDOW carries real OverallHeight/OverallWidth `1000.`.
    let window = f.get(102).expect("#102");
    assert_eq!(window.keyword, "IFCWINDOW");
    assert_eq!(window.args[8], Value::Real(1000.0));
    assert_eq!(window.args[9], Value::Real(1000.0));

    // Typed (SELECT) parameter: IFCPROPERTYSINGLEVALUE #50 wraps an
    // IFCIDENTIFIER('').
    let prop = f.get(50).expect("#50");
    assert_eq!(prop.keyword, "IFCPROPERTYSINGLEVALUE");
    assert_eq!(
        prop.args[2],
        Value::Typed {
            keyword: "IFCIDENTIFIER".into(),
            args: vec![Value::String(String::new())],
        }
    );

    // Project graph traversal from #1 (IFCPROJECT) is cycle-safe and
    // reaches the owner history.
    let project = f.get(1).expect("#1");
    assert_eq!(project.keyword, "IFCPROJECT");
    assert!(f.reachable_from(1).contains(&2));
}
