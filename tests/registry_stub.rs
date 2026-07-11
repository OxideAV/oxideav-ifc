//! `registry`-feature surface: the `Mesh3DDecoder` probes the
//! ISO 10303-21 magic, validates the exchange structure, and extracts
//! tessellated, faceted-Brep and extruded swept-solid geometry into a
//! `Scene3D` (Phase 3). A model whose only representations are
//! still-unsupported geometry styles decodes to `Unsupported`.

#![cfg(feature = "registry")]

use oxideav_ifc::{make_decoder, register_mesh3d};
use oxideav_mesh3d::{Error, Mesh3DDecoder, Mesh3DRegistry};

// Swept-solid wall: the wall body, opening and window are each an
// IfcExtrudedAreaSolid over an arbitrary (polyline) profile, now meshed
// as closed prisms by the Phase-3 extruded-area-solid slice.
const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");
// A single proxy element with one IfcTriangulatedFaceSet body (a cube).
const TESS: &[u8] = include_bytes!("fixtures/ifc4-tessellated-item.ifc");
// A column whose body is a 12-triangle triangulated face set.
const COLUMN: &[u8] = include_bytes!("fixtures/ifc4-column-straight-rectangle-tessellation.ifc");
// A cube face set with an IfcIndexedColourMap assigning per-face colours.
const COLORS: &[u8] = include_bytes!("fixtures/ifc4-tessellation-with-individual-colors.ifc");
// A dense triangulated basin whose sanitary-terminal type carries an
// IfcRelAssociatesMaterial ('Ceramic') and no surface styles.
const BASIN: &[u8] = include_bytes!("fixtures/ifc4-basin-tessellation.ifc");

#[test]
fn registry_lookup_by_extension_and_format() {
    let mut registry = Mesh3DRegistry::new();
    register_mesh3d(&mut registry);
    assert!(registry.decoder_for_extension("ifc").is_some());
    assert!(registry.decoder_for_extension("IFC").is_some());
    assert!(registry.decoder_for_format("ifc").is_some());
    assert_eq!(
        registry.decoder_extensions("ifc"),
        Some(&["ifc".to_string()][..])
    );
}

#[test]
fn decode_tessellated_fixture_yields_scene() {
    let mut decoder = make_decoder();
    let scene = decoder.decode(TESS).expect("tessellated fixture decodes");
    // The cube fixture has exactly one triangulated face set: 8 vertices,
    // 12 triangles.
    assert_eq!(scene.triangle_count(), 12, "cube body triangle count");
    assert_eq!(scene.vertex_count(), 8, "cube body vertex count");
}

#[test]
fn decode_column_fixture_yields_scene() {
    let mut decoder = make_decoder();
    let scene = decoder.decode(COLUMN).expect("column fixture decodes");
    // The column body is a 24-vertex, 12-triangle box face set.
    assert_eq!(scene.triangle_count(), 12);
    assert_eq!(scene.vertex_count(), 24);
}

#[test]
fn decode_column_positions_body_in_world_space() {
    // The column body's local vertices range ±4 in X/Y; its product
    // placement translates the body to (432, 288, 48) in world space, so
    // the decoded scene's vertices must reflect that offset rather than
    // sitting at the local origin.
    let mut decoder = make_decoder();
    let scene = decoder.decode(COLUMN).expect("column fixture decodes");

    let positions: Vec<[f32; 3]> = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .flat_map(|p| p.positions.iter().copied())
        .collect();
    assert_eq!(positions.len(), 24);

    // Local body extents are within ±4 of the placement origin, so every
    // X is in [428, 436], every Y in [284, 292], every Z in [48, 168].
    for [x, y, z] in &positions {
        assert!((428.0..=436.0).contains(x), "x out of placed range: {x}");
        assert!((284.0..=292.0).contains(y), "y out of placed range: {y}");
        assert!((48.0..=168.0).contains(z), "z out of placed range: {z}");
    }
    // At least one vertex sits at the minimum corner (428, 292, 48).
    assert!(
        positions
            .iter()
            .any(|p| (p[0] - 428.0).abs() < 1e-3 && (p[2] - 48.0).abs() < 1e-3),
        "expected a vertex at the placed base corner"
    );
}

#[test]
fn decode_extruded_swept_solid_model_yields_scene() {
    // The wall fixture's bodies are IfcExtrudedAreaSolids over polyline
    // profiles: a 3000×300×2000 wall plus an opening and a window box.
    // Each extrudes a 4-point profile into an 8-vertex / 12-triangle
    // prism, so the scene holds three boxes (24 verts, 36 triangles).
    let mut decoder = make_decoder();
    let scene = decoder
        .decode(WALL)
        .expect("wall swept-solid model decodes");
    assert_eq!(scene.meshes.len(), 3, "wall body + opening + window");
    assert_eq!(scene.vertex_count(), 24);
    assert_eq!(scene.triangle_count(), 36);

    // The wall body sweeps a 3000×300 profile +Z by 2000; some vertex
    // must reach the far corner (3000, 300, 2000).
    let positions: Vec<[f32; 3]> = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .flat_map(|p| p.positions.iter().copied())
        .collect();
    assert!(
        positions.iter().any(|p| (p[0] - 3000.0).abs() < 1e-1
            && (p[1] - 300.0).abs() < 1e-1
            && (p[2] - 2000.0).abs() < 1e-1),
        "expected the wall body's far-top corner at (3000, 300, 2000)"
    );
}

#[test]
fn decode_indexed_colour_map_splits_vertices_with_face_colours() {
    // The colour-map cube's 12 triangles carry ColourIndex
    // (1,1,2,2,3,3,1,1,1,1,1) into a red/green/yellow IfcColourRgbList
    // — 11 entries for 12 faces, so the last face falls back to white.
    let mut decoder = make_decoder();
    let scene = decoder.decode(COLORS).expect("colour fixture decodes");
    let prim = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .find(|p| !p.colors.is_empty())
        .expect("a primitive with vertex colours");
    // Vertices are split per face: 12 triangles → 36 vertices, no index
    // buffer, one colour per vertex.
    assert_eq!(prim.positions.len(), 36);
    assert!(prim.indices.is_none());
    assert_eq!(prim.colors[0].len(), 36);
    // Faces 0-1 red, 2-3 green (0,0.5,0), 4-5 yellow, 6-10 red, 11 white.
    assert_eq!(prim.colors[0][0], [1.0, 0.0, 0.0, 1.0]);
    assert_eq!(prim.colors[0][2 * 3], [0.0, 0.5, 0.0, 1.0]);
    assert_eq!(prim.colors[0][4 * 3], [1.0, 1.0, 0.0, 1.0]);
    assert_eq!(prim.colors[0][11 * 3], [1.0, 1.0, 1.0, 1.0]);
    // All three vertices of one face share its flat colour.
    assert_eq!(prim.colors[0][1], prim.colors[0][0]);
    assert_eq!(prim.colors[0][2], prim.colors[0][0]);
}

#[test]
fn decode_styled_item_becomes_primitive_material() {
    // A synthetic model: an extruded box whose representation item is
    // styled dark red (0.6, 0.1, 0.1) with transparency 0.25 through
    // IfcStyledItem → IfcPresentationStyleAssignment → IfcSurfaceStyle
    // → IfcSurfaceStyleShading → IfcColourRgb.
    let model = b"ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('styled.ifc','2026-07-09T00:00:00',('a'),('o'),'p','s','auth');
FILE_SCHEMA(('IFC4'));
ENDSEC;
DATA;
#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,2.);
#2=IFCDIRECTION((0.,0.,1.));
#3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);
#4=IFCCOLOURRGB($,0.6,0.1,0.1);
#5=IFCSURFACESTYLESHADING(#4,0.25);
#6=IFCSURFACESTYLE('DarkRed',.BOTH.,(#5));
#7=IFCPRESENTATIONSTYLEASSIGNMENT((#6));
#8=IFCSTYLEDITEM(#3,(#7),$);
#9=IFCSHAPEREPRESENTATION(#99,'Body','SweptSolid',(#3));
#10=IFCPRODUCTDEFINITIONSHAPE($,$,(#9));
ENDSEC;
END-ISO-10303-21;
";
    let mut decoder = make_decoder();
    let scene = decoder.decode(model).expect("styled model decodes");
    assert_eq!(scene.materials.len(), 1);
    let mat = &scene.materials[0];
    assert_eq!(mat.name.as_deref(), Some("DarkRed"));
    // Alpha = 1 − Transparency = 0.75.
    let c = mat.base_color;
    assert!((c[0] - 0.6).abs() < 1e-6);
    assert!((c[1] - 0.1).abs() < 1e-6);
    assert!((c[2] - 0.1).abs() < 1e-6);
    assert!((c[3] - 0.75).abs() < 1e-6);
    // The box primitive references the material.
    let prim = &scene.meshes[0].primitives[0];
    assert_eq!(prim.material, Some(oxideav_mesh3d::MaterialId(0)));
}

#[test]
fn decode_names_nodes_after_owning_product() {
    // The column fixture's body-bearing product carries a Name; the
    // scene node for its shape must use it rather than a bare #id.
    let mut decoder = make_decoder();
    let scene = decoder.decode(COLUMN).expect("column fixture decodes");
    let named: Vec<&str> = scene
        .nodes
        .iter()
        .filter_map(|n| n.name.as_deref())
        .collect();
    assert!(
        named.iter().any(|n| n.to_lowercase().contains("column")),
        "expected a product-named node, got {named:?}"
    );
}

#[test]
fn decode_semantic_material_falls_back_to_association() {
    // The basin has no IfcStyledItem; the unstyled primitive falls
    // back to the semantic material associated with the terminal's
    // type: a named, colourless Material 'Ceramic'.
    let mut decoder = make_decoder();
    let scene = decoder.decode(BASIN).expect("basin fixture decodes");
    assert_eq!(scene.materials.len(), 1);
    assert_eq!(scene.materials[0].name.as_deref(), Some("Ceramic"));
    let with_material: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .filter(|p| p.material == Some(oxideav_mesh3d::MaterialId(0)))
        .count();
    assert!(with_material >= 1, "basin primitive carries the material");
}

#[test]
fn decode_non_step_input_is_invalid_data() {
    let mut decoder = make_decoder();
    match decoder.decode(b"not a step file") {
        Err(Error::InvalidData(msg)) => assert!(msg.contains("ISO-10303-21"), "{msg}"),
        other => panic!("expected InvalidData, got {other:?}"),
    }
}

#[test]
fn decode_truncated_step_is_invalid_data() {
    let mut decoder = make_decoder();
    // Valid magic, then a torn-off file.
    match decoder.decode(b"ISO-10303-21;\nHEADER;\n") {
        Err(Error::InvalidData(_)) => {}
        other => panic!("expected InvalidData, got {other:?}"),
    }
}

#[test]
fn registry_decode_via_factory() {
    let mut registry = Mesh3DRegistry::new();
    register_mesh3d(&mut registry);
    let mut decoder = registry.decoder_for_extension("ifc").unwrap();
    let scene = decoder.decode(TESS).expect("decode via factory");
    assert_eq!(scene.triangle_count(), 12);
}
