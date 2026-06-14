//! `registry`-feature surface: the `Mesh3DDecoder` probes the
//! ISO 10303-21 magic, validates the exchange structure, and extracts
//! tessellated geometry into a `Scene3D` (Phase 3). A model whose only
//! representations are unsupported geometry styles (swept solids, …)
//! decodes to `Unsupported`.

#![cfg(feature = "registry")]

use oxideav_ifc::{make_decoder, register_mesh3d};
use oxideav_mesh3d::{Error, Mesh3DDecoder, Mesh3DRegistry};

// Swept-solid wall — no tessellation, so geometry extraction reports
// the body styles as not-yet-supported.
const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");
// A single proxy element with one IfcTriangulatedFaceSet body (a cube).
const TESS: &[u8] = include_bytes!("fixtures/ifc4-tessellated-item.ifc");
// A column whose body is a 12-triangle triangulated face set.
const COLUMN: &[u8] = include_bytes!("fixtures/ifc4-column-straight-rectangle-tessellation.ifc");

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
fn decode_swept_solid_model_reports_unsupported() {
    let mut decoder = make_decoder();
    match decoder.decode(WALL) {
        Err(Error::Unsupported(msg)) => {
            assert!(msg.contains("unsupported"), "{msg}");
            assert!(msg.contains("Phase-3"), "{msg}");
        }
        other => panic!("expected Unsupported, got {other:?}"),
    }
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
