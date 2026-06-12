//! `registry`-feature surface: the `Mesh3DDecoder` stub probes the
//! ISO 10303-21 magic, validates the exchange structure, and reports
//! geometry extraction as Phase-3 unsupported.

#![cfg(feature = "registry")]

use oxideav_ifc::{make_decoder, register_mesh3d};
use oxideav_mesh3d::{Error, Mesh3DDecoder, Mesh3DRegistry};

const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");

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
fn decode_valid_ifc_reports_phase3_unsupported() {
    let mut decoder = make_decoder();
    match decoder.decode(WALL) {
        Err(Error::Unsupported(msg)) => {
            assert!(msg.contains("Phase 3"), "{msg}");
            assert!(msg.contains("127 instances"), "{msg}");
            assert!(msg.contains("IFC4"), "{msg}");
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
    assert!(matches!(decoder.decode(WALL), Err(Error::Unsupported(_))));
}
