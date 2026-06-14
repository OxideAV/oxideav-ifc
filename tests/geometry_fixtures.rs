//! Phase-3 geometry extraction over the real fixture meshes
//! (buildingSMART Sample-Test-Files, CC-BY 4.0 — see README). These
//! exercise the std-only `geometry` API; no `registry` feature needed.

use oxideav_ifc::{
    mesh_from_product_shape, parse_step, placement_transform, tessellate_item, GeometryError,
    TriMesh,
};

const BASIN: &[u8] = include_bytes!("fixtures/ifc4-basin-tessellation.ifc");
const COLUMN: &[u8] = include_bytes!("fixtures/ifc4-column-straight-rectangle-tessellation.ifc");
const ITEM: &[u8] = include_bytes!("fixtures/ifc4-tessellated-item.ifc");
const COLORS: &[u8] = include_bytes!("fixtures/ifc4-tessellation-with-individual-colors.ifc");

/// Tessellate the single `IfcTriangulatedFaceSet` at `face_set_id`.
fn mesh_of(bytes: &[u8], face_set_id: u64) -> TriMesh {
    let f = parse_step(bytes).expect("parse");
    tessellate_item(&f, face_set_id).expect("tessellate")
}

#[test]
fn cube_proxy_item() {
    // #1021 = IFCTRIANGULATEDFACESET(#1022, ...) — 8 verts, 12 triangles.
    let m = mesh_of(ITEM, 1021);
    assert_eq!(m.vertex_count(), 8);
    assert_eq!(m.triangle_count(), 12);
    // First CoordIndex triple (1,6,5) → 0-based (0,5,4).
    assert_eq!(m.triangles[0], [0, 5, 4]);
    // Point list row 1 = (-500,-500,0).
    assert_eq!(m.positions[0], [-500.0, -500.0, 0.0]);
    // Every index stays in range.
    for [a, b, c] in &m.triangles {
        for i in [a, b, c] {
            assert!((*i as usize) < m.vertex_count());
        }
    }
}

#[test]
fn column_box_item() {
    // #288 = IFCTRIANGULATEDFACESET(#287, ...) — 24 verts, 12 triangles.
    let m = mesh_of(COLUMN, 288);
    assert_eq!(m.vertex_count(), 24);
    assert_eq!(m.triangle_count(), 12);
    // (1,3,2) → (0,2,1).
    assert_eq!(m.triangles[0], [0, 2, 1]);
}

#[test]
fn colors_cube_item() {
    // #201 = IFCTRIANGULATEDFACESET(#200, ...) — 8 verts, 12 triangles.
    let m = mesh_of(COLORS, 201);
    assert_eq!(m.vertex_count(), 8);
    assert_eq!(m.triangle_count(), 12);
}

#[test]
fn basin_mesh_indices_all_in_range() {
    // #201 is the large basin face set: every resolved index must land
    // inside the point list (a regression guard for 1-based handling).
    let m = mesh_of(BASIN, 201);
    assert!(m.triangle_count() > 200, "basin is a dense mesh");
    let n = m.vertex_count();
    for [a, b, c] in &m.triangles {
        assert!((*a as usize) < n && (*b as usize) < n && (*c as usize) < n);
    }
}

#[test]
fn column_placement_positions_body_in_world_space() {
    // The column #71 is placed by #121 = IFCLOCALPLACEMENT(#67, #126),
    // where #126 = IFCAXIS2PLACEMENT3D((432,288,48), Z=[0,0,1], X=[1,0,0])
    // relative to #67 = IFCLOCALPLACEMENT($, #69) at the origin. So the
    // world transform is a pure translation by (432, 288, 48).
    let f = parse_step(COLUMN).expect("parse");
    let t = placement_transform(&f, 121).expect("placement");

    // Local body vertex #287 row 1 = (-4, 4, 0) → world (428, 292, 48).
    let local = mesh_from_product_shape(&f, 111).expect("body mesh");
    let world = local.transformed(&t);
    assert_eq!(world.vertex_count(), local.vertex_count());
    assert_eq!(world.triangles, local.triangles);

    let w0 = world.positions[0];
    assert!((w0[0] - 428.0).abs() < 1e-9, "x {w0:?}");
    assert!((w0[1] - 292.0).abs() < 1e-9, "y {w0:?}");
    assert!((w0[2] - 48.0).abs() < 1e-9, "z {w0:?}");

    // Local top row #287 entry 5 = (-4, 4, 120) → world (428, 292, 168).
    let w_top = world.positions[4];
    assert!((w_top[2] - 168.0).abs() < 1e-9, "top z {w_top:?}");
}

#[test]
fn non_geometry_id_is_unsupported() {
    let f = parse_step(ITEM).expect("parse");
    // #100 is the IfcProject — not a representation item.
    assert!(matches!(
        tessellate_item(&f, 100),
        Err(GeometryError::Unsupported(_))
    ));
}
