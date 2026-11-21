//! The scene the registry decoder builds: spatial hierarchy, opening
//! subtraction, shared representations, materials, model facts, and
//! the thread-budget / instancing options.

#![cfg(feature = "registry")]

use oxideav_ifc::{IfcDecoder, make_decoder, parse_step};
use oxideav_mesh3d::{AlphaMode, Axis, Mesh3DDecoder, Scene3D, Unit};

#[path = "../examples/support/synthetic.rs"]
mod synthetic;

const WALL: &[u8] = include_bytes!("fixtures/ifc4-wall-with-opening-and-window.ifc");
const COLUMN: &[u8] = include_bytes!("fixtures/ifc4-column-straight-rectangle-tessellation.ifc");

/// Names from the root down to the node carrying `name`.
fn path_to(scene: &Scene3D, name: &str) -> Vec<String> {
    fn walk(
        scene: &Scene3D,
        id: oxideav_mesh3d::NodeId,
        name: &str,
        path: &mut Vec<String>,
    ) -> bool {
        let node = scene.node(id).expect("node");
        path.push(node.name.clone().unwrap_or_default());
        if node.name.as_deref() == Some(name) {
            return true;
        }
        for &child in &node.children {
            if walk(scene, child, name, path) {
                return true;
            }
        }
        path.pop();
        false
    }
    let mut path = Vec::new();
    for &root in &scene.roots {
        if walk(scene, root, name, &mut path) {
            break;
        }
    }
    path
}

/// Unbalanced edges of every primitive after welding equal positions.
fn open_edges(scene: &Scene3D, mesh: usize) -> usize {
    use std::collections::HashMap;
    let mut open = 0;
    for prim in &scene.meshes[mesh].primitives {
        let mut ids: HashMap<[u32; 3], u32> = HashMap::new();
        let weld: Vec<u32> = prim
            .positions
            .iter()
            .map(|p| {
                let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
                let n = ids.len() as u32;
                *ids.entry(key).or_insert(n)
            })
            .collect();
        let idx: Vec<u32> = match &prim.indices {
            Some(oxideav_mesh3d::Indices::U32(v)) => v.clone(),
            Some(oxideav_mesh3d::Indices::U16(v)) => v.iter().map(|&i| u32::from(i)).collect(),
            None => (0..prim.positions.len() as u32).collect(),
        };
        let mut net: HashMap<(u32, u32), i32> = HashMap::new();
        for t in idx.as_chunks::<3>().0 {
            for k in 0..3 {
                let (a, b) = (weld[t[k] as usize], weld[t[(k + 1) % 3] as usize]);
                *net.entry((a.min(b), a.max(b))).or_insert(0) += if a < b { 1 } else { -1 };
            }
        }
        open += net.values().filter(|&&n| n != 0).count();
    }
    open
}

#[test]
fn spatial_structure_becomes_the_node_hierarchy() {
    let scene = make_decoder().decode(COLUMN).expect("column decodes");
    assert_eq!(scene.roots.len(), 1);
    // The fixture contains its column directly in the site.
    let path = path_to(&scene, "Column #1");
    assert_eq!(path, ["Project", "Site #1", "Column #1"]);
    let root = scene.node(scene.roots[0]).unwrap();
    assert_eq!(root.extras["ifc:class"], "IFCPROJECT");
    // The drawn node carries its identity.
    let column = scene
        .nodes
        .iter()
        .find(|n| n.name.as_deref() == Some("Column #1"))
        .unwrap();
    assert!(column.mesh.is_some());
    assert_eq!(column.extras["ifc:class"], "IFCCOLUMN");
    assert_eq!(
        column.extras["ifc:global_id"].as_str().map(str::len),
        Some(22)
    );
    // Structure nodes above it draw nothing.
    assert_eq!(scene.nodes.iter().filter(|n| n.mesh.is_some()).count(), 1);
}

#[test]
fn model_facts_reach_the_scene() {
    // The column fixture is authored in inches, IFC is Z-up.
    let scene = make_decoder().decode(COLUMN).expect("column decodes");
    assert_eq!(scene.up_axis, Axis::PosZ);
    assert_eq!(scene.unit, Unit::Inches);
    let scale = scene.extras["ifc:metres_per_unit"].as_f64().unwrap();
    assert!((scale - 0.0254).abs() < 1e-12);
    // The wall fixture is millimetres.
    let scene = make_decoder().decode(WALL).expect("wall decodes");
    assert_eq!(scene.unit, Unit::Millimetres);
}

#[test]
fn openings_are_subtracted_from_their_host() {
    let scene = make_decoder().decode(WALL).expect("wall decodes");
    // The wall (with its hole) and the window; the opening is a void.
    assert_eq!(scene.meshes.len(), 2);
    let as_authored = make_decoder()
        .with_opening_subtraction(false)
        .decode(WALL)
        .expect("wall decodes");
    assert_eq!(as_authored.meshes.len(), 3);
    // The opening lies inside the wall, so the carved scene lost its
    // volume twice: once as a solid, once out of the wall.
    let volumes: Vec<f64> = as_authored
        .meshes
        .iter()
        .map(|m| {
            let mut one = Scene3D::new();
            one.add_mesh(m.clone());
            one.signed_volume()
        })
        .collect();
    let wall = volumes.iter().cloned().fold(0.0, f64::max);
    assert!((wall - 3000.0 * 300.0 * 2000.0).abs() < 1.0, "{volumes:?}");
    let lost = as_authored.signed_volume() - scene.signed_volume();
    let opening = lost / 2.0;
    assert!(
        volumes.iter().any(|v| (v - opening).abs() < 1e-3 * opening),
        "lost {lost}, volumes {volumes:?}"
    );
    // The carved wall is still a closed solid.
    for mesh in 0..scene.meshes.len() {
        assert_eq!(open_edges(&scene, mesh), 0, "mesh {mesh}");
    }
}

#[test]
fn shared_representations_decode_identically_at_every_budget() {
    let text = synthetic::synthetic_model(400);
    let step = parse_step(text.as_bytes()).expect("parses");
    let serial = IfcDecoder::new().decode_step(&step).expect("decodes");
    // 400 proxies + one wall; the wall's three openings are voids.
    assert_eq!(serial.meshes.len(), 401);
    assert_eq!(
        serial.nodes.iter().filter(|n| n.mesh.is_some()).count(),
        401
    );
    // One material per shared surface style.
    assert_eq!(serial.materials.len(), synthetic::SHARED_MAPS);
    let half = serial
        .materials
        .iter()
        .find(|m| m.base_color[3] < 1.0)
        .expect("the half-transparent style");
    assert_eq!(half.alpha_mode, AlphaMode::Blend);
    assert!(half.double_sided);
    assert_eq!(half.metallic, 0.0);
    // Every proxy sits under the storey.
    // project → site → building → storey → proxy.
    let path = path_to(&serial, "Proxy 399");
    assert_eq!(
        path,
        ["Synthetic", "Site", "Building", "Storey", "Proxy 399"]
    );
    // The wall lost its three 1000 × 300 × 1200 openings.
    let wall = serial
        .nodes
        .iter()
        .find(|n| n.name.as_deref() == Some("Wall 0"))
        .and_then(|n| n.mesh)
        .expect("wall mesh");
    let mut one = Scene3D::new();
    one.add_mesh(serial.mesh(wall).unwrap().clone());
    let want = 6000.0 * 300.0 * 3000.0 - 3.0 * 1000.0 * 300.0 * 1200.0;
    assert!(
        (one.signed_volume() - want).abs() < 1e-6 * want,
        "{} vs {want}",
        one.signed_volume()
    );

    for threads in [2, 3, 8] {
        let scene = IfcDecoder::new()
            .with_threads(threads)
            .decode_step(&step)
            .expect("decodes");
        assert_eq!(scene.nodes.len(), serial.nodes.len());
        assert_eq!(scene.meshes.len(), serial.meshes.len());
        for (a, b) in scene.nodes.iter().zip(&serial.nodes) {
            assert_eq!(a.name, b.name);
            assert_eq!(a.children, b.children);
            assert_eq!(a.mesh, b.mesh);
        }
        for (a, b) in scene.meshes.iter().zip(&serial.meshes) {
            assert_eq!(a.primitives.len(), b.primitives.len());
            for (p, q) in a.primitives.iter().zip(&b.primitives) {
                assert_eq!(p.positions, q.positions, "{threads} threads");
                assert_eq!(p.indices, q.indices, "{threads} threads");
                assert_eq!(p.material, q.material);
            }
        }
    }
}

#[test]
fn instancing_shares_one_mesh_per_mapped_type() {
    let text = synthetic::synthetic_model(400);
    let step = parse_step(text.as_bytes()).expect("parses");
    let baked = IfcDecoder::new().decode_step(&step).expect("decodes");
    let shared = IfcDecoder::new()
        .with_instancing(true)
        .decode_step(&step)
        .expect("decodes");
    // One mesh per shared map, plus the carved wall.
    assert_eq!(shared.meshes.len(), synthetic::SHARED_MAPS + 1);
    assert_eq!(
        shared.nodes.iter().filter(|n| n.mesh.is_some()).count(),
        401
    );
    // Placed through the node transforms, the instanced scene occupies
    // the same space as the baked one.
    let (a, b) = (
        baked.bounding_box().expect("bounds"),
        shared.bounding_box().expect("bounds"),
    );
    for k in 0..3 {
        assert!((a.min[k] - b.min[k]).abs() < 1e-2, "{a:?} vs {b:?}");
        assert!((a.max[k] - b.max[k]).abs() < 1e-2, "{a:?} vs {b:?}");
    }
    assert!(shared.vertex_count() * 20 < baked.vertex_count());
}
