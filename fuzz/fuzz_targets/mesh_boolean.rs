//! Drive the mesh–mesh Boolean evaluator with hostile operand pairs:
//! each operand is a concatenation of up to four small solids (boxes,
//! tetrahedra, wedges — possibly degenerate, overlapping or inverted)
//! read off the input bytes, and every operator is evaluated. The
//! evaluator must never panic or run away (its fragment budget bounds
//! the work). When both operands are single well-formed boxes the exact
//! partition identity vol(A − B) + vol(A ∩ B) = vol(A) and the
//! watertightness of every result are asserted as well.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_ifc::{mesh_boolean, BooleanOperator, TriMesh};

/// Append `src` to `dst`, re-indexing.
fn append(dst: &mut TriMesh, src: TriMesh) {
    let base = dst.positions.len() as u32;
    dst.positions.extend(src.positions);
    dst.triangles
        .extend(src.triangles.into_iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
}

/// A coordinate in roughly [−8, 8] from one byte.
fn coord(b: u8) -> f64 {
    (b as f64 - 128.0) / 16.0
}

fn box_mesh(min: [f64; 3], max: [f64; 3]) -> TriMesh {
    let p = |x: usize, y: usize, z: usize| {
        [
            if x == 0 { min[0] } else { max[0] },
            if y == 0 { min[1] } else { max[1] },
            if z == 0 { min[2] } else { max[2] },
        ]
    };
    let positions = vec![
        p(0, 0, 0),
        p(1, 0, 0),
        p(0, 1, 0),
        p(1, 1, 0),
        p(0, 0, 1),
        p(1, 0, 1),
        p(0, 1, 1),
        p(1, 1, 1),
    ];
    let quads: [[u32; 4]; 6] = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let mut triangles = Vec::with_capacity(12);
    for q in quads {
        triangles.push([q[0], q[1], q[2]]);
        triangles.push([q[0], q[2], q[3]]);
    }
    TriMesh {
        positions,
        triangles,
    }
}

/// One solid from 13 bytes: kind + 12 coordinate bytes.
fn solid(bytes: &[u8]) -> Option<(TriMesh, bool)> {
    if bytes.len() < 13 {
        return None;
    }
    let c: Vec<f64> = bytes[1..13].iter().map(|&b| coord(b)).collect();
    match bytes[0] % 3 {
        0 => {
            // Box from a corner and (possibly zero / negative) extents.
            let min = [c[0], c[1], c[2]];
            let max = [c[0] + c[3].abs(), c[1] + c[4].abs(), c[2] + c[5].abs()];
            let well_formed = (0..3).all(|k| max[k] - min[k] > 0.25);
            Some((box_mesh(min, max), well_formed))
        }
        1 => {
            // Tetrahedron from four points (outward if positively
            // oriented; inverted otherwise — hostile on purpose).
            let positions = vec![
                [c[0], c[1], c[2]],
                [c[3], c[4], c[5]],
                [c[6], c[7], c[8]],
                [c[9], c[10], c[11]],
            ];
            let triangles = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
            Some((
                TriMesh {
                    positions,
                    triangles,
                },
                false,
            ))
        }
        _ => {
            // Wedge: a triangle extruded along z.
            let h = c[6].abs().max(0.01);
            let positions = vec![
                [c[0], c[1], c[2]],
                [c[3], c[4], c[2]],
                [c[5], c[7], c[2]],
                [c[0], c[1], c[2] + h],
                [c[3], c[4], c[2] + h],
                [c[5], c[7], c[2] + h],
            ];
            let triangles = vec![
                [0, 2, 1],
                [3, 4, 5],
                [0, 1, 4],
                [0, 4, 3],
                [1, 2, 5],
                [1, 5, 4],
                [2, 0, 3],
                [2, 3, 5],
            ];
            Some((
                TriMesh {
                    positions,
                    triangles,
                },
                false,
            ))
        }
    }
}

/// An operand: up to four solids concatenated.
fn operand(bytes: &[u8]) -> (TriMesh, bool) {
    let count = (bytes.first().copied().unwrap_or(1) % 4 + 1) as usize;
    let mut mesh = TriMesh::default();
    let mut single_box = false;
    let mut rest = &bytes[1.min(bytes.len())..];
    for i in 0..count {
        let Some((s, well_formed)) = solid(rest) else {
            break;
        };
        rest = &rest[13..];
        single_box = i == 0 && count == 1 && well_formed;
        append(&mut mesh, s);
    }
    (mesh, single_box)
}

fn closed(m: &TriMesh) -> bool {
    let mut net: std::collections::HashMap<(u32, u32), i32> = std::collections::HashMap::new();
    for t in &m.triangles {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            *net.entry((a.min(b), a.max(b))).or_insert(0) += if a < b { 1 } else { -1 };
        }
    }
    net.values().all(|&n| n == 0)
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    let split = (data[0] as usize + 1).min(data.len() - 1);
    let (a, a_box) = operand(&data[1..split]);
    let (b, b_box) = operand(&data[split..]);
    if a.is_empty() || b.is_empty() {
        return;
    }
    let d = mesh_boolean(&a, &b, BooleanOperator::Difference);
    let i = mesh_boolean(&a, &b, BooleanOperator::Intersection);
    let u = mesh_boolean(&a, &b, BooleanOperator::Union);
    if a_box && b_box {
        // Two well-formed boxes: exact partition, closed results.
        let (d, i, u) = (d.unwrap(), i.unwrap(), u.unwrap());
        let va = a.signed_volume();
        let vb = b.signed_volume();
        let scale = va.max(vb).max(1.0);
        assert!(
            (d.signed_volume() + i.signed_volume() - va).abs() < 1e-6 * scale,
            "A − B and A ∩ B do not partition A"
        );
        assert!(
            (u.signed_volume() + i.signed_volume() - va - vb).abs() < 1e-6 * scale,
            "A ∪ B and A ∩ B do not sum to A + B"
        );
        for m in [&d, &i, &u] {
            assert!(closed(m), "result is not watertight");
        }
    }
});
