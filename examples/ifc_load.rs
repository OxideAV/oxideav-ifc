//! Load an IFC model (or a generated synthetic one) and report the
//! stage timings and the scene it decodes to.
//!
//! ```sh
//! cargo run --release --example ifc_load -- model.ifc
//! cargo run --release --example ifc_load -- --synthetic 100000 --threads 4 --instancing
//! cargo run --release --example ifc_load -- --boolean 256
//! ```

use std::time::Instant;

use oxideav_ifc::IfcDecoder;

#[path = "support/synthetic.rs"]
mod synthetic;

fn main() {
    let mut path: Option<String> = None;
    let mut synth: Option<usize> = None;
    let mut boolean: Option<usize> = None;
    let mut threads = 1usize;
    let mut instancing = false;
    let mut openings = true;
    let mut parse_only = false;
    let mut curves = false;
    let mut item: Option<u64> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--synthetic" => synth = args.next().and_then(|v| v.parse().ok()),
            "--boolean" => boolean = args.next().and_then(|v| v.parse().ok()),
            "--threads" => threads = args.next().and_then(|v| v.parse().ok()).unwrap_or(1),
            "--instancing" => instancing = true,
            "--keep-openings" => openings = false,
            "--parse-only" => parse_only = true,
            "--curves" => curves = true,
            "--item" => item = args.next().and_then(|v| v.parse().ok()),
            other => path = Some(other.to_string()),
        }
    }
    if let Some(segments) = boolean {
        boolean_cost(segments.max(8));
        return;
    }
    let bytes: Vec<u8> = match (synth, path) {
        (Some(n), _) => synthetic::synthetic_model(n).into_bytes(),
        (None, Some(p)) => std::fs::read(&p).expect("readable file"),
        _ => {
            eprintln!(
                "usage: ifc_load (<file.ifc> | --synthetic <products> | --boolean <segments>) \
                 [--threads <n>] [--instancing] [--keep-openings] [--curves] [--parse-only] [--item <id>]"
            );
            std::process::exit(2);
        }
    };
    let mib = bytes.len() as f64 / (1024.0 * 1024.0);
    let t = Instant::now();
    let step = oxideav_ifc::parse_step(&bytes).expect("parses");
    let parse = t.elapsed().as_secs_f64();
    println!(
        "parse: {mib:.2} MiB, {} instances, {parse:.3} s, {:.1} MiB/s",
        step.len(),
        mib / parse
    );
    drop(bytes);
    if parse_only {
        return;
    }
    if let Some(id) = item {
        // One representation item on its own.
        match oxideav_ifc::tessellate_item(&step, id) {
            Ok(mesh) => println!(
                "item #{id}: {} triangles, {} vertices, volume {:.6e}, closed: {}",
                mesh.triangle_count(),
                mesh.vertex_count(),
                mesh.signed_volume(),
                oxideav_ifc::is_closed_mesh(&mesh)
            ),
            Err(e) => println!("item #{id}: {e:?}"),
        }
        return;
    }
    let decoder = IfcDecoder::new()
        .with_threads(threads)
        .with_instancing(instancing)
        .with_opening_subtraction(openings)
        .with_curves(curves);
    let t = Instant::now();
    match decoder.decode_step(&step) {
        Ok(scene) => {
            let mesh = t.elapsed().as_secs_f64();
            let drawn = scene.nodes.iter().filter(|n| n.mesh.is_some()).count();
            println!(
                "mesh: {mesh:.3} s — {} nodes ({drawn} drawn), {} meshes, {} materials, \
                 {} triangles, {} vertices, volume {:.6e}",
                scene.nodes.len(),
                scene.meshes.len(),
                scene.materials.len(),
                scene.triangle_count(),
                scene.vertex_count(),
                scene.signed_volume()
            );
        }
        Err(e) => println!("mesh: {:.3} s — no scene: {e}", t.elapsed().as_secs_f64()),
    }
}

/// Time a small tool against a dense host: a latitude / longitude
/// sphere of `segments` meridians (radius 1000) minus a 200-unit box
/// straddling its surface on the +x axis.
fn boolean_cost(segments: usize) {
    use oxideav_ifc::{BooleanOperator, TriMesh, mesh_boolean};
    let (n, m) = (segments, segments / 2);
    let tau = std::f64::consts::TAU;
    let mut host = TriMesh::default();
    host.positions.push([0.0, 0.0, -1000.0]);
    for j in 1..m {
        let lat = -tau / 4.0 + tau / 2.0 * (j as f64) / (m as f64);
        for i in 0..n {
            let lon = tau * (i as f64) / (n as f64);
            host.positions.push([
                1000.0 * lat.cos() * lon.cos(),
                1000.0 * lat.cos() * lon.sin(),
                1000.0 * lat.sin(),
            ]);
        }
    }
    host.positions.push([0.0, 0.0, 1000.0]);
    let ring = |j: usize, i: usize| (1 + (j - 1) * n + i % n) as u32;
    let top = (host.positions.len() - 1) as u32;
    for i in 0..n {
        host.triangles.push([0, ring(1, i + 1), ring(1, i)]);
        for j in 1..m - 1 {
            let (a, b, c, d) = (
                ring(j, i),
                ring(j, i + 1),
                ring(j + 1, i + 1),
                ring(j + 1, i),
            );
            host.triangles.push([a, b, c]);
            host.triangles.push([a, c, d]);
        }
        host.triangles
            .push([top, ring(m - 1, i), ring(m - 1, i + 1)]);
    }
    let (lo, hi) = ([900.0, -100.0, -100.0], [1100.0, 100.0, 100.0]);
    let corner = |k: usize| {
        [
            if k & 1 == 0 { lo[0] } else { hi[0] },
            if k & 2 == 0 { lo[1] } else { hi[1] },
            if k & 4 == 0 { lo[2] } else { hi[2] },
        ]
    };
    let tool = TriMesh {
        positions: (0..8).map(corner).collect(),
        triangles: vec![
            [0, 2, 3],
            [0, 3, 1],
            [4, 5, 7],
            [4, 7, 6],
            [0, 1, 5],
            [0, 5, 4],
            [2, 6, 7],
            [2, 7, 3],
            [0, 4, 6],
            [0, 6, 2],
            [1, 3, 7],
            [1, 7, 5],
        ],
    };
    let t = Instant::now();
    let out = mesh_boolean(&host, &tool, BooleanOperator::Difference).expect("evaluates");
    println!(
        "boolean: host {} triangles − box: {:.3} s, {} triangles out, volume {:.6e} (host {:.6e})",
        host.triangle_count(),
        t.elapsed().as_secs_f64(),
        out.triangle_count(),
        out.signed_volume(),
        host.signed_volume()
    );
}
