//! Load an IFC model (or a generated synthetic one) and report the
//! stage timings and the scene it decodes to.
//!
//! ```sh
//! cargo run --release --example ifc_load -- model.ifc
//! cargo run --release --example ifc_load -- --synthetic 100000 --threads 4 --instancing
//! ```

use std::time::Instant;

use oxideav_ifc::IfcDecoder;

#[path = "support/synthetic.rs"]
mod synthetic;

fn main() {
    let mut path: Option<String> = None;
    let mut synth: Option<usize> = None;
    let mut threads = 1usize;
    let mut instancing = false;
    let mut openings = true;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--synthetic" => synth = args.next().and_then(|v| v.parse().ok()),
            "--threads" => threads = args.next().and_then(|v| v.parse().ok()).unwrap_or(1),
            "--instancing" => instancing = true,
            "--keep-openings" => openings = false,
            other => path = Some(other.to_string()),
        }
    }
    let bytes: Vec<u8> = match (synth, path) {
        (Some(n), _) => synthetic::synthetic_model(n).into_bytes(),
        (None, Some(p)) => std::fs::read(&p).expect("readable file"),
        _ => {
            eprintln!(
                "usage: ifc_load (<file.ifc> | --synthetic <products>) \
                 [--threads <n>] [--instancing] [--keep-openings]"
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
    let decoder = IfcDecoder::new()
        .with_threads(threads)
        .with_instancing(instancing)
        .with_opening_subtraction(openings);
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
