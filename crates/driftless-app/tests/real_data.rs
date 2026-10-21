//! Loading and the octree on large clouds: the colour cloud of the Büro-Scan
//! (20261020173822) from `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`) and a synthetic
//! cloud of 50 million points. Only prints; the bounds are set from these numbers. Run with
//! `cargo test -p driftless-app --release --test real_data -- --ignored --nocapture
//! --test-threads 1`.

use driftless_app::cloud::load_las;
use driftless_app::lod::{build_octree, Octree};
use driftless_app::synthetic::synthetic_cloud;
use driftless_io::progress::Progress;
use std::path::PathBuf;
use std::time::Instant;

fn mib(bytes: u64) -> f64 {
    bytes as f64 / (1u64 << 20) as f64
}

fn print_tree(tree: &Octree) {
    let depth = tree.nodes.iter().map(|n| n.depth).max().unwrap_or(0);
    let largest = tree.nodes.iter().map(|n| n.count).max().unwrap_or(0);
    eprintln!(
        "{} nodes, depth {depth}, largest node {largest} points, {} blocks",
        tree.nodes.len(),
        tree.chunks.len()
    );
}

#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA"]
fn buero_scan_lod_build() {
    let Some(dir) = std::env::var_os("DRIFTLESS_TESTDATA") else {
        eprintln!("DRIFTLESS_TESTDATA nicht gesetzt, Test übersprungen");
        return;
    };
    let path = PathBuf::from(dir).join("ergebnisse-m5/buero/karte/color.las");
    if !path.exists() {
        eprintln!(
            "{} fehlt (Task 13 erzeugt sie), Test übersprungen",
            path.display()
        );
        return;
    }
    let t = Instant::now();
    let points = load_las(&path, None, &Progress::new()).unwrap();
    let load = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let tree = build_octree(&points, &Progress::new()).unwrap();
    let build = t.elapsed().as_secs_f64();
    eprintln!(
        "{} points, loaded in {load:.2} s, octree in {build:.2} s",
        points.len()
    );
    print_tree(&tree);
    assert_eq!(tree.vertices.len(), points.len());
}

#[test]
#[ignore = "50 million points, several GB of memory"]
fn synthetic_50m_build() {
    let t = Instant::now();
    let points = synthetic_cloud(50_000_000, 1);
    let generate = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let tree = build_octree(&points, &Progress::new()).unwrap();
    let build = t.elapsed().as_secs_f64();
    let rss = driftless_steps::map::peak_rss_bytes().map_or(f64::NAN, mib);
    eprintln!(
        "{} points, generated in {generate:.2} s, octree in {build:.2} s, peak RSS {rss:.0} MiB",
        points.len()
    );
    print_tree(&tree);
    assert_eq!(tree.vertices.len(), points.len());
}
