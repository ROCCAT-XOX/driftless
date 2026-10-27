//! Memory of the octree build on the colour cloud of the Büro-Scan (20261020173822) from
//! `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`): its own test binary with the counting
//! allocator, exactly one test. Run with `cargo test -p driftless-app --release --test
//! lod_memory_real -- --ignored --nocapture`.

#[global_allocator]
static ALLOC: driftless_io::testalloc::Counting = driftless_io::testalloc::Counting;

use driftless_app::cloud::load_las;
use driftless_app::lod::build_octree;
use driftless_io::progress::Progress;
use std::path::PathBuf;

#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA"]
fn buero_scan_octree_live_heap() {
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
    let points = load_las(&path, None, &Progress::new()).unwrap();
    let n = points.len();
    let (peak, tree) =
        driftless_io::testalloc::peak_during(|| build_octree(&points, &Progress::new()).unwrap());
    let limit = 64 * n + (32 << 20);
    eprintln!(
        "{n} points, {} nodes, peak {peak} B ({:.1} B/point), limit {limit} B",
        tree.nodes.len(),
        peak as f64 / n.max(1) as f64
    );
    assert!(
        peak <= limit,
        "peak {peak} B for {n} points, limit {limit} B"
    );
}
