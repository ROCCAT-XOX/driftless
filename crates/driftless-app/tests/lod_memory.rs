//! Memory of the octree build: its own test binary, because the counting allocator is
//! global and a second test running in parallel would disturb the peak. Exactly one test.

#[global_allocator]
static ALLOC: driftless_io::testalloc::Counting = driftless_io::testalloc::Counting;

use driftless_app::lod::build_octree;
use driftless_app::synthetic::synthetic_cloud;
use driftless_io::progress::Progress;

/// Vertices 20 B, `source` 4 B, sort keys 16 B per point, plus buffers.
#[test]
fn octree_live_heap_per_point() {
    let points = synthetic_cloud(2_000_000, 1);
    let n = points.len();
    let (peak, tree) =
        driftless_io::testalloc::peak_during(|| build_octree(&points, &Progress::new()).unwrap());
    assert_eq!(tree.vertices.len(), n);
    let limit = 64 * n + (32 << 20);
    eprintln!(
        "{n} points, {} nodes, peak {peak} B ({:.1} B/point)",
        tree.nodes.len(),
        peak as f64 / n as f64
    );
    assert!(
        peak <= limit,
        "peak {peak} B for {n} points, limit {limit} B"
    );
}
