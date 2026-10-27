//! Memory of opening a LAS file in the app ([`load_points`]): its own test binary with the
//! counting allocator, exactly one test. The points go block by block into the
//! [`driftless_app::cloud::PointSet`], so the peak stays near its 24 B per point (plus the low
//! bytes of a possible 8-bit file and the reader's batch), not at the 43 B per point of a whole
//! `Points` plus the columns being converted.

#[global_allocator]
static ALLOC: driftless_io::testalloc::Counting = driftless_io::testalloc::Counting;

use driftless_app::cloud::load_points;
use driftless_export::{write_points, PointFormat, Points};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;

/// Bytes per point at the peak; the old loader before the export crate took about 28.
const MAX_BYTES_PER_POINT: usize = 32;
/// The LAS reader's batch of 2²⁰ points (36 B each in format 7) and small buffers.
const FIXED: usize = 48 << 20;

#[test]
fn las_load_peak_per_point() {
    let n = 3_000_000;
    let mut s = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        s
    };
    let xyz = (0..n)
        .map(|_| {
            let r = next();
            [
                512_000.0 + (r % 40_000) as f64 * 0.000_5,
                5_400_000.0 + (r >> 16) as f64 % 20_000.0 * 0.000_5,
                (r >> 32) as f64 % 6_000.0 * 0.000_5,
            ]
        })
        .collect();
    let points = Points {
        xyz,
        rgb: Some((0..n).map(|i| [i as u16, 40_000, 65_535]).collect()),
        intensity: Some((0..n).map(|i| (i * 7) as u16).collect()),
        source: Some((0..n).map(|i| u16::from(i % 5 == 0)).collect()),
        time: Some((0..n).map(|i| 1.7e9 + i as f64 * 1e-5).collect()),
        flags: Some((0..n).map(|i| (i % 2) as u8).collect()),
        scans: Vec::new(),
    };
    let dir = tmp("app-load-memory");
    let path = dir.join("wolke.las");
    write_points(&path, PointFormat::Las, &points, &Progress::new()).unwrap();
    drop(points);

    let (peak, set) =
        driftless_io::testalloc::peak_during(|| load_points(&path, None, &Progress::new()));
    let set = set.unwrap();
    assert_eq!(set.len(), n);
    let limit = MAX_BYTES_PER_POINT * n + FIXED;
    eprintln!(
        "{n} points, peak {peak} B ({:.1} B/point), limit {limit} B",
        peak as f64 / n as f64
    );
    assert!(
        peak <= limit,
        "peak {peak} B for {n} points, limit {limit} B"
    );
    std::fs::remove_dir_all(dir).unwrap();
}
