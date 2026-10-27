//! Memory of opening a real cloud in the app ([`load_points`]): its own test binary with the
//! counting allocator, exactly one test. The files are the colour cloud of the Büro-Scan
//! (20261020173822) under `$DRIFTLESS_TESTDATA` and any further files (e.g. an E57 or LAZ
//! export of it) listed in `DRIFTLESS_LOAD_FILES`, separated by `:`. Run with
//! `cargo test -p driftless-app --release --test load_memory_real -- --ignored --nocapture`.

#[global_allocator]
static ALLOC: driftless_io::testalloc::Counting = driftless_io::testalloc::Counting;

use driftless_app::cloud::load_points;
use driftless_io::progress::Progress;
use std::path::PathBuf;

/// Bytes per point the loaded cloud may take at its peak while reading.
const MAX_BYTES_PER_POINT: usize = 40;

#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA"]
fn buero_scan_load_peak() {
    let mut files: Vec<PathBuf> = Vec::new();
    if let Some(p) =
        driftless_io::testdata::path("jmk6-2026-10-20/20261020173822/driftless/color.las")
    {
        files.push(p);
    }
    if let Some(extra) = std::env::var_os("DRIFTLESS_LOAD_FILES") {
        files.extend(std::env::split_paths(&extra));
    }
    files.retain(|p| p.exists());
    if files.is_empty() {
        eprintln!("keine Datei gefunden, Test übersprungen");
        return;
    }
    for path in files {
        let (peak, points) =
            driftless_io::testalloc::peak_during(|| load_points(&path, None, &Progress::new()));
        let points = points.unwrap();
        let n = points.len().max(1);
        eprintln!(
            "{}: {} points, peak {peak} B ({:.1} B/point)",
            path.display(),
            points.len(),
            peak as f64 / n as f64
        );
        let limit = MAX_BYTES_PER_POINT * n + (64 << 20);
        assert!(
            peak <= limit,
            "{}: peak {peak} B, limit {limit} B",
            path.display()
        );
    }
}
