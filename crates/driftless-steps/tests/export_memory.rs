//! Memory of an export: its own test binary, because the counting allocator is global and a
//! second test running in parallel would disturb the peak. Exactly one test.

use driftless_export::testutil::synthetic_points;
use driftless_export::{write_points, Layout, PointFormat};
use driftless_io::progress::Progress;
use driftless_io::testalloc::{peak_during, Counting};
use driftless_io::testscan::tmp;
use driftless_steps::export::{run_export, ExportFormat, ExportRequest, ExportSource};

#[global_allocator]
static ALLOC: Counting = Counting;

/// 2 million points with every column, from LAS into every point format: the peak stays
/// within twice the bytes of the columns plus 64 MB.
#[test]
fn export_peak_memory_per_format() {
    let n = 2_000_000;
    let layout = Layout {
        rgb: true,
        intensity: true,
        source: true,
        time: true,
        flags: true,
    };
    let points = synthetic_points(n, 5, layout);
    // xyz f64 × 3, rgb u16 × 3, intensity u16, source u16, time f64, flags u8.
    let column_bytes = n * (24 + 6 + 2 + 2 + 8 + 1);
    let dir = tmp("export-memory");
    let src = dir.join("quelle.las");
    write_points(&src, PointFormat::Las, &points, &Progress::new()).unwrap();
    drop(points);
    let limit = 2 * column_bytes + (64 << 20);
    for format in [
        ExportFormat::Las,
        ExportFormat::Laz,
        ExportFormat::E57,
        ExportFormat::Ply,
        ExportFormat::Pcd,
    ] {
        let out = dir.join(format!("ziel.{}", format.extension()));
        let req = ExportRequest {
            source: ExportSource::PointFile(src.clone()),
            out: out.clone(),
            format,
            overwrite: false,
        };
        let (peak, report) = peak_during(|| run_export(&req, &Progress::new()).unwrap());
        assert_eq!(report.count, n as u64);
        eprintln!(
            "{format:?}: peak {:.1} MB, limit {:.1} MB, {:.2} s",
            peak as f64 / 1e6,
            limit as f64 / 1e6,
            report.seconds
        );
        assert!(peak <= limit, "{format:?}: peak {peak} B, limit {limit} B");
        std::fs::remove_file(&out).unwrap();
    }
    std::fs::remove_dir_all(dir).unwrap();
}
