//! Merging real scans (all `#[ignore]`, need `DRIFTLESS_TESTDATA` and the maps of the scans
//! in their run folder `<scan>/driftless`, computed beforehand with the release CLI).
//!
//! ```sh
//! cargo test --release -p driftless-steps --test merge_real -- --ignored --nocapture
//! ```
//!
//! The model goes to `$DRIFTLESS_TESTDATA/ergebnisse-b1/modell-test-<pid>` (or
//! `DRIFTLESS_B1_OUT`); it is not deleted, so it can be looked at.

use driftless_io::progress::{LogLine, Progress};
use driftless_map::load_stage;
use driftless_slam::align::{align_pair, AlignConfig, Verdict};
use driftless_steps::map::{clean_stage_id, map_input_for, CLEAN_BIN};
use driftless_steps::merge::{run_merge, MergeMember, MergeRequest};
use driftless_steps::slam::SlamSettings;
use driftless_traj::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::path::PathBuf;
use std::time::Instant;

/// The three scans of the black room of 22.10.; the first is the reference.
const BLACK_ROOM: [&str; 3] = [
    "jmk6-2026-10-22/20261022133554",
    "jmk6-2026-10-22/20261022132520",
    "jmk6-2026-10-22/20261022133028",
];
/// The office of 20.10.
const OFFICE: &str = "jmk6-2026-10-20/20261020173822";

fn scan(rel: &str) -> PathBuf {
    driftless_io::testdata::path(rel).expect("DRIFTLESS_TESTDATA")
}

fn member(rel: &str) -> MergeMember {
    let scan = scan(rel);
    let run = scan.join("driftless");
    MergeMember {
        scan,
        slam_dir: run.clone(),
        map_dir: run,
        slam: SlamSettings::default(),
    }
}

fn out_dir() -> PathBuf {
    let dir = std::env::var_os("DRIFTLESS_B1_OUT")
        .map(PathBuf::from)
        .or_else(|| driftless_io::testdata::path("ergebnisse-b1"))
        .expect("DRIFTLESS_TESTDATA")
        .join(format!("modell-test-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

/// Progress that prints every line as it comes.
fn console() -> Progress {
    Progress::with_log(|line| match line {
        LogLine::Info(s) => println!("{s}"),
        LogLine::Warning(s) => println!("Warnung: {s}"),
    })
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn black_room_three_scans() {
    let req = MergeRequest {
        members: BLACK_ROOM.iter().map(|r| member(r)).collect(),
        out_dir: out_dir(),
        align: AlignConfig::default(),
        voxel: 0.01,
        smooth: true,
        include_unsure: false,
        pairs: Vec::new(),
    };
    let report = run_merge(&req, &console()).unwrap();
    println!("{}", serde_json::to_string_pretty(&report.members).unwrap());
    println!(
        "{} Punkte, {:.1} s, Spitzen-RAM {:.2} GB → {}",
        report.points,
        report.seconds_total,
        report.peak_rss_bytes as f64 / 1e9,
        req.out_dir.display()
    );
    for m in &report.members[1..] {
        let name = m.scan.display();
        assert_eq!(m.verdict, "sicher", "{name}: {:?}", m.reason);
        assert!(m.overlap > 0.8, "{name}: overlap {:.3}", m.overlap);
        assert!(
            m.residual_median_mm <= 5.0,
            "{name}: residual {:.2} mm",
            m.residual_median_mm
        );
    }
    if report.peak_rss_bytes > 0 {
        assert!(
            report.peak_rss_bytes < 4_000_000_000,
            "peak {:.2} GB",
            report.peak_rss_bytes as f64 / 1e9
        );
    }
}

/// Every point of the cleaned office map, the first of each 2 cm cell.
fn office_points() -> Vec<Vector3<f64>> {
    let m = member(OFFICE);
    let input = map_input_for(&m.scan, &m.slam_dir, &m.map_dir, m.slam.clone());
    let id = clean_stage_id(&input).unwrap();
    let (cloud, _) = load_stage(&m.map_dir.join(CLEAN_BIN), &id).unwrap();
    let mut seen = std::collections::HashSet::new();
    cloud
        .xyz
        .iter()
        .map(|p| Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2])))
        .filter(|p| seen.insert((p / 0.02).map(|c| c.floor() as i64)))
        .collect()
}

/// The `q`-quantile of the x coordinates.
fn x_quantile(points: &[Vector3<f64>], q: f64) -> f64 {
    let mut x: Vec<f64> = points.iter().map(|p| p.x).collect();
    x.sort_by(f64::total_cmp);
    x[((x.len() - 1) as f64 * q).round() as usize]
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn corridor_cut_from_office() {
    let points = office_points();
    // Two parts along x sharing a strip of ≈ 15 % of the points.
    let (lo, hi) = (x_quantile(&points, 0.425), x_quantile(&points, 0.575));
    let target: Vec<Vector3<f64>> = points.iter().filter(|p| p.x <= hi).copied().collect();
    let part: Vec<Vector3<f64>> = points.iter().filter(|p| p.x >= lo).copied().collect();
    drop(points);
    // The second part moved by a known pose: the alignment has to find its inverse.
    let moved = Isometry3::from_parts(
        Translation3::new(3.0, -2.0, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 70f64.to_radians()),
    );
    let source: Vec<Vector3<f64>> = part
        .iter()
        .map(|p| (moved * Point3::from(*p)).coords)
        .collect();
    let truth = moved.inverse();
    let start = Instant::now();
    let a = align_pair(&source, &target, &AlignConfig::default(), &Progress::new());
    let c = Point3::from(source.iter().sum::<Vector3<f64>>() / source.len() as f64);
    let err = (a.pose * c - truth * c).norm();
    let rot = (a.pose.rotation.inverse() * truth.rotation)
        .angle()
        .to_degrees();
    println!(
        "{} + {} Punkte, {:.1} s: {:?} {:?}, Überlappung {:.3}, Kanten {:.3}, Restfehler {:.2} mm, \
         Fehler {err:.4} m {rot:.3}°",
        source.len(),
        target.len(),
        start.elapsed().as_secs_f64(),
        a.verdict,
        a.method,
        a.quality.overlap,
        a.quality.agreement,
        a.quality.residual_median_mm
    );
    if a.verdict == Verdict::Sure {
        assert!(err < 0.03, "sure with error {err:.4} m {rot:.3}°");
    }
}
