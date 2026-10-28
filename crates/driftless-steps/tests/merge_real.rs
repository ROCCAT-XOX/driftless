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
use driftless_slam::align::{align_pair, align_with_pairs, AlignConfig, AlignMethod, Verdict};
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
        ply: false,
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
        println!(
            "{name}: Wände {:.2} mm, Restfehler {:.2} mm",
            m.wall_residual_mm, m.residual_median_mm
        );
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
         Wände {:.2} mm, Fehler {err:.4} m {rot:.3}°",
        source.len(),
        target.len(),
        start.elapsed().as_secs_f64(),
        a.verdict,
        a.method,
        a.quality.overlap,
        a.quality.agreement,
        a.quality.residual_median_mm,
        a.quality.wall_residual_mm
    );
    if a.verdict == Verdict::Sure {
        assert!(err < 0.03, "sure with error {err:.4} m {rot:.3}°");
    }
}

/// Source, target, the pose of the source in the target and the shared strip `(lo, hi)` in x.
type Strip = (
    Vec<Vector3<f64>>,
    Vec<Vector3<f64>>,
    Isometry3<f64>,
    (f64, f64),
);

/// The office cut as in [`corridor_cut_from_office`], sharing a strip of `share` of the
/// points.
fn office_strip(points: &[Vector3<f64>], share: f64) -> Strip {
    let (lo, hi) = (
        x_quantile(points, 0.5 - share / 2.0),
        x_quantile(points, 0.5 + share / 2.0),
    );
    let target: Vec<Vector3<f64>> = points.iter().filter(|p| p.x <= hi).copied().collect();
    let moved = Isometry3::from_parts(
        Translation3::new(3.0, -2.0, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 70f64.to_radians()),
    );
    let source: Vec<Vector3<f64>> = points
        .iter()
        .filter(|p| p.x >= lo)
        .map(|p| (moved * Point3::from(*p)).coords)
        .collect();
    (source, target, moved.inverse(), (lo, hi))
}

/// Four places of the shared strip `x ∈ [lo, hi]`, far apart: the points at 2 % and 98 % of
/// y near the floor, and the lowest and highest point around the middle of y.
fn strip_marks(points: &[Vector3<f64>], (lo, hi): (f64, f64)) -> Vec<Vector3<f64>> {
    let mut strip: Vec<Vector3<f64>> = points
        .iter()
        .filter(|p| p.x >= lo && p.x <= hi)
        .copied()
        .collect();
    strip.sort_by(|a, b| a.y.total_cmp(&b.y));
    let at = |q: f64| strip[((strip.len() - 1) as f64 * q).round() as usize];
    let mid = at(0.5).y;
    let middle: Vec<&Vector3<f64>> = strip.iter().filter(|p| (p.y - mid).abs() < 0.3).collect();
    let low = **middle.iter().min_by(|a, b| a.z.total_cmp(&b.z)).unwrap();
    let high = **middle.iter().max_by(|a, b| a.z.total_cmp(&b.z)).unwrap();
    vec![at(0.02), at(0.98), low, high]
}

/// Point pairs by hand where the office parts share only a narrow strip: the right pose there
/// overlaps too little for an automatic sure result (5 %: overlap 0.10, edges 0.82; 2 %:
/// 0.043, 0.66), the pairs place it surely. A wrong pair set (one place on another point of the strip) is
/// refused.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn corridor_pairs_narrow_strip() {
    let points = office_points();
    let cfg = AlignConfig::default();
    for share in [0.05, 0.02] {
        let (source, target, truth, strip) = office_strip(&points, share);
        let marks = strip_marks(&points, strip);
        // A careful click: 1 cm off in the target, 1 cm the other way in the source.
        let errors = [
            Vector3::new(0.01, 0.0, 0.0),
            Vector3::new(0.0, 0.01, 0.0),
            Vector3::new(0.0, 0.0, 0.01),
            Vector3::new(-0.007, 0.007, 0.0),
        ];
        let inv = truth.inverse();
        let pairs: Vec<(Vector3<f64>, Vector3<f64>)> = marks
            .iter()
            .zip(&errors)
            .map(|(m, e)| ((inv * Point3::from(m - e)).coords, m + e))
            .collect();
        let c = Point3::from(source.iter().sum::<Vector3<f64>>() / source.len() as f64);
        let start = Instant::now();
        let a = align_with_pairs(&source, &target, &pairs, &cfg, &Progress::new());
        let err = (a.pose * c - truth * c).norm();
        println!(
            "Streifen {:.0} %, Punktpaare: {:?}, Überlappung {:.3}, Kanten {:.3}, Restfehler {:.2} \
             mm, Wände {:.2} mm, Paare {:.1} mm, Fehler {err:.4} m, {:.1} s",
            100.0 * share,
            a.verdict,
            a.quality.overlap,
            a.quality.agreement,
            a.quality.residual_median_mm,
            a.quality.wall_residual_mm,
            a.pair_rms_mm.unwrap_or(f64::NAN),
            start.elapsed().as_secs_f64()
        );
        assert_eq!(a.method, AlignMethod::PointPairs);
        assert_eq!(a.verdict, Verdict::Sure);
        assert!(err < 0.03, "error {err:.4} m");
        let mut wrong = pairs.clone();
        wrong[2].1 = marks[0];
        let w = align_with_pairs(&source, &target, &wrong, &cfg, &Progress::new());
        println!(
            "Streifen {:.0} %, falsches Paar: {:?}, Paare {:.1} mm",
            100.0 * share,
            w.verdict,
            w.pair_rms_mm.unwrap_or(f64::NAN)
        );
        assert!(
            matches!(&w.verdict, Verdict::Unsure(r) if r.contains("Punktpaare")),
            "{:?}",
            w.verdict
        );
    }
}
