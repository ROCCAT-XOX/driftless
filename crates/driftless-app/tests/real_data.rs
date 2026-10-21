//! The program on real data from `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`): loading and
//! the octree of the colour cloud of the Büro-Scan (20261020173822), recomputing it after a
//! loop, the steps of the program against the CLI on `Test`, and a synthetic cloud of 50
//! million points. Needs the outputs of the release CLI under `ergebnisse-m5/` (commands in
//! `docs/ergebnisse/2026-10-22-meilenstein-5.md`). Run with
//! `cargo test -p driftless-app --release --test real_data -- --ignored --nocapture
//! --test-threads 1`.

use driftless_app::cloud::{load_cloud, view_bounds, LoadedCloud};
use driftless_app::lod::{build_octree, Octree};
use driftless_app::project::{run_map_job, run_slam_job, Project, SlamJob};
use driftless_app::synthetic::synthetic_cloud;
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use driftless_slam::SlamReport;
use driftless_steps::correct;
use driftless_steps::map::{peak_rss_bytes, MapSettings};
use driftless_steps::slam::SlamSettings;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Octree build of the Büro cloud at most (s); measured 0.20 s on the M4 Pro.
const BUERO_BUILD_MAX_S: f64 = 10.0;
/// A recompute of the Büro run after a loop, as the program does it, at most (s).
const BUERO_REUSE_MAX_S: f64 = 15.0;

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

/// `$DRIFTLESS_TESTDATA/<rel>`, or `None` (with a message) if the variable or the path is
/// missing.
fn data(rel: &str) -> Option<PathBuf> {
    let Some(dir) = std::env::var_os("DRIFTLESS_TESTDATA") else {
        eprintln!("DRIFTLESS_TESTDATA nicht gesetzt, Test übersprungen");
        return None;
    };
    let path = PathBuf::from(dir).join(rel);
    if !path.exists() {
        eprintln!(
            "{} fehlt (siehe docs/testdaten.md), Test übersprungen",
            path.display()
        );
        return None;
    }
    Some(path)
}

/// Root cube against the box „alles zeigen“ frames (without 0,5 % per side and axis): how
/// much a few far points enlarge the octree.
fn print_outliers(c: &LoadedCloud) {
    let root = &c.tree.nodes[0];
    let (min, max) = c.points.bounds();
    let (vmin, vmax) = view_bounds(&c.points).unwrap();
    let outside = c
        .points
        .xyz
        .iter()
        .filter(|p| (0..3).any(|k| p[k] < vmin[k] - 1.0 || p[k] > vmax[k] + 1.0))
        .count();
    eprintln!(
        "root cube {:.1} m; bounds {:.1} × {:.1} × {:.1} m; without 0.5 % per side {:.1} × {:.1} × {:.1} m; {outside} points more than 1 m outside that box",
        root.size,
        max[0] - min[0],
        max[1] - min[1],
        max[2] - min[2],
        vmax[0] - vmin[0],
        vmax[1] - vmin[1],
        vmax[2] - vmin[2],
    );
}

/// Loads the Büro colour cloud as the program does (with the quality of its run), bounds the
/// octree build, then loads it a second time while the first is still held, as when a cloud
/// is replaced by another of the same size.
#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA"]
fn buero_scan_lod_build() {
    let Some(run) = data("ergebnisse-m5/buero") else {
        return;
    };
    let path = run.join("karte/color.las");
    let before = peak_rss_bytes().map_or(f64::NAN, mib);
    let c = load_cloud(&path, Some(&run), &Progress::new()).unwrap();
    let after = peak_rss_bytes().map_or(f64::NAN, mib);
    eprintln!(
        "{} points, loaded in {:.2} s, octree in {:.2} s, peak RSS {before:.0} → {after:.0} MiB",
        c.points.len(),
        c.load_seconds,
        c.build_seconds
    );
    print_tree(&c.tree);
    print_outliers(&c);
    assert_eq!(c.tree.vertices.len(), c.points.len());
    assert!(c.points.quality_known, "quality from slam.json");
    assert!(
        c.build_seconds <= BUERO_BUILD_MAX_S,
        "octree {:.1} s, at most {BUERO_BUILD_MAX_S} s",
        c.build_seconds
    );
    let second = load_cloud(&path, Some(&run), &Progress::new()).unwrap();
    let both = peak_rss_bytes().map_or(f64::NAN, mib);
    eprintln!("second load while the first is held: peak RSS {both:.0} MiB");
    assert_eq!(second.points.len(), c.points.len());
}

/// The SLAM files of the run `src` copied into `dst` (not the map).
fn copy_slam_run(src: &Path, dst: &Path) {
    fs::create_dir_all(dst).unwrap();
    for name in [
        "slam.json",
        "traj.tum",
        "traj_odom.tum",
        "map.las",
        "submaps.bin",
    ] {
        fs::copy(src.join(name), dst.join(name)).unwrap();
    }
}

/// Times of a loop between the first submap and the submap of the second half whose anchor
/// is nearest to it, as one would pick them on the trajectory.
fn start_end_loop(report: &SlamReport) -> (f64, f64) {
    let first = &report.submaps[0];
    let half = report.submaps.len() / 2;
    let dist = |a: [f64; 3]| {
        (0..3)
            .map(|k| (a[k] - first.anchor[k]).powi(2))
            .sum::<f64>()
    };
    let end = report.submaps[half..]
        .iter()
        .min_by(|x, y| dist(x.anchor).total_cmp(&dist(y.anchor)))
        .unwrap();
    let mid = |s: &driftless_slam::report::SubmapReport| (s.t_start + s.t_end) / 2.0;
    (mid(first), mid(end))
}

/// The program recomputes the Büro run after a new loop (the CLI run added to a project,
/// then a loop and „Neu rechnen“) within [`BUERO_REUSE_MAX_S`]; the run gets a new state.
#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA"]
fn buero_scan_reuse_in_seconds() {
    let (Some(run), Some(scan)) = (
        data("ergebnisse-m5/buero"),
        data("jmk6-2026-10-20/20261020173822"),
    ) else {
        return;
    };
    let dir = tmp("app-buero-reuse");
    let copy = dir.join("lauf");
    copy_slam_run(&run, &copy);
    let mut p = Project::create(&dir.join("projekt"), "Büro").unwrap();
    let id = p.add_scan(&scan).unwrap();
    p.add_run(&id, &copy).unwrap();
    p.save().unwrap();

    let report: SlamReport =
        serde_json::from_str(&fs::read_to_string(copy.join("slam.json")).unwrap()).unwrap();
    let (a, b) = start_end_loop(&report);
    let before = fs::read(copy.join("traj.tum")).unwrap();
    let t = Instant::now();
    let nr = correct::add_loop(&copy, a, b, "Prüfung").unwrap();
    let run_info = p.runs(&id).remove(0);
    let done = run_slam_job(&p, &id, &SlamJob::Reuse(run_info), &Progress::new()).unwrap();
    let seconds = t.elapsed().as_secs_f64();
    let manual = done.outcome.report.loops.iter().find(|l| l.manual).unwrap();
    eprintln!(
        "loop {nr} at {a:.1} s ↔ {b:.1} s (submaps {} ↔ {}, accepted {}): recomputed in {seconds:.2} s (run_slam {:.2} s)",
        manual.a, manual.b, manual.accepted, done.outcome.seconds
    );
    let states = p.runs(&id).remove(0).history;
    eprintln!(
        "states: {:?}",
        states.iter().map(|h| h.kind.label()).collect::<Vec<_>>()
    );
    assert!(copy.join("traj.vorher.tum").is_file());
    assert_eq!(fs::read(copy.join("traj.vorher.tum")).unwrap(), before);
    assert_ne!(
        fs::read(copy.join("traj.tum")).unwrap(),
        before,
        "the loop moved the trajectory"
    );
    assert!(states.len() >= 2, "übernommen and the recompute");
    assert!(
        seconds <= BUERO_REUSE_MAX_S,
        "{seconds:.1} s, at most {BUERO_REUSE_MAX_S} s"
    );
    let _ = fs::remove_dir_all(&dir);
}

/// `map.json` without the fields that hold run times or memory.
fn map_json_without_runtime(path: &Path) -> serde_json::Value {
    let mut v: serde_json::Value = serde_json::from_str(&fs::read_to_string(path).unwrap())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let o = v.as_object_mut().unwrap();
    o.remove("seconds_total");
    o.remove("peak_rss_bytes");
    if let Some(steps) = o.get_mut("steps").and_then(|s| s.as_object_mut()) {
        for step in steps.values_mut() {
            if let Some(step) = step.as_object_mut() {
                step.remove("seconds");
                step.remove("peak_rss_bytes_after");
            }
        }
    }
    v
}

/// Compares the files `names` of `app` with those of `cli`: byte for byte, `map.json`
/// without run times.
fn assert_same_files(app: &Path, cli: &Path, names: &[&str]) {
    for name in names {
        let (a, c) = (app.join(name), cli.join(name));
        if *name == "map.json" {
            assert_eq!(
                map_json_without_runtime(&a),
                map_json_without_runtime(&c),
                "{name}"
            );
        } else {
            let (x, y) = (fs::read(&a).unwrap(), fs::read(&c).unwrap());
            assert!(x == y, "{name}: {} B (app) vs {} B (CLI)", x.len(), y.len());
        }
        eprintln!("{name}: gleich");
    }
}

/// The steps as the program runs them (a new run in a project, then a map in its own
/// folder) write the same files as `driftless slam` and `driftless map` (release CLI, into
/// `ergebnisse-m5/test`) on `Test`.
#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA, about 4 min"]
fn app_steps_match_cli() {
    let (Some(cli), Some(scan)) = (data("ergebnisse-m5/test"), data("Test")) else {
        return;
    };
    let dir = tmp("app-steps-cli");
    let mut p = Project::create(&dir.join("projekt"), "Test").unwrap();
    let id = p.add_scan(&scan).unwrap();
    p.save().unwrap();
    let t = Instant::now();
    let slam = run_slam_job(
        &p,
        &id,
        &SlamJob::Full(SlamSettings::default()),
        &Progress::new(),
    )
    .unwrap();
    eprintln!("slam {:.1} s", t.elapsed().as_secs_f64());
    assert_same_files(
        &slam.run_dir,
        &cli,
        &[
            "traj.tum",
            "traj_odom.tum",
            "map.las",
            "slam.json",
            "submaps.bin",
        ],
    );
    let run = p.runs(&id).remove(0);
    let t = Instant::now();
    let map = run_map_job(&run, &scan, &MapSettings::default(), &Progress::new()).unwrap();
    eprintln!("map {:.1} s", t.elapsed().as_secs_f64());
    assert_same_files(
        &map.dir,
        &cli,
        &[
            "dense.las",
            "dense.bin",
            "clean.las",
            "removed.las",
            "clean.bin",
            "color.las",
            "color.ply",
            "map.json",
        ],
    );
    let _ = fs::remove_dir_all(&dir);
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
    let rss = peak_rss_bytes().map_or(f64::NAN, mib);
    eprintln!(
        "{} points, generated in {generate:.2} s, octree in {build:.2} s, peak RSS {rss:.0} MiB",
        points.len()
    );
    print_tree(&tree);
    assert_eq!(tree.vertices.len(), points.len());
}
