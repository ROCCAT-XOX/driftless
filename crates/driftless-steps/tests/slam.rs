//! `run_slam` on synthetic scans: outputs, progress and cancelling.

use driftless_io::progress::{LogLine, Progress, ProgressSnapshot};
use driftless_io::testscan::{room_bag, room_bag_secs, tmp};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::StepError;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};

fn request(scan: &Path, out: &Path, reuse: bool) -> SlamRequest {
    SlamRequest {
        scan: scan.to_path_buf(),
        out: out.to_path_buf(),
        settings: SlamSettings::default(),
        reuse,
        history: false,
    }
}

/// Every file below `dir` (relative path → contents); empty if `dir` is missing.
fn files(dir: &Path) -> BTreeMap<String, Vec<u8>> {
    fn walk(base: &Path, dir: &Path, out: &mut BTreeMap<String, Vec<u8>>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for e in entries {
            let path = e.unwrap().path();
            if path.is_dir() {
                walk(base, &path, out);
            } else {
                let rel = path.strip_prefix(base).unwrap().display().to_string();
                out.insert(rel, fs::read(&path).unwrap());
            }
        }
    }
    let mut out = BTreeMap::new();
    walk(dir, dir, &mut out);
    out
}

/// Progress that cancels itself once `phase` has counted `after` items.
fn cancel_at(phase: &'static str, after: u64) -> Progress {
    let p = Progress::new();
    let handle = p.clone();
    p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == phase && s.done >= after {
            handle.cancel();
        }
    })
}

#[test]
fn slam_writes_cli_outputs() {
    let dir = tmp("steps-slam-outputs");
    room_bag(&dir, true);
    let out = dir.join("out");
    let p = Progress::new();
    let outcome = run_slam(&request(&dir, &out, false), &p).unwrap();
    for f in [
        "traj.tum",
        "traj_odom.tum",
        "map.las",
        "slam.json",
        "submaps.bin",
    ] {
        assert!(out.join(f).is_file(), "{f} missing");
    }
    assert_eq!(outcome.report.submaps.len(), 1);
    assert!(outcome.map_points > 1000, "{}", outcome.map_points);
    assert!(outcome.seconds > 0.0);
    assert_eq!(outcome.history, None);
    assert_eq!(
        fs::read_to_string(out.join("slam.json")).unwrap(),
        serde_json::to_string_pretty(&outcome.report).unwrap() + "\n"
    );

    // the console lines of `driftless slam`, in order
    let log = p.take_log();
    let infos: Vec<&str> = log
        .iter()
        .filter_map(|l| match l {
            LogLine::Info(s) => Some(s.as_str()),
            LogLine::Warning(_) => None,
        })
        .collect();
    assert_eq!(infos.len(), 4, "{log:?}");
    assert!(
        infos[0].starts_with("15 Posen aus 15 LiDAR-Frames"),
        "{infos:?}"
    );
    assert!(infos[1].starts_with("1 Teilkarten, "), "{infos:?}");
    assert!(infos[1].ends_with("keine Schleife gefunden (0 Kandidaten verworfen)"));
    assert!(infos[2].starts_with("Start↔Ende "), "{infos:?}");
    assert!(infos[3].starts_with("Laufzeit "), "{infos:?}");
    assert!(infos[3].ends_with(&format!("→ {}", out.display())));
}

#[test]
fn slam_cancelled_writes_nothing() {
    let dir = tmp("steps-slam-cancel");
    room_bag_secs(&dir, true, 3.0);
    let out = dir.join("out");

    let err = run_slam(&request(&dir, &out, false), &cancel_at("LiDAR lesen", 5)).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert!(files(&out).is_empty(), "{:?}", files(&out).keys());

    // a cancelled run over a finished one leaves it as it was
    run_slam(&request(&dir, &out, false), &Progress::new()).unwrap();
    let before = files(&out);
    let err = run_slam(&request(&dir, &out, false), &cancel_at("LiDAR lesen", 5)).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(files(&out), before);

    // also over a run with other settings, whose submaps.bin a full run would replace
    let mut other = request(&dir, &out, false);
    other.settings.lidar_to_imu = Some([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
    run_slam(&other, &Progress::new()).unwrap();
    let before = files(&out);
    let err = run_slam(&request(&dir, &out, false), &cancel_at("LiDAR lesen", 5)).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(files(&out), before);
}

/// Once `submaps.bin` is replaced, a full run finishes even when cancelled: the folder never
/// holds a cache of one run next to the trajectory of another.
#[test]
fn full_run_finishes_after_the_cache_is_written() {
    let dir = tmp("steps-slam-late-cancel");
    room_bag(&dir, true);
    let out = dir.join("out");
    let p = cancel_at("Schleifen schließen", 0);
    let outcome = run_slam(&request(&dir, &out, false), &p).unwrap();
    assert!(p.is_cancelled());
    assert_eq!(outcome.report.submaps.len(), 1);
    let reference = dir.join("reference");
    run_slam(&request(&dir, &reference, false), &Progress::new()).unwrap();
    for f in ["traj.tum", "slam.json", "submaps.bin", "map.las"] {
        assert_eq!(
            fs::read(out.join(f)).unwrap(),
            fs::read(reference.join(f)).unwrap(),
            "{f}"
        );
    }
}

#[test]
fn reuse_cancelled_keeps_traj() {
    let dir = tmp("steps-reuse-cancel");
    room_bag(&dir, true);
    let out = dir.join("out");
    run_slam(&request(&dir, &out, false), &Progress::new()).unwrap();
    let before = files(&out);

    // cancelled while the pose graph is solved: the last check before anything is renamed
    let err = run_slam(
        &request(&dir, &out, true),
        &cancel_at("Schleifen schließen", 0),
    )
    .unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert!(!out.join("traj.vorher.tum").exists());
    assert_eq!(files(&out), before);

    // cancelled up front
    let p = Progress::new();
    p.cancel();
    assert_eq!(
        run_slam(&request(&dir, &out, true), &p).unwrap_err(),
        StepError::Cancelled
    );
    assert_eq!(files(&out), before);
}

#[test]
fn progress_counts_frames() {
    let dir = tmp("steps-progress");
    room_bag(&dir, true);
    let out = dir.join("out");
    let last: Arc<Mutex<Option<ProgressSnapshot>>> = Arc::default();
    let seen = last.clone();
    let p = Progress::new().with_observer(move |s| {
        if s.phase == "LiDAR lesen" {
            *seen.lock().unwrap() = Some(s.clone());
        }
    });
    run_slam(&request(&dir, &out, false), &p).unwrap();
    let last = last
        .lock()
        .unwrap()
        .clone()
        .expect("phase LiDAR lesen reported");
    assert_eq!(last.total, Some(15));
    assert_eq!(last.done, 15);
}
