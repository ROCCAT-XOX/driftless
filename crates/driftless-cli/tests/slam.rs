mod common;

use assert_cmd::Command;
use common::{room_bag, room_bag_at, tmp};
use predicates::prelude::*;
use std::fs;
use std::path::Path;

fn slam(dir: &Path, out: &Path) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("slam")
        .arg(dir)
        .arg("-o")
        .arg(out)
        .assert()
}

#[test]
fn slam_writes_all_outputs() {
    let dir = tmp("basic");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out)
        .success()
        .stdout(predicate::str::contains("15 Posen"))
        .stdout(predicate::str::contains("1 Teilkarten"))
        .stdout(predicate::str::contains("keine Schleife gefunden"))
        .stdout(predicate::str::contains("Start↔Ende"))
        .stdout(predicate::str::contains("Laufzeit"));

    let traj = driftless_traj::read_tum(&out.join("traj.tum")).unwrap();
    let odom = driftless_traj::read_tum(&out.join("traj_odom.tum")).unwrap();
    assert_eq!(traj.len(), 15);
    assert_eq!(traj, odom, "single submap: corrected = odometry");
    assert!(
        traj.iter().all(|p| p.p.norm() < 0.02),
        "static sensor drifted"
    );

    let mut las = las::Reader::from_path(out.join("map.las")).unwrap();
    assert!(las.header().number_of_points() > 1000);
    let p = las.read_points(1).unwrap();
    let p = p.points().next().unwrap().unwrap();
    assert!(p.x.abs() < 7.0 && p.y.abs() < 6.0 && p.z.abs() < 3.0);

    let json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(out.join("slam.json")).unwrap()).unwrap();
    // absolute start of the scan: the stamp of the first pose (bags start at 1000 s)
    let scan_start = json["scan_start"].as_f64().unwrap();
    assert!(
        (scan_start - traj[0].t).abs() < 1e-6,
        "scan_start {scan_start}"
    );
    assert!(traj[0].t >= 1000.0, "{}", traj[0].t);
    let submaps = json["submaps"].as_array().unwrap();
    assert_eq!(submaps.len(), 1);
    assert_eq!(submaps[0]["t_start"], 0.0);
    let t_end = submaps[0]["t_end"].as_f64().unwrap();
    assert!((t_end - 1.4).abs() < 1e-6, "t_end {t_end}");
    assert_eq!(submaps[0]["anchor"].as_array().unwrap().len(), 3);
    assert_eq!(submaps[0]["odom_anchor"], submaps[0]["anchor"]);
    assert!(json["loops"].as_array().unwrap().is_empty());
    assert!(json["graph"].is_null());
    assert!(json["start_end_gap"]["before"].is_number());
    assert!(json["path_length"]["after"].is_number());
}

#[test]
fn slam_is_byte_identical_on_rerun() {
    let dir = tmp("rerun");
    room_bag(&dir, true);
    let (a, b) = (dir.join("a"), dir.join("b"));
    slam(&dir, &a).success();
    slam(&dir, &b).success();
    for f in ["traj.tum", "traj_odom.tum", "slam.json"] {
        assert_eq!(
            fs::read(a.join(f)).unwrap(),
            fs::read(b.join(f)).unwrap(),
            "{f}"
        );
    }
}

#[test]
fn slam_rejects_missing_imu() {
    let dir = tmp("no-imu");
    room_bag(&dir, false);
    slam(&dir, &dir.join("out"))
        .failure()
        .stderr(predicate::str::contains("keine IMU-Messungen"));
    assert!(!dir.join("out/traj.tum").exists());
}

/// `-o` of another scan's run (README: one folder per scan): refused in German, nothing
/// replaced, its corrections not applied.
#[test]
fn slam_refuses_the_folder_of_another_scan() {
    let dir = tmp("slam-fremd");
    let (a, b) = (dir.join("a"), dir.join("b"));
    fs::create_dir_all(&a).unwrap();
    fs::create_dir_all(&b).unwrap();
    room_bag(&a, true);
    room_bag_at(&b, true, 5000.0, 1.5);
    let out = dir.join("out");
    slam(&a, &out).success();
    let traj = fs::read(out.join("traj.tum")).unwrap();
    slam(&b, &out)
        .failure()
        .stderr(predicate::str::contains("Fehler: "))
        .stderr(predicate::str::contains(
            "enthält den Lauf eines anderen Scans",
        ))
        .stderr(predicate::str::contains("eigenen Zielordner"));
    assert_eq!(fs::read(out.join("traj.tum")).unwrap(), traj);
    slam(&b, &b.join("driftless")).success();
}

/// A full run into a folder that holds a run is refused: its trajectory (and a map made from
/// it) would be replaced without a word. `--ueberschreiben` replaces it, `--reuse` only
/// recomputes it; a folder with other files only is fine, an unfinished run needs
/// `--ueberschreiben` too.
#[test]
fn slam_refuses_an_existing_run() {
    let dir = tmp("slam-vorhanden");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out).success();
    let traj = fs::read(out.join("traj.tum")).unwrap();
    // A trajectory changed since (corrections, other settings): it must stay.
    let changed = [traj.as_slice(), b"# geaendert\n"].concat();
    fs::write(out.join("traj.tum"), &changed).unwrap();
    slam(&dir, &out)
        .failure()
        .stderr(predicate::str::contains(format!(
            "Fehler: {} enthält schon einen Lauf (submaps.bin, traj.tum, traj_odom.tum, \
             map.las, slam.json) und wird nicht überschrieben",
            out.display()
        )))
        .stderr(predicate::str::contains("--ueberschreiben"))
        .stderr(predicate::str::contains("--reuse"));
    assert_eq!(fs::read(out.join("traj.tum")).unwrap(), changed);

    Command::cargo_bin("driftless")
        .unwrap()
        .arg("slam")
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .arg("--ueberschreiben")
        .assert()
        .success();
    assert_eq!(fs::read(out.join("traj.tum")).unwrap(), traj);
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("slam")
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .arg("--reuse")
        .assert()
        .success();
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["slam", "x", "-o", "y", "--reuse", "--ueberschreiben"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("--ueberschreiben"));

    // Unfinished (killed before `slam.json`): said so, refused as well.
    fs::remove_file(out.join("slam.json")).unwrap();
    slam(&dir, &out).failure().stderr(predicate::str::contains(
        "enthält schon einen unfertigen Lauf",
    ));
    // Other files only (no run file): fine.
    let fresh = dir.join("andere-dateien");
    fs::create_dir_all(&fresh).unwrap();
    fs::write(fresh.join("notiz.txt"), "").unwrap();
    slam(&dir, &fresh).success();
}
