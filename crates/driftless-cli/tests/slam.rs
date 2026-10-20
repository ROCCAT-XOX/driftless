mod common;

use assert_cmd::Command;
use common::{room_bag, tmp};
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
