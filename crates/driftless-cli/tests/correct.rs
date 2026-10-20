//! `driftless correct` and `driftless slam --reuse`.

mod common;

use assert_cmd::Command;
use common::{room_bag, room_bag_secs, tmp};
use predicates::prelude::*;
use std::fs;
use std::path::Path;

fn driftless() -> Command {
    Command::cargo_bin("driftless").unwrap()
}

fn slam(dir: &Path, out: &Path, reuse: bool) -> assert_cmd::assert::Assert {
    let mut cmd = driftless();
    cmd.arg("slam").arg(dir).arg("-o").arg(out);
    if reuse {
        cmd.arg("--reuse");
    }
    cmd.assert()
}

fn correct(out: &Path, args: &[&str]) -> assert_cmd::assert::Assert {
    driftless().arg("correct").arg(out).args(args).assert()
}

fn slam_json(out: &Path) -> serde_json::Value {
    serde_json::from_str(&fs::read_to_string(out.join("slam.json")).unwrap()).unwrap()
}

const HINT: &str = "`driftless slam` ohne `--reuse` neu laufen lassen";

#[test]
fn reuse_is_fast_and_equal() {
    let dir = tmp("reuse");
    room_bag(&dir, true);
    let out = dir.join("out");

    // no cache yet
    slam(&dir, &out, true)
        .failure()
        .stderr(predicate::str::contains("kein Zwischenspeicher"))
        .stderr(predicate::str::contains(HINT));
    assert!(!out.join("traj.tum").exists());

    slam(&dir, &out, false).success();
    assert!(out.join("submaps.bin").exists());
    let first: Vec<_> = ["traj.tum", "traj_odom.tum", "slam.json", "map.las"]
        .iter()
        .map(|f| fs::read(out.join(f)).unwrap())
        .collect();

    slam(&dir, &out, true)
        .success()
        .stdout(predicate::str::contains(
            "Zwischenspeicher submaps.bin: 1 Teilkarten",
        ))
        .stdout(predicate::str::contains("Posen aus").not())
        .stdout(predicate::str::contains("Laufzeit"));
    for (f, before) in ["traj.tum", "traj_odom.tum", "slam.json", "map.las"]
        .iter()
        .zip(&first)
    {
        assert_eq!(&fs::read(out.join(f)).unwrap(), before, "{f}");
    }
    assert_eq!(fs::read(out.join("traj.vorher.tum")).unwrap(), first[0]);
}

#[test]
fn reuse_rejects_other_settings() {
    let dir = tmp("reuse-other");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false).success();
    driftless()
        .arg("slam")
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .arg("--reuse")
        .args(["--lidar-to-imu", "0.1", "0", "0", "0", "0", "0", "1"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("anderen Scan"))
        .stderr(predicate::str::contains(HINT));
    assert!(!out.join("traj.vorher.tum").exists(), "nothing touched");
}

#[test]
fn correct_loop_then_reuse() {
    // 12 s standing still: 120 frames, two submaps (100 + 20 frames, 0–9.9 s and 10–11.9 s)
    let dir = tmp("correct");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    correct(&out, &["list"])
        .failure()
        .stderr(predicate::str::contains("Zielordner"));
    slam(&dir, &out, false)
        .success()
        .stdout(predicate::str::contains("2 Teilkarten"));

    correct(&out, &["list"])
        .success()
        .stdout(predicate::str::contains("keine Korrekturen"));
    correct(&out, &["loop", "1.0", "11.0", "--note", "Startpunkt"])
        .success()
        .stdout(predicate::str::contains("Eintrag 1 angehängt"))
        .stdout(predicate::str::contains("--reuse"));
    let toml = fs::read_to_string(out.join("corrections.toml")).unwrap();
    assert!(
        toml.contains("[[loop]]") && toml.contains("note = \"Startpunkt\""),
        "{toml}"
    );

    // rejected loops leave the file as it is
    for (args, msg) in [
        (["loop", "1.0", "5.0"], "derselben Teilkarte"),
        (["loop", "1.0", "30.0"], "außerhalb des Scans"),
        (["loop", "-1.0", "11.0"], "außerhalb des Scans"),
        (["loop", "11.0", "1.0"], "doppelte Schleife"),
        (["loop", "2.0", "10.5"], "doppelte Schleife"),
    ] {
        correct(&out, &args)
            .failure()
            .stderr(predicate::str::contains(msg));
    }
    assert_eq!(
        fs::read_to_string(out.join("corrections.toml")).unwrap(),
        toml
    );

    correct(&out, &["list"])
        .success()
        .stdout(predicate::str::contains("1   aktiv"))
        .stdout(predicate::str::contains("Startpunkt"));

    let before = fs::read(out.join("traj.tum")).unwrap();
    slam(&dir, &out, true)
        .success()
        .stdout(predicate::str::contains("1 manuelle Schleifen"))
        .stdout(predicate::str::contains("(1 manuell)"))
        // standing still, the two submaps share too few planes for the ICP check
        .stderr(predicate::str::contains(
            "Stelle passt nicht sauber, nur Position übernommen",
        ));
    assert_eq!(fs::read(out.join("traj.vorher.tum")).unwrap(), before);
    let json = slam_json(&out);
    let loops = json["loops"].as_array().unwrap();
    assert_eq!(loops.len(), 1, "{loops:?}");
    assert_eq!(loops[0]["manual"], true);
    assert_eq!(loops[0]["accepted"], true);
    assert_eq!((&loops[0]["a"], &loops[0]["b"]), (&0.into(), &1.into()));

    // disable keeps the entry and takes the loop out again
    correct(&out, &["disable", "1"])
        .success()
        .stdout(predicate::str::contains("deaktiviert"));
    correct(&out, &["disable", "1"])
        .success()
        .stdout(predicate::str::contains("schon deaktiviert"));
    correct(&out, &["enable", "5"])
        .failure()
        .stderr(predicate::str::contains("kein Eintrag 5"));
    correct(&out, &["list"])
        .success()
        .stdout(predicate::str::contains("1   deaktiviert"));
    slam(&dir, &out, true).success();
    assert!(slam_json(&out)["loops"].as_array().unwrap().is_empty());
    let toml = fs::read_to_string(out.join("corrections.toml")).unwrap();
    assert!(
        toml.contains("enabled = false") && toml.contains("Startpunkt"),
        "{toml}"
    );

    // while entry 1 is off, the same pair may be entered again; switching 1 back on would
    // then join the same submaps twice and is refused like `loop`
    correct(&out, &["loop", "2.0", "10.5"])
        .success()
        .stdout(predicate::str::contains("Eintrag 2 angehängt"));
    let toml = fs::read_to_string(out.join("corrections.toml")).unwrap();
    correct(&out, &["enable", "1"])
        .failure()
        .stderr(predicate::str::contains("Eintrag 1 nicht eingeschaltet"))
        .stderr(predicate::str::contains("doppelte Schleife"));
    assert_eq!(
        fs::read_to_string(out.join("corrections.toml")).unwrap(),
        toml
    );
    correct(&out, &["disable", "2"]).success();

    correct(&out, &["enable", "1"]).success();
    slam(&dir, &out, true).success();
    assert_eq!(slam_json(&out)["loops"].as_array().unwrap().len(), 1);
}

/// A correction that no longer fits fails the full run only after the cache is written: the
/// message says how to switch it off and recompute.
#[test]
fn full_run_with_bad_correction_hints_disable_and_reuse() {
    let dir = tmp("bad-correction");
    room_bag(&dir, true);
    let out = dir.join("out");
    fs::create_dir_all(&out).unwrap();
    // the scan is 1.5 s long
    fs::write(
        out.join("corrections.toml"),
        "[[loop]]\na = 0.5\nb = 30.0\nenabled = true\n",
    )
    .unwrap();
    slam(&dir, &out, false)
        .failure()
        .stderr(predicate::str::contains("außerhalb des Scans"))
        .stderr(predicate::str::contains("disable <nr>"))
        .stderr(predicate::str::contains("--reuse"));
    assert!(out.join("submaps.bin").exists());
    correct(&out, &["disable", "1"]).success();
    slam(&dir, &out, true).success();
}

/// Write errors name the file.
#[test]
fn write_errors_name_the_file() {
    let dir = tmp("write-errors");
    room_bag(&dir, true);
    let out = dir.join("out");
    // a folder in the way of the cache
    fs::create_dir_all(out.join("submaps.bin").join("x")).unwrap();
    slam(&dir, &out, false)
        .failure()
        .stderr(predicate::str::contains("submaps.bin nicht schreibbar"));
    fs::remove_dir_all(out.join("submaps.bin")).unwrap();

    slam(&dir, &out, false).success();
    fs::create_dir_all(out.join("traj.vorher.tum").join("x")).unwrap();
    slam(&dir, &out, true)
        .failure()
        .stderr(predicate::str::contains("traj.tum nicht in"))
        .stderr(predicate::str::contains("traj.vorher.tum umbenennbar"));
}
