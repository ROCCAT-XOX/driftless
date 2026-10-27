//! `driftless merge` on two synthetic scans of one furnished room from two places.

mod common;

use assert_cmd::Command;
use common::{furnished_room_bag, tmp};
use predicates::prelude::*;
use std::path::Path;

fn driftless(args: &[&std::ffi::OsStr]) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless").unwrap().args(args).assert()
}

/// Scan folder with SLAM and map in `<scan>/driftless`.
fn prepare(scan: &Path) {
    let run = scan.join("driftless");
    driftless(&[
        "slam".as_ref(),
        scan.as_os_str(),
        "-o".as_ref(),
        run.as_os_str(),
    ])
    .success();
    driftless(&[
        "map".as_ref(),
        scan.as_os_str(),
        "--slam".as_ref(),
        run.as_os_str(),
    ])
    .success();
}

#[test]
fn merges_two_scans_and_checks_the_folders() {
    let dir = tmp("merge-cli");
    let (a, b) = (dir.join("raum-a"), dir.join("raum-b"));
    std::fs::create_dir_all(&a).unwrap();
    std::fs::create_dir_all(&b).unwrap();
    furnished_room_bag(&a, true, [0.0; 3], 0.0);
    furnished_room_bag(&b, true, [0.8, 0.5, 0.0], 30f64.to_radians());
    prepare(&a);
    prepare(&b);

    // `--slam` once for two scans.
    let out = dir.join("modell");
    let slam_a = a.join("driftless");
    driftless(&[
        "merge".as_ref(),
        a.as_os_str(),
        b.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
        "--slam".as_ref(),
        slam_a.as_os_str(),
    ])
    .failure()
    .stderr(predicate::str::contains("--slam").and(predicate::str::contains("je Scan")));
    assert!(!out.exists());

    driftless(&[
        "merge".as_ref(),
        a.as_os_str(),
        b.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ])
    .success()
    .stdout(predicate::str::contains("raum-a: Bezug"))
    .stdout(predicate::str::contains("raum-b: sicher"))
    .stdout(predicate::str::contains("Modell:"));
    assert!(out.join("modell.json").is_file());
    assert!(out.join("modell.las").is_file());
    assert!(out.join("modell.ply").is_file());

    // Nothing is overwritten.
    driftless(&[
        "merge".as_ref(),
        a.as_os_str(),
        b.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ])
    .failure()
    .stderr(predicate::str::contains("schon ein Modell"));
}
