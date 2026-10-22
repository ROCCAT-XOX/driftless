//! `driftless surface` on the synthetic room scan with camera images.

mod common;

use assert_cmd::Command;
use common::{scan_with_images, tmp};
use predicates::prelude::*;
use std::fs;
use std::path::Path;

fn run(cmd: &str, dir: &Path, out: &Path, extra: &[&str]) -> assert_cmd::assert::Assert {
    let flag = if cmd == "slam" { "-o" } else { "--slam" };
    Command::cargo_bin("driftless")
        .unwrap()
        .arg(cmd)
        .arg(dir)
        .arg(flag)
        .arg(out)
        .args(extra)
        .assert()
}

#[test]
fn surface_writes_files_and_refuses_a_missing_map() {
    let dir = tmp("surface-cli");
    scan_with_images(&dir);
    let out = dir.join("out");
    run("slam", &dir, &out, &[]).success();

    // no map yet
    run("surface", &dir, &out, &["--voxel", "0.15"])
        .failure()
        .stderr(predicate::str::contains("Karte fehlt oder ist veraltet"));
    assert!(!out.join("oberflaeche").exists());

    run("map", &dir, &out, &["--step", "clean"]).success();
    run("surface", &dir, &out, &["--voxel", "0.15"])
        .success()
        .stdout(predicate::str::contains("Oberfläche:"))
        .stdout(predicate::str::contains("Laufzeit"));
    let dest = out.join("oberflaeche");
    for f in [
        "oberflaeche.glb",
        "oberflaeche.obj",
        "oberflaeche.mtl",
        "oberflaeche.json",
    ] {
        assert!(dest.join(f).is_file(), "{f} missing");
    }
    let json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(dest.join("oberflaeche.json")).unwrap()).unwrap();
    assert_eq!(json["settings"]["voxel"], 0.15);
    assert_eq!(json["axes"], "glb: Y oben; obj: Z oben");

    // another target and no texture
    let other = dir.join("grau");
    run(
        "surface",
        &dir,
        &out,
        &[
            "--voxel",
            "0.15",
            "--no-texture",
            "-o",
            other.to_str().unwrap(),
        ],
    )
    .success()
    .stderr(predicate::str::contains("Warnung").not());
    assert!(other.join("oberflaeche.glb").is_file());
}
