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

fn report(dir: &Path) -> serde_json::Value {
    serde_json::from_str(&fs::read_to_string(dir.join("oberflaeche.json")).unwrap()).unwrap()
}

fn has_gpu() -> bool {
    driftless_gpu::GpuContext::new(driftless_gpu::GpuPref::Auto)
        .unwrap()
        .is_some()
}

#[test]
fn surface_json_reports_gpu() {
    let dir = tmp("surface-gpu-report");
    scan_with_images(&dir);
    let out = dir.join("out");
    run("slam", &dir, &out, &[]).success();
    run("map", &dir, &out, &["--step", "clean"]).success();
    let auto = dir.join("auto");
    let assert = run(
        "surface",
        &dir,
        &out,
        &["--voxel", "0.15", "-o", auto.to_str().unwrap()],
    )
    .success();
    let json = report(&auto);
    let gpu = &json["gpu"];
    for k in ["used", "name", "backend", "driver"] {
        assert!(gpu.get(k).is_some(), "gpu.{k} missing: {gpu}");
    }
    assert_eq!(gpu["used"], has_gpu(), "{gpu}");
    for k in ["quality", "pass"] {
        assert!(gpu["seconds"][k].is_number(), "gpu.seconds.{k}: {gpu}");
    }
    if has_gpu() {
        assert!(gpu["name"].as_str().is_some_and(|n| !n.is_empty()), "{gpu}");
        assert.stdout(predicate::str::contains("Ansichten auf der Grafikkarte"));
    } else {
        assert.stdout(predicate::str::contains("Ansichten auf der CPU"));
        run("surface", &dir, &out, &["--voxel", "0.15", "--gpu", "an"])
            .failure()
            .stderr(predicate::str::contains("Grafikkarte"));
    }
    // The CPU: the same input id (the GPU is no part of it) and the same faces.
    let cpu = dir.join("cpu");
    run(
        "surface",
        &dir,
        &out,
        &[
            "--voxel",
            "0.15",
            "--gpu",
            "aus",
            "-o",
            cpu.to_str().unwrap(),
        ],
    )
    .success()
    .stdout(predicate::str::contains("Grafikkarte").not());
    let off = report(&cpu);
    assert_eq!(off["gpu"]["used"], false);
    assert!(off["gpu"]["name"].is_null());
    assert_eq!(off["input_id"], json["input_id"]);
    assert_eq!(off["stats"]["faces_final"], json["stats"]["faces_final"]);
    let textured =
        |j: &serde_json::Value| j["stats"]["texture"]["faces_textured"].as_f64().unwrap();
    assert!((textured(&off) - textured(&json)).abs() <= 0.01 * textured(&off));
    run("surface", &dir, &out, &["--gpu", "vielleicht"])
        .failure()
        .stderr(predicate::str::contains("auto|an|aus"));
}

#[test]
fn surface_falls_back_when_gpu_budget_exceeded() {
    let dir = tmp("surface-gpu-budget");
    scan_with_images(&dir);
    let out = dir.join("out");
    run("slam", &dir, &out, &[]).success();
    run("map", &dir, &out, &["--step", "clean"]).success();
    let (cpu, small) = (dir.join("cpu"), dir.join("small"));
    run(
        "surface",
        &dir,
        &out,
        &[
            "--voxel",
            "0.15",
            "--gpu",
            "aus",
            "-o",
            cpu.to_str().unwrap(),
        ],
    )
    .success();
    // A budget of 1 kB: `auto` says why and looks on the CPU, with the same files.
    let budget = |extra: &[&str]| {
        Command::cargo_bin("driftless")
            .unwrap()
            .env("DRIFTLESS_GPU_BUDGET", "1000")
            .arg("surface")
            .arg(&dir)
            .arg("--slam")
            .arg(&out)
            .args(["--voxel", "0.15", "-o", small.to_str().unwrap()])
            .args(extra)
            .assert()
    };
    let a = budget(&[]).success();
    let files: Vec<String> = report(&cpu)["files"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| f.as_str().unwrap().to_string())
        .filter(|f| f != "oberflaeche.json")
        .collect();
    assert!(files.len() > 3, "{files:?}");
    for f in &files {
        assert_eq!(
            fs::read(small.join(f)).unwrap(),
            fs::read(cpu.join(f)).unwrap(),
            "{f}"
        );
    }
    let gpu = &report(&small)["gpu"];
    assert_eq!(gpu["used"], false, "{gpu}");
    if has_gpu() {
        a.stdout(predicate::str::contains("Ansichten auf der CPU"));
        assert!(gpu["fallback"]
            .as_str()
            .is_some_and(|w| w.contains("Grafikspeicher")));
        budget(&["--gpu", "an"])
            .failure()
            .stderr(predicate::str::contains("Grafikspeicher"));
    }
}

/// `--gpu auto` without a usable device writes why into the report (no DX12 outside Windows:
/// `WGPU_BACKEND=dx12` hides every adapter); `--gpu aus` writes no reason.
#[cfg(not(windows))]
#[test]
fn surface_report_says_why_auto_took_the_cpu() {
    let dir = tmp("surface-gpu-none");
    scan_with_images(&dir);
    let out = dir.join("out");
    run("slam", &dir, &out, &[]).success();
    run("map", &dir, &out, &["--step", "clean"]).success();
    let none = dir.join("none");
    Command::cargo_bin("driftless")
        .unwrap()
        .env("WGPU_BACKEND", "dx12")
        .arg("surface")
        .arg(&dir)
        .arg("--slam")
        .arg(&out)
        .args(["--voxel", "0.3", "-o", none.to_str().unwrap()])
        .assert()
        .success()
        .stdout(predicate::str::contains("Ansichten auf der CPU"));
    let gpu = &report(&none)["gpu"];
    assert_eq!(gpu["used"], false, "{gpu}");
    assert_eq!(gpu["fallback"], "keine Grafikkarte gefunden", "{gpu}");
    run(
        "surface",
        &dir,
        &out,
        &[
            "--voxel",
            "0.3",
            "--gpu",
            "aus",
            "-o",
            none.to_str().unwrap(),
        ],
    )
    .success();
    assert!(report(&none)["gpu"]["fallback"].is_null());
}
