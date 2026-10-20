use assert_cmd::Command;
use predicates::prelude::*;
use serde_json::Value;
use std::path::PathBuf;

fn tmp(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("driftless-compare-{}-{name}", std::process::id()))
}

fn write(name: &str, rows: &[(f64, f64, f64, f64)]) -> PathBuf {
    let path = tmp(name);
    let mut s = String::from("# timestamp tx ty tz qx qy qz qw\n");
    for (t, x, y, z) in rows {
        s.push_str(&format!("{t} {x} {y} {z} 0 0 0 1\n"));
    }
    std::fs::write(&path, s).unwrap();
    path
}

fn path_rows(shift: f64) -> Vec<(f64, f64, f64, f64)> {
    (0..20)
        .map(|i| {
            let t = i as f64;
            (t, t.cos() * 2.0 + shift, t.sin() + 0.1 * t, 0.05 * t * t)
        })
        .collect()
}

#[test]
fn compare_text_output() {
    let a = write("a.tum", &path_rows(0.0));
    let b = write("b.tum", &path_rows(3.0));
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["compare", a.to_str().unwrap(), b.to_str().unwrap()])
        .assert()
        .success()
        .stdout(predicate::str::contains("Abweichung (ATE, RMSE): 0.000 m"))
        .stdout(predicate::str::contains("Start↔Ende a/b"))
        .stdout(predicate::str::contains("Länge a/b"))
        .stdout(predicate::str::contains("Zugeordnete Posen: 20"));
}

#[test]
fn compare_json_output() {
    let a = write("ja.tum", &path_rows(0.0));
    let out = Command::cargo_bin("driftless")
        .unwrap()
        .args([
            "compare",
            "--json",
            a.to_str().unwrap(),
            a.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    assert!(out.status.success());
    let v: Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(v["ate_rmse_m"].as_f64().unwrap() < 1e-9);
    assert_eq!(v["matched_poses"], 20);
    // Counts are integers in JSON, not floats.
    assert_eq!(v["poses"]["a"], 20);
    assert_eq!(v["poses"]["b"], 20);
    assert!(v["poses"]["a"].is_u64());
    assert!(v["path_length_m"]["a"].as_f64().unwrap() > 0.0);
}

#[test]
fn compare_reports_unmatched_and_bad_files() {
    let a = write("ua.tum", &path_rows(0.0));
    let far: Vec<_> = path_rows(0.0)
        .into_iter()
        .map(|(t, x, y, z)| (t + 500.0, x, y, z))
        .collect();
    let b = write("ub.tum", &far);
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["compare", a.to_str().unwrap(), b.to_str().unwrap()])
        .assert()
        .failure()
        .stderr(predicate::str::contains("zu wenige zugeordnete Posen"));
    let bad = tmp("bad.tum");
    std::fs::write(&bad, "1 2 3\n").unwrap();
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["compare", bad.to_str().unwrap(), a.to_str().unwrap()])
        .assert()
        .failure()
        .stderr(predicate::str::contains("Zeile 1"));
}
