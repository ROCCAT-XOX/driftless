//! `driftless --version`: the version and the commit it was built from, the same as every
//! result names under `programm`.

mod common;

use assert_cmd::Command;
use common::{room_bag, tmp};

/// `(version, commit)` of `driftless --version`.
fn version_and_commit() -> (String, String) {
    let out = Command::cargo_bin("driftless")
        .unwrap()
        .arg("--version")
        .output()
        .unwrap();
    assert!(out.status.success());
    let line = String::from_utf8(out.stdout).unwrap();
    let line = line.trim_end();
    // ^driftless \d+\.\d+\.\d+ \(.+\)$
    let rest = line.strip_prefix("driftless ").expect(line);
    let (version, commit) = rest.split_once(' ').expect(line);
    let commit = commit
        .strip_prefix('(')
        .and_then(|c| c.strip_suffix(')'))
        .expect(line);
    assert!(!commit.is_empty() && !line.contains('\n'), "{line}");
    (version.to_string(), commit.to_string())
}

#[test]
fn version_has_commit() {
    let (version, _) = version_and_commit();
    let parts: Vec<&str> = version.split('.').collect();
    assert_eq!(parts.len(), 3, "{version}");
    assert!(
        parts
            .iter()
            .all(|p| !p.is_empty() && p.chars().all(|c| c.is_ascii_digit())),
        "{version}"
    );
    assert_eq!(version, env!("CARGO_PKG_VERSION"));
}

/// The result of a step names the program that made it, as `--version` does.
#[test]
fn results_name_the_program() {
    let dir = tmp("version-programm");
    room_bag(&dir, true);
    let out = dir.join("out");
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("slam")
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .assert()
        .success();
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(out.join("slam.json")).unwrap()).unwrap();
    let (version, commit) = version_and_commit();
    assert_eq!(
        json["programm"],
        serde_json::json!({"version": version, "commit": commit})
    );
}
