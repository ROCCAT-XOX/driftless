//! `driftless --version`: the version and the commit it was built from.

use assert_cmd::Command;

#[test]
fn version_has_commit() {
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
    let parts: Vec<&str> = version.split('.').collect();
    assert_eq!(parts.len(), 3, "{line}");
    assert!(
        parts
            .iter()
            .all(|p| !p.is_empty() && p.chars().all(|c| c.is_ascii_digit())),
        "{line}"
    );
    assert_eq!(version, env!("CARGO_PKG_VERSION"));
    let commit = commit
        .strip_prefix('(')
        .and_then(|c| c.strip_suffix(')'))
        .expect(line);
    assert!(!commit.is_empty() && !line.contains('\n'), "{line}");
}
