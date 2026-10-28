//! `driftless export` on synthetic point files: format from the extension or `--format`,
//! existing targets only with `--ueberschreiben`.

use assert_cmd::Command;
use driftless_export::testutil::synthetic_points;
use driftless_export::{read_points, write_points, Layout, PointFormat};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use predicates::prelude::*;
use std::path::{Path, PathBuf};

fn driftless(args: &[&std::ffi::OsStr]) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless").unwrap().args(args).assert()
}

/// `quelle.las` with 1500 coloured points in `dir`.
fn source(dir: &Path) -> PathBuf {
    let path = dir.join("quelle.las");
    let layout = Layout {
        rgb: true,
        intensity: true,
        ..Layout::default()
    };
    let points = synthetic_points(1_500, 9, layout);
    write_points(&path, PointFormat::Las, &points, &Progress::new()).unwrap();
    path
}

#[test]
fn export_writes_laz() {
    let dir = tmp("export-laz");
    let src = source(&dir);
    let out = dir.join("Büro Scan.laz");
    driftless(&[
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ])
    .success()
    .stdout(
        predicate::str::starts_with("1500 Punkte → ")
            .and(predicate::str::contains("Büro Scan.laz ("))
            .and(predicate::str::contains(" MB, ")),
    );
    let bytes = std::fs::read(&out).unwrap();
    assert_ne!(bytes[104] & 0x80, 0, "compressed");
    assert_eq!(read_points(&out, &Progress::new()).unwrap().len(), 1500);
}

#[test]
fn export_needs_ueberschreiben() {
    let dir = tmp("export-exists");
    let src = source(&dir);
    let out = dir.join("da.ply");
    std::fs::write(&out, b"alt").unwrap();
    let args = [
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ];
    driftless(&args)
        .failure()
        .stderr(predicate::str::contains(format!(
            "Fehler: {} gibt es schon – mit --ueberschreiben ersetzen",
            out.display()
        )));
    assert_eq!(std::fs::read(&out).unwrap(), b"alt");
    let mut again = args.to_vec();
    again.push("--ueberschreiben".as_ref());
    driftless(&again).success();
    assert_eq!(read_points(&out, &Progress::new()).unwrap().len(), 1500);
    // The source itself is never replaced.
    driftless(&[
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        src.as_os_str(),
        "--ueberschreiben".as_ref(),
    ])
    .failure()
    .stderr(predicate::str::contains("Ziel ist die Quelle selbst"));
}

#[test]
fn export_unknown_extension_lists_formats() {
    let dir = tmp("export-ext");
    let src = source(&dir);
    let out = dir.join("wolke.xyz");
    driftless(&[
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ])
    .failure()
    .stderr(predicate::str::contains(
        "Fehler: unbekannte Endung „.xyz“ – mit --format angeben: las, laz, e57, ply, pcd, \
             glb, obj, ply-netz, tum, csv",
    ));
    assert!(!out.exists());
    driftless(&[
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
        "--format".as_ref(),
        "xyz".as_ref(),
    ])
    .failure()
    .stderr(predicate::str::contains("unbekanntes Format „xyz“"));
}

#[test]
fn export_format_overrides_extension() {
    let dir = tmp("export-format");
    let src = source(&dir);
    let out = dir.join("wolke.txt");
    driftless(&[
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
        "--format".as_ref(),
        "PCD".as_ref(),
    ])
    .success();
    let bytes = std::fs::read(&out).unwrap();
    assert!(bytes.starts_with(b"# .PCD v0.7"));
    // A mesh format for points: the kinds that fit are named.
    let glb = dir.join("netz.glb");
    driftless(&[
        "export".as_ref(),
        src.as_os_str(),
        "-o".as_ref(),
        glb.as_os_str(),
    ])
    .failure()
    .stderr(predicate::str::contains(
        "glb (Y oben) passt nicht zu Punktwolke: möglich sind LAS 1.4",
    ));
}

#[test]
fn export_run_folder_that_is_also_the_map() {
    let dir = tmp("export-run-map");
    std::fs::write(dir.join("map.json"), "{}").unwrap();
    std::fs::write(
        dir.join("traj.tum"),
        "# timestamp tx ty tz qx qy qz qw\n1.000000000 0.000000000 0.000000000 0.000000000 \
         0.000000000 0.000000000 0.000000000 1.000000000\n",
    )
    .unwrap();
    let out = dir.join("bahn.csv");
    driftless(&[
        "export".as_ref(),
        dir.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ])
    .success()
    .stdout(predicate::str::starts_with("1 Lagen → "));
    assert_eq!(
        std::fs::read_to_string(&out).unwrap(),
        "stempel;zeit_s;x;y;z;qx;qy;qz;qw\n1,000000000;0,000;0,000000000;0,000000000;\
         0,000000000;0,000000000;0,000000000;0,000000000;1,000000000\n"
    );
}
