//! `driftless plan` on synthetic rooms: a map folder without any raw data.

mod common;

use assert_cmd::Command;
use common::tmp;
use driftless_export::testutil::write_map_folder;
use driftless_plan::testutil::{Scene, TestData};
use driftless_steps::objects::{ObjectInput, ObjectKind, Objects};
use driftless_steps::plan::{read_plan, PLAN_REPORT, TOP_VIEW_FILE};
use predicates::prelude::*;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::sync::OnceLock;

const FLOOR: f64 = -1.3;
const LOOP: [[f64; 3]; 4] = [
    [1.0, 1.0, 0.0],
    [4.0, 1.0, 0.0],
    [4.0, 3.0, 0.0],
    [1.0, 3.0, 0.0],
];

fn one_room() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        Scene::new()
            .spacing(0.015)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, 2.6)
            .walk(&LOOP)
            .build()
    })
}

fn two_storeys() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        let upper = LOOP.map(|p| [p[0], p[1], 2.9]);
        Scene::new()
            .spacing(0.015)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, 2.6)
            .room([0.0, 0.0], [5.0, 4.0], 1.6, 2.6)
            .walk(&LOOP)
            .walk(&upper)
            .build()
    })
}

fn plan(args: &[&str], source: &Path) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("plan")
        .arg(source)
        .args(args)
        .assert()
}

#[test]
fn plan_writes_folder() {
    let dir = tmp("plan-cli");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    plan(&[], &dir)
        .success()
        .stdout(predicate::str::contains("Grundriss:"))
        .stdout(predicate::str::contains("Laufzeit"));
    let dest = dir.join("grundriss");
    for f in [PLAN_REPORT, TOP_VIEW_FILE] {
        assert!(dest.join(f).is_file(), "{f} fehlt");
    }
    assert_eq!(read_plan(&dest).unwrap().plan.rooms.len(), 1);

    let other = dir.join("woanders");
    plan(&["-o", other.to_str().unwrap()], &dir).success();
    assert!(other.join(PLAN_REPORT).is_file());
}

#[test]
fn second_run_says_it_fits() {
    let dir = tmp("plan-cli-fits");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    plan(&[], &dir).success();
    let report = dir.join("grundriss").join(PLAN_REPORT);
    let before = fs::read(&report).unwrap();
    plan(&[], &dir)
        .success()
        .stdout(predicate::str::contains("passt schon"))
        .stdout(predicate::str::contains("Laufzeit").not());
    assert_eq!(fs::read(&report).unwrap(), before, "nicht neu gerechnet");
}

#[test]
fn several_storeys_fail_with_the_list() {
    let dir = tmp("plan-cli-storeys");
    let data = two_storeys();
    write_map_folder(&dir, &data.xyz, &data.poses);
    plan(&[], &dir)
        .failure()
        .stderr(predicate::str::contains("Mehrere Stockwerke"))
        .stderr(predicate::str::contains(
            "1 · Boden -1,30 m · Decke 1,30 m · ",
        ))
        .stderr(predicate::str::contains(
            "2 · Boden 1,60 m · Decke 4,20 m · ",
        ))
        .stderr(predicate::str::contains(" % der Laufbahn"))
        .stderr(predicate::str::contains("--stockwerk"));
    assert!(!dir.join("grundriss").exists(), "nichts geschrieben");
}

#[test]
fn stockwerk_picks_one() {
    let dir = tmp("plan-cli-pick");
    let data = two_storeys();
    write_map_folder(&dir, &data.xyz, &data.poses);
    plan(&["--stockwerk", "2"], &dir).success();
    let report = read_plan(&dir.join("grundriss")).unwrap();
    assert!((report.plan.storey.floor_z - 1.6).abs() < 0.02);
    assert_eq!(report.settings.storey, Some(2));

    plan(&["--stockwerk", "3"], &dir)
        .failure()
        .stderr(predicate::str::contains("Stockwerk 3 gibt es nicht"));
}

#[test]
fn bad_source_says_why() {
    let dir = tmp("plan-cli-bad");
    plan(&[], &dir)
        .failure()
        .stderr(predicate::str::contains("weder eine Karte"));
    plan(&[], &dir.join("fehlt"))
        .failure()
        .stderr(predicate::str::contains("weder eine Karte"));
    assert!(!dir.join("grundriss").exists());
}

/// A socket at (2, 2) in the map folder `dir`, 30 cm above the floor.
fn put_socket(dir: &Path) {
    let mut objects = Objects::default();
    objects
        .add(ObjectInput {
            kind: ObjectKind::Socket,
            points: vec![[2.0, 2.0, FLOOR + 0.3]],
            name: "Bad".to_string(),
            note: String::new(),
            connection: None,
            diameter_mm: None,
            height_m: None,
            sill_m: None,
            properties: BTreeMap::new(),
        })
        .unwrap();
    objects.save(dir).unwrap();
}

/// Group code and value of every pair of a DXF file, values as Latin-1.
fn pairs(path: &Path) -> Vec<(i32, String)> {
    let bytes = fs::read(path).unwrap();
    let text: String = bytes.iter().map(|&b| b as char).collect();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len() % 2, 0, "je Gruppe zwei Zeilen");
    lines
        .chunks(2)
        .map(|p| (p[0].trim().parse().unwrap(), p[1].to_string()))
        .collect()
}

#[test]
fn plan_dxf_writes_the_file() {
    let dir = tmp("plan-cli-dxf");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    put_socket(&dir);
    let out = dir.join("grundriss.dxf");
    plan(&["--dxf", out.to_str().unwrap()], &dir)
        .success()
        .stdout(predicate::str::contains("Elemente → "));
    let all = pairs(&out);
    assert!(all.contains(&(1, "AC1015".to_string())));
    assert!(all.iter().any(|p| p.0 == 9 && p.1 == "$INSUNITS"));
    assert!(all.iter().any(|p| p.0 == 2 && p.1 == "Steckdose"));
    assert!(
        all.iter().any(|p| p.0 == 0 && p.1 == "INSERT"),
        "die Steckdose"
    );
    assert!(all.iter().any(|p| p.0 == 8 && p.1 == "Wand"));
    assert!(dir.join("grundriss").join(PLAN_REPORT).is_file());

    // A plan that fits is not computed again, the DXF is written anyway.
    let again = dir.join("zweite.dxf");
    plan(&["--dxf", again.to_str().unwrap()], &dir)
        .success()
        .stdout(predicate::str::contains("passt schon"));
    assert!(again.is_file());

    // The plan elsewhere: the objects still come from the map.
    let other = tmp("plan-cli-dxf-woanders");
    let out = other.join("woanders.dxf");
    plan(
        &[
            "-o",
            other.join("plan").to_str().unwrap(),
            "--dxf",
            out.to_str().unwrap(),
        ],
        &dir,
    )
    .success();
    assert!(pairs(&out).iter().any(|p| p.0 == 0 && p.1 == "INSERT"));
}

#[test]
fn lage_punktwolke() {
    let dir = tmp("plan-cli-lage");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let walls = dir.join("waende.dxf");
    let cloud = dir.join("wolke.dxf");
    plan(&["--dxf", walls.to_str().unwrap()], &dir).success();
    plan(
        &["--dxf", cloud.to_str().unwrap(), "--lage", "punktwolke"],
        &dir,
    )
    .success();
    let height = |path: &Path| -> f64 {
        let all = pairs(path);
        let start = all.iter().position(|p| p.1 == "ENTITIES").unwrap();
        let value = &all[start..]
            .iter()
            .find(|p| p.0 == 38)
            .expect("LWPOLYLINE")
            .1;
        value.parse().unwrap()
    };
    assert!(height(&walls).abs() < 1e-9);
    assert!((height(&cloud) + 1300.0).abs() < 20.0, "{}", height(&cloud));

    plan(&["--lage", "irgendwo", "--dxf", "x.dxf"], &dir).failure();
    plan(&["--lage", "punktwolke"], &dir)
        .failure()
        .stderr(predicate::str::contains("--dxf"));
}

#[test]
fn dxf_exists_needs_ueberschreiben() {
    let dir = tmp("plan-cli-dxf-da");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let out = dir.join("plan.dxf");
    fs::write(&out, "alt").unwrap();
    let args = ["--dxf", out.to_str().unwrap()];
    plan(&args, &dir)
        .failure()
        .stderr(predicate::str::contains("gibt es schon"))
        .stderr(predicate::str::contains("--ueberschreiben"));
    assert_eq!(fs::read_to_string(&out).unwrap(), "alt");
    assert!(!dir.join("grundriss").exists(), "vor dem Rechnen geprüft");

    plan(&["--dxf", out.to_str().unwrap(), "--ueberschreiben"], &dir).success();
    assert!(fs::read(&out).unwrap().starts_with(b"999"));
}

#[test]
fn export_of_plan_folder() {
    let dir = tmp("plan-cli-export");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    plan(&[], &dir).success();
    put_socket(&dir);
    let out = dir.join("x.dxf");
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("export")
        .arg(dir.join("grundriss"))
        .arg("-o")
        .arg(&out)
        .assert()
        .success()
        .stdout(predicate::str::contains("Elemente → "));
    assert!(pairs(&out).iter().any(|p| p.0 == 0 && p.1 == "INSERT"));
}
