//! `driftless plan` on synthetic rooms: a map folder without any raw data.

mod common;

use assert_cmd::Command;
use common::tmp;
use driftless_export::testutil::write_map_folder;
use driftless_plan::testutil::{Scene, TestData};
use driftless_steps::plan::{read_plan, PLAN_REPORT, TOP_VIEW_FILE};
use predicates::prelude::*;
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
