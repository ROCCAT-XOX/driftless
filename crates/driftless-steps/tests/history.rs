//! `verlauf/`: one state per SLAM run of the app, and switching corrections back.

use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag, room_bag_secs, tmp};
use driftless_slam::Corrections;
use driftless_steps::history::{self, HistoryKind};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings, CORRECTIONS_FILE};
use driftless_steps::{correct, StepError};
use std::fs;
use std::path::Path;

fn slam(scan: &Path, out: &Path, reuse: bool, history: bool) -> Option<std::path::PathBuf> {
    let req = SlamRequest {
        scan: scan.to_path_buf(),
        out: out.to_path_buf(),
        settings: SlamSettings::default(),
        reuse,
        history,
    };
    run_slam(&req, &Progress::new()).unwrap().history
}

fn stand(dir: &Path) -> toml::Table {
    toml::from_str(&fs::read_to_string(dir.join("stand.toml")).unwrap()).unwrap()
}

#[test]
fn history_records_each_run() {
    // 12 s standing still: two submaps, so a manual loop between them is allowed
    let dir = tmp("steps-history");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");

    let first = slam(&dir, &out, false, true).unwrap();
    assert_eq!(first, out.join("verlauf").join("001"));
    correct::add_loop(&out, 1.0, 11.0, "Startpunkt").unwrap();
    let second = slam(&dir, &out, true, true).unwrap();
    let with_loop = fs::read(out.join(CORRECTIONS_FILE)).unwrap();
    correct::set_enabled(&out, 1, false).unwrap();
    let third = slam(&dir, &out, true, true).unwrap();
    assert_eq!(third, out.join("verlauf").join("003"));

    let list = history::list(&out);
    let nrs: Vec<u32> = list.iter().map(|e| e.nr).collect();
    assert_eq!(nrs, [1, 2, 3]);
    let kinds: Vec<HistoryKind> = list.iter().map(|e| e.kind).collect();
    assert_eq!(
        kinds,
        [HistoryKind::Full, HistoryKind::Reuse, HistoryKind::Reuse]
    );
    assert_eq!(list[1].dir, second);
    for (e, art) in list.iter().zip(["voll", "neu gerechnet", "neu gerechnet"]) {
        let s = stand(&e.dir);
        assert_eq!(s["nr"].as_integer(), Some(i64::from(e.nr)));
        assert_eq!(s["art"].as_str(), Some(art));
        assert_eq!(s["erstellt"].as_str(), Some(e.created.as_str()));
        assert!(s["sekunden"].as_float().unwrap() >= 0.0);
        assert!(
            e.created.ends_with('Z') && e.created.len() == 20,
            "{}",
            e.created
        );
        for f in ["traj.tum", "slam.json", CORRECTIONS_FILE] {
            assert!(e.dir.join(f).is_file(), "{f} in {}", e.dir.display());
        }
    }

    // the corrections that were in force for each run
    let c1 = Corrections::load(&first.join(CORRECTIONS_FILE)).unwrap();
    assert!(c1.loops.is_empty());
    let header = fs::read_to_string(first.join(CORRECTIONS_FILE)).unwrap();
    assert!(header.starts_with("# Driftless-Korrekturen"), "{header}");
    assert_eq!(fs::read(second.join(CORRECTIONS_FILE)).unwrap(), with_loop);
    let c3 = Corrections::load(&third.join(CORRECTIONS_FILE)).unwrap();
    assert_eq!(c3.loops.len(), 1);
    assert!(!c3.loops[0].enabled);
    // and their results
    assert_eq!(
        fs::read(third.join("traj.tum")).unwrap(),
        fs::read(out.join("traj.tum")).unwrap()
    );
    assert_ne!(
        fs::read(second.join("traj.tum")).unwrap(),
        fs::read(third.join("traj.tum")).unwrap(),
        "the loop moved the trajectory"
    );
}

/// A run made outside the app (no `verlauf/`) is kept as a state before the app recomputes
/// it, so its trajectory is not lost when the next reuse replaces `traj.vorher.tum`.
#[test]
fn reuse_of_run_without_history_keeps_it_first() {
    let dir = tmp("steps-history-import");
    room_bag(&dir, true);
    let out = dir.join("out");
    assert_eq!(slam(&dir, &out, false, false), None);
    assert!(!out.join("verlauf").exists());
    let traj = fs::read(out.join("traj.tum")).unwrap();

    let recorded = slam(&dir, &out, true, true).unwrap();
    let list = history::list(&out);
    let kinds: Vec<HistoryKind> = list.iter().map(|e| e.kind).collect();
    assert_eq!(kinds, [HistoryKind::Imported, HistoryKind::Reuse]);
    assert_eq!(recorded, list[1].dir);
    assert_eq!(stand(&list[0].dir)["art"].as_str(), Some("übernommen"));
    assert_eq!(fs::read(list[0].dir.join("traj.tum")).unwrap(), traj);

    // only once
    slam(&dir, &out, true, true).unwrap();
    assert_eq!(history::list(&out).len(), 3);
}

fn write_corrections(path: &Path, flags: &[bool]) {
    let mut c = Corrections::default();
    for (i, on) in flags.iter().enumerate() {
        let t = i as f64;
        c.add_loop(
            t,
            t + 10.0,
            &format!("Nr {}", i + 1),
            "2026-10-21T10:00:00Z",
        );
        c.loops[i].enabled = *on;
    }
    c.save(path).unwrap();
}

#[test]
fn restore_sets_flags_never_removes() {
    let out = tmp("steps-restore");
    fs::write(out.join("traj.tum"), "1000 0 0 0 0 0 0 1\n").unwrap();
    fs::write(out.join("slam.json"), "{}\n").unwrap();
    let path = out.join(CORRECTIONS_FILE);
    write_corrections(&path, &[true]);
    let stand = history::record(&out, HistoryKind::Reuse, 1.5).unwrap();
    assert_eq!(stand, out.join("verlauf").join("001"));

    // later: entry 1 off, two more on
    write_corrections(&path, &[false, true, true]);
    assert_eq!(history::restore_flags(&out, 1).unwrap(), 3);
    let c = Corrections::load(&path).unwrap();
    let flags: Vec<bool> = c.loops.iter().map(|l| l.enabled).collect();
    assert_eq!(flags, [true, false, false]);
    let notes: Vec<&str> = c.loops.iter().map(|l| l.note.as_str()).collect();
    assert_eq!(notes, ["Nr 1", "Nr 2", "Nr 3"]);
    // again: nothing left to change
    assert_eq!(history::restore_flags(&out, 1).unwrap(), 0);

    assert!(matches!(
        history::restore_flags(&out, 7),
        Err(StepError::Failed(_))
    ));
}

#[test]
fn list_skips_unreadable_states() {
    let out = tmp("steps-history-skip");
    fs::write(out.join("traj.tum"), "1000 0 0 0 0 0 0 1\n").unwrap();
    fs::write(out.join("slam.json"), "{}\n").unwrap();
    history::record(&out, HistoryKind::Full, 2.0).unwrap();
    // a state without stand.toml (interrupted copy) and one with garbage
    fs::create_dir_all(out.join("verlauf/002")).unwrap();
    fs::create_dir_all(out.join("verlauf/003")).unwrap();
    fs::write(out.join("verlauf/003/stand.toml"), "nr = \"x\"").unwrap();
    let next = history::record(&out, HistoryKind::Reuse, 1.0).unwrap();
    assert_eq!(next, out.join("verlauf").join("004"));
    let nrs: Vec<u32> = history::list(&out).iter().map(|e| e.nr).collect();
    assert_eq!(nrs, [1, 4]);
}
