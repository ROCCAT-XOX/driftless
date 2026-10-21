//! `verlauf/`: one state per SLAM run of the app, and switching corrections back.

use driftless_io::progress::{LogLine, Progress};
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

/// A run of the app recomputed by the CLI (no state): the next reuse of the app keeps the
/// CLI's trajectory as a state, so the reuses after it cannot replace it unseen.
#[test]
fn reuse_after_a_cli_reuse_keeps_the_cli_result() {
    // 12 s: two submaps, a manual loop moves the trajectory
    let dir = tmp("steps-history-cli-between");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    slam(&dir, &out, false, true).unwrap();
    correct::add_loop(&out, 1.0, 11.0, "").unwrap();
    assert_eq!(slam(&dir, &out, true, false), None);
    let cli = fs::read(out.join("traj.tum")).unwrap();
    assert_ne!(
        fs::read(history::list(&out)[0].dir.join("traj.tum")).unwrap(),
        cli
    );

    slam(&dir, &out, true, true).unwrap();
    slam(&dir, &out, true, true).unwrap();
    slam(&dir, &out, true, true).unwrap();
    let list = history::list(&out);
    let kinds: Vec<HistoryKind> = list.iter().map(|e| e.kind).collect();
    assert_eq!(
        kinds,
        [
            HistoryKind::Full,
            HistoryKind::Imported,
            HistoryKind::Reuse,
            HistoryKind::Reuse,
            HistoryKind::Reuse
        ]
    );
    assert_eq!(fs::read(list[1].dir.join("traj.tum")).unwrap(), cli);
    // the CLI run had the loop on
    let then = Corrections::load(&list[1].dir.join(CORRECTIONS_FILE)).unwrap();
    assert!(then.loops[0].enabled);
}

/// The kept state of a run made without history has the switches that run was computed
/// with, not the ones of the corrections file at the reuse.
#[test]
fn imported_state_has_the_switches_of_its_run() {
    // 12 s: two submaps, a manual loop between them is allowed
    let dir = tmp("steps-history-import-flags");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    slam(&dir, &out, false, false);
    correct::add_loop(&out, 1.0, 11.0, "neu").unwrap();
    slam(&dir, &out, true, true);
    let list = history::list(&out);
    assert_eq!(list[0].kind, HistoryKind::Imported);
    let then = Corrections::load(&list[0].dir.join(CORRECTIONS_FILE)).unwrap();
    assert_eq!(then.loops.len(), 1);
    assert!(!then.loops[0].enabled, "the run had no loop");
    // back to that state: the new loop is switched off, not removed
    assert_eq!(history::restore_flags(&out, 1).unwrap(), 1);
    let now = Corrections::load(&out.join(CORRECTIONS_FILE)).unwrap();
    assert_eq!(now.loops.len(), 1);
    assert!(!now.loops[0].enabled);

    // a run made with the loop keeps it on
    let out = dir.join("out2");
    slam(&dir, &out, false, false);
    correct::add_loop(&out, 1.0, 11.0, "").unwrap();
    slam(&dir, &out, true, false);
    slam(&dir, &out, true, true);
    let first = &history::list(&out)[0];
    assert_eq!(first.kind, HistoryKind::Imported);
    let then = Corrections::load(&first.dir.join(CORRECTIONS_FILE)).unwrap();
    assert!(then.loops[0].enabled);
}

/// A disabled entry before the enabled one on the same pair of submaps does not take the
/// pair: the entry the run was computed with is the one recorded as on.
#[test]
fn imported_state_prefers_the_enabled_entry_of_a_pair() {
    let dir = tmp("steps-history-import-claim");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    slam(&dir, &out, false, false);
    correct::add_loop(&out, 1.0, 11.0, "alt").unwrap();
    correct::set_enabled(&out, 1, false).unwrap();
    // the same two submaps, other times: allowed while entry 1 is off
    correct::add_loop(&out, 2.0, 11.5, "neu").unwrap();
    slam(&dir, &out, true, false);
    slam(&dir, &out, true, true);
    let first = &history::list(&out)[0];
    assert_eq!(first.kind, HistoryKind::Imported);
    let then = Corrections::load(&first.dir.join(CORRECTIONS_FILE)).unwrap();
    let flags: Vec<bool> = then.loops.iter().map(|l| l.enabled).collect();
    assert_eq!(flags, [false, true]);
}

/// An entry added after the run cannot have made its loop, also when the entry that did
/// was switched off since.
#[test]
fn imported_state_skips_entries_added_after_the_run() {
    let dir = tmp("steps-history-import-later");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    slam(&dir, &out, false, false);
    correct::add_loop(&out, 1.0, 11.0, "alt").unwrap();
    slam(&dir, &out, true, false);
    correct::set_enabled(&out, 1, false).unwrap();
    // the same pair, added after that run (a stamp in the future to be independent of the clock)
    correct::add_loop_at(&out, 2.0, 11.5, "neu", "2999-01-01T00:00:00Z").unwrap();
    slam(&dir, &out, true, true);
    let first = &history::list(&out)[0];
    assert_eq!(first.kind, HistoryKind::Imported);
    let then = Corrections::load(&first.dir.join(CORRECTIONS_FILE)).unwrap();
    let flags: Vec<bool> = then.loops.iter().map(|l| l.enabled).collect();
    assert_eq!(flags, [true, false]);
}

/// `added` with an offset is compared as a time: an hour before the run, written in +09:00
/// (so that as text it sorts after the run's time), the entry still claims its pair.
#[test]
fn imported_state_compares_added_as_time() {
    use std::time::{Duration, SystemTime};
    let dir = tmp("steps-history-import-offset");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    slam(&dir, &out, false, false);
    let hour_ago = SystemTime::now() - Duration::from_secs(3600);
    // the same moment in +09:00
    let local = correct::utc(hour_ago + Duration::from_secs(9 * 3600));
    let added = local.replace('Z', "+09:00");
    assert!(
        added > correct::utc_now(),
        "{added} sorts after now as text"
    );
    correct::add_loop_at(&out, 1.0, 11.0, "", &added).unwrap();
    slam(&dir, &out, true, false);
    slam(&dir, &out, true, true);
    let first = &history::list(&out)[0];
    assert_eq!(first.kind, HistoryKind::Imported);
    let then = Corrections::load(&first.dir.join(CORRECTIONS_FILE)).unwrap();
    assert!(then.loops[0].enabled, "made the run's loop");
}

/// A state keeps the corrections the run was computed with: a change written while the
/// run is going (by another program) is not recorded as if it had been applied.
#[test]
fn state_has_the_corrections_read_at_the_start() {
    let dir = tmp("steps-history-edit-during");
    room_bag_secs(&dir, true, 12.0);
    let out = dir.join("out");
    slam(&dir, &out, false, true);
    correct::add_loop(&out, 1.0, 11.0, "vorher").unwrap();
    let before = fs::read(out.join(CORRECTIONS_FILE)).unwrap();
    let target = out.clone();
    // after the pose graph (a reuse logs the cache line), switch the entry off
    let progress = Progress::with_log(move |line| {
        if matches!(line, LogLine::Info(t) if t.starts_with("Zwischenspeicher")) {
            correct::set_enabled(&target, 1, false).unwrap();
        }
    });
    let req = SlamRequest {
        scan: dir.clone(),
        out: out.clone(),
        settings: SlamSettings::default(),
        reuse: true,
        history: true,
    };
    let state = run_slam(&req, &progress).unwrap().history.unwrap();
    assert!(
        !correct::load(&out).unwrap().loops[0].enabled,
        "edited meanwhile"
    );
    assert_eq!(fs::read(state.join(CORRECTIONS_FILE)).unwrap(), before);
}

#[test]
fn record_without_slam_json() {
    let out = tmp("steps-history-no-json");
    fs::write(out.join("traj.tum"), "1000 0 0 0 0 0 0 1\n").unwrap();
    let dir = history::record(&out, HistoryKind::Full, 1.0).unwrap();
    assert!(dir.join("traj.tum").is_file());
    assert!(!dir.join("slam.json").exists());
    assert_eq!(history::list(&out).len(), 1);
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
