//! Manual corrections through the library, in the file format of `driftless correct`.

use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag, tmp};
use driftless_slam::Corrections;
use driftless_steps::correct;
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings, CORRECTIONS_FILE};
use driftless_steps::StepError;
use std::fs;

#[test]
fn add_loop_round_trip_with_cli_format() {
    let run = tmp("steps-correct-format");
    let added = "2026-10-21T12:34:56Z";
    assert_eq!(
        correct::add_loop_at(&run, 1.5, 20.25, "Treppe", added).unwrap(),
        1
    );
    assert_eq!(correct::add_loop_at(&run, 3.0, 40.0, "", added).unwrap(), 2);

    let reference = run.join("reference.toml");
    let mut c = Corrections::default();
    c.add_loop(1.5, 20.25, "Treppe", added);
    c.add_loop(3.0, 40.0, "", added);
    c.save(&reference).unwrap();

    let written = run.join(CORRECTIONS_FILE);
    assert_eq!(fs::read(&written).unwrap(), fs::read(&reference).unwrap());
    assert_eq!(
        correct::load(&run).unwrap(),
        Corrections::load(&reference).unwrap()
    );

    // `add_loop` stamps the current time
    correct::add_loop(&run, 5.0, 50.0, "").unwrap();
    let now = &correct::load(&run).unwrap().loops[2].added;
    assert!(now.len() == 20 && now.ends_with('Z') && now.as_bytes()[10] == b'T');
}

#[test]
fn add_loop_rejects_same_submap() {
    // 1.5 s: a single submap, so both times fall into it
    let dir = tmp("steps-correct-same");
    room_bag(&dir, true);
    let out = dir.join("out");
    let err = correct::load(&out).unwrap_err();
    assert!(err.to_string().starts_with("Zielordner"), "{err}");

    let req = SlamRequest {
        scan: dir.clone(),
        out: out.clone(),
        settings: SlamSettings::default(),
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let err = correct::add_loop(&out, 0.2, 1.0, "").unwrap_err();
    assert!(
        matches!(&err, StepError::Failed(m) if m.contains("derselben Teilkarte")),
        "{err}"
    );
    assert!(!out.join(CORRECTIONS_FILE).exists(), "nothing written");

    let err = correct::add_loop(&out, -1.0, 1.0, "").unwrap_err();
    assert!(err.to_string().contains("außerhalb des Scans"), "{err}");
    let err = correct::set_enabled(&out, 1, true).unwrap_err();
    assert!(err.to_string().contains("kein Eintrag 1"), "{err}");
}

/// A run of two laps round the synthetic room (`testutil::Circle`), which closes automatic
/// loops: the run folder, and the start times of the submaps of each accepted automatic loop.
fn run_with_automatic_loops(name: &str) -> (std::path::PathBuf, Vec<(f64, f64)>) {
    use driftless_odom::testutil::{room_scene, walk_bag, Circle};
    let dir = tmp(name);
    let walk = Circle {
        still: 1.0,
        radius: 2.0,
        speed: 1.0,
        laps: 2.0,
    };
    walk_bag(&dir, &room_scene(), &walk, walk.duration(), 360, 1000.0);
    let out = dir.join("out");
    let req = SlamRequest {
        scan: dir.clone(),
        out: out.clone(),
        settings: SlamSettings::default(),
        reuse: false,
        history: true,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let report = read_report(&out);
    let loops = automatic(&report)
        .into_iter()
        .filter(|l| l.accepted)
        .map(|l| (report.submaps[l.a].t_start, report.submaps[l.b].t_start))
        .collect();
    (out, loops)
}

fn read_report(out: &std::path::Path) -> driftless_slam::SlamReport {
    serde_json::from_str(&fs::read_to_string(out.join("slam.json")).unwrap()).unwrap()
}

fn automatic(report: &driftless_slam::SlamReport) -> Vec<driftless_slam::report::LoopReport> {
    report.loops.iter().filter(|l| !l.manual).cloned().collect()
}

/// Recomputes the run from its submaps, as „Jetzt neu rechnen“ and `slam --reuse` do.
fn reuse(out: &std::path::Path) {
    let scan = out.parent().unwrap().to_path_buf();
    let req = SlamRequest {
        scan,
        out: out.to_path_buf(),
        settings: SlamSettings::default(),
        reuse: true,
        history: true,
    };
    run_slam(&req, &Progress::new()).unwrap();
}

/// „verwenden“ off for an automatic loop writes a `[[reject]]` with the start times of its
/// submaps; a reuse (`slam --reuse`) then leaves the loop out („von Hand abgeschaltet“); on
/// again removes the entry and the loop is used again. Times anywhere inside the two submaps
/// name the same loop.
#[test]
fn loop_switch_writes_and_removes_the_reject() {
    let (out, loops) = run_with_automatic_loops("steps-correct-verwerfen");
    assert!(!loops.is_empty(), "no automatic loop");
    let (a, b) = loops[0];
    let added = "2026-10-28T09:00:00Z";
    assert!(correct::set_loop_used_at(&out, a, b, false, added).unwrap());
    // again, and by other times inside the same submaps: nothing new
    assert!(!correct::set_loop_used_at(&out, a + 2.0, b + 3.0, false, added).unwrap());
    let c = correct::load(&out).unwrap();
    assert_eq!(c.rejects.len(), 1);
    assert_eq!((c.rejects[0].a, c.rejects[0].b), (a, b));
    assert_eq!(c.rejects[0].added, added);
    let text = fs::read_to_string(out.join(CORRECTIONS_FILE)).unwrap();
    assert!(text.contains("[[reject]]"), "{text}");
    // `correct list` shows it
    let list = correct::render_list(&c, &out.join(CORRECTIONS_FILE));
    assert!(
        list.contains("abgeschaltet") && list.contains("automatisch"),
        "{list}"
    );

    reuse(&out);
    let report = read_report(&out);
    let vetoed: Vec<_> = automatic(&report)
        .into_iter()
        .filter(|l| (report.submaps[l.a].t_start, report.submaps[l.b].t_start) == (a, b))
        .collect();
    assert_eq!(vetoed.len(), 1, "{:?}", report.loops);
    assert!(!vetoed[0].accepted);
    assert_eq!(vetoed[0].reason.as_deref(), Some(driftless_slam::VETOED));

    // on again, by times inside the submaps: the entry goes, the loop is used
    assert!(correct::set_loop_used(&out, a + 1.0, b + 1.0, true).unwrap());
    assert!(!correct::set_loop_used(&out, a, b, true).unwrap());
    assert!(correct::load(&out).unwrap().rejects.is_empty());
    reuse(&out);
    let report = read_report(&out);
    assert!(automatic(&report).iter().any(|l| (
        report.submaps[l.a].t_start,
        report.submaps[l.b].t_start
    ) == (a, b)
        && l.accepted));

    // times that name no loop of two submaps are refused, nothing written
    let before = fs::read(out.join(CORRECTIONS_FILE)).unwrap();
    let err = correct::set_loop_used(&out, a, a + 0.5, false).unwrap_err();
    assert!(err.to_string().contains("derselben Teilkarte"), "{err}");
    let err = correct::set_loop_used(&out, a, 1e6, false).unwrap_err();
    assert!(err.to_string().contains("außerhalb des Scans"), "{err}");
    assert_eq!(fs::read(out.join(CORRECTIONS_FILE)).unwrap(), before);
}

/// „Diesen Stand wiederherstellen“ of a state before a loop was switched off switches it on
/// again: the rejects are restored as in that state.
#[test]
fn restore_brings_back_the_rejects_of_the_state() {
    let (out, loops) = run_with_automatic_loops("steps-correct-verwerfen-stand");
    let (a, b) = loops[0];
    // state 1: the full run, nothing switched off
    correct::set_loop_used(&out, a, b, false).unwrap();
    reuse(&out);
    // state 2 holds the reject
    let states = driftless_steps::history::list(&out);
    assert_eq!(states.len(), 2);
    let then = Corrections::load(&states[1].dir.join(CORRECTIONS_FILE)).unwrap();
    assert_eq!(then.rejects.len(), 1);
    assert_eq!(driftless_steps::history::restore_flags(&out, 1).unwrap(), 1);
    assert!(correct::load(&out).unwrap().rejects.is_empty());
    assert_eq!(driftless_steps::history::restore_flags(&out, 2).unwrap(), 1);
    assert_eq!(correct::load(&out).unwrap().rejects, then.rejects);
}
