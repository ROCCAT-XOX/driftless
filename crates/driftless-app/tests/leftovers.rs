//! After an interruption (decision 4 of track U): what a job cut off by a crash, a force quit or
//! a power cut left behind – a run, map, surface or model folder without its final report, a
//! stale `<name>.tmp` – is listed in „Daten“ as „unvollständig“ (no longer as a run or a map that
//! waits), and „Aufräumen“ moves it to the trash. What a job may still write (its folder marked
//! busy by a running job, or written to a moment ago) is left alone. The tree is read again when
//! the window gets the focus back (debounced): a scan folder that came back is no longer missing.

use driftless_app::leftovers::{self, Leftover, LeftoverKind, BUSY_FILE, BUSY_STALE};
use driftless_app::project::{canonical, new_surface_dir, run_slam_job, Project, SlamJob, StepDir};
use driftless_app::ui::{text, Action, DriftlessApp, Selection, UiMessage, FOCUS_GAP};
use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::slam::SlamSettings;
use eframe::egui;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

type H = Harness<'static, DriftlessApp>;

/// The trash of a test: `papierkorb` in its temporary folder (`driftless-cli-…`).
fn trash_of(path: &Path) -> PathBuf {
    let root = path
        .ancestors()
        .find(|a| {
            a.file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with("driftless-cli-"))
        })
        .expect("in a test folder");
    root.join("papierkorb")
}

fn test_trash(path: &Path) -> Result<(), String> {
    static N: AtomicUsize = AtomicUsize::new(0);
    let trash = trash_of(path);
    fs::create_dir_all(&trash).map_err(|e| e.to_string())?;
    let name = path.file_name().unwrap().to_string_lossy();
    let n = N.fetch_add(1, Ordering::SeqCst);
    fs::rename(path, trash.join(format!("{n}-{name}"))).map_err(|e| e.to_string())
}

/// The names of what lies in the trash of the test in `dir`.
fn trashed(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = fs::read_dir(trash_of(dir))
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            name.split_once('-')
                .map_or(name.clone(), |(_, n)| n.to_string())
        })
        .collect();
    names.sort();
    names
}

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e.clone()),
            _ => None,
        })
        .collect()
}

/// Sets the time `file` was last written.
fn written_at(file: &Path, t: SystemTime) {
    fs::File::options()
        .write(true)
        .open(file)
        .unwrap()
        .set_modified(t)
        .unwrap();
}

/// A file `name` in `dir` (made with its folders), with `bytes` bytes.
fn file(dir: &Path, name: &str, bytes: usize) -> PathBuf {
    fs::create_dir_all(dir).unwrap();
    let path = dir.join(name);
    fs::write(&path, vec![7u8; bytes]).unwrap();
    path
}

/// A project with the scan `halle` and a run `slam-001` with a map `karte-001` and a surface
/// `oberflaeche-001` as complete folders would be (their reports – the surface's after its
/// glb –, here files of a few bytes).
fn project(dir: &Path) -> (PathBuf, PathBuf) {
    let raw = dir.join("halle");
    fs::create_dir_all(&raw).unwrap();
    room_bag_secs(&raw, true, 1.0);
    let project = dir.join("projekt");
    let mut p = Project::create(&project, "Halle").unwrap();
    p.add_scan(&raw).unwrap();
    p.save().unwrap();
    let project = canonical(&project).unwrap();
    let run = project.join("scans/halle/slam-001");
    file(&run, "slam.json", 10);
    file(&run, "traj.tum", 10);
    file(&run.join("karte-001"), "map.json", 10);
    let surface = run.join("karte-001/oberflaeche-001");
    file(&surface, "oberflaeche.glb", 10);
    file(&surface, "oberflaeche.json", 10);
    (project, run)
}

/// What a crash leaves in `project` (run `run`): a run with only `lauf.toml`, a map with only
/// a stage being written, a stale stage of a complete map, a surface folder with nothing in
/// it, one with its glb but without its report (cut off between the two), a model without its
/// report.
fn crash(project: &Path, run: &Path) {
    file(&project.join("scans/halle/slam-002"), "lauf.toml", 100);
    file(&run.join("karte-002"), "dense.bin.tmp", 2000);
    file(&run.join("karte-001"), "clean.bin.tmp", 1000);
    fs::create_dir_all(run.join("karte-001/oberflaeche-002")).unwrap();
    file(
        &run.join("karte-001/oberflaeche-003"),
        "oberflaeche.glb",
        300,
    );
    file(&project.join("modelle/modell-001"), "modell.las.tmp", 500);
}

fn labels(found: &[Leftover]) -> Vec<(LeftoverKind, String, u64)> {
    let mut out: Vec<_> = found
        .iter()
        .map(|l| (l.kind, l.label.clone(), l.bytes))
        .collect();
    out.sort_by(|a, b| a.1.cmp(&b.1));
    out
}

#[test]
fn finds_what_a_cut_off_job_left() {
    let dir = tmp("reste-finden");
    let (project, run) = project(&dir);
    let ids = ["halle".to_string()];
    assert_eq!(leftovers::find(&project, &ids), Vec::new());
    crash(&project, &run);
    let found = leftovers::find(&project, &ids);
    assert_eq!(
        labels(&found),
        [
            (LeftoverKind::Map, "Karte 002 (SLAM 001)".into(), 2000),
            (LeftoverKind::Model, "Modell 001".into(), 500),
            (
                LeftoverKind::Surface,
                "Oberfläche 002 (Karte 001)".into(),
                0
            ),
            (
                LeftoverKind::Surface,
                "Oberfläche 003 (Karte 001)".into(),
                300
            ),
            (LeftoverKind::Run, "SLAM 002".into(), 100),
            (LeftoverKind::Temp, "clean.bin.tmp (Karte 001)".into(), 1000),
        ]
    );
    // a run of another scan id, or folders of other names, are not the project's
    file(&project.join("scans/anderer/slam-001"), "lauf.toml", 1);
    file(&project.join("scans/halle/notizen"), "a.txt", 1);
    assert_eq!(leftovers::find(&project, &ids).len(), 6);
}

/// What a recompute cut off in a run's or a map's own folder leaves (by a crash, or by an
/// older program): the run without its report, but with its history and its finished map; a
/// second map without its report, but with a finished surface – and in it a surface cut off; a
/// third map with nothing finished in it.
fn cut_off_in_place(run: &Path) {
    fs::remove_file(run.join("slam.json")).unwrap();
    fs::create_dir_all(run.join("verlauf/001")).unwrap();
    let k2 = run.join("karte-002");
    file(&k2, "clean.bin", 50);
    file(&k2.join("oberflaeche-001"), "oberflaeche.glb", 10);
    file(&k2.join("oberflaeche-001"), "oberflaeche.json", 10);
    fs::create_dir_all(k2.join("oberflaeche-002")).unwrap();
    file(&run.join("karte-003"), "dense.bin.tmp", 20);
}

/// A run or map folder without its report that still holds finished results – maps or
/// surfaces with their reports, the history of a run – is no leftover: only what in it is
/// incomplete is offered (I1 of the final review).
#[test]
fn finished_results_are_never_leftovers() {
    let dir = tmp("reste-fertiges");
    let (project, run) = project(&dir);
    cut_off_in_place(&run);
    let ids = ["halle".to_string()];
    assert_eq!(
        labels(&leftovers::find(&project, &ids)),
        [
            (LeftoverKind::Map, "Karte 003 (SLAM 001)".into(), 20),
            (
                LeftoverKind::Surface,
                "Oberfläche 002 (Karte 002)".into(),
                0
            ),
        ]
    );
    // each kind of finished result keeps the run on its own
    for (keep, path) in [
        ("history", "verlauf"),
        ("map", "karte-001/map.json"),
        ("surface", "karte-002/oberflaeche-001/oberflaeche.json"),
    ] {
        let only = tmp(&format!("reste-fertiges-{keep}"));
        let (project, run) = project_with(&only, &run, path);
        assert!(
            leftovers::holds_results(&run),
            "{keep}: {:?}",
            leftovers::find(&project, &ids)
        );
        assert!(
            !leftovers::find(&project, &ids)
                .iter()
                .any(|l| l.kind == LeftoverKind::Run),
            "{keep}"
        );
    }
    // the tree keeps them, marked incomplete; the map with nothing finished is not in it
    let p = Project::open(&project).unwrap();
    let tree = driftless_app::ui::read_tree(&p);
    let node = &tree.scans[0].runs[0];
    assert!(node.incomplete && !node.has_result);
    let maps: Vec<(&Path, bool)> = node
        .maps
        .iter()
        .map(|m| (m.dir.as_path(), m.incomplete))
        .collect();
    assert_eq!(
        maps,
        [
            (run.join("karte-001").as_path(), false),
            (run.join("karte-002").as_path(), true)
        ]
    );
    assert_eq!(node.maps[1].surfaces.len(), 1);
}

/// A copy of the run `run` of [`project`] in a new project in `dir`, of which only `keep` (a
/// path inside the run) is finished: the report of the run is gone, and so is every other
/// finished result.
fn project_with(dir: &Path, run: &Path, keep: &str) -> (PathBuf, PathBuf) {
    let (project, copy) = project(dir);
    fs::remove_file(copy.join("slam.json")).unwrap();
    fs::remove_file(copy.join("karte-001/map.json")).unwrap();
    fs::remove_file(copy.join("karte-001/oberflaeche-001/oberflaeche.json")).unwrap();
    let source = run.join(keep);
    let target = copy.join(keep);
    if source.is_dir() {
        fs::create_dir_all(&target).unwrap();
    } else {
        fs::create_dir_all(target.parent().unwrap()).unwrap();
        fs::copy(&source, &target).unwrap();
    }
    (project, copy)
}

/// A folder a running job marks as busy ([`BUSY_FILE`], touched while it runs) is left alone
/// with what lies in it; a mark that is no longer touched (the job was cut off) is not.
#[test]
fn a_busy_folder_is_left_alone() {
    let dir = tmp("reste-in-arbeit");
    let (project, run) = project(&dir);
    let ids = ["halle".to_string()];
    let busy = run.join("karte-002");
    file(&busy, "dense.bin.tmp", 10);
    let mark = file(&busy, BUSY_FILE, 10);
    assert_eq!(leftovers::find(&project, &ids), Vec::new());
    written_at(&mark, SystemTime::now() - BUSY_STALE * 2);
    assert_eq!(leftovers::find(&project, &ids).len(), 1);
}

/// A job's new folder is marked busy while the job runs, and the mark is touched; kept, the mark
/// goes; dropped (cancelled or failed), the folder goes.
#[test]
fn a_step_folder_is_marked_while_the_job_runs() {
    let dir = tmp("reste-markiert");
    let (project, run) = project(&dir);
    let step = new_surface_dir(&run.join("karte-001")).unwrap();
    let mark = step.path().join(BUSY_FILE);
    assert!(mark.is_file());
    assert_eq!(leftovers::find(&project, &["halle".into()]), Vec::new());
    let kept = step.keep();
    assert!(!kept.join(BUSY_FILE).exists());
    let step: StepDir = new_surface_dir(&run.join("karte-001")).unwrap();
    let path = step.path().to_path_buf();
    drop(step);
    assert!(!path.exists());
}

/// When each leftover was last written, newest first among a folder and what lies directly in
/// it: what was written within the quiet time is not offered yet (a job of another program may
/// still write it).
#[test]
fn what_was_just_written_is_not_offered_yet() {
    let dir = tmp("reste-ruhe");
    let (project, run) = project(&dir);
    crash(&project, &run);
    let found = leftovers::find(&project, &["halle".into()]);
    let now = SystemTime::now();
    let shown = |quiet: Duration, at: SystemTime| {
        found
            .iter()
            .filter(|l| leftovers::settled(l, quiet, at))
            .count()
    };
    assert_eq!(shown(Duration::from_secs(120), now), 0);
    assert_eq!(
        shown(Duration::from_secs(120), now + Duration::from_secs(180)),
        6
    );
    assert_eq!(shown(Duration::ZERO, now + Duration::from_secs(1)), 6);
}

type Window = H;

fn harness(app: DriftlessApp) -> Window {
    let mut h = Harness::builder()
        .with_size([500.0, 1400.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    settle(&mut h);
    h
}

/// Frames until no step and no background work is left.
fn settle(h: &mut Window) {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            h.step();
            return;
        }
        assert!(Instant::now() < deadline, "background work did not end");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn run_action(h: &mut Window, action: Action) {
    h.state_mut().apply(action);
    settle(h);
}

/// A window on the project in `project`, offering leftovers at once (no quiet time).
fn window(project: &Path) -> Window {
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_trash(test_trash);
    app.state_mut().leftover_quiet = Some(Duration::ZERO);
    let mut h = harness(app);
    run_action(&mut h, Action::OpenProject(project.to_path_buf()));
    assert!(
        h.state().state().project.is_some(),
        "{:?}",
        errors(h.state())
    );
    h
}

#[test]
fn leftovers_are_listed_and_cleaned_up() {
    let dir = tmp("reste-aufraeumen");
    let (project, run) = project(&dir);
    crash(&project, &run);
    let mut h = window(&project);
    run_action(&mut h, Action::Select(Selection::Scan("halle".into())));
    // not in the tree as a run or a map
    let state = h.state().state();
    assert_eq!(state.scans[0].runs.len(), 1, "{:?}", state.scans[0].runs);
    // only the complete map: „Bilder“ does not count the leftover as waiting for „Daten“
    let maps: Vec<&Path> = state.scans[0].runs[0]
        .maps
        .iter()
        .map(|m| m.dir.as_path())
        .collect();
    assert_eq!(maps, [run.join("karte-001")]);
    // a surface cut off between its glb and its report is no surface of the map: not listed,
    // so neither shown nor loaded as finished
    let surfaces: Vec<&Path> = state.scans[0].runs[0].maps[0]
        .surfaces
        .iter()
        .map(|s| s.dir.as_path())
        .collect();
    assert_eq!(surfaces, [run.join("karte-001/oberflaeche-001")]);
    // listed as „unvollständig“, with their sizes
    h.get_by_label_contains(text::INCOMPLETE_TITLE);
    h.get_by_label(&text::incomplete_entry("SLAM 002", 100));
    h.get_by_label(&text::incomplete_entry("Karte 002 (SLAM 001)", 2000));
    h.get_by_label(text::CLEAN_UP).click();
    settle(&mut h);
    assert_eq!(
        trashed(&dir),
        [
            "clean.bin.tmp",
            "karte-002",
            "modell-001",
            "oberflaeche-002",
            "oberflaeche-003",
            "slam-002"
        ]
    );
    assert!(run.join("karte-001/map.json").is_file(), "the rest stays");
    assert!(h.query_by_label_contains(text::INCOMPLETE_TITLE).is_none());
    assert!(h.state().state().leftovers.is_empty());
    assert_eq!(errors(h.state()), Vec::<String>::new());
    let last = h.state().state().messages.last().cloned();
    assert!(
        matches!(&last, Some(UiMessage::Info(t)) if t.contains("Papierkorb")),
        "{last:?}"
    );
}

/// A run and a map kept for their finished results stay in the tree as „unvollständig – neu
/// rechnen“, why and what to do on hover; „Aufräumen“ moves only what in them is incomplete.
#[test]
fn incomplete_folders_stay_in_the_tree() {
    let dir = tmp("reste-unvollstaendig");
    let (project, run) = project(&dir);
    cut_off_in_place(&run);
    let mut h = window(&project);
    h.ctx.all_styles_mut(|s| s.interaction.tooltip_delay = 0.0);
    run_action(&mut h, Action::Select(Selection::Scan("halle".into())));
    // „SLAM 001 · 1,2 KB · unvollständig – neu rechnen“ (the size once measured)
    let marked = |name: &'static str| {
        move |n: &egui_kittest::kittest::AccessKitNode<'_>| {
            n.label()
                .is_some_and(|l| l.starts_with(name) && l.ends_with(&text::incomplete_folder("")))
        }
    };
    let count = h
        .query_all_by_label_contains(text::INCOMPLETE_RECOMPUTE)
        .count();
    assert_eq!(count, 2, "the run and the second map");
    h.get_by(marked("SLAM 001")).hover();
    h.run();
    h.get_by_label_contains(text::RUN_INCOMPLETE_HINT);
    h.get_by(marked("Karte 002")).hover();
    h.run();
    h.get_by_label_contains(text::MAP_INCOMPLETE_HINT);
    // the complete map is not marked
    assert!(h.query_by(marked("Karte 001")).is_none());

    h.get_by_label(text::CLEAN_UP).click();
    settle(&mut h);
    assert_eq!(trashed(&dir), ["karte-003", "oberflaeche-002"]);
    for kept in [
        "verlauf/001",
        "karte-001/map.json",
        "karte-002/oberflaeche-001/oberflaeche.json",
    ] {
        assert!(run.join(kept).exists(), "{kept}");
    }
    assert_eq!(
        h.query_all_by_label_contains(text::INCOMPLETE_RECOMPUTE)
            .count(),
        2
    );
}

/// „Aufräumen“ looks again when it moves a folder: one that holds finished results by then,
/// that has its report by now, or that a job (another window) marked busy since the list was
/// made, stays.
#[test]
fn clean_up_looks_again_before_it_moves() {
    let dir = tmp("reste-nochmal");
    let (project, run) = project(&dir);
    crash(&project, &run);
    let mut h = window(&project);
    // meanwhile a surface of the cut-off map is finished, the surface cut off after its glb
    // gets its report (a step of the command line finished it), and another window starts a
    // job in the cut-off run
    let k2 = run.join("karte-002");
    file(&k2.join("oberflaeche-001"), "oberflaeche.json", 10);
    file(
        &run.join("karte-001/oberflaeche-003"),
        "oberflaeche.json",
        10,
    );
    file(&project.join("scans/halle/slam-002"), BUSY_FILE, 10);
    h.get_by_label(text::CLEAN_UP).click();
    settle(&mut h);
    let moved = trashed(&dir);
    for kept in ["karte-002", "oberflaeche-003", "slam-002"] {
        assert!(!moved.contains(&kept.to_string()), "{kept}: {moved:?}");
    }
    assert!(moved.contains(&"modell-001".to_string()), "{moved:?}");
    assert!(moved.contains(&"oberflaeche-002".to_string()), "{moved:?}");
    assert!(k2.join("oberflaeche-001/oberflaeche.json").is_file());
    assert!(run
        .join("karte-001/oberflaeche-003/oberflaeche.glb")
        .is_file());
}

/// What a reuse of `run` cut off after six of its renames leaves (files only, no real kill):
/// the trajectory before the last reuse, the report, the odometry and the map aside as
/// `<name>.alt.tmp`, the old trajectory moved to `traj.vorher.tum`, the new one in, the other
/// new files still staged as `<name>.tmp`.
fn cut_off_reuse(run: &Path) {
    for name in ["traj.vorher.tum", "slam.json", "traj_odom.tum", "map.las"] {
        fs::rename(run.join(name), run.join(format!("{name}.alt.tmp"))).unwrap();
    }
    fs::rename(run.join("traj.tum"), run.join("traj.vorher.tum")).unwrap();
    file(run, "traj.tum", 10);
    for name in ["traj_odom.tum", "map.las", "slam.json"] {
        file(run, &format!("{name}.tmp"), 10);
    }
}

/// A reuse cut off while it put its new files in place leaves the run without its report, the
/// old files aside and the new ones half in place. None of its `.tmp` files is a leftover – not
/// even without the history of the run is the folder one – and „Aufräumen“ never moves them,
/// also one listed before the reuse was cut off; the run stays in the tree as „unvollständig –
/// neu rechnen“. The next run of the step puts the old files back and computes as usual (the
/// re-review of the final fix round).
#[test]
fn a_reuse_cut_off_is_left_for_the_next_run() {
    let dir = tmp("reste-tausch");
    let raw = dir.join("halle");
    fs::create_dir_all(&raw).unwrap();
    room_bag_secs(&raw, true, 1.5);
    let mut p = Project::create(&dir.join("projekt"), "Halle").unwrap();
    p.add_scan(&raw).unwrap();
    p.save().unwrap();
    let full = SlamJob::Full(SlamSettings::default());
    run_slam_job(&p, "halle", &full, &Progress::new()).unwrap();
    let run = p.runs("halle").remove(0);
    // reused once: its trajectory before the last reuse is in place
    let reuse = SlamJob::Reuse(run.clone());
    run_slam_job(&p, "halle", &reuse, &Progress::new()).unwrap();
    let traj = fs::read(run.dir.join("traj.tum")).unwrap();
    // a stale file of an earlier crash, offered for „Aufräumen“
    file(&run.dir, "map.las.tmp", 10);
    let project = canonical(&p.dir).unwrap();
    let mut h = window(&project);
    h.get_by_label(&text::incomplete_entry("map.las.tmp (SLAM 001)", 10));

    cut_off_reuse(&run.dir);
    let ids = ["halle".to_string()];
    assert_eq!(leftovers::find(&project, &ids), Vec::new());
    for name in ["slam.json.alt.tmp", "map.las.tmp"] {
        assert!(!leftovers::may_clean_up(&run.dir.join(name)), "{name}");
    }
    let history = run.dir.join("verlauf");
    fs::rename(&history, dir.join("verlauf")).unwrap();
    assert!(leftovers::holds_results(&run.dir));
    assert_eq!(leftovers::find(&project, &ids), Vec::new());
    fs::rename(dir.join("verlauf"), &history).unwrap();

    h.get_by_label(text::CLEAN_UP).click();
    settle(&mut h);
    assert_eq!(trashed(&dir), Vec::<String>::new());
    assert_eq!(errors(h.state()), Vec::<String>::new());
    for name in [
        "slam.json.alt.tmp",
        "traj.vorher.tum.alt.tmp",
        "map.las.tmp",
    ] {
        assert!(run.dir.join(name).is_file(), "{name}");
    }
    let state = h.state().state();
    assert!(state.leftovers.is_empty(), "{:?}", state.leftovers);
    let node = &state.scans[0].runs[0];
    assert!(node.incomplete, "in the tree, marked");

    let run = p.runs("halle").remove(0);
    run_slam_job(&p, "halle", &SlamJob::Reuse(run.clone()), &Progress::new()).unwrap();
    assert_eq!(fs::read(run.dir.join("traj.vorher.tum")).unwrap(), traj);
    assert_eq!(leftovers::find(&project, &ids), Vec::new());
    assert!(p.runs("halle")[0].has_result);
}

/// While a step runs in the window, „Aufräumen“ waits (the reason on hover).
#[test]
fn clean_up_waits_for_the_step() {
    let dir = tmp("reste-schritt");
    let (project, run) = project(&dir);
    crash(&project, &run);
    let mut h = window(&project);
    h.state_mut().state_mut().running = Some(Action::Inspect("halle".into()));
    h.step();
    h.step();
    assert!(h
        .get_by_label(text::CLEAN_UP)
        .accesskit_node()
        .is_disabled());
    h.state_mut().state_mut().running = None;
    h.step();
    h.step();
    assert!(!h
        .get_by_label(text::CLEAN_UP)
        .accesskit_node()
        .is_disabled());
}

/// The focus back in the window reads the tree again (a scan folder that came back is found),
/// at most once per [`FOCUS_GAP`].
#[test]
fn focus_reads_the_tree_again() {
    let dir = tmp("reste-fokus");
    let (project, _run) = project(&dir);
    let raw = dir.join("halle");
    let away = dir.join("halle-weg");
    fs::rename(&raw, &away).unwrap();
    let mut h = window(&project);
    let missing = |h: &Window| {
        matches!(
            h.state().state().scans[0].state,
            driftless_app::project::ScanState::Missing(_)
        )
    };
    assert!(missing(&h));
    // the drive is attached again; the window gets the focus back
    fs::rename(&away, &raw).unwrap();
    // the window loses the focus and gets it back; the read follows a moment later (waited
    // for by `settle`)
    let focus = |h: &mut Window| {
        h.event(egui::Event::WindowFocused(false));
        h.step();
        h.event(egui::Event::WindowFocused(true));
        settle(h);
    };
    focus(&mut h);
    assert!(!missing(&h));
    // within the gap: not read again
    fs::rename(&raw, &away).unwrap();
    focus(&mut h);
    assert!(!missing(&h));
    std::thread::sleep(FOCUS_GAP);
    focus(&mut h);
    assert!(missing(&h));
}

/// What a touch-up cut off left in a surface folder (`nacharbeit.tmp-…`, R2-M1 of the final
/// review) is offered for „Aufräumen“ – unless it was cut off while it put its new cache in
/// place: the next touch-up puts the old one back, so its folders are left alone until then.
#[test]
fn touchup_leftovers_are_offered() {
    let dir = tmp("reste-nacharbeit");
    let (project, run) = project(&dir);
    let ids = ["halle".to_string()];
    let surface = run.join("karte-001/oberflaeche-001");
    let new = surface.join("nacharbeit.tmp-4242-neu");
    file(&new, "aenderungen.bin", 30);
    assert_eq!(
        labels(&leftovers::find(&project, &ids)),
        [(
            LeftoverKind::Temp,
            "nacharbeit.tmp-4242-neu (Oberfläche 001)".into(),
            30
        )]
    );
    assert!(leftovers::may_clean_up(&new));

    // cut off between its two renames: an intact old cache aside, none in place
    let aside = surface.join("nacharbeit.tmp-4242-alt");
    file(&aside, "aenderungen.bin", 4);
    fs::write(
        aside.join("nacharbeit.json"),
        r#"{"input_id":"x","glb_bytes":10,"edit_version":1,"patches_bytes":4,"steps":[]}"#,
    )
    .unwrap();
    assert_eq!(leftovers::find(&project, &ids), Vec::new());
    assert!(!leftovers::may_clean_up(&aside));
    assert!(!leftovers::may_clean_up(&new));

    // with a cache in place both are leftovers; the cache itself never is
    fs::create_dir_all(surface.join("nacharbeit")).unwrap();
    let found = leftovers::find(&project, &ids);
    let paths: Vec<&Path> = found.iter().map(|l| l.path.as_path()).collect();
    assert_eq!(paths, [aside.as_path(), new.as_path()]);
    assert!(leftovers::may_clean_up(&aside));
}
