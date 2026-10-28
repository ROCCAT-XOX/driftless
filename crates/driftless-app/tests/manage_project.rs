//! Managing a project in the window (decision 1 of track U): a scan's card has „Umbenennen …“,
//! „Ordner neu zuordnen …“ (for a missing scan folder, only the same scan) and „Aus Projekt
//! entfernen“ (the entry only, never the raw data); a run, map, surface or model has
//! „Löschen …“, which asks with its size, what lies in it and what depends on it, refuses while
//! a step works on it or the view shows it (with „Aus der Ansicht nehmen“), and moves the
//! folder to the trash, never deleting it for good. The trash of these tests is a folder of
//! their own.

use driftless_app::project::{canonical, ModelInfo, Project};
use driftless_app::trash::Trash;
use driftless_app::ui::{text, Action, CloudSummary, DriftlessApp, Queued, Selection, UiMessage};
use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag_at, room_bag_secs, tmp};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

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

/// Moves `path` into the trash of its test as `<n>-<name>`, as the system's trash would.
fn test_trash(path: &Path) -> Result<(), String> {
    static N: AtomicUsize = AtomicUsize::new(0);
    let trash = trash_of(path);
    fs::create_dir_all(&trash).map_err(|e| e.to_string())?;
    let name = path.file_name().unwrap().to_string_lossy();
    let n = N.fetch_add(1, Ordering::SeqCst);
    fs::rename(path, trash.join(format!("{n}-{name}"))).map_err(|e| e.to_string())
}

/// Whether a folder `name` with `file` inside lies in the trash of the test in `dir`.
fn in_trash(dir: &Path, name: &str, file: &str) -> bool {
    let trash = fs::read_dir(trash_of(dir)).into_iter().flatten().flatten();
    trash.into_iter().any(|e| {
        e.file_name()
            .to_string_lossy()
            .ends_with(&format!("-{name}"))
            && e.path().join(file).exists()
    })
}

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {})).with_trash(test_trash)
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

fn harness(app: DriftlessApp) -> H {
    let mut h = Harness::builder()
        .with_size([500.0, 1400.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    settle(&mut h);
    h
}

/// Frames until no step and no background work is left.
fn settle(h: &mut H) {
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

/// Applies `action` and waits until everything it started has ended.
fn run(h: &mut H, action: Action) {
    h.state_mut().apply(action);
    settle(h);
}

/// A few frames: a new window is measured in its first frame and placed in the next.
fn frames(h: &mut H) {
    for _ in 0..4 {
        h.step();
    }
}

/// A scan folder `name` in `dir`, recorded from bag time `start` for `secs` seconds.
fn scan(dir: &Path, name: &str, start: f64, secs: f64) -> PathBuf {
    let scan = dir.join(name);
    fs::create_dir_all(&scan).unwrap();
    room_bag_at(&scan, true, start, secs);
    scan
}

/// Opens the context menu of the entry `label` (a right click) and clicks `entry` in it.
fn menu(h: &mut H, label: &str, entry: &str) {
    h.get_by_label(label).click_secondary();
    frames(h);
    h.get_by_label(entry).click();
    frames(h);
}

/// Whether `entry` of the context menu of `label` can be clicked (the menu is closed again).
fn menu_enabled(h: &mut H, label: &str, entry: &str) -> bool {
    h.get_by_label(label).click_secondary();
    frames(h);
    let enabled = !h.get_by_label(entry).accesskit_node().is_disabled();
    h.key_press(eframe::egui::Key::Escape);
    frames(h);
    enabled
}

fn enabled(h: &H, label: &str) -> bool {
    !h.get_by_label(label).accesskit_node().is_disabled()
}

#[test]
fn rename_and_remove_a_scan() {
    let dir = tmp("verwalten-scan");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let a = scan(&dir, "scan-a", 1000.0, 1.5);
    let b = scan(&dir, "scan-b", 2000.0, 1.5);
    let mut h = harness(app());
    run(&mut h, Action::OpenProject(project.clone()));
    run(&mut h, Action::AddScan(a));
    run(&mut h, Action::AddScan(b.clone()));
    run(&mut h, Action::RunSlam("scan-b".into()));
    run(&mut h, Action::Select(Selection::None));

    // „Umbenennen …“: the name changes, the folder and id stay
    menu(&mut h, "scan-a", text::RENAME_SCAN);
    h.get_by_label(text::SCAN_NAME).focus();
    h.step();
    h.get_by_label(text::SCAN_NAME).type_text(" Ost");
    h.step();
    h.get_by_label(text::RENAME_CONFIRM).click();
    settle(&mut h);
    let on_disk = Project::open(&project).unwrap();
    assert_eq!(on_disk.scans[0].name, "scan-a Ost");
    assert_eq!(on_disk.scans[0].id, "scan-a");
    h.get_by_label("scan-a Ost");
    assert_eq!(h.state().state().ask, None);

    // „Aus Projekt entfernen“: says that the raw data and the results stay; only the entry goes
    menu(&mut h, "scan-b", text::REMOVE_SCAN);
    let results = canonical(&project).unwrap().join("scans/scan-b");
    h.get_by_label_contains("Die Rohdaten bleiben unberührt");
    h.get_by_label(text::REMOVE_CONFIRM).click();
    settle(&mut h);
    let on_disk = Project::open(&project).unwrap();
    assert_eq!(on_disk.scans.len(), 1);
    assert!(b.join("LIDAR_0.bag").is_file(), "the raw data stays");
    assert!(
        results.join("slam-001/slam.json").is_file(),
        "the results stay"
    );
    assert!(h.query_by_label("scan-b").is_none());
    assert_eq!(errors(h.state()), Vec::<String>::new());
    // added again: its results are there again
    run(&mut h, Action::AddScan(b));
    assert_eq!(h.state().state().scans[1].runs.len(), 1);

    // „Abbrechen“ (and Esc) close a question without a change
    menu(&mut h, "scan-a Ost", text::REMOVE_SCAN);
    h.get_by_label(text::ASK_CANCEL).click();
    frames(&mut h);
    assert_eq!(h.state().state().ask, None);
    menu(&mut h, "scan-a Ost", text::RENAME_SCAN);
    h.key_press(eframe::egui::Key::Escape);
    frames(&mut h);
    assert_eq!(h.state().state().ask, None);
    assert_eq!(Project::open(&project).unwrap().scans.len(), 2);
}

#[test]
fn relink_a_missing_scan_only_to_the_same_scan() {
    let dir = tmp("verwalten-zuordnen");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let raw = scan(&dir, "rohdaten", 1000.0, 1.5);
    let mut h = harness(app());
    run(&mut h, Action::OpenProject(project.clone()));
    run(&mut h, Action::AddScan(raw.clone()));
    run(&mut h, Action::RunSlam("rohdaten".into()));
    run(&mut h, Action::Select(Selection::None));
    // only for a missing scan folder
    assert!(!menu_enabled(&mut h, "rohdaten", text::RELINK_SCAN));

    // the raw data moves to another drive: the scan is missing
    let moved = dir.join("anderes-laufwerk").join("rohdaten");
    fs::create_dir_all(moved.parent().unwrap()).unwrap();
    fs::rename(&raw, &moved).unwrap();
    run(&mut h, Action::OpenProject(project.clone()));
    let missing = text::scan_missing("rohdaten");
    h.get_by_label(&missing);
    assert!(menu_enabled(&mut h, &missing, text::RELINK_SCAN));
    let stored = Project::open(&project).unwrap().scans[0].path.clone();

    // a different scan is refused with the reason: other bags, or recorded at another time
    // (here the same bags by name and size, as one scanner writes them)
    let longer = scan(&dir, "laengerer-scan", 1000.0, 2.5);
    let later = scan(&dir, "spaeterer-scan", 5000.0, 1.5);
    for (other, why) in [
        (
            longer,
            "Anderer Scan: die LiDAR-Aufnahmen unterscheiden sich",
        ),
        (later, "Anderer Scan: zu anderer Zeit aufgenommen"),
    ] {
        h.state_mut().state_mut().messages.clear();
        run(
            &mut h,
            Action::RelinkScan {
                id: "rohdaten".into(),
                dir: other,
            },
        );
        let e = errors(h.state());
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].starts_with(why), "{e:?}");
        assert_eq!(Project::open(&project).unwrap().scans[0].path, stored);
    }

    // the same scan in its new place is taken
    h.state_mut().state_mut().messages.clear();
    run(
        &mut h,
        Action::RelinkScan {
            id: "rohdaten".into(),
            dir: moved.clone(),
        },
    );
    assert_eq!(errors(h.state()), Vec::<String>::new());
    assert_eq!(
        Project::open(&project).unwrap().scans[0].path,
        canonical(&moved).unwrap()
    );
    h.get_by_label("rohdaten");
    assert!(h.query_by_label(&missing).is_none());
    assert_eq!(h.state().state().scans[0].runs.len(), 1, "its run stays");
}

/// A scan taken out of the project leaves its results under `scans/<id>`: another scan folder
/// of the same name added later (`Tag2/EG` after `Tag1/EG`) gets an id of its own instead of
/// taking them over as its own runs and maps (M1 of the final review); the scan itself, added
/// again, gets them back under its id.
#[test]
fn a_removed_scans_id_goes_to_no_other_scan() {
    let dir = tmp("verwalten-kennung");
    let project = dir.join("projekt");
    Project::create(&project, "Haus").unwrap();
    let day1 = scan(&dir.join("tag1"), "EG", 1000.0, 1.5);
    let day2 = scan(&dir.join("tag2"), "EG", 5000.0, 1.5);
    let mut h = harness(app());
    run(&mut h, Action::OpenProject(project.clone()));
    run(&mut h, Action::AddScan(day1.clone()));
    run(&mut h, Action::RunSlam("EG".into()));
    run(&mut h, Action::RemoveScan("EG".into()));
    assert!(h.state().state().scans.is_empty());
    let results = canonical(&project).unwrap().join("scans/EG/slam-001");
    assert!(results.join("slam.json").is_file(), "the results stay");

    // another folder named EG, recorded at another time: an id of its own, no runs
    run(&mut h, Action::AddScan(day2));
    assert_eq!(Project::open(&project).unwrap().scans[0].id, "EG-2");
    assert!(h.state().state().scans[0].runs.is_empty());
    // the scan of the results, added again: its id and its run
    run(&mut h, Action::AddScan(day1));
    let on_disk = Project::open(&project).unwrap();
    assert_eq!(on_disk.scans[1].id, "EG");
    assert_eq!(h.state().state().scans[1].runs.len(), 1);
    assert_eq!(h.state().state().scans[1].runs[0].dir, results);
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

/// A scan added before the bags were kept (no `bags` in `project.toml`) is told by the times
/// of its run; without a run it cannot be told, and is taken with a note.
#[test]
fn relink_of_an_old_entry() {
    let dir = tmp("verwalten-zuordnen-alt");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let raw = scan(&dir, "rohdaten", 1000.0, 1.5);
    let mut p = Project::open(&project).unwrap();
    p.add_scan(&raw).unwrap();
    p.save().unwrap();
    // as written before: without the bags
    let toml = fs::read_to_string(project.join("project.toml")).unwrap();
    assert!(toml.contains("bags = ["), "{toml}");
    let old: String = toml
        .lines()
        .filter(|l| !l.starts_with("bags = "))
        .map(|l| format!("{l}\n"))
        .collect();
    fs::write(project.join("project.toml"), old).unwrap();
    let mut p = Project::open(&project).unwrap();
    assert!(p.scans[0].bags.is_empty());
    let other = scan(&dir, "anderer-scan", 1000.0, 2.5);
    // no run: cannot be told
    assert!(!p.relink_scan("rohdaten", &other).unwrap());
    assert_eq!(p.scans[0].bags.len(), 1, "the bags are kept from now on");
}

/// A project with the scan `halle`, its map and a surface of it, computed in the window that
/// moves to the trash with `trash`.
fn project_with_surface(dir: &Path, trash: Trash) -> (H, PathBuf, PathBuf) {
    let raw = dir.join("halle");
    fs::create_dir_all(&raw).unwrap();
    room_bag_secs(&raw, true, 1.5);
    let mut h = harness(app().with_trash(trash));
    run(
        &mut h,
        Action::NewProject {
            dir: dir.join("projekt"),
            name: "Halle".into(),
        },
    );
    run(&mut h, Action::AddScan(raw));
    h.state_mut().state_mut().map_settings.step = driftless_steps::map::MapStep::Clean;
    run(&mut h, Action::ComputeMap("halle".into()));
    let run_dir = canonical(&dir.join("projekt"))
        .unwrap()
        .join("scans/halle/slam-001");
    let map = run_dir.join("karte-001");
    h.state_mut().state_mut().surface_settings.texture = false;
    h.state_mut().state_mut().surface_settings.config.voxel = 0.05;
    run(&mut h, Action::RunSurface(map.clone()));
    assert!(map.join("oberflaeche-001/oberflaeche.glb").is_file());
    assert_eq!(errors(h.state()), Vec::<String>::new());
    (h, run_dir, map)
}

/// The entry of `dir` in the tree as shown: its label with its size.
fn entry(h: &H, dir: &Path, label: &str) -> String {
    let bytes = h.state().state().folder_size(dir).expect("measured");
    text::with_size(label, bytes)
}

#[test]
fn delete_moves_to_the_trash_after_asking() {
    let dir = tmp("verwalten-loeschen");
    let (mut h, run_dir, map) = project_with_surface(&dir, test_trash);
    run(&mut h, Action::Select(Selection::Run(run_dir.clone())));
    let label = entry(&h, &map, "Karte 001");
    let size = h.state().state().folder_size(&map).unwrap();
    let surface = map.join("oberflaeche-001");
    let surface_label = entry(&h, &surface, "Oberfläche 001");

    // the question names the size and what lies in it
    menu(&mut h, &label, text::DELETE);
    h.get_by_label(&text::delete_question("Karte 001", Some(size)));
    h.get_by_label(&text::delete_contains(&[surface_label]));
    // the map's cloud is shown: locked, until it is taken out of the view
    assert!(h.state().state().cloud.is_some());
    h.get_by_label_contains("Wird gerade angezeigt");
    assert!(!enabled(&h, text::DELETE_CONFIRM));
    h.get_by_label(text::UNSHOW).click();
    frames(&mut h);
    assert!(h.state().state().cloud.is_none());
    assert!(h.state().state().surface.is_none());
    assert!(h.query_by_label_contains("Wird gerade angezeigt").is_none());
    assert!(enabled(&h, text::DELETE_CONFIRM));
    h.get_by_label(text::DELETE_CONFIRM).click();
    settle(&mut h);
    assert!(!map.exists());
    assert!(
        in_trash(&dir, "karte-001", "map.json"),
        "in the trash, not deleted"
    );
    assert!(h.query_by_label(&label).is_none());
    assert_eq!(errors(h.state()), Vec::<String>::new());
    let last = h.state().state().messages.last().cloned();
    assert_eq!(
        last,
        Some(UiMessage::Info(text::deleted("Karte 001", Some(size))))
    );

    // the run, with the run drawn: the same
    let run_label = entry(&h, &run_dir, "SLAM 001");
    menu(&mut h, &run_label, text::DELETE);
    h.get_by_label(text::UNSHOW).click();
    frames(&mut h);
    h.get_by_label(text::DELETE_CONFIRM).click();
    settle(&mut h);
    assert!(in_trash(&dir, "slam-001", "slam.json"));
    assert!(h.state().state().scans[0].runs.is_empty());
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

/// A trash that refuses (a network drive on Windows, a full disk): nothing goes, the reason is
/// told.
#[test]
fn a_refused_trash_deletes_nothing() {
    fn refuse(path: &Path) -> Result<(), String> {
        Err(text::trash_unavailable(path, "Netzlaufwerk"))
    }
    let dir = tmp("verwalten-kein-papierkorb");
    let (mut h, _run_dir, map) = project_with_surface(&dir, refuse);
    run(&mut h, Action::Unshow(map.clone()));
    run(&mut h, Action::Delete(map.clone()));
    assert!(map.join("map.json").is_file());
    let e = errors(h.state());
    assert_eq!(e.len(), 1, "{e:?}");
    assert!(
        e[0].starts_with("Kein Papierkorb für diesen Ordner (Netzlaufwerk)"),
        "{e:?}"
    );
}

/// A model of the map `map` of the run `run`, as the tree reads it (only what the questions
/// look at is real).
fn model_of(dir: PathBuf, run: &Path, map: &Path) -> ModelInfo {
    let mut identity = [0.0; 16];
    for k in 0..4 {
        identity[k * 5] = 1.0;
    }
    let member = serde_json::json!({
        "scan": "/rohdaten/halle", "slam_dir": run, "map_dir": map, "slam": {},
        "transform": identity, "placed_against": null, "method": "bezug", "verdict": "sicher",
        "reason": null, "overlap": 1.0, "residual_median_mm": 0.0, "residual_rms_mm": 0.0,
        "agreement": 1.0, "included": true, "color_gain": [1.0, 1.0, 1.0], "points": 1,
    });
    let report = serde_json::json!({
        "version": 2, "input_id": "x", "members": [member], "points": 1,
        "seconds_total": 1.0, "peak_rss_bytes": 0,
    });
    ModelInfo {
        dir,
        label: "Modell 001".into(),
        report: serde_json::from_value(report).unwrap(),
        surfaces: Vec::new(),
    }
}

/// While a step computes in it, a folder cannot be deleted (the menu entry is locked); while
/// the view shows it or a model that uses it, the question says so; the question of a map and
/// of its run names the models that use it.
#[test]
fn delete_is_refused_while_computing_or_shown_and_names_the_users() {
    let dir = tmp("verwalten-abhaengig");
    let (mut h, run_dir, map) = project_with_surface(&dir, test_trash);
    h.state_mut().apply(Action::Unshow(map.clone()));
    h.state_mut()
        .apply(Action::Select(Selection::Scan("halle".into())));
    settle(&mut h);
    let mut state = h.state().state().clone();
    assert_eq!(state.delete_blocked(&map), None, "{:?}", state.correct.run);

    // a step of the run runs: its run and its maps are locked, also in the menu
    state.running = Some(Action::RunMap(run_dir.clone()));
    let why = state.delete_blocked(&map).expect("computing");
    assert!(why.starts_with("Wird gerade gerechnet"), "{why}");
    assert!(state.delete_blocked(&run_dir).is_some());
    let label = entry(&h, &map, "Karte 001");
    h.state_mut().state_mut().running = Some(Action::RunMap(run_dir.clone()));
    assert!(!menu_enabled(&mut h, &label, text::DELETE));
    h.state_mut().state_mut().running = None;
    assert!(menu_enabled(&mut h, &label, text::DELETE));
    // a step of another kind (the raw data only) locks nothing
    state.running = Some(Action::Inspect("halle".into()));
    assert_eq!(state.delete_blocked(&map), None);
    state.running = None;
    // a surface of the map in the queue of „Alle Oberflächen rechnen“
    state.queue.push_back(Queued::Surface(map.clone()));
    assert!(state.delete_blocked(&map).is_some());
    state.queue.clear();

    // a model that uses the map: named in the question of the map and of its run
    let model_dir = canonical(&dir.join("projekt"))
        .unwrap()
        .join("modelle/modell-001");
    state
        .models
        .push(model_of(model_dir.clone(), &run_dir, &map));
    assert_eq!(state.map_users(&map), ["Modell 001"]);
    assert_eq!(state.map_users(&run_dir), ["Modell 001"]);
    let question = state.delete_question(&map).expect("a question");
    assert_eq!(question.used_by, ["Modell 001"]);
    assert!(
        question.contains[0].starts_with("Oberfläche 001"),
        "{question:?}"
    );
    // the model shown in the view: its map cannot be deleted
    state.cloud = Some(CloudSummary {
        path: model_dir.join("modell.las"),
        points: 1,
        has_rgb: false,
        has_scans: true,
        quality_known: false,
        z_range: [0.0, 1.0],
        ceiling: None,
        point_m: 0.01,
    });
    let why = state.delete_blocked(&map).expect("its model is shown");
    assert!(
        why.starts_with("Wird gerade angezeigt (modell.las)"),
        "{why}"
    );
}

/// A run the command line made and was added by hand lies outside the project: it is only
/// taken out of the project, its folder stays.
#[test]
fn a_run_from_outside_is_only_taken_out() {
    let dir = tmp("verwalten-fremder-lauf");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let raw = scan(&dir, "halle", 1000.0, 1.5);
    let mut h = harness(app());
    run(&mut h, Action::OpenProject(project.clone()));
    run(&mut h, Action::AddScan(raw.clone()));
    // a run computed elsewhere, added to the scan
    let outside = dir.join("cli-lauf");
    driftless_steps::slam::run_slam(
        &driftless_steps::slam::SlamRequest {
            scan: raw,
            out: outside.clone(),
            settings: Default::default(),
            reuse: false,
            history: false,
        },
        &Progress::new(),
    )
    .unwrap();
    let mut p = Project::open(&project).unwrap();
    p.add_run("halle", &outside).unwrap();
    p.save().unwrap();
    run(&mut h, Action::OpenProject(project.clone()));
    run(&mut h, Action::Select(Selection::Scan("halle".into())));
    let added = &h.state().state().scans[0].runs[0];
    let (run_dir, shown) = (added.dir.clone(), added.label.clone());
    let label = entry(&h, &run_dir, &shown);
    menu(&mut h, &label, text::REMOVE_RUN);
    settle(&mut h);
    assert!(outside.join("slam.json").is_file(), "the folder stays");
    assert!(Project::open(&project).unwrap().scans[0]
        .extra_runs
        .is_empty());
    assert!(h.state().state().scans[0].runs.is_empty());
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

/// An empty name is refused (the window locks „Umbenennen“ then), as is an unknown scan.
#[test]
fn rename_refuses_an_empty_name() {
    let dir = tmp("verwalten-name");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let raw = scan(&dir, "halle", 1000.0, 1.5);
    let mut p = Project::open(&project).unwrap();
    p.add_scan(&raw).unwrap();
    let e = p.rename_scan("halle", "  ").unwrap_err().to_string();
    assert_eq!(e, text::SCAN_NAME_EMPTY);
    assert!(p.rename_scan("unbekannt", "Halle").is_err());
    assert!(p.remove_scan("unbekannt").is_err());
    p.rename_scan("halle", "  Halle Ost ").unwrap();
    assert_eq!(p.scans[0].name, "Halle Ost");
}

/// The system's trash itself (macOS: through the file manager, without asking the Finder).
/// Run by hand: it puts a small folder into the user's trash.
#[test]
#[ignore = "puts a folder into the real trash"]
fn system_trash_takes_a_folder() {
    let probe = tmp("echter-papierkorb").inside("driftless-papierkorb-probe");
    fs::write(probe.join("probe.txt"), "Driftless").unwrap();
    driftless_app::trash::system_trash(&probe).unwrap();
    assert!(!probe.exists());
}

/// A step that starts between the click and the move to the trash keeps the folder: the
/// check is made again when the trash is about to take it.
#[test]
fn a_step_started_meanwhile_keeps_the_folder() {
    let dir = tmp("verwalten-inzwischen");
    let (mut h, run_dir, map) = project_with_surface(&dir, test_trash);
    run(&mut h, Action::Unshow(map.clone()));
    h.state_mut().apply(Action::Delete(map.clone()));
    h.state_mut().state_mut().running = Some(Action::RunMap(run_dir.clone()));
    h.step();
    h.state_mut().state_mut().running = None;
    settle(&mut h);
    assert!(map.join("map.json").is_file());
    let e = errors(h.state());
    assert_eq!(e.len(), 1, "{e:?}");
    assert!(e[0].starts_with("Wird gerade gerechnet"), "{e:?}");
}
