//! Two writers on one project (decision 7 of track U): two windows of the program, or the
//! window and the command line. `project.toml` is changed under a lock file and on what is on
//! disk at that moment, so a change of the other window is kept; a lock another writer holds
//! refuses the change with a German message. The same lock guards `corrections.toml` and
//! `punktpaare.toml`. The precision and the fineness chosen are kept per project
//! (`auswahl.toml`).

use driftless_app::project::{canonical, Project};
use driftless_app::ui::{Action, DriftlessApp, UiMessage};
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::correct;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

/// Ticks until no step and no background work is left.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        app.tick();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "background work did not end");
        std::thread::sleep(Duration::from_millis(2));
    }
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

/// A scan folder `name` in `dir`.
fn scan(dir: &Path, name: &str) -> PathBuf {
    let scan = dir.join(name);
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 1.0);
    scan
}

/// A window with the project in `project` open.
fn window(project: &Path) -> DriftlessApp {
    let mut app = app();
    app.apply(Action::OpenProject(project.to_path_buf()));
    settle(&mut app);
    assert!(app.state().project.is_some(), "{:?}", errors(&app));
    app
}

/// Window A adds „Scan A“, window B, opened before that, adds „Scan B“: both stay (before, B
/// wrote its old copy and A's scan was gone without a word).
#[test]
fn two_windows_keep_each_others_changes() {
    let dir = tmp("zwei-fenster");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let (scan_a, scan_b) = (scan(&dir, "scan-a"), scan(&dir, "scan-b"));
    let mut a = window(&project);
    let mut b = window(&project);
    a.apply(Action::AddScan(scan_a));
    settle(&mut a);
    b.apply(Action::AddScan(scan_b));
    settle(&mut b);
    assert_eq!(errors(&a), Vec::<String>::new());
    assert_eq!(errors(&b), Vec::<String>::new());
    let ids = |p: &Project| p.scans.iter().map(|s| s.id.clone()).collect::<Vec<_>>();
    let on_disk = Project::open(&project).unwrap();
    assert_eq!(ids(&on_disk), ["scan-a", "scan-b"]);
    // B shows both, and says that the project had changed meanwhile
    assert_eq!(
        ids(b.state().project.as_ref().unwrap()),
        ["scan-a", "scan-b"]
    );
    assert_eq!(b.state().scans.len(), 2);
    let infos: Vec<&UiMessage> = b
        .state()
        .messages
        .iter()
        .filter(|m| matches!(m, UiMessage::Info(t) if t.contains("anderen Fenster")))
        .collect();
    assert_eq!(infos.len(), 1, "{:?}", b.state().messages);
    // A sees B's scan when it reads the project again
    a.apply(Action::OpenProject(project.clone()));
    settle(&mut a);
    assert_eq!(a.state().scans.len(), 2);
}

/// While another writer holds the lock of `project.toml`, a change is refused with a German
/// message that names who holds it; nothing is written. A lock left behind by a program that
/// ended is taken over.
#[test]
fn a_held_lock_refuses_the_change() {
    let dir = tmp("gesperrt");
    let project = dir.join("projekt");
    Project::create(&project, "Halle").unwrap();
    let raw = scan(&dir, "scan-a");
    let mut a = window(&project);
    let lock = canonical(&project).unwrap().join("project.toml.lock");
    fs::write(&lock, "Prozess 4711 · Benutzer kollege · Rechner Werkstatt").unwrap();
    let before = fs::read(project.join("project.toml")).unwrap();
    a.apply(Action::AddScan(raw.clone()));
    let e = errors(&a);
    assert_eq!(e.len(), 1, "{e:?}");
    assert!(
        e[0].starts_with(
            "project.toml wird gerade von einem anderen Fenster oder Programm geändert"
        ),
        "{e:?}"
    );
    assert!(e[0].contains("Rechner Werkstatt"), "{e:?}");
    assert_eq!(fs::read(project.join("project.toml")).unwrap(), before);
    assert!(lock.exists(), "another's lock is not removed");

    // left behind: older than the stale time
    let old = std::time::SystemTime::now() - driftless_io::lock::LOCK_STALE * 2;
    fs::File::options()
        .write(true)
        .open(&lock)
        .unwrap()
        .set_modified(old)
        .unwrap();
    a.apply(Action::AddScan(raw));
    settle(&mut a);
    assert_eq!(errors(&a).len(), 1, "{:?}", errors(&a));
    assert_eq!(Project::open(&project).unwrap().scans.len(), 1);
    assert!(!lock.exists(), "the lock goes with the change");
}

/// The lock guards `corrections.toml` too: a held lock refuses a loop, in the window and on
/// the command line (both go through `driftless_steps::correct`).
#[test]
fn corrections_are_locked_too() {
    let dir = tmp("gesperrt-korrekturen");
    let run = dir.join("lauf");
    fs::create_dir_all(&run).unwrap();
    let lock = run.join("corrections.toml.lock");
    fs::write(&lock, "Prozess 4711").unwrap();
    let e = correct::add_loop(&run, 1.0, 2.0, "")
        .unwrap_err()
        .to_string();
    assert!(
        e.starts_with("corrections.toml wird gerade von einem anderen Fenster oder Programm"),
        "{e}"
    );
    assert!(!run.join("corrections.toml").exists());
    fs::remove_file(&lock).unwrap();
    assert_eq!(correct::add_loop(&run, 1.0, 2.0, "").unwrap(), 1);
    assert!(!lock.exists());
}

/// „Genauigkeit“, „Feinheit“ and „ohne Fotos“ are kept per project: opened again (also in a
/// new window, after a restart) the project has what was chosen in it; another project keeps
/// its own.
#[test]
fn choices_are_kept_per_project() {
    let dir = tmp("auswahl-je-projekt");
    let (halle, buero) = (dir.join("halle"), dir.join("buero"));
    Project::create(&halle, "Halle").unwrap();
    Project::create(&buero, "Büro").unwrap();
    let mut a = window(&halle);
    let defaults = (
        a.state().map_settings.switches.voxel,
        a.state().surface_settings.config.voxel,
        a.state().surface_settings.texture,
    );
    a.state_mut().map_settings.switches.voxel = 0.005;
    a.state_mut().surface_settings.config.voxel = 0.01;
    a.state_mut().surface_settings.texture = false;
    settle(&mut a);
    assert!(canonical(&halle).unwrap().join("auswahl.toml").is_file());
    // another project: its own (here the defaults)
    a.apply(Action::OpenProject(buero.clone()));
    settle(&mut a);
    let chosen = |app: &DriftlessApp| {
        (
            app.state().map_settings.switches.voxel,
            app.state().surface_settings.config.voxel,
            app.state().surface_settings.texture,
        )
    };
    assert_eq!(chosen(&a), defaults);
    a.state_mut().map_settings.switches.voxel = 0.02;
    settle(&mut a);
    // back: what was chosen in it
    a.apply(Action::OpenProject(halle.clone()));
    settle(&mut a);
    assert_eq!(chosen(&a), (0.005, 0.01, false));
    // a new window (a restart) too
    let b = window(&halle);
    assert_eq!(chosen(&b), (0.005, 0.01, false));
    let c = window(&buero);
    assert_eq!(chosen(&c).0, 0.02);
}
