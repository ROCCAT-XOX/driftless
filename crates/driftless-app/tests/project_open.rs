//! Opening a project in the window (decision 3 of track U): `project.toml` and the tree are read
//! in the background, the window keeps drawing and says „Projekt wird gelesen …“ meanwhile; a
//! project in cloud storage is opened with a warning in the status bar and the Protokoll (once
//! per session); the dialogs start in the projects folder, then in the folder used last, kept
//! between starts.

use driftless_app::project::{canonical, Project};
use driftless_app::ui::{text, Action, DialogFolder, DriftlessApp, LogKind, UiMessage, UI_KEY};
use driftless_io::testscan::tmp;
use eframe::egui;
use egui_kittest::kittest::Queryable;
use egui_kittest::Harness;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, DriftlessApp>;

fn harness(app: DriftlessApp) -> H {
    Harness::builder()
        .with_size([1200.0, 800.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app)
}

/// Frames until `cond` holds (at most 30 s).
fn frames_until(h: &mut H, what: &str, cond: impl Fn(&DriftlessApp) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while !cond(h.state()) {
        assert!(Instant::now() < deadline, "{what}");
        h.step();
        std::thread::sleep(Duration::from_millis(2));
    }
    h.step();
}

/// Writes `project.toml` into the named pipe `path` when dropped (also when the test fails, so
/// that the reader waiting for it ends before the window is dropped and waits for it).
#[cfg(unix)]
struct Writer {
    path: PathBuf,
    text: &'static str,
    done: bool,
}

#[cfg(unix)]
impl Writer {
    fn write(&mut self) {
        if !self.done {
            self.done = true;
            std::fs::write(&self.path, self.text).unwrap();
        }
    }
}

#[cfg(unix)]
impl Drop for Writer {
    fn drop(&mut self) {
        self.write();
    }
}

/// A project whose `project.toml` is a named pipe: reading it waits until the test writes it,
/// as a slow network share or a file iCloud moved away would. The window keeps drawing, says
/// that it reads the project (in the field and in the status bar), and shows the project once
/// it has been read.
#[cfg(unix)]
#[test]
fn opening_does_not_stop_the_window() {
    let dir = tmp("oeffnen-langsam");
    let project = dir.join("Halle");
    std::fs::create_dir_all(&project).unwrap();
    let pipe = project.join("project.toml");
    let made = std::process::Command::new("mkfifo")
        .arg(&pipe)
        .status()
        .unwrap();
    assert!(made.success());
    let mut h = harness(DriftlessApp::new(Arc::new(|| {})));
    let mut writer = Writer {
        path: pipe,
        text: "format = 1\nname = \"Halle langsam\"\n",
        done: false,
    };
    h.state_mut().apply(Action::OpenProject(project.clone()));
    // frames go on while the reader waits
    let started = Instant::now();
    for _ in 0..5 {
        h.step();
    }
    assert!(started.elapsed() < Duration::from_secs(5));
    assert_eq!(h.state().state().opening, Some(project.clone()));
    assert!(h.state().state().project.is_none());
    h.get_by_label(text::PROJECT_READING);
    h.get_by_label_contains(text::READ_PROJECT_JOB);
    // the window answers: ☰ opens its menu
    h.get_by_label(text::MENU_PROJECT).click();
    h.step();
    h.step();
    h.get_by_label(text::NEW_PROJECT);
    h.key_press(egui::Key::Escape);
    h.step();

    writer.write();
    frames_until(&mut h, "project not opened", |app| {
        app.state().project.is_some() && !app.wants_repaint_while_busy()
    });
    let state = h.state().state();
    assert_eq!(state.project.as_ref().unwrap().name, "Halle langsam");
    assert_eq!(state.opening, None);
    assert!(h.query_by_label(text::PROJECT_READING).is_none());
    h.get_by_label(&text::project_line("Halle langsam", 0));
}

fn in_cloud(_: &Path) -> Option<String> {
    Some("iCloud Drive".into())
}

/// A project in cloud storage opens as any other, with a warning that says why it had better
/// lie elsewhere: in the status bar (yellow) and the Protokoll, once per session.
#[test]
fn a_project_in_cloud_storage_opens_with_a_warning() {
    let dir = tmp("oeffnen-wolke");
    Project::create(&dir.join("Halle"), "Halle").unwrap();
    let mut h = harness(DriftlessApp::new(Arc::new(|| {})).with_cloud_check(in_cloud));
    h.state_mut().apply(Action::OpenProject(dir.join("Halle")));
    frames_until(&mut h, "project not opened", |app| {
        app.state().project.is_some() && !app.wants_repaint_while_busy()
    });
    let warning = text::project_in_cloud("iCloud Drive", &canonical(&dir.join("Halle")).unwrap());
    let state = h.state().state();
    assert_eq!(state.project.as_ref().unwrap().name, "Halle");
    assert_eq!(
        state.messages.last(),
        Some(&UiMessage::Warning(warning.clone()))
    );
    let entry = state.log.last().unwrap();
    assert_eq!(
        (entry.kind, entry.text.as_str()),
        (LogKind::Warning, warning.as_str())
    );
    assert!(warning.contains("hochgeladen") && warning.contains("langsam"));
    // shown with the reason first
    h.get_by_label_contains("Projekt liegt in iCloud Drive: große Ergebnisse werden");

    // again in this session: no second warning
    let warnings = |app: &DriftlessApp| {
        app.state()
            .messages
            .iter()
            .filter(|m| matches!(m, UiMessage::Warning(_)))
            .count()
    };
    h.state_mut().apply(Action::OpenProject(dir.join("Halle")));
    frames_until(&mut h, "read again", |app| !app.wants_repaint_while_busy());
    assert_eq!(warnings(h.state()), 1);

    // a new project in cloud storage too
    h.state_mut().apply(Action::NewProject {
        dir: dir.join("Neu"),
        name: "Neu".into(),
    });
    frames_until(&mut h, "new project not read", |app| {
        !app.wants_repaint_while_busy()
    });
    assert_eq!(h.state().state().project.as_ref().unwrap().name, "Neu");
    assert_eq!(warnings(h.state()), 2);
}

/// The storage of eframe, in memory.
#[derive(Default)]
struct Storage(HashMap<String, String>);

impl eframe::Storage for Storage {
    fn get_string(&self, key: &str) -> Option<String> {
        self.0.get(key).cloned()
    }
    fn set_string(&mut self, key: &str, value: String) {
        self.0.insert(key.to_string(), value);
    }
    fn remove_string(&mut self, key: &str) {
        self.0.remove(key);
    }
    fn flush(&mut self) {}
}

/// The dialogs start in the projects folder (made when first needed), then in the folder they
/// were last used in, each group its own, kept between starts.
#[test]
fn dialogs_start_in_the_projects_folder_then_the_last_one() {
    let dir = tmp("oeffnen-startordner");
    let home = dir.join("Driftless Projekte");
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_projects_home(Some(home.clone()));
    for kind in [
        DialogFolder::Projects,
        DialogFolder::Scans,
        DialogFolder::Files,
    ] {
        assert_eq!(app.dialog_folder(kind), Some(home.clone()));
    }
    assert!(home.is_dir());
    let scans = dir.join("Rohdaten");
    std::fs::create_dir_all(&scans).unwrap();
    app.remember_folder(DialogFolder::Scans, Some(&scans));
    assert_eq!(app.dialog_folder(DialogFolder::Scans), Some(scans.clone()));
    assert_eq!(
        app.dialog_folder(DialogFolder::Projects),
        Some(home.clone())
    );

    // kept between starts
    let mut storage = Storage::default();
    eframe::App::save(&mut app, &mut storage);
    assert!(storage.0[UI_KEY].contains("Rohdaten"));
    let mut back = DriftlessApp::new(Arc::new(|| {})).with_projects_home(Some(home.clone()));
    back.restore_ui(&egui::Context::default(), Some(&storage));
    assert_eq!(back.dialog_folder(DialogFolder::Scans), Some(scans.clone()));
    // a folder that is gone: the projects folder again
    std::fs::remove_dir_all(&scans).unwrap();
    assert_eq!(back.dialog_folder(DialogFolder::Scans), Some(home));
    let _ = std::fs::remove_dir_all(&dir);
}
