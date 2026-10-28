//! Disk use in the window (decision 2 of track U): each run, map, surface and model shows its
//! size in „Daten“, measured in the background; the project line says the project's total and
//! the free space of its drive. Below 10 GB free, the field shows a warning and the Protokoll
//! gets one (once); a step that will need more than is free is warned about when it starts.

use driftless_app::project::canonical;
use driftless_app::ui::{steps, text, Action, DriftlessApp, LogKind, Selection, UiMessage};
use driftless_io::testscan::{room_bag_secs, tmp};
use egui_kittest::kittest::Queryable;
use egui_kittest::Harness;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

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

/// A project with the scan `halle` (1.5 s) and its map (cleaned, no images): the app, the run
/// and the map folder.
fn project_with_map(dir: &Path, app: DriftlessApp) -> (DriftlessApp, PathBuf, PathBuf) {
    let scan = dir.join("halle");
    std::fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 1.5);
    let mut app = app;
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    app.apply(Action::AddScan(scan));
    settle(&mut app);
    app.state_mut().map_settings.step = driftless_steps::map::MapStep::Clean;
    app.apply(Action::ComputeMap("halle".into()));
    settle(&mut app);
    let run = canonical(&dir.join("projekt"))
        .unwrap()
        .join("scans/halle/slam-001");
    let map = run.join("karte-001");
    assert!(map.join("map.json").is_file());
    (app, run, map)
}

#[test]
fn sizes_show_in_the_tree_and_the_project_line() {
    let dir = tmp("platz-baum");
    let (mut app, run, map) = project_with_map(&dir, DriftlessApp::new(Arc::new(|| {})));
    let disk = app.state().disk.clone().expect("measured");
    let (run_bytes, map_bytes) = (disk.folders[&run], disk.folders[&map]);
    assert!(
        map_bytes > 0 && run_bytes > map_bytes,
        "{run_bytes} {map_bytes}"
    );
    assert!(disk.total >= run_bytes);
    assert!(disk.free.is_some());
    assert!(disk.raw.values().all(|&b| b > 0));
    app.apply(Action::Select(Selection::Run(run.clone())));
    settle(&mut app);
    let mut h = Harness::builder()
        .with_size([400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    steps::daten::set_advanced(&h.ctx, false);
    h.step();
    h.step();
    h.get_by_label(&text::with_size("SLAM 001", run_bytes));
    h.get_by_label(&text::with_size("Karte 001", map_bytes));
    h.get_by_label(&text::disk_line(disk.total, disk.free));
}

fn little(_: &Path) -> Option<u64> {
    Some(3_000)
}

/// Little free space: a warning in the field and, once, in the Protokoll; a step that needs
/// more than is free is warned about when it starts (it still runs).
#[test]
fn little_space_is_warned_about() {
    let dir = tmp("platz-knapp");
    let app = DriftlessApp::new(Arc::new(|| {})).with_free_space(little);
    let (mut app, _, _) = project_with_map(&dir, app);
    let low = text::low_space(3_000);
    let warnings = |app: &DriftlessApp, what: &str| {
        app.state()
            .log
            .iter()
            .filter(|e| e.kind == LogKind::Warning && e.text.starts_with(what))
            .count()
    };
    assert_eq!(warnings(&app, &low), 1, "{:?}", app.state().log);
    // the next steps and reads say it no more
    app.apply(Action::Inspect("halle".into()));
    settle(&mut app);
    assert_eq!(warnings(&app, &low), 1);
    // a step that needs more than is free
    app.apply(Action::RunSlam("halle".into()));
    let needs =
        app.state().messages.iter().any(
            |m| matches!(m, UiMessage::Warning(t) if t.starts_with("SLAM rechnen braucht etwa")),
        );
    assert!(needs, "{:?}", app.state().messages);
    assert!(app.state().job.is_some(), "it still runs");
    settle(&mut app);
    // the field shows the warning
    let mut h = Harness::builder()
        .with_size([400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.step();
    h.step();
    h.get_by_label_contains("Nur noch 3 KB frei");
}
