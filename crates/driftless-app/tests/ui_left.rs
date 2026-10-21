//! The project tree on the left, without a window: what it shows and which actions its
//! entries return.

use driftless_app::project::{MapInfo, Project, RunInfo, ScanEntry, ScanState};
use driftless_app::ui::{left, text, Action, AppState, ScanNode, Selection};
use driftless_steps::slam::SlamSettings;
use egui_kittest::kittest::Queryable;
use egui_kittest::Harness;
use std::path::{Path, PathBuf};

fn run(dir: &str, label: &str, maps: Vec<MapInfo>) -> RunInfo {
    RunInfo {
        dir: PathBuf::from(dir),
        label: label.into(),
        settings: SlamSettings::default(),
        has_result: true,
        has_cache: true,
        has_previous: false,
        cloud: None,
        maps,
        history: Vec::new(),
    }
}

/// A project „Halle“ with the scan „buero“: SLAM 001 with Karte 001 (color.las), SLAM 002.
fn state() -> AppState {
    let map = MapInfo {
        dir: PathBuf::from("/p/scans/buero/slam-001/karte-001"),
        label: "Karte 001".into(),
        settings: None,
        clouds: vec![PathBuf::from("/p/scans/buero/slam-001/karte-001/color.las")],
    };
    let project = Project {
        dir: PathBuf::from("/p"),
        name: "Halle".into(),
        scans: vec![ScanEntry {
            id: "buero".into(),
            path: PathBuf::from("/daten/buero"),
            name: "buero".into(),
            extra_runs: Vec::new(),
        }],
    };
    AppState {
        project: Some(project),
        scans: vec![ScanNode {
            id: "buero".into(),
            name: "buero".into(),
            state: ScanState::Ready,
            no_color: None,
            runs: vec![
                run("/p/scans/buero/slam-001", "SLAM 001", vec![map]),
                run("/p/scans/buero/slam-002", "SLAM 002", Vec::new()),
            ],
        }],
        ..AppState::default()
    }
}

/// A harness that shows the tree of `state` and keeps the last action.
fn harness(state: AppState) -> Harness<'static, (AppState, Option<Action>)> {
    Harness::builder().with_size([320.0, 600.0]).build_ui_state(
        |ui, (state, action): &mut (AppState, Option<Action>)| {
            if let Some(a) = left::show(ui, state) {
                *action = Some(a);
            }
        },
        (state, None),
    )
}

fn top(h: &Harness<'_, (AppState, Option<Action>)>, label: &str) -> f32 {
    h.get_by_label(label).rect().top()
}

#[test]
fn left_shows_scans_runs_maps() {
    let mut h = harness(state());
    h.run();
    h.get_by_label(text::PANEL_PROJECT);
    h.get_by_label("Halle");
    let order = ["buero", "SLAM 001", "Karte 001", "color.las", "SLAM 002"];
    let tops: Vec<f32> = order.iter().map(|l| top(&h, l)).collect();
    assert!(
        tops.windows(2).all(|w| w[0] < w[1]),
        "order {order:?} at {tops:?}"
    );
    h.get_by_label(text::ADD_SCAN);
    h.get_by_label(text::ADD_RUN);
}

#[test]
fn clicking_run_selects_it() {
    let mut h = harness(state());
    h.run();
    h.get_by_label("SLAM 002").click();
    h.run();
    assert_eq!(
        h.state().1,
        Some(Action::Select(Selection::Run(PathBuf::from(
            "/p/scans/buero/slam-002"
        ))))
    );
    h.get_by_label("color.las").click();
    h.run();
    assert_eq!(
        h.state().1,
        Some(Action::Select(Selection::Cloud(PathBuf::from(
            "/p/scans/buero/slam-001/karte-001/color.las"
        ))))
    );
    h.get_by_label("buero").click();
    h.run();
    assert_eq!(
        h.state().1,
        Some(Action::Select(Selection::Scan("buero".into())))
    );
}

#[test]
fn add_buttons_return_actions() {
    let mut h = harness(state());
    h.run();
    h.get_by_label(text::ADD_RUN).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::PickAddRun("buero".into())));
    h.get_by_label(text::ADD_SCAN).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::PickAddScan));
}

#[test]
fn missing_scan_is_marked() {
    let gone = Path::new("/nicht/vorhanden/buero");
    let project = Project {
        dir: PathBuf::from("/nicht/vorhanden/projekt"),
        name: "Halle".into(),
        scans: vec![ScanEntry {
            id: "buero".into(),
            path: gone.to_path_buf(),
            name: "buero".into(),
            extra_runs: Vec::new(),
        }],
    };
    let mut state = AppState::default();
    state.set_project(project);
    let ScanState::Missing(reason) = state.scans[0].state.clone() else {
        panic!("scan should be missing: {:?}", state.scans[0].state);
    };
    assert!(reason.contains("nicht gefunden"), "{reason}");

    let mut h = harness(state);
    h.ctx.all_styles_mut(|s| s.interaction.tooltip_delay = 0.0);
    h.run();
    let label = format!("buero ({})", text::SCAN_MISSING);
    h.get_by_label(&label).hover();
    h.run();
    h.get_by_label(&reason);
    // still selectable: its runs stay viewable
    h.get_by_label(&label).click();
    h.run();
    assert_eq!(
        h.state().1,
        Some(Action::Select(Selection::Scan("buero".into())))
    );
}

#[test]
fn no_project_offers_open_and_new() {
    let mut h = harness(AppState::default());
    h.run();
    h.get_by_label(text::NO_PROJECT);
    h.get_by_label(text::NEW_PROJECT).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::PickNewProject));
    h.get_by_label(text::OPEN_PROJECT).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::PickOpenProject));
    assert!(h.query_by_label(text::ADD_SCAN).is_none());
}
