//! The layout of the window: the rail with ☰, the three steps and ⚙, the field of the active
//! step, the settings window, the empty states and the stored layout.

use driftless_app::project::{Project, ScanEntry, ScanState};
use driftless_app::render::{ColorMode, PointSize, ShowMode};
use driftless_app::ui::{
    rail, steps, text, Action, AppState, DriftlessApp, ScanNode, Selection, Step, SurfaceSummary,
    ViewSettings, UI_KEY,
};
use driftless_io::testscan::tmp;
use eframe::egui::{self, accesskit::Role};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

fn window(app: DriftlessApp) -> Harness<'static, DriftlessApp> {
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run();
    h
}

/// A harness that shows `show` with `state` and keeps the last action.
fn panel(
    state: AppState,
    show: fn(&mut egui::Ui, &mut AppState) -> Option<Action>,
) -> Harness<'static, (AppState, Option<Action>)> {
    let mut h = Harness::builder().with_size([320.0, 700.0]).build_ui_state(
        move |ui, (state, action): &mut (AppState, Option<Action>)| {
            if let Some(a) = show(ui, state) {
                *action = Some(a);
            }
        },
        (state, None),
    );
    h.run();
    h
}

fn surface() -> SurfaceSummary {
    SurfaceSummary {
        path: PathBuf::from("oberflaeche.glb"),
        triangles: 10,
        pages: 1,
        origin: [0.0; 3],
        z_range: [0.0, 3.0],
    }
}

/// A project „Halle“ with one scan, the scan selected (not read from disk).
fn project_state() -> AppState {
    AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Halle".into(),
            scans: vec![ScanEntry {
                id: "buero".into(),
                path: PathBuf::from("/daten/buero"),
                name: "buero".into(),
                extra_runs: Vec::new(),
            }],
        }),
        scans: vec![ScanNode {
            id: "buero".into(),
            name: "buero".into(),
            state: ScanState::Ready,
            no_color: None,
            runs: Vec::new(),
        }],
        selection: Selection::Scan("buero".into()),
        ..AppState::default()
    }
}

#[test]
fn rail_switches_step() {
    let mut h = window(app());
    assert_eq!(h.state().state().step, Step::Daten);
    h.get_by_role_and_label(Role::Label, text::STEP_DATA);
    h.get_by_role_and_label(Role::Button, text::STEP_IMAGES)
        .click();
    h.run();
    assert_eq!(h.state().state().step, Step::Bilder);
    h.get_by_role_and_label(Role::Label, text::STEP_IMAGES);
    assert!(h
        .query_by_role_and_label(Role::Label, text::STEP_DATA)
        .is_none());
    h.get_by_role_and_label(Role::Button, text::STEP_PLAN)
        .click();
    h.run();
    assert_eq!(h.state().state().step, Step::Planung);
    h.get_by_role_and_label(Role::Label, text::STEP_PLAN);
}

#[test]
fn step_sets_show_mode() {
    let mut app = app();
    app.state_mut().surface = Some(surface());
    app.apply(Action::SetStep(Step::Bilder));
    assert_eq!(app.state().view.show, ShowMode::Surface);
    // by hand: holds until the next change of step
    app.state_mut().view.show = ShowMode::Cloud;
    app.apply(Action::SetStep(Step::Daten));
    assert_eq!(app.state().view.show, ShowMode::Cloud);
    app.apply(Action::SetStep(Step::Bilder));
    assert_eq!(app.state().view.show, ShowMode::Surface);
    app.apply(Action::SetStep(Step::Planung));
    assert_eq!(app.state().view.show, ShowMode::Cloud);

    // a surface still being loaded counts as one
    let mut app = self::app();
    let glb = tmp("ui-layout-laedt").join("oberflaeche.glb");
    app.apply(Action::LoadSurface(glb));
    assert!(app.is_loading_surface());
    assert!(app.state().surface.is_none());
    app.apply(Action::SetStep(Step::Bilder));
    assert_eq!(app.state().view.show, ShowMode::Surface);

    // without a surface the cloud stays, with a hint
    let mut h = window(self::app());
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.run();
    assert_eq!(h.state().state().step, Step::Bilder);
    assert_eq!(h.state().state().view.show, ShowMode::Cloud);
    h.get_by_label(text::NO_SURFACE_YET);
    h.state_mut().apply(Action::SetStep(Step::Daten));
    h.run();
    assert!(h.query_by_label(text::NO_SURFACE_YET).is_none());
}

#[test]
fn active_step_again_keeps_manual_view() {
    let mut app = app();
    app.state_mut().surface = Some(surface());
    app.apply(Action::SetStep(Step::Bilder));
    app.state_mut().view.show = ShowMode::Both;
    // a click on the step already shown changes nothing
    app.apply(Action::SetStep(Step::Bilder));
    assert_eq!(app.state().view.show, ShowMode::Both);

    // the same through the rail
    let mut h = window(app);
    h.get_by_role_and_label(Role::Button, text::STEP_IMAGES)
        .click();
    h.run();
    assert_eq!(h.state().state().step, Step::Bilder);
    assert_eq!(h.state().state().view.show, ShowMode::Both);
}

#[test]
fn menu_has_all_project_actions() {
    let entries = [
        (text::NEW_PROJECT, Action::PickNewProject),
        (text::OPEN_PROJECT, Action::PickOpenProject),
        (text::ADD_SCAN, Action::PickAddScan),
        (text::ADD_RUN, Action::PickAddRun("buero".into())),
        (text::OPEN_FILE, Action::PickOpenFile),
    ];
    let mut h = panel(project_state(), rail::show);
    for (label, _) in &entries {
        assert!(h.query_by_label(label).is_none(), "{label} before ☰");
    }
    for (label, expected) in entries {
        h.get_by_role_and_label(Role::Button, text::MENU_PROJECT)
            .click();
        h.run();
        for (l, _) in [
            (text::NEW_PROJECT, ()),
            (text::OPEN_PROJECT, ()),
            (text::ADD_SCAN, ()),
            (text::ADD_RUN, ()),
            (text::OPEN_FILE, ()),
            (text::EXPORT, ()),
        ] {
            h.get_by_label(l);
        }
        // one entry opens every format: no separate ones for clouds and surfaces
        assert!(h.query_by_label("Oberfläche öffnen …").is_none());
        assert!(h.query_by_label("Punktwolke öffnen …").is_none());
        h.state_mut().1 = None;
        h.get_by_label(label).click();
        h.run();
        assert_eq!(h.state().1, Some(expected), "{label}");
        // the menu closes after a choice
        assert!(h.query_by_label(text::OPEN_FILE).is_none(), "{label}");
    }
}

#[test]
fn menu_without_project_or_scan_explains_locked_entries() {
    let mut h = panel(AppState::default(), rail::show);
    h.get_by_role_and_label(Role::Button, text::MENU_PROJECT)
        .click();
    h.run();
    let disabled = |h: &Harness<'_, (AppState, Option<Action>)>, l: &str| {
        h.get_by_label(l).accesskit_node().is_disabled()
    };
    assert!(!disabled(&h, text::NEW_PROJECT));
    assert!(!disabled(&h, text::OPEN_PROJECT));
    assert!(disabled(&h, text::ADD_SCAN));
    assert!(disabled(&h, text::ADD_RUN));
    assert!(!disabled(&h, text::OPEN_FILE));
    // nothing shown: nothing to export
    assert!(disabled(&h, text::EXPORT));
}

#[test]
fn settings_window_has_view_settings() {
    let mut h = window(app());
    for label in [text::POINT_SIZE, text::BUDGET, text::COLOR_MODE] {
        assert!(h.query_by_label(label).is_none(), "{label} before ⚙");
    }
    h.get_by_role_and_label(Role::Button, text::SETTINGS)
        .click();
    h.run();
    assert!(h.state().state().settings_window.open);
    // world size is the default: the smallest size and the spot size from the map
    assert_eq!(h.state().state().view.point_size, PointSize::World);
    for label in [
        text::POINT_SIZE_MODE,
        text::POINT_SIZE_PIXEL,
        text::POINT_SIZE_WORLD,
        text::POINT_SIZE_MIN,
        text::SPOT_SIZE,
        text::SPOT_FROM_MAP,
    ] {
        h.get_by_label(label);
    }
    // off „aus der Karte“: the size of the map (here the default) to set by hand
    assert_eq!(h.state().state().view.point_m, None);
    h.get_by_label(text::SPOT_FROM_MAP).click();
    h.run();
    assert_eq!(h.state().state().view.point_m, Some(0.015));
    // „Pixel“: a fixed size, square or round
    h.get_by_label(text::POINT_SIZE_PIXEL).click();
    h.run();
    assert_eq!(h.state().state().view.point_size, PointSize::Pixel);
    for label in [
        text::POINT_SIZE,
        text::ROUND_POINTS,
        text::BUDGET,
        text::COLOR_MODE,
        text::CAMERA_MODE,
        text::ORBIT,
        text::FLY,
        text::LAYERS,
        text::LAYER_TRAJECTORY,
        text::LAYER_ODOMETRY,
        text::LAYER_LOOPS,
        text::LAYER_REJECTED,
        text::LAYER_SUBMAPS,
        text::LAYER_COMPARE,
    ] {
        h.get_by_label(label);
    }
    let before = h.state().state().view.point_px;
    h.get_by_label(text::POINT_SIZE).click();
    h.run();
    let after = h.state().state().view.point_px;
    assert_ne!(before, after, "the slider did not change the point size");
    h.get_by_label(text::FLY).click();
    h.run();
    assert_eq!(
        h.state().state().view.camera_mode,
        driftless_app::camera::CameraMode::Fly
    );
    h.get_by_label(text::MODE_INTENSITY).click();
    h.run();
    assert_eq!(h.state().state().view.color_mode, ColorMode::Intensity);
    // „Grafikkarte nutzen“: Auto by default, the choice is kept in the state
    assert_eq!(h.state().state().gpu, driftless_gpu::GpuPref::Auto);
    h.get_by_label(text::GPU);
    h.get_by_label(text::GPU_OFF).click();
    h.run();
    assert_eq!(h.state().state().gpu, driftless_gpu::GpuPref::Off);
    h.get_by_label(text::GPU_ON).click();
    h.run();
    assert_eq!(h.state().state().gpu, driftless_gpu::GpuPref::On);
    // ⚙ again closes it
    h.get_by_role_and_label(Role::Button, text::SETTINGS)
        .click();
    h.run();
    assert!(!h.state().state().settings_window.open);
    assert!(h.query_by_label(text::POINT_SIZE).is_none());
}

/// „je Scan“ is offered for a cloud of several scans only.
#[test]
fn settings_offer_scan_mode_only_for_models() {
    for has_scans in [false, true] {
        let mut app = app();
        app.state_mut().cloud = Some(driftless_app::ui::CloudSummary {
            path: PathBuf::from("modell.las"),
            points: 10,
            has_rgb: true,
            has_scans,
            quality_known: false,
            z_range: [0.0, 1.0],
            ceiling: None,
            point_m: 0.015,
        });
        app.state_mut().settings_window.open = true;
        let mut h = window(app);
        h.get_by_label(text::MODE_RGB);
        if !has_scans {
            assert!(h.query_by_label(text::MODE_SCAN).is_none());
            continue;
        }
        h.get_by_label(text::MODE_SCAN).click();
        h.run();
        assert_eq!(h.state().state().view.color_mode, ColorMode::Scan);
    }
}

#[test]
fn empty_states_explain_next_step() {
    for step in [Step::Daten, Step::Bilder, Step::Planung] {
        let state = AppState {
            step,
            ..AppState::default()
        };
        let mut h = panel(state, steps::show);
        h.get_by_label(text::EMPTY_NO_PROJECT);
        h.get_by_label(text::OPEN_PROJECT).click();
        h.run();
        assert_eq!(h.state().1, Some(Action::PickOpenProject), "{step:?}");
        h.get_by_label(text::NEW_PROJECT).click();
        h.run();
        assert_eq!(h.state().1, Some(Action::PickNewProject), "{step:?}");
    }

    // a project without scans: Daten asks for one
    let mut state = project_state();
    if let Some(p) = &mut state.project {
        p.scans.clear();
    }
    state.scans.clear();
    state.selection = Selection::None;
    let mut h = panel(state.clone(), steps::show);
    h.get_by_label(text::EMPTY_NO_SCANS);
    h.get_by_label(text::ADD_SCAN).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::PickAddScan));

    // Bilder without a map sends to Daten
    state.step = Step::Bilder;
    let mut h = panel(state.clone(), steps::show);
    h.get_by_label(text::EMPTY_NO_MAPS);
    h.get_by_label(text::GO_TO_DATA).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::SetStep(Step::Daten)));

    // Planung says what comes
    state.step = Step::Planung;
    let h = panel(state, steps::show);
    h.get_by_label(text::PLAN_COMING);
}

/// eframe's storage in memory.
#[derive(Default)]
struct Storage(HashMap<String, String>);

impl eframe::Storage for Storage {
    fn get_string(&self, key: &str) -> Option<String> {
        self.0.get(key).cloned()
    }
    fn set_string(&mut self, key: &str, value: String) {
        self.0.insert(key.into(), value);
    }
    fn remove_string(&mut self, key: &str) {
        self.0.remove(key);
    }
    fn flush(&mut self) {}
}

#[test]
fn old_stored_layout_is_ignored() {
    let old_panel = egui::Id::new("projekt");
    for stored in [
        // the layout of before, under the old key
        vec![("ui", r#"{"links":280.0,"rechts":340.0}"#)],
        // the old format under the new key
        vec![(UI_KEY, r#"{"links":280.0}"#)],
        vec![(UI_KEY, "kein json")],
        vec![(UI_KEY, r#"{"step":"Gibtsnicht"}"#)],
        Vec::new(),
    ] {
        let storage = Storage(
            stored
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        );
        let ctx = egui::Context::default();
        ctx.memory_mut(|m| m.data.insert_persisted(old_panel, 280.0f32));
        let mut app = app();
        app.restore_ui(&ctx, Some(&storage));
        assert_eq!(app.state().step, Step::Daten, "{stored:?}");
        assert_eq!(app.state().view, ViewSettings::default(), "{stored:?}");
        // the window state of the old layout is gone
        let old = ctx.memory_mut(|m| m.data.get_persisted::<f32>(old_panel));
        assert_eq!(old, None, "{stored:?}");
    }

    // what is stored now comes back, and the window state stays
    let mut app = app();
    app.apply(Action::SetStep(Step::Bilder));
    app.state_mut().view.point_px = 4.5;
    app.state_mut().view.color_mode = ColorMode::Quality;
    let mut storage = Storage::default();
    eframe::App::save(&mut app, &mut storage);
    let ctx = egui::Context::default();
    ctx.memory_mut(|m| m.data.insert_persisted(old_panel, 300.0f32));
    let mut back = self::app();
    back.restore_ui(&ctx, Some(&storage));
    assert_eq!(back.state().step, Step::Bilder);
    assert_eq!(back.state().view.point_px, 4.5);
    assert_eq!(back.state().view.color_mode, ColorMode::Quality);
    let kept = ctx.memory_mut(|m| m.data.get_persisted::<f32>(old_panel));
    assert_eq!(kept, Some(300.0));
}

/// A `ui-v2` stored before the spot size existed (without `point_size` and `point_m`)
/// still loads: its settings come back, the two new fields take their defaults (spots from
/// the map). And both new fields survive saving and loading.
#[test]
fn stored_view_without_spot_fields_loads() {
    use driftless_app::render::PointSize;
    // what is stored now, minus the two new fields: the format of before
    let mut app = app();
    app.apply(Action::SetStep(Step::Bilder));
    app.state_mut().view.point_px = 3.5;
    app.state_mut().view.color_mode = ColorMode::Intensity;
    let mut stored: serde_json::Value = serde_json::from_str(&app.ui_to_store()).unwrap();
    let fields = stored.as_object_mut().unwrap();
    assert!(fields.remove("point_size").is_some());
    assert!(fields.remove("point_m").is_some());
    let storage = Storage([(UI_KEY.to_string(), stored.to_string())].into());
    let mut back = self::app();
    back.restore_ui(&egui::Context::default(), Some(&storage));
    assert_eq!(back.state().step, Step::Bilder);
    assert_eq!(back.state().view.point_px, 3.5);
    assert_eq!(back.state().view.color_mode, ColorMode::Intensity);
    assert_eq!(back.state().view.point_size, PointSize::World);
    assert_eq!(back.state().view.point_m, None);

    // round trip of both new fields
    for (size, m) in [(PointSize::Pixel, Some(0.04)), (PointSize::World, None)] {
        let mut app = self::app();
        app.state_mut().view.point_size = size;
        app.state_mut().view.point_m = m;
        let mut storage = Storage::default();
        eframe::App::save(&mut app, &mut storage);
        let mut back = self::app();
        back.restore_ui(&egui::Context::default(), Some(&storage));
        assert_eq!(back.state().view.point_size, size);
        assert_eq!(back.state().view.point_m, m);
    }
    // a hand-set size out of range is clamped when loaded
    let mut app = self::app();
    app.state_mut().view.point_m = Some(5.0);
    let mut storage = Storage::default();
    eframe::App::save(&mut app, &mut storage);
    let mut back = self::app();
    back.restore_ui(&egui::Context::default(), Some(&storage));
    assert_eq!(back.state().view.point_m, Some(text::POINT_M_RANGE[1]));
}

#[test]
fn projects_of_disk_show_in_daten() {
    // a real (empty) project: Daten shows its name and the hint for a first scan
    let dir = tmp("ui-layout-leer");
    Project::create(&dir, "Leer").unwrap();
    let mut app = app();
    app.apply(Action::OpenProject(dir));
    let h = window(app);
    h.get_by_label(&text::project_line("Leer", 0));
    h.get_by_label(text::EMPTY_NO_SCANS);
}

/// „Grafikkarte nutzen“ survives saving and loading; stored without it (before it existed) or
/// with an unknown word it is Auto.
#[test]
fn stored_gpu_choice() {
    use driftless_gpu::GpuPref;
    for gpu in [GpuPref::Off, GpuPref::On, GpuPref::Auto] {
        let mut app = app();
        app.state_mut().gpu = gpu;
        let mut storage = Storage::default();
        eframe::App::save(&mut app, &mut storage);
        let mut back = self::app();
        back.restore_ui(&egui::Context::default(), Some(&storage));
        assert_eq!(back.state().gpu, gpu);
    }
    for stored in [None, Some("vielleicht")] {
        let mut app = app();
        app.state_mut().gpu = GpuPref::Off;
        app.state_mut().view.point_px = 3.5;
        let mut ui: serde_json::Value = serde_json::from_str(&app.ui_to_store()).unwrap();
        let fields = ui.as_object_mut().unwrap();
        match stored {
            None => assert!(fields.remove("gpu").is_some()),
            Some(word) => {
                fields.insert("gpu".into(), word.into());
            }
        }
        let storage = Storage([(UI_KEY.to_string(), ui.to_string())].into());
        let mut back = self::app();
        back.restore_ui(&egui::Context::default(), Some(&storage));
        assert_eq!(back.state().gpu, GpuPref::Auto, "{stored:?}");
        assert_eq!(back.state().view.point_px, 3.5, "{stored:?}");
    }
}
