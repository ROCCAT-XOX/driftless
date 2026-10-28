//! Step 2 „Bilder“ and step 3 „Planung“: the maps with their surfaces below them, the
//! fineness switch, „Alle Oberflächen rechnen“ with the queue of „Daten“ (only maps without a
//! current surface, with the fineness chosen when it started), „ohne Fotos“ under
//! „Erweitert“, and the placeholder of „Planung“.

use driftless_app::project::{
    canonical, MapInfo, MapStatus, ModelInfo, Project, RunInfo, ScanEntry, ScanState, SurfaceInfo,
    SurfaceSettings,
};
use driftless_app::ui::steps::bilder::{model_surface_status, surface_status, SurfaceStatus};
use driftless_app::ui::theme::icon;
use driftless_app::ui::{
    steps, text, Action, AppState, DriftlessApp, Queued, ScanNode, Selection, Step, UiMessage,
};
use driftless_io::images::RgbImage;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_map::surface::{encode_pages, write_glb, SurfaceConfig, TexturePage, TexturedMesh};
use driftless_steps::map::{MapStep, MapSwitches};
use driftless_steps::merge::{ModelReport, MERGE_VERSION};
use driftless_steps::slam::SlamSettings;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, (AppState, Option<Action>)>;

/// A harness that shows the field of `state.step` and keeps the last action.
fn panel(state: AppState) -> H {
    let mut h = Harness::builder()
        .with_size([320.0, 1200.0])
        .build_ui_state(
            |ui, (state, action): &mut (AppState, Option<Action>)| {
                if let Some(a) = steps::show(ui, state) {
                    *action = Some(a);
                }
            },
            (state, None),
        );
    h.run();
    h
}

/// Clicks `label` and returns the action it gave.
fn click(h: &mut H, label: &str) -> Option<Action> {
    h.state_mut().1 = None;
    h.get_by_label(label).click();
    h.run();
    h.state_mut().1.take()
}

fn surface(dir: &str, label: &str) -> SurfaceInfo {
    SurfaceInfo {
        dir: PathBuf::from(dir),
        label: label.into(),
        stale: false,
        settings: Some(SurfaceSettings::default()),
        hints: Vec::new(),
    }
}

fn map(dir: &str, surfaces: Vec<SurfaceInfo>) -> MapInfo {
    MapInfo {
        dir: PathBuf::from(dir),
        label: "Karte 001".into(),
        settings: Some(MapSwitches::default()),
        clouds: Vec::new(),
        surfaces,
    }
}

fn run(dir: &str, maps: Vec<MapInfo>) -> RunInfo {
    RunInfo {
        dir: PathBuf::from(dir),
        label: "SLAM 001".into(),
        settings: SlamSettings::default(),
        has_result: true,
        has_cache: true,
        has_previous: false,
        has_odometry: true,
        cloud: None,
        maps,
        history: Vec::new(),
    }
}

const MAP_A: &str = "/p/scans/a/slam-001/karte-001";
const MAP_B: &str = "/p/scans/b/slam-001/karte-001";
const MAP_C: &str = "/p/scans/c/slam-001/karte-001";
const MAP_D: &str = "/p/scans/d/slam-001/karte-001";

/// A project „Halle“ (not read from disk) in step „Bilder“ with four scans of one map each:
/// a with a current „Oberfläche 001“ and a stale „Oberfläche 002“, b without a surface, c
/// with only a stale „Oberfläche 003“, and d whose map no longer fits its run.
fn four_maps() -> AppState {
    let ids = ["a", "b", "c", "d"];
    let maps = [
        map(
            MAP_A,
            vec![
                surface(&format!("{MAP_A}/oberflaeche-001"), "Oberfläche 001"),
                surface(&format!("{MAP_A}/oberflaeche-002"), "Oberfläche 002"),
            ],
        ),
        map(MAP_B, Vec::new()),
        map(
            MAP_C,
            vec![surface(
                &format!("{MAP_C}/oberflaeche-001"),
                "Oberfläche 003",
            )],
        ),
        map(MAP_D, Vec::new()),
    ];
    let mut state = AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Halle".into(),
            scans: ids
                .iter()
                .map(|id| ScanEntry {
                    id: (*id).into(),
                    path: PathBuf::from(format!("/daten/{id}")),
                    name: format!("Scan {id}"),
                    extra_runs: Vec::new(),
                    bags: Vec::new(),
                })
                .collect(),
        }),
        scans: ids
            .iter()
            .zip(maps)
            .map(|(id, map)| ScanNode {
                id: (*id).into(),
                name: format!("Scan {id}"),
                state: ScanState::Ready,
                no_color: None,
                runs: vec![run(&format!("/p/scans/{id}/slam-001"), vec![map])],
            })
            .collect(),
        step: Step::Bilder,
        ..AppState::default()
    };
    let status = |stale| MapStatus {
        seconds: Some(10.0),
        stale,
    };
    state.set_map_status(
        [
            (MAP_A.into(), status(false)),
            (format!("{MAP_A}/oberflaeche-001").into(), status(false)),
            (format!("{MAP_A}/oberflaeche-002").into(), status(true)),
            (MAP_B.into(), status(false)),
            (MAP_C.into(), status(false)),
            (format!("{MAP_C}/oberflaeche-001").into(), status(true)),
            (MAP_D.into(), status(true)),
        ]
        .into(),
    );
    state
}

fn map_name(id: &str) -> String {
    text::map_of(&format!("Scan {id}"), "SLAM 001", "Karte 001")
}

#[test]
fn surfaces_listed_per_map() {
    let state = four_maps();
    let status: Vec<SurfaceStatus> = state
        .scans
        .iter()
        .map(|s| surface_status(&state, &s.runs[0].maps[0]))
        .collect();
    assert_eq!(
        status,
        [
            SurfaceStatus::Done,
            SurfaceStatus::New,
            SurfaceStatus::Stale,
            SurfaceStatus::MapStale
        ]
    );
    let mut h = panel(state);
    h.get_by_label(text::STEP_IMAGES);
    // the heading stands out over everything below it, as in „Daten“
    let heading = h.get_by_label(text::STEP_IMAGES).rect().height();
    let section = h.get_by_label(text::SURFACE_FINENESS).rect().height();
    assert!(heading > section, "heading {heading} ≤ section {section}");
    let a = h.get_by_label(&map_name("a")).rect();
    let b = h.get_by_label(&map_name("b")).rect();
    let c = h.get_by_label(&map_name("c")).rect();
    let d = h.get_by_label(&map_name("d")).rect();
    // each status on the row of its map
    for (row, status) in [
        (a, text::STATUS_DONE),
        (b, text::STATUS_NEW),
        (c, text::STATUS_STALE),
        (d, text::STATUS_MAP_STALE),
    ] {
        let found = h
            .query_all_by_label(status)
            .any(|s| (s.rect().center().y - row.center().y).abs() < 6.0);
        assert!(found, "{status} not next to its map");
    }
    // the surfaces stand below their map, before the next one; stale ones say so
    let between = |label: &str, top: f32, bottom: f32| {
        let y = h.get_by_label(label).rect().center().y;
        assert!(
            top < y && y < bottom,
            "{label} at {y}, not in {top}…{bottom}"
        );
    };
    between("Oberfläche 001", a.bottom(), b.top());
    between(&text::surface_stale("Oberfläche 002"), a.bottom(), b.top());
    between(&text::surface_stale("Oberfläche 003"), c.bottom(), d.top());
    assert!(text::surface_stale("Oberfläche 002").contains(text::SURFACE_STALE));
    assert!(h.query_by_label("Oberfläche 002").is_none());

    // a click on a surface selects it, and the program loads it
    let dir = tmp("ui-bilder-laden");
    let glb_dir = dir.join("oberflaeche-001");
    small_glb(&glb_dir.join("oberflaeche.glb"));
    h.state_mut().0.scans[0].runs[0].maps[0].surfaces[0].dir = glb_dir.clone();
    h.run();
    let action = click(&mut h, "Oberfläche 001");
    assert_eq!(
        action,
        Some(Action::Select(Selection::Surface(glb_dir.clone())))
    );
    let mut app = app();
    app.apply(action.unwrap());
    assert!(app.is_loading_surface());
    let deadline = Instant::now() + Duration::from_secs(30);
    while app.is_loading_surface() {
        assert!(Instant::now() < deadline, "surface not loaded");
        std::thread::sleep(Duration::from_millis(2));
        app.tick();
    }
    assert_eq!(
        app.state().surface.as_ref().map(|s| s.path.clone()),
        Some(glb_dir.join("oberflaeche.glb"))
    );

    // a click on a map card selects the map
    assert_eq!(
        click(&mut h, &map_name("b")),
        Some(Action::Select(Selection::Map(MAP_B.into())))
    );
}

#[test]
fn main_button_surfaces_all_or_the_selected_map() {
    let mut h = panel(four_maps());
    assert_eq!(
        click(&mut h, text::COMPUTE_ALL_SURFACES),
        Some(Action::ComputeAllSurfaces)
    );
    // with a map (or a surface of it) selected the main button computes its surface
    h.state_mut().0.selection = Selection::Surface(format!("{MAP_A}/oberflaeche-001").into());
    h.run();
    assert_eq!(
        click(&mut h, text::STEP_SURFACE),
        Some(Action::RunSurface(MAP_A.into()))
    );
    h.get_by_label(text::THIS_SURFACE);
    // „Alle Oberflächen rechnen“ stays at hand
    assert_eq!(
        click(&mut h, text::COMPUTE_ALL_SURFACES),
        Some(Action::ComputeAllSurfaces)
    );

    // every map with a current surface: nothing to compute, and the field says so
    let mut state = four_maps();
    state.scans.truncate(1);
    let mut h = panel(state);
    h.get_by_label(text::ALL_SURFACED);
    assert_eq!(click(&mut h, text::COMPUTE_ALL_SURFACES), None);
    // done, but with another fineness than the one chosen: how to get it
    assert!(h.query_by_label(text::SURFACE_RECOMPUTE_HINT).is_none());
    click(&mut h, "1,5 cm");
    h.get_by_label(text::SURFACE_RECOMPUTE_HINT);
    // the fineness of its surface (the default, 2 cm) again: no hint
    click(&mut h, "2 cm");
    assert!(h.query_by_label(text::SURFACE_RECOMPUTE_HINT).is_none());
}

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
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

/// Settings that are quick on the synthetic room (a point every 10 cm or so) at `voxel`.
fn quick(voxel: f64) -> SurfaceSettings {
    SurfaceSettings {
        config: SurfaceConfig {
            voxel,
            min_weight: 1,
            min_component_faces: 10,
            page_size: 256,
            simplify_error: 0.01,
            ..SurfaceConfig::default()
        },
        texture: true,
    }
}

type W = Harness<'static, DriftlessApp>;

/// Frames until no step and no background work is left; returns each step seen running.
fn settle(h: &mut W) -> Vec<Action> {
    let mut seen: Vec<Action> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        h.step();
        let app = h.state();
        if let Some(a) = &app.state().running {
            if seen.last() != Some(a) {
                seen.push(a.clone());
            }
        }
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            return seen;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// The map folder of the newest run of scan `id`.
fn map_of(h: &W, id: &str) -> PathBuf {
    let node = h.state().state().scan_node(id).unwrap();
    node.runs.last().unwrap().maps.last().unwrap().dir.clone()
}

fn surfaces_of(h: &W, map: &Path) -> Vec<SurfaceInfo> {
    h.state().state().map(map).unwrap().2.surfaces.clone()
}

#[test]
fn compute_all_surfaces_queues_missing() {
    // only maps without a current surface, in the order of the tree; the stale map is left
    // to „Daten“
    let state = four_maps();
    assert_eq!(
        state.maps_to_surface(),
        [PathBuf::from(MAP_B), MAP_C.into()]
    );

    // on disk: two scans with a cleaned map each, a with a surface
    let dir = tmp("ui-bilder-alle");
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    for name in ["a", "b"] {
        let scan = dir.join(name);
        fs::create_dir_all(&scan).unwrap();
        room_bag_secs(&scan, true, 1.5);
        app.apply(Action::AddScan(scan));
    }
    app.state_mut().map_settings.step = MapStep::Clean;
    app.state_mut().surface_settings = quick(0.15);
    let mut h = Harness::builder()
        .with_size([1400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.state_mut().apply(Action::ComputeAllMaps);
    settle(&mut h);
    let (a, b) = (map_of(&h, "a"), map_of(&h, "b"));
    h.state_mut().apply(Action::RunSurface(a.clone()));
    settle(&mut h);
    assert_eq!(errors(h.state()), Vec::<String>::new());
    assert_eq!(surfaces_of(&h, &a).len(), 1);

    // the fineness from the switch, then „Alle Oberflächen rechnen“; nothing selected
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.state_mut().apply(Action::Select(Selection::None));
    h.step();
    h.get_by_label("2 cm").click();
    h.step();
    assert_eq!(h.state().state().surface_settings.config.voxel, 0.02);
    h.get_by_label(text::COMPUTE_ALL_SURFACES).click();
    h.step();
    let queue: Vec<Queued> = h.state().state().queue.iter().cloned().collect();
    assert_eq!(queue, [Queued::Surface(b.clone())]);
    assert_eq!(
        h.state().state().running,
        Some(Action::RunSurface(b.clone()))
    );
    assert!(matches!(
        surface_status(h.state().state(), h.state().state().map(&b).unwrap().2),
        SurfaceStatus::Running(_)
    ));
    // changed while it runs: the queue keeps the fineness it started with
    h.state_mut().state_mut().surface_settings.config.voxel = 0.015;
    let seen = settle(&mut h);
    assert_eq!(seen, [Action::RunSurface(b.clone())]);
    assert_eq!(errors(h.state()), Vec::<String>::new());
    assert!(h.state().state().queue.is_empty());
    let made = surfaces_of(&h, &b);
    assert_eq!(made.len(), 1);
    assert_eq!(made[0].settings.as_ref().unwrap().config.voxel, 0.02);
    assert_eq!(h.state().state().surface_settings.config.voxel, 0.015);
    // a was not computed again
    assert_eq!(surfaces_of(&h, &a).len(), 1);
    assert_eq!(
        canonical(&made[0].dir).unwrap(),
        canonical(&b.join("oberflaeche-001")).unwrap()
    );

    // nothing left: nothing starts
    h.state_mut().apply(Action::ComputeAllSurfaces);
    assert!(h.state().state().queue.is_empty());
    assert!(h.state().state().running.is_none());
}

#[test]
fn cancel_empties_the_surface_queue() {
    let state = four_maps();
    assert_eq!(state.maps_to_surface().len(), 2);
    // „Abbrechen“, close and project switch stop it as they stop the maps' queue: the same
    // queue (tested in ui_daten); here only that surfaces go through it
    let dir = tmp("ui-bilder-abbrechen");
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    for name in ["a", "b"] {
        let scan = dir.join(name);
        fs::create_dir_all(&scan).unwrap();
        room_bag_secs(&scan, true, 1.5);
        app.apply(Action::AddScan(scan));
    }
    app.state_mut().map_settings.step = MapStep::Clean;
    app.state_mut().surface_settings = quick(0.15);
    let mut h = Harness::builder()
        .with_size([1400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.state_mut().apply(Action::ComputeAllMaps);
    settle(&mut h);
    let b = map_of(&h, "b");
    h.state_mut().apply(Action::ComputeAllSurfaces);
    assert_eq!(h.state().state().queue.len(), 2);
    h.state_mut().apply(Action::Cancel);
    assert!(h.state().state().queue.is_empty());
    let seen = settle(&mut h);
    assert!(seen.len() <= 1, "{seen:?}");
    assert!(surfaces_of(&h, &b).is_empty());
}

#[test]
fn no_photos_is_advanced() {
    let mut h = panel(four_maps());
    assert!(h.query_by_label(text::SURFACE_NO_PHOTOS).is_none());
    click(&mut h, text::ADVANCED);
    h.get_by_label(text::SURFACE_NO_PHOTOS);
    assert!(h.state().0.surface_settings.texture);
    click(&mut h, text::SURFACE_NO_PHOTOS);
    assert!(!h.state().0.surface_settings.texture);
    // closed again: gone
    click(&mut h, text::ADVANCED);
    assert!(h.query_by_label(text::SURFACE_NO_PHOTOS).is_none());
}

#[test]
fn fineness_switch_sets_only_the_voxel() {
    let mut h = panel(four_maps());
    for (voxel, label) in text::SURFACE_VOXEL_CHOICES {
        click(&mut h, label);
        assert_eq!(h.state().0.surface_settings.config.voxel, voxel);
    }
    assert_eq!(
        SurfaceSettings {
            config: SurfaceConfig {
                voxel: SurfaceConfig::default().voxel,
                ..h.state().0.surface_settings.config.clone()
            },
            ..h.state().0.surface_settings.clone()
        },
        SurfaceSettings::default()
    );
    // 1 cm needs much memory
    click(&mut h, "1 cm");
    h.get_by_label(text::SURFACE_FINE_HINT);
    click(&mut h, "2 cm");
    assert!(h.query_by_label(text::SURFACE_FINE_HINT).is_none());
}

#[test]
fn planning_shows_placeholder() {
    let mut state = four_maps();
    state.step = Step::Planung;
    let h = panel(state);
    h.get_by_label(text::STEP_PLAN);
    h.get_by_label(text::PLAN_COMING);
    assert!(text::PLAN_COMING.contains("Grundriss"));
    assert!(text::PLAN_COMING.contains("Maße"));
    assert!(text::PLAN_COMING.contains("DWG"));
}

/// Writes a one-triangle glb to `path`.
fn small_glb(path: &Path) {
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    let mut tex = TexturedMesh::default();
    let positions: Vec<[f32; 3]> = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    tex.pages = vec![TexturePage {
        uvs: vec![[0.5; 2]; positions.len()],
        indices: vec![[0, 1, 2]],
        positions,
        image: RgbImage {
            width: 4,
            height: 4,
            data: vec![128; 48],
        },
    }];
    write_glb(path, &tex, &encode_pages(&tex, 90).unwrap()).unwrap();
}

/// The info messages of the status bar.
fn infos(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Info(i) => Some(i.clone()),
            _ => None,
        })
        .collect()
}

#[test]
fn maps_left_to_daten_are_named() {
    // only a (surfaced) and d (its map no longer fits): nothing to queue, but not „all done“
    let mut state = four_maps();
    state.scans = vec![state.scans[0].clone(), state.scans[3].clone()];
    assert!(state.maps_to_surface().is_empty());
    let mut h = panel(state.clone());
    let wait = text::maps_wait_for_data(1);
    h.get_by_label(&wait);
    assert!(h.query_by_label(text::ALL_SURFACED).is_none());
    assert_eq!(click(&mut h, text::COMPUTE_ALL_SURFACES), None);
    // the click says it too
    let mut app = app();
    *app.state_mut() = state;
    app.apply(Action::ComputeAllSurfaces);
    assert!(app.state().queue.is_empty());
    assert_eq!(infos(&app), std::slice::from_ref(&wait));
    assert!(!infos(&app).iter().any(|i| i == text::ALL_SURFACED));
    assert_eq!(
        text::maps_wait_for_data(3),
        "3 Karten erst in „Daten“ neu rechnen"
    );

    // with something to queue the skipped ones are named as well
    let state = four_maps();
    let h = panel(state);
    h.get_by_label(&wait);
}

#[test]
fn map_states_of_their_own() {
    let mut state = four_maps();
    // b's scan cannot be read, c's map has no map.json yet
    state.scans[1].state = ScanState::Missing("fehlt".into());
    state.scans[2].runs[0].maps[0].settings = None;
    let status: Vec<SurfaceStatus> = state
        .scans
        .iter()
        .map(|s| surface_status(&state, &s.runs[0].maps[0]))
        .collect();
    assert_eq!(
        status,
        [
            SurfaceStatus::Done,
            SurfaceStatus::ScanMissing,
            SurfaceStatus::MapNotReady,
            SurfaceStatus::MapStale
        ]
    );
    assert!(state.maps_to_surface().is_empty());
    let h = panel(state.clone());
    for (id, label) in [
        ("b", text::STATUS_SCAN_MISSING),
        ("c", text::STATUS_MAP_NOT_READY),
        ("d", text::STATUS_MAP_STALE),
    ] {
        let row = h.get_by_label(&map_name(id)).rect();
        let found = h
            .query_all_by_label(label)
            .any(|s| (s.rect().center().y - row.center().y).abs() < 6.0);
        assert!(found, "{label} not next to map {id}");
    }
    h.get_by_label(&text::maps_wait_for_data(3));
}

#[test]
fn stale_map_locks_its_surface() {
    let mut state = four_maps();
    state.selection = Selection::Map(MAP_D.into());
    let mut h = panel(state);
    h.get_by_label(text::STATUS_MAP_STALE_HINT);
    assert_eq!(click(&mut h, text::STEP_SURFACE), None);
    // a fitting map: free
    h.state_mut().0.selection = Selection::Map(MAP_B.into());
    h.run();
    assert!(h.query_by_label(text::STATUS_MAP_STALE_HINT).is_none());
    assert_eq!(
        click(&mut h, text::STEP_SURFACE),
        Some(Action::RunSurface(MAP_B.into()))
    );
}

/// Ticks like frames do until no step and no background work is left.
fn tick_settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(120);
    while app.wants_repaint_while_busy() {
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
        app.tick();
    }
}

/// An app with a new project in `dir` and the scans `names`, each with a cleaned map, quick
/// surface settings; returns the map folders.
fn mapped_project(dir: &Path, names: &[&str]) -> (DriftlessApp, Vec<PathBuf>) {
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    for name in names {
        let scan = dir.join(name);
        fs::create_dir_all(&scan).unwrap();
        room_bag_secs(&scan, true, 1.5);
        app.apply(Action::AddScan(scan));
    }
    app.state_mut().map_settings.step = MapStep::Clean;
    app.state_mut().surface_settings = quick(0.15);
    app.apply(Action::ComputeAllMaps);
    tick_settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    let maps = names
        .iter()
        .map(|id| {
            let node = app.state().scan_node(id).unwrap();
            node.runs.last().unwrap().maps.last().unwrap().dir.clone()
        })
        .collect();
    (app, maps)
}

fn has_surface(map: &Path) -> bool {
    map.join("oberflaeche-001").exists()
}

#[test]
fn closing_empties_the_surface_queue() {
    let dir = tmp("ui-bilder-schliessen");
    let (mut app, maps) = mapped_project(&dir, &["a", "b"]);
    app.apply(Action::ComputeAllSurfaces);
    assert_eq!(app.state().queue.len(), 2);
    assert!(matches!(app.state().queue[0], Queued::Surface(_)));
    assert!(!app.close_requested());
    assert!(app.state().queue.is_empty());
    tick_settle(&mut app);
    assert!(app.take_close());
    assert!(!has_surface(&maps[1]));
}

#[test]
fn project_switch_empties_the_surface_queue() {
    let dir = tmp("ui-bilder-wechsel");
    let (mut app, maps) = mapped_project(&dir, &["a", "b"]);
    app.apply(Action::ComputeAllSurfaces);
    assert_eq!(app.state().queue.len(), 2);
    app.apply(Action::NewProject {
        dir: dir.join("anderes"),
        name: "Anderes".into(),
    });
    assert!(app.state().queue.is_empty());
    tick_settle(&mut app);
    assert_eq!(app.state().project.as_ref().unwrap().name, "Anderes");
    assert!(app.state().queue.is_empty());
    assert!(app.state().running.is_none());
    assert!(!has_surface(&maps[1]));
}

#[test]
fn failed_surface_stops_the_queue() {
    let dir = tmp("ui-bilder-fehler");
    let (mut app, maps) = mapped_project(&dir, &["a", "b", "c"]);
    app.apply(Action::ComputeAllSurfaces);
    assert_eq!(app.state().queue.len(), 3);
    // b's scan folder goes while a's surface is computed
    fs::remove_dir_all(dir.join("b")).unwrap();
    tick_settle(&mut app);
    assert!(has_surface(&maps[0]));
    assert!(!errors(&app).is_empty(), "no error shown");
    assert!(app.state().queue.is_empty());
    assert!(!has_surface(&maps[1]));
    assert!(!has_surface(&maps[2]));
}

#[test]
fn queued_map_that_left_the_project_says_so() {
    let dir = tmp("ui-bilder-weg");
    let (mut app, maps) = mapped_project(&dir, &["a", "b"]);
    app.apply(Action::ComputeAllSurfaces);
    assert_eq!(app.state().queue.len(), 2);
    // b's map folder goes while a's surface is computed
    fs::remove_dir_all(&maps[1]).unwrap();
    tick_settle(&mut app);
    assert!(has_surface(&maps[0]));
    assert!(app.state().queue.is_empty());
    let errors = errors(&app);
    assert!(
        errors.iter().any(|e| e.contains(text::MAP_GONE)),
        "{errors:?}"
    );
}

/// A stale surface selected: its state („veraltet – Karte oder Lauf neu gerechnet“) wraps in
/// the field and does not widen it.
#[test]
fn stale_surface_does_not_widen_the_field() {
    let mut app = app();
    *app.state_mut() = four_maps();
    app.state_mut().selection = Selection::Surface(format!("{MAP_C}/oberflaeche-001").into());
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run();
    h.get_by_label(text::THIS_SURFACE);
    let width = h.state().layout().step.width();
    assert!(width <= 251.0, "the field grew to {width}");
}

/// Ticks until the step running is `want` (or nothing runs any more).
fn tick_until_running(app: &mut DriftlessApp, want: &Action) {
    let deadline = Instant::now() + Duration::from_secs(120);
    while app.state().running.as_ref() != Some(want) && app.wants_repaint_while_busy() {
        assert!(Instant::now() < deadline, "{want:?} did not start");
        std::thread::sleep(Duration::from_millis(5));
        app.tick();
    }
}

#[test]
fn queue_loads_only_its_last_result_and_keeps_the_step_view() {
    use driftless_app::render::ShowMode;
    let dir = tmp("ui-bilder-ansicht");
    let (mut app, maps) = mapped_project(&dir, &["a", "b"]);
    // the maps' queue showed only its last map
    let shown = app.state().cloud.as_ref().map(|c| c.path.clone());
    assert!(
        shown.as_ref().is_some_and(|p| p.starts_with(&maps[1])),
        "{shown:?}"
    );
    // „Alle Oberflächen rechnen“ in „Bilder“, then off to „Daten“ while it runs
    app.apply(Action::SetStep(Step::Bilder));
    app.apply(Action::Select(Selection::None));
    app.apply(Action::ComputeAllSurfaces);
    app.apply(Action::SetStep(Step::Daten));
    // the first surface done, the second running: nothing of the first was loaded
    tick_until_running(&mut app, &Action::RunSurface(maps[1].clone()));
    assert!(has_surface(&maps[0]));
    assert!(app.state().surface.is_none() && !app.is_loading_surface());
    assert_eq!(app.state().selection, Selection::None);
    tick_settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    // the last one is loaded, but „Daten“ keeps showing the points
    let surface = app.state().surface.as_ref().map(|s| s.path.clone());
    assert!(
        surface.as_ref().is_some_and(|p| p.starts_with(&maps[1])),
        "{surface:?}"
    );
    assert_eq!(app.state().view.show, ShowMode::Cloud);
    // in „Bilder“ a finished surface is shown
    app.apply(Action::SetStep(Step::Bilder));
    assert_eq!(app.state().view.show, ShowMode::Surface);
    app.state_mut().view.show = ShowMode::Cloud;
    app.state_mut().surface_settings = quick(0.2);
    app.apply(Action::Select(Selection::Map(maps[0].clone())));
    tick_settle(&mut app);
    app.apply(Action::RunSurface(maps[0].clone()));
    tick_settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    assert_eq!(app.state().view.show, ShowMode::Surface);
}

#[test]
fn queue_loads_nothing_after_a_load_by_hand() {
    let dir = tmp("ui-bilder-handlast");
    let (mut app, maps) = mapped_project(&dir, &["a", "b"]);
    app.apply(Action::SetStep(Step::Bilder));
    app.apply(Action::Select(Selection::None));
    app.apply(Action::ComputeAllSurfaces);
    // the user looks at map a meanwhile: the queue's result does not replace it
    app.apply(Action::Select(Selection::Map(maps[0].clone())));
    tick_settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    assert!(has_surface(&maps[0]) && has_surface(&maps[1]));
    assert_eq!(app.state().selection, Selection::Map(maps[0].clone()));
    assert!(app.state().surface.is_none());
}

const MODEL: &str = "/p/modelle/modell-001";

/// [`four_maps`] with „Modell 001“ of scans a and b (its report read from JSON, as on disk).
fn with_model(surfaces: Vec<SurfaceInfo>) -> AppState {
    let member = |id: &str| {
        serde_json::json!({
            "scan": format!("/daten/{id}"),
            "slam_dir": format!("/p/scans/{id}/slam-001"),
            "map_dir": format!("/p/scans/{id}/slam-001/karte-001"),
            "slam": SlamSettings::default(),
            "transform": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "placed_against": null,
            "method": "bezug",
            "verdict": "sicher",
            "reason": null,
            "overlap": 1.0,
            "residual_median_mm": 0.0,
            "residual_rms_mm": 0.0,
            "agreement": 1.0,
            "included": true,
            "color_gain": [1.0, 1.0, 1.0],
            "points": 10,
        })
    };
    let report: ModelReport = serde_json::from_value(serde_json::json!({
        "version": MERGE_VERSION,
        "input_id": "0",
        "members": [member("a"), member("b")],
        "points": 20,
        "seconds_total": 3.0,
        "peak_rss_bytes": 0,
    }))
    .unwrap();
    let mut state = four_maps();
    state.models = vec![ModelInfo {
        dir: MODEL.into(),
        label: "Modell 001".into(),
        report,
        surfaces,
    }];
    state
}

#[test]
fn model_card_computes_its_surface() {
    let name = text::model_card("Modell 001", 2);
    assert_eq!(name, "Modell 001 · 2 Scans");
    let state = with_model(Vec::new());
    let model = state.models[0].clone();
    assert_eq!(model_surface_status(&state, &model), SurfaceStatus::New);
    let mut h = panel(state);
    // selected with a click; the main button then computes its surface
    assert_eq!(
        click(&mut h, &name),
        Some(Action::Select(Selection::Model(MODEL.into())))
    );
    h.state_mut().0.selection = Selection::Model(MODEL.into());
    h.run();
    assert_eq!(
        click(&mut h, text::STEP_SURFACE),
        Some(Action::RunModelSurface {
            model: MODEL.into()
        })
    );
    // also with a surface of the model selected
    let surface = format!("{MODEL}/oberflaeche-001");
    let mut state = with_model(vec![surface_info(&surface)]);
    state.selection = Selection::Surface(surface.clone().into());
    let mut h = panel(state);
    assert_eq!(
        click(&mut h, text::STEP_SURFACE),
        Some(Action::RunModelSurface {
            model: MODEL.into()
        })
    );
}

fn surface_info(dir: &str) -> SurfaceInfo {
    surface(dir, "Oberfläche 001")
}

/// The hint of a surface that stays open in places, as the surface step writes it.
const OPEN: &str = "Stellenweise offen: An 23 % der dünn gemessenen Stellen lag kein Messpunkt \
                    näher als 1,5 cm. Dort bleibt die Oberfläche offen, statt etwas zu erfinden. \
                    Das ist in großen Räumen üblich. Schließen lassen sich solche Stellen nur \
                    durch dichteres Scannen (näher heran, langsamer), nicht durch eine feinere \
                    Einstellung.";

/// The hints of a surface for the user (`hinweise` of its report): short and muted below its
/// entry with the whole hint on hover, and in full under „Diese Oberfläche“ once it is
/// selected; for a surface of a map and of a model. A surface without hints shows none.
#[test]
fn surface_hints_are_shown() {
    let short = format!("{} {}", icon::HINT, text::surface_hint_short(OPEN));
    assert_eq!(
        text::surface_hint_short(OPEN),
        "Hinweis: Stellenweise offen"
    );
    let full = format!("{} {}: {OPEN}", icon::HINT, text::SURFACE_HINT);
    let hinted = format!("{MAP_A}/oberflaeche-001");
    let mut state = four_maps();
    state.scans[0].runs[0].maps[0].surfaces[0].hints = vec![OPEN.into()];
    let mut h = panel(state.clone());
    h.ctx.all_styles_mut(|s| s.interaction.tooltip_delay = 0.0);
    // only below the surface with the hint (three surfaces are listed)
    assert_eq!(h.get_all_by_label(&short).count(), 1);
    assert!(h.query_by_label(OPEN).is_none());
    h.get_by_label(&short).hover();
    h.run();
    h.get_by_label(OPEN);
    // selected: in full below its state
    state.selection = Selection::Surface(hinted.into());
    let h = panel(state.clone());
    h.get_by_label(text::THIS_SURFACE);
    h.get_by_label(&full);
    // another surface of the same map selected: not its hint
    state.selection = Selection::Surface(format!("{MAP_A}/oberflaeche-002").into());
    let h = panel(state);
    h.get_by_label(text::THIS_SURFACE);
    assert!(h.query_by_label(&full).is_none());

    // a surface of a model
    let model_surface = format!("{MODEL}/oberflaeche-001");
    let mut hinted = surface_info(&model_surface);
    hinted.hints = vec![OPEN.into()];
    let mut state = with_model(vec![hinted]);
    let h = panel(state.clone());
    h.get_by_label(&short);
    state.selection = Selection::Surface(model_surface.into());
    let h = panel(state);
    h.get_by_label(&full);
}

#[test]
fn model_surface_states() {
    let surface = format!("{MODEL}/oberflaeche-001");
    let status = |stale| MapStatus {
        seconds: Some(1.0),
        stale,
    };
    // a current surface: done
    let mut state = with_model(vec![surface_info(&surface)]);
    let model = state.models[0].clone();
    assert_eq!(model_surface_status(&state, &model), SurfaceStatus::Done);
    // it no longer fits (model_surface_fits): stale, marked in the list
    let mut map_status = state.map_status.clone();
    map_status.insert(surface.clone().into(), status(true));
    state.set_map_status(map_status.clone());
    let model = state.models[0].clone();
    assert!(model.surfaces[0].stale);
    assert_eq!(model_surface_status(&state, &model), SurfaceStatus::Stale);
    let h = panel(state.clone());
    h.get_by_label(&text::surface_stale("Oberfläche 001"));
    // being computed
    state.running = Some(Action::RunModelSurface {
        model: MODEL.into(),
    });
    assert!(matches!(
        model_surface_status(&state, &model),
        SurfaceStatus::Running(_)
    ));
    // the model itself no longer fits its maps: locked with the reason
    state.running = None;
    map_status.insert(MODEL.into(), status(true));
    state.set_map_status(map_status);
    let model = state.models[0].clone();
    assert_eq!(
        model_surface_status(&state, &model),
        SurfaceStatus::ModelStale
    );
    state.selection = Selection::Model(MODEL.into());
    let h = panel(state.clone());
    assert!(h
        .get_by_label(text::STEP_SURFACE)
        .accesskit_node()
        .is_disabled());
    h.get_by_label(text::STATUS_MODEL_STALE);
    h.get_by_label(text::STATUS_MODEL_STALE_HINT);
    // merged by another version of the program: the same state, its own reason
    state.models[0].report.version = MERGE_VERSION - 1;
    let h = panel(state);
    h.get_by_label(text::STATUS_MODEL_STALE);
    h.get_by_label(text::STATUS_MODEL_STALE_VERSION_HINT);
    assert!(h.query_by_label(text::STATUS_MODEL_STALE_HINT).is_none());
}
