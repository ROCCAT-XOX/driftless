//! The field „Planung“ and the plan view, without a window: a project with a map made by hand
//! (no raw data: a plan needs none), computed, shown, switched, exported and deleted.

use driftless_app::leftovers::LeftoverKind;
use driftless_app::project::{canonical, PlanInfo, PLAN_NUMBERED};
use driftless_app::ui::objects;
use driftless_app::ui::plan::{refresh_drawing, PlanMode};
use driftless_app::ui::plan_view::{bounds, PlanCamera};
use driftless_app::ui::{text, Action, Ask, DriftlessApp, Selection, Step, UiMessage};
use driftless_export::testutil::write_map_folder;
use driftless_io::testscan::tmp;
use driftless_plan::testutil::{Scene, TestData};
use driftless_steps::export::{ExportFormat, ExportRequest, ExportSource};
use driftless_steps::plan::{read_plan, TOP_VIEW_FILE};
use driftless_steps::plan_drawing::PlanPlacement;
use eframe::egui;
use egui_kittest::kittest::Queryable;
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

type H = Harness<'static, DriftlessApp>;

const FLOOR: f64 = -1.3;
const HEIGHT: f64 = 2.6;
const SPACING: f64 = 0.015;
const LOOP: [[f64; 3]; 4] = [
    [1.0, 1.0, 0.0],
    [4.0, 1.0, 0.0],
    [4.0, 3.0, 0.0],
    [1.0, 3.0, 0.0],
];
/// The trajectory written over the one of a map, to make its plan stale.
const OTHER_TRAJ: &str = "0.0 1.0 1.0 0.0 0.0 0.0 0.0 1.0\n0.1 1.5 1.0 0.0 0.0 0.0 0.0 1.0\n";

/// One room of 5 × 4 m and a walk around in it.
fn one_room() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        Scene::new()
            .spacing(SPACING)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, HEIGHT)
            .walk(&LOOP)
            .build()
    })
}

/// The same room twice, one storey above the other, with a walk on each.
fn two_storeys() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        let upper = LOOP.map(|p| [p[0], p[1], 2.9]);
        Scene::new()
            .spacing(SPACING)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, HEIGHT)
            .room([0.0, 0.0], [5.0, 4.0], 1.6, HEIGHT)
            .walk(&LOOP)
            .walk(&upper)
            .build()
    })
}

/// Four walls without a floor: no plan comes of it.
fn no_floor() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        let (low, high) = (FLOOR, FLOOR + HEIGHT);
        Scene::new()
            .spacing(SPACING)
            .behind([0.0, 0.0, low], [5.0, 0.0, high])
            .behind([0.0, 4.0, low], [5.0, 4.0, high])
            .behind([0.0, 0.0, low], [0.0, 4.0, high])
            .behind([5.0, 0.0, low], [5.0, 4.0, high])
            .walk(&LOOP)
            .build()
    })
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

/// Frames until no step and no background work is left.
fn settle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            break;
        }
        assert!(Instant::now() < deadline, "step did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
    h.step();
}

/// Applies `action` and runs frames until it ended, without an error.
fn run(h: &mut H, action: Action) {
    h.state_mut().apply(action.clone());
    settle(h);
    assert_eq!(errors(h.state()), Vec::<String>::new(), "{action:?}");
}

fn test_trash(path: &Path) -> Result<(), String> {
    static N: AtomicUsize = AtomicUsize::new(0);
    let trash =
        std::env::temp_dir().join(format!("driftless-plan-papierkorb-{}", std::process::id()));
    fs::create_dir_all(&trash).map_err(|e| e.to_string())?;
    let n = N.fetch_add(1, Ordering::SeqCst);
    fs::rename(path, trash.join(n.to_string())).map_err(|e| e.to_string())
}

/// The project in `dir` with the scan `halle` (its raw data do not exist), a run and a map
/// `karte-001` of `data`, as `write_map_folder` writes it. Returns the map folder.
fn write_project(dir: &Path, data: &TestData) -> PathBuf {
    fs::create_dir_all(dir).unwrap();
    let toml = "format = 1\nname = \"Halle\"\n\n[[scan]]\nid = \"halle\"\n\
                path = \"rohdaten/halle\"\nname = \"Halle\"\n";
    fs::write(dir.join("project.toml"), toml).unwrap();
    let run = dir.join("scans/halle/slam-001");
    let map = run.join("karte-001");
    write_map_folder(&map, &data.xyz, &data.poses);
    fs::write(run.join("slam.json"), "{}").unwrap();
    fs::copy(map.join("traj.tum"), run.join("traj.tum")).unwrap();
    map
}

/// The window on the project in `dir`, in „Planung“.
fn open(dir: &Path) -> H {
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_trash(test_trash);
    app.state_mut().leftover_quiet = Some(Duration::ZERO);
    app.apply(Action::OpenProject(dir.to_path_buf()));
    let mut h = Harness::builder()
        .with_size([1400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    settle(&mut h);
    run(&mut h, Action::SetStep(Step::Planung));
    h
}

/// The window on a new project in `dir/projekt` with a map of `data`, and the map folder as
/// the program knows it.
fn project_with_map(dir: &Path, data: &TestData) -> (H, PathBuf) {
    let project = dir.join("projekt");
    let map = write_project(&project, data);
    let h = open(&project);
    let map = canonical(&project)
        .unwrap()
        .join(map.strip_prefix(&project).unwrap());
    (h, map)
}

fn compute(h: &mut H, map: &Path) -> PathBuf {
    let owner = map.to_path_buf();
    run(
        h,
        Action::RunPlan {
            owner,
            storey: None,
        },
    );
    map.join("grundriss-001")
}

/// The numbered plan folders in `map`.
fn plan_folders(map: &Path) -> Vec<String> {
    let mut names: Vec<String> = fs::read_dir(map)
        .unwrap()
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with(PLAN_NUMBERED))
        .collect();
    names.sort();
    names
}

/// The plans of the map `map` in the tree.
fn plans_of(h: &H, map: &Path) -> Vec<PlanInfo> {
    h.state()
        .state()
        .map(map)
        .map(|(_, _, m)| m.plans.clone())
        .expect("map in the tree")
}

#[test]
fn compute_shows_the_plan() {
    let dir = tmp("plan-compute");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    assert!(plan.join("grundriss.json").is_file());
    assert_eq!(plan_folders(&map), ["grundriss-001"]);
    let state = h.state().state();
    assert_eq!(state.plan.mode, PlanMode::Plan);
    assert_eq!(state.selection, Selection::Plan(plan.clone()));
    let shown = state.plan.shown.as_ref().expect("shown");
    assert_eq!(shown.dir, plan);
    assert!(shown.top_view.is_some());
    let infos = plans_of(&h, &map);
    assert_eq!(infos.len(), 1);
    assert_eq!(infos[0].label, "Grundriss 001");
    assert!(!infos[0].stale);
    // the list of layers with their numbers, and the plan itself
    h.get_by_label("Raum (1)");
    h.get_by_label("Wand (4)");
    h.get_by_label("Raum 1");
    h.get_by_label(text::STATUS_DONE);
}

#[test]
fn second_compute_reuses() {
    let dir = tmp("plan-reuse");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    let written = fs::metadata(plan.join("grundriss.json"))
        .unwrap()
        .modified()
        .unwrap();
    compute(&mut h, &map);
    assert_eq!(plan_folders(&map), ["grundriss-001"]);
    assert_eq!(
        fs::metadata(plan.join("grundriss.json"))
            .unwrap()
            .modified()
            .unwrap(),
        written,
        "nothing written again"
    );
    let said = text::plan_fits(&plan);
    assert!(
        h.state()
            .state()
            .messages
            .iter()
            .any(|m| m.text().contains(&said)),
        "{:?}",
        h.state().state().messages
    );
}

#[test]
fn several_storeys_ask_and_pick() {
    let dir = tmp("plan-storeys");
    let (mut h, map) = project_with_map(&dir, two_storeys());
    let owner = map.clone();
    run(
        &mut h,
        Action::RunPlan {
            owner,
            storey: None,
        },
    );
    assert_eq!(plan_folders(&map), Vec::<String>::new());
    let storeys = {
        let plan = &h.state().state().plan;
        let (asked, storeys) = plan.storeys.clone().expect("asked");
        assert_eq!(asked, map);
        assert_eq!(storeys.len(), 2);
        storeys
    };
    h.get_by_label(text::PLAN_SEVERAL_STOREYS);
    let line = text::storey_line(&storeys[1]);
    assert!(line.starts_with("Boden "), "{line}");
    assert!(line.contains("der Laufbahn"), "{line}");
    h.get_by_label(&line).click();
    settle(&mut h);
    assert_eq!(errors(h.state()), Vec::<String>::new());
    let state = h.state().state();
    assert!(state.plan.storeys.is_none());
    let shown = state.plan.shown.as_ref().expect("shown");
    assert_eq!(shown.report.settings.storey, Some(2));
    assert_eq!(plans_of(&h, &map)[0].storey, Some(2));
    assert_eq!(plan_folders(&map), ["grundriss-001"]);
}

#[test]
fn stale_after_traj_changes() {
    let dir = tmp("plan-stale-traj");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    assert!(!plans_of(&h, &map)[0].stale);
    fs::write(map.join("traj.tum"), OTHER_TRAJ).unwrap();
    h.state_mut().state_mut().refresh();
    settle(&mut h);
    assert!(plans_of(&h, &map)[0].stale);
    h.get_by_label(text::STATUS_STALE);
}

#[test]
fn stale_map_makes_plan_stale() {
    let dir = tmp("plan-stale-map");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    let mut status = h.state().state().map_status.clone();
    status.get_mut(&map).expect("status of the map").stale = true;
    h.state_mut().state_mut().set_map_status(status);
    assert!(plans_of(&h, &map)[0].stale);
    h.step();
    h.get_by_label(text::STATUS_STALE);
}

#[test]
fn layer_off_hides_it_in_the_drawing() {
    let dir = tmp("plan-layer");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    h.get_by_label("Raum 1");
    h.get_by_label("Raum (1)").click();
    h.run();
    assert!(h.query_by_label("Raum 1").is_none());
    let state = h.state().state();
    assert!(state.plan.hidden.contains("Raum"));
    let layer = |name: &str, state: &driftless_app::ui::AppState| {
        let drawing = &state.plan.shown.as_ref().unwrap().drawing;
        drawing.layers.iter().find(|l| l.name == name).unwrap().on
    };
    assert!(!layer("Raum", state));
    assert!(layer("Wand", state));
    run(
        &mut h,
        Action::SetPlanLayer {
            layer: "Raum".into(),
            on: true,
        },
    );
    h.get_by_label("Raum 1");
}

#[test]
fn top_view_can_be_switched_off() {
    let dir = tmp("plan-top-view");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    assert!(h.state().state().plan.top_view);
    h.get_by_label(text::PLAN_TOP_VIEW_PICTURE);
    h.get_by_label(text::PLAN_TOP_VIEW).click();
    h.run();
    assert!(!h.state().state().plan.top_view);
    assert!(h.query_by_label(text::PLAN_TOP_VIEW_PICTURE).is_none());
    // the rest of the plan stays
    h.get_by_label("Raum 1");
}

#[test]
fn plan_without_top_view_file_still_shows() {
    let dir = tmp("plan-no-picture");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    fs::remove_file(plan.join(TOP_VIEW_FILE)).unwrap();
    run(&mut h, Action::ShowPlan(plan));
    let shown = h.state().state().plan.shown.clone().expect("shown");
    assert!(shown.top_view.is_none());
    h.get_by_label("Raum 1");
    assert!(h.query_by_label(text::PLAN_TOP_VIEW_PICTURE).is_none());
}

#[test]
fn zoom_keeps_the_point_under_the_cursor() {
    let size = [900.0, 640.0];
    let mut camera = PlanCamera::default();
    camera.fit([0.0, 0.0], [5000.0, 4000.0], size);
    let point = [3210.0, 1480.0];
    let cursor = camera.to_screen(point, size);
    let scale = camera.mm_per_point;
    camera.zoom_at(cursor, size, 3.0);
    assert!(camera.mm_per_point < scale, "zoomed in");
    let after = camera.to_screen(point, size);
    assert!((after[0] - cursor[0]).abs() < 0.5, "{after:?} / {cursor:?}");
    assert!((after[1] - cursor[1]).abs() < 0.5, "{after:?} / {cursor:?}");
    camera.zoom_at(cursor, size, -6.0);
    assert!(camera.mm_per_point > scale, "zoomed out");
    let after = camera.to_screen(point, size);
    assert!((after[0] - cursor[0]).abs() < 0.5, "{after:?} / {cursor:?}");
    assert!((after[1] - cursor[1]).abs() < 0.5, "{after:?} / {cursor:?}");
}

#[test]
fn f_fits_the_plan() {
    let dir = tmp("plan-fit");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    {
        let camera = &mut h.state_mut().state_mut().plan.camera;
        camera.centre_mm = [1.0e7, -3.0e6];
        camera.mm_per_point = 500.0;
    }
    h.step();
    h.key_press(egui::Key::F);
    h.step();
    let state = h.state().state();
    let shown = state.plan.shown.as_ref().expect("shown");
    let (min, max) = bounds(&shown.drawing);
    let view = h.state().layout().view;
    let size = [view.width(), view.height()];
    for p in [min, max, [min[0], max[1]], [max[0], min[1]]] {
        let at = state.plan.camera.to_screen(p, size);
        assert!(
            (0.0..=size[0]).contains(&at[0]),
            "{p:?} at {at:?}, view {size:?}"
        );
        assert!(
            (0.0..=size[1]).contains(&at[1]),
            "{p:?} at {at:?}, view {size:?}"
        );
    }
    // not lost in a corner: the plan fills most of the height or the width
    let (a, b) = (
        state.plan.camera.to_screen(min, size),
        state.plan.camera.to_screen(max, size),
    );
    let share = ((b[0] - a[0]).abs() / size[0]).max((a[1] - b[1]).abs() / size[1]);
    assert!(share > 0.7, "{share}");
}

#[test]
fn switch_to_3d_and_back() {
    let dir = tmp("plan-switch");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    h.get_by_label("Raum 1");
    h.get_by_label(text::PLAN_SWITCH_3D).click();
    settle(&mut h);
    assert_eq!(h.state().state().plan.mode, PlanMode::View3d);
    assert!(h.query_by_label("Raum 1").is_none());
    // the cloud of the map comes for the view
    assert!(h.state().state().cloud.is_some());
    h.get_by_label(text::PLAN_SWITCH_PLAN).click();
    h.run();
    assert_eq!(h.state().state().plan.mode, PlanMode::Plan);
    h.get_by_label("Raum 1");
    // Daten keeps its 3D view (the run of this project has no report the view could read: an
    // error of the fixture, so no `run` here)
    h.state_mut().apply(Action::SetStep(Step::Daten));
    settle(&mut h);
    assert!(h.query_by_label("Raum 1").is_none());
}

#[test]
fn switching_to_3d_brings_the_cloud_of_the_plan_s_map() {
    let dir = tmp("plan-zwei-karten");
    let project = dir.join("projekt");
    let first = write_project(&project, one_room());
    let other_map = first.parent().unwrap().join("karte-002");
    write_map_folder(&other_map, &one_room().xyz, &one_room().poses);
    let mut h = open(&project);
    let known = |p: &Path| {
        canonical(&project)
            .unwrap()
            .join(p.strip_prefix(&project).unwrap())
    };
    let (first, other_map) = (known(&first), known(&other_map));
    let plan = compute(&mut h, &first);

    // the view still shows the other map, the plan of the first one is on the screen
    run(
        &mut h,
        Action::LoadCloud {
            path: other_map.join("color.las"),
            run: None,
        },
    );
    assert_eq!(objects::view_owner(h.state().state()), Some(other_map));
    run(&mut h, Action::ShowPlan(plan));
    h.get_by_label(text::PLAN_SWITCH_3D).click();
    settle(&mut h);
    assert_eq!(h.state().state().plan.mode, PlanMode::View3d);
    assert_eq!(objects::view_owner(h.state().state()), Some(first.clone()));
    // the plan shown again keeps its map
    h.get_by_label(text::PLAN_SWITCH_PLAN).click();
    h.run();
    h.get_by_label(text::PLAN_SWITCH_3D).click();
    settle(&mut h);
    assert_eq!(objects::view_owner(h.state().state()), Some(first));
}

#[test]
fn dxf_button_opens_export_with_placement() {
    let dir = tmp("plan-dxf");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    h.get_by_label(text::PLAN_DXF).click();
    h.run();
    {
        let dialog = h.state().state().export.as_ref().expect("open");
        assert_eq!(dialog.format, ExportFormat::Dxf);
        assert_eq!(dialog.formats(), [ExportFormat::Dxf]);
        match dialog.source() {
            ExportSource::Plan { dir, export } => {
                assert_eq!(*dir, plan);
                assert_eq!(export.placement, PlanPlacement::Walls);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(dialog.choices[0].1, "Grundriss 001");
    }
    h.get_by_label(text::EXPORT_PLACEMENT_WALLS);
    h.get_by_label(text::EXPORT_PLACEMENT_CLOUD).click();
    h.run();
    match h.state().state().export.as_ref().expect("open").source() {
        ExportSource::Plan { export, .. } => {
            assert_eq!(export.placement, PlanPlacement::AsCloud)
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn hidden_layer_goes_into_the_export() {
    let dir = tmp("plan-hidden");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    run(
        &mut h,
        Action::SetPlanLayer {
            layer: "Raum".into(),
            on: false,
        },
    );
    run(&mut h, Action::OpenPlanExport);
    let source = {
        let dialog = h.state().state().export.as_ref().expect("open");
        match dialog.source() {
            ExportSource::Plan { export, .. } => {
                assert_eq!(export.hidden, ["Raum".to_string()])
            }
            other => panic!("{other:?}"),
        }
        dialog.source().clone()
    };
    let out = dir.join("plan.dxf");
    let request = ExportRequest {
        source,
        out: out.clone(),
        format: ExportFormat::Dxf,
        overwrite: false,
    };
    run(&mut h, Action::RunExport(request));
    let written = fs::read(&out).unwrap();
    let written = String::from_utf8_lossy(&written).replace("\r\n", "\n");
    let at = written.find("\nRaum\n").expect("layer Raum");
    let colour = written[at..].lines().skip(1).collect::<Vec<_>>().join(" ");
    assert!(
        colour.contains("62 -3"),
        "{}",
        &colour[..60.min(colour.len())]
    );
    // the walls stay on
    let at = written.find("\nWand\n").expect("layer Wand");
    let colour = written[at..].lines().skip(1).collect::<Vec<_>>().join(" ");
    assert!(
        colour.contains("62 7"),
        "{}",
        &colour[..60.min(colour.len())]
    );
}

#[test]
fn moved_project_plan_still_fits() {
    let dir = tmp("plan-moved");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    drop(h);
    let moved = dir.join("woanders");
    fs::rename(dir.join("projekt"), &moved).unwrap();
    let mut h = open(&moved);
    let moved = canonical(&moved).unwrap();
    let map = moved.join("scans/halle/slam-001/karte-001");
    assert!(!plans_of(&h, &map)[0].stale);
    // computed again: nothing to do, also without the raw data
    let plan_now = moved.join(
        plan.strip_prefix(canonical(&dir).unwrap().join("projekt"))
            .unwrap(),
    );
    compute(&mut h, &map);
    assert_eq!(plan_folders(&map), ["grundriss-001"]);
    assert_eq!(h.state().state().plan.shown.as_ref().unwrap().dir, plan_now);
}

#[test]
fn leftover_without_report_is_offered() {
    let dir = tmp("plan-leftover");
    let (mut h, map) = project_with_map(&dir, one_room());
    let cut_off = map.join("grundriss-002");
    fs::create_dir_all(&cut_off).unwrap();
    fs::write(cut_off.join(TOP_VIEW_FILE), [0u8; 300]).unwrap();
    h.state_mut().state_mut().refresh();
    settle(&mut h);
    let leftovers = h.state().state().leftovers.clone();
    assert_eq!(leftovers.len(), 1, "{leftovers:?}");
    assert_eq!(leftovers[0].kind, LeftoverKind::Plan);
    assert_eq!(leftovers[0].path, cut_off);
    assert_eq!(leftovers[0].label, "Grundriss 002 (Karte 001)");
    assert_eq!(leftovers[0].bytes, 300);
    // not listed as a plan of the map
    assert_eq!(plans_of(&h, &map), Vec::<PlanInfo>::new());
}

#[test]
fn finished_plan_is_no_leftover() {
    let dir = tmp("plan-no-leftover");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    h.state_mut().state_mut().refresh();
    settle(&mut h);
    assert_eq!(h.state().state().leftovers, Vec::new());
}

#[test]
fn delete_plan_asks() {
    let dir = tmp("plan-delete");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    run(&mut h, Action::AskDelete(plan.clone()));
    match h.state().state().ask.clone() {
        Some(Ask::Delete(q)) => {
            assert_eq!(q.dir, plan);
            assert_eq!(q.label, "Grundriss 001");
        }
        other => panic!("{other:?}"),
    }
    assert!(plan.is_dir(), "only asked");
    // shown: it has to leave the view first
    assert!(h.state().state().delete_shown(&plan).is_some());
    run(&mut h, Action::Unshow(plan.clone()));
    assert!(h.state().state().plan.shown.is_none());
    assert!(h.state().state().delete_shown(&plan).is_none());
    run(&mut h, Action::Delete(plan.clone()));
    assert!(!plan.exists());
    assert_eq!(plans_of(&h, &map), Vec::<PlanInfo>::new());
}

#[test]
fn a_map_with_the_plan_says_so_when_deleted() {
    let dir = tmp("plan-of-map");
    let (mut h, map) = project_with_map(&dir, one_room());
    compute(&mut h, &map);
    let question = h
        .state()
        .state()
        .delete_question(&map)
        .expect("in the tree");
    assert!(
        question
            .contains
            .iter()
            .any(|c| c.starts_with("Grundriss 001")),
        "{:?}",
        question.contains
    );
}

#[test]
fn plan_failure_says_why() {
    let dir = tmp("plan-failure");
    let (mut h, map) = project_with_map(&dir, no_floor());
    h.state_mut().apply(Action::RunPlan {
        owner: map.clone(),
        storey: None,
    });
    settle(&mut h);
    let errors = errors(h.state());
    assert_eq!(errors.len(), 1, "{errors:?}");
    assert!(errors[0].contains("Kein Boden"), "{errors:?}");
    assert_eq!(plan_folders(&map), Vec::<String>::new());
    assert!(h.state().state().plan.shown.is_none());
    h.get_by_label(text::NOT_COMPUTED);
}

#[test]
fn objects_of_the_owner_are_in_the_shown_plan() {
    use driftless_steps::objects::{ObjectInput, ObjectKind, Objects};
    let dir = tmp("plan-objects");
    let (mut h, map) = project_with_map(&dir, one_room());
    let plan = compute(&mut h, &map);
    Objects::change(&map, |o| {
        o.add(ObjectInput {
            kind: ObjectKind::Socket,
            points: vec![[1.0, 1.0, 0.3]],
            name: String::new(),
            note: String::new(),
            connection: None,
            diameter_mm: None,
            properties: Default::default(),
        })
    })
    .unwrap();
    run(&mut h, Action::ShowPlan(plan));
    let state = h.state().state();
    let shown = state.plan.shown.as_ref().expect("shown");
    assert_eq!(shown.objects.all().len(), 1);
    assert_eq!(read_plan(&shown.dir).unwrap().plan.rooms.len(), 1);
    h.get_by_label("Steckdose (1)");
    // the hook for the objects of task 10 after a change
    let plan = &mut h.state_mut().state_mut().plan;
    refresh_drawing(plan);
}
