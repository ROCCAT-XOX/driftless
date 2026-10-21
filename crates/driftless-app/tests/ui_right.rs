//! The steps and properties on the right, without a window: which steps a selection offers,
//! when they are locked and why, the state of each step, and the map settings.

use driftless_app::jobs::JobStatus;
use driftless_app::project::{MapInfo, MapStatus, Project, RunInfo, ScanEntry, ScanState};
use driftless_app::ui::{right, text, Action, AppState, InspectResult, ScanNode, Selection};
use driftless_io::progress::ProgressSnapshot;
use driftless_steps::history::{HistoryEntry, HistoryKind};
use driftless_steps::inspect::{Device, Report, TopicReport};
use driftless_steps::map::{MapSettings, MapStep, MapSwitches};
use driftless_steps::slam::SlamSettings;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::path::PathBuf;

const RUN: &str = "/p/scans/buero/slam-001";
const MAP: &str = "/p/scans/buero/slam-001/karte-001";

fn run(has_cache: bool, maps: Vec<MapInfo>) -> RunInfo {
    RunInfo {
        dir: PathBuf::from(RUN),
        label: "SLAM 001".into(),
        settings: SlamSettings::default(),
        has_result: true,
        has_cache,
        has_previous: false,
        has_odometry: true,
        cloud: None,
        maps,
        history: Vec::new(),
        history_gaps: Default::default(),
    }
}

fn map(switches: Option<MapSwitches>) -> MapInfo {
    MapInfo {
        dir: PathBuf::from(MAP),
        label: "Karte 001".into(),
        settings: switches,
        clouds: Vec::new(),
    }
}

/// A project with the scan „buero“ and the run `run`, `selection` selected.
fn state(run: RunInfo, selection: Selection) -> AppState {
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
            runs: vec![run],
        }],
        selection,
        ..AppState::default()
    }
}

fn scan_selected() -> AppState {
    state(run(true, Vec::new()), Selection::Scan("buero".into()))
}

fn run_selected(has_cache: bool) -> AppState {
    state(run(has_cache, Vec::new()), Selection::Run(RUN.into()))
}

type H = Harness<'static, (AppState, Option<Action>)>;

/// A harness that shows the right panel of `state` and keeps the last action.
fn harness(state: AppState) -> H {
    let mut h = Harness::builder()
        .with_size([360.0, 1400.0])
        .build_ui_state(
            |ui, (state, action): &mut (AppState, Option<Action>)| {
                if let Some(a) = right::show(ui, state) {
                    *action = Some(a);
                }
            },
            (state, None),
        );
    h.ctx.all_styles_mut(|s| s.interaction.tooltip_delay = 0.0);
    h.run();
    h
}

fn is_disabled(h: &H, label: &str) -> bool {
    h.get_by_label(label).accesskit_node().is_disabled()
}

/// Clicks `label` and returns the action it gave.
fn click(h: &mut H, label: &str) -> Option<Action> {
    h.state_mut().1 = None;
    h.get_by_label(label).click();
    h.run();
    h.state_mut().1.take()
}

/// Hovers `label` and checks that the tooltip `tip` shows.
fn tooltip(h: &mut H, label: &str, tip: &str) {
    h.get_by_label(label).hover();
    h.run();
    h.get_by_label(tip);
}

#[test]
fn scan_offers_inspect_and_slam() {
    let mut h = harness(state(
        run(true, Vec::new()),
        Selection::Scan("buero".into()),
    ));
    h.get_by_label(text::PANEL_STEPS);
    h.get_by_label(text::PANEL_PROPERTIES);
    // a run exists, but its time is unknown (no history)
    assert!(h.query_by_label(text::STEP_REUSE).is_none());
    assert!(h.query_by_label(text::STEP_MAP).is_none());
    assert_eq!(
        click(&mut h, text::STEP_INSPECT),
        Some(Action::Inspect("buero".into()))
    );
    assert_eq!(
        click(&mut h, text::STEP_SLAM),
        Some(Action::RunSlam("buero".into()))
    );
    // not inspected yet; SLAM done (a run with a result, no runtime known)
    h.get_by_label(text::NOT_COMPUTED);
    h.get_by_label(&text::step_done(None));
    // the SLAM settings of a new run can be changed
    h.get_by_label(text::SLAM_SETTINGS_NEW);
    click(&mut h, text::LIDAR_POSE);
    assert_eq!(
        h.state().0.slam_settings.lidar_to_imu,
        Some([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    );
    // a scan without runs: SLAM not computed
    let mut s = scan_selected();
    s.scans[0].runs.clear();
    let h = harness(s);
    assert_eq!(h.query_all_by_label(text::NOT_COMPUTED).count(), 2);
}

#[test]
fn run_offers_reuse_and_map() {
    let mut h = harness(run_selected(true));
    assert!(h.query_by_label(text::STEP_INSPECT).is_none());
    assert_eq!(
        click(&mut h, text::STEP_REUSE),
        Some(Action::Reuse(RUN.into()))
    );
    assert_eq!(
        click(&mut h, text::STEP_MAP),
        Some(Action::RunMap(RUN.into()))
    );
    // the run's SLAM settings are shown, not edited
    h.get_by_label(text::SLAM_SETTINGS_OF_RUN);
    h.get_by_label(text::LIDAR_POSE_NONE);
    assert!(h.query_by_label(text::LIDAR_POSE).is_none());
}

#[test]
fn reuse_disabled_without_cache_with_reason() {
    let mut h = harness(run_selected(false));
    assert!(is_disabled(&h, text::STEP_REUSE));
    assert!(!is_disabled(&h, text::STEP_MAP));
    assert_eq!(click(&mut h, text::STEP_REUSE), None);
    tooltip(&mut h, text::STEP_REUSE, text::NO_CACHE);
}

#[test]
fn steps_disabled_while_busy() {
    let busy = |mut s: AppState| {
        s.job = Some(JobStatus {
            label: text::STEP_SLAM.into(),
            progress: ProgressSnapshot::default(),
            seconds: 3.0,
            cancelling: false,
        });
        s
    };
    let reason = text::busy(text::STEP_SLAM);
    let mut h = harness(busy(scan_selected()));
    for step in [text::STEP_INSPECT, text::STEP_SLAM] {
        assert!(is_disabled(&h, step), "{step}");
        assert_eq!(click(&mut h, step), None);
    }
    tooltip(&mut h, text::STEP_SLAM, &reason);

    let mut h = harness(busy(run_selected(true)));
    for step in [text::STEP_REUSE, text::STEP_MAP] {
        assert!(is_disabled(&h, step), "{step}");
        assert_eq!(click(&mut h, step), None);
    }
    tooltip(&mut h, text::STEP_MAP, &reason);
    // the settings stay editable
    click(&mut h, text::GHOSTS);
    assert!(!h.state().0.map_settings.switches.ghosts);
}

#[test]
fn missing_scan_locks_its_steps() {
    let why = "Scan-Ordner /daten/buero nicht gefunden";
    let missing = |mut s: AppState| {
        s.scans[0].state = ScanState::Missing(why.into());
        s
    };
    let mut h = harness(missing(scan_selected()));
    for step in [text::STEP_INSPECT, text::STEP_SLAM] {
        assert!(is_disabled(&h, step), "{step}");
    }
    tooltip(&mut h, text::STEP_INSPECT, why);
    let h = harness(missing(run_selected(true)));
    for step in [text::STEP_REUSE, text::STEP_MAP] {
        assert!(is_disabled(&h, step), "{step}");
    }
}

#[test]
fn map_settings_edit_all_switches() {
    assert_eq!(AppState::default().map_settings, MapSettings::default());
    let mut h = harness(run_selected(true));
    for (label, step) in [
        (text::STEP_DENSE, MapStep::Dense),
        (text::STEP_CLEAN, MapStep::Clean),
        (text::STEP_COLOR, MapStep::Color),
        (text::STEP_ALL, MapStep::All),
    ] {
        click(&mut h, label);
        assert_eq!(h.state().0.map_settings.step, step, "{label}");
    }
    for label in [
        text::GHOSTS,
        text::OUTLIERS,
        text::THROUGH,
        text::SMOOTH,
        text::HIGHLIGHTS,
        text::VIGNETTING,
    ] {
        click(&mut h, label);
    }
    for (value, label) in text::DOWNSCALE_CHOICES.into_iter().rev() {
        click(&mut h, label);
        assert_eq!(
            h.state().0.map_settings.switches.downscale,
            value,
            "{label}"
        );
    }
    let d = MapSwitches::default();
    assert_eq!(
        h.state().0.map_settings,
        MapSettings {
            step: MapStep::All,
            switches: MapSwitches {
                ghosts: !d.ghosts,
                outliers: !d.outliers,
                through: !d.through,
                smooth: !d.smooth,
                highlights: !d.highlights,
                vignetting: !d.vignetting,
                voxel: d.voxel,
                downscale: 1,
            },
        }
    );
    // the voxel size is a number field
    h.get_by_label(text::VOXEL);
    // all switches back on their defaults: the defaults of `driftless map` again
    for label in [
        text::GHOSTS,
        text::OUTLIERS,
        text::THROUGH,
        text::SMOOTH,
        text::HIGHLIGHTS,
        text::VIGNETTING,
    ] {
        click(&mut h, label);
    }
    click(&mut h, text::DOWNSCALE_CHOICES[1].1);
    assert_eq!(h.state().0.map_settings, MapSettings::default());
}

#[test]
fn invalid_voxel_blocks_map() {
    let mut s = run_selected(true);
    s.map_settings.switches.voxel = 0.0;
    let mut h = harness(s);
    let message = "--voxel: Voxelgröße muss größer als 0 sein";
    h.get_by_label(message);
    assert!(is_disabled(&h, text::STEP_MAP));
    assert_eq!(click(&mut h, text::STEP_MAP), None);
    // reuse does not depend on the map settings
    assert!(!is_disabled(&h, text::STEP_REUSE));
    h.state_mut().0.map_settings.switches.voxel = 0.05;
    h.run();
    assert!(h.query_by_label(message).is_none());
    assert!(!is_disabled(&h, text::STEP_MAP));
}

#[test]
fn stale_map_is_marked() {
    let with_map = |status: MapStatus, switches: MapSwitches| {
        let mut s = state(
            run(true, vec![map(Some(switches))]),
            Selection::Run(RUN.into()),
        );
        s.map_status.insert(MAP.into(), status);
        s
    };
    let stale = MapStatus {
        seconds: Some(4.0),
        stale: true,
    };
    let fresh = MapStatus {
        seconds: Some(12.34),
        stale: false,
    };
    // the map of the current settings no longer fits the run
    let h = harness(with_map(stale.clone(), MapSwitches::default()));
    h.get_by_label(text::STALE);
    // a fitting map: done, with its runtime
    let h = harness(with_map(fresh.clone(), MapSwitches::default()));
    h.get_by_label("fertig (12,3 s)");
    assert!(h.query_by_label(text::STALE).is_none());
    // a map of other switches does not count for the current settings
    let other = MapSwitches {
        ghosts: false,
        ..MapSwitches::default()
    };
    let h = harness(with_map(stale.clone(), other));
    h.get_by_label(text::NOT_COMPUTED);
    assert!(h.query_by_label(text::STALE).is_none());
    // the map itself selected: its own state, whatever the settings
    let mut s = with_map(stale, other_switches());
    s.selection = Selection::Map(MAP.into());
    let h = harness(s);
    h.get_by_label(text::THIS_MAP);
    h.get_by_label(text::STALE);
    h.get_by_label(text::STEP_MAP);
}

fn other_switches() -> MapSwitches {
    MapSwitches {
        voxel: 0.2,
        ..MapSwitches::default()
    }
}

#[test]
fn inspect_report_as_table() {
    let mut s = scan_selected();
    s.inspect = Some(InspectResult {
        scan: "buero".into(),
        report: Report {
            device: Some(Device {
                sn: None,
                model: Some("Testgerät".into()),
            }),
            duration_s: 75.0,
            topics: vec![TopicReport {
                topic: "/lidar".into(),
                msg_type: "sensor_msgs/msg/PointCloud2".into(),
                count: 1234,
                rate_hz: Some(10.0),
                first: Some(0.0),
                last: Some(75.0),
                non_monotonic: 0,
                decode_errors: 0,
            }],
            dropped_points: 0,
            problems: vec!["IMU fehlt".into()],
        },
        seconds: 2.5,
    });
    let h = harness(s);
    h.get_by_label(text::INSPECT_RESULT);
    h.get_by_label("Testgerät");
    h.get_by_label("1:15 min");
    h.get_by_label("/lidar");
    h.get_by_label("1.234");
    h.get_by_label("10,0 Hz");
    h.get_by_label("IMU fehlt");
    h.get_by_label("fertig (2,5 s)");
    // another scan's report is not shown
    let mut s = h.state().0.clone();
    s.inspect.as_mut().unwrap().scan = "andere".into();
    let h = harness(s);
    assert!(h.query_by_label(text::INSPECT_RESULT).is_none());
}

#[test]
fn nothing_selected_asks_for_a_selection() {
    let h = harness(state(run(true, Vec::new()), Selection::None));
    h.get_by_label(text::SELECT_FOR_STEPS);
    assert!(h.query_by_label(text::STEP_SLAM).is_none());
}

#[test]
fn reuse_row_shows_last_reuse() {
    let h = harness(run_selected(true));
    h.get_by_label(text::NOT_REUSED);
    let entry = |nr, kind, seconds| HistoryEntry {
        nr,
        dir: PathBuf::from(format!("{RUN}/verlauf/{nr:03}")),
        kind,
        created: "2026-10-21T10:00:00Z".into(),
        seconds,
    };
    let mut s = run_selected(true);
    s.scans[0].runs[0].history = vec![
        entry(1, HistoryKind::Full, 30.0),
        entry(2, HistoryKind::Reuse, 1.26),
    ];
    let h = harness(s);
    h.get_by_label("fertig (1,3 s)");
    assert!(h.query_by_label(text::NOT_REUSED).is_none());
}

#[test]
fn scan_without_images_locks_color_step() {
    let why = "keine Kamerabilder: /daten/buero enthält keine IMAGE_*.bag";
    let mut s = run_selected(true);
    s.scans[0].no_color = Some(why.into());
    let mut h = harness(s);
    // „Alles“ (the default) stays possible and says that colouring is skipped
    h.get_by_label(&text::color_skipped(why));
    assert!(!is_disabled(&h, text::STEP_MAP));
    // „Einfärben“ is locked with the reason
    assert!(is_disabled(&h, text::STEP_COLOR));
    click(&mut h, text::STEP_COLOR);
    assert_eq!(h.state().0.map_settings.step, MapStep::All);
    tooltip(&mut h, text::STEP_COLOR, why);
    // dense and clean need no images
    click(&mut h, text::STEP_CLEAN);
    assert_eq!(h.state().0.map_settings.step, MapStep::Clean);
    assert!(h.query_by_label(&text::color_skipped(why)).is_none());
    // a step „Einfärben“ chosen before: the map button is locked
    h.state_mut().0.map_settings.step = MapStep::Color;
    h.run();
    assert!(is_disabled(&h, text::STEP_MAP));
    tooltip(&mut h, text::STEP_MAP, why);
}

#[test]
fn selected_map_shows_its_switches_as_template() {
    let recorded = MapSwitches {
        ghosts: false,
        voxel: 0.2,
        downscale: 4,
        ..MapSwitches::default()
    };
    let mut s = state(
        run(true, vec![map(Some(recorded.clone()))]),
        Selection::Map(MAP.into()),
    );
    s.map_settings.step = MapStep::Clean;
    let mut h = harness(s);
    h.get_by_label(text::MAP_SWITCHES);
    h.get_by_label(&text::switch_value(text::GHOSTS, false));
    h.get_by_label(&text::switch_value(text::OUTLIERS, true));
    h.get_by_label("Voxelgröße (m): 0,200");
    h.get_by_label("Bilder verkleinern: 1/4");
    assert_eq!(h.state().0.map_settings.switches, MapSwitches::default());
    click(&mut h, text::USE_AS_TEMPLATE);
    assert_eq!(h.state().0.map_settings.switches, recorded);
    assert_eq!(h.state().0.map_settings.step, MapStep::Clean, "step stays");
    // a map without readable map.json
    let s = state(run(true, vec![map(None)]), Selection::Map(MAP.into()));
    let h = harness(s);
    h.get_by_label(text::MAP_SWITCHES_UNKNOWN);
    assert!(h.query_by_label(text::USE_AS_TEMPLATE).is_none());
}
