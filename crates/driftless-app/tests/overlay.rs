//! The run drawn over the cloud: trajectory per submap in its quality colour, odometry,
//! loops between the anchors, anchors and picked places as markers; times of picked poses.

use driftless_app::camera::{Camera, CameraMode};
use driftless_app::lod::PointVertex;
use driftless_app::overlay::{
    RunOverlay, ANCHOR_SIZE, LOOP_WIDTH, ODOMETRY_WIDTH, PICK_COLOR, PICK_RING, PICK_RING_SIZE,
    PICK_SIZE, REJECTED_ALPHA, TRAJECTORY_WIDTH,
};
use driftless_app::pick::pick_pose;
use driftless_app::render::LineVertex;
use driftless_app::ui::ViewSettings;
use driftless_io::testscan::tmp;
use driftless_slam::report::{BeforeAfter, LoopReport, SubmapReport};
use driftless_slam::SlamReport;
use driftless_steps::overlay::{
    quality_color, LOOP_ACCEPTED, LOOP_MANUAL, LOOP_REJECTED, ODOMETRY_COLOR,
};
use driftless_traj::nalgebra::{UnitQuaternion, Vector3};
use driftless_traj::{write_tum, Pose};
use std::fs;
use std::path::{Path, PathBuf};

const SCAN_START: f64 = 1_700_000_000.5;
/// A large offset like UTM coordinates.
const OFFSET: [f64; 3] = [500_000.25, 5_400_000.75, 110.0];

fn pose(t: f64, x: f64) -> Pose {
    Pose {
        t: SCAN_START + t,
        p: Vector3::new(OFFSET[0] + x, OFFSET[1], OFFSET[2]),
        q: UnitQuaternion::identity(),
    }
}

fn submap(id: usize, t_start: f64, quality: f32, x: f64) -> SubmapReport {
    SubmapReport {
        id,
        t_start,
        t_end: t_start + 2.9,
        quality,
        anchor: [OFFSET[0] + x, OFFSET[1], OFFSET[2]],
        odom_anchor: [OFFSET[0] + x, OFFSET[1] + 0.5, OFFSET[2]],
    }
}

fn loop_report(a: usize, b: usize, manual: bool, accepted: bool) -> LoopReport {
    LoopReport {
        a,
        b,
        manual,
        accepted,
        reason: None,
        inlier_ratio: 0.5,
        rms: 0.05,
    }
}

/// A run folder: 3 submaps of 3 s with quality 0, 0.5 and 1, one pose per second along x,
/// and three loops (accepted, rejected, manual).
fn run_dir(name: &str) -> PathBuf {
    let dir = tmp(name);
    let report = SlamReport {
        scan_start: SCAN_START,
        submaps: vec![
            submap(0, 0.0, 0.0, 0.0),
            submap(1, 3.0, 0.5, 3.0),
            submap(2, 6.0, 1.0, 6.0),
        ],
        loops: vec![
            loop_report(0, 2, false, true),
            loop_report(0, 1, false, false),
            loop_report(1, 2, true, true),
        ],
        graph: None,
        start_end_gap: BeforeAfter {
            before: 0.0,
            after: 0.0,
        },
        path_length: BeforeAfter {
            before: 0.0,
            after: 0.0,
        },
    };
    fs::write(
        dir.join("slam.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    let poses: Vec<Pose> = (0..9).map(|i| pose(f64::from(i), f64::from(i))).collect();
    write_tum(&dir.join("traj.tum"), &poses).unwrap();
    let odom: Vec<Pose> = (0..9)
        .map(|i| pose(f64::from(i), f64::from(i) * 1.1))
        .collect();
    write_tum(&dir.join("traj_odom.tum"), &odom).unwrap();
    dir
}

fn only(trajectory: bool, odometry: bool, loops: bool, submaps: bool) -> ViewSettings {
    ViewSettings {
        show_trajectory: trajectory,
        show_odometry: odometry,
        show_loops: loops,
        show_submaps: submaps,
        ..ViewSettings::default()
    }
}

fn rgb(v: u32) -> [u8; 3] {
    [v as u8, (v >> 8) as u8, (v >> 16) as u8]
}

/// The segments as (start, end, colour).
fn segments(lines: &[LineVertex]) -> Vec<([f32; 3], [f32; 3], [u8; 3])> {
    lines
        .chunks(2)
        .map(|s| {
            assert_eq!(s[0].rgba, s[1].rgba, "one colour per segment");
            (s[0].pos, s[1].pos, rgb(s[0].rgba))
        })
        .collect()
}

fn load(dir: &Path) -> RunOverlay {
    RunOverlay::load(dir, Some(OFFSET)).unwrap()
}

#[test]
fn strips_follow_submaps_and_quality() {
    let o = load(&run_dir("overlay-strips"));
    let (lines, markers) = o.lines(&only(true, false, false, false), &[]);
    assert!(markers.is_empty());
    let segs = segments(&lines);
    // poses 0..=3 red, 3..=6 yellow, 6..=8 green: 8 segments
    let colors: Vec<[u8; 3]> = segs.iter().map(|s| s.2).collect();
    let (red, yellow, green) = (quality_color(0.0), quality_color(0.5), quality_color(1.0));
    assert_eq!(
        colors,
        [red, red, red, yellow, yellow, yellow, green, green]
    );
    assert_eq!(
        (red, yellow, green),
        ([255, 0, 0], [255, 255, 0], [0, 255, 0])
    );
    // no gap: each segment starts where the one before ended, from pose 0 to pose 8
    for w in segs.windows(2) {
        assert_eq!(w[0].1, w[1].0);
    }
    assert!(lines.iter().all(|v| v.width == TRAJECTORY_WIDTH));
    assert_eq!(segs[0].0, [0.0, 0.0, 0.0]);
    assert_eq!(segs[7].1, [8.0, 0.0, 0.0]);

    // the odometry alone: gray, its own poses
    let (lines, _) = o.lines(&only(false, true, false, false), &[]);
    let segs = segments(&lines);
    assert_eq!(segs.len(), 8);
    assert!(segs.iter().all(|s| s.2 == ODOMETRY_COLOR));
    assert!(lines.iter().all(|v| v.width == ODOMETRY_WIDTH));
    assert!((segs[7].1[0] - 8.8).abs() < 1e-5, "{:?}", segs[7]);
    // nothing switched on: nothing drawn
    let (lines, markers) = o.lines(&only(false, false, false, false), &[]);
    assert!(lines.is_empty() && markers.is_empty());
}

#[test]
fn loops_colored_by_kind() {
    let o = load(&run_dir("overlay-loops"));
    let (lines, markers) = o.lines(&only(false, false, true, false), &[]);
    let segs = segments(&lines);
    let anchor = |x: f32| [x, 0.0, 0.0];
    // „Schleifen“: the accepted and the manual ones, opaque
    assert_eq!(
        segs,
        [
            (anchor(0.0), anchor(6.0), LOOP_ACCEPTED),
            (anchor(3.0), anchor(6.0), LOOP_MANUAL),
        ]
    );
    assert!(lines
        .iter()
        .all(|v| v.rgba >> 24 == 0xff && v.width == LOOP_WIDTH));
    assert!(markers.is_empty());
    // the rejected candidates on their own layer, faint and thin
    let settings = ViewSettings {
        show_rejected: true,
        ..only(false, false, false, false)
    };
    assert!(!ViewSettings::default().show_rejected, "off by default");
    let (lines, _) = o.lines(&settings, &[]);
    assert_eq!(
        segments(&lines),
        [(anchor(0.0), anchor(3.0), LOOP_REJECTED)]
    );
    assert_eq!(lines[0].rgba >> 24, u32::from(REJECTED_ALPHA));
    assert!(lines[0].width < LOOP_WIDTH);
    // the submaps: their anchors as markers in their quality colour
    let (lines, markers) = o.lines(&only(false, false, false, true), &[]);
    assert!(lines.is_empty());
    let shown: Vec<([f32; 3], [u8; 3], u32)> = markers
        .iter()
        .map(|m| (m.pos, rgb(m.rgbi), m.extra))
        .collect();
    assert_eq!(
        shown,
        [
            (anchor(0.0), quality_color(0.0), ANCHOR_SIZE),
            (anchor(3.0), quality_color(0.5), ANCHOR_SIZE),
            (anchor(6.0), quality_color(1.0), ANCHOR_SIZE),
        ]
    );
}

#[test]
fn picked_places_are_large_ringed_markers_on_top() {
    let o = load(&run_dir("overlay-picks"));
    let (_, markers) = o.lines(&only(true, false, false, true), &[2, 7]);
    // anchors first, the picked places last (drawn over them): a dark ring, then the colour
    assert_eq!(markers.len(), 3 + 4);
    let picks: Vec<&PointVertex> = markers[3..].iter().collect();
    let shown: Vec<([f32; 3], [u8; 3], u32)> = picks
        .iter()
        .map(|m| (m.pos, rgb(m.rgbi), m.extra))
        .collect();
    let at = |x: f32| [x, 0.0, 0.0];
    assert_eq!(
        shown,
        [
            (at(2.0), PICK_RING, PICK_RING_SIZE),
            (at(2.0), PICK_COLOR, PICK_SIZE),
            (at(7.0), PICK_RING, PICK_RING_SIZE),
            (at(7.0), PICK_COLOR, PICK_SIZE),
        ]
    );
    const { assert!(PICK_RING_SIZE > PICK_SIZE && PICK_SIZE > ANCHOR_SIZE) };
    // a colour of its own: not white (walls), not the loop or quality colours
    assert_ne!(PICK_COLOR, [255, 255, 255]);
    assert!(![LOOP_ACCEPTED, LOOP_MANUAL, LOOP_REJECTED, ODOMETRY_COLOR].contains(&PICK_COLOR));
    // a pick also shows with every layer off; an unknown index is skipped
    let (_, markers) = o.lines(&only(false, false, false, false), &[4, 99]);
    assert_eq!(markers.len(), 2);
}

#[test]
fn time_of_pick_is_since_scan_start() {
    let o = load(&run_dir("overlay-time"));
    assert_eq!(o.scan_start, SCAN_START);
    assert_eq!(o.time_of(0), 0.0);
    assert!((o.time_of(5) - 5.0).abs() < 1e-6, "{}", o.time_of(5));
    // and back: the pose nearest to a time
    assert_eq!(o.pose_at(5.2), Some(5));
    assert_eq!(o.pose_at(-3.0), Some(0));
    assert_eq!(o.pose_at(100.0), Some(8));
}

#[test]
fn overlay_without_cloud_uses_first_pose_origin() {
    let dir = run_dir("overlay-origin");
    let o = RunOverlay::load(&dir, None).unwrap();
    // the first pose in whole metres
    assert_eq!(o.origin, [500_000.0, 5_400_001.0, 110.0]);
    assert_eq!(o.positions[0], [0.25, -0.25, 0.0]);
    assert_eq!(o.positions[8], [8.25, -0.25, 0.0]);
    // a cloud arrives: its origin from now on, millimetres kept
    let mut o = o;
    o.set_origin(Some(OFFSET));
    assert_eq!(o.origin, OFFSET);
    assert_eq!(o.positions[3], [3.0, 0.0, 0.0]);
    let (min, max) = o.bounds().unwrap();
    assert_eq!((min, max), ([0.0, 0.0, 0.0], [8.0, 0.0, 0.0]));
}

/// Poses at UTM coordinates are picked on the screen in the frame of the cloud.
#[test]
fn pick_pose_at_large_offset() {
    let o = load(&run_dir("overlay-pick"));
    // looking along −y at pose 4 (x = 4) from 10 m away
    let cam = Camera {
        mode: CameraMode::Fly,
        eye: [4.0, 10.0, 0.0].into(),
        yaw: -std::f32::consts::FRAC_PI_2,
        pitch: 0.0,
        ..Camera::default()
    };
    let viewport = [400.0, 300.0];
    let vp = cam.view_proj(viewport[0] / viewport[1]);
    let hit = pick_pose(&o.positions, &vp, viewport, [200.0, 150.0], 6.0);
    assert_eq!(hit, Some(4));
}

/// At a crossing of the trajectory the place B prefers a pose of another submap than A.
#[test]
fn pick_at_a_crossing_prefers_another_submap() {
    let mut o = load(&run_dir("overlay-kreuzung"));
    // pose 7 (submap 2) lies where pose 1 (submap 0) is
    o.positions[7] = o.positions[1];
    assert_eq!((o.submap_of(1), o.submap_of(7)), (0, 2));
    let cam = Camera {
        mode: CameraMode::Fly,
        eye: [1.0, 10.0, 0.0].into(),
        yaw: -std::f32::consts::FRAC_PI_2,
        pitch: 0.0,
        ..Camera::default()
    };
    let viewport = [400.0, 300.0];
    let vp = cam.view_proj(viewport[0] / viewport[1]);
    let centre = [200.0, 150.0];
    assert_eq!(o.pick(&vp, viewport, centre, 6.0, None), Some(1));
    assert_eq!(o.pick(&vp, viewport, centre, 6.0, Some(0)), Some(7));
    // nothing of another submap within reach: the nearest one all the same
    assert_eq!(o.pick(&vp, viewport, centre, 6.0, Some(1)), Some(1));
    assert_eq!(o.pick(&vp, viewport, [5.0, 5.0], 6.0, Some(0)), None);
}

#[test]
fn missing_run_files_are_a_german_error() {
    let dir = tmp("overlay-leer");
    let err = RunOverlay::load(&dir, None).unwrap_err().to_string();
    assert!(err.contains("slam.json nicht lesbar"), "{err}");
}
