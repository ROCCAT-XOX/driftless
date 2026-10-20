//! `corrections.toml` and how `solve` checks and uses manual loops.

use driftless_odom::FrameQuality;
use driftless_slam::{
    close_loops, solve, Corrections, LoopCheck, LoopVerdict, ManualLoop, ManualLoopError,
    SlamConfig, Submap, SubmapFrame, POSITION_ONLY,
};
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::fs;

#[test]
fn corrections_append_and_disable_keep_history() {
    let dir = std::env::temp_dir().join(format!("driftless-corr-{}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    let path = dir.join("corrections.toml");

    let mut c = Corrections::load(&path).unwrap();
    assert!(c.loops.is_empty(), "missing file = no corrections");
    assert_eq!(c.add_loop(12.3, 250.1, "Treppe", "2026-10-20T10:00:00Z"), 1);
    assert_eq!(c.add_loop(5.0, 99.5, "", "2026-10-20T10:01:00Z"), 2);
    c.save(&path).unwrap();
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.contains("[[loop]]"), "{text}");
    assert!(text.contains("a = 12.3"), "{text}");
    assert!(text.contains("note = \"Treppe\""), "{text}");

    let mut c = Corrections::load(&path).unwrap();
    assert!(c.set_enabled(1, false).unwrap());
    assert!(!c.set_enabled(1, false).unwrap(), "already disabled");
    c.save(&path).unwrap();

    let c = Corrections::load(&path).unwrap();
    assert_eq!(c.loops.len(), 2, "disable keeps the entry");
    assert!(!c.loops[0].enabled);
    assert_eq!(
        (c.loops[0].a, c.loops[0].b, c.loops[0].note.as_str()),
        (12.3, 250.1, "Treppe")
    );
    assert_eq!(c.loops[0].added, "2026-10-20T10:00:00Z");
    assert_eq!(
        c.manual_loops(),
        vec![ManualLoop {
            a_t: 5.0,
            b_t: 99.5
        }]
    );

    let mut c = c;
    assert!(c.set_enabled(1, true).unwrap());
    assert_eq!(c.manual_loops().len(), 2);
    let e = c.set_enabled(3, false).unwrap_err();
    assert!(e.contains("kein Eintrag 3"), "{e}");
    assert!(c.set_enabled(0, false).is_err());
}

fn planar(x: f64, y: f64, yaw: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, y, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
    )
}

/// `n` submaps of 10 frames each at 10 Hz, starting at t = 100 s, along a straight line,
/// without points (every ICP fails).
fn line(n: usize) -> (Vec<Submap>, Vec<Isometry3<f64>>) {
    let submaps: Vec<Submap> = (0..n)
        .map(|id| Submap {
            id,
            anchor: planar(id as f64, 0.0, 0.0),
            frames: (0..10)
                .map(|i| SubmapFrame {
                    index: id * 10 + i,
                    t: 100.0 + (id * 10 + i) as f64 * 0.1,
                    offset: planar(i as f64 * 0.1, 0.0, 0.0),
                    quality: FrameQuality::first(),
                })
                .collect(),
            points: Vec::new(),
        })
        .collect();
    let anchors = submaps.iter().map(|m| m.anchor).collect();
    (submaps, anchors)
}

fn manual(a_t: f64, b_t: f64) -> ManualLoop {
    ManualLoop { a_t, b_t }
}

#[test]
fn manual_loop_time_outside_scan_errors() {
    let (s, a) = line(4);
    // the scan runs from 0 to 3.9 s
    for t in [-0.1, 3.95, f64::NAN, f64::INFINITY] {
        let e = solve(
            s.clone(),
            a.clone(),
            &[manual(0.5, t)],
            &SlamConfig::default(),
        )
        .unwrap_err();
        assert!(
            matches!(e, ManualLoopError::OutsideScan { .. }),
            "{t}: {e:?}"
        );
        assert!(e.to_string().contains("außerhalb des Scans"), "{e}");
    }
    assert!(solve(s, a, &[manual(0.0, 3.9)], &SlamConfig::default()).is_ok());
}

#[test]
fn manual_loop_same_submap_errors() {
    let (s, a) = line(4);
    let e = solve(s, a, &[manual(1.0, 1.9)], &SlamConfig::default()).unwrap_err();
    assert_eq!(
        e,
        ManualLoopError::SameSubmap {
            a_t: 1.0,
            b_t: 1.9,
            submap: 1
        }
    );
    assert!(e.to_string().contains("derselben Teilkarte 1"), "{e}");
}

#[test]
fn duplicate_manual_loop_errors() {
    let (s, a) = line(4);
    // the second loop joins the same submaps 0 and 3 (reversed and at other frames)
    let e = solve(
        s,
        a,
        &[manual(0.2, 3.5), manual(3.1, 0.8)],
        &SlamConfig::default(),
    )
    .unwrap_err();
    assert!(
        matches!(e, ManualLoopError::Duplicate { a: 0, b: 3, .. }),
        "{e:?}"
    );
    assert!(e.to_string().contains("doppelte Schleife"), "{e}");
}

/// Without points the ICP of a manual loop fails: the loop then only pulls the two marked
/// positions together (with less weight than the odometry) and is reported as manual.
#[test]
fn manual_loop_without_icp_keeps_position_only() {
    let (s, a) = line(4);
    // mark frame 0 (t = 0.0) and the last frame of submap 3 (t = 3.9 s, 3.9 m further)
    let r = solve(s, a, &[manual(0.0, 3.9)], &SlamConfig::default()).unwrap();
    assert_eq!(r.loops.len(), 1);
    let l = &r.loops[0];
    assert!(l.manual);
    assert!(matches!(l.verdict, LoopVerdict::PositionOnly(_)), "{l:?}");
    assert!(r.graph.is_some());
    let poses = r.poses();
    let gap = (poses[0].p - poses[poses.len() - 1].p).norm();
    assert!(gap < 0.9 * 3.9, "marked positions still {gap} m apart");
    let rep = driftless_slam::SlamReport::new(&r);
    assert!(rep.loops[0].manual && rep.loops[0].accepted);
    assert!(rep.loops[0]
        .reason
        .as_deref()
        .unwrap()
        .starts_with(POSITION_ONLY));
}

/// A manual loop the odometry contradicts stays in the graph: the consistency check only
/// rejects automatic loops.
#[test]
fn manual_loop_is_never_rejected_as_inconsistent() {
    let odom: Vec<_> = (0..10).map(|i| planar(i as f64, 0.0, 0.0)).collect();
    let quality = vec![1.0; 9];
    let check = |manual| LoopCheck {
        a: 0,
        b: 9,
        relative: planar(2.0, 0.0, 0.0),
        inlier_ratio: 1.0,
        rms: 0.0,
        degeneracy: 0.2,
        verdict: LoopVerdict::Accepted,
        manual,
    };
    let mut auto = vec![check(false)];
    close_loops(&odom, &quality, &mut auto);
    assert_eq!(
        auto[0].verdict,
        LoopVerdict::Rejected("widerspricht anderen Schleifen")
    );

    let mut by_hand = vec![check(true)];
    let (anchors, report) = close_loops(&odom, &quality, &mut by_hand);
    assert_eq!(by_hand[0].verdict, LoopVerdict::Accepted);
    assert!(report.is_some());
    let d = (anchors[9].translation.vector - anchors[0].translation.vector).norm();
    assert!(d < 5.0, "manual loop did not pull: {d} m");
}
