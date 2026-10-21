//! How a SLAM run is drawn, shared by the debug viewer and the desktop program: the colour
//! ramp of the submap quality, the colours of the loops and which submap a pose belongs to.

use driftless_slam::report::{LoopReport, SlamReport, SubmapReport};
use driftless_traj::Pose;

/// Slack when comparing times read from `traj.tum` (9 decimals) with `slam.json`.
pub const TIME_TOLERANCE: f64 = 1e-6;

pub const LOOP_ACCEPTED: [u8; 3] = [0, 200, 0];
pub const LOOP_REJECTED: [u8; 3] = [140, 140, 140];
pub const LOOP_MANUAL: [u8; 3] = [40, 120, 255];

/// Colour of a submap quality: red (0) via yellow (0.5) to green (1).
pub fn quality_color(quality: f32) -> [u8; 3] {
    let q = if quality.is_finite() {
        quality.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let ramp = |x: f32| (x * 255.0).round() as u8;
    if q < 0.5 {
        [255, ramp(q * 2.0), 0]
    } else {
        [ramp(2.0 - q * 2.0), 255, 0]
    }
}

/// Colour of a loop line: manual, else accepted or rejected.
pub fn loop_color(l: &LoopReport) -> [u8; 3] {
    if l.manual {
        LOOP_MANUAL
    } else if l.accepted {
        LOOP_ACCEPTED
    } else {
        LOOP_REJECTED
    }
}

/// Index of the submap a time `t` (seconds since the start of the scan) belongs to: the last
/// one that started at or before it (within [`TIME_TOLERANCE`]); 0 before the first.
pub fn submap_at(submaps: &[SubmapReport], t: f64) -> usize {
    submaps
        .partition_point(|m| m.t_start <= t + TIME_TOLERANCE)
        .saturating_sub(1)
}

fn xyz(p: &Pose) -> [f32; 3] {
    [p.p.x as f32, p.p.y as f32, p.p.z as f32]
}

/// One line strip per submap of `report` through the poses of `corrected` (see
/// [`submap_at`]), coloured by quality. Each strip also ends on the first point of the next
/// one, so the coloured line has no gaps. Submaps without poses are skipped.
pub fn trajectory_strips(
    report: &SlamReport,
    corrected: &[Pose],
) -> (Vec<Vec<[f32; 3]>>, Vec<[u8; 3]>) {
    let submaps = &report.submaps;
    let t0 = report.scan_start;
    let mut groups: Vec<Vec<[f32; 3]>> = vec![Vec::new(); submaps.len()];
    for pose in corrected {
        if let Some(g) = groups.get_mut(submap_at(submaps, pose.t - t0)) {
            g.push(xyz(pose));
        }
    }
    let mut strips: Vec<Vec<[f32; 3]>> = Vec::new();
    let mut colors = Vec::new();
    let mut last: Option<usize> = None;
    for (i, g) in groups.iter().enumerate() {
        if g.is_empty() {
            continue;
        }
        if let Some(prev) = last {
            strips[prev].push(g[0]);
        }
        strips.push(g.clone());
        colors.push(quality_color(submaps[i].quality));
        last = Some(strips.len() - 1);
    }
    (strips, colors)
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_slam::report::BeforeAfter;
    use driftless_traj::nalgebra::{UnitQuaternion, Vector3};

    /// A pose on a submap start, rounded below it by the 9 decimals of `traj.tum`, belongs
    /// to that submap.
    #[test]
    fn trajectory_strips_tolerate_rounded_times() {
        let sm = |id: usize, t_start: f64, quality: f32| SubmapReport {
            id,
            t_start,
            t_end: t_start + 0.5,
            quality,
            anchor: [0.0; 3],
            odom_anchor: [0.0; 3],
        };
        let none = BeforeAfter {
            before: 0.0,
            after: 0.0,
        };
        let scan_start = 1000.25;
        let pose = |t: f64, x: f64| Pose {
            t,
            p: Vector3::new(x, 0.0, 0.0),
            q: UnitQuaternion::identity(),
        };
        let report = SlamReport {
            scan_start,
            submaps: vec![sm(0, 0.0, 1.0), sm(1, 0.6, 0.0)],
            loops: Vec::new(),
            graph: None,
            start_end_gap: none.clone(),
            path_length: none,
        };
        // the second pose is the first frame of submap 1, written 0.4 ns early
        let corrected = vec![
            pose(scan_start, 0.0),
            pose(scan_start + 0.6 - 4e-10, 1.0),
            pose(scan_start + 1.0, 2.0),
        ];
        let (strips, colors) = trajectory_strips(&report, &corrected);
        assert_eq!(colors, [quality_color(1.0), quality_color(0.0)]);
        assert_eq!(strips[0].len(), 2, "{strips:?}");
        assert_eq!(strips[1].len(), 2, "{strips:?}");
        assert_eq!(strips[1][0], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn quality_color_endpoints() {
        assert_eq!(quality_color(0.0), [255, 0, 0]);
        assert_eq!(quality_color(0.5), [255, 255, 0]);
        assert_eq!(quality_color(1.0), [0, 255, 0]);
        assert_eq!(quality_color(-3.0), [255, 0, 0]);
        assert_eq!(quality_color(f32::NAN), [255, 0, 0]);
    }

    #[test]
    fn loop_colors() {
        let l = |manual, accepted| LoopReport {
            a: 0,
            b: 1,
            manual,
            accepted,
            reason: None,
            inlier_ratio: 0.0,
            rms: 0.0,
        };
        assert_eq!(loop_color(&l(true, false)), LOOP_MANUAL);
        assert_eq!(loop_color(&l(false, true)), LOOP_ACCEPTED);
        assert_eq!(loop_color(&l(false, false)), LOOP_REJECTED);
    }
}
