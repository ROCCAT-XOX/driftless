//! How a SLAM run is drawn, shared by the debug viewer and the desktop program: the colour
//! ramp of the submap quality, the colours of the loops and which submap a pose belongs to.

use crate::error::{failed, StepError};
use driftless_slam::report::{LoopReport, SlamReport, SubmapReport};
use driftless_traj::{read_tum, Pose};
use std::path::Path;

/// Slack when comparing times read from `traj.tum` (9 decimals) with `slam.json`.
pub const TIME_TOLERANCE: f64 = 1e-6;

pub const LOOP_ACCEPTED: [u8; 3] = [0, 200, 0];
pub const LOOP_REJECTED: [u8; 3] = [140, 140, 140];
pub const LOOP_MANUAL: [u8; 3] = [40, 120, 255];
/// The odometry before loop closure: a hue of its own, apart from the quality ramp, the loop
/// colours and gray.
pub const ODOMETRY_COLOR: [u8; 3] = [110, 200, 230];

/// Whether a loop is a rejected candidate (not manual, not in the pose graph).
pub fn is_rejected(l: &LoopReport) -> bool {
    !l.manual && !l.accepted
}

/// Submap quality (its worst frame, `slam.json`) at and below which the trajectory is red:
/// barely held by the geometry, its place may have slipped (a featureless corridor, a door
/// turned through, a frame without a match).
pub const QUALITY_RED: f32 = 0.01;
/// The quality drawn yellow: halfway between red and green on the logarithmic scale.
pub const QUALITY_YELLOW: f32 = 0.05;
/// The quality at and above which the trajectory is green: well held.
///
/// The scale is the display of the quality only (the pose graph weighs the odometry by the
/// quality as before). It is set on the real runs of the analysis (28.10.): on the good office
/// run (median 0.21, 93 % of its submaps below 0.5) the linear red-to-green ramp showed more
/// than half of the trajectory red; with this scale at most a tenth of a good run is red and
/// most of it green (`tests::real_runs_are_mostly_green`). The quality is a product of factors
/// (overlap, constraint, residual) taken at the worst frame, so it spans decades: the scale is
/// logarithmic.
pub const QUALITY_GREEN: f32 = 0.25;

/// Where a submap quality lies on the colour ramp: 0 (red) at [`QUALITY_RED`] and below, 1
/// (green) at [`QUALITY_GREEN`] and above, logarithmic between them (yellow, 0.5, at
/// [`QUALITY_YELLOW`]); 0 for a quality that is no number.
pub fn quality_ramp(quality: f32) -> f32 {
    if quality.is_nan() || quality <= QUALITY_RED {
        return 0.0;
    }
    ((quality / QUALITY_RED).ln() / (QUALITY_GREEN / QUALITY_RED).ln()).clamp(0.0, 1.0)
}

/// Colour of a submap quality: red via yellow to green along [`quality_ramp`].
pub fn quality_color(quality: f32) -> [u8; 3] {
    let t = quality_ramp(quality);
    let ramp = |x: f32| (x * 255.0).round() as u8;
    if t < 0.5 {
        [255, ramp(t * 2.0), 0]
    } else {
        [ramp(2.0 - t * 2.0), 255, 0]
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

/// One group of pose indices per submap of `report` through the poses of `corrected` (see
/// [`submap_at`]), with the index of that submap (for its quality colour). Each group also
/// ends on the first pose of the next one, so the coloured line has no gaps. Submaps without
/// poses are skipped.
pub fn trajectory_groups(report: &SlamReport, corrected: &[Pose]) -> Vec<(Vec<usize>, usize)> {
    let submaps = &report.submaps;
    let t0 = report.scan_start;
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); submaps.len()];
    for (i, pose) in corrected.iter().enumerate() {
        if let Some(g) = groups.get_mut(submap_at(submaps, pose.t - t0)) {
            g.push(i);
        }
    }
    let mut out: Vec<(Vec<usize>, usize)> = Vec::new();
    for (submap, g) in groups.into_iter().enumerate() {
        let Some(&first) = g.first() else {
            continue;
        };
        if let Some((prev, _)) = out.last_mut() {
            prev.push(first);
        }
        out.push((g, submap));
    }
    out
}

/// One line strip per submap of `report` through the poses of `corrected`
/// ([`trajectory_groups`]), coloured by quality.
pub fn trajectory_strips(
    report: &SlamReport,
    corrected: &[Pose],
) -> (Vec<Vec<[f32; 3]>>, Vec<[u8; 3]>) {
    trajectory_groups(report, corrected)
        .into_iter()
        .map(|(g, submap)| {
            let strip = g.iter().map(|&i| xyz(&corrected[i])).collect();
            (strip, quality_color(report.submaps[submap].quality))
        })
        .unzip()
}

/// What a `driftless slam` output folder holds for drawing it: the report, the corrected
/// trajectory and the odometry.
#[derive(Debug, Clone)]
pub struct RunTrack {
    pub report: SlamReport,
    pub corrected: Vec<Pose>,
    pub odometry: Vec<Pose>,
}

/// Reads `slam.json`, `traj.tum` and `traj_odom.tum` of the run folder `dir`.
pub fn load_run(dir: &Path) -> Result<RunTrack, StepError> {
    let json_path = dir.join("slam.json");
    let text = std::fs::read_to_string(&json_path)
        .map_err(|e| format!("{} nicht lesbar: {e}", json_path.display()))?;
    let report: SlamReport = serde_json::from_str(&text).map_err(|e| {
        format!(
            "{} ist kein gültiger SLAM-Bericht: {e}; mit `driftless slam` neu erzeugen",
            json_path.display()
        )
    })?;
    Ok(RunTrack {
        report,
        corrected: read_tum(&dir.join("traj.tum")).map_err(failed)?,
        odometry: read_tum(&dir.join("traj_odom.tum")).map_err(failed)?,
    })
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
        // the same strips as pose indices
        let groups = trajectory_groups(&report, &corrected);
        assert_eq!(groups, [(vec![0, 1], 0), (vec![1, 2], 1)]);
        assert_eq!(colors, [quality_color(1.0), quality_color(0.0)]);
        assert_eq!(strips[0].len(), 2, "{strips:?}");
        assert_eq!(strips[1].len(), 2, "{strips:?}");
        assert_eq!(strips[1][0], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn quality_color_points() {
        // red at and below 0.01, yellow at 0.05, green at and above 0.25
        assert_eq!(quality_color(0.0), [255, 0, 0]);
        assert_eq!(quality_color(QUALITY_RED), [255, 0, 0]);
        assert_eq!(quality_color(QUALITY_YELLOW), [255, 255, 0]);
        assert_eq!(quality_color(QUALITY_GREEN), [0, 255, 0]);
        assert_eq!(quality_color(1.0), [0, 255, 0]);
        assert_eq!(quality_color(-3.0), [255, 0, 0]);
        assert_eq!(quality_color(f32::NAN), [255, 0, 0]);
        // yellow halfway on the logarithmic scale
        assert!((QUALITY_YELLOW - (QUALITY_RED * QUALITY_GREEN).sqrt()).abs() < 1e-6);
        // never darker going up: red rises first, then green falls
        let mut last = quality_ramp(0.0);
        for k in 0..=1000 {
            let r = quality_ramp(k as f32 / 1000.0);
            assert!(r >= last && (0.0..=1.0).contains(&r), "{k}");
            last = r;
        }
    }

    /// The submap qualities (worst frame, as `slam.json` stores them, sorted) of the real runs
    /// the analysis of 28.10. looked at, all good results: the office B (start↔end 2.16 →
    /// 0.13 m), the RayStudio test scan (1.8 cm ATE against RayStudio) and the black room
    /// S1–S3 (start↔end ≤ 2.5 cm). The ramp tells good from bad on them: at most a tenth of
    /// the submaps of a good run is red, most are green. The old linear ramp showed the office
    /// red to more than half.
    #[test]
    fn real_runs_are_mostly_green() {
        const OFFICE: [f32; 59] = [
            0.005, 0.005, 0.018, 0.019, 0.019, 0.025, 0.027, 0.028, 0.030, 0.031, 0.048, 0.050,
            0.051, 0.051, 0.053, 0.055, 0.058, 0.065, 0.066, 0.081, 0.092, 0.105, 0.112, 0.129,
            0.138, 0.153, 0.159, 0.171, 0.198, 0.206, 0.216, 0.228, 0.242, 0.254, 0.261, 0.267,
            0.279, 0.310, 0.316, 0.317, 0.323, 0.324, 0.326, 0.345, 0.347, 0.356, 0.363, 0.374,
            0.388, 0.395, 0.400, 0.417, 0.424, 0.468, 0.490, 0.575, 0.604, 0.608, 0.643,
        ];
        const TEST: [f32; 28] = [
            0.008, 0.013, 0.030, 0.034, 0.058, 0.064, 0.067, 0.071, 0.083, 0.093, 0.119, 0.129,
            0.152, 0.162, 0.162, 0.169, 0.275, 0.278, 0.291, 0.299, 0.306, 0.330, 0.331, 0.409,
            0.432, 0.473, 0.685, 0.697,
        ];
        const S1: [f32; 35] = [
            0.006, 0.026, 0.031, 0.091, 0.126, 0.154, 0.163, 0.173, 0.242, 0.274, 0.278, 0.341,
            0.414, 0.419, 0.420, 0.421, 0.469, 0.470, 0.483, 0.538, 0.539, 0.544, 0.565, 0.605,
            0.630, 0.652, 0.674, 0.746, 0.753, 0.820, 0.820, 0.886, 0.899, 0.900, 0.911,
        ];
        const S2: [f32; 26] = [
            0.085, 0.095, 0.105, 0.130, 0.216, 0.288, 0.327, 0.383, 0.474, 0.612, 0.669, 0.813,
            0.833, 0.834, 0.859, 0.867, 0.898, 0.898, 0.900, 0.901, 0.904, 0.907, 0.908, 0.908,
            0.922, 0.949,
        ];
        const S3: [f32; 27] = [
            0.016, 0.022, 0.044, 0.058, 0.062, 0.078, 0.088, 0.103, 0.116, 0.129, 0.134, 0.147,
            0.300, 0.490, 0.501, 0.535, 0.560, 0.592, 0.630, 0.660, 0.680, 0.707, 0.827, 0.877,
            0.880, 0.893, 0.919,
        ];
        // red: the first quarter of the ramp, green: the last quarter
        let shares = |q: &[f32], ramp: &dyn Fn(f32) -> f32| {
            let n = q.len() as f32;
            let red = q.iter().filter(|&&x| ramp(x) < 0.25).count() as f32 / n;
            let green = q.iter().filter(|&&x| ramp(x) >= 0.75).count() as f32 / n;
            (red, green)
        };
        for (name, q) in [
            ("Büro", &OFFICE[..]),
            ("Test", &TEST[..]),
            ("S1", &S1[..]),
            ("S2", &S2[..]),
            ("S3", &S3[..]),
        ] {
            let (red, green) = shares(q, &quality_ramp);
            assert!(red <= 0.1, "{name}: {:.0} % rot", red * 100.0);
            assert!(green >= 0.6, "{name}: {:.0} % grün", green * 100.0);
        }
        let (red, green) = shares(&OFFICE, &|q| q);
        assert!(
            red > 0.5 && green == 0.0,
            "old ramp: {red} red, {green} green"
        );
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
