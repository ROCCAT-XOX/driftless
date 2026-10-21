//! A SLAM run drawn over the cloud: the trajectory per submap in the colour of its quality,
//! the odometry in gray, the loops between the submap anchors (accepted green, rejected
//! gray, manual blue), the anchors and the picked places as markers. Colours and the
//! assignment of poses to submaps are those of the debug viewer
//! ([`driftless_steps::overlay`]).
//!
//! Positions are relative to an origin (the cloud's, else the first pose in whole metres),
//! subtracted in f64 before the cast, like the cloud's own points.

use crate::lod::PointVertex;
use crate::pick::relative_to;
use crate::render::LineVertex;
use crate::ui::ViewSettings;
use driftless_slam::SlamReport;
use driftless_steps::overlay::{load_run, loop_color, quality_color, trajectory_groups};
use driftless_steps::StepError;
use driftless_traj::Pose;
use std::path::{Path, PathBuf};

/// Colour of the odometry line (as in the debug viewer).
pub const ODOMETRY_COLOR: [u8; 3] = [130, 130, 130];
/// Colour of a picked place.
pub const PICK_COLOR: [u8; 3] = [255, 255, 255];
/// Marker sizes in percent of the marker size of the view ([`crate::render::FrameParams`]).
pub const ANCHOR_SIZE: u32 = 50;
pub const PICK_SIZE: u32 = 100;

/// A run folder as the view draws it.
#[derive(Debug, Clone)]
pub struct RunOverlay {
    pub run: PathBuf,
    /// Absolute stamp of the start of the scan (s); times of the user are relative to it.
    pub scan_start: f64,
    /// The corrected trajectory (`traj.tum`).
    pub poses: Vec<Pose>,
    /// The odometry before loop closure (`traj_odom.tum`).
    pub odometry: Vec<Pose>,
    /// What the positions are relative to.
    pub origin: [f64; 3],
    /// The positions of [`RunOverlay::poses`] relative to [`RunOverlay::origin`] (for
    /// drawing and [`crate::pick::pick_pose`]).
    pub positions: Vec<[f32; 3]>,
    /// The same for [`RunOverlay::odometry`].
    pub odometry_positions: Vec<[f32; 3]>,
    pub report: SlamReport,
}

/// The first pose in whole metres: the origin without a cloud.
pub fn first_pose_origin(poses: &[Pose]) -> [f64; 3] {
    poses
        .first()
        .map_or([0.0; 3], |p| [p.p.x, p.p.y, p.p.z].map(f64::round))
}

fn packed(c: [u8; 3]) -> u32 {
    u32::from(c[0]) | u32::from(c[1]) << 8 | u32::from(c[2]) << 16
}

fn line_rgba(c: [u8; 3]) -> u32 {
    packed(c) | 0xff << 24
}

impl RunOverlay {
    /// Reads `slam.json`, `traj.tum` and `traj_odom.tum` of `run`; positions relative to
    /// `origin`, without one to [`first_pose_origin`].
    pub fn load(run: &Path, origin: Option<[f64; 3]>) -> Result<RunOverlay, StepError> {
        let track = load_run(run)?;
        let mut overlay = RunOverlay {
            run: run.to_path_buf(),
            scan_start: track.report.scan_start,
            poses: track.corrected,
            odometry: track.odometry,
            origin: [0.0; 3],
            positions: Vec::new(),
            odometry_positions: Vec::new(),
            report: track.report,
        };
        overlay.set_origin(origin);
        Ok(overlay)
    }

    /// Makes the positions relative to `origin` (the cloud's), without one to
    /// [`first_pose_origin`].
    pub fn set_origin(&mut self, origin: Option<[f64; 3]>) {
        self.origin = origin.unwrap_or_else(|| first_pose_origin(&self.poses));
        let rel = |poses: &[Pose]| {
            poses
                .iter()
                .map(|p| relative_to(self.origin, [p.p.x, p.p.y, p.p.z]))
                .collect()
        };
        self.positions = rel(&self.poses);
        self.odometry_positions = rel(&self.odometry);
    }

    /// Seconds since the start of the scan of pose `pose` (as `driftless correct` takes them).
    pub fn time_of(&self, pose: usize) -> f64 {
        self.poses[pose].t - self.scan_start
    }

    /// The pose nearest in time to `t` (seconds since the start of the scan); the earlier on
    /// a tie. `None` without poses.
    pub fn pose_at(&self, t: f64) -> Option<usize> {
        let abs = self.scan_start + t;
        let after = self.poses.partition_point(|p| p.t < abs);
        [after.checked_sub(1), Some(after)]
            .into_iter()
            .flatten()
            .filter(|&i| i < self.poses.len())
            .min_by(|&i, &j| {
                let d = |k: usize| (self.poses[k].t - abs).abs();
                d(i).total_cmp(&d(j))
            })
    }

    /// Bounds of the trajectory (relative); `None` without poses.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let first = *self.positions.first()?;
        Some(
            self.positions
                .iter()
                .fold((first, first), |(mut lo, mut hi), p| {
                    for k in 0..3 {
                        lo[k] = lo[k].min(p[k]);
                        hi[k] = hi[k].max(p[k]);
                    }
                    (lo, hi)
                }),
        )
    }

    fn anchor(&self, submap: usize) -> Option<[f32; 3]> {
        self.report
            .submaps
            .get(submap)
            .map(|m| relative_to(self.origin, m.anchor))
    }

    /// What the view draws of the run with the layers of `settings`: line segments (pairs of
    /// vertices) of the trajectory per submap in its quality colour, of the odometry and of
    /// the loops; markers of the submap anchors, then of the picked poses `picks` (large and
    /// white, always shown, unknown indices skipped).
    pub fn lines(
        &self,
        settings: &ViewSettings,
        picks: &[usize],
    ) -> (Vec<LineVertex>, Vec<PointVertex>) {
        let mut lines = Vec::new();
        let mut segment = |a: [f32; 3], b: [f32; 3], c: [u8; 3]| {
            let rgba = line_rgba(c);
            lines.push(LineVertex { pos: a, rgba });
            lines.push(LineVertex { pos: b, rgba });
        };
        if settings.show_trajectory {
            for (group, submap) in trajectory_groups(&self.report, &self.poses) {
                let c = quality_color(self.report.submaps[submap].quality);
                for w in group.windows(2) {
                    segment(self.positions[w[0]], self.positions[w[1]], c);
                }
            }
        }
        if settings.show_odometry {
            for w in self.odometry_positions.windows(2) {
                segment(w[0], w[1], ODOMETRY_COLOR);
            }
        }
        if settings.show_loops {
            for l in &self.report.loops {
                if let (Some(a), Some(b)) = (self.anchor(l.a), self.anchor(l.b)) {
                    segment(a, b, loop_color(l));
                }
            }
        }
        let mut markers = Vec::new();
        if settings.show_submaps {
            for (i, m) in self.report.submaps.iter().enumerate() {
                if let Some(pos) = self.anchor(i) {
                    markers.push(PointVertex {
                        pos,
                        rgbi: packed(quality_color(m.quality)),
                        extra: ANCHOR_SIZE,
                    });
                }
            }
        }
        for &i in picks {
            if let Some(&pos) = self.positions.get(i) {
                markers.push(PointVertex {
                    pos,
                    rgbi: packed(PICK_COLOR),
                    extra: PICK_SIZE,
                });
            }
        }
        (lines, markers)
    }
}
