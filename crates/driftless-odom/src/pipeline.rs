//! Frame-by-frame LiDAR-IMU odometry: IMU prediction, deskew, voxel downsampling,
//! point-to-plane ICP against the local voxel map, map update. No loop closure.

use crate::deskew::{deskew, frame_points};
use crate::icp::{align, IcpParams, IcpTarget};
use crate::imu_integrate::{
    estimate_initialisation, propagate, sanitise_imu, ImuCleanup, ImuModel, ImuState,
};
use crate::voxel_map::{voxel_downsample, voxel_key, FixedState, VoxelKey, VoxelMap};
use driftless_io::msg::{ImuSample, LidarFrame};
use driftless_traj::Pose;
use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use std::collections::HashSet;

/// Seconds of IMU data at the start used for gravity and gyro bias.
pub const INIT_WINDOW_S: f64 = 1.0;
/// Points kept per local-map voxel.
pub const MAP_POINTS_PER_VOXEL: usize = 20;
/// Share of the ICP position correction fed back into the velocity (0 = IMU only, 1 = all).
pub const VELOCITY_GAIN: f64 = 0.5;
/// Resolution of the output map in [`OdomResult::map_points`] (m).
pub const OUTPUT_MAP_VOXEL: f64 = 0.1;

#[derive(Debug, Clone, PartialEq)]
pub struct OdomConfig {
    /// Downsampling of each scan before registration (m).
    pub voxel_size: f64,
    /// Voxel size of the local map (m).
    pub map_voxel: f64,
    /// Points farther from the LiDAR are ignored; the local map keeps this radius (m).
    pub max_range: f64,
    /// Points closer to the LiDAR are ignored (operator, device) (m).
    pub min_range: f64,
    pub icp_max_iter: u32,
    /// LiDAR → IMU (body) extrinsic. Poses are reported for the body frame.
    pub lidar_to_imu: Isometry3<f64>,
    /// Motion-compensate each sweep with the IMU prediction. `false` treats all points as
    /// captured at the frame end (only for comparisons).
    pub deskew: bool,
}

impl Default for OdomConfig {
    fn default() -> Self {
        OdomConfig {
            voxel_size: 0.3,
            map_voxel: 0.5,
            max_range: 60.0,
            min_range: 0.5,
            icp_max_iter: 30,
            lidar_to_imu: Isometry3::identity(),
            deskew: true,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct OdomStats {
    pub frames_in: usize,
    /// Frames that produced a pose.
    pub frames_used: usize,
    /// Frames ignored: empty, or not later than the previous frame.
    pub frames_skipped: usize,
    pub points_time_skipped: usize,
    pub points_time_clamped: usize,
    pub imu: ImuCleanup,
    /// The first [`INIT_WINDOW_S`] of IMU looked static (see `estimate_initialisation`).
    pub static_start: bool,
    /// Frames where ICP found too few correspondences; the IMU prediction was kept.
    pub icp_failures: usize,
    pub icp_iterations: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct OdomResult {
    /// Body pose at the end of each used frame (world: gravity aligned, start = origin).
    pub poses: Vec<Pose>,
    /// Deskewed scan points in the world frame, one per [`OUTPUT_MAP_VOXEL`] voxel.
    pub map_points: Vec<[f32; 3]>,
    pub stats: OdomStats,
}

/// Global output map, deduplicated per voxel, in insertion order.
struct OutputMap {
    seen: HashSet<VoxelKey, FixedState>,
    points: Vec<[f32; 3]>,
}

impl OutputMap {
    fn add(&mut self, p: &Vector3<f64>) {
        if self.seen.insert(voxel_key(p, OUTPUT_MAP_VOXEL)) {
            self.points.push([p.x as f32, p.y as f32, p.z as f32]);
        }
    }
}

pub fn run_odometry(
    frames: impl Iterator<Item = LidarFrame>,
    imu: &[ImuSample],
    cfg: &OdomConfig,
) -> OdomResult {
    let (imu, cleanup) = sanitise_imu(imu);
    let init = estimate_initialisation(&imu, INIT_WINDOW_S);
    let mut stats = OdomStats {
        imu: cleanup,
        static_start: init.as_ref().is_some_and(|i| i.is_static),
        ..OdomStats::default()
    };
    let (start_rot, model) = match init {
        Some(i) => (i.rot, i.model),
        None => (
            UnitQuaternion::identity(),
            ImuModel {
                gravity: Vector3::new(0.0, 0.0, -9.81),
                gyro_bias: Vector3::zeros(),
                accel_bias: Vector3::zeros(),
            },
        ),
    };
    let icp_params = IcpParams {
        max_iter: cfg.icp_max_iter,
        max_corr_dist: 2.0 * cfg.map_voxel,
        ..IcpParams::default()
    };
    let lidar_origin = cfg.lidar_to_imu.translation.vector;
    let (min2, max2) = (cfg.min_range.powi(2), cfg.max_range.powi(2));

    let mut state: Option<ImuState> = None;
    let mut local_map = VoxelMap::new(cfg.map_voxel, MAP_POINTS_PER_VOXEL, 0.5 * cfg.voxel_size);
    let mut target: Option<IcpTarget> = None;
    let mut out_map = OutputMap {
        seen: HashSet::default(),
        points: Vec::new(),
    };
    let mut poses = Vec::new();

    for frame in frames {
        stats.frames_in += 1;
        let mut fp = frame_points(&frame, &cfg.lidar_to_imu);
        stats.points_time_skipped += fp.skipped;
        stats.points_time_clamped += fp.clamped;
        fp.points.retain(|tp| {
            let d2 = (tp.p - lidar_origin).norm_squared();
            d2 >= min2 && d2 <= max2
        });
        let prev_t = state.as_ref().map(|s| s.t);
        if fp.points.is_empty() || prev_t.is_some_and(|t| fp.t_end <= t) {
            stats.frames_skipped += 1;
            continue;
        }
        let start = state
            .take()
            .unwrap_or_else(|| ImuState::at_rest(fp.t_start, start_rot));
        let motion = propagate(&start, &imu, &model, fp.t_end);
        let pred = motion[motion.len() - 1].clone();
        let body = if cfg.deskew {
            deskew(&fp.points, &motion, fp.t_end)
        } else {
            deskew(&fp.points, &[], fp.t_end)
        };

        let pose = match &target {
            Some(tgt) => {
                let source = voxel_downsample(&body, cfg.voxel_size);
                let r = align(&source, tgt, &pred.pose(), &icp_params);
                stats.icp_iterations += u64::from(r.iterations);
                if r.correspondences < 6 {
                    stats.icp_failures += 1;
                }
                r.pose
            }
            None => pred.pose(),
        };

        // The ICP position correction over this frame mostly stems from a velocity error;
        // feed part of it back (a full correction amplifies ICP jitter into the prediction).
        let dt = fp.t_end - start.t;
        let vel = if prev_t.is_some() && dt > 1e-3 {
            pred.vel + (pose.translation.vector - pred.pos) * (VELOCITY_GAIN / dt)
        } else {
            pred.vel
        };
        state = Some(ImuState {
            t: fp.t_end,
            rot: pose.rotation,
            pos: pose.translation.vector,
            vel,
        });

        let world: Vec<Vector3<f64>> = voxel_downsample(&body, 0.5 * cfg.voxel_size)
            .iter()
            .map(|p| (pose * Point3::from(*p)).coords)
            .collect();
        local_map.insert(&world);
        local_map.remove_far(&pose.translation.vector, cfg.max_range);
        target = IcpTarget::new(local_map.points());
        for p in &body {
            out_map.add(&(pose * Point3::from(*p)).coords);
        }

        poses.push(Pose {
            t: fp.t_end,
            p: pose.translation.vector,
            q: pose.rotation,
        });
        stats.frames_used += 1;
    }
    OdomResult {
        poses,
        map_points: out_map.points,
        stats,
    }
}
