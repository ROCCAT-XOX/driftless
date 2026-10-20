//! Dense cloud: the odometry's deskewed sweeps placed with the corrected trajectory.
//!
//! Per frame, placing the points is a pure per-point map (pose times point, range to the
//! LiDAR; run in parallel, order kept) followed by a sequential reduction into the voxel grid
//! in point order, so the result is identical in every run.

use crate::{Cloud, MapError};
use driftless_io::msg::{ImuSample, LidarFrame};
use driftless_odom::voxel_map::{voxel_key, FixedState, VoxelKey};
use driftless_odom::{run_odometry_with, FrameOutput, OdomConfig, OdomStats};
use driftless_traj::Pose;
use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use rayon::prelude::*;
use std::collections::hash_map::{Entry, HashMap};

/// Largest difference between a corrected pose's time and its frame's end time (s). TUM files
/// keep nine decimals, so anything larger means another scan or other settings.
pub const POSE_TIME_TOLERANCE: f64 = 1e-6;

#[derive(Debug, Clone, PartialEq)]
pub struct DenseConfig {
    /// Edge of the voxels of which each keeps one point (m).
    pub voxel: f64,
    /// Points farther from the LiDAR are left out (m).
    pub max_range: f64,
    /// Points closer to the LiDAR are left out (operator, device) (m).
    pub min_range: f64,
    /// Every `ray_step`-th point in range of each frame goes into the [`RaySource`], starting
    /// at point `frame index mod ray_step`: the start rotates from frame to frame so that a
    /// step sharing a factor with the ring count does not leave out the same rings in every
    /// frame.
    pub ray_step: usize,
}

impl Default for DenseConfig {
    fn default() -> Self {
        DenseConfig {
            voxel: 0.02,
            max_range: 30.0,
            min_range: 0.5,
            ray_step: 4,
        }
    }
}

impl DenseConfig {
    fn check(&self) -> Result<(), MapError> {
        let bad = |msg: &str| Err(MapError::Config(msg.into()));
        if !(self.voxel.is_finite() && self.voxel > 0.0) {
            return bad("Voxelgröße muss größer als 0 sein");
        }
        if !(self.min_range >= 0.0 && self.max_range.is_finite() && self.min_range < self.max_range)
        {
            return bad("Mindestabstand muss zwischen 0 und dem Höchstabstand liegen");
        }
        if self.ray_step == 0 {
            return bad("Strahlabstand (ray_step) muss mindestens 1 sein");
        }
        Ok(())
    }
}

/// Sensor rays for free-space checks: a subset of each frame's points (after the range
/// filter) with the LiDAR position they were seen from, all in world coordinates.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RaySource {
    /// LiDAR origin per used frame, at the frame end (the time the sweep is deskewed to;
    /// an approximation for points captured earlier in the sweep).
    pub sensor: Vec<[f32; 3]>,
    /// Ray end points, frame by frame.
    pub ends: Vec<[f32; 3]>,
    /// Index into `ends` where each frame starts; one more entry than frames.
    pub frame_start: Vec<u32>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct DenseStats {
    /// Frames placed (= poses of the trajectory).
    pub frames: usize,
    /// Points within [`DenseConfig::min_range`]..[`DenseConfig::max_range`].
    pub points_in_range: u64,
    /// Points in the cloud. A nearer point replaces the one in its voxel, nothing is ever
    /// dropped, so this is also the most the cloud held at any time.
    pub points: usize,
    pub rays: usize,
    pub odom: OdomStats,
}

fn to_f32(v: &Vector3<f64>) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

/// A body point in the world and its distance to the LiDAR. The sweep is deskewed to the
/// frame end, so the distance is taken from the LiDAR position at the frame end (an
/// approximation: the point was seen from wherever the LiDAR was at its capture time).
struct Placed {
    world: Vector3<f64>,
    range: f32,
}

/// Places body point `p` with the body pose `pose`; `None` if its squared distance to the
/// LiDAR origin `origin` (body frame) lies outside `[min2, max2]`.
fn place(
    pose: &Isometry3<f64>,
    origin: &Vector3<f64>,
    p: &Vector3<f64>,
    min2: f64,
    max2: f64,
) -> Option<Placed> {
    let d2 = (p - origin).norm_squared();
    (d2 >= min2 && d2 <= max2).then(|| Placed {
        world: (pose * Point3::from(*p)).coords,
        range: d2.sqrt() as f32,
    })
}

/// Cloud with one point per voxel: the nearest seen so far; on a tie the earlier one stays.
#[derive(Default)]
struct VoxelCloud {
    voxel: f64,
    slot: HashMap<VoxelKey, u32, FixedState>,
    range: Vec<f32>,
    cloud: Cloud,
}

impl VoxelCloud {
    fn add(&mut self, p: &Placed, intensity: f32, t: f64, frame: u32) {
        let xyz = to_f32(&p.world);
        match self.slot.entry(voxel_key(&p.world, self.voxel)) {
            Entry::Occupied(e) => {
                let i = *e.get() as usize;
                if p.range < self.range[i] {
                    self.range[i] = p.range;
                    let c = &mut self.cloud;
                    (c.xyz[i], c.intensity[i], c.t[i], c.frame[i]) = (xyz, intensity, t, frame);
                }
            }
            Entry::Vacant(e) => {
                let i = u32::try_from(self.range.len()).expect("fewer than 2^32 points");
                e.insert(i);
                self.range.push(p.range);
                let c = &mut self.cloud;
                c.xyz.push(xyz);
                c.intensity.push(intensity);
                c.t.push(t);
                c.frame.push(frame);
            }
        }
    }
}

/// Dense cloud of a scan: runs the odometry over `frames` again (same frames, `imu` and
/// `odom_cfg` as the SLAM run) and places every deskewed sweep with its corrected body pose
/// `corrected[index]` (one per used frame, e.g. the SLAM `traj.tum`). Each point keeps the
/// intensity of its LiDAR return and the frame's end time. Also returns every
/// [`DenseConfig::ray_step`]-th point per frame (rotating start) as [`RaySource`].
///
/// Fails if `corrected` does not belong to these frames (count or times differ).
pub fn build_dense(
    frames: impl Iterator<Item = LidarFrame>,
    imu: &[ImuSample],
    odom_cfg: &OdomConfig,
    corrected: &[Pose],
    cfg: &DenseConfig,
) -> Result<(Cloud, RaySource, DenseStats), MapError> {
    cfg.check()?;
    let origin = odom_cfg.lidar_to_imu.translation.vector;
    let (min2, max2) = (cfg.min_range.powi(2), cfg.max_range.powi(2));
    let mut grid = VoxelCloud {
        voxel: cfg.voxel,
        ..VoxelCloud::default()
    };
    let mut rays = RaySource {
        frame_start: vec![0],
        ..RaySource::default()
    };
    let mut points_in_range = 0u64;
    let mut time_error: Option<MapError> = None;

    let mut sink = |f: FrameOutput| {
        if time_error.is_some() {
            return;
        }
        let Some(c) = corrected.get(f.index) else {
            return; // count mismatch, reported below
        };
        if (c.t - f.pose.t).abs() > POSE_TIME_TOLERANCE {
            time_error = Some(MapError::PoseTime {
                index: f.index,
                pose_t: c.t,
                frame_t: f.pose.t,
            });
            return;
        }
        let pose = Isometry3::from_parts(Translation3::from(c.p), c.q);
        let frame = u32::try_from(f.index).expect("fewer than 2^32 frames");
        let placed: Vec<Option<Placed>> = f
            .body_points
            .par_iter()
            .map(|p| place(&pose, &origin, p, min2, max2))
            .collect();
        rays.sensor
            .push(to_f32(&(pose * Point3::from(origin)).coords));
        let first_ray = f.index % cfg.ray_step;
        let mut k = 0usize;
        for (p, &i) in placed.iter().zip(f.intensity) {
            let Some(p) = p else { continue };
            if k % cfg.ray_step == first_ray {
                rays.ends.push(to_f32(&p.world));
            }
            k += 1;
            grid.add(p, i, c.t, frame);
        }
        points_in_range += k as u64;
        let end = u32::try_from(rays.ends.len()).expect("fewer than 2^32 rays");
        rays.frame_start.push(end);
    };
    let odom = run_odometry_with(frames, imu, odom_cfg, &mut sink);

    if odom.poses.len() != corrected.len() {
        return Err(MapError::PoseCount {
            poses: corrected.len(),
            frames: odom.poses.len(),
        });
    }
    if let Some(e) = time_error {
        return Err(e);
    }
    let stats = DenseStats {
        frames: odom.poses.len(),
        points_in_range,
        points: grid.cloud.len(),
        rays: rays.ends.len(),
        odom: odom.stats,
    };
    Ok((grid.cloud, rays, stats))
}
