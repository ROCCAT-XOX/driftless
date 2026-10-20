//! Submaps: short stretches of the odometry trajectory with their points in the frame of the
//! first pose (the anchor). Loop closure later moves the anchors, not the points.

use driftless_odom::voxel_map::{voxel_key, FixedState, VoxelKey};
use driftless_odom::{FrameOutput, FrameQuality, OUTPUT_MAP_VOXEL};
use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use std::collections::HashSet;

/// When a submap is closed.
#[derive(Debug, Clone)]
pub struct SubmapConfig {
    /// Path length since the anchor (m) at which the submap is closed.
    pub max_length: f64,
    /// Frame count at which the submap is closed.
    pub max_frames: usize,
}

impl Default for SubmapConfig {
    fn default() -> Self {
        SubmapConfig {
            max_length: 5.0,
            max_frames: 100,
        }
    }
}

/// One frame of a submap.
#[derive(Debug, Clone)]
pub struct SubmapFrame {
    /// Position in the odometry result (`FrameOutput::index`).
    pub index: usize,
    pub t: f64,
    /// `anchor⁻¹ · pose`: frame pose in the anchor frame.
    pub offset: Isometry3<f64>,
    pub quality: FrameQuality,
}

#[derive(Debug, Clone)]
pub struct Submap {
    pub id: usize,
    /// Odometry pose of the first frame.
    pub anchor: Isometry3<f64>,
    pub frames: Vec<SubmapFrame>,
    /// Points in the anchor frame, one per [`OUTPUT_MAP_VOXEL`] voxel.
    pub points: Vec<[f32; 3]>,
}

impl Submap {
    /// Worst frame score in the submap (1 = good). A submap always has at least one frame.
    pub fn quality(&self) -> f32 {
        self.frames
            .iter()
            .map(|f| f.quality.score())
            .fold(f32::INFINITY, f32::min)
    }

    /// The points in the world frame if the submap sits at `anchor` (the odometry anchor or
    /// an optimised one).
    pub fn world_points(&self, anchor: &Isometry3<f64>) -> impl Iterator<Item = [f32; 3]> + '_ {
        let anchor = *anchor;
        self.points.iter().map(move |p| {
            let w = anchor * Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            [w.x as f32, w.y as f32, w.z as f32]
        })
    }
}

/// A submap under construction.
struct Open {
    submap: Submap,
    seen: HashSet<VoxelKey, FixedState>,
    last_pos: Vector3<f64>,
    length: f64,
}

/// Cuts the stream of odometry frames into submaps. The frame that reaches the length or
/// frame limit is the last one of its submap; the next frame starts the next one.
pub struct SubmapBuilder {
    cfg: SubmapConfig,
    open: Option<Open>,
    done: Vec<Submap>,
}

impl SubmapBuilder {
    pub fn new(cfg: SubmapConfig) -> Self {
        SubmapBuilder {
            cfg,
            open: None,
            done: Vec::new(),
        }
    }

    pub fn push(&mut self, f: FrameOutput) {
        let pose = Isometry3::from_parts(Translation3::from(f.pose.p), f.pose.q);
        let open = self.open.get_or_insert_with(|| Open {
            submap: Submap {
                id: self.done.len(),
                anchor: pose,
                frames: Vec::new(),
                points: Vec::new(),
            },
            seen: HashSet::default(),
            last_pos: f.pose.p,
            length: 0.0,
        });
        open.length += (f.pose.p - open.last_pos).norm();
        open.last_pos = f.pose.p;

        let offset = open.submap.anchor.inverse() * pose;
        open.submap.frames.push(SubmapFrame {
            index: f.index,
            t: f.pose.t,
            offset,
            quality: f.quality,
        });
        for p in f.body_points {
            let a = (offset * Point3::from(*p)).coords;
            if open.seen.insert(voxel_key(&a, OUTPUT_MAP_VOXEL)) {
                open.submap
                    .points
                    .push([a.x as f32, a.y as f32, a.z as f32]);
            }
        }

        if open.length >= self.cfg.max_length || open.submap.frames.len() >= self.cfg.max_frames {
            if let Some(o) = self.open.take() {
                self.done.push(o.submap);
            }
        }
    }

    /// Closes the submap in progress (however short) and returns all submaps in order.
    pub fn finish(mut self) -> Vec<Submap> {
        if let Some(o) = self.open.take() {
            self.done.push(o.submap);
        }
        self.done
    }
}
