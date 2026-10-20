//! Voxel downsampling and the local voxel map the scans are registered against.
//!
//! All hash containers use a fixed hasher, so iteration order (and with it every result
//! downstream) is the same in every run.

use nalgebra::Vector3;
use std::collections::hash_map::DefaultHasher;
use std::collections::{HashMap, HashSet};
use std::hash::BuildHasherDefault;

/// Deterministic hasher (SipHash with fixed keys).
pub type FixedState = BuildHasherDefault<DefaultHasher>;

pub type VoxelKey = [i32; 3];

pub fn voxel_key(p: &Vector3<f64>, voxel: f64) -> VoxelKey {
    // `as` saturates for huge values, which only merges far-away garbage into edge voxels.
    [
        (p.x / voxel).floor() as i32,
        (p.y / voxel).floor() as i32,
        (p.z / voxel).floor() as i32,
    ]
}

/// Keeps the first point of every voxel, in input order.
pub fn voxel_downsample(points: &[Vector3<f64>], voxel: f64) -> Vec<Vector3<f64>> {
    let mut seen: HashSet<VoxelKey, FixedState> =
        HashSet::with_capacity_and_hasher(points.len(), FixedState::default());
    points
        .iter()
        .filter(|p| seen.insert(voxel_key(p, voxel)))
        .copied()
        .collect()
}

/// Sparse voxel grid holding up to `max_per_voxel` points per voxel (first come, first kept).
/// A new point closer than `min_dist` to a point already in its voxel is skipped, so a
/// device standing still does not pile up copies of the same points.
#[derive(Debug, Clone)]
pub struct VoxelMap {
    voxel: f64,
    max_per_voxel: usize,
    min_dist: f64,
    voxels: HashMap<VoxelKey, Vec<Vector3<f64>>, FixedState>,
}

impl VoxelMap {
    pub fn new(voxel: f64, max_per_voxel: usize, min_dist: f64) -> Self {
        VoxelMap {
            voxel,
            max_per_voxel,
            min_dist,
            voxels: HashMap::default(),
        }
    }

    pub fn insert(&mut self, points: &[Vector3<f64>]) {
        let min2 = self.min_dist * self.min_dist;
        for p in points {
            let v = self.voxels.entry(voxel_key(p, self.voxel)).or_default();
            if v.len() < self.max_per_voxel && v.iter().all(|q| (q - p).norm_squared() >= min2) {
                v.push(*p);
            }
        }
    }

    /// Drops voxels whose first point is farther than `radius` from `center`.
    pub fn remove_far(&mut self, center: &Vector3<f64>, radius: f64) {
        let r2 = radius * radius;
        self.voxels.retain(|_, pts| {
            pts.first()
                .is_some_and(|p| (p - center).norm_squared() <= r2)
        });
    }

    /// All points, voxel by voxel.
    pub fn points(&self) -> Vec<Vector3<f64>> {
        self.voxels.values().flatten().copied().collect()
    }

    pub fn len(&self) -> usize {
        self.voxels.values().map(Vec::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.voxels.is_empty()
    }
}
