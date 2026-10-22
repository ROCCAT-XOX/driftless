//! The truncated signed distance field.
//!
//! Sparse blocks of 8³ voxels. Every point votes, for each voxel its measurement ray crosses
//! within ±`truncation_voxels · voxel` of the point, with the signed distance of the voxel
//! centre to the point along the ray (positive in front of the surface, on the sensor side)
//! and weight 1. The votes are summed as integers (in [`VOTE_STEPS`] steps per band, and
//! counts), so the result does not depend on the order the parallel workers add them in. All
//! voxels sit in two flat arrays (sum i32, weight u32: 8 bytes per voxel), filled as atomics
//! and taken over in place.

use super::{SurfaceConfig, SurfaceInput};
use crate::MapError;
use driftless_io::progress::Progress;
use driftless_odom::voxel_map::FixedState;
use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::atomic::{AtomicI32, AtomicU32, Ordering};

/// Edge of a block in voxels.
const BLOCK: i32 = 8;
const BLOCK_VOXELS: usize = (BLOCK * BLOCK * BLOCK) as usize;
/// A vote is the sdf in steps of band / `VOTE_STEPS` (at 2 cm and 4 voxels: 0.08 mm), at most
/// ±`VOTE_STEPS`; an i32 sum holds two million votes of one voxel.
const VOTE_STEPS: f64 = 1000.0;
/// Points per parallel work item (and per progress step).
const CHUNK: usize = 4096;

/// Fused signed distances; see the module docs.
#[derive(Debug)]
pub struct Tsdf {
    voxel: f64,
    min_weight: u32,
    /// The band (m): a vote step is `band / VOTE_STEPS`.
    band: f64,
    /// Block keys, sorted.
    keys: Vec<[i32; 3]>,
    index: HashMap<[i32; 3], u32, FixedState>,
    /// Sum of the sdf votes of voxel `l` of block `b` at `b · BLOCK_VOXELS + l`.
    sum: Vec<i32>,
    /// Number of votes, same layout.
    weight: Vec<u32>,
}

fn block_of(key: [i32; 3]) -> ([i32; 3], usize) {
    let b = key.map(|k| k.div_euclid(BLOCK));
    let l = key.map(|k| k.rem_euclid(BLOCK) as usize);
    (b, l[0] + 8 * l[1] + 64 * l[2])
}

/// Calls `f(voxel key, sdf)` once for every voxel the ray from `sensor` to `p` crosses within
/// `band` (m) of `p`; the sdf (m, clamped to ±`band`) is the voxel centre's distance in front
/// of `p` along the ray. Does nothing for non-finite or degenerate rays.
fn for_each_voxel(
    sensor: [f32; 3],
    p: [f32; 3],
    voxel: f64,
    band: f64,
    mut f: impl FnMut([i32; 3], f64),
) {
    let s = sensor.map(f64::from);
    let d = [
        f64::from(p[0]) - s[0],
        f64::from(p[1]) - s[1],
        f64::from(p[2]) - s[2],
    ];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if !len.is_finite() || len < 1e-6 {
        return;
    }
    let u = [d[0] / len, d[1] / len, d[2] / len];
    let start = (len - band).max(0.0);
    let end = len + band;
    // Half-voxel steps do not skip any voxel the ray crosses over more than half its size.
    let step = 0.5 * voxel;
    let n = ((end - start) / step).ceil() as usize;
    let mut prev: Option<[i32; 3]> = None;
    for i in 0..=n {
        let t = (start + i as f64 * step).min(end);
        let key = [0, 1, 2].map(|a| ((s[a] + u[a] * t) / voxel).floor() as i32);
        if prev == Some(key) {
            continue;
        }
        prev = Some(key);
        let c = key.map(|k| (f64::from(k) + 0.5) * voxel);
        let along = (c[0] - s[0]) * u[0] + (c[1] - s[1]) * u[1] + (c[2] - s[2]) * u[2];
        f(key, (len - along).clamp(-band, band));
    }
}

impl Tsdf {
    /// Fuses the points of `input`.
    pub fn integrate(
        input: &SurfaceInput,
        cfg: &SurfaceConfig,
        progress: &Progress,
    ) -> Result<Tsdf, MapError> {
        cfg.check()?;
        if input.sensor_of_point.len() != input.xyz.len() {
            return Err(MapError::Config(
                "Messposition fehlt für manche Punkte der Oberfläche".into(),
            ));
        }
        if input
            .sensor_of_point
            .iter()
            .any(|&s| s as usize >= input.sensors.len())
        {
            return Err(MapError::Config(
                "Punkt der Oberfläche verweist auf eine Messposition, die es nicht gibt".into(),
            ));
        }
        let voxel = cfg.voxel;
        let band = f64::from(cfg.truncation_voxels) * voxel;
        let ray = |i: usize, f: &mut dyn FnMut([i32; 3], f64)| {
            let sensor = input.sensors[input.sensor_of_point[i] as usize];
            for_each_voxel(sensor, input.xyz[i], voxel, band, f);
        };
        let total = input.xyz.len() as u64;

        // Pass 1: which blocks are touched. Sorted, so the layout does not depend on threads.
        progress.phase("Oberfläche: Blöcke", Some(total));
        let ranges: Vec<std::ops::Range<usize>> = (0..input.xyz.len())
            .step_by(CHUNK)
            .map(|a| a..(a + CHUNK).min(input.xyz.len()))
            .collect();
        let per_chunk: Vec<Vec<[i32; 3]>> = ranges
            .par_iter()
            .map(|r| {
                if progress.is_cancelled() {
                    return Vec::new();
                }
                let mut keys = Vec::new();
                for i in r.clone() {
                    ray(i, &mut |k, _| keys.push(block_of(k).0));
                }
                keys.sort_unstable();
                keys.dedup();
                progress.advance(r.len() as u64);
                keys
            })
            .collect();
        if progress.is_cancelled() {
            return Err(MapError::Cancelled);
        }
        let mut keys: Vec<[i32; 3]> = per_chunk.into_iter().flatten().collect();
        keys.sort_unstable();
        keys.dedup();
        let index: HashMap<[i32; 3], u32, FixedState> = keys
            .iter()
            .enumerate()
            .map(|(i, &k)| (k, i as u32))
            .collect();

        // Pass 2: the votes.
        progress.phase("Oberfläche: Voxel", Some(total));
        let voxels = keys.len() * BLOCK_VOXELS;
        let sums: Vec<AtomicI32> = (0..voxels).map(|_| AtomicI32::new(0)).collect();
        let weights: Vec<AtomicU32> = (0..voxels).map(|_| AtomicU32::new(0)).collect();
        let per_metre = VOTE_STEPS / band;
        ranges.par_iter().for_each(|r| {
            if progress.is_cancelled() {
                return;
            }
            for i in r.clone() {
                ray(i, &mut |k, sdf| {
                    let (b, l) = block_of(k);
                    let v = index[&b] as usize * BLOCK_VOXELS + l;
                    sums[v].fetch_add((sdf * per_metre).round() as i32, Ordering::Relaxed);
                    weights[v].fetch_add(1, Ordering::Relaxed);
                });
            }
            progress.advance(r.len() as u64);
        });
        if progress.is_cancelled() {
            return Err(MapError::Cancelled);
        }
        // Same size and alignment: the collects reuse the allocations.
        let sum: Vec<i32> = sums.into_iter().map(AtomicI32::into_inner).collect();
        let weight: Vec<u32> = weights.into_iter().map(AtomicU32::into_inner).collect();
        Ok(Tsdf {
            voxel,
            min_weight: cfg.min_weight,
            band,
            keys,
            index,
            sum,
            weight,
        })
    }

    /// The mean signed distance (m) and the weight of the voxel `key`, or `None` if it has
    /// fewer than `min_weight` measurements.
    pub fn sample(&self, key: [i32; 3]) -> Option<(f32, u32)> {
        let (b, l) = block_of(key);
        let v = *self.index.get(&b)? as usize * BLOCK_VOXELS + l;
        let w = self.weight[v];
        if w < self.min_weight {
            return None;
        }
        Some((
            (f64::from(self.sum[v]) * self.band / (f64::from(w) * VOTE_STEPS)) as f32,
            w,
        ))
    }

    /// Bytes of the voxel arrays (sum and weight).
    pub fn voxel_bytes(&self) -> usize {
        std::mem::size_of_val(self.sum.as_slice()) + std::mem::size_of_val(self.weight.as_slice())
    }

    /// Edge length of a voxel (m); voxel `k` spans `[k·voxel, (k+1)·voxel)`.
    pub fn voxel(&self) -> f64 {
        self.voxel
    }

    /// Keys of the blocks (8³ voxels, key = voxel key ÷ 8) that hold any measurement, sorted.
    pub fn blocks(&self) -> impl Iterator<Item = [i32; 3]> + '_ {
        self.keys.iter().copied()
    }
}

#[cfg(test)]
mod tests {
    use super::super::testutil::plane_x;
    use super::*;

    /// At 1 cm, the voxel size the expectations below are written for.
    fn integrate(scan: &super::super::testutil::Scan) -> Tsdf {
        let cfg = SurfaceConfig {
            voxel: 0.01,
            ..SurfaceConfig::default()
        };
        Tsdf::integrate(&scan.input(), &cfg, &Progress::new()).unwrap()
    }

    #[test]
    fn plane_zero_crossing_at_wall() {
        // Rays closer together than the voxels, so every voxel in the band is hit often.
        let scan = plane_x([0.0; 3], 2.0, 0.3, 0.004);
        let t = integrate(&scan);
        assert_eq!(t.voxel(), 0.01);
        // Voxel 199 has its centre at 1.995 m, voxel 200 at 2.005 m.
        let (front, wf) = t.sample([199, 0, 0]).expect("observed in front");
        let (back, wb) = t.sample([200, 0, 0]).expect("observed behind");
        assert!(wf >= 2 && wb >= 2);
        assert!((front - 0.005).abs() < 0.002, "front {front}");
        assert!((back + 0.005).abs() < 0.002, "back {back}");
        let mut last = f32::INFINITY;
        // The band is 4 cm: voxels 196 to 203.
        for x in 196..=203 {
            let (v, _) = t
                .sample([x, 0, 0])
                .unwrap_or_else(|| panic!("no sample at {x}"));
            assert!(v < last, "sdf not falling at {x}");
            assert_eq!(v > 0.0, x < 200, "sign at {x}");
            last = v;
        }
    }

    #[test]
    fn eight_bytes_per_voxel() {
        let scan = plane_x([0.0; 3], 2.0, 0.3, 0.01);
        let t = integrate(&scan);
        assert_eq!(t.voxel_bytes(), t.blocks().count() * 512 * 8);
    }

    #[test]
    fn unobserved_is_none() {
        let scan = plane_x([0.0; 3], 2.0, 0.3, 0.01);
        let t = integrate(&scan);
        assert!(t.sample([300, 0, 0]).is_none());
        assert!(t.sample([50, 0, 0]).is_none());
        assert!(t.sample([200, 5000, 0]).is_none());
        assert!(t.blocks().count() > 0);
    }

    #[test]
    fn deterministic_parallel() {
        let scan = plane_x([0.0, 0.1, -0.2], 2.0, 0.5, 0.01);
        let a = integrate(&scan);
        let b = integrate(&scan);
        assert_eq!(
            a.blocks().collect::<Vec<_>>(),
            b.blocks().collect::<Vec<_>>()
        );
        for x in 185..215 {
            for y in -60..60 {
                for z in -60..60 {
                    assert_eq!(a.sample([x, y, z]), b.sample([x, y, z]));
                }
            }
        }
    }

    #[test]
    fn config_check_rejects() {
        let scan = plane_x([0.0; 3], 2.0, 0.1, 0.01);
        for cfg in [
            SurfaceConfig {
                voxel: 0.0,
                ..SurfaceConfig::default()
            },
            SurfaceConfig {
                page_size: 0,
                ..SurfaceConfig::default()
            },
        ] {
            assert!(cfg.check().is_err());
            assert!(Tsdf::integrate(&scan.input(), &cfg, &Progress::new()).is_err());
        }
        assert!(SurfaceConfig::default().check().is_ok());
    }

    #[test]
    fn rejects_bad_sensor_index() {
        let mut scan = plane_x([0.0; 3], 2.0, 0.1, 0.01);
        scan.sensor_of_point[3] = 1;
        let r = Tsdf::integrate(&scan.input(), &SurfaceConfig::default(), &Progress::new());
        assert!(matches!(r, Err(MapError::Config(_))));
    }

    #[test]
    fn sensors_follow_poses_and_mount() {
        use driftless_traj::Pose;
        use nalgebra::{Isometry3, UnitQuaternion, Vector3};
        // Turned 90° about z: a mount 1 m along x ends up 1 m along y.
        let pose = Pose {
            t: 0.0,
            p: Vector3::new(10.0, 0.0, 0.0),
            q: UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        };
        let mount = Isometry3::translation(1.0, 0.0, 0.5);
        let s = super::super::sensors_for_frames(&[pose], &mount);
        assert!((s[0][0] - 10.0).abs() < 1e-5 && (s[0][1] - 1.0).abs() < 1e-5);
        assert!((s[0][2] - 0.5).abs() < 1e-5);
    }
}
