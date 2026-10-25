//! Offset between passes through the same wall, measured on the dense cloud before any
//! cleaning (see [`pass_quality`]). A measurement only: nothing in the cloud changes.
//!
//! The cloud is cut into 3D cells of [`PATCH`]. A cell with enough points on one plane is a
//! patch; its points are split into passes by capture time (the scanner came back later), and
//! the passes' mean levels over the plane are compared. A single pass is as thick as the
//! sensor noise; two passes that disagree add their offset on top, which no smoothing of a
//! single pass can remove.
//!
//! # How the figures relate to the campaign's Python analysis
//! `tools/qualitaet/q_analyze.py` (`refine`) measures patches on the (at most 12) largest room
//! planes, after a 5 degree / 1 cm filter that drops objects, and takes the per-plane aggregate
//! (median over planes of the per-plane p90). This module has no planes: it measures every
//! planar 30 cm cell of the whole cloud, furniture included, and aggregates over cells. So:
//! - the population differs (all planar cells vs. room-plane patches without objects);
//! - `pass_offset_p90_mm` is the p90 over cells, not a median over planes of per-plane p90;
//! - `within_pass_rms_mm` is, as in Python, the median over cells of each cell's within-pass RMS;
//! - a wall within about 1 cm of a cell face is split over two cells, which can hide its offset
//!   (each part sees fewer or only one pass), so the offset here is biased downwards.
//!
//! The figures are therefore comparable, not identical. The campaign's rule applies: they agree
//! if within 1 mm, otherwise the Python value counts.

use crate::clean::{spread_axes, Moments};
use driftless_odom::voxel_map::{FixedState, VoxelKey};
use nalgebra::Vector3;
use rayon::prelude::*;
use std::collections::HashMap;

/// Edge of the cells (m).
const PATCH: f64 = 0.30;
/// Points a cell needs to count as a patch.
const MIN_PATCH_POINTS: usize = 300;
/// A gap in capture time above this starts a new pass (s).
const PASS_GAP_S: f64 = 5.0;
/// Points a pass needs in a cell to count.
const MIN_PASS_POINTS: usize = 40;
/// A cell is planar if the least spread of its points is below this share of the middle one
/// (the planarity of [`crate::SmoothConfig`]).
const PLANARITY: f64 = 0.05;

/// Quality of the cloud's planar patches (see [`pass_quality`]); lengths in mm.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PassQuality {
    /// Planar cells evaluated.
    pub patches: usize,
    /// Of them, with at least two passes.
    pub patches_multi_pass: usize,
    /// Median over patches of the RMS distance to the patch's plane.
    pub local_rms_mm: Option<f64>,
    /// Median over patches of the span between the 2.5 % and 97.5 % quantile of the distances.
    pub thickness95_mm: Option<f64>,
    /// Median over patches of the RMS distance of the points to their own pass's level (only
    /// passes with enough points count; patches without such a pass do not enter).
    pub within_pass_rms_mm: Option<f64>,
    /// Median and 90 % quantile over multi-pass patches of the distance between the highest
    /// and the lowest pass level. The p90 is taken over cells (there are no planes here), unlike
    /// Python's median over planes of per-plane p90.
    pub pass_offset_median_mm: Option<f64>,
    pub pass_offset_p90_mm: Option<f64>,
}

/// What one patch contributes.
struct Patch {
    rms: f64,
    thickness95: f64,
    /// RMS of the residuals to the pass levels; `None` if no pass was big enough.
    within: Option<f64>,
    /// Highest minus lowest pass level; needs two passes.
    offset: Option<f64>,
}

/// Measures the planar patches of a cloud with capture times `t` (s), one per point.
///
/// Deterministic: the cells are evaluated in key order and combined in that order.
///
/// # Panics
/// If `xyz` and `t` differ in length.
pub fn pass_quality(xyz: &[[f32; 3]], t: &[f64]) -> PassQuality {
    assert_eq!(xyz.len(), t.len(), "one capture time per point");
    let mut cells: HashMap<VoxelKey, Vec<u32>, FixedState> = HashMap::default();
    for (i, p) in xyz.iter().enumerate() {
        cells
            .entry(crate::clean::key(p, PATCH))
            .or_default()
            .push(i as u32);
    }
    let mut cells: Vec<(VoxelKey, Vec<u32>)> = cells
        .into_iter()
        .filter(|(_, idx)| idx.len() >= MIN_PATCH_POINTS)
        .collect();
    cells.sort_unstable_by_key(|(k, _)| *k);
    let patches: Vec<Patch> = cells
        .par_iter()
        .filter_map(|(k, idx)| patch(xyz, t, k, idx))
        .collect();

    let multi: Vec<f64> = patches.iter().filter_map(|p| p.offset).collect();
    let mm = |m: f64| m * 1000.0;
    PassQuality {
        patches: patches.len(),
        patches_multi_pass: multi.len(),
        local_rms_mm: median(patches.iter().map(|p| p.rms).collect()).map(mm),
        thickness95_mm: median(patches.iter().map(|p| p.thickness95).collect()).map(mm),
        within_pass_rms_mm: median(patches.iter().filter_map(|p| p.within).collect()).map(mm),
        pass_offset_median_mm: median(multi.clone()).map(mm),
        pass_offset_p90_mm: quantile_sorted(&sorted(multi), 0.9).map(mm),
    }
}

/// The patch of one cell, or `None` if its points are not on a plane.
fn patch(xyz: &[[f32; 3]], t: &[f64], key: &VoxelKey, idx: &[u32]) -> Option<Patch> {
    // Relative to the cell's lowest corner, for precision.
    let corner = Vector3::new(key[0], key[1], key[2]).map(|c| f64::from(c) * PATCH);
    let rel = |i: u32| crate::clean::vec3(&xyz[i as usize]) - corner;
    let mut m = Moments::ZERO;
    for &i in idx {
        m.add(&rel(i));
    }
    let (mean, cov) = m.mean_cov();
    let spread = spread_axes(&cov);
    let planar = spread[0].0 < PLANARITY * spread[1].0;
    // NaN (degenerate cells) compares false, so it counts as not planar.
    if !planar {
        return None;
    }
    let normal = spread[0].1;
    let resid: Vec<f64> = idx.iter().map(|&i| normal.dot(&(rel(i) - mean))).collect();
    let rms = (resid.iter().map(|r| r * r).sum::<f64>() / resid.len() as f64).sqrt();
    let all = sorted(resid.clone());
    let thickness95 = quantile_sorted(&all, 0.975)? - quantile_sorted(&all, 0.025)?;

    // Passes: points with a usable time, in time order, split at gaps.
    let mut timed: Vec<(f64, f64)> = idx
        .iter()
        .zip(&resid)
        .filter(|(&i, _)| t[i as usize].is_finite())
        .map(|(&i, &r)| (t[i as usize], r))
        .collect();
    timed.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    let (mut within_sq, mut within_n) = (0.0, 0usize);
    let mut levels = Vec::new();
    let mut start = 0;
    for end in 1..=timed.len() {
        if end < timed.len() && timed[end].0 - timed[end - 1].0 <= PASS_GAP_S {
            continue;
        }
        let pass = &timed[start..end];
        start = end;
        if pass.len() < MIN_PASS_POINTS {
            continue;
        }
        let level = median(pass.iter().map(|p| p.1).collect())?;
        within_sq += pass.iter().map(|p| (p.1 - level).powi(2)).sum::<f64>();
        within_n += pass.len();
        levels.push(level);
    }
    let offset = (levels.len() >= 2).then(|| {
        let max = levels.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min = levels.iter().copied().fold(f64::INFINITY, f64::min);
        max - min
    });
    Some(Patch {
        rms,
        thickness95,
        within: (within_n > 0).then(|| (within_sq / within_n as f64).sqrt()),
        offset,
    })
}

fn sorted(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_unstable_by(f64::total_cmp);
    v
}

/// Value at rank `q` (0..1) of a sorted slice, nearest rank; `None` if empty.
fn quantile_sorted(v: &[f64], q: f64) -> Option<f64> {
    (!v.is_empty()).then(|| v[((v.len() - 1) as f64 * q).round() as usize])
}

fn median(v: Vec<f64>) -> Option<f64> {
    let v = sorted(v);
    match v.len() {
        0 => None,
        n if n % 2 == 1 => Some(v[n / 2]),
        n => Some((v[n / 2 - 1] + v[n / 2]) / 2.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic jitter in ±1 mm (xorshift), no rand crate.
    struct Jitter(u64);
    impl Jitter {
        fn next(&mut self) -> f32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            ((self.0 % 2001) as f32 - 1000.0) / 1_000_000.0
        }
    }

    /// A 2 × 2 m wall at `x`, a point every 5 mm, captured from `t0` to `t0 + 10`.
    fn wall(x: f32, t0: f64, seed: u64, xyz: &mut Vec<[f32; 3]>, t: &mut Vec<f64>) {
        let mut j = Jitter(seed);
        let n = 400;
        for iy in 0..n {
            for iz in 0..n {
                let k = (iy * n + iz) as f64 / f64::from(n * n);
                xyz.push([
                    x + j.next(),
                    0.1 + iy as f32 * 0.005,
                    0.1 + iz as f32 * 0.005,
                ]);
                t.push(t0 + 10.0 * k);
            }
        }
    }

    // The walls stand at x = 0.1 (not 0): a plane on a cell face would be cut in two by its jitter.
    fn two_passes() -> (Vec<[f32; 3]>, Vec<f64>) {
        let (mut xyz, mut t) = (Vec::new(), Vec::new());
        wall(0.1, 0.0, 0x9E37_79B9_7F4A_7C15, &mut xyz, &mut t);
        wall(0.108, 100.0, 0xD1B5_4A32_D192_ED03, &mut xyz, &mut t);
        (xyz, t)
    }

    fn one_pass() -> (Vec<[f32; 3]>, Vec<f64>) {
        let (mut xyz, mut t) = (Vec::new(), Vec::new());
        wall(0.1, 0.0, 0x9E37_79B9_7F4A_7C15, &mut xyz, &mut t);
        (xyz, t)
    }

    #[test]
    fn two_passes_8mm_apart() {
        let (xyz, t) = two_passes();
        let q = pass_quality(&xyz, &t);
        let offset = q.pass_offset_median_mm.expect("offset");
        assert!((offset - 8.0).abs() < 0.5, "{q:?}");
        assert!(q.patches_multi_pass > 0, "{q:?}");
        assert!(q.within_pass_rms_mm.unwrap() < 1.5, "{q:?}");
    }

    #[test]
    fn within_pass_rms_is_median_over_cells() {
        // Three single-pass cells on a wall at x = 0.1 with jitter of +-1, +-1 and +-9 mm:
        // per-cell RMS 0.58, 0.58 and 5.2 mm. The median is 0.58; one pooled RMS would be ~3.
        let (mut xyz, mut t) = (Vec::new(), Vec::new());
        for (cell, amp) in [(0, 1.0f32), (1, 1.0), (2, 9.0)] {
            let mut j = Jitter(0x9E37_79B9_7F4A_7C15 + cell as u64);
            for iy in 0..60 {
                for iz in 0..60 {
                    xyz.push([
                        0.1 + amp * j.next(),
                        0.01 + cell as f32 * 0.3 + iy as f32 * 0.005,
                        0.01 + iz as f32 * 0.005,
                    ]);
                    t.push(f64::from(iy * 60 + iz) * 1e-3);
                }
            }
        }
        let q = pass_quality(&xyz, &t);
        assert_eq!(q.patches, 3, "{q:?}");
        let w = q.within_pass_rms_mm.unwrap();
        assert!((w - 0.58).abs() < 0.1, "median expected, got {w} ({q:?})");
    }

    #[test]
    fn one_pass_has_no_offset() {
        let (xyz, t) = one_pass();
        let q = pass_quality(&xyz, &t);
        assert_eq!(q.pass_offset_median_mm, None);
        assert_eq!(q.pass_offset_p90_mm, None);
        assert_eq!(q.patches_multi_pass, 0);
        assert!(q.patches > 0 && q.local_rms_mm.is_some(), "{q:?}");
    }

    #[test]
    fn constant_or_bad_times_do_not_panic() {
        let (xyz, _) = two_passes();
        let q = pass_quality(&xyz, &vec![0.0; xyz.len()]);
        assert_eq!(q.pass_offset_median_mm, None);
        let (xyz, mut t) = one_pass();
        t[0] = f64::NAN;
        t[1] = f64::INFINITY;
        assert_eq!(pass_quality(&xyz, &t).pass_offset_median_mm, None);
        assert_eq!(pass_quality(&[], &[]), PassQuality::default());
    }

    #[test]
    fn curved_cells_are_skipped() {
        // Fibonacci sphere of radius 0.2 m around a cell corner.
        let n = 20_000;
        let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
        let xyz: Vec<[f32; 3]> = (0..n)
            .map(|i| {
                let z = 1.0 - 2.0 * (f64::from(i) + 0.5) / f64::from(n);
                let r = (1.0 - z * z).sqrt();
                let a = golden * f64::from(i);
                [
                    (0.2 * r * a.cos()) as f32,
                    (0.2 * r * a.sin()) as f32,
                    (0.2 * z) as f32,
                ]
            })
            .collect();
        let q = pass_quality(&xyz, &vec![0.0; xyz.len()]);
        assert_eq!(q.patches, 0, "{q:?}");
    }
}
