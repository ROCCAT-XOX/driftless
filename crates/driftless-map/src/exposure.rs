//! Exposure and white balance between the cameras and over time.
//!
//! Every used image `i` gets a gain triple `g_i` (linear RGB): a surface point `j` of colour
//! `a_j` shows in it as `c_ij = g_i · a_j`. The gains come from the views the colour pass kept
//! (see [`crate::colorize`]): up to [`ExposureConfig::samples`] points with views from at
//! least 2 images are taken, and per channel the least-squares problem
//!
//! ```text
//! Σ_ij (log c_ij − log g_i − log a_j)²  +  smooth · Σ_(i,i') (log g_i − log g_i')²
//! ```
//!
//! is solved, the second sum over consecutive images `i, i'` of the same camera, first plainly
//! and then in three rounds with Huber weights on the residuals. The fit
//! cannot see a factor common to all images; it is fixed by `mean(log g) = 0`. Dark
//! (< [`DARK`]) and saturated (> [`SATURATED`]) observations say nothing about the gain and
//! are left out per channel. Each view's colour is then divided by its image's gain before
//! the blend.
//!
//! The `a_j` are eliminated (for given gains each is the mean of its point's
//! `log c_ij − log g_i`), which leaves a sparse, symmetric system in the image gains only. It
//! is solved by conjugate gradients without forming the matrix, in a fixed order.

use crate::color::{srgb_value, View};
use crate::MapError;
use rayon::prelude::*;

/// Observations darker than this (linear, about sRGB 10/255) are left out of the fit.
pub const DARK: f32 = 0.003;
/// Observations brighter than this (linear, about sRGB 250/255) count as saturated and are
/// left out of the fit.
pub const SATURATED: f32 = 0.95;
/// Pulls every log gain towards 0 with this weight (one observation weighs 1): makes the
/// system solvable for images without any observation, without moving the others.
const RIDGE: f64 = 1e-6;
/// Re-weighted rounds after the plain least-squares fit.
const ROBUST_ROUNDS: usize = 3;
/// Residuals (in log colour) up to this count fully; larger ones get the weight
/// `HUBER_DELTA / |residual|`.
const HUBER_DELTA: f64 = 0.1;
/// The conjugate gradients stop once the residual is this share of the right-hand side.
const CG_TOLERANCE: f64 = 1e-10;

#[derive(Debug, Clone, PartialEq)]
pub struct ExposureConfig {
    /// Weight of the smoothness between consecutive images of the same camera, against 1 per
    /// observation.
    pub smooth: f64,
    /// Gains are clamped to `[1/max_gain, max_gain]` (at least 1).
    pub max_gain: f64,
    /// Points used for the fit; 0 turns the equalisation off (every gain 1).
    pub samples: usize,
}

impl Default for ExposureConfig {
    fn default() -> Self {
        ExposureConfig {
            smooth: 1.0,
            max_gain: 4.0,
            samples: 50_000,
        }
    }
}

impl ExposureConfig {
    pub(crate) fn check(&self) -> Result<(), MapError> {
        if !(self.smooth.is_finite() && self.smooth >= 0.0) {
            return Err(MapError::Config(
                "Glättung des Belichtungsabgleichs darf nicht negativ sein".into(),
            ));
        }
        if !(self.max_gain.is_finite() && self.max_gain >= 1.0) {
            return Err(MapError::Config(
                "größter Belichtungsfaktor muss mindestens 1 sein".into(),
            ));
        }
        Ok(())
    }
}

/// The observed linear colours of sampled points, per image.
#[derive(Debug, Clone, PartialEq)]
pub struct Observations {
    /// Camera of every image; images are numbered in capture order.
    cameras: Vec<usize>,
    /// Point `j`'s views are `views[starts[j]..starts[j + 1]]`.
    starts: Vec<usize>,
    views: Vec<(u32, [f32; 3])>,
}

impl Observations {
    /// No points yet; `cameras[i]` is the camera of image `i`, images in capture order (the
    /// smoothness links consecutive images of a camera).
    pub fn new(cameras: Vec<usize>) -> Observations {
        Observations {
            cameras,
            starts: vec![0],
            views: Vec::new(),
        }
    }

    /// Adds a point with its views: (image, observed linear colour), at most one per image.
    pub fn push_point(&mut self, views: &[(u32, [f32; 3])]) {
        assert!(
            views.iter().all(|v| (v.0 as usize) < self.cameras.len()),
            "view of an unknown image"
        );
        self.views.extend_from_slice(views);
        self.starts.push(self.views.len());
    }

    pub fn images(&self) -> usize {
        self.cameras.len()
    }

    pub fn points(&self) -> usize {
        self.starts.len() - 1
    }

    fn point(&self, j: usize) -> &[(u32, [f32; 3])] {
        &self.views[self.starts[j]..self.starts[j + 1]]
    }
}

/// The gain of one used image.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageGain {
    pub camera: String,
    /// Image stamp (s), as in the bag.
    pub stamp: f64,
    /// Linear (r, g, b) factor: the image showed colours this much brighter than the scan's
    /// mean; its views are divided by it.
    pub gain: [f32; 3],
}

/// Gain triple of every image of `obs` (linear RGB), clamped to
/// `[1/max_gain, max_gain]`. Images without usable observations follow their neighbours of
/// the same camera; with none at all every gain is 1. Same input, same result.
pub fn estimate_gains(obs: &Observations, cfg: &ExposureConfig) -> Vec<[f32; 3]> {
    let n = obs.images();
    let bound = cfg.max_gain.max(1.0).ln();
    // Consecutive images of the same camera.
    let mut last: Vec<Option<usize>> = Vec::new();
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    for (i, &c) in obs.cameras.iter().enumerate() {
        if c >= last.len() {
            last.resize(c + 1, None);
        }
        if let Some(prev) = last[c].replace(i) {
            pairs.push((prev, i));
        }
    }
    let mut gains = vec![[1.0f32; 3]; n];
    for c in 0..3 {
        let x = solve_channel(obs, c, &pairs, cfg.smooth);
        for (g, x) in gains.iter_mut().zip(x) {
            g[c] = x.clamp(-bound, bound).exp() as f32;
        }
    }
    gains
}

/// Log gains of channel `c`, with mean 0.
fn solve_channel(obs: &Observations, c: usize, pairs: &[(usize, usize)], smooth: f64) -> Vec<f64> {
    let n = obs.images();
    // The usable observations (image, log colour) of every point with at least 2 of them.
    let mut starts = vec![0usize];
    let mut seen: Vec<(usize, f64)> = Vec::new();
    for j in 0..obs.points() {
        let start = seen.len();
        seen.extend(
            obs.point(j)
                .iter()
                .filter(|v| (DARK..=SATURATED).contains(&v.1[c]))
                .map(|v| (v.0 as usize, f64::from(v.1[c]).ln())),
        );
        if seen.len() - start < 2 {
            seen.truncate(start);
        } else {
            starts.push(seen.len());
        }
    }
    if starts.len() == 1 || n == 0 {
        return vec![0.0; n];
    }
    // Least squares, then re-weighted rounds with Huber weights on the residuals: views that
    // disagree with the fit (misregistered edges, highlights, small occlusion errors) lose
    // influence.
    let mut w = vec![1.0; seen.len()];
    let mut x = solve_weighted(n, &starts, &seen, &w, pairs, smooth);
    for _ in 0..ROBUST_ROUNDS {
        for j in 0..starts.len() - 1 {
            let r = starts[j]..starts[j + 1];
            let ws: f64 = w[r.clone()].iter().sum();
            let log_a = seen[r.clone()]
                .iter()
                .zip(&w[r.clone()])
                .map(|(o, w)| w * (o.1 - x[o.0]))
                .sum::<f64>()
                / ws;
            for k in r {
                let e = (seen[k].1 - x[seen[k].0] - log_a).abs();
                w[k] = if e <= HUBER_DELTA {
                    1.0
                } else {
                    HUBER_DELTA / e
                };
            }
        }
        x = solve_weighted(n, &starts, &seen, &w, pairs, smooth);
    }
    let mean = x.iter().sum::<f64>() / n as f64;
    x.iter().map(|x| x - mean).collect()
}

/// Solves the weighted problem: point `j`'s observations are `seen[starts[j]..starts[j + 1]]`
/// with weights `w`.
fn solve_weighted(
    n: usize,
    starts: &[usize],
    seen: &[(usize, f64)],
    w: &[f64],
    pairs: &[(usize, usize)],
    smooth: f64,
) -> Vec<f64> {
    let points = starts.len() - 1;
    let range = |j: usize| starts[j]..starts[j + 1];
    // Right-hand side and diagonal of the system A x = b.
    let mut b = vec![0.0; n];
    let mut diag = vec![RIDGE; n];
    for j in 0..points {
        let r = range(j);
        let ws: f64 = w[r.clone()].iter().sum();
        let mean = seen[r.clone()]
            .iter()
            .zip(&w[r.clone()])
            .map(|(o, w)| w * o.1)
            .sum::<f64>()
            / ws;
        for (&(i, l), &wk) in seen[r.clone()].iter().zip(&w[r]) {
            b[i] += wk * (l - mean);
            diag[i] += wk * (1.0 - wk / ws);
        }
    }
    for &(i, k) in pairs {
        diag[i] += smooth;
        diag[k] += smooth;
    }
    let apply = |x: &[f64], out: &mut [f64]| {
        for (o, xi) in out.iter_mut().zip(x) {
            *o = RIDGE * xi;
        }
        for j in 0..points {
            let r = range(j);
            let ws: f64 = w[r.clone()].iter().sum();
            let mean = seen[r.clone()]
                .iter()
                .zip(&w[r.clone()])
                .map(|(o, w)| w * x[o.0])
                .sum::<f64>()
                / ws;
            for (&(i, _), &wk) in seen[r.clone()].iter().zip(&w[r]) {
                out[i] += wk * (x[i] - mean);
            }
        }
        for &(i, k) in pairs {
            let d = smooth * (x[i] - x[k]);
            out[i] += d;
            out[k] -= d;
        }
    };

    // Jacobi-preconditioned conjugate gradients from x = 0.
    let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
    let mut x = vec![0.0; n];
    let mut r = b.clone();
    let mut z: Vec<f64> = r.iter().zip(&diag).map(|(r, d)| r / d).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let mut ap = vec![0.0; n];
    let stop = CG_TOLERANCE * dot(&b, &b).sqrt();
    for _ in 0..10 * n + 100 {
        if dot(&r, &r).sqrt() <= stop {
            break;
        }
        apply(&p, &mut ap);
        let alpha = rz / dot(&p, &ap);
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
            z[i] = r[i] / diag[i];
        }
        let rz_next = dot(&r, &z);
        let beta = rz_next / rz;
        rz = rz_next;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }
    x
}

/// Up to `samples` points with views from at least 2 images, evenly spread over the cloud
/// order, and their views. `slots` holds `k` views per point (see `crate::color`), images
/// are numbered by the views' rank; `cameras[rank]` is the camera of each.
pub(crate) fn sample_observations(
    slots: &[View],
    k: usize,
    cameras: Vec<usize>,
    samples: usize,
) -> Observations {
    let eligible: Vec<usize> = slots
        .par_chunks(k)
        .map(|s| s.iter().filter(|v| v.weight > 0.0).count() >= 2)
        .collect::<Vec<bool>>()
        .iter()
        .enumerate()
        .filter_map(|(j, &e)| e.then_some(j))
        .collect();
    let m = eligible.len();
    let take = m.min(samples);
    let mut obs = Observations::new(cameras);
    let mut views = Vec::with_capacity(k);
    for s in 0..take {
        let j = eligible[s * m / take];
        views.clear();
        views.extend(
            slots[j * k..(j + 1) * k]
                .iter()
                .filter(|v| v.weight > 0.0)
                .map(|v| (v.rank, v.rgb)),
        );
        obs.push_point(&views);
    }
    obs
}

/// Divides every kept view's colour by its image's gain.
pub(crate) fn apply_gains(slots: &mut [View], gains: &[[f32; 3]]) {
    slots
        .par_iter_mut()
        .filter(|v| v.weight > 0.0)
        .for_each(|v| {
            let g = &gains[v.rank as usize];
            for (c, g) in v.rgb.iter_mut().zip(g) {
                *c /= g;
            }
        });
}

/// Mean colour spread between the views of a point, over the points with at least 2 views:
/// the standard deviation of the views' colours per channel, in sRGB units (0–255),
/// averaged over the channels. 0 without such points.
pub(crate) fn view_spread(slots: &[View], k: usize) -> f64 {
    let spreads: Vec<Option<f64>> = slots
        .par_chunks(k)
        .map(|s| {
            let views: Vec<[f64; 3]> = s
                .iter()
                .filter(|v| v.weight > 0.0)
                .map(|v| v.rgb.map(srgb_value))
                .collect();
            (views.len() >= 2).then(|| {
                let m = views.len() as f64;
                (0..3)
                    .map(|c| {
                        let mean = views.iter().map(|v| v[c]).sum::<f64>() / m;
                        (views.iter().map(|v| (v[c] - mean).powi(2)).sum::<f64>() / m).sqrt()
                    })
                    .sum::<f64>()
                    / 3.0
            })
        })
        .collect();
    let (sum, count) = spreads
        .iter()
        .flatten()
        .fold((0.0, 0usize), |(s, n), v| (s + v, n + 1));
    if count == 0 {
        0.0
    } else {
        sum / count as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_images_split_the_ratio() {
        // Image 1 sees everything twice as bright: gains 1/√2 and √2 (smoothness pulls a
        // little, 200 observations against weight 1).
        let mut obs = Observations::new(vec![0, 0]);
        for j in 0..100 {
            let a = 0.1 + 0.002 * j as f32;
            obs.push_point(&[(0, [a; 3]), (1, [2.0 * a; 3])]);
        }
        let free = ExposureConfig {
            smooth: 0.0,
            ..ExposureConfig::default()
        };
        let g = estimate_gains(&obs, &free);
        let s = std::f32::consts::SQRT_2;
        assert!(
            (g[0][0] - 1.0 / s).abs() < 1e-5 && (g[1][2] - s).abs() < 1e-5,
            "{g:?}"
        );
        let smoothed = estimate_gains(&obs, &ExposureConfig::default());
        assert!(
            smoothed[1][0] < g[1][0] && smoothed[1][0] > 1.35,
            "{smoothed:?}"
        );
    }

    #[test]
    fn dark_and_saturated_views_are_ignored_per_channel() {
        // Red is saturated in image 1 and blue dark in image 0: those channels have nothing
        // to compare; green shows the 1.5× ratio.
        let mut obs = Observations::new(vec![0, 1]);
        for j in 0..50 {
            let a = 0.2 + 0.004 * j as f32;
            obs.push_point(&[(0, [0.6, a, 0.001]), (1, [1.0, 1.5 * a, 0.3])]);
        }
        let free = ExposureConfig {
            smooth: 0.0,
            ..ExposureConfig::default()
        };
        let g = estimate_gains(&obs, &free);
        assert_eq!(g[0][0], 1.0);
        assert_eq!(g[1][2], 1.0);
        assert!((g[1][1] / g[0][1] - 1.5).abs() < 1e-5, "{g:?}");
    }

    #[test]
    fn outlying_views_do_not_pull_the_gain() {
        // 100 points show image 1 twice as bright, 10 more (misregistered) five times.
        let mut obs = Observations::new(vec![0, 1]);
        for j in 0..110 {
            let a = 0.05 + 0.001 * (j % 100) as f32;
            let ratio = if j < 100 { 2.0 } else { 5.0 };
            obs.push_point(&[(0, [a; 3]), (1, [ratio * a; 3])]);
        }
        let g = estimate_gains(&obs, &ExposureConfig::default());
        let ratio = g[1][0] / g[0][0];
        // Plain least squares: exp((100 ln 2 + 10 ln 5) / 110) = 2.17.
        // Huber weights bound the outliers' pull (to 2.04 here).
        assert!((ratio - 2.0).abs() < 0.06, "{ratio}");
    }

    #[test]
    fn image_without_views_follows_its_neighbours() {
        // Images 0, 1, 2 of one camera; image 1 has no views; 0 and 2 differ by 4×.
        let mut obs = Observations::new(vec![0, 0, 0]);
        for j in 0..100 {
            let a = 0.05 + 0.001 * j as f32;
            obs.push_point(&[(0, [a; 3]), (2, [4.0 * a; 3])]);
        }
        let g = estimate_gains(&obs, &ExposureConfig::default());
        // In the log midway: gain 1 (the mean of the three logs is 0).
        assert!((g[1][0] - 1.0).abs() < 1e-3, "{g:?}");
        assert!(g[0][0] < 0.6 && g[2][0] > 1.7, "{g:?}");
    }
}
