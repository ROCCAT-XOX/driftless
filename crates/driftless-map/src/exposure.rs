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
//!
//! The fisheyes also darken towards the image edge. With [`ExposureConfig::vignetting`] the
//! model becomes `c_ij = g_i · V_cam(θ_ij) · a_j` with `V(θ) = exp(b1·θ² + b2·θ⁴)` per camera
//! and channel (θ the view's angle from the optical axis), and per channel
//! [`VIGNETTE_ROUNDS`] alternating steps fit (a) the gains as above on `log c − log V` and
//! (b) per camera `b1, b2` by weighted least squares (2 × 2, the Huber weights of (a)) on
//! `log c − log g_i − log a_j`, the `a_j` again eliminated; a last (a) fits the gains to the
//! final falloff. There is no constant term: `V(0) = 1`, the overall brightness stays fixed by
//! `mean(log g) = 0`. A camera whose views barely differ in θ (5 % to 95 % quantile closer
//! than [`VIGNETTE_MIN_SPREAD`] overall, or than [`VIGNETTE_MIN_POINT_SPREAD`] between the
//! views of the same point), or that has fewer than [`VIGNETTE_MIN_OBSERVATIONS`] keeps
//! `V = 1`. Each view is divided by `g_i · V_cam(θ)` before the blend.

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
/// Alternations between the gains and the edge falloff, per channel.
pub const VIGNETTE_ROUNDS: usize = 3;
/// A camera's falloff is fitted only if the 5 % to 95 % quantile range of its views' angles
/// from the optical axis reaches this (rad) ...
pub const VIGNETTE_MIN_SPREAD: f64 = 0.35;
/// ... the same range of the deviations from the mean angle of the same point's views of the
/// camera reaches this (rad, about 3°; Promedia: 0.18–0.23) ...
pub const VIGNETTE_MIN_POINT_SPREAD: f64 = 0.05;
/// ... and it has at least this many observations in the channel.
pub const VIGNETTE_MIN_OBSERVATIONS: usize = 1000;

#[derive(Debug, Clone, PartialEq)]
pub struct ExposureConfig {
    /// Weight of the smoothness between consecutive images of the same camera, against 1 per
    /// observation.
    pub smooth: f64,
    /// Gains are clamped to `[1/max_gain, max_gain]` (at least 1).
    pub max_gain: f64,
    /// Points used for the fit; 0 turns the equalisation off (every gain 1).
    pub samples: usize,
    /// Fit the edge falloff of every camera together with the gains.
    pub vignetting: bool,
}

impl Default for ExposureConfig {
    fn default() -> Self {
        ExposureConfig {
            smooth: 1.0,
            max_gain: 4.0,
            samples: 50_000,
            vignetting: true,
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
    views: Vec<ViewObs>,
}

/// One observation of a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewObs {
    pub image: u32,
    /// Observed linear colour.
    pub rgb: [f32; 3],
    /// Angle from the image's optical axis (rad).
    pub theta: f32,
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

    /// Adds a point with its views, at most one per image.
    pub fn push_point(&mut self, views: &[ViewObs]) {
        assert!(
            views
                .iter()
                .all(|v| (v.image as usize) < self.cameras.len()),
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

    /// The views of point `j`.
    pub fn point(&self, j: usize) -> &[ViewObs] {
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

/// Edge falloff of every camera: `V(θ) = exp(b1·θ² + b2·θ⁴)` per channel, `V(0) = 1`.
#[derive(Debug, Clone, PartialEq)]
pub struct Vignetting {
    /// Per camera (as numbered in [`Observations::new`]), per channel: `[b1, b2]` (θ in rad).
    pub coeffs: Vec<[[f64; 2]; 3]>,
    /// Factors are clamped to `[1/max_gain, 1]`.
    pub max_gain: f64,
}

impl Default for Vignetting {
    fn default() -> Self {
        Vignetting::flat(0, ExposureConfig::default().max_gain)
    }
}

impl Vignetting {
    /// `V = 1` for `cameras` cameras.
    pub fn flat(cameras: usize, max_gain: f64) -> Vignetting {
        Vignetting {
            coeffs: vec![[[0.0; 2]; 3]; cameras],
            max_gain,
        }
    }

    /// Falloff of `camera` in `channel` at `theta` (rad) from the optical axis, in
    /// `[1/max_gain, 1]`; 1 for an unknown camera.
    pub fn factor(&self, camera: usize, channel: usize, theta: f32) -> f32 {
        self.log_factor(camera, channel, f64::from(theta)).exp() as f32
    }

    fn log_factor(&self, camera: usize, channel: usize, theta: f64) -> f64 {
        let Some(c) = self.coeffs.get(camera) else {
            return 0.0;
        };
        let [b1, b2] = c[channel];
        let t2 = theta * theta;
        (b1 * t2 + b2 * t2 * t2).clamp(-self.max_gain.max(1.0).ln(), 0.0)
    }
}

/// Gain triple of every image of `obs` (linear RGB), clamped to
/// `[1/max_gain, max_gain]`; the edge falloff is fitted with them but not returned (see
/// [`estimate_exposure`]).
pub fn estimate_gains(obs: &Observations, cfg: &ExposureConfig) -> Vec<[f32; 3]> {
    estimate_exposure(obs, cfg).0
}

/// Gain triple of every image of `obs` (linear RGB), clamped to `[1/max_gain, max_gain]`, and
/// the edge falloff of every camera (flat with [`ExposureConfig::vignetting`] off). Images
/// without usable observations follow their neighbours of the same camera; with none at all
/// every gain is 1. Same input, same result.
pub fn estimate_exposure(obs: &Observations, cfg: &ExposureConfig) -> (Vec<[f32; 3]>, Vignetting) {
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
    let mut vignetting = Vignetting::flat(last.len(), cfg.max_gain);
    let mut gains = vec![[1.0f32; 3]; n];
    for c in 0..3 {
        let seen = Seen::new(obs, c);
        let x = if cfg.vignetting {
            fit_channel(&seen, &pairs, cfg.smooth, c, &mut vignetting)
        } else {
            fit_gains(&seen, &seen.log, &pairs, cfg.smooth).0
        };
        for (g, x) in gains.iter_mut().zip(x) {
            g[c] = x.clamp(-bound, bound).exp() as f32;
        }
    }
    (gains, vignetting)
}

/// The usable observations of one channel: those within [`DARK`] to [`SATURATED`], of the
/// points with at least 2 of them. Point `j`'s are `starts[j]..starts[j + 1]`; kept as
/// parallel arrays, 16 bytes per observation.
struct Seen {
    /// Camera of every image.
    cameras: Vec<usize>,
    starts: Vec<usize>,
    image: Vec<u32>,
    theta: Vec<f32>,
    /// Log colour.
    log: Vec<f64>,
}

impl Seen {
    fn new(obs: &Observations, c: usize) -> Seen {
        // Upper bound of the observations: exact capacities keep the buffers from doubling.
        let usable = obs
            .views
            .iter()
            .filter(|v| (DARK..=SATURATED).contains(&v.rgb[c]))
            .count();
        let mut seen = Seen {
            cameras: obs.cameras.clone(),
            starts: Vec::with_capacity(obs.points() + 1),
            image: Vec::with_capacity(usable),
            theta: Vec::with_capacity(usable),
            log: Vec::with_capacity(usable),
        };
        seen.starts.push(0);
        for j in 0..obs.points() {
            let start = seen.log.len();
            for v in obs
                .point(j)
                .iter()
                .filter(|v| (DARK..=SATURATED).contains(&v.rgb[c]))
            {
                seen.image.push(v.image);
                seen.theta.push(v.theta);
                seen.log.push(f64::from(v.rgb[c]).ln());
            }
            if seen.log.len() - start < 2 {
                seen.image.truncate(start);
                seen.theta.truncate(start);
                seen.log.truncate(start);
            } else {
                seen.starts.push(seen.log.len());
            }
        }
        seen
    }

    fn images(&self) -> usize {
        self.cameras.len()
    }

    fn points(&self) -> usize {
        self.starts.len() - 1
    }

    fn range(&self, j: usize) -> std::ops::Range<usize> {
        self.starts[j]..self.starts[j + 1]
    }

    fn camera(&self, k: usize) -> usize {
        self.cameras[self.image[k] as usize]
    }
}

/// Log gains of channel `c` (mean 0), alternating with the falloff of every camera, which is
/// written into `vignetting`.
fn fit_channel(
    seen: &Seen,
    pairs: &[(usize, usize)],
    smooth: f64,
    c: usize,
    vignetting: &mut Vignetting,
) -> Vec<f64> {
    let observable: Vec<usize> = (0..vignetting.coeffs.len())
        .filter(|&cam| observable(seen, cam))
        .collect();
    if observable.is_empty() {
        return fit_gains(seen, &seen.log, pairs, smooth).0;
    }
    // Log colours divided by the current falloff.
    let mut corrected = seen.log.clone();
    for _ in 0..VIGNETTE_ROUNDS {
        let (x, w) = fit_gains(seen, &corrected, pairs, smooth);
        for &cam in &observable {
            let fitted = fit_falloff(seen, &x, &w, cam, vignetting, c);
            vignetting.coeffs[cam][c] = fitted;
        }
        for (k, l) in corrected.iter_mut().enumerate() {
            *l = seen.log[k] - vignetting.log_factor(seen.camera(k), c, f64::from(seen.theta[k]));
        }
    }
    fit_gains(seen, &corrected, pairs, smooth).0
}

/// Whether the falloff of camera `cam` can be told apart from the gains and the albedo: at
/// least [`VIGNETTE_MIN_OBSERVATIONS`] observations, the angles of its views spread over
/// [`VIGNETTE_MIN_SPREAD`] and their deviations from the mean angle of the same point's views
/// of this camera over [`VIGNETTE_MIN_POINT_SPREAD`] (5 % to 95 % quantile). Only the second
/// shows the falloff: a camera that sees every point at the same angle in all its images
/// (standing still) cannot.
fn observable(seen: &Seen, cam: usize) -> bool {
    let views = (0..seen.image.len())
        .filter(|&k| seen.camera(k) == cam)
        .count();
    let mut theta: Vec<f64> = Vec::with_capacity(views);
    let mut deviation: Vec<f64> = Vec::with_capacity(views);
    for j in 0..seen.points() {
        let start = theta.len();
        let range = seen.range(j);
        theta.extend(
            range
                .filter(|&k| seen.camera(k) == cam)
                .map(|k| f64::from(seen.theta[k])),
        );
        let mine = &theta[start..];
        if mine.len() >= 2 {
            let mean = mine.iter().sum::<f64>() / mine.len() as f64;
            deviation.extend(mine.iter().map(|t| t - mean));
        }
    }
    let spread = |v: &mut Vec<f64>| {
        v.sort_by(f64::total_cmp);
        let q = |p: f64| v[((v.len() - 1) as f64 * p).round() as usize];
        q(0.95) - q(0.05)
    };
    theta.len() >= VIGNETTE_MIN_OBSERVATIONS
        && deviation.len() >= 2
        && spread(&mut theta) >= VIGNETTE_MIN_SPREAD
        && spread(&mut deviation) >= VIGNETTE_MIN_POINT_SPREAD
}

/// New `[b1, b2]` of camera `cam` in channel `c` for the log gains `x` and weights `w`, the
/// other cameras' falloff (`vignetting`, clamped as it is applied) held: weighted least squares on
/// `log c − log g_i − log a_j`, with `a_j` eliminated as in the gain fit. Together with them
/// a constant log offset of the camera's images is fitted and dropped: the gains of (a) take
/// up the camera's mean falloff, and without the offset (b) would only see the rest, so the
/// alternation would approach the falloff slowly. Where the offset is not observable (no
/// point seen by another camera too) only `b1, b2` are fitted. Unchanged if the system is
/// singular.
fn fit_falloff(
    seen: &Seen,
    x: &[f64],
    w: &[f64],
    cam: usize,
    vignetting: &Vignetting,
    c: usize,
) -> [f64; 2] {
    // Basis per view: offset, θ², θ⁴ of this camera (0 for the other cameras' views).
    let basis = |t: f64| {
        let t2 = t * t;
        [1.0, t2, t2 * t2]
    };
    let (mut a, mut r) = ([[0.0f64; 3]; 3], [0.0f64; 3]);
    let mut psi: Vec<[f64; 3]> = Vec::new();
    let mut rest: Vec<f64> = Vec::new();
    let mut weight = 0.0;
    for j in 0..seen.points() {
        let range = seen.range(j);
        if !range.clone().any(|k| seen.camera(k) == cam) {
            continue;
        }
        // Per view: the residual of the full model with this camera's falloff left out, and
        // this camera's basis; both centred on the point's weighted mean (the elimination of
        // `a_j`).
        psi.clear();
        rest.clear();
        let ws: f64 = w[range.clone()].iter().sum();
        let (mut mean_rest, mut mean_psi) = (0.0, [0.0; 3]);
        for k in range.clone() {
            let theta = f64::from(seen.theta[k]);
            let (p, model) = if seen.camera(k) == cam {
                (basis(theta), 0.0)
            } else {
                ([0.0; 3], vignetting.log_factor(seen.camera(k), c, theta))
            };
            let e = seen.log[k] - x[seen.image[k] as usize] - model;
            mean_rest += w[k] * e;
            for (m, p) in mean_psi.iter_mut().zip(p) {
                *m += w[k] * p;
            }
            psi.push(p);
            rest.push(e);
        }
        mean_rest /= ws;
        mean_psi = mean_psi.map(|m| m / ws);
        for ((p, e), &wk) in psi.iter().zip(&rest).zip(&w[range]) {
            let p: [f64; 3] = std::array::from_fn(|u| p[u] - mean_psi[u]);
            let e = e - mean_rest;
            weight += wk;
            for u in 0..3 {
                r[u] += wk * p[u] * e;
                for v in 0..3 {
                    a[u][v] += wk * p[u] * p[v];
                }
            }
        }
    }
    // The offset counts as observable if its centred basis carries at least 1 % of the
    // weight (views of points shared with another camera).
    if a[0][0] >= 0.01 * weight {
        if let Some(b) = solve3(&a, &r) {
            return [b[1], b[2]];
        }
    }
    let det = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    if det.is_nan() || det <= 1e-12 * a[1][1] * a[2][2] {
        return vignetting.coeffs[cam][c];
    }
    [
        (a[2][2] * r[1] - a[1][2] * r[2]) / det,
        (a[1][1] * r[2] - a[2][1] * r[1]) / det,
    ]
}

/// Solution of the symmetric 3 × 3 system `a x = r` by Cramer's rule; `None` if it is
/// (nearly) singular.
fn solve3(a: &[[f64; 3]; 3], r: &[f64; 3]) -> Option<[f64; 3]> {
    let det3 = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let det = det3(a);
    if det.is_nan() || det <= 1e-12 * a[0][0] * a[1][1] * a[2][2] {
        return None;
    }
    Some(std::array::from_fn(|col| {
        let mut m = *a;
        for (row, v) in m.iter_mut().zip(r) {
            row[col] = *v;
        }
        det3(&m) / det
    }))
}

/// Log gains (mean 0) for the log colours `log` of the observations of `seen`, and the Huber
/// weights of the last round.
fn fit_gains(
    seen: &Seen,
    log: &[f64],
    pairs: &[(usize, usize)],
    smooth: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = seen.images();
    let mut w = vec![1.0; log.len()];
    if seen.points() == 0 || n == 0 {
        return (vec![0.0; n], w);
    }
    let (starts, image) = (&seen.starts, &seen.image);
    // Least squares, then re-weighted rounds with Huber weights on the residuals: views that
    // disagree with the fit (misregistered edges, highlights, small occlusion errors) lose
    // influence.
    let mut x = solve_weighted(n, starts, image, log, &w, pairs, smooth);
    for _ in 0..ROBUST_ROUNDS {
        for j in 0..starts.len() - 1 {
            let r = starts[j]..starts[j + 1];
            let ws: f64 = w[r.clone()].iter().sum();
            let log_a = r
                .clone()
                .map(|k| w[k] * (log[k] - x[image[k] as usize]))
                .sum::<f64>()
                / ws;
            for k in r {
                let e = (log[k] - x[image[k] as usize] - log_a).abs();
                w[k] = if e <= HUBER_DELTA {
                    1.0
                } else {
                    HUBER_DELTA / e
                };
            }
        }
        x = solve_weighted(n, starts, image, log, &w, pairs, smooth);
    }
    let mean = x.iter().sum::<f64>() / n as f64;
    (x.iter().map(|x| x - mean).collect(), w)
}

/// Solves the weighted problem: point `j`'s observations are `starts[j]..starts[j + 1]`, of
/// images `image` with log colours `log` and weights `w`.
fn solve_weighted(
    n: usize,
    starts: &[usize],
    image: &[u32],
    log: &[f64],
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
        let mean = r.clone().map(|k| w[k] * log[k]).sum::<f64>() / ws;
        for k in r {
            let (i, wk) = (image[k] as usize, w[k]);
            b[i] += wk * (log[k] - mean);
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
            let mean = r.clone().map(|k| w[k] * x[image[k] as usize]).sum::<f64>() / ws;
            for k in r {
                let i = image[k] as usize;
                out[i] += w[k] * (x[i] - mean);
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
    let eligible: Vec<bool> = slots
        .par_chunks(k)
        .map(|s| s.iter().filter(|v| v.weight > 0.0).count() >= 2)
        .collect();
    let m = eligible.iter().filter(|&&e| e).count();
    let take = m.min(samples);
    // The `s`-th sample is the `s · m / take`-th eligible point; one pass in cloud order.
    let mut picked: Vec<usize> = Vec::with_capacity(take);
    let mut nth = 0usize;
    for (j, &e) in eligible.iter().enumerate() {
        if picked.len() == take {
            break;
        }
        if e {
            if nth == picked.len() * m / take {
                picked.push(j);
            }
            nth += 1;
        }
    }
    drop(eligible);
    let count = |j: usize| {
        slots[j * k..(j + 1) * k]
            .iter()
            .filter(|v| v.weight > 0.0)
            .count()
    };
    let total = picked.iter().map(|&j| count(j)).sum();
    let mut obs = Observations::new(cameras);
    obs.starts.reserve_exact(take);
    obs.views.reserve_exact(total);
    let mut views = Vec::with_capacity(k);
    for j in picked {
        views.clear();
        views.extend(
            slots[j * k..(j + 1) * k]
                .iter()
                .filter(|v| v.weight > 0.0)
                .map(|v| ViewObs {
                    image: v.rank,
                    rgb: v.linear(),
                    theta: v.theta(),
                }),
        );
        obs.push_point(&views);
    }
    obs
}

/// Colour of view `v` divided by its image's gain and its camera's falloff at the view's
/// angle; `cameras[rank]` is the camera of each image.
fn corrected(v: &View, gains: &[[f32; 3]], vignetting: &Vignetting, cameras: &[usize]) -> [f32; 3] {
    let i = v.rank as usize;
    let (g, cam, theta) = (&gains[i], cameras[i], v.theta());
    let c = v.linear();
    std::array::from_fn(|ch| c[ch] / (g[ch] * vignetting.factor(cam, ch, theta)))
}

/// Divides every kept view's colour by its image's gain and its camera's falloff.
pub(crate) fn apply_exposure(
    slots: &mut [View],
    gains: &[[f32; 3]],
    vignetting: &Vignetting,
    cameras: &[usize],
) {
    slots
        .par_iter_mut()
        .filter(|v| v.weight > 0.0)
        .for_each(|v| {
            let c = corrected(v, gains, vignetting, cameras);
            v.set_linear(c);
        });
}

/// Width of the angle classes of [`vignette_residual`] (rad, 10°).
const RESIDUAL_CLASS: f64 = 10.0 * std::f64::consts::PI / 180.0;
/// Classes up to 180°.
const RESIDUAL_CLASSES: usize = 18;

/// Remaining edge falloff after dividing every view by its gain and falloff: per view of a
/// point with at least 2 usable views (raw colour within [`DARK`] to [`SATURATED`] in every
/// channel), the log of its luminance (Rec. 709) minus the mean over the point's views; per
/// camera and 10° class of the view's angle with at least [`VIGNETTE_MIN_OBSERVATIONS`] views
/// the mean of these, in % (100 × log); per camera the largest absolute class mean, averaged
/// over the cameras with such a class. 0 without any.
pub(crate) fn vignette_residual(
    slots: &[View],
    k: usize,
    gains: &[[f32; 3]],
    vignetting: &Vignetting,
    cameras: &[usize],
) -> f64 {
    let n = cameras.iter().max().map_or(0, |&c| c + 1);
    let classes = n * RESIDUAL_CLASSES;
    // Per chunk of points: (summed residual, views) per camera and class; summed in order.
    let partial: Vec<Vec<(f64, usize)>> = slots
        .par_chunks(k * SUM_CHUNK)
        .map(|chunk| {
            let mut sums = vec![(0.0f64, 0usize); classes];
            let mut views: Vec<(usize, f64)> = Vec::with_capacity(k);
            for s in chunk.chunks(k) {
                views.clear();
                views.extend(
                    s.iter()
                        .filter(|v| {
                            v.weight > 0.0
                                && v.linear().iter().all(|c| (DARK..=SATURATED).contains(c))
                        })
                        .map(|v| {
                            let c = corrected(v, gains, vignetting, cameras);
                            let lum = 0.2126 * f64::from(c[0])
                                + 0.7152 * f64::from(c[1])
                                + 0.0722 * f64::from(c[2]);
                            let class = ((f64::from(v.theta()) / RESIDUAL_CLASS) as usize)
                                .min(RESIDUAL_CLASSES - 1);
                            (
                                cameras[v.rank as usize] * RESIDUAL_CLASSES + class,
                                lum.ln(),
                            )
                        }),
                );
                if views.len() < 2 {
                    continue;
                }
                let mean = views.iter().map(|v| v.1).sum::<f64>() / views.len() as f64;
                for &(class, l) in &views {
                    sums[class].0 += l - mean;
                    sums[class].1 += 1;
                }
            }
            sums
        })
        .collect();
    let mut sums = vec![(0.0f64, 0usize); classes];
    for chunk in &partial {
        for (t, p) in sums.iter_mut().zip(chunk) {
            t.0 += p.0;
            t.1 += p.1;
        }
    }
    let worst: Vec<f64> = sums
        .chunks(RESIDUAL_CLASSES.max(1))
        .filter_map(|cam| {
            cam.iter()
                .filter(|c| c.1 >= VIGNETTE_MIN_OBSERVATIONS)
                .map(|c| (100.0 * c.0 / c.1 as f64).abs())
                .reduce(f64::max)
        })
        .collect();
    if worst.is_empty() {
        0.0
    } else {
        worst.iter().sum::<f64>() / worst.len() as f64
    }
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
                .map(|v| v.linear().map(srgb_value))
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

/// Systematic colour difference between the cameras on the points they both see.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct CameraDifference {
    /// sRGB units (0–255), see [`camera_difference`].
    pub(crate) value: f64,
    /// Points with views from at least 2 cameras (counted once per camera pair).
    pub(crate) points: usize,
}

/// Systematic colour difference between the cameras: for every camera pair and every point
/// with views from both, the difference of the two cameras' mean view colours per channel
/// (sRGB, 0–255); per pair the absolute value of its mean over the points, averaged over the
/// channels; then averaged over the pairs, weighted by their points. Unlike the spread, the
/// scatter of single views averages out and what remains is the offset an exposure or white
/// balance difference between the cameras causes. `cameras[rank]` is the camera of each
/// image; 0 without such points.
pub(crate) fn camera_difference(slots: &[View], k: usize, cameras: &[usize]) -> CameraDifference {
    let n = cameras.iter().max().map_or(0, |&c| c + 1);
    // Per chunk of points: (summed difference per channel, points) of every camera pair
    // `a * n + b`; chunks are summed in order, so the result is the same in every run.
    let partial: Vec<Vec<([f64; 3], usize)>> = slots
        .par_chunks(k * SUM_CHUNK)
        .map(|chunk| {
            let mut pairs = vec![([0.0f64; 3], 0usize); n * n];
            let mut sum = vec![([0.0f64; 3], 0usize); n];
            for s in chunk.chunks(k) {
                sum.fill(([0.0; 3], 0));
                for v in s.iter().filter(|v| v.weight > 0.0) {
                    let e = &mut sum[cameras[v.rank as usize]];
                    for (a, c) in e.0.iter_mut().zip(v.linear()) {
                        *a += srgb_value(c);
                    }
                    e.1 += 1;
                }
                for a in 0..n {
                    for b in a + 1..n {
                        let ((sa, na), (sb, nb)) = (sum[a], sum[b]);
                        if na > 0 && nb > 0 {
                            let e = &mut pairs[a * n + b];
                            for c in 0..3 {
                                e.0[c] += sa[c] / na as f64 - sb[c] / nb as f64;
                            }
                            e.1 += 1;
                        }
                    }
                }
            }
            pairs
        })
        .collect();
    let mut pairs = vec![([0.0f64; 3], 0usize); n * n];
    for chunk in &partial {
        add_sums(&mut pairs, chunk);
    }
    let points: usize = pairs.iter().map(|p| p.1).sum();
    if points == 0 {
        return CameraDifference::default();
    }
    let value = pairs
        .iter()
        .filter(|p| p.1 > 0)
        .map(|(sum, m)| sum.iter().map(|s| (s / *m as f64).abs()).sum::<f64>() / 3.0 * *m as f64)
        .sum::<f64>()
        / points as f64;
    CameraDifference { value, points }
}

/// Points per chunk of the colour metrics' partial sums.
const SUM_CHUNK: usize = 1 << 16;

/// Adds `part` to `total` entry by entry.
fn add_sums(total: &mut [([f64; 3], usize)], part: &[([f64; 3], usize)]) {
    for (t, p) in total.iter_mut().zip(part) {
        for c in 0..3 {
            t.0[c] += p.0[c];
        }
        t.1 += p.1;
    }
}

/// Mean colour offset of the images: every view of a point with at least 2 views differs from
/// the mean of that point's views (sRGB, 0–255, per channel); per image the mean of these
/// differences over its views, as an absolute value averaged over the channels; then averaged
/// over the images, weighted by their views. The scatter of single views averages out within
/// an image; what remains is how much brighter or darker (or tinted) the image as a whole
/// shows the scan, from the camera or from the exposure at the time. Images are numbered by
/// the views' rank, `images` of them; 0 without such points.
pub(crate) fn image_offset(slots: &[View], k: usize, images: usize) -> f64 {
    // Per chunk of points: (summed difference to the point's mean per channel, views) of
    // every image; chunks are summed in order.
    let partial: Vec<Vec<([f64; 3], usize)>> = slots
        .par_chunks(k * SUM_CHUNK)
        .map(|chunk| {
            let mut sums = vec![([0.0f64; 3], 0usize); images];
            let mut views: Vec<(u32, [f64; 3])> = Vec::with_capacity(k);
            for s in chunk.chunks(k) {
                views.clear();
                views.extend(
                    s.iter()
                        .filter(|v| v.weight > 0.0)
                        .map(|v| (v.rank, v.linear().map(srgb_value))),
                );
                if views.len() < 2 {
                    continue;
                }
                let m = views.len() as f64;
                let mean: [f64; 3] =
                    std::array::from_fn(|c| views.iter().map(|v| v.1[c]).sum::<f64>() / m);
                for (i, col) in &views {
                    let e = &mut sums[*i as usize];
                    for c in 0..3 {
                        e.0[c] += col[c] - mean[c];
                    }
                    e.1 += 1;
                }
            }
            sums
        })
        .collect();
    let mut sums = vec![([0.0f64; 3], 0usize); images];
    for chunk in &partial {
        add_sums(&mut sums, chunk);
    }
    let views: usize = sums.iter().map(|s| s.1).sum();
    if views == 0 {
        return 0.0;
    }
    sums.iter()
        .filter(|s| s.1 > 0)
        .map(|(sum, _)| sum.iter().map(|s| s.abs()).sum::<f64>() / 3.0)
        .sum::<f64>()
        / views as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view(image: u32, rgb: [f32; 3]) -> ViewObs {
        ViewObs {
            image,
            rgb,
            theta: 0.0,
        }
    }

    #[test]
    fn two_images_split_the_ratio() {
        // Image 1 sees everything twice as bright: gains 1/√2 and √2 (smoothness pulls a
        // little, 200 observations against weight 1).
        let mut obs = Observations::new(vec![0, 0]);
        for j in 0..100 {
            let a = 0.1 + 0.002 * j as f32;
            obs.push_point(&[view(0, [a; 3]), view(1, [2.0 * a; 3])]);
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
            obs.push_point(&[view(0, [0.6, a, 0.001]), view(1, [1.0, 1.5 * a, 0.3])]);
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
            obs.push_point(&[view(0, [a; 3]), view(1, [ratio * a; 3])]);
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
            obs.push_point(&[view(0, [a; 3]), view(2, [4.0 * a; 3])]);
        }
        let g = estimate_gains(&obs, &ExposureConfig::default());
        // In the log midway: gain 1 (the mean of the three logs is 0).
        assert!((g[1][0] - 1.0).abs() < 1e-3, "{g:?}");
        assert!(g[0][0] < 0.6 && g[2][0] > 1.7, "{g:?}");
    }
}
