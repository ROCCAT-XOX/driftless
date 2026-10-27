//! A point cloud loaded from LAS, LAZ, E57, PLY or PCD ([`driftless_export::read_points`]) for
//! the 3D view: columns relative to an origin in whole metres, so that `f32` keeps millimetres
//! even for coordinates of ~10⁶ m.

use crate::lod::{build_octree, Octree};
use driftless_export::{Layout, PointFormat, PointSink, Points, ReadEnd, ReadStart};
use driftless_io::progress::Progress;
use driftless_slam::report::SlamReport;
use driftless_steps::overlay::submap_at;
use driftless_steps::StepError;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Intensity shown when a file carries none (all zero, as `map.las` of a SLAM run).
pub const MID_GRAY: u8 = 128;

/// The points of a cloud, one entry per point in every column.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PointSet {
    /// Centre of the hull, rounded to whole metres.
    pub origin: [f64; 3],
    /// Position relative to `origin`.
    pub xyz: Vec<[f32; 3]>,
    pub rgb: Option<Vec<[u8; 3]>>,
    pub intensity: Vec<u8>,
    /// Seconds since the start of the scan; without a SLAM report since the first point.
    pub t: Vec<f32>,
    /// Quality of the point's submap × 255; 255 without a report.
    pub quality: Vec<u8>,
    /// Whether `quality` comes from a SLAM report; without one, or for a file without point
    /// times (e.g. a run's `map.las`), it is unknown (all 255).
    pub quality_known: bool,
    /// LAS `user_data`: 1 = without measured colour, or the removal reason.
    pub flags: Vec<u8>,
    /// The scan of each point in a model merged from several (LAS `point_source_id`, PLY and
    /// PCD `scan`, the `data3D` entry of an E57); empty if unknown or if all points have the
    /// same.
    pub source: Vec<u16>,
}

impl PointSet {
    /// Points without colour, time or flags: mid gray, quality 255 (tests, synthetic clouds).
    pub fn from_xyz(origin: [f64; 3], xyz: Vec<[f32; 3]>) -> Self {
        let n = xyz.len();
        PointSet {
            origin,
            xyz,
            rgb: None,
            intensity: vec![MID_GRAY; n],
            t: vec![0.0; n],
            quality: vec![255; n],
            quality_known: false,
            flags: vec![0; n],
            source: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.xyz.len()
    }

    pub fn is_empty(&self) -> bool {
        self.xyz.is_empty()
    }

    /// Per-axis minimum and maximum of `xyz`; zero for an empty set.
    pub fn bounds(&self) -> ([f32; 3], [f32; 3]) {
        if self.xyz.is_empty() {
            return ([0.0; 3], [0.0; 3]);
        }
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for p in &self.xyz {
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        (min, max)
    }
}

/// The quality × 255 of the submap of the time `t` (seconds since the scan start); 255
/// without submaps.
fn quality_at(submaps: &[driftless_slam::report::SubmapReport], t: f64) -> u8 {
    submaps
        .get(submap_at(submaps, t))
        .map_or(255, |m| (m.quality.clamp(0.0, 1.0) * 255.0).round() as u8)
}

/// The [`PointSet`] of `points` read by [`driftless_export`], for every format alike:
/// - origin = per axis `round((min + max) / 2)` of the points' bounds (0 without points);
/// - colour and intensity `>> 8`; without intensity, or all 0, [`MID_GRAY`];
/// - with times: `t` since the scan start of `slam` (else since the first point), and with
///   `slam` the quality of each point's submap ([`submap_at`]); without times `t` = 0 and the
///   quality unknown (255);
/// - flags as they are (0 without), the scan number if it differs between points.
///
/// The columns are converted one after the other and each is freed right after.
pub fn point_set(points: Points, slam: Option<&SlamReport>) -> PointSet {
    let n = points.len();
    let origin = points
        .bounds()
        .map(|(min, max)| std::array::from_fn(|k| ((min[k] + max[k]) / 2.0).round()))
        .filter(|o: &[f64; 3]| o.iter().all(|v| v.is_finite()))
        .unwrap_or([0.0; 3]);
    let Points {
        xyz,
        rgb,
        intensity,
        source,
        time,
        flags,
        scans: _,
    } = points;
    // `iter` and `drop`, not `into_iter`: a collect in place could keep the larger buffer.
    let rel: Vec<[f32; 3]> = xyz
        .iter()
        .map(|p| std::array::from_fn(|k| (p[k] - origin[k]) as f32))
        .collect();
    drop(xyz);
    let rgb = rgb.map(|c| {
        let eight: Vec<[u8; 3]> = c.iter().map(|v| v.map(|c| (c >> 8) as u8)).collect();
        drop(c);
        eight
    });
    let intensity = match intensity {
        Some(i) if i.iter().any(|&v| v != 0) => {
            let eight: Vec<u8> = i.iter().map(|&v| (v >> 8) as u8).collect();
            drop(i);
            eight
        }
        _ => vec![MID_GRAY; n],
    };
    let (t, quality, quality_known) = match time {
        Some(time) => {
            let t0 = slam
                .map(|s| s.scan_start)
                .or_else(|| time.first().copied())
                .unwrap_or(0.0);
            let submaps = slam.map_or(&[][..], |s| &s.submaps[..]);
            let quality = time
                .iter()
                .map(|&g| {
                    if slam.is_some() {
                        quality_at(submaps, g - t0)
                    } else {
                        255
                    }
                })
                .collect();
            let t = time.iter().map(|&g| (g - t0) as f32).collect();
            drop(time);
            (t, quality, slam.is_some())
        }
        None => (vec![0.0; n], vec![255; n], false),
    };
    let flags = flags.unwrap_or_else(|| vec![0; n]);
    let source = source
        .filter(|s| s.windows(2).any(|w| w[0] != w[1]))
        .unwrap_or_default();
    PointSet {
        origin,
        xyz: rel,
        rgb,
        intensity,
        t,
        quality,
        quality_known,
        flags,
        source,
    }
}

/// Reads the cloud at `path` in any format [`driftless_export::read_points`] knows, by its
/// extension (LAS, LAZ, E57, PLY, PCD; phase [`driftless_export::PHASE_READ`], a cancel check
/// per block), as a [`PointSet`] ([`point_set`]). With `slam`, points with times get the
/// quality of their submap. Errors carry the reader's German message and name the file.
///
/// LAS, LAZ and E57 go block by block straight into the [`PointSet`] ([`Streamed`]): at the
/// peak about 28 B per point instead of the 43 B of [`Points`] plus the columns being
/// converted. PLY and PCD scale their intensities by the maximum of the whole file and go
/// through [`Points`].
pub fn load_points(
    path: &Path,
    slam: Option<&SlamReport>,
    progress: &Progress,
) -> Result<PointSet, StepError> {
    let mut sink = Streamed::new(slam);
    let end = match PointFormat::from_path(path) {
        Some(PointFormat::Las | PointFormat::Laz) => {
            driftless_export::las::read_las_into(path, &mut sink, progress)?
        }
        Some(PointFormat::E57) => driftless_export::e57::read_e57_into(path, &mut sink, progress)?,
        _ => {
            let points = driftless_export::read_points(path, progress)?;
            return Ok(point_set(points, slam));
        }
    };
    Ok(sink.finish(&end))
}

/// Points of the first block checked against the bounds a file states.
const ORIGIN_SAMPLE: usize = 16;

/// Builds a [`PointSet`] from the blocks of a streaming reader, with the same result as
/// [`point_set`] of the whole [`Points`]:
/// - positions relative to the centre of the stated bounds (if the first points lie inside
///   them) or to the first point, in whole metres; at the end shifted to the rounded centre
///   of the real bounds if that differs (an exact whole-metre step);
/// - colour and intensity as their high byte, and their low byte as long as no value above
///   255 has come, for a file that turns out to hold 8-bit values ([`ReadEnd`]).
struct Streamed<'a> {
    slam: Option<&'a SlamReport>,
    set: PointSet,
    layout: Layout,
    stated: Option<([f64; 3], [f64; 3])>,
    /// Origin of `set.xyz` while reading; `None` before the first point.
    origin: Option<[f64; 3]>,
    min: [f64; 3],
    max: [f64; 3],
    t0: Option<f64>,
    low_rgb: Option<Vec<[u8; 3]>>,
    low_intensity: Option<Vec<u8>>,
    any_intensity: bool,
}

impl<'a> Streamed<'a> {
    fn new(slam: Option<&'a SlamReport>) -> Self {
        Streamed {
            slam,
            set: PointSet::default(),
            layout: Layout::default(),
            stated: None,
            origin: None,
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
            t0: slam.map(|s| s.scan_start),
            low_rgb: None,
            low_intensity: None,
            any_intensity: false,
        }
    }

    /// The origin for the first block: the centre of the stated bounds if they are finite,
    /// not all zero and hold the first points (slack for the LAS rounding), else the first
    /// point, in whole metres.
    fn first_origin(&self, first: &[[f64; 3]]) -> [f64; 3] {
        const SLACK: f64 = 0.01;
        let p0 = first[0];
        match self.stated {
            Some((min, max))
                if min.iter().chain(&max).all(|v| v.is_finite())
                    && min.iter().chain(&max).any(|&v| v != 0.0)
                    && first.iter().take(ORIGIN_SAMPLE).all(|p| {
                        (0..3).all(|k| p[k] >= min[k] - SLACK && p[k] <= max[k] + SLACK)
                    }) =>
            {
                std::array::from_fn(|k| ((min[k] + max[k]) / 2.0).round())
            }
            _ => p0.map(|v| if v.is_finite() { v.round() } else { 0.0 }),
        }
    }

    fn finish(mut self, end: &ReadEnd) -> PointSet {
        let set = &mut self.set;
        let n = set.xyz.len();
        // the origin of `point_set`: the rounded centre of the real bounds
        let origin: [f64; 3] = std::array::from_fn(|k| ((self.min[k] + self.max[k]) / 2.0).round());
        let origin = if n > 0 && origin.iter().all(|v| v.is_finite()) {
            origin
        } else {
            [0.0; 3]
        };
        let read = self.origin.unwrap_or(origin);
        if read != origin {
            let shift: [f32; 3] = std::array::from_fn(|k| (read[k] - origin[k]) as f32);
            for p in &mut set.xyz {
                for k in 0..3 {
                    p[k] += shift[k];
                }
            }
        }
        set.origin = origin;
        if end.rgb_eight_bit {
            if let (Some(rgb), Some(low)) = (&mut set.rgb, self.low_rgb.take()) {
                *rgb = low;
            }
        }
        if end.intensity_eight_bit {
            if let Some(low) = self.low_intensity.take() {
                set.intensity = low;
            }
        }
        if !self.layout.intensity || !self.any_intensity {
            set.intensity.clear();
            set.intensity.resize(n, MID_GRAY);
        }
        set.quality_known = self.layout.time && self.slam.is_some();
        if set.source.windows(2).all(|w| w[0] == w[1]) {
            set.source = Vec::new();
        }
        self.set
    }
}

impl PointSink for Streamed<'_> {
    fn start(&mut self, start: &ReadStart) {
        let n = start.capacity;
        let layout = start.layout;
        self.layout = layout;
        self.stated = start.bounds;
        self.set = PointSet {
            origin: [0.0; 3],
            xyz: Vec::with_capacity(n),
            rgb: layout.rgb.then(|| Vec::with_capacity(n)),
            intensity: Vec::with_capacity(n),
            t: Vec::with_capacity(n),
            quality: Vec::with_capacity(n),
            quality_known: false,
            flags: Vec::with_capacity(n),
            source: Vec::with_capacity(if layout.source { n } else { 0 }),
        };
        self.low_rgb = layout.rgb.then(|| Vec::with_capacity(n));
        self.low_intensity = layout.intensity.then(|| Vec::with_capacity(n));
    }

    fn block(&mut self, block: &Points) {
        if block.xyz.is_empty() {
            return;
        }
        let origin = match self.origin {
            Some(o) => o,
            None => *self.origin.insert(self.first_origin(&block.xyz)),
        };
        let set = &mut self.set;
        for p in &block.xyz {
            for (k, &v) in p.iter().enumerate() {
                self.min[k] = self.min[k].min(v);
                self.max[k] = self.max[k].max(v);
            }
            set.xyz
                .push(std::array::from_fn(|k| (p[k] - origin[k]) as f32));
        }
        if let (Some(rgb), Some(c)) = (&mut set.rgb, &block.rgb) {
            rgb.extend(c.iter().map(|v| v.map(|c| (c >> 8) as u8)));
            if c.iter().any(|v| v.iter().any(|&c| c > 255)) {
                self.low_rgb = None;
            }
            if let Some(low) = &mut self.low_rgb {
                low.extend(c.iter().map(|v| v.map(|c| c as u8)));
            }
        }
        match &block.intensity {
            Some(i) => {
                set.intensity.extend(i.iter().map(|&v| (v >> 8) as u8));
                self.any_intensity |= i.iter().any(|&v| v != 0);
                if i.iter().any(|&v| v > 255) {
                    self.low_intensity = None;
                }
                if let Some(low) = &mut self.low_intensity {
                    low.extend(i.iter().map(|&v| v as u8));
                }
            }
            None => set.intensity.resize(set.xyz.len(), MID_GRAY),
        }
        match &block.time {
            Some(time) => {
                let t0 = *self.t0.get_or_insert(time[0]);
                let submaps = self.slam.map_or(&[][..], |s| &s.submaps[..]);
                for &g in time {
                    set.t.push((g - t0) as f32);
                    set.quality.push(if self.slam.is_some() {
                        quality_at(submaps, g - t0)
                    } else {
                        255
                    });
                }
            }
            None => {
                set.t.resize(set.xyz.len(), 0.0);
                set.quality.resize(set.xyz.len(), 255);
            }
        }
        match &block.flags {
            Some(f) => set.flags.extend_from_slice(f),
            None => set.flags.resize(set.xyz.len(), 0),
        }
        if let Some(s) = &block.source {
            set.source.extend_from_slice(s);
        }
    }
}

/// A cloud ready for the 3D view: its points and their levels of detail.
///
/// The [`PointSet`] stays in memory next to the octree: the point info reads its columns
/// (time, colour, flags) through [`Octree::source`], and picking reads the positions of the
/// octree's vertices. That costs 22 B per point for the columns (19 B without colours) on
/// top of 24 B for the octree (vertex + source index), ≈ 2,3 GB for 50 million points; the
/// GPU holds another 20 B per point.
#[derive(Debug, Clone)]
pub struct LoadedCloud {
    pub path: PathBuf,
    pub points: PointSet,
    pub tree: Octree,
    pub load_seconds: f64,
    pub build_seconds: f64,
    /// What „alles zeigen“ frames ([`view_bounds`]); `None` without points.
    pub view_bounds: Option<([f32; 3], [f32; 3])>,
    /// Lowest and highest z of all points (relative), for the height cut.
    pub z_range: [f32; 2],
    /// A height (relative z) just below the ceiling ([`estimate_ceiling`]), if there is one.
    pub ceiling: Option<f32>,
    /// Spot diameter in metres for the world-size view ([`point_m_of_map`] of its folder).
    pub point_m: f32,
}

/// Spot diameter in metres when the map's grid is unknown.
pub const DEFAULT_POINT_M: f32 = 0.015;
/// Spot diameter per voxel of the dense grid: a little larger, so that spots overlap.
const POINT_M_PER_VOXEL: f64 = 1.5;

/// The spot diameter for the clouds of the map folder `dir`: its `map.json`'s
/// `settings.dense.voxel` × 1.5, else [`DEFAULT_POINT_M`].
pub fn point_m_of_map(dir: &Path) -> f32 {
    std::fs::read_to_string(dir.join("map.json"))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| v["settings"]["dense"]["voxel"].as_f64())
        .filter(|v| v.is_finite() && *v > 0.0)
        .map_or(DEFAULT_POINT_M, |v| (v * POINT_M_PER_VOXEL) as f32)
}

impl LoadedCloud {
    /// Builds the levels of detail of `points` (phase [`crate::lod::PHASE_BUILD`]).
    pub fn build(
        path: PathBuf,
        points: PointSet,
        load_seconds: f64,
        progress: &Progress,
    ) -> Result<Self, StepError> {
        let t = Instant::now();
        let tree = build_octree(&points, progress)?;
        // Computed here, in the job, so that the window does not wait for them.
        let (min, max) = points.bounds();
        Ok(LoadedCloud {
            view_bounds: view_bounds(&points),
            z_range: [min[2], max[2]],
            ceiling: estimate_ceiling(&points),
            // a bare file name (the synthetic cloud) has no map folder
            point_m: path
                .parent()
                .filter(|d| !d.as_os_str().is_empty())
                .map_or(DEFAULT_POINT_M, point_m_of_map),
            path,
            points,
            tree,
            load_seconds,
            build_seconds: t.elapsed().as_secs_f64(),
        })
    }

    /// Whether the file is the `removed.las` of a map, whose flags are removal reasons.
    pub fn is_removed(&self) -> bool {
        self.path.file_name().is_some_and(|n| n == "removed.las")
    }
}

/// Reads `path` ([`load_points`]) and builds its levels of detail: phase
/// [`driftless_export::PHASE_READ`], then [`crate::lod::PHASE_BUILD`]. With `run`, the
/// `slam.json` of that run folder gives each point with a time the quality of its submap;
/// without it (or unreadable, or for a file without point times) the quality is 255.
pub fn load_cloud(
    path: &Path,
    run: Option<&Path>,
    progress: &Progress,
) -> Result<LoadedCloud, StepError> {
    let report: Option<SlamReport> = run
        .and_then(|dir| std::fs::read_to_string(dir.join("slam.json")).ok())
        .and_then(|t| serde_json::from_str(&t).ok());
    let t = Instant::now();
    let points = load_points(path, report.as_ref(), progress)?;
    LoadedCloud::build(
        path.to_path_buf(),
        points,
        t.elapsed().as_secs_f64(),
        progress,
    )
}

/// Points looked at by [`view_bounds`] at most.
const BOUNDS_SAMPLE: usize = 200_000;
/// Share of the points cut off on each side of each axis by [`view_bounds`].
pub const BOUNDS_CUT: f64 = 0.005;

/// The box „alles zeigen“ frames: per axis from the 0,5 % to the 99,5 % point
/// ([`BOUNDS_CUT`]) of an evenly spread sample, so that a few far outliers do not shrink the
/// cloud to a dot. Up to 1 % of the points per axis may therefore lie outside the frame.
/// `None` without points.
pub fn view_bounds(points: &PointSet) -> Option<([f32; 3], [f32; 3])> {
    let n = points.len();
    if n == 0 {
        return None;
    }
    let step = n.div_ceil(BOUNDS_SAMPLE);
    let mut min = [0.0; 3];
    let mut max = [0.0; 3];
    for k in 0..3 {
        let mut v: Vec<f32> = points.xyz.iter().step_by(step).map(|p| p[k]).collect();
        v.sort_unstable_by(f32::total_cmp);
        let cut = ((v.len() as f64 * BOUNDS_CUT) as usize).min((v.len() - 1) / 2);
        min[k] = v[cut];
        max[k] = v[v.len() - 1 - cut];
    }
    Some((min, max))
}

/// Points looked at by [`estimate_ceiling`] at most.
const CEILING_SAMPLE: usize = 1_000_000;
/// Height of a bin of the z histogram (m).
const CEILING_BIN: f32 = 0.05;
/// Most bins (a taller cloud gets wider bins).
const CEILING_MAX_BINS: usize = 20_000;
/// A bin is part of a horizontal layer when it holds this many times the median bin …
const CEILING_DENSE: f64 = 4.0;
/// … and at least this share of the sample.
const CEILING_MIN_SHARE: f64 = 0.005;
/// How far below the layer the estimate lies (m).
const CEILING_MARGIN: f32 = 0.05;

/// A height just below the ceiling: the topmost dense horizontal layer of the z histogram
/// (5 cm bins over an evenly spread sample; dense = 4 × the median occupied bin and 0,5 %
/// of the points), minus 5 cm under its lowest bin. `None` if the cloud is flat or has no
/// dense layer above another one (nothing that looks like floor and ceiling).
pub fn estimate_ceiling(points: &PointSet) -> Option<f32> {
    let n = points.len();
    if n == 0 {
        return None;
    }
    let step = n.div_ceil(CEILING_SAMPLE);
    let z: Vec<f32> = points.xyz.iter().step_by(step).map(|p| p[2]).collect();
    let (lo, hi) = z
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), &v| {
            (a.min(v), b.max(v))
        });
    let span = hi - lo;
    if span.is_nan() || span <= 4.0 * CEILING_BIN {
        return None;
    }
    let bins = (((hi - lo) / CEILING_BIN).ceil() as usize).clamp(1, CEILING_MAX_BINS);
    let width = (hi - lo) / bins as f32;
    let mut count = vec![0u32; bins];
    for &v in &z {
        let b = (((v - lo) / width) as usize).min(bins - 1);
        count[b] += 1;
    }
    let mut occupied: Vec<u32> = count.iter().copied().filter(|&c| c > 0).collect();
    occupied.sort_unstable();
    let median = f64::from(occupied[occupied.len() / 2]);
    let threshold = (CEILING_DENSE * median).max(CEILING_MIN_SHARE * z.len() as f64);
    let dense = |b: usize| f64::from(count[b]) >= threshold;
    let top = (0..bins).rev().find(|&b| dense(b))?;
    let mut bottom = top;
    while bottom > 0 && dense(bottom - 1) {
        bottom -= 1;
    }
    // Another dense layer below (the floor) makes the top one a ceiling.
    if !(0..bottom).any(dense) {
        return None;
    }
    Some(lo + bottom as f32 * width - CEILING_MARGIN)
}

/// Gap (m) along x between the model and the scan shown beside it for point pairs.
pub const PAIR_GAP: f64 = 1.0;

/// The cloud of „Von Hand ausrichten“: the model and, beside it, one of its scans.
#[derive(Debug, Clone)]
pub struct PairCloud {
    pub cloud: LoadedCloud,
    /// Added to the points of the scan (absolute, in the frame of that scan) to place them
    /// beside the model: a point of the scan shown at `p` lies at `p − shift` in its frame.
    pub shift: [f64; 3],
}

/// Keeps the entries of `v` whose `keep` is true.
fn retain_by<T>(v: &mut Vec<T>, keep: &[bool]) {
    let mut k = keep.iter();
    v.retain(|_| *k.next().unwrap_or(&true));
}

/// Gray colours from intensities, for one part of a cloud whose other part has colours.
fn grays(intensity: &[u8]) -> impl Iterator<Item = [u8; 3]> + '_ {
    intensity.iter().map(|&i| [i; 3])
}

/// One point set of the points of a `model` (in the model frame, [`PointSet::source`] = the
/// number of their scan, all `lone` if the model has none) without those of scan `member`,
/// and of `scan` (in its own frame, from its map) shifted along x until its box starts
/// [`PAIR_GAP`] beyond the box of the rest of the model (both boxes as [`view_bounds`] frames
/// them, so that a few far points do not push the scan away); its points get `member` as
/// their scan. The origin is that of the model. Colours of only one part are completed with
/// grays of the intensity; times and flags are kept, qualities are unknown (255). Returns the
/// set and the shift ([`PairCloud::shift`]).
pub fn combine_pair_points(
    mut model: PointSet,
    lone: u16,
    mut scan: PointSet,
    member: u16,
) -> (PointSet, [f64; 3]) {
    if model.source.is_empty() {
        model.source = vec![lone; model.len()];
    }
    if model.source.contains(&member) {
        let keep: Vec<bool> = model.source.iter().map(|&s| s != member).collect();
        retain_by(&mut model.xyz, &keep);
        if let Some(rgb) = &mut model.rgb {
            retain_by(rgb, &keep);
        }
        retain_by(&mut model.intensity, &keep);
        retain_by(&mut model.t, &keep);
        retain_by(&mut model.quality, &keep);
        retain_by(&mut model.flags, &keep);
        retain_by(&mut model.source, &keep);
    }
    if model.is_empty() {
        model.origin = scan.origin;
    }
    let mut shift = [0.0; 3];
    if let (Some((_, max)), Some((min, _))) = (view_bounds(&model), view_bounds(&scan)) {
        let model_end = model.origin[0] + f64::from(max[0]);
        let scan_start = scan.origin[0] + f64::from(min[0]);
        shift[0] = model_end + PAIR_GAP - scan_start;
    }
    let rel: [f64; 3] = [0, 1, 2].map(|k| scan.origin[k] + shift[k] - model.origin[k]);
    let n = scan.len();
    let mut out = model;
    out.xyz.reserve_exact(n);
    out.xyz.extend(
        scan.xyz
            .iter()
            .map(|p| [0, 1, 2].map(|k| (f64::from(p[k]) + rel[k]) as f32)),
    );
    scan.xyz = Vec::new();
    out.rgb = match (out.rgb.take(), scan.rgb.take()) {
        (None, None) => None,
        (Some(mut a), b) => {
            a.reserve_exact(n);
            match b {
                Some(b) => a.extend(b),
                None => a.extend(grays(&scan.intensity)),
            }
            Some(a)
        }
        (None, Some(b)) => {
            let mut a: Vec<[u8; 3]> = Vec::with_capacity(out.intensity.len() + n);
            a.extend(grays(&out.intensity));
            a.extend(b);
            Some(a)
        }
    };
    out.intensity.reserve_exact(n);
    out.intensity.extend(&scan.intensity);
    out.t.reserve_exact(n);
    out.t.extend(&scan.t);
    out.quality = vec![255; out.len()];
    out.quality_known = false;
    out.flags.reserve_exact(n);
    out.flags.extend(&scan.flags);
    out.source.reserve_exact(n);
    out.source.resize(out.len(), member);
    (out, shift)
}

/// Reads the `modell.las` of a model and the cloud `scan_cloud` of its scan `member` (any
/// format of [`load_points`], in the frame of that scan), sets them side by side
/// ([`combine_pair_points`]; `lone` is the scan of a model with points of only one) and builds
/// the levels of detail; `name` names the cloud. In a job: phases
/// [`driftless_export::PHASE_READ`] (twice) and [`crate::lod::PHASE_BUILD`], each with cancel
/// checks.
pub fn load_pair_cloud(
    name: PathBuf,
    model_las: &Path,
    lone: u16,
    scan_cloud: &Path,
    member: u16,
    progress: &Progress,
) -> Result<PairCloud, StepError> {
    let t = Instant::now();
    let model = load_points(model_las, None, progress)?;
    let scan = load_points(scan_cloud, None, progress)?;
    let (points, shift) = combine_pair_points(model, lone, scan, member);
    let cloud = LoadedCloud::build(name, points, t.elapsed().as_secs_f64(), progress)?;
    Ok(PairCloud { cloud, shift })
}
