//! A point cloud loaded from LAS or PLY ([`driftless_export::ply`]) for the 3D view: columns
//! relative to an origin in whole metres, so that `f32` keeps millimetres even for
//! coordinates of ~10⁶ m.

use crate::lod::{build_octree, Octree};
use driftless_export::Points;
use driftless_io::progress::Progress;
use driftless_slam::report::SlamReport;
use driftless_steps::overlay::submap_at;
use driftless_steps::StepError;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Points read per batch (one cancel check and one progress step each).
const BATCH: usize = 1 << 20;

/// Intensity shown when a file carries none (all zero, as `map.las` of a SLAM run).
pub const MID_GRAY: u8 = 128;

/// Phase name while a cloud is read.
pub const PHASE_LOAD: &str = "Wolke laden";

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
    /// The scan of each point in a model merged from several (LAS `point_source_id`, PLY
    /// `scan`); empty if unknown or if all points have the same.
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

/// Points checked against the header bounds before trusting them for the origin.
const ORIGIN_SAMPLE: usize = 16;

/// Centre of the header bounds in whole metres, or the first point in whole metres if the
/// bounds are not finite, all zero, or miss one of the `first` points (stale header of a
/// foreign writer); 0 without points.
fn origin_of(bounds: &las::Bounds, first: &[[f64; 3]]) -> [f64; 3] {
    let Some(p0) = first.first() else {
        return [0.0; 3];
    };
    let min = [bounds.min.x, bounds.min.y, bounds.min.z];
    let max = [bounds.max.x, bounds.max.y, bounds.max.z];
    // Slack for the LAS coordinate rounding.
    const SLACK: f64 = 0.01;
    let usable = min.iter().chain(&max).all(|v| v.is_finite())
        && min.iter().chain(&max).any(|&v| v != 0.0)
        && first
            .iter()
            .all(|p| (0..3).all(|k| p[k] >= min[k] - SLACK && p[k] <= max[k] + SLACK));
    if usable {
        [0, 1, 2].map(|k| ((min[k] + max[k]) / 2.0).round())
    } else {
        p0.map(f64::round)
    }
}

/// Reads a LAS file in batches of 2²⁰ points (phase [`PHASE_LOAD`], total from the header,
/// a cancel check per batch). With `slam`, each point's time `gps_time − scan_start` picks
/// its submap ([`submap_at`]) and so its quality. LAS intensity 0–65535 becomes `>> 8`; a
/// file whose intensities are all 0 (none recorded) is shown [`MID_GRAY`]. Files whose
/// colours (or intensities) never exceed 255 store 8-bit values; those are taken as they are.
/// The origin comes from the header bounds unless they do not fit the first points. The
/// `point_source_id` becomes [`PointSet::source`] if it differs between points.
pub fn load_las(
    path: &Path,
    slam: Option<&SlamReport>,
    progress: &Progress,
) -> Result<PointSet, StepError> {
    let err = |e: las::Error| StepError::Failed(format!("{}: {e}", path.display()));
    let mut reader = las::Reader::from_path(path).map_err(err)?;
    let header = reader.header();
    let total = header.number_of_points();
    let has_color = header.point_format().has_color;
    let bounds = header.bounds();
    let capacity = usize::try_from(total).unwrap_or(0);
    let mut out = PointSet {
        origin: [0.0; 3],
        xyz: Vec::with_capacity(capacity),
        rgb: has_color.then(|| Vec::with_capacity(capacity)),
        intensity: Vec::with_capacity(capacity),
        t: Vec::with_capacity(capacity),
        quality: Vec::with_capacity(capacity),
        quality_known: slam.is_some(),
        flags: Vec::with_capacity(capacity),
        source: Vec::with_capacity(capacity),
    };
    // Time zero: the scan start of the report, else the first point's time.
    let mut t0 = slam.map(|s| s.scan_start);
    let submaps = slam.map_or(&[][..], |s| &s.submaps[..]);
    let quality_of = |t: f64| quality_at(submaps, t);

    // The point format of a file has times for all points or for none.
    let mut timed = false;
    progress.phase(PHASE_LOAD, Some(total));
    let mut batch = las::PointDataBuilder::new()
        .for_header(reader.header())
        .build();
    // Low bytes of colour and intensity, kept until the whole file shows whether it stores
    // 16-bit values (as the LAS spec says) or 8-bit ones (as some foreign writers do).
    let mut low: Vec<[u8; 4]> = Vec::with_capacity(capacity);
    let (mut max_rgb, mut max_intensity) = (0u16, 0u16);
    let mut first_batch = true;
    loop {
        if progress.is_cancelled() {
            return Err(StepError::Cancelled);
        }
        let read = reader.fill_points(BATCH as u64, &mut batch).map_err(err)?;
        if read == 0 {
            break;
        }
        if first_batch {
            first_batch = false;
            let mut first = Vec::with_capacity(ORIGIN_SAMPLE);
            for p in batch.points().take(ORIGIN_SAMPLE) {
                let p = p.map_err(err)?;
                first.push([p.x, p.y, p.z]);
            }
            out.origin = origin_of(&bounds, &first);
        }
        let origin = out.origin;
        for p in batch.points() {
            let p = p.map_err(err)?;
            out.xyz.push([
                (p.x - origin[0]) as f32,
                (p.y - origin[1]) as f32,
                (p.z - origin[2]) as f32,
            ]);
            let c = p.color.unwrap_or_default();
            if let Some(rgb) = &mut out.rgb {
                rgb.push([c.red, c.green, c.blue].map(|v| (v >> 8) as u8));
                max_rgb = max_rgb.max(c.red).max(c.green).max(c.blue);
            }
            out.intensity.push((p.intensity >> 8) as u8);
            max_intensity = max_intensity.max(p.intensity);
            low.push([c.red, c.green, c.blue, p.intensity].map(|v| v as u8));
            out.flags.push(p.user_data);
            out.source.push(p.point_source_id);
            match p.gps_time {
                Some(g) => {
                    timed = true;
                    let start = *t0.get_or_insert(g);
                    let t = g - start;
                    out.t.push(t as f32);
                    out.quality
                        .push(if slam.is_some() { quality_of(t) } else { 255 });
                }
                None => {
                    out.t.push(0.0);
                    out.quality.push(255);
                }
            }
        }
        progress.advance(read);
    }
    // 8-bit values: scaled ×257 to 16 bits and back, i.e. taken as they are.
    if let Some(rgb) = out.rgb.as_mut().filter(|_| max_rgb <= 255) {
        for (c, l) in rgb.iter_mut().zip(&low) {
            *c = [l[0], l[1], l[2]];
        }
    }
    if max_intensity <= 255 {
        for (i, l) in out.intensity.iter_mut().zip(&low) {
            *i = l[3];
        }
    }
    drop(low);
    // without times the report cannot pick a submap
    out.quality_known &= timed;
    if out.intensity.iter().all(|&i| i == 0) {
        out.intensity.fill(MID_GRAY);
    }
    if out.source.windows(2).all(|w| w[0] == w[1]) {
        out.source = Vec::new();
    }
    Ok(out)
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

/// Reads a PLY file ([`driftless_export::ply::read_ply`]: phase
/// [`driftless_export::PHASE_READ`], a cancel check per block) as a [`PointSet`]
/// ([`point_set`]). PLY has no point times: `t` is 0 and the quality unknown (255).
pub fn load_ply(path: &Path, progress: &Progress) -> Result<PointSet, StepError> {
    let points = driftless_export::ply::read_ply(path, progress)?;
    Ok(point_set(points, None))
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

/// Reads `path` and builds its levels of detail: phase [`PHASE_LOAD`] (for a PLY
/// [`driftless_export::PHASE_READ`]), then [`crate::lod::PHASE_BUILD`]. A file ending in `.ply` (any case) is read by [`load_ply`],
/// any other as LAS. With `run`, the `slam.json` of that run folder gives each point of a
/// LAS the quality of its submap; without it (or unreadable, or for a PLY, which has no
/// point times) the quality is 255.
pub fn load_cloud(
    path: &Path,
    run: Option<&Path>,
    progress: &Progress,
) -> Result<LoadedCloud, StepError> {
    let report: Option<SlamReport> = run
        .and_then(|dir| std::fs::read_to_string(dir.join("slam.json")).ok())
        .and_then(|t| serde_json::from_str(&t).ok());
    let t = Instant::now();
    let points = if is_ply(path) {
        load_ply(path, progress)?
    } else {
        load_las(path, report.as_ref(), progress)?
    };
    LoadedCloud::build(
        path.to_path_buf(),
        points,
        t.elapsed().as_secs_f64(),
        progress,
    )
}

/// Whether `path` ends in `.ply`, in any case.
pub fn is_ply(path: &Path) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("ply"))
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

/// Reads the `modell.las` of a model and the cloud `scan_cloud` of its scan `member` (LAS or
/// PLY, in the frame of that scan), sets them side by side ([`combine_pair_points`]; `lone`
/// is the scan of a model with points of only one) and builds the levels of detail; `name`
/// names the cloud. In a job: phases [`PHASE_LOAD`] (twice) and
/// [`crate::lod::PHASE_BUILD`], each with cancel checks.
pub fn load_pair_cloud(
    name: PathBuf,
    model_las: &Path,
    lone: u16,
    scan_cloud: &Path,
    member: u16,
    progress: &Progress,
) -> Result<PairCloud, StepError> {
    let t = Instant::now();
    let model = load_las(model_las, None, progress)?;
    let scan = if is_ply(scan_cloud) {
        load_ply(scan_cloud, progress)?
    } else {
        load_las(scan_cloud, None, progress)?
    };
    let (points, shift) = combine_pair_points(model, lone, scan, member);
    let cloud = LoadedCloud::build(name, points, t.elapsed().as_secs_f64(), progress)?;
    Ok(PairCloud { cloud, shift })
}
