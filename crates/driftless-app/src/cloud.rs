//! A point cloud loaded from LAS for the 3D view: columns relative to an origin in whole
//! metres, so that `f32` keeps millimetres even for coordinates of ~10⁶ m.

use crate::lod::{build_octree, Octree};
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
    /// Whether `quality` comes from a SLAM report; without one it is unknown (all 255).
    pub quality_known: bool,
    /// LAS `user_data`: 1 = without measured colour, or the removal reason.
    pub flags: Vec<u8>,
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
/// The origin comes from the header bounds unless they do not fit the first points.
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
    };
    // Time zero: the scan start of the report, else the first point's time.
    let mut t0 = slam.map(|s| s.scan_start);
    let submaps = slam.map_or(&[][..], |s| &s.submaps[..]);
    let quality_of = |t: f64| {
        submaps
            .get(submap_at(submaps, t))
            .map_or(255, |m| (m.quality.clamp(0.0, 1.0) * 255.0).round() as u8)
    };

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
            match p.gps_time {
                Some(g) => {
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
    if out.intensity.iter().all(|&i| i == 0) {
        out.intensity.fill(MID_GRAY);
    }
    Ok(out)
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

/// Reads `path` and builds its levels of detail: phase [`PHASE_LOAD`], then
/// [`crate::lod::PHASE_BUILD`]. With `run`, the `slam.json` of that run folder gives each
/// point the quality of its submap; without it (or unreadable) the quality is 255.
pub fn load_cloud(
    path: &Path,
    run: Option<&Path>,
    progress: &Progress,
) -> Result<LoadedCloud, StepError> {
    let report: Option<SlamReport> = run
        .and_then(|dir| std::fs::read_to_string(dir.join("slam.json")).ok())
        .and_then(|t| serde_json::from_str(&t).ok());
    let t = Instant::now();
    let points = load_las(path, report.as_ref(), progress)?;
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
