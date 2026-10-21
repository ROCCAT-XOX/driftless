//! A point cloud loaded from LAS for the 3D view: columns relative to an origin in whole
//! metres, so that `f32` keeps millimetres even for coordinates of ~10⁶ m.

use driftless_io::progress::Progress;
use driftless_slam::report::SlamReport;
use driftless_steps::overlay::submap_at;
use driftless_steps::StepError;
use std::path::Path;

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

/// Reads a LAS file in batches of 2²⁰ points (phase [`PHASE_LOAD`], total from the header,
/// a cancel check per batch). With `slam`, each point's time `gps_time − scan_start` picks
/// its submap ([`submap_at`]) and so its quality. LAS intensity 0–65535 becomes `>> 8`; a
/// file whose intensities are all 0 (none recorded) is shown [`MID_GRAY`].
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
    let origin = if total == 0 {
        [0.0; 3]
    } else {
        let b = header.bounds();
        let centre = [
            (b.min.x + b.max.x),
            (b.min.y + b.max.y),
            (b.min.z + b.max.z),
        ]
        .map(|s| (s / 2.0).round());
        if centre.iter().all(|c| c.is_finite()) {
            centre
        } else {
            [0.0; 3]
        }
    };
    let capacity = usize::try_from(total).unwrap_or(0);
    let mut out = PointSet {
        origin,
        xyz: Vec::with_capacity(capacity),
        rgb: has_color.then(|| Vec::with_capacity(capacity)),
        intensity: Vec::with_capacity(capacity),
        t: Vec::with_capacity(capacity),
        quality: Vec::with_capacity(capacity),
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
    loop {
        if progress.is_cancelled() {
            return Err(StepError::Cancelled);
        }
        let read = reader.fill_points(BATCH as u64, &mut batch).map_err(err)?;
        if read == 0 {
            break;
        }
        for p in batch.points() {
            let p = p.map_err(err)?;
            out.xyz.push([
                (p.x - origin[0]) as f32,
                (p.y - origin[1]) as f32,
                (p.z - origin[2]) as f32,
            ]);
            if let Some(rgb) = &mut out.rgb {
                let c = p.color.unwrap_or_default();
                rgb.push([c.red, c.green, c.blue].map(|v| (v >> 8) as u8));
            }
            out.intensity.push((p.intensity >> 8) as u8);
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
    if out.intensity.iter().all(|&i| i == 0) {
        out.intensity.fill(MID_GRAY);
    }
    Ok(out)
}
