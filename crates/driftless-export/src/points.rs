//! The point model: [`Points`] as columns, and [`PointSource`], through which every writer
//! reads points without a copy.

use crate::{e57, las, pcd, ply, ExportError, MAX_FLOAT_COORD};
use driftless_io::progress::Progress;
use nalgebra::Isometry3;
use std::path::Path;

/// Which columns a cloud has besides xyz.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Layout {
    pub rgb: bool,
    pub intensity: bool,
    pub source: bool,
    pub time: bool,
    pub flags: bool,
}

/// One scan of a cloud merged from several (a member of a model).
#[derive(Clone, Debug, PartialEq)]
pub struct ScanInfo {
    /// Its number, as in [`Points::source`].
    pub source: u16,
    pub name: String,
    /// Scan frame → file frame.
    pub pose: Isometry3<f64>,
}

/// A point cloud as columns of equal length; a column that is `None` is not there.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Points {
    pub xyz: Vec<[f64; 3]>,
    /// 16 bit per channel.
    pub rgb: Option<Vec<[u16; 3]>>,
    /// 0–65535.
    pub intensity: Option<Vec<u16>>,
    /// The scan of each point (its number in the model).
    pub source: Option<Vec<u16>>,
    /// Seconds, as stored (LAS `gps_time`).
    pub time: Option<Vec<f64>>,
    /// LAS `user_data`: 1 = no measured colour, or the reason a point was removed.
    pub flags: Option<Vec<u8>>,
    pub scans: Vec<ScanInfo>,
}

impl Points {
    pub fn len(&self) -> usize {
        self.xyz.len()
    }

    pub fn is_empty(&self) -> bool {
        self.xyz.is_empty()
    }

    /// Per-axis minimum and maximum; `None` without points.
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        bounds_of(self)
    }

    /// Drops the columns that say nothing: intensity and flags that are all 0, and a scan
    /// number that is the same for every point.
    pub fn drop_empty_columns(&mut self) {
        if self
            .intensity
            .as_ref()
            .is_some_and(|c| c.iter().all(|&v| v == 0))
        {
            self.intensity = None;
        }
        if self
            .flags
            .as_ref()
            .is_some_and(|c| c.iter().all(|&v| v == 0))
        {
            self.flags = None;
        }
        if self
            .source
            .as_ref()
            .is_some_and(|c| c.windows(2).all(|w| w[0] == w[1]))
        {
            self.source = None;
        }
    }
}

/// Read access to a cloud, point by point, for the writers. A column the [`Layout`] does not
/// have reads as zero.
pub trait PointSource {
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn layout(&self) -> Layout;
    fn xyz(&self, i: usize) -> [f64; 3];
    /// 16 bit per channel.
    fn rgb(&self, i: usize) -> [u16; 3];
    /// 0–255 (LAS stores `round(v × 257)`).
    fn intensity(&self, i: usize) -> f32;
    fn source(&self, i: usize) -> u16;
    fn time(&self, i: usize) -> f64;
    fn flags(&self, i: usize) -> u8;
    fn scans(&self) -> &[ScanInfo];
}

/// `col[i]`, or the default without the column.
fn at<T: Copy + Default>(col: &Option<Vec<T>>, i: usize) -> T {
    col.as_ref().map_or_else(T::default, |c| c[i])
}

impl PointSource for Points {
    fn len(&self) -> usize {
        self.xyz.len()
    }

    fn layout(&self) -> Layout {
        Layout {
            rgb: self.rgb.is_some(),
            intensity: self.intensity.is_some(),
            source: self.source.is_some(),
            time: self.time.is_some(),
            flags: self.flags.is_some(),
        }
    }

    fn xyz(&self, i: usize) -> [f64; 3] {
        self.xyz[i]
    }

    fn rgb(&self, i: usize) -> [u16; 3] {
        at(&self.rgb, i)
    }

    fn intensity(&self, i: usize) -> f32 {
        f32::from(at(&self.intensity, i)) / 257.0
    }

    fn source(&self, i: usize) -> u16 {
        at(&self.source, i)
    }

    fn time(&self, i: usize) -> f64 {
        at(&self.time, i)
    }

    fn flags(&self, i: usize) -> u8 {
        at(&self.flags, i)
    }

    fn scans(&self) -> &[ScanInfo] {
        &self.scans
    }
}

/// Per-axis minimum and maximum of the points; `None` without points. Coordinates that are
/// not finite are skipped.
pub fn bounds_of(src: &dyn PointSource) -> Option<([f64; 3], [f64; 3])> {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for i in 0..src.len() {
        for (k, v) in src.xyz(i).into_iter().enumerate() {
            min[k] = min[k].min(v);
            max[k] = max[k].max(v);
        }
    }
    (!src.is_empty()).then_some((min, max))
}

/// Whether a coordinate reaches [`MAX_FLOAT_COORD`], so that `float` would lose precision.
pub fn needs_double(src: &dyn PointSource) -> bool {
    bounds_of(src)
        .is_some_and(|(min, max)| min.iter().chain(&max).any(|v| v.abs() >= MAX_FLOAT_COORD))
}

/// A file format for point clouds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointFormat {
    Las,
    Laz,
    Ply,
    Pcd,
    E57,
}

impl PointFormat {
    /// The format of a file name's extension (any case); `None` for others.
    pub fn from_path(path: &Path) -> Option<Self> {
        let ext = path.extension()?.to_str()?.to_ascii_lowercase();
        match ext.as_str() {
            "las" => Some(PointFormat::Las),
            "laz" => Some(PointFormat::Laz),
            "ply" => Some(PointFormat::Ply),
            "pcd" => Some(PointFormat::Pcd),
            "e57" => Some(PointFormat::E57),
            _ => None,
        }
    }

    /// The usual extension, without the dot.
    pub fn extension(self) -> &'static str {
        match self {
            PointFormat::Las => "las",
            PointFormat::Laz => "laz",
            PointFormat::Ply => "ply",
            PointFormat::Pcd => "pcd",
            PointFormat::E57 => "e57",
        }
    }
}

/// Reads a point cloud in the format of its extension, then drops the columns that say
/// nothing ([`Points::drop_empty_columns`]).
pub fn read_points(path: &Path, progress: &Progress) -> Result<Points, ExportError> {
    let mut points = match PointFormat::from_path(path) {
        Some(PointFormat::Las | PointFormat::Laz) => las::read_las(path, progress)?,
        Some(PointFormat::Ply) => ply::read_ply(path, progress)?,
        Some(PointFormat::Pcd) => pcd::read_pcd(path, progress)?,
        Some(PointFormat::E57) => e57::read_e57(path, progress)?,
        None => {
            return Err(ExportError::format(
                path,
                "unbekanntes Punktwolken-Format (erwartet: .las, .laz, .ply, .pcd, .e57)",
            ))
        }
    };
    points.drop_empty_columns();
    Ok(points)
}

/// Writes `src` as `format` to `path`, through `<name>.tmp` next to it.
pub fn write_points(
    path: &Path,
    format: PointFormat,
    src: &dyn PointSource,
    progress: &Progress,
) -> Result<(), ExportError> {
    match format {
        PointFormat::Las => las::write_las(path, src, false, progress),
        PointFormat::Laz => las::write_las(path, src, true, progress),
        PointFormat::Ply => ply::write_ply(path, src, progress),
        PointFormat::Pcd => pcd::write_pcd(path, src, progress),
        PointFormat::E57 => e57::write_e57(path, src, progress),
    }
}
