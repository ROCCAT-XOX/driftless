//! Writing a [`Cloud`] as LAS or binary PLY. Both files are byte-identical for the same cloud
//! (no creation date or random id in the header). Both go through the one writer of their
//! format in [`driftless_export`].

use crate::{Cloud, MapError, RemovalReason};
use driftless_export::{Layout, PointSource, ScanInfo};
use driftless_io::progress::Progress;
use std::path::Path;

pub use driftless_export::las::LAS_SCALE;

/// What goes into `user_data` (the flags of a [`PointSource`]).
#[derive(Clone, Copy)]
enum Flags<'a> {
    /// 1 for a point without a measured colour (`colored == Some(false)`), else 0.
    Uncolored,
    /// The [`RemovalReason`] code of each point.
    Reasons(&'a [RemovalReason]),
}

/// A [`Cloud`] as [`PointSource`], without a copy: time = capture time `t`, intensity as in
/// the cloud (0–255), colour widened ×257 to 16 bit.
struct CloudPoints<'a> {
    cloud: &'a Cloud,
    sources: Option<&'a [u16]>,
    flags: Flags<'a>,
}

impl PointSource for CloudPoints<'_> {
    fn len(&self) -> usize {
        self.cloud.len()
    }

    fn layout(&self) -> Layout {
        Layout {
            rgb: self.cloud.rgb.is_some(),
            intensity: true,
            source: self.sources.is_some(),
            time: true,
            flags: match self.flags {
                Flags::Uncolored => self.cloud.colored.is_some(),
                Flags::Reasons(_) => true,
            },
        }
    }

    fn xyz(&self, i: usize) -> [f64; 3] {
        self.cloud.xyz[i].map(f64::from)
    }

    fn rgb(&self, i: usize) -> [u16; 3] {
        self.cloud
            .rgb
            .as_ref()
            .map_or([0; 3], |c| c[i].map(|v| u16::from(v) * 257))
    }

    fn intensity(&self, i: usize) -> f32 {
        self.cloud.intensity[i]
    }

    fn source(&self, i: usize) -> u16 {
        self.sources.map_or(0, |s| s[i])
    }

    fn time(&self, i: usize) -> f64 {
        self.cloud.t[i]
    }

    fn flags(&self, i: usize) -> u8 {
        match self.flags {
            Flags::Uncolored => u8::from(self.cloud.colored.as_ref().is_some_and(|c| !c[i])),
            Flags::Reasons(r) => r[i] as u8,
        }
    }

    fn scans(&self) -> &[ScanInfo] {
        &[]
    }
}

fn write_cloud_las(path: &Path, points: &CloudPoints) -> Result<(), MapError> {
    driftless_export::las::write_las(path, points, false, &Progress::new()).map_err(MapError::from)
}

/// LAS 1.4: point format 7 (XYZ, intensity, GPS time, RGB) if the cloud has colours,
/// otherwise format 6 (without RGB); 0.1 mm around the rounded middle of the cloud. GPS time
/// holds the capture time as raw Unix seconds (not GPS week or adjusted standard GPS time).
///
/// `user_data` is 1 for points that got no measured colour (`colored == Some(false)`), else 0;
/// CloudCompare shows it as the scalar field "User Data".
pub fn write_las(path: &Path, cloud: &Cloud) -> Result<(), MapError> {
    write_cloud_las(
        path,
        &CloudPoints {
            cloud,
            sources: None,
            flags: Flags::Uncolored,
        },
    )
}

/// Like [`write_las`] for a cloud merged from several scans: `point_source_id` holds the
/// scan of each point (its number in the model).
///
/// # Panics
/// If `sources` does not have one entry per point.
pub fn write_las_sources(path: &Path, cloud: &Cloud, sources: &[u16]) -> Result<(), MapError> {
    assert_eq!(sources.len(), cloud.len(), "one source per point");
    write_cloud_las(
        path,
        &CloudPoints {
            cloud,
            sources: Some(sources),
            flags: Flags::Uncolored,
        },
    )
}

/// Like [`write_las`] for the removed points: `user_data` holds the [`RemovalReason`] code
/// (1 ghost, 2 outlier, 3 seen through a window).
///
/// # Panics
/// If `reasons` does not have one entry per point.
pub fn write_removed_las(
    path: &Path,
    cloud: &Cloud,
    reasons: &[RemovalReason],
) -> Result<(), MapError> {
    assert_eq!(reasons.len(), cloud.len(), "one reason per removed point");
    write_cloud_las(
        path,
        &CloudPoints {
            cloud,
            sources: None,
            flags: Flags::Reasons(reasons),
        },
    )
}

/// Binary little-endian PLY: `x y z intensity` as float, `red green blue` as uchar if the
/// cloud has colours, then `colored` (uchar, 1 = measured colour, 0 = none) if the cloud
/// knows which points were coloured. Goes through [`driftless_export::ply::write_ply`]
/// (`double` for xyz once a coordinate reaches [`driftless_export::MAX_FLOAT_COORD`]).
pub fn write_ply(path: &Path, cloud: &Cloud) -> Result<(), MapError> {
    write_cloud_ply(
        path,
        &CloudPoints {
            cloud,
            sources: None,
            flags: Flags::Uncolored,
        },
    )
}

/// Like [`write_ply`] for a cloud merged from several scans, with one more property `scan`
/// (ushort): the scan of each point (its number in the model).
///
/// # Panics
/// If `sources` does not have one entry per point.
pub fn write_ply_sources(path: &Path, cloud: &Cloud, sources: &[u16]) -> Result<(), MapError> {
    assert_eq!(sources.len(), cloud.len(), "one source per point");
    write_cloud_ply(
        path,
        &CloudPoints {
            cloud,
            sources: Some(sources),
            flags: Flags::Uncolored,
        },
    )
}

fn write_cloud_ply(path: &Path, points: &CloudPoints) -> Result<(), MapError> {
    driftless_export::ply::write_ply(path, points, &Progress::new()).map_err(MapError::from)
}
