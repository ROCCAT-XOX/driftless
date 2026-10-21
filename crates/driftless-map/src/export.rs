//! Writing a [`Cloud`] as LAS or binary PLY. Both files are byte-identical for the same cloud
//! (no creation date or random id in the header).

use crate::{Cloud, MapError, RemovalReason};
use std::io::Write;
use std::path::Path;

/// LAS coordinate resolution (m): 0.1 mm.
pub const LAS_SCALE: f64 = 0.0001;

/// LiDAR intensity (0–255) → LAS intensity (0–65535).
fn las_intensity(i: f32) -> u16 {
    (f64::from(i) * 257.0).round().clamp(0.0, 65535.0) as u16
}

/// Per-axis minimum of the cloud rounded down to whole metres; zero for an empty cloud.
fn las_offset(cloud: &Cloud) -> [f64; 3] {
    if cloud.is_empty() {
        return [0.0; 3];
    }
    let mut min = [f32::INFINITY; 3];
    for p in &cloud.xyz {
        for (m, v) in min.iter_mut().zip(p) {
            *m = m.min(*v);
        }
    }
    min.map(|m| f64::from(m).floor())
}

/// LAS 1.2: point format 3 (XYZ, intensity, GPS time, RGB) if the cloud has colours,
/// otherwise format 1 (without RGB). GPS time holds the capture time as raw Unix seconds
/// (not GPS week or adjusted standard GPS time).
///
/// `user_data` is 1 for points that got no measured colour (`colored == Some(false)`), else 0;
/// CloudCompare shows it as the scalar field "User Data".
pub fn write_las(path: &Path, cloud: &Cloud) -> Result<(), MapError> {
    write_las_with(path, cloud, |i| {
        u8::from(cloud.colored.as_ref().is_some_and(|c| !c[i]))
    })
}

/// Like [`write_las`] for the removed points: `user_data` holds the [`RemovalReason`] code
/// (1 ghost, 2 outlier).
///
/// # Panics
/// If `reasons` does not have one entry per point.
pub fn write_removed_las(
    path: &Path,
    cloud: &Cloud,
    reasons: &[RemovalReason],
) -> Result<(), MapError> {
    assert_eq!(reasons.len(), cloud.len(), "one reason per removed point");
    write_las_with(path, cloud, |i| reasons[i] as u8)
}

fn write_las_with(
    path: &Path,
    cloud: &Cloud,
    user_data: impl Fn(usize) -> u8,
) -> Result<(), MapError> {
    let err = |source| MapError::Las {
        path: path.display().to_string(),
        source,
    };
    let mut builder = las::Builder::from((1, 2));
    builder.point_format =
        las::point::Format::new(if cloud.rgb.is_some() { 3 } else { 1 }).map_err(err)?;
    builder.date = None;
    builder.generating_software = "driftless".into();
    let offset = las_offset(cloud);
    builder.transforms = las::Vector {
        x: las::Transform {
            scale: LAS_SCALE,
            offset: offset[0],
        },
        y: las::Transform {
            scale: LAS_SCALE,
            offset: offset[1],
        },
        z: las::Transform {
            scale: LAS_SCALE,
            offset: offset[2],
        },
    };
    let header = builder.into_header().map_err(err)?;
    let mut w = las::Writer::from_path(path, header).map_err(err)?;
    for i in 0..cloud.len() {
        let [x, y, z] = cloud.xyz[i].map(f64::from);
        let color = cloud.rgb.as_ref().map(|c| {
            let [r, g, b] = c[i].map(|v| u16::from(v) * 257);
            las::Color::new(r, g, b)
        });
        w.write_point(las::Point {
            x,
            y,
            z,
            intensity: las_intensity(cloud.intensity[i]),
            gps_time: Some(cloud.t[i]),
            color,
            user_data: user_data(i),
            ..Default::default()
        })
        .map_err(err)?;
    }
    w.close().map_err(err)
}

/// Binary little-endian PLY: `x y z intensity` as float, `red green blue` as uchar if the
/// cloud has colours, then `colored` (uchar, 1 = measured colour, 0 = none) if the cloud
/// knows which points were coloured.
pub fn write_ply(path: &Path, cloud: &Cloud) -> Result<(), MapError> {
    let err = |source| MapError::Io {
        path: path.display().to_string(),
        source,
    };
    let file = std::fs::File::create(path).map_err(err)?;
    let mut w = std::io::BufWriter::new(file);
    let mut header = format!(
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\n\
         property float x\nproperty float y\nproperty float z\nproperty float intensity\n",
        cloud.len()
    );
    if cloud.rgb.is_some() {
        header += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    if cloud.colored.is_some() {
        header += "property uchar colored\n";
    }
    header += "end_header\n";
    w.write_all(header.as_bytes()).map_err(err)?;
    for i in 0..cloud.len() {
        let mut v = [0u8; 20];
        for (k, c) in cloud.xyz[i].iter().chain([&cloud.intensity[i]]).enumerate() {
            v[4 * k..4 * k + 4].copy_from_slice(&c.to_le_bytes());
        }
        let n = match &cloud.rgb {
            Some(rgb) => {
                v[16..19].copy_from_slice(&rgb[i]);
                19
            }
            None => 16,
        };
        let n = match &cloud.colored {
            Some(c) => {
                v[n] = u8::from(c[i]);
                n + 1
            }
            None => n,
        };
        w.write_all(&v[..n]).map_err(err)?;
    }
    w.flush().map_err(err)
}
