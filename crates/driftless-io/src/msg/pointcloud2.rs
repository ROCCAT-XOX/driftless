//! `sensor_msgs/PointCloud2`.

use super::{Cursor, MsgError};

/// One LiDAR return.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarPoint {
    pub xyz: [f32; 3],
    pub intensity: f32,
    pub ring: u16,
    /// Time offset within the frame, seconds.
    pub t_rel: f64,
}

/// One decoded scan.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarFrame {
    pub stamp: f64,
    pub points: Vec<LidarPoint>,
    /// Points discarded because xyz was non-finite or exactly (0,0,0).
    pub dropped: u32,
}

#[derive(Clone, Copy)]
struct Field {
    offset: usize,
    datatype: u8,
}

impl Field {
    fn size(self) -> usize {
        match self.datatype {
            1 | 2 => 1,
            3 | 4 => 2,
            5..=7 => 4,
            _ => 8,
        }
    }

    /// Reads the value at `base + offset`, converted to `f64`. Bounds are checked by the caller.
    fn read(self, point: &[u8]) -> f64 {
        let b = &point[self.offset..self.offset + self.size()];
        match self.datatype {
            1 => f64::from(b[0] as i8),
            2 => f64::from(b[0]),
            3 => f64::from(i16::from_le_bytes(b.try_into().unwrap())),
            4 => f64::from(u16::from_le_bytes(b.try_into().unwrap())),
            5 => f64::from(i32::from_le_bytes(b.try_into().unwrap())),
            6 => f64::from(u32::from_le_bytes(b.try_into().unwrap())),
            7 => f64::from(f32::from_le_bytes(b.try_into().unwrap())),
            _ => f64::from_le_bytes(b.try_into().unwrap()),
        }
    }
}

pub fn decode_pointcloud2(data: &[u8]) -> Result<LidarFrame, MsgError> {
    let mut c = Cursor::new(data);
    let stamp = c.header()?;
    let height = c.u32()? as usize;
    let width = c.u32()? as usize;

    let nfields = c.u32()?;
    let (mut x, mut y, mut z) = (None, None, None);
    let (mut intensity, mut ring, mut timestamp) = (None, None, None);
    for _ in 0..nfields {
        let name = c.string()?;
        let offset = c.u32()? as usize;
        let datatype = c.u8()?;
        let _count = c.u32()?;
        if !(1..=8).contains(&datatype) {
            return Err(MsgError::Malformed(format!(
                "field `{name}` has unknown datatype {datatype}"
            )));
        }
        let f = Field { offset, datatype };
        match name.as_str() {
            "x" => x = Some(f),
            "y" => y = Some(f),
            "z" => z = Some(f),
            "intensity" => intensity = Some(f),
            "ring" => ring = Some(f),
            "timestamp" => timestamp = Some(f),
            _ => {}
        }
    }
    let big_endian = c.u8()? != 0;
    let point_step = c.u32()? as usize;
    let row_step = c.u32()? as usize;
    let n = c.u32()? as usize;
    let cloud = c.take(n)?;

    if big_endian {
        return Err(MsgError::Unsupported("big-endian PointCloud2".into()));
    }
    let (Some(x), Some(y), Some(z)) = (x, y, z) else {
        return Err(MsgError::Malformed(
            "PointCloud2 lacks one of the x, y, z fields".into(),
        ));
    };
    for f in [Some(x), Some(y), Some(z), intensity, ring, timestamp]
        .into_iter()
        .flatten()
    {
        if f.offset + f.size() > point_step {
            return Err(MsgError::Malformed(format!(
                "field at offset {} exceeds point_step {point_step}",
                f.offset
            )));
        }
    }

    let total = height
        .checked_mul(width)
        .ok_or_else(|| MsgError::Malformed("height*width overflows".into()))?;
    let row_step = if row_step == 0 && height <= 1 {
        point_step.saturating_mul(width)
    } else {
        row_step
    };
    // Last byte needed: start of the final row plus width points.
    let needed = if total == 0 {
        0
    } else {
        (height - 1)
            .checked_mul(row_step)
            .and_then(|r| r.checked_add(width.checked_mul(point_step)?))
            .ok_or_else(|| MsgError::Malformed("PointCloud2 size overflows".into()))?
    };
    if needed > cloud.len() {
        return Err(MsgError::Malformed(format!(
            "PointCloud2 declares {needed} data bytes but holds {}",
            cloud.len()
        )));
    }

    let mut points = Vec::with_capacity(total);
    let mut dropped = 0u32;
    for row in 0..height {
        for col in 0..width {
            let start = row * row_step + col * point_step;
            let p = &cloud[start..start + point_step];
            let xyz = [x.read(p) as f32, y.read(p) as f32, z.read(p) as f32];
            if !xyz.iter().all(|v| v.is_finite()) || xyz == [0.0; 3] {
                dropped += 1;
                continue;
            }
            points.push(LidarPoint {
                xyz,
                intensity: intensity.map_or(0.0, |f| f.read(p) as f32),
                ring: ring.map_or(0, |f| f.read(p) as u16),
                t_rel: timestamp.map_or(0.0, |f| f.read(p)),
            });
        }
    }
    Ok(LidarFrame {
        stamp,
        points,
        dropped,
    })
}
