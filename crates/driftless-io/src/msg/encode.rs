//! Encoders for synthetic test bags (feature `test-util`).

use super::{CompressedImage, ImuSample, LidarPoint};

fn put_string(out: &mut Vec<u8>, s: &str) {
    out.extend_from_slice(&(s.len() as u32).to_le_bytes());
    out.extend_from_slice(s.as_bytes());
}

fn put_header(out: &mut Vec<u8>, stamp: f64, frame_id: &str) {
    let sec = stamp.floor();
    let nsec = ((stamp - sec) * 1e9).round();
    let (sec, nsec) = if nsec >= 1e9 {
        (sec as u32 + 1, 0)
    } else {
        (sec as u32, nsec as u32)
    };
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&sec.to_le_bytes());
    out.extend_from_slice(&nsec.to_le_bytes());
    put_string(out, frame_id);
}

/// Vanjee layout: x,y,z,intensity f32 @0/4/8/12, ring u16 @16, timestamp f64 @18, point_step 26.
pub fn encode_pointcloud2(stamp: f64, frame_id: &str, points: &[LidarPoint]) -> Vec<u8> {
    const STEP: u32 = 26;
    let mut out = Vec::new();
    put_header(&mut out, stamp, frame_id);
    out.extend_from_slice(&1u32.to_le_bytes()); // height
    out.extend_from_slice(&(points.len() as u32).to_le_bytes()); // width
    let fields: [(&str, u32, u8); 6] = [
        ("x", 0, 7),
        ("y", 4, 7),
        ("z", 8, 7),
        ("intensity", 12, 7),
        ("ring", 16, 4),
        ("timestamp", 18, 8),
    ];
    out.extend_from_slice(&(fields.len() as u32).to_le_bytes());
    for (name, offset, datatype) in fields {
        put_string(&mut out, name);
        out.extend_from_slice(&offset.to_le_bytes());
        out.push(datatype);
        out.extend_from_slice(&1u32.to_le_bytes());
    }
    out.push(0); // is_bigendian
    out.extend_from_slice(&STEP.to_le_bytes());
    out.extend_from_slice(&(STEP * points.len() as u32).to_le_bytes()); // row_step
    out.extend_from_slice(&(STEP * points.len() as u32).to_le_bytes()); // data length
    for p in points {
        for v in p.xyz {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&p.intensity.to_le_bytes());
        out.extend_from_slice(&p.ring.to_le_bytes());
        out.extend_from_slice(&p.t_rel.to_le_bytes());
    }
    out.push(1); // is_dense
    out
}

/// Orientation and all covariances are zero.
pub fn encode_imu(s: &ImuSample) -> Vec<u8> {
    let mut out = Vec::new();
    put_header(&mut out, s.stamp, "imu");
    let zeros = |out: &mut Vec<u8>, n: usize| out.extend(std::iter::repeat_n(0u8, 8 * n));
    zeros(&mut out, 4 + 9);
    for v in s.gyro {
        out.extend_from_slice(&v.to_le_bytes());
    }
    zeros(&mut out, 9);
    for v in s.accel {
        out.extend_from_slice(&v.to_le_bytes());
    }
    zeros(&mut out, 9);
    out
}

pub fn encode_compressed_image(img: &CompressedImage) -> Vec<u8> {
    let mut out = Vec::new();
    put_header(&mut out, img.stamp, "camera");
    put_string(&mut out, &img.format);
    out.extend_from_slice(&(img.data.len() as u32).to_le_bytes());
    out.extend_from_slice(&img.data);
    out
}
