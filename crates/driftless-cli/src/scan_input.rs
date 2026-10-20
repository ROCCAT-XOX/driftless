//! Reading a scan for the commands that run the odometry (`odom`, `slam`): IMU and LiDAR
//! messages from the bags, the LiDAR → IMU extrinsic, and writing LAS point clouds.
//!
//! The IMU is read first (small), the LiDAR frames are then streamed from a reader thread.

use crate::topics::{bag_read_error, file_name, TopicPicker};
use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_odom::OdomStats;
use driftless_traj::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::path::{Path, PathBuf};
use std::sync::mpsc::sync_channel;

pub const IMU_TYPE: &str = "sensor_msgs/Imu";
pub const CLOUD_TYPE: &str = "sensor_msgs/PointCloud2";

/// Calls `f` with every message of `msg_type` in `bags` until it returns `false`, from the
/// topic chosen by [`TopicPicker`]. A truncated bag ends with a warning (everything before the
/// cut is kept); any other read error fails. An explicit topic no bag has fails.
fn for_each_message(
    bags: &[PathBuf],
    msg_type: &str,
    wanted: Option<&str>,
    option: &str,
    mut f: impl FnMut(&[u8]) -> bool,
) -> Result<(), String> {
    let mut picker = TopicPicker::new(msg_type, wanted, option);
    for bag in bags {
        let mut reader = BagReader::open(bag).map_err(|e| format!("{}: {e}", file_name(bag)))?;
        let Some(topic) = picker.pick(&reader)? else {
            continue;
        };
        for item in reader.messages(&[topic.as_str()]) {
            match item {
                Ok(m) => {
                    if !f(&m.data) {
                        return Ok(());
                    }
                }
                Err(e) => {
                    bag_read_error(bag, e)?;
                    break;
                }
            }
        }
    }
    picker.finish()
}

/// `x y z qx qy qz qw` (LiDAR pose in the IMU frame) → isometry; identity without. The
/// quaternion is normalised.
pub fn parse_extrinsic(v: Option<&[f64]>) -> Result<Isometry3<f64>, String> {
    let Some(v) = v else {
        return Ok(Isometry3::identity());
    };
    let [x, y, z, qx, qy, qz, qw] = v else {
        return Err("--lidar-to-imu braucht 7 Zahlen: x y z qx qy qz qw".into());
    };
    let q = Quaternion::new(*qw, *qx, *qy, *qz);
    let finite = [x, y, z].iter().all(|v| v.is_finite());
    if !finite || !q.norm().is_finite() || q.norm() <= 1e-6 {
        return Err("--lidar-to-imu: ungültige Quaternion oder Position".into());
    }
    Ok(Isometry3::from_parts(
        Translation3::new(*x, *y, *z),
        UnitQuaternion::from_quaternion(q),
    ))
}

/// All IMU samples of the scan and the number of messages that could not be decoded. No
/// sample at all is an error: the odometry needs the IMU.
pub fn read_imu(bags: &[PathBuf], topic: Option<&str>) -> Result<(Vec<ImuSample>, usize), String> {
    let mut imu = Vec::new();
    let mut bad = 0usize;
    for_each_message(bags, IMU_TYPE, topic, "--imu-topic", |data| {
        match decode_imu(data) {
            Ok(s) => imu.push(s),
            Err(_) => bad += 1,
        }
        true
    })?;
    if imu.is_empty() {
        return Err(format!(
            "keine IMU-Messungen ({IMU_TYPE}) gelesen; die Odometrie braucht eine IMU"
        ));
    }
    Ok((imu, bad))
}

/// What the LiDAR reader thread skipped.
pub struct LidarReadStats {
    /// Messages that could not be decoded.
    pub bad: usize,
    /// Points the decoder dropped (non-finite and the like).
    pub dropped: u64,
}

/// Streams the LiDAR frames of `bags` from a reader thread into `consume` and returns its
/// result. If `consume` stops early, the reader stops too.
pub fn with_lidar_frames<R>(
    bags: &[PathBuf],
    topic: Option<&str>,
    consume: impl FnOnce(std::sync::mpsc::IntoIter<LidarFrame>) -> R,
) -> Result<(R, LidarReadStats), Box<dyn std::error::Error>> {
    let (tx, rx) = sync_channel::<LidarFrame>(16);
    std::thread::scope(|s| {
        let reader = s.spawn(move || {
            let (mut bad, mut dropped) = (0usize, 0u64);
            for_each_message(bags, CLOUD_TYPE, topic, "--lidar-topic", |data| {
                match decode_pointcloud2(data) {
                    Ok(f) => {
                        dropped += u64::from(f.dropped);
                        tx.send(f).is_ok()
                    }
                    Err(_) => {
                        bad += 1;
                        true
                    }
                }
            })
            .map(|()| LidarReadStats { bad, dropped })
        });
        let result = consume(rx.into_iter());
        let stats = reader.join().map_err(|_| "Lesethread abgestürzt")??;
        Ok((result, stats))
    })
}

/// Fails if the odometry saw no LiDAR frame at all.
pub fn require_frames(stats: &OdomStats) -> Result<(), String> {
    if stats.frames_in == 0 {
        return Err(format!("keine LiDAR-Frames ({CLOUD_TYPE}) gelesen"));
    }
    Ok(())
}

/// Warns if gravity and gyro bias could only be guessed (no static start).
pub fn warn_if_not_static(stats: &OdomStats) {
    if !stats.static_start {
        eprintln!(
            "Warnung: Gerät stand in der ersten Sekunde nicht still; Schwerkraft und Kreisel-Offset \
             sind nur grob geschätzt"
        );
    }
}

pub fn write_las(path: &Path, points: &[[f32; 3]]) -> Result<(), Box<dyn std::error::Error>> {
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(0)?;
    let mut w = las::Writer::from_path(path, builder.into_header()?)?;
    for p in points {
        w.write_point(las::Point {
            x: f64::from(p[0]),
            y: f64::from(p[1]),
            z: f64::from(p[2]),
            ..Default::default()
        })?;
    }
    w.close()?;
    Ok(())
}
