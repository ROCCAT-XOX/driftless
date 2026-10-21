//! Reading a scan for the steps that run the odometry (`odom`, `slam`, `map`): topic choice,
//! IMU and LiDAR messages from the bags, the LiDAR → IMU extrinsic, and writing LAS point
//! clouds.
//!
//! The IMU is read first (small), the LiDAR frames are then streamed from a reader thread.

use crate::error::{check_cancel, StepError};
use driftless_io::bag::{BagError, BagReader};
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::progress::Progress;
use driftless_odom::OdomStats;
use driftless_traj::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::collections::BTreeSet;
use std::io::{Read, Seek};
use std::path::{Path, PathBuf};
use std::sync::mpsc::sync_channel;

pub const IMU_TYPE: &str = "sensor_msgs/Imu";
pub const CLOUD_TYPE: &str = "sensor_msgs/PointCloud2";
pub const IMAGE_TYPE: &str = "sensor_msgs/CompressedImage";

/// The topic of type `msg_type` to read from this bag.
///
/// With `wanted`, that topic if the bag has it with this type. Without, the only topic of
/// this type; several are an error naming them and the option (`option`) to choose one.
/// `Ok(None)`: the bag has no matching topic.
pub fn pick_topic<R: Read + Seek>(
    reader: &BagReader<R>,
    msg_type: &str,
    wanted: Option<&str>,
    option: &str,
) -> Result<Option<String>, String> {
    let mut topics: Vec<&str> = reader
        .connections()
        .iter()
        .filter(|c| c.msg_type == msg_type)
        .map(|c| c.topic.as_str())
        .collect();
    topics.sort_unstable();
    topics.dedup();
    match (wanted, topics.as_slice()) {
        (Some(w), _) => Ok(topics.contains(&w).then(|| w.to_string())),
        (None, []) => Ok(None),
        (None, [one]) => Ok(Some((*one).to_string())),
        (None, many) => Err(format!(
            "mehrere {msg_type}-Topics ({}): mit {option} eins wählen",
            many.join(", ")
        )),
    }
}

/// Topic choice across all bags of a project. With an explicit topic name that no bag has,
/// [`TopicPicker::finish`] fails and names the topics of that type that do exist.
pub struct TopicPicker<'a> {
    msg_type: &'a str,
    wanted: Option<&'a str>,
    option: &'a str,
    found: bool,
    available: BTreeSet<String>,
}

impl<'a> TopicPicker<'a> {
    pub fn new(msg_type: &'a str, wanted: Option<&'a str>, option: &'a str) -> Self {
        TopicPicker {
            msg_type,
            wanted,
            option,
            found: false,
            available: BTreeSet::new(),
        }
    }

    /// The topic to read from this bag (see [`pick_topic`]).
    pub fn pick<R: Read + Seek>(
        &mut self,
        reader: &BagReader<R>,
    ) -> Result<Option<String>, String> {
        self.available.extend(
            reader
                .connections()
                .iter()
                .filter(|c| c.msg_type == self.msg_type)
                .map(|c| c.topic.clone()),
        );
        let topic = pick_topic(reader, self.msg_type, self.wanted, self.option)?;
        self.found |= topic.is_some();
        Ok(topic)
    }

    /// Call after all bags: an explicitly requested topic must exist in at least one bag.
    pub fn finish(self) -> Result<(), String> {
        match self.wanted {
            Some(w) if !self.found => {
                let have: Vec<&str> = self.available.iter().map(String::as_str).collect();
                Err(if have.is_empty() {
                    format!(
                        "{}: Topic {w} nicht gefunden, die Bags haben kein {}-Topic",
                        self.option, self.msg_type
                    )
                } else {
                    format!(
                        "{}: Topic {w} nicht gefunden; vorhandene {}-Topics: {}",
                        self.option,
                        self.msg_type,
                        have.join(", ")
                    )
                })
            }
            _ => Ok(()),
        }
    }
}

/// A read error ends the bag: a truncated copy only with a warning (everything before the
/// cut is kept), anything else (unsupported compression, malformed or unreadable data) fails.
pub fn bag_read_error(bag: &Path, e: BagError, progress: &Progress) -> Result<(), String> {
    let name = file_name(bag);
    match e {
        BagError::Truncated { .. } => {
            progress.warn(format!("{name}: {e}"));
            Ok(())
        }
        e => Err(format!("{name}: {e}")),
    }
}

/// File name for messages (the full path is noise and may be personal).
pub fn file_name(p: &Path) -> String {
    p.file_name().map_or_else(
        || p.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// All `sensor_msgs/CompressedImage` topics of the image bags, sorted.
pub fn image_topics(bags: &[PathBuf]) -> Result<Vec<String>, String> {
    let mut topics = BTreeSet::new();
    for bag in bags {
        let reader = BagReader::open(bag).map_err(|e| format!("{}: {e}", file_name(bag)))?;
        topics.extend(
            reader
                .connections()
                .iter()
                .filter(|c| c.msg_type == IMAGE_TYPE)
                .map(|c| c.topic.clone()),
        );
    }
    Ok(topics.into_iter().collect())
}

/// Calls `f` with every message of `msg_type` in `bags` until it returns `false`, from the
/// topic chosen by [`TopicPicker`]. A truncated bag ends with a warning (everything before the
/// cut is kept); any other read error fails. An explicit topic no bag has fails.
fn for_each_message(
    bags: &[PathBuf],
    msg_type: &str,
    wanted: Option<&str>,
    option: &str,
    progress: &Progress,
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
                    bag_read_error(bag, e, progress)?;
                    break;
                }
            }
        }
    }
    picker.finish()
}

/// Number of messages [`for_each_message`] will see, from the bag indexes; `None` if a bag
/// has no index or the topic choice fails (that error comes again when reading).
fn message_total(bags: &[PathBuf], msg_type: &str, wanted: Option<&str>) -> Option<u64> {
    let mut picker = TopicPicker::new(msg_type, wanted, "");
    let mut total = 0;
    for bag in bags {
        let mut reader = BagReader::open(bag).ok()?;
        if let Some(topic) = picker.pick(&reader).ok()? {
            total += reader.message_count(&topic)?;
        }
    }
    Some(total)
}

/// `x y z qx qy qz qw` as given on the command line; any other count is an error.
pub fn extrinsic_values(v: Option<&[f64]>) -> Result<Option<[f64; 7]>, String> {
    v.map(|v| {
        <[f64; 7]>::try_from(v)
            .map_err(|_| "--lidar-to-imu braucht 7 Zahlen: x y z qx qy qz qw".into())
    })
    .transpose()
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

/// All IMU samples of the scan and the number of messages that could not be decoded
/// (phase „IMU lesen“). No sample at all is an error: the odometry needs the IMU.
pub fn read_imu(
    bags: &[PathBuf],
    topic: Option<&str>,
    progress: &Progress,
) -> Result<(Vec<ImuSample>, usize), StepError> {
    progress.phase("IMU lesen", None);
    let mut imu = Vec::new();
    let mut bad = 0usize;
    for_each_message(bags, IMU_TYPE, topic, "--imu-topic", progress, |data| {
        match decode_imu(data) {
            Ok(s) => imu.push(s),
            Err(_) => bad += 1,
        }
        !progress.is_cancelled()
    })?;
    check_cancel(progress)?;
    if imu.is_empty() {
        return Err(format!(
            "keine IMU-Messungen ({IMU_TYPE}) gelesen; die Odometrie braucht eine IMU"
        )
        .into());
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
///
/// Phase „LiDAR lesen“ with the message count of the bag indexes as total; every message
/// read counts one. Once `progress` is cancelled the stream ends early, so the caller must
/// check [`Progress::is_cancelled`] before it uses the result.
pub fn with_lidar_frames<R>(
    bags: &[PathBuf],
    topic: Option<&str>,
    progress: &Progress,
    consume: impl FnOnce(std::sync::mpsc::IntoIter<LidarFrame>) -> R,
) -> Result<(R, LidarReadStats), StepError> {
    progress.phase("LiDAR lesen", message_total(bags, CLOUD_TYPE, topic));
    let (tx, rx) = sync_channel::<LidarFrame>(16);
    std::thread::scope(|s| {
        let reader = s.spawn(move || {
            let (mut bad, mut dropped) = (0usize, 0u64);
            for_each_message(bags, CLOUD_TYPE, topic, "--lidar-topic", progress, |data| {
                if progress.is_cancelled() {
                    return false;
                }
                let more = match decode_pointcloud2(data) {
                    Ok(f) => {
                        dropped += u64::from(f.dropped);
                        tx.send(f).is_ok()
                    }
                    Err(_) => {
                        bad += 1;
                        true
                    }
                };
                progress.advance(1);
                more
            })
            .map(|()| LidarReadStats { bad, dropped })
        });
        let result = consume(rx.into_iter());
        let stats = reader
            .join()
            .map_err(|_| StepError::from("Lesethread abgestürzt"))??;
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
pub fn warn_if_not_static(stats: &OdomStats, progress: &Progress) {
    if !stats.static_start {
        progress.warn(
            "Gerät stand in der ersten Sekunde nicht still; Schwerkraft und Kreisel-Offset \
             sind nur grob geschätzt",
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrinsic_values_need_seven() {
        assert_eq!(extrinsic_values(None), Ok(None));
        let seven = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(extrinsic_values(Some(&seven)), Ok(Some(seven)));
        let err = extrinsic_values(Some(&[0.0; 14])).unwrap_err();
        assert_eq!(
            err,
            parse_extrinsic(Some(&[0.0; 14])).unwrap_err(),
            "same message as before"
        );
    }
}
