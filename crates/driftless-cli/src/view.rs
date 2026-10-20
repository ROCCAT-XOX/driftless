//! `driftless view <ordner>`: Fehlersuche-Viewer (rerun) für eine Rohaufnahme.
//!
//! Die Bags werden Nachricht für Nachricht gelesen, geloggt und verworfen;
//! der Speicher bleibt unabhängig von der Aufnahmelänge klein.

use crate::topics::{file_name, TopicPicker};
use driftless_io::bag::{BagError, BagReader};
use driftless_io::msg::{decode_compressed_image, decode_imu, decode_pointcloud2, LidarFrame};
use driftless_io::project::discover_project;
use driftless_traj::read_tum;
use std::path::{Path, PathBuf};

const TIMELINE: &str = "stamp";
const CLOUD_TYPE: &str = "sensor_msgs/PointCloud2";
const IMU_TYPE: &str = "sensor_msgs/Imu";
const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs/CompressedImage";
const RAW_IMAGE_TYPE: &str = "sensor_msgs/Image";

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Trajektorie (TUM-Datei) als Linie einblenden
    #[arg(long)]
    traj: Option<PathBuf>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
    /// Nur jedes N-te Kamerabild loggen
    #[arg(long, default_value_t = 5, value_parser = clap::value_parser!(u32).range(1..))]
    every: u32,
    /// Statt den Viewer zu starten in diese .rrd-Datei schreiben (kein Fenster)
    #[arg(long, value_name = "DATEI.rrd")]
    dry_run: Option<PathBuf>,
}

type BoxError = Box<dyn std::error::Error>;

/// Grayscale ramp, normalised per frame to the maximum intensity.
fn intensity_colors(frame: &LidarFrame) -> Vec<rerun::Color> {
    let max = frame
        .points
        .iter()
        .map(|p| p.intensity)
        .filter(|v| v.is_finite())
        .fold(0.0_f32, f32::max);
    let scale = if max > 0.0 { 255.0 / max } else { 0.0 };
    frame
        .points
        .iter()
        .map(|p| {
            let v = if p.intensity.is_finite() {
                (p.intensity * scale).clamp(0.0, 255.0) as u8
            } else {
                0
            };
            rerun::Color::from_rgb(v, v, v)
        })
        .collect()
}

/// Entity path of a camera topic: `/camera_left/image_raw` becomes `camera/left`,
/// `/alphasense/cam0/image_raw/compressed` becomes `camera/alphasense/cam0`.
fn camera_entity(topic: &str) -> String {
    let mut parts: Vec<&str> = topic.split('/').filter(|p| !p.is_empty()).collect();
    while parts.len() > 1 && matches!(parts.last(), Some(&("compressed" | "image_raw" | "image"))) {
        parts.pop();
    }
    if let Some(first) = parts.first_mut() {
        *first = first.strip_prefix("camera_").unwrap_or(first);
    }
    format!("camera/{}", parts.join("/"))
}

fn set_stamp(rec: &rerun::RecordingStream, stamp: f64) {
    rec.set_duration_secs(TIMELINE, stamp);
}

/// Topics to show from one bag.
struct BagPlan {
    path: PathBuf,
    lidar: Option<String>,
    imu: Option<String>,
    cameras: Vec<String>,
}

#[derive(Default)]
struct Counts {
    lidar: u64,
    imu: u64,
    images: u64,
}

/// Opens every bag once (index only) and decides which topics to log. Fails before the
/// viewer starts if the topic choice is ambiguous, an explicit name does not exist, or
/// there is no LiDAR topic at all.
fn plan_bags(
    bags: &[PathBuf],
    args: &Args,
    warnings: &mut Vec<String>,
) -> Result<Vec<BagPlan>, BoxError> {
    let mut lidar = TopicPicker::new(CLOUD_TYPE, args.lidar_topic.as_deref(), "--lidar-topic");
    let mut imu = TopicPicker::new(IMU_TYPE, args.imu_topic.as_deref(), "--imu-topic");
    let mut plans = Vec::new();
    let mut raw_images = std::collections::BTreeSet::new();
    for path in bags {
        let reader = BagReader::open(path).map_err(|e| format!("{}: {e}", file_name(path)))?;
        let mut cameras: Vec<String> = Vec::new();
        for c in reader.connections() {
            if c.msg_type == COMPRESSED_IMAGE_TYPE && !cameras.contains(&c.topic) {
                cameras.push(c.topic.clone());
            } else if c.msg_type == RAW_IMAGE_TYPE {
                raw_images.insert(c.topic.clone());
            }
        }
        cameras.sort();
        plans.push(BagPlan {
            path: path.clone(),
            lidar: lidar.pick(&reader)?,
            imu: imu.pick(&reader)?,
            cameras,
        });
    }
    lidar.finish()?;
    imu.finish()?;
    if plans.iter().all(|p| p.lidar.is_none()) {
        return Err(
            format!("kein {CLOUD_TYPE}-Topic in den Bags; es gibt nichts zu zeigen").into(),
        );
    }
    if !raw_images.is_empty() {
        let names: Vec<&str> = raw_images.iter().map(String::as_str).collect();
        warnings.push(format!(
            "{RAW_IMAGE_TYPE}-Topics ({}) werden noch nicht angezeigt, nur {COMPRESSED_IMAGE_TYPE}",
            names.join(", ")
        ));
    }
    Ok(plans)
}

fn log_bag(
    rec: &rerun::RecordingStream,
    plan: &BagPlan,
    every: u32,
    counts: &mut Counts,
    warnings: &mut Vec<String>,
) -> Result<(), BoxError> {
    let name = file_name(&plan.path);
    let topics: Vec<&str> = plan
        .lidar
        .iter()
        .chain(&plan.imu)
        .chain(&plan.cameras)
        .map(String::as_str)
        .collect();
    if topics.is_empty() {
        return Ok(());
    }
    let mut reader = BagReader::open(&plan.path)?;
    let mut image_count: std::collections::HashMap<String, u32> = Default::default();
    let mut decode_errors = 0u64;
    for item in reader.messages(&topics) {
        let msg = match item {
            Ok(m) => m,
            Err(BagError::Truncated { offset }) => {
                warnings.push(format!(
                    "{name}: Datei abgeschnitten bei Byte {offset}, Rest übersprungen"
                ));
                break;
            }
            Err(e) => return Err(format!("{name}: {e}").into()),
        };
        let topic = Some(&msg.topic);
        if topic == plan.lidar.as_ref() {
            match decode_pointcloud2(&msg.data) {
                Ok(f) => {
                    set_stamp(rec, f.stamp);
                    rec.log(
                        "lidar/points",
                        &rerun::Points3D::new(f.points.iter().map(|p| p.xyz))
                            .with_colors(intensity_colors(&f)),
                    )?;
                    counts.lidar += 1;
                }
                Err(_) => decode_errors += 1,
            }
        } else if topic == plan.imu.as_ref() {
            match decode_imu(&msg.data) {
                Ok(s) => {
                    set_stamp(rec, s.stamp);
                    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
                        rec.log(
                            format!("imu/gyro/{axis}"),
                            &rerun::Scalars::single(s.gyro[i]),
                        )?;
                        rec.log(
                            format!("imu/accel/{axis}"),
                            &rerun::Scalars::single(s.accel[i]),
                        )?;
                    }
                    counts.imu += 1;
                }
                Err(_) => decode_errors += 1,
            }
        } else {
            let n = image_count.entry(msg.topic.clone()).or_insert(0);
            let take = n.is_multiple_of(every);
            *n += 1;
            if !take {
                continue;
            }
            match decode_compressed_image(&msg.data) {
                Ok(img) => {
                    set_stamp(rec, img.stamp);
                    rec.log(
                        camera_entity(&msg.topic),
                        &rerun::EncodedImage::from_file_contents(img.data),
                    )?;
                    counts.images += 1;
                }
                Err(_) => decode_errors += 1,
            }
        }
    }
    if decode_errors > 0 {
        warnings.push(format!("{name}: {decode_errors} Nachrichten nicht lesbar"));
    }
    Ok(())
}

pub fn run(args: &Args) -> Result<(), BoxError> {
    let project = discover_project(&args.folder)?;
    for p in &project.problems {
        eprintln!("Hinweis: {p}");
    }
    // Everything that can fail on the input is checked before a viewer window opens.
    let line: Option<Vec<[f32; 3]>> = match &args.traj {
        Some(traj) => Some(
            read_tum(traj)?
                .iter()
                .map(|p| [p.p.x as f32, p.p.y as f32, p.p.z as f32])
                .collect(),
        ),
        None => None,
    };
    let mut warnings = Vec::new();
    let all_bags: Vec<PathBuf> = project
        .lidar_bags
        .iter()
        .chain(&project.image_bags)
        .cloned()
        .collect();
    let plans = plan_bags(&all_bags, args, &mut warnings)?;

    let builder = rerun::RecordingStreamBuilder::new("driftless");
    let rec = match &args.dry_run {
        Some(file) => builder.save(file)?,
        None => builder.spawn().map_err(|e| {
            format!(
                "Viewer konnte nicht gestartet werden: {e}\n\
                 Der rerun-Viewer muss installiert sein: cargo install rerun-cli --version 0.38.1 --locked"
            )
        })?,
    };
    if let Some(line) = line {
        rec.log_static("trajectory", &rerun::LineStrips3D::new([line]))?;
    }
    let mut counts = Counts::default();
    for plan in &plans {
        log_bag(&rec, plan, args.every, &mut counts, &mut warnings)?;
    }
    rec.flush_blocking()?;
    for w in &warnings {
        eprintln!("Warnung: {w}");
    }
    println!(
        "{} LiDAR-Frames, {} IMU-Messungen, {} Kamerabilder geloggt",
        counts.lidar, counts.imu, counts.images
    );
    if let Some(file) = &args.dry_run {
        println!("Aufzeichnung geschrieben: {}", file.display());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::camera_entity;

    #[test]
    fn camera_entities() {
        assert_eq!(camera_entity("/camera_left/image_raw"), "camera/left");
        assert_eq!(
            camera_entity("/alphasense/cam0/image_raw/compressed"),
            "camera/alphasense/cam0"
        );
        assert_eq!(camera_entity("/cam"), "camera/cam");
    }
}
