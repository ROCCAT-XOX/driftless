//! `driftless export-glim <ordner> -o <dir>`: writes LiDAR frames and IMU samples
//! as two little-endian binary streams for the GLIM spike driver (`spikes/glim/`).
//!
//! `imu.bin`:   magic `DLIMU01\0`, then per sample `f64 stamp, f64 ax ay az, f64 gx gy gz` (56 bytes).
//! `lidar.bin`: magic `DLPTS01\0`, then per frame `f64 stamp, u32 n` and n points of
//!              `f32 x y z, f32 intensity, f32 t_rel, u32 ring` (24 bytes).
//!
//! Records are written in bag order; invalid points are already dropped by the decoder.
//! Absolute per-point stamps (Hesai) are turned into offsets from the frame stamp, offsets
//! below zero are written as 0 (same rule as the odometry, `driftless_odom::deskew`).

use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2};
use driftless_io::project::discover_project;
use driftless_odom::deskew::point_time_base;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use crate::topics::{bag_read_error, file_name, TopicPicker};

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Zielordner für imu.bin und lidar.bin
    #[arg(short = 'o', long = "out")]
    out: PathBuf,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
}

const IMU_MAGIC: &[u8; 8] = b"DLIMU01\0";
const LIDAR_MAGIC: &[u8; 8] = b"DLPTS01\0";

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let project = discover_project(&args.folder)?;
    fs::create_dir_all(&args.out)?;
    let mut imu = BufWriter::new(File::create(args.out.join("imu.bin"))?);
    let mut lidar = BufWriter::new(File::create(args.out.join("lidar.bin"))?);
    imu.write_all(IMU_MAGIC)?;
    lidar.write_all(LIDAR_MAGIC)?;

    let mut lidar_picker = TopicPicker::new(
        "sensor_msgs/PointCloud2",
        args.lidar_topic.as_deref(),
        "--lidar-topic",
    );
    let mut imu_picker =
        TopicPicker::new("sensor_msgs/Imu", args.imu_topic.as_deref(), "--imu-topic");
    let (mut frames, mut samples, mut points, mut dropped, mut errors) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    for bag in &project.lidar_bags {
        let mut reader = BagReader::open(bag).map_err(|e| format!("{}: {e}", file_name(bag)))?;
        let lidar_topic = lidar_picker.pick(&reader)?;
        let imu_topic = imu_picker.pick(&reader)?;
        let topics: Vec<&str> = lidar_topic
            .iter()
            .chain(&imu_topic)
            .map(String::as_str)
            .collect();
        if topics.is_empty() {
            continue;
        }
        for item in reader.messages(&topics) {
            let msg = match item {
                Ok(m) => m,
                Err(e) => {
                    // Truncated copy: keep everything read so far; anything else fails.
                    bag_read_error(bag, e)?;
                    break;
                }
            };
            if imu_topic.as_ref() == Some(&msg.topic) {
                match decode_imu(&msg.data) {
                    Ok(s) => {
                        for v in [s.stamp].iter().chain(&s.accel).chain(&s.gyro) {
                            imu.write_all(&v.to_le_bytes())?;
                        }
                        samples += 1;
                    }
                    Err(_) => errors += 1,
                }
            } else {
                match decode_pointcloud2(&msg.data) {
                    Ok(f) => {
                        let base = point_time_base(&f);
                        lidar.write_all(&f.stamp.to_le_bytes())?;
                        lidar.write_all(&(f.points.len() as u32).to_le_bytes())?;
                        for p in &f.points {
                            let t_rel = (p.t_rel - base).max(0.0) as f32;
                            for v in [p.xyz[0], p.xyz[1], p.xyz[2], p.intensity, t_rel] {
                                lidar.write_all(&v.to_le_bytes())?;
                            }
                            lidar.write_all(&u32::from(p.ring).to_le_bytes())?;
                        }
                        frames += 1;
                        points += f.points.len() as u64;
                        dropped += u64::from(f.dropped);
                    }
                    Err(_) => errors += 1,
                }
            }
        }
    }
    lidar_picker.finish()?;
    imu_picker.finish()?;
    if frames == 0 {
        return Err("keine LiDAR-Frames (sensor_msgs/PointCloud2) gelesen".into());
    }
    imu.flush()?;
    lidar.flush()?;
    println!(
        "{frames} LiDAR-Frames ({points} Punkte, {dropped} verworfen), {samples} IMU-Messungen, \
         {errors} nicht lesbare Nachrichten → {}",
        args.out.display()
    );
    Ok(())
}
