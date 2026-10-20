//! `driftless export-glim <ordner> -o <dir>`: writes LiDAR frames and IMU samples
//! as two little-endian binary streams for the GLIM spike driver (`spikes/glim/`).
//!
//! `imu.bin`:   magic `DLIMU01\0`, then per sample `f64 stamp, f64 ax ay az, f64 gx gy gz` (56 bytes).
//! `lidar.bin`: magic `DLPTS01\0`, then per frame `f64 stamp, u32 n` and n points of
//!              `f32 x y z, f32 intensity, f32 t_rel, u32 ring` (24 bytes).
//!
//! Records are written in bag order; invalid points are already dropped by the decoder.

use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2};
use driftless_io::project::discover_project;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Zielordner für imu.bin und lidar.bin
    #[arg(short = 'o', long = "out")]
    out: PathBuf,
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

    let (mut frames, mut samples, mut points, mut dropped, mut errors) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    for bag in &project.lidar_bags {
        let mut reader = BagReader::open(bag)?;
        let types: HashMap<String, String> = reader
            .connections()
            .iter()
            .map(|c| (c.topic.clone(), c.msg_type.clone()))
            .collect();
        for item in reader.messages(&[]) {
            let msg = match item {
                Ok(m) => m,
                Err(e) => {
                    // Truncated copy: keep everything read so far.
                    eprintln!("Warnung: {}: {e}", bag.display());
                    break;
                }
            };
            match types.get(&msg.topic).map(String::as_str) {
                Some("sensor_msgs/Imu") => match decode_imu(&msg.data) {
                    Ok(s) => {
                        for v in [s.stamp].iter().chain(&s.accel).chain(&s.gyro) {
                            imu.write_all(&v.to_le_bytes())?;
                        }
                        samples += 1;
                    }
                    Err(_) => errors += 1,
                },
                Some("sensor_msgs/PointCloud2") => match decode_pointcloud2(&msg.data) {
                    Ok(f) => {
                        lidar.write_all(&f.stamp.to_le_bytes())?;
                        lidar.write_all(&(f.points.len() as u32).to_le_bytes())?;
                        for p in &f.points {
                            for v in [p.xyz[0], p.xyz[1], p.xyz[2], p.intensity, p.t_rel as f32] {
                                lidar.write_all(&v.to_le_bytes())?;
                            }
                            lidar.write_all(&u32::from(p.ring).to_le_bytes())?;
                        }
                        frames += 1;
                        points += f.points.len() as u64;
                        dropped += u64::from(f.dropped);
                    }
                    Err(_) => errors += 1,
                },
                _ => {}
            }
        }
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
