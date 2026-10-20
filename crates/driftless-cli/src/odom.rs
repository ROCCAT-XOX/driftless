//! `driftless odom <ordner> -o <out_dir>`: LiDAR-IMU odometry (Rust baseline, no loop closure).
//! Writes `traj.tum` (body poses) and `map.las` (deskewed points, 10 cm grid).
//!
//! The IMU is read first (small), the LiDAR frames are then streamed from a reader thread.

use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_odom::{run_odometry, OdomConfig};
use driftless_traj::{path_length, start_end_gap, write_tum};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::mpsc::sync_channel;
use std::time::Instant;

const IMU_TYPE: &str = "sensor_msgs/Imu";
const CLOUD_TYPE: &str = "sensor_msgs/PointCloud2";

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Zielordner für traj.tum und map.las
    #[arg(short = 'o', long = "out")]
    out: PathBuf,
}

/// Topics of `msg_type` in this bag.
fn topics_of(reader: &BagReader<std::io::BufReader<fs::File>>, msg_type: &str) -> Vec<String> {
    let types: HashMap<&str, &str> = reader
        .connections()
        .iter()
        .map(|c| (c.topic.as_str(), c.msg_type.as_str()))
        .collect();
    let mut t: Vec<String> = types
        .into_iter()
        .filter(|(_, ty)| *ty == msg_type)
        .map(|(topic, _)| topic.to_string())
        .collect();
    t.sort();
    t
}

/// Calls `f` with every message of `msg_type` in `bags` until it returns `false`. A read error
/// (truncated copy) ends that bag with a warning; everything before it is kept.
fn for_each_message(
    bags: &[PathBuf],
    msg_type: &str,
    mut f: impl FnMut(&[u8]) -> bool,
) -> Result<(), String> {
    for bag in bags {
        let mut reader = BagReader::open(bag).map_err(|e| format!("{}: {e}", bag.display()))?;
        let topics = topics_of(&reader, msg_type);
        if topics.is_empty() {
            continue;
        }
        let topics: Vec<&str> = topics.iter().map(String::as_str).collect();
        for item in reader.messages(&topics) {
            match item {
                Ok(m) => {
                    if !f(&m.data) {
                        return Ok(());
                    }
                }
                Err(e) => {
                    eprintln!("Warnung: {}: {e}", bag.display());
                    break;
                }
            }
        }
    }
    Ok(())
}

fn write_las(path: &Path, points: &[[f32; 3]]) -> Result<(), Box<dyn std::error::Error>> {
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

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    let project = discover_project(&args.folder)?;
    fs::create_dir_all(&args.out)?;

    let mut imu: Vec<ImuSample> = Vec::new();
    let mut bad = 0usize;
    for_each_message(&project.lidar_bags, IMU_TYPE, |data| {
        match decode_imu(data) {
            Ok(s) => imu.push(s),
            Err(_) => bad += 1,
        }
        true
    })?;

    let bags = project.lidar_bags.clone();
    let (tx, rx) = sync_channel::<LidarFrame>(16);
    let reader = std::thread::spawn(move || {
        let mut bad = 0usize;
        let r = for_each_message(&bags, CLOUD_TYPE, |data| match decode_pointcloud2(data) {
            Ok(f) => tx.send(f).is_ok(),
            Err(_) => {
                bad += 1;
                true
            }
        });
        r.map(|()| bad)
    });
    let result = run_odometry(rx.into_iter(), &imu, &OdomConfig::default());
    bad += reader.join().map_err(|_| "Lesethread abgestürzt")??;

    write_tum(&args.out.join("traj.tum"), &result.poses)?;
    write_las(&args.out.join("map.las"), &result.map_points)?;

    let s = &result.stats;
    if !s.static_start {
        eprintln!(
            "Warnung: Gerät stand in der ersten Sekunde nicht still; Schwerkraft und Kreisel-Offset \
             sind nur grob geschätzt"
        );
    }
    println!(
        "{} Posen aus {} LiDAR-Frames ({} übersprungen), {} IMU-Messungen, {} Kartenpunkte",
        result.poses.len(),
        s.frames_in,
        s.frames_skipped,
        imu.len(),
        result.map_points.len()
    );
    println!(
        "Länge {:.1} m, Start↔Ende {:.3} m, ICP fehlgeschlagen in {} Frames, {} nicht lesbare Nachrichten",
        path_length(&result.poses),
        start_end_gap(&result.poses),
        s.icp_failures,
        bad
    );
    println!(
        "Laufzeit {:.1} s → {}",
        start.elapsed().as_secs_f64(),
        args.out.display()
    );
    Ok(())
}
