//! `driftless odom <ordner> -o <out_dir>`: LiDAR-IMU odometry (Rust baseline, no loop closure).
//! Writes `traj.tum` (body poses) and `map.las` (deskewed points, 10 cm grid).
//!
//! The IMU is read first (small), the LiDAR frames are then streamed from a reader thread.

use crate::topics::{bag_read_error, file_name, TopicPicker};
use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_odom::{run_odometry, OdomConfig};
use driftless_traj::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use driftless_traj::{path_length, start_end_gap, write_tum};
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
    /// LiDAR-Lage im IMU-Rahmen: x y z qx qy qz qw (Standard: Identität)
    #[arg(long, num_args = 7, value_names = ["X", "Y", "Z", "QX", "QY", "QZ", "QW"], allow_negative_numbers = true)]
    lidar_to_imu: Option<Vec<f64>>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
    /// Punkte nicht mit der IMU-Bewegung entzerren (nur zum Vergleich)
    #[arg(long)]
    no_deskew: bool,
}

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

/// `x y z qx qy qz qw` (LiDAR pose in the IMU frame) → isometry. The quaternion is normalised.
fn parse_extrinsic(v: &[f64]) -> Result<Isometry3<f64>, String> {
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
    let cfg = OdomConfig {
        lidar_to_imu: match &args.lidar_to_imu {
            Some(v) => parse_extrinsic(v)?,
            None => Isometry3::identity(),
        },
        deskew: !args.no_deskew,
        ..OdomConfig::default()
    };
    let project = discover_project(&args.folder)?;
    fs::create_dir_all(&args.out)?;

    let mut imu: Vec<ImuSample> = Vec::new();
    let mut bad = 0usize;
    let imu_topic = args.imu_topic.as_deref();
    for_each_message(
        &project.lidar_bags,
        IMU_TYPE,
        imu_topic,
        "--imu-topic",
        |data| {
            match decode_imu(data) {
                Ok(s) => imu.push(s),
                Err(_) => bad += 1,
            }
            true
        },
    )?;

    if imu.is_empty() {
        return Err(format!(
            "keine IMU-Messungen ({IMU_TYPE}) gelesen; die Odometrie braucht eine IMU"
        )
        .into());
    }

    let bags = project.lidar_bags.clone();
    let lidar_topic = args.lidar_topic.clone();
    let (tx, rx) = sync_channel::<LidarFrame>(16);
    let reader = std::thread::spawn(move || {
        let (mut bad, mut dropped) = (0usize, 0u64);
        let r = for_each_message(
            &bags,
            CLOUD_TYPE,
            lidar_topic.as_deref(),
            "--lidar-topic",
            |data| match decode_pointcloud2(data) {
                Ok(f) => {
                    dropped += u64::from(f.dropped);
                    tx.send(f).is_ok()
                }
                Err(_) => {
                    bad += 1;
                    true
                }
            },
        );
        r.map(|()| (bad, dropped))
    });
    let result = run_odometry(rx.into_iter(), &imu, &cfg);
    let (bad_frames, dropped) = reader.join().map_err(|_| "Lesethread abgestürzt")??;
    bad += bad_frames;

    if result.stats.frames_in == 0 {
        return Err(format!("keine LiDAR-Frames ({CLOUD_TYPE}) gelesen").into());
    }
    write_tum(&args.out.join("traj.tum"), &result.poses)?;
    write_las(&args.out.join("map.las"), &result.map_points)?;

    let s = &result.stats;
    if !imu.is_empty() && !s.static_start {
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
    let c = &s.imu;
    println!(
        "IMU bereinigt: {} umsortiert, {} doppelte, {} nicht endliche, {} Lücken; \
         Punktzeiten: {} Punkte übersprungen, {} auf 0 begrenzt; Decoder verwarf {} Punkte",
        if c.reordered { "ja" } else { "nein" },
        c.duplicates,
        c.non_finite,
        c.gaps,
        s.points_time_skipped,
        s.points_time_clamped,
        dropped
    );
    println!(
        "Laufzeit {:.1} s{} → {}",
        start.elapsed().as_secs_f64(),
        if cfg.deskew { "" } else { " (ohne Deskew)" },
        args.out.display()
    );
    Ok(())
}
