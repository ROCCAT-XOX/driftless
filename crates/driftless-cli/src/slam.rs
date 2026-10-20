//! `driftless slam <ordner> -o <out_dir>`: odometry, submaps, loop closure. Writes
//! `traj.tum` (corrected), `traj_odom.tum` (odometry before loop closure), `map.las` (submaps
//! at the corrected anchors, 10 cm grid) and `slam.json` (submaps, loop checks, graph).

use crate::scan_input::{
    parse_extrinsic, read_imu, require_frames, warn_if_not_static, with_lidar_frames, write_las,
};
use driftless_io::project::discover_project;
use driftless_odom::OdomConfig;
use driftless_slam::{run_slam, LoopVerdict, SlamConfig, SlamReport};
use driftless_traj::write_tum;
use std::fs;
use std::path::PathBuf;
use std::time::Instant;

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Zielordner für traj.tum, traj_odom.tum, map.las und slam.json
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
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    let cfg = SlamConfig {
        odom: OdomConfig {
            lidar_to_imu: parse_extrinsic(args.lidar_to_imu.as_deref())?,
            ..OdomConfig::default()
        },
        ..SlamConfig::default()
    };
    let project = discover_project(&args.folder)?;
    fs::create_dir_all(&args.out)?;

    let (imu, bad) = read_imu(&project.lidar_bags, args.imu_topic.as_deref())?;
    let ((result, stats), read) =
        with_lidar_frames(&project.lidar_bags, args.lidar_topic.as_deref(), |frames| {
            run_slam(frames, &imu, &cfg)
        })?;
    require_frames(&stats)?;

    let report = SlamReport::new(&result);
    let poses = result.poses();
    write_tum(&args.out.join("traj.tum"), &poses)?;
    write_tum(&args.out.join("traj_odom.tum"), &result.odom_poses())?;
    let map = result.map_points();
    write_las(&args.out.join("map.las"), &map)?;
    fs::write(
        args.out.join("slam.json"),
        serde_json::to_string_pretty(&report)? + "\n",
    )?;

    warn_if_not_static(&stats);
    println!(
        "{} Posen aus {} LiDAR-Frames ({} übersprungen), {} IMU-Messungen, {} Kartenpunkte, \
         {} nicht lesbare Nachrichten",
        poses.len(),
        stats.frames_in,
        stats.frames_skipped,
        imu.len(),
        map.len(),
        bad + read.bad
    );
    let accepted = result
        .loops
        .iter()
        .filter(|l| l.verdict == LoopVerdict::Accepted)
        .count();
    let rejected = result.loops.len() - accepted;
    let loops = if accepted == 0 {
        format!("keine Schleife gefunden ({rejected} Kandidaten verworfen)")
    } else {
        format!("Schleifen: {accepted} akzeptiert, {rejected} verworfen")
    };
    println!("{} Teilkarten, {loops}", result.submaps.len());
    println!(
        "Start↔Ende {:.3} m → {:.3} m, Länge {:.1} m → {:.1} m",
        report.start_end_gap.before,
        report.start_end_gap.after,
        report.path_length.before,
        report.path_length.after
    );
    println!(
        "Laufzeit {:.1} s → {}",
        start.elapsed().as_secs_f64(),
        args.out.display()
    );
    Ok(())
}
