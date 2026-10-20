//! `driftless odom <ordner> -o <out_dir>`: LiDAR-IMU odometry (Rust baseline, no loop closure).
//! Writes `traj.tum` (body poses) and `map.las` (deskewed points, 10 cm grid).

use crate::scan_input::{
    parse_extrinsic, read_imu, require_frames, warn_if_not_static, with_lidar_frames, write_las,
};
use driftless_io::project::discover_project;
use driftless_odom::{run_odometry, OdomConfig};
use driftless_traj::{path_length, start_end_gap, write_tum};
use std::fs;
use std::path::PathBuf;
use std::time::Instant;

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

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    let cfg = OdomConfig {
        lidar_to_imu: parse_extrinsic(args.lidar_to_imu.as_deref())?,
        deskew: !args.no_deskew,
        ..OdomConfig::default()
    };
    let project = discover_project(&args.folder)?;
    fs::create_dir_all(&args.out)?;

    let (imu, mut bad) = read_imu(&project.lidar_bags, args.imu_topic.as_deref())?;
    let (result, read) =
        with_lidar_frames(&project.lidar_bags, args.lidar_topic.as_deref(), |frames| {
            run_odometry(frames, &imu, &cfg)
        })?;
    bad += read.bad;
    let dropped = read.dropped;

    require_frames(&result.stats)?;
    write_tum(&args.out.join("traj.tum"), &result.poses)?;
    write_las(&args.out.join("map.las"), &result.map_points)?;

    let s = &result.stats;
    warn_if_not_static(s);
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
