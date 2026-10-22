//! `driftless slam <ordner> -o <out_dir> [--reuse]`: odometry, submaps, loop closure. Writes
//! `traj.tum`, `traj_odom.tum`, `map.las`, `slam.json` and `submaps.bin`; `--reuse` only
//! recomputes from `submaps.bin` and keeps the previous `traj.tum` as `traj.vorher.tum`. The
//! run itself is [`driftless_steps::slam::run_slam`].

use driftless_steps::scan::extrinsic_values;
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::StepError;
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Zielordner für traj.tum, traj_odom.tum, map.las, slam.json und submaps.bin
    #[arg(short = 'o', long = "out")]
    out: PathBuf,
    /// Nur neu rechnen: Teilkarten aus submaps.bin laden, Korrekturen anwenden, Ausgaben neu
    /// schreiben (Sekunden statt Minuten). Die bisherige traj.tum wird zu traj.vorher.tum;
    /// eine ältere traj.vorher.tum wird dabei ersetzt.
    #[arg(long)]
    reuse: bool,
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
    let settings = SlamSettings {
        lidar_to_imu: extrinsic_values(args.lidar_to_imu.as_deref())?,
        lidar_topic: args.lidar_topic.clone(),
        imu_topic: args.imu_topic.clone(),
    };
    let req = SlamRequest {
        scan: args.folder.clone(),
        out: args.out.clone(),
        settings,
        reuse: args.reuse,
        history: false,
    };
    run_slam(&req, &crate::console()).map_err(|e| match e {
        StepError::BadCorrection(m) => StepError::Failed(format!(
            "{m}\nKorrektur mit `driftless correct {out} list` suchen, mit `driftless correct {out} \
             disable <nr>` abschalten und `driftless slam` mit denselben Angaben und `--reuse` \
             erneut starten (der Zwischenspeicher ist geschrieben)",
            out = args.out.display(),
        )),
        e => e,
    })?;
    Ok(())
}
