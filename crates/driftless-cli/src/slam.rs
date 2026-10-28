//! `driftless slam <ordner> -o <out_dir> [--reuse | --ueberschreiben]`: odometry, submaps,
//! loop closure. Writes `traj.tum`, `traj_odom.tum`, `map.las`, `slam.json` and `submaps.bin`;
//! `--reuse` only recomputes from `submaps.bin` and keeps the previous `traj.tum` as
//! `traj.vorher.tum`. A full run into a folder that holds a run is refused without
//! `--ueberschreiben`, one into a folder with the run of another scan always. The run itself
//! is [`driftless_steps::slam::run_slam`].

use driftless_steps::scan::extrinsic_values;
use driftless_steps::slam::{
    other_scan_refusal, run_files_in, run_slam, SlamRequest, SlamSettings, SLAM_REPORT,
};
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
    /// Einen Lauf im Zielordner durch einen vollen Lauf ersetzen; ohne das wird ein
    /// Zielordner mit einem Lauf abgelehnt (der einer Karte zugrunde liegen kann)
    #[arg(long, conflicts_with = "reuse")]
    ueberschreiben: bool,
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
    if !args.reuse {
        // The run of another scan first: that one is refused also with --ueberschreiben.
        if let Some(why) = other_scan_refusal(&req) {
            return Err(why.into());
        }
        let found = run_files_in(&args.out);
        if !args.ueberschreiben && !found.is_empty() {
            let run = if found.contains(&SLAM_REPORT) {
                "einen Lauf"
            } else {
                "einen unfertigen Lauf"
            };
            return Err(format!(
                "{} enthält schon {run} ({}) und wird nicht überschrieben – mit --reuse nur neu \
                 rechnen, mit --ueberschreiben ersetzen oder einen anderen Zielordner wählen",
                args.out.display(),
                found.join(", ")
            )
            .into());
        }
    }
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
