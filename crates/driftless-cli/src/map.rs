//! `driftless map <scan> --slam <ziel> [--step dense|clean|color|all]`: the map of a SLAM run
//! in three steps, each with its own files in the SLAM output folder: `dense.las`/`dense.bin`,
//! `clean.las`/`removed.las`/`clean.bin`, `color.las`/`color.ply`, plus `map.json`. The run
//! itself is [`driftless_steps::map::run_map`].

use driftless_map::{ColorConfig, DenseConfig};
use driftless_steps::map::{run_map, MapInput, MapRequest, MapSettings, MapStep, MapSwitches};
use driftless_steps::scan::extrinsic_values;
use driftless_steps::slam::SlamSettings;
use std::path::PathBuf;

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum Step {
    Dense,
    Clean,
    Color,
    All,
}

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag; zum Einfärben auch IMAGE_*.bag und calibration/calib.json)
    scan: PathBuf,
    /// Zielordner von `driftless slam` (liest traj.tum); die Ergebnisse landen ebenfalls dort
    #[arg(long, value_name = "ZIEL")]
    slam: PathBuf,
    /// Schritt: dense (dichte Wolke), clean (bereinigt), color (eingefärbt) oder all. Das
    /// Ergebnis des vorigen Schritts wird aus dem Zielordner genommen, wenn es zu Eingabe und
    /// Einstellungen passt, sonst neu erzeugt
    #[arg(long, value_enum, default_value_t = Step::All)]
    step: Step,
    /// Geister (Bewegtes) nicht entfernen
    #[arg(long)]
    no_ghosts: bool,
    /// Ausreißer nicht entfernen
    #[arg(long)]
    no_outliers: bool,
    /// Durchblick-Punkte (hinter Fenstern gemessen) nicht entfernen
    #[arg(long)]
    no_through: bool,
    /// Wände nicht auf ihre lokale Ebene glätten
    #[arg(long)]
    no_smooth: bool,
    /// Glanzlichter beim Einfärben abwerten (experimentell: senkt Spiegelungen auf Böden, macht
    /// aber die Übergänge zwischen Bildern sichtbarer)
    #[arg(long)]
    highlights: bool,
    /// Randabfall der Kameras nicht mitschätzen
    #[arg(long)]
    no_vignetting: bool,
    /// Voxelgröße der dichten Wolke in m (ein Punkt je Voxel)
    #[arg(long, value_name = "M", default_value_t = DenseConfig::default().voxel)]
    voxel: f64,
    /// Bilder zum Einfärben verkleinert dekodieren: 1, 2 oder 4
    #[arg(long, default_value_t = ColorConfig::default().downscale)]
    downscale: u32,
    /// LiDAR-Lage im IMU-Rahmen: x y z qx qy qz qw (wie bei `slam`; Standard: Identität)
    #[arg(long, num_args = 7, value_names = ["X", "Y", "Z", "QX", "QY", "QZ", "QW"], allow_negative_numbers = true)]
    lidar_to_imu: Option<Vec<f64>>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
}

impl From<Step> for MapStep {
    fn from(s: Step) -> Self {
        match s {
            Step::Dense => MapStep::Dense,
            Step::Clean => MapStep::Clean,
            Step::Color => MapStep::Color,
            Step::All => MapStep::All,
        }
    }
}

fn settings(args: &Args) -> MapSettings {
    MapSettings {
        step: args.step.into(),
        switches: MapSwitches {
            ghosts: !args.no_ghosts,
            outliers: !args.no_outliers,
            through: !args.no_through,
            smooth: !args.no_smooth,
            highlights: args.highlights,
            vignetting: !args.no_vignetting,
            voxel: args.voxel,
            downscale: args.downscale,
        },
    }
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let map = settings(args);
    map.check()?;
    let slam = SlamSettings {
        lidar_to_imu: extrinsic_values(args.lidar_to_imu.as_deref())?,
        lidar_topic: args.lidar_topic.clone(),
        imu_topic: args.imu_topic.clone(),
    };
    let req = MapRequest {
        input: MapInput {
            scan: args.scan.clone(),
            slam_dir: args.slam.clone(),
            slam,
            map,
        },
        out_dir: args.slam.clone(),
    };
    run_map(&req, &crate::console())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[derive(Parser)]
    struct Cli {
        #[command(flatten)]
        args: Args,
    }

    fn parse(extra: &[&str]) -> MapSettings {
        let argv = ["map", "scan", "--slam", "ziel"].iter().chain(extra);
        settings(&Cli::try_parse_from(argv).unwrap().args)
    }

    /// `driftless map` without options runs with the library defaults.
    #[test]
    fn settings_default_equals_cli_defaults() {
        assert_eq!(parse(&[]), MapSettings::default());
        let all_off = parse(&[
            "--no-ghosts",
            "--no-outliers",
            "--no-through",
            "--no-smooth",
            "--highlights",
            "--no-vignetting",
            "--step",
            "clean",
        ]);
        assert_eq!(all_off.step, MapStep::Clean);
        let s = &all_off.switches;
        assert!(!s.ghosts && !s.outliers && !s.through && !s.smooth && !s.vignetting);
        assert!(s.highlights);
    }
}
