//! `driftless surface <scan> --slam <ziel> [--map <karte>] [-o <ziel>]`: the photo-like
//! surface of a map as glb and obj (`oberflaeche.glb`, `oberflaeche.obj` with `.mtl` and
//! texture pages, `oberflaeche.json`). The run itself is
//! [`driftless_steps::surface::run_surface`].

use driftless_gpu::GpuPref;
use driftless_map::surface::SurfaceConfig;
use driftless_steps::map::{MapInput, MapSettings, MapStep, MapSwitches, REPORT};
use driftless_steps::scan::extrinsic_values;
use driftless_steps::slam::SlamSettings;
use driftless_steps::surface::{run_surface, SurfaceRequest};
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag; für die Textur auch IMAGE_*.bag und calibration/calib.json)
    scan: PathBuf,
    /// Zielordner von `driftless slam` (liest traj.tum)
    #[arg(long, value_name = "ZIEL")]
    slam: PathBuf,
    /// Ordner der Karte (mit clean.bin und map.json von `driftless map`); Standard: der
    /// SLAM-Ordner
    #[arg(long, value_name = "KARTE")]
    map: Option<PathBuf>,
    /// Zielordner der Oberfläche (Standard: <karte>/oberflaeche)
    #[arg(short, long, value_name = "ZIEL")]
    output: Option<PathBuf>,
    /// Voxelgröße der Oberfläche in m (kleiner = feiner, mehr Speicher und Zeit)
    #[arg(long, value_name = "M", default_value_t = SurfaceConfig::default().voxel)]
    voxel: f64,
    /// Keine Textur aus den Kamerabildern, nur graue Flächen
    #[arg(long)]
    no_texture: bool,
    /// LiDAR-Lage im IMU-Rahmen: x y z qx qy qz qw (wie bei `slam` und `map`; Standard: Identität)
    #[arg(long, num_args = 7, value_names = ["X", "Y", "Z", "QX", "QY", "QZ", "QW"], allow_negative_numbers = true)]
    lidar_to_imu: Option<Vec<f64>>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
    /// Grafikkarte für die Ansichten je Fläche: auto (nutzen, wenn eine taugliche da ist, sonst
    /// CPU mit Hinweis), an (verlangen) oder aus (nur CPU). Das Ergebnis ist gleichwertig.
    #[arg(long, value_name = "auto|an|aus", default_value_t = GpuPref::Auto)]
    gpu: GpuPref,
}

fn config(args: &Args) -> SurfaceConfig {
    SurfaceConfig {
        voxel: args.voxel,
        ..SurfaceConfig::default()
    }
}

/// The switches the map was made with (its `map.json`), else the defaults of `driftless map`.
fn map_switches(map_dir: &std::path::Path) -> MapSwitches {
    std::fs::read_to_string(map_dir.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .and_then(|j| MapSwitches::from_map_json(&j))
        .unwrap_or_default()
}

fn request(args: &Args) -> Result<SurfaceRequest, Box<dyn std::error::Error>> {
    let map_dir = args.map.clone().unwrap_or_else(|| args.slam.clone());
    let out_dir = args
        .output
        .clone()
        .unwrap_or_else(|| map_dir.join("oberflaeche"));
    let slam = SlamSettings {
        lidar_to_imu: extrinsic_values(args.lidar_to_imu.as_deref())?,
        lidar_topic: args.lidar_topic.clone(),
        imu_topic: args.imu_topic.clone(),
    };
    let switches = map_switches(&map_dir);
    Ok(SurfaceRequest {
        map: MapInput {
            scan: args.scan.clone(),
            slam_dir: args.slam.clone(),
            slam,
            map: MapSettings {
                step: MapStep::All,
                switches,
            },
        },
        map_dir,
        out_dir,
        config: config(args),
        texture: !args.no_texture,
        gpu: args.gpu,
    })
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let req = request(args)?;
    req.config.check()?;
    run_surface(&req, &crate::console())?;
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

    fn parse(extra: &[&str]) -> SurfaceRequest {
        let argv = ["surface", "scan", "--slam", "ziel"].iter().chain(extra);
        request(&Cli::try_parse_from(argv).unwrap().args).unwrap()
    }

    /// `driftless surface` without options runs with the library defaults, in the SLAM
    /// folder.
    #[test]
    fn defaults_equal_library_defaults() {
        let r = parse(&[]);
        assert_eq!(r.config, SurfaceConfig::default());
        assert!(r.texture);
        assert_eq!(r.map_dir, PathBuf::from("ziel"));
        assert_eq!(r.out_dir, PathBuf::from("ziel/oberflaeche"));
        let r = parse(&["--voxel", "0.02", "--no-texture", "--map", "k", "-o", "x"]);
        assert_eq!(r.config.voxel, 0.02);
        assert!(!r.texture);
        assert_eq!(r.map_dir, PathBuf::from("k"));
        assert_eq!(r.out_dir, PathBuf::from("x"));
    }

    #[test]
    fn gpu_switch_parses() {
        assert_eq!(parse(&[]).gpu, GpuPref::Auto);
        assert_eq!(parse(&["--gpu", "an"]).gpu, GpuPref::On);
        assert_eq!(parse(&["--gpu", "aus"]).gpu, GpuPref::Off);
        let argv = ["surface", "scan", "--slam", "ziel", "--gpu", "ja"];
        assert!(Cli::try_parse_from(argv).is_err());
    }
}
