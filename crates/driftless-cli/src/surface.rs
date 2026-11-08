//! `driftless surface <scan> --slam <ziel> [--map <karte>] [-o <ziel>] [--obj]`: the
//! photo-like surface of a map as glb (`oberflaeche.glb`, `oberflaeche.json`; with `--obj`
//! also `oberflaeche.obj` with `.mtl` and texture pages). The run itself is
//! [`driftless_steps::surface::run_surface`]; with `--model <modell>` instead of the scan
//! the surface of a merged model ([`driftless_steps::surface::run_model_surface`]).

use driftless_gpu::GpuPref;
use driftless_map::surface::{NoiseConfig, SurfaceConfig};
use driftless_steps::map::map_input_for;
use driftless_steps::scan::extrinsic_values;
use driftless_steps::slam::SlamSettings;
use driftless_steps::surface::{
    run_model_surface, run_surface, ModelSurfaceRequest, SurfaceRequest,
};
use std::path::{Path, PathBuf};

/// What `--ohne` switches off.
#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Without {
    /// Ecken nicht an die Ebene der Messpunkte legen
    Anlegen,
    /// Gut belichtete Bilder nicht bevorzugen
    Belichtung,
    /// Bildrauschen nicht dämpfen
    Entrauschen,
    /// Dünne Teile ohne Fläche verdecken die Wand dahinter nicht
    Verdeckung,
    /// Alle vier abschalten
    Alles,
}

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag; für die Textur auch IMAGE_*.bag und calibration/calib.json)
    #[arg(required_unless_present = "model")]
    scan: Option<PathBuf>,
    /// Zielordner von `driftless slam` (liest traj.tum)
    #[arg(long, value_name = "ZIEL", required_unless_present = "model")]
    slam: Option<PathBuf>,
    /// Ordner eines Modells von `driftless merge` (mit modell.json) statt eines Scans: die
    /// Oberfläche aus den Punkten und Bildern aller Scans im Modell
    #[arg(
        long,
        value_name = "MODELL",
        conflicts_with_all = ["scan", "slam", "map", "lidar_to_imu", "lidar_topic", "imu_topic"]
    )]
    model: Option<PathBuf>,
    /// Ordner der Karte (mit clean.bin und map.json von `driftless map`); Standard: der
    /// SLAM-Ordner
    #[arg(long, value_name = "KARTE")]
    map: Option<PathBuf>,
    /// Zielordner der Oberfläche (Standard: <karte>/oberflaeche, mit --model <modell>/oberflaeche)
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
    /// Auch oberflaeche.obj mit .mtl und den Texturseiten als JPEG schreiben (Z oben); sonst
    /// nur oberflaeche.glb (ein OBJ macht auch `driftless export`)
    #[arg(long)]
    obj: bool,
    /// Maßnahmen gegen Rauschen abschalten, zum Vergleich; mehrere mit Komma getrennt
    #[arg(long, value_enum, value_name = "WAS", value_delimiter = ',')]
    ohne: Vec<Without>,
}

/// All measures against noise except those in `without`.
fn noise(without: &[Without]) -> NoiseConfig {
    let mut n = NoiseConfig::ON;
    for w in without {
        match w {
            Without::Anlegen => n.snap = false,
            Without::Belichtung => n.exposure = false,
            Without::Entrauschen => n.denoise = false,
            Without::Verdeckung => n.occlude = false,
            Without::Alles => n = NoiseConfig::OFF,
        }
    }
    n
}

fn config(args: &Args) -> SurfaceConfig {
    SurfaceConfig {
        voxel: args.voxel,
        noise: noise(&args.ohne),
        ..SurfaceConfig::default()
    }
}

fn request(args: &Args) -> Result<SurfaceRequest, Box<dyn std::error::Error>> {
    let (Some(scan), Some(slam_dir)) = (&args.scan, &args.slam) else {
        return Err("Scan-Ordner und --slam angeben, oder --model".into());
    };
    let map_dir = args.map.clone().unwrap_or_else(|| slam_dir.clone());
    let out_dir = args
        .output
        .clone()
        .unwrap_or_else(|| map_dir.join("oberflaeche"));
    let slam = SlamSettings {
        lidar_to_imu: extrinsic_values(args.lidar_to_imu.as_deref())?,
        lidar_topic: args.lidar_topic.clone(),
        imu_topic: args.imu_topic.clone(),
    };
    Ok(SurfaceRequest {
        map: map_input_for(scan, slam_dir, &map_dir, slam),
        map_dir,
        out_dir,
        config: config(args),
        texture: !args.no_texture,
        gpu: args.gpu,
        obj: args.obj,
    })
}

fn model_request(args: &Args, model: &Path) -> ModelSurfaceRequest {
    ModelSurfaceRequest {
        model_dir: model.to_path_buf(),
        model: None,
        out_dir: args
            .output
            .clone()
            .unwrap_or_else(|| model.join("oberflaeche")),
        config: config(args),
        texture: !args.no_texture,
        gpu: args.gpu,
        obj: args.obj,
    }
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(model) = &args.model {
        let req = model_request(args, model);
        req.config.check()?;
        run_model_surface(&req, &crate::console())?;
        return Ok(());
    }
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
    fn model_needs_no_scan() {
        let argv = ["surface", "--model", "m", "--voxel", "0.05", "--no-texture"];
        let args = Cli::try_parse_from(argv).unwrap().args;
        let model = args.model.clone().unwrap();
        let r = model_request(&args, &model);
        assert_eq!(r.model_dir, PathBuf::from("m"));
        assert_eq!(r.out_dir, PathBuf::from("m/oberflaeche"));
        assert_eq!(r.config.voxel, 0.05);
        assert!(!r.texture);
        let argv = ["surface", "--model", "m", "-o", "x", "--gpu", "aus"];
        let args = Cli::try_parse_from(argv).unwrap().args;
        let r = model_request(&args, Path::new("m"));
        assert_eq!(r.out_dir, PathBuf::from("x"));
        assert_eq!(r.gpu, GpuPref::Off);
        for argv in [
            &["surface"][..],
            &["surface", "scan"],
            &["surface", "--slam", "ziel"],
            &["surface", "scan", "--slam", "ziel", "--model", "m"],
            &["surface", "--model", "m", "--map", "k"],
        ] {
            assert!(Cli::try_parse_from(argv).is_err(), "{argv:?}");
        }
    }

    #[test]
    fn gpu_switch_parses() {
        assert_eq!(parse(&[]).gpu, GpuPref::Auto);
        assert_eq!(parse(&["--gpu", "an"]).gpu, GpuPref::On);
        assert_eq!(parse(&["--gpu", "aus"]).gpu, GpuPref::Off);
        let argv = ["surface", "scan", "--slam", "ziel", "--gpu", "ja"];
        assert!(Cli::try_parse_from(argv).is_err());
    }

    #[test]
    fn ohne_switches_measures_off() {
        let noise = |extra: &[&str]| parse(extra).config.noise;
        assert_eq!(noise(&[]), NoiseConfig::ON);
        assert_eq!(noise(&["--ohne", "alles"]), NoiseConfig::OFF);
        assert_eq!(
            noise(&["--ohne", "anlegen,verdeckung"]),
            NoiseConfig {
                snap: false,
                occlude: false,
                ..NoiseConfig::ON
            }
        );
        // Every word switches its own measure.
        let one = |word, off: fn(&mut NoiseConfig)| {
            let mut want = NoiseConfig::ON;
            off(&mut want);
            assert_eq!(noise(&["--ohne", word]), want, "{word}");
        };
        one("anlegen", |n| n.snap = false);
        one("belichtung", |n| n.exposure = false);
        one("entrauschen", |n| n.denoise = false);
        one("verdeckung", |n| n.occlude = false);
        // Given twice, or `alles` within a list.
        assert_eq!(
            noise(&["--ohne", "belichtung", "--ohne", "entrauschen"]),
            NoiseConfig {
                exposure: false,
                denoise: false,
                ..NoiseConfig::ON
            }
        );
        assert_eq!(noise(&["--ohne", "anlegen,alles"]), NoiseConfig::OFF);
    }

    #[test]
    fn ohne_works_for_a_model() {
        let argv = ["surface", "--model", "m", "--ohne", "entrauschen"];
        let args = Cli::try_parse_from(argv).unwrap().args;
        let r = model_request(&args, Path::new("m"));
        assert_eq!(
            r.config.noise,
            NoiseConfig {
                denoise: false,
                ..NoiseConfig::ON
            }
        );
        let args = Cli::try_parse_from(["surface", "--model", "m"])
            .unwrap()
            .args;
        assert_eq!(
            model_request(&args, Path::new("m")).config.noise,
            NoiseConfig::ON
        );
    }

    #[test]
    fn ohne_refuses_unknown_words() {
        for value in ["glaetten", "anlegen,glaetten", "Alles", ""] {
            let argv = ["surface", "scan", "--slam", "ziel", "--ohne", value];
            assert!(Cli::try_parse_from(argv).is_err(), "{value:?}");
        }
    }
}
