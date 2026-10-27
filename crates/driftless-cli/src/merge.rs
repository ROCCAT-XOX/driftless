//! `driftless merge <scan>... -o <ziel>`: several scans of one place merged into one model
//! (`modell.las`, `modell.ply`, `modell.json`). The first scan is the reference; its frame is
//! the model frame. The run itself is [`driftless_steps::merge::run_merge`].
//!
//! `--punktpaare <datei>` takes point pairs marked by hand (a file like `punktpaare.toml`,
//! [`driftless_steps::pairs`]); its map folders are relative to the folder of the file.

use driftless_slam::align::AlignConfig;
use driftless_steps::merge::{pair_members, run_merge, MergeMember, MergeRequest, ModelReport};
use driftless_steps::pairs::PointPairs;
use driftless_steps::scan::extrinsic_values;
use driftless_steps::slam::SlamSettings;
use std::path::{Path, PathBuf};

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag), der erste ist der Bezug
    #[arg(required = true, value_name = "SCAN")]
    scans: Vec<PathBuf>,
    /// Zielordner des Modells (darf noch kein Modell enthalten)
    #[arg(short = 'o', long = "out", value_name = "ZIEL")]
    out: PathBuf,
    /// Lauf-Ordner von `driftless slam` (liest traj.tum), genau einmal je Scan in derselben
    /// Reihenfolge; Standard: <scan>/driftless
    #[arg(long, value_name = "ORDNER")]
    slam: Vec<PathBuf>,
    /// Ordner der Karte (mit clean.bin und map.json von `driftless map`), genau einmal je Scan
    /// in derselben Reihenfolge; Standard: der Lauf-Ordner
    #[arg(long, value_name = "KARTE")]
    map: Vec<PathBuf>,
    /// Punktpaare von Hand (TOML wie punktpaare.toml, Kartenordner relativ zur Datei): ersetzen
    /// die automatische Ausrichtung der genannten Scans
    #[arg(long, value_name = "DATEI")]
    punktpaare: Option<PathBuf>,
    /// Auch unsicher ausgerichtete Scans ins Modell nehmen (gescheiterte nie)
    #[arg(long)]
    auch_unsichere: bool,
    /// Rastergröße des Modells in m (ein Punkt je Zelle)
    #[arg(long, value_name = "M", default_value_t = 0.01)]
    voxel: f64,
    /// Die Flächen der zusammengeführten Punkte nicht gemeinsam glätten
    #[arg(long)]
    ohne_glaetten: bool,
    /// LiDAR-Lage im IMU-Rahmen: x y z qx qy qz qw (wie bei `slam` und `map`, für alle Scans;
    /// Standard: Identität)
    #[arg(long, num_args = 7, value_names = ["X", "Y", "Z", "QX", "QY", "QZ", "QW"], allow_negative_numbers = true)]
    lidar_to_imu: Option<Vec<f64>>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
}

/// `given` as one folder per scan, or `default` of each scan if none is given.
fn per_scan(
    option: &str,
    given: &[PathBuf],
    scans: &[PathBuf],
    default: impl Fn(usize) -> PathBuf,
) -> Result<Vec<PathBuf>, String> {
    match given.len() {
        0 => Ok((0..scans.len()).map(default).collect()),
        n if n == scans.len() => Ok(given.to_vec()),
        n => Err(format!(
            "--{option} genau einmal je Scan angeben, in derselben Reihenfolge ({} Scans, {n} × \
             --{option})",
            scans.len()
        )),
    }
}

fn request(args: &Args) -> Result<MergeRequest, Box<dyn std::error::Error>> {
    let slam = SlamSettings {
        lidar_to_imu: extrinsic_values(args.lidar_to_imu.as_deref())?,
        lidar_topic: args.lidar_topic.clone(),
        imu_topic: args.imu_topic.clone(),
    };
    let slam_dirs = per_scan("slam", &args.slam, &args.scans, |i| {
        args.scans[i].join("driftless")
    })?;
    let map_dirs = per_scan("map", &args.map, &args.scans, |i| slam_dirs[i].clone())?;
    let members = args
        .scans
        .iter()
        .zip(slam_dirs)
        .zip(map_dirs)
        .map(|((scan, slam_dir), map_dir)| MergeMember {
            scan: scan.clone(),
            slam_dir,
            map_dir,
            slam: slam.clone(),
        })
        .collect();
    let pairs = match &args.punktpaare {
        None => Vec::new(),
        Some(file) => {
            if !file.is_file() {
                return Err(format!("Punktpaare: {} nicht gefunden", file.display()).into());
            }
            let base = file.parent().unwrap_or(Path::new(""));
            PointPairs::load(file)?.resolved(base)
        }
    };
    Ok(MergeRequest {
        members,
        out_dir: args.out.clone(),
        align: AlignConfig::default(),
        voxel: args.voxel,
        smooth: !args.ohne_glaetten,
        include_unsure: args.auch_unsichere,
        pairs,
    })
}

/// A warning for every switched-on entry of the point pairs that names no two scans of
/// `req` (a mistyped folder would otherwise be ignored silently).
fn unused_pairs(req: &MergeRequest) -> Vec<String> {
    req.pairs
        .iter()
        .enumerate()
        .filter(|(_, e)| e.enabled && pair_members(req, e).is_none())
        .map(|(k, e)| {
            format!(
                "Punktpaare Eintrag {}: {} und {} sind nicht zwei der gewählten Karten, nicht \
                 genutzt",
                k + 1,
                e.a.display(),
                e.b.display()
            )
        })
        .collect()
}

fn folder_name(p: &Path) -> String {
    p.file_name().map_or_else(
        || p.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// One line per scan: name, verdict, overlap, residual and why it is unsure.
fn lines(report: &ModelReport) -> Vec<String> {
    report
        .members
        .iter()
        .map(|m| {
            let name = folder_name(&m.scan);
            if m.placed_against.is_none() {
                return format!("{name}: Bezug");
            }
            let by_hand = if m.method == "punktpaare" {
                " (Punktpaare)"
            } else {
                ""
            };
            let mut line = format!(
                "{name}: {}{by_hand}, Überlappung {:.0} %, Restfehler {:.1} mm",
                m.verdict,
                100.0 * m.overlap,
                m.residual_median_mm
            );
            if let Some(why) = &m.reason {
                line += &format!(" – {why}");
            }
            if !m.included {
                line += if m.verdict == "unsicher" {
                    "; nicht im Modell (--auch-unsichere nimmt ihn auf)"
                } else {
                    "; nicht im Modell"
                };
            }
            line
        })
        .collect()
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let req = request(args)?;
    let console = crate::console();
    for w in unused_pairs(&req) {
        console.warn(w);
    }
    let report = run_merge(&req, &console)?;
    for line in lines(&report) {
        println!("{line}");
    }
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

    fn parse(argv: &[&str]) -> Result<MergeRequest, String> {
        let argv = ["merge"].iter().chain(argv);
        let cli = Cli::try_parse_from(argv).map_err(|e| e.to_string())?;
        request(&cli.args).map_err(|e| e.to_string())
    }

    #[test]
    fn folders_default_to_the_run_folder() {
        let r = parse(&["a", "b", "-o", "m"]).unwrap();
        assert_eq!(r.out_dir, PathBuf::from("m"));
        let dirs: Vec<_> = r
            .members
            .iter()
            .map(|m| (m.scan.clone(), m.slam_dir.clone(), m.map_dir.clone()))
            .collect();
        assert_eq!(
            dirs,
            vec![
                ("a".into(), "a/driftless".into(), "a/driftless".into()),
                ("b".into(), "b/driftless".into(), "b/driftless".into()),
            ]
        );
        assert_eq!(r.voxel, 0.01);
        assert!(r.smooth && !r.include_unsure && r.pairs.is_empty());
        assert_eq!(r.align, AlignConfig::default());

        let r = parse(&[
            "a",
            "b",
            "-o",
            "m",
            "--slam",
            "s1",
            "--slam",
            "s2",
            "--map",
            "k1",
            "--map",
            "k2",
            "--auch-unsichere",
            "--ohne-glaetten",
            "--voxel",
            "0.02",
        ])
        .unwrap();
        assert_eq!(r.members[1].slam_dir, PathBuf::from("s2"));
        assert_eq!(r.members[0].map_dir, PathBuf::from("k1"));
        assert!(r.include_unsure && !r.smooth);
        assert_eq!(r.voxel, 0.02);
        // Only --slam: the map is in the given run folder.
        let r = parse(&["a", "b", "-o", "m", "--slam", "s1", "--slam", "s2"]).unwrap();
        assert_eq!(r.members[1].map_dir, PathBuf::from("s2"));
    }

    #[test]
    fn point_pairs_relative_to_their_file() {
        let dir =
            std::env::temp_dir().join(format!("driftless-merge-paare-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("paare.toml");
        let pts = "[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]";
        let entry = |a: &str, b: &str, enabled: bool| {
            format!(
                "[[paar]]\na = \"{a}\"\nb = \"{b}\"\npoints_a = {pts}\npoints_b = {pts}\n\
                 enabled = {enabled}\n"
            )
        };
        let text = [
            entry("a/driftless", "b/driftless", true),
            entry("a/driftless", "c/driftless", true),
            entry("a/driftless", "c/driftless", false),
        ]
        .concat();
        std::fs::write(&file, text).unwrap();
        let a = dir.join("a");
        let b = dir.join("b");
        let f = file.to_str().unwrap();
        let r = parse(&[
            a.to_str().unwrap(),
            b.to_str().unwrap(),
            "-o",
            "m",
            "--punktpaare",
            f,
        ])
        .unwrap();
        assert_eq!(r.pairs.len(), 3);
        assert_eq!(r.pairs[0].a, dir.join("a").join("driftless"));
        assert_eq!(r.pairs[0].b, r.members[1].map_dir);
        // Only the second names a folder that is not a map of the merge.
        let w = unused_pairs(&r);
        assert_eq!(w.len(), 1, "{w:?}");
        assert!(w[0].contains("Eintrag 2"), "{w:?}");

        let e = parse(&["a", "b", "-o", "m", "--punktpaare", "fehlt.toml"]).unwrap_err();
        assert!(e.contains("fehlt.toml nicht gefunden"), "{e}");
        std::fs::write(&file, "[[paar]]\na = 1\n").unwrap();
        let e = parse(&["a", "b", "-o", "m", "--punktpaare", f]).unwrap_err();
        assert!(e.contains("nicht lesbar"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn folders_must_be_given_once_per_scan() {
        let e = parse(&["a", "b", "-o", "m", "--slam", "s"]).unwrap_err();
        assert!(e.contains("--slam genau einmal je Scan"), "{e}");
        let e = parse(&[
            "a", "b", "-o", "m", "--map", "1", "--map", "2", "--map", "3",
        ])
        .unwrap_err();
        assert!(e.contains("3 × --map"), "{e}");
        assert!(parse(&["-o", "m"]).is_err(), "at least one scan");
    }
}
