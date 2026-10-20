//! `driftless slam <ordner> -o <out_dir> [--reuse]`: odometry, submaps, loop closure. Writes
//! `traj.tum` (corrected), `traj_odom.tum` (odometry before loop closure), `map.las` (submaps
//! at the corrected anchors, 10 cm grid), `slam.json` (submaps, loop checks, graph) and
//! `submaps.bin` (the submaps for `--reuse`). The manual loops of `corrections.toml` in the
//! output folder are always applied.
//!
//! `--reuse` skips the odometry: it loads `submaps.bin`, solves the pose graph again and
//! keeps the previous `traj.tum` as `traj.vorher.tum`.

use crate::scan_input::{
    parse_extrinsic, read_imu, require_frames, warn_if_not_static, with_lidar_frames, write_las,
};
use driftless_io::project::discover_project;
use driftless_odom::OdomConfig;
use driftless_slam::{
    build_submaps, load_cache, save_cache, solve, Corrections, LoopVerdict, SlamConfig, SlamReport,
    SlamResult, POSITION_ONLY,
};
use driftless_traj::write_tum;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const CACHE_FILE: &str = "submaps.bin";
pub const CORRECTIONS_FILE: &str = "corrections.toml";

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

/// What the cache depends on: names and sizes of the LiDAR bags, the chosen topics and the
/// settings.
fn input_id(bags: &[PathBuf], args: &Args, cfg: &SlamConfig) -> Result<String, String> {
    let mut id = String::new();
    for bag in bags {
        let name = bag
            .file_name()
            .map(|n| n.to_string_lossy())
            .unwrap_or_default();
        let len = fs::metadata(bag)
            .map_err(|e| format!("{} nicht lesbar: {e}", bag.display()))?
            .len();
        id += &format!("{name} {len}\n");
    }
    id += &format!(
        "lidar_topic {:?}\nimu_topic {:?}\n{cfg:?}\n",
        args.lidar_topic, args.imu_topic
    );
    Ok(id)
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
    let input = input_id(&project.lidar_bags, args, &cfg)?;
    let cache = args.out.join(CACHE_FILE);
    let corrections = Corrections::load(&args.out.join(CORRECTIONS_FILE))?;
    let manual = corrections.manual_loops();
    // A bad correction fails only in `solve`; the cache is written by then.
    let solve_hint = |e: driftless_slam::ManualLoopError| {
        format!(
            "{e}\nKorrektur mit `driftless correct {out} list` suchen, mit `driftless correct {out} \
             disable <nr>` abschalten und `driftless slam` mit denselben Angaben und `--reuse` \
             erneut starten (der Zwischenspeicher ist geschrieben)",
            out = args.out.display(),
        )
    };

    let result = if args.reuse {
        let (submaps, odom_anchors) = load_cache(&cache, &input)?;
        let result = solve(submaps, odom_anchors, &manual, &cfg).map_err(solve_hint)?;
        let traj = args.out.join("traj.tum");
        let previous = args.out.join("traj.vorher.tum");
        if traj.exists() {
            fs::rename(&traj, &previous).map_err(|e| {
                format!(
                    "{} nicht in {} umbenennbar: {e}",
                    traj.display(),
                    previous.display()
                )
            })?;
        }
        println!(
            "Zwischenspeicher {CACHE_FILE}: {} Teilkarten, {} manuelle Schleifen",
            result.submaps.len(),
            manual.len()
        );
        result
    } else {
        fs::create_dir_all(&args.out)?;
        let (imu, bad) = read_imu(&project.lidar_bags, args.imu_topic.as_deref())?;
        let ((submaps, stats), read) =
            with_lidar_frames(&project.lidar_bags, args.lidar_topic.as_deref(), |frames| {
                build_submaps(frames, &imu, &cfg)
            })?;
        require_frames(&stats)?;
        let odom_anchors: Vec<_> = submaps.iter().map(|m| m.anchor).collect();
        save_cache(&cache, &submaps, &odom_anchors, &input)
            .map_err(|e| format!("Zwischenspeicher {} nicht schreibbar: {e}", cache.display()))?;
        let result = solve(submaps, odom_anchors, &manual, &cfg).map_err(solve_hint)?;
        warn_if_not_static(&stats);
        println!(
            "{} Posen aus {} LiDAR-Frames ({} übersprungen), {} IMU-Messungen, \
             {} nicht lesbare Nachrichten",
            stats.frames_used,
            stats.frames_in,
            stats.frames_skipped,
            imu.len(),
            bad + read.bad
        );
        result
    };

    let (report, points) = write_outputs(&args.out, &result)?;
    print_summary(&result, &report, points);
    println!(
        "Laufzeit {:.1} s → {}",
        start.elapsed().as_secs_f64(),
        args.out.display()
    );
    Ok(())
}

/// Writes everything but the cache; returns the report and the number of map points.
fn write_outputs(
    out: &Path,
    result: &SlamResult,
) -> Result<(SlamReport, usize), Box<dyn std::error::Error>> {
    let report = SlamReport::new(result);
    write_tum(&out.join("traj.tum"), &result.poses())?;
    write_tum(&out.join("traj_odom.tum"), &result.odom_poses())?;
    let map = result.map_points();
    write_las(&out.join("map.las"), &map)?;
    fs::write(
        out.join("slam.json"),
        serde_json::to_string_pretty(&report)? + "\n",
    )?;
    Ok((report, map.len()))
}

fn print_summary(result: &SlamResult, report: &SlamReport, points: usize) {
    for (l, r) in result.loops.iter().zip(&report.loops) {
        if let LoopVerdict::PositionOnly(why) = l.verdict {
            let (a, b) = (&report.submaps[l.a], &report.submaps[l.b]);
            eprintln!(
                "Warnung: manuelle Schleife Teilkarte {} ({:.1}–{:.1} s) ↔ {} ({:.1}–{:.1} s): \
                 {POSITION_ONLY} ({why}; Anteil passender Punkte {:.2})",
                l.a, a.t_start, a.t_end, l.b, b.t_start, b.t_end, r.inlier_ratio
            );
        }
    }
    let accepted = result
        .loops
        .iter()
        .filter(|l| !matches!(l.verdict, LoopVerdict::Rejected(_)))
        .count();
    let rejected = result.loops.len() - accepted;
    let manual = result.loops.iter().filter(|l| l.manual).count();
    let loops = if accepted == 0 {
        format!("keine Schleife gefunden ({rejected} Kandidaten verworfen)")
    } else if manual == 0 {
        format!("Schleifen: {accepted} akzeptiert, {rejected} verworfen")
    } else {
        format!("Schleifen: {accepted} akzeptiert ({manual} manuell), {rejected} verworfen")
    };
    println!(
        "{} Teilkarten, {} Kartenpunkte, {loops}",
        result.submaps.len(),
        points
    );
    println!(
        "Start↔Ende {:.3} m → {:.3} m, Länge {:.1} m → {:.1} m",
        report.start_end_gap.before,
        report.start_end_gap.after,
        report.path_length.before,
        report.path_length.after
    );
}
