//! `driftless map <scan> --slam <ziel> [--step dense|clean|color|all]`: the map of a SLAM run
//! in three steps, each with its own files in the SLAM output folder:
//!
//! - **dense**: dense cloud from the corrected `traj.tum` → `dense.las`, `dense.bin`
//! - **clean**: ghosts and outliers removed → `clean.las`, `removed.las`, `clean.bin`
//! - **color**: coloured from the camera images → `color.las`, `color.ply`
//!
//! plus `map.json` (settings, statistics and runtimes of the steps, peak memory, and under
//! `stale` the files of earlier runs that no longer fit; they get a warning, nothing is ever
//! deleted).
//!
//! The requested step always runs; the step before it is taken from its `*.bin` in the folder
//! if that was written for the same input and settings (input id as in `submaps.bin`), and
//! rebuilt otherwise. The `*.bin` files hold the cloud at full precision with the frame index
//! of every point (and, in `dense.bin`, the sensor rays the cleaning needs); LAS rounds
//! coordinates and intensity and has no frame column, so it is export only.

use crate::calib_check::image_topics;
use crate::scan_input::{parse_extrinsic, read_imu, warn_if_not_static, with_lidar_frames};
use driftless_io::calib::load_calibration;
use driftless_io::camera::Camera;
use driftless_io::project::{discover_project, Project};
use driftless_map::{
    build_dense, clean, colorize, content_hash, load_stage, save_stage, write_las, write_ply,
    CleanConfig, CleanStats, Cloud, ColorCamera, ColorConfig, ColorStats, DenseConfig, DenseStats,
    ImageSource, RaySource, StageError,
};
use driftless_odom::OdomConfig;
use driftless_slam::SlamConfig;
use driftless_traj::{read_tum, Pose};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

type BoxError = Box<dyn std::error::Error>;

const DENSE_BIN: &str = "dense.bin";
const CLEAN_BIN: &str = "clean.bin";
const REPORT: &str = "map.json";

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

/// Names and sizes of `bags`, one per line.
fn bag_list(bags: &[PathBuf]) -> Result<String, String> {
    let mut s = String::new();
    for bag in bags {
        let name = bag
            .file_name()
            .map(|n| n.to_string_lossy())
            .unwrap_or_default();
        let len = fs::metadata(bag)
            .map_err(|e| format!("{} nicht lesbar: {e}", bag.display()))?
            .len();
        s += &format!("{name} {len}\n");
    }
    Ok(s)
}

/// Everything colouring needs besides the cloud and the trajectory.
struct ColorInput {
    images: ImageSource,
    cams: Vec<ColorCamera>,
    /// Image bags, topics and calibration, for the input id.
    id: String,
}

/// The camera images and mounts of the scan, or why the scan cannot be coloured (German).
fn color_input(
    scan: &Path,
    project: &Project,
    lidar_to_imu: &driftless_traj::nalgebra::Isometry3<f64>,
) -> Result<ColorInput, String> {
    if project.image_bags.is_empty() {
        return Err(format!(
            "keine Kamerabilder: {} enthält keine IMAGE_*.bag",
            scan.display()
        ));
    }
    let calib_path = project.calibration.as_ref().ok_or_else(|| {
        format!(
            "keine Kalibrierung: {} fehlt",
            scan.join("calibration").join("calib.json").display()
        )
    })?;
    let calib_bytes = fs::read(calib_path)
        .map_err(|e| format!("Kalibrierung {} nicht lesbar: {e}", calib_path.display()))?;
    let calib = load_calibration(calib_path)
        .map_err(|e| format!("Kalibrierung {}: {e}", calib_path.display()))?;
    let mut cams = Vec::new();
    for (name, c) in &calib.cameras {
        let Some(lidar_to_cam) = c.lidar_to_cam else {
            eprintln!("Warnung: Kamera {name} hat keine transform_matrix, übersprungen");
            continue;
        };
        let camera = Camera::from_calib(name, c).map_err(|e| format!("Kamera {name}: {e}"))?;
        cams.push(ColorCamera::new(camera, &lidar_to_cam, lidar_to_imu));
    }
    if cams.is_empty() {
        return Err("keine Kamera mit transform_matrix in calib.json".into());
    }
    let topics = image_topics(&project.image_bags)?;
    if topics.is_empty() {
        return Err(format!(
            "keine Kamerabilder ({}) in den IMAGE_*.bag",
            crate::calib_check::IMAGE_TYPE
        ));
    }
    let id = format!(
        "images\n{}topics {topics:?}\ncalib {:016x} {}\n",
        bag_list(&project.image_bags)?,
        content_hash(&calib_bytes),
        calib_bytes.len()
    );
    Ok(ColorInput {
        images: ImageSource {
            bags: project.image_bags.clone(),
            topics,
        },
        cams,
        id,
    })
}

/// Input ids of the steps: each one includes the one before.
struct Ids {
    dense: String,
    clean: String,
    /// `None` if the scan cannot be coloured.
    color: Option<String>,
}

fn hex(id: &str) -> String {
    format!("{:016x}", content_hash(id.as_bytes()))
}

/// Peak resident memory of this process so far (bytes), if the platform reports it.
#[cfg(unix)]
fn peak_rss_bytes() -> Option<u64> {
    let mut usage = std::mem::MaybeUninit::<libc::rusage>::zeroed();
    // SAFETY: getrusage fills the struct for RUSAGE_SELF; an all-zero rusage is valid.
    let usage = unsafe {
        if libc::getrusage(libc::RUSAGE_SELF, usage.as_mut_ptr()) != 0 {
            return None;
        }
        usage.assume_init()
    };
    let max = u64::try_from(usage.ru_maxrss).ok()?;
    // macOS reports bytes, Linux and the BSDs kibibytes.
    Some(if cfg!(target_os = "macos") {
        max
    } else {
        max * 1024
    })
}

#[cfg(windows)]
fn peak_rss_bytes() -> Option<u64> {
    use windows_sys::Win32::System::ProcessStatus::{
        GetProcessMemoryInfo, PROCESS_MEMORY_COUNTERS,
    };
    use windows_sys::Win32::System::Threading::GetCurrentProcess;
    // SAFETY: an all-zero PROCESS_MEMORY_COUNTERS is valid; the call writes at most `cb` bytes
    // into it for the pseudo handle of this process.
    unsafe {
        let mut c: PROCESS_MEMORY_COUNTERS = std::mem::zeroed();
        c.cb = std::mem::size_of::<PROCESS_MEMORY_COUNTERS>() as u32;
        (GetProcessMemoryInfo(GetCurrentProcess(), &mut c, c.cb) != 0)
            .then_some(c.PeakWorkingSetSize as u64)
    }
}

#[cfg(not(any(unix, windows)))]
fn peak_rss_bytes() -> Option<u64> {
    None
}

fn dense_json(s: &DenseStats) -> Value {
    json!({
        "frames": s.frames,
        "points_in_range": s.points_in_range,
        "points": s.points,
        "rays": s.rays,
        "odom": {
            "frames_in": s.odom.frames_in,
            "frames_used": s.odom.frames_used,
            "frames_skipped": s.odom.frames_skipped,
            "icp_failures": s.odom.icp_failures,
            "static_start": s.odom.static_start,
        },
    })
}

fn clean_json(s: &CleanStats) -> Value {
    json!({"kept": s.kept, "ghosts": s.ghosts, "outliers": s.outliers})
}

/// Colour statistics with the gains summarised per camera (count, smallest, largest and mean
/// gain per channel) instead of one entry per image.
/// Warnings about a colour run that the user should see (German, without the "Warnung: ").
fn color_warnings(s: &ColorStats) -> Vec<String> {
    let mut out = Vec::new();
    if s.image_bags_truncated > 0 {
        out.push(format!(
            "{} Bilddatei(en) sind abgeschnitten, die Bilder bis dahin wurden genutzt",
            s.image_bags_truncated
        ));
    }
    let cameras: BTreeSet<&str> = s.gains.iter().map(|g| g.camera.as_str()).collect();
    if cameras.len() > 1 && s.camera_difference_points == 0 {
        out.push(
            "die Kameras sehen keine gemeinsamen Punkte, Belichtung zwischen ihnen nicht \
             abgeglichen"
                .into(),
        );
    }
    out
}

fn color_json(s: &ColorStats) -> Value {
    struct Sum {
        n: usize,
        min: [f32; 3],
        max: [f32; 3],
        sum: [f64; 3],
    }
    let mut per: BTreeMap<&str, Sum> = BTreeMap::new();
    for g in &s.gains {
        let e = per.entry(&g.camera).or_insert(Sum {
            n: 0,
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            sum: [0.0; 3],
        });
        e.n += 1;
        for c in 0..3 {
            e.min[c] = e.min[c].min(g.gain[c]);
            e.max[c] = e.max[c].max(g.gain[c]);
            e.sum[c] += f64::from(g.gain[c]);
        }
    }
    let per_camera: serde_json::Map<String, Value> = per
        .into_iter()
        .map(|(name, e)| {
            let mean = e.sum.map(|v| v / e.n as f64);
            (
                name.to_string(),
                json!({"images": e.n, "min": e.min, "max": e.max, "mean": mean}),
            )
        })
        .collect();
    let points = s.points_colored + s.points_uncolored;
    json!({
        "images_used": s.images_used,
        "images_blurry": s.images_blurry,
        "images_outside_trajectory": s.images_outside_trajectory,
        "images_bad": s.images_bad,
        "image_bags_truncated": s.image_bags_truncated,
        "points_colored": s.points_colored,
        "points_uncolored": s.points_uncolored,
        "colored_share": if points == 0 { 0.0 } else { s.points_colored as f64 / points as f64 },
        "gains": {
            "images": s.gains.len(),
            "min": s.gain_min,
            "max": s.gain_max,
            "per_camera": per_camera,
        },
        "spread_before": s.spread_before,
        "spread_after": s.spread_after,
        "camera_difference_before": s.camera_difference_before,
        "camera_difference_after": s.camera_difference_after,
        "camera_difference_points": s.camera_difference_points,
        "image_offset_before": s.image_offset_before,
        "image_offset_after": s.image_offset_after,
    })
}

fn settings_json(
    args: &Args,
    odom: &OdomConfig,
    d: &DenseConfig,
    c: &CleanConfig,
    k: &ColorConfig,
) -> Value {
    let t = odom.lidar_to_imu.translation.vector;
    let q = odom.lidar_to_imu.rotation.quaternion();
    json!({
        "lidar_to_imu": [t.x, t.y, t.z, q.i, q.j, q.k, q.w],
        "lidar_topic": args.lidar_topic,
        "imu_topic": args.imu_topic,
        "dense": {
            "voxel": d.voxel,
            "min_range": d.min_range,
            "max_range": d.max_range,
            "ray_step": d.ray_step,
        },
        "clean": {
            "ghosts": c.ghosts,
            "outliers": c.outliers,
            "ghost_voxel": c.ghost_voxel,
            "ghost_ratio": c.ghost_ratio,
            "min_passes": c.min_passes,
            "outlier_k": c.outlier_k,
            "outlier_sigma": c.outlier_sigma,
        },
        "color": {
            "downscale": k.downscale,
            "time_offset": k.time_offset,
            "max_dist": k.max_dist,
            "depth_tolerance": k.depth_tolerance,
            "zbuffer_scale": k.zbuffer_scale,
            "min_sharpness": k.min_sharpness,
            "max_angular_rate": k.max_angular_rate,
            "views_per_point": k.views_per_point,
            "exposure": {
                "smooth": k.exposure.smooth,
                "max_gain": k.exposure.max_gain,
                "samples": k.exposure.samples,
            },
        },
    })
}

/// One map run: inputs, settings and the `map.json` entries of the steps.
struct MapRun<'a> {
    args: &'a Args,
    project: Project,
    out: PathBuf,
    poses: Vec<Pose>,
    odom: OdomConfig,
    dense_cfg: DenseConfig,
    clean_cfg: CleanConfig,
    color_cfg: ColorConfig,
    ids: Ids,
    /// `map.json` of an earlier run (`Null` if there is none).
    previous: Value,
    steps: BTreeMap<&'static str, Value>,
}

fn seconds(start: Instant) -> f64 {
    (start.elapsed().as_secs_f64() * 1000.0).round() / 1000.0
}

impl MapRun<'_> {
    fn path(&self, file: &str) -> PathBuf {
        self.out.join(file)
    }

    /// Entry of a step that ran now.
    fn record(
        &mut self,
        step: &'static str,
        id: &str,
        start: Instant,
        stats: Value,
        files: &[&str],
    ) {
        self.steps.insert(
            step,
            json!({
                "input_id": hex(id),
                "reused": false,
                "seconds": seconds(start),
                "peak_rss_bytes_after": peak_rss_bytes(),
                "files": files,
                "stats": stats,
            }),
        );
    }

    /// The earlier entry of `step` if it was written for input `id`.
    fn previous_entry(&self, step: &str, id: &str) -> Option<Value> {
        let e = &self.previous["steps"][step];
        (e["input_id"].as_str() == Some(hex(id).as_str())).then(|| e.clone())
    }

    /// Entry of a step whose result was taken from the folder.
    fn record_reused(&mut self, step: &'static str, id: &str) {
        let mut e = self
            .previous_entry(step, id)
            .unwrap_or_else(|| json!({"input_id": hex(id)}));
        e["reused"] = json!(true);
        self.steps.insert(step, e);
    }

    /// Loads the stage `file` for `id` (with rays if `rays`); `None` (with a message why) if
    /// it does not fit.
    fn try_reuse(
        &self,
        label: &str,
        file: &str,
        id: &str,
        rays: bool,
    ) -> Option<(Cloud, Option<RaySource>)> {
        match load_stage(&self.path(file), id) {
            Ok((_, None)) if rays => {
                println!("{label}: {file} hat keine Strahlen, wird neu erzeugt");
                None
            }
            Ok(stage) => {
                println!("{label}: {file} wiederverwendet ({} Punkte)", stage.0.len());
                Some(stage)
            }
            Err(StageError::Missing(_)) => {
                println!("{label}: {file} fehlt, wird erzeugt");
                None
            }
            Err(e) => {
                println!("{label}: {file} {e}, wird neu erzeugt");
                None
            }
        }
    }

    fn run_dense(&mut self) -> Result<(Cloud, RaySource), BoxError> {
        let start = Instant::now();
        let bags = &self.project.lidar_bags;
        let (imu, _) = read_imu(bags, self.args.imu_topic.as_deref())?;
        let (dense, _) = with_lidar_frames(bags, self.args.lidar_topic.as_deref(), |frames| {
            build_dense(frames, &imu, &self.odom, &self.poses, &self.dense_cfg)
        })?;
        let (cloud, rays, stats) = dense?;
        drop(imu);
        warn_if_not_static(&stats.odom);
        write_las(&self.path("dense.las"), &cloud)?;
        let id = self.ids.dense.clone();
        save_stage(&self.path(DENSE_BIN), &id, &cloud, Some(&rays))
            .map_err(|e| format!("{DENSE_BIN} nicht schreibbar: {e}"))?;
        println!(
            "Dicht: {} Punkte aus {} Frames ({} im Messbereich, {} Strahlen), {:.1} s → dense.las",
            stats.points,
            stats.frames,
            stats.points_in_range,
            stats.rays,
            start.elapsed().as_secs_f64()
        );
        self.record(
            "dense",
            &id,
            start,
            dense_json(&stats),
            &["dense.las", DENSE_BIN],
        );
        Ok((cloud, rays))
    }

    /// The dense cloud from `dense.bin` if it fits, else computed.
    fn dense_input(&mut self) -> Result<(Cloud, RaySource), BoxError> {
        let id = self.ids.dense.clone();
        if let Some((cloud, Some(rays))) = self.try_reuse("Dicht", DENSE_BIN, &id, true) {
            self.record_reused("dense", &id);
            return Ok((cloud, rays));
        }
        self.run_dense()
    }

    fn run_clean(&mut self, cloud: Cloud, rays: RaySource) -> Result<Cloud, BoxError> {
        let start = Instant::now();
        let (kept, removed, stats) = clean(cloud, &rays, &self.clean_cfg)?;
        drop(rays);
        write_las(&self.path("clean.las"), &kept)?;
        write_las(&self.path("removed.las"), &removed)?;
        drop(removed);
        let id = self.ids.clean.clone();
        save_stage(&self.path(CLEAN_BIN), &id, &kept, None)
            .map_err(|e| format!("{CLEAN_BIN} nicht schreibbar: {e}"))?;
        println!(
            "Bereinigt: {} Punkte behalten, entfernt {} Geister und {} Ausreißer, {:.1} s → \
             clean.las, removed.las",
            stats.kept,
            stats.ghosts,
            stats.outliers,
            start.elapsed().as_secs_f64()
        );
        self.record(
            "clean",
            &id,
            start,
            clean_json(&stats),
            &["clean.las", "removed.las", CLEAN_BIN],
        );
        Ok(kept)
    }

    /// The cleaned cloud from `clean.bin` if it fits, else computed (from a fitting
    /// `dense.bin` or from scratch).
    fn clean_input(&mut self) -> Result<Cloud, BoxError> {
        let id = self.ids.clean.clone();
        if let Some((cloud, _)) = self.try_reuse("Bereinigt", CLEAN_BIN, &id, false) {
            self.record_reused("clean", &id);
            return Ok(cloud);
        }
        let (cloud, rays) = self.dense_input()?;
        self.run_clean(cloud, rays)
    }

    fn run_color(&mut self, mut cloud: Cloud, input: ColorInput) -> Result<(), BoxError> {
        let start = Instant::now();
        let stats = colorize(
            &mut cloud,
            input.images,
            &self.poses,
            &input.cams,
            &self.color_cfg,
        )?;
        for w in color_warnings(&stats) {
            eprintln!("Warnung: {w}");
        }
        write_las(&self.path("color.las"), &cloud)?;
        write_ply(&self.path("color.ply"), &cloud)?;
        let share = 100.0 * stats.points_colored as f64 / cloud.len().max(1) as f64;
        println!(
            "Eingefärbt: {share:.1} % der {} Punkte; Bilder {} genutzt, {} unscharf, {} außerhalb \
             der Laufbahn, {} kaputt; Belichtung {:.2}–{:.2}, {:.1} s → color.las, color.ply",
            cloud.len(),
            stats.images_used,
            stats.images_blurry,
            stats.images_outside_trajectory,
            stats.images_bad,
            stats.gain_min,
            stats.gain_max,
            start.elapsed().as_secs_f64()
        );
        let id = self.ids.color.clone().expect("colour input exists");
        self.record(
            "color",
            &id,
            start,
            color_json(&stats),
            &["color.las", "color.ply"],
        );
        Ok(())
    }
}

/// Output files of each step, and the file of the step before it that they are made from.
const STEP_FILES: [(&str, &[&str], &str); 3] = [
    ("dense", &["dense.las", DENSE_BIN], "traj.tum"),
    (
        "clean",
        &["clean.las", "removed.las", CLEAN_BIN],
        "dense.las",
    ),
    ("color", &["color.las", "color.ply"], "clean.las"),
];

/// Output files in `out` of steps that have no current entry in `steps` (not run and no
/// longer fitting, or skipped): they are from an earlier run. Nothing is deleted; each group
/// gets a warning. Returns their names, in step order.
fn warn_stale(out: &Path, steps: &BTreeMap<&'static str, Value>) -> Vec<String> {
    let mut stale = Vec::new();
    for (step, files, source) in STEP_FILES {
        let current = steps.get(step).is_some_and(|e| e.get("skipped").is_none());
        if current {
            continue;
        }
        let found: Vec<&str> = files
            .iter()
            .copied()
            .filter(|f| out.join(f).exists())
            .collect();
        if found.is_empty() {
            continue;
        }
        let verb = if found.len() == 1 {
            "stammt"
        } else {
            "stammen"
        };
        let fits = if found.len() == 1 { "passt" } else { "passen" };
        eprintln!(
            "Warnung: {} {verb} aus einem früheren Lauf und {fits} nicht mehr zu {source}",
            found.join(", ")
        );
        stale.extend(found.iter().map(|f| f.to_string()));
    }
    stale
}

pub fn run(args: &Args) -> Result<(), BoxError> {
    let start = Instant::now();
    if !matches!(args.downscale, 1 | 2 | 4) {
        return Err("--downscale muss 1, 2 oder 4 sein".into());
    }
    if !(args.voxel.is_finite() && args.voxel > 0.0) {
        return Err("--voxel: Voxelgröße muss größer als 0 sein".into());
    }
    // The odometry exactly as `driftless slam` runs it.
    let odom = SlamConfig {
        odom: OdomConfig {
            lidar_to_imu: parse_extrinsic(args.lidar_to_imu.as_deref())?,
            ..OdomConfig::default()
        },
        ..SlamConfig::default()
    }
    .odom;
    let project = discover_project(&args.scan)?;

    // Colouring is checked first: `--step color` must fail before any work.
    let wants_color = matches!(args.step, Step::Color | Step::All);
    let color = if wants_color {
        color_input(&args.scan, &project, &odom.lidar_to_imu)
    } else {
        Err(String::new())
    };
    if let (Step::Color, Err(why)) = (args.step, &color) {
        return Err(why.clone().into());
    }

    let traj_path = args.slam.join("traj.tum");
    let traj_bytes = fs::read(&traj_path).map_err(|e| {
        format!(
            "Laufbahn {} nicht lesbar: {e}; zuerst `driftless slam <scan> -o <ziel>` laufen lassen",
            traj_path.display()
        )
    })?;
    let poses =
        read_tum(&traj_path).map_err(|e| format!("Laufbahn {}: {e}", traj_path.display()))?;

    let dense_cfg = DenseConfig {
        voxel: args.voxel,
        ..DenseConfig::default()
    };
    let clean_cfg = CleanConfig {
        ghosts: !args.no_ghosts,
        outliers: !args.no_outliers,
        ..CleanConfig::default()
    };
    let color_cfg = ColorConfig {
        downscale: args.downscale,
        ..ColorConfig::default()
    };
    let dense_id = format!(
        "driftless {} map dense\n{}lidar_topic {:?}\nimu_topic {:?}\n{odom:?}\ntraj {:016x} {}\n\
         {dense_cfg:?}\n",
        env!("CARGO_PKG_VERSION"),
        bag_list(&project.lidar_bags)?,
        args.lidar_topic,
        args.imu_topic,
        content_hash(&traj_bytes),
        traj_bytes.len(),
    );
    let clean_id = format!("{dense_id}clean {clean_cfg:?}\n");
    let color_id = color
        .as_ref()
        .ok()
        .map(|c| format!("{clean_id}{}color {color_cfg:?}\n", c.id));
    let previous = fs::read_to_string(args.slam.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or(Value::Null);

    let settings = settings_json(args, &odom, &dense_cfg, &clean_cfg, &color_cfg);
    let mut run = MapRun {
        args,
        project,
        out: args.slam.clone(),
        poses,
        odom,
        dense_cfg,
        clean_cfg,
        color_cfg,
        ids: Ids {
            dense: dense_id,
            clean: clean_id,
            color: color_id,
        },
        previous,
        steps: BTreeMap::new(),
    };

    match args.step {
        Step::Dense => {
            run.run_dense()?;
        }
        Step::Clean => {
            let (cloud, rays) = run.dense_input()?;
            run.run_clean(cloud, rays)?;
        }
        Step::Color => {
            let cloud = run.clean_input()?;
            run.run_color(cloud, color.expect("checked above"))?;
        }
        Step::All => {
            let (cloud, rays) = run.run_dense()?;
            let cloud = run.run_clean(cloud, rays)?;
            match color {
                Ok(input) => run.run_color(cloud, input)?,
                Err(why) => {
                    eprintln!("Warnung: nicht eingefärbt, {why}");
                    run.steps.insert("color", json!({"skipped": why}));
                }
            }
        }
    }

    // Steps that did not run now stay in map.json while their input still fits.
    let current = [
        ("dense", Some(run.ids.dense.clone())),
        ("clean", Some(run.ids.clean.clone())),
        ("color", run.ids.color.clone()),
    ];
    for (step, id) in current {
        if run.steps.contains_key(step) {
            continue;
        }
        if let Some(mut e) = id.and_then(|id| run.previous_entry(step, &id)) {
            e["reused"] = json!(true);
            run.steps.insert(step, e);
        }
    }

    let stale = warn_stale(&run.out, &run.steps);

    let peak = peak_rss_bytes();
    let report = json!({
        "settings": settings,
        "steps": run.steps,
        "seconds_total": seconds(start),
        "stale": stale,
        "peak_rss_bytes": peak,
    });
    let report_path = run.path(REPORT);
    fs::write(&report_path, serde_json::to_string_pretty(&report)? + "\n")
        .map_err(|e| format!("{} nicht schreibbar: {e}", report_path.display()))?;
    let ram = peak.map_or_else(
        || "Spitzen-RAM unbekannt".to_string(),
        |b| format!("Spitzen-RAM {:.2} GB", b as f64 / 1e9),
    );
    println!(
        "Laufzeit {:.1} s, {ram} → {}",
        start.elapsed().as_secs_f64(),
        report_path.display()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_map::ImageGain;

    fn gain(camera: &str) -> ImageGain {
        ImageGain {
            camera: camera.into(),
            stamp: 0.0,
            gain: [1.0; 3],
        }
    }

    #[test]
    fn warns_when_cameras_share_no_points() {
        let mut s = ColorStats {
            gains: vec![gain("left"), gain("right")],
            ..ColorStats::default()
        };
        let w = color_warnings(&s);
        assert_eq!(w.len(), 1);
        assert!(w[0].contains("keine gemeinsamen Punkte"), "{w:?}");
        // Shared points, or only one camera in use: no warning.
        s.camera_difference_points = 500;
        assert!(color_warnings(&s).is_empty());
        s.camera_difference_points = 0;
        s.gains = vec![gain("left")];
        assert!(color_warnings(&s).is_empty());
    }

    #[test]
    fn warns_about_truncated_image_bags() {
        let s = ColorStats {
            image_bags_truncated: 1,
            ..ColorStats::default()
        };
        let w = color_warnings(&s);
        assert_eq!(w.len(), 1);
        assert!(w[0].contains("abgeschnitten"), "{w:?}");
    }
}
