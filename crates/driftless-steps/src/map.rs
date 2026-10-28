//! The map of a SLAM run in three steps, each with its own files in the output folder:
//!
//! - **dense**: dense cloud from the corrected `traj.tum` → `dense.bin` (`dense.las`)
//! - **clean**: ghosts, points seen through windows and outliers removed → `removed.las`,
//!   `clean.bin` (`clean.las`)
//! - **color**: coloured from the camera images → `color.las` (`color.ply`)
//!
//! plus `map.json` (settings, statistics and runtimes of the steps, peak memory, and under
//! `stale` the files of earlier runs that no longer fit; they get a warning, nothing is ever
//! deleted).
//!
//! A run removes the `.tmp` files a killed run of its files left at its start, and
//! `map.json` before it writes its first file; `map.json` comes last again, so a folder with
//! a `map.json` holds what it describes.
//!
//! Of the clouds to look at, a run writes one: the LAS of its last step, so `color.las`, or
//! `clean.las` where the scan cannot be coloured (and `dense.las` for the dense step alone).
//! [`MapRequest::all_files`] (`driftless map --alle-dateien`) writes the files in brackets
//! above as well: the LAS of every step and `color.ply`.
//!
//! The requested step always runs; the step before it is taken from its `*.bin` in the folder
//! if that was written for the same input and settings (input id as in `submaps.bin`), and
//! rebuilt otherwise. Only a whole map ([`MapStep::All`]) that is current already
//! ([`current_map`]) is not computed again: [`run_map`] says so and returns its `map.json`
//! ([`recompute_map`] computes it all the same). The `*.bin` files hold the cloud at full precision with the frame index
//! of every point (and, in `dense.bin`, the sensor rays the cleaning needs); LAS rounds
//! coordinates and intensity and has no frame column, so it is export only.
//!
//! The trajectory is read from the SLAM folder; the results go to the output folder (the CLI
//! uses the SLAM folder for both).

use crate::error::{check_cancel, failed, StepError};
use crate::leftovers::{remove_report, remove_stale_tmp};
use crate::scan::{image_topics, read_imu, warn_if_not_static, with_lidar_frames, IMAGE_TYPE};
use crate::slam::SlamSettings;
use crate::{PROGRAM, PROGRAM_KEY};
use driftless_gpu::{CpuFallback, GpuChoice, GpuContext, GpuPref};
use driftless_io::atomic_write;
use driftless_io::calib::load_calibration;
use driftless_io::camera::Camera;
use driftless_io::progress::Progress;
use driftless_io::project::{discover_project, Project};
use driftless_map::{
    build_dense, clean_with, colorize_with, content_hash, load_stage, save_stage, stage_fits,
    write_las, write_ply, write_removed_las, CleanConfig, CleanStats, Cloud, ColorCamera,
    ColorConfig, ColorStats, DenseConfig, DenseStats, ExposureConfig, ImageSource, RaySource,
    SmoothConfig, StageError,
};
use driftless_odom::OdomConfig;
use driftless_traj::{read_tum, Pose};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

type BoxError = Box<dyn std::error::Error>;

pub const DENSE_BIN: &str = "dense.bin";
pub const CLEAN_BIN: &str = "clean.bin";
// The version of each step in its input id. The program version is part of no id (stage,
// cache, `*_fits`): a new release keeps every result that still fits. Raise a step's version
// only when that step gives another result (or other files) for the same input and
// settings; the ids of the steps after it change with it.
/// Part of the dense step's input id (see the rule above). 1: the first release.
const DENSE_VERSION: u32 = 1;
/// Part of the clean step's input id: raised whenever the cleaning gives another result
/// (or other files) for the same settings, so an older `clean.bin` / `removed.las` is
/// rebuilt instead of reused. 2: removal reasons in `removed.las`, points seen through
/// windows. 3: kept points projected onto their local plane (thinner walls).
const CLEAN_VERSION: u32 = 3;
/// Part of the color step's input id, raised like [`CLEAN_VERSION`] whenever the colouring
/// gives another result for the same settings. 2: image sharpness divided by the mean gray
/// value, not its square.
const COLOR_VERSION: u32 = 2;
pub const REPORT: &str = "map.json";

/// Which step to run (the ones before it are taken from the folder if they fit).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MapStep {
    Dense,
    Clean,
    Color,
    All,
}

/// The settings of a map folder: what `map.json` records under `settings` and the CLI
/// switches set. `Default` = the defaults of `driftless map`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapSwitches {
    /// Remove ghosts (moving things).
    pub ghosts: bool,
    /// Remove outliers.
    pub outliers: bool,
    /// Remove points measured through windows.
    pub through: bool,
    /// Flatten walls onto their local plane.
    pub smooth: bool,
    /// Devalue highlights when colouring (experimental).
    pub highlights: bool,
    /// Estimate the edge falloff of the cameras.
    pub vignetting: bool,
    /// Voxel size of the dense cloud (m).
    pub voxel: f64,
    /// Decode images downscaled: 1, 2 or 4.
    pub downscale: u32,
    /// Colour on the GPU (`--gpu auto|an|aus`). Gives an equivalent result: not part of the
    /// colour input id, not in `settings` of `map.json` (the colour step's entry records under
    /// `gpu` what was used), and ignored by [`MapSwitches::same_map`].
    #[serde(default, with = "gpu_pref")]
    pub gpu: GpuPref,
}

/// `GpuPref` as its CLI word (`auto`, `an`, `aus`).
mod gpu_pref {
    use driftless_gpu::GpuPref;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(p: &GpuPref, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(p)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<GpuPref, D::Error> {
        String::deserialize(d)?
            .parse()
            .map_err(serde::de::Error::custom)
    }
}

impl Default for MapSwitches {
    fn default() -> Self {
        MapSwitches {
            ghosts: true,
            outliers: true,
            through: true,
            smooth: true,
            highlights: false,
            vignetting: true,
            voxel: DenseConfig::default().voxel,
            downscale: ColorConfig::default().downscale,
            gpu: GpuPref::default(),
        }
    }
}

impl MapSwitches {
    /// Fails with the message of the CLI option that is out of range.
    pub fn check(&self) -> Result<(), StepError> {
        if !matches!(self.downscale, 1 | 2 | 4) {
            return Err("--downscale muss 1, 2 oder 4 sein".into());
        }
        if !(self.voxel.is_finite() && self.voxel > 0.0) {
            return Err("--voxel: Voxelgröße muss größer als 0 sein".into());
        }
        Ok(())
    }

    /// Whether `self` and `other` make the same map: all switches but `gpu` agree.
    pub fn same_map(&self, other: &MapSwitches) -> bool {
        let without_gpu = |s: &MapSwitches| MapSwitches {
            gpu: GpuPref::default(),
            ..s.clone()
        };
        without_gpu(self) == without_gpu(other)
    }

    /// The switches recorded in `map.json` (the whole file); `None` if they are missing. The
    /// GPU preference is not recorded there: `gpu` is the default.
    pub fn from_map_json(map_json: &Value) -> Option<Self> {
        let s = &map_json["settings"];
        Some(MapSwitches {
            ghosts: s["clean"]["ghosts"].as_bool()?,
            outliers: s["clean"]["outliers"].as_bool()?,
            through: s["clean"]["through"].as_bool()?,
            smooth: s["clean"]["smooth"]["enabled"].as_bool()?,
            highlights: s["color"]["highlights"].as_bool()?,
            vignetting: s["color"]["exposure"]["vignetting"].as_bool()?,
            voxel: s["dense"]["voxel"].as_f64()?,
            downscale: u32::try_from(s["color"]["downscale"].as_u64()?).ok()?,
            gpu: GpuPref::default(),
        })
    }

    fn dense_config(&self) -> DenseConfig {
        DenseConfig {
            voxel: self.voxel,
            ..DenseConfig::default()
        }
    }

    fn clean_config(&self) -> CleanConfig {
        CleanConfig {
            ghosts: self.ghosts,
            outliers: self.outliers,
            through: self.through,
            smooth: SmoothConfig {
                enabled: self.smooth,
                ..SmoothConfig::default()
            },
            ..CleanConfig::default()
        }
    }

    fn color_config(&self) -> ColorConfig {
        ColorConfig {
            downscale: self.downscale,
            highlights: self.highlights,
            exposure: ExposureConfig {
                vignetting: self.vignetting,
                ..ExposureConfig::default()
            },
            ..ColorConfig::default()
        }
    }
}

/// Step and switches of one map run. `Default` = `driftless map` without options.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapSettings {
    pub step: MapStep,
    pub switches: MapSwitches,
}

impl Default for MapSettings {
    fn default() -> Self {
        MapSettings {
            step: MapStep::All,
            switches: MapSwitches::default(),
        }
    }
}

impl MapSettings {
    /// Fails with the message of the CLI option that is out of range.
    pub fn check(&self) -> Result<(), StepError> {
        self.switches.check()
    }
}

/// What a map is made from.
#[derive(Clone, Debug, PartialEq)]
pub struct MapInput {
    /// Scan folder (with LIDAR_*.bag; for colouring also IMAGE_*.bag and the calibration).
    pub scan: PathBuf,
    /// SLAM folder: `traj.tum` is read from it.
    pub slam_dir: PathBuf,
    pub slam: SlamSettings,
    pub map: MapSettings,
}

/// What the map in `map_dir` was made from: `scan` and the trajectory of `slam_dir` with
/// `slam`, and the switches of its `map.json` (the defaults of `driftless map` without one).
/// The step is [`MapStep::All`]; the stage ids do not depend on it.
pub fn map_input_for(scan: &Path, slam_dir: &Path, map_dir: &Path, slam: SlamSettings) -> MapInput {
    let switches = fs::read_to_string(map_dir.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .and_then(|j| MapSwitches::from_map_json(&j))
        .unwrap_or_default();
    MapInput {
        scan: scan.to_path_buf(),
        slam_dir: slam_dir.to_path_buf(),
        slam,
        map: MapSettings {
            step: MapStep::All,
            switches,
        },
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MapRequest {
    pub input: MapInput,
    /// Where the results go (and where earlier ones are reused from).
    pub out_dir: PathBuf,
    /// Every step writes its cloud as LAS (`dense.las`, `clean.las`), and colouring
    /// `color.ply` as well (`driftless map --alle-dateien`); else only the last step's LAS
    /// (see the module documentation). Part of no id: the files hold the same points.
    pub all_files: bool,
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
pub(crate) struct ColorInput {
    pub(crate) images: ImageSource,
    pub(crate) cams: Vec<ColorCamera>,
    /// Image bags, topics and calibration, for the input id.
    pub(crate) id: String,
}

/// The camera images and mounts of the scan, or why the scan cannot be coloured (German).
pub(crate) fn color_input(
    scan: &Path,
    project: &Project,
    lidar_to_imu: &driftless_traj::nalgebra::Isometry3<f64>,
    progress: &Progress,
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
            progress.warn(format!(
                "Kamera {name} hat keine transform_matrix, übersprungen"
            ));
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
            "keine Kamerabilder ({IMAGE_TYPE}) in den IMAGE_*.bag"
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

/// The short text form of an input id: its content hash, 16 hex digits.
pub fn hex(id: &str) -> String {
    format!("{:016x}", content_hash(id.as_bytes()))
}

/// Input id of the dense step: [`DENSE_VERSION`], LiDAR bags, topics, odometry, trajectory
/// and dense settings.
fn dense_id(
    project: &Project,
    slam: &SlamSettings,
    odom: &OdomConfig,
    traj_bytes: &[u8],
    dense_cfg: &DenseConfig,
) -> Result<String, String> {
    Ok(format!(
        "map dense v{DENSE_VERSION}\n{}lidar_topic {:?}\nimu_topic {:?}\n{odom:?}\n\
         traj {:016x} {}\n{dense_cfg:?}\n",
        bag_list(&project.lidar_bags)?,
        slam.lidar_topic,
        slam.imu_topic,
        content_hash(traj_bytes),
        traj_bytes.len(),
    ))
}

/// Input id of the clean step: the dense input and the clean settings.
fn clean_id(dense_id: &str, clean_cfg: &CleanConfig) -> String {
    format!("{dense_id}clean v{CLEAN_VERSION} {clean_cfg:?}\n")
}

/// Input id of the color step: the clean input, the images and calibration
/// ([`ColorInput::id`]) and the colour settings.
fn color_id(clean_id: &str, images: &str, color_cfg: &ColorConfig) -> String {
    format!("{clean_id}{images}color v{COLOR_VERSION} {color_cfg:?}\n")
}

/// Peak resident memory of this process so far (bytes), if the platform reports it.
#[cfg(unix)]
pub fn peak_rss_bytes() -> Option<u64> {
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
pub fn peak_rss_bytes() -> Option<u64> {
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
pub fn peak_rss_bytes() -> Option<u64> {
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
    let q = &s.dense_quality;
    json!({
        "kept": s.kept,
        "ghosts": s.ghosts,
        "through": s.through,
        "outliers": s.outliers,
        "smooth": {
            "moved": s.smooth.moved,
            "mean_shift": s.smooth.mean_shift,
            "thickness_before": s.smooth.thickness_before,
            "thickness_after": s.smooth.thickness_after,
        },
        "dense_quality": {
            "patches": q.patches,
            "patches_multi_pass": q.patches_multi_pass,
            "local_rms_mm": q.local_rms_mm,
            "thickness95_mm": q.thickness95_mm,
            "within_pass_rms_mm": q.within_pass_rms_mm,
            "pass_offset_median_mm": q.pass_offset_median_mm,
            "pass_offset_p90_mm": q.pass_offset_p90_mm,
        },
    })
}

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
    // More than 1 % of the images.
    if s.images_gain_clamped * 100 > s.images_used {
        out.push(format!(
            "{} Bilder ({:.0} %) stoßen an die Grenze des Belichtungsabgleichs",
            s.images_gain_clamped,
            100.0 * s.images_gain_clamped as f64 / s.images_used.max(1) as f64
        ));
    }
    out
}

/// Colour statistics with the gains summarised per camera (count, smallest, largest and mean
/// gain per channel) instead of one entry per image; `cameras` names the cameras in the order
/// of the edge falloff.
fn color_json(s: &ColorStats, cameras: &[&str]) -> Value {
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
    let vignetting: serde_json::Map<String, Value> = cameras
        .iter()
        .zip(&s.vignetting.coeffs)
        .map(|(name, c)| (name.to_string(), json!(c)))
        .collect();
    let points = s.points_colored + s.points_uncolored;
    json!({
        "images_used": s.images_used,
        "images_blurry": s.images_blurry,
        "images_outside_trajectory": s.images_outside_trajectory,
        "images_bad": s.images_bad,
        "sharpness_brightness_rank_corr": s.sharpness_brightness_rank_corr,
        "views_rejected": s.views_rejected,
        "points_with_rejected": s.points_with_rejected,
        "views_rim_excluded": s.views_rim_excluded,
        "floor_highlight_share_before": s.floor_highlight_share_before,
        "floor_highlight_share_after": s.floor_highlight_share_after,
        "seam_jump_before": s.seam_jump_before,
        "seam_jump_after": s.seam_jump_after,
        "image_bags_truncated": s.image_bags_truncated,
        "points_colored": s.points_colored,
        "points_uncolored": s.points_uncolored,
        "colored_share": if points == 0 { 0.0 } else { s.points_colored as f64 / points as f64 },
        "gains": {
            "images": s.gains.len(),
            "min": s.gain_min,
            "max": s.gain_max,
            "per_camera": per_camera,
            "clamped_images": s.images_gain_clamped,
        },
        "vignetting": vignetting,
        "vignette_residual_before": s.vignette_residual_before,
        "vignette_residual_after": s.vignette_residual_after,
        "spread_before": s.spread_before,
        "spread_after": s.spread_after,
        "camera_difference_before": s.camera_difference_before,
        "camera_difference_after": s.camera_difference_after,
        "camera_difference_points": s.camera_difference_points,
        "image_offset_before": s.image_offset_before,
        "image_offset_after": s.image_offset_after,
    })
}

/// Why the CPU computes although the GPU was asked for (`--gpu auto|an`), for the console
/// and the report; `None` for `--gpu aus`.
pub(crate) fn fallback_reason(pref: GpuPref, why: &CpuFallback) -> Option<String> {
    (pref != GpuPref::Off).then(|| why.to_string())
}

/// What the colour step (or the views of the surface) ran on: `used`, and the device's `name`, `backend` and `driver`
/// (`null` on the CPU); `fallback`: why the opened GPU was not used (`--gpu auto`).
pub(crate) fn gpu_json(gpu: Option<&GpuContext>, fallback: Option<&str>) -> Value {
    match (gpu.map(GpuContext::info), fallback) {
        (Some(i), None) => json!({
            "used": true,
            "name": i.name,
            "backend": i.backend,
            "driver": i.driver,
        }),
        (_, fallback) => json!({
            "used": false,
            "name": null,
            "backend": null,
            "driver": null,
            "fallback": fallback,
        }),
    }
}

fn settings_json(
    slam: &SlamSettings,
    odom: &OdomConfig,
    d: &DenseConfig,
    c: &CleanConfig,
    k: &ColorConfig,
) -> Value {
    let t = odom.lidar_to_imu.translation.vector;
    let q = odom.lidar_to_imu.rotation.quaternion();
    json!({
        "lidar_to_imu": [t.x, t.y, t.z, q.i, q.j, q.k, q.w],
        "lidar_topic": slam.lidar_topic,
        "imu_topic": slam.imu_topic,
        "dense": {
            "voxel": d.voxel,
            "min_range": d.min_range,
            "max_range": d.max_range,
            "ray_step": d.ray_step,
        },
        "clean": {
            "ghosts": c.ghosts,
            "outliers": c.outliers,
            "through": c.through,
            "ghost_voxel": c.ghost_voxel,
            "ghost_ratio": c.ghost_ratio,
            "min_passes": c.min_passes,
            "outlier_k": c.outlier_k,
            "outlier_sigma": c.outlier_sigma,
            "window_reach": c.window_reach,
            "through_beyond": c.through_beyond,
            "smooth": {
                "enabled": c.smooth.enabled,
                "radius": c.smooth.radius,
                "max_neighbours": c.smooth.max_neighbours,
                "min_neighbours": c.smooth.min_neighbours,
                "planarity": c.smooth.planarity,
                "max_shift": c.smooth.max_shift,
            },
        },
        "color": {
            "downscale": k.downscale,
            "time_offset": k.time_offset,
            "max_dist": k.max_dist,
            "depth_tolerance": k.depth_tolerance,
            "zbuffer_scale": k.zbuffer_scale,
            "min_sharpness": k.min_sharpness,
            "max_angular_rate": k.max_angular_rate,
            "views_kept": k.views_kept,
            "views_blended": k.views_blended,
            "view_gap": k.view_gap,
            "highlights": k.highlights,
            "highlight_ratio": k.highlight_ratio,
            "highlight_abs": k.highlight_abs,
            "rim_theta": k.rim_theta,
            "exposure": {
                "smooth": k.exposure.smooth,
                "max_gain": k.exposure.max_gain,
                "samples": k.exposure.samples,
                "vignetting": k.exposure.vignetting,
            },
        },
    })
}

/// One map run: inputs, settings and the `map.json` entries of the steps.
struct MapRun<'a> {
    input: &'a MapInput,
    progress: &'a Progress,
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
    /// [`MapRequest::all_files`].
    all_files: bool,
    /// `map.json` of an earlier run is gone (before the first file is written).
    report_removed: bool,
}

fn seconds(start: Instant) -> f64 {
    (start.elapsed().as_secs_f64() * 1000.0).round() / 1000.0
}

impl MapRun<'_> {
    fn path(&self, file: &str) -> PathBuf {
        self.out.join(file)
    }

    /// Last check before a step writes its files; creates the output folder and, before the
    /// first file of the run, removes `map.json` (written again last).
    fn ready_to_write(&mut self) -> Result<(), StepError> {
        check_cancel(self.progress)?;
        fs::create_dir_all(&self.out)
            .map_err(|e| format!("{} nicht anlegbar: {e}", self.out.display()))?;
        if !self.report_removed {
            remove_report(&self.path(REPORT))?;
            self.report_removed = true;
        }
        Ok(())
    }

    /// Entry of a step that ran now and wrote `files`. The files an earlier run wrote for the
    /// same input that are still there stay listed: every step gives the same files for the
    /// same input. In the order of [`STEP_FILES`].
    fn record(
        &mut self,
        step: &'static str,
        id: &str,
        start: Instant,
        stats: Value,
        files: &[&str],
    ) {
        let before = self.previous_entry(step, id).unwrap_or(Value::Null);
        let written_before = |f: &str| {
            before["files"]
                .as_array()
                .is_some_and(|l| l.iter().any(|v| v == f))
                && self.path(f).is_file()
        };
        let of_step = STEP_FILES
            .iter()
            .find(|(s, _, _)| *s == step)
            .map_or(&[][..], |(_, f, _)| *f);
        let listed: Vec<&str> = of_step
            .iter()
            .copied()
            .filter(|f| files.contains(f) || written_before(f))
            .collect();
        self.steps.insert(
            step,
            json!({
                "input_id": hex(id),
                "reused": false,
                "seconds": seconds(start),
                "peak_rss_bytes_after": peak_rss_bytes(),
                "files": listed,
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
        let info = |m: String| self.progress.info(m);
        match load_stage(&self.path(file), id) {
            Ok((_, None)) if rays => {
                info(format!(
                    "{label}: {file} hat keine Strahlen, wird neu erzeugt"
                ));
                None
            }
            Ok(stage) => {
                info(format!(
                    "{label}: {file} wiederverwendet ({} Punkte)",
                    stage.0.len()
                ));
                Some(stage)
            }
            Err(StageError::Missing(_)) => {
                info(format!("{label}: {file} fehlt, wird erzeugt"));
                None
            }
            Err(e) => {
                info(format!("{label}: {file} {e}, wird neu erzeugt"));
                None
            }
        }
    }

    /// The dense step; its `dense.las` if it is the `last` step of the run (or with
    /// [`MapRequest::all_files`]).
    fn run_dense(&mut self, last: bool) -> Result<(Cloud, RaySource), StepError> {
        let start = Instant::now();
        let bags = &self.project.lidar_bags;
        let slam = &self.input.slam;
        let (imu, _) = read_imu(bags, slam.imu_topic.as_deref(), self.progress)?;
        let (dense, _) =
            with_lidar_frames(bags, slam.lidar_topic.as_deref(), self.progress, |frames| {
                build_dense(frames, &imu, &self.odom, &self.poses, &self.dense_cfg)
            })?;
        // After a cancel the frames stopped early and the cloud does not fit the poses.
        check_cancel(self.progress)?;
        let (cloud, rays, stats) = dense?;
        drop(imu);
        warn_if_not_static(&stats.odom, self.progress);
        self.ready_to_write()?;
        let las = last || self.all_files;
        let write = || -> Result<(), BoxError> {
            if las {
                write_las(&self.path("dense.las"), &cloud)?;
            }
            save_stage(&self.path(DENSE_BIN), &self.ids.dense, &cloud, Some(&rays))
                .map_err(|e| format!("{DENSE_BIN} nicht schreibbar: {e}"))?;
            Ok(())
        };
        write()?;
        let files: &[&str] = if las {
            &["dense.las", DENSE_BIN]
        } else {
            &[DENSE_BIN]
        };
        self.progress.info(format!(
            "Dicht: {} Punkte aus {} Frames ({} im Messbereich, {} Strahlen), {:.1} s → {}",
            stats.points,
            stats.frames,
            stats.points_in_range,
            stats.rays,
            start.elapsed().as_secs_f64(),
            files.join(", ")
        ));
        let id = self.ids.dense.clone();
        self.record("dense", &id, start, dense_json(&stats), files);
        Ok((cloud, rays))
    }

    /// The dense cloud from `dense.bin` if it fits, else computed.
    fn dense_input(&mut self) -> Result<(Cloud, RaySource), StepError> {
        let id = self.ids.dense.clone();
        if let Some((cloud, Some(rays))) = self.try_reuse("Dicht", DENSE_BIN, &id, true) {
            self.record_reused("dense", &id);
            return Ok((cloud, rays));
        }
        self.run_dense(false)
    }

    /// The clean step; its `clean.las` if it is the `last` step of the run (or with
    /// [`MapRequest::all_files`]).
    fn run_clean(&mut self, cloud: Cloud, rays: RaySource, last: bool) -> Result<Cloud, StepError> {
        let start = Instant::now();
        let (kept, removed, reasons, stats) =
            clean_with(cloud, &rays, &self.clean_cfg, self.progress)?;
        drop(rays);
        self.ready_to_write()?;
        let las = last || self.all_files;
        let write = || -> Result<(), BoxError> {
            if las {
                write_las(&self.path("clean.las"), &kept)?;
            }
            write_removed_las(&self.path("removed.las"), &removed, &reasons)?;
            Ok(())
        };
        write()?;
        drop(removed);
        save_stage(&self.path(CLEAN_BIN), &self.ids.clean, &kept, None)
            .map_err(|e| format!("{CLEAN_BIN} nicht schreibbar: {e}"))?;
        let smooth = if self.clean_cfg.smooth.enabled {
            let sm = &stats.smooth;
            format!(
                "; Wände geglättet: {} Punkte um im Mittel {:.1} mm verschoben, Wanddicke \
                 {:.1} → {:.1} mm",
                sm.moved,
                1000.0 * sm.mean_shift,
                1000.0 * sm.thickness_before,
                1000.0 * sm.thickness_after
            )
        } else {
            String::new()
        };
        let files: &[&str] = if las {
            &["clean.las", "removed.las", CLEAN_BIN]
        } else {
            &["removed.las", CLEAN_BIN]
        };
        self.progress.info(format!(
            "Bereinigt: {} Punkte behalten, entfernt {} Geister, {} Durchblick und {} \
             Ausreißer{smooth}, {:.1} s → {}",
            stats.kept,
            stats.ghosts,
            stats.through,
            stats.outliers,
            start.elapsed().as_secs_f64(),
            files.join(", ")
        ));
        let id = self.ids.clean.clone();
        self.record("clean", &id, start, clean_json(&stats), files);
        Ok(kept)
    }

    /// The cleaned cloud from `clean.bin` if it fits, else computed (from a fitting
    /// `dense.bin` or from scratch).
    fn clean_input(&mut self) -> Result<Cloud, StepError> {
        let id = self.ids.clean.clone();
        if let Some((cloud, _)) = self.try_reuse("Bereinigt", CLEAN_BIN, &id, false) {
            self.record_reused("clean", &id);
            return Ok(cloud);
        }
        let (cloud, rays) = self.dense_input()?;
        self.run_clean(cloud, rays, false)
    }

    /// Colours `cloud` on `device` (opened by [`run_map`] before any work, so `--gpu an`
    /// without a GPU fails at once).
    fn run_color(
        &mut self,
        mut cloud: Cloud,
        input: ColorInput,
        device: GpuChoice,
    ) -> Result<(), StepError> {
        let start = Instant::now();
        let pref = self.input.map.switches.gpu;
        let mut no_device = None;
        let gpu = match device {
            GpuChoice::Gpu(ctx) => {
                let i = ctx.info();
                self.progress.info(format!(
                    "Einfärben auf der Grafikkarte: {} ({})",
                    i.name, i.backend
                ));
                Some(ctx)
            }
            GpuChoice::Cpu(why) => {
                no_device = fallback_reason(pref, &why);
                if let Some(why) = &no_device {
                    self.progress.info(format!("Einfärben auf der CPU: {why}"));
                }
                None
            }
        };
        let stats = colorize_with(
            &mut cloud,
            input.images,
            &self.poses,
            &input.cams,
            &self.color_cfg,
            self.progress,
            gpu.as_ref(),
        )?;
        let fallback = stats.gpu_fallback.as_deref().or(no_device.as_deref());
        let gpu_report = gpu_json(gpu.as_ref(), fallback);
        drop(gpu);
        self.ready_to_write()?;
        for w in color_warnings(&stats) {
            self.progress.warn(w);
        }
        let write = || -> Result<(), BoxError> {
            write_las(&self.path("color.las"), &cloud)?;
            if self.all_files {
                write_ply(&self.path("color.ply"), &cloud)?;
            }
            Ok(())
        };
        write()?;
        let files: &[&str] = if self.all_files {
            &["color.las", "color.ply"]
        } else {
            &["color.las"]
        };
        let share = 100.0 * stats.points_colored as f64 / cloud.len().max(1) as f64;
        self.progress.info(format!(
            "Eingefärbt: {share:.1} % der {} Punkte; Bilder {} genutzt, {} unscharf, {} außerhalb \
             der Laufbahn, {} kaputt; Belichtung {:.2}–{:.2}, {:.1} s → {}",
            cloud.len(),
            stats.images_used,
            stats.images_blurry,
            stats.images_outside_trajectory,
            stats.images_bad,
            stats.gain_min,
            stats.gain_max,
            start.elapsed().as_secs_f64(),
            files.join(", ")
        ));
        self.progress.info(format!(
            "Farbqualität: {} Glanzansichten verworfen ({} Punkte), {} Randansichten ausgelassen; \
             Bodenglanz {:.1} → {:.1} %, Nahtsprung {:.1} → {:.1}, Randabfall {:.1} → {:.1} %",
            stats.views_rejected,
            stats.points_with_rejected,
            stats.views_rim_excluded,
            100.0 * stats.floor_highlight_share_before,
            100.0 * stats.floor_highlight_share_after,
            stats.seam_jump_before,
            stats.seam_jump_after,
            stats.vignette_residual_before,
            stats.vignette_residual_after
        ));
        let id = self.ids.color.clone().expect("colour input exists");
        let names: Vec<&str> = input.cams.iter().map(|c| c.camera.name.as_str()).collect();
        self.record("color", &id, start, color_json(&stats, &names), files);
        if let Some(e) = self.steps.get_mut("color") {
            e["gpu"] = gpu_report;
        }
        Ok(())
    }
}

/// Every output file of each step (also those written only with [`MapRequest::all_files`]),
/// and the file of the step before it that they are made from (one that is always written).
const STEP_FILES: [(&str, &[&str], &str); 3] = [
    ("dense", &["dense.las", DENSE_BIN], "traj.tum"),
    ("clean", &["clean.las", "removed.las", CLEAN_BIN], DENSE_BIN),
    ("color", &["color.las", "color.ply"], CLEAN_BIN),
];

/// Whether a map writes the file `name` (one of [`STEP_FILES`] or `map.json`).
fn owns(name: &str) -> bool {
    name == REPORT || STEP_FILES.iter().any(|(_, files, _)| files.contains(&name))
}

/// Output files in `out` that the current entry of their step in `steps` does not list (its
/// `files`): the step has none (not run and no longer fitting, or skipped), or did not write
/// them for its input (`dense.las` of an earlier `--alle-dateien` run, say). They are from an
/// earlier run. Nothing is deleted; each group gets a warning. Returns their names, in step
/// order.
fn warn_stale(
    out: &Path,
    steps: &BTreeMap<&'static str, Value>,
    progress: &Progress,
) -> Vec<String> {
    let mut stale = Vec::new();
    for (step, files, source) in STEP_FILES {
        let current = steps.get(step).filter(|e| e.get("skipped").is_none());
        let listed = |f: &str| {
            current.is_some_and(|e| {
                e["files"]
                    .as_array()
                    .is_some_and(|l| l.iter().any(|v| v == f))
            })
        };
        let found: Vec<&str> = files
            .iter()
            .copied()
            .filter(|f| out.join(f).exists() && !listed(f))
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
        progress.warn(format!(
            "{} {verb} aus einem früheren Lauf und {fits} nicht mehr zu {source}",
            found.join(", ")
        ));
        stale.extend(found.iter().map(|f| f.to_string()));
    }
    stale
}

/// Reads the trajectory of the SLAM folder: its bytes (for the input id) and its poses.
pub(crate) fn read_traj(slam_dir: &Path) -> Result<(Vec<u8>, Vec<Pose>), String> {
    let traj_path = slam_dir.join("traj.tum");
    let traj_bytes = fs::read(&traj_path).map_err(|e| {
        format!(
            "Laufbahn {} nicht lesbar: {e}; zuerst `driftless slam <scan> -o <ziel>` laufen lassen",
            traj_path.display()
        )
    })?;
    let poses =
        read_tum(&traj_path).map_err(|e| format!("Laufbahn {}: {e}", traj_path.display()))?;
    Ok((traj_bytes, poses))
}

/// Whether `dir` holds a map of `input`: its `map.json` has the same switches (the step does
/// not matter) and a dense entry for the current LiDAR bags, odometry settings and
/// trajectory, and the stages it claims as current (`dense.bin`, `clean.bin`) are those, if
/// they are there: one of another run next to the report – what a run of an older version
/// left when it was killed between its files – does not fit. Reads only their headers.
pub fn folder_fits(input: &MapInput, dir: &Path) -> bool {
    let fits = || -> Option<bool> {
        let text = fs::read_to_string(dir.join(REPORT)).ok()?;
        let report: Value = serde_json::from_str(&text).ok()?;
        if !MapSwitches::from_map_json(&report)?.same_map(&input.map.switches) {
            return Some(false);
        }
        let odom = input.slam.config().ok()?.odom;
        let project = discover_project(&input.scan).ok()?;
        let traj_bytes = fs::read(input.slam_dir.join("traj.tum")).ok()?;
        let switches = &input.map.switches;
        let dense = dense_id(
            &project,
            &input.slam,
            &odom,
            &traj_bytes,
            &switches.dense_config(),
        )
        .ok()?;
        let clean = clean_id(&dense, &switches.clean_config());
        let claims = |step: &str, id: &str| {
            report["steps"][step]["input_id"].as_str() == Some(hex(id).as_str())
        };
        // A stage that is not there is no reason: `dense.bin` may go to save space.
        let stage_agrees = |file: &str, id: &str| {
            let path = dir.join(file);
            !path.exists() || stage_fits(&path, id)
        };
        Some(
            claims("dense", &dense)
                && stage_agrees(DENSE_BIN, &dense)
                && (!claims("clean", &clean) || stage_agrees(CLEAN_BIN, &clean)),
        )
    };
    fits().unwrap_or(false)
}

/// Input id of the dense step (and so of `dense.bin`) of a map of `input`.
pub fn dense_stage_id(input: &MapInput) -> Result<String, StepError> {
    let odom = input.slam.config()?.odom;
    let project = discover_project(&input.scan).map_err(failed)?;
    let (traj_bytes, _) = read_traj(&input.slam_dir)?;
    dense_id(
        &project,
        &input.slam,
        &odom,
        &traj_bytes,
        &input.map.switches.dense_config(),
    )
    .map_err(Into::into)
}

/// Input id of the clean step (and so of `clean.bin`) of a map of `input`: what a surface
/// is built from.
pub fn clean_stage_id(input: &MapInput) -> Result<String, StepError> {
    let odom = input.slam.config()?.odom;
    let project = discover_project(&input.scan).map_err(failed)?;
    let (traj_bytes, _) = read_traj(&input.slam_dir)?;
    let switches = &input.map.switches;
    let dense = dense_id(
        &project,
        &input.slam,
        &odom,
        &traj_bytes,
        &switches.dense_config(),
    )?;
    Ok(clean_id(&dense, &switches.clean_config()))
}

/// The stages in `dir` ([`DENSE_BIN`], [`CLEAN_BIN`], in this order) that a map of `input`
/// would take over: written for the same LiDAR bags, odometry settings, trajectory and
/// switches of their step. Reads only their headers.
pub fn fitting_stages(input: &MapInput, dir: &Path) -> Vec<&'static str> {
    let ids = || -> Option<(String, String)> {
        let odom = input.slam.config().ok()?.odom;
        let project = discover_project(&input.scan).ok()?;
        let traj_bytes = fs::read(input.slam_dir.join("traj.tum")).ok()?;
        let switches = &input.map.switches;
        let dense = dense_id(
            &project,
            &input.slam,
            &odom,
            &traj_bytes,
            &switches.dense_config(),
        )
        .ok()?;
        let clean = clean_id(&dense, &switches.clean_config());
        Some((dense, clean))
    };
    let Some((dense, clean)) = ids() else {
        return Vec::new();
    };
    [(DENSE_BIN, dense), (CLEAN_BIN, clean)]
        .into_iter()
        .filter(|(file, id)| stage_fits(&dir.join(file), id))
        .map(|(file, _)| file)
        .collect()
}

/// The `map.json` in [`MapRequest::out_dir`] if a whole map ([`MapStep::All`]) of `req` is
/// there already, so that computing it again would give the same files: the same switches;
/// an entry of the current input for every step (colour: skipped for the same reason, if
/// the scan cannot be coloured); `dense.bin` and `clean.bin` of that input; every file the
/// entries list, and every file `req` writes among them (with [`MapRequest::all_files`] the
/// LAS of every step and `color.ply`). Cheap: the input ids (bag sizes, `traj.tum`, the
/// calibration), the stage headers and the files' existence.
pub fn current_map(req: &MapRequest) -> Option<Value> {
    let input = &req.input;
    if input.map.step != MapStep::All {
        return None;
    }
    let dir = &req.out_dir;
    let report: Value = serde_json::from_str(&fs::read_to_string(dir.join(REPORT)).ok()?).ok()?;
    let switches = &input.map.switches;
    if !MapSwitches::from_map_json(&report)?.same_map(switches) {
        return None;
    }
    let odom = input.slam.config().ok()?.odom;
    let project = discover_project(&input.scan).ok()?;
    let traj_bytes = fs::read(input.slam_dir.join("traj.tum")).ok()?;
    let dense = dense_id(
        &project,
        &input.slam,
        &odom,
        &traj_bytes,
        &switches.dense_config(),
    )
    .ok()?;
    let clean = clean_id(&dense, &switches.clean_config());
    // Its warnings were given when the map was made.
    let color = color_input(&input.scan, &project, &odom.lidar_to_imu, &Progress::new())
        .map(|c| color_id(&clean, &c.id, &switches.color_config()));
    let steps = &report["steps"];
    let current = |step: &str, id: &str| {
        steps[step].get("skipped").is_none()
            && steps[step]["input_id"].as_str() == Some(hex(id).as_str())
    };
    let color_current = match &color {
        Ok(id) => current("color", id),
        Err(why) => steps["color"]["skipped"].as_str() == Some(why.as_str()),
    };
    if !(current("dense", &dense)
        && current("clean", &clean)
        && color_current
        && stage_fits(&dir.join(DENSE_BIN), &dense)
        && stage_fits(&dir.join(CLEAN_BIN), &clean))
    {
        return None;
    }
    let listed: Vec<&str> = ["dense", "clean", "color"]
        .iter()
        .filter_map(|s| steps[*s]["files"].as_array())
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    let mut wanted = vec![DENSE_BIN, "removed.las", CLEAN_BIN];
    wanted.push(if color.is_ok() {
        "color.las"
    } else {
        "clean.las"
    });
    if req.all_files {
        wanted.extend(["dense.las", "clean.las"]);
        if color.is_ok() {
            wanted.push("color.ply");
        }
    }
    let complete =
        wanted.iter().all(|f| listed.contains(f)) && listed.iter().all(|f| dir.join(f).is_file());
    complete.then_some(report)
}

/// Runs the map as `driftless map` does; every console line goes to `progress` in the same
/// order. Returns the `map.json` written to [`MapRequest::out_dir`].
///
/// A whole map ([`MapStep::All`]) that is current already ([`current_map`]) is not computed
/// again: the step says so and returns the `map.json` that is there. [`recompute_map`]
/// computes it all the same.
///
/// A cancel is honoured before each step writes its files: a cancelled step writes nothing,
/// steps finished before it keep their files, `map.json` is not written.
pub fn run_map(req: &MapRequest, progress: &Progress) -> Result<Value, StepError> {
    req.input.map.check()?;
    if let Some(report) = current_map(req) {
        remove_stale_tmp(&req.out_dir, owns, progress);
        progress.info(format!(
            "Karte ist aktuell: gleiche Eingaben und Einstellungen wie in {}, nichts neu \
             gerechnet",
            req.out_dir.join(REPORT).display()
        ));
        return Ok(report);
    }
    recompute_map(req, progress)
}

/// [`run_map`] without the check whether the map is current: every requested step runs
/// (`driftless map --neu`).
pub fn recompute_map(req: &MapRequest, progress: &Progress) -> Result<Value, StepError> {
    let start = Instant::now();
    let input = &req.input;
    let switches = &input.map.switches;
    input.map.check()?;
    // The odometry exactly as `driftless slam` runs it.
    let odom = input.slam.config()?.odom;
    let project = discover_project(&input.scan).map_err(failed)?;

    // Colouring is checked first: step color must fail before any work.
    let step = input.map.step;
    let wants_color = matches!(step, MapStep::Color | MapStep::All);
    let color = if wants_color {
        color_input(&input.scan, &project, &odom.lidar_to_imu, progress)
    } else {
        Err(String::new())
    };
    if let (MapStep::Color, Err(why)) = (step, &color) {
        return Err(why.clone().into());
    }
    let device = match &color {
        Ok(_) => Some(GpuContext::open(switches.gpu).map_err(|e| e.to_string())?),
        Err(_) => None,
    };

    let (traj_bytes, poses) = read_traj(&input.slam_dir)?;
    remove_stale_tmp(&req.out_dir, owns, progress);
    let dense_cfg = switches.dense_config();
    let clean_cfg = switches.clean_config();
    let color_cfg = switches.color_config();
    let dense_id = dense_id(&project, &input.slam, &odom, &traj_bytes, &dense_cfg)?;
    let clean_id = clean_id(&dense_id, &clean_cfg);
    let color_id = color
        .as_ref()
        .ok()
        .map(|c| color_id(&clean_id, &c.id, &color_cfg));
    let previous = fs::read_to_string(req.out_dir.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or(Value::Null);

    let settings = settings_json(&input.slam, &odom, &dense_cfg, &clean_cfg, &color_cfg);
    let mut run = MapRun {
        input,
        progress,
        project,
        out: req.out_dir.clone(),
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
        all_files: req.all_files,
        report_removed: false,
    };

    match step {
        MapStep::Dense => {
            run.run_dense(true)?;
        }
        MapStep::Clean => {
            let (cloud, rays) = run.dense_input()?;
            run.run_clean(cloud, rays, true)?;
        }
        MapStep::Color => {
            let cloud = run.clean_input()?;
            run.run_color(
                cloud,
                color.expect("checked above"),
                device.expect("opened with the colour input"),
            )?;
        }
        MapStep::All => {
            let (cloud, rays) = run.run_dense(false)?;
            // Without colour the cleaned cloud is the one to look at.
            let cloud = run.run_clean(cloud, rays, color.is_err())?;
            match color {
                Ok(input) => {
                    run.run_color(cloud, input, device.expect("opened with the colour input"))?
                }
                Err(why) => {
                    progress.warn(format!("nicht eingefärbt, {why}"));
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

    let stale = warn_stale(&run.out, &run.steps, progress);

    let peak = peak_rss_bytes();
    let mut report = json!({
        "settings": settings,
        "steps": run.steps,
        "seconds_total": seconds(start),
        "stale": stale,
        "peak_rss_bytes": peak,
    });
    report[PROGRAM_KEY] = json!(PROGRAM);
    let report_path = run.path(REPORT);
    let text = serde_json::to_string_pretty(&report).map_err(failed)? + "\n";
    atomic_write(&report_path, text.as_bytes())
        .map_err(|e| format!("{} nicht schreibbar: {e}", report_path.display()))?;
    let ram = peak.map_or_else(
        || "Spitzen-RAM unbekannt".to_string(),
        |b| format!("Spitzen-RAM {:.2} GB", b as f64 / 1e9),
    );
    progress.info(format!(
        "Laufzeit {:.1} s, {ram} → {}",
        start.elapsed().as_secs_f64(),
        report_path.display()
    ));
    Ok(report)
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

    #[test]
    fn clamped_gains_warn_above_one_percent_only() {
        let clamped = |n| ColorStats {
            images_used: 200,
            images_gain_clamped: n,
            ..ColorStats::default()
        };
        assert!(color_warnings(&clamped(0)).is_empty());
        assert!(color_warnings(&clamped(2)).is_empty(), "exactly 1 %");
        assert_eq!(color_warnings(&clamped(3)).len(), 1);
    }

    #[test]
    fn warns_about_clamped_gains() {
        let s = ColorStats {
            images_used: 200,
            images_gain_clamped: 26,
            ..ColorStats::default()
        };
        let w = color_warnings(&s);
        assert_eq!(
            w,
            vec!["26 Bilder (13 %) stoßen an die Grenze des Belichtungsabgleichs"]
        );
    }
}
