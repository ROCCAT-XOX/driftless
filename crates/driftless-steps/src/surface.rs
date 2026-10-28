//! The photo-like surface of a map: `oberflaeche.glb` (its texture pages inside) and
//! `oberflaeche.json`, with [`SurfaceRequest::obj`] (`driftless surface --obj`) also
//! `oberflaeche.obj` (+ `.mtl` and the texture pages as JPEG; else „Exportieren …“ makes an
//! OBJ from the glb), built from the `clean.bin` of the map folder (points with the frame
//! they were measured in) and the camera images of the scan.
//!
//! The input id is the clean step's id, the camera images and calibration, the surface
//! settings and [`SURFACE_VERSION`]. A `clean.bin` of another input (or none) is refused:
//! the surface would not fit the map. A scan without images or calibration gets a gray
//! surface and a warning. A cancel before the files are written leaves the folder untouched;
//! the JSON is written last, so a folder with a report holds a complete surface.
//!
//! The surface of a merged model ([`run_model_surface`]) is made the same way from the
//! `clean.bin` of every scan in the model, moved by its `T_i`, with the LiDAR positions and
//! the camera images of every such scan (one texture source each). Its input id is the
//! model's id, the clean id and images of every scan in it, the settings and
//! [`SURFACE_VERSION`].

use crate::error::{check_cancel, failed, StepError};
use crate::map::{
    clean_stage_id, color_input, dense_stage_id, fallback_reason, gpu_json, hex, map_input_for,
    peak_rss_bytes, read_traj, ColorInput, MapInput, CLEAN_BIN, DENSE_BIN,
};
use crate::merge::{member_transform, read_model, ModelMember, ModelReport};
use driftless_export::mesh;
use driftless_gpu::{GpuChoice, GpuContext, GpuPref};
use driftless_io::progress::Progress;
use driftless_io::project::discover_project;
use driftless_io::{atomic_write, paths};
use driftless_map::surface::{
    build_mesh, encode_pages, sensors_for_frames, texture_mesh, Accuracy, AccuracySplit,
    SurfaceConfig, SurfaceInput, SurfaceStats, TextureSource,
};
use driftless_map::{load_stage, stage_fits};
use driftless_traj::nalgebra::{Isometry3, Point3};
use driftless_traj::Pose;
use serde_json::{json, Value};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const SURFACE_REPORT: &str = "oberflaeche.json";
/// Part of the input id: raised whenever the surface gives another result for the same
/// settings, so an older folder counts as stale. Only then: the program version is no part
/// of the id (see the rule at the step versions in [`crate::map`]).
const SURFACE_VERSION: u32 = 13;
/// File stem of the glb and the obj.
const STEM: &str = "oberflaeche";
const AXES: &str = "glb: Y oben; obj: Z oben";

#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceRequest {
    /// What the map (`clean.bin` in [`Self::map_dir`]) was made from.
    pub map: MapInput,
    /// Folder of the map: `clean.bin` is read from it.
    pub map_dir: PathBuf,
    /// Where the surface files go.
    pub out_dir: PathBuf,
    pub config: SurfaceConfig,
    /// Texture the surface from the camera images (`false` = `--no-texture`: gray).
    pub texture: bool,
    /// Find the views per face on the GPU (`--gpu auto|an|aus`). Gives an equivalent
    /// result: not part of the input id; the report says what was used.
    pub gpu: GpuPref,
    /// Also `oberflaeche.obj` with `.mtl` and the page JPEGs (`--obj`). The same surface:
    /// not part of the input id.
    pub obj: bool,
}

fn stale_map() -> StepError {
    "Karte fehlt oder ist veraltet: zuerst `driftless map …` rechnen".into()
}

/// The input id (full text) of a surface and the id of the `clean.bin` it is made from.
struct Plan {
    id: String,
    clean_id: String,
}

/// The camera images of `scan` for the texture, or why it has none (German).
fn images_of(
    scan: &Path,
    lidar_to_imu: &Isometry3<f64>,
    progress: &Progress,
) -> Result<Result<ColorInput, String>, StepError> {
    let project = discover_project(scan).map_err(failed)?;
    Ok(color_input(scan, &project, lidar_to_imu, progress))
}

fn images_part(
    req: &SurfaceRequest,
    progress: &Progress,
) -> Result<(String, Option<ColorInput>), StepError> {
    if !req.texture {
        return Ok(("texture off\n".into(), None));
    }
    let odom = req.map.slam.config()?.odom;
    match images_of(&req.map.scan, &odom.lidar_to_imu, progress)? {
        Ok(c) => Ok((c.id.clone(), Some(c))),
        Err(why) => {
            progress.warn(format!("Oberfläche ohne Textur, {why}"));
            Ok(("no images\n".into(), None))
        }
    }
}

fn plan(
    req: &SurfaceRequest,
    progress: &Progress,
) -> Result<(Plan, Option<ColorInput>), StepError> {
    let clean_id = clean_stage_id(&req.map)?;
    let (images, color) = images_part(req, progress)?;
    let id = format!(
        "{clean_id}surface v{SURFACE_VERSION}\n{images}{:?}\n",
        req.config
    );
    Ok((Plan { id, clean_id }, color))
}

/// Whether `out_dir` holds the surface of this request: its report has the current input id
/// and `clean.bin` still fits the map.
pub fn surface_fits(req: &SurfaceRequest) -> bool {
    let fits = || -> Option<bool> {
        let text = fs::read_to_string(req.out_dir.join(SURFACE_REPORT)).ok()?;
        let report: Value = serde_json::from_str(&text).ok()?;
        // Warnings are of no interest here.
        let (plan, _) = plan(req, &Progress::new()).ok()?;
        Some(
            report["input_id"].as_str() == Some(hex(&plan.id).as_str())
                && stage_fits(&req.map_dir.join(CLEAN_BIN), &plan.clean_id),
        )
    };
    fits().unwrap_or(false)
}

fn settings_json(c: &SurfaceConfig, texture: bool) -> Value {
    json!({
        "texture": texture,
        "voxel": c.voxel,
        "truncation_voxels": c.truncation_voxels,
        "min_weight": c.min_weight,
        "coarse_fallback": c.coarse_fallback,
        "max_fill_gap": c.max_fill_gap,
        "min_component_faces": c.min_component_faces,
        "max_hole_perimeter": c.max_hole_perimeter,
        "simplify_error": c.simplify_error,
        "max_edge": c.max_edge,
        "min_texel": c.min_texel,
        "page_size": c.page_size,
        "views_per_face": c.views_per_face,
        "smooth_weight": c.smooth_weight,
        "jpeg_quality": c.jpeg_quality,
        "color": {
            "downscale": c.color.downscale,
            "highlights": c.color.highlights,
            "max_dist": c.color.max_dist,
            "depth_tolerance": c.color.depth_tolerance,
            "min_sharpness": c.color.min_sharpness,
            "view_gap": c.color.view_gap,
        },
    })
}

fn accuracy_json(a: &Accuracy) -> Value {
    json!({
        "vertices": a.vertices,
        "median_mm": a.median_mm,
        "mean_mm": a.mean_mm,
        "p95_mm": a.p95_mm,
    })
}

fn accuracy_split_json(a: &AccuracySplit) -> Value {
    json!({
        "all": accuracy_json(&a.all),
        "walls": accuracy_json(&a.walls),
        "edges": accuracy_json(&a.edges),
    })
}

/// The points of `dense.bin` of the map, to measure the accuracy against the cloud before
/// cleaning and smoothing. Any failure gives `None` and a line: the surface does not need it.
fn load_dense(req: &SurfaceRequest, progress: &Progress) -> Option<Vec<[f32; 3]>> {
    let loaded = dense_stage_id(&req.map)
        .map_err(|e| e.to_string())
        .and_then(|id| {
            load_stage(&req.map_dir.join(DENSE_BIN), &id)
                .map_err(|e| e.to_string())
                .map(|(cloud, _)| cloud.xyz)
        });
    match loaded {
        Ok(xyz) => Some(xyz),
        Err(why) => {
            progress.info(format!(
                "{DENSE_BIN} nicht lesbar ({why}): keine Genauigkeit gegen die ungeglättete Wolke"
            ));
            None
        }
    }
}

fn stats_json(s: &SurfaceStats) -> Value {
    json!({
        "tsdf_blocks": s.tsdf_blocks,
        "tsdf_bytes": s.tsdf_bytes,
        "faces_raw": s.faces_raw,
        "cells_from_coarse": s.cells_from_coarse,
        "faces_unsupported": s.faces_unsupported,
        "faces_final": s.faces_final,
        "vertices": s.vertices,
        "cleanup": {
            "components_removed": s.cleanup.components_removed,
            "faces_removed": s.cleanup.faces_removed,
            "holes_filled": s.cleanup.holes_filled,
            "holes_left": s.cleanup.holes_left(),
            "holes_too_large": s.cleanup.holes_too_large,
            "holes_rim": s.cleanup.holes_rim,
            "holes_untriangulable": s.cleanup.holes_untriangulable,
        },
        "texture": {
            "faces_textured": s.texture.faces_textured,
            "faces_untextured": s.texture.faces_untextured,
            "charts": s.texture.charts,
            "pages": s.texture.pages,
            "texel_median_mm": s.texture.texel_median_mm,
            "texels_total": s.texture.texels_total,
            "fill_ratio": s.texture.fill_ratio,
            "islands_absorbed": s.texture.islands_absorbed,
            "gray_filled": s.texture.gray_filled,
            "untextured_area_share": s.texture.untextured_area_share,
        },
        "seams": {
            "seam_jump_before": s.seams.seam_jump_before,
            "seam_jump_after": s.seams.seam_jump_after,
            "seam_step_before": s.seams.seam_step_before,
            "seam_step_after": s.seams.seam_step_after,
            "seam_step_median_before": s.seams.seam_step_median_before,
            "seam_step_median_after": s.seams.seam_step_median_after,
            "seam_step_per_m": s.seams.seam_step_per_m,
            "corrections": s.seams.corrections,
            "clamped": s.seams.clamped,
            "content_pairs": s.seams.content_pairs,
        },
        "accuracy_mean_mm": s.accuracy.all.mean_mm,
        "accuracy_p95_mm": s.accuracy.all.p95_mm,
        "accuracy": accuracy_split_json(&s.accuracy),
        "accuracy_dense": s.accuracy_dense.as_ref().map(accuracy_split_json),
        "seconds": s.seconds,
        "gpu_batches": s.gpu_batches,
    })
}

/// The OBJ, its `.mtl` and page JPEGs of an earlier run in `dir` that the files of this run
/// (`keep`) do not include. Nothing is deleted: they are warned about and listed as stale in
/// the report.
fn stale_files(dir: &Path, keep: &[String]) -> Vec<String> {
    let prefix = format!("{STEM}_seite_");
    let (obj, mtl) = (format!("{STEM}.obj"), format!("{STEM}.mtl"));
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut stale: Vec<String> = entries
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| {
            let page = n.starts_with(&prefix) && n.ends_with(".jpg");
            (page || *n == obj || *n == mtl) && !keep.contains(n)
        })
        .collect();
    stale.sort();
    stale
}

/// Opens the device for the views per face if images are to be read (`wanted`), so that
/// `--gpu an` without a GPU fails before any work. Returns it, or why the CPU is used.
fn open_gpu(
    pref: GpuPref,
    wanted: bool,
    progress: &Progress,
) -> Result<(Option<GpuContext>, Option<String>), StepError> {
    if !wanted {
        return Ok((None, None));
    }
    match GpuContext::open(pref).map_err(|e| e.to_string())? {
        GpuChoice::Gpu(ctx) => {
            let i = ctx.info();
            progress.info(format!(
                "Ansichten auf der Grafikkarte: {} ({})",
                i.name, i.backend
            ));
            Ok((Some(ctx), None))
        }
        GpuChoice::Cpu(why) => {
            let no_device = fallback_reason(pref, &why);
            if let Some(why) = &no_device {
                progress.info(format!("Ansichten auf der CPU: {why}"));
            }
            Ok((None, no_device))
        }
    }
}

/// Points with the positions they were measured from, owned: [`SurfaceInput`] of
/// [`make_surface`], freed before the texture pages are made.
struct Points {
    xyz: Vec<[f32; 3]>,
    sensor_of_point: Vec<u32>,
    sensors: Vec<[f32; 3]>,
    dense: Option<Vec<[f32; 3]>>,
}

/// What [`make_surface`] writes and reports.
struct Target<'a> {
    out_dir: &'a Path,
    config: &'a SurfaceConfig,
    texture: bool,
    /// Full text of the input id.
    id: &'a str,
    /// One more entry of the report (key, value).
    extra: Option<(&'static str, Value)>,
    /// Also the OBJ with its `.mtl` and pages.
    obj: bool,
}

/// Mesh, texture, files and report of a surface: the part shared by [`run_surface`] and
/// [`run_model_surface`]. Returns the report as written.
fn make_surface(
    points: Points,
    sources: Vec<TextureSource>,
    gpu: (Option<GpuContext>, Option<String>),
    target: Target,
    start: Instant,
    progress: &Progress,
) -> Result<Value, StepError> {
    let (gpu, no_device) = gpu;
    let input = SurfaceInput {
        xyz: &points.xyz,
        sensor_of_point: &points.sensor_of_point,
        sensors: &points.sensors,
        dense_xyz: points.dense.as_deref(),
    };
    let (mesh, mut stats) = build_mesh(&input, target.config, progress)?;
    // The points are not needed for the texture: free them before the pages are made.
    drop(points);
    let tex = texture_mesh(
        &mesh,
        &sources,
        target.config,
        progress,
        &mut stats,
        gpu.as_ref(),
    )?;
    drop((sources, mesh));
    let fallback = stats.gpu_fallback.as_deref().or(no_device.as_deref());
    let mut gpu_report = gpu_json(gpu.as_ref(), fallback);
    drop(gpu);
    let v = &stats.views_seconds;
    gpu_report["seconds"] = json!({
        "quality": v.quality,
        "pass": v.pass,
        "waiting_for_images": v.waiting_for_images,
    });

    check_cancel(progress)?;
    let quality = target.config.jpeg_quality;
    let jpegs = encode_pages(&tex, quality)?;
    // One conversion for both writers; the textured mesh and the pages go before writing.
    let pages = tex.pages.len();
    let surface = tex.to_mesh(&jpegs)?;
    drop((tex, jpegs));
    // Last chance to stop: from here on files are written. The writers get a progress of
    // their own that cannot be cancelled: a cancel between or in them would leave a new glb
    // next to the old OBJ and report.
    check_cancel(progress)?;
    let out = target.out_dir;
    fs::create_dir_all(out).map_err(|e| format!("{} nicht anlegbar: {e}", out.display()))?;
    let report_path = out.join(SURFACE_REPORT);
    let writing = Progress::new();
    mesh::write_glb(&out.join(format!("{STEM}.glb")), &surface, &writing)?;
    let mut files = vec![format!("{STEM}.glb")];
    if target.obj {
        mesh::write_obj(&out.join(format!("{STEM}.obj")), &surface, &writing)?;
        files.extend([format!("{STEM}.obj"), format!("{STEM}.mtl")]);
        files.extend((0..pages).map(|p| format!("{STEM}_seite_{p:02}.jpg")));
    }
    drop(surface);
    files.push(SURFACE_REPORT.to_string());
    let stale = stale_files(out, &files);
    if !stale.is_empty() {
        let (verb, belongs) = if stale.len() == 1 {
            ("stammt", "gehört")
        } else {
            ("stammen", "gehören")
        };
        progress.warn(format!(
            "{} {verb} aus einem früheren Lauf und {belongs} nicht mehr zur Oberfläche",
            stale.join(", ")
        ));
    }

    let peak = peak_rss_bytes();
    let mut report = json!({
        "input_id": hex(target.id),
        "settings": settings_json(target.config, target.texture),
        "stats": stats_json(&stats),
        "seconds_total": (start.elapsed().as_secs_f64() * 1000.0).round() / 1000.0,
        "peak_rss_bytes": peak,
        "gpu": gpu_report,
        "axes": AXES,
        "files": files,
        "stale": stale,
    });
    if let Some((key, value)) = target.extra {
        report[key] = value;
    }
    let text = serde_json::to_string_pretty(&report).map_err(failed)? + "\n";
    atomic_write(&report_path, text.as_bytes())
        .map_err(|e| format!("{} nicht schreibbar: {e}", report_path.display()))?;

    let t = &stats.texture;
    let (all, walls) = (&stats.accuracy.all, &stats.accuracy.walls);
    progress.info(format!(
        "Oberfläche: {} Flächen ({} mit Bild, {} grau) auf {} Seite(n), Abweichung zu den \
         Punkten im Mittel {:.1} mm (95 % unter {:.1} mm), auf ebenen Wänden Median {:.1} mm",
        stats.faces_final,
        t.faces_textured,
        t.faces_untextured,
        t.pages,
        all.mean_mm,
        all.p95_mm,
        walls.median_mm
    ));
    let ram = peak.map_or_else(
        || "Spitzen-RAM unbekannt".to_string(),
        |b| format!("Spitzen-RAM {:.2} GB", b as f64 / 1e9),
    );
    progress.info(format!(
        "Laufzeit {:.1} s, {ram} → {}",
        start.elapsed().as_secs_f64(),
        out.display()
    ));
    // The value as it reads back from the file (floats lose their last bit in the text).
    serde_json::from_str(&text).map_err(failed)
}

/// Runs the surface as `driftless surface` does; every console line goes to `progress`.
/// Returns the `oberflaeche.json` written to [`SurfaceRequest::out_dir`].
///
/// Fails with „Karte fehlt oder ist veraltet“ if `clean.bin` does not fit the map. A cancel
/// before the files are written leaves everything as it was.
pub fn run_surface(req: &SurfaceRequest, progress: &Progress) -> Result<Value, StepError> {
    let start = Instant::now();
    req.config.check()?;
    req.map.map.check()?;
    let odom = req.map.slam.config()?.odom;
    let (plan, color) = plan(req, progress)?;
    let gpu = open_gpu(req.gpu, color.is_some(), progress)?;
    let (_, poses) = read_traj(&req.map.slam_dir)?;

    let clean_path = req.map_dir.join(CLEAN_BIN);
    let (mut cloud, _) = load_stage(&clean_path, &plan.clean_id).map_err(|e| {
        progress.info(format!("{CLEAN_BIN}: {e}"));
        stale_map()
    })?;
    if cloud.frame.iter().any(|&f| f as usize >= poses.len()) {
        return Err(stale_map());
    }
    progress.info(format!("Karte: {} Punkte aus {CLEAN_BIN}", cloud.len()));
    let sensors = sensors_for_frames(&poses, &odom.lidar_to_imu);
    let dense = load_dense(req, progress);
    let points = Points {
        xyz: std::mem::take(&mut cloud.xyz),
        sensor_of_point: std::mem::take(&mut cloud.frame),
        sensors,
        dense,
    };
    // The rest of the cloud is not needed.
    drop(cloud);
    let sources: Vec<TextureSource> = color
        .map(|c| TextureSource {
            images: c.images,
            poses,
            cams: c.cams,
        })
        .into_iter()
        .collect();
    let target = Target {
        out_dir: &req.out_dir,
        config: &req.config,
        texture: req.texture,
        id: &plan.id,
        extra: None,
        obj: req.obj,
    };
    make_surface(points, sources, gpu, target, start, progress)
}

/// The surface of a merged model.
#[derive(Clone, Debug, PartialEq)]
pub struct ModelSurfaceRequest {
    /// Folder of the model: `modell.json` of `driftless merge` is read from it.
    pub model_dir: PathBuf,
    /// The model as read from [`Self::model_dir`], its folders as the caller found them (the
    /// program takes a model older than the relative paths onto its project folder);
    /// `None`: read here ([`read_model`]).
    pub model: Option<ModelReport>,
    /// Where the surface files go.
    pub out_dir: PathBuf,
    pub config: SurfaceConfig,
    /// Texture the surface from the camera images of the scans (`false`: gray).
    pub texture: bool,
    /// As [`SurfaceRequest::gpu`].
    pub gpu: GpuPref,
    /// As [`SurfaceRequest::obj`].
    pub obj: bool,
}

/// A scan of the model whose points are in it.
struct PlannedMember {
    /// Scan folder name, for messages and the report.
    name: String,
    input: MapInput,
    map_dir: PathBuf,
    clean_id: String,
    /// `T_i`: world of the scan → model.
    transform: Isometry3<f64>,
    lidar_to_imu: Isometry3<f64>,
    /// Its camera images, if the texture is on and it has any.
    color: Option<ColorInput>,
}

/// The input id (full text) of a model surface and the members in the model.
struct ModelPlan {
    id: String,
    members: Vec<PlannedMember>,
}

fn member_name(m: &ModelMember) -> String {
    m.scan.file_name().map_or_else(
        || m.scan.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

fn stale_member(name: &str) -> StepError {
    format!("Karte von {name} fehlt oder ist veraltet: erst Karte rechnen, dann neu zusammenführen")
        .into()
}

fn model_plan(req: &ModelSurfaceRequest, progress: &Progress) -> Result<ModelPlan, StepError> {
    let model = match &req.model {
        Some(m) => m.clone(),
        None => read_model(&req.model_dir)?,
    };
    let mut id = format!("model {}\nsurface v{SURFACE_VERSION}\n", model.input_id);
    let mut members = Vec::new();
    for (k, m) in model.members.iter().enumerate().filter(|(_, m)| m.included) {
        let name = member_name(m);
        let input = map_input_for(&m.scan, &m.slam_dir, &m.map_dir, m.slam.clone());
        input.map.check()?;
        let clean_id = clean_stage_id(&input)?;
        // The map was computed anew after the merge: `T_i` may not fit it any more.
        if m.clean_id.as_ref().is_some_and(|c| *c != hex(&clean_id)) {
            return Err(stale_member(&name));
        }
        let lidar_to_imu = m.slam.config()?.odom.lidar_to_imu;
        let (images, color) = if !req.texture {
            ("texture off\n".to_string(), None)
        } else {
            match images_of(&m.scan, &lidar_to_imu, progress)? {
                Ok(c) => (c.id.clone(), Some(c)),
                Err(why) => {
                    progress.warn(format!("{name} trägt nichts zur Textur bei, {why}"));
                    ("no images\n".to_string(), None)
                }
            }
        };
        id += &format!("member {k}\n{clean_id}{images}");
        members.push(PlannedMember {
            name,
            input,
            map_dir: m.map_dir.clone(),
            clean_id,
            transform: member_transform(m),
            lidar_to_imu,
            color,
        });
    }
    if members.is_empty() {
        return Err(format!("Modell {} enthält keinen Scan", req.model_dir.display()).into());
    }
    id += &format!("{:?}\n", req.config);
    Ok(ModelPlan { id, members })
}

/// Whether `out_dir` holds the surface of this model request: its report has the current
/// input id and the `clean.bin` of every scan in the model still fits its map.
pub fn model_surface_fits(req: &ModelSurfaceRequest) -> bool {
    let fits = || -> Option<bool> {
        let text = fs::read_to_string(req.out_dir.join(SURFACE_REPORT)).ok()?;
        let report: Value = serde_json::from_str(&text).ok()?;
        // Warnings are of no interest here.
        let plan = model_plan(req, &Progress::new()).ok()?;
        Some(
            report["input_id"].as_str() == Some(hex(&plan.id).as_str())
                && plan
                    .members
                    .iter()
                    .all(|m| stage_fits(&m.map_dir.join(CLEAN_BIN), &m.clean_id)),
        )
    };
    fits().unwrap_or(false)
}

/// `t · pose`: a pose of a scan's world in the model.
fn moved_pose(t: &Isometry3<f64>, pose: &Pose) -> Pose {
    Pose {
        t: pose.t,
        p: t.rotation * pose.p + t.translation.vector,
        q: t.rotation * pose.q,
    }
}

/// The points of every member moved to the model, their LiDAR positions (the sensors of a
/// member after those of the members before it) and a texture source per member with
/// images, its poses in the model.
fn model_points(
    plan: ModelPlan,
    progress: &Progress,
) -> Result<(Points, Vec<TextureSource>), StepError> {
    let n = plan.members.len();
    progress.phase("Oberfläche: Karten laden", Some(n as u64));
    let mut xyz = Vec::new();
    let mut sensor_of_point = Vec::new();
    let mut sensors: Vec<[f32; 3]> = Vec::new();
    let mut sources = Vec::new();
    for m in plan.members {
        check_cancel(progress)?;
        let (_, poses) = read_traj(&m.input.slam_dir).map_err(|e| {
            progress.info(format!("{}: {e}", m.name));
            stale_member(&m.name)
        })?;
        let clean_path = m.map_dir.join(CLEAN_BIN);
        let (cloud, _) = load_stage(&clean_path, &m.clean_id).map_err(|e| {
            progress.info(format!("{}: {e}", clean_path.display()));
            stale_member(&m.name)
        })?;
        if cloud.frame.iter().any(|&f| f as usize >= poses.len()) {
            return Err(stale_member(&m.name));
        }
        // Every sensor index of the model has to fit `u32`.
        let offset = u32::try_from(sensors.len() + poses.len())
            .map(|_| sensors.len() as u32)
            .map_err(|_| "Zu viele Frames für eine Oberfläche")?;
        progress.info(format!(
            "{}: {} Punkte aus {CLEAN_BIN}",
            m.name,
            cloud.len()
        ));
        let t = &m.transform;
        xyz.reserve(cloud.len());
        xyz.extend(cloud.xyz.iter().map(|p| {
            let w = t * Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            [w.x as f32, w.y as f32, w.z as f32]
        }));
        sensor_of_point.extend(cloud.frame.iter().map(|&f| f + offset));
        drop(cloud);
        let poses: Vec<Pose> = poses.iter().map(|p| moved_pose(t, p)).collect();
        sensors.extend(sensors_for_frames(&poses, &m.lidar_to_imu));
        if let Some(c) = m.color {
            sources.push(TextureSource {
                images: c.images,
                poses,
                cams: c.cams,
            });
        }
        progress.advance(1);
    }
    progress.info(format!(
        "Modell: {} Punkte aus {n} Scan(s), {} davon mit Bildern",
        xyz.len(),
        sources.len()
    ));
    let points = Points {
        xyz,
        sensor_of_point,
        sensors,
        dense: None,
    };
    Ok((points, sources))
}

/// The model folder relative to the surface folder (`..` for one inside it), if one can be
/// computed ([`paths::relative_to`]; both made absolute as given, the surface folder may not
/// exist yet).
fn model_dir_rel(req: &ModelSurfaceRequest) -> Option<String> {
    let out = std::path::absolute(&req.out_dir).ok()?;
    paths::relative_to(&out, &std::path::absolute(&req.model_dir).ok()?)
}

/// Runs the surface of the model in [`ModelSurfaceRequest::model_dir`] as
/// `driftless surface --model` does: the points, LiDAR positions and camera images of every
/// scan in the model (`included`), in the model frame. Without `dense.bin` the accuracy is
/// measured against the merged points only. Returns the `oberflaeche.json` written to
/// [`ModelSurfaceRequest::out_dir`]; it holds `model` (the folder and the scans in it).
///
/// Fails if `modell.json` is missing or the map of a scan in it is missing or was computed
/// anew after the merge. A cancel before the files are written leaves everything as it was.
pub fn run_model_surface(
    req: &ModelSurfaceRequest,
    progress: &Progress,
) -> Result<Value, StepError> {
    let start = Instant::now();
    req.config.check()?;
    let plan = model_plan(req, progress)?;
    let id = plan.id.clone();
    let names: Vec<String> = plan.members.iter().map(|m| m.name.clone()).collect();
    let with_images = plan.members.iter().any(|m| m.color.is_some());
    let gpu = open_gpu(req.gpu, with_images, progress)?;
    let (points, sources) = model_points(plan, progress)?;
    let target = Target {
        out_dir: &req.out_dir,
        config: &req.config,
        texture: req.texture,
        id: &id,
        extra: Some((
            "model",
            json!({
                "dir": req.model_dir.display().to_string(),
                "dir_rel": model_dir_rel(req),
                "members": names,
            }),
        )),
        obj: req.obj,
    };
    make_surface(points, sources, gpu, target, start, progress)
}
