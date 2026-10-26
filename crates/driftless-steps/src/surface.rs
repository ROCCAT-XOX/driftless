//! The photo-like surface of a map: `oberflaeche.glb`, `oberflaeche.obj` (+ `.mtl` and the
//! texture pages as JPEG) and `oberflaeche.json`, built from the `clean.bin` of the map
//! folder (points with the frame they were measured in) and the camera images of the scan.
//!
//! The input id is the clean step's id, the camera images and calibration, the surface
//! settings and [`SURFACE_VERSION`]. A `clean.bin` of another input (or none) is refused:
//! the surface would not fit the map. A scan without images or calibration gets a gray
//! surface and a warning. A cancel before the files are written leaves the folder untouched;
//! the JSON is written last, so a folder with a report holds a complete surface.

use crate::error::{check_cancel, failed, StepError};
use crate::map::{
    clean_stage_id, color_input, dense_stage_id, fallback_reason, gpu_json, hex, peak_rss_bytes,
    read_traj, MapInput, CLEAN_BIN, DENSE_BIN,
};
use driftless_gpu::{GpuChoice, GpuContext, GpuPref};
use driftless_io::atomic_write;
use driftless_io::progress::Progress;
use driftless_io::project::discover_project;
use driftless_map::surface::{
    build_mesh, encode_pages, sensors_for_frames, texture_mesh, write_glb, write_obj, Accuracy,
    AccuracySplit, SurfaceConfig, SurfaceInput, SurfaceStats, TextureSource,
};
use driftless_map::{load_stage, stage_fits};
use serde_json::{json, Value};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const SURFACE_REPORT: &str = "oberflaeche.json";
/// Part of the input id: raised whenever the surface gives another result for the same
/// settings, so an older folder counts as stale.
const SURFACE_VERSION: u32 = 12;
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
}

fn stale_map() -> StepError {
    "Karte fehlt oder ist veraltet: zuerst `driftless map …` rechnen".into()
}

/// The input id (full text) of a surface and the id of the `clean.bin` it is made from.
struct Plan {
    id: String,
    clean_id: String,
}

fn images_part(
    req: &SurfaceRequest,
    progress: &Progress,
) -> Result<(String, Option<crate::map::ColorInput>), StepError> {
    if !req.texture {
        return Ok(("texture off\n".into(), None));
    }
    let odom = req.map.slam.config()?.odom;
    let project = discover_project(&req.map.scan).map_err(failed)?;
    match color_input(&req.map.scan, &project, &odom.lidar_to_imu, progress) {
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
) -> Result<(Plan, Option<crate::map::ColorInput>), StepError> {
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
        },
        "seams": {
            "seam_jump_before": s.seams.seam_jump_before,
            "seam_jump_after": s.seams.seam_jump_after,
            "seam_step_before": s.seams.seam_step_before,
            "seam_step_after": s.seams.seam_step_after,
            "seam_step_median_before": s.seams.seam_step_median_before,
            "seam_step_median_after": s.seams.seam_step_median_after,
            "corrections": s.seams.corrections,
            "clamped": s.seams.clamped,
            "content_pairs": s.seams.content_pairs,
        },
        "accuracy_mean_mm": s.accuracy.all.mean_mm,
        "accuracy_p95_mm": s.accuracy.all.p95_mm,
        "accuracy": accuracy_split_json(&s.accuracy),
        "accuracy_dense": s.accuracy_dense.as_ref().map(accuracy_split_json),
        "seconds": s.seconds,
    })
}

/// Page JPEGs of an earlier run in `dir` that the files of this run (`keep`) do not include.
/// Nothing is deleted: they are warned about and listed as stale in the report.
fn stale_pages(dir: &Path, keep: &[String]) -> Vec<String> {
    let prefix = format!("{STEM}_seite_");
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut stale: Vec<String> = entries
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with(&prefix) && n.ends_with(".jpg") && !keep.contains(n))
        .collect();
    stale.sort();
    stale
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
    // The device is opened before any work, so `--gpu an` without a GPU fails at once.
    let mut no_device = None;
    let gpu = match &color {
        None => None,
        Some(_) => match GpuContext::open(req.gpu).map_err(|e| e.to_string())? {
            GpuChoice::Gpu(ctx) => {
                let i = ctx.info();
                progress.info(format!(
                    "Ansichten auf der Grafikkarte: {} ({})",
                    i.name, i.backend
                ));
                Some(ctx)
            }
            GpuChoice::Cpu(why) => {
                no_device = fallback_reason(req.gpu, &why);
                if let Some(why) = &no_device {
                    progress.info(format!("Ansichten auf der CPU: {why}"));
                }
                None
            }
        },
    };
    let (_, poses) = read_traj(&req.map.slam_dir)?;

    let clean_path = req.map_dir.join(CLEAN_BIN);
    let (cloud, _) = load_stage(&clean_path, &plan.clean_id).map_err(|e| {
        progress.info(format!("{CLEAN_BIN}: {e}"));
        stale_map()
    })?;
    if cloud.frame.iter().any(|&f| f as usize >= poses.len()) {
        return Err(stale_map());
    }
    progress.info(format!("Karte: {} Punkte aus {CLEAN_BIN}", cloud.len()));
    let sensors = sensors_for_frames(&poses, &odom.lidar_to_imu);
    let dense = load_dense(req, progress);
    let input = SurfaceInput {
        xyz: &cloud.xyz,
        sensor_of_point: &cloud.frame,
        sensors: &sensors,
        dense_xyz: dense.as_deref(),
    };
    let sources: Vec<TextureSource> = color
        .map(|c| TextureSource {
            images: c.images,
            poses,
            cams: c.cams,
        })
        .into_iter()
        .collect();
    let (mesh, mut stats) = build_mesh(&input, &req.config, progress)?;
    // The points are not needed for the texture: free them before the pages are made.
    drop((cloud, dense));
    let tex = texture_mesh(
        &mesh,
        &sources,
        &req.config,
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

    // Last chance to stop: from here on files are written.
    check_cancel(progress)?;
    let out = &req.out_dir;
    fs::create_dir_all(out).map_err(|e| format!("{} nicht anlegbar: {e}", out.display()))?;
    let report_path = out.join(SURFACE_REPORT);
    let quality = req.config.jpeg_quality;
    let jpegs = encode_pages(&tex, quality)?;
    write_glb(&out.join(format!("{STEM}.glb")), &tex, &jpegs)?;
    write_obj(out, STEM, &tex, &jpegs)?;
    drop(jpegs);
    let mut files = vec![
        format!("{STEM}.glb"),
        format!("{STEM}.obj"),
        format!("{STEM}.mtl"),
    ];
    files.extend((0..tex.pages.len()).map(|p| format!("{STEM}_seite_{p:02}.jpg")));
    files.push(SURFACE_REPORT.to_string());
    let stale = stale_pages(out, &files);
    if !stale.is_empty() {
        progress.warn(format!(
            "{} stammen aus einem früheren Lauf und gehören nicht mehr zur Oberfläche",
            stale.join(", ")
        ));
    }

    let peak = peak_rss_bytes();
    let report = json!({
        "input_id": hex(&plan.id),
        "settings": settings_json(&req.config, req.texture),
        "stats": stats_json(&stats),
        "seconds_total": (start.elapsed().as_secs_f64() * 1000.0).round() / 1000.0,
        "peak_rss_bytes": peak,
        "gpu": gpu_report,
        "axes": AXES,
        "files": files,
        "stale": stale,
    });
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
