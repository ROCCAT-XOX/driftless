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
    clean_stage_id, color_input, hex, peak_rss_bytes, read_traj, MapInput, CLEAN_BIN,
};
use driftless_io::progress::Progress;
use driftless_io::project::discover_project;
use driftless_map::surface::{
    build_surface, sensors_for_frames, write_glb, write_obj, SurfaceConfig, SurfaceInput,
    SurfaceStats, TextureSource,
};
use driftless_map::{load_stage, stage_fits};
use serde_json::{json, Value};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const SURFACE_REPORT: &str = "oberflaeche.json";
/// Part of the input id: raised whenever the surface gives another result for the same
/// settings, so an older folder counts as stale.
const SURFACE_VERSION: u32 = 2;
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

fn stats_json(s: &SurfaceStats) -> Value {
    json!({
        "tsdf_blocks": s.tsdf_blocks,
        "faces_raw": s.faces_raw,
        "faces_final": s.faces_final,
        "vertices": s.vertices,
        "cleanup": {
            "components_removed": s.cleanup.components_removed,
            "faces_removed": s.cleanup.faces_removed,
            "holes_filled": s.cleanup.holes_filled,
            "holes_left": s.cleanup.holes_left,
        },
        "texture": {
            "faces_textured": s.texture.faces_textured,
            "faces_untextured": s.texture.faces_untextured,
            "charts": s.texture.charts,
            "pages": s.texture.pages,
            "texel_median_mm": s.texture.texel_median_mm,
            "texels_total": s.texture.texels_total,
            "fill_ratio": s.texture.fill_ratio,
        },
        "seams": {
            "seam_jump_before": s.seams.seam_jump_before,
            "seam_jump_after": s.seams.seam_jump_after,
        },
        "accuracy_mean_mm": s.accuracy_mean_mm,
        "accuracy_p95_mm": s.accuracy_p95_mm,
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
    let input = SurfaceInput {
        xyz: &cloud.xyz,
        sensor_of_point: &cloud.frame,
        sensors: &sensors,
    };
    let sources: Vec<TextureSource> = color
        .map(|c| TextureSource {
            images: c.images,
            poses,
            cams: c.cams,
        })
        .into_iter()
        .collect();
    let (tex, stats) = build_surface(&input, &sources, &req.config, progress)?;
    drop(sources);
    drop(cloud);

    // Last chance to stop: from here on files are written.
    check_cancel(progress)?;
    let out = &req.out_dir;
    fs::create_dir_all(out).map_err(|e| format!("{} nicht anlegbar: {e}", out.display()))?;
    let report_path = out.join(SURFACE_REPORT);
    let quality = req.config.jpeg_quality;
    write_glb(&out.join(format!("{STEM}.glb")), &tex, quality)?;
    write_obj(out, STEM, &tex, quality)?;
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
        "axes": AXES,
        "files": files,
        "stale": stale,
    });
    let text = serde_json::to_string_pretty(&report).map_err(failed)? + "\n";
    let tmp = out.join(format!("{SURFACE_REPORT}.tmp"));
    fs::write(&tmp, &text)
        .and_then(|()| fs::rename(&tmp, &report_path))
        .map_err(|e| {
            let _ = fs::remove_file(&tmp);
            format!("{} nicht schreibbar: {e}", report_path.display())
        })?;

    let t = &stats.texture;
    progress.info(format!(
        "Oberfläche: {} Flächen ({} mit Bild, {} grau) auf {} Seite(n), Abweichung zu den \
         Punkten im Mittel {:.1} mm (95 % unter {:.1} mm)",
        stats.faces_final,
        t.faces_textured,
        t.faces_untextured,
        t.pages,
        stats.accuracy_mean_mm,
        stats.accuracy_p95_mm
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
