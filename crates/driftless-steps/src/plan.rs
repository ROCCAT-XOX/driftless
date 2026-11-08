//! The floor plan of a map or a merged model: `draufsicht.png` and `grundriss.json` (the plan
//! and the report), from the cloud and the trajectory alone ([`driftless_plan`]). Neither the
//! raw data nor a surface are needed, so a plan stays current in a project moved without its
//! scans and can be computed anew there.
//!
//! A map is read from its folder: `color.las`, else `clean.las`, and `traj.tum` there or one
//! folder up. A model: `modell.las` and the trajectories of every scan in it (`included`),
//! moved by its `T_i`. A model merged by another version of the merge is refused
//! ([`version_fits`]).
//!
//! The input id holds what the plan is computed from, and no path: for a map the id of its
//! cleaned cloud from `map.json`, name and size of the LAS file, the content of the
//! trajectory; for a model its id, the size of its LAS file, the clean id and the content of
//! the trajectory of every scan in it; for both the settings and [`PLAN_VERSION`].
//!
//! A run removes the `.tmp` files a killed run of its files left at its start, and the report
//! before it writes the picture; the report comes last again, with the size of the picture,
//! so a folder with a report holds a complete plan. A cancel before the files are written
//! leaves everything as it was, and so does a map of several storeys, which writes nothing and
//! names them ([`PlanOutcome::SeveralStoreys`]).

use crate::error::{check_cancel, failed, StepError};
use crate::leftovers::{remove_report, remove_stale_tmp};
use crate::map::{hex, peak_rss_bytes, read_traj, REPORT};
use crate::merge::{
    member_transform, read_model, version_fits, ModelMember, ModelReport, MERGE_VERSION, MODEL_LAS,
    MODEL_REPORT,
};
use crate::report_text;
use driftless_export::{read_points, Points};
use driftless_io::progress::Progress;
use driftless_io::{atomic_write, TempPath};
use driftless_map::content_hash;
use driftless_plan::{analyze, Analysis, FloorPlan, PlanConfig, PlanInput, Storey, TopView};
use driftless_traj::nalgebra::Point3;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const PLAN_REPORT: &str = "grundriss.json";
pub const TOP_VIEW_FILE: &str = "draufsicht.png";
/// Part of the input id: raised whenever the plan gives another result for the same input,
/// so an older folder counts as stale. Only then: the program version is no part of the id
/// (see the rule at the step versions in [`crate::map`]).
pub const PLAN_VERSION: u32 = 1;

/// The clouds of a map, the first that exists is read.
const MAP_CLOUDS: [&str; 2] = ["color.las", "clean.las"];
const MAP_TRAJ: &str = "traj.tum";

/// What a plan is made from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanSource {
    /// A map folder (with `map.json`).
    Map(PathBuf),
    /// A model folder (with `modell.json`).
    Model(PathBuf),
}

impl PlanSource {
    /// What `dir` is: a map if it has a `map.json`, a model if it has a `modell.json`.
    pub fn detect(dir: &Path) -> Result<PlanSource, StepError> {
        if dir.join(REPORT).is_file() {
            Ok(PlanSource::Map(dir.to_path_buf()))
        } else if dir.join(MODEL_REPORT).is_file() {
            Ok(PlanSource::Model(dir.to_path_buf()))
        } else {
            Err(format!(
                "{} ist weder eine Karte ({REPORT}) noch ein Modell ({MODEL_REPORT})",
                dir.display()
            )
            .into())
        }
    }

    /// The folder of the map or model.
    pub fn dir(&self) -> &Path {
        match self {
            PlanSource::Map(dir) | PlanSource::Model(dir) => dir,
        }
    }
}

/// What can be set for a plan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanSettings {
    /// Which storey to draw, counted from 1 (lowest first); `None` if the map has only one.
    pub storey: Option<usize>,
}

/// A floor plan to compute.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanRequest {
    pub source: PlanSource,
    /// The model as read from [`PlanSource::dir`], its folders as the caller found them (as
    /// [`crate::surface::ModelSurfaceRequest::model`]); `None`: read here ([`read_model`]).
    pub model: Option<ModelReport>,
    /// Where the files go.
    pub out_dir: PathBuf,
    pub settings: PlanSettings,
}

/// How [`TOP_VIEW_FILE`] lies in the frame of the plan.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TopViewInfo {
    pub width: u32,
    pub height: u32,
    /// The corner of the picture with the smallest x and y.
    pub origin: [f64; 2],
    /// Edge of a pixel (metres).
    pub cell: f64,
}

impl From<&TopView> for TopViewInfo {
    fn from(view: &TopView) -> Self {
        TopViewInfo {
            width: view.width,
            height: view.height,
            origin: view.origin,
            cell: view.cell,
        }
    }
}

/// [`PLAN_REPORT`]: the plan and how it came about.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanReport {
    /// `hex` of the input id.
    pub input_id: String,
    /// [`PLAN_VERSION`].
    pub version: u32,
    pub settings: PlanSettings,
    pub plan: FloorPlan,
    pub top_view: TopViewInfo,
    /// The files of the plan, the report last.
    pub files: Vec<String>,
    /// The size of every file in [`Self::files`] but the report.
    pub bytes: BTreeMap<String, u64>,
    /// Points of the cloud read.
    pub points: u64,
    pub seconds_total: f64,
    pub peak_rss_bytes: Option<u64>,
}

/// What came out of [`run_plan`].
#[derive(Clone, Debug, PartialEq)]
pub enum PlanOutcome {
    Done(Box<PlanReport>),
    /// Nothing is guessed and nothing written: one of these has to be chosen
    /// ([`PlanSettings::storey`]).
    SeveralStoreys(Vec<Storey>),
}

/// What a plan is computed from, found without reading the cloud.
struct Prepared {
    /// Full text of the input id.
    id: String,
    /// The cloud to read.
    cloud: PathBuf,
    /// The trajectory, in the frame of the cloud.
    poses: Vec<[f64; 3]>,
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .map_or_else(String::new, |n| n.to_string_lossy().into_owned())
}

fn size_of(path: &Path) -> Result<u64, StepError> {
    let meta = fs::metadata(path).map_err(|e| format!("{} nicht lesbar: {e}", path.display()))?;
    Ok(meta.len())
}

fn positions(poses: &[driftless_traj::Pose]) -> impl Iterator<Item = [f64; 3]> + '_ {
    poses.iter().map(|p| [p.p.x, p.p.y, p.p.z])
}

/// The id of the cleaned cloud of the map in `dir`, from its `map.json`.
fn clean_id_of_map(dir: &Path) -> Result<String, StepError> {
    let path = dir.join(REPORT);
    let unreadable = |e: &dyn fmt::Display| format!("Karte: {} nicht lesbar: {e}", path.display());
    let text = fs::read_to_string(&path).map_err(|e| unreadable(&e))?;
    let report: Value = serde_json::from_str(&text).map_err(|e| unreadable(&e))?;
    match report["steps"]["clean"]["input_id"].as_str() {
        Some(id) => Ok(id.to_string()),
        None => Err(format!(
            "Karte {} ist noch nicht bereinigt: zuerst die Karte rechnen",
            dir.display()
        )
        .into()),
    }
}

fn prepare_map(dir: &Path, settings: &PlanSettings) -> Result<Prepared, StepError> {
    let clean_id = clean_id_of_map(dir)?;
    let cloud = MAP_CLOUDS
        .iter()
        .map(|name| dir.join(name))
        .find(|path| path.is_file())
        .ok_or_else(|| {
            format!(
                "Karte {} hat weder {} noch {}: ohne Punktwolke gibt es keinen Grundriss",
                dir.display(),
                MAP_CLOUDS[0],
                MAP_CLOUDS[1]
            )
        })?;
    let traj_dir = [dir.to_path_buf(), dir.join("..")]
        .into_iter()
        .find(|d| d.join(MAP_TRAJ).is_file())
        .ok_or_else(|| {
            format!(
                "Karte {}: keine Laufbahn ({MAP_TRAJ}) im Ordner oder einen Ordner darüber",
                dir.display()
            )
        })?;
    let (traj, poses) = read_traj(&traj_dir)?;
    let id = format!(
        "plan v{PLAN_VERSION}\nmap {clean_id}\ncloud {} {}\ntraj {:016x}\n{settings:?}\n",
        file_name(&cloud),
        size_of(&cloud)?,
        content_hash(&traj),
    );
    Ok(Prepared {
        id,
        cloud,
        poses: positions(&poses).collect(),
    })
}

fn member_name(m: &ModelMember) -> String {
    m.scan.file_name().map_or_else(
        || m.slam_dir.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

fn prepare_model(
    dir: &Path,
    given: Option<&ModelReport>,
    settings: &PlanSettings,
) -> Result<Prepared, StepError> {
    let read;
    let model = match given {
        Some(model) => model,
        None => {
            read = read_model(dir)?;
            &read
        }
    };
    if !version_fits(model) {
        return Err(format!(
            "Modell veraltet: {} wurde mit einer anderen Programmversion zusammengeführt \
             (v{}, dieses Programm v{MERGE_VERSION}), erst neu zusammenführen",
            dir.display(),
            model.version
        )
        .into());
    }
    let cloud = dir.join(MODEL_LAS);
    let mut id = format!(
        "plan v{PLAN_VERSION}\nmodel {}\ncloud {MODEL_LAS} {}\n",
        model.input_id,
        size_of(&cloud)?,
    );
    let mut poses = Vec::new();
    let mut scans = 0;
    for (k, m) in model.members.iter().enumerate().filter(|(_, m)| m.included) {
        let (traj, scan_poses) =
            read_traj(&m.slam_dir).map_err(|e| format!("Scan {}: {e}", member_name(m)))?;
        let clean_id = m.clean_id.as_deref().unwrap_or("-");
        id += &format!("member {k} {clean_id} {:016x}\n", content_hash(&traj));
        let t = member_transform(m);
        poses.extend(positions(&scan_poses).map(|[x, y, z]| {
            let moved = t * Point3::new(x, y, z);
            [moved.x, moved.y, moved.z]
        }));
        scans += 1;
    }
    if scans == 0 {
        return Err(format!("Modell {} enthält keinen Scan", dir.display()).into());
    }
    id += &format!("{settings:?}\n");
    Ok(Prepared { id, cloud, poses })
}

fn prepare(req: &PlanRequest) -> Result<Prepared, StepError> {
    match &req.source {
        PlanSource::Map(dir) => prepare_map(dir, &req.settings),
        PlanSource::Model(dir) => prepare_model(dir, req.model.as_ref(), &req.settings),
    }
}

/// Whether a plan writes the file `name`.
fn owns(name: &str) -> bool {
    name == TOP_VIEW_FILE || name == PLAN_REPORT
}

/// Whether the files the report lists are there, with the sizes it recorded.
fn files_intact(dir: &Path, report: &PlanReport) -> bool {
    report
        .files
        .iter()
        .filter(|f| f.as_str() != PLAN_REPORT)
        .all(|f| {
            fs::metadata(dir.join(f))
                .is_ok_and(|m| m.is_file() && report.bytes.get(f) == Some(&m.len()))
        })
}

/// Whether [`PlanRequest::out_dir`] holds the plan of this request: its report has the current
/// input id and the files it lists are there with their sizes. Reads no cloud, only the report
/// and the trajectories.
pub fn plan_fits(req: &PlanRequest) -> bool {
    let fits = || -> Option<bool> {
        let report = read_plan(&req.out_dir).ok()?;
        let prepared = prepare(req).ok()?;
        Some(report.input_id == hex(&prepared.id) && files_intact(&req.out_dir, &report))
    };
    fits().unwrap_or(false)
}

/// Reads the [`PLAN_REPORT`] of a plan folder.
pub fn read_plan(dir: &Path) -> Result<PlanReport, StepError> {
    let path = dir.join(PLAN_REPORT);
    let text = fs::read_to_string(&path)
        .map_err(|e| format!("Grundriss {} nicht lesbar: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("Grundriss {}: {e}", path.display()).into())
}

/// Writes the picture as PNG, through `<name>.tmp` next to it.
fn write_top_view(path: &Path, view: &TopView) -> Result<(), StepError> {
    let err = |e: &dyn fmt::Display| format!("{} nicht schreibbar: {e}", path.display());
    let temp = TempPath::new(path);
    let file = fs::File::create(temp.tmp()).map_err(|e| err(&e))?;
    let mut out = BufWriter::new(file);
    let mut encoder = png::Encoder::new(&mut out, view.width, view.height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| err(&e))?;
    writer.write_image_data(&view.rgba).map_err(|e| err(&e))?;
    writer.finish().map_err(|e| err(&e))?;
    out.flush().map_err(|e| err(&e))?;
    // Closed before the rename: Windows renames only closed files reliably.
    drop(out);
    temp.commit().map_err(|e| err(&e))?;
    Ok(())
}

/// The computed plan and what the report says about the run.
struct Computed {
    plan: Box<FloorPlan>,
    view: TopView,
    points: u64,
}

/// Writes the picture and last the report; returns the report as it reads back from the file
/// (floats lose their last bit in the text).
fn write_plan(
    req: &PlanRequest,
    id: &str,
    done: Computed,
    start: Instant,
    progress: &Progress,
) -> Result<PlanReport, StepError> {
    let out = &req.out_dir;
    fs::create_dir_all(out).map_err(|e| format!("{} nicht anlegbar: {e}", out.display()))?;
    let report_path = out.join(PLAN_REPORT);
    remove_report(&report_path)?;
    let picture = out.join(TOP_VIEW_FILE);
    write_top_view(&picture, &done.view)?;
    let size = fs::metadata(&picture).map_or(0, |m| m.len());
    let peak = peak_rss_bytes();
    let report = PlanReport {
        input_id: hex(id),
        version: PLAN_VERSION,
        settings: req.settings,
        plan: *done.plan,
        top_view: TopViewInfo::from(&done.view),
        files: vec![TOP_VIEW_FILE.to_string(), PLAN_REPORT.to_string()],
        bytes: BTreeMap::from([(TOP_VIEW_FILE.to_string(), size)]),
        points: done.points,
        seconds_total: (start.elapsed().as_secs_f64() * 1000.0).round() / 1000.0,
        peak_rss_bytes: peak,
    };
    let text = report_text(&report)?;
    // Read back before it is written: a report that cannot be read is never left behind.
    let back: PlanReport = serde_json::from_str(&text).map_err(failed)?;
    atomic_write(&report_path, text.as_bytes())
        .map_err(|e| format!("{} nicht schreibbar: {e}", report_path.display()))?;

    let plan = &back.plan;
    progress.info(format!(
        "Grundriss: {} Wandflächen, {} Öffnungen, {} Räume",
        plan.faces.len(),
        plan.distinct_openings().len(),
        plan.rooms.len()
    ));
    for hint in &plan.hints {
        progress.info(format!("Hinweis: {hint}"));
    }
    let ram = peak.map_or_else(
        || "Spitzen-RAM unbekannt".to_string(),
        |b| format!("Spitzen-RAM {:.2} GB", b as f64 / 1e9),
    );
    progress.info(format!(
        "Laufzeit {:.1} s, {ram} → {}",
        start.elapsed().as_secs_f64(),
        out.display()
    ));
    Ok(back)
}

/// Computes the plan of a map or a model, as `driftless plan` does; every console line goes to
/// `progress`. Returns the report written to [`PlanRequest::out_dir`], or the storeys to choose
/// from; then nothing is written.
///
/// Fails if the cloud or a trajectory cannot be read, the model is of another version of the
/// merge or no plan comes out of the cloud. A cancel before the files are written leaves
/// everything as it was.
pub fn run_plan(req: &PlanRequest, progress: &Progress) -> Result<PlanOutcome, StepError> {
    let start = Instant::now();
    check_cancel(progress)?;
    let prepared = prepare(req)?;
    remove_stale_tmp(&req.out_dir, owns, progress);

    // The other columns are not needed: only these stay in memory.
    let Points { xyz, rgb, .. } = read_points(&prepared.cloud, progress)?;
    check_cancel(progress)?;
    progress.info(format!(
        "{} Punkte aus {}, {} Lagen der Laufbahn",
        xyz.len(),
        file_name(&prepared.cloud),
        prepared.poses.len()
    ));
    let input = PlanInput {
        xyz: &xyz,
        rgb: rgb.as_deref(),
        poses: &prepared.poses,
    };
    let config = PlanConfig {
        storey: req.settings.storey,
        ..PlanConfig::default()
    };
    let (plan, view) = match analyze(&input, &config, progress)? {
        Analysis::Plan(plan, view) => (plan, view),
        Analysis::SeveralStoreys(storeys) => return Ok(PlanOutcome::SeveralStoreys(storeys)),
    };
    let points = xyz.len() as u64;
    drop((xyz, rgb));

    // Last chance to stop: from here on files are written, and they are written to the end.
    check_cancel(progress)?;
    progress.phase("Grundriss schreiben", None);
    let done = Computed { plan, view, points };
    let report = write_plan(req, &prepared.id, done, start, progress)?;
    Ok(PlanOutcome::Done(Box::new(report)))
}
