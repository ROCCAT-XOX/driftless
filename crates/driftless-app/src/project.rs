//! A Driftless project on disk:
//!
//! ```text
//! <projekt>/project.toml
//! <projekt>/scans/<scan-id>/slam-001/          a SLAM run, the same folder as `driftless slam -o`
//!                          slam-001/lauf.toml  scan and settings of the run
//!                          slam-001/verlauf/   states of the run (driftless_steps::history)
//!                          slam-001/karte-001/ a map of the run, one folder per set of switches
//!                karte-001/oberflaeche-001/    a surface of the map (`driftless surface -o`)
//! <projekt>/modelle/modell-001/                 scans merged into one model (`driftless merge -o`)
//!                  modell-001/oberflaeche-001/  a surface of the model (`driftless surface --model`)
//! ```
//!
//! `project.toml`:
//!
//! ```toml
//! format = 1
//! name = "Büro"
//!
//! [[scan]]
//! id = "buero"
//! path = "rohdaten/buero"        # relative to the project folder if inside it, else absolute
//! name = "Büro, Erdgeschoss"
//! extra_runs = ["/pfad/zu/cli-lauf"]  # run folders made by the CLI, added by hand
//!
//! [[scan]]
//! id = "halle"
//! path = "/Volumes/Daten/Rohdaten/halle"
//! path_rel = "../Rohdaten/halle" # beside an absolute path: relative to the project folder
//! name = "Halle"
//! extra_runs = ["/Volumes/Daten/cli/lauf"]
//! extra_runs_rel = ["../cli/lauf"]  # one per entry of `extra_runs`, "" where none
//! ```
//!
//! Relative paths are written with `/` and read with `/` or `\`, so a project works on macOS
//! and Windows alike. A path outside the project folder is written absolute and, where one can
//! be computed (the same volume, drive or share), relative to the project folder as well
//! (`path_rel`, `extra_runs_rel`); reading takes the relative one where that folder exists,
//! else the absolute one. So a project moved or copied together with its raw data still finds
//! them, and one moved alone finds them where they were. `extra_runs_rel` counts only with as
//! many entries as `extra_runs`. `lauf.toml` holds `scan` (stored the same way, with
//! `scan_rel`), `erstellt` (UTC) and the [`SlamSettings`] under `[slam]`; `erstellt` is the
//! only value that differs between two equal runs (like `erstellt`/`sekunden` in `stand.toml`
//! and the runtimes in `map.json`).
//!
//! The paths in `modell.json` are relative to the model folder as well
//! ([`driftless_steps::merge`]). A model written before that has absolute paths only: if the
//! project folder was renamed, moved or copied since, [`Project::models`] takes the folders
//! that lay in the project folder then into the project folder now ([`merged_in`]).
//!
//! A scan folder that is gone (an unplugged drive) is reported by [`Project::scan_state`];
//! the project still opens and its runs are still listed.
//!
//! Folders for results are only created by the jobs ([`run_slam_job`], [`run_map_job`]), never
//! on the UI thread, and a job removes a folder it created when it is cancelled or fails. A
//! full SLAM run always gets a new `slam-NNN` (so no earlier `traj.tum` is replaced); a reuse
//! recomputes a run in its folder and, if it fails, leaves it as it was
//! ([`driftless_steps::slam`]). A map is never computed into an existing `karte-NNN`: one whose
//! map is current is taken as it is, any other map goes into a new one ([`map_dir_for`]). A
//! surface is only computed if no `oberflaeche-NNN` of the map fits the request, and then into a
//! new one ([`run_surface_job`]); a folder without `oberflaeche.json` is no surface, even with
//! its glb (the report is written last). A job that writes into an existing folder (a reuse)
//! marks it busy as a job's new folder is ([`StepDir`], [`mark_busy`]). A run or map folder
//! without its report that holds finished results (maps, surfaces, the history of a run) is
//! no leftover: it stays in the tree as „unvollständig“ ([`crate::leftovers::holds_results`]).
//! Every merge gets a new `modell-NNN`
//! ([`run_merge_job`]); a folder without `modell.json` is no model (the report is written
//! last). `project.toml`
//! does not list the models: [`Project::models`] reads the folder. Point pairs marked by hand
//! are kept in `modelle/punktpaare.toml` ([`driftless_steps::pairs`], paths relative to that
//! folder) and count in every merge of the project.

use crate::ui::text;
use driftless_gpu::GpuPref;
use driftless_io::atomic_write;
use driftless_io::paths::{relative_text, relative_to};
use driftless_io::progress::Progress;
use driftless_io::project::discover_project;
use driftless_map::surface::SurfaceConfig;
use driftless_slam::align::AlignConfig;
use driftless_steps::correct::utc_now;
use driftless_steps::history::{self, HistoryEntry};
use driftless_steps::map::{
    clean_stage_id, current_map, fitting_stages, folder_fits, hex, map_input_for, recompute_map,
    take_current_map, MapInput, MapRequest, MapSettings, MapStep, MapSwitches, CLEAN_BIN,
    DENSE_BIN, REPORT,
};
use driftless_steps::merge::{
    read_model, run_merge, version_fits, MergeMember, MergeRequest, ModelReport,
};
use driftless_steps::pairs::{
    stored_path as from_stored, stored_text, PointPairEntry, PointPairs, POINT_PAIRS_FILE,
};
use driftless_steps::slam::{
    cache_fits, run_fits_scan, run_slam, scan_times, SlamOutcome, SlamRequest, SlamSettings,
    CACHE_FILE, CORRECTIONS_FILE,
};
use driftless_steps::surface::{
    model_surface_fits, run_model_surface, run_surface, surface_fits, ModelSurfaceRequest,
    SurfaceRequest, SURFACE_REPORT,
};
use driftless_steps::StepError;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};

pub const PROJECT_FILE: &str = "project.toml";
/// The choices of the project's last session: `<projekt>/auswahl.toml` ([`Choices`]).
pub const CHOICES_FILE: &str = "auswahl.toml";
pub const RUN_FILE: &str = "lauf.toml";
pub const SCANS_DIR: &str = "scans";
/// The models of the project: `modelle/modell-NNN`.
pub const MODELS_DIR: &str = "modelle";
pub const MODEL_PREFIX: &str = "modell-";
/// One point per cell of this size (m) in a model, as `driftless merge` does by default.
pub const MODEL_VOXEL: f64 = 0.01;
const FORMAT: i64 = 1;
pub(crate) const RUN_PREFIX: &str = "slam-";
pub(crate) const MAP_PREFIX: &str = "karte-";
/// Surfaces of a map: `oberflaeche-NNN` made here, other `oberflaeche…` folders by the CLI.
pub(crate) const SURFACE_PREFIX: &str = "oberflaeche";
pub(crate) const SURFACE_NUMBERED: &str = "oberflaeche-";
/// The clouds of a map folder that can be shown, best first.
const MAP_CLOUDS: [&str; 4] = ["color.las", "clean.las", "dense.las", "removed.las"];

/// Why a project could not be read or changed (German, for display).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProjectError(pub String);

impl fmt::Display for ProjectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ProjectError {}

impl From<String> for ProjectError {
    fn from(s: String) -> Self {
        ProjectError(s)
    }
}

impl From<ProjectError> for StepError {
    fn from(e: ProjectError) -> Self {
        StepError::from(e.0)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Project {
    /// The project folder (absolute).
    pub dir: PathBuf,
    pub name: String,
    pub scans: Vec<ScanEntry>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScanEntry {
    /// Folder name under `scans/`, unique in the project.
    pub id: String,
    /// The scan folder (absolute).
    pub path: PathBuf,
    /// Shown name.
    pub name: String,
    /// Run folders made by the CLI and added by hand (absolute).
    pub extra_runs: Vec<PathBuf>,
    /// The LiDAR bags of the scan when it was added (name, bytes): what tells it from another
    /// scan when its folder is assigned anew ([`Project::relink_scan`]). Empty for a scan added
    /// before this was kept.
    pub bags: Vec<(String, u64)>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ScanState {
    Ready,
    /// Why the scan cannot be read (German).
    Missing(String),
}

#[derive(Clone, Debug, PartialEq)]
pub struct RunInfo {
    pub dir: PathBuf,
    /// „SLAM 001“ for a run of the project, the folder name for an added one.
    pub label: String,
    /// From `lauf.toml`; the defaults without it.
    pub settings: SlamSettings,
    /// `slam.json` and `traj.tum` exist.
    pub has_result: bool,
    /// Its report `slam.json` is missing (a recompute was cut off, or an older program's):
    /// the tree keeps such a run only while it holds finished results, as „unvollständig – neu
    /// rechnen“ ([`crate::leftovers::holds_results`]).
    pub incomplete: bool,
    /// `submaps.bin` exists: the run can be recomputed.
    pub has_cache: bool,
    /// `traj.vorher.tum` exists.
    pub has_previous: bool,
    /// `traj_odom.tum` exists.
    pub has_odometry: bool,
    /// `map.las` of the run.
    pub cloud: Option<PathBuf>,
    pub maps: Vec<MapInfo>,
    pub history: Vec<HistoryEntry>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MapInfo {
    pub dir: PathBuf,
    /// „Karte 001“, or „Karte (im Lauf-Ordner)“ for a map the CLI wrote into the run folder.
    pub label: String,
    /// The switches recorded in `map.json`; `None` without a readable one.
    pub settings: Option<MapSwitches>,
    /// The existing ones of `color.las`, `clean.las`, `dense.las`, `removed.las`, in this order.
    pub clouds: Vec<PathBuf>,
    /// The surfaces of the map: its subfolders `oberflaeche…` with an `oberflaeche.json` (the
    /// step writes it last; one without it is cut off, see [`crate::leftovers`]).
    pub surfaces: Vec<SurfaceInfo>,
    /// Its report `map.json` is missing (computed in its folder by an older program or the
    /// command line, and cut off): the tree keeps such a map only while it holds finished
    /// surfaces, as „unvollständig – neu rechnen“ ([`crate::leftovers::holds_results`]).
    pub incomplete: bool,
}

/// A surface folder of a map.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceInfo {
    pub dir: PathBuf,
    /// „Oberfläche 001“ for `oberflaeche-001`, else the folder name.
    pub label: String,
    /// It no longer fits its map (the map or the run was computed anew since). False until
    /// the background check said so ([`surface_status`], `AppState::map_status`).
    pub stale: bool,
    /// What it was made with, from its `oberflaeche.json` ([`SurfaceSettings::from_report`]);
    /// `None` without a readable one.
    pub settings: Option<SurfaceSettings>,
    /// The hints of its `oberflaeche.json` for the user (`hinweise`, German, e.g. why it stays
    /// open in places); empty without them (also in a report of before the field).
    pub hints: Vec<String>,
}

/// A model of the project: scans merged into one (`modelle/modell-NNN` with `modell.json`).
#[derive(Clone, Debug)]
pub struct ModelInfo {
    pub dir: PathBuf,
    /// „Modell 001“.
    pub label: String,
    /// Its `modell.json`: where each scan sits and how sure that is.
    pub report: ModelReport,
    /// Its surfaces: subfolders `oberflaeche…` with an `oberflaeche.json`, as for a map.
    pub surfaces: Vec<SurfaceInfo>,
}

impl ModelInfo {
    /// Whether it was merged by another version of the merge than this program's
    /// ([`driftless_steps::merge::version_fits`]): stale whatever its maps, merge anew.
    pub fn other_version(&self) -> bool {
        !version_fits(&self.report)
    }
}

/// A scan chosen for a model, with the run and the map of it to merge.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MergePick {
    pub scan_id: String,
    pub run: PathBuf,
    pub map: PathBuf,
}

/// What the next surface is computed with: the settings of `driftless surface` (the program
/// offers the voxel size and „ohne Fotos“; the rest are the defaults).
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceSettings {
    pub config: SurfaceConfig,
    /// Texture from the camera images (`false` = `--no-texture`: gray).
    pub texture: bool,
}

impl Default for SurfaceSettings {
    fn default() -> Self {
        SurfaceSettings {
            config: SurfaceConfig::default(),
            texture: true,
        }
    }
}

impl SurfaceSettings {
    /// The settings recorded under `settings` in an `oberflaeche.json`. The report does not
    /// hold every camera setting; those take the defaults, which is what the program and
    /// `driftless surface` always use. `None` if a value is missing or of the wrong type;
    /// reports from before the coarse grid have no `coarse_fallback` and read as `true`, and
    /// ones without `max_fill_gap` take the default.
    pub fn from_report(report: &serde_json::Value) -> Option<SurfaceSettings> {
        let s = &report["settings"];
        let f64_of = |v: &serde_json::Value| v.as_f64();
        let u64_of = |v: &serde_json::Value| v.as_u64();
        let c = &s["color"];
        let defaults = SurfaceConfig::default();
        let config = SurfaceConfig {
            voxel: f64_of(&s["voxel"])?,
            truncation_voxels: f64_of(&s["truncation_voxels"])? as f32,
            min_weight: u64_of(&s["min_weight"])?.try_into().ok()?,
            coarse_fallback: match &s["coarse_fallback"] {
                serde_json::Value::Null => true,
                v => v.as_bool()?,
            },
            max_fill_gap: match &s["max_fill_gap"] {
                serde_json::Value::Null => defaults.max_fill_gap,
                v => v.as_f64()?,
            },
            min_component_faces: u64_of(&s["min_component_faces"])?.try_into().ok()?,
            max_hole_perimeter: f64_of(&s["max_hole_perimeter"])?,
            simplify_error: f64_of(&s["simplify_error"])?,
            max_edge: f64_of(&s["max_edge"])?,
            min_texel: f64_of(&s["min_texel"])?,
            page_size: u64_of(&s["page_size"])?.try_into().ok()?,
            views_per_face: u64_of(&s["views_per_face"])?.try_into().ok()?,
            smooth_weight: f64_of(&s["smooth_weight"])? as f32,
            jpeg_quality: u64_of(&s["jpeg_quality"])?.try_into().ok()?,
            color: driftless_map::ColorConfig {
                downscale: u64_of(&c["downscale"])?.try_into().ok()?,
                highlights: c["highlights"].as_bool()?,
                max_dist: f64_of(&c["max_dist"])?,
                depth_tolerance: f64_of(&c["depth_tolerance"])?,
                min_sharpness: f64_of(&c["min_sharpness"])?,
                view_gap: f64_of(&c["view_gap"])?,
                ..defaults.color
            },
        };
        Some(SurfaceSettings {
            config,
            texture: s["texture"].as_bool()?,
        })
    }
}

/// The lock of the file `file` while it is read, changed and written: another window of the
/// program or the command line may change it at the same time.
fn lock(file: &Path) -> Result<driftless_io::lock::FileLock, ProjectError> {
    driftless_io::lock::FileLock::acquire(file).map_err(|e| ProjectError(e.to_string()))
}

/// Changes `project.toml` of the project in `dir` with `change`: under its lock, on the
/// project as it is on disk now (so a change another window or program saved meanwhile is
/// kept), then saved. Returns the project as written, whether the file differed from `known`
/// (what the window showed) before the change, and what `change` returned.
pub fn change_project<T>(
    dir: &Path,
    known: Option<&Project>,
    change: impl FnOnce(&mut Project) -> Result<T, ProjectError>,
) -> Result<(Project, bool, T), ProjectError> {
    let _lock = lock(&dir.join(PROJECT_FILE))?;
    let mut project = Project::open(dir)?;
    let changed_meanwhile = known.is_some_and(|k| *k != project);
    let out = change(&mut project)?;
    project.save()?;
    Ok((project, changed_meanwhile, out))
}

/// What was chosen in a project for the next map and surface, kept in its folder
/// (`auswahl.toml`), so that the project opens with them again, also on another computer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Choices {
    /// „Genauigkeit“: the voxel size of the next map (m).
    pub genauigkeit: f64,
    /// „Feinheit“: the voxel size of the next surface (m).
    pub feinheit: f64,
    /// „ohne Fotos (graue Fläche)“.
    pub ohne_fotos: bool,
}

/// The comment at the top of `auswahl.toml`.
const CHOICES_HEADER: &str =
    "# Driftless: die zuletzt in diesem Projekt gewählte Genauigkeit und Feinheit.\n";

impl Choices {
    /// The choices in `settings` (map) and `surface`.
    pub fn of(map: &MapSettings, surface: &SurfaceSettings) -> Choices {
        Choices {
            genauigkeit: map.switches.voxel,
            feinheit: surface.config.voxel,
            ohne_fotos: !surface.texture,
        }
    }

    /// Sets them in `map` and `surface`; a value that is no valid size keeps the one there.
    pub fn apply(&self, map: &mut MapSettings, surface: &mut SurfaceSettings) {
        let valid = |v: f64| v.is_finite() && v > 0.0;
        if valid(self.genauigkeit) {
            map.switches.voxel = self.genauigkeit;
        }
        if valid(self.feinheit) {
            surface.config.voxel = self.feinheit;
        }
        surface.texture = !self.ohne_fotos;
    }

    /// The choices of the project in `dir`; `None` without a readable `auswahl.toml`.
    pub fn read(dir: &Path) -> Option<Choices> {
        let text = fs::read_to_string(dir.join(CHOICES_FILE)).ok()?;
        toml::from_str(&text).ok()
    }

    /// Writes them into the project in `dir`, under the lock of the file.
    pub fn save(&self, dir: &Path) -> Result<(), ProjectError> {
        let file = dir.join(CHOICES_FILE);
        let _lock = lock(&file)?;
        let body = toml::to_string(self)
            .map_err(|e| ProjectError(text::about_path(&e.to_string(), &file)))?;
        atomic_write(&file, format!("{CHOICES_HEADER}{body}").as_bytes())
            .map_err(|e| ProjectError(text::io_error("auswahl.toml nicht schreibbar", &file, &e)))
    }
}

/// What a map folder holds, for the step list: read from its `map.json`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapStatus {
    /// `seconds_total` of its `map.json`.
    pub seconds: Option<f64>,
    /// Its switches are recorded, but it no longer fits the run (the trajectory, the bags or
    /// the odometry settings changed since): computing a map would make it anew.
    pub stale: bool,
}

/// A folder a job writes into. One the job created ([`StepDir::is_new`]) is removed again
/// when the guard is dropped without [`StepDir::keep`], i.e. when the job is cancelled or
/// fails; while the job runs it is marked busy ([`crate::leftovers::BUSY_FILE`]), so that no
/// window offers it for „Aufräumen“ or „Löschen …“.
#[must_use]
#[derive(Debug)]
pub struct StepDir {
    path: PathBuf,
    new: bool,
    beat: Option<Beat>,
}

impl StepDir {
    /// The folder `path` just created for a job: marked busy until kept or dropped.
    fn made(path: PathBuf) -> StepDir {
        StepDir {
            beat: Beat::start(&path),
            path,
            new: true,
        }
    }

    /// The existing folder `path`, which a job takes as it is (a current map: nothing is
    /// written); never removed.
    fn existing(path: PathBuf) -> StepDir {
        StepDir {
            path,
            new: false,
            beat: None,
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Created for this job (else an existing folder that is never removed).
    pub fn is_new(&self) -> bool {
        self.new
    }

    /// Keeps the folder; returns its path.
    pub fn keep(mut self) -> PathBuf {
        // the mark goes with it
        self.beat = None;
        self.new = false;
        std::mem::take(&mut self.path)
    }
}

impl Drop for StepDir {
    fn drop(&mut self) {
        self.beat = None;
        if self.new {
            let _ = fs::remove_dir_all(&self.path);
        }
    }
}

/// The mark of a folder a job writes into in place (a reuse of a run, „Diesen Stand
/// wiederherstellen“): [`crate::leftovers::BUSY_FILE`] while the guard lives, so that another
/// window neither offers the folder for „Aufräumen“ nor lets it be deleted.
#[must_use]
#[derive(Debug)]
pub struct BusyMark {
    _beat: Option<Beat>,
}

/// Marks the existing folder `dir` busy until the guard is dropped (nothing if the mark
/// cannot be written: the job runs all the same).
pub fn mark_busy(dir: &Path) -> BusyMark {
    BusyMark {
        _beat: Beat::start(dir),
    }
}

/// The mark of a busy folder ([`crate::leftovers::BUSY_FILE`], naming the process): written at
/// once, then touched every [`crate::leftovers::BUSY_BEAT`] by a thread of its own until this
/// is dropped; then it is removed.
#[derive(Debug)]
struct Beat {
    file: PathBuf,
    stop: Option<std::sync::mpsc::Sender<()>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Beat {
    fn start(dir: &Path) -> Option<Beat> {
        use crate::leftovers::{BUSY_BEAT, BUSY_FILE};
        use std::sync::mpsc::{channel, RecvTimeoutError};
        let file = dir.join(BUSY_FILE);
        fs::write(&file, driftless_io::lock::holder_text()).ok()?;
        let (stop, wait) = channel::<()>();
        let touch = file.clone();
        let thread = std::thread::Builder::new()
            .name("in-arbeit".into())
            .spawn(move || {
                while let Err(RecvTimeoutError::Timeout) = wait.recv_timeout(BUSY_BEAT) {
                    let touched = fs::File::options().write(true).open(&touch);
                    let _ = touched.and_then(|f| f.set_modified(std::time::SystemTime::now()));
                }
            });
        let Ok(thread) = thread else {
            let _ = fs::remove_file(&file);
            return None;
        };
        Some(Beat {
            file,
            stop: Some(stop),
            thread: Some(thread),
        })
    }
}

impl Drop for Beat {
    fn drop(&mut self) {
        // the thread ends when the sender goes
        self.stop = None;
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        let _ = fs::remove_file(&self.file);
    }
}

#[derive(Serialize, Deserialize)]
struct ProjectFile {
    format: i64,
    name: String,
    #[serde(default, rename = "scan")]
    scans: Vec<ScanFile>,
}

#[derive(Serialize, Deserialize)]
struct ScanFile {
    id: String,
    /// Relative to the project folder if inside it, else absolute.
    path: String,
    /// Beside an absolute [`Self::path`]: relative to the project folder, if one can be
    /// computed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    path_rel: Option<String>,
    name: String,
    #[serde(default)]
    extra_runs: Vec<String>,
    /// One per entry of [`Self::extra_runs`]: the relative path beside an absolute one, `""`
    /// where there is none. Written only if one entry has one.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    extra_runs_rel: Vec<String>,
    /// [`ScanEntry::bags`] as „<name> <bytes>“.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    bags: Vec<String>,
}

#[derive(Serialize, Deserialize)]
struct RunFile {
    scan: String,
    /// As [`ScanFile::path_rel`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    scan_rel: Option<String>,
    erstellt: String,
    #[serde(default)]
    slam: SlamSettings,
}

/// The absolute, resolved form of an existing path, in its plain form on Windows: `C:\…`
/// and `\\server\share\…`, without the `\\?\` that resolving adds (also for a share or a
/// mapped drive). Paths in a [`Project`] are always in this form.
pub use driftless_io::paths::canonical;

/// `path` as written to a project file: relative to `base` with `/` if inside it; else
/// absolute, and beside it relative to `base` where one can be computed (the same volume,
/// drive or share, [`relative_to`]). Compared as text, so a share of Windows counts as well.
fn to_stored(base: &Path, path: &Path) -> Result<(String, Option<String>), ProjectError> {
    let text = path
        .to_str()
        .ok_or_else(|| ProjectError(text::about_path("Pfad ist kein gültiger Text", path)))?;
    let inside = base
        .to_str()
        .and_then(|b| relative_text(b, text))
        .filter(|rel| rel != "." && rel != ".." && !rel.starts_with("../"));
    Ok(match inside {
        Some(rel) => (rel, None),
        None => (
            text.to_string(),
            relative_to(base, path).filter(|rel| rel != "."),
        ),
    })
}

/// A path of a project file as written by [`to_stored`]: the relative one beside it (if
/// any, and not empty) where that folder exists, else the path itself; both relative to
/// `base` if relative.
fn from_stored_pair(base: &Path, path: &str, rel: Option<&str>) -> PathBuf {
    rel.filter(|r| !r.is_empty())
        .map(|r| from_stored(base, r))
        .filter(|p| p.is_dir())
        .unwrap_or_else(|| from_stored(base, path))
}

/// Windows device names, which name no folder whatever the extension.
const WINDOWS_DEVICES: [&str; 22] = [
    "con", "prn", "aux", "nul", "com1", "com2", "com3", "com4", "com5", "com6", "com7", "com8",
    "com9", "lpt1", "lpt2", "lpt3", "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9",
];

/// A scan id names exactly one folder in `scans/`: a single plain name, without `/`, `\`
/// and `:`, not `.` or `..`. Windows drops a trailing dot or space (`".. "` would become
/// `..`) and maps device names to devices, so both are refused on every platform.
fn check_id(id: &str) -> Result<(), ProjectError> {
    let mut parts = Path::new(id).components();
    let single = matches!(
        (parts.next(), parts.next()),
        (Some(Component::Normal(_)), None)
    );
    let stem = id.split('.').next().unwrap_or("").trim_end().to_lowercase();
    if single
        && !id.contains(['/', '\\', ':'])
        && id != "."
        && id != ".."
        && !id.ends_with(['.', ' '])
        && !WINDOWS_DEVICES.contains(&stem.as_str())
    {
        Ok(())
    } else {
        Err(ProjectError(format!(
            "ungültige Scan-Kennung „{id}“: erlaubt ist nur ein einfacher Ordnername \
             (ohne /, \\ und :, nicht . oder .., ohne Punkt oder Leerzeichen am Ende, kein \
             Windows-Gerätename)"
        )))
    }
}

/// Every id valid and unique, ignoring case (on macOS and Windows `Buero` and `buero` are
/// the same folder).
fn check_ids(scans: &[ScanEntry]) -> Result<(), ProjectError> {
    let mut seen = std::collections::BTreeSet::new();
    for s in scans {
        check_id(&s.id)?;
        if !seen.insert(s.id.to_lowercase()) {
            return Err(ProjectError(format!(
                "Scan-Kennung „{}“ kommt mehrfach vor (Groß- und Kleinschreibung zählen nicht)",
                s.id
            )));
        }
    }
    Ok(())
}

/// Checks that the run in `run` (its `slam.json`) was made from the scan folder `scan`: its
/// frames lie within the times of the scan's LiDAR bags. Without an index in the bags the
/// times are unknown and the run is taken.
fn check_run_scan(run: &Path, scan: &Path) -> Result<(), ProjectError> {
    let file = run.join("slam.json");
    let report: serde_json::Value = fs::read_to_string(&file)
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .ok_or_else(|| {
            ProjectError(text::about_path(
                "Kein lesbarer SLAM-Bericht (slam.json fehlt oder ist beschädigt)",
                &file,
            ))
        })?;
    let start = report["scan_start"].as_f64().ok_or_else(|| {
        ProjectError(text::about_path(
            "slam.json ist beschädigt: „scan_start“ fehlt",
            &file,
        ))
    })?;
    let end = start
        + report["submaps"]
            .as_array()
            .and_then(|m| m.last())
            .and_then(|m| m["t_end"].as_f64())
            .unwrap_or(0.0);
    let project =
        discover_project(scan).map_err(|e| ProjectError(format!("Lauf nicht prüfbar: {e}")))?;
    let Some((lo, hi)) = scan_times(&project.lidar_bags) else {
        return Ok(());
    };
    if run_fits_scan((start, end), (lo, hi)) {
        Ok(())
    } else {
        Err(ProjectError(format!(
            "Lauf gehört nicht zu diesem Scan: er umfasst {start:.1}–{end:.1} s, der Scan \
             {lo:.1}–{hi:.1} s: {}\nLauf: {}\nScan: {}",
            run.file_name().unwrap_or_default().to_string_lossy(),
            run.display(),
            scan.display()
        )))
    }
}

/// The LiDAR bags of a scan folder: name and bytes.
fn bag_stamps(found: &driftless_io::project::Project) -> Vec<(String, u64)> {
    found
        .lidar_bags
        .iter()
        .map(|b| {
            let name = b
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            let bytes = fs::metadata(b).map_or(0, |m| m.len());
            (name, bytes)
        })
        .collect()
}

/// Whether the scan folder `found` holds the scan `scan` with the runs `runs` ([`Project::
/// relink_scan`]): the same bags by name and size as when it was added, and recorded at the
/// times of its runs. `Ok(false)`: nothing to tell it by (added before the bags were kept, and
/// no run yet).
fn same_scan(
    scan: &ScanEntry,
    runs: &[RunInfo],
    found: &driftless_io::project::Project,
) -> Result<bool, ProjectError> {
    let different = |why: String| ProjectError(text::about_path(&why, &found.root));
    let mut checked = false;
    if !scan.bags.is_empty() {
        let now = bag_stamps(found);
        if now != scan.bags {
            return Err(different(text::other_scan_bags(&scan.bags, &now)));
        }
        checked = true;
    }
    // the recording times of a run (from the start of the scan to the end of its last submap)
    let run = runs
        .iter()
        .find_map(|r| driftless_steps::slam::run_times(&r.dir));
    if let (Some(run), Some(times)) = (run, scan_times(&found.lidar_bags)) {
        if !run_fits_scan(run, times) {
            return Err(different(text::other_scan_times(run, times)));
        }
        checked = true;
    }
    Ok(checked)
}

/// Two paths name the same folder (resolved where they exist).
fn same_dir(a: &Path, b: &Path) -> bool {
    let resolve = |p: &Path| canonical(p).unwrap_or_else(|_| p.to_path_buf());
    resolve(a) == resolve(b)
}

/// Subfolders `<prefix><digits>` of `dir` as (number, digits, path), by number.
pub(crate) fn numbered(dir: &Path, prefix: &str) -> Vec<(u64, String, PathBuf)> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut out: Vec<_> = entries
        .filter_map(Result::ok)
        .filter(|e| e.path().is_dir())
        .filter_map(|e| {
            let name = e.file_name().to_str()?.to_string();
            let digits = name.strip_prefix(prefix)?;
            if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
                return None;
            }
            Some((digits.parse().ok()?, digits.to_string(), e.path()))
        })
        .collect();
    out.sort();
    out
}

/// Creates the next free `<prefix>NNN` in `dir` after the numbers present.
fn create_numbered(dir: &Path, prefix: &str) -> Result<StepDir, ProjectError> {
    fs::create_dir_all(dir)
        .map_err(|e| ProjectError(text::io_error("Ordner nicht anlegbar", dir, &e)))?;
    let mut n = numbered(dir, prefix).last().map_or(0, |m| m.0) + 1;
    loop {
        let path = dir.join(format!("{prefix}{n:03}"));
        match fs::create_dir(&path) {
            Ok(()) => return Ok(StepDir::made(path)),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => n += 1,
            Err(e) => {
                return Err(ProjectError(text::io_error(
                    "Ordner nicht anlegbar",
                    &path,
                    &e,
                )))
            }
        }
    }
}

/// The surfaces in the map folder `dir`: `oberflaeche-NNN` by number, then other folders
/// `oberflaeche…` (made with `driftless surface -o`) by name; only those with their report
/// `oberflaeche.json`. The surface step writes it after the glb: a folder with the glb but
/// without the report was cut off between the two and is no finished surface (it is offered
/// for „Aufräumen“, [`crate::leftovers`]).
fn surfaces_of(dir: &Path) -> Vec<SurfaceInfo> {
    let numbered = numbered(dir, SURFACE_NUMBERED);
    let mut others: Vec<PathBuf> = fs::read_dir(dir)
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with(SURFACE_PREFIX))
                && !numbered.iter().any(|(_, _, d)| d == p)
        })
        .collect();
    others.sort();
    let labelled = numbered
        .into_iter()
        .map(|(_, digits, path)| (text::surface_label(&digits), path))
        .chain(others.into_iter().map(|p| {
            let name = p
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            (name, p)
        }));
    labelled
        .filter(|(_, p)| p.join(SURFACE_REPORT).is_file())
        .map(|(label, dir)| {
            let report = fs::read_to_string(dir.join(SURFACE_REPORT))
                .ok()
                .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok());
            let settings = report.as_ref().and_then(SurfaceSettings::from_report);
            SurfaceInfo {
                label,
                stale: false,
                settings,
                hints: report.as_ref().map(surface_hints).unwrap_or_default(),
                dir,
            }
        })
        .collect()
}

/// The hints for the user in the surface report `report` (`hinweise`): its texts, nothing
/// else; none without the field.
fn surface_hints(report: &serde_json::Value) -> Vec<String> {
    report["hinweise"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|h| h.as_str().map(String::from))
        .collect()
}

fn map_info(dir: &Path, label: String) -> MapInfo {
    let settings = fs::read_to_string(dir.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .and_then(|v| MapSwitches::from_map_json(&v));
    MapInfo {
        dir: dir.to_path_buf(),
        label,
        settings,
        clouds: MAP_CLOUDS
            .iter()
            .map(|f| dir.join(f))
            .filter(|p| p.is_file())
            .collect(),
        surfaces: surfaces_of(dir),
        incomplete: !dir.join(REPORT).is_file(),
    }
}

fn run_info(dir: &Path, label: String) -> RunInfo {
    let file = |name: &str| dir.join(name).is_file();
    let settings = fs::read_to_string(dir.join(RUN_FILE))
        .ok()
        .and_then(|t| toml::from_str::<RunFile>(&t).ok())
        .map(|r| r.slam)
        .unwrap_or_default();
    let history = history::list(dir);
    let mut maps = Vec::new();
    if file(REPORT) {
        maps.push(map_info(dir, "Karte (im Lauf-Ordner)".into()));
    }
    for (_, digits, path) in numbered(dir, MAP_PREFIX) {
        maps.push(map_info(&path, format!("Karte {digits}")));
    }
    RunInfo {
        dir: dir.to_path_buf(),
        label,
        settings,
        has_result: file("slam.json") && file("traj.tum"),
        incomplete: !file("slam.json"),
        has_cache: file(CACHE_FILE),
        has_previous: file("traj.vorher.tum"),
        has_odometry: file("traj_odom.tum"),
        cloud: file("map.las").then(|| dir.join("map.las")),
        maps,
        history,
    }
}

/// The start↔end gap after the optimisation of the state in folder `dir`, from its
/// `slam.json`; `None` if that cannot be read. Reads the whole file: call it off the UI thread.
pub fn state_gap(dir: &Path) -> Option<f64> {
    #[derive(Deserialize)]
    struct Gap {
        after: f64,
    }
    #[derive(Deserialize)]
    struct Report {
        start_end_gap: Gap,
    }
    let text = fs::read_to_string(dir.join("slam.json")).ok()?;
    let r: Report = serde_json::from_str(&text).ok()?;
    Some(r.start_end_gap.after)
}

/// The name of the run in folder `dir`: „SLAM 001“ for `slam-001`, else the folder name.
pub fn run_label(dir: &Path) -> String {
    let name = dir
        .file_name()
        .map_or_else(|| dir.display().to_string(), |n| n.to_string_lossy().into());
    match name.strip_prefix(RUN_PREFIX) {
        Some(d) if !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()) => format!("SLAM {d}"),
        _ => name,
    }
}

/// The project folder an old model (one without relative paths in `modell.json`) was merged
/// in: the folder its runs `<projekt>/scans/<id>/slam-NNN` lay in then, if every such run
/// says the same (runs elsewhere, added from the command line, say nothing).
fn merged_in(report: &ModelReport) -> Option<PathBuf> {
    let mut then: Option<&Path> = None;
    for m in &report.members {
        let run = &m.slam_dir;
        let numbered_run = run
            .file_name()
            .and_then(|n| n.to_str())
            .and_then(|n| n.strip_prefix(RUN_PREFIX))
            .is_some_and(|d| !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()));
        let scans = run.parent().and_then(Path::parent);
        let Some(project) = scans
            .filter(|s| numbered_run && s.file_name().is_some_and(|n| n == SCANS_DIR))
            .and_then(Path::parent)
        else {
            continue;
        };
        if then.is_some_and(|t| t != project) {
            return None;
        }
        then = Some(project);
    }
    then.map(Path::to_path_buf)
}

/// An old model ([`merged_in`]) of a project renamed, moved or copied since: every folder of
/// its scans that lay in the project folder then is taken in the project folder `now`, where
/// it exists there (else it stays as written). A model with relative paths needs none of
/// this ([`read_model`]).
fn rebase_old_model(report: &mut ModelReport, now: &Path) {
    let relative = report
        .members
        .iter()
        .any(|m| m.scan_rel.is_some() || m.slam_dir_rel.is_some() || m.map_dir_rel.is_some());
    let Some(then) = merged_in(report).filter(|then| !relative && then != now) else {
        return;
    };
    for m in &mut report.members {
        for p in [&mut m.scan, &mut m.slam_dir, &mut m.map_dir] {
            let moved = p.strip_prefix(&then).ok().map(|rest| now.join(rest));
            if let Some(found) = moved.filter(|f| f.is_dir()) {
                *p = found;
            }
        }
    }
}

impl Project {
    /// A new project in `dir` (created if missing; an existing folder must be empty apart
    /// from hidden files). `project.toml` is written right away.
    pub fn create(dir: &Path, name: &str) -> Result<Project, ProjectError> {
        if dir.join(PROJECT_FILE).exists() {
            return Err(ProjectError(text::about_path(
                "Hier ist schon ein Projekt – „Projekt öffnen …“ nehmen",
                dir,
            )));
        }
        if dir.exists() {
            let entries = fs::read_dir(dir)
                .map_err(|e| ProjectError(text::io_error("Ordner nicht lesbar", dir, &e)))?;
            let busy = entries
                .filter_map(Result::ok)
                .any(|e| !e.file_name().to_string_lossy().starts_with('.'));
            if busy {
                return Err(ProjectError(text::about_path(
                    "Ordner ist nicht leer – ein neues Projekt braucht einen leeren Ordner",
                    dir,
                )));
            }
        }
        fs::create_dir_all(dir)
            .map_err(|e| ProjectError(text::io_error("Ordner nicht anlegbar", dir, &e)))?;
        let project = Project {
            dir: canonical(dir)
                .map_err(|e| ProjectError(text::io_error("Ordner nicht lesbar", dir, &e)))?,
            name: name.to_string(),
            scans: Vec::new(),
        };
        project.save()?;
        Ok(project)
    }

    /// Reads `dir/project.toml`.
    pub fn open(dir: &Path) -> Result<Project, ProjectError> {
        let file = dir.join(PROJECT_FILE);
        let content = fs::read_to_string(&file).map_err(|e| {
            ProjectError(if e.kind() == io::ErrorKind::NotFound {
                text::about_path("Kein Projekt: project.toml fehlt", dir)
            } else {
                text::io_error("project.toml nicht lesbar", &file, &e)
            })
        })?;
        let bad = |what: &str| ProjectError(text::about_path(what, &file));
        let table: toml::Table = toml::from_str(&content)
            .map_err(|e| ProjectError(text::damaged_toml(&file, &content, &e)))?;
        match table.get("format").and_then(toml::Value::as_integer) {
            Some(FORMAT) => {}
            Some(n) => {
                return Err(bad(&format!(
                    "Projektformat {n} wird von diesem Programm nicht unterstützt (neueres \
                     Driftless nötig)"
                )))
            }
            None => return Err(bad("project.toml ist beschädigt: „format“ fehlt")),
        }
        let pf: ProjectFile = table.try_into().map_err(|e: toml::de::Error| {
            bad(&format!(
                "project.toml ist beschädigt: {}",
                e.message().lines().next().unwrap_or_default()
            ))
        })?;
        let dir = canonical(dir)
            .map_err(|e| ProjectError(text::io_error("Projektordner nicht lesbar", dir, &e)))?;
        let scans = pf
            .scans
            .into_iter()
            .map(|s| {
                // the relative paths of the runs count only one per run
                let runs_rel =
                    (s.extra_runs_rel.len() == s.extra_runs.len()).then_some(&s.extra_runs_rel);
                ScanEntry {
                    path: from_stored_pair(&dir, &s.path, s.path_rel.as_deref()),
                    extra_runs: s
                        .extra_runs
                        .iter()
                        .enumerate()
                        .map(|(k, r)| {
                            let rel = runs_rel.map(|v| v[k].as_str());
                            from_stored_pair(&dir, r, rel)
                        })
                        .collect(),
                    bags: s
                        .bags
                        .iter()
                        .filter_map(|b| {
                            let (name, bytes) = b.rsplit_once(' ')?;
                            Some((name.to_string(), bytes.parse().ok()?))
                        })
                        .collect(),
                    id: s.id,
                    name: s.name,
                }
            })
            .collect::<Vec<_>>();
        check_ids(&scans).map_err(|e| bad(&format!("project.toml ist beschädigt: {e}")))?;
        Ok(Project {
            dir,
            name: pf.name,
            scans,
        })
    }

    /// Writes `project.toml` (to a temporary file, then renamed).
    pub fn save(&self) -> Result<(), ProjectError> {
        check_ids(&self.scans)?;
        let scans = self
            .scans
            .iter()
            .map(|s| {
                let (path, path_rel) = to_stored(&self.dir, &s.path)?;
                let runs = s
                    .extra_runs
                    .iter()
                    .map(|r| to_stored(&self.dir, r))
                    .collect::<Result<Vec<_>, _>>()?;
                let extra_runs_rel = if runs.iter().any(|(_, rel)| rel.is_some()) {
                    runs.iter()
                        .map(|(_, rel)| rel.clone().unwrap_or_default())
                        .collect()
                } else {
                    Vec::new()
                };
                Ok(ScanFile {
                    id: s.id.clone(),
                    path,
                    path_rel,
                    name: s.name.clone(),
                    extra_runs: runs.into_iter().map(|(text, _)| text).collect(),
                    extra_runs_rel,
                    bags: s
                        .bags
                        .iter()
                        .map(|(name, bytes)| format!("{name} {bytes}"))
                        .collect(),
                })
            })
            .collect::<Result<_, ProjectError>>()?;
        let pf = ProjectFile {
            format: FORMAT,
            name: self.name.clone(),
            scans,
        };
        let file = self.dir.join(PROJECT_FILE);
        let content = toml::to_string(&pf)
            .map_err(|e| ProjectError(text::about_path(&e.to_string(), &file)))?;
        atomic_write(&file, content.as_bytes())
            .map_err(|e| ProjectError(text::io_error("project.toml nicht schreibbar", &file, &e)))
    }

    pub fn scan(&self, id: &str) -> Option<&ScanEntry> {
        self.scans.iter().find(|s| s.id == id)
    }

    fn unknown(id: &str) -> ProjectError {
        ProjectError(format!("Scan {id} gehört nicht zum Projekt"))
    }

    /// Adds a scan folder (it must have LiDAR bags); returns its id: the folder name, with
    /// `-2`, `-3`, … if taken (ignoring case) – by a scan of the project, or by the results of
    /// another scan under `scans/<id>` (one taken out of the project, see
    /// [`Project::results_of_another_scan`]). The same folder twice is refused. Call
    /// [`Project::save`] afterwards.
    pub fn add_scan(&mut self, scan_dir: &Path) -> Result<String, ProjectError> {
        let path = canonical(scan_dir)
            .map_err(|_| ProjectError(text::about_path("Scan-Ordner nicht gefunden", scan_dir)))?;
        let found = discover_project(&path).map_err(|e| ProjectError(e.to_string()))?;
        if let Some(s) = self.scans.iter().find(|s| same_dir(&s.path, &path)) {
            return Err(ProjectError(text::about_path(
                &format!("Scan ist schon im Projekt (als „{}“)", s.name),
                &path,
            )));
        }
        let base = path
            .file_name()
            .map_or_else(|| "scan".to_string(), |n| n.to_string_lossy().into_owned());
        check_id(&base).map_err(|e| ProjectError(format!("{e}; Ordner umbenennen")))?;
        let mut id = base.clone();
        let mut n = 1;
        let taken = |id: &str| {
            self.scans
                .iter()
                .any(|s| s.id.to_lowercase() == id.to_lowercase())
                || self.results_of_another_scan(id, &found)
        };
        while taken(&id) {
            n += 1;
            id = format!("{base}-{n}");
        }
        self.scans.push(ScanEntry {
            id: id.clone(),
            path,
            name: base,
            extra_runs: Vec::new(),
            bags: bag_stamps(&found),
        });
        Ok(id)
    }

    /// Whether `scans/<id>` holds the runs of another scan than the scan folder `found` (one
    /// taken out of the project): none of its runs was recorded at the times of `found`'s LiDAR
    /// bags ([`run_fits_scan`]), or one was recorded at others. Such an id is not given to
    /// `found`, which would list those runs and maps as its own; a folder of the same scan,
    /// taken out and added again, gives it its results back. A folder without runs holds
    /// nothing to take over. Reads the bags' indexes and the runs' `slam.json`.
    fn results_of_another_scan(&self, id: &str, found: &driftless_io::project::Project) -> bool {
        let Ok(base) = self.runs_dir(id) else {
            return false;
        };
        let runs = numbered(&base, RUN_PREFIX);
        if runs.is_empty() {
            return false;
        }
        let times = scan_times(&found.lidar_bags);
        let mut fits = false;
        for (_, _, run) in &runs {
            match (driftless_steps::slam::run_times(run), times) {
                (Some(run), Some(scan)) if run_fits_scan(run, scan) => fits = true,
                // recorded at other times: another scan's
                (Some(_), Some(_)) => return true,
                // cannot be told (no report, bags without an index)
                _ => {}
            }
        }
        !fits
    }

    /// Gives the scan `id` the shown name `name` (trimmed, not empty); its folder and id stay.
    /// Call [`Project::save`] afterwards.
    pub fn rename_scan(&mut self, id: &str, name: &str) -> Result<(), ProjectError> {
        let name = name.trim();
        if name.is_empty() {
            return Err(ProjectError(text::SCAN_NAME_EMPTY.into()));
        }
        let scan = self
            .scans
            .iter_mut()
            .find(|s| s.id == id)
            .ok_or_else(|| Self::unknown(id))?;
        scan.name = name.to_string();
        Ok(())
    }

    /// Takes the scan `id` out of the project: only its entry; the raw data and the results
    /// under `scans/<id>` stay (added again, the scan gets them back). Call [`Project::save`]
    /// afterwards.
    pub fn remove_scan(&mut self, id: &str) -> Result<ScanEntry, ProjectError> {
        let k = self
            .scans
            .iter()
            .position(|s| s.id == id)
            .ok_or_else(|| Self::unknown(id))?;
        Ok(self.scans.remove(k))
    }

    /// Takes the run folder `run` made by the CLI out of the scan `id` (only the entry; the
    /// folder stays). Call [`Project::save`] afterwards.
    pub fn remove_run(&mut self, id: &str, run: &Path) -> Result<(), ProjectError> {
        let scan = self
            .scans
            .iter_mut()
            .find(|s| s.id == id)
            .ok_or_else(|| Self::unknown(id))?;
        let before = scan.extra_runs.len();
        scan.extra_runs.retain(|r| !same_dir(r, run));
        if scan.extra_runs.len() == before {
            return Err(ProjectError(text::unknown_run(run)));
        }
        Ok(())
    }

    /// Points the scan `id` at the folder `dir` (its data moved: another drive, another
    /// computer), if that holds the same scan: the same LiDAR bags by name and size as when it
    /// was added, and recorded at the times of its runs ([`run_fits_scan`]). A different scan
    /// is refused with the reason. Returns whether it could be checked (a scan without runs,
    /// added before the bags were kept, cannot be). Reads the bags: call it off the UI thread.
    /// Call [`Project::save`] afterwards.
    pub fn relink_scan(&mut self, id: &str, dir: &Path) -> Result<bool, ProjectError> {
        let path = canonical(dir)
            .map_err(|_| ProjectError(text::about_path("Scan-Ordner nicht gefunden", dir)))?;
        let found = discover_project(&path).map_err(|e| ProjectError(e.to_string()))?;
        if let Some(other) = self
            .scans
            .iter()
            .find(|s| s.id != id && same_dir(&s.path, &path))
        {
            return Err(ProjectError(text::about_path(
                &format!("Ordner ist schon der Scan „{}“", other.name),
                &path,
            )));
        }
        let runs = self.runs(id);
        let scan = self.scan(id).ok_or_else(|| Self::unknown(id))?;
        let checked = same_scan(scan, &runs, &found)?;
        let scan = self
            .scans
            .iter_mut()
            .find(|s| s.id == id)
            .ok_or_else(|| Self::unknown(id))?;
        scan.path = path;
        if scan.bags.is_empty() {
            scan.bags = bag_stamps(&found);
        }
        Ok(checked)
    }

    /// Adds a run folder made by the CLI (`driftless slam -o`, with `slam.json`) to the scan
    /// `scan_id`; its frames must lie within the times of the scan. Call [`Project::save`]
    /// afterwards.
    pub fn add_run(&mut self, scan_id: &str, run_dir: &Path) -> Result<(), ProjectError> {
        let known: Vec<PathBuf> = self.runs(scan_id).into_iter().map(|r| r.dir).collect();
        let scan = self
            .scans
            .iter_mut()
            .find(|s| s.id == scan_id)
            .ok_or_else(|| Self::unknown(scan_id))?;
        let path = canonical(run_dir)
            .map_err(|_| ProjectError(text::about_path("Lauf-Ordner nicht gefunden", run_dir)))?;
        if !path.join("slam.json").is_file() {
            return Err(ProjectError(text::about_path(
                "Kein SLAM-Lauf: slam.json fehlt",
                &path,
            )));
        }
        if known.iter().any(|k| same_dir(k, &path)) {
            return Err(ProjectError(text::about_path(
                "Lauf ist schon im Projekt",
                &path,
            )));
        }
        check_run_scan(&path, &scan.path)?;
        scan.extra_runs.push(path);
        Ok(())
    }

    /// Whether the scan folder can be read.
    pub fn scan_state(&self, scan_id: &str) -> ScanState {
        let Some(scan) = self.scan(scan_id) else {
            return ScanState::Missing(Self::unknown(scan_id).0);
        };
        if !scan.path.is_dir() {
            return ScanState::Missing(text::about_path("Scan-Ordner nicht gefunden", &scan.path));
        }
        match discover_project(&scan.path) {
            Ok(_) => ScanState::Ready,
            Err(e) => ScanState::Missing(e.to_string()),
        }
    }

    /// Why the scan cannot be coloured (no `IMAGE_*.bag`, no `calibration/calib.json`), in the
    /// words of `driftless map`; `None` if it can, or if the scan cannot be read at all (see
    /// [`Project::scan_state`]). Only lists the folder.
    pub fn color_missing(&self, scan_id: &str) -> Option<String> {
        let scan = self.scan(scan_id)?;
        let found = discover_project(&scan.path).ok()?;
        if found.image_bags.is_empty() {
            return Some(format!(
                "keine Kamerabilder: {} enthält keine IMAGE_*.bag",
                scan.path.display()
            ));
        }
        if found.calibration.is_none() {
            return Some(format!(
                "keine Kalibrierung: {} fehlt",
                scan.path.join("calibration").join("calib.json").display()
            ));
        }
        None
    }

    /// `scans/<id>`, refused unless it is a folder directly in `scans/`.
    fn runs_dir(&self, scan_id: &str) -> Result<PathBuf, ProjectError> {
        check_id(scan_id)?;
        let scans = self.dir.join(SCANS_DIR);
        let dir = scans.join(scan_id);
        if dir.parent() != Some(scans.as_path()) {
            return Err(ProjectError(format!(
                "ungültige Scan-Kennung „{scan_id}“: {} liegt nicht in {}",
                dir.display(),
                scans.display()
            )));
        }
        Ok(dir)
    }

    /// The runs of a scan: `scans/<id>/slam-NNN` by number, then the added ones in the order
    /// they were added. Empty for an unknown scan or an invalid id.
    pub fn runs(&self, scan_id: &str) -> Vec<RunInfo> {
        let (Some(scan), Ok(base)) = (self.scan(scan_id), self.runs_dir(scan_id)) else {
            return Vec::new();
        };
        let own = numbered(&base, RUN_PREFIX)
            .into_iter()
            .map(|(_, digits, path)| run_info(&path, format!("SLAM {digits}")));
        let extra = scan.extra_runs.iter().map(|dir| {
            let name = dir
                .file_name()
                .map_or_else(|| dir.display().to_string(), |n| n.to_string_lossy().into());
            run_info(dir, name)
        });
        own.chain(extra).collect()
    }

    /// The models of the project: `modelle/modell-NNN` with a readable `modell.json`, by
    /// number (a folder without one is a merge still running, stopped or broken), with the
    /// folders of their scans as found ([`read_model`]; an old model of a project renamed,
    /// moved or copied since, see [`rebase_old_model`]). Reads each report; call it when the
    /// tree is read, not every frame.
    pub fn models(&self) -> Vec<ModelInfo> {
        numbered(&self.dir.join(MODELS_DIR), MODEL_PREFIX)
            .into_iter()
            .filter_map(|(_, digits, dir)| {
                let mut report = read_model(&dir).ok()?;
                rebase_old_model(&mut report, &self.dir);
                self.current_scans(&mut report);
                Some(ModelInfo {
                    label: text::model_label(&digits),
                    surfaces: surfaces_of(&dir),
                    report,
                    dir,
                })
            })
            .collect()
    }

    /// Each scan of the model `report` whose run is one of the project's (`scans/<id>/slam-NNN`,
    /// or a run added by hand) in the folder the project has for it now: `modell.json` keeps
    /// the scan folder of the merge, which is out of date once the scan was assigned a new
    /// folder („Ordner neu zuordnen …“). So the surface of the model takes the photos from where
    /// the scan is, and whether its maps changed is told from there.
    fn current_scans(&self, report: &mut ModelReport) {
        for m in &mut report.members {
            if let Some(scan) = self.scan_of_run(&m.slam_dir) {
                m.scan = scan.path.clone();
            }
        }
    }

    /// The scan of the project whose run lies in `run`: `<projekt>/scans/<id>/slam-NNN` of the
    /// scan `id`, or a run added to it by hand.
    fn scan_of_run(&self, run: &Path) -> Option<&ScanEntry> {
        let own = run
            .strip_prefix(self.dir.join(SCANS_DIR))
            .ok()
            .and_then(|rest| {
                let mut parts = rest.components();
                let (Some(Component::Normal(id)), Some(Component::Normal(name)), None) =
                    (parts.next(), parts.next(), parts.next())
                else {
                    return None;
                };
                let digits = name.to_str()?.strip_prefix(RUN_PREFIX)?;
                if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
                    return None;
                }
                self.scan(id.to_str()?)
            });
        own.or_else(|| {
            self.scans
                .iter()
                .find(|s| s.extra_runs.iter().any(|r| same_dir(r, run)))
        })
    }

    /// Creates the next `scans/<id>/slam-NNN` with its `lauf.toml`. For a job: the folder goes
    /// again when the guard is dropped without [`StepDir::keep`].
    pub fn new_run_dir(
        &self,
        scan_id: &str,
        settings: &SlamSettings,
    ) -> Result<StepDir, ProjectError> {
        let scan = self.scan(scan_id).ok_or_else(|| Self::unknown(scan_id))?;
        let dir = create_numbered(&self.runs_dir(scan_id)?, RUN_PREFIX)?;
        let (scan_text, scan_rel) = to_stored(&self.dir, &scan.path)?;
        let run = RunFile {
            scan: scan_text,
            scan_rel,
            erstellt: utc_now(),
            slam: settings.clone(),
        };
        let file = dir.path().join(RUN_FILE);
        let content = toml::to_string(&run)
            .map_err(|e| ProjectError(text::about_path(&e.to_string(), &file)))?;
        atomic_write(&file, content.as_bytes())
            .map_err(|e| ProjectError(text::io_error("lauf.toml nicht schreibbar", &file, &e)))?;
        Ok(dir)
    }
}

/// Hard link, or a copy where the file system has none. A stage file is only ever replaced
/// by a rename (`save_stage`), never written into, so a link never changes the other folder.
fn link_or_copy(from: &Path, to: &Path) -> io::Result<()> {
    fs::hard_link(from, to).or_else(|_| fs::copy(from, to).map(|_| ()))
}

/// The map folder for `input` in the run: an existing `karte-NNN` only if its map of `input`
/// is current already ([`current_map`]: computing it would write nothing), else a new
/// `karte-NNN` after the last ([`new_map_dir`]). A map is never computed anew in its own
/// folder: one that stopped half-way (cancelled, a full disk) would have lost its report, and
/// with it the surfaces in it ([`crate::leftovers`]); the old map stays as it is. The map in
/// the run folder itself is never written into. For a job: reads `traj.tum`, may copy large
/// files.
pub fn map_dir_for(
    run: &RunInfo,
    input: &MapInput,
    progress: &Progress,
) -> Result<StepDir, StepError> {
    if let Some(dir) = current_map_dir(run, input) {
        return Ok(StepDir::existing(dir));
    }
    new_map_dir(run, input, progress)
}

/// The youngest `karte-NNN` of `run` whose whole map of `input` is current ([`current_map`];
/// never for a single step, which always writes).
fn current_map_dir(run: &RunInfo, input: &MapInput) -> Option<PathBuf> {
    numbered(&run.dir, MAP_PREFIX)
        .into_iter()
        .rev()
        .map(|(_, _, dir)| dir)
        .find(|dir| {
            let req = MapRequest {
                input: input.clone(),
                out_dir: dir.clone(),
                all_files: false,
            };
            current_map(&req).is_some()
        })
}

/// A new `karte-NNN` after the last of `run`, into which the stages the step of `input` would
/// take over (`dense.bin` for clean, `clean.bin` or else `dense.bin` for color) are linked
/// from the youngest map folder (or else the run folder) that has one that fits.
fn new_map_dir(run: &RunInfo, input: &MapInput, progress: &Progress) -> Result<StepDir, StepError> {
    let maps = numbered(&run.dir, MAP_PREFIX);
    let target = create_numbered(&run.dir, MAP_PREFIX)?;
    let wanted: &[&str] = match input.map.step {
        MapStep::Dense | MapStep::All => &[],
        MapStep::Clean => &[DENSE_BIN],
        MapStep::Color => &[CLEAN_BIN, DENSE_BIN],
    };
    if !wanted.is_empty() {
        // youngest first, up to the first that has a fitting stage; the run folder holds a
        // map made by the CLI
        let sources = maps
            .iter()
            .rev()
            .map(|m| m.2.clone())
            .chain([run.dir.clone()]);
        for src in sources {
            let fits = fitting_stages(input, &src);
            // with the cleaned cloud, the dense one is not needed
            let Some(file) = wanted.iter().find(|f| fits.contains(f)) else {
                continue;
            };
            progress.phase("Zwischenstände übernehmen", None);
            link_or_copy(&src.join(file), &target.path().join(file)).map_err(|e| {
                text::io_error("Zwischenstand nicht kopierbar", &src.join(file), &e)
            })?;
            let from = src.file_name().unwrap_or_default().to_string_lossy();
            progress.info(format!("{file} aus {from} übernommen"));
            break;
        }
        if progress.is_cancelled() {
            return Err(StepError::Cancelled);
        }
    }
    Ok(target)
}

/// The [`MapStatus`] of `map` of `run`, made from the scan folder `scan`. Reads `map.json`,
/// `traj.tum` and the list of bags, so call it when the tree is read, not every frame. A map
/// whose scan cannot be read is never stale (that cannot be told).
pub fn map_status(scan: &Path, run: &RunInfo, map: &MapInfo) -> MapStatus {
    let seconds = fs::read_to_string(map.dir.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| v["seconds_total"].as_f64());
    let stale = map.settings.as_ref().is_some_and(|switches| {
        let input = MapInput {
            scan: scan.to_path_buf(),
            slam_dir: run.dir.clone(),
            slam: run.settings.clone(),
            map: MapSettings {
                step: MapStep::All,
                switches: switches.clone(),
            },
        };
        discover_project(scan).is_ok() && !folder_fits(&input, &map.dir)
    });
    MapStatus { seconds, stale }
}

/// What a SLAM job computes.
#[derive(Clone, Debug, PartialEq)]
pub enum SlamJob {
    /// A full run into a new `slam-NNN`.
    Full(SlamSettings),
    /// Recompute this run from its `submaps.bin` (with its settings), in its folder.
    Reuse(RunInfo),
}

#[derive(Clone, Debug)]
pub struct SlamDone {
    pub run_dir: PathBuf,
    pub outcome: SlamOutcome,
}

#[derive(Clone, Debug)]
pub struct MapDone {
    pub dir: PathBuf,
    /// The `map.json` written.
    pub report: serde_json::Value,
}

/// Runs SLAM for the scan `scan_id`, with history. A full run always gets a new folder, which
/// is removed again if the run is cancelled or fails. A reuse of a run whose result no state
/// holds (e.g. one made or recomputed by the CLI) first keeps it as state „übernommen“
/// (`run_slam`); one without `lauf.toml` whose cache needs other than the default settings
/// is refused with a message saying so.
pub fn run_slam_job(
    project: &Project,
    scan_id: &str,
    job: &SlamJob,
    progress: &Progress,
) -> Result<SlamDone, StepError> {
    let scan = project
        .scan(scan_id)
        .ok_or_else(|| Project::unknown(scan_id))?
        .path
        .clone();
    match job {
        SlamJob::Full(settings) => {
            let dir = project.new_run_dir(scan_id, settings)?;
            let req = SlamRequest {
                scan,
                out: dir.path().to_path_buf(),
                settings: settings.clone(),
                reuse: false,
                history: true,
            };
            let outcome = run_slam(&req, progress)?;
            Ok(SlamDone {
                run_dir: dir.keep(),
                outcome,
            })
        }
        SlamJob::Reuse(run) => {
            // in place: another window must not offer or delete the folder meanwhile
            let _busy = mark_busy(&run.dir);
            reuse(scan, run, progress)
        }
    }
}

/// Recomputes `run` of the scan folder `scan` from its `submaps.bin`, in its folder, with
/// history; a failure or a cancel leaves it as it was ([`run_slam`]).
fn reuse(scan: PathBuf, run: &RunInfo, progress: &Progress) -> Result<SlamDone, StepError> {
    let req = SlamRequest {
        scan,
        out: run.dir.clone(),
        settings: run.settings.clone(),
        reuse: true,
        history: true,
    };
    reuse_refusal(&req)?;
    let outcome = run_slam(&req, progress)?;
    Ok(SlamDone {
        run_dir: run.dir.clone(),
        outcome,
    })
}

/// Why a reuse of a run cannot be computed here: an added CLI run has no `lauf.toml`, its
/// settings are only assumed, and its cache needs others.
fn reuse_refusal(req: &SlamRequest) -> Result<(), StepError> {
    if settings_differ(req) {
        return Err(text::about_path(
            "Nicht neu rechenbar: der Lauf wurde mit anderen SLAM-Einstellungen als den \
             Standardwerten gerechnet (LiDAR-Lage oder Topics), die nicht bei ihm stehen; den \
             Scan mit diesen Einstellungen neu rechnen (Scan wählen, „Erweitert“, \
             SLAM-Einstellungen, „SLAM rechnen“)",
            &req.out,
        )
        .into());
    }
    Ok(())
}

/// Why a map of `run` cannot be made here: as [`reuse_refusal`], the odometry of the map
/// would use the assumed default settings, not the ones the run was computed with.
fn map_refusal(run: &RunInfo, scan: &Path) -> Result<(), StepError> {
    let req = SlamRequest {
        scan: scan.to_path_buf(),
        out: run.dir.clone(),
        settings: run.settings.clone(),
        reuse: true,
        history: false,
    };
    if settings_differ(&req) {
        return Err(text::about_path(
            "Karte nicht rechenbar: der Lauf wurde mit anderen SLAM-Einstellungen als den \
             Standardwerten gerechnet (LiDAR-Lage oder Topics), die nicht bei ihm stehen; den \
             Scan mit diesen Einstellungen neu rechnen (Scan wählen, „Erweitert“, \
             SLAM-Einstellungen, „SLAM rechnen“)",
            &run.dir,
        )
        .into());
    }
    Ok(())
}

/// A run without `lauf.toml` (its settings only assumed) whose `submaps.bin` was written with
/// other settings. Without a cache this cannot be told, and the run counts as fitting.
fn settings_differ(req: &SlamRequest) -> bool {
    let assumed = !req.out.join(RUN_FILE).exists();
    assumed && req.out.join(CACHE_FILE).is_file() && !cache_fits(req)
}

/// Restores the switches of the corrections of `run` as in its state `nr`, then recomputes it
/// like [`SlamJob::Reuse`] (a new state; the old ones stay). Checked first whether the run can
/// be recomputed here; if the recompute fails or is cancelled, `corrections.toml` is written
/// back as it was, so the file always fits `traj.tum`.
pub fn restore_job(
    project: &Project,
    scan_id: &str,
    run: &RunInfo,
    nr: u32,
    progress: &Progress,
) -> Result<SlamDone, StepError> {
    let scan = project
        .scan(scan_id)
        .ok_or_else(|| Project::unknown(scan_id))?
        .path
        .clone();
    reuse_refusal(&SlamRequest {
        scan: scan.clone(),
        out: run.dir.clone(),
        settings: run.settings.clone(),
        reuse: true,
        history: true,
    })?;
    // in place: another window must not offer or delete the folder meanwhile
    let _busy = mark_busy(&run.dir);
    let path = run.dir.join(CORRECTIONS_FILE);
    let before = match fs::read(&path) {
        Ok(bytes) => Some(bytes),
        Err(e) if e.kind() == io::ErrorKind::NotFound => None,
        Err(e) => return Err(text::io_error("Korrekturen nicht lesbar", &path, &e).into()),
    };
    history::restore_flags(&run.dir, nr)?;
    let done = reuse(scan, run, progress);
    if done.is_err() {
        let _lock = lock(&path)?;
        let back = match &before {
            Some(bytes) => atomic_write(&path, bytes),
            None if path.exists() => fs::remove_file(&path),
            None => Ok(()),
        };
        if let Err(e) = back {
            return Err(text::io_error(
                "Neu rechnen fehlgeschlagen und Korrekturen nicht zurückgeschrieben",
                &path,
                &e,
            )
            .into());
        }
    }
    done
}

/// Makes a map of `run` from the scan folder `scan` into the folder [`map_dir_for`] picks: a
/// map that is current already is taken as it is (nothing written), any other is computed
/// into a new folder, which is removed again if the map is cancelled or fails. Refused like a
/// reuse for a run without `lauf.toml` whose cache needs other than the default settings.
pub fn run_map_job(
    run: &RunInfo,
    scan: &Path,
    settings: &MapSettings,
    progress: &Progress,
) -> Result<MapDone, StepError> {
    settings.check()?;
    map_refusal(run, scan)?;
    let input = MapInput {
        scan: scan.to_path_buf(),
        slam_dir: run.dir.clone(),
        slam: run.settings.clone(),
        map: settings.clone(),
    };
    let request = |dir: &Path| MapRequest {
        input: input.clone(),
        out_dir: dir.to_path_buf(),
        // one cloud to look at; the rest comes from „Exportieren …“
        all_files: false,
    };
    let mut dir = map_dir_for(run, &input, progress)?;
    if !dir.is_new() {
        if let Some(report) = take_current_map(&request(dir.path()), progress) {
            return Ok(MapDone {
                dir: dir.keep(),
                report,
            });
        }
        // changed since it was picked (another window or program): never in place
        dir = new_map_dir(run, &input, progress)?;
    }
    let report = recompute_map(&request(dir.path()), progress)?;
    Ok(MapDone {
        dir: dir.keep(),
        report,
    })
}

/// Creates the next `oberflaeche-NNN` in the map folder `map_dir`. For a job: the folder goes
/// again when the guard is dropped without [`StepDir::keep`].
pub fn new_surface_dir(map_dir: &Path) -> Result<StepDir, ProjectError> {
    create_numbered(map_dir, SURFACE_NUMBERED)
}

/// The request of a surface of `map` of `run` with `settings` into `out_dir`, computed on
/// `gpu`: the map as it was made (its switches from `map.json`), the run with its settings.
fn surface_request(
    scan: &Path,
    run: &RunInfo,
    map: &MapInfo,
    settings: &SurfaceSettings,
    gpu: GpuPref,
    out_dir: &Path,
) -> Result<SurfaceRequest, StepError> {
    let switches = map.settings.clone().ok_or(text::SURFACE_NEEDS_MAP)?;
    Ok(SurfaceRequest {
        map: MapInput {
            scan: scan.to_path_buf(),
            slam_dir: run.dir.clone(),
            slam: run.settings.clone(),
            map: MapSettings {
                step: MapStep::All,
                switches,
            },
        },
        map_dir: map.dir.clone(),
        out_dir: out_dir.to_path_buf(),
        config: settings.config.clone(),
        texture: settings.texture,
        gpu,
        // the glb; an OBJ comes from „Exportieren …“
        obj: false,
    })
}

/// The status of `surface` of `map` of `run`, made from the scan folder `scan`: its runtime
/// (`seconds_total` of `oberflaeche.json`), and stale if it no longer fits the map
/// ([`surface_fits`]: the map, the run, the images or the surface step changed since). A report
/// whose settings cannot be read in full was written by an earlier version of the program and
/// is stale too, and so is a folder without a report (the step writes it last: it was
/// stopped before, or the folder was copied incompletely). Reads the lists of bags, the
/// calibration, `traj.tum` and the head of `clean.bin`: call it off the UI thread. A surface
/// with a report whose scan cannot be read is never stale (that cannot be told).
pub fn surface_status(
    scan: &Path,
    run: &RunInfo,
    map: &MapInfo,
    surface: &SurfaceInfo,
) -> MapStatus {
    let report = fs::read_to_string(surface.dir.join(SURFACE_REPORT))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok());
    let seconds = report.as_ref().and_then(|v| v["seconds_total"].as_f64());
    let stale = report.is_none()
        || (discover_project(scan).is_ok()
            && surface.settings.as_ref().is_none_or(|settings| {
                surface_request(scan, run, map, settings, GpuPref::default(), &surface.dir)
                    .is_ok_and(|req| !surface_fits(&req))
            }));
    MapStatus { seconds, stale }
}

#[derive(Clone, Debug)]
pub struct SurfaceDone {
    pub dir: PathBuf,
    /// The `oberflaeche.json` of the surface.
    pub report: serde_json::Value,
    /// A surface of the map already fitted the request: nothing was computed.
    pub reused: bool,
}

/// Makes a surface of `map` of `run` (from the scan folder `scan`) with `settings`, as
/// `driftless surface` does: if an `oberflaeche-NNN` of the map already fits, that one is
/// kept and nothing is computed; else into a new `oberflaeche-NNN`, which is removed again if
/// the step is cancelled or fails. Refused like a map for a run without `lauf.toml` whose
/// cache needs other than the default settings, and for a map without `map.json`.
pub fn run_surface_job(
    run: &RunInfo,
    scan: &Path,
    map: &MapInfo,
    settings: &SurfaceSettings,
    gpu: GpuPref,
    progress: &Progress,
) -> Result<SurfaceDone, StepError> {
    settings.config.check()?;
    map_refusal(run, scan)?;
    for (_, _, dir) in numbered(&map.dir, SURFACE_NUMBERED).iter().rev() {
        let req = surface_request(scan, run, map, settings, gpu, dir)?;
        if surface_fits(&req) {
            progress.info(text::surface_fits(dir));
            let report = fs::read_to_string(dir.join(SURFACE_REPORT))
                .ok()
                .and_then(|t| serde_json::from_str(&t).ok())
                .unwrap_or_default();
            return Ok(SurfaceDone {
                dir: dir.clone(),
                report,
                reused: true,
            });
        }
        if progress.is_cancelled() {
            return Err(StepError::Cancelled);
        }
    }
    let dir = new_surface_dir(&map.dir)?;
    let req = surface_request(scan, run, map, settings, gpu, dir.path())?;
    let report = run_surface(&req, progress)?;
    Ok(SurfaceDone {
        dir: dir.keep(),
        report,
        reused: false,
    })
}

/// The members of a merge of `picks`: fewer than two, or a scan, run or map not of the
/// project, are refused.
fn merge_members(project: &Project, picks: &[MergePick]) -> Result<Vec<MergeMember>, StepError> {
    if picks.len() < 2 {
        return Err(text::MERGE_NEEDS_TWO.into());
    }
    let mut members = Vec::with_capacity(picks.len());
    for pick in picks {
        let scan = project
            .scan(&pick.scan_id)
            .ok_or_else(|| Project::unknown(&pick.scan_id))?;
        let run = project
            .runs(&pick.scan_id)
            .into_iter()
            .find(|r| same_dir(&r.dir, &pick.run))
            .ok_or_else(|| text::unknown_run(&pick.run))?;
        if !run.maps.iter().any(|m| same_dir(&m.dir, &pick.map)) {
            return Err(text::unknown_map(&pick.map).into());
        }
        members.push(MergeMember {
            scan: scan.path.clone(),
            slam_dir: run.dir.clone(),
            map_dir: pick.map.clone(),
            slam: run.settings,
        });
    }
    Ok(members)
}

/// Merges the scans `picks` (the first is the reference) into a new `modelle/modell-NNN`, as
/// `driftless merge` does with its defaults; `include_unsure` takes unsure scans in as well.
/// Each run is taken with the settings of its `lauf.toml`, and the switched-on point pairs of
/// `modelle/punktpaare.toml` that name two of the maps replace their automatic alignment.
/// Fewer than two scans, a scan, run or map not of the project, or a pair file that cannot be
/// read are refused before a folder is made; the new folder is removed again if the merge is
/// cancelled or fails. Returns the model folder.
pub fn run_merge_job(
    project: &Project,
    picks: &[MergePick],
    include_unsure: bool,
    progress: &Progress,
) -> Result<PathBuf, StepError> {
    let members = merge_members(project, picks)?;
    let models = project.dir.join(MODELS_DIR);
    let pairs = PointPairs::load(&point_pairs_file(project))?.resolved(&models);
    let dir = create_numbered(&models, MODEL_PREFIX)?;
    let req = MergeRequest {
        members,
        out_dir: dir.path().to_path_buf(),
        align: AlignConfig::default(),
        voxel: MODEL_VOXEL,
        smooth: true,
        include_unsure,
        pairs,
        ply: false,
    };
    run_merge(&req, progress)?;
    Ok(dir.keep())
}

/// `modelle/punktpaare.toml` of the project.
pub fn point_pairs_file(project: &Project) -> PathBuf {
    project.dir.join(MODELS_DIR).join(POINT_PAIRS_FILE)
}

/// The point pairs of the project in file order, `a` and `b` as resolved folders (where they
/// exist), so that they compare equal to [`driftless_steps::merge::ModelMember::map_dir`].
/// No file means none. Reads the file: call it when the tree is read, not every frame.
pub fn point_pairs(project: &Project) -> Result<Vec<PointPairEntry>, String> {
    let models = project.dir.join(MODELS_DIR);
    let pairs = PointPairs::load(&point_pairs_file(project)).map_err(|e| e.to_string())?;
    let resolve = |p: &Path| canonical(p).unwrap_or_else(|_| p.to_path_buf());
    Ok(pairs
        .resolved(&models)
        .into_iter()
        .map(|e| PointPairEntry {
            a: resolve(&e.a),
            b: resolve(&e.b),
            ..e
        })
        .collect())
}

/// Switches entry `index` (0-based) of `modelle/punktpaare.toml` on or off and saves the
/// file; the entry itself stays. Counts from the next merge on.
pub fn set_point_pair_enabled(
    project: &Project,
    index: usize,
    enabled: bool,
) -> Result<(), StepError> {
    let file = point_pairs_file(project);
    let _lock = lock(&file)?;
    let mut pairs = PointPairs::load(&file)?;
    pairs.set_enabled(index, enabled)?;
    pairs.save(&file)
}

/// The scans of `report` as picks of the project, in its order (the first is the reference),
/// each with the run and map it was merged from. Fails for a scan no longer in the project.
pub fn model_picks(project: &Project, report: &ModelReport) -> Result<Vec<MergePick>, StepError> {
    report
        .members
        .iter()
        .map(|m| {
            let scan = project
                .scans
                .iter()
                .find(|s| same_dir(&s.path, &m.scan))
                .ok_or_else(|| text::scan_not_in_project(&m.scan))?;
            Ok(MergePick {
                scan_id: scan.id.clone(),
                run: m.slam_dir.clone(),
                map: m.map_dir.clone(),
            })
        })
        .collect()
}

/// „Übernehmen und neu zusammenführen“: adds `entry` (map folders absolute, points in the
/// frame of each scan) to `modelle/punktpaare.toml`, its folders relative to that file and
/// with the time it was added, then merges the scans of `model` again as they were merged
/// ([`model_picks`], unsure ones taken in if the model was merged so) into a new `modell-NNN`
/// ([`run_merge_job`]). An entry that cannot be used ([`PointPairs::add`]) adds nothing and
/// makes no model; the entry stays saved if the merge then fails or is cancelled (it can be
/// switched off).
pub fn run_pairs_merge_job(
    project: &Project,
    model: &ModelInfo,
    entry: PointPairEntry,
    progress: &Progress,
) -> Result<PathBuf, StepError> {
    let picks = model_picks(project, &model.report)?;
    merge_members(project, &picks)?;
    let include_unsure = model.report.include_unsure;
    let file = point_pairs_file(project);
    let models = project.dir.join(MODELS_DIR);
    fs::create_dir_all(&models)
        .map_err(|e| ProjectError(text::io_error("Ordner nicht anlegbar", &models, &e)))?;
    let lock = lock(&file)?;
    let mut pairs = PointPairs::load(&file)?;
    pairs.add(PointPairEntry {
        a: PathBuf::from(stored_text(&models, &entry.a)?),
        b: PathBuf::from(stored_text(&models, &entry.b)?),
        added: utc_now(),
        ..entry
    })?;
    if progress.is_cancelled() {
        return Err(StepError::Cancelled);
    }
    pairs.save(&file)?;
    drop(lock);
    progress.info(text::PAIRS_SAVED.to_string());
    run_merge_job(project, &picks, include_unsure, progress)
}

/// The request of a surface of `model` with `settings` into `out_dir`, computed on `gpu`.
fn model_surface_request(
    model: &ModelInfo,
    settings: &SurfaceSettings,
    gpu: GpuPref,
    out_dir: &Path,
) -> ModelSurfaceRequest {
    ModelSurfaceRequest {
        model_dir: model.dir.clone(),
        // the scans' folders as the project found them
        model: Some(model.report.clone()),
        out_dir: out_dir.to_path_buf(),
        config: settings.config.clone(),
        texture: settings.texture,
        gpu,
        obj: false,
    }
}

/// Whether a scan folder of `model` cannot be read (an unplugged drive): then nothing of the
/// model can be told stale.
fn model_scan_missing(model: &ModelInfo) -> bool {
    model
        .report
        .members
        .iter()
        .filter(|m| m.included)
        .any(|m| discover_project(&m.scan).is_err())
}

/// The status of `model` itself: its runtime, and stale if it was merged by another version
/// of the merge ([`ModelInfo::other_version`]) or the map of a scan in it was computed anew
/// since the merge (the place of the scan in the model may not fit it any more: merge anew).
/// Reads `traj.tum` and the bag lists of its scans: call it off the UI thread. Whether a map
/// changed cannot be told while a scan of the model cannot be read: then only the version
/// counts.
pub fn model_status(model: &ModelInfo) -> MapStatus {
    let changed = |m: &driftless_steps::merge::ModelMember| {
        let Some(id) = &m.clean_id else {
            return false;
        };
        let input = map_input_for(&m.scan, &m.slam_dir, &m.map_dir, m.slam.clone());
        clean_stage_id(&input).map_or(true, |now| hex(&now) != *id)
    };
    let stale = model.other_version()
        || (!model_scan_missing(model)
            && model
                .report
                .members
                .iter()
                .filter(|m| m.included)
                .any(changed));
    MapStatus {
        seconds: Some(model.report.seconds_total),
        stale,
    }
}

/// The status of `surface` of `model`, as [`surface_status`] for a map: stale if it no
/// longer fits the model ([`model_surface_fits`]), without a readable report, with settings
/// an earlier program wrote, or of a model of another merge version. Call it off the UI
/// thread.
pub fn model_surface_status(model: &ModelInfo, surface: &SurfaceInfo) -> MapStatus {
    let report = fs::read_to_string(surface.dir.join(SURFACE_REPORT))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok());
    let seconds = report.as_ref().and_then(|v| v["seconds_total"].as_f64());
    let stale = report.is_none()
        || model.other_version()
        || (!model_scan_missing(model)
            && surface.settings.as_ref().is_none_or(|settings| {
                let req = model_surface_request(model, settings, GpuPref::default(), &surface.dir);
                !model_surface_fits(&req)
            }));
    MapStatus { seconds, stale }
}

/// Makes a surface of `model` with `settings` from the points and images of all its scans,
/// as `driftless surface --model` does: if an `oberflaeche-NNN` of the model already fits,
/// that one is kept and nothing is computed; else into a new `oberflaeche-NNN`, removed
/// again if the step is cancelled or fails.
pub fn run_model_surface_job(
    model: &ModelInfo,
    settings: &SurfaceSettings,
    gpu: GpuPref,
    progress: &Progress,
) -> Result<SurfaceDone, StepError> {
    settings.config.check()?;
    for (_, _, dir) in numbered(&model.dir, SURFACE_NUMBERED).iter().rev() {
        if model_surface_fits(&model_surface_request(model, settings, gpu, dir)) {
            progress.info(text::surface_fits(dir));
            let report = fs::read_to_string(dir.join(SURFACE_REPORT))
                .ok()
                .and_then(|t| serde_json::from_str(&t).ok())
                .unwrap_or_default();
            return Ok(SurfaceDone {
                dir: dir.clone(),
                report,
                reused: true,
            });
        }
        if progress.is_cancelled() {
            return Err(StepError::Cancelled);
        }
    }
    let dir = new_surface_dir(&model.dir)?;
    let req = model_surface_request(model, settings, gpu, dir.path());
    let report = run_model_surface(&req, progress)?;
    Ok(SurfaceDone {
        dir: dir.keep(),
        report,
        reused: false,
    })
}

// Unix paths: `/p` is not absolute on Windows.
#[cfg(all(test, unix))]
mod tests {
    use super::*;

    #[test]
    fn stored_paths() {
        let base = Path::new("/p");
        let stored = |p: &str| to_stored(base, Path::new(p)).unwrap();
        assert_eq!(stored("/p/a/b"), ("a/b".into(), None));
        // outside: absolute, and relative beside it
        assert_eq!(stored("/q/a"), ("/q/a".into(), Some("../q/a".into())));
        assert_eq!(stored("/p"), ("/p".into(), None));
        assert_eq!(from_stored(base, "a/b"), Path::new("/p/a/b"));
        assert_eq!(from_stored(base, "a\\b"), Path::new("/p/a/b"));
        assert_eq!(from_stored(base, "./a//b/"), Path::new("/p/a/b"));
        assert_eq!(from_stored(base, "/q/a"), Path::new("/q/a"));
    }

    #[test]
    fn surface_settings_from_old_and_new_reports() {
        let c = SurfaceConfig::default();
        let mut report = serde_json::json!({"settings": {
            "texture": true,
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
        }});
        // Before the coarse grid: no key, read as on, so it matches the defaults.
        assert_eq!(
            SurfaceSettings::from_report(&report),
            Some(SurfaceSettings::default())
        );
        report["settings"]["coarse_fallback"] = false.into();
        let off = SurfaceSettings::from_report(&report).unwrap();
        assert!(!off.config.coarse_fallback);
        report["settings"]["coarse_fallback"] = "ja".into();
        assert_eq!(SurfaceSettings::from_report(&report), None);
        // Before the fill filter: no key, the default; a stored value is kept.
        report["settings"]["coarse_fallback"] = true.into();
        report["settings"]["max_fill_gap"] = 0.03.into();
        let wide = SurfaceSettings::from_report(&report).unwrap();
        assert_eq!(wide.config.max_fill_gap, 0.03);
        report["settings"]["max_fill_gap"] = "weit".into();
        assert_eq!(SurfaceSettings::from_report(&report), None);
    }
}
