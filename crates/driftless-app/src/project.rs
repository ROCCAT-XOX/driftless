//! A Driftless project on disk:
//!
//! ```text
//! <projekt>/project.toml
//! <projekt>/scans/<scan-id>/slam-001/          a SLAM run, the same folder as `driftless slam -o`
//!                          slam-001/lauf.toml  scan and settings of the run
//!                          slam-001/verlauf/   states of the run (driftless_steps::history)
//!                          slam-001/karte-001/ a map of the run, one folder per set of switches
//!                karte-001/oberflaeche-001/    a surface of the map (`driftless surface -o`)
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
//! ```
//!
//! Relative paths are written with `/` and read with `/` or `\`, so a project works on macOS
//! and Windows alike. `lauf.toml` holds `scan` (stored the same way), `erstellt` (UTC) and the
//! [`SlamSettings`] under `[slam]`; `erstellt` is the only value that differs between two equal
//! runs (like `erstellt`/`sekunden` in `stand.toml` and the runtimes in `map.json`).
//!
//! A scan folder that is gone (an unplugged drive) is reported by [`Project::scan_state`];
//! the project still opens and its runs are still listed.
//!
//! Folders for results are only created by the jobs ([`run_slam_job`], [`run_map_job`]), never
//! on the UI thread, and a job removes a folder it created when it is cancelled or fails. A
//! full SLAM run always gets a new `slam-NNN` (so no earlier `traj.tum` is replaced), and a map
//! is written only into a `karte-NNN` whose `map.json` fits the request, or into a new one. A
//! surface is only computed if no `oberflaeche-NNN` of the map fits the request, and then into a
//! new one ([`run_surface_job`]).

use crate::surface::SURFACE_GLB;
use crate::ui::text;
use driftless_io::atomic_write;
use driftless_io::bag::BagReader;
use driftless_io::progress::Progress;
use driftless_io::project::discover_project;
use driftless_map::surface::SurfaceConfig;
use driftless_steps::correct::utc_now;
use driftless_steps::history::{self, HistoryEntry};
use driftless_steps::map::{
    fitting_stages, folder_fits, run_map, MapInput, MapRequest, MapSettings, MapStep, MapSwitches,
    CLEAN_BIN, DENSE_BIN, REPORT,
};
use driftless_steps::slam::{
    cache_fits, run_slam, SlamOutcome, SlamRequest, SlamSettings, CACHE_FILE, CORRECTIONS_FILE,
};
use driftless_steps::surface::{run_surface, surface_fits, SurfaceRequest, SURFACE_REPORT};
use driftless_steps::StepError;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};

pub const PROJECT_FILE: &str = "project.toml";
pub const RUN_FILE: &str = "lauf.toml";
pub const SCANS_DIR: &str = "scans";
const FORMAT: i64 = 1;
const RUN_PREFIX: &str = "slam-";
const MAP_PREFIX: &str = "karte-";
/// Surfaces of a map: `oberflaeche-NNN` made here, other `oberflaeche…` folders by the CLI.
const SURFACE_PREFIX: &str = "oberflaeche";
const SURFACE_NUMBERED: &str = "oberflaeche-";
/// The clouds of a map folder that can be shown, best first.
const MAP_CLOUDS: [&str; 4] = ["color.las", "clean.las", "dense.las", "removed.las"];
/// How far (s) a run's frames may lie outside the bag times of its scan: frame stamps come
/// from the sensor, bag times from the recorder.
const RUN_TIME_TOLERANCE: f64 = 5.0;

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
    /// The surfaces of the map: its subfolders `oberflaeche…` with an `oberflaeche.glb`.
    pub surfaces: Vec<SurfaceInfo>,
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
    /// `driftless surface` always use. `None` if a value is missing or of the wrong type.
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
/// fails.
#[must_use]
#[derive(Debug)]
pub struct StepDir {
    path: PathBuf,
    new: bool,
}

impl StepDir {
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Created for this job (else an existing folder that is never removed).
    pub fn is_new(&self) -> bool {
        self.new
    }

    /// Keeps the folder; returns its path.
    pub fn keep(mut self) -> PathBuf {
        self.new = false;
        std::mem::take(&mut self.path)
    }
}

impl Drop for StepDir {
    fn drop(&mut self) {
        if self.new {
            let _ = fs::remove_dir_all(&self.path);
        }
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
    path: String,
    name: String,
    #[serde(default)]
    extra_runs: Vec<String>,
}

#[derive(Serialize, Deserialize)]
struct RunFile {
    scan: String,
    erstellt: String,
    #[serde(default)]
    slam: SlamSettings,
}

/// The absolute, resolved form of an existing path (without the `\\?\` that Windows adds to
/// a plain drive path). Paths in a [`Project`] are always in this form.
pub fn canonical(path: &Path) -> io::Result<PathBuf> {
    let p = fs::canonicalize(path)?;
    if cfg!(windows) {
        if let Some(rest) = p.to_str().and_then(|s| s.strip_prefix(r"\\?\")) {
            if rest.as_bytes().get(1) == Some(&b':') {
                return Ok(PathBuf::from(rest));
            }
        }
    }
    Ok(p)
}

/// `path` as written to a project file: relative to `base` with `/` if inside it, else
/// absolute.
fn to_stored(base: &Path, path: &Path) -> Result<String, ProjectError> {
    let text = match path.strip_prefix(base) {
        Ok(rel) if !rel.as_os_str().is_empty() => rel
            .components()
            .map(|c| c.as_os_str().to_str())
            .collect::<Option<Vec<_>>>()
            .map(|parts| parts.join("/")),
        _ => path.to_str().map(str::to_string),
    };
    text.ok_or_else(|| ProjectError(format!("Pfad {} ist kein gültiger Text", path.display())))
}

/// Absolute on Windows: `C:\…`, `C:/…` or `\\server\…` (also read on another system, where
/// such a path is just not found instead of being taken as relative).
fn windows_absolute(text: &str) -> bool {
    let b = text.as_bytes();
    let drive =
        b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && matches!(b[2], b'/' | b'\\');
    drive || text.starts_with("\\\\") || text.starts_with("//")
}

/// A path from a project file: absolute as written, relative (with `/` or `\`) to `base`.
fn from_stored(base: &Path, text: &str) -> PathBuf {
    let p = Path::new(text);
    if p.is_absolute() || windows_absolute(text) {
        return p.to_path_buf();
    }
    let mut out = base.to_path_buf();
    for part in text
        .split(['/', '\\'])
        .filter(|s| !s.is_empty() && *s != ".")
    {
        out.push(part);
    }
    out
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
        .ok_or_else(|| ProjectError(format!("{} nicht lesbar", file.display())))?;
    let start = report["scan_start"]
        .as_f64()
        .ok_or_else(|| ProjectError(format!("{}: scan_start fehlt", file.display())))?;
    let end = start
        + report["submaps"]
            .as_array()
            .and_then(|m| m.last())
            .and_then(|m| m["t_end"].as_f64())
            .unwrap_or(0.0);
    let project =
        discover_project(scan).map_err(|e| ProjectError(format!("Lauf nicht prüfbar: {e}")))?;
    let mut range: Option<(f64, f64)> = None;
    for bag in &project.lidar_bags {
        let Some((a, b)) = BagReader::open(bag).ok().and_then(|mut r| r.time_range()) else {
            return Ok(());
        };
        range = Some(range.map_or((a, b), |(lo, hi)| (lo.min(a), hi.max(b))));
    }
    let Some((lo, hi)) = range else {
        return Ok(());
    };
    if start >= lo - RUN_TIME_TOLERANCE && end <= hi + RUN_TIME_TOLERANCE {
        Ok(())
    } else {
        Err(ProjectError(format!(
            "Lauf {} gehört nicht zum Scan {}: er umfasst {start:.1}–{end:.1} s, der Scan \
             {lo:.1}–{hi:.1} s",
            run.display(),
            scan.display()
        )))
    }
}

/// Two paths name the same folder (resolved where they exist).
fn same_dir(a: &Path, b: &Path) -> bool {
    let resolve = |p: &Path| canonical(p).unwrap_or_else(|_| p.to_path_buf());
    resolve(a) == resolve(b)
}

/// Subfolders `<prefix><digits>` of `dir` as (number, digits, path), by number.
fn numbered(dir: &Path, prefix: &str) -> Vec<(u64, String, PathBuf)> {
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
        .map_err(|e| ProjectError(format!("{} nicht anlegbar: {e}", dir.display())))?;
    let mut n = numbered(dir, prefix).last().map_or(0, |m| m.0) + 1;
    loop {
        let path = dir.join(format!("{prefix}{n:03}"));
        match fs::create_dir(&path) {
            Ok(()) => return Ok(StepDir { path, new: true }),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => n += 1,
            Err(e) => {
                return Err(ProjectError(format!(
                    "{} nicht anlegbar: {e}",
                    path.display()
                )))
            }
        }
    }
}

/// The surfaces in the map folder `dir`: `oberflaeche-NNN` by number, then other folders
/// `oberflaeche…` (made with `driftless surface -o`) by name; only those with a glb.
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
        .filter(|(_, p)| p.join(SURFACE_GLB).is_file())
        .map(|(label, dir)| {
            let settings = fs::read_to_string(dir.join(SURFACE_REPORT))
                .ok()
                .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
                .and_then(|v| SurfaceSettings::from_report(&v));
            SurfaceInfo {
                label,
                stale: false,
                settings,
                dir,
            }
        })
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

impl Project {
    /// A new project in `dir` (created if missing; an existing folder must be empty apart
    /// from hidden files). `project.toml` is written right away.
    pub fn create(dir: &Path, name: &str) -> Result<Project, ProjectError> {
        if dir.join(PROJECT_FILE).exists() {
            return Err(ProjectError(format!(
                "Projekt existiert schon: {}",
                dir.join(PROJECT_FILE).display()
            )));
        }
        if dir.exists() {
            let entries = fs::read_dir(dir)
                .map_err(|e| ProjectError(format!("{} nicht lesbar: {e}", dir.display())))?;
            let busy = entries
                .filter_map(Result::ok)
                .any(|e| !e.file_name().to_string_lossy().starts_with('.'));
            if busy {
                return Err(ProjectError(format!(
                    "Ordner {} ist nicht leer; ein neues Projekt braucht einen leeren Ordner",
                    dir.display()
                )));
            }
        }
        fs::create_dir_all(dir)
            .map_err(|e| ProjectError(format!("{} nicht anlegbar: {e}", dir.display())))?;
        let project = Project {
            dir: canonical(dir)
                .map_err(|e| ProjectError(format!("{} nicht lesbar: {e}", dir.display())))?,
            name: name.to_string(),
            scans: Vec::new(),
        };
        project.save()?;
        Ok(project)
    }

    /// Reads `dir/project.toml`.
    pub fn open(dir: &Path) -> Result<Project, ProjectError> {
        let file = dir.join(PROJECT_FILE);
        let text = fs::read_to_string(&file).map_err(|e| {
            ProjectError(if e.kind() == io::ErrorKind::NotFound {
                format!("Kein Projekt: {} fehlt", file.display())
            } else {
                format!("{} nicht lesbar: {e}", file.display())
            })
        })?;
        let bad = |e: &dyn fmt::Display| ProjectError(format!("{}: {e}", file.display()));
        let table: toml::Table = toml::from_str(&text).map_err(|e| bad(&e))?;
        match table.get("format").and_then(toml::Value::as_integer) {
            Some(FORMAT) => {}
            Some(n) => {
                return Err(bad(&format!(
                    "Projektformat {n} wird von diesem Programm nicht unterstützt"
                )))
            }
            None => return Err(bad(&"`format` fehlt")),
        }
        let pf: ProjectFile = table.try_into().map_err(|e| bad(&e))?;
        let dir = canonical(dir).map_err(|e| bad(&e))?;
        let scans = pf
            .scans
            .into_iter()
            .map(|s| ScanEntry {
                path: from_stored(&dir, &s.path),
                extra_runs: s.extra_runs.iter().map(|r| from_stored(&dir, r)).collect(),
                id: s.id,
                name: s.name,
            })
            .collect::<Vec<_>>();
        check_ids(&scans).map_err(|e| bad(&e))?;
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
                Ok(ScanFile {
                    id: s.id.clone(),
                    path: to_stored(&self.dir, &s.path)?,
                    name: s.name.clone(),
                    extra_runs: s
                        .extra_runs
                        .iter()
                        .map(|r| to_stored(&self.dir, r))
                        .collect::<Result<_, _>>()?,
                })
            })
            .collect::<Result<_, ProjectError>>()?;
        let pf = ProjectFile {
            format: FORMAT,
            name: self.name.clone(),
            scans,
        };
        let file = self.dir.join(PROJECT_FILE);
        let text =
            toml::to_string(&pf).map_err(|e| ProjectError(format!("{}: {e}", file.display())))?;
        atomic_write(&file, text.as_bytes())
            .map_err(|e| ProjectError(format!("{} nicht schreibbar: {e}", file.display())))
    }

    pub fn scan(&self, id: &str) -> Option<&ScanEntry> {
        self.scans.iter().find(|s| s.id == id)
    }

    fn unknown(id: &str) -> ProjectError {
        ProjectError(format!("Scan {id} gehört nicht zum Projekt"))
    }

    /// Adds a scan folder (it must have LiDAR bags); returns its id: the folder name, with
    /// `-2`, `-3`, … if taken (ignoring case). The same folder twice is refused. Call [`Project::save`]
    /// afterwards.
    pub fn add_scan(&mut self, scan_dir: &Path) -> Result<String, ProjectError> {
        let path = canonical(scan_dir).map_err(|_| {
            ProjectError(format!("Scan-Ordner {} nicht gefunden", scan_dir.display()))
        })?;
        discover_project(&path).map_err(|e| ProjectError(e.to_string()))?;
        if let Some(s) = self.scans.iter().find(|s| same_dir(&s.path, &path)) {
            return Err(ProjectError(format!(
                "Scan {} ist schon im Projekt (als {})",
                path.display(),
                s.id
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
        });
        Ok(id)
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
        let path = canonical(run_dir).map_err(|_| {
            ProjectError(format!("Lauf-Ordner {} nicht gefunden", run_dir.display()))
        })?;
        if !path.join("slam.json").is_file() {
            return Err(ProjectError(format!(
                "{} ist kein SLAM-Lauf: slam.json fehlt",
                path.display()
            )));
        }
        if known.iter().any(|k| same_dir(k, &path)) {
            return Err(ProjectError(format!(
                "Lauf {} ist schon im Projekt",
                path.display()
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
            return ScanState::Missing(format!(
                "Scan-Ordner {} nicht gefunden",
                scan.path.display()
            ));
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

    /// Creates the next `scans/<id>/slam-NNN` with its `lauf.toml`. For a job: the folder goes
    /// again when the guard is dropped without [`StepDir::keep`].
    pub fn new_run_dir(
        &self,
        scan_id: &str,
        settings: &SlamSettings,
    ) -> Result<StepDir, ProjectError> {
        let scan = self.scan(scan_id).ok_or_else(|| Self::unknown(scan_id))?;
        let dir = create_numbered(&self.runs_dir(scan_id)?, RUN_PREFIX)?;
        let run = RunFile {
            scan: to_stored(&self.dir, &scan.path)?,
            erstellt: utc_now(),
            slam: settings.clone(),
        };
        let file = dir.path().join(RUN_FILE);
        let text =
            toml::to_string(&run).map_err(|e| ProjectError(format!("{}: {e}", file.display())))?;
        atomic_write(&file, text.as_bytes())
            .map_err(|e| ProjectError(format!("{} nicht schreibbar: {e}", file.display())))?;
        Ok(dir)
    }
}

/// Hard link, or a copy where the file system has none. A stage file is only ever replaced
/// by a rename (`save_stage`), never written into, so a link never changes the other folder.
fn link_or_copy(from: &Path, to: &Path) -> io::Result<()> {
    fs::hard_link(from, to).or_else(|_| fs::copy(from, to).map(|_| ()))
}

/// The map folder for `input` in the run: the youngest `karte-NNN` that fits it
/// ([`folder_fits`]), else a new `karte-NNN` after the last. Into a new folder the stages
/// the step would take over (`dense.bin` for clean, `clean.bin` or else `dense.bin` for
/// color) are linked from the youngest map folder (or else the run folder) that has one
/// that fits.
/// The map in the run folder itself is never written into. For a job: reads `traj.tum`,
/// may copy large files.
pub fn map_dir_for(
    run: &RunInfo,
    input: &MapInput,
    progress: &Progress,
) -> Result<StepDir, StepError> {
    let maps = numbered(&run.dir, MAP_PREFIX);
    if let Some((_, _, dir)) = maps.iter().rev().find(|(_, _, d)| folder_fits(input, d)) {
        return Ok(StepDir {
            path: dir.clone(),
            new: false,
        });
    }
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
                format!(
                    "{} nicht nach {} kopierbar: {e}",
                    src.join(file).display(),
                    target.path().display()
                )
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
    }
}

/// Why a reuse of a run cannot be computed here: an added CLI run has no `lauf.toml`, its
/// settings are only assumed, and its cache needs others.
fn reuse_refusal(req: &SlamRequest) -> Result<(), StepError> {
    if settings_differ(req) {
        return Err(format!(
            "Lauf wurde mit anderen Einstellungen gerechnet als den Standardwerten (z. B. \
             --lidar-to-imu, --lidar-topic oder --imu-topic) und lässt sich hier nicht neu \
             rechnen; mit `driftless slam <scan> -o {} --reuse` und denselben Angaben neu rechnen",
            req.out.display()
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
        return Err(format!(
            "Lauf wurde mit anderen Einstellungen gerechnet als den Standardwerten (z. B. \
             --lidar-to-imu, --lidar-topic oder --imu-topic); die Karte lässt sich hier nicht \
             rechnen; mit `driftless map <scan> --slam {}` und denselben Angaben rechnen",
            run.dir.display()
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
        scan,
        out: run.dir.clone(),
        settings: run.settings.clone(),
        reuse: true,
        history: true,
    })?;
    let path = run.dir.join(CORRECTIONS_FILE);
    let before = match fs::read(&path) {
        Ok(bytes) => Some(bytes),
        Err(e) if e.kind() == io::ErrorKind::NotFound => None,
        Err(e) => return Err(format!("{} nicht lesbar: {e}", path.display()).into()),
    };
    history::restore_flags(&run.dir, nr)?;
    let done = run_slam_job(project, scan_id, &SlamJob::Reuse(run.clone()), progress);
    if done.is_err() {
        let back = match &before {
            Some(bytes) => atomic_write(&path, bytes),
            None if path.exists() => fs::remove_file(&path),
            None => Ok(()),
        };
        if let Err(e) = back {
            return Err(format!(
                "Neu rechnen fehlgeschlagen und {} nicht zurückgeschrieben: {e}",
                path.display()
            )
            .into());
        }
    }
    done
}

/// Makes a map of `run` from the scan folder `scan` into the folder [`map_dir_for`] picks; a
/// new folder is removed again if the map is cancelled or fails. Refused like a reuse for a
/// run without `lauf.toml` whose cache needs other than the default settings.
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
    let dir = map_dir_for(run, &input, progress)?;
    let req = MapRequest {
        input,
        out_dir: dir.path().to_path_buf(),
    };
    let report = run_map(&req, progress)?;
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

/// The request of a surface of `map` of `run` with `settings` into `out_dir`: the map as it
/// was made (its switches from `map.json`), the run with its settings.
fn surface_request(
    scan: &Path,
    run: &RunInfo,
    map: &MapInfo,
    settings: &SurfaceSettings,
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
    })
}

/// The status of `surface` of `map` of `run`, made from the scan folder `scan`: its runtime
/// (`seconds_total` of `oberflaeche.json`), and stale if it no longer fits the map
/// ([`surface_fits`]: the map, the run, the images or the program changed since). A report
/// whose settings cannot be read in full was written by an earlier version of the program and
/// is stale too. Reads the lists of bags, the calibration, `traj.tum` and the head of
/// `clean.bin`: call it off the UI thread. A surface without a report, or whose scan cannot
/// be read, is never stale (that cannot be told).
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
    let stale = report.is_some()
        && discover_project(scan).is_ok()
        && surface.settings.as_ref().is_none_or(|settings| {
            surface_request(scan, run, map, settings, &surface.dir)
                .is_ok_and(|req| !surface_fits(&req))
        });
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
    progress: &Progress,
) -> Result<SurfaceDone, StepError> {
    settings.config.check()?;
    map_refusal(run, scan)?;
    for (_, _, dir) in numbered(&map.dir, SURFACE_NUMBERED).iter().rev() {
        let req = surface_request(scan, run, map, settings, dir)?;
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
    let req = surface_request(scan, run, map, settings, dir.path())?;
    let report = run_surface(&req, progress)?;
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
        assert_eq!(to_stored(base, Path::new("/p/a/b")).unwrap(), "a/b");
        assert_eq!(to_stored(base, Path::new("/q/a")).unwrap(), "/q/a");
        assert_eq!(from_stored(base, "a/b"), Path::new("/p/a/b"));
        assert_eq!(from_stored(base, "a\\b"), Path::new("/p/a/b"));
        assert_eq!(from_stored(base, "./a//b/"), Path::new("/p/a/b"));
        assert_eq!(from_stored(base, "/q/a"), Path::new("/q/a"));
    }
}
