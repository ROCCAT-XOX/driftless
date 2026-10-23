//! The window: the panels show an [`AppState`] and hand back what was asked for as an
//! [`Action`]; only [`DriftlessApp`] carries actions out (dialogs, project changes, jobs).
//! So the panels can be tested without a window (`egui_kittest`).

pub mod actions;
mod app;
pub mod compare;
pub mod correct;
pub mod left;
pub mod right;
pub mod status;
pub mod text;
pub mod view3d;

pub use app::{parse_args, Args, DriftlessApp};

use crate::camera::CameraMode;
use crate::jobs::JobStatus;
use crate::project::{map_status, state_gap, MapStatus, Project, RunInfo, ScanState};
use crate::render::{ColorMode, ShowMode};
use driftless_io::progress::LogLine;
use driftless_steps::inspect;
use driftless_steps::map::MapSettings;
use driftless_steps::slam::SlamSettings;
use std::collections::BTreeMap;
use std::path::PathBuf;

/// What is selected in the project tree.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum Selection {
    #[default]
    None,
    /// A scan, by id.
    Scan(String),
    /// A run folder.
    Run(PathBuf),
    /// A map folder.
    Map(PathBuf),
    /// A LAS file.
    Cloud(PathBuf),
    /// A state in the history of a run.
    History { run: PathBuf, nr: u32 },
}

/// What a panel asks for. `Pick…` open a dialog first.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    PickNewProject,
    PickOpenProject,
    OpenProject(PathBuf),
    NewProject {
        dir: PathBuf,
        name: String,
    },
    PickAddScan,
    AddScan(PathBuf),
    /// Add a run folder made by the CLI to the scan (id).
    PickAddRun(String),
    Select(Selection),
    LoadCloud {
        path: PathBuf,
        run: Option<PathBuf>,
    },
    /// Choose a surface (`oberflaeche.glb`) to show.
    PickOpenSurface,
    /// Show the surface of a `.glb`, or of the `oberflaeche.glb` in a folder.
    LoadSurface(PathBuf),
    /// Check a scan (id).
    Inspect(String),
    /// A new SLAM run of the scan (id).
    RunSlam(String),
    /// Recompute the run from its submaps.
    Reuse(PathBuf),
    /// A map of the run.
    RunMap(PathBuf),
    Cancel,
    /// Clears the messages of the status bar.
    DismissMessages,
    AddLoop {
        run: PathBuf,
        a: f64,
        b: f64,
        note: String,
    },
    SetLoopEnabled {
        run: PathBuf,
        nr: usize,
        enabled: bool,
    },
    /// Switch the corrections of the run back as in state `nr`, then recompute it (a new
    /// state; the old ones stay).
    RestoreHistory {
        run: PathBuf,
        nr: u32,
    },
    /// Compare trajectory `a` (aligned to `b`) with `b`.
    Compare {
        a: compare::TrajRef,
        b: compare::TrajRef,
    },
    /// Put the `map.json` of two map folders side by side.
    CompareMaps {
        a: PathBuf,
        b: PathBuf,
    },
}

/// A message for the status bar (German).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UiMessage {
    Info(String),
    Error(String),
}

/// How the 3D view draws.
#[derive(Debug, Clone, PartialEq)]
pub struct ViewSettings {
    pub color_mode: ColorMode,
    /// Point diameter in points (logical pixels).
    pub point_px: f32,
    pub round: bool,
    /// Most points drawn per frame.
    pub budget: usize,
    pub camera_mode: CameraMode,
    pub show_trajectory: bool,
    pub show_odometry: bool,
    /// Accepted and manual loops.
    pub show_loops: bool,
    /// The rejected loop candidates (faint).
    pub show_rejected: bool,
    pub show_submaps: bool,
    /// The trajectory A of the comparison (orange).
    pub show_compare: bool,
    /// Height cut: lowest and highest z drawn (relative to the cloud's origin); `None`
    /// draws everything. Reset when another cloud is shown.
    pub cut: Option<[f32; 2]>,
    /// Surface, points or both, while a surface is loaded.
    pub show: ShowMode,
}

impl Default for ViewSettings {
    fn default() -> Self {
        ViewSettings {
            color_mode: ColorMode::Rgb,
            point_px: 2.0,
            round: true,
            budget: 6_000_000,
            camera_mode: CameraMode::Orbit,
            show_trajectory: true,
            show_odometry: false,
            show_loops: true,
            show_rejected: false,
            show_submaps: false,
            show_compare: true,
            cut: None,
            show: ShowMode::Surface,
        }
    }
}

/// The loaded cloud, as the panels need it.
#[derive(Debug, Clone, PartialEq)]
pub struct CloudSummary {
    pub path: PathBuf,
    pub points: usize,
    pub has_rgb: bool,
    /// Whether the cloud has submap qualities (a SLAM report).
    pub quality_known: bool,
    /// Lowest and highest relative z.
    pub z_range: [f32; 2],
    /// A height just below the ceiling, if one was found.
    pub ceiling: Option<f32>,
}

/// The loaded surface, as the panels need it.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSummary {
    pub path: PathBuf,
    pub triangles: usize,
    pub pages: usize,
    /// The surface's origin (whole metres).
    pub origin: [f64; 3],
    /// Lowest and highest z relative to `origin` (the height cut while no cloud is shown).
    pub z_range: [f32; 2],
}

/// The last frame of the 3D view.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewStats {
    pub drawn: usize,
    pub total: usize,
    pub frame_ms: f32,
    /// `frame_ms` is GPU time (else time between frames).
    pub gpu: bool,
}

/// A scan as the tree shows it. Read from disk by [`AppState::refresh`], not every frame.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanNode {
    pub id: String,
    pub name: String,
    pub state: ScanState,
    /// Why the scan cannot be coloured ([`Project::color_missing`]); `None` if it can.
    pub no_color: Option<String>,
    pub runs: Vec<RunInfo>,
}

/// The last check of a scan.
#[derive(Debug, Clone, PartialEq)]
pub struct InspectResult {
    /// Scan id.
    pub scan: String,
    pub report: inspect::Report,
    pub seconds: f64,
}

/// Everything the panels show. Made without a GPU or window.
#[derive(Debug, Clone, Default)]
pub struct AppState {
    pub project: Option<Project>,
    /// The scans of [`AppState::project`] with their runs, as last read from disk.
    pub scans: Vec<ScanNode>,
    pub selection: Selection,
    pub map_settings: MapSettings,
    pub slam_settings: SlamSettings,
    pub view: ViewSettings,
    /// Oldest first; the status bar shows the last.
    pub messages: Vec<UiMessage>,
    /// The running step, as of this frame.
    pub job: Option<JobStatus>,
    /// Messages of the steps, oldest first.
    pub log: Vec<LogLine>,
    /// The step action that is running (set by [`actions::start_step`]).
    pub step: Option<Action>,
    /// The last check of a scan, shown under „Eigenschaften“ while that scan is selected.
    pub inspect: Option<InspectResult>,
    /// The map folders of the tree whose status is known. Filled in the background (see
    /// [`AppState::map_status_job`]), so a map may be missing here for a moment.
    pub map_status: BTreeMap<PathBuf, MapStatus>,
    /// Raised by every [`AppState::refresh`]: the program recomputes [`AppState::map_status`]
    /// in the background whenever it changed.
    pub tree_version: u64,
    /// The start↔end gap of each history state of the tree, by state folder; read in the
    /// background like [`AppState::map_status`].
    pub history_gaps: BTreeMap<PathBuf, f64>,
    /// The cloud in the 3D view.
    pub cloud: Option<CloudSummary>,
    /// The surface in the 3D view.
    pub surface: Option<SurfaceSummary>,
    /// A cloud being loaded, as of this frame (the loader runs next to the steps).
    pub loading: Option<JobStatus>,
    /// The last frame of the 3D view.
    pub view_stats: Option<ViewStats>,
    /// The point clicked in the 3D view.
    pub picked: Option<view3d::PointInfo>,
    /// The manual corrections of the run the view draws.
    pub correct: correct::CorrectState,
    /// The comparison of two trajectories and of two maps.
    pub compare: compare::CompareState,
}

impl AppState {
    /// Shows `project` from now on: nothing selected, tree read from disk.
    pub fn set_project(&mut self, project: Project) {
        self.project = Some(project);
        self.selection = Selection::None;
        self.inspect = None;
        self.refresh();
    }

    /// Reads the scans' state and runs again (after opening, adding, and after each step).
    pub fn refresh(&mut self) {
        self.scans = self.project.as_ref().map_or_else(Vec::new, |p| {
            p.scans
                .iter()
                .map(|s| ScanNode {
                    id: s.id.clone(),
                    name: s.name.clone(),
                    state: p.scan_state(&s.id),
                    no_color: p.color_missing(&s.id),
                    runs: p.runs(&s.id),
                })
                .collect()
        });
        // Statuses of maps still in the tree stay shown until the background job replaces them.
        let maps: std::collections::BTreeSet<&PathBuf> = self
            .scans
            .iter()
            .flat_map(|s| &s.runs)
            .flat_map(|r| &r.maps)
            .map(|m| &m.dir)
            .collect();
        self.map_status.retain(|dir, _| maps.contains(dir));
        self.tree_version += 1;
    }

    /// The work of computing [`AppState::map_status`] for the current tree, to run off the
    /// UI thread: it reads `traj.tum` and the bag list of every map ([`map_status`]).
    pub fn map_status_job(&self) -> impl FnOnce() -> BTreeMap<PathBuf, MapStatus> + Send + 'static {
        let mut maps = Vec::new();
        if let Some(project) = &self.project {
            for node in &self.scans {
                let Some(scan) = project.scan(&node.id) else {
                    continue;
                };
                for run in &node.runs {
                    for map in &run.maps {
                        maps.push((scan.path.clone(), run.clone(), map.clone()));
                    }
                }
            }
        }
        move || {
            maps.into_iter()
                .map(|(scan, run, map)| (map.dir.clone(), map_status(&scan, &run, &map)))
                .collect()
        }
    }

    /// The work of reading the gaps of the history states not known yet
    /// ([`AppState::history_gaps`]), to run off the UI thread.
    pub fn history_gaps_job(&self) -> impl FnOnce() -> BTreeMap<PathBuf, f64> + Send + 'static {
        let dirs: Vec<PathBuf> = self
            .scans
            .iter()
            .flat_map(|s| &s.runs)
            .flat_map(|r| &r.history)
            .map(|e| e.dir.clone())
            .filter(|d| !self.history_gaps.contains_key(d))
            .collect();
        move || {
            dirs.into_iter()
                .filter_map(|d| state_gap(&d).map(|g| (d, g)))
                .collect()
        }
    }

    /// The scan with this id.
    pub fn scan_node(&self, id: &str) -> Option<&ScanNode> {
        self.scans.iter().find(|s| s.id == id)
    }

    /// The run in folder `dir`, with its scan.
    pub fn run(&self, dir: &std::path::Path) -> Option<(&ScanNode, &RunInfo)> {
        self.scans
            .iter()
            .find_map(|s| s.runs.iter().find(|r| r.dir == dir).map(|r| (s, r)))
    }

    /// The run the selection belongs to: the run itself, or the run of a selected map,
    /// cloud or state.
    pub fn selected_run(&self) -> Option<(&ScanNode, &RunInfo)> {
        let owns = |r: &RunInfo| match &self.selection {
            Selection::Run(d) => r.dir == *d,
            Selection::Map(d) => r.maps.iter().any(|m| m.dir == *d),
            Selection::Cloud(c) => {
                r.cloud.as_ref() == Some(c) || r.maps.iter().any(|m| m.clouds.contains(c))
            }
            Selection::History { run, .. } => r.dir == *run,
            Selection::None | Selection::Scan(_) => false,
        };
        self.scans
            .iter()
            .find_map(|s| s.runs.iter().find(|r| owns(r)).map(|r| (s, r)))
    }
}
