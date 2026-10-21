//! The window: the panels show an [`AppState`] and hand back what was asked for as an
//! [`Action`]; only [`DriftlessApp`] carries actions out (dialogs, project changes, jobs).
//! So the panels can be tested without a window (`egui_kittest`).

mod app;
pub mod left;
pub mod status;
pub mod text;

pub use app::{parse_args, DriftlessApp};

use crate::camera::CameraMode;
use crate::jobs::JobStatus;
use crate::project::{Project, RunInfo, ScanState};
use crate::render::ColorMode;
use driftless_io::progress::LogLine;
use driftless_steps::map::MapSettings;
use driftless_steps::slam::SlamSettings;
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
    RestoreHistory {
        run: PathBuf,
        nr: u32,
    },
}

/// A message for the status bar (German).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UiMessage {
    Info(String),
    Error(String),
}

/// How the 3D view draws (used from the 3D view on).
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
    pub show_loops: bool,
    pub show_submaps: bool,
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
            show_submaps: false,
        }
    }
}

/// A scan as the tree shows it. Read from disk by [`AppState::refresh`], not every frame.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanNode {
    pub id: String,
    pub name: String,
    pub state: ScanState,
    pub runs: Vec<RunInfo>,
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
}

impl AppState {
    /// Shows `project` from now on: nothing selected, tree read from disk.
    pub fn set_project(&mut self, project: Project) {
        self.project = Some(project);
        self.selection = Selection::None;
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
                    runs: p.runs(&s.id),
                })
                .collect()
        });
    }
}
