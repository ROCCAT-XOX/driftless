//! The window: the panels show an [`AppState`] and hand back what was asked for as an
//! [`Action`]; only [`DriftlessApp`] carries actions out (dialogs, project changes, jobs).
//! So the panels can be tested without a window (`egui_kittest`).
//!
//! Layout: the rail at the left edge ([`rail`]: ☰, the three [`Step`]s, ⚙), next to it the
//! field of the active step ([`steps`]), the 3D view ([`view3d`]) in the rest with its floating
//! bars ([`tools`]: info, the switch Punkte · Fläche · beides, the tools, the cut and the bar of
//! the brush, [`brush`]), the status bar ([`status`]) at the bottom, the settings window ⚙
//! ([`settings`]), the window „Exportieren“ ([`export`]), the window „Über Driftless“
//! ([`about`]), and „Melden“ (⇧⇧) with the list of the reports ([`feedback`]).

pub mod about;
pub mod actions;
mod app;
pub mod brush;
pub mod compare;
pub mod correct;
pub mod export;
pub mod feedback;
pub mod manage;
pub mod rail;
pub mod settings;
pub mod status;
pub mod steps;
pub mod text;
pub mod theme;
pub mod tools;
pub mod view3d;

pub use crate::leftovers::Leftover;
pub use app::{
    parse_args, Args, DialogFolder, DriftlessApp, LastFolders, FOCUS_DELAY, FOCUS_GAP, UI_KEY,
};

use crate::camera::CameraMode;
use crate::jobs::JobStatus;
use crate::project::{
    map_status, model_status, model_surface_status, point_pairs, state_gap, surface_status,
    MapStatus, MergePick, ModelInfo, Project, ProjectError, RunInfo, ScanState, SurfaceSettings,
};
use crate::render::{ColorMode, PointSize, ShowMode};
use driftless_gpu::GpuPref;
use driftless_steps::export::{ExportRequest, ExportSource};
use driftless_steps::inspect;
use driftless_steps::map::MapSettings;
use driftless_steps::marks::{DabInput, SmoothLevel};
use driftless_steps::pairs::PointPairEntry;
use driftless_steps::slam::SlamSettings;
use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};

/// The steps of the work, in the order they are done; the rail switches between them.
#[derive(
    Clone, Copy, PartialEq, Eq, Hash, Debug, Default, serde::Serialize, serde::Deserialize,
)]
pub enum Step {
    /// Scans, maps, corrections: the view shows the cloud.
    #[default]
    Daten,
    /// Surfaces of the maps: the view shows the surface.
    Bilder,
    /// Floor plan and measures (comes with Baustein 5).
    Planung,
}

/// The settings window ⚙.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SettingsWindow {
    pub open: bool,
}

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
    /// A surface folder of a map.
    Surface(PathBuf),
    /// A state in the history of a run.
    History { run: PathBuf, nr: u32 },
    /// A model folder (`modelle/modell-NNN`).
    Model(PathBuf),
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
    /// Choose a file to show: a cloud (LAS, LAZ, E57, PLY, PCD; without a run) or a surface
    /// (glb, OBJ), told apart by the extension.
    PickOpenFile,
    /// Show the surface of a `.glb` or `.obj`, or of the `oberflaeche.glb` in a folder; the
    /// one shown is loaded again only if its file changed since.
    LoadSurface(PathBuf),
    /// [`Action::LoadSurface`], also when the surface is shown and unchanged (picked again).
    ReloadSurface(PathBuf),
    /// Check a scan (id).
    Inspect(String),
    /// A new SLAM run of the scan (id).
    RunSlam(String),
    /// Recompute the run from its submaps.
    Reuse(PathBuf),
    /// A map of the run.
    RunMap(PathBuf),
    /// A surface of the map (folder).
    RunSurface(PathBuf),
    /// Open the field „Scans zusammenführen“ in „Daten“.
    OpenMerge,
    /// Merge the scans into a new model; the first pick is the reference.
    RunMerge {
        picks: Vec<MergePick>,
        include_unsure: bool,
    },
    /// A surface of the model (folder) from the images of all its scans.
    RunModelSurface {
        model: PathBuf,
    },
    /// „Von Hand ausrichten“ of scan `member` (from 0) of the model (folder): shows the
    /// model and that scan side by side and opens the tool „Punktpaare“.
    StartPairs {
        model: PathBuf,
        member: usize,
    },
    /// Saves `entry` in `modelle/punktpaare.toml` and merges the scans of the model (folder)
    /// again into a new model ([`crate::project::run_pairs_merge_job`]).
    ApplyPairs {
        model: PathBuf,
        entry: PointPairEntry,
    },
    /// Switches entry `index` (from 0) of `modelle/punktpaare.toml` on or off.
    SetPairEnabled {
        index: usize,
        enabled: bool,
    },
    Cancel,
    /// Clears the messages of the status bar.
    DismissMessages,
    /// Show everything in the view (the button „Alles“, as the key `F`).
    FrameAll,
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
    /// Switches the automatic loop between the submaps at times `a` and `b` (seconds since the
    /// start of the scan) of the run on or off (`[[reject]]` of `corrections.toml`), then
    /// recomputes the run from its submaps.
    SetLoopUsed {
        run: PathBuf,
        a: f64,
        b: f64,
        used: bool,
    },
    /// Switch the corrections of the run back as in state `nr`, then recompute it (a new
    /// state; the old ones stay).
    RestoreHistory {
        run: PathBuf,
        nr: u32,
    },
    /// Show another step: also sets what the view shows (Daten and Planung the cloud,
    /// Bilder the surface if there is one or one is being loaded). The active step again
    /// changes nothing.
    SetStep(Step),
    /// Queue every new or stale scan ([`steps::daten::scan_status`]), in the order of the
    /// list, and compute them one after the other: SLAM first if the scan has no run with a
    /// result, then the map of its newest run with the current map settings.
    ComputeAllMaps,
    /// [`Action::ComputeAllMaps`] for this one scan (id), whatever its status.
    ComputeMap(String),
    /// Queue every map without a current surface ([`AppState::maps_to_surface`]), in the
    /// order of the list, and compute their surfaces one after the other with the current
    /// surface settings.
    ComputeAllSurfaces,
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
    /// Open the window „Exportieren“ for this source (the context menu of the tree).
    OpenExport(ExportSource),
    /// Open the window „Exportieren“ for what the view shows ([`export::shown_sources`]: the
    /// cloud, the surface, or both to choose from).
    OpenExportShown,
    /// „Ziel wählen …“: the save dialog of the system, then [`export::target_picked`].
    PickExportTarget,
    /// „Ersetzen“ was answered: closes the question and exports.
    ConfirmExport(ExportRequest),
    /// Export as asked, as a step (also without a project).
    RunExport(ExportRequest),
    /// ☰ „Protokoll-Ordner öffnen“: the folder of the log file in Finder or Explorer.
    OpenLogFolder,
    /// Asks before a change of the project ([`AppState::ask`]).
    Ask(Ask),
    /// „Löschen …“ of the folder `dir` of the tree: asks with its size and what depends on it.
    AskDelete(PathBuf),
    /// Closes the question without a change.
    CloseAsk,
    /// Gives the scan (id) the shown name.
    RenameScan {
        id: String,
        name: String,
    },
    /// Takes the scan (id) out of the project: only its entry.
    RemoveScan(String),
    /// Takes the run folder made by the CLI out of the scan (id): only the entry.
    RemoveRun {
        scan: String,
        run: PathBuf,
    },
    /// „Ordner neu zuordnen …“ of the scan (id): the folder dialog first.
    PickRelinkScan(String),
    /// Points the scan (id) at the folder `dir`, if it holds the same scan.
    RelinkScan {
        id: String,
        dir: PathBuf,
    },
    /// Moves the folder `dir` of the tree to the trash.
    Delete(PathBuf),
    /// „Aus der Ansicht nehmen“: what the view shows from the folder `dir` (or from a model
    /// that uses it) goes: the cloud, the surface, the run drawn, „Von Hand ausrichten“.
    Unshow(PathBuf),
    /// „Aufräumen“: what jobs cut off left behind ([`AppState::leftovers_shown`]) goes to the
    /// trash.
    CleanUp,
    /// A stroke of the brush (press to release) on a surface of the map or model in `owner`:
    /// `dabs` into Stelle `mark`, or into a new one with `None` (also when `mark` is gone).
    /// Saved in its `stellen.toml` in the background; one stroke is one step back for ↶.
    Paint {
        owner: PathBuf,
        mark: Option<u32>,
        dabs: Vec<DabInput>,
    },
    /// Chooses the Stelle the brush paints into (of the map or model of the surface shown);
    /// `None`: the next stroke begins a new one.
    SelectMark(Option<u32>),
    /// Chooses the Stelle and shows it in the view, looking straight at it.
    FrameMark(u32),
    /// Takes the Stelle out of `stellen.toml` with all its touch-up (right click „Löschen“);
    /// ↶ brings it back.
    DeleteMark(u32),
    /// ↶, ⌘Z: takes back the youngest stroke, step or „Löschen“ of the Stellen of the surface
    /// shown; a step taken back (or a Stelle with steps) computes the touch-up anew.
    UndoMark,
    /// „Glätten“ of Stelle `mark` of the surface shown with `level`: the step goes into
    /// `stellen.toml`, then the touch-up of the surface is computed ([`Action::Touchup`]).
    Smooth {
        mark: u32,
        level: SmoothLevel,
    },
    /// „Bilder neu einspielen“ of Stelle `mark` of the surface shown, with its next variant;
    /// then the touch-up as for [`Action::Smooth`]. Refused while the raw data are gone.
    Retexture {
        mark: u32,
    },
    /// „Vorher“ (`false`) or „Nachher“ (`true`): the surface shown as computed, or touched up.
    ShowEdit(bool),
    /// „Melden …“ (⇧⇧): the window „Melden“, about Stelle `mark` of the surface shown if given,
    /// with „Markierte Stellen anhängen (n)“ ticked where there are Stellen. With a GPU it opens
    /// once the picture of the window has come, so the picture shows the window without it.
    OpenFeedback {
        mark: Option<u32>,
    },
    /// „Melden“ in the dialog: the report is written as a folder in the background
    /// ([`crate::feedback::write`]), the dialog closes; „Meldung gespeichert“ says when it is.
    SendFeedback,
    /// ☰ → „Meldungen …“: the window with the reports of this computer, read anew.
    OpenReports,
    /// „Ordner öffnen“ of a report: its folder in Finder or Explorer.
    OpenReportFolder(PathBuf),
    /// „Zur Ansicht“ of the report in the folder: its project is opened, then its step and
    /// selection are set, and once what they load is shown the camera is set as it was.
    ReportView(PathBuf),
    /// The touch-up of the surface in the folder `surface` as a step, with its progress and
    /// „Abbrechen“ ([`crate::surface::run_touchup_job`]): the steps of the Stellen of its map or
    /// model replayed and kept in `nacharbeit/`; the view gets the touched-up state. `step`:
    /// the step just added (Stelle, seq) – what came of it is told, and if it changed nothing
    /// (or did not run to its end) it is taken out of `stellen.toml` again. `tell`: it applies
    /// the Stellen to a surface shown, exported or new – what came of it is told (see
    /// [`brush::TouchupWish`]). The window queues it ([`brush::TouchupState::waiting`]) once the
    /// Stellen are saved and no step runs; „Abbrechen“ stops it alone, not the queue of „Alle
    /// rechnen“ it runs in.
    Touchup {
        surface: PathBuf,
        step: Option<(u32, u64)>,
        tell: bool,
    },
}

/// A question the window asks before a change of the project, in a small window over the view
/// ([`manage`]).
#[derive(Debug, Clone, PartialEq)]
pub enum Ask {
    /// „Umbenennen …“ of a scan: its id and the name being typed.
    RenameScan { id: String, name: String },
    /// „Aus Projekt entfernen“ of a scan.
    RemoveScan { id: String },
    /// „Löschen …“ of a folder of the tree.
    Delete(DeleteQuestion),
}

/// What „Löschen …“ asks: the folder, its name and size, what lies in it, and the models
/// that use it (a map, or a map of a run).
#[derive(Debug, Clone, PartialEq)]
pub struct DeleteQuestion {
    pub dir: PathBuf,
    pub label: String,
    pub bytes: Option<u64>,
    pub contains: Vec<String>,
    pub used_by: Vec<String>,
}

/// What the queue of [`AppState::queue`] computes one after the other.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Queued {
    /// A scan folder ([`crate::project::ScanEntry::path`]): SLAM if it has no run with a
    /// result, then the map of its newest run ([`Action::ComputeAllMaps`]).
    Scan(PathBuf),
    /// A map folder: its surface ([`Action::ComputeAllSurfaces`]).
    Surface(PathBuf),
}

/// A message for the status bar (German). Warnings and errors also go into the Protokoll, and
/// every message into the log file (`crate::logfile`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UiMessage {
    Info(String),
    /// Something to look at that stops nothing (a project in iCloud Drive, little space).
    Warning(String),
    Error(String),
}

impl UiMessage {
    pub fn text(&self) -> &str {
        match self {
            UiMessage::Info(t) | UiMessage::Warning(t) | UiMessage::Error(t) => t,
        }
    }
}

/// How a line of the Protokoll is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogKind {
    /// A message of a step.
    Info,
    /// A warning of a step, or of the window.
    Warning,
    /// An error of the window (a step that failed, a file that could not be read).
    Error,
}

/// A line of the Protokoll: what a step said, or a warning or error of the status bar, with
/// the local time it came („14:03:12“). The text is as shown: system errors in German
/// ([`text::german`]), symbols the fonts lack replaced ([`text::shown`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogEntry {
    pub time: String,
    pub kind: LogKind,
    pub text: String,
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
    /// Fixed pixels or spots in metres (the default).
    pub point_size: PointSize,
    /// Spot diameter in metres; `None` = from the map ([`CloudSummary::point_m`]).
    pub point_m: Option<f32>,
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
            show: ShowMode::default(),
            point_size: PointSize::default(),
            point_m: None,
        }
    }
}

/// The loaded cloud, as the panels need it.
#[derive(Debug, Clone, PartialEq)]
pub struct CloudSummary {
    pub path: PathBuf,
    pub points: usize,
    pub has_rgb: bool,
    /// Whether the cloud is made of several scans ([`crate::cloud::PointSet::source`]).
    pub has_scans: bool,
    /// Whether the cloud has submap qualities (a SLAM report).
    pub quality_known: bool,
    /// Lowest and highest relative z.
    pub z_range: [f32; 2],
    /// A height just below the ceiling, if one was found.
    pub ceiling: Option<f32>,
    /// The spot diameter (m) of its map ([`crate::cloud::point_m_of_map`]).
    pub point_m: f32,
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
    /// The triangles of the surface while it is shown alone (the line then names them
    /// instead of the points).
    pub triangles: Option<usize>,
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

/// The tree of a project as the field shows it: its scans with their runs, its models and the
/// point pairs. Read from disk off the UI thread ([`read_tree`]), shown by
/// [`AppState::set_tree`].
#[derive(Debug, Clone, Default)]
pub struct Tree {
    pub scans: Vec<ScanNode>,
    pub models: Vec<ModelInfo>,
    pub point_pairs: Vec<PointPairEntry>,
    /// Why `modelle/punktpaare.toml` cannot be read.
    pub point_pairs_error: Option<String>,
    /// What jobs cut off left behind ([`crate::leftovers`]).
    pub leftovers: Vec<Leftover>,
    /// The folders a running job marks busy, with who holds them
    /// ([`crate::leftovers::busy_folders`]).
    pub busy: Vec<(PathBuf, String)>,
    /// The Stellen of the maps and models, by folder ([`brush::read_marks`]).
    pub marks: BTreeMap<PathBuf, brush::MarksRead>,
}

/// Reads the tree of `project` from disk: every scan folder, run, map, surface and model, the
/// Stellen painted on them, and what jobs cut off left behind. A run without its `slam.json`
/// or a map without its `map.json` (one being computed, or a leftover) is not in the tree –
/// unless it holds finished results (maps, surfaces, the history of a run): then it stays,
/// marked incomplete
/// ([`RunInfo::incomplete`], [`crate::project::MapInfo::incomplete`]; see
/// [`crate::leftovers`]). Can take seconds on a network share or with files iCloud moved away:
/// call it off the UI thread.
pub fn read_tree(project: &Project) -> Tree {
    use crate::leftovers::holds_results;
    let complete = |s: &crate::project::ScanEntry, mut run: RunInfo| {
        // a run made by the CLI and added by hand stays as it is
        let added = s.extra_runs.contains(&run.dir);
        if !added && run.incomplete && !holds_results(&run.dir) {
            return None;
        }
        run.maps.retain(|m| !m.incomplete || holds_results(&m.dir));
        Some(run)
    };
    let scans: Vec<ScanNode> = project
        .scans
        .iter()
        .map(|s| ScanNode {
            id: s.id.clone(),
            name: s.name.clone(),
            state: project.scan_state(&s.id),
            no_color: project.color_missing(&s.id),
            runs: project
                .runs(&s.id)
                .into_iter()
                .filter_map(|r| complete(s, r))
                .collect(),
        })
        .collect();
    let ids: Vec<String> = project.scans.iter().map(|s| s.id.clone()).collect();
    let (point_pairs, point_pairs_error) = match point_pairs(project) {
        Ok(pairs) => (pairs, None),
        Err(e) => (Vec::new(), Some(e)),
    };
    let models = project.models();
    let owners = scans
        .iter()
        .flat_map(|s: &ScanNode| &s.runs)
        .flat_map(|r| &r.maps)
        .map(|m| m.dir.clone())
        .chain(models.iter().map(|m| m.dir.clone()));
    let marks = brush::read_marks(owners);
    Tree {
        scans,
        models,
        point_pairs,
        point_pairs_error,
        leftovers: crate::leftovers::find(&project.dir, &ids),
        busy: crate::leftovers::busy_folders(&project.dir),
        marks,
    }
}

/// What reading a project folder found: the project as its `project.toml` says now, its tree,
/// the cloud storage it lies in (`cloud`, see [`crate::places::cloud_storage`]) and what was
/// chosen in it last (`auswahl.toml`).
#[derive(Debug, Clone)]
pub struct ProjectRead {
    pub project: Project,
    pub tree: Tree,
    pub cloud: Option<String>,
    pub choices: Option<crate::project::Choices>,
}

/// How the window tells whether a folder lies in cloud storage (the system's way, or a test's).
pub type CloudCheck = fn(&Path) -> Option<String>;

/// Opens the project in `dir` and reads its tree ([`read_tree`]); `cloud` tells the cloud
/// storage of its folder. Off the UI thread: `project.toml`, the tree and the cloud check may
/// each wait for a slow disk.
pub fn read_project(dir: &Path, cloud: CloudCheck) -> Result<ProjectRead, ProjectError> {
    let project = Project::open(dir)?;
    let tree = read_tree(&project);
    let cloud = cloud(&project.dir);
    let choices = crate::project::Choices::read(&project.dir);
    Ok(ProjectRead {
        project,
        tree,
        cloud,
        choices,
    })
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
    /// The models of the project ([`Project::models`]), as last read from disk.
    pub models: Vec<ModelInfo>,
    /// The field „Scans zusammenführen“.
    pub merge: steps::merge::MergeState,
    /// „Von Hand ausrichten“, while it is open.
    pub pairs: Option<tools::PairSession>,
    /// The point pairs of the project ([`point_pairs`]), as last read from disk …
    pub point_pairs: Vec<PointPairEntry>,
    /// … or why `modelle/punktpaare.toml` cannot be read.
    pub point_pairs_error: Option<String>,
    /// A switch of a point pair is being written.
    pub pairs_writing: bool,
    pub selection: Selection,
    pub map_settings: MapSettings,
    pub slam_settings: SlamSettings,
    /// What the next surface of a map is computed with.
    pub surface_settings: SurfaceSettings,
    /// „Grafikkarte nutzen“ (⚙): what maps and surfaces are computed on. Gives an
    /// equivalent result, so it is not part of the settings a map or surface is told by.
    pub gpu: GpuPref,
    pub view: ViewSettings,
    /// Oldest first; the status bar shows the last.
    pub messages: Vec<UiMessage>,
    /// The running step, as of this frame.
    pub job: Option<JobStatus>,
    /// The Protokoll: messages of the steps, and the warnings and errors of the status bar,
    /// oldest first.
    pub log: Vec<LogEntry>,
    /// The step action that is running (set by [`actions::start_step`]).
    pub running: Option<Action>,
    /// The step of the work shown in the field next to the rail.
    pub step: Step,
    /// What is still to compute by [`Action::ComputeAllMaps`] or
    /// [`Action::ComputeAllSurfaces`], in order; the first is the one being computed. Emptied
    /// by „Abbrechen“ (not of a touch-up the window put in between), a failed step, closing and
    /// a project switch.
    pub queue: VecDeque<Queued>,
    /// The background check of [`AppState::map_status`] has not caught up with the tree yet:
    /// until then a stale map or surface counts as fitting, so „Alle rechnen“ and „Alle
    /// Oberflächen rechnen“ wait for it.
    pub map_status_pending: bool,
    /// The settings window ⚙.
    pub settings_window: SettingsWindow,
    /// The window „Über Driftless“ is open (☰ opens it, its × closes it).
    pub about_open: bool,
    /// The window „Exportieren“, while it is open.
    pub export: Option<export::ExportDialog>,
    /// The last check of a scan, shown under „Eigenschaften“ while that scan is selected.
    pub inspect: Option<InspectResult>,
    /// The map and surface folders of the tree whose status is known. Filled in the
    /// background (see [`AppState::map_status_job`]), so a folder may be missing here for a
    /// moment.
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
    /// The point clicked in the 3D view with the tool „Punkt wählen“.
    pub picked: Option<view3d::PointInfo>,
    /// The tool of the view; always exactly one.
    pub tool: tools::Tool,
    /// The measurements of the tool „Messen“, oldest first (not saved yet: Baustein 5).
    pub measurements: Vec<tools::Measurement>,
    /// The first end of a measurement being made (absolute); Esc or another tool drops it.
    pub measure_a: Option<[f64; 3]>,
    /// The manual corrections of the run the view draws.
    pub correct: correct::CorrectState,
    /// The comparison of two trajectories and of two maps.
    pub compare: compare::CompareState,
    /// The folder of the project being opened: its `project.toml` and tree are read in the
    /// background, the field says so meanwhile.
    pub opening: Option<PathBuf>,
    /// The tree is being read again (after a change, a step, the window coming to the front):
    /// the tree shown is the one read before, and „Alle rechnen“ waits for the new one.
    pub tree_pending: bool,
    /// The disk use of the project and the free space of its drive, as last measured in the
    /// background (after each read of the tree); `None` before the first measurement.
    pub disk: Option<crate::disk::DiskUse>,
    /// The question asked before a change of the project, while its window is open.
    pub ask: Option<Ask>,
    /// What jobs cut off left behind, as the last read of the tree found it.
    pub leftovers: Vec<Leftover>,
    /// The folders a running job marked busy when the tree was last read (this window's,
    /// another window's or program's), with who holds them: „Löschen …“ waits for them.
    pub busy: Vec<(PathBuf, String)>,
    /// How long a leftover must lie untouched before it is offered; `None`:
    /// [`crate::leftovers::QUIET`] (tests: zero).
    pub leftover_quiet: Option<std::time::Duration>,
    /// The Stellen of the maps and models, by folder, as last read with the tree and changed
    /// since by this window (a stroke shows here at once and is saved in the background):
    /// those of the surface shown are listed in „Bilder“ and marked in the view.
    pub marks: BTreeMap<PathBuf, brush::MarksRead>,
    /// The brush: its size and the Stelle it paints into.
    pub brush: brush::BrushState,
    /// The touch-up of the Stellen: „Vorher“ / „Nachher“, what the view holds, what waits.
    pub touchup: brush::TouchupState,
    /// The dialog „Melden“, while it is open.
    pub feedback_dialog: Option<feedback::FeedbackDialog>,
    /// The window „Meldungen auf diesem Rechner“, while it is open.
    pub feedback_list: Option<feedback::ReportList>,
}

impl AppState {
    /// Shows `project` from now on: nothing selected, its tree read from disk here (the window
    /// reads it in the background and calls [`AppState::show_project`]).
    pub fn set_project(&mut self, project: Project) {
        let tree = read_tree(&project);
        self.show_project(project, tree);
    }

    /// Shows `project` with its `tree` from now on: nothing selected.
    pub fn show_project(&mut self, project: Project, tree: Tree) {
        // the disk use of another project is not this one's
        if self.project.as_ref().map(|p| &p.dir) != Some(&project.dir) {
            self.disk = None;
        }
        self.project = Some(project);
        self.selection = Selection::None;
        self.inspect = None;
        self.merge = steps::merge::MergeState::default();
        tools::end_pairs(self);
        self.set_tree(tree);
    }

    /// Reads `modelle/punktpaare.toml` again ([`AppState::point_pairs`]).
    pub fn refresh_pairs(&mut self) {
        let read = self.project.as_ref().map_or(Ok(Vec::new()), point_pairs);
        (self.point_pairs, self.point_pairs_error) = match read {
            Ok(pairs) => (pairs, None),
            Err(e) => (Vec::new(), Some(e)),
        };
    }

    /// The point pairs (index from 0, entry) that name two scans of `model`, with their
    /// numbers in it (from 0).
    pub fn pairs_of<'a>(
        &'a self,
        model: &'a ModelInfo,
    ) -> impl Iterator<Item = (usize, &'a PointPairEntry, [usize; 2])> + 'a {
        let nr = |dir: &std::path::Path| model.report.members.iter().position(|m| m.map_dir == dir);
        self.point_pairs
            .iter()
            .enumerate()
            .filter_map(move |(k, e)| {
                let (a, b) = (nr(&e.a)?, nr(&e.b)?);
                (a != b).then_some((k, e, [a, b]))
            })
    }

    /// Reads the tree of the project again, here and now ([`read_tree`]; the window does it in
    /// the background).
    pub fn refresh(&mut self) {
        let tree = self.project.as_ref().map(read_tree).unwrap_or_default();
        self.set_tree(tree);
    }

    /// Shows `tree` as the tree of the project; the statuses of what is still in it stay until
    /// the background check replaces them.
    pub fn set_tree(&mut self, tree: Tree) {
        self.scans = tree.scans;
        self.models = tree.models;
        self.point_pairs = tree.point_pairs;
        self.point_pairs_error = tree.point_pairs_error;
        self.leftovers = tree.leftovers;
        self.busy = tree.busy;
        self.marks = tree.marks;
        // „Von Hand ausrichten“ of a model that is gone ends
        let gone = self
            .pairs
            .as_ref()
            .is_some_and(|s| self.model(&s.model).is_none());
        if gone {
            tools::end_pairs(self);
        }
        // Statuses of maps, models and surfaces still in the tree stay shown until the
        // background job replaces them.
        let maps = self
            .scans
            .iter()
            .flat_map(|s| &s.runs)
            .flat_map(|r| &r.maps)
            .map(|m| (&m.dir, &m.surfaces));
        let models = self.models.iter().map(|m| (&m.dir, &m.surfaces));
        let dirs: std::collections::BTreeSet<PathBuf> = maps
            .chain(models)
            .flat_map(|(dir, surfaces)| std::iter::once(dir).chain(surfaces.iter().map(|s| &s.dir)))
            .cloned()
            .collect();
        self.map_status.retain(|dir, _| dirs.contains(dir));
        self.mark_stale_surfaces();
        self.tree_version += 1;
    }

    /// Takes the statuses of the background job ([`AppState::map_status_job`]).
    pub fn set_map_status(&mut self, status: BTreeMap<PathBuf, MapStatus>) {
        self.map_status = status;
        self.mark_stale_surfaces();
    }

    /// [`crate::project::SurfaceInfo::stale`] of the tree from [`AppState::map_status`].
    fn mark_stale_surfaces(&mut self) {
        let status = &self.map_status;
        let maps = self
            .scans
            .iter_mut()
            .flat_map(|s| &mut s.runs)
            .flat_map(|r| &mut r.maps)
            .flat_map(|m| &mut m.surfaces);
        let models = self.models.iter_mut().flat_map(|m| &mut m.surfaces);
        for surface in maps.chain(models) {
            surface.stale = status.get(&surface.dir).is_some_and(|s| s.stale);
        }
    }

    /// The work of computing [`AppState::map_status`] for the current tree, to run off the
    /// UI thread: it reads `traj.tum` and the bag list of every map ([`map_status`]), every
    /// model ([`model_status`]) and every surface ([`surface_status`],
    /// [`model_surface_status`]).
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
        let models = self.models.clone();
        move || {
            let mut out = BTreeMap::new();
            for model in models {
                for surface in &model.surfaces {
                    out.insert(surface.dir.clone(), model_surface_status(&model, surface));
                }
                out.insert(model.dir.clone(), model_status(&model));
            }
            for (scan, run, map) in maps {
                for surface in &map.surfaces {
                    let status = surface_status(&scan, &run, &map, surface);
                    out.insert(surface.dir.clone(), status);
                }
                out.insert(map.dir.clone(), map_status(&scan, &run, &map));
            }
            out
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

    /// The work of measuring the disk use of the project ([`crate::disk::measure`]) for the
    /// current tree, to run off the UI thread; `free` tells the free space.
    pub fn disk_job(
        &self,
        free: crate::disk::FreeSpace,
    ) -> Option<impl FnOnce() -> crate::disk::DiskUse + Send + 'static> {
        let project = self.project.as_ref()?;
        let dir = project.dir.clone();
        let tree = Tree {
            scans: self.scans.clone(),
            models: self.models.clone(),
            ..Tree::default()
        };
        let folders = crate::disk::tree_folders(&tree);
        let scans: Vec<PathBuf> = project.scans.iter().map(|s| s.path.clone()).collect();
        Some(move || crate::disk::measure(&dir, &folders, &scans, free))
    }

    /// The leftovers „Daten“ offers for „Aufräumen“: those untouched for the quiet time.
    pub fn leftovers_shown(&self) -> Vec<&Leftover> {
        let quiet = self.leftover_quiet.unwrap_or(crate::leftovers::QUIET);
        let now = std::time::SystemTime::now();
        self.leftovers
            .iter()
            .filter(|l| crate::leftovers::settled(l, quiet, now))
            .collect()
    }

    /// Why „Aufräumen“ waits: a step runs or waits in this window.
    pub fn clean_up_blocked(&self) -> Option<&'static str> {
        (self.running.is_some() || !self.queue.is_empty()).then_some(text::CLEAN_UP_WAIT)
    }

    /// The size of a folder of the tree, if measured.
    pub fn folder_size(&self, dir: &Path) -> Option<u64> {
        self.disk.as_ref()?.folders.get(dir).copied()
    }

    /// The name of a folder of the tree with its size where measured: a run, a map, a surface
    /// or a model; `None` for another folder.
    fn sized_label(&self, dir: &Path) -> Option<String> {
        let label = self.tree_label(dir)?;
        Some(match self.folder_size(dir) {
            Some(bytes) => text::with_size(&label, bytes),
            None => label,
        })
    }

    /// The name of a folder of the tree: a run, a map, a surface or a model; `None` for
    /// another folder.
    fn tree_label(&self, dir: &Path) -> Option<String> {
        let maps = self.all_maps();
        self.scans
            .iter()
            .flat_map(|s| &s.runs)
            .find(|r| r.dir == dir)
            .map(|r| r.label.clone())
            .or_else(|| {
                self.all_maps()
                    .find(|m| m.dir == dir)
                    .map(|m| m.label.clone())
            })
            .or_else(|| self.model(dir).map(|m| m.label.clone()))
            .or_else(|| {
                maps.flat_map(|m| &m.surfaces)
                    .chain(self.models.iter().flat_map(|m| &m.surfaces))
                    .find(|s| s.dir == dir)
                    .map(|s| s.label.clone())
            })
    }

    /// The labels of the models that use the map in `dir`, or a map of the run in `dir`.
    pub fn map_users(&self, dir: &Path) -> Vec<String> {
        self.model_users(dir)
            .into_iter()
            .map(|m| m.label.clone())
            .collect()
    }

    /// The folders of the models that use the map in `dir`, or a map of the run in `dir`.
    pub fn model_users_dirs(&self, dir: &Path) -> Vec<PathBuf> {
        self.model_users(dir)
            .iter()
            .map(|m| m.dir.clone())
            .collect()
    }

    /// The models whose scans were merged from the map in `dir` or from a map of the run in
    /// `dir`.
    fn model_users(&self, dir: &Path) -> Vec<&ModelInfo> {
        self.models
            .iter()
            .filter(|m| {
                m.report
                    .members
                    .iter()
                    .any(|s| s.map_dir.starts_with(dir) || s.slam_dir == dir)
            })
            .collect()
    }

    /// The question before „Löschen …“ of the folder `dir` of the tree: its name and size,
    /// what lies in it (the maps of a run, the surfaces of a map or model) and the models that
    /// use it; `None` for a folder that is not in the tree.
    pub fn delete_question(&self, dir: &Path) -> Option<DeleteQuestion> {
        let label = self.tree_label(dir)?;
        let mut contains: Vec<String> = Vec::new();
        for run in self
            .scans
            .iter()
            .flat_map(|s| &s.runs)
            .filter(|r| r.dir == dir)
        {
            for map in &run.maps {
                contains.extend(self.sized_label(&map.dir));
            }
        }
        let surfaces = self
            .all_maps()
            .filter(|m| m.dir.starts_with(dir))
            .flat_map(|m| &m.surfaces)
            .chain(
                self.models
                    .iter()
                    .filter(|m| m.dir == dir)
                    .flat_map(|m| &m.surfaces),
            );
        for surface in surfaces {
            contains.extend(self.sized_label(&surface.dir));
        }
        Some(DeleteQuestion {
            dir: dir.to_path_buf(),
            label,
            bytes: self.folder_size(dir),
            contains,
            used_by: self.map_users(dir),
        })
    }

    /// Why the folder `dir` of the tree cannot go to the trash now ([`Self::delete_computing`],
    /// else [`Self::delete_shown`]). `None` if it can.
    pub fn delete_blocked(&self, dir: &Path) -> Option<String> {
        self.delete_computing(dir)
            .or_else(|| self.delete_shown(dir))
    }

    /// The folder `dir` and the models that use it: what a deletion of `dir` touches.
    fn touched_by(&self, dir: &Path) -> Vec<PathBuf> {
        std::iter::once(dir.to_path_buf())
            .chain(self.model_users_dirs(dir))
            .collect()
    }

    /// Whether the view shows something from the folder `dir` or from a model that uses it:
    /// the cloud, the surface, the run drawn, „Von Hand ausrichten“; with the reason (the
    /// window offers „Aus der Ansicht nehmen“, [`Action::Unshow`]).
    pub fn delete_shown(&self, dir: &Path) -> Option<String> {
        let touched = self.touched_by(dir);
        let shown = [
            self.cloud.as_ref().map(|c| c.path.as_path()),
            self.surface.as_ref().map(|s| s.path.as_path()),
            self.correct.run.as_deref(),
            self.pairs.as_ref().map(|p| p.model.as_path()),
        ];
        let p = shown
            .into_iter()
            .flatten()
            .find(|p| touched.iter().any(|t| p.starts_with(t)))?;
        let name = p.file_name().map_or_else(
            || p.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        Some(text::delete_shown(&name))
    }

    /// Whether a step works on the folder `dir` (or on a model that uses it, or on the folder
    /// it lies in), or will (the queue); or a job – this window's, another window's or
    /// program's – marked it or a folder in it busy when the tree was read ([`AppState::busy`];
    /// the window looks again right before the folder goes). With the reason.
    pub fn delete_computing(&self, dir: &Path) -> Option<String> {
        let touched = self.touched_by(dir);
        let hit = |p: &Path| dir.starts_with(p) || touched.iter().any(|t| p.starts_with(t));
        let label = self.job.as_ref().map_or("", |j| j.label.as_str());
        if let Some(step) = &self.running {
            if self.step_folders(step).iter().any(|p| hit(p)) {
                return Some(text::delete_computing(label));
            }
        }
        let queued = self.queue.iter().any(|q| match q {
            Queued::Surface(map) => hit(map),
            // a queued scan computes into its runs
            Queued::Scan(path) => self
                .scan_at(path)
                .is_some_and(|s| s.runs.iter().any(|r| hit(&r.dir))),
        });
        if queued {
            return Some(text::delete_computing(text::COMPUTE_ALL));
        }
        self.busy
            .iter()
            .find(|(folder, _)| folder.starts_with(dir))
            .map(|(_, holder)| text::delete_busy(holder))
    }

    /// The folders a step reads or writes (that must not go to the trash while it runs): the
    /// run of a recompute or map, the map of a surface, the maps and model of a merge, the
    /// source of an export.
    fn step_folders(&self, step: &Action) -> Vec<PathBuf> {
        let model_maps = |dir: &Path| -> Vec<PathBuf> {
            self.model(dir).map_or_else(Vec::new, |m| {
                std::iter::once(m.dir.clone())
                    .chain(m.report.members.iter().map(|s| s.map_dir.clone()))
                    .collect()
            })
        };
        match step {
            // a new run may reuse the folder of the newest
            Action::RunSlam(id) => self
                .scan_node(id)
                .map_or_else(Vec::new, |s| s.runs.iter().map(|r| r.dir.clone()).collect()),
            Action::Reuse(run) | Action::RunMap(run) | Action::RestoreHistory { run, .. } => {
                vec![run.clone()]
            }
            Action::RunSurface(map) => vec![map.clone()],
            // it reads the Stellen of the map or model the surface folder lies in
            Action::Touchup { surface, .. } => vec![surface.clone()],
            Action::RunMerge { picks, .. } => picks.iter().map(|p| p.map.clone()).collect(),
            Action::ApplyPairs { model, .. } | Action::RunModelSurface { model } => {
                model_maps(model)
            }
            Action::RunExport(req) => {
                use driftless_steps::export::ExportSource as S;
                let (S::Map(p)
                | S::Model(p)
                | S::Surface(p)
                | S::EditedSurface(p)
                | S::Run(p)
                | S::PointFile(p)
                | S::MeshFile(p)) = &req.source;
                vec![p.clone()]
            }
            _ => Vec::new(),
        }
    }

    /// Why the scan `id` cannot be renamed, assigned anew or taken out now: a step of it runs
    /// or waits in the queue.
    pub fn scan_busy(&self, id: &str) -> Option<String> {
        let running = self
            .running
            .as_ref()
            .and_then(|a| self.scan_of(a))
            .is_some_and(|s| s.id == id);
        let path = self
            .project
            .as_ref()
            .and_then(|p| p.scan(id))
            .map(|e| &e.path);
        let queued = self.queue.iter().any(|q| match q {
            Queued::Scan(p) => Some(p) == path,
            Queued::Surface(map) => self.map(map).is_some_and(|(s, _, _)| s.id == id),
        });
        (running || queued).then(|| text::SCAN_BUSY.to_string())
    }

    /// The map of the selection: the selected map, or the map of a selected surface.
    pub fn selected_map(&self) -> Option<&crate::project::MapInfo> {
        let maps = self
            .scans
            .iter()
            .flat_map(|s| &s.runs)
            .flat_map(|r| &r.maps);
        match &self.selection {
            Selection::Map(d) => maps.into_iter().find(|m| m.dir == *d),
            Selection::Surface(d) => maps
                .into_iter()
                .find(|m| m.surfaces.iter().any(|s| s.dir == *d)),
            _ => None,
        }
    }

    /// The model of the selection: the selected model, or the model of a selected surface.
    pub fn selected_model(&self) -> Option<&ModelInfo> {
        match &self.selection {
            Selection::Model(d) => self.model(d),
            Selection::Surface(d) => self
                .models
                .iter()
                .find(|m| m.surfaces.iter().any(|s| s.dir == *d)),
            _ => None,
        }
    }

    /// The model in folder `dir`.
    pub fn model(&self, dir: &std::path::Path) -> Option<&ModelInfo> {
        self.models.iter().find(|m| m.dir == dir)
    }

    /// The name of the scan in folder `path` as the project names it, else its folder name.
    pub fn scan_name(&self, path: &std::path::Path) -> String {
        match self.scan_at(path) {
            Some(node) => node.name.clone(),
            None => path.file_name().map_or_else(
                || path.display().to_string(),
                |n| n.to_string_lossy().into_owned(),
            ),
        }
    }

    /// Opens the field „Scans zusammenführen“; the first time with every scan ticked that
    /// has a finished map ([`steps::merge::MergeState::open`]).
    pub fn open_merge(&mut self) {
        let mut merge = std::mem::take(&mut self.merge);
        merge.open(self);
        self.merge = merge;
    }

    /// The run and map of the map folder `dir`, with its scan.
    pub fn map(
        &self,
        dir: &std::path::Path,
    ) -> Option<(&ScanNode, &RunInfo, &crate::project::MapInfo)> {
        self.scans.iter().find_map(|s| {
            s.runs
                .iter()
                .find_map(|r| r.maps.iter().find(|m| m.dir == dir).map(|m| (s, r, m)))
        })
    }

    /// The scans [`Action::ComputeAllMaps`] queues: the new and the stale ones, in the order
    /// of the list, as scan folders.
    pub fn scans_to_compute(&self) -> Vec<PathBuf> {
        let Some(project) = &self.project else {
            return Vec::new();
        };
        self.scans
            .iter()
            .filter(|s| {
                matches!(
                    steps::daten::scan_status(self, s),
                    steps::daten::ScanStatus::New | steps::daten::ScanStatus::Stale
                )
            })
            .filter_map(|s| project.scan(&s.id).map(|e| e.path.clone()))
            .collect()
    }

    /// The maps [`Action::ComputeAllSurfaces`] queues: those without a current surface
    /// ([`steps::bilder::surface_status`] new or stale), in the order of the list, as map
    /// folders. A map that waits for „Daten“ ([`AppState::maps_waiting_for_data`]) is left
    /// out.
    pub fn maps_to_surface(&self) -> Vec<PathBuf> {
        use steps::bilder::SurfaceStatus;
        self.all_maps()
            .filter(|m| {
                matches!(
                    steps::bilder::surface_status(self, m),
                    SurfaceStatus::New | SurfaceStatus::Stale
                )
            })
            .map(|m| m.dir.clone())
            .collect()
    }

    /// How many maps [`Action::ComputeAllSurfaces`] leaves out because they wait for „Daten“:
    /// the map no longer fits its run, has no readable `map.json`, or its scan cannot be read.
    /// A map kept without its report for its surfaces ([`crate::project::MapInfo::incomplete`])
    /// waits for nothing: „Daten“ computes a new one.
    pub fn maps_waiting_for_data(&self) -> usize {
        use steps::bilder::SurfaceStatus;
        self.all_maps()
            .filter(|m| !m.incomplete)
            .filter(|m| {
                matches!(
                    steps::bilder::surface_status(self, m),
                    SurfaceStatus::MapStale
                        | SurfaceStatus::MapNotReady
                        | SurfaceStatus::ScanMissing
                )
            })
            .count()
    }

    /// Every surface of the tree: of the maps, then of the models, in the order of the list.
    pub fn all_surfaces(&self) -> impl Iterator<Item = &crate::project::SurfaceInfo> {
        let maps = self.all_maps().flat_map(|m| &m.surfaces);
        maps.chain(self.models.iter().flat_map(|m| &m.surfaces))
    }

    /// Every map of the tree, in the order of the list.
    fn all_maps(&self) -> impl Iterator<Item = &crate::project::MapInfo> {
        self.scans
            .iter()
            .flat_map(|s| &s.runs)
            .flat_map(|r| &r.maps)
    }

    /// The scan with the folder `path`.
    pub fn scan_at(&self, path: &std::path::Path) -> Option<&ScanNode> {
        let entry = self
            .project
            .as_ref()?
            .scans
            .iter()
            .find(|s| s.path == path)?;
        self.scan_node(&entry.id)
    }

    /// The step that computes the head of [`AppState::queue`] further: for a scan the map of
    /// its newest run with a result, or SLAM if it has none; for a map its surface. `None` if
    /// the head is no longer in the project.
    pub fn next_queued_step(&self) -> Option<Action> {
        match self.queue.front()? {
            Queued::Scan(path) => {
                let scan = self.scan_at(path)?;
                Some(match scan.runs.iter().rev().find(|r| r.has_result) {
                    Some(run) => Action::RunMap(run.dir.clone()),
                    None => Action::RunSlam(scan.id.clone()),
                })
            }
            Queued::Surface(dir) => {
                self.map(dir)?;
                Some(Action::RunSurface(dir.clone()))
            }
        }
    }

    /// Whether `step` works on the head of [`AppState::queue`]: on its scan, or the surface
    /// of its map.
    pub fn computes_queue_head(&self, step: &Action) -> bool {
        match (self.queue.front(), step) {
            (Some(Queued::Surface(head)), Action::RunSurface(dir)) => head == dir,
            (Some(Queued::Surface(_)), _) | (None, _) => false,
            (Some(Queued::Scan(head)), step) => {
                let project = self.project.as_ref();
                self.scan_of(step)
                    .and_then(|s| project?.scan(&s.id))
                    .is_some_and(|e| e.path == *head)
            }
        }
    }

    /// The scan a step action works on.
    pub fn scan_of(&self, action: &Action) -> Option<&ScanNode> {
        match action {
            Action::Inspect(id) | Action::RunSlam(id) => self.scan_node(id),
            Action::Reuse(dir) | Action::RunMap(dir) | Action::RestoreHistory { run: dir, .. } => {
                self.run(dir).map(|(s, _)| s)
            }
            Action::RunSurface(dir) => self.map(dir).map(|(s, _, _)| s),
            _ => None,
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
    /// cloud, surface or state.
    pub fn selected_run(&self) -> Option<(&ScanNode, &RunInfo)> {
        let owns = |r: &RunInfo| match &self.selection {
            Selection::Run(d) => r.dir == *d,
            Selection::Map(d) => r.maps.iter().any(|m| m.dir == *d),
            Selection::Cloud(c) => {
                r.cloud.as_ref() == Some(c) || r.maps.iter().any(|m| m.clouds.contains(c))
            }
            Selection::Surface(d) => r
                .maps
                .iter()
                .any(|m| m.surfaces.iter().any(|s| s.dir == *d)),
            Selection::History { run, .. } => r.dir == *run,
            Selection::None | Selection::Scan(_) | Selection::Model(_) => false,
        };
        self.scans
            .iter()
            .find_map(|s| s.runs.iter().find(|r| owns(r)).map(|r| (s, r)))
    }
}
