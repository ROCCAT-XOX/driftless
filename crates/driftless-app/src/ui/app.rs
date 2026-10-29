//! The program: carries out the panels' actions, owns the job runners, the dialogs, the
//! loaded cloud, the run drawn over it and the 3D view.

use super::brush::{self, Brush, MarkChange, TouchupWish, Variant};
use super::compare::{compare_maps, compare_refs, default_refs, TrajRef};
use super::correct::{Corrected, RunLoaded};
use super::export::{self, ExportDialog};
use super::feedback::{FeedbackDialog, ReportList};
use super::tools::{self, PairSession, ScreenLayout, Tool};
use super::view3d::{effective_show, point_info, View3d, ViewEvent, ViewOverlay};
use super::{
    actions, rail, read_project, settings, status, steps, text, theme, Action, AppState,
    CloudCheck, CloudSummary, InspectResult, LogEntry, LogKind, ProjectRead, Queued, Selection,
    Step, SurfaceSummary, Tree, UiMessage, ViewSettings, ViewStats,
};
use crate::camera::CameraState;
use crate::cloud::{load_cloud, load_pair_cloud, LoadedCloud};
use crate::feedback::{DoubleShift, Feedback, FeedbackCamera, FeedbackKind, FeedbackProject};
use crate::jobs::{JobFinished, JobOutput, JobRunner, StartError};
use crate::logfile::{self, Level, LogFile};
use crate::overlay::RunOverlay;
use crate::pick::relative_to;
use crate::project::{canonical, change_project, set_point_pair_enabled, Choices, Project};
use crate::render::ShowMode;
use crate::surface::{load_mesh, surface_glb, SurfaceEdit, TouchupOutput, ViewEdit, SURFACE_GLB};
use crate::synthetic::synthetic_cloud;
use driftless_gpu::GpuPref;
use driftless_io::images::ImageIndices;
use driftless_io::progress::{LogLine, Progress};
use driftless_steps::export::{
    file_kind, ExportKind, ExportSource, MESH_EXTENSIONS, POINT_EXTENSIONS,
};
use driftless_steps::marks::{EditStep, Marks};
use driftless_steps::merge::MODEL_LAS;
use driftless_steps::surface::{NoSources, STALE_SURFACE};
use driftless_steps::touchup::{TouchupPrepared, NO_RAW_DATA};
use driftless_steps::{correct, StepError};
use eframe::egui;
use std::collections::{BTreeSet, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Key of the last opened project in eframe's storage.
const LAST_PROJECT_KEY: &str = "letztes_projekt";
/// Key of the stored layout (step and view settings) in eframe's storage. A new layout gets
/// a new version: what an older program stored is then not read.
pub const UI_KEY: &str = "ui-v2";
/// While a step runs, the window draws this often, so progress and runtime move (the job
/// runner only wakes the UI when a job ends, see `crate::jobs`).
const REPAINT_WHILE_BUSY: Duration = Duration::from_millis(100);

/// After the window got the focus back, the tree is read again this much later (switching back
/// and forth reads it once).
pub const FOCUS_DELAY: Duration = Duration::from_millis(300);
/// The focus reads the tree again at most once in this time.
pub const FOCUS_GAP: Duration = Duration::from_secs(2);

/// Lines kept in the log; older ones are dropped.
const LOG_LIMIT: usize = 10_000;
/// Messages kept for the status bar (it shows the last); older ones are dropped.
const MESSAGE_LIMIT: usize = 100;
const STEP_WIDTH: f32 = 250.0;
/// The field of the step never gets narrower than this (names are cut with „…“ instead).
const STEP_MIN_WIDTH: f32 = 220.0;
/// The hint „Noch keine Oberfläche“ stands this far below the top of the view (points), under
/// the floating info.
const HINT_TOP: f32 = 64.0;

/// Seed of the synthetic cloud of `--synthetic`.
const SYNTHETIC_SEED: u64 = 1;

/// „Melden“ opens without the picture of the window if it has not come this long (s) after it
/// was asked for (a window not drawn, a system without it).
const SHOT_WAIT: f64 = 2.0;

/// The arguments of `driftless-app [--version] [--synthetic N] [projekt-ordner]`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Args {
    pub project: Option<PathBuf>,
    /// Show a synthetic cloud of this many points (to try the view without a scan).
    pub synthetic: Option<usize>,
    /// `--version`: print version and commit, open no window.
    pub version: bool,
}

/// Reads the arguments: at most one project folder, `--synthetic N` with 1 ≤ N < 2³²
/// written as digits, optionally grouped with '_' (`50_000_000`), and `--version`.
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Args, String> {
    let mut out = Args::default();
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        if arg == "--version" {
            out.version = true;
            continue;
        }
        if arg == "--synthetic" {
            let value = args.next();
            // digits, optionally grouped with '_' (no signs, no decimal points)
            let n = value
                .as_deref()
                .filter(|v| {
                    v.starts_with(|c: char| c.is_ascii_digit())
                        && v.chars().all(|c| c.is_ascii_digit() || c == '_')
                })
                .and_then(|v| v.replace('_', "").parse::<u32>().ok())
                .filter(|&n| n > 0)
                .ok_or_else(|| text::bad_synthetic(value.as_deref()))?;
            out.synthetic = Some(n as usize);
            continue;
        }
        if arg.starts_with('-') {
            return Err(text::unknown_option(&arg));
        }
        if out.project.is_some() {
            return Err(text::one_project_only());
        }
        out.project = Some(PathBuf::from(arg));
    }
    Ok(out)
}

/// What is kept of the layout between starts, under [`UI_KEY`]: the step and how the view
/// draws. A field missing in what was stored keeps its default; an unknown one means another
/// format, and the whole is not read.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
struct StoredUi {
    step: Step,
    color_mode: crate::render::ColorMode,
    point_px: f32,
    round: bool,
    budget: usize,
    camera_mode: crate::camera::CameraMode,
    show_trajectory: bool,
    show_odometry: bool,
    show_loops: bool,
    show_rejected: bool,
    show_submaps: bool,
    show_compare: bool,
    point_size: crate::render::PointSize,
    point_m: Option<f32>,
    /// „Grafikkarte nutzen“ as its CLI word (`auto`, `an`, `aus`).
    gpu: String,
    /// The folders the dialogs were last used in ([`LastFolders`]).
    folders: LastFolders,
    /// The radius of the brush (m).
    brush_radius: f32,
}

/// Which dialogs share their start folder ([`DriftlessApp::dialog_folder`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DialogFolder {
    /// „Neues Projekt …“, „Projekt öffnen …“.
    Projects,
    /// „Scan hinzufügen …“, „Lauf hinzufügen …“.
    Scans,
    /// „Datei öffnen …“, the target of „Exportieren …“.
    Files,
}

/// The folders the dialogs were last used in (the parent of a picked project or scan, the
/// folder of a picked file), kept between starts.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LastFolders {
    pub projects: Option<PathBuf>,
    pub scans: Option<PathBuf>,
    pub files: Option<PathBuf>,
}

impl LastFolders {
    fn get(&self, kind: DialogFolder) -> Option<&Path> {
        match kind {
            DialogFolder::Projects => self.projects.as_deref(),
            DialogFolder::Scans => self.scans.as_deref(),
            DialogFolder::Files => self.files.as_deref(),
        }
    }

    fn set(&mut self, kind: DialogFolder, dir: Option<&Path>) {
        let slot = match kind {
            DialogFolder::Projects => &mut self.projects,
            DialogFolder::Scans => &mut self.scans,
            DialogFolder::Files => &mut self.files,
        };
        if let Some(dir) = dir.filter(|d| !d.as_os_str().is_empty()) {
            *slot = Some(dir.to_path_buf());
        }
    }
}

impl Default for StoredUi {
    fn default() -> Self {
        StoredUi::of(
            Step::default(),
            &ViewSettings::default(),
            GpuPref::default(),
            &LastFolders::default(),
            brush::DEFAULT_RADIUS,
        )
    }
}

impl StoredUi {
    fn of(
        step: Step,
        v: &ViewSettings,
        gpu: GpuPref,
        folders: &LastFolders,
        brush_radius: f32,
    ) -> Self {
        StoredUi {
            brush_radius,
            folders: folders.clone(),
            step,
            color_mode: v.color_mode,
            point_px: v.point_px,
            round: v.round,
            budget: v.budget,
            camera_mode: v.camera_mode,
            show_trajectory: v.show_trajectory,
            show_odometry: v.show_odometry,
            show_loops: v.show_loops,
            show_rejected: v.show_rejected,
            show_submaps: v.show_submaps,
            show_compare: v.show_compare,
            point_size: v.point_size,
            point_m: v.point_m,
            gpu: gpu.to_string(),
        }
    }

    /// „Grafikkarte nutzen“ as stored; an unknown word is `auto`.
    fn gpu(&self) -> GpuPref {
        self.gpu.parse().unwrap_or_default()
    }

    /// The radius of the brush as stored, within its range.
    fn brush_radius(&self) -> f32 {
        if self.brush_radius.is_finite() {
            self.brush_radius
                .clamp(brush::MIN_RADIUS, brush::MAX_RADIUS)
        } else {
            brush::DEFAULT_RADIUS
        }
    }

    /// The view settings as stored, the rest from `v`.
    fn apply_to(&self, v: &mut ViewSettings) {
        v.color_mode = self.color_mode;
        v.point_px = self.point_px.clamp(1.0, 10.0);
        v.round = self.round;
        v.budget = self
            .budget
            .clamp(*text::BUDGET_RANGE.start(), *text::BUDGET_RANGE.end());
        v.camera_mode = self.camera_mode;
        v.show_trajectory = self.show_trajectory;
        v.show_odometry = self.show_odometry;
        v.show_loops = self.show_loops;
        v.show_rejected = self.show_rejected;
        v.show_submaps = self.show_submaps;
        v.show_compare = self.show_compare;
        v.point_size = self.point_size;
        v.point_m = self
            .point_m
            .filter(|m| m.is_finite())
            .map(|m| m.clamp(text::POINT_M_RANGE[0], text::POINT_M_RANGE[1]));
    }
}

/// Where a cloud comes from.
#[derive(Debug, Clone, PartialEq)]
enum CloudSource {
    /// A LAS file, with the run folder whose `slam.json` gives the quality.
    File {
        path: PathBuf,
        run: Option<PathBuf>,
    },
    Synthetic(usize),
    /// A model with its scan `member` beside it, for point pairs
    /// ([`crate::cloud::load_pair_cloud`]); `name` names the cloud
    /// ([`PairSession::cloud`]), `lone` is the scan of a model with points of only one.
    Pairs {
        name: PathBuf,
        model_las: PathBuf,
        lone: u16,
        map_dir: PathBuf,
        member: u16,
    },
}

impl CloudSource {
    fn path(&self) -> PathBuf {
        match self {
            CloudSource::File { path, .. } => path.clone(),
            CloudSource::Synthetic(n) => PathBuf::from(text::synthetic_name(*n)),
            CloudSource::Pairs { name, .. } => name.clone(),
        }
    }
}

/// The cloud of the map folder `dir` to set point pairs on: `color.las`, else `clean.las`.
fn pair_scan_cloud(dir: &Path) -> Option<PathBuf> {
    ["color.las", "clean.las"]
        .into_iter()
        .map(|f| dir.join(f))
        .find(|p| p.is_file())
}

/// Quick work of the helper runner, one at a time.
#[derive(Debug, Clone, PartialEq)]
enum HelperTask {
    /// The statuses of the map folders of this tree version.
    MapStatus(u64),
    /// The start↔end gaps of the history states of this tree version.
    HistoryGaps(u64),
    /// The overlay and the corrections of a run folder.
    LoadRun(PathBuf),
    /// [`Action::AddLoop`], [`Action::SetLoopEnabled`] or [`Action::SetLoopUsed`]: they read
    /// `submaps.bin`.
    Correct(Action),
    /// Two trajectories compared (reads both, well below a second).
    Compare(TrajRef, TrajRef),
    /// Two `map.json` side by side.
    CompareMaps(PathBuf, PathBuf),
    /// [`Action::SetPairEnabled`]: entry (from 0), on or off.
    SwitchPair(usize, bool),
    /// Writes the choices into `auswahl.toml` of the project in the folder.
    SaveChoices(PathBuf, Choices),
    /// The disk use of the project for this tree version.
    DiskUse(u64),
    /// „Ordner neu zuordnen …“: points the scan (id) at the folder, if it holds the same scan
    /// (reads its bags).
    Relink { id: String, dir: PathBuf },
    /// „Aufräumen“: these leftovers (path, bytes) to the trash.
    CleanUp(Vec<(PathBuf, u64)>),
    /// „Löschen …“: the folder to the trash, with its name and size for the message.
    Trash {
        dir: PathBuf,
        label: String,
        bytes: Option<u64>,
    },
    /// A change of the Stellen of the map or model in the folder, in its `stellen.toml`
    /// ([`Marks::change`]); made on [`AppState::marks`] already.
    Marks { owner: PathBuf, change: MarkChange },
    /// „Melden“: the report written as a folder of its own ([`crate::feedback::write`]).
    Feedback(Box<FeedbackJob>),
    /// The reports in the folder, for the list (☰ → „Meldungen …“).
    ReadReports(PathBuf),
}

/// A report of „Melden“ to write ([`HelperTask::Feedback`]).
#[derive(Debug, Clone, PartialEq)]
struct FeedbackJob {
    /// The folder of the reports.
    dir: PathBuf,
    report: Feedback,
    /// The picture of the window and the Stellen, if attached.
    image: Option<Arc<egui::ColorImage>>,
    marks: Option<Marks>,
    /// The log file whose last lines go along; without one (or an empty one) `log`, the last
    /// lines of the Protokoll of the window.
    log_file: Option<PathBuf>,
    log: Vec<String>,
}

/// „Melden“ asked for, while the picture of the window is awaited: the Stelle it is about, and
/// the time of the frame that asked for the picture.
#[derive(Debug, Clone, Copy, PartialEq)]
struct FeedbackAsk {
    mark: Option<u32>,
    asked: Option<f64>,
}

/// What the picture of the window for „Melden“ is asked for with: its reply is told apart by it.
struct FeedbackShot;

/// Whether `data` came with the picture asked for by „Melden“.
fn is_feedback_shot(data: &egui::UserData) -> bool {
    data.data
        .as_ref()
        .is_some_and(|d| (**d).is::<FeedbackShot>())
}

/// „Zur Ansicht“ of a report, until its camera is set ([`DriftlessApp::advance_report_view`]).
#[derive(Debug, Clone, PartialEq)]
struct ReportView {
    /// The project of the report, to be shown first; `None`: the one shown stays.
    project: Option<PathBuf>,
    step: Option<Step>,
    selection: Option<Selection>,
    camera: Option<CameraState>,
    /// The project is shown and the step and selection are set: what they load is awaited.
    selected: bool,
}

/// The graphics adapter of `rs` as the log file names it: „Apple M4 Pro (Metal, IntegratedGpu,
/// …)“.
fn graphics_of(rs: &egui_wgpu::RenderState) -> String {
    let info = rs.adapter.get_info();
    let details = [
        format!("{:?}", info.backend),
        format!("{:?}", info.device_type),
        format!("{} {}", info.driver, info.driver_info),
    ];
    logfile::gpu_text(&info.name, &details)
}

/// What a „compute all“ button puts into the queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Batch {
    /// „Alle rechnen“ in „Daten“: the new and stale scans.
    Maps,
    /// „Alle Oberflächen rechnen“ in „Bilder“: the maps without a current surface.
    Surfaces,
}

/// The settings a queue started with, lent to each of its steps, and the count of loads by
/// hand when it started.
#[derive(Debug, Clone)]
struct QueueSettings {
    slam: driftless_steps::slam::SlamSettings,
    map: driftless_steps::map::MapSettings,
    surface: crate::project::SurfaceSettings,
    hand_loads: u64,
}

/// What a read of a project folder is for ([`DriftlessApp::read_folder`]).
#[derive(Debug, Clone, PartialEq)]
enum TreeTask {
    /// Opening the project in this folder; `restore`: the project opened last (forgotten if
    /// it cannot be read).
    Open { dir: PathBuf, restore: bool },
    /// Reading the open project again (its folder).
    Refresh(PathBuf),
}

impl TreeTask {
    fn dir(&self) -> &Path {
        match self {
            TreeTask::Open { dir, .. } | TreeTask::Refresh(dir) => dir,
        }
    }
}

/// The end of a step, kept until the tree read after it has arrived: then the result is
/// selected and shown and the queue goes on ([`DriftlessApp::finish_step`]).
#[derive(Debug)]
struct StepEnd {
    step: Option<Action>,
    output: Option<JobOutput>,
    /// It computed the head of the queue.
    queued: bool,
    seconds: f64,
}

pub struct DriftlessApp {
    state: AppState,
    /// The processing steps, one at a time. Dropping it (closing the window) cancels the
    /// running step and waits for it.
    jobs: JobRunner,
    /// Quick work off the UI thread that is no step ([`HelperTask`]).
    helper: JobRunner,
    /// What the helper does next, and what it is doing.
    helper_queue: VecDeque<HelperTask>,
    helper_running: Option<HelperTask>,
    /// Loads clouds and builds their levels of detail, beside a running step.
    loader: JobRunner,
    /// What the loader is loading, and what to load once it has stopped.
    loading: Option<CloudSource>,
    next_load: Option<CloudSource>,
    /// Loads surfaces, beside the cloud loader.
    surface_loader: JobRunner,
    /// The file it is loading (with its file stamp when the load began), and the one to load
    /// once a cancelled load has stopped.
    surface_loading: Option<(PathBuf, FileStamp)>,
    next_surface: Option<PathBuf>,
    /// The file stamp of the surface shown, when its load began.
    surface_stamp: FileStamp,
    /// The surface being loaded was dropped: its result is thrown away even if it finished
    /// before it saw the cancel.
    discard_surface: bool,
    /// The cloud shown; a new one replaces it only when it is ready.
    cloud: Option<LoadedCloud>,
    /// The run folder the cloud shown was loaded with.
    cloud_run: Option<PathBuf>,
    /// The run drawn in the view: the selected one, else the cloud's.
    overlay: Option<RunOverlay>,
    /// Raised whenever [`DriftlessApp::overlay`] changed.
    overlay_version: u64,
    view: View3d,
    /// The GPU of the window; `None` without wgpu (tests): then there is no 3D view.
    render_state: Option<egui_wgpu::RenderState>,
    /// The [`AppState::tree_version`] the map statuses were last asked for.
    status_asked: u64,
    /// The same for the gaps of the history states.
    gaps_asked: u64,
    /// A project switch that waits until the cancelled step has stopped: until then the
    /// step may still write into the old project, which therefore stays shown.
    pending: Option<Action>,
    /// „Alle rechnen“ or „Alle Oberflächen rechnen“ was asked for while the maps were being
    /// checked: it starts once the check is done ([`AppState::map_status_pending`]).
    compute_all_waiting: Option<Batch>,
    /// The settings the queue started with: every step of it uses them, even if the
    /// switches change meanwhile.
    queue_settings: Option<QueueSettings>,
    /// Clouds and surfaces asked for by hand (picked, opened) so far: a queue shows its last
    /// result only if none was asked for while it ran.
    hand_loads: u64,
    /// Whether the surface being loaded was asked for by hand: it is shown in any step, a
    /// computed one only in „Bilder“.
    surface_by_hand: bool,
    /// The window title last sent.
    title: String,
    /// Closing waits for the cancelled step; the window closes once it has ended.
    closing: bool,
    /// The stored last project could not be opened: forget it on the next save.
    forget_last: bool,
    /// [`super::theme::apply`] was called (once, at the first frame or in the creation context).
    themed: bool,
    /// Where the panels and the floating bars were in the last frame.
    layout: ScreenLayout,
    /// The log file of this start: every message of the status bar and every line of the
    /// Protokoll goes into it as it comes ([`DriftlessApp::say`], [`DriftlessApp::protocol`]).
    log_file: Option<Arc<LogFile>>,
    /// Reads project folders (`project.toml` and the tree) off the UI thread.
    tree_loader: JobRunner,
    /// What the read that runs is for, and the read to start once it has ended (the newest
    /// asked for; an open goes before a refresh).
    tree_task: Option<TreeTask>,
    tree_next: Option<TreeTask>,
    /// The result of the read that runs is not wanted any more (an open cancelled).
    tree_discard: bool,
    /// A finished step waiting for the tree read after it.
    step_end: Option<StepEnd>,
    /// How a folder is told to be in cloud storage ([`crate::places::cloud_storage`]).
    cloud_check: CloudCheck,
    /// The project folders whose cloud storage was told in this session (once each).
    cloud_told: BTreeSet<PathBuf>,
    /// The folders the dialogs were last used in, and the projects folder they start in
    /// before (`~/Driftless Projekte`; `None`: the system's choice).
    folders: LastFolders,
    projects_home: Option<PathBuf>,
    /// The choices last written into (or read from) the project in the folder: they are
    /// written again when they differ ([`DriftlessApp::keep_choices`]).
    saved_choices: Option<(PathBuf, Choices)>,
    /// The [`AppState::tree_version`] the disk use was last measured for.
    disk_asked: u64,
    /// How the free space of a drive is told ([`crate::disk::free_space`]).
    free_space: crate::disk::FreeSpace,
    /// Little free space was told (again only once there was enough in between).
    low_space_told: bool,
    /// How folders go to the trash ([`crate::trash::system_trash`]).
    trash: crate::trash::Trash,
    /// When the tree is read again after the window got the focus back ([`FOCUS_DELAY`]).
    focus_read_at: Option<Instant>,
    /// When the last such read started ([`FOCUS_GAP`]).
    focus_read: Option<Instant>,
    /// The brush at work: what it hits on the surface shown, the stroke, the Stellen as faces.
    brush: Brush,
    /// Where the camera images of the scans lie in their bags, kept for every touch-up of this
    /// window: only the first one that bakes images of a scan anew looks through its bags.
    image_indices: Arc<ImageIndices>,
    /// Changes of the Stellen saved (or refused) so far, and their count when the read of the
    /// tree that runs began: a read that began before a change was saved brings the Stellen of
    /// before it, so they are not taken from it.
    marks_saved: u64,
    tree_marks_at: u64,
    /// ⇧⇧ anywhere in the window opens „Melden“.
    double_shift: DoubleShift,
    /// „Melden“ asked for while the picture of the window is awaited.
    feedback_ask: Option<FeedbackAsk>,
    /// Where the reports go: the app's own folder ([`crate::feedback::default_dir`]) in the
    /// program eframe starts ([`DriftlessApp::from_creation_context`]); else none, unless set
    /// ([`DriftlessApp::with_feedback_dir`]), so that no test can write into the real one.
    feedback_dir: Option<PathBuf>,
    /// The dialogs of the reports being written, in order: one opens again if writing it fails.
    feedback_saving: VecDeque<FeedbackDialog>,
    /// The graphics adapter the window draws with, as the log file names it.
    graphics: Option<String>,
    /// „Zur Ansicht“ of a report, until its camera is set.
    report_view: Option<ReportView>,
}

impl DriftlessApp {
    /// Without a window; `repaint` wakes the UI when a job ends. Without a folder for the reports
    /// of „Melden“ ([`DriftlessApp::with_feedback_dir`]; the program eframe starts gets the app's
    /// own).
    pub fn new(repaint: Arc<dyn Fn() + Send + Sync>) -> Self {
        // „Nachher“ from the start, as the state says
        let mut view = View3d::default();
        view.show_edit(brush::TouchupState::default().after);
        DriftlessApp {
            state: AppState::default(),
            jobs: JobRunner::new(repaint.clone()),
            helper: JobRunner::new(repaint.clone()),
            helper_queue: VecDeque::new(),
            helper_running: None,
            loader: JobRunner::new(repaint.clone()),
            surface_loader: JobRunner::new(repaint.clone()),
            surface_loading: None,
            next_surface: None,
            surface_stamp: None,
            discard_surface: false,
            loading: None,
            next_load: None,
            cloud: None,
            cloud_run: None,
            overlay: None,
            overlay_version: 0,
            view,
            render_state: None,
            status_asked: 0,
            gaps_asked: 0,
            pending: None,
            compute_all_waiting: None,
            queue_settings: None,
            hand_loads: 0,
            surface_by_hand: false,
            title: String::new(),
            closing: false,
            forget_last: false,
            themed: false,
            layout: ScreenLayout::default(),
            log_file: None,
            tree_loader: JobRunner::new(repaint),
            tree_task: None,
            tree_next: None,
            tree_discard: false,
            step_end: None,
            cloud_check: crate::places::cloud_storage,
            cloud_told: BTreeSet::new(),
            folders: LastFolders::default(),
            projects_home: crate::places::projects_home(),
            saved_choices: None,
            disk_asked: 0,
            free_space: crate::disk::free_space,
            low_space_told: false,
            trash: crate::trash::system_trash,
            focus_read_at: None,
            focus_read: None,
            brush: Brush::default(),
            image_indices: Arc::default(),
            marks_saved: 0,
            tree_marks_at: 0,
            double_shift: DoubleShift::default(),
            feedback_ask: None,
            feedback_dir: None,
            feedback_saving: VecDeque::new(),
            graphics: None,
            report_view: None,
        }
    }

    /// With another trash (tests: not the system's).
    pub fn with_trash(mut self, trash: crate::trash::Trash) -> Self {
        self.trash = trash;
        self
    }

    /// With another way to tell the free space of a drive (tests).
    pub fn with_free_space(mut self, free: crate::disk::FreeSpace) -> Self {
        self.free_space = free;
        self
    }

    /// With another projects folder for the dialogs to start in (tests: not the home folder).
    pub fn with_projects_home(mut self, dir: Option<PathBuf>) -> Self {
        self.projects_home = dir;
        self
    }

    /// With another folder for the reports of „Melden“ (tests: never the app's own); `None`:
    /// „Melden“ cannot save.
    pub fn with_feedback_dir(mut self, dir: Option<PathBuf>) -> Self {
        self.feedback_dir = dir;
        self
    }

    /// The folder the dialogs of `kind` open in: the one they were last used in if it is still
    /// there, else the projects folder (`~/Driftless Projekte`, made now if missing).
    pub fn dialog_folder(&self, kind: DialogFolder) -> Option<PathBuf> {
        crate::places::start_folder(self.folders.get(kind), self.projects_home.clone())
    }

    /// Remembers where a dialog of `kind` was used: the folder `dir`.
    pub fn remember_folder(&mut self, kind: DialogFolder, dir: Option<&Path>) {
        self.folders.set(kind, dir);
    }

    /// A folder dialog of `kind` with its title, opening in [`DriftlessApp::dialog_folder`].
    fn dialog(&self, kind: DialogFolder, title: &str) -> rfd::FileDialog {
        let dialog = rfd::FileDialog::new().set_title(title);
        match self.dialog_folder(kind) {
            Some(dir) => dialog.set_directory(dir),
            None => dialog,
        }
    }

    /// With another way to tell cloud storage (tests).
    pub fn with_cloud_check(mut self, check: CloudCheck) -> Self {
        self.cloud_check = check;
        self
    }

    /// With the log file of this start: what the window says goes into it as well.
    pub fn with_log_file(mut self, log: Arc<LogFile>) -> Self {
        self.log_file = Some(log);
        self
    }

    /// For eframe: writes the start line into the log file `log` (with the graphics adapter
    /// eframe chose and the project to open), opens the project of `args`, else the project
    /// opened last, and starts generating the synthetic cloud if asked for. The reports of
    /// „Melden“ go into the app's own folder ([`crate::feedback::default_dir`]).
    pub fn from_creation_context(
        cc: &eframe::CreationContext<'_>,
        args: Args,
        log: Option<Arc<LogFile>>,
    ) -> Self {
        let ctx = cc.egui_ctx.clone();
        super::theme::apply(&cc.egui_ctx);
        let mut app = DriftlessApp::new(Arc::new(move || ctx.request_repaint()));
        app.log_file = log;
        app.feedback_dir = crate::feedback::default_dir();
        app.themed = true;
        app.render_state = cc.wgpu_render_state.clone();
        app.restore_ui(&cc.egui_ctx, cc.storage);
        let last = cc
            .storage
            .and_then(|s| s.get_string(LAST_PROJECT_KEY))
            .filter(|s| !s.is_empty());
        // kept: a report of „Melden“ names it too
        app.graphics = cc.wgpu_render_state.as_ref().map(graphics_of);
        if let Some(file) = &app.log_file {
            let project = args.project.clone().or(last.clone().map(PathBuf::from));
            file.write_start(app.graphics.as_deref(), project.as_deref());
        }
        if let Some(dir) = args.project {
            app.apply(Action::OpenProject(dir));
        } else {
            app.restore_last(last.as_deref());
        }
        if let Some(n) = args.synthetic {
            app.load_synthetic(n);
        }
        app
    }

    /// With the GPU of a window (or of a test renderer): the 3D view draws with it.
    pub fn with_render_state(mut self, render_state: egui_wgpu::RenderState) -> Self {
        self.graphics = Some(graphics_of(&render_state));
        self.render_state = Some(render_state);
        self
    }

    /// Starts generating a synthetic cloud of `n` points in the background.
    pub fn load_synthetic(&mut self, n: usize) {
        self.load(CloudSource::Synthetic(n));
    }

    /// The cloud shown in the 3D view.
    pub fn cloud(&self) -> Option<&LoadedCloud> {
        self.cloud.as_ref()
    }

    pub fn view(&self) -> &View3d {
        &self.view
    }

    /// Where the camera images of the scans lie in their bags, as the window keeps it for its
    /// touch-ups.
    pub fn image_indices(&self) -> &ImageIndices {
        &self.image_indices
    }

    /// Where the panels and the floating bars were in the last frame.
    pub fn layout(&self) -> &ScreenLayout {
        &self.layout
    }

    /// The 3D view (tests and tools move its camera).
    pub fn view_mut(&mut self) -> &mut View3d {
        &mut self.view
    }

    /// The run drawn in the view.
    pub fn overlay(&self) -> Option<&RunOverlay> {
        self.overlay.as_ref()
    }

    /// The pose `pose` of the drawn run was clicked: in the mode „Stellen wählen“ it sets
    /// the next place of the corrections (what the view's [`ViewEvent::PickedPose`] does).
    pub fn pose_picked(&mut self, pose: usize) {
        let Some(o) = &self.overlay else {
            return;
        };
        if pose < o.poses.len() && self.state.correct.run.as_ref() == Some(&o.run) {
            let t = o.time_of(pose);
            self.state.correct.place(t);
        }
    }

    /// The poses of the places A and B of the corrections, to mark them, and while B is
    /// chosen the submap of A (a click at a crossing prefers another one).
    fn places(&self) -> ([Option<usize>; 2], Option<usize>) {
        let (Some(o), c) = (&self.overlay, &self.state.correct) else {
            return ([None; 2], None);
        };
        if c.run.as_ref() != Some(&o.run) {
            return ([None; 2], None);
        }
        let places = [c.a.t, c.b.t].map(|t| t.and_then(|t| o.pose_at(t)));
        let avoid = (c.next == super::correct::Place::B)
            .then_some(places[0])
            .flatten()
            .map(|a| o.submap_of(a));
        (places, avoid)
    }

    /// Follows the selection: the run of the selection, else of the cloud, is drawn and its
    /// corrections are shown; reading them is queued for the helper. A model and its surfaces
    /// have no run of their own: none is drawn over them, not even that of the cloud before.
    fn sync_run(&mut self) {
        let wanted = match self.state.selected_run() {
            Some((_, r)) => Some(r.dir.clone()),
            None if self.state.selected_model().is_some() => None,
            None => self.cloud_run.clone(),
        };
        if self.state.correct.for_run(wanted.as_deref()) {
            if self.overlay.take().is_some() {
                self.overlay_version += 1;
            }
            self.helper_queue
                .retain(|t| !matches!(t, HelperTask::LoadRun(_)));
            if let Some(run) = wanted {
                self.helper_queue.push_back(HelperTask::LoadRun(run));
            }
        }
    }

    /// Reads the drawn run again (after it was recomputed).
    fn reload_run(&mut self, run: &Path) {
        if self.state.correct.run.as_deref() == Some(run) {
            let task = HelperTask::LoadRun(run.to_path_buf());
            if !self.helper_queue.contains(&task) {
                self.helper_queue.push_back(task);
            }
        }
    }

    fn run_loaded(&mut self, loaded: RunLoaded) {
        if self.state.correct.run.as_ref() != Some(&loaded.run) {
            return;
        }
        // a run without its report: the tree says so („unvollständig – neu rechnen“)
        let incomplete = self
            .state
            .run(&loaded.run)
            .is_some_and(|(_, r)| r.incomplete);
        self.state.correct.corrections = Some(loaded.corrections);
        let c = &mut self.state.correct;
        (c.auto_loops, c.submap_starts) = match &loaded.overlay {
            Ok(o) => (
                super::correct::auto_loops(&o.report),
                o.report.submaps.iter().map(|m| m.t_start).collect(),
            ),
            Err(_) => (Vec::new(), Vec::new()),
        };
        match loaded.overlay {
            Ok(mut o) => {
                o.set_origin(self.view_origin());
                self.overlay = Some(o);
                self.compare_overlay();
            }
            Err(e) => {
                self.overlay = None;
                if !incomplete {
                    self.error(e);
                }
            }
        }
        self.overlay_version += 1;
    }

    /// A change of the corrections ended: its answer goes into the panel (a refusal also into
    /// the Protokoll and the log file). An automatic loop switched on or off recomputes the
    /// run at once if no step runs (else „Jetzt neu rechnen“ waits).
    fn corrected(&mut self, action: &Action, result: Result<JobOutput, StepError>) {
        let c = &mut self.state.correct;
        c.busy = false;
        let (Action::AddLoop { run, .. }
        | Action::SetLoopEnabled { run, .. }
        | Action::SetLoopUsed { run, .. }) = action
        else {
            return;
        };
        let shown = c.run.as_ref() == Some(run);
        match result {
            Ok(JobOutput::Corrected(done)) => {
                if done.changed {
                    c.changed.insert(done.run.clone());
                    *c.writes.entry(done.run.clone()).or_default() += 1;
                }
                if shown {
                    if done.added {
                        c.clear_places();
                    }
                    c.corrections = Some(Ok(done.corrections));
                    c.message = Some(UiMessage::Info(text::shown(&done.message).into_owned()));
                }
                let switched = matches!(action, Action::SetLoopUsed { .. });
                if switched && done.changed && !self.jobs.is_busy() {
                    self.apply(Action::Reuse(done.run));
                }
            }
            Ok(_) => {}
            Err(e) if shown => {
                let m = UiMessage::Error(text::shown(&text::german(&e.to_string())).into_owned());
                self.note(&m);
                self.state.correct.message = Some(m);
            }
            Err(e) => self.error(e.to_string()),
        }
    }

    /// A warning or an error shown in a panel, not on the status bar: into the Protokoll and
    /// the log file as well.
    fn note(&mut self, m: &UiMessage) {
        let (level, kind) = match m {
            UiMessage::Info(_) => return,
            UiMessage::Warning(_) => (Level::Warning, LogKind::Warning),
            UiMessage::Error(_) => (Level::Error, LogKind::Error),
        };
        if let Some(file) = &self.log_file {
            file.write(level, m.text());
        }
        self.protocol(kind, m.text());
    }

    /// Queues a change of the corrections (one at a time).
    fn change_corrections(&mut self, action: Action) {
        let c = &mut self.state.correct;
        if c.busy {
            return;
        }
        c.busy = true;
        c.message = None;
        self.helper_queue.push_back(HelperTask::Correct(action));
    }

    /// Follows the run drawn in the view: a new one of the project gets the preset comparison
    /// ([`default_refs`]); maps of another scan are no longer chosen. A run not (yet) in the
    /// project, e.g. that of a cloud loaded before, is looked for again on the next frame.
    fn sync_compare(&mut self) {
        let wanted = self.state.correct.run.clone();
        if self.state.compare.run == wanted {
            return;
        }
        let found = wanted.as_deref().and_then(|dir| self.state.run(dir));
        let Some((scan, run)) = found else {
            self.state.compare.run = None;
            return;
        };
        let maps: Vec<PathBuf> = scan
            .runs
            .iter()
            .flat_map(|r| &r.maps)
            .map(|m| m.dir.clone())
            .collect();
        let preset = default_refs(run);
        let c = &mut self.state.compare;
        c.run = wanted;
        if [&c.map_a, &c.map_b]
            .iter()
            .any(|m| m.as_ref().is_some_and(|m| !maps.contains(m)))
        {
            c.map_a = None;
            c.map_b = None;
            c.maps = None;
        }
        if let Some((a, b)) = preset {
            self.start_compare(a, b);
        }
    }

    /// Chooses A and B and queues their comparison; the orange line goes until it is ready.
    fn start_compare(&mut self, a: TrajRef, b: TrajRef) {
        let c = &mut self.state.compare;
        c.a = Some(a.clone());
        c.b = Some(b.clone());
        c.result = None;
        self.helper_queue
            .retain(|t| !matches!(t, HelperTask::Compare(..)));
        self.helper_queue.push_back(HelperTask::Compare(a, b));
        self.compare_overlay();
    }

    fn start_compare_maps(&mut self, a: PathBuf, b: PathBuf) {
        let c = &mut self.state.compare;
        c.map_a = Some(a.clone());
        c.map_b = Some(b.clone());
        c.maps = None;
        self.helper_queue
            .retain(|t| !matches!(t, HelperTask::CompareMaps(..)));
        self.helper_queue.push_back(HelperTask::CompareMaps(a, b));
    }

    /// Draws A of the comparison over the run in the view if B belongs to that run.
    fn compare_overlay(&mut self) {
        let Some(o) = &mut self.overlay else {
            return;
        };
        let shown = self.state.compare.current().filter(|c| c.b.run() == o.run);
        if shown.is_none() && o.compare.is_empty() {
            return;
        }
        o.set_comparison(shown);
        self.overlay_version += 1;
    }

    /// True while a cloud is loaded or waits to be.
    pub fn is_loading(&self) -> bool {
        self.loader.is_busy() || self.next_load.is_some()
    }

    /// Loads `source` in the background; a load that runs is cancelled first. The cloud
    /// shown stays until the new one is ready. The cloud shown or being loaded is not
    /// loaded again.
    fn load(&mut self, source: CloudSource) {
        self.load_with(source, false);
    }

    /// [`DriftlessApp::load`]; with `force` also the cloud shown (its file was rewritten).
    fn load_with(&mut self, source: CloudSource, force: bool) {
        let path = source.path();
        let cancelling = self.loader.status().is_some_and(|s| s.cancelling);
        if self.loading.as_ref().map(CloudSource::path) == Some(path.clone()) {
            // The same load again: keep it running, or, if it is being cancelled for
            // another one, start it again once it has stopped.
            self.next_load = cancelling.then_some(source);
            return;
        }
        if !force && self.cloud.as_ref().is_some_and(|c| c.path == path) {
            // Back to the cloud shown: a load of another one is not needed any more.
            self.next_load = None;
            self.loader.cancel();
            return;
        }
        if self.loader.is_busy() {
            self.loader.cancel();
            self.next_load = Some(source);
            return;
        }
        self.next_load = None;
        let label = match &source {
            CloudSource::File { .. } | CloudSource::Pairs { .. } => text::LOAD_CLOUD,
            CloudSource::Synthetic(_) => text::SYNTHETIC_CLOUD,
        };
        let job = source.clone();
        let started = self.loader.start(label, move |p| {
            let cloud = match job {
                CloudSource::File { path, run } => load_cloud(&path, run.as_deref(), p)?,
                CloudSource::Synthetic(n) => {
                    p.phase(text::PHASE_SYNTHETIC, None);
                    let t = std::time::Instant::now();
                    let points = synthetic_cloud(n, SYNTHETIC_SEED);
                    let seconds = t.elapsed().as_secs_f64();
                    LoadedCloud::build(PathBuf::from(text::synthetic_name(n)), points, seconds, p)?
                }
                CloudSource::Pairs {
                    name,
                    model_las,
                    lone,
                    map_dir,
                    member,
                } => {
                    let scan = pair_scan_cloud(&map_dir).ok_or(text::PAIRS_NO_CLOUD)?;
                    let done = load_pair_cloud(name, &model_las, lone, &scan, member, p)?;
                    return Ok(JobOutput::PairCloud(Box::new(done)));
                }
            };
            Ok(JobOutput::Cloud(Box::new(cloud)))
        });
        match started {
            Ok(()) => self.loading = Some(source),
            Err(e) => self.error(e.to_string()),
        }
        self.state.loading = self.loader.status();
    }

    /// Takes a loaded cloud and starts the load that waited.
    fn tick_loader(&mut self) {
        if let Some(f) = self.loader.poll() {
            let source = self.loading.take();
            let pairs_load = matches!(source, Some(CloudSource::Pairs { .. }));
            match f.result {
                Ok(JobOutput::Cloud(cloud)) => {
                    // another cloud in the view ends setting point pairs
                    tools::end_pairs(&mut self.state);
                    self.show_cloud(source, *cloud);
                }
                Ok(JobOutput::PairCloud(done)) => {
                    let name = done.cloud.path.clone();
                    self.show_cloud(source, done.cloud);
                    match &mut self.state.pairs {
                        Some(s) if s.cloud == name => s.shift = Some(done.shift),
                        _ => {}
                    }
                }
                Ok(_) => {}
                // a cancel for the next load needs no message
                Err(StepError::Cancelled) if self.next_load.is_some() => {}
                Err(e) => {
                    // model and scan cannot be shown: no pairs can be set
                    if pairs_load {
                        tools::end_pairs(&mut self.state);
                    }
                    match e {
                        StepError::Cancelled => self.info(text::job_cancelled(&f.label)),
                        e => self.error(text::job_failed(&f.label, &text::step_error(&e))),
                    }
                }
            }
        }
        if !self.loader.is_busy() {
            if let Some(next) = self.next_load.take() {
                self.load(next);
            }
        }
        self.state.loading = self.loader.status();
    }

    /// Shows `cloud`, loaded from `source`, in the view in place of the one shown.
    fn show_cloud(&mut self, source: Option<CloudSource>, cloud: LoadedCloud) {
        // a surface of another map would hide this cloud: it goes; one of the same map (or
        // model) stays, and the cloud just picked is what is drawn
        let map = match &source {
            Some(CloudSource::File { path, .. }) => path.parent(),
            Some(CloudSource::Pairs { name, .. }) => name.parent(),
            _ => None,
        };
        if self.surface_of(map) {
            self.state.view.show = ShowMode::Cloud;
        } else {
            self.drop_surface();
        }
        self.cloud_run = match source {
            Some(CloudSource::File { run, .. }) => run,
            _ => None,
        };
        if let Some(o) = &mut self.overlay {
            o.set_origin(Some(cloud.tree.origin));
            self.overlay_version += 1;
        }
        self.info(text::cloud_loaded(
            &cloud.path,
            cloud.points.len(),
            cloud.load_seconds,
            cloud.build_seconds,
        ));
        self.state.cloud = Some(CloudSummary {
            path: cloud.path.clone(),
            points: cloud.points.len(),
            has_rgb: cloud.points.rgb.is_some(),
            has_scans: !cloud.points.source.is_empty(),
            quality_known: cloud.points.quality_known,
            z_range: cloud.z_range,
            ceiling: cloud.ceiling,
            point_m: cloud.point_m,
        });
        self.state.view.cut = None;
        self.state.picked = None;
        self.state.view_stats = None;
        self.cloud = Some(cloud);
        self.view.cloud_replaced();
    }

    /// „Von Hand ausrichten“ of scan `member` of the model in `dir`: model and scan are
    /// loaded side by side, coloured by scan, and the tool „Punktpaare“ is chosen. The
    /// session starts anew, its cloud is loaded anew.
    fn start_pairs(&mut self, dir: &Path, member: usize) {
        let Some(model) = self.state.model(dir).cloned() else {
            self.error(text::unknown_model(dir));
            return;
        };
        let session = model.report.members.get(member).and_then(|m| {
            let name = text::model_member(member + 1, &self.state.scan_name(&m.scan));
            PairSession::new(&model, member, name)
        });
        let (Some(session), Ok(nr)) = (session, u16::try_from(member)) else {
            self.error(text::unknown_model(dir));
            return;
        };
        let lone = model.report.members.iter().position(|m| m.included);
        let source = CloudSource::Pairs {
            name: session.cloud.clone(),
            model_las: model.dir.join(MODEL_LAS),
            lone: lone.and_then(|k| u16::try_from(k).ok()).unwrap_or(0),
            map_dir: model.report.members[member].map_dir.clone(),
            member: nr,
        };
        self.hand_loads += 1;
        self.state.selection = Selection::Model(model.dir.clone());
        self.state.view.color_mode = crate::render::ColorMode::Scan;
        self.state.pairs = Some(session);
        tools::select(&mut self.state, Tool::Pairs);
        self.load_with(source, true);
    }

    /// Queues the switch of a point pair (one at a time).
    fn switch_pair(&mut self, index: usize, enabled: bool) {
        if self.state.pairs_writing {
            return;
        }
        self.state.pairs_writing = true;
        self.helper_queue
            .push_back(HelperTask::SwitchPair(index, enabled));
    }

    /// The origin the view draws in: the cloud's, else the surface's (`None`: the run's own).
    fn view_origin(&self) -> Option<[f64; 3]> {
        let surface = self.state.surface.as_ref().map(|s| s.origin);
        self.cloud.as_ref().map(|c| c.tree.origin).or(surface)
    }

    /// Whether the surface shown, and one being loaded, belong to the map folder `map`: a
    /// surface folder lies in its map folder (`karte-NNN/oberflaeche-NNN/oberflaeche.glb`).
    fn surface_of(&self, map: Option<&Path>) -> bool {
        let in_map = |glb: &Path| map.is_some() && glb.parent().and_then(Path::parent) == map;
        let shown = self.state.surface.as_ref().is_none_or(|s| in_map(&s.path));
        let loading = self.surface_loading.as_ref().is_none_or(|(p, _)| in_map(p))
            && self
                .next_surface
                .as_deref()
                .is_none_or(|p| in_map(&surface_glb(p)));
        shown && loading
    }

    /// Takes the surface out of the view, and stops loading one.
    fn drop_surface(&mut self) {
        self.next_surface = None;
        if self.surface_loader.is_busy() {
            self.surface_loader.cancel();
            self.discard_surface = true;
        }
        if self.state.surface.take().is_some() || self.view.has_surface() {
            self.view.set_surface(None);
        }
        self.surface_stamp = None;
        // a stroke on it ends where it got to
        let stroke = self.brush.set_surface(None);
        self.state.brush.active = None;
        self.state.touchup.shown = None;
        self.state.touchup.edited = false;
        self.state.touchup.variants.clear();
        self.state.touchup.outdated = false;
        if let Some(stroke) = stroke {
            self.apply(stroke);
        }
    }

    /// Whether a surface is being loaded.
    pub fn is_loading_surface(&self) -> bool {
        self.surface_loader.is_busy() || self.next_surface.is_some()
    }

    /// Loads the surface at `path` (a `.glb`, an `.obj` or a surface folder) in the
    /// background; it replaces the surface shown once ready. A surface load that runs is
    /// cancelled.
    fn load_surface(&mut self, path: &Path) {
        self.load_surface_with(path, false);
    }

    /// [`DriftlessApp::load_surface`]; the surface shown or being loaded is loaded again only
    /// if its file changed since (`driftless surface -o` into the same folder) or with
    /// `force` (picked again from the menu).
    fn load_surface_with(&mut self, path: &Path, force: bool) {
        let glb = surface_glb(path);
        let stamp = file_stamp(&glb);
        let same = |p: &PathBuf, s: &FileStamp| *p == glb && *s == stamp && stamp.is_some();
        let loading = self
            .surface_loading
            .as_ref()
            .is_some_and(|(p, s)| same(p, s));
        let shown = self.state.surface.as_ref().map(|s| &s.path) == Some(&glb)
            && same(&glb, &self.surface_stamp)
            && !self.surface_loader.is_busy();
        if !force && self.next_surface.is_none() && (loading || shown) {
            return;
        }
        if self.surface_loader.is_busy() {
            self.surface_loader.cancel();
            self.next_surface = Some(glb);
            return;
        }
        let job = glb.clone();
        let started = self.surface_loader.start(text::LOAD_SURFACE, move |p| {
            Ok(JobOutput::Surface(Box::new(load_mesh(&job, p)?)))
        });
        match started {
            Ok(()) => self.surface_loading = Some((glb, stamp)),
            Err(e) => self.error(e.to_string()),
        }
    }

    /// Hands a loaded surface to the view.
    fn tick_surface_loader(&mut self) {
        if let Some(f) = self.surface_loader.poll() {
            self.surface_finished(f);
        }
        if !self.surface_loader.is_busy() {
            if let Some(next) = self.next_surface.take() {
                self.load_surface(&next);
            }
        }
        if self.state.loading.is_none() {
            self.state.loading = self.surface_loader.status();
        }
    }

    fn surface_finished(&mut self, f: JobFinished) {
        let (path, stamp) = self.surface_loading.take().unwrap_or_default();
        if std::mem::take(&mut self.discard_surface) {
            return;
        }
        match f.result {
            Ok(JobOutput::Surface(surface)) => {
                let mut surface = *surface;
                self.surface_stamp = stamp;
                let (triangles, pages) = (surface.triangles(), surface.pages.len());
                self.info(text::surface_loaded(&path, triangles, pages, f.seconds));
                let z_range = surface.bounds().map_or([0.0; 2], |(lo, hi)| [lo[2], hi[2]]);
                // the Stelle chosen belongs to the map or model of the surface before
                let owner = |p: &Path| brush::owner_of(p).map(Path::to_path_buf);
                let before = self.state.surface.as_ref().and_then(|s| owner(&s.path));
                if before != owner(&path) {
                    self.state.brush.active = None;
                }
                let pick = surface.pick.take();
                let touchup = surface.touchup.take();
                // a stroke on the surface before ends where it got to
                let stroke = self
                    .brush
                    .set_surface(pick.map(|pick| (path.clone(), surface.origin, pick)));
                self.state.surface = Some(SurfaceSummary {
                    path,
                    triangles,
                    pages,
                    origin: surface.origin,
                    z_range,
                });
                if self.cloud.is_none() {
                    // the surface's origin becomes the view's: the cut starts anew, the run
                    // moves along
                    self.state.view.cut = None;
                    if let Some(o) = &mut self.overlay {
                        o.set_origin(Some(surface.origin));
                        self.overlay_version += 1;
                    }
                }
                self.view.set_surface(Some(surface));
                // the surface just picked is what is drawn; a computed one only in „Bilder“,
                // the other steps keep showing the points
                if std::mem::take(&mut self.surface_by_hand) || self.state.step == Step::Bilder {
                    self.state.view.show = ShowMode::Surface;
                }
                self.surface_touchup(touchup);
                if let Some(stroke) = stroke {
                    self.apply(stroke);
                }
            }
            Ok(_) => {}
            // a cancel for the next load needs no message
            Err(StepError::Cancelled) if self.next_surface.is_some() => {}
            Err(StepError::Cancelled) => self.info(text::job_cancelled(&f.label)),
            Err(e) => self.error(text::job_failed(&f.label, &text::step_error(&e))),
        }
    }

    /// The touch-up of the surface just shown, as the loader found it: the view gets the
    /// touched-up state at once if its cache fits; else (or if it was computed without the raw
    /// data that are back) the touch-up is queued, to say what came of it. Only for a surface of
    /// a map or model of the project.
    fn surface_touchup(&mut self, touchup: Option<crate::surface::LoadedTouchup>) {
        self.state.touchup.load += 1;
        self.state.touchup.shown = None;
        self.state.touchup.edited = false;
        self.state.touchup.variants.clear();
        self.state.touchup.outdated = false;
        let (Some(touchup), Some(dir)) = (
            touchup,
            brush::shown_surface(&self.state).map(Path::to_path_buf),
        ) else {
            return;
        };
        if touchup.edit.is_some() {
            self.set_view_edit(touchup.edit);
        }
        if touchup.stale {
            self.want_touchup(dir, None, true);
        }
    }

    /// Loads the cloud or surface of `selection` if it is one, a cloud with the quality of
    /// its run; for a map its best cloud, in „Bilder“ its newest fitting surface if it has one.
    fn load_selected(&mut self) {
        match self.state.selection.clone() {
            Selection::Map(dir) => {
                let surface = self
                    .state
                    .map(&dir)
                    .filter(|_| self.state.step == Step::Bilder)
                    .and_then(|(_, _, m)| m.surfaces.iter().rev().find(|s| !s.stale))
                    .map(|s| s.dir.clone());
                match surface {
                    Some(surface) => {
                        self.surface_by_hand = true;
                        self.load_surface(&surface);
                    }
                    None => self.load_best_of_map(&dir),
                }
            }
            Selection::Cloud(path) if self.is_model_cloud(&path) => self.load_model_cloud(path),
            Selection::Cloud(path) => {
                let run = self.state.selected_run().map(|(_, r)| r.dir.clone());
                self.load(CloudSource::File { path, run });
            }
            Selection::Surface(dir) => self.load_surface(&dir),
            Selection::Model(dir) => self.load_model_cloud(dir.join(MODEL_LAS)),
            _ => {}
        }
    }

    /// Whether `path` is the `modell.las` of a model of the project.
    fn is_model_cloud(&self, path: &Path) -> bool {
        self.state
            .models
            .iter()
            .any(|m| m.dir.join(MODEL_LAS) == path)
    }

    /// Loads the `modell.las` of a model and colours it by scan, once: another colour chosen
    /// afterwards stays until a model is picked again.
    fn load_model_cloud(&mut self, path: PathBuf) {
        self.state.view.color_mode = crate::render::ColorMode::Scan;
        self.load(CloudSource::File { path, run: None });
    }

    /// Loads the best cloud of the map in `dir` (colour, else clean, else dense), with the
    /// quality of its run; the map stays selected.
    fn load_best_of_map(&mut self, dir: &Path) {
        let best = self.state.scans.iter().flat_map(|s| &s.runs).find_map(|r| {
            let map = r.maps.iter().find(|m| m.dir == dir)?;
            map.clouds
                .iter()
                .find(|c| c.file_name().is_none_or(|n| n != "removed.las"))
                .map(|c| (c.clone(), r.dir.clone()))
        });
        if let Some((path, run)) = best {
            self.load(CloudSource::File {
                path,
                run: Some(run),
            });
        }
    }

    /// Loads the `map.las` of the run in `dir`, if there is one, also when it is shown
    /// (a recomputed run rewrote it).
    fn load_run_cloud(&mut self, dir: &Path) {
        let cloud = self.state.run(dir).and_then(|(_, r)| r.cloud.clone());
        if let Some(path) = cloud {
            let source = CloudSource::File {
                path,
                run: Some(dir.to_path_buf()),
            };
            self.load_with(source, true);
        }
    }

    /// Opens the stored last project (an empty value is none), in the background. If it cannot
    /// be opened, the message is shown once and the value is forgotten
    /// ([`DriftlessApp::last_to_store`]).
    pub fn restore_last(&mut self, stored: Option<&str>) {
        let Some(last) = stored.filter(|s| !s.is_empty()) else {
            return;
        };
        self.read_folder(TreeTask::Open {
            dir: PathBuf::from(last),
            restore: true,
        });
    }

    /// Reads a project folder in the background for `task`; while a read runs, it is read once
    /// that has ended (the newest task wins, an open goes before a refresh).
    fn read_folder(&mut self, task: TreeTask) {
        if self.tree_loader.is_busy() {
            let open_waits = matches!(self.tree_next, Some(TreeTask::Open { .. }));
            if !(open_waits && matches!(task, TreeTask::Refresh(_))) {
                self.tree_next = Some(task);
            }
            // a read of another folder for an open is not wanted any more
            self.tree_discard = matches!(self.tree_next, Some(TreeTask::Open { .. }));
            self.sync_tree_state();
            return;
        }
        let dir = task.dir().to_path_buf();
        let cloud = self.cloud_check;
        let started = self.tree_loader.start(text::READ_PROJECT_JOB, move |_| {
            Ok(JobOutput::ProjectRead(Box::new(read_project(&dir, cloud)?)))
        });
        match started {
            Ok(()) => {
                self.tree_task = Some(task);
                self.tree_discard = false;
                self.tree_marks_at = self.marks_saved;
            }
            Err(e) => self.error(e.to_string()),
        }
        self.sync_tree_state();
    }

    /// Reads the open project again in the background (after a change, a step, the window
    /// coming to the front); its tree stays shown until the new one is there.
    fn refresh_tree(&mut self) {
        if let Some(dir) = self.state.project.as_ref().map(|p| p.dir.clone()) {
            self.read_folder(TreeTask::Refresh(dir));
        }
    }

    /// Whether a read of a project folder runs or waits.
    fn tree_pending(&self) -> bool {
        self.tree_loader.is_busy() || self.tree_next.is_some()
    }

    /// [`AppState::opening`] and [`AppState::tree_pending`] as the reads stand.
    fn sync_tree_state(&mut self) {
        let open = |t: &Option<TreeTask>| match t {
            Some(TreeTask::Open { dir, .. }) => Some(dir.clone()),
            _ => None,
        };
        let running = open(&self.tree_task).filter(|_| !self.tree_discard);
        self.state.opening = open(&self.tree_next).or(running);
        self.state.tree_pending = self.tree_pending();
    }

    /// Takes a project folder that has been read: an opened project is shown, a read of the
    /// open one replaces its tree; then a waiting read starts, and once none is left the end
    /// of a step that waited for it is carried out and the queue goes on.
    fn tick_tree(&mut self) {
        if let Some(f) = self.tree_loader.poll() {
            let task = self.tree_task.take();
            let discard = std::mem::take(&mut self.tree_discard);
            match (task, f.result) {
                (_, _) if discard => {}
                (Some(TreeTask::Open { .. }), Ok(JobOutput::ProjectRead(read))) => {
                    self.opened(*read)
                }
                (Some(TreeTask::Open { restore, .. }), Err(e)) => {
                    let e = text::step_error(&e);
                    if restore {
                        self.error(text::last_project_gone(&e));
                        self.forget_last = true;
                    } else {
                        self.error(e);
                    }
                }
                // a read of the project shown (another one may have been opened since)
                (Some(TreeTask::Refresh(dir)), result) if self.is_shown(&dir) => match result {
                    Ok(JobOutput::ProjectRead(read)) => {
                        let ProjectRead {
                            project,
                            tree,
                            cloud,
                            ..
                        } = *read;
                        self.state.project = Some(project);
                        // Stellen changed while the tree was read are newer than its
                        let changed =
                            self.marks_pending() || self.marks_saved != self.tree_marks_at;
                        let marks = changed.then(|| std::mem::take(&mut self.state.marks));
                        self.state.set_tree(tree);
                        if let Some(marks) = marks {
                            self.state.marks = marks;
                        }
                        self.tell_cloud(&dir, cloud);
                    }
                    Ok(_) => {}
                    Err(e) => self.error(text::step_error(&e)),
                },
                _ => {}
            }
            if let Some(next) = self.tree_next.take() {
                self.read_folder(next);
            }
        }
        self.sync_tree_state();
        if !self.tree_pending() {
            if let Some(end) = self.step_end.take() {
                self.finish_step(end);
            }
            self.advance_queue();
        }
    }

    /// An opened project: shown from now on, nothing selected; a queue of the project before
    /// stops, its measurements and surface go.
    fn opened(&mut self, read: ProjectRead) {
        self.stop_queue();
        self.state.touchup.waiting.clear();
        // the measurements belong to the project shown (they are not saved yet)
        self.state.measurements.clear();
        self.state.measure_a = None;
        self.drop_surface();
        let ProjectRead {
            project,
            tree,
            cloud,
            choices,
        } = read;
        let (name, dir) = (project.name.clone(), project.dir.clone());
        self.state.show_project(project, tree);
        // what was chosen in it last, else the defaults
        let choices = choices.unwrap_or_else(|| {
            Choices::of(
                &driftless_steps::map::MapSettings::default(),
                &crate::project::SurfaceSettings::default(),
            )
        });
        choices.apply(
            &mut self.state.map_settings,
            &mut self.state.surface_settings,
        );
        self.saved_choices = Some((
            dir.clone(),
            Choices::of(&self.state.map_settings, &self.state.surface_settings),
        ));
        self.info(text::project_opened(&name));
        self.tell_cloud(&dir, cloud);
    }

    /// Takes a measurement of the disk use; little free space ([`crate::disk::LOW_SPACE`]) is
    /// warned about once, again only after there was enough in between.
    fn disk_measured(&mut self, disk: crate::disk::DiskUse) {
        let free = disk.free;
        self.state.disk = Some(disk);
        match free {
            Some(free) if free < crate::disk::LOW_SPACE => {
                if !std::mem::replace(&mut self.low_space_told, true) {
                    self.warn(text::low_space(free));
                }
            }
            Some(_) => self.low_space_told = false,
            None => {}
        }
    }

    /// What the step `step` will write at most ([`crate::disk::need`]), from the raw data of
    /// its scans as last measured; `None` without a measurement or for a step that writes
    /// little.
    fn room_needed(&self, step: &Action) -> Option<u64> {
        use crate::disk::{need, NeedKind};
        let disk = self.state.disk.as_ref()?;
        let project = self.state.project.as_ref()?;
        let raw_of_scan = |path: &Path| disk.raw.get(path).copied().unwrap_or(0);
        let raw_of_id = |id: &str| project.scan(id).map_or(0, |e| raw_of_scan(&e.path));
        let map_voxel = self.state.map_settings.switches.voxel;
        let surface_voxel = self.state.surface_settings.config.voxel;
        let model_raw = |dir: &Path| {
            self.state.model(dir).map_or(0, |m| {
                m.report.members.iter().map(|m| raw_of_scan(&m.scan)).sum()
            })
        };
        Some(match step {
            Action::RunSlam(id) => need(NeedKind::Slam, raw_of_id(id), map_voxel),
            Action::RunMap(run) => {
                let (scan, _) = self.state.run(run)?;
                need(NeedKind::Map, raw_of_id(&scan.id), map_voxel)
            }
            Action::RunSurface(map) => {
                let (scan, _, _) = self.state.map(map)?;
                need(NeedKind::Surface, raw_of_id(&scan.id), surface_voxel)
            }
            Action::RunMerge { picks, .. } => {
                let raw = picks.iter().map(|p| raw_of_id(&p.scan_id)).sum();
                need(NeedKind::Model, raw, map_voxel)
            }
            Action::ApplyPairs { model, .. } => need(NeedKind::Model, model_raw(model), map_voxel),
            Action::RunModelSurface { model } => {
                need(NeedKind::Surface, model_raw(model), surface_voxel)
            }
            _ => return None,
        })
    }

    /// A step that started: if it will need more than is free on the drive of the project, a
    /// warning (it still runs).
    fn check_room(&mut self, step: &Action) {
        let free = self.state.disk.as_ref().and_then(|d| d.free);
        let (Some(free), Some(need)) = (free, self.room_needed(step)) else {
            return;
        };
        if need > free {
            let label = self
                .state
                .job
                .as_ref()
                .map_or_else(String::new, |j| j.label.clone());
            self.warn(text::needs_room(&label, need, free));
        }
    }

    /// Writes the precision, fineness and „ohne Fotos“ chosen into `auswahl.toml` of the
    /// project once they differ from what it holds (in the background, the newest only).
    fn keep_choices(&mut self) {
        let Some(dir) = self.state.project.as_ref().map(|p| p.dir.clone()) else {
            return;
        };
        if self.state.opening.is_some() {
            return;
        }
        let now = Choices::of(&self.state.map_settings, &self.state.surface_settings);
        if self
            .saved_choices
            .as_ref()
            .is_some_and(|(d, c)| *d == dir && *c == now)
        {
            return;
        }
        self.saved_choices = Some((dir.clone(), now.clone()));
        self.helper_queue
            .retain(|t| !matches!(t, HelperTask::SaveChoices(..)));
        self.helper_queue
            .push_back(HelperTask::SaveChoices(dir, now));
    }

    /// Warns once per session and project folder that it lies in cloud storage.
    fn tell_cloud(&mut self, dir: &Path, cloud: Option<String>) {
        if let Some(cloud) = cloud {
            if self.cloud_told.insert(dir.to_path_buf()) {
                self.warn(text::project_in_cloud(&cloud, dir));
            }
        }
    }

    /// Takes the layout stored under [`UI_KEY`]. Without one, or with one that cannot be read
    /// (an older program, another format), the defaults hold and egui's stored window state
    /// (panel widths, folded sections, windows of the old layout) is dropped, so that the
    /// first start after an update shows the new layout clean.
    pub fn restore_ui(&mut self, ctx: &egui::Context, storage: Option<&dyn eframe::Storage>) {
        let stored = storage
            .and_then(|s| s.get_string(UI_KEY))
            .and_then(|s| serde_json::from_str::<StoredUi>(&s).ok());
        match stored {
            Some(ui) => {
                self.state.step = ui.step;
                ui.apply_to(&mut self.state.view);
                self.state.gpu = ui.gpu();
                self.state.brush.radius = ui.brush_radius();
                self.folders = ui.folders;
            }
            None => ctx.memory_mut(|m| {
                m.data.clear();
                m.reset_areas();
            }),
        }
    }

    /// The layout to store under [`UI_KEY`].
    pub fn ui_to_store(&self) -> String {
        let ui = StoredUi::of(
            self.state.step,
            &self.state.view,
            self.state.gpu,
            &self.folders,
            self.state.brush.radius,
        );
        serde_json::to_string(&ui).unwrap_or_default()
    }

    /// What to store as the last project: the open one, empty to forget a gone one, `None`
    /// to keep what is stored. A folder whose path is not valid UTF-8 is not stored (the
    /// storage holds text, and a changed path would point elsewhere).
    pub fn last_to_store(&self) -> Option<String> {
        match &self.state.project {
            Some(p) => p.dir.to_str().map(str::to_string),
            None if self.forget_last => Some(String::new()),
            None => None,
        }
    }

    /// The window asks to close. While a step runs the close is put off (returns false):
    /// the step is cancelled and [`DriftlessApp::take_close`] says when to close, so the
    /// window never waits for it inside the event loop.
    pub fn close_requested(&mut self) -> bool {
        // a stroke or ↶ not yet in `stellen.toml`, a report not yet in its folder, are saved
        // first
        if !self.jobs.is_busy() && !self.marks_pending() && !self.feedback_pending() {
            return true;
        }
        if !self.closing {
            self.closing = true;
            if self.jobs.is_busy() {
                self.pending = None;
                self.stop_queue();
                self.jobs.cancel();
                self.info(text::CLOSE_AFTER_CANCEL.to_string());
            }
        }
        false
    }

    /// True once, after a put-off close, when the step has ended and the Stellen and the reports
    /// are saved.
    pub fn take_close(&mut self) -> bool {
        let saving = self.marks_pending() || self.feedback_pending();
        if self.closing && !self.jobs.is_busy() && !saving {
            self.closing = false;
            return true;
        }
        false
    }

    pub fn state(&self) -> &AppState {
        &self.state
    }

    /// For tests and the panels' settings.
    pub fn state_mut(&mut self) -> &mut AppState {
        &mut self.state
    }

    /// Starts a processing step (refused while one runs).
    pub fn start_job(
        &mut self,
        label: impl Into<String>,
        job: impl FnOnce(&Progress) -> Result<JobOutput, StepError> + Send + 'static,
    ) -> Result<(), StartError> {
        self.jobs.start(label, job)?;
        self.state.job = self.jobs.status();
        Ok(())
    }

    /// True while a step or background work runs: the window must then keep drawing.
    pub fn wants_repaint_while_busy(&self) -> bool {
        self.jobs.is_busy()
            || self.helper.is_busy()
            || !self.helper_queue.is_empty()
            || self.is_loading()
            || self.surface_loader.is_busy()
            || self.tree_pending()
            || self.step_end.is_some()
            || self.status_asked != self.state.tree_version
            || self.gaps_asked != self.state.tree_version
            || (self.state.project.is_some() && self.disk_asked != self.state.tree_version)
            || self.focus_read_at.is_some()
            || !self.state.touchup.waiting.is_empty()
            || self.report_view.is_some()
    }

    /// The window got the focus back: the tree is read again [`FOCUS_DELAY`] later (a scan
    /// folder may have come back, another window or program may have computed), at most once
    /// per [`FOCUS_GAP`].
    fn focus_back(&mut self) {
        let recent = self.focus_read.is_some_and(|t| t.elapsed() < FOCUS_GAP);
        if self.state.project.is_some() && self.focus_read_at.is_none() && !recent {
            self.focus_read_at = Some(Instant::now() + FOCUS_DELAY);
        }
    }

    /// Takes what the helper finished and starts its next task: queued ones first, then the
    /// map statuses when the tree changed.
    fn tick_helper(&mut self) {
        if let Some(f) = self.helper.poll() {
            match (self.helper_running.take(), f.result) {
                (Some(HelperTask::MapStatus(v)), Ok(JobOutput::MapStatus(status))) => {
                    // an older tree's statuses are dropped; the newer job follows
                    if v == self.state.tree_version {
                        self.state.set_map_status(*status);
                    }
                }
                (Some(HelperTask::HistoryGaps(_)), Ok(JobOutput::HistoryGaps(gaps))) => {
                    // states never change: gaps of an older tree stay valid
                    self.state.history_gaps.extend(*gaps);
                }
                (Some(HelperTask::LoadRun(_)), Ok(JobOutput::Run(loaded))) => {
                    self.run_loaded(*loaded);
                }
                (Some(HelperTask::Correct(action)), result) => self.corrected(&action, result),
                (Some(HelperTask::Compare(a, b)), result) => {
                    let c = &mut self.state.compare;
                    if c.a.as_ref() == Some(&a) && c.b.as_ref() == Some(&b) {
                        c.result = Some(match result {
                            Ok(JobOutput::Compare(done)) => Ok(*done),
                            Ok(_) => Err(text::NOT_A_STEP.into()),
                            Err(e) => Err(e.to_string()),
                        });
                        self.compare_overlay();
                    }
                }
                (Some(HelperTask::SwitchPair(index, enabled)), result) => {
                    self.pair_switched(index, enabled, result.map(|_| ()));
                }
                (Some(HelperTask::SaveChoices(..)), Err(e)) => self.error(e.to_string()),
                (Some(HelperTask::Relink { id, .. }), result) => self.relinked(&id, result),
                (Some(HelperTask::CleanUp(_)), Ok(JobOutput::CleanedUp { moved, failed })) => {
                    if !moved.is_empty() {
                        self.info(text::cleaned_up(moved.len(), moved.iter().sum()));
                    }
                    for e in failed {
                        self.error(e);
                    }
                    self.refresh_tree();
                }
                (Some(HelperTask::Trash { dir, label, bytes }), result) => match result {
                    Ok(_) => {
                        if self.selection_in(&dir) {
                            self.state.selection = Selection::None;
                        }
                        self.info(text::deleted(&label, bytes));
                        self.refresh_tree();
                    }
                    Err(e) => self.error(text::step_error(&e)),
                },
                (Some(HelperTask::DiskUse(v)), Ok(JobOutput::DiskUse(disk))) => {
                    // a measure of an older tree still says the free space
                    if v == self.state.tree_version || self.state.disk.is_none() {
                        self.disk_measured(*disk);
                    }
                }
                (Some(HelperTask::CompareMaps(a, b)), result) => {
                    let c = &mut self.state.compare;
                    if c.map_a.as_ref() == Some(&a) && c.map_b.as_ref() == Some(&b) {
                        c.maps = Some(match result {
                            Ok(JobOutput::MapCompare(done)) => Ok(*done),
                            Ok(_) => Err(text::NOT_A_STEP.into()),
                            Err(e) => Err(e.to_string()),
                        });
                    }
                }
                (Some(HelperTask::Marks { owner, .. }), result) => {
                    self.marks_written(owner, result)
                }
                (Some(HelperTask::Feedback(_)), result) => self.feedback_written(result),
                (Some(HelperTask::ReadReports(_)), result) => self.reports_read(result),
                _ => {}
            }
        }
        if self.helper.is_busy() {
            return;
        }
        let version = self.state.tree_version;
        let task = self
            .helper_queue
            .pop_front()
            .or_else(|| (self.status_asked != version).then_some(HelperTask::MapStatus(version)))
            .or_else(|| (self.gaps_asked != version).then_some(HelperTask::HistoryGaps(version)))
            .or_else(|| {
                let project = self.state.project.is_some() && self.state.opening.is_none();
                (project && self.disk_asked != version).then_some(HelperTask::DiskUse(version))
            });
        if let Some(task) = task {
            self.start_helper(task);
        }
    }

    fn start_helper(&mut self, task: HelperTask) {
        // what is shown or computed now: a step may have started since the click
        let blocked = match &task {
            HelperTask::Trash { dir, .. } => self.state.delete_blocked(dir),
            HelperTask::CleanUp(_) => self.state.clean_up_blocked().map(str::to_string),
            _ => None,
        };
        if let Some(why) = blocked {
            self.error(why);
            return;
        }
        let started = match &task {
            HelperTask::MapStatus(_) => {
                let job = self.state.map_status_job();
                self.helper.start(text::MAP_STATUS_JOB, move |_| {
                    Ok(JobOutput::MapStatus(Box::new(job())))
                })
            }
            HelperTask::HistoryGaps(_) => {
                let job = self.state.history_gaps_job();
                self.helper.start(text::HISTORY_GAPS_JOB, move |_| {
                    Ok(JobOutput::HistoryGaps(Box::new(job())))
                })
            }
            HelperTask::LoadRun(run) => {
                let run = run.clone();
                self.helper.start(text::LOAD_RUN_JOB, move |_| {
                    let overlay = RunOverlay::load(&run, None).map_err(|e| e.to_string());
                    let corrections = correct::load(&run).map_err(|e| e.to_string());
                    Ok(JobOutput::Run(Box::new(RunLoaded {
                        run,
                        overlay,
                        corrections,
                    })))
                })
            }
            HelperTask::Correct(action) => {
                let action = action.clone();
                self.helper.start(text::CORRECT_JOB, move |_| {
                    Ok(JobOutput::Corrected(Box::new(change_corrections(&action)?)))
                })
            }
            HelperTask::Compare(a, b) => {
                let (a, b) = (a.clone(), b.clone());
                self.helper.start(text::COMPARE_JOB, move |_| {
                    Ok(JobOutput::Compare(Box::new(compare_refs(&a, &b)?)))
                })
            }
            HelperTask::CompareMaps(a, b) => {
                let (a, b) = (a.clone(), b.clone());
                self.helper.start(text::MAPS_COMPARE_JOB, move |_| {
                    Ok(JobOutput::MapCompare(Box::new(compare_maps(&a, &b)?)))
                })
            }
            &HelperTask::SwitchPair(index, enabled) => {
                let project = self.state.project.clone();
                self.helper.start(text::PAIRS_JOB, move |_| {
                    let project = project.ok_or(text::NO_PROJECT)?;
                    set_point_pair_enabled(&project, index, enabled)?;
                    Ok(JobOutput::Done)
                })
            }
            HelperTask::DiskUse(_) => match self.state.disk_job(self.free_space) {
                Some(job) => self.helper.start(text::DISK_JOB, move |_| {
                    Ok(JobOutput::DiskUse(Box::new(job())))
                }),
                None => Ok(()),
            },
            HelperTask::Relink { id, dir } => {
                let project = self.state.project.as_ref().map(|p| p.dir.clone());
                let (id, dir) = (id.clone(), dir.clone());
                self.helper.start(text::RELINK_JOB, move |_| {
                    let project = project.ok_or(text::NO_PROJECT)?;
                    let (project, _, checked) =
                        change_project(&project, None, |p| p.relink_scan(&id, &dir))?;
                    Ok(JobOutput::Relinked {
                        project: Box::new(project),
                        checked,
                    })
                })
            }
            HelperTask::CleanUp(leftovers) => {
                let (leftovers, trash) = (leftovers.clone(), self.trash);
                self.helper.start(text::CLEAN_UP_JOB, move |_| {
                    let (mut moved, mut failed) = (Vec::new(), Vec::new());
                    for (path, bytes) in leftovers {
                        // a job that started in it, or finished results that came meanwhile,
                        // keep a folder where it is
                        if !crate::leftovers::may_clean_up(&path) {
                            continue;
                        }
                        match trash(&path) {
                            Ok(()) => moved.push(bytes),
                            Err(e) => failed.push(e),
                        }
                    }
                    Ok(JobOutput::CleanedUp { moved, failed })
                })
            }
            HelperTask::Trash { dir, .. } => {
                let (dir, trash) = (dir.clone(), self.trash);
                self.helper.start(text::TRASH_JOB, move |_| {
                    // a job of another window or program that started meanwhile keeps it
                    if let Some(holder) = crate::leftovers::busy_below(&dir) {
                        return Err(text::delete_busy(&holder).into());
                    }
                    trash(&dir)?;
                    Ok(JobOutput::Done)
                })
            }
            HelperTask::SaveChoices(dir, choices) => {
                let (dir, choices) = (dir.clone(), choices.clone());
                self.helper.start(text::CHOICES_JOB, move |_| {
                    choices.save(&dir)?;
                    Ok(JobOutput::Done)
                })
            }
            HelperTask::Marks { owner, change } => {
                let (owner, change) = (owner.clone(), change.clone());
                self.helper.start(text::MARKS_JOB, move |_| {
                    let (marks, _) = Marks::change(&owner, |m| Ok(change.apply(m)))?;
                    Ok(JobOutput::Marks(Box::new(marks)))
                })
            }
            HelperTask::Feedback(job) => {
                let job = (**job).clone();
                self.helper.start(text::FEEDBACK_JOB, move |_| {
                    use crate::feedback::{last_lines, write, LOG_LINES};
                    // the log file of this start, else what the window keeps of the Protokoll
                    let lines = job
                        .log_file
                        .as_deref()
                        .and_then(|path| last_lines(path, LOG_LINES).ok())
                        .filter(|lines| !lines.is_empty())
                        .unwrap_or(job.log);
                    let image = job.image.as_deref();
                    let folder = write(&job.dir, &job.report, image, job.marks.as_ref(), &lines)
                        .map_err(|e| StepError::Failed(e.to_string()))?;
                    Ok(JobOutput::FeedbackSaved(folder))
                })
            }
            HelperTask::ReadReports(dir) => {
                let dir = dir.clone();
                self.helper.start(text::REPORTS_JOB, move |_| {
                    Ok(JobOutput::Reports(crate::feedback::read_all(&dir)))
                })
            }
        };
        match started {
            Ok(()) => {
                match task {
                    HelperTask::MapStatus(v) => self.status_asked = v,
                    HelperTask::HistoryGaps(v) => self.gaps_asked = v,
                    HelperTask::DiskUse(v) => self.disk_asked = v,
                    _ => {}
                }
                // a measurement without a project started nothing
                self.helper_running = self.helper.is_busy().then_some(task);
            }
            // the map statuses are asked again at the next frame
            Err(_)
                if matches!(
                    task,
                    HelperTask::MapStatus(_) | HelperTask::HistoryGaps(_) | HelperTask::DiskUse(_)
                ) => {}
            Err(e) => match &task {
                HelperTask::Correct(action) => {
                    self.corrected(action, Err(StepError::Failed(e.to_string())))
                }
                HelperTask::Compare(..) => self.state.compare.result = Some(Err(e.to_string())),
                HelperTask::CompareMaps(..) => self.state.compare.maps = Some(Err(e.to_string())),
                &HelperTask::SwitchPair(index, enabled) => {
                    self.pair_switched(index, enabled, Err(StepError::Failed(e.to_string())))
                }
                // tried again with the next change
                HelperTask::SaveChoices(..) => self.saved_choices = None,
                HelperTask::Relink { id, .. } => {
                    let id = id.clone();
                    self.relinked(&id, Err(StepError::Failed(e.to_string())))
                }
                HelperTask::Marks { owner, .. } => {
                    let owner = owner.clone();
                    self.marks_written(owner, Err(StepError::Failed(e.to_string())))
                }
                HelperTask::Feedback(_) => {
                    self.feedback_written(Err(StepError::Failed(e.to_string())))
                }
                HelperTask::ReadReports(_) => {
                    self.reports_read(Err(StepError::Failed(e.to_string())))
                }
                _ => self.error(e.to_string()),
            },
        }
    }

    /// Changes the Stellen of the map or model in `owner`: at once on those shown
    /// ([`AppState::marks`]; a stroke chooses its Stelle if the surface of `owner` is shown),
    /// then in its `stellen.toml` in the background, in order. A change that changes nothing
    /// writes nothing; an unreadable file is changed not at all. A change of what the touch-up
    /// is computed from ([`brush::touchup_input`]: a step added or taken back, „Löschen“ of a
    /// Stelle with steps or ↶ of that) queues the touch-up of the surface shown, to run once it
    /// is saved – any change while „Nachher“ is behind ([`brush::TouchupState::outdated`]); if a
    /// step runs meanwhile, the window says that „Nachher“ follows after it. Whether anything
    /// changed.
    fn change_marks(&mut self, owner: PathBuf, change: MarkChange) -> bool {
        let shown = brush::shown_owner(&self.state) == Some(owner.as_path());
        let read = self
            .state
            .marks
            .entry(owner.clone())
            .or_insert_with(|| Ok(Marks::default()));
        let marks = match read {
            Ok(marks) => marks,
            Err(e) => {
                let e = text::marks_not_saved(e);
                self.error(e);
                return false;
            }
        };
        let before = marks.clone();
        let painted = change.apply(marks);
        let changed = *marks != before;
        let restep = brush::touchup_input(marks) != brush::touchup_input(&before);
        if marks.is_empty() {
            self.state.marks.remove(&owner);
        }
        if !changed {
            return false;
        }
        if shown && painted.is_some() {
            self.state.brush.active = painted;
        }
        self.helper_queue
            .push_back(HelperTask::Marks { owner, change });
        let behind = self.state.touchup.outdated;
        let surface = brush::shown_surface(&self.state).map(Path::to_path_buf);
        if let (Some(surface), true) = (surface, shown && (restep || behind)) {
            self.want_touchup(surface, None, false);
            if let Some(job) = self.jobs.status().filter(|_| restep) {
                self.info(text::touchup_after_job(&job.label));
            }
        }
        true
    }

    /// A step of Stelle `nr` of the surface shown ([`Action::Smooth`], [`Action::Retexture`]):
    /// into `stellen.toml` like a stroke, then the touch-up of the surface queued, to tell what
    /// came of the step. Refused with the reason while it cannot be done
    /// ([`brush::mark_steps_blocked`]).
    fn touchup_step(&mut self, nr: u32, change: MarkChange) {
        if let Some(why) = brush::mark_steps_blocked(&self.state, nr) {
            self.warn(why);
            return;
        }
        let (Some(owner), Some(surface)) = (
            brush::shown_owner(&self.state).map(Path::to_path_buf),
            brush::shown_surface(&self.state).map(Path::to_path_buf),
        ) else {
            return;
        };
        if !self.change_marks(owner.clone(), change) {
            self.warn(text::mark_gone(nr));
            return;
        }
        // the step just added: the youngest of the Stelle
        let seq = self
            .mark_of(&owner, nr)
            .and_then(|m| m.steps.last())
            .map(EditStep::seq);
        self.want_touchup(surface, seq.map(|seq| (nr, seq)), false);
    }

    /// Stelle `nr` of the map or model in `owner`, as shown ([`AppState::marks`]).
    fn mark_of(&self, owner: &Path, nr: u32) -> Option<&driftless_steps::marks::Mark> {
        let marks = self.state.marks.get(owner)?.as_ref().ok()?;
        marks.marks.iter().find(|m| m.nr == nr)
    }

    /// Queues the touch-up of the surface folder `surface`, once per surface: a wish for one
    /// that waits goes with it (keeping the step just added, `step`, and telling what came of
    /// it if either wish does, `tell`: see [`TouchupWish`]). It starts once the Stellen are
    /// saved and no step runs ([`Self::advance_touchup`]).
    fn want_touchup(&mut self, surface: PathBuf, step: Option<(u32, u64)>, tell: bool) {
        let waiting = &mut self.state.touchup.waiting;
        match waiting.iter_mut().find(|w| w.surface == surface) {
            Some(wish) => {
                wish.step = step.or(wish.step);
                wish.tell |= tell;
            }
            None => waiting.push_back(TouchupWish {
                surface,
                step,
                tell,
            }),
        }
    }

    /// Starts the touch-up that waits first, once the Stellen are saved, no step runs, the end
    /// of the last one has been carried out and no project switch waits. One of a surface no
    /// longer in the tree is dropped.
    fn advance_touchup(&mut self) {
        if self.jobs.is_busy()
            || self.marks_pending()
            || self.step_end.is_some()
            || self.pending.is_some()
            || self.closing
        {
            return;
        }
        let Some(TouchupWish {
            surface,
            step,
            tell,
        }) = self.state.touchup.waiting.pop_front()
        else {
            return;
        };
        if !self.state.all_surfaces().any(|s| s.dir == surface) {
            return;
        }
        let action = Action::Touchup {
            surface: surface.clone(),
            step,
            tell,
        };
        // what the window has at hand: the surface shown as the loader read it, the indices
        let prepared = TouchupPrepared {
            surface: self.brush.prepared(&surface.join(SURFACE_GLB)),
            indices: Some(Arc::clone(&self.image_indices)),
        };
        if let Err(e) = actions::start_touchup(&mut self.state, &mut self.jobs, &action, prepared) {
            self.error(e);
            if let Some((nr, seq)) = step {
                self.take_back_step(&surface, nr, seq);
            }
        }
    }

    /// Takes the step `seq` of Stelle `nr` of the map or model of the surface folder `surface`
    /// out of `stellen.toml` again – one that changed nothing or did not run to its end – if it
    /// is still there.
    fn take_back_step(&mut self, surface: &Path, nr: u32, seq: u64) -> bool {
        let Some(owner) = surface.parent().map(Path::to_path_buf) else {
            return false;
        };
        let there = self
            .mark_of(&owner, nr)
            .is_some_and(|m| m.steps.iter().any(|s| s.seq() == seq));
        there && self.change_marks(owner, MarkChange::DropStep { mark: nr, seq })
    }

    /// The touched-up state of the surface shown for the view (none: as computed; one that
    /// changes nothing is none to the view), with the places its marks lie in „Nachher“.
    fn set_view_edit(&mut self, edit: Option<SurfaceEdit>) {
        self.state.touchup.shown = edit.as_ref().map(|e| e.key);
        let (edit, moved) = match edit.filter(|e| !e.is_empty()) {
            Some(e) => (Some(e.edit), e.moved),
            None => (None, Vec::new()),
        };
        self.state.touchup.edited = edit.is_some();
        self.brush.set_moved(&moved);
        self.view.set_edit(edit);
        self.view.show_edit(self.state.touchup.after);
    }

    /// A touch-up has run: the view gets the touched-up state of the surface shown (current
    /// again: „Nachher“ is no longer behind), and what came of the step just added (`pressed`:
    /// Stelle, seq) is told – done (with the hints of the touch-up), or refused and taken out of
    /// `stellen.toml` again. A touch-up that applied the Stellen to a surface shown, exported or
    /// new (`tell`) and changed its cache says so, with the steps the surface refuses and the
    /// hints – images not baked anew with what is missing. Recomputing after ↶ or „Löschen“
    /// says nothing.
    fn touchup_done(&mut self, out: TouchupOutput, pressed: Option<(u32, u64)>, tell: bool) {
        let TouchupOutput {
            surface,
            done,
            no_sources,
            changed,
            step: outcome,
            view,
        } = out;
        let shown = brush::shown_surface(&self.state) == Some(surface.as_path());
        if shown {
            if let Some((load, edit)) = view {
                match edit {
                    ViewEdit::Set(edit) if load == self.state.touchup.load => {
                        self.set_view_edit(edit)
                    }
                    _ => {}
                }
            }
            self.state.touchup.outdated = false;
            self.state.touchup.variants = done
                .outcomes
                .iter()
                .filter_map(|o| Some(((o.mark, o.seq), variant(o)?)))
                .collect();
        }
        let owner = surface.parent().map(Path::to_path_buf).unwrap_or_default();
        // the step just added, unless it was taken back or its Stelle went meanwhile
        let pressed = pressed.and_then(|(nr, seq)| {
            let step = self
                .mark_of(&owner, nr)?
                .steps
                .iter()
                .find(|s| s.seq() == seq)?;
            Some((nr, *step))
        });
        if let (Some((nr, step)), Some(outcome)) = (pressed, &outcome) {
            let retexture = matches!(step, EditStep::Retexture { .. });
            let refused = match &outcome.refused {
                Some(why) => Some(text::touchup_refused(nr, why)),
                // no images to bake from: why (what is missing, or the surface is stale)
                None if retexture && (no_sources.is_some() || outcome.without_raw_data) => {
                    Some(match &no_sources {
                        Some(why) => text::no_images_baked(why),
                        None => text::RETEXTURE_NO_RAW_DATA.to_string(),
                    })
                }
                None => None,
            };
            match (refused, step) {
                (Some(why), _) => {
                    self.warn(why);
                    self.take_back_step(&surface, nr, step.seq());
                    return;
                }
                (None, EditStep::Smooth { level, .. }) => {
                    self.info(text::touchup_smoothed(nr, level))
                }
                (None, EditStep::Retexture { .. }) => {
                    self.info(text::touchup_retextured(nr, variant(outcome)))
                }
            }
            // the result is what one wants to see
            self.state.touchup.after = true;
            self.view.show_edit(true);
        } else if tell && changed && done.steps > 0 {
            // the Stellen applied to a surface of the map or model (anew)
            let label = self
                .state
                .all_surfaces()
                .find(|s| s.dir == surface)
                .map_or_else(|| surface.display().to_string(), |s| s.label.clone());
            self.info(text::touchup_applied(&label));
            for (nr, why) in &done.refused {
                self.info(text::touchup_refused(*nr, why));
            }
        } else {
            return;
        }
        self.tell_hints(&done.hints, no_sources.as_ref());
    }

    /// Tells the hints of a touch-up: that images were not baked anew says why, with
    /// `no_sources` (what is missing), instead of the bare hint; each once.
    fn tell_hints(&mut self, hints: &[String], no_sources: Option<&NoSources>) {
        let mut told: Vec<String> = Vec::new();
        for hint in hints {
            let hint = match no_sources {
                Some(why) if hint == NO_RAW_DATA || hint == STALE_SURFACE => {
                    text::no_images_baked(why)
                }
                _ => hint.clone(),
            };
            if !told.contains(&hint) {
                told.push(hint.clone());
                self.info(hint);
            }
        }
    }

    /// A change of the Stellen of `owner` was written (or refused): the file as it is now is
    /// shown, unless a later change of it is still to be written (it brings its own); a refusal
    /// is said, and the tree – with the Stellen as the file has them – is read again.
    fn marks_written(&mut self, owner: PathBuf, result: Result<JobOutput, StepError>) {
        self.marks_saved += 1;
        let later = self
            .helper_queue
            .iter()
            .any(|t| matches!(t, HelperTask::Marks { owner: o, .. } if *o == owner));
        match result {
            Ok(JobOutput::Marks(marks)) if !later => {
                if marks.is_empty() {
                    self.state.marks.remove(&owner);
                } else {
                    self.state.marks.insert(owner, Ok(*marks));
                }
            }
            Ok(_) => {}
            Err(e) => {
                self.error(text::marks_not_saved(&text::step_error(&e)));
                self.refresh_tree();
            }
        }
    }

    /// Whether a change of the Stellen is still to be written.
    fn marks_pending(&self) -> bool {
        let marks = |t: &HelperTask| matches!(t, HelperTask::Marks { .. });
        self.helper_running.as_ref().is_some_and(marks) || self.helper_queue.iter().any(marks)
    }

    /// Chooses Stelle `nr` of the surface shown and looks straight at it.
    fn frame_mark(&mut self, nr: u32) {
        self.state.brush.active = Some(nr);
        let (Some(origin), Some((centre, normal, size))) =
            (self.view_origin(), self.brush.place_of(&self.state, nr))
        else {
            return;
        };
        let view = self.layout.view;
        let aspect = if view.height() > 0.0 {
            view.width() / view.height()
        } else {
            1.0
        };
        let centre = relative_to(origin, centre);
        self.view
            .camera_mut()
            .frame_facing(centre, normal, brush::frame_radius(size), aspect);
    }

    /// A switch of point pair `index` ended: the pairs are read again, the answer goes into
    /// the status bar.
    fn pair_switched(&mut self, index: usize, enabled: bool, result: Result<(), StepError>) {
        self.state.pairs_writing = false;
        self.state.refresh_pairs();
        match result {
            Ok(()) => self.info(text::pair_switched(index + 1, enabled)),
            Err(e) => self.error(e.to_string()),
        }
    }

    /// „Melden“ (⇧⇧, „Melden …“ in the bar of the brush), about Stelle `mark` if given. With a
    /// GPU the picture of the window is asked for first, and the dialog opens in the frame it
    /// comes in ([`Self::feedback_picture`]); without one at once, without a picture. One at a
    /// time.
    fn ask_feedback(&mut self, mark: Option<u32>) {
        if self.state.feedback_dialog.is_some() || self.feedback_ask.is_some() {
            return;
        }
        if self.render_state.is_none() {
            self.open_feedback(mark, None);
            return;
        }
        self.feedback_ask = Some(FeedbackAsk { mark, asked: None });
    }

    /// While „Melden“ waits for the picture of the window: asks for it once; opens the dialog in
    /// the frame it comes in – painted after the frame that asked, so the dialog is not in it –
    /// or without it after [`SHOT_WAIT`].
    fn feedback_picture(&mut self, ctx: &egui::Context) {
        let Some(ask) = self.feedback_ask else {
            return;
        };
        let (time, image) = ctx.input(|i| {
            let image = i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot {
                    user_data, image, ..
                } if is_feedback_shot(user_data) => Some(image.clone()),
                _ => None,
            });
            (i.time, image)
        });
        let late = ask.asked.is_some_and(|at| time - at > SHOT_WAIT);
        if image.is_some() || late {
            self.feedback_ask = None;
            self.open_feedback(ask.mark, image);
            return;
        }
        if ask.asked.is_none() {
            let shot = egui::UserData::new(FeedbackShot);
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(shot));
            self.feedback_ask = Some(FeedbackAsk {
                asked: Some(time),
                ..ask
            });
        }
        // the next frames paint the picture and bring it
        ctx.request_repaint();
    }

    /// Opens „Melden“ with the picture of the window, the Stellen of the surface shown (ticked if
    /// there are any), the Stelle `mark` named if it is one of them, and what comes along by
    /// itself ([`Self::feedback_context`]).
    fn open_feedback(&mut self, mark: Option<u32>, image: Option<Arc<egui::ColorImage>>) {
        let marks = brush::shown_marks(&self.state)
            .and_then(|(_, marks)| marks)
            .cloned();
        let named = mark.and_then(|nr| {
            let m = marks.as_ref()?.marks.iter().find(|m| m.nr == nr)?;
            Some((nr, m.label()))
        });
        let context = self.feedback_context();
        self.state.feedback_dialog = Some(FeedbackDialog::new(context, image, marks, named));
    }

    /// What comes along with a report without the user: version, system, graphics adapter, the
    /// project, the step, the selection and the camera of the view.
    fn feedback_context(&self) -> Feedback {
        let mut f = Feedback::new(FeedbackKind::default());
        f.version = super::about::version_line();
        f.system = logfile::os_text();
        f.graphics = self.graphics.clone().unwrap_or_default();
        f.step = Some(super::feedback::step_word(self.state.step).to_string());
        f.project = self
            .state
            .project
            .as_ref()
            .map(|p| FeedbackProject::new(&p.name, &p.dir));
        f.selection = super::feedback::selection_record(&self.state);
        f.camera = self.camera_state().map(FeedbackCamera::new);
        f
    }

    /// The camera of the view in absolute coordinates, if the view draws something: in the frame
    /// it last drew in.
    fn camera_state(&self) -> Option<CameraState> {
        let shown = self.cloud.is_some() || self.overlay.is_some() || self.view.has_surface();
        let drawn = shown && self.render_state.is_some();
        let origin = self.view.view_origin().filter(|_| drawn)?;
        Some(self.view.camera().state(origin))
    }

    /// „Melden“ in the dialog: the report is written in the background as a folder of its own,
    /// with the last lines of the log file (else of the Protokoll); the dialog closes. Without a
    /// folder for the reports it stays, and says so.
    fn send_feedback(&mut self) {
        let Some(dialog) = self.state.feedback_dialog.take() else {
            return;
        };
        let dir = self.feedback_dir.clone().filter(|_| dialog.can_send());
        let Some(dir) = dir else {
            if dialog.can_send() {
                self.error(text::FEEDBACK_NO_FOLDER.into());
            }
            self.state.feedback_dialog = Some(dialog);
            return;
        };
        let created = chrono::Local::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, false);
        let job = FeedbackJob {
            dir,
            report: dialog.report(created),
            image: dialog.image_to_attach(),
            marks: dialog.marks_to_attach(),
            log_file: self.log_file.as_ref().map(|f| f.path().to_path_buf()),
            log: self.protocol_lines(crate::feedback::LOG_LINES),
        };
        self.feedback_saving.push_back(dialog);
        self.helper_queue
            .push_back(HelperTask::Feedback(Box::new(job)));
    }

    /// The last `n` lines of the Protokoll of the window, as the log file writes them.
    fn protocol_lines(&self, n: usize) -> Vec<String> {
        let log = &self.state.log;
        log[log.len().saturating_sub(n)..]
            .iter()
            .map(|e| {
                let level = match e.kind {
                    LogKind::Info => Level::Info,
                    LogKind::Warning => Level::Warning,
                    LogKind::Error => Level::Error,
                };
                logfile::format_line(&e.time, level, &e.text)
                    .trim_end()
                    .to_string()
            })
            .collect()
    }

    /// A report was written (or not): „Meldung gespeichert“, and an open list reads it anew; if
    /// it failed, why, and the dialog opens again with what was typed.
    fn feedback_written(&mut self, result: Result<JobOutput, StepError>) {
        let dialog = self.feedback_saving.pop_front();
        match result {
            Ok(output) => {
                if let (JobOutput::FeedbackSaved(folder), Some(file)) = (&output, &self.log_file) {
                    file.write(Level::Info, &folder.display().to_string());
                }
                self.info(text::FEEDBACK_SAVED.to_string());
                if self.state.feedback_list.is_some() {
                    self.open_reports();
                }
            }
            Err(e) => {
                self.error(text::feedback_not_saved(&text::step_error(&e)));
                if self.state.feedback_dialog.is_none() {
                    self.state.feedback_dialog = dialog;
                }
            }
        }
    }

    /// Whether a report is still to be written.
    fn feedback_pending(&self) -> bool {
        let feedback = |t: &HelperTask| matches!(t, HelperTask::Feedback(_));
        self.helper_running.as_ref().is_some_and(feedback) || self.helper_queue.iter().any(feedback)
    }

    /// ☰ → „Meldungen …“: the window with the reports, read anew from their folder.
    fn open_reports(&mut self) {
        let Some(dir) = self.feedback_dir.clone() else {
            self.state.feedback_list = Some(ReportList {
                problem: Some(text::FEEDBACK_NO_FOLDER.into()),
                ..ReportList::default()
            });
            return;
        };
        let reading = ReportList {
            reading: true,
            ..ReportList::default()
        };
        // the reports read before stay shown until the new ones are there
        let list = self.state.feedback_list.get_or_insert(reading);
        list.reading = true;
        list.problem = None;
        self.helper_queue
            .retain(|t| !matches!(t, HelperTask::ReadReports(_)));
        self.helper_queue.push_back(HelperTask::ReadReports(dir));
    }

    /// The reports were read: into the list, if it is still open.
    fn reports_read(&mut self, result: Result<JobOutput, StepError>) {
        let Some(list) = &mut self.state.feedback_list else {
            return;
        };
        list.reading = false;
        match result {
            Ok(JobOutput::Reports(reports)) => list.reports = reports,
            Ok(_) => {}
            Err(e) => list.problem = Some(text::step_error(&e)),
        }
    }

    /// „Zur Ansicht“ of the report in `folder` (of the list): its project is opened if another
    /// one is shown, then [`Self::advance_report_view`] sets its step, selection and camera.
    fn report_view(&mut self, folder: &Path) {
        let report = self
            .state
            .feedback_list
            .as_ref()
            .and_then(|l| l.reports.iter().find(|(p, _)| p == folder))
            .map(|(_, f)| f.clone());
        let Some(report) = report else {
            self.error(text::REPORT_GONE.into());
            return;
        };
        self.state.feedback_list = None;
        let project = report.project.map(|p| p.dir);
        if let Some(dir) = &project {
            if !self.is_open(dir) {
                self.apply(Action::OpenProject(dir.clone()));
            }
        }
        let selection = report
            .selection
            .as_ref()
            .and_then(super::feedback::selection_of);
        self.report_view = Some(ReportView {
            project,
            step: report.step.as_deref().and_then(super::feedback::step_of),
            selection,
            camera: report.camera.map(|c| c.state),
            selected: false,
        });
    }

    /// „Zur Ansicht“ goes on as far as it can: once the project of the report is shown (its tree
    /// read; a switch that waits for a step to stop waits too) its step and selection are set –
    /// not before, the project shown resets the selection –; once what they load is loaded and
    /// the run of the selection is read, the camera is set, by the view once it has all of it on
    /// the GPU ([`View3d::restore_camera`]). Gives up if the project did not open (that said why).
    fn advance_report_view(&mut self) {
        let Some(view) = &self.report_view else {
            return;
        };
        if !view.selected {
            let opening = self.pending.is_some()
                || self.state.opening.is_some()
                || self.tree_pending()
                || self.step_end.is_some();
            if opening {
                return;
            }
            if let Some(dir) = view.project.clone() {
                if !self.is_open(&dir) {
                    self.report_view = None;
                    return;
                }
            }
            let Some(view) = self.report_view.as_mut() else {
                return;
            };
            view.selected = true;
            let (step, selection) = (view.step.take(), view.selection.take());
            if let Some(step) = step {
                self.set_step(step);
            }
            match selection {
                Some(s) if super::feedback::selection_known(&self.state, &s) => {
                    self.apply(Action::Select(s))
                }
                Some(_) => self.info(text::REPORT_SELECTION_GONE.into()),
                None => {}
            }
            // the run of the selection is asked for in the next tick
            return;
        }
        if self.is_loading() || self.is_loading_surface() || self.run_loading() {
            return;
        }
        if let Some(camera) = self.report_view.take().and_then(|v| v.camera) {
            self.state.view.camera_mode = camera.mode;
            self.restore_camera(camera);
        }
    }

    /// Whether the run drawn is being read.
    fn run_loading(&self) -> bool {
        let run = |t: &HelperTask| matches!(t, HelperTask::LoadRun(_));
        self.helper_running.as_ref().is_some_and(run) || self.helper_queue.iter().any(run)
    }

    /// Sets the camera of the view as `camera` says: by the view, once what it shows is up on
    /// the GPU and framed; without a GPU (or with nothing to draw) at once, in the frame the view
    /// would draw in.
    fn restore_camera(&mut self, camera: CameraState) {
        let shown = self.cloud.is_some() || self.overlay.is_some() || self.view.has_surface();
        if self.render_state.is_some() && shown {
            self.view.restore_camera(camera);
            return;
        }
        let origin = self
            .view
            .view_origin()
            .or_else(|| self.view_origin())
            .or_else(|| self.overlay.as_ref().map(|o| o.origin))
            .unwrap_or([0.0; 3]);
        self.view.camera_mut().restore(&camera, origin);
    }

    /// Whether „Zur Ansicht“ of a report is still under way: its project, selection or camera
    /// not set yet.
    pub fn report_view_pending(&self) -> bool {
        self.report_view.is_some() || self.view.camera_pending()
    }

    /// Once per frame: takes a finished step, the new log lines and the progress.
    pub fn tick(&mut self) {
        // the lines of a step before its end, so that its summary comes before „fertig“
        let lines = self.jobs.take_log();
        self.take_lines(lines);
        if let Some(finished) = self.jobs.poll() {
            let lines = self.jobs.take_log();
            self.take_lines(lines);
            self.finished(finished);
        }
        self.state.job = self.jobs.status();
        self.tick_loader();
        self.tick_surface_loader();
        // what the readers say about a file they read anyway (a missing OBJ texture, …)
        let lines = self.loader.take_log();
        self.take_lines(lines);
        let lines = self.surface_loader.take_log();
        self.take_lines(lines);
        if self.focus_read_at.is_some_and(|at| Instant::now() >= at) {
            self.focus_read_at = None;
            self.focus_read = Some(Instant::now());
            self.refresh_tree();
        }
        self.tick_tree();
        self.keep_choices();
        // the read of a project being opened, where no cloud or surface is being loaded
        if self.state.loading.is_none() && self.state.opening.is_some() {
            self.state.loading = self.tree_loader.status();
        }
        self.sync_run();
        self.sync_compare();
        self.tick_helper();
        self.advance_touchup();
        self.state.map_status_pending = self.map_status_pending();
        if let Some(batch) = self.compute_all_waiting {
            if !self.state.map_status_pending {
                self.compute_all(batch);
            }
        }
        // last: what the selection of a report loads starts here, its run is read next tick
        self.advance_report_view();
    }

    fn finished(&mut self, f: JobFinished) {
        let step = self.state.running.take();
        // whether the step computed the scan at the head of the queue (read before the tree
        // is read again: a failed step may have left nothing behind)
        let queued = step
            .as_ref()
            .is_some_and(|a| self.state.computes_queue_head(a));
        let output = match f.result {
            Ok(output) => {
                match (&output, &step) {
                    (JobOutput::Exported(report), Some(Action::RunExport(req))) => {
                        self.info(text::exported(report, req.format.kind()))
                    }
                    // the touch-up says itself what came of it
                    (JobOutput::Touchup(_), _) => {}
                    _ => self.info(text::job_done(&f.label, &text::duration(f.seconds))),
                }
                Some(output)
            }
            Err(StepError::Cancelled) => {
                self.info(text::job_cancelled(&f.label));
                // a touch-up stops alone: the queue the window put it in goes on
                if !matches!(step, Some(Action::Touchup { .. })) {
                    self.stop_queue();
                }
                None
            }
            Err(e) => {
                self.error(text::job_failed(&f.label, &text::step_error(&e)));
                None
            }
        };
        let end = StepEnd {
            step,
            output,
            queued,
            seconds: f.seconds,
        };
        // the step may have added runs or maps: its result is shown once the tree is read
        // again (also after a check or an export: whatever changed on disk meanwhile shows);
        // a touch-up changes nothing the tree shows
        let touchup = matches!(end.step, Some(Action::Touchup { .. }));
        self.step_end = Some(end);
        if !touchup {
            self.refresh_tree();
        }
    }

    /// The end of a step, once the tree after it is read: the queue counts it, its result is
    /// selected and shown (a step of a queue only as its last), and a waiting switch of the
    /// project follows.
    fn finish_step(&mut self, end: StepEnd) {
        let StepEnd {
            step,
            output,
            queued,
            seconds,
        } = end;
        let hand_loads = self.queue_settings.as_ref().map(|q| q.hand_loads);
        if queued {
            self.queue_step_done(step.as_ref(), output.is_some());
        }
        // A step of a queue shows its result only as the queue's last, and only if nothing
        // was picked by hand meanwhile: the queue does not load every result beside the next
        // job, nor take the view from the user.
        let show = !queued
            || (self.state.queue.is_empty() && hand_loads.is_none_or(|n| n == self.hand_loads));
        // an export or a touch-up shows nothing: a surface picked meanwhile stays one picked
        // by hand
        if show && !matches!(step, Some(Action::RunExport(_) | Action::Touchup { .. })) {
            self.surface_by_hand = false;
        }
        // a reuse started with the count of written changes (taken also if it failed)
        let current = match &output {
            Some(JobOutput::Slam(done)) => self.state.correct.computed_current(&done.run_dir),
            _ => {
                self.state.correct.reuse_writes = None;
                true
            }
        };
        // the pairs are saved and the model is merged anew: setting them is done
        if output.is_some() && matches!(step, Some(Action::ApplyPairs { .. })) {
            tools::end_pairs(&mut self.state);
        }
        match (output, step) {
            (Some(JobOutput::Slam(done)), _) => {
                // computed with the corrections as they are now, unless one was written since
                if current {
                    self.state.correct.changed.remove(&done.run_dir);
                }
                self.reload_run(&done.run_dir);
                if show {
                    self.state.selection = Selection::Run(done.run_dir.clone());
                    self.load_run_cloud(&done.run_dir);
                }
                // its trajectories changed: compare again, as preset (before/after) for the
                // run drawn, else the same pair if one of them is of this run
                let c = &self.state.compare;
                if c.run.as_ref() == Some(&done.run_dir) {
                    self.state.compare.run = None;
                } else if let (Some(a), Some(b)) = (c.a.clone(), c.b.clone()) {
                    if a.run() == done.run_dir || b.run() == done.run_dir {
                        self.start_compare(a, b);
                    }
                }
            }
            // a failed or cancelled restore wrote its corrections back: show them again
            (None, Some(Action::RestoreHistory { run, .. })) => self.reload_run(&run),
            (Some(JobOutput::Map(done)), _) if show => {
                self.state.selection = Selection::Map(done.dir.clone());
                self.load_best_of_map(&done.dir);
            }
            (Some(JobOutput::Model(dir)), _) if show => {
                self.state.selection = Selection::Model(dir.clone());
                self.load_model_cloud(dir.join(MODEL_LAS));
            }
            (Some(JobOutput::SurfaceStep(done)), _) => {
                if done.reused {
                    self.info(text::surface_fits(&done.dir));
                }
                if show {
                    self.state.selection = Selection::Surface(done.dir.clone());
                    self.load_surface(&done.dir);
                }
                // a new surface of a map or model with a touch-up takes it over
                let owner = done.dir.parent().unwrap_or(&done.dir);
                let marks = self.state.marks.get(owner).and_then(|m| m.as_ref().ok());
                if marks.is_some_and(brush::has_steps) {
                    self.want_touchup(done.dir.clone(), None, true);
                }
            }
            (Some(JobOutput::Touchup(out)), Some(Action::Touchup { step, tell, .. })) => {
                self.touchup_done(*out, step, tell)
            }
            // cancelled or failed: the step just added did not happen, it goes again
            (
                None,
                Some(Action::Touchup {
                    surface,
                    step: Some((nr, seq)),
                    ..
                }),
            ) => {
                if self.take_back_step(&surface, nr, seq) {
                    self.info(text::touchup_taken_back(nr));
                }
            }
            // a recompute that did not run to its end: „Nachher“ of the surface shown is behind
            // its Stellen until the next change computes it
            (None, Some(Action::Touchup { surface, .. })) => {
                let shown = brush::shown_surface(&self.state) == Some(surface.as_path());
                if shown && self.pending.is_none() && !self.closing {
                    self.state.touchup.outdated = true;
                    self.info(text::TOUCHUP_OUTDATED.into());
                }
            }
            (Some(JobOutput::Inspect(report)), Some(Action::Inspect(scan))) => {
                self.state.inspect = Some(InspectResult {
                    scan,
                    report: *report,
                    seconds,
                });
            }
            _ => {}
        }
        self.advance_touchup();
        self.advance_queue();
        if let Some(action) = self.pending.take() {
            self.apply(action);
        }
    }

    /// After a step of the queue's head: a scan is done once its map was computed, a map once
    /// its surface was. A failed step stops the whole queue (its error is in the status bar);
    /// SLAM that gave no run to map drops the head.
    fn queue_step_done(&mut self, step: Option<&Action>, ok: bool) {
        if !ok {
            self.stop_queue();
            return;
        }
        let done = match step {
            Some(Action::RunSlam(_)) => {
                !matches!(self.state.next_queued_step(), Some(Action::RunMap(_)))
            }
            _ => true,
        };
        if done {
            self.state.queue.pop_front();
        }
    }

    /// Empties the queue: nothing of it starts any more.
    fn stop_queue(&mut self) {
        self.state.queue.clear();
        self.queue_settings = None;
        self.compute_all_waiting = None;
    }

    /// Whether the map statuses of the current tree are still being checked (or the tree is
    /// still being read).
    fn map_status_pending(&self) -> bool {
        self.status_asked != self.state.tree_version
            || matches!(self.helper_running, Some(HelperTask::MapStatus(_)))
            || self.tree_pending()
            || self.step_end.is_some()
    }

    /// [`Action::ComputeAllMaps`] (the new and stale scans) or [`Action::ComputeAllSurfaces`]
    /// (the maps without a current surface), once the map check is done: until then a stale
    /// map or surface would count as fitting.
    fn compute_all(&mut self, batch: Batch) {
        if self.map_status_pending() && !self.jobs.is_busy() {
            self.compute_all_waiting = Some(batch);
            return;
        }
        self.compute_all_waiting = None;
        match batch {
            Batch::Maps => {
                let scans = self.state.scans_to_compute();
                let items = scans.into_iter().map(Queued::Scan).collect();
                self.compute(items, text::ALL_COMPUTED);
            }
            Batch::Surfaces => {
                let maps = self.state.maps_to_surface();
                let items = maps.into_iter().map(Queued::Surface).collect();
                // the maps left out are named, whether others are queued or not
                let waiting = self.state.maps_waiting_for_data();
                if waiting == 0 {
                    self.compute(items, text::ALL_SURFACED);
                } else {
                    let wait = text::maps_wait_for_data(waiting);
                    self.compute(items, &wait);
                    if !self.state.queue.is_empty() {
                        self.info(wait);
                    }
                }
            }
        }
    }

    /// Queues `items` with the current map and surface settings and starts the first;
    /// refused while a step runs. `nothing`: the message when there is nothing to queue.
    fn compute(&mut self, items: Vec<Queued>, nothing: &str) {
        if let Some(job) = self.jobs.status() {
            self.error(text::busy(&job.label));
            return;
        }
        if items.is_empty() {
            self.info(nothing.into());
            return;
        }
        self.state.queue = items.into();
        self.queue_settings = Some(QueueSettings {
            slam: self.state.slam_settings.clone(),
            map: self.state.map_settings.clone(),
            surface: self.state.surface_settings.clone(),
            hand_loads: self.hand_loads,
        });
        self.advance_queue();
    }

    /// Starts the next step of the queue unless a step runs, with the settings the queue
    /// started with. A step that cannot start stops the queue with its reason.
    fn advance_queue(&mut self) {
        // the next step is told from the tree: wait for the read after a step or a change
        if self.jobs.is_busy()
            || self.state.queue.is_empty()
            || self.tree_pending()
            || self.step_end.is_some()
        {
            return;
        }
        let started = match self.state.next_queued_step() {
            Some(step) => {
                // the step reads the settings from the state: lend it the queue's
                let own = self.queue_settings.clone().map(|q| QueueSettings {
                    slam: std::mem::replace(&mut self.state.slam_settings, q.slam),
                    map: std::mem::replace(&mut self.state.map_settings, q.map),
                    surface: std::mem::replace(&mut self.state.surface_settings, q.surface),
                    hand_loads: q.hand_loads,
                });
                let started = actions::start_step(&mut self.state, &mut self.jobs, &step);
                if started.is_ok() {
                    self.check_room(&step);
                }
                if let Some(own) = own {
                    self.state.slam_settings = own.slam;
                    self.state.map_settings = own.map;
                    self.state.surface_settings = own.surface;
                }
                started
            }
            None => Err(match self.state.queue.front() {
                Some(Queued::Surface(dir)) => text::map_gone(dir),
                _ => text::UNKNOWN_SCAN.into(),
            }),
        };
        if let Err(e) = started {
            self.error(e);
            self.stop_queue();
        }
    }

    pub fn apply(&mut self, action: Action) {
        match action {
            Action::PickNewProject => {
                if let Some(dir) = self
                    .dialog(DialogFolder::Projects, text::DIALOG_NEW_PROJECT)
                    .pick_folder()
                {
                    self.remember_folder(DialogFolder::Projects, dir.parent());
                    let name = dir.file_name().map_or_else(
                        || text::DEFAULT_PROJECT_NAME.into(),
                        |n| n.to_string_lossy().into_owned(),
                    );
                    self.apply(Action::NewProject { dir, name });
                }
            }
            Action::PickOpenProject => {
                if let Some(dir) = self
                    .dialog(DialogFolder::Projects, text::DIALOG_OPEN_PROJECT)
                    .pick_folder()
                {
                    self.remember_folder(DialogFolder::Projects, dir.parent());
                    self.apply(Action::OpenProject(dir));
                }
            }
            Action::OpenProject(dir) if self.is_open(&dir) => {
                // already shown: nothing to switch, nothing to cancel (a pending switch to
                // another project is dropped, the user chose this one); read it again
                self.pending = None;
                self.refresh_tree();
            }
            switch @ (Action::OpenProject(_) | Action::NewProject { .. })
                if self.jobs.is_busy() =>
            {
                // The step belongs to the old project: stop it first, switch when it ended.
                self.stop_queue();
                self.jobs.cancel();
                self.pending = Some(switch);
                self.info(text::SWITCH_AFTER_CANCEL.to_string());
            }
            // the end of a step of the old project is still to show: switch after it
            switch @ (Action::OpenProject(_) | Action::NewProject { .. })
                if self.step_end.is_some() =>
            {
                self.pending = Some(switch);
            }
            Action::OpenProject(dir) => self.read_folder(TreeTask::Open {
                dir,
                restore: false,
            }),
            Action::NewProject { dir, name } => match Project::create(&dir, &name) {
                Ok(p) => {
                    // a new project has no tree yet; its folder is read once for the check of
                    // cloud storage
                    let read = ProjectRead {
                        project: p,
                        tree: Tree::default(),
                        cloud: None,
                        choices: None,
                    };
                    self.opened(read);
                    self.refresh_tree();
                }
                Err(e) => self.error(e.to_string()),
            },
            Action::PickAddScan => {
                if self.state.project.is_some() {
                    if let Some(dir) = self
                        .dialog(DialogFolder::Scans, text::DIALOG_ADD_SCAN)
                        .pick_folder()
                    {
                        self.remember_folder(DialogFolder::Scans, dir.parent());
                        self.apply(Action::AddScan(dir));
                    }
                }
            }
            Action::AddScan(dir) => self.change_project(|p| {
                let id = p.add_scan(&dir)?;
                Ok(Selection::Scan(id))
            }),
            Action::PickAddRun(scan) => {
                if self.state.project.is_some() {
                    if let Some(dir) = self
                        .dialog(DialogFolder::Scans, text::DIALOG_ADD_RUN)
                        .pick_folder()
                    {
                        self.remember_folder(DialogFolder::Scans, dir.parent());
                        self.change_project(|p| {
                            p.add_run(&scan, &dir)?;
                            Ok(Selection::Scan(scan.clone()))
                        });
                    }
                }
            }
            Action::Select(selection) => {
                self.hand_loads += 1;
                self.surface_by_hand = matches!(selection, Selection::Surface(_));
                self.state.selection = selection;
                self.load_selected();
                // choosing the shown run again reads it again (e.g. after `driftless correct`)
                if let Some(run) = self.state.correct.run.clone() {
                    if self.state.selected_run().is_some_and(|(_, r)| r.dir == run) {
                        self.reload_run(&run);
                    }
                }
            }
            Action::LoadCloud { path, run } => {
                self.hand_loads += 1;
                self.load(CloudSource::File { path, run })
            }
            Action::PickOpenFile => {
                let all: Vec<&str> = POINT_EXTENSIONS
                    .iter()
                    .chain(&MESH_EXTENSIONS)
                    .copied()
                    .collect();
                if let Some(path) = self
                    .dialog(DialogFolder::Files, text::DIALOG_OPEN_FILE)
                    .add_filter(text::FILTER_ALL, &all)
                    .add_filter(text::FILTER_CLOUD, &POINT_EXTENSIONS)
                    .add_filter(text::FILTER_MESH, &MESH_EXTENSIONS)
                    .pick_file()
                {
                    self.remember_folder(DialogFolder::Files, path.parent());
                    // picked again: a surface shown is read again
                    match open_file(&path, true) {
                        Ok(action) => self.apply(action),
                        Err(hint) => self.info(hint),
                    }
                }
            }
            Action::LoadSurface(path) => {
                self.hand_loads += 1;
                self.surface_by_hand = true;
                self.load_surface(&path)
            }
            Action::ReloadSurface(path) => {
                self.hand_loads += 1;
                self.surface_by_hand = true;
                self.load_surface_with(&path, true)
            }
            // the status bar shows the step if one runs, else the load
            Action::Cancel if self.jobs.is_busy() => {
                // nothing of the queue starts after the step that is stopped – but a touch-up
                // the window put in between stops alone
                if !matches!(self.state.running, Some(Action::Touchup { .. })) {
                    self.stop_queue();
                }
                self.jobs.cancel();
            }
            Action::Cancel => {
                // a queue waiting for the map check does not start either
                self.stop_queue();
                self.next_load = None;
                self.loader.cancel();
                self.next_surface = None;
                self.surface_loader.cancel();
                // a project being opened is not opened (its read ends unseen)
                if matches!(self.tree_next, Some(TreeTask::Open { .. })) {
                    self.tree_next = None;
                }
                if matches!(self.tree_task, Some(TreeTask::Open { .. })) {
                    self.tree_discard = true;
                }
                self.sync_tree_state();
            }
            Action::DismissMessages => self.state.messages.clear(),
            Action::Ask(ask) => self.state.ask = Some(ask),
            Action::AskDelete(dir) => {
                self.state.ask = self.state.delete_question(&dir).map(super::Ask::Delete)
            }
            Action::CloseAsk => self.state.ask = None,
            Action::RenameScan { id, name } => {
                self.state.ask = None;
                match self.state.scan_busy(&id) {
                    Some(why) => self.error(why),
                    None => {
                        let keep = self.state.selection.clone();
                        self.change_project(|p| {
                            p.rename_scan(&id, &name)?;
                            Ok(keep)
                        });
                    }
                }
            }
            Action::RemoveScan(id) => {
                self.state.ask = None;
                match self.state.scan_busy(&id) {
                    Some(why) => self.error(why),
                    None => self.change_project(|p| {
                        p.remove_scan(&id)?;
                        Ok(Selection::None)
                    }),
                }
            }
            Action::RemoveRun { scan, run } => match self.state.scan_busy(&scan) {
                Some(why) => self.error(why),
                None => self.change_project(|p| {
                    p.remove_run(&scan, &run)?;
                    Ok(Selection::Scan(scan.clone()))
                }),
            },
            Action::PickRelinkScan(id) => {
                if let Some(dir) = self
                    .dialog(DialogFolder::Scans, text::DIALOG_RELINK_SCAN)
                    .pick_folder()
                {
                    self.remember_folder(DialogFolder::Scans, dir.parent());
                    self.apply(Action::RelinkScan { id, dir });
                }
            }
            Action::RelinkScan { id, dir } => match self.state.scan_busy(&id) {
                Some(why) => self.error(why),
                None => self.helper_queue.push_back(HelperTask::Relink { id, dir }),
            },
            Action::Unshow(dir) => self.unshow(&dir),
            Action::CleanUp => match self.state.clean_up_blocked() {
                Some(why) => self.error(why.to_string()),
                None => {
                    let leftovers = self
                        .state
                        .leftovers_shown()
                        .iter()
                        .map(|l| (l.path.clone(), l.bytes))
                        .collect();
                    self.helper_queue.push_back(HelperTask::CleanUp(leftovers));
                }
            },
            Action::Delete(dir) => {
                self.state.ask = None;
                let Some(question) = self.state.delete_question(&dir) else {
                    self.error(text::not_in_tree(&dir));
                    return;
                };
                // what is shown or computed now (it may have changed since the question)
                match self.state.delete_blocked(&dir) {
                    Some(why) => self.error(why),
                    None => self.helper_queue.push_back(HelperTask::Trash {
                        dir,
                        label: question.label,
                        bytes: question.bytes,
                    }),
                }
            }
            Action::OpenLogFolder => {
                if let Err(e) = super::about::open_log_folder() {
                    self.error(e);
                }
            }
            Action::FrameAll => self.view.frame_all(),
            step @ (Action::Inspect(_)
            | Action::RunSlam(_)
            | Action::Reuse(_)
            | Action::RestoreHistory { .. }
            | Action::RunMap(_)
            | Action::RunSurface(_)
            | Action::RunMerge { .. }
            | Action::ApplyPairs { .. }
            | Action::RunModelSurface { .. }) => {
                match actions::start_step(&mut self.state, &mut self.jobs, &step) {
                    Ok(()) => {
                        // the merge runs: its field closes, the model shows when it is done
                        if matches!(step, Action::RunMerge { .. }) {
                            self.state.merge.open = false;
                        }
                        self.check_room(&step);
                    }
                    Err(e) => self.error(e),
                }
            }
            // the window closes once the export runs; refused, it stays for another try
            step @ Action::RunExport(_) => {
                match actions::start_step(&mut self.state, &mut self.jobs, &step) {
                    Ok(()) => self.state.export = None,
                    Err(e) => self.error(e),
                }
            }
            Action::OpenExport(source) => {
                // the touch-up of a surface not shown may not be current yet
                if let ExportSource::EditedSurface(dir) = &source {
                    self.want_touchup(dir.clone(), None, true);
                }
                let name = export::source_name(&self.state, &source);
                self.state.export = ExportDialog::new(vec![(source, name)]);
            }
            Action::OpenExportShown => {
                let sources = export::shown_sources(&self.state);
                // its touch-up is current before the export reads it (at once if it is)
                for (source, _) in &sources {
                    if let ExportSource::EditedSurface(dir) = source {
                        self.want_touchup(dir.clone(), None, true);
                    }
                }
                match ExportDialog::new(sources) {
                    Some(dialog) => self.state.export = Some(dialog),
                    None => self.info(text::EXPORT_NEEDS_VIEW.into()),
                }
            }
            Action::PickExportTarget => self.pick_export_target(),
            Action::ConfirmExport(request) => {
                if let Some(dialog) = &mut self.state.export {
                    dialog.replace = None;
                }
                self.apply(Action::RunExport(request));
            }
            Action::OpenMerge => self.state.open_merge(),
            Action::StartPairs { model, member } => self.start_pairs(&model, member),
            Action::SetPairEnabled { index, enabled } => self.switch_pair(index, enabled),
            change @ (Action::AddLoop { .. }
            | Action::SetLoopEnabled { .. }
            | Action::SetLoopUsed { .. }) => self.change_corrections(change),
            Action::Compare { a, b } => self.start_compare(a, b),
            Action::CompareMaps { a, b } => self.start_compare_maps(a, b),
            Action::SetStep(step) => self.set_step(step),
            Action::ComputeAllMaps => self.compute_all(Batch::Maps),
            Action::ComputeAllSurfaces => self.compute_all(Batch::Surfaces),
            Action::ComputeMap(id) => {
                let path = self
                    .state
                    .project
                    .as_ref()
                    .and_then(|p| p.scan(&id))
                    .map(|e| e.path.clone());
                match path {
                    Some(path) => self.compute(vec![Queued::Scan(path)], text::ALL_COMPUTED),
                    None => self.error(text::unknown_scan(&id)),
                }
            }
            Action::Paint { owner, mark, dabs } => {
                self.change_marks(owner, MarkChange::Paint { mark, dabs });
            }
            Action::SelectMark(nr) => self.state.brush.active = nr,
            Action::FrameMark(nr) => self.frame_mark(nr),
            Action::DeleteMark(nr) => {
                if let Some(owner) = brush::shown_owner(&self.state).map(Path::to_path_buf) {
                    if self.state.brush.active == Some(nr) {
                        self.state.brush.active = None;
                    }
                    self.change_marks(owner, MarkChange::Delete(nr));
                }
            }
            Action::UndoMark => {
                if let Some(owner) = brush::shown_owner(&self.state).map(Path::to_path_buf) {
                    self.change_marks(owner, MarkChange::Undo);
                }
            }
            Action::Smooth { mark, level } => {
                self.touchup_step(mark, MarkChange::Smooth { mark, level })
            }
            Action::Retexture { mark } => match brush::retexture_blocked(&self.state) {
                Some(why) => self.warn(why),
                None => self.touchup_step(mark, MarkChange::Retexture { mark }),
            },
            Action::ShowEdit(after) => {
                self.state.touchup.after = after;
                self.view.show_edit(after);
            }
            Action::OpenFeedback { mark } => self.ask_feedback(mark),
            Action::SendFeedback => self.send_feedback(),
            Action::OpenReports => self.open_reports(),
            Action::OpenReportFolder(dir) => {
                if let Err(e) = super::about::open_with_system(dir.as_os_str()) {
                    self.error(text::open_failed(
                        &dir.display().to_string(),
                        &e.to_string(),
                    ));
                }
            }
            Action::ReportView(dir) => self.report_view(&dir),
            Action::Touchup {
                surface,
                step,
                tell,
            } => self.want_touchup(surface, step, tell),
        }
    }

    /// „Ziel wählen …“ of the window „Exportieren“: the save dialog of the system, with the
    /// name the source suggests and the format's extension; then [`export::target_picked`].
    fn pick_export_target(&mut self) {
        let Some(dialog) = &self.state.export else {
            return;
        };
        let format = dialog.format;
        let name = format!("{}.{}", dialog.source().default_name(), format.extension());
        let Some(path) = self
            .dialog(DialogFolder::Files, text::EXPORT_TITLE)
            .set_file_name(name)
            .add_filter(format.label(), &[format.extension()])
            .save_file()
        else {
            return;
        };
        self.remember_folder(DialogFolder::Files, path.parent());
        match export::target_picked(&mut self.state, path) {
            Ok(Some(action)) => self.apply(action),
            // asked in the window
            Ok(None) => {}
            Err(e) => self.error(e),
        }
    }

    /// Shows `step` and what belongs to it in the view: the surface in „Bilder“ if one is
    /// shown or being loaded, else the cloud. A switch by hand holds until the next step; the
    /// step already shown asked for again changes nothing.
    fn set_step(&mut self, step: Step) {
        if self.state.step == step {
            return;
        }
        self.state.step = step;
        // picking places of a loop belongs to „Daten“: elsewhere a click measures or picks
        if step != Step::Daten {
            self.state.correct.picking = false;
        }
        let surface = self.state.surface.is_some() || self.is_loading_surface();
        self.state.view.show = match step {
            Step::Bilder if surface => ShowMode::Surface,
            _ => ShowMode::Cloud,
        };
    }

    /// „Aus der Ansicht nehmen“: what the view shows from the folder `dir` or from a model that
    /// uses it goes (the cloud, the surface, „Von Hand ausrichten“), and the selection if it
    /// lies there; the run drawn follows ([`Self::sync_run`]).
    fn unshow(&mut self, dir: &Path) {
        let mut touched = vec![dir.to_path_buf()];
        touched.extend(self.state.model_users_dirs(dir));
        let hit = |p: &Path| touched.iter().any(|t| p.starts_with(t));
        let drawn = self.state.selected_run().is_some_and(|(_, r)| hit(&r.dir));
        if drawn || touched.iter().any(|t| self.selection_in(t)) {
            self.state.selection = Selection::None;
        }
        if self.state.cloud.as_ref().is_some_and(|c| hit(&c.path)) {
            self.cloud = None;
            self.cloud_run = None;
            self.state.cloud = None;
            self.state.view.cut = None;
            self.state.picked = None;
            self.state.view_stats = None;
            self.view.cloud_replaced();
        }
        if self.state.surface.as_ref().is_some_and(|s| hit(&s.path)) {
            self.drop_surface();
        }
        if self.state.pairs.as_ref().is_some_and(|p| hit(&p.model)) {
            tools::end_pairs(&mut self.state);
        }
        self.sync_run();
    }

    /// Whether the selection lies in the folder `dir`.
    fn selection_in(&self, dir: &Path) -> bool {
        match &self.state.selection {
            Selection::Run(p)
            | Selection::Map(p)
            | Selection::Cloud(p)
            | Selection::Surface(p)
            | Selection::Model(p)
            | Selection::History { run: p, .. } => p.starts_with(dir),
            Selection::None | Selection::Scan(_) => false,
        }
    }

    /// „Ordner neu zuordnen …“ of the scan `id` ended: the project as written and the tree
    /// read again, or the reason it was refused.
    fn relinked(&mut self, id: &str, result: Result<JobOutput, StepError>) {
        match result {
            Ok(JobOutput::Relinked { project, checked }) => {
                let name = project
                    .scan(id)
                    .map_or_else(|| id.to_string(), |s| s.name.clone());
                if self.is_shown(&project.dir) {
                    self.state.project = Some(*project);
                    self.refresh_tree();
                }
                self.info(text::relinked(&name, checked));
            }
            Ok(_) => {}
            Err(e) => self.error(text::step_error(&e)),
        }
    }

    /// Whether `dir` is the folder of the project shown (as stored, without resolving it).
    fn is_shown(&self, dir: &Path) -> bool {
        self.state.project.as_ref().is_some_and(|p| p.dir == dir)
    }

    fn is_open(&self, dir: &Path) -> bool {
        match (&self.state.project, canonical(dir)) {
            (Some(p), Ok(dir)) => p.dir == dir,
            _ => false,
        }
    }

    /// Changes the project as `project.toml` is on disk now, under its lock, and saves it
    /// ([`change_project`]): a change another window or program saved meanwhile is kept (and
    /// said), a lock another holds refuses the change with its reason. Only a saved change is
    /// shown, so the project on screen always matches `project.toml`. The tree is read again
    /// in the background.
    fn change_project(
        &mut self,
        change: impl FnOnce(&mut Project) -> Result<Selection, crate::project::ProjectError>,
    ) {
        let Some(dir) = self.state.project.as_ref().map(|p| p.dir.clone()) else {
            return;
        };
        match change_project(&dir, self.state.project.as_ref(), change) {
            Ok((project, changed_meanwhile, selection)) => {
                if changed_meanwhile {
                    self.info(text::PROJECT_CHANGED_ELSEWHERE.to_string());
                }
                self.state.project = Some(project);
                self.refresh_tree();
                self.state.selection = selection;
            }
            Err(e) => self.error(e.to_string()),
        }
    }

    fn info(&mut self, text: String) {
        self.say(UiMessage::Info(text));
    }

    fn warn(&mut self, text: String) {
        self.say(UiMessage::Warning(text));
    }

    fn error(&mut self, text: String) {
        self.say(UiMessage::Error(text));
    }

    /// Every message of the status bar comes here, in German ([`text::german`]): into the
    /// log file, onto the status bar (the same as the last one only once), and a warning or an
    /// error into the Protokoll as well.
    fn say(&mut self, m: UiMessage) {
        let (level, kind) = match &m {
            UiMessage::Info(_) => (Level::Message, None),
            UiMessage::Warning(_) => (Level::Warning, Some(LogKind::Warning)),
            UiMessage::Error(_) => (Level::Error, Some(LogKind::Error)),
        };
        let german = text::german(m.text());
        if let Some(file) = &self.log_file {
            file.write(level, &german);
        }
        let shown = text::shown(&german).into_owned();
        let m = match m {
            UiMessage::Info(_) => UiMessage::Info(shown),
            UiMessage::Warning(_) => UiMessage::Warning(shown),
            UiMessage::Error(_) => UiMessage::Error(shown),
        };
        if self.state.messages.last() == Some(&m) {
            return;
        }
        if let Some(kind) = kind {
            self.protocol(kind, m.text());
        }
        let messages = &mut self.state.messages;
        messages.push(m);
        let over = messages.len().saturating_sub(MESSAGE_LIMIT);
        messages.drain(..over);
    }

    /// A line of the Protokoll as shown, with the time; the oldest go beyond [`LOG_LIMIT`].
    /// Not into the log file: that is the caller's ([`DriftlessApp::say`],
    /// [`DriftlessApp::take_lines`]).
    fn protocol(&mut self, kind: LogKind, shown: &str) {
        let log = &mut self.state.log;
        log.push(LogEntry {
            time: chrono::Local::now().format("%H:%M:%S").to_string(),
            kind,
            text: shown.to_string(),
        });
        let over = log.len().saturating_sub(LOG_LIMIT);
        log.drain(..over);
    }

    /// What the steps and the loaders said: in German into the log file and the Protokoll.
    fn take_lines(&mut self, lines: Vec<LogLine>) {
        for line in lines {
            let (level, kind, said) = match line {
                LogLine::Info(t) => (Level::Info, LogKind::Info, t),
                LogLine::Warning(t) => (Level::Warning, LogKind::Warning, t),
            };
            let german = text::german(&said);
            if let Some(file) = &self.log_file {
                file.write(level, &german);
            }
            self.protocol(kind, &text::shown(&german));
        }
    }

    fn update_title(&mut self, ctx: &egui::Context) {
        let title = match &self.state.project {
            Some(p) => format!("{} – {}", p.name, text::APP_TITLE),
            None => text::APP_TITLE.to_string(),
        };
        if title != self.title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.title = title;
        }
    }

    /// Whether the view draws something in the quality colours: the trajectory (not while a
    /// comparison draws it in one colour), the anchors of the submaps, or the points in the
    /// colour mode „Güte“ with a SLAM report.
    fn quality_drawn(&self) -> bool {
        let v = &self.state.view;
        let run = self.overlay.as_ref().is_some_and(|o| {
            let comparing = v.show_compare && !o.compare.is_empty();
            !o.report.submaps.is_empty() && ((v.show_trajectory && !comparing) || v.show_submaps)
        });
        let points = self.cloud.as_ref().is_some_and(|c| {
            let shown = effective_show(v.show, self.view.has_surface(), true);
            c.points.quality_known
                && shown != ShowMode::Surface
                && super::view3d::effective_color_mode(
                    v.color_mode,
                    c.points.rgb.is_some(),
                    !c.points.source.is_empty(),
                ) == crate::render::ColorMode::Quality
        });
        run || points
    }

    /// In „Bilder“ without a surface: a hint over the top of the view that the cloud stays
    /// until one is computed.
    fn no_surface_hint(&self, ctx: &egui::Context, view: egui::Rect) {
        // without a map there is nothing to compute one from (the field says what to do)
        let maps = self
            .state
            .scans
            .iter()
            .flat_map(|s| &s.runs)
            .any(|r| !r.maps.is_empty())
            || !self.state.models.is_empty();
        if self.state.step != Step::Bilder
            || !maps
            || self.state.surface.is_some()
            || self.is_loading_surface()
        {
            return;
        }
        let top = view.center_top() + egui::vec2(0.0, HINT_TOP);
        egui::Area::new(egui::Id::new("hinweis_oberflaeche"))
            .fixed_pos(top)
            .pivot(egui::Align2::CENTER_TOP)
            .interactable(false)
            .show(ctx, |ui| {
                theme::float_frame().show(ui, |ui| ui.label(text::NO_SURFACE_YET));
            });
    }

    /// A file dragged over the window: a hint over its middle that letting go opens it.
    fn drop_hint(&self, ctx: &egui::Context) {
        if ctx.input(|i| i.raw.hovered_files.is_empty()) {
            return;
        }
        egui::Area::new(egui::Id::new("hinweis_loslassen"))
            .order(egui::Order::Foreground)
            .fixed_pos(ctx.content_rect().center())
            .pivot(egui::Align2::CENTER_CENTER)
            .interactable(false)
            .show(ctx, |ui| {
                theme::float_frame().show(ui, |ui| ui.heading(text::DROP_HERE));
            });
    }

    /// One frame of the window (what [`eframe::App::ui`] does; without eframe for tests).
    pub fn show(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        if !self.themed {
            // Windowless harnesses come here without the creation context: theme now, and
            // draw this pass again, so that the first frame shown is themed (the fonts with the
            // symbols take effect at the start of a pass).
            theme::apply(&ctx);
            ui.set_style(ctx.global_style());
            ctx.request_discard("theme applied");
            self.themed = true;
        }
        if ctx.input(|i| i.viewport().close_requested()) && !self.close_requested() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
        }
        let focused = ctx.input(|i| {
            i.events
                .iter()
                .any(|e| matches!(e, egui::Event::WindowFocused(true)))
        });
        if focused {
            self.focus_back();
        }
        self.tick();
        if self.take_close() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if self.wants_repaint_while_busy() {
            ctx.request_repaint_after(REPAINT_WHILE_BUSY);
        }
        self.update_title(&ctx);
        // while „Melden“ is open the keys of the view and the brush wait (Esc closes it)
        let reporting = self.state.feedback_dialog.is_some();
        if !reporting {
            show_mode_key(&ctx, &mut self.state);
            tools::keys(&ctx, &mut self.state);
        }
        // ⇧⇧ anywhere: „Melden“
        if ctx.input(|i| self.double_shift.feed(&i.events, i.time)) {
            self.ask_feedback(None);
        }
        self.feedback_picture(&ctx);

        let mut actions: Vec<Action> = Vec::new();
        if !reporting {
            actions.extend(brush::keys(&ctx, &mut self.state));
        }
        // without a surface of the project to paint on the brush goes back to „Drehen“
        if self.state.tool == Tool::Brush && brush::blocked(&self.state).is_some() {
            tools::select(&mut self.state, Tool::Orbit);
        }
        // the marks lie on the state drawn: „Nachher“ where the touch-up moved them
        self.view.show_edit(self.state.touchup.after);
        self.brush.set_after(self.state.touchup.after);
        if let Some(marks) = self.brush.sync(&mut self.state) {
            self.view.set_marks(marks);
        }
        let brush_chosen = self.state.tool == Tool::Brush;
        self.layout.rail = egui::Panel::left("leiste")
            .exact_size(rail::WIDTH)
            .resizable(false)
            .frame(theme::rail_frame())
            .show(ui, |ui| {
                actions.extend(rail::show(ui, &mut self.state));
            })
            .response
            .rect;
        self.layout.status = egui::Panel::bottom("status")
            .show(ui, |ui| {
                actions.extend(status::show(ui, &self.state));
            })
            .response
            .rect;
        self.layout.step = egui::Panel::left("schritt")
            .default_size(STEP_WIDTH)
            .min_size(STEP_MIN_WIDTH)
            .frame(theme::step_frame())
            .show(ui, |ui| {
                actions.extend(steps::show(ui, &mut self.state));
            })
            .response
            .rect;
        settings::show(&ctx, &mut self.state);
        super::about::show(&ctx, &mut self.state.about_open);
        actions.extend(export::show(&ctx, &mut self.state));
        actions.extend(super::manage::show(&ctx, &mut self.state));
        actions.extend(super::feedback::show_list(&ctx, &mut self.state));
        actions.extend(super::feedback::show_dialog(&ctx, &mut self.state));
        self.view.set_tool(self.state.tool);
        self.view
            .set_picked(self.state.picked.as_ref().map(|p| p.index));
        let origin = self.view_origin();
        let mut event = None;
        let (places, avoid_submap) = self.places();
        let overlay = self.overlay.as_ref().map(|run| ViewOverlay {
            run,
            version: self.overlay_version,
            places,
            avoid_submap,
            pick_places: self.state.correct.picking,
        });
        let shown = self.cloud.is_some() || overlay.is_some() || self.view.has_surface();
        let central =
            egui::CentralPanel::no_frame().show(ui, |ui| match (&self.render_state, shown) {
                (Some(rs), true) => {
                    let rect = ui.max_rect();
                    event = self
                        .view
                        .show(ui, rs, self.cloud.as_ref(), overlay, &self.state.view);
                    if let Some(origin) = origin {
                        let uploading = self.view.is_uploading_surface();
                        let input = self.view.brush_input();
                        let camera = self.view.camera();
                        let stroke =
                            self.brush
                                .input(&mut self.state, input, camera, origin, uploading);
                        actions.extend(stroke);
                    }
                    let active = self.state.brush.active;
                    if let Some(marks) = self.brush.take_marks(active, brush_chosen) {
                        self.view.set_marks(marks);
                        ui.ctx().request_repaint();
                    }
                    if let (Some(origin), false) = (origin, self.view.is_uploading()) {
                        let camera = self.view.camera();
                        tools::paint_measurements(ui, rect, camera, origin, &self.state);
                        tools::paint_pairs(ui, rect, camera, origin, &self.state);
                        if self.state.tool == Tool::Brush {
                            let radius = self.state.brush.radius;
                            self.brush.paint_ring(ui, rect, camera, origin, radius);
                        }
                    }
                }
                (None, true) => {
                    ui.centered_and_justified(|ui| ui.weak(text::VIEW_NO_GPU));
                }
                (_, false) => {
                    let hint = match self.state.project {
                        Some(_) => text::VIEW_EMPTY,
                        None => text::VIEW_NO_PROJECT,
                    };
                    ui.centered_and_justified(|ui| ui.weak(hint));
                }
            });
        self.state.brush.painting = self.brush.painting();
        self.no_surface_hint(&ctx, central.response.rect);
        self.drop_hint(&ctx);
        match ctx.input(|i| dropped(&i.raw.dropped_files)) {
            Some(Ok(action)) => actions.push(action),
            Some(Err(hint)) => self.info(hint),
            None => {}
        }
        let view = central.response.rect;
        actions.extend(tools::floats(&ctx, view, &mut self.state, &mut self.layout));
        let legend = self.quality_drawn();
        tools::quality_legend(&ctx, view, legend, &mut self.layout);
        match event {
            Some(ViewEvent::PickedPose(i)) => self.pose_picked(i),
            Some(ViewEvent::TrajectoryHidden) => {
                let m = UiMessage::Info(text::PICK_NEEDS_TRAJECTORY.into());
                self.state.correct.message = Some(m);
            }
            _ => {}
        }
        let shown = effective_show(
            self.state.view.show,
            self.view.has_surface(),
            self.cloud.is_some(),
        );
        let stats = match (&self.cloud, self.view.surface_triangles()) {
            (_, Some(triangles)) if shown == ShowMode::Surface => Some(ViewStats {
                drawn: 0,
                total: 0,
                triangles: Some(triangles),
                frame_ms: self.view.frame_ms(),
                gpu: self.view.gpu_timed(),
            }),
            (Some(cloud), _) => Some(ViewStats {
                drawn: self.view.drawn_points(),
                total: cloud.points.len(),
                triangles: None,
                frame_ms: self.view.frame_ms(),
                gpu: self.view.gpu_timed(),
            }),
            _ => None,
        };
        // the right panel was drawn before the view: show the new numbers next frame
        if stats.is_some() && stats != self.state.view_stats {
            self.state.view_stats = stats;
            ctx.request_repaint();
        }
        if let (Some(cloud), Some(ViewEvent::PickedPoint(i))) = (&self.cloud, event) {
            tools::point_clicked(&mut self.state, point_info(cloud, i));
            // the marker and the measurement show from the next frame on
            ctx.request_repaint();
        }
        for action in actions {
            self.apply(action);
        }
        tools::remember_text_focus(&ctx);
    }
}

/// What a step of a touch-up baked, as the list and the status bar say it ([`Variant`]).
fn variant(o: &driftless_steps::touchup::StepOutcome) -> Option<Variant> {
    Some(Variant {
        of: o.variant_of?,
        from: o.image.as_ref().map(|i| i.from),
    })
}

/// What opening the file `path` asks for, by its extension ([`file_kind`]): a point cloud is
/// shown as a cloud (without a run, with or without a project), a mesh as a surface (with
/// `reload`, also when it is shown and unchanged); of another file only a hint (`Err`).
fn open_file(path: &Path, reload: bool) -> Result<Action, String> {
    let path = path.to_path_buf();
    match file_kind(&path) {
        Some(ExportKind::Points) => Ok(Action::LoadCloud { path, run: None }),
        Some(ExportKind::Mesh) if reload => Ok(Action::ReloadSurface(path)),
        Some(ExportKind::Mesh) => Ok(Action::LoadSurface(path)),
        _ => {
            let name = path.file_name().unwrap_or_default();
            Err(text::drop_unsupported(&name.to_string_lossy()))
        }
    }
}

/// What files dropped onto the window ask for: the first one Driftless can open
/// ([`open_file`]); if there is none, the hint for the first file (`Err`). Nothing without
/// files.
fn dropped(files: &[egui::DroppedFileHandle]) -> Option<Result<Action, String>> {
    let first = files.first()?;
    Some(
        files
            .iter()
            .find_map(|f| open_file(f.path(), false).ok())
            .ok_or_else(|| open_file(first.path(), false).unwrap_err()),
    )
}

/// When a file was last written, and its size; `None` if it cannot be read.
type FileStamp = Option<(std::time::SystemTime, u64)>;

fn file_stamp(path: &Path) -> FileStamp {
    let meta = std::fs::metadata(path).ok()?;
    Some((meta.modified().ok()?, meta.len()))
}

/// The key `O` (while no text field has the keyboard) switches what the view shows:
/// Oberfläche → Punktwolke → beides; only with a surface and a cloud, like the switch.
fn show_mode_key(ctx: &egui::Context, state: &mut super::AppState) {
    let pressed = ctx.input(|i| i.key_pressed(egui::Key::O) && i.modifiers.is_none());
    if pressed && tools::keys_free(ctx) && state.surface.is_some() && state.cloud.is_some() {
        state.view.show = state.view.show.next();
    }
}

/// Carries out a change of the corrections of a run as `driftless correct` does (checked
/// against `submaps.bin`), then reads the file again.
fn change_corrections(action: &Action) -> Result<Corrected, StepError> {
    let (run, message, changed, added) = match action {
        Action::AddLoop { run, a, b, note } => {
            let nr = correct::add_loop(run, *a, *b, note)?;
            (run, correct::added_message(nr, *a, *b), true, true)
        }
        Action::SetLoopEnabled { run, nr, enabled } => {
            let changed = correct::set_enabled(run, *nr, *enabled)?;
            (
                run,
                correct::switched_message(*nr, *enabled, changed),
                changed,
                false,
            )
        }
        Action::SetLoopUsed { run, a, b, used } => {
            let changed = correct::set_loop_used(run, *a, *b, *used)?;
            (
                run,
                correct::loop_used_message(*a, *b, *used, changed),
                changed,
                false,
            )
        }
        _ => return Err(text::NOT_A_STEP.into()),
    };
    Ok(Corrected {
        run: run.clone(),
        message,
        changed,
        added,
        corrections: correct::load(run)?,
    })
}

impl eframe::App for DriftlessApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.show(ui);
    }

    fn on_exit(&mut self) {
        if let Some(file) = &self.log_file {
            file.write(Level::Info, logfile::END_TEXT);
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if let Some(last) = self.last_to_store() {
            storage.set_string(LAST_PROJECT_KEY, last);
        }
        storage.set_string(UI_KEY, self.ui_to_store());
    }
}
