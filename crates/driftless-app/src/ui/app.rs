//! The program: carries out the panels' actions, owns the job runners, the dialogs, the
//! loaded cloud, the run drawn over it and the 3D view.

use super::compare::{compare_maps, compare_refs, default_refs, TrajRef};
use super::correct::{Corrected, RunLoaded};
use super::view3d::{point_info, View3d, ViewEvent, ViewOverlay};
use super::{
    actions, left, right, status, text, Action, AppState, CloudSummary, InspectResult, Selection,
    UiMessage, ViewStats,
};
use crate::cloud::{load_cloud, LoadedCloud};
use crate::jobs::{JobFinished, JobOutput, JobRunner, StartError};
use crate::overlay::RunOverlay;
use crate::project::{canonical, Project};
use crate::synthetic::synthetic_cloud;
use driftless_io::progress::Progress;
use driftless_steps::{correct, StepError};
use eframe::egui;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

/// Key of the last opened project in eframe's storage.
const LAST_PROJECT_KEY: &str = "letztes_projekt";
/// While a step runs, the window draws this often, so progress and runtime move (the job
/// runner only wakes the UI when a job ends, see `crate::jobs`).
const REPAINT_WHILE_BUSY: Duration = Duration::from_millis(100);
/// Lines kept in the log; older ones are dropped.
const LOG_LIMIT: usize = 10_000;
/// Messages kept for the status bar (it shows the last); older ones are dropped.
const MESSAGE_LIMIT: usize = 100;
const LEFT_WIDTH: f32 = 280.0;
/// The project tree never gets narrower than this (names are cut with „…“ instead).
const LEFT_MIN_WIDTH: f32 = 220.0;
const RIGHT_WIDTH: f32 = 340.0;

/// Seed of the synthetic cloud of `--synthetic`.
const SYNTHETIC_SEED: u64 = 1;

/// The arguments of `driftless-app [--synthetic N] [projekt-ordner]`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Args {
    pub project: Option<PathBuf>,
    /// Show a synthetic cloud of this many points (to try the view without a scan).
    pub synthetic: Option<usize>,
}

/// Reads the arguments: at most one project folder, `--synthetic N` with 1 ≤ N < 2³²
/// written as digits, optionally grouped with '_' (`50_000_000`).
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Args, String> {
    let mut out = Args::default();
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
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

/// Where a cloud comes from.
#[derive(Debug, Clone, PartialEq)]
enum CloudSource {
    /// A LAS file, with the run folder whose `slam.json` gives the quality.
    File {
        path: PathBuf,
        run: Option<PathBuf>,
    },
    Synthetic(usize),
}

impl CloudSource {
    fn path(&self) -> PathBuf {
        match self {
            CloudSource::File { path, .. } => path.clone(),
            CloudSource::Synthetic(n) => PathBuf::from(text::synthetic_name(*n)),
        }
    }
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
    /// [`Action::AddLoop`] or [`Action::SetLoopEnabled`]: they read `submaps.bin`.
    Correct(Action),
    /// Two trajectories compared (reads both, well below a second).
    Compare(TrajRef, TrajRef),
    /// Two `map.json` side by side.
    CompareMaps(PathBuf, PathBuf),
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
    /// The window title last sent.
    title: String,
    /// Closing waits for the cancelled step; the window closes once it has ended.
    closing: bool,
    /// The stored last project could not be opened: forget it on the next save.
    forget_last: bool,
}

impl DriftlessApp {
    /// Without a window; `repaint` wakes the UI when a job ends.
    pub fn new(repaint: Arc<dyn Fn() + Send + Sync>) -> Self {
        DriftlessApp {
            state: AppState::default(),
            jobs: JobRunner::new(repaint.clone()),
            helper: JobRunner::new(repaint.clone()),
            helper_queue: VecDeque::new(),
            helper_running: None,
            loader: JobRunner::new(repaint),
            loading: None,
            next_load: None,
            cloud: None,
            cloud_run: None,
            overlay: None,
            overlay_version: 0,
            view: View3d::default(),
            render_state: None,
            status_asked: 0,
            gaps_asked: 0,
            pending: None,
            title: String::new(),
            closing: false,
            forget_last: false,
        }
    }

    /// For eframe: opens the project of `args`, else the project opened last, and starts
    /// generating the synthetic cloud if asked for.
    pub fn from_creation_context(cc: &eframe::CreationContext<'_>, args: Args) -> Self {
        let ctx = cc.egui_ctx.clone();
        let mut app = DriftlessApp::new(Arc::new(move || ctx.request_repaint()));
        app.render_state = cc.wgpu_render_state.clone();
        if let Some(dir) = args.project {
            app.apply(Action::OpenProject(dir));
        } else {
            let last = cc.storage.and_then(|s| s.get_string(LAST_PROJECT_KEY));
            app.restore_last(last.as_deref());
        }
        if let Some(n) = args.synthetic {
            app.load_synthetic(n);
        }
        app
    }

    /// With the GPU of a window (or of a test renderer): the 3D view draws with it.
    pub fn with_render_state(mut self, render_state: egui_wgpu::RenderState) -> Self {
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
    /// corrections are shown; reading them is queued for the helper.
    fn sync_run(&mut self) {
        let wanted = self
            .state
            .selected_run()
            .map(|(_, r)| r.dir.clone())
            .or_else(|| self.cloud_run.clone());
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
        self.state.correct.corrections = Some(loaded.corrections);
        match loaded.overlay {
            Ok(mut o) => {
                o.set_origin(self.cloud.as_ref().map(|c| c.tree.origin));
                self.overlay = Some(o);
                self.compare_overlay();
            }
            Err(e) => {
                self.overlay = None;
                self.error(e);
            }
        }
        self.overlay_version += 1;
    }

    /// A change of the corrections ended: its answer goes into the panel unchanged.
    fn corrected(&mut self, action: &Action, result: Result<JobOutput, StepError>) {
        let c = &mut self.state.correct;
        c.busy = false;
        let (Action::AddLoop { run, .. } | Action::SetLoopEnabled { run, .. }) = action else {
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
                    c.message = Some(UiMessage::Info(done.message));
                }
            }
            Ok(_) => {}
            Err(e) if shown => c.message = Some(UiMessage::Error(e.to_string())),
            Err(e) => self.error(e.to_string()),
        }
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
            CloudSource::File { .. } => text::LOAD_CLOUD,
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
            match f.result {
                Ok(JobOutput::Cloud(cloud)) => {
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
                        quality_known: cloud.points.quality_known,
                        z_range: cloud.z_range,
                        ceiling: cloud.ceiling,
                    });
                    self.state.view.cut = None;
                    self.state.picked = None;
                    self.state.view_stats = None;
                    self.cloud = Some(*cloud);
                    self.view.cloud_replaced();
                }
                Ok(_) => {}
                // a cancel for the next load needs no message
                Err(StepError::Cancelled) if self.next_load.is_some() => {}
                Err(StepError::Cancelled) => self.info(text::job_cancelled(&f.label)),
                Err(StepError::Failed(m)) => self.error(text::job_failed(&f.label, &m)),
            }
        }
        if !self.loader.is_busy() {
            if let Some(next) = self.next_load.take() {
                self.load(next);
            }
        }
        self.state.loading = self.loader.status();
    }

    /// Loads the cloud of `selection` if it is one, with the quality of its run.
    fn load_selected(&mut self) {
        if let Selection::Cloud(path) = &self.state.selection {
            let run = self.state.selected_run().map(|(_, r)| r.dir.clone());
            let path = path.clone();
            self.load(CloudSource::File { path, run });
        }
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

    /// Opens the stored last project (an empty value is none). If it cannot be opened, the
    /// message is shown once and the value is forgotten ([`DriftlessApp::last_to_store`]).
    pub fn restore_last(&mut self, stored: Option<&str>) {
        let Some(last) = stored.filter(|s| !s.is_empty()) else {
            return;
        };
        match Project::open(Path::new(last)) {
            Ok(p) => self.set_project(p),
            Err(e) => {
                self.error(text::last_project_gone(&e.to_string()));
                self.forget_last = true;
            }
        }
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
        if !self.jobs.is_busy() {
            return true;
        }
        if !self.closing {
            self.closing = true;
            self.pending = None;
            self.jobs.cancel();
            self.info(text::CLOSE_AFTER_CANCEL.to_string());
        }
        false
    }

    /// True once, after a put-off close, when the step has ended.
    pub fn take_close(&mut self) -> bool {
        if self.closing && !self.jobs.is_busy() {
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
            || self.status_asked != self.state.tree_version
            || self.gaps_asked != self.state.tree_version
    }

    /// Takes what the helper finished and starts its next task: queued ones first, then the
    /// map statuses when the tree changed.
    fn tick_helper(&mut self) {
        if let Some(f) = self.helper.poll() {
            match (self.helper_running.take(), f.result) {
                (Some(HelperTask::MapStatus(v)), Ok(JobOutput::MapStatus(status))) => {
                    // an older tree's statuses are dropped; the newer job follows
                    if v == self.state.tree_version {
                        self.state.map_status = *status;
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
            .or_else(|| (self.gaps_asked != version).then_some(HelperTask::HistoryGaps(version)));
        if let Some(task) = task {
            self.start_helper(task);
        }
    }

    fn start_helper(&mut self, task: HelperTask) {
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
        };
        match started {
            Ok(()) => {
                match task {
                    HelperTask::MapStatus(v) => self.status_asked = v,
                    HelperTask::HistoryGaps(v) => self.gaps_asked = v,
                    _ => {}
                }
                self.helper_running = Some(task);
            }
            // the map statuses are asked again at the next frame
            Err(_) if matches!(task, HelperTask::MapStatus(_) | HelperTask::HistoryGaps(_)) => {}
            Err(e) => match &task {
                HelperTask::Correct(action) => {
                    self.corrected(action, Err(StepError::Failed(e.to_string())))
                }
                HelperTask::Compare(..) => self.state.compare.result = Some(Err(e.to_string())),
                HelperTask::CompareMaps(..) => self.state.compare.maps = Some(Err(e.to_string())),
                _ => self.error(e.to_string()),
            },
        }
    }

    /// Once per frame: takes a finished step, the new log lines and the progress.
    pub fn tick(&mut self) {
        if let Some(finished) = self.jobs.poll() {
            self.finished(finished);
        }
        self.state.log.extend(self.jobs.take_log());
        let over = self.state.log.len().saturating_sub(LOG_LIMIT);
        self.state.log.drain(..over);
        self.state.job = self.jobs.status();
        self.tick_loader();
        self.sync_run();
        self.sync_compare();
        self.tick_helper();
    }

    fn finished(&mut self, f: JobFinished) {
        let step = self.state.step.take();
        let output = match f.result {
            Ok(output) => {
                self.info(text::job_done(&f.label, &text::duration(f.seconds)));
                Some(output)
            }
            Err(StepError::Cancelled) => {
                self.info(text::job_cancelled(&f.label));
                None
            }
            Err(StepError::Failed(m)) => {
                self.error(text::job_failed(&f.label, &m));
                None
            }
        };
        // the step may have added runs or maps
        self.reload_project();
        // a reuse started with the count of written changes (taken also if it failed)
        let current = match &output {
            Some(JobOutput::Slam(done)) => self.state.correct.computed_current(&done.run_dir),
            _ => {
                self.state.correct.reuse_writes = None;
                true
            }
        };
        match (output, step) {
            (Some(JobOutput::Slam(done)), _) => {
                // computed with the corrections as they are now, unless one was written since
                if current {
                    self.state.correct.changed.remove(&done.run_dir);
                }
                self.state.selection = Selection::Run(done.run_dir.clone());
                self.reload_run(&done.run_dir);
                self.load_run_cloud(&done.run_dir);
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
            (Some(JobOutput::Map(done)), _) => {
                self.state.selection = Selection::Map(done.dir.clone());
                self.load_best_of_map(&done.dir);
            }
            (Some(JobOutput::Inspect(report)), Some(Action::Inspect(scan))) => {
                self.state.inspect = Some(InspectResult {
                    scan,
                    report: *report,
                    seconds: f.seconds,
                });
            }
            _ => {}
        }
        if let Some(action) = self.pending.take() {
            self.apply(action);
        }
    }

    pub fn apply(&mut self, action: Action) {
        match action {
            Action::PickNewProject => {
                if let Some(dir) = rfd::FileDialog::new()
                    .set_title(text::DIALOG_NEW_PROJECT)
                    .pick_folder()
                {
                    let name = dir.file_name().map_or_else(
                        || text::DEFAULT_PROJECT_NAME.into(),
                        |n| n.to_string_lossy().into_owned(),
                    );
                    self.apply(Action::NewProject { dir, name });
                }
            }
            Action::PickOpenProject => {
                if let Some(dir) = rfd::FileDialog::new()
                    .set_title(text::DIALOG_OPEN_PROJECT)
                    .pick_folder()
                {
                    self.apply(Action::OpenProject(dir));
                }
            }
            Action::OpenProject(dir) if self.is_open(&dir) => {
                // already shown: nothing to switch, nothing to cancel (a pending switch to
                // another project is dropped, the user chose this one)
                self.pending = None;
                self.state.refresh();
            }
            switch @ (Action::OpenProject(_) | Action::NewProject { .. })
                if self.jobs.is_busy() =>
            {
                // The step belongs to the old project: stop it first, switch when it ended.
                self.jobs.cancel();
                self.pending = Some(switch);
                self.info(text::SWITCH_AFTER_CANCEL.to_string());
            }
            Action::OpenProject(dir) => match Project::open(&dir) {
                Ok(p) => self.set_project(p),
                Err(e) => self.error(e.to_string()),
            },
            Action::NewProject { dir, name } => match Project::create(&dir, &name) {
                Ok(p) => self.set_project(p),
                Err(e) => self.error(e.to_string()),
            },
            Action::PickAddScan => {
                if self.state.project.is_some() {
                    if let Some(dir) = rfd::FileDialog::new()
                        .set_title(text::DIALOG_ADD_SCAN)
                        .pick_folder()
                    {
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
                    if let Some(dir) = rfd::FileDialog::new()
                        .set_title(text::DIALOG_ADD_RUN)
                        .pick_folder()
                    {
                        self.change_project(|p| {
                            p.add_run(&scan, &dir)?;
                            Ok(Selection::Scan(scan.clone()))
                        });
                    }
                }
            }
            Action::Select(selection) => {
                self.state.selection = selection;
                self.load_selected();
                // choosing the shown run again reads it again (e.g. after `driftless correct`)
                if let Some(run) = self.state.correct.run.clone() {
                    if self.state.selected_run().is_some_and(|(_, r)| r.dir == run) {
                        self.reload_run(&run);
                    }
                }
            }
            Action::LoadCloud { path, run } => self.load(CloudSource::File { path, run }),
            // the status bar shows the step if one runs, else the load
            Action::Cancel if self.jobs.is_busy() => self.jobs.cancel(),
            Action::Cancel => {
                self.next_load = None;
                self.loader.cancel();
            }
            Action::DismissMessages => self.state.messages.clear(),
            step @ (Action::Inspect(_)
            | Action::RunSlam(_)
            | Action::Reuse(_)
            | Action::RestoreHistory { .. }
            | Action::RunMap(_)) => {
                if let Err(e) = actions::start_step(&mut self.state, &mut self.jobs, &step) {
                    self.error(e);
                }
            }
            change @ (Action::AddLoop { .. } | Action::SetLoopEnabled { .. }) => {
                self.change_corrections(change)
            }
            Action::Compare { a, b } => self.start_compare(a, b),
            Action::CompareMaps { a, b } => self.start_compare_maps(a, b),
        }
    }

    /// Reads the open project again from disk, keeping the selection.
    fn reload_project(&mut self) {
        let Some(dir) = self.state.project.as_ref().map(|p| p.dir.clone()) else {
            return;
        };
        match Project::open(&dir) {
            Ok(p) => {
                self.state.project = Some(p);
                self.state.refresh();
            }
            Err(e) => self.error(e.to_string()),
        }
    }

    fn set_project(&mut self, project: Project) {
        self.info(text::project_opened(&project.name));
        self.state.set_project(project);
    }

    fn is_open(&self, dir: &Path) -> bool {
        match (&self.state.project, canonical(dir)) {
            (Some(p), Ok(dir)) => p.dir == dir,
            _ => false,
        }
    }

    /// Changes a copy of the project and saves it; only a saved change is shown, so the
    /// project on screen always matches `project.toml`.
    fn change_project(
        &mut self,
        change: impl FnOnce(&mut Project) -> Result<Selection, crate::project::ProjectError>,
    ) {
        let Some(mut project) = self.state.project.clone() else {
            return;
        };
        match change(&mut project).and_then(|sel| project.save().map(|()| sel)) {
            Ok(selection) => {
                self.state.project = Some(project);
                self.state.refresh();
                self.state.selection = selection;
            }
            Err(e) => self.error(e.to_string()),
        }
    }

    fn info(&mut self, text: String) {
        self.message(UiMessage::Info(text));
    }

    fn error(&mut self, text: String) {
        self.message(UiMessage::Error(text));
    }

    fn message(&mut self, m: UiMessage) {
        let messages = &mut self.state.messages;
        messages.push(m);
        let over = messages.len().saturating_sub(MESSAGE_LIMIT);
        messages.drain(..over);
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

    /// One frame of the window (what [`eframe::App::ui`] does; without eframe for tests).
    pub fn show(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        if ctx.input(|i| i.viewport().close_requested()) && !self.close_requested() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
        }
        self.tick();
        if self.take_close() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if self.wants_repaint_while_busy() {
            ctx.request_repaint_after(REPAINT_WHILE_BUSY);
        }
        self.update_title(&ctx);

        let mut actions: Vec<Action> = Vec::new();
        let has_project = self.state.project.is_some();
        egui::Panel::top("menu").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button(text::MENU_PROJECT, |ui| {
                    if ui.button(text::NEW_PROJECT).clicked() {
                        actions.push(Action::PickNewProject);
                    }
                    if ui.button(text::OPEN_PROJECT).clicked() {
                        actions.push(Action::PickOpenProject);
                    }
                    if ui
                        .add_enabled(has_project, egui::Button::new(text::ADD_SCAN))
                        .clicked()
                    {
                        actions.push(Action::PickAddScan);
                    }
                });
            });
        });
        egui::Panel::bottom("status").show(ui, |ui| {
            actions.extend(status::show(ui, &self.state));
        });
        egui::Panel::left("projekt")
            .default_size(LEFT_WIDTH)
            .min_size(LEFT_MIN_WIDTH)
            .show(ui, |ui| {
                actions.extend(left::show(ui, &self.state));
            });
        egui::Panel::right("schritte")
            .default_size(RIGHT_WIDTH)
            .show(ui, |ui| {
                actions.extend(right::show(ui, &mut self.state));
            });
        let mut event = None;
        let (places, avoid_submap) = self.places();
        let overlay = self.overlay.as_ref().map(|run| ViewOverlay {
            run,
            version: self.overlay_version,
            places,
            avoid_submap,
            pick_places: self.state.correct.picking,
        });
        let shown = self.cloud.is_some() || overlay.is_some();
        egui::CentralPanel::no_frame().show(ui, |ui| match (&self.render_state, shown) {
            (Some(rs), true) => {
                event = self
                    .view
                    .show(ui, rs, self.cloud.as_ref(), overlay, &self.state.view);
            }
            (None, true) => {
                ui.centered_and_justified(|ui| ui.weak(text::VIEW_NO_GPU));
            }
            (_, false) => {
                ui.centered_and_justified(|ui| ui.weak(text::VIEW_EMPTY));
            }
        });
        match event {
            Some(ViewEvent::PickedPose(i)) => self.pose_picked(i),
            Some(ViewEvent::TrajectoryHidden) => {
                let m = UiMessage::Info(text::PICK_NEEDS_TRAJECTORY.into());
                self.state.correct.message = Some(m);
            }
            _ => {}
        }
        if let Some(cloud) = &self.cloud {
            let stats = Some(ViewStats {
                drawn: self.view.drawn_points(),
                total: cloud.points.len(),
                frame_ms: self.view.frame_ms(),
                gpu: self.view.gpu_timed(),
            });
            // the right panel was drawn before the view: show the new numbers next frame
            if stats != self.state.view_stats {
                self.state.view_stats = stats;
                ctx.request_repaint();
            }
            if let Some(ViewEvent::PickedPoint(i)) = event {
                self.state.picked = Some(point_info(cloud, i));
            }
        }
        for action in actions {
            self.apply(action);
        }
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

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if let Some(last) = self.last_to_store() {
            storage.set_string(LAST_PROJECT_KEY, last);
        }
    }
}
