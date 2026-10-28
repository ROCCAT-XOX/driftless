//! Background jobs: one step at a time on its own thread, so the UI thread never blocks.
//!
//! The UI starts a job with [`JobRunner::start`], which returns at once, and then looks at
//! [`JobRunner::status`], [`JobRunner::take_log`] and [`JobRunner::poll`] on every frame. A
//! panic inside a job becomes an error instead of taking the program down. Several runners
//! can exist side by side (a loader and a helper for quick jobs); each runs one job.
//!
//! Contract with the UI: the runner wakes the UI only once, when a job ends (the `repaint`
//! callback, called after the result is ready, so the next [`JobRunner::poll`] finds it).
//! Progress does not wake it. So the UI must
//! - call [`JobRunner::poll`] on every frame (a result is handed out only there, and the
//!   runner stays busy until it was taken), and
//! - while [`JobRunner::is_busy`], ask for the next frame itself (`request_repaint_after`,
//!   about 10 per second), so that phase, counter and runtime move.
//!
//! Closing: dropping a runner cancels its job and then waits for the thread **without a
//! time limit**. The steps check the cancel flag between their units of work and only
//! write files in whole pieces after their last check, so waiting is what keeps a file
//! that is being written from being cut off (a timeout would end the process in the middle
//! of a write). The price is that closing takes as long as the longest unit of work.

use std::any::Any;
use std::collections::BTreeMap;
use std::fmt;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Instant;

use crate::cloud::{LoadedCloud, PairCloud};
use crate::project::{MapDone, MapStatus, SlamDone, SurfaceDone};
use crate::surface::LoadedSurface;
use crate::ui::compare::{Comparison, MapComparison};
use crate::ui::correct::{Corrected, RunLoaded};
use driftless_io::progress::{LogLine, Progress, ProgressSnapshot};
use driftless_steps::export::ExportReport;
use driftless_steps::inspect;
use driftless_steps::StepError;

/// What a finished job hands back.
#[derive(Debug)]
pub enum JobOutput {
    Inspect(Box<inspect::Report>),
    /// With the run folder, so the program can select it.
    Slam(Box<SlamDone>),
    /// With the map folder.
    Map(Box<MapDone>),
    /// The step „Zusammenführen“, with the new model folder.
    Model(PathBuf),
    /// The step „Oberfläche rechnen“, with the surface folder.
    SurfaceStep(Box<SurfaceDone>),
    /// A cloud with its levels of detail (loader runner).
    Cloud(Box<LoadedCloud>),
    /// A model with one of its scans beside it, for point pairs (loader runner).
    PairCloud(Box<PairCloud>),
    /// A surface with its pages (surface loader).
    Surface(Box<LoadedSurface>),
    /// The status of every map folder of the tree (helper runner).
    MapStatus(Box<BTreeMap<PathBuf, MapStatus>>),
    /// The start↔end gaps of history states, by state folder (helper runner).
    HistoryGaps(Box<BTreeMap<PathBuf, f64>>),
    /// The overlay and the corrections of a run (helper runner).
    Run(Box<RunLoaded>),
    /// `corrections.toml` after a change (helper runner).
    Corrected(Box<Corrected>),
    /// A project folder read: `project.toml` and the tree (tree reader).
    ProjectRead(Box<crate::ui::ProjectRead>),
    /// Two trajectories compared (helper runner).
    Compare(Box<Comparison>),
    /// Two `map.json` side by side (helper runner).
    MapCompare(Box<MapComparison>),
    /// The step „Exportieren“: what it wrote.
    Exported(Box<ExportReport>),
    /// Nothing to hand back; the job's effect is on disk.
    Done,
}

/// Why `start` did not start the job.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StartError {
    /// A job is running (its label), or its result was not taken yet.
    Busy(String),
    /// The system gave no thread (the reason).
    Spawn(String),
}

impl fmt::Display for StartError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StartError::Busy(label) => write!(f, "Es läuft schon: {label}"),
            StartError::Spawn(e) => write!(f, "Hintergrund-Thread nicht startbar: {e}"),
        }
    }
}

impl std::error::Error for StartError {}

/// The running job at one moment.
#[derive(Debug, Clone)]
pub struct JobStatus {
    pub label: String,
    pub progress: ProgressSnapshot,
    pub seconds: f64,
    /// Cancel was requested, the job has not stopped yet.
    pub cancelling: bool,
}

/// A job that has ended, handed out once by [`JobRunner::poll`].
#[derive(Debug)]
pub struct JobFinished {
    pub label: String,
    pub result: Result<JobOutput, StepError>,
    pub seconds: f64,
}

struct Running {
    label: String,
    progress: Progress,
    started: Instant,
    rx: Receiver<Result<JobOutput, StepError>>,
    thread: JoinHandle<()>,
}

pub struct JobRunner {
    repaint: Arc<dyn Fn() + Send + Sync>,
    running: Option<Running>,
    /// Messages of jobs that ended before the UI took them.
    log: Vec<LogLine>,
}

impl JobRunner {
    /// `repaint` is called from the job thread when a job ends, to wake the UI.
    pub fn new(repaint: Arc<dyn Fn() + Send + Sync>) -> Self {
        JobRunner {
            repaint,
            running: None,
            log: Vec::new(),
        }
    }

    /// Starts `job` on a new thread and returns at once. Refused while a job runs or while
    /// its result has not been taken with [`JobRunner::poll`].
    pub fn start(
        &mut self,
        label: impl Into<String>,
        job: impl FnOnce(&Progress) -> Result<JobOutput, StepError> + Send + 'static,
    ) -> Result<(), StartError> {
        if let Some(r) = &self.running {
            return Err(StartError::Busy(r.label.clone()));
        }
        let label = label.into();
        let progress = Progress::new();
        let (tx, rx) = channel();
        let (p, repaint) = (progress.clone(), self.repaint.clone());
        let thread = std::thread::Builder::new()
            .name(format!("job: {label}"))
            .spawn(move || {
                let result = catch_unwind(AssertUnwindSafe(|| job(&p)))
                    .unwrap_or_else(|payload| Err(panic_error(payload)));
                // The receiver is gone only if the runner was dropped.
                let _ = tx.send(result);
                repaint();
            })
            .map_err(|e| StartError::Spawn(e.to_string()))?;
        self.running = Some(Running {
            label,
            progress,
            started: Instant::now(),
            rx,
            thread,
        });
        Ok(())
    }

    /// Asks the running job to stop; it ends at its next check.
    pub fn cancel(&self) {
        if let Some(r) = &self.running {
            r.progress.cancel();
        }
    }

    /// True from `start` until the result was taken with [`JobRunner::poll`].
    pub fn is_busy(&self) -> bool {
        self.running.is_some()
    }

    pub fn status(&self) -> Option<JobStatus> {
        self.running.as_ref().map(|r| JobStatus {
            label: r.label.clone(),
            progress: r.progress.snapshot(),
            seconds: r.started.elapsed().as_secs_f64(),
            cancelling: r.progress.is_cancelled(),
        })
    }

    /// The finished job, once; `None` while it still runs or nothing ran.
    pub fn poll(&mut self) -> Option<JobFinished> {
        let r = self.running.as_ref()?;
        let result = match r.rx.try_recv() {
            Ok(result) => result,
            Err(TryRecvError::Empty) => return None,
            Err(TryRecvError::Disconnected) => {
                Err(StepError::Failed("interner Fehler: Job beendet".into()))
            }
        };
        let r = self.running.take()?;
        let _ = r.thread.join();
        self.log.extend(r.progress.take_log());
        Some(JobFinished {
            label: r.label,
            result,
            seconds: r.started.elapsed().as_secs_f64(),
        })
    }

    /// Messages of the job since the last call, in order.
    pub fn take_log(&mut self) -> Vec<LogLine> {
        let mut lines = std::mem::take(&mut self.log);
        if let Some(r) = &self.running {
            lines.extend(r.progress.take_log());
        }
        lines
    }
}

impl Drop for JobRunner {
    fn drop(&mut self) {
        if let Some(r) = self.running.take() {
            r.progress.cancel();
            // No time limit, see the module documentation.
            let _ = r.thread.join();
        }
    }
}

fn panic_error(payload: Box<dyn Any + Send>) -> StepError {
    let text = payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "unbekannt".into());
    StepError::Failed(format!("interner Fehler: {text}"))
}
