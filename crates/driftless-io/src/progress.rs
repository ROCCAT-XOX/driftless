//! Progress reporting and cancellation for long steps.
//!
//! A [`Progress`] is a cheap handle (clones share one state) that a step reports its phase,
//! its count and its messages to, and that tells it whether it was cancelled. The CLI gives
//! it a log sink that prints; a desktop program polls [`Progress::snapshot`] and
//! [`Progress::take_log`] from its UI thread while the step runs on another thread.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError, RwLock};

/// Phase and count of a step at one moment.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ProgressSnapshot {
    pub phase: String,
    pub done: u64,
    /// `None` if the size of the phase is not known in advance.
    pub total: Option<u64>,
}

/// A message of a step.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LogLine {
    Info(String),
    Warning(String),
}

type Sink = Box<dyn Fn(&LogLine) + Send + Sync>;
type Observer = Arc<dyn Fn(&ProgressSnapshot) + Send + Sync>;

struct State {
    snapshot: ProgressSnapshot,
    /// Messages since the last `take_log` (only without a sink).
    log: Vec<LogLine>,
}

struct Inner {
    cancelled: AtomicBool,
    state: Mutex<State>,
    sink: Option<Sink>,
    observer: RwLock<Option<Observer>>,
}

/// Shared progress, messages and cancel flag of a step; `Clone` shares the state.
#[derive(Clone)]
pub struct Progress {
    inner: Arc<Inner>,
}

impl Default for Progress {
    fn default() -> Self {
        Progress::new()
    }
}

impl std::fmt::Debug for Progress {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Progress")
            .field("snapshot", &self.snapshot())
            .field("cancelled", &self.is_cancelled())
            .finish()
    }
}

impl Progress {
    /// Progress without a sink: messages are kept for [`Progress::take_log`].
    pub fn new() -> Self {
        Progress::build(None)
    }

    /// Progress whose messages go to `sink` as they come (and are not kept).
    pub fn with_log(sink: impl Fn(&LogLine) + Send + Sync + 'static) -> Self {
        Progress::build(Some(Box::new(sink)))
    }

    fn build(sink: Option<Sink>) -> Self {
        Progress {
            inner: Arc::new(Inner {
                cancelled: AtomicBool::new(false),
                state: Mutex::new(State {
                    snapshot: ProgressSnapshot::default(),
                    log: Vec::new(),
                }),
                sink,
                observer: RwLock::new(None),
            }),
        }
    }

    /// Calls `f` with the new snapshot after every [`Progress::phase`] and
    /// [`Progress::advance`] (on the reporting thread), for all clones. For tests that cancel
    /// in the middle of a step; a clone of this progress captured in `f` keeps it alive.
    #[doc(hidden)]
    pub fn with_observer(self, f: impl Fn(&ProgressSnapshot) + Send + Sync + 'static) -> Self {
        *self
            .inner
            .observer
            .write()
            .unwrap_or_else(PoisonError::into_inner) = Some(Arc::new(f));
        self
    }

    fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.inner
            .state
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
    }

    fn notify(&self, snapshot: ProgressSnapshot) {
        let observer = self
            .inner
            .observer
            .read()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        if let Some(f) = observer {
            f(&snapshot);
        }
    }

    /// Starts a phase: `done` goes back to 0, `total` is its size if known.
    pub fn phase(&self, name: &str, total: Option<u64>) {
        let snapshot = {
            let mut s = self.state();
            s.snapshot = ProgressSnapshot {
                phase: name.to_string(),
                done: 0,
                total,
            };
            s.snapshot.clone()
        };
        self.notify(snapshot);
    }

    /// Counts `n` more items of the current phase as done.
    pub fn advance(&self, n: u64) {
        let snapshot = {
            let mut s = self.state();
            s.snapshot.done = s.snapshot.done.saturating_add(n);
            s.snapshot.clone()
        };
        self.notify(snapshot);
    }

    fn log(&self, line: LogLine) {
        match &self.inner.sink {
            Some(sink) => sink(&line),
            None => self.state().log.push(line),
        }
    }

    pub fn info(&self, text: impl Into<String>) {
        self.log(LogLine::Info(text.into()));
    }

    pub fn warn(&self, text: impl Into<String>) {
        self.log(LogLine::Warning(text.into()));
    }

    /// Asks the step to stop; it returns at its next check.
    pub fn cancel(&self) {
        self.inner.cancelled.store(true, Ordering::Relaxed);
    }

    pub fn is_cancelled(&self) -> bool {
        self.inner.cancelled.load(Ordering::Relaxed)
    }

    pub fn snapshot(&self) -> ProgressSnapshot {
        self.state().snapshot.clone()
    }

    /// Messages since the last call, in order (always empty with a sink).
    pub fn take_log(&self) -> Vec<LogLine> {
        std::mem::take(&mut self.state().log)
    }
}
