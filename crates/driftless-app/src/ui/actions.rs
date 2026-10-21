//! The steps of the right panel as jobs: [`start_step`] checks what the step needs, then hands
//! it to the job runner. All work on disk – also creating the run or map folder and taking
//! over earlier stages – happens in the job ([`run_slam_job`], [`run_map_job`]), so the
//! window never waits, and a cancelled or failed step leaves no new folder behind.

use super::{text, Action, AppState};
use crate::jobs::{JobOutput, JobRunner};
use crate::project::{run_map_job, run_slam_job, ScanState, SlamJob};
use driftless_steps::inspect;

/// Starts the job of a step action (`Inspect`, `RunSlam`, `Reuse`, `RunMap`) and records it
/// in [`AppState::step`]. Refused with a German message while a job runs, for a scan that
/// cannot be read, a run without `submaps.bin` (reuse), invalid map settings, or any other
/// action.
pub fn start_step(
    state: &mut AppState,
    runner: &mut JobRunner,
    action: &Action,
) -> Result<(), String> {
    if let Some(job) = runner.status() {
        return Err(text::busy(&job.label));
    }
    let project = state.project.clone().ok_or(text::NO_PROJECT)?;
    let started = match action {
        Action::Inspect(id) => {
            let scan = ready_scan(state, id)?;
            runner.start(text::STEP_INSPECT, move |p| {
                let report = inspect::inspect(&scan, p)?;
                Ok(JobOutput::Inspect(Box::new(report)))
            })
        }
        Action::RunSlam(id) => {
            ready_scan(state, id)?;
            let settings = state.slam_settings.clone();
            settings.config().map_err(|e| e.to_string())?;
            let id = id.clone();
            runner.start(text::STEP_SLAM, move |p| {
                let done = run_slam_job(&project, &id, &SlamJob::Full(settings), p)?;
                Ok(JobOutput::Slam(Box::new(done)))
            })
        }
        Action::Reuse(dir) => {
            let (scan_id, run) = state
                .run(dir)
                .map(|(s, r)| (s.id.clone(), r.clone()))
                .ok_or_else(|| unknown_run(dir))?;
            ready_scan(state, &scan_id)?;
            if !run.has_cache {
                return Err(text::NO_CACHE.into());
            }
            runner.start(text::STEP_REUSE, move |p| {
                let done = run_slam_job(&project, &scan_id, &SlamJob::Reuse(run), p)?;
                Ok(JobOutput::Slam(Box::new(done)))
            })
        }
        Action::RunMap(dir) => {
            let (scan_id, run) = state
                .run(dir)
                .map(|(s, r)| (s.id.clone(), r.clone()))
                .ok_or_else(|| unknown_run(dir))?;
            let scan = ready_scan(state, &scan_id)?;
            let settings = state.map_settings.clone();
            settings.check().map_err(|e| e.to_string())?;
            runner.start(text::STEP_MAP, move |p| {
                let done = run_map_job(&run, &scan, &settings, p)?;
                Ok(JobOutput::Map(Box::new(done)))
            })
        }
        other => return Err(format!("kein Schritt: {other:?}")),
    };
    started.map_err(|e| e.to_string())?;
    state.step = Some(action.clone());
    state.job = runner.status();
    Ok(())
}

/// The folder of the scan `id`, if it can be read (its reason otherwise).
fn ready_scan(state: &AppState, id: &str) -> Result<std::path::PathBuf, String> {
    let entry = state
        .project
        .as_ref()
        .and_then(|p| p.scan(id))
        .ok_or_else(|| format!("{}: {id}", text::UNKNOWN_SCAN))?;
    match state.scan_node(id).map(|n| &n.state) {
        Some(ScanState::Missing(why)) => Err(why.clone()),
        _ => Ok(entry.path.clone()),
    }
}

fn unknown_run(dir: &std::path::Path) -> String {
    format!("{}: {}", text::UNKNOWN_RUN, dir.display())
}
