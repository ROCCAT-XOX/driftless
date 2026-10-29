//! The steps of the right panel as jobs: [`start_step`] checks what the step needs, then hands
//! it to the job runner. All work on disk – also creating the run or map folder and taking
//! over earlier stages – happens in the job ([`run_slam_job`], [`run_map_job`]), so the
//! window never waits, and a cancelled or failed step leaves no new folder behind. The export
//! ([`Action::RunExport`]) runs the same way, but also without a project. So does the touch-up
//! of a surface ([`Action::Touchup`]), which the window queues itself.

use super::{text, Action, AppState};
use crate::jobs::{JobOutput, JobRunner};
use crate::project::{
    restore_job, run_map_job, run_merge_job, run_model_surface_job, run_pairs_merge_job,
    run_slam_job, run_surface_job, ScanState, SlamJob,
};
use crate::surface::{run_touchup_job, TouchupJob, TouchupView, SURFACE_GLB};
use driftless_steps::export::run_export;
use driftless_steps::inspect;
use driftless_steps::map::MapStep;

/// Starts the job of a step action (`Inspect`, `RunSlam`, `Reuse`, `RestoreHistory`,
/// `RunMap`, `RunSurface`, `RunMerge`, `ApplyPairs`, `RunModelSurface`, `RunExport`,
/// `Touchup`) and records it in [`AppState::running`]. Only the export runs without a project;
/// its checks are the step's own ([`run_export`]). Refused with a German message while a job
/// runs, for a scan that cannot be read, a run without `submaps.bin` (reuse, restore), while a
/// change of the corrections is being written (reuse, restore), for invalid map or surface
/// settings, a map without `map.json` (surface), fewer than two scans to merge, a model not in
/// the project, a surface folder not in the tree (touch-up), or any other action.
pub fn start_step(
    state: &mut AppState,
    runner: &mut JobRunner,
    action: &Action,
) -> Result<(), String> {
    if let Some(job) = runner.status() {
        return Err(text::busy(&job.label));
    }
    if let Action::RunExport(req) = action {
        let req = req.clone();
        runner
            .start(text::EXPORT_TITLE, move |p| {
                Ok(JobOutput::Exported(Box::new(run_export(&req, p)?)))
            })
            .map_err(|e| e.to_string())?;
        state.running = Some(action.clone());
        state.job = runner.status();
        return Ok(());
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
            if let Some(why) = super::correct::writing(state) {
                return Err(why);
            }
            let writes = state.correct.writes_of(dir);
            state.correct.reuse_writes = Some((dir.clone(), writes));
            runner.start(text::STEP_REUSE, move |p| {
                let done = run_slam_job(&project, &scan_id, &SlamJob::Reuse(run), p)?;
                Ok(JobOutput::Slam(Box::new(done)))
            })
        }
        Action::RestoreHistory { run: dir, nr } => {
            let (scan_id, run) = state
                .run(dir)
                .map(|(s, r)| (s.id.clone(), r.clone()))
                .ok_or_else(|| unknown_run(dir))?;
            ready_scan(state, &scan_id)?;
            if !run.has_cache {
                return Err(text::NO_CACHE.into());
            }
            if let Some(why) = super::correct::writing(state) {
                return Err(why);
            }
            let writes = state.correct.writes_of(dir);
            state.correct.reuse_writes = Some((dir.clone(), writes));
            let nr = *nr;
            runner.start(text::STEP_RESTORE, move |p| {
                // in the job: the recompute reads the switches just written
                let done = restore_job(&project, &scan_id, &run, nr, p)?;
                Ok(JobOutput::Slam(Box::new(done)))
            })
        }
        Action::RunMap(dir) => {
            let (scan_id, run) = state
                .run(dir)
                .map(|(s, r)| (s.id.clone(), r.clone()))
                .ok_or_else(|| unknown_run(dir))?;
            let scan = ready_scan(state, &scan_id)?;
            let mut settings = state.map_settings.clone();
            settings.switches.gpu = state.gpu;
            settings.check().map_err(|e| e.to_string())?;
            // `Alles` skips colouring with a warning, as `driftless map` does
            if settings.step == MapStep::Color {
                if let Some(why) = state.scan_node(&scan_id).and_then(|s| s.no_color.clone()) {
                    return Err(why);
                }
            }
            runner.start(text::STEP_MAP, move |p| {
                let done = run_map_job(&run, &scan, &settings, p)?;
                Ok(JobOutput::Map(Box::new(done)))
            })
        }
        Action::RunSurface(dir) => {
            let (scan_id, run, map) = state
                .map(dir)
                .map(|(s, r, m)| (s.id.clone(), r.clone(), m.clone()))
                .ok_or_else(|| text::unknown_map(dir))?;
            let scan = ready_scan(state, &scan_id)?;
            let settings = state.surface_settings.clone();
            let gpu = state.gpu;
            settings.config.check().map_err(|e| e.to_string())?;
            if map.settings.is_none() {
                return Err(text::SURFACE_NEEDS_MAP.into());
            }
            runner.start(text::STEP_SURFACE, move |p| {
                let done = run_surface_job(&run, &scan, &map, &settings, gpu, p)?;
                Ok(JobOutput::SurfaceStep(Box::new(done)))
            })
        }
        Action::RunMerge {
            picks,
            include_unsure,
        } => {
            if picks.len() < 2 {
                return Err(text::MERGE_NEEDS_TWO.into());
            }
            for pick in picks {
                ready_scan(state, &pick.scan_id)?;
            }
            let (picks, include_unsure) = (picks.clone(), *include_unsure);
            runner.start(text::MERGE_START, move |p| {
                let dir = run_merge_job(&project, &picks, include_unsure, p)?;
                Ok(JobOutput::Model(dir))
            })
        }
        Action::ApplyPairs { model, entry } => {
            let model = state
                .model(model)
                .cloned()
                .ok_or_else(|| text::unknown_model(model))?;
            for m in &model.report.members {
                if let Some(scan) = state.scan_at(&m.scan) {
                    ready_scan(state, &scan.id)?;
                }
            }
            let entry = entry.clone();
            runner.start(text::MERGE_START, move |p| {
                let dir = run_pairs_merge_job(&project, &model, entry, p)?;
                Ok(JobOutput::Model(dir))
            })
        }
        Action::RunModelSurface { model } => {
            let model = state
                .model(model)
                .cloned()
                .ok_or_else(|| text::unknown_model(model))?;
            let settings = state.surface_settings.clone();
            let gpu = state.gpu;
            settings.config.check().map_err(|e| e.to_string())?;
            runner.start(text::STEP_SURFACE, move |p| {
                let done = run_model_surface_job(&model, &settings, gpu, p)?;
                Ok(JobOutput::SurfaceStep(Box::new(done)))
            })
        }
        Action::Touchup { surface, step } => {
            if !state.all_surfaces().any(|s| s.dir == *surface) {
                return Err(text::not_in_tree(surface));
            }
            // the surface shown gets its touched-up state for the view as well
            let view = state
                .surface
                .as_ref()
                .filter(|s| s.path == surface.join(SURFACE_GLB))
                .map(|s| TouchupView {
                    load: state.touchup.load,
                    origin: s.origin,
                    shown: state.touchup.shown,
                });
            let job = TouchupJob {
                surface: surface.clone(),
                step: *step,
                view,
            };
            runner.start(text::STEP_TOUCHUP, move |p| {
                Ok(JobOutput::Touchup(Box::new(run_touchup_job(&job, p)?)))
            })
        }
        _ => return Err(text::NOT_A_STEP.into()),
    };
    started.map_err(|e| e.to_string())?;
    state.running = Some(action.clone());
    state.job = runner.status();
    Ok(())
}

/// The folder of the scan `id`, if it can be read (its reason otherwise).
fn ready_scan(state: &AppState, id: &str) -> Result<std::path::PathBuf, String> {
    let entry = state
        .project
        .as_ref()
        .and_then(|p| p.scan(id))
        .ok_or_else(|| text::unknown_scan(id))?;
    match state.scan_node(id).map(|n| &n.state) {
        Some(ScanState::Missing(why)) => Err(why.clone()),
        _ => Ok(entry.path.clone()),
    }
}

fn unknown_run(dir: &std::path::Path) -> String {
    text::unknown_run(dir)
}
