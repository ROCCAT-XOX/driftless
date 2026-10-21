//! The program: carries out the panels' actions, owns the job runner and the dialogs.

use super::{left, status, text, Action, AppState, Selection, UiMessage};
use crate::jobs::{JobFinished, JobOutput, JobRunner, StartError};
use crate::project::Project;
use driftless_io::progress::Progress;
use driftless_steps::StepError;
use eframe::egui;
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
const LEFT_WIDTH: f32 = 280.0;

/// The arguments of `driftless-app`: at most one project folder.
pub fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Option<PathBuf>, String> {
    let mut project = None;
    for arg in args {
        if arg.starts_with('-') {
            return Err(format!(
                "unbekannte Option {arg}; Aufruf: driftless-app [projekt-ordner]"
            ));
        }
        if project.is_some() {
            return Err(
                "nur ein Projektordner möglich; Aufruf: driftless-app [projekt-ordner]".into(),
            );
        }
        project = Some(PathBuf::from(arg));
    }
    Ok(project)
}

pub struct DriftlessApp {
    state: AppState,
    /// The processing steps, one at a time. Dropping it (closing the window) cancels the
    /// running step and waits for it.
    jobs: JobRunner,
    /// A project switch that waits until the cancelled step has stopped: until then the
    /// step may still write into the old project, which therefore stays shown.
    pending: Option<Action>,
    /// The window title last sent.
    title: String,
}

impl DriftlessApp {
    /// Without a window; `repaint` wakes the UI when a job ends.
    pub fn new(repaint: Arc<dyn Fn() + Send + Sync>) -> Self {
        DriftlessApp {
            state: AppState::default(),
            jobs: JobRunner::new(repaint),
            pending: None,
            title: String::new(),
        }
    }

    /// For eframe: opens `project`, else the project opened last.
    pub fn from_creation_context(
        cc: &eframe::CreationContext<'_>,
        project: Option<PathBuf>,
    ) -> Self {
        let ctx = cc.egui_ctx.clone();
        let mut app = DriftlessApp::new(Arc::new(move || ctx.request_repaint()));
        if let Some(dir) = project {
            app.apply(Action::OpenProject(dir));
        } else if let Some(last) = cc.storage.and_then(|s| s.get_string(LAST_PROJECT_KEY)) {
            match Project::open(Path::new(&last)) {
                Ok(p) => app.set_project(p),
                Err(e) => app.error(format!("{}: {e}", text::LAST_PROJECT_GONE)),
            }
        }
        app
    }

    pub fn state(&self) -> &AppState {
        &self.state
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

    /// True while a step runs: the window must then keep drawing.
    pub fn wants_repaint_while_busy(&self) -> bool {
        self.jobs.is_busy()
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
    }

    fn finished(&mut self, f: JobFinished) {
        let secs = status::duration(f.seconds);
        match f.result {
            Ok(_) => self.info(format!("{} {} ({secs})", f.label, text::JOB_DONE)),
            Err(StepError::Cancelled) => self.info(format!("{} {}", f.label, text::JOB_CANCELLED)),
            Err(StepError::Failed(m)) => self.error(format!("{}: {m}", f.label)),
        }
        // the step may have added runs or maps
        self.state.refresh();
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
                    let name = dir
                        .file_name()
                        .map_or_else(|| "Projekt".into(), |n| n.to_string_lossy().into_owned());
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
            Action::Select(selection) => self.state.selection = selection,
            Action::Cancel => self.jobs.cancel(),
            Action::DismissMessages => self.state.messages.clear(),
            Action::LoadCloud { .. }
            | Action::Inspect(_)
            | Action::RunSlam(_)
            | Action::Reuse(_)
            | Action::RunMap(_)
            | Action::AddLoop { .. }
            | Action::SetLoopEnabled { .. }
            | Action::RestoreHistory { .. } => self.info(text::NOT_YET.to_string()),
        }
    }

    fn set_project(&mut self, project: Project) {
        self.info(format!("Projekt {} geöffnet", project.name));
        self.state.set_project(project);
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
        self.state.messages.push(UiMessage::Info(text));
    }

    fn error(&mut self, text: String) {
        self.state.messages.push(UiMessage::Error(text));
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
}

impl eframe::App for DriftlessApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.tick();
        if self.wants_repaint_while_busy() {
            ui.ctx().request_repaint_after(REPAINT_WHILE_BUSY);
        }
        self.update_title(ui.ctx());

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
            .show(ui, |ui| {
                actions.extend(left::show(ui, &self.state));
            });
        egui::CentralPanel::default_margins().show(ui, |ui| {
            ui.centered_and_justified(|ui| ui.weak(text::VIEW_EMPTY));
        });
        for action in actions {
            self.apply(action);
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if let Some(p) = &self.state.project {
            storage.set_string(LAST_PROJECT_KEY, p.dir.to_string_lossy().into_owned());
        }
    }
}
