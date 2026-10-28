//! `driftless-app [--version] [--synthetic N] [projekt-ordner]`: the Driftless window.

// No console window next to the program on Windows (debug builds keep it for messages).
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use driftless_app::logfile::{self, Level, LogFile, NewEntries};
use driftless_app::ui::{about, parse_args, text, DriftlessApp, UiMessage};
use driftless_app::{icon, render};
use driftless_io::progress::LogLine;
use eframe::egui;
use std::process::ExitCode;
use std::sync::Arc;

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

/// Size of the window and Dock/taskbar symbol drawn at start (pixels).
const ICON_SIZE: u32 = 256;

fn main() -> ExitCode {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(p) => p,
        Err(e) => {
            report(&text::start_error(&e));
            return ExitCode::from(2);
        }
    };
    if args.version {
        println!("{}", about::version_line());
        return ExitCode::SUCCESS;
    }
    // The log file before the window, so that a panic at the start is in it too.
    let log = start_log();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(text::APP_TITLE)
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 500.0])
            .with_icon(egui::IconData {
                rgba: icon::icon_rgba(ICON_SIZE),
                width: ICON_SIZE,
                height: ICON_SIZE,
            }),
        // GPU timestamps where available, for the frame time of the 3D view
        wgpu_options: egui_wgpu::WgpuConfiguration {
            wgpu_setup: egui_wgpu::WgpuSetup::CreateNew(render::with_timestamps(
                egui_wgpu::WgpuSetupCreateNew::without_display_handle(),
            )),
            ..Default::default()
        },
        ..Default::default()
    };
    let window_log = log.clone();
    let result = eframe::run_native(
        text::APP_TITLE,
        options,
        Box::new(move |cc| {
            let app = DriftlessApp::from_creation_context(cc, args);
            Ok(match window_log {
                Some(log) => {
                    // the adapter eframe chose, and the project opened at the start
                    let gpu = cc.wgpu_render_state.as_ref().map(|rs| {
                        let info = rs.adapter.get_info();
                        let details = [
                            format!("{:?}", info.backend),
                            format!("{:?}", info.device_type),
                            format!("{} {}", info.driver, info.driver_info),
                        ];
                        logfile::gpu_text(&info.name, &details)
                    });
                    let project = app.state().project.as_ref().map(|p| p.dir.as_path());
                    log.write_start(gpu.as_deref(), project);
                    Box::new(Logged::new(app, log)) as Box<dyn eframe::App>
                }
                None => Box::new(app),
            })
        }),
    );
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            let message = text::window_failed(&e.to_string());
            if let Some(log) = &log {
                log.write(Level::Error, &message);
            }
            report(&message);
            ExitCode::FAILURE
        }
    }
}

/// Begins the log file of this start ([`logfile`]) and writes every panic into it; without a
/// folder for it the program runs without one.
fn start_log() -> Option<Arc<LogFile>> {
    let dir = logfile::default_dir()?;
    match LogFile::start(&dir, &about::version_line()) {
        Ok(log) => {
            let log = Arc::new(log);
            logfile::install_panic_hook(log.clone());
            Some(log)
        }
        Err(e) => {
            eprintln!(
                "{}",
                text::log_unavailable(&dir.display().to_string(), &e.to_string())
            );
            None
        }
    }
}

/// The window, with the log file: after each frame what is new in the Protokoll (the messages
/// of the steps) and in the status bar goes into the file as well. Everything else is the
/// window's own.
struct Logged {
    app: DriftlessApp,
    log: Arc<LogFile>,
    lines: NewEntries,
    messages: NewEntries,
}

impl Logged {
    fn new(app: DriftlessApp, log: Arc<LogFile>) -> Self {
        let mut logged = Logged {
            app,
            log,
            lines: NewEntries::default(),
            messages: NewEntries::default(),
        };
        // what opening the project at the start said
        logged.write_new();
        logged
    }

    fn write_new(&mut self) {
        let state = self.app.state();
        for line in self.lines.take(&state.log, log_text) {
            let level = match line {
                LogLine::Info(_) => Level::Info,
                LogLine::Warning(_) => Level::Warning,
            };
            self.log.write(level, log_text(line));
        }
        for message in self.messages.take(&state.messages, message_text) {
            let level = match message {
                UiMessage::Info(_) => Level::Message,
                UiMessage::Error(_) => Level::Error,
            };
            self.log.write(level, message_text(message));
        }
    }
}

fn log_text(line: &LogLine) -> &str {
    match line {
        LogLine::Info(t) | LogLine::Warning(t) => t,
    }
}

fn message_text(message: &UiMessage) -> &str {
    match message {
        UiMessage::Info(t) | UiMessage::Error(t) => t,
    }
}

impl eframe::App for Logged {
    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        self.app.logic(ctx, frame);
    }

    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.app.ui(ui, frame);
        self.write_new();
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.app.save(storage);
    }

    fn on_exit(&mut self) {
        self.app.on_exit();
        self.write_new();
        self.log.write(Level::Info, logfile::END_TEXT);
    }

    fn auto_save_interval(&self) -> std::time::Duration {
        self.app.auto_save_interval()
    }

    fn clear_color(&self, visuals: &egui::Visuals) -> [f32; 4] {
        self.app.clear_color(visuals)
    }

    fn persist_egui_memory(&self) -> bool {
        self.app.persist_egui_memory()
    }

    fn raw_input_hook(&mut self, ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        self.app.raw_input_hook(ctx, raw_input);
    }
}

/// An error before or instead of the window: on stderr, and where a release build has no
/// console (Windows) also in a message box.
fn report(message: &str) {
    eprintln!("{message}");
    #[cfg(all(windows, not(debug_assertions)))]
    let _ = rfd::MessageDialog::new()
        .set_level(rfd::MessageLevel::Error)
        .set_title(text::APP_TITLE)
        .set_description(message)
        .set_buttons(rfd::MessageButtons::Ok)
        .show();
}
