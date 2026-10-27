//! `driftless-app [--version] [--synthetic N] [projekt-ordner]`: the Driftless window.

// No console window next to the program on Windows (debug builds keep it for messages).
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use driftless_app::ui::{about, parse_args, text, DriftlessApp};
use driftless_app::{icon, render};
use eframe::egui;
use std::process::ExitCode;

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
    let result = eframe::run_native(
        text::APP_TITLE,
        options,
        Box::new(move |cc| Ok(Box::new(DriftlessApp::from_creation_context(cc, args)))),
    );
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            report(&text::window_failed(&e.to_string()));
            ExitCode::FAILURE
        }
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
