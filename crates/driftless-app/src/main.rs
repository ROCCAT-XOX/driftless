//! `driftless-app [projekt-ordner]`: the Driftless window.

// No console window next to the program on Windows (debug builds keep it for messages).
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use driftless_app::ui::{parse_args, text, DriftlessApp};
use eframe::egui;
use std::process::ExitCode;

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

fn main() -> ExitCode {
    let project = match parse_args(std::env::args().skip(1)) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Fehler: {e}");
            return ExitCode::from(2);
        }
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(text::APP_TITLE)
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 500.0]),
        ..Default::default()
    };
    let result = eframe::run_native(
        text::APP_TITLE,
        options,
        Box::new(move |cc| Ok(Box::new(DriftlessApp::from_creation_context(cc, project)))),
    );
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Fehler: Fenster nicht startbar: {e}");
            ExitCode::FAILURE
        }
    }
}
