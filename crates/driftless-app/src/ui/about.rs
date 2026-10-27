//! The window „Über Driftless“ (☰ → „Über Driftless …“): version, the commit it was built
//! from, licence, repository, and where the command line `driftless` is.

use super::text;
use eframe::egui::{self, Context};
use std::path::{Path, PathBuf};

/// The version of the program.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
/// The commit the program was built from (`build.rs`); „unbekannt“ without git.
pub const COMMIT: &str = env!("DRIFTLESS_COMMIT");
/// The file name of the command line next to the program in the package.
pub const CLI_NAME: &str = if cfg!(windows) {
    "driftless.exe"
} else {
    "driftless"
};

/// Width of the window (points).
const WIDTH: f32 = 340.0;

/// `driftless-app --version`: „Driftless 0.1.0 (abc123def)“.
pub fn version_line() -> String {
    format!("{} {VERSION} ({COMMIT})", text::APP_TITLE)
}

/// The command line next to this program, if it is there (as in the packages).
pub fn cli_path() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    cli_path_in(exe.parent()?)
}

/// The command line in `dir`, if it is there.
pub fn cli_path_in(dir: &Path) -> Option<PathBuf> {
    let path = dir.join(CLI_NAME);
    path.is_file().then_some(path)
}

/// The line „Kommandozeile: …“ of the window.
pub fn cli_line(cli: Option<&Path>) -> String {
    match cli {
        Some(p) => text::about_cli(&p.display().to_string()),
        None => text::ABOUT_CLI_MISSING.to_string(),
    }
}

/// Shows the window while `open` ([`super::AppState::about_open`]); its × closes it.
pub fn show(ctx: &Context, open: &mut bool) {
    egui::Window::new(text::ABOUT_TITLE)
        .id(egui::Id::new("ueber"))
        .open(open)
        .collapsible(false)
        .resizable(false)
        .default_width(WIDTH)
        .show(ctx, |ui| {
            ui.heading(text::about_version(VERSION));
            ui.label(text::about_commit(COMMIT));
            ui.label(text::ABOUT_LICENSE);
            let repository = env!("CARGO_PKG_REPOSITORY");
            ui.hyperlink_to(repository, repository);
            ui.separator();
            ui.label(cli_line(cli_path().as_deref()));
        });
}
