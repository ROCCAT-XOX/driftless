//! The window „Über Driftless“ (☰ → „Über Driftless …“): version, the commit it was built
//! from, licence, repository, the German guide („Anleitung“), where the command line
//! `driftless` is, and the licence notices of the third-party parts. Also how the system opens a
//! file, a folder or an address ([`open_with_system`]), and „Protokoll-Ordner öffnen“ of ☰
//! ([`open_log_folder`]).

use super::text;
use super::theme::DARK;
use eframe::egui::{self, Context};
use std::ffi::OsStr;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

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
/// The licence notices of the third-party parts, as the package scripts write them
/// (packaging/about.toml).
pub const NOTICES_NAME: &str = "THIRD-PARTY-NOTICES.html";
/// The German guide for colleagues (`docs/anleitung.md`) on the main branch of the repository.
pub const GUIDE_URL: &str = concat!(env!("CARGO_PKG_REPOSITORY"), "/blob/main/docs/anleitung.md");

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

/// Where the licence notices may be for a program in folder `exe_dir`, in this order: next to
/// it (the Windows install folder), in `Resources` of the bundle (macOS: the program is in
/// `Contents/MacOS`), and in `target/package`, where the package scripts write them (a build in
/// `target/debug` or `target/release`).
pub fn notices_candidates(exe_dir: &Path) -> Vec<PathBuf> {
    let mut candidates = vec![exe_dir.join(NOTICES_NAME)];
    if let Some(parent) = exe_dir.parent() {
        candidates.push(parent.join("Resources").join(NOTICES_NAME));
        candidates.push(parent.join("package").join(NOTICES_NAME));
    }
    candidates
}

/// The first of [`notices_candidates`] that is a file.
pub fn notices_path_in(exe_dir: &Path) -> Option<PathBuf> {
    notices_candidates(exe_dir)
        .into_iter()
        .find(|p| p.is_file())
}

/// The licence notices of this program, if they are there.
pub fn notices_path() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    notices_path_in(exe.parent()?)
}

/// What opens `target` (a file, a folder or a web address) with the program the system has
/// for it: `open` on macOS, the Explorer on Windows, `xdg-open` elsewhere.
pub fn open_command(target: &OsStr) -> Command {
    let program = if cfg!(target_os = "macos") {
        "open"
    } else if cfg!(windows) {
        "explorer"
    } else {
        "xdg-open"
    };
    let mut command = Command::new(program);
    command
        .arg(target)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    command
}

/// Opens `target` ([`open_command`]) without waiting; only a program that does not start is an
/// error (the Explorer reports nothing useful in its exit code).
pub fn open_with_system(target: &OsStr) -> std::io::Result<()> {
    let mut child = open_command(target).spawn()?;
    // Waited for elsewhere, so that no ended process is left over.
    std::thread::Builder::new()
        .name("öffnen".into())
        .spawn(move || {
            let _ = child.wait();
        })?;
    Ok(())
}

/// „Protokoll-Ordner öffnen“ (☰): the folder of the log file ([`crate::logfile`]) in the file
/// manager, made if missing. The error is a message for the status bar.
pub fn open_log_folder() -> Result<(), String> {
    let dir = crate::logfile::default_dir().ok_or_else(|| text::LOG_FOLDER_UNKNOWN.to_string())?;
    std::fs::create_dir_all(&dir)
        .and_then(|()| open_with_system(dir.as_os_str()))
        .map_err(|e| text::open_failed(&dir.display().to_string(), &e.to_string()))
}

/// Shows the window while `open` ([`super::AppState::about_open`]); its × closes it.
pub fn show(ctx: &Context, open: &mut bool) {
    show_with(ctx, open, notices_path().as_deref(), cli_path().as_deref());
}

/// [`show`] with the licence notices and the command line where they were found.
pub fn show_with(ctx: &Context, open: &mut bool, notices: Option<&Path>, cli: Option<&Path>) {
    // why the last thing to open did not open, until something opens
    let failed_id = egui::Id::new("ueber_nicht_geoeffnet");
    egui::Window::new(text::ABOUT_TITLE)
        .id(egui::Id::new("ueber"))
        .open(open)
        .collapsible(false)
        .resizable(false)
        .default_width(WIDTH)
        .show(ctx, |ui| {
            let mut opened = None;
            ui.heading(text::about_version(VERSION));
            ui.label(text::about_commit(COMMIT));
            ui.label(text::ABOUT_LICENSE);
            // A link of egui opens nothing here (eframe without its `links` feature).
            let repository = env!("CARGO_PKG_REPOSITORY");
            if ui.link(repository).clicked() {
                opened = Some((
                    repository.to_string(),
                    open_with_system(repository.as_ref()),
                ));
            }
            if ui
                .link(text::ABOUT_GUIDE)
                .on_hover_text(GUIDE_URL)
                .clicked()
            {
                opened = Some((GUIDE_URL.to_string(), open_with_system(GUIDE_URL.as_ref())));
            }
            ui.separator();
            ui.label(cli_line(cli));
            match notices {
                Some(path) => {
                    if ui.button(text::ABOUT_NOTICES).clicked() {
                        let target = path.display().to_string();
                        opened = Some((target, open_with_system(path.as_os_str())));
                    }
                }
                None => {
                    ui.label(text::ABOUT_NOTICES_MISSING);
                }
            }
            match opened {
                Some((_, Ok(()))) => ui.data_mut(|d| d.remove::<String>(failed_id)),
                Some((target, Err(e))) => {
                    let failed = text::open_failed(&target, &e.to_string());
                    ui.data_mut(|d| d.insert_temp(failed_id, failed));
                }
                None => {}
            }
            if let Some(failed) = ui.data(|d| d.get_temp::<String>(failed_id)) {
                ui.colored_label(DARK.error, failed);
            }
        });
}
