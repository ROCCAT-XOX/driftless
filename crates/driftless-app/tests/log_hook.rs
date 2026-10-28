//! The logger of the `log` facade ([`logfile::install_library_logger`]), in a test program of
//! its own: there is one logger per process. The warnings and errors of the libraries (wgpu,
//! eframe, winit report through `log`) go into the log file, nothing below a warning, at most
//! [`logfile::LIBRARY_LINES`] per start; the Protokoll of the window never sees them (they do
//! not pass through it: the logger holds only the file).

use driftless_app::logfile::{self, LogFile};
use std::sync::Arc;

#[test]
fn library_warnings_and_errors_go_into_the_file_only() {
    let dir = driftless_io::testscan::tmp("logfile-bibliotheken");
    let log = Arc::new(LogFile::start(&dir, "Driftless 0.1.0 (test)").unwrap());
    assert!(logfile::install_library_logger(log.clone()));
    // a second one is refused, the first stays
    let other = Arc::new(LogFile::start(&dir.join("andere"), "x").unwrap());
    assert!(!logfile::install_library_logger(other.clone()));

    log::warn!(target: "wgpu_core::device::resource", "Surface lost, recreating");
    log::error!(target: "winit::platform_impl", "event loop error");
    log::info!(target: "eframe::native", "nur eine Info");
    log::debug!(target: "wgpu_hal", "debug");
    // from another thread, as wgpu does from its callbacks
    std::thread::spawn(|| log::warn!(target: "eframe::native::run", "aus einem Thread"))
        .join()
        .unwrap();

    let text = std::fs::read_to_string(log.path()).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert!(
        lines[0].contains(" Start   Driftless 0.1.0 (test) · "),
        "{text}"
    );
    assert!(
        lines[1].ends_with(" Warnung [wgpu_core::device::resource] Surface lost, recreating"),
        "{text}"
    );
    assert!(
        lines[2].ends_with(" Fehler  [winit::platform_impl] event loop error"),
        "{text}"
    );
    assert!(
        lines[3].ends_with(" Warnung [eframe::native::run] aus einem Thread"),
        "{text}"
    );
    assert_eq!(lines.len(), 4, "no info, no debug: {text}");
    let other_text = std::fs::read_to_string(other.path()).unwrap();
    assert!(other_text.is_empty(), "{other_text}");

    // a warning every frame does not fill the disk
    for i in 0..logfile::LIBRARY_LINES + 50 {
        log::warn!(target: "wgpu_core", "Warnung {i}");
    }
    let text = std::fs::read_to_string(log.path()).unwrap();
    let library = text.lines().filter(|l| l.contains(" [wgpu_core] ")).count();
    // three were written before
    assert_eq!(library, logfile::LIBRARY_LINES - 3, "{}", text.len());
    assert!(
        text.lines()
            .last()
            .unwrap()
            .ends_with(logfile::LIBRARY_LIMIT_TEXT),
        "{}",
        text.lines().last().unwrap()
    );
    let _ = std::fs::remove_dir_all(&dir);
}
