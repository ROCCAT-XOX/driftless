//! Every text the program shows, in one place (German): fixed texts as constants, texts
//! with values as small functions.

pub const APP_TITLE: &str = "Driftless";
/// Name of a new project whose folder has no name (a drive root).
pub const DEFAULT_PROJECT_NAME: &str = "Projekt";

// Project
pub const NEW_PROJECT: &str = "Neues Projekt …";
pub const OPEN_PROJECT: &str = "Projekt öffnen …";
pub const ADD_SCAN: &str = "Scan hinzufügen …";
pub const ADD_RUN: &str = "SLAM-Ordner hinzufügen …";
pub const MENU_PROJECT: &str = "Projekt";
pub const NO_PROJECT: &str = "Kein Projekt geöffnet";
pub const NO_SCANS: &str = "Noch kein Scan im Projekt";
pub const NO_RUNS: &str = "Noch kein SLAM-Lauf";
/// After the name of a scan whose folder cannot be read.
pub const SCAN_MISSING: &str = "fehlt";

// Panels
pub const PANEL_PROJECT: &str = "Projekt";
pub const PANEL_STEPS: &str = "Schritte";
pub const PANEL_PROPERTIES: &str = "Eigenschaften";
pub const VIEW_EMPTY: &str = "Keine Wolke geladen – links eine Karte oder Wolke wählen";

// Status bar
pub const CANCEL: &str = "Abbrechen";
pub const CANCELLING: &str = "Wird abgebrochen …";
pub const READY: &str = "Bereit";
pub const LOG: &str = "Protokoll";
pub const DISMISS: &str = "Schließen";
/// After the label of a job that ended, e.g. „SLAM rechnen fertig (12 s)“.
pub const JOB_DONE: &str = "fertig";
pub const JOB_CANCELLED: &str = "abgebrochen";

// Dialogs
pub const DIALOG_NEW_PROJECT: &str = "Leeren Ordner für das neue Projekt wählen";
pub const DIALOG_OPEN_PROJECT: &str = "Projektordner wählen";
pub const DIALOG_ADD_SCAN: &str = "Scan-Ordner wählen";
pub const DIALOG_ADD_RUN: &str = "SLAM-Ordner wählen (mit slam.json)";

// Messages
pub const SWITCH_AFTER_CANCEL: &str =
    "Der laufende Schritt wird abgebrochen; danach wird das andere Projekt geöffnet";
pub const NOT_YET: &str = "Noch nicht verfügbar";
pub const LAST_PROJECT_GONE: &str = "Zuletzt geöffnetes Projekt nicht lesbar";
pub const CLOSE_AFTER_CANCEL: &str =
    "Der laufende Schritt wird abgebrochen; danach schließt das Fenster";
const USAGE: &str = "Aufruf: driftless-app [projekt-ordner]";

pub fn project_opened(name: &str) -> String {
    format!("Projekt {name} geöffnet")
}

pub fn scan_missing(name: &str) -> String {
    format!("{name} ({SCAN_MISSING})")
}

pub fn log_header(lines: usize) -> String {
    format!("{LOG} ({lines})")
}

pub fn job_done(label: &str, duration: &str) -> String {
    format!("{label} {JOB_DONE} ({duration})")
}

pub fn job_cancelled(label: &str) -> String {
    format!("{label} {JOB_CANCELLED}")
}

pub fn job_failed(label: &str, error: &str) -> String {
    format!("{label}: {error}")
}

pub fn last_project_gone(error: &str) -> String {
    format!("{LAST_PROJECT_GONE}: {error}")
}

// Start (command line)
pub fn unknown_option(arg: &str) -> String {
    format!("unbekannte Option {arg}; {USAGE}")
}

pub fn one_project_only() -> String {
    format!("nur ein Projektordner möglich; {USAGE}")
}

/// An error at start, before the window exists (stderr and, without console, a message box).
pub fn start_error(error: &str) -> String {
    format!("Fehler: {error}")
}

pub fn window_failed(error: &str) -> String {
    format!("Fehler: Fenster nicht startbar: {error}")
}

/// „12 s“ below a minute, else „3:05 min“ (seconds cut off, not rounded).
pub fn duration(seconds: f64) -> String {
    let s = seconds.max(0.0) as u64;
    if s < 60 {
        format!("{s} s")
    } else {
        format!("{}:{:02} min", s / 60, s % 60)
    }
}
