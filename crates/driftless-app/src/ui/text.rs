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

// Steps (right panel); the step names are also the labels of their jobs
pub const STEP_INSPECT: &str = "Prüfen";
pub const STEP_SLAM: &str = "SLAM rechnen";
pub const STEP_REUSE: &str = "Neu rechnen (aus Teilkarten)";
pub const STEP_MAP: &str = "Karte rechnen";
pub const NOT_COMPUTED: &str = "nicht gerechnet";
pub const STALE: &str = "veraltet – Laufbahn neu gerechnet";
pub const NO_CACHE: &str = "Kein Zwischenspeicher submaps.bin – zuerst SLAM rechnen";
pub const SELECT_FOR_STEPS: &str = "Links einen Scan oder einen Lauf wählen";
pub const THIS_MAP: &str = "Diese Karte";
pub const UNKNOWN_SCAN: &str = "Scan nicht im Projekt";
pub const UNKNOWN_RUN: &str = "Lauf nicht im Projekt";
pub const NOT_A_STEP: &str = "Diese Aktion ist kein Schritt";
pub const NOT_REUSED: &str = "noch nicht neu gerechnet";
/// Label of the background job for the map statuses (not shown in the status bar).
pub const MAP_STATUS_JOB: &str = "Karten prüfen";
pub const MAP_SWITCHES: &str = "Einstellungen dieser Karte";
pub const MAP_SWITCHES_UNKNOWN: &str = "Einstellungen unbekannt (map.json fehlt oder ist unlesbar)";
pub const USE_AS_TEMPLATE: &str = "Als Vorlage übernehmen";
pub const YES: &str = "ja";
pub const NO: &str = "nein";

// Properties (right panel)
pub const SLAM_SETTINGS: &str = "SLAM-Einstellungen";
pub const SLAM_SETTINGS_NEW: &str = "Gelten für den nächsten Lauf";
pub const SLAM_SETTINGS_OF_RUN: &str = "Mit diesen Einstellungen gerechnet";
pub const LIDAR_POSE: &str = "LiDAR-Lage im IMU-Rahmen angeben";
pub const LIDAR_POSE_NONE: &str = "LiDAR-Lage: keine (LiDAR = IMU)";
/// The seven numbers of the LiDAR pose, in this order.
pub const LIDAR_POSE_PARTS: [&str; 7] = ["x (m)", "y (m)", "z (m)", "qx", "qy", "qz", "qw"];
pub const LIDAR_TOPIC: &str = "LiDAR-Topic";
pub const IMU_TOPIC: &str = "IMU-Topic";
pub const TOPIC_HINT: &str = "leer: das einzige in den Bags";
pub const MAP_SETTINGS: &str = "Karten-Einstellungen";
pub const MAP_STEP: &str = "Schritt";
pub const STEP_DENSE: &str = "Dichte Wolke";
pub const STEP_CLEAN: &str = "Bereinigen";
pub const STEP_COLOR: &str = "Einfärben";
pub const STEP_ALL: &str = "Alles";
pub const GHOSTS: &str = "Geister entfernen";
pub const OUTLIERS: &str = "Ausreißer entfernen";
pub const THROUGH: &str = "Durchblick (Fenster) entfernen";
pub const SMOOTH: &str = "Wände glätten";
pub const HIGHLIGHTS: &str = "Glanzlichter abwerten (experimentell)";
pub const VIGNETTING: &str = "Randabfall der Kameras schätzen";
pub const VOXEL: &str = "Voxelgröße (m)";
pub const DOWNSCALE: &str = "Bilder verkleinern";
/// The choices of [`DOWNSCALE`] for 1, 2 and 4.
pub const DOWNSCALE_CHOICES: [(u32, &str); 3] = [(1, "1/1"), (2, "1/2"), (4, "1/4")];

// Check of a scan (right panel)
pub const INSPECT_RESULT: &str = "Prüfbericht";
pub const DEVICE: &str = "Gerät";
pub const DEVICE_UNKNOWN: &str = "unbekannt";
pub const DURATION: &str = "Dauer";
pub const TOPICS: &str = "Topics";
pub const TOPIC: &str = "Topic";
pub const TOPIC_TYPE: &str = "Typ";
pub const TOPIC_COUNT: &str = "Anzahl";
pub const TOPIC_RATE: &str = "Rate";
pub const PROBLEMS: &str = "Probleme";
pub const NO_PROBLEMS: &str = "keine";

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

/// „fertig (12,3 s)“; „fertig“ without a known runtime.
pub fn step_done(seconds: Option<f64>) -> String {
    match seconds {
        Some(s) => format!("{JOB_DONE} ({} s)", decimal(s, 1)),
        None => JOB_DONE.to_string(),
    }
}

pub fn unknown_scan(id: &str) -> String {
    format!("{UNKNOWN_SCAN}: {id}")
}

pub fn unknown_run(dir: &std::path::Path) -> String {
    format!("{UNKNOWN_RUN}: {}", dir.display())
}

/// „Einfärben wird übersprungen: keine Kamerabilder …“ (step „Alles“ on a scan without them).
pub fn color_skipped(why: &str) -> String {
    format!("{STEP_COLOR} wird übersprungen: {why}")
}

/// „Geister entfernen: ja“ – a recorded switch of a map.
pub fn switch_value(label: &str, on: bool) -> String {
    format!("{label}: {}", if on { YES } else { NO })
}

/// „Voxelgröße (m): 0,050“.
pub fn voxel_value(voxel: f64) -> String {
    format!("{VOXEL}: {}", decimal(voxel, 3))
}

/// „Bilder verkleinern: 1/2“.
pub fn downscale_value(downscale: u32) -> String {
    let choice = DOWNSCALE_CHOICES
        .iter()
        .find(|(v, _)| *v == downscale)
        .map_or_else(|| format!("1/{downscale}"), |(_, l)| l.to_string());
    format!("{DOWNSCALE}: {choice}")
}

/// Why a step button is locked while another step runs.
pub fn busy(label: &str) -> String {
    format!("Es läuft schon: {label}")
}

/// „x (m): 0,1“ – one number of a run's LiDAR pose.
pub fn pose_part(part: &str, value: f64) -> String {
    format!("{part}: {}", decimal(value, 4))
}

/// „LiDAR-Topic: /points“, or the hint without one.
pub fn topic_of_run(what: &str, topic: Option<&str>) -> String {
    format!("{what}: {}", topic.unwrap_or(TOPIC_HINT))
}

/// Model and serial number of the device, as far as known.
pub fn device(model: Option<&str>, sn: Option<&str>) -> String {
    match (model, sn) {
        (Some(m), Some(sn)) => format!("{m} ({sn})"),
        (Some(x), None) | (None, Some(x)) => x.to_string(),
        (None, None) => DEVICE_UNKNOWN.to_string(),
    }
}

/// „10,0 Hz“, „–“ without a rate.
pub fn rate(hz: Option<f64>) -> String {
    hz.map_or_else(|| "–".to_string(), |r| format!("{} Hz", decimal(r, 1)))
}

/// `x` with `digits` decimals and a decimal comma.
pub fn decimal(x: f64, digits: usize) -> String {
    format!("{x:.digits$}").replace('.', ",")
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
