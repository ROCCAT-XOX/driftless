//! Every text the window shows, in one place (German). Texts built from values are
//! functions next to the panel that builds them; their fixed parts are here.

pub const APP_TITLE: &str = "Driftless";

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
