//! Every text the program shows, in one place (German): fixed texts as constants, texts
//! with values as small functions.

pub const APP_TITLE: &str = "Driftless";
/// Name of a new project whose folder has no name (a drive root).
pub const DEFAULT_PROJECT_NAME: &str = "Projekt";

// Project
pub const NEW_PROJECT: &str = "Neues Projekt …";
pub const OPEN_PROJECT: &str = "Projekt öffnen …";
pub const ADD_SCAN: &str = "Scan hinzufügen …";
pub const ADD_RUN: &str = "Lauf hinzufügen …";
pub const MENU_PROJECT: &str = "Projekt";
pub const NO_PROJECT: &str = "Kein Projekt geöffnet";
pub const NO_RUNS: &str = "Noch kein SLAM-Lauf";
/// After the name of a scan whose folder cannot be read.
pub const SCAN_MISSING: &str = "fehlt";

// Rail and the fields of the steps
pub const STEP_DATA: &str = "Daten";
pub const STEP_IMAGES: &str = "Bilder";
pub const STEP_PLAN: &str = "Planung";
/// The settings window ⚙ and its button in the rail.
pub const SETTINGS: &str = "Einstellungen";
pub const EMPTY_NO_PROJECT: &str =
    "Noch kein Projekt offen. Ein Projekt öffnen oder ein neues anlegen, dann Scans hinzufügen.";
pub const EMPTY_NO_SCANS: &str =
    "Das Projekt hat noch keinen Scan. Einen Scan-Ordner hinzufügen, um zu beginnen.";
pub const EMPTY_NO_MAPS: &str = "Noch keine Karte. Zuerst in „Daten“ eine Karte rechnen.";
pub const GO_TO_DATA: &str = "Zu „Daten“";
/// Over the view in „Bilder“ while no surface is shown.
pub const NO_SURFACE_YET: &str = "Noch keine Oberfläche – im Feld „Bilder“ rechnen";
pub const PLAN_COMING: &str = "Hier entstehen Grundriss, Maße und der DWG-Export. Das folgt in \
     einem späteren Baustein.";
pub const ADD_RUN_NEEDS_SCAN: &str = "Zuerst im Feld „Daten“ einen Scan wählen";
pub const ADD_SCAN_NEEDS_PROJECT: &str = "Zuerst ein Projekt öffnen";

// Step 1 „Daten“
/// The status of a scan in the list: no map yet.
pub const STATUS_NEW: &str = "neu";
/// The scan has a map that fits its newest run.
pub const STATUS_DONE: &str = "fertig";
/// The scan has a map, but it no longer fits (or belongs to an older run).
pub const STATUS_STALE: &str = "veraltet";
/// A job of the scan is running.
pub const STATUS_RUNNING: &str = "rechnet";
/// The scan waits in the queue of „Alle rechnen“.
pub const STATUS_QUEUED: &str = "wartet";
pub const PRECISION: &str = "Genauigkeit";
pub const PRECISION_FAST: &str = "Schnell";
pub const PRECISION_EXACT: &str = "Genau";
pub const PRECISION_MAX: &str = "Maximal";
pub const PRECISION_CUSTOM: &str = "Eigene";
pub const PRECISION_MAX_HINT: &str = "braucht bei großen Aufnahmen über 4 GB Arbeitsspeicher";
/// The main button without a selected scan: every new or stale scan, one after the other.
pub const COMPUTE_ALL: &str = "Alle rechnen";
/// The main button with a scan selected: only that scan.
pub const COMPUTE_ONE: &str = "Rechnen";
pub const COMPUTE_HINT: &str =
    "Rechnet je Scan zuerst SLAM, falls nötig, dann die Punktwolke mit der gewählten Genauigkeit";
pub const ALL_COMPUTED: &str = "Alle Scans sind gerechnet";
/// With every scan done, but their maps of another precision than the one chosen.
pub const RECOMPUTE_HINT: &str = "Scan wählen und „Rechnen“ drücken";
/// While the background check of the maps runs: „Alle rechnen“ starts once it is done.
pub const MAP_STATUS_PENDING: &str = "Karten werden geprüft …";
pub const ADVANCED: &str = "Erweitert";

// Step 2 „Bilder“
/// The main button without a selected map: every map without a current surface, one after
/// the other.
pub const COMPUTE_ALL_SURFACES: &str = "Alle Oberflächen rechnen";
pub const COMPUTE_SURFACES_HINT: &str =
    "Rechnet je Karte die Oberfläche mit der gewählten Feinheit, eine nach der anderen";
pub const COMPUTE_SURFACE_HINT: &str =
    "Rechnet die Oberfläche der gewählten Karte mit der gewählten Feinheit";
pub const ALL_SURFACED: &str = "Alle Karten haben eine Oberfläche";
/// With every map surfaced, but their surfaces of another fineness than the one chosen.
pub const SURFACE_RECOMPUTE_HINT: &str = "Karte wählen und „Oberfläche rechnen“ drücken";
/// The voxel size of the surface as a switch (1 / 1,5 / 2 cm, [`SURFACE_VOXEL_CHOICES`]).
pub const SURFACE_FINENESS: &str = "Feinheit";
/// Under the switch at 1 cm.
pub const SURFACE_FINE_HINT: &str =
    "braucht bei großen Aufnahmen über 4 GB Arbeitsspeicher, dunkle Wände bleiben löchrig";
/// The status of a map whose own map no longer fits its run: its surface waits for „Daten“.
pub const STATUS_MAP_STALE: &str = "Karte veraltet";
pub const STATUS_MAP_STALE_HINT: &str =
    "Die Karte passt nicht mehr zu ihrem Lauf – zuerst in „Daten“ neu rechnen";
/// Before the state of a surface of the selected map with the fineness chosen now.
pub const WITH_THIS_FINENESS: &str = "Mit dieser Feinheit";

// Panels
pub const PANEL_STEPS: &str = "Schritte";
pub const PANEL_PROPERTIES: &str = "Eigenschaften";
pub const VIEW_EMPTY: &str = "Keine Wolke geladen – links eine Karte oder Wolke wählen";

// Steps (right panel); the step names are also the labels of their jobs
pub const STEP_INSPECT: &str = "Prüfen";
pub const STEP_SLAM: &str = "SLAM rechnen";
pub const STEP_REUSE: &str = "Neu rechnen (aus Teilkarten)";
pub const STEP_MAP: &str = "Karte rechnen";
pub const STEP_SURFACE: &str = "Oberfläche rechnen";
pub const NOT_COMPUTED: &str = "nicht gerechnet";
pub const STALE: &str = "veraltet – Laufbahn neu gerechnet";
pub const NO_CACHE: &str = "Kein Zwischenspeicher submaps.bin – zuerst SLAM rechnen";
pub const SELECT_FOR_STEPS: &str = "Oben einen Scan oder einen Lauf wählen";
pub const THIS_MAP: &str = "Diese Karte";
pub const THIS_SURFACE: &str = "Diese Oberfläche";
/// The state of a surface that no longer fits its map.
pub const SURFACE_STALE: &str = "veraltet – Karte oder Lauf neu gerechnet";
pub const SURFACE_NEEDS_MAP: &str =
    "Die Karte hat keine map.json – zuerst „Karte rechnen“ (mit „Bereinigen“ oder „Alles“)";
pub const UNKNOWN_SCAN: &str = "Scan nicht im Projekt";
pub const UNKNOWN_RUN: &str = "Lauf nicht im Projekt";
pub const UNKNOWN_MAP: &str = "Karte nicht im Projekt";
pub const NOT_A_STEP: &str = "Diese Aktion ist kein Schritt";
pub const NOT_REUSED: &str = "noch nicht neu gerechnet";
/// Label of the background job of the map and surface statuses (not shown in the status bar).
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
pub const SURFACE_SETTINGS: &str = "Oberflächen-Einstellungen";
pub const SURFACE_SETTINGS_HINT: &str = "Gelten für die nächste Oberfläche dieser Karte";
/// The choices of [`SURFACE_FINENESS`] (m): finer needs more memory and time.
pub const SURFACE_VOXEL_CHOICES: [(f64, &str); 3] =
    [(0.01, "1 cm"), (0.015, "1,5 cm"), (0.02, "2 cm")];
pub const SURFACE_VOXEL_HINT: &str =
    "Kleiner ist feiner, braucht mehr Speicher und Zeit; 1 cm braucht bei großen Aufnahmen über \
     4 GB Arbeitsspeicher, und dunkle Wände bleiben löchrig";
pub const SURFACE_NO_PHOTOS: &str = "ohne Fotos (graue Fläche)";

// The 3D view and its properties (right panel)
pub const VIEW_NO_GPU: &str = "3D-Ansicht nicht verfügbar: keine wgpu-Grafik";
pub const COLOR_MODE: &str = "Farbart";
pub const MODE_RGB: &str = "Farbe";
pub const MODE_INTENSITY: &str = "Intensität";
pub const MODE_QUALITY: &str = "Güte";
pub const POINT_SIZE: &str = "Punktgröße (px)";
pub const ROUND_POINTS: &str = "rund";
pub const BUDGET: &str = "Punktbudget";
/// Range of the point budget.
pub const BUDGET_RANGE: std::ops::RangeInclusive<usize> = 1_000_000..=30_000_000;
pub const CAMERA_MODE: &str = "Bedienung";
pub const ORBIT: &str = "Umkreisen";
pub const FLY: &str = "Fliegen";
pub const LAYERS: &str = "Ebenen";
pub const LAYER_TRAJECTORY: &str = "Laufbahn";
pub const LAYER_ODOMETRY: &str = "Odometrie";
pub const LAYER_LOOPS: &str = "Schleifen";
pub const LAYER_SUBMAPS: &str = "Teilkarten";
pub const VIEW_HELP_ORBIT: &str =
    "Linke Taste: umkreisen · rechte/mittlere Taste oder Shift: verschieben · Rad: Zoom · F oder Doppelklick ins Leere: alles zeigen · Klick: Punkt";
pub const VIEW_HELP_FLY: &str =
    "W/A/S/D, Q/E: fliegen (Shift schneller) · rechte Taste: umsehen · Rad: vor/zurück · F: alles zeigen · Klick: Punkt";
pub const LOAD_CLOUD: &str = "Wolke laden";
pub const LOAD_SURFACE: &str = "Oberfläche laden";
pub const OPEN_SURFACE: &str = "Oberfläche öffnen …";
pub const DIALOG_OPEN_SURFACE: &str = "Oberfläche öffnen (oberflaeche.glb)";
pub const SHOW_MODE: &str = "Anzeige";
pub const SHOW_SURFACE: &str = "Oberfläche";
pub const SHOW_CLOUD: &str = "Punktwolke";
pub const SHOW_BOTH: &str = "beides";
pub const SHOW_MODE_KEY_HINT: &str = "Taste O: Oberfläche → Punktwolke → beides";
/// A glb that cannot be read from disk because it is not there.
pub const SURFACE_FILE_MISSING: &str = "Datei nicht gefunden";
pub const SYNTHETIC_CLOUD: &str = "Testwolke";
pub const PHASE_SYNTHETIC: &str = "Testwolke erzeugen";
pub const POINT: &str = "Punkt";
pub const POINT_X: &str = "x";
pub const POINT_Y: &str = "y";
pub const POINT_Z: &str = "z";
pub const POINT_TIME: &str = "Zeit ab Scan-Beginn";
pub const POINT_INTENSITY: &str = "Intensität";
pub const POINT_COLOR: &str = "Farbe";
pub const POINT_QUALITY: &str = "Güte der Teilkarte";
pub const POINT_REMOVED: &str = "Entfernt als";
pub const NO_COLOR: &str = "ohne Farbe";
pub const NO_POINT: &str = "Kein Punkt angeklickt";
/// Point info and quality mode of a cloud without SLAM report.
pub const QUALITY_UNKNOWN: &str = "unbekannt";
pub const CUT: &str = "Schnitt";
pub const CUT_Z_MIN: &str = "z min";
pub const CUT_Z_MAX: &str = "z max";
pub const HIDE_CEILING: &str = "Decke ausblenden";
pub const NO_CEILING: &str = "Keine Decke erkannt";
pub const CUT_RESET: &str = "Zurücksetzen";
pub const CUT_HINT: &str =
    "Höhen relativ zum Wolkenursprung; Punkte außerhalb werden weder gezeichnet noch angeklickt";
/// Hover text of the view: what „alles zeigen“ frames.
pub const FRAME_ALL_HINT: &str =
    "Alles zeigen rahmt die Wolke ohne die äußersten 0,5 % je Achse ein (ferne Ausreißer)";
pub const QUALITY_NONE_HINT: &str =
    "Ohne SLAM-Bericht oder ohne Punktzeiten: Güte unbekannt, grau gezeigt";

// Manual corrections (right panel)
pub const CORRECTIONS: &str = "Korrekturen";
pub const CORRECTIONS_HINT: &str = "Zwei Stellen der Laufbahn wählen, an denen der Scan dieselbe \
     Umgebung zeigt; die Schleife zieht sie beim Neurechnen zusammen";
pub const PICK_PLACES: &str = "Stellen wählen";
pub const PICK_PLACES_HINT: &str =
    "Ein Klick auf die Laufbahn in der 3D-Ansicht setzt Stelle A, der nächste Stelle B";
pub const PLACE_A: &str = "Stelle A";
pub const PLACE_B: &str = "Stelle B";
pub const PLACE_HINT: &str = "Zeit ab Scan-Beginn in s";
/// The labels of the places in the view.
pub const PLACE_A_SHORT: &str = "A";
pub const PLACE_B_SHORT: &str = "B";
pub const PICK_NEEDS_TRAJECTORY: &str =
    "Stellen wählen braucht die Ebene „Laufbahn“ – unter Ansicht einschalten";
pub const LAYER_REJECTED: &str = "Verworfene Kandidaten";
pub const NEXT_PLACE_A: &str = "Nächster Klick auf die Laufbahn: Stelle A";
pub const NEXT_PLACE_B: &str = "Nächster Klick auf die Laufbahn: Stelle B";
pub const NOTE: &str = "Notiz";
pub const ADD_LOOP: &str = "Schleife eintragen";
pub const ADD_LOOP_NEEDS_PLACES: &str = "Zuerst Stelle A und Stelle B wählen";
pub const CHANGED_RECOMPUTE: &str = "Geändert – jetzt neu rechnen";
pub const RECOMPUTE_NOW: &str = "Jetzt neu rechnen";
pub const NO_CORRECTIONS: &str = "Noch keine Korrekturen";
pub const CORRECTIONS_READING: &str = "Korrekturen werden gelesen …";
pub const CORRECTION_BUSY: &str = "Korrektur wird gespeichert …";
/// The columns of the list, as `driftless correct <lauf> list` prints them.
pub const CORRECTION_COLUMNS: [&str; 6] =
    ["Nr", "Status", "a (s)", "b (s)", "hinzugefügt", "Notiz"];
pub const LOOP_SWITCH_HINT: &str = "Ein- oder ausschalten; gilt beim nächsten Neurechnen";
/// Labels of the background jobs of the corrections (not shown in the status bar).
pub const LOAD_RUN_JOB: &str = "Lauf lesen";
pub const CORRECT_JOB: &str = "Korrektur speichern";
pub const VIEW_HELP_PICK: &str = "Stellen wählen: Klick auf die Laufbahn setzt die nächste Stelle";

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
pub const LAST_PROJECT_GONE: &str = "Zuletzt geöffnetes Projekt nicht lesbar";
pub const CLOSE_AFTER_CANCEL: &str =
    "Der laufende Schritt wird abgebrochen; danach schließt das Fenster";
const USAGE: &str = "Aufruf: driftless-app [--synthetic N] [projekt-ordner]";

/// The accessible name of the switch of entry `nr`.
pub fn loop_switch(nr: usize) -> String {
    format!("Eintrag {nr} aktiv")
}

/// The pose under the mouse while choosing places.
pub fn pose_hover(t: f64, submap: usize) -> String {
    format!("{} s · Teilkarte {submap}", decimal(t, 1))
}

/// A time of a place in the field: one decimal.
pub fn place_time(t: f64) -> String {
    decimal(t, 1)
}

/// Under the heading „Bilder“: „3 Karten · 2 Oberflächen“.
pub fn surfaces_line(maps: usize, surfaces: usize) -> String {
    let maps = match maps {
        1 => "1 Karte".to_string(),
        n => format!("{n} Karten"),
    };
    let surfaces = match surfaces {
        1 => "1 Oberfläche".to_string(),
        n => format!("{n} Oberflächen"),
    };
    format!("{maps} · {surfaces}")
}

/// A map in „Bilder“: „buero · SLAM 001 · Karte 001“.
pub fn map_of(scan: &str, run: &str, map: &str) -> String {
    format!("{scan} · {run} · {map}")
}

pub fn project_opened(name: &str) -> String {
    format!("Projekt {name} geöffnet")
}

/// Under the heading „Daten“: „Projekt „Halle“ · 3 Scans“.
pub fn project_line(name: &str, scans: usize) -> String {
    let scans = match scans {
        1 => "1 Scan".to_string(),
        n => format!("{n} Scans"),
    };
    format!("Projekt „{name}“ · {scans}")
}

/// A precision that is no level: „Eigene (0,013 m)“.
pub fn precision_custom(voxel: f64) -> String {
    format!("{PRECISION_CUSTOM} ({})", voxel_metres(voxel))
}

/// Tooltip of a precision level: „Raster 0,01 m“.
pub fn precision_hover(voxel: f64) -> String {
    format!("Raster {}", voxel_metres(voxel))
}

/// „0,005 m“, „0,01 m“: a voxel size without trailing zeros.
fn voxel_metres(voxel: f64) -> String {
    let fixed = decimal(voxel, 3);
    let trimmed = fixed.trim_end_matches('0').trim_end_matches(',');
    format!("{trimmed} m")
}

/// Under the precision while a job runs: „Scan 13:30 · Karte rechnen 62 %“, without a total
/// „Scan 13:30 · Karte rechnen läuft“; without a scan only the job.
pub fn job_of_scan(scan: Option<&str>, job: &str, fraction: Option<f32>) -> String {
    let how = match fraction {
        Some(f) => format!("{} %", (f * 100.0).round() as u32),
        None => "läuft".into(),
    };
    match scan {
        Some(scan) => format!("{scan} · {job} {how}"),
        None => format!("{job} {how}"),
    }
}

/// Under the main button when the map settings are invalid.
pub fn cannot_compute(why: &str) -> String {
    format!("Nicht rechenbar: {why}")
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

/// After the message of a manual correction that could not be applied.
pub const BAD_CORRECTION_HINT: &str = "Den Eintrag unter „Korrekturen“ ausschalten und neu \
     rechnen (die Teilkarten sind gespeichert)";

/// The message of a failed step; a bad correction with the hint how to switch it off here.
pub fn step_error(e: &driftless_steps::StepError) -> String {
    match e {
        driftless_steps::StepError::BadCorrection(m) => format!("{m}\n{BAD_CORRECTION_HINT}"),
        e => e.to_string(),
    }
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

pub fn unknown_map(dir: &std::path::Path) -> String {
    format!("{UNKNOWN_MAP}: {}", dir.display())
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

/// Why `removed.las` holds a point (its `user_data`): 1 ghost, 2 outlier, 3 seen through a
/// window (as `driftless_map::clean::RemovalReason`).
pub fn removal_reason(code: u8) -> Option<&'static str> {
    match code {
        1 => Some("Geist"),
        2 => Some("Ausreißer"),
        3 => Some("Durchblick"),
        _ => None,
    }
}

/// `512345.678` → „512.345,678 m“: millimetres, thousands separated by dots.
pub fn metres(x: f64) -> String {
    let fixed = format!("{:.3}", x.abs());
    let (whole, frac) = fixed.split_once('.').unwrap_or((&fixed, "000"));
    let whole = super::status::count(whole.parse().unwrap_or(0));
    // −0,000 is shown as 0,000
    let sign = if x < 0.0 && fixed != "0.000" { "-" } else { "" };
    format!("{sign}{whole},{frac} m")
}

/// „5,9 Mio.“ from a million on, else „300.000“.
pub fn point_count(n: usize) -> String {
    if n >= 1_000_000 {
        format!("{} Mio.", decimal(n as f64 / 1e6, 1))
    } else {
        super::status::count(n as u64)
    }
}

/// „gezeichnet 5,9 Mio. von 15,6 Mio. Punkten · 8,4 ms GPU“; without GPU timer the time
/// between frames: „… · 16,7 ms je Bild“.
pub fn drawn_stats(drawn: usize, total: usize, frame_ms: f32, gpu: bool) -> String {
    format!(
        "gezeichnet {} von {} Punkten · {} ms {}",
        point_count(drawn),
        point_count(total),
        decimal(f64::from(frame_ms), 1),
        if gpu { "GPU" } else { "je Bild" }
    )
}

/// „Wolke wird hochgeladen … 40 %“.
pub fn uploading(percent: f32) -> String {
    format!("Wolke wird hochgeladen … {percent:.0} %")
}

/// A height of the cut: „2,85 m“.
pub fn height(z: f32) -> String {
    format!("{} m", decimal(f64::from(z), 2))
}

/// The name of the synthetic cloud of `n` points.
pub fn synthetic_name(n: usize) -> String {
    format!("{SYNTHETIC_CLOUD} ({} Punkte)", point_count(n))
}

/// „--synthetic braucht eine Punktzahl“ and similar.
pub fn bad_synthetic(value: Option<&str>) -> String {
    match value {
        None => format!("--synthetic braucht eine Punktzahl; {USAGE}"),
        Some(v) => format!("--synthetic {v}: keine gültige Punktzahl; {USAGE}"),
    }
}

/// „Oberfläche 001“: a surface folder `oberflaeche-001` in the tree.
pub fn surface_label(digits: &str) -> String {
    format!("Oberfläche {digits}")
}

/// A stale surface in the tree: „Oberfläche 001 · veraltet – Karte oder Lauf neu gerechnet“.
pub fn surface_stale(label: &str) -> String {
    format!("{label} · {SURFACE_STALE}")
}

/// The step found a surface that fits: nothing to compute.
pub fn surface_fits(dir: &std::path::Path) -> String {
    let name = dir.file_name().map_or_else(
        || dir.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    format!("{name} passt schon zu Karte und Einstellungen; nichts zu rechnen")
}

/// A file that is no glb (or is cut off).
pub fn bad_glb(error: &str) -> String {
    format!("keine gültige glb-Datei ({error})")
}

/// A glTF whose data lie in a file of their own.
pub fn external_buffer(uri: &str) -> String {
    format!(
        "Daten liegen in einer eigenen Datei ({uri}); nur eine glb mit allen Daten in der Datei \
         wird gelesen (wie `driftless surface` sie schreibt)"
    )
}

/// „Oberfläche: 712.218 Dreiecke · 8,4 ms GPU“ – the stats line of a surface shown alone.
pub fn surface_stats(triangles: usize, frame_ms: f32, gpu: bool) -> String {
    format!(
        "Oberfläche: {} Dreiecke · {} ms {}",
        super::status::count(triangles as u64),
        decimal(f64::from(frame_ms), 1),
        if gpu { "GPU" } else { "je Bild" }
    )
}

/// „Oberfläche geladen: oberflaeche-001/oberflaeche.glb (712.218 Dreiecke, 12 Seite(n),
/// 3,4 s)“.
pub fn surface_loaded(path: &std::path::Path, triangles: usize, pages: usize, s: f64) -> String {
    let name: Vec<String> = path
        .iter()
        .rev()
        .take(2)
        .map(|c| c.to_string_lossy().into_owned())
        .collect();
    let name: Vec<&str> = name.iter().rev().map(String::as_str).collect();
    format!(
        "Oberfläche geladen: {} ({} Dreiecke, {pages} Seite(n), {} s)",
        name.join("/"),
        point_count(triangles),
        decimal(s, 1)
    )
}

/// „color.las geladen: 15,6 Mio. Punkte (Laden 4,1 s, Detailstufen 2,3 s)“.
pub fn cloud_loaded(path: &std::path::Path, points: usize, load_s: f64, build_s: f64) -> String {
    let name = path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    format!(
        "{name} geladen: {} Punkte (Laden {} s, Detailstufen {} s)",
        point_count(points),
        decimal(load_s, 1),
        decimal(build_s, 1)
    )
}

// Comparison and history
pub const COMPARE: &str = "Vergleichen";
pub const COMPARE_HINT: &str =
    "Zwei Laufbahnen des Scans: A wird an B ausgerichtet und orange in der Ansicht gezeigt";
pub const COMPARE_A: &str = "Laufbahn A";
pub const COMPARE_B: &str = "Laufbahn B (Bezug)";
pub const CHOOSE: &str = "wählen …";
pub const COMPARING: &str = "Wird verglichen …";
pub const COMPARE_JOB: &str = "Laufbahnen vergleichen";
pub const COMPARE_ATE: &str = "Abweichung (ATE, RMSE)";
pub const COMPARE_GAP: &str = "Start↔Ende A/B";
pub const COMPARE_LENGTH: &str = "Länge A/B";
pub const COMPARE_MATCHED: &str = "Zugeordnete Posen";
pub const LAYER_COMPARE: &str = "Vergleich (A)";
pub const COMPARE_WITH_CURRENT: &str = "Vergleichen mit aktuell";
pub const RESTORE_STATE: &str = "Diesen Stand wiederherstellen";
pub const RESTORE_HINT: &str = "Schaltet die Korrekturen wie in diesem Stand und rechnet den Lauf \
                                neu; das ergibt einen neuen Stand, die alten bleiben";
pub const STEP_RESTORE: &str = "Stand wiederherstellen";
pub const HISTORY: &str = "Verlauf";
pub const MAPS_COMPARE: &str = "Karten vergleichen";
pub const MAP_A: &str = "Karte A";
pub const MAP_B: &str = "Karte B";
pub const MAPS_COMPARING: &str = "Karten werden gelesen …";
pub const MAPS_COMPARE_JOB: &str = "Karten vergleichen";
pub const MAPS_NEED_TWO: &str = "Der Scan hat noch keine zwei Karten";
pub const MAP_COLUMNS: [&str; 3] = ["", "A", "B"];
pub const MAP_ON: &str = "an";
pub const MAP_OFF: &str = "aus";
pub const MAP_POINTS_DENSE: &str = "Punkte dichte Wolke";
pub const MAP_POINTS_KEPT: &str = "Punkte behalten";
pub const MAP_REMOVED_GHOSTS: &str = "Entfernt: Geister";
pub const MAP_REMOVED_OUTLIERS: &str = "Entfernt: Ausreißer";
pub const MAP_REMOVED_THROUGH: &str = "Entfernt: Durchblick";
pub const MAP_THICKNESS_BEFORE: &str = "Wanddicke vorher";
pub const MAP_THICKNESS_AFTER: &str = "Wanddicke nachher";
pub const MAP_POINTS_COLORED: &str = "Punkte eingefärbt";
pub const MAP_CAMERA_BEFORE: &str = "Kamera-Unterschied vorher";
pub const MAP_CAMERA_AFTER: &str = "Kamera-Unterschied nachher";
pub const MAP_SEAM_BEFORE: &str = "Nahtsprung vorher";
pub const MAP_SEAM_AFTER: &str = "Nahtsprung nachher";
pub const MAP_SECONDS_DENSE: &str = "Laufzeit Dichte Wolke";
pub const MAP_SECONDS_CLEAN: &str = "Laufzeit Bereinigen";
pub const MAP_SECONDS_COLOR: &str = "Laufzeit Einfärben";
pub const MAP_SECONDS_TOTAL: &str = "Laufzeit gesamt";
/// A value that is missing (a step that did not run, an older `map.json`).
pub const MISSING: &str = "–";

/// „0,123 m“ – a value of the comparison, three decimals as `driftless compare`.
pub fn compare_metres(x: f64) -> String {
    format!("{} m", decimal(x, 3))
}

/// „Stand 003“.
pub fn state_name(nr: u32) -> String {
    format!("Stand {nr:03}")
}

/// The orange line is drawn only in the view of the run of B.
pub fn compare_not_drawn(run: &str) -> String {
    format!("Die orange Laufbahn A erscheint in der Ansicht von {run}")
}

/// „21.10.2026 16:03“: an RFC 3339 time in the local time of this computer; other text
/// unchanged.
pub fn stamp(created: &str) -> String {
    chrono::DateTime::parse_from_rfc3339(created.trim()).map_or_else(
        |_| created.to_string(),
        |t| {
            t.with_timezone(&chrono::Local)
                .format("%d.%m.%Y %H:%M")
                .to_string()
        },
    )
}

/// A state of the history in the tree: „Stand 003 · neu gerechnet · 21.10.2026 16:03 ·
/// Start↔Ende 0,123 m“ (without the gap if `slam.json` could not be read).
pub fn history_entry(e: &driftless_steps::history::HistoryEntry, gap: Option<f64>) -> String {
    let mut s = format!(
        "{} · {} · {}",
        state_name(e.nr),
        e.kind.label(),
        stamp(&e.created)
    );
    if let Some(g) = gap {
        s.push_str(&format!(" · Start↔Ende {}", compare_metres(g)));
    }
    s
}
pub const HISTORY_GAPS_JOB: &str = "Verlauf lesen";

/// The legend of a comparison in the view: „A: SLAM 001 Odometrie“.
pub fn legend(which: &str, name: &str) -> String {
    format!("{which}: {name}")
}
