//! Every text the program shows, in one place (German): fixed texts as constants, texts
//! with values as small functions. Messages of the steps and of the system pass through
//! [`german`] (system errors in German, the window's actions instead of command lines), and
//! what the window shows of them through [`shown`] (symbols its fonts lack) and
//! [`short_paths`] (file names instead of whole paths, the reason stays in view).

use std::borrow::Cow;
use std::path::Path;

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
/// Over the window while a file is dragged over it.
pub const DROP_HERE: &str = "Loslassen zum Öffnen";
/// A file dropped onto the window (or picked) that Driftless cannot open.
pub const DROP_UNSUPPORTED: &str = "Dateiformat nicht unterstützt";

/// [`DROP_UNSUPPORTED`] with the name of the (first) file dropped.
pub fn drop_unsupported(name: &str) -> String {
    format!("{DROP_UNSUPPORTED}: {name} (LAS, LAZ, E57, PLY, PCD, GLB oder OBJ)")
}

pub const PLAN_COMING: &str =
    "Hier entstehen später Grundriss, Maße und der DWG-Export; heute geht das Messen in der Ansicht.";
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
/// The tooltip of „Scan hinzufügen …“ as the main button of an empty project.
pub const ADD_SCAN_HINT: &str = "Ordner mit LIDAR_*.bag wählen";
pub const ALL_COMPUTED: &str = "Alle Scans sind gerechnet";
/// With every scan done, but their maps of another precision than the one chosen.
pub const RECOMPUTE_HINT: &str = "Scan wählen und „Rechnen“ drücken";
/// While the background check of the maps runs: „Alle rechnen“ starts once it is done.
pub const MAP_STATUS_PENDING: &str = "Karten werden geprüft …";
pub const ADVANCED: &str = "Erweitert";

// Merging scans into a model („Daten“)
/// Opens the field that merges scans; shown once two scans have a finished map.
pub const MERGE_OPEN: &str = "Zusammenführen …";
pub const MERGE_OPEN_HINT: &str = "Mehrere Scans desselben Orts zu einem Modell vereinen";
pub const MERGE_TITLE: &str = "Scans zusammenführen";
pub const MERGE_HINT: &str =
    "Der erste angehakte Scan ist der Bezug: die anderen werden an ihm ausgerichtet.";
/// Beside the first ticked scan, and in the model for the scan the others were placed at.
pub const MERGE_REFERENCE: &str = "Bezug";
pub const MERGE_NO_MAP: &str = "Noch keine fertige Karte – zuerst rechnen";
pub const MERGE_MAP: &str = "Karte";
pub const MERGE_UNSURE: &str = "Auch unsichere übernehmen";
pub const MERGE_UNSURE_HINT: &str = "Nimmt auch Scans ins Modell, deren Lage nicht sicher \
     gefunden wurde. Gescheiterte nie.";
/// The start button of the field, and the label of the job.
pub const MERGE_START: &str = "Zusammenführen";
pub const MERGE_START_HINT: &str = "Richtet die Scans aneinander aus und legt ein neues Modell an";
pub const MERGE_CLOSE: &str = "Schließen";
pub const MERGE_NEEDS_TWO: &str = "Mindestens zwei Scans anhaken";
/// Above the models in „Daten“.
pub const MODELS: &str = "Modelle";
/// How surely a scan was placed in a model (the words of `modell.json`).
pub const VERDICT_SURE: &str = "sicher";
pub const VERDICT_UNSURE: &str = "unsicher";
pub const VERDICT_FAILED: &str = "gescheitert";
/// A scan of a model whose points are not in it.
pub const NOT_IN_MODEL: &str = "nicht im Modell";
pub const NOT_IN_MODEL_UNSURE: &str = "„Auch unsichere übernehmen“ nimmt ihn auf";
pub const VERDICT_HINT: &str = "Überlappung: welcher Teil des Scans im Modell wiederkehrt. \
     Restversatz: wie weit seine Punkte im Mittel neben denen der anderen liegen. Wände: \
     derselbe Versatz an der schlechtesten Wandrichtung.";
/// A model whose maps were computed anew since the merge.
pub const MODEL_STALE: &str = "veraltet – Karte eines Scans neu gerechnet";
pub const STATUS_MODEL_STALE: &str = "Modell veraltet";
pub const STATUS_MODEL_STALE_HINT: &str =
    "Eine Karte des Modells wurde neu gerechnet – in „Daten“ neu zusammenführen";
pub const COMPUTE_MODEL_SURFACE_HINT: &str =
    "Rechnet die Oberfläche des Modells aus den Bildern aller seiner Scans";
pub const UNKNOWN_MODEL: &str = "Modell nicht im Projekt";

/// „Modell 001“: a model folder `modell-001`.
pub fn model_label(digits: &str) -> String {
    format!("Modell {digits}")
}

/// A model in „Bilder“: „Modell 001 · 3 Scans“.
pub fn model_card(label: &str, scans: usize) -> String {
    match scans {
        1 => format!("{label} · 1 Scan"),
        n => format!("{label} · {n} Scans"),
    }
}

/// A stale model in the tree: „Modell 001 · veraltet – Karte eines Scans neu gerechnet“.
pub fn model_stale(label: &str) -> String {
    format!("{label} · {MODEL_STALE}")
}

/// A scan of a model: „2 · Scan 13:30“ (its number, as the point info names it).
pub fn model_member(nr: usize, name: &str) -> String {
    format!("{nr} · {name}")
}

/// Under a scan of a model: „Überlappung 98 % · Restversatz 1,8 mm“.
pub fn member_numbers(overlap: f64, residual_mm: f64) -> String {
    format!(
        "Überlappung {} % · Restversatz {} mm",
        (overlap * 100.0).round() as i64,
        decimal(residual_mm, 1)
    )
}

/// Under a scan of a model placed by point pairs: „Punktpaare · Überlappung 10 % · Restversatz
/// 0,5 mm“.
pub fn member_numbers_pairs(overlap: f64, residual_mm: f64) -> String {
    format!("{PAIRS_LIST} · {}", member_numbers(overlap, residual_mm))
}

/// On hover over a scan placed by point pairs: „Von Hand über Punktpaare ausgerichtet, sie
/// liegen im Mittel 1,2 cm neben der Lage“.
pub fn member_pairs_fit(rms_mm: f64) -> String {
    format!(
        "Von Hand über Punktpaare ausgerichtet, sie liegen im Mittel {} cm neben der Lage",
        decimal(rms_mm / 10.0, 1)
    )
}

/// On hover over a scan of a model: „Wände 1,9 mm“ (the worst wall orientation).
pub fn member_walls(wall_mm: f64) -> String {
    format!("Wände {} mm", decimal(wall_mm, 1))
}

pub fn unknown_model(dir: &std::path::Path) -> String {
    format!("{UNKNOWN_MODEL}: {}", dir.display())
}

pub fn scan_not_in_project(scan: &std::path::Path) -> String {
    format!("Scan {} ist nicht mehr im Projekt", scan.display())
}

// Point pairs by hand („Von Hand ausrichten“)
/// Under an unsure or failed scan of a model.
pub const PAIRS_START: &str = "Von Hand ausrichten";
pub const PAIRS_START_HINT: &str = "Drei gleiche Stellen im Modell und in diesem Scan \
     anklicken; danach wird neu zusammengeführt";
pub const PAIRS_TITLE: &str = "Von Hand ausrichten";
pub const PAIRS_HINT: &str = "Je Paar zuerst eine Stelle im Modell anklicken, dann dieselbe \
     Stelle im Scan rechts daneben. Mindestens drei Paare, weit auseinander und nicht auf einer \
     Linie.";
pub const PAIRS_LOADING: &str = "Modell und Scan werden geladen …";
pub const PAIRS_WANTS_MODEL: &str =
    "Das ist ein Punkt im Scan – zuerst die Stelle im Modell anklicken";
pub const PAIRS_WANTS_SCAN: &str =
    "Das ist ein Punkt im Modell – jetzt dieselbe Stelle im Scan anklicken";
pub const PAIRS_NOT_SHOWN: &str = "Die Ansicht zeigt nicht Modell und Scan – neu beginnen";
pub const PAIRS_UNDO: &str = "Letzten Punkt zurück";
pub const PAIRS_CANCEL: &str = "Abbrechen";
pub const PAIRS_APPLY: &str = "Übernehmen und neu zusammenführen";
pub const PAIRS_APPLY_HINT: &str =
    "Speichert die Paare in modelle/punktpaare.toml und legt mit denselben Scans ein neues \
     Modell an";
pub const PAIRS_NEED_THREE: &str = "Mindestens drei Paare setzen";
pub const PAIRS_HALF: &str = "Zuerst dieselbe Stelle im Scan anklicken";
pub const PAIRS_NO_CLOUD: &str = "Der Scan hat keine Punktwolke (color.las oder clean.las)";
pub const PAIRS_SAVED: &str = "Punktpaare gespeichert";
/// The tool of the view while pairs are set.
pub const TOOL_PAIRS: &str = "Punktpaare";
pub const TOOL_PAIRS_HINT: &str =
    "Klick setzt abwechselnd einen Punkt im Modell und denselben im Scan";
/// Above the list of the point pairs of a model.
pub const PAIRS_LIST: &str = "Punktpaare";
pub const PAIR_SWITCH_HINT: &str =
    "Abgeschaltete Paare bleiben gespeichert und zählen ab dem nächsten Zusammenführen nicht";
/// The label of the job that switches a pair.
pub const PAIRS_JOB: &str = "Punktpaar speichern";

/// „Scan 3 · Scan 13:35 an Modell 001“.
pub fn pairs_subject(member: &str, model: &str) -> String {
    format!("{member} an {model}")
}

/// What the next click of the tool „Punktpaare“ sets: „Paar 2: Stelle im Modell anklicken“.
pub fn pair_next(nr: usize, in_model: bool) -> String {
    if in_model {
        format!("Paar {nr}: Stelle im Modell anklicken")
    } else {
        format!("Paar {nr}: dieselbe Stelle im Scan anklicken")
    }
}

/// „3 Paare gesetzt“.
pub fn pairs_set(n: usize) -> String {
    match n {
        1 => "1 Paar gesetzt".into(),
        n => format!("{n} Paare gesetzt"),
    }
}

/// The cloud of model and scan side by side, named in the view and the messages.
pub fn pair_cloud_name(member: &str) -> String {
    format!("Modell und {member}")
}

/// A point pair of a model: „Paar 1 · Scan 1 und 3 · 4 Punkte“ (the numbers of its scans).
pub fn pair_row(nr: usize, a: usize, b: usize, points: usize) -> String {
    format!("Paar {nr} · Scan {a} und {b} · {}", self::points(points))
}

/// Beside a point pair the merge cannot use (a hand edit of the file); the reason on hover.
pub const PAIR_UNUSABLE: &str = "nicht nutzbar";

/// The name of the switch of point pair `nr` (from 1), for screen readers and tests.
pub fn pair_switch(nr: usize) -> String {
    format!("Punktpaar {nr} an")
}

/// „Punktpaar 2 abgeschaltet – zählt ab dem nächsten Zusammenführen“.
pub fn pair_switched(nr: usize, enabled: bool) -> String {
    let what = if enabled {
        "eingeschaltet"
    } else {
        "abgeschaltet"
    };
    format!("Punktpaar {nr} {what} – zählt ab dem nächsten Zusammenführen")
}

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
    "braucht bei großen Aufnahmen bis 8 GB Arbeitsspeicher und rund 20 Minuten";
/// The status of a map whose own map no longer fits its run: its surface waits for „Daten“.
pub const STATUS_MAP_STALE: &str = "Karte veraltet";
pub const STATUS_MAP_STALE_HINT: &str =
    "Die Karte passt nicht mehr zu ihrem Lauf – zuerst in „Daten“ neu rechnen";
/// The status of a map without `map.json` yet (still being computed, or failed).
pub const STATUS_MAP_NOT_READY: &str = "Karte noch nicht fertig";
/// The status of a map whose scan folder cannot be read.
pub const STATUS_SCAN_MISSING: &str = "Scan fehlt";
/// A map of the queue of „Alle Oberflächen rechnen“ is no longer in the project.
pub const MAP_GONE: &str = "Karte nicht mehr im Projekt";
/// Before the state of a surface of the selected map with the fineness chosen now.
pub const WITH_THIS_FINENESS: &str = "Mit dieser Feinheit";

// Panels
pub const PANEL_STEPS: &str = "Schritte";
pub const PANEL_PROPERTIES: &str = "Eigenschaften";
pub const VIEW_EMPTY: &str = "Keine Wolke geladen – links eine Karte oder Wolke wählen";
/// The empty view without a project.
pub const VIEW_NO_PROJECT: &str = "Projekt öffnen oder Datei hineinziehen";

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
    "Kleiner ist feiner, braucht mehr Speicher und Zeit; 1 cm braucht bei großen Aufnahmen bis \
     8 GB Arbeitsspeicher und rund 20 Minuten";
pub const SURFACE_NO_PHOTOS: &str = "ohne Fotos (graue Fläche)";

// The 3D view and its settings (window ⚙)
pub const VIEW_NO_GPU: &str = "3D-Ansicht nicht verfügbar: keine wgpu-Grafik";
pub const COLOR_MODE: &str = "Farbart";
pub const MODE_RGB: &str = "Farbe";
pub const MODE_INTENSITY: &str = "Intensität";
pub const MODE_QUALITY: &str = "Güte";
pub const MODE_SCAN: &str = "je Scan";
pub const POINT_SIZE: &str = "Punktgröße (px)";
pub const ROUND_POINTS: &str = "rund";
pub const POINT_SIZE_MODE: &str = "Punkte";
pub const POINT_SIZE_PIXEL: &str = "Pixel";
pub const POINT_SIZE_WORLD: &str = "Meter";
pub const POINT_SIZE_MODE_HINT: &str = "Meter: jeder Punkt ist ein runder Fleck in echter Größe, aus \
     der Nähe schließen sich die Flecken zu einem Bild. Pixel: jeder Punkt gleich groß, egal wie weit weg.";
/// The point size slider in world size: the smallest a spot gets.
pub const POINT_SIZE_MIN: &str = "Mindestgröße (px)";
pub const SPOT_SIZE: &str = "Fleckgröße (cm)";
pub const SPOT_FROM_MAP: &str = "aus der Karte";
pub const SPOT_FROM_MAP_HINT: &str = "Raster der Karte × 1,5 (ohne Angabe 1,5 cm)";
/// Range of a spot size set by hand (m).
pub const POINT_M_RANGE: [f32; 2] = [0.002, 0.2];
pub const BUDGET: &str = "Punktbudget";
/// Range of the point budget.
pub const BUDGET_RANGE: std::ops::RangeInclusive<usize> = 1_000_000..=30_000_000;
pub const CAMERA_MODE: &str = "Bedienung";
pub const ORBIT: &str = "Umkreisen";
pub const FLY: &str = "Fliegen";
pub const LAYERS: &str = "Ebenen";
pub const GPU: &str = "Grafikkarte nutzen";
pub const GPU_HINT: &str =
    "Für Karte und Oberfläche. Auto: die Grafikkarte, wenn eine da ist, sonst \
     der Prozessor. An: nur mit Grafikkarte, ohne eine bricht der Schritt ab. Aus: immer der \
     Prozessor. Das Ergebnis ist gleichwertig, nur die Rechenzeit unterscheidet sich.";
pub const GPU_AUTO: &str = "Auto";
pub const GPU_ON: &str = "An";
pub const GPU_OFF: &str = "Aus";
pub const LAYER_TRAJECTORY: &str = "Laufbahn";
pub const LAYER_ODOMETRY: &str = "Odometrie";
pub const LAYER_LOOPS: &str = "Schleifen";
pub const LAYER_SUBMAPS: &str = "Teilkarten";
pub const VIEW_HELP_ORBIT: &str =
    "Linke Taste: wie das Werkzeug unten · rechte/mittlere Taste oder Shift: verschieben · Rad: Zoom · F oder Doppelklick ins Leere: alles zeigen · O: Fläche/Punkte/beides · Esc: Messen abbrechen";
pub const VIEW_HELP_FLY: &str =
    "W/A/S/D, Q/E: fliegen (Shift schneller) · linke Taste: wie das Werkzeug unten · rechte Taste: umsehen · Rad: vor/zurück · F: alles zeigen · O: Fläche/Punkte/beides · Esc: Messen abbrechen";
pub const LOAD_CLOUD: &str = "Wolke laden";
pub const LOAD_SURFACE: &str = "Oberfläche laden";
/// ☰: open a point cloud or a mesh (surface) file.
pub const OPEN_FILE: &str = "Datei öffnen …";
pub const DIALOG_OPEN_FILE: &str = "Datei öffnen (LAS, LAZ, E57, PLY, PCD, GLB, OBJ)";
/// Filters of the dialog „Datei öffnen …“: everything, point clouds, meshes.
pub const FILTER_ALL: &str = "Punktwolken und Netze";
pub const FILTER_CLOUD: &str = "Punktwolke";
pub const FILTER_MESH: &str = "Netz";
// Export (☰, the context menu of the tree, the window „Exportieren“)
pub const EXPORT: &str = "Exportieren …";
/// The window, and the label of the export in the status bar.
pub const EXPORT_TITLE: &str = "Exportieren";
pub const EXPORT_WHAT: &str = "Was";
pub const EXPORT_FORMAT: &str = "Format";
pub const EXPORT_PICK: &str = "Ziel wählen …";
pub const EXPORT_REPLACE: &str = "Ersetzen";
/// Why „Exportieren …“ in ☰ is locked.
pub const EXPORT_NEEDS_VIEW: &str = "Zuerst eine Punktwolke oder Oberfläche anzeigen";

// The window „Über Driftless“ (☰)
pub const ABOUT: &str = "Über Driftless …";
pub const ABOUT_TITLE: &str = "Über Driftless";
pub const ABOUT_LICENSE: &str = "Lizenz: Apache-2.0 OR MIT";
pub const ABOUT_CLI_MISSING: &str = "Kommandozeile: nicht im Paket (cargo run -p driftless-cli)";
/// Opens the licence notices of the third-party parts (THIRD-PARTY-NOTICES.html).
pub const ABOUT_NOTICES: &str = "Lizenzen Dritter …";
pub const ABOUT_NOTICES_MISSING: &str = "Lizenzen Dritter: nicht im Paket (packaging/about.toml)";
/// ☰: the folder of the log file (`crate::logfile`) in the file manager.
pub const OPEN_LOG_FOLDER: &str = "Protokoll-Ordner öffnen";
/// „Protokoll-Ordner öffnen“ when the system names no folder for it.
pub const LOG_FOLDER_UNKNOWN: &str = "Kein Protokoll-Ordner: HOME bzw. LOCALAPPDATA fehlt";

/// A file, folder or address the system could not open: „/Users/…/Logs/Driftless nicht geöffnet:
/// …“.
pub fn open_failed(what: &str, error: &str) -> String {
    format!("{what} nicht geöffnet: {error}")
}

/// On stderr, when the log file of a start cannot be begun.
pub fn log_unavailable(dir: &str, error: &str) -> String {
    format!("Protokolldatei in {dir} nicht anlegbar: {error}")
}

/// „Driftless 0.1.0“.
pub fn about_version(version: &str) -> String {
    format!("{APP_TITLE} {version}")
}

/// „Commit abc123def“.
pub fn about_commit(commit: &str) -> String {
    format!("Commit {commit}")
}

/// „Kommandozeile: /Applications/Driftless.app/Contents/MacOS/driftless“.
pub fn about_cli(path: &str) -> String {
    format!("Kommandozeile: {path}")
}

/// A source in the window: „Karte 001 – color.las“ (its name and the file that is read).
pub fn export_choice(name: &str, file: &str) -> String {
    format!("{name} – {file}")
}

/// „Diese Dateien gibt es schon:\nx.mtl\nErsetzen?“ – the files by name (they are next to the
/// target just chosen).
pub fn export_replace_question(files: &[std::path::PathBuf]) -> String {
    let names: Vec<String> = files.iter().map(|f| file_name(f)).collect();
    format!(
        "Diese Dateien gibt es schon:\n{}\n{EXPORT_REPLACE}?",
        names.join("\n")
    )
}

/// „Exportiert: modell.laz (1.234.567 Punkte, 12,3 MB)“ – the main file by name.
pub fn exported(
    report: &driftless_steps::export::ExportReport,
    kind: driftless_steps::export::ExportKind,
) -> String {
    use driftless_steps::export::ExportKind;
    let what = match kind {
        ExportKind::Points => "Punkte",
        ExportKind::Mesh => "Dreiecke",
        ExportKind::Trajectory => "Lagen",
    };
    let file = report
        .files
        .first()
        .map(|f| file_name(f))
        .unwrap_or_default();
    format!(
        "Exportiert: {file} ({} {what}, {} MB)",
        super::status::count(report.count),
        decimal(report.bytes as f64 / 1e6, 1)
    )
}

/// The name of `path`, or all of it if it has none.
fn file_name(path: &std::path::Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// The switch at the top right of the view.
pub const SHOW_MODE: &str = "Ansicht";
pub const SHOW_SURFACE: &str = "Fläche";
pub const SHOW_CLOUD: &str = "Punkte";
pub const SHOW_BOTH: &str = "beides";
pub const SHOW_MODE_KEY_HINT: &str = "Taste O wechselt der Reihe nach: Fläche, Punkte, beides";
/// Why „Punkte“ and „beides“ are locked while only a surface is loaded.
pub const SHOW_NEEDS_CLOUD: &str = "Keine Punktwolke geladen – nur die Fläche";
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
/// The scan of a point of a merged model, counted from 1 as in the model's list.
pub const POINT_SCAN: &str = "Scan";
pub const NO_COLOR: &str = "ohne Farbe";
/// Tooltip of the × of the clicked point.
pub const CLOSE_POINT: &str = "Punkt schließen";
/// Point info and quality mode of a cloud without SLAM report.
pub const QUALITY_UNKNOWN: &str = "unbekannt";
pub const CUT_MIN: &str = "z min";
pub const CUT_MAX: &str = "z max";
pub const HIDE_CEILING: &str = "Decke ausblenden";
pub const NO_CEILING: &str = "Keine Decke erkannt";
pub const CUT_RESET: &str = "Zurücksetzen";
pub const CUT_HINT: &str =
    "Höhen relativ zum Wolkenursprung; Punkte außerhalb werden weder gezeichnet noch angeklickt";
/// Tooltip of the line „Schnitt aktiv: …“ in the info of the view.
pub const CUT_ACTIVE_HINT: &str =
    "Punkte außerhalb dieser Höhen sind ausgeblendet; Werkzeug „Schnitt“ ändert oder setzt zurück";
/// Tooltip of „Alles“: what it frames.
pub const FRAME_ALL_HINT: &str = "Alles zeigen (Taste F); ohne die äußersten 0,5 % je Achse \
     (ferne Ausreißer)";

// The tools at the bottom of the view; exactly one is active. „Alles“ is a button, no tool.
pub const TOOL_ORBIT: &str = "Drehen";
pub const TOOL_PAN: &str = "Verschieben";
pub const TOOL_PICK: &str = "Punkt wählen";
pub const TOOL_MEASURE: &str = "Messen";
pub const TOOL_CUT: &str = "Schnitt";
pub const FIT_ALL: &str = "Alles";
pub const TOOL_ORBIT_HINT: &str = "Linke Taste dreht die Ansicht um die Mitte";
pub const TOOL_PAN_HINT: &str = "Linke Taste verschiebt die Ansicht";
pub const TOOL_PICK_HINT: &str = "Klick auf einen Punkt zeigt seine Werte oben links";
pub const TOOL_MEASURE_HINT: &str =
    "Zwei Punkte nacheinander anklicken: ihr Abstand in Metern · Esc bricht ab";
pub const MEASURE_FIRST: &str = "Messen: ersten Punkt anklicken";
pub const MEASURE_SECOND: &str = "Messen: zweiten Punkt anklicken · Esc bricht ab";
pub const PICK_POINT: &str = "Punkt wählen: auf einen Punkt klicken";
/// Under the hint of „Messen“ / „Punkt wählen“ while only the surface is shown: the clicks
/// hit the points of the cloud, not the surface.
pub const PICKS_ON_CLOUD: &str = "misst an der Punktwolke, nicht an der Fläche";
/// Measuring and picking look at points only.
pub const TOOL_NEEDS_CLOUD: &str = "Messen und Punkt wählen brauchen eine Punktwolke";

// Measurements (field „Planung“)
pub const MEASUREMENTS: &str = "Messungen";
pub const NO_MEASUREMENTS: &str =
    "Noch keine Messungen. Unten in der Ansicht „Messen“ wählen und zwei Punkte anklicken.";
pub const MEASUREMENTS_NOT_SAVED: &str =
    "Messungen werden noch nicht gespeichert: sie gehen beim Schließen verloren.";
pub const CLEAR_MEASUREMENTS: &str = "Messungen löschen";
/// The legend of the quality colours in the view.
pub const QUALITY_LEGEND: &str = "Güte";
pub const QUALITY_GOOD: &str = "gut verortet";
pub const QUALITY_WEAK: &str = "wenig Halt – prüfen";
pub const QUALITY_BAD: &str = "kaum Halt – Lage unsicher";
pub const QUALITY_LEGEND_HINT: &str = "Wie fest die Umgebung jede Teilkarte der Laufbahn \
     gehalten hat (ihr schwächster Moment). Gelb und Rot: wenig Wände oder Kanten im Blick, etwa \
     in einem leeren Flur, in einer Tür oder bei einer schnellen Drehung – dort kann die Laufbahn \
     verrutscht sein; die Karte an dieser Stelle prüfen.";
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
    "Stellen wählen braucht die Ebene „Laufbahn“ – in den Einstellungen einschalten";
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
/// The automatic loops of the run in „Korrekturen“, each with its switch.
pub const AUTO_LOOPS: &str = "Automatische Schleifen";
pub const AUTO_LOOPS_HINT: &str = "Eine Schleife, die zwei verschiedene Stellen verbindet, \
     abschalten: der Lauf wird gleich neu gerechnet (nur der Posengraph, wenige Sekunden).";
pub const NO_AUTO_LOOPS: &str = "Keine automatische Schleife";
/// The columns of the list of the automatic loops.
pub const AUTO_LOOP_COLUMNS: [&str; 4] = ["verwenden", "Teilkarten", "Beginn (s)", "Überlappung"];
pub const AUTO_LOOP_SWITCH_HINT: &str = "Aus: die Schleife bleibt beim Neurechnen außen vor \
     (corrections.toml, [[reject]]); an: sie zählt wieder";

/// The accessible name of the switch of the automatic loop between submaps `a` and `b`.
pub fn auto_loop_switch(a: usize, b: usize) -> String {
    format!("Schleife Teilkarte {a} und {b} verwenden")
}

/// The submaps of an automatic loop: „3 und 17“.
pub fn auto_loop_pair(a: usize, b: usize) -> String {
    format!("{a} und {b}")
}

/// The starts of its two submaps: „12,3 / 250,1“.
pub fn auto_loop_times(a: f64, b: f64) -> String {
    format!("{} / {}", decimal(a, 1), decimal(b, 1))
}

/// Overlap of a loop in percent: „34 %“.
pub fn percent(share: f64) -> String {
    format!("{} %", (share * 100.0).round() as i64)
}

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

/// Below „Alle Oberflächen rechnen“: the maps it leaves out because they wait for „Daten“
/// (stale, not done, or of a scan that cannot be read).
pub fn maps_wait_for_data(n: usize) -> String {
    match n {
        1 => "1 Karte erst in „Daten“ neu rechnen".to_string(),
        n => format!("{n} Karten erst in „Daten“ neu rechnen"),
    }
}

/// A queued map that is no longer in the project.
pub fn map_gone(dir: &std::path::Path) -> String {
    format!("{MAP_GONE}: {}", dir.display())
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
    format!("Projekt „{name}“ geöffnet")
}

/// After a change of the project when `project.toml` had been changed by another window or
/// program since it was read: the change was made on what that wrote.
pub const PROJECT_CHANGED_ELSEWHERE: &str = "project.toml war inzwischen von einem anderen \
     Fenster oder Programm geändert; die Änderung ist auf dessen Stand gemacht, beides bleibt";
/// Label of the background job that writes `auswahl.toml`.
pub const CHOICES_JOB: &str = "Auswahl speichern";

// Managing the project: scans, and folders to the trash
pub const RENAME_SCAN: &str = "Umbenennen …";
pub const RELINK_SCAN: &str = "Ordner neu zuordnen …";
pub const RELINK_ONLY_MISSING: &str =
    "Nur für einen Scan, dessen Ordner fehlt (verschoben, Laufwerk nicht angesteckt)";
pub const REMOVE_SCAN: &str = "Aus Projekt entfernen";
pub const REMOVE_RUN: &str = "Aus Projekt entfernen";
pub const DELETE: &str = "Löschen …";
/// The window „Scan umbenennen“ and its field.
pub const RENAME_TITLE: &str = "Scan umbenennen";
pub const SCAN_NAME: &str = "Name";
pub const SCAN_NAME_EMPTY: &str = "Der Name darf nicht leer sein";
pub const RENAME_CONFIRM: &str = "Umbenennen";
pub const REMOVE_TITLE: &str = "Aus Projekt entfernen";
pub const REMOVE_CONFIRM: &str = "Entfernen";
pub const DELETE_TITLE: &str = "Löschen";
pub const DELETE_CONFIRM: &str = "In den Papierkorb";
/// In the question „Löschen“ while the view shows something of the folder.
pub const UNSHOW: &str = "Aus der Ansicht nehmen";
pub const ASK_CANCEL: &str = "Abbrechen";
pub const DIALOG_RELINK_SCAN: &str = "Neuen Ordner des Scans wählen (mit LIDAR_*.bag)";
/// Label of the background work that assigns a scan folder or moves a folder to the trash.
pub const RELINK_JOB: &str = "Scan zuordnen";
pub const TRASH_JOB: &str = "In den Papierkorb";
/// Why a scan cannot be changed now: a step of it runs or waits.
pub const SCAN_BUSY: &str = "Ein Schritt dieses Scans läuft oder wartet – erst danach";
/// A run in the run folder itself (made by the CLI): it is the run folder.
pub const MAP_IN_RUN: &str = "Liegt im Lauf-Ordner: mit dem Lauf löschen";

/// The question before „Aus Projekt entfernen“ of a scan.
pub fn remove_scan_question(
    name: &str,
    raw: &std::path::Path,
    results: &std::path::Path,
) -> String {
    format!(
        "Scan „{name}“ aus dem Projekt entfernen? Nur der Eintrag geht. Die Rohdaten bleiben \
         unberührt ({}), ebenso die Ergebnisse unter {}: wird der Scan wieder hinzugefügt, sind \
         sie wieder da.",
        raw.display(),
        results.display()
    )
}

/// The question before „Löschen …“: „Karte 001 löschen? Der Ordner (640 MB) wandert in den
/// Papierkorb.“ (without a size while it is not measured).
pub fn delete_question(label: &str, bytes: Option<u64>) -> String {
    match bytes {
        Some(bytes) => format!(
            "„{label}“ löschen? Der Ordner ({}) wandert in den Papierkorb und lässt sich von dort \
             zurückholen.",
            size(bytes)
        ),
        None => format!(
            "„{label}“ löschen? Der Ordner wandert in den Papierkorb und lässt sich von dort \
             zurückholen."
        ),
    }
}

/// What lies in a folder to delete: „Darin: Karte 001, Oberfläche 001“.
pub fn delete_contains(list: &[String]) -> String {
    format!("Darin: {}", list.join(", "))
}

/// The models that use a map to delete.
pub fn delete_used_by(list: &[String]) -> String {
    format!(
        "Davon hängt ab: {} – ihre Oberfläche und „Von Hand ausrichten“ gehen danach nicht mehr \
         (das Modell selbst bleibt sichtbar).",
        list.join(", ")
    )
}

/// After a folder went to the trash.
pub fn deleted(label: &str, bytes: Option<u64>) -> String {
    match bytes {
        Some(bytes) => format!("„{label}“ in den Papierkorb gelegt ({})", size(bytes)),
        None => format!("„{label}“ in den Papierkorb gelegt"),
    }
}

/// Why a folder cannot go to the trash now: it, or something that depends on it, is shown.
pub fn delete_shown(what: &str) -> String {
    format!("Wird gerade angezeigt ({what}) – erst „{UNSHOW}“")
}

/// Why a folder cannot go to the trash now: a step works on it, or it waits in the queue.
pub fn delete_computing(label: &str) -> String {
    format!("Wird gerade gerechnet ({label}) – nach dem Schritt oder „Abbrechen“")
}

/// A folder the trash cannot take (a network share, a removable drive on Windows).
pub fn trash_unavailable(path: &std::path::Path, why: &str) -> String {
    about_path(
        &format!("Kein Papierkorb für diesen Ordner ({why}); nichts gelöscht"),
        path,
    )
}

/// The trash of the system refused (German reason first, the system's text below).
pub fn trash_failed(path: &std::path::Path, error: &str) -> String {
    format!(
        "{}\n{error}",
        about_path("Nicht in den Papierkorb gelegt; nichts gelöscht", path)
    )
}

/// After „Ordner neu zuordnen …“.
pub fn relinked(name: &str, checked: bool) -> String {
    if checked {
        format!("Scan „{name}“ liest jetzt aus dem neuen Ordner")
    } else {
        format!(
            "Scan „{name}“ liest jetzt aus dem neuen Ordner (nicht prüfbar: der Scan hat noch \
             keinen Lauf)"
        )
    }
}

/// A folder of another scan: the bags differ.
pub fn other_scan_bags(then: &[(String, u64)], now: &[(String, u64)]) -> String {
    let list = |bags: &[(String, u64)]| {
        bags.iter()
            .map(|(name, bytes)| format!("{name} {}", size(*bytes)))
            .collect::<Vec<_>>()
            .join(", ")
    };
    format!(
        "Anderer Scan: die LiDAR-Aufnahmen unterscheiden sich (vorher {}, hier {})",
        list(then),
        list(now)
    )
}

/// A folder of another scan: recorded at other times than the runs of the scan.
pub fn other_scan_times(run: (f64, f64), scan: (f64, f64)) -> String {
    format!(
        "Anderer Scan: zu anderer Zeit aufgenommen (der Lauf des Scans umfasst {}–{} s, dieser \
         Ordner {}–{} s)",
        decimal(run.0, 1),
        decimal(run.1, 1),
        decimal(scan.0, 1),
        decimal(scan.1, 1)
    )
}

// After an interruption: what jobs cut off left behind
/// Beside each leftover in „Daten“.
pub const INCOMPLETE: &str = "unvollständig";
/// The start of the line above the leftovers.
pub const INCOMPLETE_TITLE: &str = "Unvollständig";
pub const CLEAN_UP: &str = "Aufräumen";
pub const CLEAN_UP_HINT: &str = "Legt diese Reste abgebrochener Schritte in den Papierkorb";
pub const CLEAN_UP_WAIT: &str = "Erst nach dem Schritt, der gerade läuft oder wartet";
pub const CLEAN_UP_JOB: &str = "Aufräumen";
/// At most this many leftovers are listed by name; the rest is counted.
pub const INCOMPLETE_LISTED: usize = 5;

/// The line above the leftovers: „Unvollständig – Reste abgebrochener Schritte (3 · 2,4 GB)“.
pub fn incomplete_title(count: usize, bytes: u64) -> String {
    format!(
        "{INCOMPLETE_TITLE} – Reste abgebrochener Schritte (Absturz, Stromausfall): {count} · {}",
        size(bytes)
    )
}

/// One leftover: „SLAM 002 · unvollständig · 12 KB“.
pub fn incomplete_entry(label: &str, bytes: u64) -> String {
    format!("{label} · {INCOMPLETE} · {}", size(bytes))
}

/// Below the leftovers listed: „und 4 weitere“.
pub fn incomplete_more(count: usize) -> String {
    format!("und {count} weitere")
}

/// After „Aufräumen“.
pub fn cleaned_up(count: usize, bytes: u64) -> String {
    format!(
        "{count} {} in den Papierkorb gelegt ({})",
        if count == 1 { "Rest" } else { "Reste" },
        size(bytes)
    )
}

/// The label of the read of a project folder (status bar while a project is opened).
pub const READ_PROJECT_JOB: &str = "Projekt lesen";
/// In the field while a project is being opened.
pub const PROJECT_READING: &str = "Projekt wird gelesen …";

/// A project in cloud storage: what happens to its files and what to do instead.
pub fn project_in_cloud(cloud: &str, dir: &std::path::Path) -> String {
    format!(
        "Projekt liegt in {cloud}: große Ergebnisse werden hochgeladen und bei knappem Platz \
         vom Rechner genommen, dann lädt jede Wolke erst langsam wieder herunter. Besser ein \
         lokaler Ordner, z. B. „{}“ im Benutzerordner.\n{}",
        crate::places::PROJECTS_FOLDER,
        dir.display()
    )
}

/// Under the heading „Daten“: „Projekt „Halle“ · 3 Scans“.
pub fn project_line(name: &str, scans: usize) -> String {
    let scans = match scans {
        1 => "1 Scan".to_string(),
        n => format!("{n} Scans"),
    };
    format!("Projekt „{name}“ · {scans}")
}

/// A size in bytes, decimal units, a decimal only below 10: „3 KB“, „3,4 MB“, „640 MB“,
/// „1,2 GB“, „85 GB“.
pub fn size(bytes: u64) -> String {
    let b = bytes as f64;
    if b < 1e6 {
        format!("{} KB", (b / 1e3).ceil() as u64)
    } else if b < 1e7 {
        format!("{} MB", decimal(b / 1e6, 1))
    } else if b < 1e9 {
        format!("{} MB", (b / 1e6).round() as u64)
    } else if b < 1e10 {
        format!("{} GB", decimal(b / 1e9, 1))
    } else {
        format!("{} GB", (b / 1e9).round() as u64)
    }
}

/// An entry of the tree with the size of its folder: „Karte 001 · 640 MB“.
pub fn with_size(label: &str, bytes: u64) -> String {
    format!("{label} · {}", size(bytes))
}

/// Under the project line: „12,4 GB im Projekt · 85 GB frei“ (without the free space if the
/// system does not say it).
pub fn disk_line(total: u64, free: Option<u64>) -> String {
    match free {
        Some(free) => format!("{} im Projekt · {} frei", size(total), size(free)),
        None => format!("{} im Projekt", size(total)),
    }
}

/// Little free space on the drive of the project.
pub fn low_space(free: u64) -> String {
    format!(
        "Nur noch {} frei auf dem Laufwerk des Projekts – Karten und Oberflächen brauchen \
         schnell mehrere GB; Platz schaffen (alte Ergebnisse löschen) oder das Projekt auf ein \
         größeres Laufwerk legen",
        size(free)
    )
}

/// A step that will need more than is free.
pub fn needs_room(label: &str, need: u64, free: u64) -> String {
    format!(
        "{label} braucht etwa {}, frei sind nur {} – der Schritt läuft, bricht aber ab, wenn \
         der Datenträger voll wird",
        size(need),
        size(free)
    )
}
/// Label of the background measurement of the disk use (not shown in the status bar).
pub const DISK_JOB: &str = "Platz messen";

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

/// „Wolke wird hochgeladen … 40 %“.
pub fn uploading(percent: f32) -> String {
    format!("Wolke wird hochgeladen … {percent:.0} %")
}

/// A distance: „2,00 m“ (centimetres).
pub fn meters(d: f64) -> String {
    format!("{} m", decimal(d, 2))
}

/// „Messung 1“: the name of a measurement in the field „Planung“ (counted from 1).
pub fn measurement_name(nr: usize) -> String {
    format!("Messung {nr}")
}

/// The info at the top left of the view: „Punktwolke · 6,0 Mio. Punkte“.
pub fn info_cloud(points: usize) -> String {
    format!("Punktwolke · {}", self::points(points))
}

/// „Oberfläche · 712.218 Dreiecke“.
pub fn info_surface(triangles: usize) -> String {
    format!(
        "Oberfläche · {} Dreiecke",
        super::status::count(triangles as u64)
    )
}

/// The numbers of the last frame below the info: „gezeichnet 5,9 Mio. · 8,4 ms GPU“, for a
/// surface alone (`drawn` none) only the time; without GPU timer the time between frames
/// („… ms je Bild“).
pub fn frame_line(drawn: Option<usize>, frame_ms: f32, gpu: bool) -> String {
    let time = format!(
        "{} ms {}",
        decimal(f64::from(frame_ms), 1),
        if gpu { "GPU" } else { "je Bild" }
    );
    match drawn {
        Some(n) => format!("gezeichnet {} · {time}", point_count(n)),
        None => time,
    }
}

/// A height of the cut: „2,85 m“.
pub fn height(z: f32) -> String {
    format!("{} m", decimal(f64::from(z), 2))
}

/// The line in the info of the view while a height cut hides points and its bar is not shown:
/// „Schnitt aktiv: z -0,50 bis 2,80 m“.
pub fn cut_active([lo, hi]: [f32; 2]) -> String {
    format!(
        "Schnitt aktiv: z {} bis {}",
        decimal(f64::from(lo), 2),
        height(hi)
    )
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

/// The page images of a surface claim more pixels than are loaded (both in pixels).
pub fn textures_too_large(pixels: u64, max: u64) -> String {
    format!(
        "Texturen zu groß: {} Mio. Pixel, höchstens {} Mio. werden geladen",
        pixels / 1_000_000,
        max / 1_000_000
    )
}

/// A page whose image could not be decoded (`page` counts from 0).
pub fn bad_texture(page: usize, error: &str) -> String {
    format!("Seite {page}: Textur nicht lesbar ({error})")
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
        "{name} geladen: {} (Laden {} s, Detailstufen {} s)",
        self::points(points),
        decimal(load_s, 1),
        decimal(build_s, 1)
    )
}

/// „1 Punkt“, „12.345 Punkte“, „15,6 Mio. Punkte“.
pub fn points(n: usize) -> String {
    match n {
        1 => "1 Punkt".into(),
        n => format!("{} Punkte", point_count(n)),
    }
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
pub const COMPARE_GAP: &str = "Start–Ende A/B";
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
/// Start–Ende 0,123 m“ (without the gap if `slam.json` could not be read).
pub fn history_entry(e: &driftless_steps::history::HistoryEntry, gap: Option<f64>) -> String {
    let mut s = format!(
        "{} · {} · {}",
        state_name(e.nr),
        e.kind.label(),
        stamp(&e.created)
    );
    if let Some(g) = gap {
        s.push_str(&format!(" · Start–Ende {}", compare_metres(g)));
    }
    s
}
pub const HISTORY_GAPS_JOB: &str = "Verlauf lesen";

/// The legend of a comparison in the view: „A: SLAM 001 Odometrie“.
pub fn legend(which: &str, name: &str) -> String {
    format!("{which}: {name}")
}

// Messages: system errors in German, symbols the fonts lack, paths shortened

/// The reason of a system error in German, by its kind; `None` for a kind without one (its
/// own text then stays).
pub fn io_reason(kind: std::io::ErrorKind) -> Option<&'static str> {
    use std::io::ErrorKind as K;
    Some(match kind {
        K::NotFound => "nicht gefunden",
        K::PermissionDenied => "keine Berechtigung",
        K::AlreadyExists => "gibt es schon",
        K::StorageFull | K::QuotaExceeded => "Datenträger voll",
        K::ReadOnlyFilesystem => "Datenträger schreibgeschützt",
        K::TimedOut => "keine Antwort (Netzlaufwerk oder iCloud nicht erreichbar?)",
        K::NotADirectory => "kein Ordner",
        K::IsADirectory => "ist ein Ordner",
        K::DirectoryNotEmpty => "Ordner nicht leer",
        K::InvalidFilename => "ungültiger Name",
        K::FileTooLarge => "Datei zu groß",
        K::ResourceBusy | K::ExecutableFileBusy => "wird gerade von einem anderen Programm benutzt",
        K::StaleNetworkFileHandle
        | K::NetworkUnreachable
        | K::NetworkDown
        | K::HostUnreachable
        | K::NotConnected
        | K::ConnectionReset
        | K::ConnectionAborted => "Netzlaufwerk nicht erreichbar",
        K::CrossesDevices => "liegt auf einem anderen Laufwerk",
        K::UnexpectedEof => "Datei endet zu früh (beschädigt?)",
        K::InvalidData => "Inhalt beschädigt",
        K::OutOfMemory => "Arbeitsspeicher reicht nicht",
        K::Interrupted => "unterbrochen",
        _ => return None,
    })
}

/// The command-line hints of the steps and what to do in the window instead.
const APP_HINTS: [(&str, &str); 4] = [
    (
        "; mit `driftless slam` neu erzeugen",
        "; den Scan neu rechnen (Scan wählen, „Rechnen“)",
    ),
    (
        "zuerst `driftless slam <scan> -o <ziel>` laufen lassen",
        "zuerst den Scan rechnen (Scan wählen, „Rechnen“)",
    ),
    (
        "zuerst `driftless map <scan> --slam <ziel>` laufen lassen",
        "zuerst die Karte rechnen (Scan wählen, „Rechnen“)",
    ),
    (
        "mit `driftless slam` neu rechnen",
        "den Scan neu rechnen (Scan wählen, „Rechnen“)",
    ),
];

/// `message` as the window says it: every system error („Permission denied (os error 13)“)
/// as its German reason ([`io_reason`]), a command-line hint of a step as the action in the
/// window ([`APP_HINTS`]; one that names its own folders: „zuerst den Scan rechnen“).
pub fn german(message: &str) -> String {
    let mut out = message.to_string();
    // „<system text> (os error N)“: the text is the system's own for N
    let mut from = 0;
    while let Some(at) = out[from..].find("(os error ") {
        let start = from + at;
        let digits: String = out[start + 10..]
            .chars()
            .take_while(|c| c.is_ascii_digit() || *c == '-')
            .collect();
        let end = start + 10 + digits.len();
        let Some(code) = digits
            .parse::<i32>()
            .ok()
            .filter(|_| out[end..].starts_with(')'))
        else {
            from = end;
            continue;
        };
        let system = std::io::Error::from_raw_os_error(code);
        let english = system.to_string();
        let reason = io_reason(system.kind());
        match (reason, out[..end + 1].rfind(&english)) {
            (Some(reason), Some(begin)) if begin + english.len() == end + 1 => {
                out.replace_range(begin..end + 1, reason);
                from = begin + reason.len();
            }
            _ => from = end + 1,
        }
    }
    for (cli, app) in APP_HINTS {
        out = out.replace(cli, app);
    }
    // the readers of TOML and JSON: where in the file
    out = places(
        &out,
        "TOML parse error at line ",
        ", column ",
        "beschädigt (Zeile ",
    );
    out = places(&out, " at line ", " column ", " (Zeile ");
    // any other: „zuerst `driftless …` laufen lassen“
    while let Some(start) = out.find("zuerst `driftless ") {
        let Some(len) = out[start..].find("` laufen lassen") else {
            break;
        };
        let end = start + len + "` laufen lassen".len();
        out.replace_range(
            start..end,
            "zuerst den Scan rechnen (Scan wählen, „Rechnen“)",
        );
    }
    out
}

/// `text` with „<from>N<sep>M“ (two numbers) as „<to>N, Spalte M)“: where a reader of TOML
/// or JSON stopped, in German.
fn places(text: &str, from: &str, sep: &str, to: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(at) = rest.find(from) {
        let after = &rest[at + from.len()..];
        let line: String = after.chars().take_while(char::is_ascii_digit).collect();
        let after_line = &after[line.len()..];
        let column = after_line
            .strip_prefix(sep)
            .map(|c| {
                c.chars()
                    .take_while(char::is_ascii_digit)
                    .collect::<String>()
            })
            .unwrap_or_default();
        if line.is_empty() || column.is_empty() {
            out.push_str(&rest[..at + from.len()]);
            rest = after;
            continue;
        }
        out.push_str(&rest[..at]);
        out.push_str(&format!("{to}{line}, Spalte {column})"));
        rest = &after_line[sep.len() + column.len()..];
    }
    out.push_str(rest);
    out
}

/// The symbols the fonts of the window lack (they come in the messages of the steps, which
/// a terminal shows as they are), with the symbol of the icon font in their place.
const SYMBOLS: [(char, &str); 12] = [
    ('→', egui_phosphor::regular::ARROW_RIGHT),
    ('←', egui_phosphor::regular::ARROW_LEFT),
    ('↔', egui_phosphor::regular::ARROWS_LEFT_RIGHT),
    ('↑', egui_phosphor::regular::ARROW_UP),
    ('↓', egui_phosphor::regular::ARROW_DOWN),
    ('✓', egui_phosphor::regular::CHECK),
    ('✔', egui_phosphor::regular::CHECK),
    ('✗', egui_phosphor::regular::X),
    ('✘', egui_phosphor::regular::X),
    ('⚠', egui_phosphor::regular::WARNING),
    ('▸', egui_phosphor::regular::CARET_RIGHT),
    ('▾', egui_phosphor::regular::CARET_DOWN),
];

/// `text` with the symbols the fonts lack ([`SYMBOLS`]) drawn by the icon font.
pub fn shown(text: &str) -> Cow<'_, str> {
    if !text.chars().any(|c| SYMBOLS.iter().any(|(s, _)| *s == c)) {
        return Cow::Borrowed(text);
    }
    let mut out = String::with_capacity(text.len() + 8);
    for c in text.chars() {
        match SYMBOLS.iter().find(|(s, _)| *s == c) {
            Some((_, icon)) => out.push_str(icon),
            None => out.push(c),
        }
    }
    Cow::Owned(out)
}

/// Whether `word` starts an absolute path: `/…` (not `//` alone), `C:\…`, `C:/…` or `\\…`.
fn starts_path(word: &str) -> bool {
    let b = word.as_bytes();
    let drive =
        b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && matches!(b[2], b'\\' | b'/');
    (b.len() > 1 && b[0] == b'/') || drive || word.starts_with(r"\\")
}

/// `text` with every absolute path cut to its last part (the file or folder name): a
/// message then says its reason within one line. A path ends before a space unless the next
/// word goes on with `/` or `\` (a folder name with spaces, „/Projekte/Halle 2026/scans“),
/// and before a closing „:“, „;“, „,“ or „)“.
pub fn short_paths(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for (k, line) in text.split('\n').enumerate() {
        if k > 0 {
            out.push('\n');
        }
        let words: Vec<&str> = line.split(' ').collect();
        let mut i = 0;
        while i < words.len() {
            if i > 0 {
                out.push(' ');
            }
            let (lead, word) = split_lead(words[i]);
            if !starts_path(word) {
                out.push_str(words[i]);
                i += 1;
                continue;
            }
            // the words of the path: further words while they go on with a separator
            let mut path = word.to_string();
            let mut j = i + 1;
            while j < words.len() && !ends_path(&path) {
                let next = words[j];
                let continues = next
                    .find(['/', '\\'])
                    .is_some_and(|at| !next[..at].contains([':', ';', ',', ')']));
                if !continues {
                    break;
                }
                path.push(' ');
                path.push_str(next);
                j += 1;
            }
            let (path, tail) = split_tail(&path);
            out.push_str(lead);
            out.push_str(last_part(path));
            out.push_str(tail);
            i = j;
        }
    }
    out
}

/// An opening bracket or quote before a path, and the rest.
fn split_lead(word: &str) -> (&str, &str) {
    let at = word
        .find(|c: char| !matches!(c, '(' | '„' | '"' | '\''))
        .unwrap_or(word.len());
    word.split_at(at)
}

/// Whether a path read so far already ends with what closes it („:“ and the like).
fn ends_path(path: &str) -> bool {
    path.ends_with([':', ';', ',', ')', '“', '"'])
}

/// The path and what closes it (punctuation that belongs to the sentence).
fn split_tail(path: &str) -> (&str, &str) {
    let trimmed = path.trim_end_matches([':', ';', ',', ')', '“', '"', '.']);
    path.split_at(trimmed.len())
}

/// The last part of `path` (`/` or `\` separated), the path itself if it ends with one.
fn last_part(path: &str) -> &str {
    path.trim_end_matches(['/', '\\'])
        .rsplit(['/', '\\'])
        .next()
        .filter(|p| !p.is_empty())
        .unwrap_or(path)
}

/// A message about `path`: the reason first, then the name of the file or folder, the whole
/// path on the next line (the window shows it on hover, the Protokoll and the log file in
/// full): „Ordner ist nicht leer: Projekte\n/Users/…/Projekte“.
pub fn about_path(reason: &str, path: &Path) -> String {
    let name = path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    format!("{reason}: {name}\n{}", path.display())
}

/// [`about_path`] for a system error with the file it was about: „keine Berechtigung:
/// karte.laz\n…“ (the reason of [`io_reason`], else the system's text).
pub fn io_error(what: &str, path: &Path, e: &std::io::Error) -> String {
    let reason = io_reason(e.kind()).map_or_else(|| german(&e.to_string()), str::to_string);
    about_path(&format!("{what}: {reason}"), path)
}

/// The line and column (from 1) of byte `at` of `text`.
pub fn line_column(text: &str, at: usize) -> (usize, usize) {
    let before = &text[..at.min(text.len())];
    let line = before.matches('\n').count() + 1;
    let column = before.rsplit('\n').next().map_or(0, |l| l.chars().count()) + 1;
    (line, column)
}

/// A TOML file that cannot be read: „project.toml ist beschädigt (Zeile 3, Spalte 7):
/// <what the reader says>“, the whole path on the next line.
pub fn damaged_toml(path: &Path, text: &str, e: &toml::de::Error) -> String {
    let name = path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    let place = e.span().map(|span| line_column(text, span.start));
    let what = e.message().lines().next().unwrap_or_default().trim();
    let head = match place {
        Some((line, column)) => format!("{name} ist beschädigt (Zeile {line}, Spalte {column})"),
        None => format!("{name} ist beschädigt"),
    };
    format!("{head}: {what}\n{}", path.display())
}
