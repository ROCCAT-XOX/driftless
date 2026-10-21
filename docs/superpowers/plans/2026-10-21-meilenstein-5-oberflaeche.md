# Meilenstein 5 – Programm mit Oberfläche – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ein Desktop-Programm `driftless-app` (macOS arm64, Windows x64): Projekte und Scans links, flüssige 3D-Ansicht der Punktwolke in der Mitte (Ziel ~50 Mio. Punkte), Schritte und Eigenschaften rechts; `inspect`, `slam`, `slam --reuse` und `map` laufen im Hintergrund mit Fortschritt und Abbruch, die Laufbahn ist nach Güte gefärbt, Schleifen werden durch Anklicken zweier Stellen eingetragen und in Sekunden neu gerechnet, Vorher/Nachher ist vergleichbar, nichts wird überschrieben.

**Architecture:** Die Ablauflogik, die heute im Binary `driftless-cli` steckt (Bags lesen, `slam`, `correct`, `map`, `inspect`, `compare`), zieht in eine neue Bibliothek `driftless-steps`; CLI und Oberfläche rufen dieselben Funktionen auf (kein Aufruf des CLI-Programms). Jede lange Funktion bekommt ein gemeinsames `Progress` (Phase, Zähler, Abbruch, Meldungen) aus `driftless-io`. Die neue Crate `driftless-app` enthält Projektmodell, Job-Läufer, LAS-Laden, Octree mit Detailstufen, Kamera- und Picking-Mathematik (alles ohne GPU testbar), einen eigenen wgpu-Renderer (instanzierte Punkt-Quads, Reverse-Z, in eine eigene Textur, die egui als Bild zeigt) und die egui-Panels, die nur `Action`s zurückgeben und deshalb ohne Fenster testbar sind.

**Tech Stack:** Rust 1.99; neu `eframe`/`egui`/`egui-wgpu` 0.36 (wgpu 30 über `egui_wgpu::wgpu`, Metal/DX12/Vulkan), `rfd` 0.17 (Dateidialoge), `bytemuck` (Vertexdaten), `egui_kittest` 0.36 (nur Tests); weiter `nalgebra`, `rayon`, `las`, `serde`, `toml`, `mimalloc`.

**Spec:** `docs/superpowers/specs/2026-10-20-driftless-design.md` (Ziel 2 „Manuelle Korrektur … Vorher/Nachher-Vergleich“, Ziel 5 „Programm mit Oberfläche“, Abschnitt 5 Plattformen/GPU, Abschnitt 6 Oberfläche egui + eigener wgpu-Renderer, „Antwortzeit Korrekturen in Sekunden“). Ausgangslage: Branch `meilenstein-4` (Commit `c66805c`), `README.md` (Befehlstabelle), `docs/ergebnisse/2026-10-21-meilenstein-4.md`.

## Entscheidungen (Controller, verbindlich)

- M5 = nur die Oberfläche. Export E57/LAZ/PCD und Installer/Signatur sind M6.
- Neue Crate `driftless-app`, nutzt die Bibliotheken direkt, ruft nie das CLI-Programm auf.
- Inhalt: Scan-Ordner/Projekt öffnen; `inspect`, `slam`, `slam --reuse`, `map` im Hintergrund mit Fortschritt und Abbruch; 3D-Ansicht mit LOD/Octree für ≥ 50 Mio. Punkte (Farbe/Intensität/Güte, Umkreisen/Fliegen, Punkt anklicken); Laufbahn nach Güte, Schleifen und Teilkarten eingeblendet; manuelle Schleife durch zwei angeklickte Stellen, dieselbe `corrections.toml` wie `driftless correct`, auflisten/ab-/einschalten, dann Neu-Rechnen aus Teilkarten; Vorher/Nachher (`traj.vorher.tum` gegen `traj.tum`, Start↔Ende, ATE); Eigenschaften-Panel mit allen Schaltern von `driftless map`; Oberfläche Deutsch.
- Tests: Kernlogik ohne Fenster; Renderer mit wgpu-Rauchtest ohne Fenster, der sauber überspringt, wenn kein Adapter da ist; echte Daten `#[ignore]` über `$DRIFTLESS_TESTDATA`; LOD-Aufbauzeit und Speicher auf der 15-Mio.-Punkte-Wolke des Büro-Scans (20261020173822) messen.

## Entscheidungen (dieser Plan)

- **Eigenes Binary `driftless-app` statt `driftless gui`:** Das CLI bleibt ohne GPU-/Fenster-Abhängigkeiten (läuft auf Servern, seine Tests linken kein wgpu/winit); das Oberflächen-Binary braucht unter Windows `#![windows_subsystem = "windows"]` (kein Konsolenfenster), was ein gemeinsames Binary unmöglich macht; der Installer (M6) packt genau dieses Binary als `.app`/`.exe`.
- **Ablauflogik in `driftless-steps`** statt `driftless-cli` als Bibliothek: sonst zöge die App `clap` und `rerun` mit. Das CLI ändert sein Verhalten nicht (gleiche Dateien, gleiche Texte, alle CLI-Tests unverändert grün).
- **egui/eframe 0.36** (0.36.2 vom 08.09.2026, rust-version 1.95, wgpu 30), nicht 0.37.0 (erst seit 15.10.2026). `eframe` mit `default-features = false, features = ["wgpu", "default_fonts", "accesskit", "persistence"]`.
- **Lizenzen:** `epaint_default_fonts` steht unter `(MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0` (nur die eingebetteten Schriften). `deny.toml` bekommt `OFL-1.1` und `Ubuntu-font-1.0` mit Kommentar; weitere Lizenzen, die `cargo deny` meldet, nur wenn permissiv (z. B. `CC0-1.0`), jede mit Kommentar; GPL/LGPL/AGPL → anhalten und melden.
- **Projekt auf der Platte** (Spec Abschnitt 7): Ordner mit `project.toml`; jeder SLAM-Lauf ist ein CLI-kompatibler `-o`-Ordner `scans/<scan-id>/slam-NNN/` (zusätzlich `lauf.toml` mit den Einstellungen und `verlauf/NNN/`), jede Karte ein eigener Ordner `slam-NNN/karte-NNN/`. Mit dem CLI erzeugte Ordner lassen sich als Lauf hinzufügen.
- **„Nichts wird überschrieben“ in der App:** Ein neuer SLAM-Lauf bekommt einen neuen Ordner. Neu-Rechnen aus Teilkarten bleibt im Lauf-Ordner wie `--reuse` (`traj.vorher.tum`), legt aber nach jedem Lauf einen Stand `verlauf/NNN/` an (`traj.tum`, `slam.json`, die dabei gültige `corrections.toml`, `stand.toml`); jeder Stand ist vergleichbar und wiederherstellbar (Schalter der Korrekturen zurückstellen, nie löschen). Eine Karte mit anderen Einstellungen oder zu einer neu gerechneten Laufbahn bekommt einen neuen `karte-NNN`; passende `dense.bin`/`clean.bin` werden aus dem jüngsten Kartenordner kopiert. Das CLI schreibt keinen Verlauf (Verhalten unverändert).
- **Ein Verarbeitungs-Job zur Zeit** (plus ein getrennter Lade-Job für Wolken): verhindert, dass zwei Schritte in denselben Ordner schreiben. Abbruch wirkt bis vor dem Schreiben; Schreiben wird nicht unterbrochen; ein abgebrochener Schritt hinterlässt keine Dateien.
- **Renderer:** alle Octree-Knoten liegen auf der GPU (Blöcke ≤ 4.194.304 Punkte à 20 B = 80 MB, unter der Standardgrenze 256 MB je Puffer); je Bild wird nach Bildschirmgröße und Punktbudget ausgewählt. Picking auf der CPU über den Octree (testbar, kein ID-Puffer). Kein LOD-Cache auf der Platte in M5 (erst, wenn die Messung in Task 13 es verlangt).
- Nicht in M5: Abschnitte der Laufbahn verwerfen (Spec Ziel 2) – nicht im Umfang des Controllers; Kandidat für M6.
- Welt ist z-oben (Schwerkraft aus der Odometrie). Kamera-Mathematik mit `nalgebra` (schon im Workspace), kein `glam`.

## Global Constraints

- Lizenz Apache-2.0 OR MIT; nur permissive Abhängigkeiten, `cargo deny check` grün (Ausnahmen siehe oben, je mit Kommentar).
- macOS arm64 und Windows x64 nativ, CI grün auf `macos-14` und `windows-latest`; `rust-version = "1.99"`; GPU über wgpu (Metal auf Mac, DX12/Vulkan unter Windows), kein CUDA.
- Deterministisch: gleiche Eingabe → byte-gleiche Ausgabedateien und gleicher Octree (feste Sortierschlüssel, keine Abhängigkeit von der Thread-Zahl).
- Spec: „Nichts wird überschrieben, jede Korrektur ist ein rücknehmbarer Schritt“; „manueller Schleifenschluss aktualisiert die Laufbahn in Sekunden“; 3D-Ansicht „flüssig bis ~50 Mio. Punkte“ (hier: ≥ 30 Bilder/s beim Drehen auf dem M4 Pro und der RTX 5080).
- Die Oberfläche blockiert nie: alles über ~50 ms (Laden, LOD-Aufbau, Schritte, Neu-Rechnen) läuft im Hintergrund.
- `driftless-cli` behält Befehle, Dateien und Texte; alle bestehenden Tests bleiben unverändert grün.
- Oberfläche und Meldungen Deutsch, Code-Kommentare Englisch. Scan-Daten, Bilder, Seriennummern, Kundennamen, persönliche Pfade nie ins Repo; Tests mit echten Daten `#[ignore]` über `$DRIFTLESS_TESTDATA`. Der lange Testscan heißt in Code und Doku „Büro-Scan (20261020173822)“.
- `export PATH="$HOME/.cargo/bin:$PATH"` vor jedem `cargo`.

## Review Focus

- Scan-Ordner verschoben oder Laufwerk abgezogen → Projekt öffnet trotzdem, Läufe und Wolken bleiben ansehbar, Schritte, die den Scan brauchen, sind mit deutschem Grund gesperrt (Task 3: `missing_scan_folder_is_reported_not_fatal`).
- Fenster schließen oder Projekt wechseln, während ein Schritt läuft → Job wird abgebrochen, keine halb geschriebenen Dateien (Task 4: `dropping_runner_cancels_job`; Task 2: `slam_cancelled_writes_nothing`, `reuse_cancelled_keeps_traj`, `map_cancelled_writes_nothing`).
- Retina-/HiDPI-Bildschirm (`pixels_per_point` 2) und Windows mit 150 % Skalierung → Klick trifft den Punkt unter dem Mauszeiger, nicht einen versetzten (Task 6: `pick_uses_physical_pixels`).
- Wolke mit großen Koordinaten (Versatz ~ 5·10⁵ m), ein einziger Punkt, lauter gleiche Punkte, leere LAS → baut, zeigt ohne Zittern, kein Absturz (Task 5: `far_coordinates_keep_mm_precision`, `degenerate_clouds_build`).
- Zwei angeklickte Stellen in derselben Teilkarte, außerhalb des Scans oder doppelt → deutsche Meldung aus der Prüfung von `driftless correct`, `corrections.toml` bleibt unverändert (Task 11: `invalid_loop_shows_error_and_writes_nothing`).

---

### Task 1: Fortschritt und Abbruch in den Bibliotheken

**Files:**
- Create: `crates/driftless-io/src/progress.rs`, `crates/driftless-io/src/testscan.rs` (feature `test-util`: `tmp`, `time`, `room_scan`, `room_bag`, `room_bag_secs` aus `crates/driftless-cli/tests/common/mod.rs` hierher verschoben; dort nur noch `pub use driftless_io::testscan::*;`)
- Modify: `crates/driftless-io/src/lib.rs`, `crates/driftless-io/src/bag/reader.rs`, `crates/driftless-map/src/color.rs`, `crates/driftless-map/src/clean.rs`, `crates/driftless-map/src/lib.rs`
- Test: `crates/driftless-io/tests/progress.rs`, `crates/driftless-io/tests/bag.rs`, `crates/driftless-map/tests/color.rs`, `crates/driftless-map/tests/clean.rs`

**Interfaces:**
- Produces:
  - `driftless_io::progress::Progress` (`Clone`, intern `Arc`; `Send + Sync`): `Progress::new() -> Self`; `Progress::with_log(sink: impl Fn(&LogLine) + Send + Sync + 'static) -> Self`; `phase(&self, name: &str, total: Option<u64>)` (setzt `done = 0`); `advance(&self, n: u64)`; `info(&self, text: impl Into<String>)`; `warn(&self, text: impl Into<String>)`; `cancel(&self)`; `is_cancelled(&self) -> bool`; `snapshot(&self) -> ProgressSnapshot`; `take_log(&self) -> Vec<LogLine>` (Meldungen seit dem letzten Aufruf, Reihenfolge erhalten).
  - `pub struct ProgressSnapshot { pub phase: String, pub done: u64, pub total: Option<u64> }`, `pub enum LogLine { Info(String), Warning(String) }`.
  - `BagReader::message_count(&mut self, topic: &str) -> Option<u64>`: Summe der Zähler der Chunk-Info-Einträge (op `0x06`) im Index für die Verbindungen des Topics; `None` ohne Index oder bei beschädigtem Index (nie ein Fehler).
  - `driftless_map::colorize_with(cloud: &mut Cloud, images: ImageSource, poses: &[Pose], cams: &[ColorCamera], cfg: &ColorConfig, progress: &Progress) -> Result<ColorStats, MapError>`: Phasen „Bilder prüfen“ und „Einfärben“ (je Bild `advance(1)`, Gesamtzahl aus der Bildliste), Abbruch je Bild geprüft. `colorize` ruft es mit `Progress::new()`.
  - `driftless_map::clean_with(cloud: Cloud, rays: &RaySource, cfg: &CleanConfig, progress: &Progress)` gleiche Rückgabe wie `clean`; Abbruch zwischen den Teilschritten Geister → Durchblick → Ausreißer → Glätten; Phasen mit diesen deutschen Namen.
  - `MapError::Cancelled` mit Text „abgebrochen“.
- [ ] **Step 1: Tests** – `progress_phase_resets_counter`, `progress_is_shared_between_clones` (Abbruch in einem Klon sichtbar im anderen, über Threads), `log_keeps_order_and_reaches_sink`. `message_count_from_index` (`BagWriter` indiziert, 2 Topics mit 7 und 3 Nachrichten → `Some(7)`, `Some(3)`, unbekanntes Topic `Some(0)`), `message_count_unindexed_is_none` (`.indexed(false)`). `colorize_with_matches_colorize` (gleiche Szene wie `color.rs`-Tests: `rgb`, `colored` und Statistik gleich), `colorize_cancelled_returns_cancelled` (vor dem Aufruf abgebrochen → `Err(MapError::Cancelled)`, Wolke unverändert), `colorize_reports_images` (`done == total` am Ende). `clean_cancelled_returns_cancelled`.
- [ ] **Step 2: Laufen lassen** `cargo test -p driftless-io -p driftless-map` → FAIL (Funktionen fehlen).
- [ ] **Step 3: Implementieren**, dann `cargo test --workspace` → PASS (CLI-Tests mit den verschobenen Hilfen unverändert grün).
- [ ] **Step 4: Commit** `feat(io,map): Fortschritt und Abbruch für lange Schritte`

### Task 2: Bibliothek `driftless-steps` – Ablauflogik aus dem CLI

**Files:**
- Create: `crates/driftless-steps/Cargo.toml` (deps: `driftless-io`, `-odom`, `-slam`, `-map`, `-traj`, `las`, `rayon`, `serde`, `serde_json`, `toml`; dev: `driftless-io` mit `test-util`), `crates/driftless-steps/src/{lib.rs, error.rs, scan.rs, inspect.rs, slam.rs, correct.rs, map.rs, compare.rs, history.rs}`
- Modify: `Cargo.toml` (Workspace-Mitglied, `[profile.dev.package.driftless-steps] opt-level = 2`), `crates/driftless-cli/Cargo.toml`, `crates/driftless-cli/src/{main.rs, inspect.rs, slam.rs, correct.rs, map.rs, compare.rs, odom.rs, calib_check.rs, export_glim.rs}`; `scan_input.rs` und `topics.rs` ziehen nach `driftless-steps/src/scan.rs` um.
- Test: `crates/driftless-steps/tests/{slam.rs, correct.rs, map.rs, history.rs}`

**Interfaces:**
- Consumes: `Progress`, `colorize_with`, `clean_with`, `BagReader::message_count`, `testscan` (Task 1).
- Produces:
  - `pub enum StepError { Cancelled, Failed(String) }` (`Display` deutsch, `Cancelled` = „abgebrochen“; `From<String>`, `From<&str>`, `From<Box<dyn Error>>`, `From<MapError>` mit `MapError::Cancelled → Cancelled`).
  - `scan`: bisherige Funktionen aus `scan_input.rs`/`topics.rs` öffentlich, `with_lidar_frames` mit zusätzlichem `progress: &Progress` (Phase „LiDAR lesen“, Gesamtzahl aus `message_count`, je Frame `advance(1)`; bei Abbruch endet der Frame-Strom), `image_topics` aus `calib_check.rs`.
  - `inspect`: `Report`, `Device`, `TopicReport` wie heute; `pub fn inspect(folder: &Path, progress: &Progress) -> Result<Report, StepError>`; `pub fn render_text(r: &Report) -> String` (bisher `render`).
  - `slam`: `#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)] pub struct SlamSettings { pub lidar_to_imu: Option<[f64; 7]>, pub lidar_topic: Option<String>, pub imu_topic: Option<String> }`; `pub struct SlamRequest { pub scan: PathBuf, pub out: PathBuf, pub settings: SlamSettings, pub reuse: bool, pub history: bool }`; `pub struct SlamOutcome { pub report: SlamReport, pub map_points: usize, pub seconds: f64, pub history: Option<PathBuf> }`; `pub fn run_slam(req: &SlamRequest, progress: &Progress) -> Result<SlamOutcome, StepError>`. Alle bisherigen Konsolenzeilen gehen als `progress.info`/`progress.warn` in derselben Reihenfolge raus. Abbruch wird geprüft nach der Odometrie, vor `save_cache`, vor dem Umbenennen nach `traj.vorher.tum` und vor `write_outputs`; danach nicht mehr. Konstanten `CACHE_FILE`, `CORRECTIONS_FILE` hier.
  - `history`: `pub fn record(out: &Path, kind: HistoryKind, seconds: f64) -> Result<PathBuf, StepError>` legt `verlauf/NNN/` (NNN = größte vorhandene + 1, drei Stellen) an mit Kopien von `traj.tum`, `slam.json`, `corrections.toml` (leere Datei mit Kopfzeile, wenn keine) und `stand.toml { nr, art = "voll" | "neu gerechnet", erstellt (UTC RFC 3339), sekunden }`; `pub enum HistoryKind { Full, Reuse }`; `pub struct HistoryEntry { pub nr: u32, pub dir: PathBuf, pub kind: HistoryKind, pub created: String }`; `pub fn list(out: &Path) -> Vec<HistoryEntry>` (nach `nr`, unlesbare Einträge übersprungen); `pub fn restore_flags(out: &Path, nr: u32) -> Result<usize, StepError>`: setzt in der aktuellen `corrections.toml` Eintrag i auf den Schalter von Eintrag i im Stand, später angehängte Einträge auf aus; nie entfernt; liefert die Zahl geänderter Einträge. `run_slam` ruft `record` bei `history: true` nach dem Schreiben.
  - `correct`: `pub fn load(run: &Path) -> Result<Corrections, StepError>`; `pub fn check_new_loop(run: &Path, c: &Corrections, a: f64, b: f64) -> Result<(), StepError>` (bisheriger Code); `pub fn add_loop(run: &Path, a: f64, b: f64, note: &str) -> Result<usize, StepError>` (prüfen, anhängen, speichern; Nummer ab 1); `pub fn set_enabled(run: &Path, nr: usize, enabled: bool) -> Result<bool, StepError>` (Einschalten wird wie eine neue Schleife geprüft); `pub fn utc_now() -> String`. Fehlender Lauf-Ordner → bisherige Meldung.
  - `map`: `#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)] pub enum MapStep { Dense, Clean, Color, All }`; `#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)] pub struct MapSettings { pub step: MapStep, pub ghosts: bool, pub outliers: bool, pub through: bool, pub smooth: bool, pub highlights: bool, pub vignetting: bool, pub voxel: f64, pub downscale: u32 }`, `Default` = CLI-Standard (`All`, an, an, an, an, aus, an, `DenseConfig::default().voxel`, `ColorConfig::default().downscale`); `pub fn check(&self) -> Result<(), StepError>` (bisherige Meldungen zu `--voxel`/`--downscale`). `pub struct MapRequest { pub scan: PathBuf, pub slam_dir: PathBuf, pub out_dir: PathBuf, pub slam: SlamSettings, pub map: MapSettings }`; `pub fn run_map(req: &MapRequest, progress: &Progress) -> Result<serde_json::Value /* map.json */, StepError>`: liest `traj.tum` aus `slam_dir`, schreibt alles nach `out_dir` (CLI: beide = `--slam`). Abbruch vor jedem Schreiben einer Datei; ein abgebrochener Schritt schreibt nichts, vorher fertige Schritte bleiben. `pub fn folder_fits(req: &MapRequest, dir: &Path) -> bool`: `dir/map.json` hat dieselben `settings` (ohne `step`) und einen `dense`-Eintrag für die aktuelle Dense-Eingabe-Kennung. `map.json` bleibt Feld für Feld wie heute.
  - `compare`: `Report`, `Pair` wie heute; `pub fn compare(a: &[Pose], b: &[Pose], max_dt: f64) -> Result<Report, StepError>`.
  - CLI: jedes Modul parst nur noch Argumente, baut die Anfrage und druckt; `Progress::with_log` druckt `Info` nach stdout, `Warning` als „Warnung: …“ nach stderr.
- [ ] **Step 1: Ist-Stand festhalten** – vor dem Umbau mit dem Release-Binary auf einer synthetischen Szene aus `driftless-cli/tests/map.rs` (`scan_with_images`) und auf `Test` (`$DRIFTLESS_TESTDATA`) `slam`, `slam --reuse`, `map` laufen lassen, Ausgaben (ohne `map.json`-Felder `seconds*`, `peak_rss_bytes`) und Konsole nach `$DRIFTLESS_TESTDATA/ergebnisse-m5/vorher/` sichern.
- [ ] **Step 2: Tests** (`driftless-steps`, synthetische Szenen aus `testscan`) – `slam_writes_cli_outputs` (`traj.tum`, `traj_odom.tum`, `map.las`, `slam.json`, `submaps.bin`), `slam_cancelled_writes_nothing` (Abbruch nach 5 Frames aus einem `with_log`-Hook: Ausgabeordner leer bzw. unverändert), `reuse_cancelled_keeps_traj` (`traj.tum` byte-gleich, kein `traj.vorher.tum`), `progress_counts_frames` (`total == Some(Frames der Bag)`, am Ende `done == total`), `history_records_each_run` (voll + 2× reuse → `verlauf/001..003`, `art` stimmt, `corrections.toml` im Stand = die beim Lauf gültige), `restore_sets_flags_never_removes` (3 Einträge, Stand mit nur Eintrag 1 aktiv → danach 1 an, 2 und 3 aus, Länge 3), `add_loop_round_trip_with_cli_format` (Datei aus `add_loop` = Datei, die `Corrections::add_loop` + `save` mit gleicher Zeit schreibt; `load` liest beide gleich), `add_loop_rejects_same_submap`, `map_cancelled_writes_nothing` (Abbruch während `clean` → kein `clean.las`, `removed.las`, `clean.bin`; `dense.*` von vorher unverändert), `map_out_dir_separate_from_slam_dir` (Ergebnisse nur in `out_dir`, `slam_dir` unverändert), `folder_fits_detects_new_traj` (nach Reuse mit neuer Schleife → `false`, gleiche Einstellungen und Laufbahn → `true`), `settings_default_equals_cli_defaults`.
- [ ] **Step 3: Laufen lassen** `cargo test -p driftless-steps` → FAIL.
- [ ] **Step 4: Umziehen und implementieren**; `cargo test --workspace` → PASS, CLI-Tests unverändert.
- [ ] **Step 5: Vergleich** – Step 1 wiederholen nach `…/nachher/`; `cmp` aller Dateien → gleich, Konsolentexte gleich (nur Laufzeiten verschieden).
- [ ] **Step 6: Commit** `refactor: Ablauflogik in driftless-steps, CLI unverändert`

### Task 3: Projektmodell

**Files:**
- Create: `crates/driftless-app/Cargo.toml` (vorerst nur Bibliothek: deps `driftless-steps`, `-slam`, `-traj`, `-io`, `serde`, `toml`, `serde_json`, `rayon`, `nalgebra`, `las`, `bytemuck = { version = "1", features = ["derive"] }`), `crates/driftless-app/src/lib.rs`, `crates/driftless-app/src/project.rs`
- Modify: `Cargo.toml` (Mitglied, `[profile.dev.package.driftless-app] opt-level = 2`)
- Test: `crates/driftless-app/tests/project.rs`

**Interfaces:**
- Consumes: `SlamSettings`, `MapSettings`, `MapRequest`, `folder_fits`, `history::list` (Task 2).
- Produces:
  - Dateien: `project.toml` = `format = 1`, `name`, `[[scan]] id, path, name, extra_runs = [...]`; `lauf.toml` im Lauf-Ordner = `scan` (Pfad), `erstellt`, `[slam]` = `SlamSettings`.
  - `pub struct Project { pub dir: PathBuf, pub name: String, pub scans: Vec<ScanEntry> }`, `pub struct ScanEntry { pub id: String, pub path: PathBuf, pub name: String, pub extra_runs: Vec<PathBuf> }`.
  - `Project::create(dir: &Path, name: &str) -> Result<Project, ProjectError>` (Ordner leer oder neu; vorhandene `project.toml` → Fehler „Projekt existiert schon“); `Project::open(dir: &Path)`; `Project::save(&self)` (temporär schreiben + umbenennen, wie `Corrections::save`); `add_scan(&mut self, scan_dir: &Path) -> Result<String, ProjectError>` (prüft mit `discover_project`; `id` = Ordnername, bei Doppel `-2`, `-3`; derselbe Ordner zweimal → Fehler); `add_run(&mut self, scan_id: &str, run_dir: &Path)` (CLI-Ordner mit `slam.json`, sonst Fehler); `new_run_dir(&self, scan_id: &str, settings: &SlamSettings) -> Result<PathBuf, ProjectError>` (nächster `slam-NNN`, angelegt mit `lauf.toml`).
  - `pub fn scan_state(&self, scan_id: &str) -> ScanState` mit `pub enum ScanState { Ready, Missing(String) /* deutsch */ }`.
  - `pub fn runs(&self, scan_id: &str) -> Vec<RunInfo>`: Ordner unter `scans/<id>/slam-*` nach Nummer, dann `extra_runs` in Eintragsreihenfolge. `pub struct RunInfo { pub dir: PathBuf, pub label: String /* „SLAM 001“ bzw. Ordnername */, pub settings: SlamSettings /* aus lauf.toml, sonst Default */, pub has_result: bool /* slam.json + traj.tum */, pub has_cache: bool /* submaps.bin */, pub has_previous: bool /* traj.vorher.tum */, pub maps: Vec<MapInfo>, pub history: Vec<HistoryEntry> }`; `pub struct MapInfo { pub dir: PathBuf, pub label: String, pub settings: Option<MapSettings>, pub clouds: Vec<PathBuf> /* vorhandene von color.las, clean.las, dense.las, removed.las in dieser Reihenfolge */ }`. Ein CLI-Lauf mit `map.json` im Lauf-Ordner selbst erscheint als Karte „Karte (im Lauf-Ordner)“; `map.las` des Laufs als Wolke des Laufs.
  - `pub fn map_dir_for(run: &RunInfo, req: &MapRequest) -> Result<PathBuf, ProjectError>`: jüngster `karte-NNN`, für den `folder_fits` gilt; sonst neuer `karte-(N+1)` mit Kopien (`std::fs::copy`) von `dense.bin`/`clean.bin` des jüngsten Kartenordners, falls vorhanden.
- [ ] **Step 1: Tests** – `create_open_round_trip`, `create_refuses_existing_project`, `scan_ids_are_unique`, `same_scan_twice_is_refused`, `runs_are_numbered_and_sorted` (`slam-002` vor `slam-010`), `cli_folder_can_be_added_as_run` (Ordner aus `run_slam` ohne `lauf.toml`: Default-Einstellungen, Karte im Lauf-Ordner erkannt), `missing_scan_folder_is_reported_not_fatal` (Scan-Ordner gelöscht: `open` klappt, `scan_state` = `Missing("Scan-Ordner … nicht gefunden")`, `runs` weiter vollständig), `run_without_cache_cannot_reuse` (`has_cache == false`), `map_dir_reused_when_settings_and_traj_fit`, `new_map_dir_on_changed_settings_copies_bins`, `new_map_dir_after_reuse` (neue Laufbahn → neuer Ordner, alter unverändert), `relative_and_windows_style_paths` (Pfade im Projektordner relativ gespeichert, außerhalb absolut).
- [ ] **Step 2: Laufen lassen** `cargo test -p driftless-app --test project` → FAIL.
- [ ] **Step 3: Implementieren**, Step 1 → PASS.
- [ ] **Step 4: Commit** `feat(app): Projektmodell mit Läufen, Karten und Verlauf`

### Task 4: Job-Läufer

**Files:**
- Create: `crates/driftless-app/src/jobs.rs`
- Test: `crates/driftless-app/tests/jobs.rs`

**Interfaces:**
- Consumes: `Progress`, `StepError`, `inspect::Report`, `SlamOutcome` (Tasks 1–2).
- Produces:
  - `pub struct JobRunner`; `JobRunner::new(repaint: Arc<dyn Fn() + Send + Sync>) -> Self`; `start(&mut self, label: impl Into<String>, job: impl FnOnce(&Progress) -> Result<JobOutput, StepError> + Send + 'static) -> Result<(), Busy>` (eigener Thread `std::thread::Builder` mit Namen, kehrt sofort zurück); `cancel(&self)`; `is_busy(&self) -> bool`; `status(&self) -> Option<JobStatus>`; `poll(&mut self) -> Option<JobFinished>` (einmal je fertigem Job); `take_log(&mut self) -> Vec<LogLine>`. `Drop` bricht einen laufenden Job ab und wartet höchstens 5 s.
  - `pub struct Busy;` (Text „Es läuft schon: <label>“), `pub struct JobStatus { pub label: String, pub progress: ProgressSnapshot, pub seconds: f64, pub cancelling: bool }`, `pub struct JobFinished { pub label: String, pub result: Result<JobOutput, StepError>, pub seconds: f64 }`.
  - `pub enum JobOutput { Inspect(Box<inspect::Report>), Slam(Box<SlamOutcome>), Map(Box<serde_json::Value>), Done }` (`Cloud(Box<LoadedCloud>)` ergänzt Task 10, wo `LoadedCloud` entsteht).
  - Panik im Job → `Err(StepError::Failed("interner Fehler: <Text>"))` (`catch_unwind`), die Oberfläche läuft weiter.
- [ ] **Step 1: Tests** – `start_returns_immediately` (Job wartet auf einen Kanal; `start` < 50 ms, `is_busy`), `status_shows_phase_and_counts`, `second_job_is_refused_while_busy`, `cancel_reaches_job` (Job wartet auf `is_cancelled` → `Err(Cancelled)`, `status.cancelling` vorher `true`), `panicking_job_becomes_error`, `dropping_runner_cancels_job`, `log_lines_arrive_in_order`, `repaint_is_requested_on_finish`.
- [ ] **Step 2: Laufen lassen** → FAIL. **Step 3:** implementieren → PASS (`cargo test -p driftless-app --test jobs`).
- [ ] **Step 4: Commit** `feat(app): Hintergrund-Jobs mit Fortschritt und Abbruch`

### Task 5: Wolke laden und Octree mit Detailstufen

**Files:**
- Create: `crates/driftless-app/src/cloud.rs`, `crates/driftless-app/src/lod.rs`, `crates/driftless-app/tests/lod_memory.rs` (eigener Test-Binary mit zählendem `#[global_allocator]`, wie `driftless-map/tests/memory.rs`)
- Test: `crates/driftless-app/tests/{cloud.rs, lod.rs, real_data.rs}`

**Interfaces:**
- Consumes: `SlamReport` (Teilkarten-Güte, `scan_start`), `Progress`.
- Produces:
  - `pub struct PointSet { pub origin: [f64; 3] /* Mitte der Hülle, ganze Meter */, pub xyz: Vec<[f32; 3]> /* relativ zu origin */, pub rgb: Option<Vec<[u8; 3]>>, pub intensity: Vec<u8>, pub t: Vec<f32> /* s ab Scan-Beginn, ohne Bericht ab erstem Punkt */, pub quality: Vec<u8> /* Teilkarten-Güte × 255, 255 ohne Bericht */, pub flags: Vec<u8> /* LAS user_data: 1 = ohne Farbe bzw. Entfernungsgrund */ }` mit `len()`, `bounds() -> ([f32; 3], [f32; 3])`.
  - `pub fn load_las(path: &Path, slam: Option<&SlamReport>, progress: &Progress) -> Result<PointSet, StepError>`: Stapel zu 2²⁰ Punkten, Phase „Wolke laden“ mit Gesamtzahl aus dem Header, Abbruch je Stapel; Güte je Punkt über `gps_time − scan_start` → Teilkarte (`partition_point` auf `t_start`, wie `trajectory_strips` im Viewer); Intensität LAS 0–65535 → `>> 8`.
  - `#[repr(C)] #[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)] pub struct PointVertex { pub pos: [f32; 3], pub rgbi: u32 /* r | g<<8 | b<<16 | intensity<<24 */, pub extra: u32 /* quality | flags<<8 */ }` (20 B; ohne Farbe rgb = Intensität grau).
  - `pub const LEAF_MAX: usize = 32_768; pub const NODE_GRID_BITS: u32 = 6; pub const CHUNK_POINTS: u32 = 4_194_304; pub const MAX_DEPTH: u32 = 15;`
  - `pub struct Node { pub min: [f32; 3], pub size: f32, pub spacing: f32 /* size / 64 */, pub first: u32, pub count: u32, pub children: [u32; 8] /* NO_NODE = u32::MAX */, pub depth: u8 }`, `pub struct Octree { pub origin: [f64; 3], pub nodes: Vec<Node> /* Breitensuche, 0 = Wurzel */, pub vertices: Vec<PointVertex> /* je Knoten zusammenhängend */, pub source: Vec<u32> /* Vertex → Punktindex in PointSet */, pub chunks: Vec<std::ops::Range<u32>> /* Vertexbereiche ≤ CHUNK_POINTS, nur an Knotengrenzen */ }`.
  - `pub fn build_octree(points: &PointSet, progress: &Progress) -> Result<Octree, StepError>` = `build_octree_with(points, CHUNK_POINTS, progress)`; `#[doc(hidden)] pub fn build_octree_with(points: &PointSet, chunk_points: u32, progress: &Progress) -> Result<Octree, StepError>` (kleine Blöcke für den Renderer-Test). Algorithmus (deterministisch, ohne Doppelung):

    ```text
    cube = kleinster Würfel um bounds (Kante ≥ 1 mm); Morton-Code 21 Bit je Achse
    sortiere (code, index) mit par_sort_unstable  -- Schlüssel eindeutig ⇒ thread-unabhängig
    node(d, Bereich):
      wenn len ≤ LEAF_MAX oder d == MAX_DEPTH: alle Punkte gehören dem Knoten (Blatt)
      sonst: Stichprobe = je Lauf gleichen Präfixes von 3·(d+6) Bit der erste Punkt
             (ein Punkt je Zelle eines 64³-Gitters des Knotenwürfels) → gehört dem Knoten;
             Rest bleibt sortiert, Kind k = Lauf mit Präfix 3·(d+1) Bit, rekursiv (rayon::join)
    Knoten in Breitensuche nummerieren, Vertices in dieser Reihenfolge, Blöcke an Knotengrenzen
    ```
  - In `lod.rs` (Task 6 erzeugt sie aus der Kamera): `pub struct Frustum { pub planes: [[f32; 4]; 5] /* nx, ny, nz, d; innen ≥ 0; fern offen */ }` mit `intersects_aabb(&self, min: [f32; 3], max: [f32; 3]) -> bool`; `pub struct ViewInfo { pub frustum: Frustum, pub eye: [f32; 3], pub near: f32, pub pixels_per_radian: f32, pub min_spacing_px: f32 }`.
  - `pub fn select_nodes(tree: &Octree, view: &ViewInfo, budget: usize) -> Vec<u32>`: Prioritätswarteschlange nach Bildschirmgröße `spacing · pixels_per_radian / max(Abstand Auge–Knotenmitte, near)`, Gleichstand → kleinere Knotennummer; nur Knoten im Sichtkörper; ein Knoten kommt nur dazu, wenn sein Elternknoten drin ist und noch ins Budget passt; Kinder nur, wenn die Bildschirmgröße des Elternknotens > `min_spacing_px`.
- [ ] **Step 1: Tests Laden** – `loads_color_las_from_export` (`driftless_map::write_las` einer kleinen Wolke → Koordinaten auf 0,1 mm, Farben, `flags` = `user_data`), `quality_from_submap_time` (Punkte mit `gps_time` in Teilkarte 0 und 2 → deren Güte), `loads_las_without_color` (Format 1: `rgb == None`), `far_coordinates_keep_mm_precision` (Versatz 512.345,678 m: relativ auf 1 mm genau), `load_cancelled_returns_cancelled`.
- [ ] **Step 2: Tests Octree** – `every_point_exactly_once` (`source` ist eine Permutation von `0..n`, 1 Mio. Zufallspunkte mit festem Startwert), `points_inside_their_node`, `inner_nodes_hold_one_point_per_cell`, `leaves_are_small` (≤ `LEAF_MAX` außer bei `MAX_DEPTH`), `chunks_respect_limit_and_node_boundaries`, `octree_is_deterministic` (zwei Läufe, einmal in `rayon::ThreadPool` mit 1 Thread: Vertices byte-gleich), `degenerate_clouds_build` (leer → nur Wurzel ohne Punkte; 1 Punkt; 100.000 gleiche Punkte → endet, alle Punkte genau einmal), `selection_respects_budget`, `selection_is_ancestor_closed`, `near_nodes_refined_first` (Kamera an einem Ende eines langen Flurs: Knoten nahe der Kamera tiefer als ferne), `large_budget_draws_everything` (alles im Bild, Budget ≥ n → Summe `count == n`), `outside_frustum_not_selected`.
- [ ] **Step 3: Speichertest** (`lod_memory.rs`, genau ein Test) – `octree_live_heap_per_point`: 2 Mio. Punkte, Spitze des zusätzlichen lebenden Speichers während `build_octree` ≤ 64 B × Punkte + 32 MB (Vertices 20 B, `source` 4 B, Sortierschlüssel 16 B, Puffer).
- [ ] **Step 4: Laufen lassen** → FAIL; **implementieren** → PASS (`cargo test -p driftless-app`).
- [ ] **Step 5: Echte Daten** (`real_data.rs`, `#[ignore]`) – `buero_scan_lod_build`: `$DRIFTLESS_TESTDATA/ergebnisse-m5/buero/karte/color.las` (Task 13 Step 1 erzeugt sie; fehlt sie, Test mit Hinweis beendet) laden und bauen; druckt Punkte, Ladezeit, Aufbauzeit, Knoten, Tiefe; Schranken setzt Task 13. `synthetic_50m_build` (50 Mio. Punkte aus festem Startwert, Räume aus Platten): druckt Aufbauzeit und Spitzen-RSS.
- [ ] **Step 6: Commit** `feat(app): LAS laden und Octree mit Detailstufen`

### Task 6: Kamera und Picking

**Files:**
- Create: `crates/driftless-app/src/camera.rs`, `crates/driftless-app/src/pick.rs`
- Test: `crates/driftless-app/tests/{camera.rs, pick.rs}`

**Interfaces:**
- Produces:
  - `pub enum CameraMode { Orbit, Fly }`; `pub struct Camera { pub mode: CameraMode, pub target: Vector3<f32>, pub yaw: f32, pub pitch: f32 /* ±89° */, pub distance: f32, pub eye: Vector3<f32> /* Fliegen */, pub fov_y: f32 /* 60° */, pub near: f32 /* 0,05 m */ }` (z oben); `Camera::default()`.
  - `eye_position(&self) -> Vector3<f32>`, `view(&self) -> Matrix4<f32>`, `proj(&self, aspect: f32) -> Matrix4<f32>` (Reverse-Z, unendlich fern: Tiefe 1 bei `near`, → 0 im Unendlichen), `view_proj(&self, aspect: f32) -> Matrix4<f32>`, `frustum(&self, aspect: f32) -> lod::Frustum`, `view_info(&self, viewport_px: [f32; 2], min_spacing_px: f32) -> ViewInfo` (`-> lod::ViewInfo`, `pixels_per_radian = viewport_px[1] / fov_y`).
  - Bedienung: `orbit(&mut self, dx_px: f32, dy_px: f32)` (0,3° je Pixel), `pan(&mut self, dx_px, dy_px, viewport_px: [f32; 2])` (Ziel wandert so, dass der Punkt im Ziel unter dem Mauszeiger bleibt), `zoom(&mut self, steps: f32)` (Faktor 0,85 je Schritt, ≥ 0,1 m), `look(&mut self, dx_px, dy_px)`, `fly(&mut self, forward: f32, right: f32, up: f32, dt: f32, speed: f32)`, `set_mode(&mut self, mode: CameraMode)` (Wechsel ohne Sprung des Blicks), `frame_bounds(&mut self, min: [f32; 3], max: [f32; 3])` (alles im Bild, Blick schräg von oben).
  - `pub struct Ray { pub origin: Vector3<f32>, pub dir: Vector3<f32> }`; `Camera::ray(&self, pixel_px: [f32; 2], viewport_px: [f32; 2]) -> Ray`. Alle Pixelangaben sind physische Pixel; die Oberfläche rechnet egui-Punkte mit `pixels_per_point` um.
  - `pub fn project(view_proj: &Matrix4<f32>, p: [f32; 3], viewport_px: [f32; 2]) -> Option<[f32; 3]>` (Pixel x, y, Tiefe; `None` hinter der Kamera).
  - `pub fn pick_point(tree: &Octree, drawn: &[u32], cam: &Camera, viewport_px: [f32; 2], pixel_px: [f32; 2], radius_px: f32) -> Option<u32>` (Vertexindex): unter allen gezeichneten Punkten im Radius der vorderste (größte Reverse-Z-Tiefe), Gleichstand → kleinster Index; Knoten vorher per Strahl-Kegel gegen ihre Hülle verworfen.
  - `pub fn pick_pose(positions: &[[f32; 3]], view_proj: &Matrix4<f32>, viewport_px: [f32; 2], pixel_px: [f32; 2], radius_px: f32) -> Option<usize>`: kleinster Bildschirmabstand, Gleichstand → vorderste, dann kleinster Index.
- [ ] **Step 1: Tests** – `orbit_keeps_target_in_centre` (Ziel projiziert auf Bildmitte ±0,5 px nach beliebigem `orbit`), `reverse_z_depth_range` (Tiefe bei `near` = 1, bei 1.000 m < 1e-4, monoton fallend), `ray_hits_projected_point` (Punkt projizieren, Strahl durch diesen Pixel: Abstand < 1 mm bei 10 m), `pan_keeps_point_under_cursor`, `zoom_never_passes_target`, `mode_switch_keeps_view` (Blickrichtung gleich auf 1e-5), `fly_moves_along_view`, `frame_bounds_shows_all_corners`, `frustum_culls_behind_and_beside`, `pitch_is_clamped`. Picking: `picks_front_point` (zwei Punkte auf einem Strahl: der nähere), `pick_outside_radius_is_none`, `pick_is_deterministic_on_ties`, `pick_pose_nearest_on_screen`, `pick_uses_physical_pixels` (Viewport 800×600 Punkte bei `pixels_per_point = 2`: Mausposition in Punkten × 2 trifft den Punkt, ohne Umrechnung nicht).
- [ ] **Step 2: Laufen lassen** → FAIL; **Step 3:** implementieren → PASS.
- [ ] **Step 4: Commit** `feat(app): Kamera und Picking`

### Task 7: wgpu-Renderer für Punkte und Linien

**Files:**
- Modify: `crates/driftless-app/Cargo.toml` (`egui-wgpu = "0.36"`, Zugriff auf wgpu nur über `egui_wgpu::wgpu`; `pollster = "0.4"` für den Test), `deny.toml` (Lizenzen siehe Entscheidungen)
- Create: `crates/driftless-app/src/render/mod.rs`, `crates/driftless-app/src/render/points.wgsl`, `crates/driftless-app/src/render/lines.wgsl`
- Test: `crates/driftless-app/tests/render.rs`

**Interfaces:**
- Consumes: `Octree`, `PointVertex`, `Octree::chunks` (Task 5), `Camera::view_proj` (Task 6).
- Produces:
  - `#[repr(u32)] pub enum ColorMode { Rgb = 0, Intensity = 1, Quality = 2 }` (`Rgb` ohne Farben = Intensität; Güte mit der Rampe rot → gelb → grün wie `quality_color` im Viewer).
  - `#[repr(C)] #[derive(Pod, Zeroable)] pub struct LineVertex { pub pos: [f32; 3], pub rgba: u32 }`.
  - `pub struct FrameParams { pub view_proj: [[f32; 4]; 4], pub viewport_px: [f32; 2], pub point_px: f32, pub color_mode: ColorMode, pub round: bool, pub background: [f32; 4] }`.
  - `pub struct RenderTarget { pub color: wgpu::Texture, pub depth: wgpu::Texture, pub size: [u32; 2] }`, `RenderTarget::new(device: &wgpu::Device, size: [u32; 2], format: wgpu::TextureFormat) -> Self` (Farbe `RENDER_ATTACHMENT | TEXTURE_BINDING | COPY_SRC`, Tiefe `Depth32Float`).
  - `pub struct PointRenderer`; `PointRenderer::new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self`; `upload_cloud(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, tree: &Octree)` (ein Vertexpuffer je Block; ersetzt die vorige Wolke); `set_overlay(&mut self, device: &wgpu::Device, lines: &[LineVertex], markers: &[PointVertex])`; `render(&self, device: &wgpu::Device, queue: &wgpu::Queue, target: &RenderTarget, frame: &FrameParams, tree: &Octree, drawn: &[u32])`.
  - Zeichnen: je Punkt eine Instanz (`step_mode: Instance`), 4 Ecken als Dreiecksstreifen, Versatz im Clip-Raum `point_px / viewport_px · 2 · w`; runde Punkte per `discard` außerhalb des Kreises; Tiefe Reverse-Z (`GreaterEqual`, löschen mit 0); ein Draw-Aufruf je ausgewähltem Knoten (`draw(0..4, first..first+count)` relativ zum Block). Linien `LineList` 1 px, ohne Tiefentest-Schreiben, über der Wolke mit Tiefentest; Marker wie Punkte mit `ColorMode::Rgb`.
  - `pub fn headless_device() -> Option<(wgpu::Device, wgpu::Queue, wgpu::AdapterInfo)>` (Backends `PRIMARY`, ohne Oberfläche, `pollster::block_on`; `None`, wenn kein Adapter).
- [ ] **Step 1: Abhängigkeiten** – `egui-wgpu` eintragen, `cargo deny check` → Lizenzmeldungen lesen; nur permissive Lizenzen mit Kommentar in `deny.toml` aufnehmen (`OFL-1.1`, `Ubuntu-font-1.0` über `epaint_default_fonts` spätestens in Task 8), sonst anhalten; `cargo deny check` grün.
- [ ] **Step 2: Tests** (alle beginnen mit `let Some((device, queue, info)) = headless_device() else { eprintln!("kein GPU-Adapter, Test übersprungen"); return; };` und schreiben `info.name`/`info.backend` ins Protokoll; Ziel 64 × 64 `Rgba8Unorm`, Rücklesen über `copy_texture_to_buffer`, 256 B je Zeile) – `renders_point_at_centre` (roter Punkt im Ziel, `point_px = 8`: Mitte R > 200, G, B < 50; Ecke = Hintergrund), `nearer_point_wins` (rot 2 m, blau 4 m auf derselben Achse, blau zuerst hochgeladen: Mitte rot), `intensity_mode_is_gray`, `quality_mode_uses_ramp` (Güte 0 → rot, 255 → grün), `nothing_outside_frustum` (Punkt hinter der Kamera: Bild = Hintergrund), `lines_draw_over_background`, `chunks_beyond_first_render` (`build_octree_with(points, 4, progress)` aus Task 5: Punkt im zweiten Block sichtbar).
- [ ] **Step 3: Laufen lassen** → FAIL (bzw. übersprungen ohne Adapter); **implementieren** → PASS lokal (Metal); auf CI grün (Windows meist über WARP, macOS-Runner ggf. übersprungen).
- [ ] **Step 4: Commit** `feat(app): wgpu-Renderer für Punktwolken und Linien`

### Task 8: Programmrahmen – Fenster, drei Bereiche, Projekt öffnen

**Files:**
- Modify: `crates/driftless-app/Cargo.toml` (`[[bin]] name = "driftless-app"`, `eframe = { version = "0.36", default-features = false, features = ["wgpu", "default_fonts", "accesskit", "persistence"] }`, `rfd = "0.17"`, `mimalloc = { version = "0.1", default-features = false }`; dev: `egui_kittest = "0.36"`), `deny.toml`
- Create: `crates/driftless-app/src/main.rs` (`#![windows_subsystem = "windows"]` außer in Debug, `mimalloc` als `#[global_allocator]`, Argumente `[projekt-ordner]` und `--synthetic <N>` per `std::env::args`), `crates/driftless-app/src/ui/{mod.rs, text.rs, left.rs, status.rs}`
- Test: `crates/driftless-app/tests/ui_left.rs`, `crates/driftless-app/tests/ui_status.rs`

**Interfaces:**
- Consumes: `Project`, `RunInfo`, `ScanState` (Task 3), `JobRunner` (Task 4).
- Produces:
  - `pub struct AppState { pub project: Option<Project>, pub selection: Selection, pub map_settings: MapSettings, pub slam_settings: SlamSettings, pub view: ViewSettings, pub messages: Vec<UiMessage>, pub job: Option<JobStatus>, pub log: Vec<LogLine>, … }` – alles, was die Panels zeigen; ohne GPU erzeugbar (`AppState::default()`).
  - `pub enum Selection { None, Scan(String), Run(PathBuf), Map(PathBuf), Cloud(PathBuf), History { run: PathBuf, nr: u32 } }`.
  - `pub enum Action { PickNewProject, PickOpenProject, OpenProject(PathBuf), NewProject { dir: PathBuf, name: String }, PickAddScan, AddScan(PathBuf), PickAddRun(String), Select(Selection), LoadCloud { path: PathBuf, run: Option<PathBuf> }, Inspect(String), RunSlam(String), Reuse(PathBuf), RunMap(PathBuf), Cancel, AddLoop { run: PathBuf, a: f64, b: f64, note: String }, SetLoopEnabled { run: PathBuf, nr: usize, enabled: bool }, Compare { a: TrajRef, b: TrajRef }, RestoreHistory { run: PathBuf, nr: u32 } }` (`TrajRef` aus Task 12; bis dahin ohne diese Variante).
  - Panels: `left::show(ui: &mut egui::Ui, state: &AppState) -> Option<Action>` (Baum Projekt → Scans → „SLAM 001“ … → Karten, Wolken, Verlauf; fehlender Scan grau mit Grund als Tooltip); `status::show(ui, state) -> Option<Action>` (unten: Phase, Fortschrittsbalken bzw. Drehsymbol ohne Gesamtzahl, Laufzeit, „Abbrechen“; aufklappbares Protokoll, Warnungen gelb). Rechts und Mitte folgen in Tasks 9–10.
  - `pub struct DriftlessApp` (`eframe::App`): führt `Action`s aus, öffnet Dialoge (`rfd::FileDialog`) nur hier, pollt die Jobs je Bild, `request_repaint_after(100 ms)` solange ein Job läuft; merkt sich das zuletzt geöffnete Projekt (eframe-Speicher). Fensterschließen bricht laufende Jobs ab (über `Drop` von `JobRunner`).
  - `ui/text.rs`: alle sichtbaren Texte als `pub const` (z. B. `NEW_PROJECT = "Neues Projekt …"`, `OPEN_PROJECT = "Projekt öffnen …"`, `ADD_SCAN = "Scan hinzufügen …"`, `ADD_RUN = "SLAM-Ordner hinzufügen …"`, `CANCEL = "Abbrechen"`, `CANCELLING = "Wird abgebrochen …"`, `PANEL_PROJECT = "Projekt"`, `PANEL_STEPS = "Schritte"`, `PANEL_PROPERTIES = "Eigenschaften"`).
- [ ] **Step 1: Tests** (`egui_kittest::Harness::new_ui`, Zugriff über die Beschriftungen) – `left_shows_scans_runs_maps` (Projekt mit 1 Scan, 2 Läufen, 1 Karte: alle Einträge da, richtige Reihenfolge), `clicking_run_selects_it` (→ `Action::Select(Selection::Run(..))`), `missing_scan_is_marked`, `no_project_offers_open_and_new`, `status_shows_progress_and_cancel` (`JobStatus` mit 40/100 → Text „40 / 100“, Klick „Abbrechen“ → `Action::Cancel`), `status_without_total_shows_spinner`, `cancelling_disables_button`.
- [ ] **Step 2: Laufen lassen** → FAIL; **Step 3:** implementieren → PASS; `cargo deny check` grün.
- [ ] **Step 4: Von Hand** – `cargo run -p driftless-app --release`: Fenster öffnet auf macOS (Metal), neues Projekt anlegen, Scan hinzufügen, Baum erscheint; Fenstergröße ändern ruckelt nicht.
- [ ] **Step 5: Commit** `feat(app): Programmfenster mit Projektbaum und Statusleiste`

### Task 9: Schritte und Eigenschaften (rechts)

**Files:**
- Create: `crates/driftless-app/src/ui/right.rs`, `crates/driftless-app/src/ui/actions.rs` (Ausführung der Schritt-Actions im `DriftlessApp`)
- Modify: `crates/driftless-app/src/ui/mod.rs`, `crates/driftless-app/src/ui/text.rs`
- Test: `crates/driftless-app/tests/ui_right.rs`, `crates/driftless-app/tests/actions.rs`

**Interfaces:**
- Consumes: `run_slam`, `run_map`, `inspect`, `MapSettings::check` (Task 2), `Project::new_run_dir`, `map_dir_for` (Task 3), `JobRunner::start` (Task 4).
- Produces:
  - `right::show(ui: &mut egui::Ui, state: &mut AppState) -> Option<Action>`: Bereich „Schritte“ je nach Auswahl – Scan: „Prüfen“ (`Inspect`), „SLAM rechnen“ (`RunSlam`, neuer Lauf-Ordner); Lauf: „Neu rechnen (aus Teilkarten)“ (`Reuse`, gesperrt ohne `submaps.bin` mit Grund „Kein Zwischenspeicher submaps.bin – zuerst SLAM rechnen“), „Karte rechnen“ (`RunMap`); jede Zeile mit Zustand „nicht gerechnet“ / „fertig (12,3 s)“ / „veraltet – Laufbahn neu gerechnet“ (aus `map.json`/`slam.json`/`folder_fits`). Bereich „Eigenschaften“: SLAM-Einstellungen (LiDAR-Lage 7 Zahlen, Topics; nur für neue Läufe änderbar, sonst angezeigt) und alle Schalter von `driftless map` als `MapSettings` – Schritt (Dichte Wolke / Bereinigen / Einfärben / Alles), „Geister entfernen“, „Ausreißer entfernen“, „Durchblick (Fenster) entfernen“, „Wände glätten“, „Glanzlichter abwerten (experimentell)“, „Randabfall der Kameras schätzen“, „Voxelgröße (m)“, „Bilder verkleinern 1/2/4“; ungültige Werte rot mit der Meldung aus `check`, „Karte rechnen“ dann gesperrt. Inspect-Ergebnis als Tabelle (Gerät, Dauer, Topics mit Anzahl/Rate, Probleme).
  - Alle Schritt-Buttons sind gesperrt, solange ein Verarbeitungs-Job läuft (Tooltip „Es läuft schon: …“); Ansehen, Auswählen und Laden bleiben möglich. Gesperrt ist auch alles, was den Scan braucht, bei `ScanState::Missing`.
  - `actions.rs`: `pub fn start_step(state: &mut AppState, runner: &mut JobRunner, action: &Action) -> Result<(), String>` startet den passenden Job (`SlamRequest { history: true, .. }`, `MapRequest { out_dir: map_dir_for(..) }`); nach Ende lädt der App-Rahmen das Projekt neu (`Project::open`), wählt den neuen Lauf/die neue Karte aus und lädt deren beste Wolke (Task 10).
- [ ] **Step 1: Tests** – UI: `scan_offers_inspect_and_slam`, `reuse_disabled_without_cache_with_reason`, `steps_disabled_while_busy`, `map_settings_edit_all_switches` (jeden Schalter umlegen → `state.map_settings` entsprechend; Standard = `MapSettings::default()`), `invalid_voxel_blocks_map` (Voxel 0 → Meldung „--voxel: Voxelgröße muss größer als 0 sein“ sichtbar, Button gesperrt), `stale_map_is_marked`. Actions (ohne Fenster, synthetische Szene aus `testscan`): `slam_action_creates_new_run_dir` (zweimal → `slam-001`, `slam-002`, beide vollständig, erster unverändert), `reuse_action_records_history`, `map_action_with_new_settings_creates_new_map_dir`.
- [ ] **Step 2: Laufen lassen** → FAIL; **Step 3:** implementieren → PASS.
- [ ] **Step 4: Commit** `feat(app): Schritte starten und Karten-Einstellungen`

### Task 10: 3D-Ansicht

**Files:**
- Create: `crates/driftless-app/src/ui/view3d.rs`, `crates/driftless-app/src/synthetic.rs` (Testwolke für `--synthetic N`: Räume aus Platten, fester Startwert)
- Modify: `crates/driftless-app/src/ui/mod.rs`, `crates/driftless-app/src/ui/right.rs` (Ansichts-Eigenschaften, Punktinfo), `crates/driftless-app/src/jobs.rs` (`JobOutput::Cloud(Box<LoadedCloud>)`)
- Test: `crates/driftless-app/tests/view3d.rs`, `crates/driftless-app/tests/synthetic.rs`

**Interfaces:**
- Consumes: `load_las`, `build_octree`, `select_nodes` (Task 5), `Camera`, `pick_point` (Task 6), `PointRenderer`, `RenderTarget` (Task 7).
- Produces:
  - `pub struct ViewSettings { pub color_mode: ColorMode, pub point_px: f32 /* 2,0 */, pub round: bool, pub budget: usize /* Start 6.000.000, 1–30 Mio. */, pub camera_mode: CameraMode, pub show_trajectory: bool, pub show_odometry: bool, pub show_loops: bool, pub show_submaps: bool }`.
  - `pub struct LoadedCloud { pub path: PathBuf, pub points: PointSet, pub tree: Octree, pub load_seconds: f64, pub build_seconds: f64 }`; Laden + Aufbau in einem Job auf dem Lade-Läufer (Phase „Wolke laden“, dann „Detailstufen bauen“); eine neue Wolke ersetzt die alte erst, wenn sie fertig ist.
  - `pub struct View3d { camera: Camera, target: Option<RenderTarget>, texture: Option<egui::TextureId>, drawn: Vec<u32>, frame_ms: f32, … }`; `View3d::show(&mut self, ui: &mut egui::Ui, render_state: &egui_wgpu::RenderState, cloud: Option<&LoadedCloud>, settings: &ViewSettings) -> Option<ViewEvent>`: Zielgröße = Panelgröße × `pixels_per_point`, bei Größenänderung neu angelegt und über `Renderer::register_native_texture`/`update_egui_texture_from_wgpu_texture` gezeigt; je Bild `select_nodes` → `render` → `ui.image`. Bedienung: Umkreisen linke Maustaste, Verschieben rechte/mittlere Taste oder Shift, Zoom Mausrad; Fliegen: WASD + Q/E, Maus mit rechter Taste schaut, Shift schneller; Taste F bzw. Doppelklick ins Leere = alles zeigen; Klick ohne Ziehen = Punkt anklicken.
  - `pub enum ViewEvent { PickedPoint(u32), PickedPose(usize) }`; Punktinfo in „Eigenschaften“: Koordinaten (absolut, `origin` + relativ, mm), Zeit ab Scan-Beginn, Intensität, Farbe bzw. „ohne Farbe“, Güte der Teilkarte, bei `removed.las` der Grund (Geist / Ausreißer / Durchblick).
  - Ansichts-Eigenschaften: Farbe / Intensität / Güte, Punktgröße, rund, Punktbudget, Umkreisen / Fliegen, Ebenen ein/aus; Anzeige „gezeichnet 5,9 Mio. von 15,6 Mio. Punkten · 8,4 ms“.
  - `pub fn synthetic_cloud(n: usize, seed: u64) -> PointSet`.
- [ ] **Step 1: Tests** – `view_target_size_uses_pixels_per_point` (Funktion `target_size(panel_points: egui::Vec2, ppp: f32) -> [u32; 2]`: 400×300 bei 2 → 800×600, nie 0), `click_vs_drag_threshold` (`is_click(press, release, moved_px)`: < 4 px und < 300 ms = Klick), `fly_keys_map_to_motion`, `color_mode_falls_back_without_rgb` (Wolke ohne Farben → angebotene Modi Intensität/Güte, `Rgb` wählt Intensität), `pick_info_formats_absolute_coordinates` (Ursprung 512.000 m: Anzeige „512.345,678 m“ mit deutschen Trennzeichen), `removed_reason_text` (1 → „Geist“, 2 → „Ausreißer“, 3 → „Durchblick“), `synthetic_cloud_is_deterministic`.
- [ ] **Step 2: Laufen lassen** → FAIL; **Step 3:** implementieren → PASS.
- [ ] **Step 4: Von Hand** – `cargo run -p driftless-app --release -- --synthetic 50000000`: Laden blockiert die Oberfläche nicht (Fenster bleibt bedienbar, Fortschritt läuft), Drehen flüssig, Bildzeit-Anzeige notieren (Mac); Klick auf einen Punkt zeigt seine Daten.
- [ ] **Step 5: Commit** `feat(app): 3D-Ansicht mit Detailstufen, Farbmodi und Punktinfo`

### Task 11: Laufbahn, Schleifen, Teilkarten und manuelle Korrektur

**Files:**
- Create: `crates/driftless-app/src/overlay.rs`, `crates/driftless-app/src/ui/correct.rs`
- Modify: `crates/driftless-app/src/ui/{mod.rs, right.rs, view3d.rs, text.rs}`
- Test: `crates/driftless-app/tests/overlay.rs`, `crates/driftless-app/tests/ui_correct.rs`

**Interfaces:**
- Consumes: `SlamReport`, `read_tum` (Lauf-Ordner), `correct::{load, add_loop, set_enabled}` (Task 2), `pick_pose` (Task 6), `PointRenderer::set_overlay` (Task 7), `Action::Reuse` (Task 9).
- Produces:
  - `pub struct RunOverlay { pub scan_start: f64, pub poses: Vec<Pose>, pub odometry: Vec<Pose>, pub positions: Vec<[f32; 3]> /* relativ zum Wolkenursprung */, pub report: SlamReport }`; `RunOverlay::load(run: &Path, origin: [f64; 3]) -> Result<RunOverlay, StepError>`; `lines(&self, settings: &ViewSettings, picks: &[usize]) -> (Vec<LineVertex>, Vec<PointVertex>)`: Laufbahn je Teilkarte in deren Gütefarbe (gleiche Zuordnung und Rampe wie `trajectory_strips`/`quality_color` im Viewer), Odometrie grau, Schleifen zwischen den Ankern (akzeptiert grün, verworfen grau, manuell blau – Farben wie `LOOP_*` im Viewer), Anker als Marker, angeklickte Stellen als große weiße Marker; `time_of(&self, pose: usize) -> f64` = `poses[pose].t − scan_start`.
  - Ohne geladene Wolke ist der Ursprung der erste Pose-Punkt (ganze Meter); Laufbahn ist auch ohne Wolke sichtbar.
  - Korrektur-Panel (`correct::show(ui, state) -> Option<Action>`, rechts bei Auswahl eines Laufs): „Stelle A“ / „Stelle B“ (Zeit in s, per Klick auf die Laufbahn gesetzt – Klickmodus „Stellen wählen“ – oder von Hand eintippbar, eine Nachkommastelle angezeigt), Notiz, „Schleife eintragen“ → `Action::AddLoop`; Liste aller Einträge wie `driftless correct list` (Nr, Status, a, b, hinzugefügt, Notiz) mit Schalter je Eintrag → `Action::SetLoopEnabled`; danach Hinweis „Geändert – jetzt neu rechnen“ mit Button → `Action::Reuse`. Meldungen aus `add_loop`/`set_enabled` erscheinen unverändert (deutsch) im Panel.
- [ ] **Step 1: Tests** – Overlay: `strips_follow_submaps_and_quality` (3 Teilkarten, Güte 0/0,5/1 → Farben rot/gelb/grün, Streifen ohne Lücke), `loops_colored_by_kind`, `time_of_pick_is_since_scan_start`, `overlay_without_cloud_uses_first_pose_origin`. Korrektur (synthetische Szene mit `submaps.bin`): `pick_two_places_adds_loop` (zwei `PickedPose` → Felder gesetzt → Klick „Schleife eintragen“ → `Action::AddLoop` mit diesen Zeiten; ausgeführt → `corrections.toml` hat den Eintrag, `driftless correct <lauf> list` (Binary, `assert_cmd` als dev-dep) zeigt ihn), `invalid_loop_shows_error_and_writes_nothing` (beide Stellen in derselben Teilkarte → Meldung der Prüfung sichtbar, Datei byte-gleich wie vorher), `toggle_entry_and_reuse` (Eintrag aus → Hinweis + `Action::Reuse`), `cli_written_corrections_are_listed` (mit `driftless correct … loop` angelegt → im Panel gleich).
- [ ] **Step 2: Laufen lassen** → FAIL; **Step 3:** implementieren → PASS.
- [ ] **Step 4: Commit** `feat(app): Laufbahn nach Güte und Schleifen per Klick`

### Task 12: Vorher/Nachher und Verlauf

**Files:**
- Create: `crates/driftless-app/src/ui/compare.rs`
- Modify: `crates/driftless-app/src/overlay.rs` (zweite Laufbahn), `crates/driftless-app/src/ui/{mod.rs, left.rs, right.rs}`, `crates/driftless-app/src/ui/actions.rs`
- Test: `crates/driftless-app/tests/compare.rs`

**Interfaces:**
- Consumes: `compare::compare` (Task 2), `history::{list, restore_flags}` (Task 2), `RunInfo::history`, `has_previous` (Task 3), `RunOverlay` (Task 11).
- Produces:
  - `pub enum TrajRef { Current(PathBuf), Odometry(PathBuf), Previous(PathBuf), History { run: PathBuf, nr: u32 } }` mit `path(&self) -> PathBuf` (`traj.tum`, `traj_odom.tum`, `traj.vorher.tum`, `verlauf/NNN/traj.tum`) und `label(&self) -> String` („SLAM 001 aktuell“, „… Odometrie“, „… vorher“, „… Stand 003“).
  - `pub struct Comparison { pub a: TrajRef, pub b: TrajRef, pub report: compare::Report }`; `pub fn compare_refs(a: &TrajRef, b: &TrajRef) -> Result<Comparison, StepError>` (`max_dt` 0,01 s wie das CLI; a wird an b ausgerichtet).
  - Vergleichs-Panel: Auswahl A und B aus allen Laufbahnen des Scans (alle Läufe, deren Odometrie, `vorher`, Verlaufsstände); Voreinstellung nach einem Neu-Rechnen: A = vorher, B = aktuell, sonst A = Odometrie, B = aktuell. Tabelle „Abweichung (ATE, RMSE)“, „Start↔Ende A/B“, „Länge A/B“, „Zugeordnete Posen“ (Werte in m, drei Nachkommastellen, wie `driftless compare`); Laufbahn A in der 3D-Ansicht orange überlagert (ausgerichtet). Läuft im Lade-Läufer (Rechnen < 1 s, blockiert trotzdem nicht).
  - Karten vergleichen: zwei Kartenordner eines Scans wählen → Einstellungen und Kennzahlen aus `map.json` nebeneinander (Punkte je Schritt, entfernte Punkte je Grund, Wanddicke vorher/nachher, Kamera-Unterschied, Nahtsprung, Laufzeiten), abweichende Werte hervorgehoben; `pub fn map_rows(a: &serde_json::Value, b: &serde_json::Value) -> Vec<(String, String, String)>` (Bezeichnung deutsch, Wert A, Wert B; fehlende Werte „–“).
  - Verlauf im Baum (je Stand Nr, Art, Zeit, Start↔Ende aus `slam.json`); bei Auswahl „Vergleichen mit aktuell“ und „Diesen Stand wiederherstellen“ → `Action::RestoreHistory` = `restore_flags` + `Reuse` (neuer Stand; alte bleiben).
- [ ] **Step 1: Tests** – `previous_vs_current_after_reuse` (synthetische Szene: Lauf, Schleife, Reuse → Voreinstellung vorher/aktuell, Werte = `driftless compare traj.vorher.tum traj.tum --json` des Binaries), `odometry_vs_current_default_without_previous`, `history_refs_resolve`, `restore_history_creates_new_state` (Stand 001 wiederherstellen → Korrekturschalter wie in 001, neuer Stand 003 mit `traj.tum` byte-gleich zu 001), `compare_mismatched_trajectories_reports_error` (keine gemeinsamen Zeiten → deutsche Meldung „zu wenige zugeordnete Posen …“ im Panel, kein Absturz), `map_rows_side_by_side` (zwei `map.json` mit anderem `--no-smooth` → Zeile „Wände glätten“ an/aus, fehlender Schritt „–“).
- [ ] **Step 2: Laufen lassen** → FAIL; **Step 3:** implementieren → PASS.
- [ ] **Step 4: Commit** `feat(app): Vorher/Nachher-Vergleich und Verlauf`

### Task 13: Prüfung mit echten Daten, Messung, Doku

**Files:**
- Modify: `crates/driftless-app/tests/real_data.rs` (`#[ignore]`), `README.md` (Abschnitt „Desktop program“ mit Start, Bedienung, Projektordner; Status-Zeile), `docs/testdaten.md` (Ordner `ergebnisse-m5/`)
- Create: `docs/ergebnisse/YYYY-MM-DD-meilenstein-5.md` (Datum des Messtags)

- [ ] **Step 1: Daten vorbereiten** – Büro-Scan (20261020173822) mit dem Release-CLI: `driftless slam <scan> -o $DRIFTLESS_TESTDATA/ergebnisse-m5/buero` und `driftless map <scan> --slam …/buero` (dann `…/buero/color.las` nach `…/buero/karte/` kopieren, wie in Task 5 erwartet); `Test` ebenso nach `…/ergebnisse-m5/test`.
- [ ] **Step 2: Messen** (M4 Pro; Windows-PC mit RTX 5080) – `cargo test -p driftless-app --release -- --ignored --nocapture`: Ladezeit und LOD-Aufbauzeit der Büro-Wolke (≈ 15,6 Mio. Punkte), Spitzen-RSS (`/usr/bin/time -l` bzw. Task-Manager) der App nach dem Laden; 50 Mio. synthetisch: Aufbauzeit, RSS, Bildzeit beim Drehen bei Budget 6 Mio. und 12 Mio.; Neu-Rechnen nach einer Schleife auf dem Büro-Scan in der App (Zeit vom Klick bis zur neuen Laufbahn); `slam` und `map` aus der App gegen das CLI: Dateien byte-gleich (`cmp`, ohne Laufzeitfelder in `map.json`).
- [ ] **Step 3: Schranken als Tests** – `buero_scan_lod_build`: Aufbau ≤ 10 s, zusätzlicher Speicher ≤ 64 B/Punkt (gemessen + Spielraum, Werte im Ergebnisdokument begründet); `buero_scan_reuse_in_seconds`: `run_slam` mit `reuse: true, history: true` ≤ 15 s; `app_steps_match_cli` (Test-Scan: Ausgaben aus `run_slam`/`run_map` = CLI-Ausgaben). Reicht die Bildzeit bei 50 Mio. nicht für ≥ 30 Bilder/s, Standardbudget senken (Wert und Grund ins Ergebnis); reicht die Aufbauzeit nicht, LOD-Cache als offenen Punkt für M6 notieren, nicht in M5 bauen.
- [ ] **Step 4: Ablauf von Hand auf beiden Plattformen** – Projekt anlegen, Büro-Scan hinzufügen, Prüfen, SLAM rechnen (Abbrechen ausprobieren: keine Dateien), Karte rechnen, Wolke in allen drei Farbmodi, Umkreisen/Fliegen, Punkt anklicken, zwei Stellen auf der Laufbahn als Schleife eintragen, neu rechnen, Vorher/Nachher ansehen, Eintrag abschalten, Stand wiederherstellen; Retina und Windows 150 %: Klick trifft. Bildschirmfotos nur lokal unter `$DRIFTLESS_TESTDATA/ergebnisse-m5/`.
- [ ] **Step 5: Ergebnisdokument** (Deutsch, Tabellen: Zeiten, Speicher, Bildzeiten je Plattform/GPU, Abweichungen; ehrliche Grenzen: kein LOD-Cache, ein Job zur Zeit, Linien 1 px, Export erst M6) und README.
- [ ] **Step 6: Gesamtprüfung** – `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo test --workspace`, `cargo deny check`, `cargo test --release -- --ignored`; CI auf `macos-14` und `windows-latest` grün.
- [ ] **Step 7: Commit** `test(app): Prüfung mit echten Daten, Ergebnisse Meilenstein 5`
