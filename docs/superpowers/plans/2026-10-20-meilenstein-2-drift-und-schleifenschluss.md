# Meilenstein 2 – Drift sichtbar machen, Schleifenschluss automatisch und von Hand – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Aus der Odometrie von Meilenstein 1 wird ein SLAM: Teilkarten, Pose-Graph mit automatischem Schleifenschluss, Güte je Pose, manuelle Korrekturen („diese zwei Stellen sind gleich“) als rücknehmbare Einträge, die in Sekunden neu gerechnet werden.

**Architecture:** `driftless-odom` liefert je Frame Pose, Güte und entzerrte Punkte über eine Senke. Neu: `driftless-graph` (SE(3)-Pose-Graph, Levenberg-Marquardt, robuster Kern, dünn besetzte Cholesky) und `driftless-slam` (Teilkarten, Schleifensuche + ICP-Prüfung, Korrekturen, Zwischenspeicher, `run_slam`). CLI bekommt `slam` und `correct`; der Viewer zeigt Güte und Schleifen.

**Tech Stack:** wie Meilenstein 1, dazu `nalgebra-sparse` (Apache-2.0, `CscCholesky`), `serde` + `toml` (Korrekturdatei), `serde_json` (Bericht).

**Spec:** `docs/superpowers/specs/2026-10-20-driftless-design.md` (Ziel 2 „Drift abfangen“, Abschnitt 6 „Antwortzeit Korrekturen“). Entscheidung: `docs/entscheidungen/0001-slam-basis.md` (eigener Rust-Kern).

## Global Constraints

- Lizenz Apache-2.0 OR MIT; nur MIT/Apache/BSD-Abhängigkeiten, `cargo deny check` grün.
- macOS arm64 und Windows x64 nativ, CI (`.github/workflows/ci.yml`) grün auf beiden; `rust-version = "1.99"`.
- Deterministisch: zwei Läufe auf denselben Daten liefern byte-gleiche `traj.tum` (rayon nur mit fester Reduktionsreihenfolge, keine HashMap-Iteration mit Zufalls-Hasher in Ergebnispfaden; `FixedState` aus `voxel_map` verwenden).
- Spec: „Nichts wird überschrieben, jede Korrektur ist ein rücknehmbarer Schritt.“ Korrekturen werden nur angehängt oder deaktiviert, nie gelöscht; Rohdaten werden nie verändert.
- Spec: „manueller Schleifenschluss aktualisiert die Laufbahn in Sekunden (Pose-Graph über gespeicherte Teilkarten, kein kompletter SLAM-Neulauf)“ → Neurechnen aus dem Zwischenspeicher < 10 s für den JMK6-Testscan (266 s).
- Spec: „Ganze Aufnahmen dürfen nicht komplett in den RAM müssen.“ Teilkarten speichern nur ausgedünnte Punkte (`OUTPUT_MAP_VOXEL` = 0,1 m), keine Roh-Frames.
- Code-Kommentare Englisch, Meldungen für Nutzer Deutsch (wie Meilenstein 1). Scan-Daten nie ins Repo; Tests mit echten Daten `#[ignore]`.

## Review Focus

- Scan ohne Rundgang (Hilti exp14, JMK6-Test) → keine Schleife erfunden, Ergebnis nicht schlechter als Odometrie, Meldung „keine Schleife gefunden“ (Task 4, Task 8).
- Wiederkehrende Struktur (Flur mit gleichen Säulen) → falscher Schleifenkandidat wird verworfen oder vom robusten Kern entschärft (Task 3, Task 4).
- Manuelle Schleife mit Zeiten außerhalb des Scans, in derselben Teilkarte oder doppelt eingetragen → klare deutsche Fehlermeldung, kein Absturz (Task 6).
- Zwischenspeicher alt, abgeschnitten oder zu anderem Scan → Fehler mit Hinweis „`driftless slam` ohne `--reuse` neu laufen lassen“, kein Panic (Task 6).
- Sehr kurzer Scan (eine Teilkarte) → läuft durch, Graph wird übersprungen, Ausgabe = Odometrie (Task 5).

---

### Task 1: Güte je Frame in der Odometrie und Frame-Senke

**Files:**
- Modify: `crates/driftless-odom/src/icp.rs` (`IcpResult`), `crates/driftless-odom/src/pipeline.rs`, `crates/driftless-odom/src/lib.rs`
- Create: `crates/driftless-odom/src/quality.rs`
- Test: `crates/driftless-odom/tests/quality.rs`, bestehende `tests/pipeline.rs` bleiben grün

**Interfaces:**
- Produces:
  - `IcpResult` neu: `residual_rms: f64` (gewichteter Punkt-zu-Ebene-Rest der letzten Iteration, m), `inlier_ratio: f64` (Korrespondenzen / Quellpunkte), `constraint: [f64; 6]` (Eigenwerte der Hesse-Matrix ohne Prior, aufsteigend, je durch Korrespondenzzahl geteilt).
  - `pub struct FrameQuality { pub residual_rms: f64, pub inlier_ratio: f64, pub degeneracy: f64, pub icp_ok: bool }` mit `pub fn score(&self) -> f32` in `[0, 1]` (1 = gut). `degeneracy` = kleinster Eigenwert des 3×3-Translationsblocks der Hesse-Matrix / Korrespondenzzahl (wie schlecht ist die schwächste Richtung festgelegt). Erster Frame: `FrameQuality::first()` mit Score 1.
  - `pub struct FrameOutput<'a> { pub index: usize, pub pose: Pose, pub quality: FrameQuality, pub body_points: &'a [Vector3<f64>] }` (entzerrt, Body-Rahmen, vor Ausdünnung).
  - `pub fn run_odometry_with(frames, imu, cfg, sink: &mut dyn FnMut(FrameOutput)) -> OdomResult`; `run_odometry` ruft sie mit leerer Senke auf. `OdomResult` neu: `quality: Vec<FrameQuality>` (gleich lang wie `poses`).

- [ ] **Step 1: Tests schreiben** – `quality_room_is_well_constrained`: Walk in `room_scene()` → Median `degeneracy` > 0,05 und Median `score()` > 0,8. `quality_flags_featureless_corridor`: Walk in einem Flur ohne Säulen (`Scene { room: aabb([-3,-1.6,-1.3],[60,1.6,1.7]), boxes: vec![] }`) → Median `degeneracy` < 0,2 × Raumwert und Median `score()` < 0,5. `sink_sees_every_used_frame`: Senke wird genau `poses.len()`-mal mit aufsteigendem `index` aufgerufen, Posen gleich `OdomResult::poses`. `quality_len_matches_poses`.
- [ ] **Step 2: Tests laufen lassen** – `cargo test -p driftless-odom --test quality` → FAIL (Typen fehlen).
- [ ] **Step 3: Implementieren.** `score()`: Produkt aus vier Faktoren in [0,1] – `icp_ok` (sonst 0), `clamp(inlier_ratio / 0.6)`, `clamp(degeneracy / DEGENERACY_GOOD)` und `clamp(1 − residual_rms / 0.1)`; Konstante `DEGENERACY_GOOD` aus dem Raumtest ableiten und dokumentieren. Hesse-Eigenwerte ohne den Prior-Anteil berechnen.
- [ ] **Step 4: Alle Odometrie-Tests grün**, `odometry_is_deterministic` eingeschlossen – `cargo test -p driftless-odom`.
- [ ] **Step 5: Commit** `feat(odom): Güte je Frame und Frame-Senke`

### Task 2: Pose-Graph (`driftless-graph`)

**Files:**
- Create: `crates/driftless-graph/` (`Cargo.toml`, `src/lib.rs`, `src/se3.rs`, `src/optimize.rs`), Workspace-`Cargo.toml` ergänzen
- Test: `crates/driftless-graph/tests/graph.rs`

**Interfaces:**
- Produces:
  - `pub struct PoseGraph { pub nodes: Vec<Isometry3<f64>>, pub edges: Vec<Edge>, pub fixed: usize }` (Knoten `fixed` bleibt fest, Standard 0).
  - `pub struct Edge { pub from: usize, pub to: usize, pub measurement: Isometry3<f64>, pub information: Matrix6<f64>, pub robust: Option<f64> }` – `measurement` = `T_from⁻¹ · T_to`; Fehler `e = [rot(Z⁻¹·T_from⁻¹·T_to).scaled_axis; trans]` (Reihenfolge rot, trans wie `icp.rs`); `robust = Some(c)` → Cauchy-Kern mit Breite `c` auf `sqrt(eᵀΩe)`.
  - `pub struct OptimizeReport { pub iterations: u32, pub initial_cost: f64, pub final_cost: f64, pub converged: bool, pub edge_cost: Vec<f64> }` (Kosten je Kante nach der Optimierung, für den Bericht).
  - `pub fn optimize(graph: &mut PoseGraph, max_iter: u32) -> OptimizeReport` – Levenberg-Marquardt, Störung rechts (`T · Exp(δ)`), Jacobis analytisch oder zentrale Differenzen (Schritt 1e-6), Normalgleichung als dünne Matrix, `nalgebra_sparse::factorization::CscCholesky`. Feste Summationsreihenfolge (Kanten in Listenreihenfolge).

- [ ] **Step 1: Tests schreiben** – `chain_without_loop_stays_put` (Kette aus 20 Knoten mit exakten Messungen: Kosten 0, Knoten unverändert ≤ 1e-9). `loop_closure_removes_injected_drift`: Rechteck 4 × 10 m, 40 Knoten, Odometrie-Messungen mit 0,5°/Knoten Gierfehler, eine Schleifenkante letzter→erster mit wahrer Messung und 100-facher Information → Positionsfehler aller Knoten nach Optimierung < 10 % des Fehlers davor, letzter Knoten < 0,05 m von Wahrheit. `cauchy_limits_false_loop`: wie vorher plus eine falsche Schleifenkante (Knoten 10 ↔ 30, Messung um 3 m verschoben, `robust: Some(1.0)`) → maximaler Positionsfehler < 0,3 m; dieselbe Kante ohne robusten Kern verschlechtert deutlich (Fehler > 1 m) – belegt, dass der Kern wirkt. `optimize_is_deterministic` (zweimal → bitgleich). `single_node_graph_is_noop`.
- [ ] **Step 2: FAIL prüfen** – `cargo test -p driftless-graph`.
- [ ] **Step 3: Implementieren**, `cargo deny check` nach Aufnahme von `nalgebra-sparse`.
- [ ] **Step 4: PASS** – `cargo test -p driftless-graph`, `cargo clippy --workspace -- -D warnings`.
- [ ] **Step 5: Commit** `feat(graph): SE(3)-Pose-Graph mit Levenberg-Marquardt und Cauchy-Kern`

### Task 3: Teilkarten (`driftless-slam`, Teil 1)

**Files:**
- Create: `crates/driftless-slam/` (`Cargo.toml`, `src/lib.rs`, `src/submap.rs`), Workspace ergänzen
- Create: `crates/driftless-slam/tests/common/mod.rs` – synthetische Teilkarten aus `Scene::surface_samples` (Szenen-/Trajektorien-Helfer aus `driftless-odom/tests/common` hierher kopieren oder als `test-util`-Feature von `driftless-odom` bereitstellen; Entscheidung Implementierer, im Bericht nennen)
- Test: `crates/driftless-slam/tests/submap.rs`

**Interfaces:**
- Consumes: `FrameOutput`, `FrameQuality`, `voxel_downsample`, `OUTPUT_MAP_VOXEL`.
- Produces:
  - `pub struct Submap { pub id: usize, pub anchor: Isometry3<f64>, pub frames: Vec<SubmapFrame>, pub points: Vec<[f32; 3]> }` – `anchor` = Odometrie-Pose des ersten Frames; `points` im Anker-Rahmen, ausgedünnt auf `OUTPUT_MAP_VOXEL`.
  - `pub struct SubmapFrame { pub index: usize, pub t: f64, pub offset: Isometry3<f64>, pub quality: FrameQuality }` (`offset` = `anchor⁻¹ · pose`).
  - `pub struct SubmapConfig { pub max_length: f64 /* 5.0 m */, pub max_frames: usize /* 100 */ }`.
  - `pub struct SubmapBuilder` mit `new(cfg)`, `push(&mut self, f: FrameOutput)`, `finish(self) -> Vec<Submap>`. Neue Teilkarte, sobald Weglänge seit Anker ≥ `max_length` oder `max_frames` erreicht. Letzter Frame einer Teilkarte ist zugleich nicht Teil der nächsten (keine Doppelung).
  - `impl Submap { pub fn quality(&self) -> f32 /* Minimum der Frame-Scores */; pub fn world_points(&self, anchor: &Isometry3<f64>) -> impl Iterator<Item = [f32; 3]> }`.

- [ ] **Step 1: Tests** – `splits_by_length` (12 m gerade Strecke, 10 Hz, 1 m/s → 3 Teilkarten, Frames lückenlos und ohne Doppelung). `splits_by_frame_count` (Stillstand 15 s → `ceil(150/100)` Teilkarten). `points_round_trip` (Punkt aus `body_points` → Welt über Frame-Pose == `world_points(anchor)` bis 0,1 m Voxelraster). `quality_is_minimum`.
- [ ] **Step 2: FAIL**, **Step 3: implementieren**, **Step 4: PASS** – `cargo test -p driftless-slam --test submap`.
- [ ] **Step 5: Commit** `feat(slam): Teilkarten aus der Odometrie`

### Task 4: Schleifensuche und -prüfung

**Files:**
- Create: `crates/driftless-slam/src/loops.rs`
- Test: `crates/driftless-slam/tests/loops.rs`

**Interfaces:**
- Consumes: `Submap`, `driftless_odom::icp::{align, IcpTarget, IcpParams}`.
- Produces:
  - `pub struct LoopConfig { pub min_gap: usize /* 3 Teilkarten */, pub base_radius: f64 /* 3.0 m */, pub radius_per_m: f64 /* 0.05 */, pub max_candidates: usize /* 3 je Teilkarte */, pub icp_voxel: f64 /* 0.25 m */, pub min_inlier_ratio: f64 /* 0.5 */, pub max_rms: f64 /* 0.05 m */, pub min_degeneracy: f64 }` mit `Default`.
  - `pub struct LoopCandidate { pub a: usize, pub b: usize }` (`a < b`, Teilkarten-ids).
  - `pub fn find_candidates(submaps: &[Submap], anchors: &[Isometry3<f64>], cfg: &LoopConfig) -> Vec<LoopCandidate>` – Paare mit `b − a ≥ min_gap`, Ankerabstand < `base_radius + radius_per_m × Weglänge zwischen a und b`; je `b` höchstens `max_candidates` nächste; Ausgabe sortiert nach `(b, a)`.
  - `pub enum LoopVerdict { Accepted, Rejected(&'static str) }` (Grund deutsch, z. B. „zu wenig Überlappung“).
  - `pub struct LoopCheck { pub a: usize, pub b: usize, pub relative: Isometry3<f64>, pub inlier_ratio: f64, pub rms: f64, pub degeneracy: f64, pub verdict: LoopVerdict, pub manual: bool }`.
  - `pub fn verify(submaps: &[Submap], anchors: &[Isometry3<f64>], c: &LoopCandidate, cfg: &LoopConfig) -> LoopCheck` – ICP der Punkte von `b` (ausgedünnt auf `icp_voxel`) gegen `a`, Start = `anchors[a]⁻¹ · anchors[b]`, grob-zu-fein (`max_corr_dist` 1,0 → 0,5 → 0,25 m), schwacher Prior. Gegenprüfung: Ergebnis darf sich mit vertauschten Rollen (a gegen b) um höchstens 0,05 m / 1° unterscheiden, sonst `Rejected("nicht eindeutig")`.

- [ ] **Step 1: Tests** (synthetisch, Teilkarten direkt aus Oberflächenproben, keine Odometrie) – `finds_loop_after_rectangle` (Rundgang um zwei Kisten in `room_scene`, Anker mit eingespielter Drift 0,4 m am Ende → Kandidat erster↔letzter wird gefunden und akzeptiert; `relative` auf 0,03 m / 0,5° genau). `no_candidates_on_straight_walk`. `rejects_repetitive_corridor`: zwei Teilkarten in `corridor_scene` 3 m (eine Säulenperiode) auseinander, Anker so gefälscht, dass sie am selben Ort zu liegen scheinen → `Rejected` oder `relative` gibt die wahre Verschiebung (nie die falsche „gleich“-Lösung). `rejects_low_overlap`. `candidates_are_sorted_and_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-slam --test loops`).
- [ ] **Step 5: Commit** `feat(slam): Schleifensuche mit ICP-Prüfung`

### Task 5: `run_slam`, Bericht und `driftless slam`

**Files:**
- Create: `crates/driftless-slam/src/pipeline.rs`, `crates/driftless-slam/src/report.rs`, `crates/driftless-cli/src/slam.rs`
- Modify: `crates/driftless-cli/src/main.rs`, `crates/driftless-cli/src/odom.rs` (Bag-Lese- und LAS-Schreib-Hilfen nach `crates/driftless-cli/src/scan_input.rs` ziehen und von `odom` und `slam` teilen)
- Test: `crates/driftless-slam/tests/pipeline.rs`, `crates/driftless-cli/tests/slam.rs`

**Interfaces:**
- Consumes: Tasks 1–4.
- Produces:
  - `pub struct SlamConfig { pub odom: OdomConfig, pub submap: SubmapConfig, pub loops: LoopConfig, pub rounds: u32 /* 2 */ }`.
  - `pub struct SlamResult { pub submaps: Vec<Submap>, pub anchors: Vec<Isometry3<f64>> /* optimiert */, pub odom_anchors: Vec<Isometry3<f64>>, pub loops: Vec<LoopCheck>, pub graph: Option<OptimizeReport> }` mit `pub fn poses(&self) -> Vec<Pose>` (jede Frame-Pose = `anchors[s] · offset`) und `pub fn odom_poses(&self) -> Vec<Pose>`.
  - `pub fn run_slam(frames, imu, cfg: &SlamConfig) -> (SlamResult, OdomStats)` und `pub fn solve(submaps: Vec<Submap>, odom_anchors: Vec<Isometry3<f64>>, manual: &[ManualLoop], cfg: &SlamConfig) -> SlamResult` (nur Graphteil; Task 6 nutzt ihn für das schnelle Neurechnen; `ManualLoop` kommt aus Task 6 – hier mit leerem Slice aufrufen und den Typ als `pub struct ManualLoop { pub a_t: f64, pub b_t: f64 }` in `loops.rs` vorab anlegen).
  - Odometrie-Kanten zwischen aufeinanderfolgenden Teilkarten: Information `diag(1e4,1e4,1e4, 1e3,1e3,1e3) × max(Teilkarten-Güte, 0.05)` – schwache Abschnitte geben nach, dort landet die Korrektur. Schleifenkanten: `diag(1e4… , 1e3…)`, `robust: Some(1.0)`. Ablauf: Kandidaten → prüfen → optimieren, `rounds`-mal mit den neuen Ankern.
  - `driftless slam <ordner> -o <ziel> [--lidar-to-imu …] [--lidar-topic T] [--imu-topic T]` schreibt `traj.tum` (korrigiert), `traj_odom.tum` (vorher), `map.las` (aus Teilkarten an korrigierten Ankern, 0,1-m-Raster), `slam.json` (`report.rs`: je Teilkarte `id, t_start, t_end, quality, anchor [x,y,z], odom_anchor [x,y,z]` (t ab Scan-Beginn); je Prüfung `a, b, manual, accepted, reason, inlier_ratio, rms`; Graph `initial_cost, final_cost, iterations`; Kennzahlen `start_end_gap`, `path_length` vorher/nachher). Konsole: Teilkarten, Schleifen akzeptiert/verworfen bzw. „keine Schleife gefunden“, Start↔Ende vorher → nachher, Laufzeit.

- [ ] **Step 1: Tests** – `slam_without_loop_equals_odometry` (gerader Flur: `poses()` == `odom_poses()` bis 1e-6). `slam_single_submap_skips_graph` (`graph == None`). `slam_closes_synthetic_loop` (Rundgang in `room_scene` mit IMU-Gyro-Bias wie in `pipeline_corrects_gyro_drift_with_icp`, Start = Ende: `start_end_gap` nachher < vorher und < 0,05 m). `slam_is_deterministic`. CLI: `slam_writes_all_outputs` (synthetische Bag über `driftless_io` `test-util`-Writer, alle vier Dateien vorhanden, `slam.json` parsebar), `slam_rejects_missing_imu` (deutsche Fehlermeldung wie `odom`).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test --workspace`).
- [ ] **Step 5: Commit** `feat(slam): Pose-Graph-SLAM und driftless slam`

### Task 6: Zwischenspeicher, manuelle Korrekturen, schnelles Neurechnen

**Files:**
- Create: `crates/driftless-slam/src/cache.rs`, `crates/driftless-slam/src/corrections.rs`, `crates/driftless-cli/src/correct.rs`
- Modify: `crates/driftless-slam/src/pipeline.rs` (`solve` mit manuellen Schleifen), `crates/driftless-cli/src/slam.rs` (`--reuse`), `main.rs`
- Test: `crates/driftless-slam/tests/cache.rs`, `crates/driftless-slam/tests/corrections.rs`, `crates/driftless-cli/tests/correct.rs`

**Interfaces:**
- Produces:
  - `submaps.bin` im Zielordner: eigenes Little-Endian-Format, Magic `DLSM`, Version `1`, danach Prüfsumme der Eingabe (Dateinamen + Größen der LIDAR-Bags + `SlamConfig` als Text) und die Teilkarten mit Odometrie-Ankern. `pub fn save_cache(path, submaps, odom_anchors, input_id: &str) -> io::Result<()>`, `pub fn load_cache(path, input_id: &str) -> Result<(Vec<Submap>, Vec<Isometry3<f64>>), CacheError>`; `CacheError` deutsch: falsche Version, abgeschnitten, anderer Scan – jeweils mit Hinweis „`driftless slam` ohne `--reuse` neu laufen lassen“. Längenfelder vor dem Allozieren gegen die Restdateigröße prüfen (wie beim Bag-Parser).
  - `corrections.toml` im Zielordner, Einträge nur anhängen: `[[loop]] a = 12.3, b = 250.1, enabled = true, note = "…", added = "2026-10-20T…"` – Zeiten in Sekunden **ab Scan-Beginn** (erster LiDAR-Frame). `pub struct Corrections { pub loops: Vec<LoopEntry> }`, `load`/`save`, `pub fn manual_loops(&self) -> Vec<ManualLoop>` (nur aktive).
  - Manuelle Schleife in `solve`: Teilkarten zu `a_t`/`b_t` suchen, ICP wie Task 4, aber Start = gleiche Position, Drehung aus aktueller Schätzung, Radius grob 2,0 m; akzeptiert → normale Schleifenkante ohne robusten Kern; ICP scheitert → reine Positionskante (`information = diag(0,0,0, 1e2,1e2,1e2)`) und Warnung „Stelle passt nicht sauber, nur Position übernommen“. `LoopCheck.manual = true`.
  - CLI: `driftless correct <ziel> list` (nummerierte Liste mit Status), `driftless correct <ziel> loop <a_s> <b_s> [--note TEXT]`, `driftless correct <ziel> disable <nr>`, `driftless correct <ziel> enable <nr>`. `driftless slam <ordner> -o <ziel> --reuse` lädt `submaps.bin`, liest `corrections.toml`, rechnet nur `solve` und schreibt alle Ausgaben neu (vorherige `traj.tum` wird zu `traj.vorher.tum` umbenannt, damit Vorher/Nachher vergleichbar bleibt).

- [ ] **Step 1: Tests** – Cache: `cache_round_trip` (bitgleich), `cache_rejects_other_input`, `cache_rejects_truncated` (jede Abschnittslänge 0..n → Fehler, kein Panic), `cache_rejects_huge_length_field`. Korrekturen: `corrections_append_and_disable_keep_history` (disable lässt Eintrag mit `enabled = false` stehen), `manual_loop_time_outside_scan_errors`, `manual_loop_same_submap_errors`, `duplicate_manual_loop_errors`. Pipeline: `manual_loop_fixes_missed_loop` (synthetischer Rundgang mit `LoopConfig { base_radius: 0.0, radius_per_m: 0.0, .. }` → automatisch keine Schleife; manuelle Schleife Start/Ende → `start_end_gap` < 0,05 m). CLI: `reuse_is_fast_and_equal` (`slam` dann `slam --reuse` ohne Korrekturen → gleiche `traj.tum`; `--reuse` ohne `submaps.bin` → deutscher Fehler), `correct_loop_then_reuse`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test --workspace`).
- [ ] **Step 5: Commit** `feat(slam): Korrekturen als Einträge, Zwischenspeicher, slam --reuse`

### Task 7: Drift im Viewer zeigen

**Files:**
- Modify: `crates/driftless-cli/src/view.rs`, `crates/driftless-cli/tests/view.rs`

**Interfaces:**
- Consumes: `slam.json`, `traj.tum`, `traj_odom.tum` aus Task 5.
- Produces: `driftless view <ordner> --slam <ziel>`: Laufbahn als Linien je Teilkarte, eingefärbt nach Güte (rot 0 → gelb 0,5 → grün 1); Odometrie-Laufbahn grau dünn daneben; Schleifen als Linien zwischen den Ankern (grün akzeptiert, grau verworfen, blau manuell) mit Beschriftung `a↔b`; Teilkarten-Anker als Punkte mit Beschriftung `id` und Zeit ab Scan-Beginn (damit man Zeiten für `correct loop` ablesen kann). Bestehendes `--traj` bleibt.

- [ ] **Step 1: Tests** (`--dry-run datei.rrd`, wie bestehende View-Tests) – `view_slam_logs_quality_and_loops` (Entities `slam/trajectory`, `slam/odometry`, `slam/loops`, `slam/anchors` vorhanden), `view_slam_missing_report_errors` (deutsch, nennt fehlende Datei), `quality_color_endpoints` (0 → rot, 1 → grün).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-cli`).
- [ ] **Step 5: Commit** `feat(view): Güte, Schleifen und Teilkarten anzeigen`

### Task 8: Prüfung mit echten Daten, Doku

**Files:**
- Create: `crates/driftless-slam/tests/real_data.rs` (alle `#[ignore]`), `docs/ergebnisse/2026-10-20-meilenstein-2.md`
- Modify: `docs/testdaten.md` (Rundgang-Sequenz eintragen), `README.md` (Befehle `slam`, `correct`, Status), `crates/driftless-slam/src/loops.rs` (nur Schwellwerte, falls die Messung es verlangt – jede Änderung im Ergebnisdokument begründen)

**Interfaces:** keine neuen.

- [ ] **Step 1: Messen** (Release-Build) auf drei Datensätzen, je Odometrie vs. SLAM: Hilti exp14 (ATE gegen `exp14_basement_2_imu.txt`, Extrinsik aus `docs/testdaten.md`), Hilti-Rundgang-Sequenz (vom Controller bereitgestellt, Pfad in `docs/testdaten.md`), JMK6 `Test` (ATE gegen RayStudio `trj.txt`, Start↔Ende). Dazu: Laufzeit voll und `--reuse`, Spitzen-RAM (`/usr/bin/time -l`), Schleifen akzeptiert/verworfen.
- [ ] **Step 2: Tests mit den gemessenen Schranken** – `exp14_slam_not_worse` (ATE SLAM ≤ ATE Odometrie × 1,05), `loop_sequence_slam_better` (ATE SLAM < ATE Odometrie und Schranke = Messwert × 1,2), `jmk6_slam_not_worse_than_odom` (ATE gegen `trj.txt`), `jmk6_reuse_under_10_s`. `cargo test --release -p driftless-slam -- --ignored` → PASS.
- [ ] **Step 3: Ergebnisdokument** (Deutsch, Tabellen, ehrliche Einschränkungen) und README/Testdaten aktualisieren.
- [ ] **Step 4: Gesamtprüfung** – `cargo test --workspace`, `cargo clippy --workspace -- -D warnings`, `cargo deny check`, `cargo fmt --check`.
- [ ] **Step 5: Commit** `test(slam): Prüfung mit echten Daten, Ergebnisse Meilenstein 2`
