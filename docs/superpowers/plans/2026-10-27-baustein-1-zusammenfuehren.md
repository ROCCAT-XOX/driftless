# Baustein 1 – Mehrere Scans zu einem Modell zusammenführen Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mehrere Scans (je ein SLAM-Lauf mit Karte) werden automatisch zueinander ausgerichtet und ergeben ein Modell (`modell.las`, `modell.ply`, Lagen, Güte je Scan). Auf dem Modell läuft die fotoähnliche Oberfläche mit den Bildern aller Scans. Dazu kommen PLY/LAS öffnen und Hineinziehen im Programm sowie Hilfe von Hand über drei Punktpaare.

**Architecture:**
- **Ausrichtung** kommt als neues Modul `driftless-slam/src/align.rs`:
  1. Eigene FPFH-Merkmale auf 10-cm-Punkten.
  2. RANSAC über Drehung um die Hochachse plus Verschiebung (4 Freiheitsgrade, die Karten sind nach der Schwerkraft ausgerichtet).
  3. Mehrere Kandidaten, je Kandidat stufenweises ICP in 6 Freiheitsgraden (`odom::icp::align`).
  4. Güte aus Überlappung und Restfehler. Zwei verschiedene, fast gleich gute Kandidaten heißen „unsicher“.
  5. Gier-Rastersuche als zweiter Weg.
- **Zusammenführen** ist der Schritt `driftless-steps/src/merge.rs`:
  - Er baut eine Kette: Jeder Scan dockt an den am besten passenden, schon platzierten Scan an.
  - Er gleicht die Farbe je Scan an, dünnt auf 1 cm aus (gefärbte Punkte gewinnen) und glättet gemeinsam.
  - Er schreibt `modell.las` (Scan-Nummer in `point_source_id`), `modell.ply` und `modell.json`.
- **Oberfläche aus dem Modell:** Die Textur kann mehrere Bildquellen (`TextureSource`) verarbeiten, jede schon im Modell-Rahmen. `run_model_surface` setzt die Punkte und Sensoren aller Scans zusammen.
- **App:**
  - Ordner `<projekt>/modelle/modell-NNN/`, Auswahl der Scans und Knoten im Baum.
  - Farbmodus „je Scan“, „Punktwolke öffnen …“, Hineinziehen.
  - Punktpaare von Hand.

**Tech Stack:** Rust 2021, nalgebra 0.33, kiddo 6, rayon, las 0.11, egui/eframe 0.36, wgpu, clap 4, serde/toml.

**Spec:** `docs/superpowers/specs/2026-10-22-zusammenfuehren-design.md` (mit Nachtrag 27.10.: Ziel 8 und Abschnitt 2a).

**Codeübersicht (Stand 27.10.):**
- Ein Schritt besteht aus `…Request`, Lauf-Funktion mit `Progress`/Abbruch, `input_id` (Text + Version, `hex(content_hash)`) und dem Bericht zuletzt (`atomic_write`).
- Die App legt nummerierte Ordner über `create_numbered` an. Der Wächter `StepDir` löscht einen neuen Ordner bei Fehler oder Abbruch.
- `SurfaceInput` nimmt laut Doku schon aneinandergehängte Listen mehrerer Scans. Mehrere Bildquellen lehnt bisher nur `check_sources` (`surface/mod.rs:251`) ab, und `texture_mesh` nimmt `sources.first()`.
- `views.rs`, `atlas.rs` und `exposure` gehen von einem Bildstrom aus: Bilder werden über (Kamera-Index, Zeitstempel) gefunden, Kameras über den Namen.
- Es gibt keinen PLY-Leser, kein „Wolke öffnen“, kein Hineinziehen und keine Mehrfachauswahl.
- Die Karten sind nur über den IMU-Start nach der Schwerkraft ausgerichtet: In den Daten vom 22.10. sind bis 0,6° Neigung zwischen den Scans. Deshalb folgt nach der 4-DoF-Grobsuche ein 6-DoF-ICP.

## Global Constraints

- **Lizenz** Apache-2.0 OR MIT, kein fremder Code unter GPL/AGPL oder unklarer Lizenz. FPFH, RANSAC und PLY werden selbst geschrieben, ohne neue Abhängigkeit außer `kiddo`/`rayon` in `driftless-slam`.
- **Keine Scan-Daten, Kundennamen oder Zugangsdaten im Repo.** Tests mit echten Daten sind `#[ignore = "needs DRIFTLESS_TESTDATA"]` und laufen über `driftless_io::testdata::path`.
- **Speicher:** die drei Scans vom 22.10. (≈ 17 Mio. Punkte) zusammenführen unter 4 GB Spitze.
- **Laufzeit:** das Zusammenführen der drei Scans ≤ 120 s auf VM 101 mit 6 Threads.
- **Nichts wird überschrieben.** Jedes Zusammenführen ist ein neuer Ordner `modell-NNN`. Abbrechen hinterlässt keine halben Dateien: vor dem Schreiben Abbruch prüfen, den Bericht zuletzt und atomar schreiben, in der App über `StepDir`.
- **Rahmen:**
  - Modell-Rahmen = Rahmen des ersten gewählten Scans (Bezug, Lage = Einheit).
  - `T_i` bildet Welt von Scan i auf das Modell ab, `Isometry3<f64>`.
  - In JSON/TOML als `[f64; 16]` zeilenweise (row-major) gespeichert.
- **Güte-Wörter** im Bericht und in der Oberfläche: `"sicher"`, `"unsicher"`, `"gescheitert"`. Ein unsicherer oder gescheiterter Scan kommt nur mit `include_unsure` (CLI `--auch-unsichere`, App-Häkchen) ins Modell, nie still.
- **Texte für Nutzer** auf Deutsch, kurz, App-Texte in `crates/driftless-app/src/ui/text.rs`. Code-Kommentare und Doku im Stil der Datei: Englisch, genaue Zahlen.
- **CI** auf macOS und Windows grün: `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo test --workspace`, `cargo deny check`.
- **Schwere Läufe** nur auf VM 101 (`~/.local/bin/driftless-vm`), 6 Threads, nice 10, ein schwerer Lauf zur Zeit. Push und Merge nur nach Yannicks Okay.
- **Commits** mit den Trailern der Sitzung, noreply-Mail ist im Repo gesetzt. Vor dem Bauen `export PATH=$HOME/.cargo/bin:$PATH`.

## Review Focus

1. **Symmetrischer Raum** (leere Kiste, 180° gedreht gleich): darf nie „sicher“ mit falscher Drehung melden. Test in Task 1: `symmetric_box_is_unsure`.
2. **Scan ohne Farbe** (kein `color.las`, keine Bilder): Das Zusammenführen klappt, die Punkte bleiben ungefärbt, und die Oberfläche nimmt die Bilder der anderen. Tests in Task 2: `member_without_color_merges`, in Task 4: `model_surface_with_one_imageless_member`.
3. **Abbruch während des Zusammenführens:** Kein `modell.json` und keine halbe Wolke bleibt zurück. Test in Task 2: `cancel_leaves_no_report`, in Task 6 über `StepDir`.
4. **Fremde PLY** (ASCII, Big-Endian, `double`, Farbe als float, ohne Farbe, kaputter Kopf): wird gelesen oder ergibt eine klare Meldung, keine Panik. Tests in Task 5.
5. **Ungültige Auswahl:** nur ein Scan, derselbe Lauf doppelt, Karte fehlt oder veraltet. Klare Meldung, kein Modell. Test in Task 2: `refuses_bad_members`.

---

### Task 1: Ausrichtung zweier Punktwolken (`driftless-slam/src/align.rs`)

**Files:**
- Create: `crates/driftless-slam/src/align.rs`, `crates/driftless-slam/src/features.rs` (Normalen, FPFH)
- Modify: `crates/driftless-slam/src/lib.rs` (Module + `pub use`), `crates/driftless-slam/Cargo.toml` (`kiddo = { version = "6", default-features = false }`, `rayon = "1.12.0"`)
- Modify: `crates/driftless-slam/src/loops.rs`: `register`/`stage_params` nach `pub(crate)` öffnen, damit `align.rs` sie nutzt statt sie zu kopieren
- Test: `crates/driftless-slam/tests/align.rs`, Szenen-Helfer in derselben Datei

**Interfaces:**
- Produces:
```rust
pub struct AlignConfig {
    pub voxel: f64,           // 0.10  Merkmale und RANSAC
    pub normal_k: usize,      // 12
    pub feature_radius: f64,  // 0.30
    pub ransac_iters: usize,  // 20_000
    pub inlier_dist: f64,     // 0.10
    pub candidates: usize,    // 5     unterschiedliche Kandidaten (> 0.3 m oder > 10° auseinander)
    pub yaw_step_deg: f64,    // 5.0   Rastersuche
    pub fine_voxel: f64,      // 0.02
    pub overlap_dist: f64,    // 0.05
    pub min_overlap: f64,     // 0.15
    pub max_residual_mm: f64, // 5.0   Median der Abstände innerhalb overlap_dist
    pub max_tilt_deg: f64,    // 3.0   Neigung der gefundenen Lage gegen die Hochachse
    pub ambiguity: f64,       // 0.9   zweitbester unterschiedlicher Kandidat ≥ 0.9 × bester → unsicher
}
impl Default for AlignConfig
pub enum AlignMethod { Features, Grid, PointPairs }
pub enum Verdict { Sure, Unsure(String), Failed(String) }   // Gründe auf Deutsch
impl Verdict { pub fn word(&self) -> &'static str }          // "sicher" | "unsicher" | "gescheitert"
pub struct PairQuality { pub overlap: f64, pub residual_median_mm: f64, pub residual_rms_mm: f64, pub tilt_deg: f64 }
pub struct Alignment {
    pub pose: Isometry3<f64>,                 // source-Welt → target-Welt
    pub method: AlignMethod,
    pub quality: PairQuality,
    pub verdict: Verdict,
    pub runner_up: Option<(Isometry3<f64>, f64)>, // Lage und Überlappung des besten anderen Kandidaten
}
pub fn align_pair(source: &[Vector3<f64>], target: &[Vector3<f64>], cfg: &AlignConfig, progress: &Progress) -> Alignment
pub fn align_with_pairs(source: &[Vector3<f64>], target: &[Vector3<f64>], pairs: &[(Vector3<f64>, Vector3<f64>)], cfg: &AlignConfig, progress: &Progress) -> Alignment
pub fn pose_from_pairs(pairs: &[(Vector3<f64>, Vector3<f64>)]) -> Option<Isometry3<f64>> // Kabsch; None bei < 3 Paaren oder fast auf einer Linie
pub fn pair_quality(source: &[Vector3<f64>], target: &[Vector3<f64>], pose: &Isometry3<f64>, cfg: &AlignConfig) -> PairQuality
```
`Progress` ist `driftless_io::progress::Progress`. Bei Abbruch (`progress.is_cancelled()`) kehrt `align_pair` früh mit `Verdict::Failed("abgebrochen")` zurück. Der Aufrufer prüft den Abbruch selbst.

**Algorithmus (`align_pair`):**
1. Beide Wolken auf `voxel` ausdünnen (`odom::voxel_map::voxel_downsample`).
2. Normalen über `normal_k` Nachbarn (Hauptachsen wie `clean::spread_axes`; die Funktion ist in `driftless-map` crate-privat, die kleine Zerlegung in `features.rs` neu schreiben).
3. FPFH mit 33 Klassen in `feature_radius`.
4. Für jeden source-Punkt den nächsten Nachbarn im Merkmalsraum suchen, nur gegenseitige Treffer behalten.
5. RANSAC über Paare von Treffern:
   - Gier aus der xy-Richtung der beiden Verbindungsvektoren, Verschiebung aus dem Schwerpunkt.
   - Paare verwerfen, deren Länge oder Höhendifferenz um mehr als `inlier_dist` abweicht.
   - Treffer mit Abstand < `inlier_dist` zählen.
   - Die besten `candidates` unterschiedlichen Kandidaten behalten.
6. Hat der beste Kandidat weniger als 12 Inlier: Rastersuche, Gier alle `yaw_step_deg`, Verschiebung über die Mitte der Begrenzungsboxen, je Gier ein grobes ICP. `method = Grid`.
7. Je Kandidat `register` mit den Stufen `[1.0, 0.5, 0.25]` und danach eine Stufe auf `fine_voxel` mit `max_corr_dist = overlap_dist`, dann `pair_quality`.
   - Bester Kandidat = größte Überlappung.
   - `runner_up` = bester Kandidat, der nach ICP mehr als 0,3 m oder 10° entfernt liegt.
8. Urteil:
   - `Failed`: kein Kandidat.
   - `Unsure`, Grund jeweils kurz und deutsch:
     - Überlappung < `min_overlap`,
     - Restfehler > `max_residual_mm`,
     - Neigung > `max_tilt_deg`,
     - `runner_up` mit Überlappung ≥ `ambiguity` × beste.
   - sonst `Sure`.
9. Die Rechenschritte 3–7 laufen mit rayon und sind deterministisch: fester Zufallsstartwert, Ergebnis unabhängig von der Thread-Zahl.

`align_with_pairs`: `pose_from_pairs` als Start, danach die Schritte 7–8 nur mit diesem Kandidaten. `method = PointPairs`. Der Mehrdeutigkeitstest entfällt, weil der Mensch entschieden hat.

- [ ] **Step 1: Szenen-Helfer und fehlschlagende Tests schreiben** (`tests/align.rs`).
  - Szene: Raum 8×5×2,8 m aus Wand-, Boden- und Deckenpunkten im Abstand 2 cm, dazu drei Kisten und eine Säule an unsymmetrischen Stellen. Rauschen σ = 3 mm, fester Startwert.
  - `source` = Teilausschnitt mit bekannter Lage `T` (Gier 37°, t = (1.2, −0.8, 0.05)), so dass `target ≈ T · source`.
  - Tests:
    - `aligns_rotated_room`: Lagefehler < 1 cm und < 0,3°, `Verdict::Sure`, Überlappung > 0,5.
    - `aligns_with_small_tilt`: wie oben plus 0,6° Neigung um x, Fehler < 1 cm / 0,3°.
    - `symmetric_box_is_unsure`: leere Kiste 6×4×2,5 m, `source` = Kiste um 180° um die Mitte gedreht, Urteil nicht `Sure`.
    - `disjoint_rooms_are_not_sure`: zwei verschiedene Räume ohne gemeinsamen Teil, Urteil nicht `Sure`.
    - `corridor_overlap_aligns`: Raum A + Flur und Flur + Raum B, gemeinsamer Teil ≈ 15 % der Punkte. Entweder `Sure` mit Fehler < 2 cm, oder nicht `Sure`; nie `Sure` mit Fehler ≥ 2 cm.
    - `pose_from_pairs_recovers_transform`: exakt bis 1e-9.
    - `pose_from_pairs_rejects_collinear`: drei Punkte auf einer Linie → `None`.
    - `align_with_pairs_refines`: drei Paare mit 5 cm Störung, Fehler < 1 cm, `method == PointPairs`.
    - `deterministic_across_threads`: gleiche Lage mit `rayon::ThreadPoolBuilder` 1 und 4 Threads.
- [ ] **Step 2:** `cargo test -p driftless-slam --test align` schlägt fehl: Modul fehlt.
- [ ] **Step 3:** `features.rs` und `align.rs` nach den Signaturen und dem Algorithmus oben bauen; `register` öffnen.
- [ ] **Step 4:** `cargo test -p driftless-slam` grün. `cargo clippy -p driftless-slam --all-targets -- -D warnings` sauber. Alle Align-Tests zusammen < 30 s im Release-Test-Profil des Repos.
- [ ] **Step 5:** Commit `feat(slam): Ausrichtung zweier Wolken über Merkmale, RANSAC und ICP`.

### Task 2: Schritt Zusammenführen und `driftless merge`

**Files:**
- Create: `crates/driftless-steps/src/merge.rs`, `crates/driftless-cli/src/merge.rs`
- Modify:
  - `crates/driftless-steps/src/lib.rs`.
  - `crates/driftless-cli/src/main.rs`: `Command::Merge`.
  - `crates/driftless-map/src/export.rs`: `write_las_sources`, `write_ply_sources`, `pub use` in `lib.rs`.
  - `crates/driftless-io/src/testscan.rs`: Raum mit Möbeln aus versetzter Lage.
  - Den Baustein `MapInput` aus `scan`/`slam_dir`/`map_dir` aus `crates/driftless-cli/src/surface.rs::request` als `pub fn map_input_for(scan: &Path, slam_dir: &Path, map_dir: &Path, slam: SlamSettings) -> MapInput` nach `driftless-steps/src/map.rs` ziehen und in der CLI nutzen.
- Test: `crates/driftless-steps/tests/merge.rs`, `crates/driftless-cli/tests/merge.rs`, `crates/driftless-steps/tests/merge_real.rs` (ignoriert)

**Interfaces:**
- Consumes: Task 1 `align_pair`, `align_with_pairs`, `AlignConfig`, `Verdict`.
- Produces:
```rust
pub const MERGE_VERSION: u32 = 1;
pub const MODEL_REPORT: &str = "modell.json";
pub const MODEL_LAS: &str = "modell.las";
pub const MODEL_PLY: &str = "modell.ply";
pub struct MergeMember { pub scan: PathBuf, pub slam_dir: PathBuf, pub map_dir: PathBuf, pub slam: SlamSettings }
pub struct MergeRequest {
    pub members: Vec<MergeMember>,       // [0] = Bezug
    pub out_dir: PathBuf,
    pub align: AlignConfig,
    pub voxel: f64,                      // 0.01
    pub smooth: bool,                    // true: smooth_surfaces über die zusammengeführten Punkte
    pub include_unsure: bool,            // false
    pub pairs: Vec<PointPairEntry>,      // Task 7; bis dahin leer
}
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ModelMember {
    pub scan: PathBuf, pub slam_dir: PathBuf, pub map_dir: PathBuf, pub slam: SlamSettings,
    pub transform: [f64; 16], pub placed_against: Option<usize>,
    pub method: String,                  // "bezug" | "merkmale" | "raster" | "punktpaare"
    pub verdict: String, pub reason: Option<String>,
    pub overlap: f64, pub residual_median_mm: f64, pub residual_rms_mm: f64,
    pub included: bool, pub color_gain: [f32; 3], pub points: u64,
}
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ModelReport { pub version: u32, pub input_id: String, pub members: Vec<ModelMember>, pub points: u64, pub seconds_total: f64, pub peak_rss_bytes: u64 }
pub fn run_merge(req: &MergeRequest, progress: &Progress) -> Result<ModelReport, StepError>
pub fn read_model(dir: &Path) -> Result<ModelReport, StepError>
pub fn member_transform(m: &ModelMember) -> Isometry3<f64>
pub fn merge_input_id(req: &MergeRequest) -> Result<String, StepError> // MERGE_VERSION + clean_stage_id je Mitglied + Einstellungen + aktive Punktpaare
```
`SlamSettings` braucht dafür `Serialize`/`Deserialize`. Falls es diese Ableitungen noch nicht hat, werden sie ergänzt.

**Ablauf `run_merge`:**
1. Prüfen, Meldungen auf Deutsch, als `StepError::Failed`:
   - mindestens 2 Mitglieder,
   - kein Paar (scan, slam_dir) doppelt,
   - jede Karte hat `clean.bin`, das zu `clean_stage_id` passt; sonst „Karte fehlt oder ist veraltet: <Pfad> – erst Karte rechnen“.
2. Je Mitglied `clean.bin` laden (`load_stage`), xyz als f64 für die Ausrichtung. Farbe aus `color.las` desselben Kartenordners, sonst ungefärbt.
3. Kette:
   - Mitglied 0 ist platziert mit `T_0 = I`.
   - Solange unplatzierte Mitglieder übrig sind: für jedes unplatzierte i und jedes platzierte j `align_pair(i, j)` rechnen, je Paar zwischengespeichert.
   - Das Paar mit dem besten Urteil gewinnt, bei gleichem Urteil die größte Überlappung. `T_i = T_j · pose`.
   - Unsichere und gescheiterte werden mit ihrem besten Ergebnis eingetragen, `included = include_unsure && verdict != gescheitert`.
4. Farbangleich:
   - Je eingeschlossenem Mitglied i > 0 die 1-cm-Voxel, die in i und in schon übernommenen Mitgliedern gefärbt sind.
   - `color_gain` = Median-Verhältnis je Kanal, begrenzt auf [0,5; 2,0], sonst 1.
5. Zusammenlegen:
   - Alle eingeschlossenen Punkte in den Modell-Rahmen.
   - Je `voxel`-Zelle ein Punkt: gefärbt vor ungefärbt, dann kleinere Mitgliedsnummer.
   - Bei `smooth` danach `smooth_surfaces` mit `SmoothConfig::default()`.
6. Vor dem Schreiben Abbruch prüfen. Dann `modell.las` (`point_source_id` = Mitgliedsnummer), `modell.ply` (zusätzliche Eigenschaft `scan` als `ushort`) und zuletzt `modell.json` per `atomic_write`.

**CLI:**
- Aufruf: `driftless merge <SCAN>... -o <OUT> [--slam <DIR>]... [--map <DIR>]... [--auch-unsichere] [--voxel 0.01] [--ohne-glaetten] [--lidar-to-imu …] [--lidar-topic …] [--imu-topic …]`.
- Standard für `--slam` ist `<scan>/driftless`, für `--map` der Lauf-Ordner. Wenn `--slam`/`--map` angegeben sind, dann genau einmal je Scan in derselben Reihenfolge.
- Ausgabe je Scan eine Zeile: Name, Urteil, Überlappung %, Restfehler mm, und bei unsicher den Grund.

- [ ] **Step 1: Testszene.**
  - `testscan::furnished_room_bag(dir: &Path, imu: bool, offset: [f64; 3], yaw: f64)`: der Raum von `room_scan` mit zwei Kisten an unsymmetrischen Stellen, aufgenommen von einer um `offset`/`yaw` versetzten Lage.
  - `room_scan` selbst bleibt unverändert, damit bestehende Tests gleich bleiben.
- [ ] **Step 2: Fehlschlagende Tests** (`steps/tests/merge.rs`). Je zwei Scans rechnen: Lage A = Ursprung, Lage B = (0,8, 0,5, 0), Gier 30°. Jeweils SLAM und Karte mit `run_slam`/`run_map`, Fertigdaten über die vorhandenen Test-Helfer der Schritte. Tests:
  - `merges_two_scans`: Urteil B „sicher“; `member_transform(B)` weicht < 2 cm / 0,5° von der wahren relativen Lage ab; `modell.las` hat `point_source_id` {0, 1}; `modell.json` lesbar über `read_model`.
  - `member_without_color_merges`: ein Mitglied ohne `color.las` ergibt ein Modell, dessen Punkte dieses Mitglieds ungefärbt sind.
  - `refuses_bad_members`: ein Mitglied; derselbe Lauf zweimal; Karte ohne `clean.bin`. Je `StepError::Failed` mit Text, kein `modell.json`.
  - `cancel_leaves_no_report`: `Progress::cancel()` vor dem Start ergibt `StepError::Cancelled`, kein `modell.json`, kein `modell.las`.
  - `same_input_same_id`: `merge_input_id` ist bei gleicher Anfrage gleich und ändert sich mit `voxel`.
  - `unsure_member_left_out`: ein Mitglied mit leerem Raum ohne Möbel (symmetrisch). Es ist „unsicher“ und `included == false`; mit `include_unsure` dabei.
- [ ] **Step 3:** `cargo test -p driftless-steps --test merge` schlägt fehl.
- [ ] **Step 4:** `merge.rs`, `write_las_sources(path, cloud, sources: &[u16])`, `write_ply_sources(path, cloud, sources: &[u16])` und `map_input_for` bauen.
- [ ] **Step 5: CLI-Test** (`cli/tests/merge.rs`): zwei Scans; `driftless merge a b -o out` ergibt Exit 0, Ausgabe enthält „sicher“, `out/modell.json` existiert; `--slam` einmal bei zwei Scans ergibt eine klare Fehlermeldung.
- [ ] **Step 6: Ignorierter Test mit echten Daten** (`merge_real.rs`):
  - `black_room_three_scans`: Testdaten `jmk6-2026-10-22/20261022133554`, `…132520`, `…133028`, jeweils mit Lauf-Ordner `driftless/`, Bezug 133554. Alle „sicher“, Überlappung > 0,8, Restfehler-Median ≤ 5 mm. Spitzen-Speicher und Sekunden ausgeben.
  - `corridor_cut_from_office`: Büro `jmk6-2026-10-20/20261020173822`, Lauf `driftless/`. Die Karte wird in zwei Teile entlang x geschnitten, mit ≈ 15 % gemeinsamem Streifen; ein Teil wird um bekannte Gier 70° und t = (3, −2, 0) bewegt; `align_pair` direkt. Entweder „sicher“ mit Fehler < 3 cm oder nicht „sicher“.
- [ ] **Step 7:** `cargo test --workspace` grün, clippy sauber. Commit `feat: Schritt Zusammenführen und driftless merge`.

### Task 3: Textur aus mehreren Bildquellen (`driftless-map`)

**Files:**
- Modify:
  - `crates/driftless-map/src/surface/mod.rs` (`check_sources` entfernen, `texture_mesh` mit allen Quellen),
  - `surface/views.rs` (`UsedImage`, `ViewTable`, `collect_views_on`, `cpu_views`),
  - `surface/views_gpu.rs`,
  - `surface/atlas.rs` (`bake`, `absorb_islands`),
  - `surface/level.rs`, wo Bilder über Rang/Kamera aufgelöst werden.
- Test: in `surface/mod.rs` den Test `build_surface_textures_painted_room`, der zwei Quellen ablehnt, ersetzen; neue Tests dort und in `views.rs`.

**Interfaces:**
- Consumes: `TextureSource { images, poses, cams }`. Jede Quelle kommt mit Lagen schon im gemeinsamen Rahmen; der Aufrufer multipliziert `T_i` von links.
- Produces:
  - `UsedImage { source: usize, camera: usize /* index into sources[source].cams */, stamp, world_from_cam }`.
  - `ViewTable::rank_of(source: usize, camera: usize, stamp: Time) -> Option<usize>`.
  - Ränge sind global: Quelle für Quelle, innerhalb einer Quelle wie heute (Zeitstempel, Kameraname).
  - `texture_mesh(mesh, sources: &[TextureSource], …)` und `build_surface(…, sources, …)` nehmen 1..n Quellen. 0 Quellen heißt wie heute „ohne Textur“.
  - Doppelte Kameranamen sind nur **innerhalb** einer Quelle ein Fehler.
- Belichtung gemeinsam: ein `Observations` über alle Bilder aller Quellen.
  - Kamera-Kennung je (Quelle, Kamera): eigene Randabdunklung je Scan, Glättung nur innerhalb einer Quelle.
  - Die Verstärkungen werden über Punkte verbunden, die mehrere Quellen sehen.
  - `mean(log g) = 0` über alle Bilder.
- Bei einer Quelle bleibt alles byte-gleich wie heute. Prüfen mit dem bestehenden Test, der Seiten-Hashes vergleicht. Gibt es keinen, `one_source_pages_unchanged` gegen einen vor der Änderung festgehaltenen Hash ergänzen.

- [ ] **Step 1: Fehlschlagende Tests** (Szene `testutil::scene` / `walk_data`):
  - `two_sources_cover_more`: Die Bilder der Szene in zwei Quellen teilen, Quelle 1 sieht nur die linke, Quelle 2 nur die rechte Hälfte des Raums. Mit beiden ist `faces_untextured` kleiner als mit jeder allein; die Farbe trifft die gemalte Wand wie im bestehenden Farbtest.
  - `two_sources_exposure_joined`: Bilder der Quelle 2 um ×1,5 heller gerendert. Nach dem Ausgleich weicht die Wandfarbe beider Hälften < 8 (sRGB) voneinander ab.
  - `two_sources_same_camera_name`: beide Quellen mit Kamera „left“, kein Fehler; zwei gleiche Namen in **einer** Quelle bleiben ein Fehler.
  - `second_source_in_other_frame`: Quelle 2 mit Lagen `T · pose` und die Punkte ebenfalls mit `T` bewegt; Ergebnis gleich wie ohne `T`, bis auf Rundung.
  - `one_source_pages_unchanged`.
- [ ] **Step 2:** Tests schlagen fehl.
- [ ] **Step 3:** Umbauen. Jede Stelle, die heute `src` oder `src.images` oder `src.cams` nimmt, läuft über die Quellen nacheinander, auf CPU und GPU. Die Seiten bleiben deterministisch.
- [ ] **Step 4:** `cargo test -p driftless-map` grün, darunter `color_gpu`/`gpu_visibility`, soweit auf der Maschine eine GPU da ist. clippy sauber.
- [ ] **Step 5:** Commit `feat(map): Textur aus mehreren Bildquellen mit gemeinsamer Belichtung`.

### Task 4: Oberfläche aus einem Modell

**Files:**
- Modify:
  - `crates/driftless-steps/src/surface.rs`: gemeinsamen Kern aus `run_surface` herausziehen, `run_model_surface` neu.
  - `crates/driftless-cli/src/surface.rs`: `--model`; `scan`/`--slam` nur Pflicht ohne `--model`.
- Test: `crates/driftless-steps/tests/surface.rs`, `crates/driftless-cli/tests/surface.rs`

**Interfaces:**
- Consumes: Task 2 `read_model`, `member_transform`, `ModelMember`, `map_input_for`; Task 3 mehrere `TextureSource`.
- Produces:
```rust
pub struct ModelSurfaceRequest { pub model_dir: PathBuf, pub out_dir: PathBuf, pub config: SurfaceConfig, pub texture: bool, pub gpu: GpuPref }
pub fn run_model_surface(req: &ModelSurfaceRequest, progress: &Progress) -> Result<Value, StepError>
pub fn model_surface_fits(req: &ModelSurfaceRequest) -> bool
```
- Eingang: nur Mitglieder mit `included`.
  - Je Mitglied `clean.bin` mit `T_i` bewegen und anhängen.
  - Sensoren = `T_i · sensors_for_frames(poses_i, lidar_to_imu_i)`; `sensor_of_point` um die Zahl der vorherigen Sensoren versetzen.
  - Je Mitglied mit Bildern eine `TextureSource` mit Lagen `T_i · pose` (`color_input` wie heute je Scan).
  - `dense_xyz = None`; die Genauigkeit wird gegen die zusammengesetzten Punkte gemessen.
- `input_id` = `ModelReport.input_id` + `surface v{SURFACE_VERSION}` + Einstellungen wie bei `plan()`. `SURFACE_VERSION` bleibt 12, weil sich für einen Lauf nichts ändert.
- `oberflaeche.json` wie heute, dazu `"model": { "dir", "members": [<Scan-Ordnername>…] }`. Standard-Ausgabe der CLI ist `<model>/oberflaeche`.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `model_surface_textures_both_scans`: Modell aus Task-2-Szene mit Bildern (`scan_with_images`-Muster für beide Lagen), `faces_untextured` < dem Wert eines einzelnen Scans.
  - `model_surface_with_one_imageless_member`: ein Mitglied ohne Bilder, die Textur kommt aus dem anderen, kein Fehler.
  - `model_surface_fits_after_run`: true nach dem Lauf, false nach Änderung von `voxel`.
  - CLI: `driftless surface --model m --voxel 0.05` ergibt Exit 0; `driftless surface` ohne `scan` und ohne `--model` ergibt einen Clap-Fehler.
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen; `run_surface` ruft denselben Kern, das Ergebnis bleibt für Läufe gleich (bestehende Tests grün).
- [ ] **Step 4:** `cargo test --workspace` grün, clippy sauber. Commit `feat: Oberfläche aus einem Modell mit den Bildern aller Scans`.

### Task 5: PLY lesen, Dateien öffnen und hineinziehen, Farbmodus „je Scan“

**Files:**
- Create: `crates/driftless-app/src/ply.rs`
- Modify:
  - `crates/driftless-app/src/cloud.rs`: `PointSet.source: Vec<u16>`, `load_las` liest `point_source_id`, `load_cloud` wählt nach Endung.
  - `lod.rs`: Bits 16–31 von `extra` = Scan.
  - `render/mod.rs` (`ColorMode::Scan = 3`), `render/points.wgsl` (Palette mit 8 unterscheidbaren Farben, Scan modulo 8).
  - `ui/view3d.rs::color_modes`.
  - `ui/rail.rs::project_menu` („Punktwolke öffnen …“), `ui/app.rs` (Aktion und Hineinziehen), `ui/text.rs`.
- Test: `crates/driftless-app/tests/cloud.rs`, `crates/driftless-app/tests/ui_app.rs` (oder die passende bestehende UI-Testdatei)

**Interfaces:**
- Produces:
  - `pub fn load_ply(path: &Path, progress: &Progress) -> Result<PointSet, StepError>`.
  - `PointSet.source`: leer, wenn unbekannt oder nur ein Wert vorkommt.
  - `ColorMode::Scan` wird nur angeboten, wenn `source` nicht leer ist.
  - `Action::PickOpenCloud`.
- PLY:
  - `format ascii 1.0 | binary_little_endian 1.0 | binary_big_endian 1.0`.
  - Element `vertex` mit `x y z` als `float`/`double` (auch `float32`/`float64`).
  - Farbe optional: `red green blue` als `uchar` oder als `float` 0–1.
  - Optional `intensity` oder `scalar_intensity`, sowie `scan` (`ushort`, aus Task 2).
  - Weitere Eigenschaften und Elemente (z. B. `face`) werden übersprungen.
  - Kaputter oder unbekannter Kopf ergibt `StepError::Failed("PLY nicht lesbar: …")`.
  - Lesen in Blöcken mit Abbruchprüfung wie `load_las`.
- Hineinziehen:
  - `.las`/`.ply` → `Action::LoadCloud { path, run: None }`; `.glb` → `Action::LoadSurface`; anderes → Hinweis „Dateiformat nicht unterstützt“.
  - Bei mehreren Dateien die erste passende.
  - Während eine Datei über dem Fenster schwebt, ein Hinweis „Loslassen zum Öffnen“.
  - Ohne Projekt nur ansehen.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `ply_round_trip_from_write_ply` (Datei aus `driftless_map::write_ply`)
  - `ply_ascii_with_float_color`
  - `ply_big_endian_double`
  - `ply_without_color`
  - `ply_skips_faces`
  - `ply_broken_header_is_error`: abgeschnittener Kopf und falsche Anzahl ergeben `Err`, keine Panik
  - `las_point_source_becomes_scan`: LAS aus `write_las_sources` mit {0, 1} ergibt `source`; einheitlich 0 ergibt leer
  - UI: `dropped_las_loads_cloud`, `dropped_unknown_shows_hint` mit dem Test-Harness der bestehenden UI-Tests
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen. **Step 4:** `cargo test -p driftless-app` grün; Schnappschüsse, wo die UI sich sichtbar ändert, bewusst neu erzeugen und im Commit nennen; clippy sauber.
- [ ] **Step 5:** Commit `feat(app): PLY öffnen, Dateien hineinziehen, Farbe je Scan`.

### Task 6: Modelle in der App

**Files:**
- Modify:
  - `crates/driftless-app/src/project.rs`: `MODELS_DIR = "modelle"`, `MODEL_PREFIX = "modell-"`, `ModelInfo`, `Project::models()`, `run_merge_job`, `run_model_surface_job`.
  - `ui/mod.rs`: `Selection::Model(PathBuf)`, `Action::OpenMerge`, `Action::RunMerge{…}`, `Action::RunModelSurface{model}`.
  - `jobs.rs`: `JobOutput::Model(PathBuf)`.
  - `ui/actions.rs`: `start_step`.
  - `ui/app.rs`: `finished`, `apply`.
  - `ui/steps/daten.rs`, `ui/steps/tree.rs`, `ui/steps/bilder.rs`, `ui/text.rs`.
  - Neu `ui/steps/merge.rs` für das Auswahlfeld.
- Test: `crates/driftless-app/tests/project.rs`, `tests/ui_daten.rs`, `tests/ui_bilder.rs`

**Interfaces:**
- Consumes: Task 2 `run_merge`, `MergeRequest`, `read_model`, `ModelReport`; Task 4 `run_model_surface`, `model_surface_fits`; Task 5 `ColorMode::Scan`.
- Produces:
```rust
pub struct ModelInfo { pub dir: PathBuf, pub label: String, pub report: ModelReport, pub surfaces: Vec<SurfaceInfo> }
pub struct MergePick { pub scan_id: String, pub run: PathBuf, pub map: PathBuf }
pub fn run_merge_job(project: &Project, picks: &[MergePick], include_unsure: bool, progress: &Progress) -> Result<PathBuf, StepError>
pub fn run_model_surface_job(model: &ModelInfo, settings: &SurfaceSettings, gpu: GpuPref, progress: &Progress) -> Result<SurfaceDone, StepError>
```
- Ordner: `<projekt>/modelle/modell-NNN/`, angelegt mit `create_numbered`, geschützt durch `StepDir`. `project.toml` braucht keinen neuen Eintrag, `models()` liest den Ordner.
- Daten-Schritt:
  - Knopf „Zusammenführen …“, sobald zwei Scans eine fertige Karte haben.
  - Das Feld listet je Scan ein Häkchen und eine Auswahl Lauf/Karte (Muster `compare.rs` ComboBox, Vorgabe die neueste Karte des neuesten Laufs), dazu „Auch unsichere übernehmen“.
  - Der erste angehakte Scan ist der Bezug.
  - Start erst ab zwei Haken.
- Baum: Knoten „Modell 001“.
  - Darunter je Scan eine Zeile: Name, Urteil (✓ sicher / ⚠ unsicher / ✗ gescheitert, mit Grund als Tooltip), Überlappung %, Restfehler mm.
  - Dazu das Blatt `modell.las` und die Oberflächen des Modells.
  - Auswahl lädt `modell.las` und schaltet einmalig auf `ColorMode::Scan`.
- Bilder-Schritt: Karte je Modell mit „Oberfläche rechnen“ und dem Veraltet-Hinweis über `model_surface_fits`.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `merge_job_creates_model_dir` (Projekt mit zwei synthetischen Scans aus Task 2; `modell-001` mit `modell.json`; ein zweiter Lauf ergibt `modell-002`)
  - `cancelled_merge_job_leaves_no_dir`
  - `models_listed_with_verdicts`
  - UI: `merge_button_needs_two_maps`, `merge_dialog_start_needs_two_ticks`, `model_node_shows_verdicts`
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen. **Step 4:** `cargo test -p driftless-app` grün, Schnappschüsse bewusst aktualisieren, clippy sauber, auch `cargo clippy --release -p driftless-app --bins -- -D warnings`.
- [ ] **Step 5:** Commit `feat(app): Scans zusammenführen und Modelle ansehen`.

### Task 7: Punktpaare von Hand (Kern, Schritt, CLI)

**Files:**
- Create: `crates/driftless-steps/src/pairs.rs`
- Modify: `crates/driftless-steps/src/merge.rs` (Paare nutzen), `crates/driftless-cli/src/merge.rs` (`--punktpaare <DATEI>`)
- Test: `crates/driftless-steps/tests/merge.rs`

**Interfaces:**
- Consumes: Task 1 `align_with_pairs`, `pose_from_pairs`; Task 2 `MergeRequest.pairs`.
- Produces:
```rust
pub const POINT_PAIRS_FILE: &str = "punktpaare.toml";   // im Ordner <projekt>/modelle/
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct PointPairEntry {
    pub a: PathBuf, pub b: PathBuf,          // Kartenordner der beiden Scans
    pub points_a: Vec<[f64; 3]>, pub points_b: Vec<[f64; 3]>, // je ≥ 3, jeweils im eigenen Welt-Rahmen des Scans
    pub enabled: bool, pub note: String, pub added: String,
}
pub struct PointPairs { pub entries: Vec<PointPairEntry> }
impl PointPairs {
    pub fn load(path: &Path) -> Result<Self, StepError>   // fehlende Datei = leer
    pub fn save(&self, path: &Path) -> Result<(), StepError> // atomic_write
    pub fn add(&mut self, e: PointPairEntry) -> Result<usize, StepError> // < 3 Paare oder ungleiche Länge = Fehler
    pub fn set_enabled(&mut self, index: usize, enabled: bool) -> Result<(), StepError>
}
```
- Muster `driftless_slam::Corrections`: nichts wird gelöscht, nur ab- und zugeschaltet. Pfade relativ zum Projekt mit `/` wie in `project.toml`.
- Im Merge:
  - Gibt es für ein Paar (i, j) in beliebiger Richtung einen eingeschalteten Eintrag, ersetzt `align_with_pairs` für dieses Paar `align_pair`. Bei umgekehrter Richtung werden die Punkte getauscht.
  - In der Kette schlägt ein Punktpaar-Ergebnis mit Urteil „sicher“ jedes automatische Ergebnis.
  - Ausgeschaltete Einträge zählen nicht und ändern `merge_input_id` nicht.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `pairs_rescue_unsure_member`: Das symmetrische Mitglied aus `unsure_member_left_out` plus drei richtige Paare ergibt „sicher“, `method == "punktpaare"`, Fehler < 2 cm.
  - `disabled_pairs_are_ignored`
  - `pairs_round_trip_and_reject_two_points`
  - CLI: `--punktpaare datei` wirkt.
- [ ] **Step 2–4:** fehlschlagen, bauen, `cargo test --workspace` grün.
- [ ] **Step 5:** Commit `feat: Punktpaare von Hand beim Zusammenführen`.

### Task 8: Punktpaare von Hand in der App

**Files:**
- Modify: `crates/driftless-app/src/ui/steps/merge.rs`, `ui/steps/tree.rs`, `ui/tools.rs` (Werkzeug „Punktpaare“), `ui/app.rs`, `ui/mod.rs`, `ui/text.rs`, `cloud.rs`
- Test: `crates/driftless-app/tests/ui_tools.rs`, `tests/project.rs`

**Interfaces:**
- Consumes: Task 7 `PointPairs`, `PointPairEntry`; Task 6 `ModelInfo`, `run_merge_job` (bekommt die eingeschalteten Paare aus `<projekt>/modelle/punktpaare.toml`); `pick.rs`.
- Ablauf:
  1. Im Baum am unsicheren oder gescheiterten Scan steht „Von Hand ausrichten“.
  2. Die Ansicht zeigt eine gemeinsame `PointSet`: die übernommenen Mitglieder im Modell-Rahmen, dazu den gewählten Scan aus seiner `color.las` in seinem eigenen Rahmen, um die Breite der Modell-Box plus 1 m entlang x versetzt. Farbmodus „je Scan“.
  3. Das Werkzeug fragt abwechselnd „Punkt im Modell“ / „gleicher Punkt im Scan“. Jeder Klick über `pick_point` und `source` zeigt eine nummerierte Marke.
  4. Ab drei Paaren ist „Übernehmen und neu zusammenführen“ aktiv. Die Modellpunkte werden über `T_j⁻¹` des Mitglieds, zu dem der Punkt gehört, in den Rahmen dieses Mitglieds zurückgerechnet, beim Scan der Versatz abgezogen.
  5. Danach `PointPairs::add`, speichern, und ein neuer Merge-Lauf mit denselben Mitgliedern ergibt `modell-NNN+1`.
- Liste der Punktpaare im Modell-Knoten mit Schalter je Eintrag, ab- und zuschaltbar wie bei Schleifen.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `pair_tool_alternates_and_needs_three`
  - `pair_points_mapped_back_to_member_frames`: Rückrechnung mit bekanntem `T_j` und Versatz
  - `pair_toggle_saves`
- [ ] **Step 2–4:** fehlschlagen, bauen, `cargo test -p driftless-app` grün, Schnappschüsse bewusst aktualisieren, clippy sauber (auch Release-Bins).
- [ ] **Step 5:** Commit `feat(app): Punktpaare von Hand setzen`.

### Task 9: Messlauf und Ergebnisbericht (Controller, nicht Subagent)

- [ ] **Step 1:** Zweig auf VM 101 bringen (`driftless-vm sync`) und Release bauen.
  - Testdaten-Ordner so verlinken, dass `jmk6-2026-10-22/2026102213{2520,3028,3554}/driftless` auf die Läufe der Qualitätsrunde zeigen (`~/td/quality/b2`).
  - Ignorierte Tests aus Task 2 laufen lassen.
- [ ] **Step 2:** `driftless merge` der drei Scans (Bezug 133554), dann `driftless surface --model … --voxel 0.01 --gpu an` und `--voxel 0.02`.
  - Messen nach Spec 2a gegen S1 einzeln: Versatz, Wanddicke (`q_analyze.py variant` auf `modell.las`), `seam_step`, ohne Textur %, Löcher, Wände Median, Zeit, Spitzen-Speicher.
  - Gemeinsames Glätten an/aus vergleichen.
- [ ] **Step 3:** Flur-Fall aus dem Büro-Scan (Test `corridor_cut_from_office`) mit Ergebnis.
- [ ] **Step 4:** Schwellen aus Task 1 anhand der Werte belegen oder anpassen (Spec §6), als Ruling ins Ledger.
- [ ] **Step 5:** `docs/ergebnisse/2026-10-27-baustein-1.md` mit Tabellen wie in der Qualitätsrunde, Commit.
- [ ] **Step 6:** App auf dem Mac starten, die drei Scans über die Oberfläche zusammenführen, Bild „je Scan“ und Oberfläche ansehen, Bildschirmfoto für Yannick.
