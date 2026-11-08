# Weniger Rauschen Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Die Oberfläche wird von selbst ruhiger: Ecken legen sich an die Ebene der Messpunkte, die Textur kommt aus gut belichteten Fotos und wird entrauscht, dünne Teile ohne Fläche malen sich nicht mehr auf die Wand dahinter. Jede Maßnahme lässt sich zum Vergleich abschalten.

**Spec:** `docs/superpowers/specs/2026-11-08-weniger-rauschen-design.md` (Maßnahmen, Werte, Begründungen; Abschnitt 11 ist die Prüfung am Schluss).

**Architecture:**
- **Schalter:** `SurfaceConfig.noise: NoiseConfig` (vier bool, Vorgabe an, `NoiseConfig::OFF`). Mit `OFF` entsteht bytegleich, was heute entsteht.
- **M1** `driftless-map/src/surface/snap.rs`: Ecken auf die Ebene der bereinigten Punkte, in `build_mesh` vor `cleanup`. Ein gemeinsamer k-d-Baum für `drop_unsupported`, M1, Genauigkeit.
- **M2** `atlas.rs::choose_labels`: Güte der Ansicht mal Belichtungsgewicht.
- **M3** `surface/denoise.rs`: geführter Filter je Karte, zwischen `bake` und `level_seams`, auch in `bake_region`.
- **M4** `surface/occlude.rs`: Verdecker aus den Punkten fern vom Netz; zweiter Tiefenpuffer je Bild (CPU in `views.rs`, GPU in `faces.wgsl`), verdeckte Ansicht = 4 % Güte + Träger-Merkmal; `verdecker.bin` neben der Oberfläche für „Bilder neu“.

**Tech Stack:** wie bisher (Rust 2021, nalgebra, rayon, kiddo, wgpu). Keine neue Abhängigkeit.

## Vorgaben von Yannick (binden)

- **Schnell bauen.** Keine Reviews je Aufgabe, keine Mess- oder Prüfläufe mit echten Daten zwischendurch. Ein Gesamt-Review und ein Prüflauf ganz am Schluss (Spec Abschnitt 11).
- Unit-Tests nur mit synthetischen Daten. Mac und Windows: keine exakten Gleitkommavergleiche über transzendente Funktionen (Windows-libm rundet anders); feste Prüfsummen nur hinter `#[cfg(all(target_os = "macos", target_arch = "aarch64"))]`, wie in den vorhandenen Tests.

## Global Constraints

- App-Texte deutsch und schlicht, Code-Bezeichner englisch. App klein halten: nur `crates/driftless-app/src/project.rs` (`from_report`). Der parallele Zweig `grundriss` berührt die App stärker.
- Determinismus: geordnete rayon-Maps, Sortierungen mit Gleichstandsregel, keine HashMap-Reihenfolge, keine Zeit, kein Zufall.
- Schwere Läufe mit echten Daten: erst am Schluss, nur auf VM 101 (6 Threads, `nice 10`). Keine schweren Rechenläufe auf dem Mac.
- Scandaten, Bilder aus Projekten und Kundennamen kommen nie ins Repository.
- Vor jedem Commit grün: `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, die Tests der berührten Crates (`cargo test -p <crate>`). Vorher `export PATH=$HOME/.cargo/bin:$PATH`. In Aufgabe 8 zusätzlich `cargo test --workspace`, `cargo clippy --release -p driftless-app --bins -- -D warnings`, `cargo deny check`.
- Commits auf `rauschen` enden mit:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01JqZ6P3km8DjVvZsNNDs9MM
  ```

## Feste Prüfsummen

Die Bytegleichheit zu 0.1.0 entfällt für die Vorgabe (alles an). Der Weg mit `NoiseConfig::OFF` bleibt bytegleich und wird von diesen Tests bewacht. Sie bekommen `noise: NoiseConfig::OFF` in ihrer Einstellung; **ihre Werte bleiben stehen**:

| Test | Datei | Änderung |
|---|---|---|
| `surface::tests::one_source_pages_unchanged` | `crates/driftless-map/src/surface/mod.rs` | `cfg` mit `OFF` |
| `surface::tests::full_surface_unchanged` | `crates/driftless-map/src/surface/mod.rs` | `cfg` im `run` mit `OFF` |
| `surface::edit::retexture::tests::retexture_unchanged` (15 Prüfsummen) | `crates/driftless-map/src/surface/edit/retexture.rs` | `walk_config()` (dort) und `testscan_config()` (`surface/testutil.rs`) mit `OFF` |

Nicht betroffen: `files_unchanged_by_move` (`surface/write.rs`, nur der Schreiber), `color_ply_unchanged_by_move` (`driftless-map/tests/export.rs`), die Konstanten in `driftless-app/src/lod.rs`, `driftless-slam/src/align.rs` und `edit/bvh.rs`.

Die Kommentare in `retexture_unchanged` zu `EDIT_VERSION 2` bleiben richtig (die Werte gehören zum Weg ohne Maßnahmen); ein Satz dazu genügt.

Andere Tests, die mit der Vorgabe genaue alte Kartenzahlen oder Pixel erwarten, bekommen ebenfalls `OFF` samt kurzem Kommentar. Schwellen werden nicht aufgeweicht.

## Review Focus

1. **`OFF` ist wirklich bytegleich:** die drei Tests oben plus Probe im Prüflauf; jeder neue Weg steht hinter seinem Schalter.
2. **M1 verschiebt nur auf Messung zu:** Kanten, dünne Teile, weit entfernte Ecken bleiben (Tests in Aufgabe 2).
3. **M3 mischt keine Karten:** Rechtecke und Masken (Test in Aufgabe 4).
4. **M4 ohne Alternative:** Eine Fläche, die nur verdeckte Ansichten hat, wird nicht grau (Test in Aufgabe 5).
5. **CPU und GPU gleichwertig in M4** (Test in Aufgabe 6).

---

### Task 1: Schalter und Versionen

Lies: `crates/driftless-map/src/surface/mod.rs`, `crates/driftless-steps/src/surface.rs` (`settings_json`, `surface_config`), `crates/driftless-cli/src/surface.rs`, `crates/driftless-app/src/project.rs` (`from_report`).

**Files:**
- Modify: `crates/driftless-map/src/surface/mod.rs`, `crates/driftless-map/src/surface/edit/mod.rs` (`EDIT_VERSION` 3), `crates/driftless-map/src/surface/testutil.rs`, `crates/driftless-map/src/surface/edit/retexture.rs` (nur `walk_config`), `crates/driftless-steps/src/surface.rs` (`SURFACE_VERSION` 14), `crates/driftless-cli/src/surface.rs`, `crates/driftless-app/src/project.rs`

**Interfaces:**
- Produces:
```rust
// driftless-map/src/surface/mod.rs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoiseConfig { pub snap: bool, pub exposure: bool, pub denoise: bool, pub occlude: bool }
impl NoiseConfig { pub const ON: NoiseConfig; pub const OFF: NoiseConfig; }   // Default = ON
// SurfaceConfig { …, pub noise: NoiseConfig }

// driftless-steps/src/surface.rs
pub fn noise_config(settings: &Value) -> Option<NoiseConfig>;   // fehlt der Eintrag: Vorgabe; falscher Typ: None
```
- Bericht: `settings.noise = { "snap": …, "exposure": …, "denoise": …, "occlude": … }`.
- CLI: `--ohne <WAS>[,<WAS>…]` mit `anlegen|belichtung|entrauschen|verdeckung|alles` (clap `ValueEnum`, `value_delimiter = ','`), auch für `--model`.

- [ ] **Step 1: Failing tests:**
  - CLI: `--ohne alles`, `--ohne anlegen,verdeckung`, unbekannter Wert wird abgelehnt.
  - `noise_config` liest die Felder; fehlender Eintrag gibt `ON`; falscher Typ `None`.
  - Bericht schreiben und lesen ergibt dieselbe Einstellung, auch in `SurfaceSettings::from_report` (App).
  - `--ohne alles` ändert die Eingabe-Kennung (`plan_text`).
- [ ] **Step 2: Umsetzen.** Nur Schalter, Bericht, Versionen; die Maßnahmen kommen in den nächsten Aufgaben. Die Einstellungen aus der Tabelle „Feste Prüfsummen“ auf `OFF` setzen.
- [ ] **Step 3:** `cargo test -p driftless-map -p driftless-steps -p driftless-cli -p driftless-app` grün, alle Prüfsummen unverändert.
- [ ] **Step 4:** Commit `feat(oberflaeche): Schalter gegen Rauschen, Oberfläche v14`.

### Task 2: M1 Flächen an die Messpunkte legen

Lies: Spec Abschnitt 4 M1; `crates/driftless-map/src/smooth.rs` (`project`), `surface/mod.rs` (`build_mesh`, `drop_unsupported`, `accuracy`), `surface/testutil.rs`.

**Files:**
- Create: `crates/driftless-map/src/surface/snap.rs`
- Modify: `crates/driftless-map/src/smooth.rs` (`project` wird `pub(crate)`), `crates/driftless-map/src/surface/mod.rs`, `crates/driftless-steps/src/surface.rs` (`stats_json`)

**Interfaces:**
- Produces:
```rust
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SnapStats { pub vertices: usize, pub moved: usize, pub no_plane: usize, pub too_far: usize, pub reverted: usize, pub mean_shift_mm: f64 }
pub(crate) fn snap_to_points(mesh: &mut Mesh, tree: &ImmutableKdTree<f32, 3>, xyz: &[[f32; 3]], cfg: &SurfaceConfig) -> SnapStats;
// SurfaceStats { …, pub snap: SnapStats, pub roughness: Accuracy }
// AccuracySplit { …, pub walls_plane: Accuracy }
```
- Ablauf in `build_mesh`: `extract` → `drop(sdf)` → k-d-Baum der Punkte → `drop_unsupported` → M1 (`cfg.noise.snap`) → `cleanup` → `simplify` → Genauigkeit mit demselben Baum.
- `walls_plane`: je Wand-Ecke (`flat_vertices`) Abstand zur Ebene der Punkte im Umkreis von 3 cm (mindestens 10 Punkte, eben; sonst ausgelassen); auch gegen `dense_xyz`.
- `roughness`: je Wand-Ecke Betrag von `(Ecke − Mittel der Nachbarn) · Eckennormale` (mm); wird immer berechnet, auch mit `snap` aus.

- [ ] **Step 1: Failing tests** (`snap.rs` und `surface/mod.rs`):
  - `noisy_wall_gets_flat`: Wand aus Punkten mit 3 mm Rauschen entlang der Normalen, `build_mesh` mit und ohne `snap`: `roughness` mindestens halbiert, `walls_plane` (95 %) kleiner.
  - `room_corner_stays`: `box_room`: Ecken des Netzes an den Raumkanten höchstens 1 Voxel neben ihrer Lage ohne `snap`.
  - `thin_plate_stays`: zwei parallele Flächen 2 cm auseinander: nicht zusammengezogen.
  - `far_vertex_stays`, `too_few_points_stays`, `flipped_face_reverted`.
  - `snap_is_deterministic`: Punkte in anderer Reihenfolge, gleiches Netz.
  - `snap_off_keeps_mesh`: mit `snap` aus gleiche Lagen und Indizes wie ohne Aufruf.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS. Bestehende Tests von `build_mesh` bleiben grün.
- [ ] **Step 5:** Kennzahlen in `stats_json`. Commit `feat(oberflaeche): Flächen an die Messpunkte legen`.

### Task 3: M2 Gut belichtete Bilder bevorzugen

Lies: Spec Abschnitt 4 M2; `surface/atlas.rs` (`choose_labels`), `surface/views.rs` (`FaceView`).

**Files:**
- Modify: `crates/driftless-map/src/surface/atlas.rs`

**Interfaces:**
- Produces:
```rust
pub const GOOD_LEVEL: f32 = 64.0;
pub const MIN_EXPOSURE_WEIGHT: f32 = 0.25;
pub const CLIPPED_LEVEL: u8 = 250;
pub const CLIPPED_WEIGHT: f32 = 0.5;
pub(crate) fn exposure_weight(colour: [u8; 3]) -> f32;   // L = (R + 2G + B) / 4; clamp(L / GOOD_LEVEL, MIN, 1), ausgefressen CLIPPED_WEIGHT
```
- `choose_labels`: mit `cfg.noise.exposure` wirkt `score · exposure_weight(colour)` in der Startwahl und in den Kosten (bezogen auf die beste gewichtete Ansicht). Gilt auch, wenn `smooth_weight` 0 ist. Ohne den Schalter ändert sich nichts.

- [ ] **Step 1: Failing tests:**
  - `dark_view_loses_to_brighter`: zwei gleich gute Ansichten, eine mit `L = 15`, eine mit `L = 60`: die helle gewinnt.
  - `sharp_view_can_still_win`: dunkle Ansicht mit 16-facher Güte gewinnt trotzdem.
  - `clipped_view_loses`, `bright_scene_unchanged` (alle `L ≥ 64`: dieselben Labels), `exposure_off_keeps_labels`.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** Commit `feat(oberflaeche): gut belichtete Bilder bevorzugen`.

### Task 4: M3 Bildrauschen dämpfen

Lies: Spec Abschnitt 4 M3; `surface/atlas.rs` (`for_each_texel`, `fill_rest`, `ChartInfo`, `bake_region`), `surface/level.rs` (Modulkopf), `surface/edit/retexture.rs` (um Zeile 1393).

**Files:**
- Create: `crates/driftless-map/src/surface/denoise.rs`
- Modify: `crates/driftless-map/src/surface/mod.rs` (`texture_mesh`), `crates/driftless-map/src/surface/atlas.rs` (`TextureStats`), `crates/driftless-map/src/surface/edit/retexture.rs`, `crates/driftless-steps/src/surface.rs` (`stats_json`)

**Interfaces:**
- Produces:
```rust
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct DenoiseStats { pub noise_before: f64, pub noise_after: f64 }
/// Jede Karte von `tex` für sich; `image_of(c)`: das Foto der Karte c (für Karten, die zu klein für eine eigene Schätzung sind).
pub(crate) fn denoise_charts(mesh: &Mesh, tex: &mut TexturedMesh, image_of: impl Fn(usize) -> u32 + Sync) -> DenoiseStats;
// TextureStats { …, pub noise_before: f64, pub noise_after: f64 }
```
- Aufruf: `texture_mesh` nach `bake`, vor `level_seams`; `retexture` nach `bake_region`, vor `level_region`; beide nur mit `cfg.noise.denoise`.
- Kacheln von höchstens 512² Texeln mit Halo, damit der Speicher begrenzt bleibt. Nur `+ − × ÷`.

- [ ] **Step 1: Failing tests** (synthetische Karten, Rauschen aus `testutil::Lcg`):
  - `flat_noise_goes_down`: flache Karte mit `σ = 6`: nachher höchstens die Hälfte.
  - `edges_stay_sharp`: Streifen 2 Texel breit, Kontrast 40 Stufen: mindestens 80 % des Kontrasts bleiben.
  - `chroma_noise_goes_down_more`: Farbrauschen sinkt stärker als Helligkeitsrauschen.
  - `clean_chart_untouched`: `σ < 1`: Bytes gleich.
  - `charts_do_not_mix`: zwei Karten nebeneinander mit verschiedenen Farben: keine Farbe der einen in der anderen; Grau-Block und Texel außerhalb der Dreiecke unverändert.
  - `small_chart_uses_image_noise`, `denoise_is_deterministic`.
  - `texture_mesh_seams_no_worse`: Walk-Szene mit `denoise` an und aus: `seam_step_after` nicht größer, `noise_after < noise_before` bei verrauschten Fotos.
  - `retexture_is_denoised`: `retexture_region` auf verrauschten Fotos: neue Seite weniger Rauschen; andere Seiten unverändert.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** Kennzahlen in `stats_json`. Commit `feat(oberflaeche): Bildrauschen je Karte dämpfen`.

### Task 5: M4 Verdecker finden und auf dem Prozessor anwenden

Lies: Spec Abschnitt 4 M4; `surface/views.rs` (`image_views`, `Pass`, `cpu_views`, `filter_views`, Tag), `crates/driftless-map/src/visibility.rs` (`DepthBuffer`, `PointGrid`), `surface/simplify.rs` (`far_vertices`).

**Files:**
- Create: `crates/driftless-map/src/surface/occlude.rs`
- Modify: `crates/driftless-map/src/surface/views.rs`, `crates/driftless-map/src/visibility.rs`, `crates/driftless-map/src/surface/simplify.rs` (`far_vertices` teilt sich die Suche mit Punkten), `crates/driftless-map/src/surface/mod.rs`, `crates/driftless-steps/src/surface.rs` (`make_surface`, `stats_json`)

**Interfaces:**
- Produces:
```rust
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Occluders { pub points: Vec<[f32; 3]>, pub spacing: f32 }
impl Occluders { pub fn is_empty(&self) -> bool; }
pub fn find_occluders(mesh: &Mesh, xyz: &[[f32; 3]], cfg: &SurfaceConfig) -> Occluders;
pub const OCCLUDER_GAP: f64 = 0.02;          // näher als die Fläche um mehr als das: verdeckt
pub const OCCLUDED_SCORE: f32 = 0.04;        // Anteil der Güte einer verdeckten Ansicht
pub const OCCLUDER_MIN_EXTENT: f64 = 0.30;
pub const OCCLUDER_MAX: usize = 1_500_000;

/// `texture_mesh` mit Verdeckern; `texture_mesh` selbst ruft dies ohne Verdecker auf (Tests bleiben).
pub fn texture_mesh_with(mesh: &Mesh, occluders: &Occluders, sources: &[TextureSource], cfg: &SurfaceConfig, progress: &Progress, stats: &mut SurfaceStats, gpu: Option<&GpuContext>) -> Result<TexturedMesh, MapError>;
// DepthBuffer::is_behind(&self, p: &Projection, gap: f64) -> bool   // nur eigene Zelle
// SurfaceStats { …, pub occluders: usize }
```
- `build_surface` und `make_surface` (vor `drop(points)`): mit `cfg.noise.occlude` `find_occluders`, dann `texture_mesh_with`.
- Verdeckt: eine von 7 Proben (Mitte, 3 Ecken, 3 Kantenmitten) `is_behind`. Dann `score · OCCLUDED_SCORE` und `TAG_CARRIER`; mehr ändert sich nicht, `filter_views` wirft die Ansicht, sobald es eine andere gibt. In dieser Aufgabe läuft M4 nur auf dem Prozessor; `--gpu` ignoriert Verdecker bis Aufgabe 6.

- [ ] **Step 1: Failing tests:**
  - `find_occluders`: Stab vor einer Wand, ohne Fläche: wird gefunden; Punkte auf der Fläche nicht; Gruppe unter 30 cm nicht; Zelle mit 2 Punkten nicht; zu viele: höchstens `OCCLUDER_MAX`; gleiches Ergebnis in anderer Punktreihenfolge.
  - `rod_is_not_painted_on_wall`: Szene wie `thin_occluder_not_baked` (Stab in den Fotos, nicht im Netz, in den Punkten): ohne Verdecker liegt der Stab auf der Wand, mit Verdeckern die Wandfarbe.
  - `only_occluded_views_stay`: Fläche, die nur von verdeckten Ansichten gesehen wird: behält ein Bild, wird nicht grau.
  - `face_in_front_of_occluder_is_seen`, `no_occluders_same_views`.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** `occluders` in `stats_json`. Commit `feat(oberflaeche): Teile ohne Fläche verdecken die Wand dahinter`.

### Task 6: M4 auf der Grafikkarte

Lies: `crates/driftless-gpu/src/faces.rs`, `crates/driftless-gpu/src/faces.wgsl`, `crates/driftless-map/src/surface/views_gpu.rs`, `crates/driftless-gpu/tests/faces.rs`.

**Files:**
- Modify: `crates/driftless-gpu/src/faces.rs`, `crates/driftless-gpu/src/faces.wgsl`, `crates/driftless-map/src/surface/views_gpu.rs`, `crates/driftless-map/src/surface/views.rs`
- Test: `crates/driftless-gpu/tests/faces.rs`, Modul-Tests in `views.rs`

**Interfaces:**
- `FaceViews::new` bekommt die Verdecker (Punkte relativ zum Ursprung, `spacing`) und `FaceParams.occluder_gap`, `occluded_score`.
- `faces.wgsl`: neuer Speicher für Verdecker (`array<vec4<f32>>`) und ein zweiter Tiefenpuffer gleicher Größe, eigener Compute-Einstieg `splat_occluders_main` (gleiche Regel wie `splat_point`, aber in den zweiten Puffer). `views_main` prüft 7 Proben wie die CPU. Ohne Verdecker: kein Durchlauf, ein leerer Puffer.
- `faces_pass_bytes` zählt beides mit.

- [ ] **Step 1: Failing tests** (übersprungen ohne Grafikadapter, Muster `partly_hidden_face_rejected_gpu`):
  - `occluded_view_is_tagged_gpu`: Stab vor Fläche: dieselben Ansichten als verdeckt markiert wie auf der CPU.
  - `rod_is_not_painted_on_wall_gpu`: wie Aufgabe 5.
  - `no_occluders_same_views_gpu`.
  - Speicherbudget: `faces_pass_bytes` wächst um die Verdecker.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS (`cargo test -p driftless-gpu -p driftless-map`).
- [ ] **Step 5:** Commit `feat(gpu): Verdecker im Tiefentest der Flächen`.

### Task 7: Verdecker für „Bilder neu“

Lies: `crates/driftless-steps/src/surface.rs` (`owns`, `make_surface`, `files`), `crates/driftless-steps/src/touchup.rs` (um Zeile 700), `crates/driftless-map/src/surface/edit/retexture.rs` (`retexture_region`, `region_views`), `crates/driftless-map/src/surface/views.rs` (`windowed_views`).

**Files:**
- Modify: `crates/driftless-map/src/surface/occlude.rs`, `crates/driftless-map/src/surface/views.rs`, `crates/driftless-map/src/surface/edit/retexture.rs`, `crates/driftless-steps/src/surface.rs`, `crates/driftless-steps/src/touchup.rs`

**Interfaces:**
- Produces:
```rust
pub const OCCLUDERS_FILE: &str = "verdecker.bin";
impl Occluders {
    pub fn write(&self, path: &Path) -> Result<(), MapError>;   // atomar: Kennung, spacing, Anzahl, f32-Punkte
    pub fn read(path: &Path) -> Option<Occluders>;              // fehlt oder kaputt: None
}
// retexture_region(…, occluders: &Occluders, …)
```
- `make_surface` schreibt die Datei nur mit `noise.occlude` und nicht leeren Verdeckern; sie steht in `files` und `bytes`; `owns` kennt sie. Ohne Datei: wie heute (`Occluders::default()`).
- `windowed_views` prüft die Verdeckung wie `image_views`, mit einem Fensterpuffer. „Bilder neu“ berücksichtigt das Merkmal schon (Modulkopf `retexture.rs`, Schritt 2).

- [ ] **Step 1: Failing tests:**
  - Datei schreiben und lesen ergibt dieselben Punkte; kaputte Datei: `None`.
  - `surface_fits` ist falsch, wenn `verdecker.bin` fehlt oder eine andere Größe hat; mit `noise.occlude` aus wird keine Datei geschrieben.
  - `retexture_keeps_rod_off_the_wall`: Stab wie in Aufgabe 5, `retexture_region` mit Verdeckern.
  - `run_touchup` liest die Datei; ohne Datei läuft alles wie vorher.
  - Modelloberfläche schreibt die Datei ebenso.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** Commit `feat(nacharbeit): Verdecker auch beim Bilder-neu-Einspielen`.

### Task 8: Werkzeug für den Prüflauf, Reliefbild, Doku

Lies: Spec Abschnitt 11; `tools/qualitaet/run_vorlagen.sh`, `tools/qualitaet/q_images.sh`, `tools/qualitaet/q_table.py`, `crates/driftless-steps/tests/quality_real.rs`, `crates/driftless-map/src/surface/preview.rs`.

**Files:**
- Create: `tools/qualitaet/run_rauschen.sh`
- Modify: `crates/driftless-map/src/surface/preview.rs` und `surface/mod.rs` (`render_relief`), `crates/driftless-steps/tests/quality_real.rs` (`QA_RELIEF`), `tools/qualitaet/q_table.py`, `tools/qualitaet/README.md`, `docs/anleitung.md`, `CHANGELOG.md`

**Interfaces:**
- `pub fn render_relief(tex: &TexturedMesh, eye: [f32; 3], target: [f32; 3], fov_deg: f32, size: [u32; 2]) -> RgbImage` – dieselbe Kamera wie `render_preview`, Grau mit Licht von schräg nach Flächennormale, ohne Textur. Nur für Prüfbilder.
- `run_rauschen.sh <label>`: nach dem Muster von `run_vorlagen.sh` alle Läufe der Tabelle in Spec Abschnitt 11 (`--ohne` je Variante, Laufbahn und Karte einmal je Scan, Modell über `driftless merge`), abgekoppelt startbar; Bilder mit `QA_B_SURF` nebeneinander.
- `q_table.py`: neue Zeilen für `roughness`, `walls_plane`, `noise_before/after`, `snap`, `occluders`.

- [ ] **Step 1:** Test `render_relief_shows_bumps` (`preview.rs`): zwei Flächen in verschiedener Richtung haben verschiedenes Grau; ohne Netz der Hintergrund.
- [ ] **Step 2:** Umsetzen; Skript und Tabelle anpassen (nichts davon läuft jetzt auf echten Daten).
- [ ] **Step 3:** `docs/anleitung.md`: kurzer Abschnitt, was die Oberfläche jetzt von selbst macht, und `--ohne`. `CHANGELOG.md` unter `[Unreleased]`. Bekannte Grenze: kleine Teile und Saum an Kanten.
- [ ] **Step 4:** `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo clippy --release -p driftless-app --bins -- -D warnings`, `cargo deny check` grün.
- [ ] **Step 5:** Commit `docs: weniger Rauschen, Prüfwerkzeug für den Lauf am Schluss`.

---

## Danach (nicht Teil dieser Aufgaben)

Prüflauf auf VM 101 nach Spec Abschnitt 11, Bericht `docs/ergebnisse/2026-11-08-weniger-rauschen.md`, danach ein Gesamt-Review über den Zweig; beides erst, wenn alle Aufgaben fertig sind. Der Zweig `grundriss` wird anschließend mit diesem zusammengeführt.
