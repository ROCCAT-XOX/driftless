# Baustein 4: Neue Ansicht – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Das Programm bekommt den freigegebenen Aufbau B. Links eine schmale Leiste mit den drei Schritten
1 Daten · 2 Bilder · 3 Planung, daneben das Feld des Schritts, schwebende Werkzeuge über der 3D-Ansicht, ein
modernes dunkles Thema und Punkte als farbige Flecken in Metergröße.

**Architecture:**
- Nur `crates/driftless-app` ändert sich, die Rechenschritte bleiben unberührt.
- Ein neues `theme.rs` legt Farben, Abstände und Rundungen an einer Stelle fest.
- Ein `Step`-Zustand in `AppState` bestimmt, welches Feld links gezeigt wird und welche Ansicht gilt.
- Die heutigen Felder `left.rs`/`right.rs` werden auf `steps/{daten,bilder,planung}.rs` verteilt.
- Selten gebrauchte Regler wandern unter „Erweitert“ (je Schritt) oder ins Fenster ⚙ (für das ganze Programm).
- Der Punkt-Shader bekommt eine Größe in Metern mit Grenzen in Pixeln.

**Tech Stack:** Rust, egui/eframe 0.36, egui-wgpu, wgpu 30, egui_kittest (UI- und Bildtests).

**Spec:** `docs/superpowers/specs/2026-10-24-ansicht-design.md`
(Entwurf: `.superpowers/brainstorm/40119-1792835473/content/layout-b-v2.html`)

## Global Constraints

- Nur `crates/driftless-app` (+ `README.md`, `docs/`). `driftless-map`, `driftless-steps` und `driftless-cli`
  ändern sich nicht. Der Zweig `baustein-gpu` arbeitet parallel daran.
- Alle sichtbaren Texte sind Deutsch und stehen in `src/ui/text.rs`. Kommentare sind Englisch.
- Keine Funktion geht verloren: Projekt anlegen/öffnen, Scan/Lauf hinzufügen, Oberfläche öffnen, Karte und
  Oberfläche rechnen, Abbrechen, Korrekturen (Schleifen), Vergleich, Teilkarten, verworfene Kandidaten,
  Schnitt, Punkt anklicken, Protokoll – notfalls unter „Erweitert“ oder ⚙.
- Die Geschwindigkeit bleibt: gleiches Punktbudget, gleiche LOD, keine zusätzlichen Durchläufe über alle
  Punkte je Bild.
- Farben nur aus `theme.rs`. Dunkles Thema: Grund ≈ `#0f1115`, Felder ≈ `#151920`, Flächen ≈ `#1b1f27`,
  Akzent ≈ `#2f6fed`, Rundung 8–12 px.
- Neue Crates nur mit Lizenz MIT/Apache-2.0/OFL-1.1/Zlib. `cargo deny check` bleibt grün.
- Kein schwerer Rechenlauf auf dem Mac. Tests nutzen synthetische Szenen, Ansichtstests nutzen vorhandene
  Ergebnisse.
- Genauigkeit (Schritt 1) steuert nur das Raster der Karte: **Schnell = 0,02 m, Genau = 0,01 m (heutige
  Vorgabe), Maximal = 0,005 m**. Bei „Maximal“ steht der Hinweis „braucht bei großen Aufnahmen über 4 GB
  Arbeitsspeicher“. Ein Wert, der keiner Stufe entspricht (aus „Erweitert“), zeigt „Eigene“.
- `cargo test -p driftless-app`, `cargo clippy -p driftless-app --all-targets -- -D warnings` und `cargo fmt`
  müssen grün sein.
- Commit-Trailer:
  ```
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01EaCFfju4adDubj5wkFzZX4
  ```

## Review Focus

1. **Leeres Projekt oder gar kein Projekt:** Jeder Schritt zeigt einen freundlichen Leerzustand mit dem
   nächsten Handgriff („Projekt öffnen“, „Scan hinzufügen“). Er zeigt keine leere Fläche und keine
   ausgegrauten Knöpfe ohne Erklärung. Test in Task 2.
2. **Schrittwechsel, während ein Auftrag läuft:** Fortschritt und Abbrechen bleiben in der Statusleiste
   sichtbar, und der Auftrag läuft weiter. Test in Task 3.
3. **Kleines Fenster (1024 × 640) und lange Scan-Namen:** Nichts überlappt, und Namen werden mit „…“
   gekürzt. Test in Task 6.
4. **Flecken sehr nah oder sehr fern:** Die Größe bleibt zwischen den Grenzen in Pixeln. Es entstehen keine
   riesigen Flecken vor der Kamera und keine Punkte, die verschwinden. Test in Task 5.
5. **Alte gespeicherte Einstellungen:** Der erste Start nach dem Update stürzt nicht ab und zeigt keinen
   kaputten Aufbau. Alter gespeicherter Fensterzustand wird verworfen. Test in Task 2.

---

### Task 1: Thema

**Files:**
- Create: `crates/driftless-app/src/ui/theme.rs`
- Modify: `crates/driftless-app/src/ui/mod.rs` (`pub mod theme`), `src/ui/app.rs` (`theme::apply(&ctx)` einmal
  beim Start), `Cargo.toml`, falls eine Symbol- oder Schrift-Crate dazukommt
- Test: `crates/driftless-app/tests/ui_theme.rs`

**Interfaces:**
- Produces:
  - `pub struct Palette { pub bg, pub panel, pub surface, pub surface_hi, pub accent, pub text, pub text_dim, pub ok, pub warn, pub error: egui::Color32 }`
  - `pub const DARK: Palette`
  - `pub fn apply(ctx: &egui::Context)`, die Visuals und Spacing setzt
  - `pub mod icon` mit `&'static str`-Konstanten `MENU, DATA, IMAGES, PLAN, SETTINGS, ORBIT, PAN, PICK, MEASURE, CUT, FIT, CHECK, RUNNING`
  - `pub fn float_frame() -> egui::Frame`, halbdurchsichtig mit Rundung 12 px, für die schwebenden Leisten
  - `pub fn view_background(painter: &egui::Painter, rect: egui::Rect)`, der weiche Verlauf hinter der Ansicht
- Bei den Symbolen: eine Symbolschrift (z. B. `egui-phosphor`), wenn ihre Lizenz erlaubt ist. Sonst Unicode-Zeichen,
  die die eingebaute Schrift hat. Das Ergebnis steht im Report.

- [ ] **Step 1: Write the failing tests**
  - `theme_sets_dark_visuals`: Nach `apply` gilt `ctx.style().visuals.dark_mode == true`. `panel_fill == DARK.panel`.
    `widgets.inactive.corner_radius` liegt zwischen 8 und 12. `selection.bg_fill == DARK.accent`.
  - `icons_render_in_font`: Jede Konstante in `icon` wird mit den Schriften aus `apply` ohne Ersatzzeichen
    gesetzt. Prüfung über `ctx.fonts(|f| f.has_glyphs(...))`.
- [ ] **Step 2: Run to verify they fail**
  - Run: `~/.cargo/bin/cargo test -p driftless-app --test ui_theme`
  - Expected: FAIL, weil `theme` nicht existiert.
- [ ] **Step 3: Implement**
  - `theme.rs` umsetzen, nah an den Werten des Entwurfs.
  - `apply` wird in `DriftlessApp::from_creation_context` und im Test-Harness (`DriftlessApp::new`)
    aufgerufen. Danach gilt es für alle bestehenden Felder.
  - `view_background` wird in `view3d` gezeichnet, bevor die Wolke gerendert wird. Die Clear-Farbe wird
    transparent, oder der Verlauf wird als Shader-Hintergrund gezeichnet: Entscheidung des Bauenden, ohne
    zusätzlichen Durchlauf über die Punkte.
- [ ] **Step 4: Run to verify they pass**, plus `~/.cargo/bin/cargo test -p driftless-app` (alles grün).
- [ ] **Step 5: Commit** `feat(app): dunkles Thema und Symbole`

### Task 2: Aufbau mit Schrittleiste

**Files:**
- Create: `crates/driftless-app/src/ui/rail.rs` (Leiste), `src/ui/steps/mod.rs`, `src/ui/steps/daten.rs`, `src/ui/steps/bilder.rs`,
  `src/ui/steps/planung.rs` (zunächst die heutigen Inhalte verteilt, siehe unten), `src/ui/settings.rs` (Fenster ⚙)
- Modify: `src/ui/mod.rs` (`Step`, `AppState.step`), `src/ui/app.rs` (`show`: statt Menüleiste plus linkem und rechtem Feld
  jetzt Leiste plus Schrittfeld plus Ansicht plus Statusleiste), `src/ui/left.rs`, `src/ui/right.rs` (aufgehen
  in den Schrittfeldern bzw. gelöscht), `src/ui/text.rs`
- Test: `crates/driftless-app/tests/ui_layout.rs`; bestehende `ui_*.rs` an den neuen Aufbau anpassen

**Interfaces:**
- Produces:
  - `#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, serde::Serialize, serde::Deserialize)] pub enum Step { #[default] Daten, Bilder, Planung }`
  - `AppState.step: Step`
  - `Action::SetStep(Step)`
  - `pub fn rail::show(ui, state) -> Option<Action>`
  - `pub fn steps::show(ui, state) -> Option<Action>`, die nach `state.step` verteilt
  - `pub struct SettingsWindow { pub open: bool }` in `AppState`
  - `pub fn settings::show(ctx, state)`
- Regel aus dem Spec: `Action::SetStep(s)` setzt die Ansicht neu.
  - `Daten` und `Planung` → `ShowMode::Cloud`.
  - `Bilder` → `ShowMode::Surface`, wenn eine Oberfläche geladen ist oder geladen wird. Sonst bleibt `Cloud`
    mit Hinweis.
  - Umschalten von Hand gilt bis zum nächsten Schrittwechsel.
- Erste Verteilung:
  - Daten: Projektbaum (Scans/Läufe/Karten), Karte rechnen, Korrekturen, Vergleich.
  - Bilder: Oberflächen, Oberfläche rechnen.
  - Planung: Leerzustand.
  - ⚙: Punktgröße, rund, Punktbudget, Farbart, Bedienung (Umkreisen/Fliegen), Ebenen (Laufbahn, Odometrie,
    Schleifen, verworfene Kandidaten, Teilkarten, Vergleich A).
  - ☰ öffnet ein kleines Menü mit Neues Projekt, Projekt öffnen, Scan hinzufügen, Lauf hinzufügen,
    Oberfläche öffnen.
- Die Feinarbeit an Daten und Bilder folgt in Task 3/4.
- Persistenz: Der Schlüssel des gespeicherten UI-Zustands bekommt eine neue Version (z. B. `"ui-v2"`). Ein
  alter Zustand wird ignoriert, nicht gelesen.

- [ ] **Step 1: Write the failing tests** (egui_kittest, wie die bestehenden `ui_*.rs`)
  - `rail_switches_step`: Klick auf „Bilder“ in der Leiste → `state().step == Step::Bilder`. Das Feld zeigt die
    Überschrift `text::STEP_IMAGES`.
  - `step_sets_show_mode`: Oberfläche geladen. `SetStep(Bilder)` → `ShowMode::Surface`. Von Hand auf `Cloud`
    umschalten, dann `SetStep(Daten)` und `SetStep(Bilder)` → wieder `Surface`. Ohne Oberfläche
    `SetStep(Bilder)` → `Cloud`, und `text::NO_SURFACE_YET` ist sichtbar.
  - `menu_has_all_project_actions`: Klick auf ☰ zeigt alle fünf Einträge. Jeder löst die bisherige `Action` aus.
  - `settings_window_has_view_settings`: Klick auf ⚙ zeigt Punktgröße, Punktbudget, Farbart, Bedienung und
    Ebenen. Eine Änderung der Punktgröße landet in `state().view.point_px`.
  - `empty_states_explain_next_step`: Ohne Projekt zeigt jeder Schritt einen Hinweis und einen Knopf zum
    nächsten Handgriff (`text::EMPTY_NO_PROJECT`). Mit Projekt ohne Scans zeigt Daten `text::EMPTY_NO_SCANS`
    und einen Knopf „Scan hinzufügen“. *(Review Focus 1)*
  - `old_stored_layout_is_ignored`: `restore` mit einem gespeicherten Zustand im alten Format oder unter
    altem Schlüssel → kein Absturz, `step == Daten`, Standard-Ansicht. *(Review Focus 5)*
- [ ] **Step 2: Run to verify they fail**
  - Run: `~/.cargo/bin/cargo test -p driftless-app --test ui_layout`
  - Expected: FAIL.
- [ ] **Step 3: Implement**
  - Leiste ≈ 60 px mit Symbolen und kurzer Beschriftung darunter, wie im Entwurf. Aktiv = Akzent.
  - Schrittfeld ≈ 250 px, veränderbar, min. 220.
  - Bestehende Tests anpassen, die nach alten Überschriften oder Menüs suchen. Ihr Inhalt bleibt gleich, nur
    der Weg dorthin ändert sich. Kein Test wird ersatzlos gelöscht.
- [ ] **Step 4: Run** `~/.cargo/bin/cargo test -p driftless-app` → alles grün. Clippy und fmt sind sauber.
- [ ] **Step 5: Commit** `feat(app): Schrittleiste Daten · Bilder · Planung`

### Task 3: Schritt 1 „Daten“

**Files:**
- Modify: `src/ui/steps/daten.rs`, `src/ui/text.rs`, `src/ui/mod.rs` (Warteschlange), `src/ui/app.rs` (Aufträge nacheinander)
- Test: `crates/driftless-app/tests/ui_daten.rs`

**Interfaces:**
- Consumes: `Step` und `steps::show` (Task 2), das heutige `MapStatus`/`set_map_status`, die Karten-Einstellungen
  (`switches.voxel`) und die heutige Aktion „Karte rechnen“.
- Produces:
  - `pub enum Precision { Schnell, Genau, Maximal, Eigene }`
  - `pub fn precision_of(voxel: f64) -> Precision`
  - `pub fn voxel_of(p: Precision) -> Option<f64>`, mit den Werten aus Global Constraints
  - `Action::ComputeAllMaps`
  - `AppState.queue: VecDeque<PathBuf>` (Scans, die noch gerechnet werden)

- [ ] **Step 1: Write the failing tests**
  - `precision_maps_to_voxel`:
    - `voxel_of(Schnell) == Some(0.02)`, `voxel_of(Genau) == Some(0.01)`, `voxel_of(Maximal) == Some(0.005)`.
    - `precision_of(0.01) == Genau`, `precision_of(0.013) == Eigene`.
  - `scan_list_shows_status`:
    - Synthetisches Projekt mit drei Scans: keiner gerechnet, einer fertig, einer veraltet.
    - Die Liste zeigt je Scan den Namen und `text::STATUS_NEW` / `STATUS_DONE` / `STATUS_STALE`.
  - `compute_all_queues_new_and_stale`:
    - `ComputeAllMaps` stellt genau die neuen und veralteten Scans in die Warteschlange, in Baum-Reihenfolge.
    - Es läuft immer nur ein Auftrag. Ist er fertig, startet der nächste. Abbrechen leert die Schlange.
  - `maximal_shows_memory_hint`: „Maximal“ gewählt → `text::PRECISION_MAX_HINT` ist sichtbar.
  - `job_survives_step_switch`:
    - Auftrag läuft, dann `SetStep(Bilder)`.
    - Der Auftrag läuft weiter. Statusleiste mit Fortschritt und Knopf „Abbrechen“ sind sichtbar. *(Review Focus 2)*
  - `advanced_holds_switches_compare_correct`:
    - „Erweitert“ ist zu → kein Schalter „Geister“, kein Vergleich, keine Korrekturen sichtbar.
    - Aufgeklappt → alle sichtbar und bedienbar.
- [ ] **Step 2: Run to verify they fail**
  - Run: `~/.cargo/bin/cargo test -p driftless-app --test ui_daten`
  - Expected: FAIL.
- [ ] **Step 3: Implement**, nach dem Entwurf:
  - Überschrift „Daten“, darunter gedämpft Projektname und Scananzahl.
  - Scan-Karten mit Status-Symbol und Fortschritt.
  - „+ Scan hinzufügen“.
  - Genauigkeit als Dreifach-Umschalter.
  - Hauptknopf „Alle rechnen“, bzw. „Rechnen“, wenn ein Scan gewählt ist und nur der gerechnet werden soll.
  - „▸ Erweitert“ mit den heutigen Schaltern (Geister, Ausreißer, …, Voxel als Zahl, Verkleinern),
    Korrekturen, Vergleich, Teilkarten und Kandidaten.
  - Läufe und Karten eines Scans sind aufklappbar unter dem Scan, nicht als eigener Baum.
- [ ] **Step 4: Run** `~/.cargo/bin/cargo test -p driftless-app` → grün.
- [ ] **Step 5: Commit** `feat(app): Schritt Daten mit Genauigkeit und „Alle rechnen“`

### Task 4: Schritt 2 „Bilder“ und Schritt 3 „Planung“

**Files:**
- Modify: `src/ui/steps/bilder.rs`, `src/ui/steps/planung.rs`, `src/ui/text.rs`, `src/ui/app.rs`
- Test: `crates/driftless-app/tests/ui_bilder.rs`

**Interfaces:**
- Consumes:
  - `SurfaceInfo` mit stale, die Oberflächen-Einstellungen (`SURFACE_VOXEL_CHOICES`, „ohne Fotos“) und
    „Oberfläche rechnen“ aus Baustein 2.
  - Die Warteschlange aus Task 3. Die Liste der Messungen in Planung kommt erst mit Task 6.
- Produces: `Action::ComputeAllSurfaces`, das je Karte ohne aktuelle Oberfläche einen Auftrag einreiht.

- [ ] **Step 1: Write the failing tests**
  - `surfaces_listed_per_map`: Die Oberflächen stehen unter ihrer Karte, die veralteten mit
    `text::SURFACE_STALE`. Klick lädt die Oberfläche.
  - `compute_all_surfaces_queues_missing`: Es werden nur Karten ohne aktuelle Oberfläche eingereiht. Die
    Feinheit aus dem Umschalter wird übernommen.
  - `no_photos_is_advanced`: „ohne Fotos“ ist nur unter „Erweitert“ sichtbar.
  - `planning_shows_placeholder`: Planung zeigt `text::PLAN_COMING` (Grundriss, Maße, DWG folgen).
- [ ] **Step 2: Run to verify they fail**
  - Run: `~/.cargo/bin/cargo test -p driftless-app --test ui_bilder`
  - Expected: FAIL.
- [ ] **Step 3: Implement** im selben Aufbau wie Daten: Überschrift, Karten-Liste, Feinheit 1 / 1,5 / 2 cm,
  Hauptknopf „Alle Oberflächen rechnen“ und „▸ Erweitert“.
- [ ] **Step 4: Run** `~/.cargo/bin/cargo test -p driftless-app` → grün.
- [ ] **Step 5: Commit** `feat(app): Schritte Bilder und Planung`

### Task 5: Punkte als Flecken in Metergröße

**Files:**
- Modify: `crates/driftless-app/src/render/mod.rs` (`FrameParams`), `src/render/points.wgsl`, `src/ui/mod.rs`
  (`ViewSettings`), `src/ui/view3d.rs`, `src/ui/settings.rs`
- Test: `crates/driftless-app/tests/render.rs` (neue Tests im bestehenden Stil mit `headless_device`; ohne
  Adapter überspringen)

**Interfaces:**
- Produces:
  - `pub enum PointSize { Pixel, World }` in `render`. Vorgabe `World`.
  - `FrameParams.point_size: PointSize`, `FrameParams.point_m: f32` (Durchmesser in m),
    `FrameParams.point_px_range: [f32; 2]` (min/max in physischen Pixeln)
  - `ViewSettings.point_size: PointSize`
  - `ViewSettings.point_m: Option<f32>` (`None` = aus der Karte)
- Regeln:
  - Die Größe in Metern kommt aus der Raster-Einstellung der geladenen Karte (`map.json`, `dense.voxel`) × 1,5.
    Ohne Angabe gilt 0,015 m.
  - Pixelgrenzen: min = `point_px` der Einstellung (heute 2 px), max = 24 px × Pixel je Punkt.
  - Weicher Rand: Der Fleck ist rund (`discard` außerhalb des Kreises). Im äußeren Ring (r > 0,8) wird die
    Farbe zur Mitte hin leicht abgedunkelt, damit sich Flecken optisch trennen. Kein Blending, damit
    Tiefenpuffer und Geschwindigkeit bleiben.
  - Der Pixel-Modus bleibt unter ⚙ wählbar.

- [ ] **Step 1: Write the failing tests**
  - `world_size_shrinks_with_distance`: zwei gleiche Punkte in 1 m und 4 m Abstand, `point_m = 0.05`. Die
    gezeichnete Breite in Pixeln beim nahen ist ≈ 4× die des fernen (±1 px).
  - `world_size_clamped`: Ein Punkt in 0,05 m Abstand ist höchstens `max` px breit. Ein Punkt in 200 m Abstand
    ist mindestens `min` px breit (sichtbar). *(Review Focus 4)*
  - `pixel_mode_unchanged`: Mit `PointSize::Pixel` ist das Bild Pixel für Pixel gleich dem heutigen
    Rendertest (bestehende Referenz bzw. Vergleich mit dem bisherigen Pfad).
  - `point_m_from_map`: Eine Karte mit `dense.voxel = 0.02` → `point_m == 0.03`. Ohne `map.json` → `0.015`.
- [ ] **Step 2: Run to verify they fail**
  - Run: `~/.cargo/bin/cargo test -p driftless-app --test render`
  - Expected: FAIL.
- [ ] **Step 3: Implement**
  - Im Vertex-Shader die Größe aus `point_m`, `clip.w` und der Brennweite aus `view_proj` berechnen, dann
    auf `point_px_range` begrenzen.
  - `FrameParams` bleibt 16-Byte-ausgerichtet.
  - Die LOD-Auswahl bleibt unverändert.
- [ ] **Step 4: Run** `~/.cargo/bin/cargo test -p driftless-app` → grün.
  - Kurz prüfen, wie die Bildzeit beim synthetischen Test (`--synthetic 6_000_000`, eine Messung im
    Release-Build) im Vergleich zum Pixel-Modus aussieht. Die Zahl kommt in den Report.
  - Erwartet: höchstens 10 % langsamer.
- [ ] **Step 5: Commit** `feat(app): Punkte als Flecken in Metergröße`

### Task 6: Schwebende Werkzeuge, Messen, Statusleiste

**Files:**
- Create: `crates/driftless-app/src/ui/tools.rs` (Werkzeugleiste und Messen)
- Modify: `src/ui/view3d.rs` (schwebende Leisten oben links, oben rechts, unten Mitte; Werkzeug-Zustand),
  `src/ui/status.rs` (schlank), `src/ui/steps/planung.rs` (Messungen), `src/ui/text.rs`
- Test: `crates/driftless-app/tests/ui_tools.rs`

**Interfaces:**
- Consumes: das heutige Picking (`pick.rs`), `CameraMode` und `ViewSettings.cut`.
- Produces:
  - `pub enum Tool { Orbit, Pan, Pick, Measure, Cut }` als `AppState.tool`. Vorgabe `Orbit`.
  - `pub struct Measurement { pub a: [f64; 3], pub b: [f64; 3] }` mit `fn distance(&self) -> f64`
  - `AppState.measurements: Vec<Measurement>`
- Regeln:
  - Immer genau ein Werkzeug ist aktiv.
  - Orbit/Pan bewegen die Kamera mit der linken Taste. Rechte/mittlere Taste und Rad bleiben wie heute.
  - Pick wählt einen Punkt.
  - Measure: Der erste Klick setzt A, der zweite B, die Messung wird angehängt. Esc bricht ab. Linie und
    Abstand stehen in der Ansicht.
  - Cut zeigt die z-Regler und „Decke ausblenden“ in einer schwebenden Leiste über der Werkzeugleiste.
  - „Alles“ ist ein Knopf und kein Werkzeug: alles zeigen wie heute mit `F`.
  - Oben rechts steht der Umschalter Punkte · Fläche · beides (`O` bleibt), oben links eine Info („Punktwolke ·
    6,0 Mio. Punkte“ bzw. „Oberfläche · 712.218 Dreiecke“).

- [ ] **Step 1: Write the failing tests**
  - `exactly_one_tool_active`: Klick auf „Messen“ → `tool == Measure`. Klick auf „Schnitt“ → `tool == Cut`.
    Die Leiste hebt genau einen Knopf hervor.
  - `measure_two_points_gives_distance`: Synthetische Wolke. Zwei Klicks auf bekannte Punkte im Abstand
    2,00 m → `measurements.len() == 1`, `distance()` ≈ 2,00 ± 0,02. In der Ansicht steht `text::meters(2.0)`,
    und die Messung steht im Feld Planung.
  - `escape_cancels_measure`: Ein Klick, dann Esc → keine Messung.
  - `cut_tool_shows_sliders`: Werkzeug Schnitt → `text::CUT_MIN`/`CUT_MAX` sichtbar. Eine Änderung landet in
    `view.cut`.
  - `small_window_no_overlap`: Fenster 1024 × 640, Scan-Name mit 80 Zeichen. Die Rechtecke von Leiste,
    Schrittfeld und den drei schwebenden Leisten überlappen sich nicht. Der Name endet mit „…“.
    *(Review Focus 3)*
- [ ] **Step 2: Run to verify they fail**
  - Run: `~/.cargo/bin/cargo test -p driftless-app --test ui_tools`
  - Expected: FAIL.
- [ ] **Step 3: Implement**
  - Leisten mit `theme::float_frame()` und `egui::Area`, an den Rand der Ansicht gebunden.
  - Statusleiste: eine Zeile mit letzter Meldung links und laufendem Auftrag plus Fortschritt plus
    „Abbrechen“ rechts. Das Protokoll lässt sich aufklappen wie heute.
- [ ] **Step 4: Run** `~/.cargo/bin/cargo test -p driftless-app` → grün.
- [ ] **Step 5: Commit** `feat(app): schwebende Werkzeuge und Messen`

### Task 7: Bildtests, Doku, Abnahmebilder

**Files:**
- Create: `crates/driftless-app/tests/ui_snapshots.rs`, `docs/ergebnisse/2026-10-24-baustein-4.md`
- Modify: `README.md` (Abschnitt Programm: die drei Schritte, Werkzeuge, ⚙, Tastatur `O`/`F`/Esc),
  `crates/driftless-app/tests/ui_screenshots.rs` (neue Bilder je Schritt)

- [ ] **Step 1: Snapshot-Tests**
  - Mit `egui_kittest` und `snapshot`: `snapshot_daten`, `snapshot_bilder`, `snapshot_planung`, jeweils mit
    synthetischer Szene, 1400 × 900 und Skalierung 1×.
  - Laufen nur auf macOS (`#[cfg(target_os = "macos")]`), mit einer Pixel-Toleranz, die Schrift-Rundung
    erlaubt.
  - Die Referenzbilder liegen im Repo unter `tests/snapshots/` (nur synthetisch, keine Scan-Daten).
- [ ] **Step 2: Run** `~/.cargo/bin/cargo test -p driftless-app --test ui_snapshots` → grün,
  nachdem die Referenzen erzeugt wurden.
- [ ] **Step 3: Abnahmebilder**
  - Das Programm mit dem vorhandenen Testprojekt (Scratchpad, verweist auf `~/Downloads/20261022133554`)
    öffnen. Es wird nichts gerechnet und nichts in den Scan-Ordner geschrieben.
  - Bildschirmfotos der drei Schritte machen, plus eine Messung und den Fleck-Modus nah an einer Wand:
    `~/Documents/Privat/driftless-testdata/ergebnisse-b4/{daten,bilder,planung,messen,flecken}.png`
  - Zusätzlich bei Skalierung 2× (`with_pixels_per_point(2.0)`): `daten-2x.png`.
- [ ] **Step 4: Ergebnisdokument**
  - Inhalt: was sich geändert hat, Bildzeit Fleck- gegen Pixel-Modus aus Task 5, und wo jede alte Funktion
    jetzt liegt (Tabelle alt → neu).
  - Offen: Windows-Handtest, helles Thema.
- [ ] **Step 5: Commit** `docs: Baustein 4 Ergebnisse und README`
