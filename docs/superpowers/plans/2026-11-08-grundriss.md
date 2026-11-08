# Grundriss automatisch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Aus der Karte oder dem Modell eines Stockwerks rechnet Driftless einen Grundriss mit Wänden, Türen, Fenstern, Räumen und Maßen, zeigt ihn im Feld „Planung“, nimmt von Hand gesetzte Objekte (Steckdose, Anschluss, Kabel, Rohr) und Messungen auf und schreibt alles als DXF mit eigenen Layern.

**Architecture:**
- **Erkennung** (neue Crate `driftless-plan`, nur Geometrie, keine Dateien): `analyze` nimmt Punkte und Laufbahn und gibt `FloorPlan` + `TopView` oder „mehrere Stockwerke“.
- **Schritt** (`driftless-steps/src/plan.rs`, Ordner `grundriss-NNN`) und **Objekte** (`objects.rs`, `objekte.toml` bei Karte oder Modell) folgen `surface.rs` und `marks.rs`.
- **DXF:** generische `Drawing` und AC1015-Schreiber in `driftless-export/src/dxf.rs`; `plan_drawing.rs` macht aus Plan + Objekten eine `Drawing`. Dieselbe `Drawing` zeichnet die App (`ui/plan_view.rs`) und schreibt der Export.

**Tech Stack:** Rust 2021, nalgebra 0.33, rayon, serde/serde_json/toml, png, egui 0.36, `driftless-io` (`atomic_write`, `TempPath`, `FileLock`, `Progress`). Neu nur als Test-Abhängigkeit: `dxf` 0.6.2 (MIT).

**Spec:** `docs/superpowers/specs/2026-11-08-grundriss-design.md`

## Global Constraints

- Arbeitsweise laut Yannick: schnell bauen, **keine Mess- oder Prüfläufe zwischendurch**; echte Daten erst in „Prüfung am Schluss“ (Spec 12), ein Gesamt-Review ganz am Ende. Tests nur mit synthetischen Daten.
- Oberflächen-Rechnung nicht anfassen (`crates/driftless-map/**`, `crates/driftless-steps/src/surface.rs`): Dort arbeitet parallel der Zweig `rauschen`.
- App-Änderungen in eigenen Modulen (`ui/plan.rs`, `ui/plan_view.rs`, `ui/objects.rs`); in bestehenden Dateien nur Einhängepunkte.
- Texte der App deutsch, schlicht, in `crates/driftless-app/src/ui/text.rs`; Bezeichner englisch. Höhen-Ebenen heißen „Stockwerk“, Zeichnungs-Ebenen „Layer“.
- Layer genau so: `Wand` 7, `Tür` 30, `Fenster` 4, `Öffnung-unklar` 8, `Raum` 3, `Bemaßung` 2, `Messung` 6, `Steckdose` 1, `Anschluss` 200, `Kabel` 5, `Rohr` 34.
- DXF AC1015, Text, `$INSUNITS = 4`, `$MEASUREMENT = 1`, `$DWGCODEPAGE = ANSI_1252`, Koordinaten in mm, Zahlen mit festem Punkt.
- Maßtexte „8,30 m“: zwei Stellen, Komma, Einheit.
- Tests: keine exakten Gleitkomma-Vergleiche auf Werten aus `sin`, `cos`, `atan2`, `exp`, `ln` (Windows rundet anders), immer mit Toleranz.
- Deterministisch auf demselben System: kein Zufall (Rauschen im Test über festen LCG), Sortierungen mit Gleichstandsregel, keine Reihenfolge aus einer HashMap.
- Keine Scan-Daten, Bilder, Kundennamen ins Repository. Keine GPL/AGPL im Programm, `cargo deny check` grün.
- Vor jedem Commit grün: `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, die Tests der berührten Crates (`cargo test -p <crate>`); vorher `export PATH=$HOME/.cargo/bin:$PATH`. Vor dem letzten Commit zusätzlich `cargo test --workspace`, `cargo clippy --release -p driftless-app --bins -- -D warnings`, `cargo deny check`.
- Commits enden mit `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` und `Claude-Session: https://claude.ai/code/session_01JqZ6P3km8DjVvZsNNDs9MM`.

## Review Focus

1. **Streupunkte weit draußen** (Spiegelung, Rest durch ein Fenster, 80 m entfernt): Das Raster wächst nicht mit, Maße und Räume bleiben richtig (Test in Aufgabe 4).
2. **Projekt verschoben, Rohdaten abgezogen:** Der Plan bleibt aktuell und lässt sich neu rechnen, denn er liest nur die Karte (Tests in Aufgabe 5 und 9).
3. **Fremde Zeichen in Objektnamen und Notizen** (Zeilenumbruch, Emoji, `;`, sehr lang): Das DXF bleibt lesbar, jede Gruppe bleibt zwei Zeilen (Tests in Aufgabe 7 und 8).
4. **Objekte ohne aktuellen Plan oder außerhalb des Plans:** Sie gehen nicht verloren, stehen in der Liste und, sobald ein Plan da ist, an der richtigen Stelle im DXF, auch weit außerhalb der Wände (Tests in Aufgabe 8 und 10).
5. **Karte, aus der kein Plan wird** (leere Wolke, nur eine Wand, nur Boden): klare deutsche Meldung, kein halber Ordner, kein Absturz (Tests in Aufgabe 4 und 5).

---

### Task 1: Crate `driftless-plan`, Testräume, Stockwerke

**Files:**
- Create: `crates/driftless-plan/Cargo.toml` (Abhängigkeiten `driftless-io`, `nalgebra` 0.33 ohne Standardmerkmale mit `std`, `rayon` 1.12, `serde` mit `derive`, `thiserror` 2; Merkmal `test-util`), `crates/driftless-plan/src/lib.rs`, `src/levels.rs`, `src/testutil.rs`
- Modify: `Cargo.toml` (Member, `[profile.dev.package.driftless-plan] opt-level = 2`), `Cargo.lock`
- Test: in den Modulen

**Interfaces:**
- Produces:
```rust
pub struct PlanInput<'a> { pub xyz: &'a [[f64; 3]], pub rgb: Option<&'a [[u16; 3]]>, pub poses: &'a [[f64; 3]] } // Meter, Z oben
pub struct PlanConfig { pub cell: f64, pub storey: Option<usize> } // Default 0.02, None; storey zählt ab 1
pub struct Storey { pub floor_z: f64, pub ceiling_z: Option<f64>, pub pose_share: f64, pub floor_area: f64 } // serde
pub struct Leveling { pub normal: [f64; 3], pub pivot: [f64; 3], pub tilt_deg: f64 } // serde; NONE; apply/undo
pub struct StoreySearch { pub storeys: Vec<Storey>, pub hints: Vec<String> } // storeys nach floor_z aufsteigend
pub struct StoreyPoints { pub xyz: Vec<[f32; 3]>, pub rgb: Option<Vec<[u16; 3]>>, pub poses: Vec<[f32; 3]>,
                          pub origin: [f64; 3], pub leveling: Leveling, pub height: Option<f32> } // z über dem Boden
#[derive(thiserror::Error)] pub enum PlanError { Cancelled, TooFewPoints, NoPoses, NoFloor, NoWalls, NoSuchStorey { asked: usize, found: usize } } // Meldungen deutsch
pub fn find_storeys(input: &PlanInput, progress: &Progress) -> Result<StoreySearch, PlanError>;
pub fn level_for(input: &PlanInput, storey: &Storey) -> Leveling;
pub fn storey_points(input: &PlanInput, storey: &Storey, leveling: &Leveling) -> StoreyPoints; // nur Punkte von Boden − 10 cm bis Decke + 10 cm (ohne Decke: Boden + 3,5 m), nur Lagen dieses Stockwerks
// testutil (Merkmal test-util)
pub struct Scene; pub struct TestData { pub xyz: Vec<[f64; 3]>, pub poses: Vec<[f64; 3]> }
impl Scene {
    pub fn new() -> Scene;
    pub fn room(self, min: [f64; 2], max: [f64; 2], floor: f64, height: f64) -> Scene; // wie polygon, vier Wände ab der Südseite gegen den Uhrzeigersinn
    pub fn polygon(self, corners: &[[f64; 2]], floor: f64, height: f64) -> Scene;      // Boden, Decke, Wände; Wand i von corners[i] nach corners[i+1]
    pub fn door(self, wall: usize, at: f64, width: f64, height: f64) -> Scene;         // alle Tür-/Fenster-Angaben auf den zuletzt angelegten Raum
    pub fn window(self, wall: usize, at: f64, width: f64, sill: f64, height: f64) -> Scene;
    pub fn closed_door(self, wall: usize, at: f64, width: f64, height: f64, recess: f64) -> Scene;
    pub fn no_wall(self, wall: usize) -> Scene; pub fn open_top(self) -> Scene;
    pub fn block(self, min: [f64; 3], max: [f64; 3]) -> Scene; // Möbel; nimmt Wand- und Bodenpunkte im Grundriss weg, wenn der Block sie berührt oder ≤ 10 cm davorsteht
    pub fn slab(self, min: [f64; 2], max: [f64; 2], z: f64) -> Scene; // waagerechte Fläche
    pub fn behind(self, min: [f64; 3], max: [f64; 3]) -> Scene;       // Punkte hinter einer Öffnung
    pub fn walk(self, way: &[[f64; 3]]) -> Scene; pub fn spacing(self, m: f64) -> Scene; pub fn noise(self, sigma: f64) -> Scene;
    pub fn yaw(self, deg: f64) -> Scene; pub fn tilt(self, deg: f64) -> Scene; pub fn build(self) -> TestData;
}
impl TestData { pub fn input(&self) -> PlanInput<'_>; }
```
- Werte: Höhen-Stufen 1 cm, Flächen-Zellen 10 cm, Boden-Fenster 0,5–2,2 m unter dem Median einer Laufbahn-Gruppe, Boden mindestens 1,5 m² und 40 % der größten Fläche im Fenster, Laufbahn-Gruppe ab 10 % der Lagen, Stockwerke ab 1,0 m Abstand der Böden, Absatz 5–50 cm, Schiefe wird ab 0,3° korrigiert.

- [ ] **Step 1: Failing tests** in `levels.rs` (Raum 8,3 × 7,9 × 2,6 m, Boden bei z = −1,3, Laufbahn bei z = 0):
  - `one_room`: ein Stockwerk, `floor_z` und `ceiling_z` auf 1 cm, `floor_area` > 50.
  - `table_is_no_floor`: `slab` 2 × 1 m, 0,75 m über dem Boden → Boden unverändert.
  - `two_storeys`: Räume mit Böden bei −1,3 und +1,6, Laufbahn in beiden → zwei Stockwerke, aufsteigend, `pose_share` je 0,5 ± 0,1.
  - `big_area_below_is_a_hint`: `slab` 20 m² 3 m unter dem Boden ohne Laufbahn → ein Stockwerk, Hinweis enthält „tiefer“.
  - `step_in_floor_is_a_hint`: `slab` 4 m², 0,2 m über dem Boden → ein Stockwerk, Hinweis enthält „Absatz“.
  - `tilt_is_taken_out`: `tilt(1.0)` → `tilt_deg` 1 ± 0,05; danach Bodenpunkte auf ± 5 mm eben. `small_tilt_is_kept`: `tilt(0.1)` → `Leveling::NONE`.
  - `no_ceiling`: `open_top` → `ceiling_z == None`. `only_walls_is_no_floor`: Err `NoFloor`. `no_poses`: Err `NoPoses`. `too_few_points`: Err `TooFewPoints`.
  - `storey_points_only_this_storey`: Punkte und Lagen des oberen Stockwerks fehlen, `z` über dem Boden auf 1 cm.
  - `same_input_same_result`, `cancel_stops` (Err `Cancelled`).
- [ ] **Step 2:** `cargo test -p driftless-plan` → FAIL.
- [ ] **Step 3:** Crate und `levels.rs` umsetzen (Ebene durch den Boden mit `nalgebra::SymmetricEigen`). Testräume mit festem LCG, Punkte im Raster `spacing` (Vorgabe 1 cm), Lage über `yaw`/`tilt` zuletzt.
- [ ] **Step 4:** Tests PASS.
- [ ] **Step 5:** Commit `feat(grundriss): Crate driftless-plan, Stockwerke finden`.

### Task 2: Wände

**Files:**
- Create: `crates/driftless-plan/src/grid.rs` (je Zelle eine `u64`-Maske der belegten Höhenscheiben), `crates/driftless-plan/src/walls.rs`
- Test: in den Modulen

**Interfaces:**
- Consumes: `StoreyPoints`, `PlanConfig`, `Scene`.
- Produces:
```rust
pub struct WallFace { pub a: [f64; 2], pub b: [f64; 2], pub inward: [f64; 2], pub seen_m: f64 } // serde; Rahmen von StoreyPoints; inward zeigt in den Raum
pub struct WallPair { pub front: usize, pub back: usize, pub thickness: f64 }                   // serde
pub struct Walls { pub axis: [f64; 2], pub faces: Vec<WallFace>, pub pairs: Vec<WallPair>, pub cell: f64 } // serde; axis Einheitsvektor
pub fn find_walls(points: &StoreyPoints, cfg: &PlanConfig, progress: &Progress) -> Result<Walls, PlanError>;
```
- Werte: Band Boden + 15 cm bis Decke − 15 cm (ohne Decke bis 2,2 m), 64 Scheiben, Masken der 3×3-Nachbarschaft zusammengefasst; stark ab 55 %, schwach ab 25 %, schwache nur mit starker Verbindung. Hauptrichtung über schärfstes Projektionsprofil in 0,5°-, verfeinert 0,1°-Schritten, gefaltet auf 90°. Wandlinien: erst Hauptrichtung und Senkrechte (Senkrechte als `(-y, x)`, ohne Winkelfunktion), dann schräge Reste in 1°-Schritten, Stützlänge ab 0,5 m, ≤ 2° an der Hauptrichtung wird eingerastet. Abstand der Linie: Median der Punkte in ± 4 cm. Lücken ≤ 25 cm überbrücken; gleiche Linie (± 3 cm) mit Lücke bis 3 m wird eine Fläche (Tür-Lücken, entschieden in Aufgabe 3), `seen_m` ist die wirklich gemessene Länge. Stücke unter 30 cm fallen weg. Ecken schließen bis 25 cm. `inward`: Seite mit den meisten Lagen der Laufbahn. Paare: gegenläufige `inward`, parallel auf 1°, Abstand 5–60 cm, Überlappung ≥ 50 % der kürzeren.

- [ ] **Step 1: Failing tests** in `walls.rs`:
  - `box_room`: vier Flächen, Längen 8,30 und 7,90 m auf 1 cm, jede Ecke trifft die nächste auf 1 mm.
  - `rotated_room_keeps_right_angles`: `yaw(23.0)` → `axis` 23° ± 0,2° (gefaltet auf 90°), `dot` benachbarter Richtungen betragsmäßig < 1e-9, Längen auf 1 cm.
  - `l_shaped_room`: `polygon` mit 6 Ecken → sechs Flächen.
  - `shared_wall_has_thickness`: zwei Räume, 12 cm Wand, Tür dazwischen, Laufbahn durch beide → ein `WallPair` mit 0,12 ± 0,01.
  - `furniture_is_no_wall`: Tisch und Schrank 1,2 m hoch frei im Raum → keine Fläche mit Mitte weiter als 30 cm innerhalb des Raums.
  - `slanted_wall_stays_slanted`: 60°-Wand → Richtung auf 0,5°, nicht auf die Hauptrichtung gelegt.
  - `door_gap_stays_one_face`, `window_does_not_cut_the_wall`: Tür bzw. Fenster → weiter vier Flächen, `seen_m` kleiner als die Länge bei der Tür.
  - `noisy_and_sparse`: `noise(0.005)`, `spacing(0.02)` → vier Flächen, Längen auf 2 cm.
  - `inward_points_to_the_walk`, `same_input_same_result`, `cancel_stops`.
- [ ] **Step 2:** `cargo test -p driftless-plan walls` → FAIL.
- [ ] **Step 3:** `grid.rs`, `walls.rs` umsetzen.
- [ ] **Step 4:** Tests PASS.
- [ ] **Step 5:** Commit `feat(grundriss): Wände aus der Draufsicht`.

### Task 3: Türen, Fenster, Unklares

**Files:**
- Create: `crates/driftless-plan/src/sight.rs` (3D-Bitraster 5 cm, Sichtlinie), `crates/driftless-plan/src/openings.rs`
- Test: in den Modulen

**Interfaces:**
- Consumes: `StoreyPoints`, `Walls`.
- Produces:
```rust
pub enum OpeningKind { Door, Window, Unclear }  // serde
pub struct Opening { pub kind: OpeningKind, pub face: usize, pub from: f64, pub to: f64, pub sill: f64, pub head: f64, pub crossed: bool, pub note: String } // serde; from/to Meter ab a; sill/head über dem Boden; note deutsch, bei Unclear der Grund
pub fn find_openings(points: &StoreyPoints, walls: &Walls, progress: &Progress) -> Result<Vec<Opening>, PlanError>; // sortiert nach face, from
```
- Werte: Wandbild 4-cm-Zellen, Punkte ± 3 cm um die Fläche; leer unter 15 % des Medians der belegten Zellen; Schließen um eine Zelle. Sichtlinie von bis zu 2000 Lagen (ausgedünnt auf 0,5 m Weg) auf der Seite `inward`, bis 12 m, Winkel zur Normalen bis 75°, endet 8 cm vor der Wand; offen = mindestens 50 % der Lochzellen mit freier Sicht. Tür: Unterkante ≤ 10 cm, Höhe ≥ 1,7 m, Breite 0,55–2,6 m, offen und (durchquert oder Punkte 10 cm bis 3 m dahinter); durchquert reicht bei verstellter Öffnung. Fenster: offen, Unterkante ≥ 20 cm, ≥ 25 × 25 cm, Wand darunter auf ≥ 50 % der Breite. Geschlossene Tür: Rechteck 1,5–10 cm hinter der Fläche, Unterkante ≤ 10 cm, 1,8–2,4 m hoch, 0,6–1,3 m breit → Unclear. Sonst offen und ≥ 25 × 25 cm → Unclear. Verdeckt → nichts, die Wand läuft durch.
- Texte `note`: `"Bis zum Boden offen, nichts dahinter: Tür oder bodentiefes Fenster?"`, `"Tür geschlossen?"`, `"Öffnung in Größe oder Form weder Tür noch Fenster"`.

- [ ] **Step 1: Failing tests** in `openings.rs` (Raum wie oben, Laufbahn innen):
  - `door_found`: Tür 0,90 × 2,05 m in Wand 0, `behind` dahinter → genau eine `Door`, Breite und Höhe auf 3 cm, `sill` ≤ 3 cm.
  - `walkthrough_makes_a_door`: nichts dahinter, Laufbahn geht durch → `Door`, `crossed`.
  - `window_found`: 1,20 m breit, Brüstung 0,90, Höhe 1,30 → `Window`, `sill` und `head` auf 3 cm.
  - `cabinet_is_no_door`: `block` 1,0 × 0,6 × 2,1 m an der Wand → keine Öffnung, Wand durchgehend.
  - `open_to_floor_alone_is_unclear`, `closed_door_is_unclear` (`recess` 4 cm), `very_wide_gap_is_unclear` (3,0 m bis zum Boden): `Unclear` mit dem jeweiligen Text.
  - `small_hole_is_nothing`: 15 × 15 cm → keine Öffnung.
  - `two_openings_in_order`, `noise_keeps_the_window` (`noise(0.005)`), `same_input_same_result`, `cancel_stops`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS (`cargo test -p driftless-plan openings`).
- [ ] **Step 5:** Commit `feat(grundriss): Türen und Fenster in den Wänden`.

### Task 4: Räume, Maße, Gesamtergebnis

**Files:**
- Create: `crates/driftless-plan/src/rooms.rs`, `src/dims.rs`, `src/top_view.rs`, `src/analyze.rs`
- Modify: `crates/driftless-plan/src/lib.rs`
- Test: in den Modulen

**Interfaces:**
- Consumes: Aufgaben 1–3.
- Produces:
```rust
pub struct Room { pub nr: u32, pub outline: Vec<[f64; 2]>, pub area: f64, pub height: Option<f64>, pub open: bool, pub label_at: [f64; 2] }  // serde
pub struct Dim { pub a: [f64; 2], pub b: [f64; 2], pub line_a: [f64; 2], pub line_b: [f64; 2], pub text: String }  // serde; a,b gemessen, line_* Lage der Maßlinie
pub struct PlanFrame { pub axis: [f64; 2], pub origin: [f64; 3], pub leveling: Leveling } // serde
impl PlanFrame { pub fn to_plan(&self, map: [f64; 3]) -> [f64; 3]; pub fn to_map(&self, plan: [f64; 3]) -> [f64; 3]; } // Drehung nur über axis
pub struct FloorPlan { pub storey: Storey, pub storeys: Vec<Storey>, pub frame: PlanFrame, pub faces: Vec<WallFace>, pub pairs: Vec<WallPair>,
                       pub openings: Vec<Opening>, pub rooms: Vec<Room>, pub dims: Vec<Dim>, pub hints: Vec<String>, pub cell: f64 } // serde; alles im Plan-Rahmen: Hauptrichtung = x, linke untere Ecke = (0, 0), Boden = 0
pub struct TopView { pub width: u32, pub height: u32, pub origin: [f64; 2], pub cell: f64, pub rgba: Vec<u8> } // Zeile 0 = größtes y
pub enum Analysis { Plan(Box<FloorPlan>, TopView), SeveralStoreys(Vec<Storey>) }
pub fn analyze(input: &PlanInput, cfg: &PlanConfig, progress: &Progress) -> Result<Analysis, PlanError>;
pub fn metres(x: f64) -> String; // „8,30 m“
```
- Werte: Raumraster 4 cm, Boden ± 4 cm, Schließen 20 cm, Füllung von den Lagen, Räume ab 1 m². Grenzen: Wände, Türen und Unklares als Linie. Umriss vereinfacht (5 cm), je Kante nächste parallele Wand bis 12 cm, Ecken = Schnittpunkte, Stufen unter 10 cm fallen weg; offen, wenn über 30 % des Umfangs keine Wand. `nr` nach Fläche absteigend, bei Gleichstand nach kleinstem x, dann y. Maße: je Kante ≥ 30 cm, Maßlinie 30 cm im Raum; je Paar eine Dicke als Text. Raster-Grenzen aus Zellen mit mindestens 20 Punkten auf einem 50-cm-Raster, über 8192 Zellen je Seite wächst die Zelle (Hinweis). Draufsicht: je Zelle die Farbe des höchsten Punkts im Band, ohne Farbe Grau nach Höhe; höchstens 4096 Pixel je Seite. Hinweise deutsch: Absatz, Schiefe, offener Raum, Anzahl Unklares.

- [ ] **Step 1: Failing tests** in `analyze.rs`:
  - `one_room_plan`: ein Raum, vier Ecken, 8,30 × 7,90 m auf 1 cm, `area` 65,57 ± 0,3, `height` 2,60 ± 0,02, nicht `open`.
  - `two_rooms_split_at_the_door`: zwei Räume, `nr` nach Fläche.
  - `furniture_does_not_split_the_room`, `open_room`: `no_wall(2)` → `open`, Hinweis enthält „offen“.
  - `plan_frame_is_aligned`: `yaw(23.0)` → Wände achsenparallel (`|dy|` bzw. `|dx|` < 1e-6), `min` auf (0, 0) auf 1 mm; `to_map(to_plan(p))` zurück auf 1 µm.
  - `dimensions`: je Kante ein `Dim`, Text der 8,30-m-Wand ist „8,29 m“, „8,30 m“ oder „8,31 m“.
  - `several_storeys_ask_first`: zwei Stockwerke, `storey: None` → `SeveralStoreys`; `Some(2)` → Plan des oberen; `Some(3)` → `NoSuchStorey`.
  - `far_stray_points_do_not_blow_up_the_grid`: 500 Punkte 80 m entfernt → Raum und Maße unverändert auf 1 cm, `TopView` höchstens 1000 Pixel breit.
  - `huge_extent_coarsens_the_cell`: Räume 300 m auseinander → `cell` > 0,02, Hinweis, kein Absturz.
  - `top_view_shows_the_floor`: Breite passt auf 2 Pixel, außerhalb durchsichtig, auf dem Boden deckend.
  - `single_wall_is_no_plan` (nur eine Wand + Boden → `NoWalls`), `empty_cloud` (`TooFewPoints`), `same_input_same_result`, `cancel_stops`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS (`cargo test -p driftless-plan`).
- [ ] **Step 5:** Commit `feat(grundriss): Räume, Maße und Plan-Rahmen`.

### Task 5: Schritt und Befehl `driftless plan`

**Files:**
- Create: `crates/driftless-steps/src/plan.rs`, `crates/driftless-cli/src/plan.rs`
- Modify: `crates/driftless-steps/src/lib.rs`, `src/error.rs` (`From<PlanError>`), `crates/driftless-steps/Cargo.toml` (`driftless-plan`; `png` von Test nach normal; Test-Abhängigkeit `driftless-plan` mit `test-util`), `crates/driftless-export/src/testutil.rs` (`write_map_folder`), `crates/driftless-cli/src/main.rs` (Befehl `Plan`, in `runs_a_step`), `crates/driftless-cli/Cargo.toml` (`driftless-plan`, als Test-Abhängigkeit mit `test-util`)
- Test: `crates/driftless-steps/tests/plan.rs`, `crates/driftless-cli/tests/plan.rs`

**Interfaces:**
- Consumes: `analyze`; `driftless_export::read_points`; `read_model`, `version_fits`, `member_transform`, `MODEL_LAS`, `MODEL_REPORT`.
- Produces:
```rust
pub const PLAN_REPORT: &str = "grundriss.json";
pub const TOP_VIEW_FILE: &str = "draufsicht.png";
pub const PLAN_VERSION: u32 = 1;
pub enum PlanSource { Map(PathBuf), Model(PathBuf) }
impl PlanSource { pub fn detect(dir: &Path) -> Result<PlanSource, StepError>; pub fn dir(&self) -> &Path; } // map.json bzw. modell.json
pub struct PlanSettings { pub storey: Option<usize> }  // serde, Default
pub struct PlanRequest { pub source: PlanSource, pub model: Option<ModelReport>, pub out_dir: PathBuf, pub settings: PlanSettings } // model wie ModelSurfaceRequest::model
pub struct TopViewInfo { pub width: u32, pub height: u32, pub origin: [f64; 2], pub cell: f64 } // serde
pub struct PlanReport { pub input_id: String, pub version: u32, pub settings: PlanSettings, pub plan: FloorPlan, pub top_view: TopViewInfo,
                        pub files: Vec<String>, pub bytes: BTreeMap<String, u64>, pub points: u64, pub seconds_total: f64, pub peak_rss_bytes: Option<u64> } // serde; `programm` wie die anderen Berichte
pub enum PlanOutcome { Done(Box<PlanReport>), SeveralStoreys(Vec<Storey>) }
pub fn run_plan(req: &PlanRequest, progress: &Progress) -> Result<PlanOutcome, StepError>;
pub fn plan_fits(req: &PlanRequest) -> bool;
pub fn read_plan(dir: &Path) -> Result<PlanReport, StepError>;
// driftless-export, testutil: Kartenordner für die Tests von Schritt, CLI und App
pub fn write_map_folder(dir: &Path, xyz: &[[f64; 3]], poses: &[[f64; 3]]); // color.las, map.json mit steps.clean.input_id, traj.tum
```
- Eingabe: Karte: `color.las`, sonst `clean.las`; Laufbahn `traj.tum` im Kartenordner oder einen Ordner darüber. Modell: `modell.las`, Laufbahnen aller Scans mit `included`, je mit `member_transform` bewegt. Anderer `MERGE_VERSION` → Fehler „Modell veraltet …“.
- Kennung (Text, in `input_id` per `hex`): Karte `plan v{PLAN_VERSION}\nmap {steps.clean.input_id aus map.json}\ncloud {Name} {Bytes}\ntraj {content_hash}\n{settings:?}\n`; Modell `plan v…\nmodel {input_id}\ncloud modell.las {Bytes}\n`, je Scan `member {k} {clean_id oder -} {content_hash der Laufbahn}\n`, dann Einstellungen.
- Ablauf: `.tmp`-Reste entfernen, Bericht entfernen, PNG über `TempPath`, Bericht zuletzt mit `atomic_write` (`report_text`, Name und Größe jeder Datei). Abbruch vor dem Schreiben lässt alles, wie es war. Mehrere Stockwerke: nichts wird geschrieben.
- CLI: `driftless plan <QUELLE> [-o <ZIEL>] [--stockwerk <N>]`, Ziel-Vorgabe `<quelle>/grundriss`. Passt der Ordner schon, sagt es das und rechnet nicht. Mehrere Stockwerke: Exit-Code ≠ 0, Liste „1 · Boden … · Decke … · … % der Laufbahn“, Hinweis auf `--stockwerk`.

- [ ] **Step 1: Failing tests** (`tests/plan.rs`; Karte aus `Scene` und `write_map_folder`; Modell aus `ModelReport` von Hand):
  - `map_gives_plan_and_files`: `grundriss.json`, `draufsicht.png`, `read_plan` hat einen Raum, `bytes` stimmen, `plan_fits`.
  - `model_gives_plan`: zwei Scans, `transform` nicht Einheit → Plan im Modellrahmen; ein Scan mit `included = false` trägt keine Lagen bei.
  - `traj_or_cloud_or_storey_change_makes_it_stale`, `model_of_another_version_is_refused`, `plan_needs_no_raw_data` (kein Scan-Ordner, läuft und passt).
  - `several_storeys_write_nothing`, `no_plan_is_a_clear_error_and_no_report` (leere Wolke, Meldung deutsch), `cancel_leaves_no_report`, `killed_run_is_cleaned_up` (`draufsicht.png.tmp` weg).
  - `report_survives_a_moved_project`: Ordner kopieren → `read_plan` und `plan_fits` wahr.
  - CLI: `plan_writes_folder`, `second_run_says_it_fits`, `several_storeys_fail_with_the_list`, `stockwerk_picks_one`, `bad_source_says_why`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen (Muster `surface.rs`: `remove_stale_tmp`, `remove_report`, `peak_rss_bytes`). **Step 4:** PASS (`cargo test -p driftless-steps --test plan`, `cargo test -p driftless-cli --test plan`).
- [ ] **Step 5:** Commit `feat(grundriss): Schritt und Befehl driftless plan`.

### Task 6: Objekte

**Files:**
- Create: `crates/driftless-steps/src/objects.rs`
- Modify: `crates/driftless-steps/src/lib.rs`
- Test: `crates/driftless-steps/tests/objects.rs`

**Interfaces:**
- Produces:
```rust
pub const OBJECTS_FILE: &str = "objekte.toml";
pub enum ObjectKind { Socket, Connection, Cable, Pipe, Measure }  // serde: steckdose | anschluss | kabel | rohr | messung
pub enum ConnectionKind { Power, Water, Drain, Heating, Gas, Network, Other } // serde: strom | wasser | abwasser | heizung | gas | netzwerk | sonstiges
pub struct ObjectInput { pub kind: ObjectKind, pub points: Vec<[f64; 3]>, pub name: String, pub note: String,
                         pub connection: Option<ConnectionKind>, pub diameter_mm: Option<f32>, pub properties: BTreeMap<String, String> }
pub struct Object { pub nr: u32, pub kind: ObjectKind, pub points: Vec<[f64; 3]>, pub name: String, pub note: String,
                    pub connection: Option<ConnectionKind>, pub diameter_mm: Option<f32>, pub source: String /* "hand" */,
                    pub properties: BTreeMap<String, String>, pub seq: u64 /* unbekannte Schlüssel bleiben */ }
impl Object { pub fn label(&self) -> String; } // „Steckdose 3“, mit Name „Steckdose 3 · Küche“
pub enum Undone { Added(u32), Removed(u32), Changed(u32) }
pub enum ObjectsError { /* Io, Format, Invalid(String), Lock; Texte deutsch */ }
pub struct Objects { /* objekt, geloescht, next_nr, next_seq */ }
impl Objects {
    pub fn load(owner_dir: &Path) -> Result<Objects, ObjectsError>;    // fehlende Datei = leer
    pub fn save(&self, owner_dir: &Path) -> Result<(), ObjectsError>;
    pub fn change<T>(owner_dir: &Path, f: impl FnOnce(&mut Objects) -> Result<T, ObjectsError>) -> Result<(Objects, T), ObjectsError>;
    pub fn add(&mut self, input: ObjectInput) -> Result<u32, ObjectsError>;
    pub fn rename(&mut self, nr: u32, name: &str) -> bool;
    pub fn set_note(&mut self, nr: u32, note: &str) -> bool;
    pub fn remove(&mut self, nr: u32) -> bool;
    pub fn remove_kind(&mut self, kind: ObjectKind) -> usize;
    pub fn undo(&mut self) -> Option<Undone>;
    pub fn can_undo(&self) -> bool;
    pub fn all(&self) -> &[Object];
    pub fn get(&self, nr: u32) -> Option<&Object>;
}
```
- Gültig: Steckdose und Anschluss genau 1 Punkt, Anschluss mit `connection`, Kabel und Rohr mindestens 2, Rohr mit `diameter_mm` > 0, Messung genau 2; alle Koordinaten endlich. Punkte in Metern im Rahmen der Karte bzw. des Modells, Datei gebaut wie `marks.rs` (Kopfkommentar, `FileLock`, `atomic_write`, `#[serde(flatten)]` für unbekannte Schlüssel, `geloescht` für Rückgängig).

- [ ] **Step 1: Failing tests** (`tests/objects.rs`): `roundtrip_keeps_everything` (auch `[objekt.eigenschaften]` und ein unbekannter Schlüssel), `missing_file_is_empty`, `invalid_inputs_are_refused` (je Regel oben, Meldung deutsch), `numbers_are_not_used_twice`, `undo_order_and_restart` (Add, Rename, Notiz, Remove, in umgekehrter Folge, auch nach Laden), `remove_kind_is_one_undo`, `change_keeps_what_another_window_saved`, `file_name_is_safe` (nicht `oberflaeche…`, nicht `.tmp`), `source_is_hand`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat(grundriss): Objekte je Karte und Modell speichern`.

### Task 7: DXF-Schreiber

**Files:**
- Create: `crates/driftless-export/src/dxf.rs`
- Modify: `crates/driftless-export/src/lib.rs`, `crates/driftless-export/Cargo.toml` (Test-Abhängigkeit `dxf = "0.6.2"`, MIT)
- Test: `crates/driftless-export/tests/dxf.rs`

**Interfaces:**
- Produces:
```rust
pub struct Drawing { pub layers: Vec<Layer>, pub blocks: Vec<Block>, pub entities: Vec<Entity>, pub comments: Vec<String> } // Koordinaten in mm
pub struct Layer { pub name: String, pub color: u8, pub on: bool }
pub enum Shape {
    Line { a: [f64; 2], b: [f64; 2], dashed: bool },
    Polyline { points: Vec<[f64; 2]>, closed: bool, dashed: bool },
    Circle { centre: [f64; 2], radius: f64 },
    Text { at: [f64; 2], height: f64, rotation_deg: f64, value: String },
}
pub struct Block { pub name: String, pub shapes: Vec<Shape>, pub attributes: Vec<String> /* Tags, je ein ATTDEF */ }
pub enum EntityKind { Shape(Shape), Insert { block: String, at: [f64; 2], attributes: Vec<(String, String)> } }
pub struct Entity { pub layer: String, pub kind: EntityKind, pub xdata: Vec<String> }  // xdata unter APPID DRIFTLESS
pub fn dxf_bytes(drawing: &Drawing, elevation_mm: f64) -> Result<Vec<u8>, String>;
pub fn write_dxf(path: &Path, drawing: &Drawing, elevation_mm: f64, progress: &Progress) -> Result<(), ExportError>; // über TempPath
```
- Aufbau AC1015: `HEADER` (`$ACADVER`, `$DWGCODEPAGE`, `$INSBASE`, `$EXTMIN`, `$EXTMAX`, `$LIMMIN`, `$LIMMAX`, `$LTSCALE`, `$CLAYER`, `$TEXTSTYLE`, `$DIMSTYLE`, `$LUNITS`, `$AUNITS`, `$INSUNITS`, `$MEASUREMENT`, `$EXTNAMES`, `$LWDISPLAY`, `$HANDSEED`); `CLASSES` mit `LAYOUT`, `ACDBDICTIONARYWDFLT`, `ACDBPLACEHOLDER`; neun `TABLES` (`VPORT` `*Active`, `LTYPE` `ByBlock`, `ByLayer`, `Continuous`, `GESTRICHELT`, `LAYER` `0` und die eigenen, `STYLE` `Standard` mit `arial.ttf`, `VIEW`, `UCS`, `APPID` `ACAD` und `DRIFTLESS`, `DIMSTYLE` `Standard`, `BLOCK_RECORD`); `BLOCKS` (`*Model_Space`, `*Paper_Space`, eigene); `ENTITIES`; `OBJECTS` (Wurzel-`DICTIONARY` mit `ACAD_GROUP`, `ACAD_LAYOUT` mit `Model` und `Layout1`, `ACAD_MLINESTYLE` mit `Standard`, `ACAD_PLOTSETTINGS`, `ACAD_PLOTSTYLENAME` mit `Normal`). Jedes Objekt mit eindeutigem Handle (5), Besitzer (330) und `100`-Unterklassen. Vorbild für die Reihenfolge: was `ezdxf.new("R2000")` schreibt. `LWPOLYLINE` mit Elevation 38. `INSERT` mit `66 = 1`, `ATTRIB`, `SEQEND`. Ausgeschalteter Layer: negative Farbe.
- Text: Zeichen als Windows-1252 (eigene Tabelle); außerhalb, im Bereich bis U+FFFF: `\U+XXXX`; darüber `?`. Zeilenumbrüche und Steuerzeichen in Werten werden zu Leerzeichen. Zahlen `{:.3}`. Layer- und Blocknamen ohne `< > / \ " : ; ? * | =`, Backtick und Steuerzeichen, sonst Fehler; `xdata`-Werte auf 255 Zeichen gekürzt.

- [ ] **Step 1: Failing tests** (`tests/dxf.rs`, gelesen mit `dxf::Drawing::load_file`):
  - `header_says_r2000_and_millimetres`: `version == AcadVersion::R2000`, `default_drawing_units == Millimeters`; `$DWGCODEPAGE` im Text = `ANSI_1252`.
  - `layers_read_back`: `Tür`, `Öffnung-unklar`, `Bemaßung` genau so; ausgeschalteter Layer ist aus.
  - `entities_read_back`: Linie, geschlossene Polylinie mit `elevation_mm`, Kreis, Text, `INSERT` mit Attributen, `XDATA` unter `DRIFTLESS`; gestrichelt = `GESTRICHELT`.
  - `handles_are_unique_and_seed_is_higher`, `sections_in_order` (eigener Durchlauf über Paare, je Gruppe genau zwei Zeilen).
  - `umlauts_are_windows_1252`: rohe Bytes haben 0xFC für ü, 0x80 für €; `Ω` als `\U+03A9`; Emoji als `?`.
  - `newlines_in_values_do_not_break_the_file`: Text mit `\n`, `\r`, `;` → liest sich, Anzahl der Paare gerade.
  - `very_long_values_are_cut_in_xdata`, `bad_layer_name_is_refused`, `extents_in_header`.
  - `line_is_written_as_expected`: die Bytes einer `LINE` genau wie erwartet; `same_drawing_same_bytes`.
  - `no_tmp_left`, `cancel_writes_nothing`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS (`cargo test -p driftless-export --test dxf`), `cargo deny check` grün.
- [ ] **Step 5:** Commit `feat(export): DXF-Schreiber (AutoCAD 2000)`.

### Task 8: Plan als Zeichnung, Export

**Files:**
- Create: `crates/driftless-steps/src/plan_drawing.rs`
- Modify: `crates/driftless-steps/src/lib.rs`, `crates/driftless-steps/src/export.rs`, `crates/driftless-steps/Cargo.toml` (Test-Abhängigkeit `dxf`), `crates/driftless-cli/src/plan.rs`
- Test: `crates/driftless-steps/tests/plan_drawing.rs`, `crates/driftless-cli/tests/plan.rs`

**Interfaces:**
- Consumes: `PlanReport`, `Objects`, `Drawing`, `write_dxf`.
- Produces:
```rust
pub enum PlanPlacement { Walls, AsCloud }                   // Default Walls
pub struct PlanExport { pub placement: PlanPlacement, pub hidden: Vec<String> } // Default; hidden = Layer-Namen
pub const LAYERS: [(&str, u8); 11];                         // Reihenfolge wie in Global Constraints
pub fn plan_drawing(report: &PlanReport, objects: &Objects, export: &PlanExport) -> Drawing;
pub fn elevation_mm(report: &PlanReport, export: &PlanExport) -> f64;  // Walls 0, AsCloud Bodenhöhe
// export.rs
ExportFormat::Dxf  // name "dxf", label "DXF (AutoCAD 2000)", kind Plan
ExportKind::Plan
ExportSource::Plan { dir: PathBuf, export: PlanExport }  // Ordner grundriss-*; Objekte aus dem Ordner darüber
```
- Zeichnung in mm. `Walls`: Plan-Rahmen; `AsCloud`: `frame.to_map`, gleiche Drehung auch für Texte. Wandflächen ohne die Öffnungen; Endkappen an Öffnungen (Dicke aus dem Paar, sonst 120 mm). Tür: zwei Laibungen, Schwelle gestrichelt, Text „Tür 0,90 × 2,05 m“. Fenster: Doppellinie 60 mm, Text „Fenster 1,20 × 1,30 m, BRH 0,90 m“. Unklar: gestrichelt, Text = `note`. Raum: geschlossener Umriss, Text „Raum 1“, „65,57 m²“, „Höhe 2,60 m“ (zweite Zeile als eigener Text). Maße als Linie + Häkchen + Text. Messung: Linie + Text. Kommentare: Programm, Version, Lage; „Rohre: von Hand gesetzt, automatische Erkennung folgt“.
- Blöcke `Steckdose` und `Anschluss` (etwa 150 mm) mit Tags `NR`, `NAME`, `ART`, `HOEHE`, `NOTIZ`; freie Eigenschaften als weitere Attribute (Tag aus dem Schlüssel: Großbuchstaben, Umlaute umgeschrieben, nur `A-Z0-9_`). `HOEHE` = mm über dem Boden, ganzzahlig. Punktobjekte flach bei z = 0 bzw. Bodenhöhe. Kabel und Rohr: Linienzug, Text „Rohr 3 · Ø 50 mm“, `xdata` `NR=`, `NAME=`, `NOTIZ=`, `QUELLE=`, freie Eigenschaften, `DURCHMESSER_MM=`.
- `plan`-CLI um `--dxf <DATEI> [--lage waende|punktwolke] [--ueberschreiben]`; `driftless export <grundriss-ordner> -o x.dxf`.

- [ ] **Step 1: Failing tests:**
  - `plan_drawing.rs`: `walls_cut_at_openings`, `door_and_window_have_symbols_and_text` (Text enthält „BRH“), `unclear_shows_its_reason`, `room_label_and_dimensions`, `all_layers_exist_even_when_empty`, `hidden_layer_is_off`, `as_cloud_moves_it_and_walls_is_default` (eine Wand 23° gedreht, Punkte stimmen auf 1 mm mit der Karte), `socket_has_attributes` (`NR`, `NAME`, `ART`, `HOEHE` auf 1 mm), `free_properties_become_attributes`, `cable_and_pipe_have_text_and_xdata`, `measurement_on_its_layer`, `object_far_outside_stays_where_it_is`, `odd_characters_in_names_read_back` (`ä\nb;😀`, sehr lang).
  - Export: `dxf_export_reads_back` (mit `dxf`-Crate: Layer, Blöcke, `$INSUNITS`), `dxf_only_for_a_plan`, `no_overwrite_without_asking`, `objects_changed_after_plan_are_exported`, `plan_of_a_folder_is_detected`.
  - CLI: `plan_dxf_writes_the_file`, `lage_punktwolke`, `dxf_exists_needs_ueberschreiben`, `export_of_plan_folder`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS (`cargo test -p driftless-steps --test plan_drawing --test export`, `cargo test -p driftless-cli --test plan`).
- [ ] **Step 5:** Commit `feat(grundriss): Plan und Objekte als DXF exportieren`.

### Task 9: Feld „Planung“ und Plan-Ansicht

**Files:**
- Create: `crates/driftless-app/src/ui/plan.rs` (Zustand, Laden, Layer-Liste, Hinweise), `crates/driftless-app/src/ui/plan_view.rs` (2D-Fläche)
- Modify: `crates/driftless-app/src/project.rs` (`PlanInfo`, `MapInfo.plans`, `ModelInfo.plans`, `plan_status`, `run_plan_job`), `src/leftovers.rs` (`LeftoverKind::Plan`: `grundriss-NNN` ohne Bericht), `src/jobs.rs`, `ui/mod.rs`, `ui/actions.rs`, `ui/app.rs` (Zentrale Fläche: im Schritt Planung und Modus Plan statt `self.view.show`), `ui/steps/planung.rs` (Platzhalter ersetzen), `ui/tools.rs` (`show_switch`: „Plan · 3D“ in Planung), `ui/export.rs` (`ExportKind::Plan`, Wahl der Lage), `ui/text.rs`, `crates/driftless-app/Cargo.toml` (`driftless-plan`, als Test-Abhängigkeit mit `test-util`, `driftless-export` mit `test-util`)
- Test: `crates/driftless-app/tests/plan_app.rs` (Aufbau wie `tests/ui_surface.rs`); anpassen `ui_snapshots.rs`, `ui_layout.rs`, `leftovers.rs`

**Interfaces:**
- Consumes: Aufgaben 5 und 8.
- Produces:
```rust
// project.rs
pub const PLAN_NUMBERED: &str = "grundriss-";
pub struct PlanInfo { pub dir: PathBuf, pub label: String /* „Grundriss 001“ */, pub stale: bool, pub storey: Option<usize>, pub hints: Vec<String> }
pub fn plan_status(owner: &Path, plan: &PlanInfo) -> MapStatus;
pub enum PlanJobDone { Done { dir: PathBuf, reused: bool }, SeveralStoreys(Vec<Storey>) }
pub fn run_plan_job(owner: &Path, model: Option<&ModelInfo>, storey: Option<usize>, progress: &Progress) -> Result<PlanJobDone, StepError>; // neue grundriss-NNN nur, wenn keine passt
// jobs.rs
JobOutput::PlanStep(Box<PlanJobDone>), JobOutput::PlanLoaded(Box<ShownPlan>)
// ui/mod.rs
Selection::Plan(PathBuf);
Action::RunPlan { owner: PathBuf, storey: Option<usize> }, Action::ShowPlan(PathBuf), Action::SetPlanMode(PlanMode), Action::SetPlanLayer { layer: String, on: bool }, Action::SetTopView(bool), Action::OpenPlanExport
// ui/plan.rs
pub enum PlanMode { Plan, View3d }   // Vorgabe Plan
pub struct PlanState { pub owner: Option<PathBuf>, pub shown: Option<Arc<ShownPlan>>, pub mode: PlanMode, pub hidden: BTreeSet<String>,
                       pub top_view: bool, pub storeys: Option<(PathBuf, Vec<Storey>)>, pub selected_object: Option<u32>, pub camera: PlanCamera }
pub struct ShownPlan { pub dir: PathBuf, pub report: PlanReport, pub objects: Objects, pub drawing: Drawing, pub top_view: Option<egui::ColorImage> }
pub fn refresh_drawing(state: &mut PlanState); // nach Änderung der Objekte oder Layer
// ui/plan_view.rs
pub struct PlanCamera { pub centre_mm: [f64; 2], pub mm_per_point: f64 }
impl PlanCamera { pub fn fit(&mut self, min: [f64; 2], max: [f64; 2], size: [f32; 2]); pub fn zoom_at(&mut self, cursor: [f32; 2], size: [f32; 2], steps: f32); pub fn to_screen(&self, p: [f64; 2], size: [f32; 2]) -> [f32; 2]; }
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action>;
```
- Feld wie „Bilder“ (`card`, `state_label`, `main_button`, `pinned`): je Karte und Modell eine Karte mit Stand, Hauptknopf „Grundriss rechnen“, darunter die Grundrisse. „Mehrere Stockwerke gefunden“ mit Knopf je Stockwerk. Layer mit Haken und Anzahl, `Rohr` mit „von Hand · automatisch später“; „Draufsicht“; Hinweise; „DXF …“. Messungen wie bisher darunter. Gezeichnet wird aus `ShownPlan::drawing` mit dem `egui`-Painter, Mausrad zum Zeiger, Ziehen verschiebt, `F` passt ein. Ein Plan veraltet, wenn `plan_fits` falsch ist oder Karte/Modell veraltet (`map_status`).
- Texte in `text.rs`: `STEP_PLAN` bleibt; `PLAN_COMING` entfällt.

- [ ] **Step 1: Failing tests** (`tests/plan_app.rs`, `egui_kittest`, Projekt mit Karte aus `Scene` und `write_map_folder`):
  - `compute_shows_the_plan`: `Action::RunPlan` → `grundriss-001`, Modus Plan, Layer-Liste mit Anzahl, Text „Raum 1“ sichtbar.
  - `second_compute_reuses`, `several_storeys_ask_and_pick`, `stale_after_traj_changes`, `stale_map_makes_plan_stale`.
  - `layer_off_hides_it_in_the_drawing`, `top_view_can_be_switched_off`, `plan_without_top_view_file_still_shows`.
  - `zoom_keeps_the_point_under_the_cursor` (`PlanCamera`, Toleranz 0,5 Punkte), `f_fits_the_plan`, `switch_to_3d_and_back`.
  - `dxf_button_opens_export_with_placement` (Format DXF, Wahl „An den Wänden“ / „Wie die Punktwolke“), `hidden_layer_goes_into_the_export`.
  - `moved_project_plan_still_fits`, `leftover_without_report_is_offered`, `finished_plan_is_no_leftover`, `delete_plan_asks`.
  - `plan_failure_says_why`: Karte ohne Boden → Meldung, kein Ordner.
  - Bestehende Bilder: `ui_snapshots` für „Planung“ neu (`UPDATE_SNAPSHOTS=1`), `ui_layout` grün.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS (`cargo test -p driftless-app --test plan_app --test ui_snapshots --test ui_layout --test leftovers`).
- [ ] **Step 5:** Commit `feat(app): Grundriss im Feld Planung zeigen und rechnen`.

### Task 10: Werkzeug „Objekt“, Messungen speichern, Anleitung

**Files:**
- Create: `crates/driftless-app/src/ui/objects.rs`
- Modify: `ui/tools.rs` (`Tool::Object`, `TOOLS` auf 7, `picks_points`, Leiste über den Werkzeugen, `ScreenLayout.objects`), `ui/app.rs` (Treffer weitergeben, Zeichnen, Tasten, Objekte laden), `ui/view3d.rs` (nur wenn nötig: Doppelklick beendet Linienzug, rahmt nicht), `ui/brush.rs` (Oberflächen-Treffer als Zugang; ⌘Z nur bei „Objekt“ an die Objekte), `ui/mod.rs` (Actions), `ui/plan.rs`, `ui/steps/planung.rs` (Liste der Objekte, „gespeichert“), `ui/text.rs`, `ui/theme.rs` (`icon::OBJECT = p::PLUG`), `docs/anleitung.md`, `CHANGELOG.md`
- Test: `crates/driftless-app/tests/objects_app.rs`; anpassen `ui_tools.rs` (sieben Werkzeuge)

**Interfaces:**
- Consumes: `Objects`, `ObjectInput`, `PlanState`, `SurfacePick`, `shown_owner`, `paint_measurements` als Muster.
- Produces:
```rust
// ui/objects.rs
pub struct ObjectTool { pub kind: ObjectKind, pub connection: ConnectionKind, pub diameter_mm: f32, pub line: Vec<[f64; 3]>, pub message: Option<String> } // Vorgabe Steckdose, Strom, 50 mm
pub fn view_owner(state: &AppState) -> Option<PathBuf>;       // Karte/Modell der gezeigten Wolke oder Oberfläche, nur aus dem Projekt
pub fn point_clicked(state: &mut AppState, position: [f64; 3]) -> Option<Action>;
pub fn finish_line(state: &mut AppState) -> Option<Action>;   // Enter, Doppelklick
pub fn undo_point(state: &mut AppState); pub fn cancel_line(state: &mut AppState);
pub fn measured(state: &mut AppState) -> Option<Action>;      // nach einer neuen Messung: Action::AddObject, wenn view_owner da ist
pub fn bar(ctx: &Context, view: Rect, bottom: f32, state: &mut AppState) -> (Rect, Option<Action>);
pub fn paint(ui: &mut Ui, rect: Rect, camera: &Camera, origin: [f64; 3], state: &AppState);
// ui/mod.rs
Action::AddObject { owner: PathBuf, input: ObjectInput }, Action::RemoveObject { owner: PathBuf, nr: u32 }, Action::RenameObject { owner: PathBuf, nr: u32, name: String }, Action::SetObjectNote { owner: PathBuf, nr: u32, note: String },
Action::UndoObject, Action::FrameObject(u32), Action::ClearMeasurements
```
- Ohne `view_owner` ist „Objekt“ aus, mit Hinweis „Objekte brauchen eine Karte oder ein Modell des Projekts“. Objekte werden im Hintergrund in `objekte.toml` geschrieben (`HelperTask` wie die Stellen, `Objects::change`), Anzeige sofort. In 3D sichtbar im Schritt „Planung“ und bei gewähltem Werkzeug, nach Layer-Haken. Messungen: Der Schritt lädt die Messungen des `view_owner` in `AppState::measurements`; „Messungen löschen“ löscht sie als ein Rückgängig-Schritt. Ohne `view_owner` wie bisher nur im Speicher, mit dem alten Hinweis.
- `anleitung.md`: neuer Abschnitt „Grundriss und DXF“ vor „Ergebnis weitergeben“, Inhaltsverzeichnis und Verweise anpassen, Satz „sie werden noch nicht gespeichert“ ersetzen, „Bekannte Grenzen“ ergänzen (Rohre von Hand, Türen ohne Aufschlag, ein Stockwerk je Plan). `CHANGELOG.md` unter `[Unreleased]`.

- [ ] **Step 1: Failing tests** (`tests/objects_app.rs`):
  - `socket_with_one_click`, `connection_kind_is_saved`, `cable_with_clicks_and_enter`, `pipe_needs_a_diameter`, `backspace_and_esc`, `double_click_ends_without_a_double_point`.
  - `off_without_owner_with_hint`, `surface_only_is_hit_by_ray`, `cloud_is_hit_like_measuring`.
  - `undo_with_object_tool_leaves_stellen_alone`, `undo_with_brush_leaves_objects_alone`, `undo_after_restart`.
  - `objects_show_in_plan_and_dxf`, `objects_before_a_plan_stay_and_appear_later`, `objects_survive_a_new_plan`, `objects_of_a_stale_plan_stay_listed`, `rename_note_and_delete_from_list`.
  - `measurement_is_saved_and_back_after_restart`, `foreign_cloud_measurement_stays_in_memory`, `clear_measurements_is_one_undo`.
  - `ui_tools`: sieben Werkzeuge, keine Überlappung im kleinen Fenster.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen, Anleitung und CHANGELOG schreiben. **Step 4:** PASS (`cargo test -p driftless-app --test objects_app --test ui_tools`), dann `cargo test --workspace`, `cargo clippy --release -p driftless-app --bins -- -D warnings`, `cargo deny check`.
- [ ] **Step 5:** Commit `feat(app): Objekte in der 3D-Ansicht setzen, Messungen speichern`.

---

## Danach: Prüfung am Schluss

Nicht Teil der Aufgaben; Ablauf und Maßstäbe stehen in der Spec, Abschnitt 12. Reihenfolge: VM-Lauf, Korrekturen, ein Gesamt-Review (Opus), `docs/ergebnisse/2026-11-xx-grundriss.md`. Für LibreCAD auf der VM `QT_QPA_PLATFORM=offscreen`, geht `dxf2pdf` damit nicht, öffnet Yannick die Datei auf dem Mac.
