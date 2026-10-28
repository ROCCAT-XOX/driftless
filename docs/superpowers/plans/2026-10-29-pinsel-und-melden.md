# Pinsel und Melden Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In der App malt Yannick Stellen auf eine Oberfläche, glättet sie dort (leicht · stark · ganz eben) oder spielt ihre Bilder automatisch neu ein, mit Vorher/Nachher, Rückgängig, Speichern im Projekt und Export; mit ⇧⇧ meldet er überall Fehler und Wünsche in einen Ordner auf dem Rechner und sieht unter ☰ → „Meldungen“, was daraus wurde.

**Architecture:**
- **Stellen** (`driftless-steps/src/marks.rs`): je Karte bzw. Modell eine Datei `stellen.toml` im Ordner `karte-NNN` bzw. `modell-NNN`, gebaut wie `modelle/punktpaare.toml` (`driftless-steps/src/pairs.rs`). Eine Stelle ist eine Liste von Tupfern (Mitte absolut in f64, Normale, Radius, radieren) plus ihre Schritte (Glätten, Bilder neu). Alle Oberflächen der Karte/des Modells teilen sie.
- **Nacharbeit rechnen** (`driftless-map/src/surface/edit/`, nur Prozessor, deterministisch): die gerechnete glb wird verschweißt (gleiche Bits = gleicher Punkt, wie `driftless-export/src/mesh/ply.rs:27-54`), Tupfer wählen Flächen, Glätten verschiebt Punkte, „Bilder neu“ backt die Stelle auf eine **neue, angehängte Seite**. Ergebnis je Schritt ist ein `SurfacePatch`; `apply_patches` macht daraus die nachgearbeitete Oberfläche. Alte Seiten bleiben Byte für Byte, nichts wird neu gepackt.
- **Schritt** (`driftless-steps/src/touchup.rs`): rechnet alle Schritte aller Stellen der Reihe nach für eine Oberfläche und legt die Patches als Zwischenspeicher in `oberflaeche-NNN/nacharbeit/` (mit Schlüssel; passt er nicht mehr, wird neu gerechnet). `ExportSource::EditedSurface` exportiert die nachgearbeitete Oberfläche; die CLI bleibt unverändert.
- **App:** Pinsel als Werkzeug, Treffer per BVH auf der verschweißten Oberfläche (`driftless-map/src/surface/edit/bvh.rs`, im Ladeauftrag gebaut), Markierung als zweiter Zeichendurchgang, Vorher/Nachher als Ersatzpuffer je Seite plus Zusatzseiten im `MeshRenderer`. Aktionen laufen als Hintergrundaufträge über `JobRunner`.
- **Melden** (`driftless-app/src/feedback.rs`, `ui/feedback.rs`): ⇧⇧-Erkennung als reine Zustandsmaschine, Bildschirmfoto über `ViewportCommand::Screenshot`, Dialog als `egui::Modal`, je Meldung ein Ordner mit `meldung.toml` (Stand und Antwort als Klartext), Liste unter ☰.

**Tech Stack:** Rust 2021, egui/eframe/egui_kittest 0.36.2, wgpu 30, nalgebra 0.33, rayon, serde/toml, image/png (vorhanden), driftless-io (`atomic_write`, `FileLock`, `Progress`, `testscan`). Keine neue große Abhängigkeit (kein `bvh`, `parry3d`, `obvhs`).

**Spec:** `docs/superpowers/specs/2026-10-29-pinsel-und-melden-design.md`

**Vorarbeit (nur lokal, nicht im Repository):** Berichte zum Code unter `/private/tmp/claude-501/-Users-yannickstumpf/92c3665e-e920-455c-a524-07e922920a7a/scratchpad/pinsel/` – `a-oberflaeche.md` (Oberfläche, Texturen, Export), `b-app.md` (Zustand, Aktionen, Felder, Aufträge, Tastatur, Tests), `c-ansicht.md` (Treffer, Netz zeichnen, Kamera, Maus). Jede Aufgabe nennt, welchen sie lesen soll.

## Global Constraints

- Texte der App deutsch, schlicht, in `crates/driftless-app/src/ui/text.rs`; Bezeichner im Code englisch wie im übrigen Code.
- Fertige Ergebnisordner bleiben Byte für Byte; `surface_fits`/`files_intact` müssen nach jeder Nacharbeit weiter passen.
- Neue Dateien: `stellen.toml` im Karten- bzw. Modellordner, Zwischenspeicher nur unter `oberflaeche-NNN/nacharbeit/`. Kein neuer Name beginnt mit `oberflaeche`, keine Datei bleibt als `.tmp` liegen; „Aufräumen“ (`crates/driftless-app/src/leftovers.rs`) und Löschen eines Laufs/einer Karte behandeln sie wie die übrigen Dateien des Ordners.
- Nacharbeit rechnet nur der Prozessor; gleiche Stellen und Schritte ergeben auf jedem Rechner dieselben Bytes (Regeln: `a-oberflaeche.md` §4 – geordnete rayon-Maps, Sortierungen mit Gleichstandsregel, keine HashMap-Reihenfolge, keine Zeit, kein Zufall).
- Pinsel: Vorgabe 0,20 m, Regler 0,05–1,00 m; linke Taste malt, ⌥/Alt radiert, Shift+links verschiebt, rechts dreht (Umlaufmodus), Mitte verschiebt; ⌘Z bzw. Strg+Z nimmt zurück.
- ⇧⇧: zwei Tipper auf Shift (links oder rechts), je Tipper Drücken→Loslassen ≤ 0,30 s, zweites Drücken ≤ 0,40 s nach dem ersten Loslassen; jede andere Taste, jeder Text, jede Maustaste, jedes Mausrad und Fokusverlust dazwischen setzt zurück.
- Meldungsordner: macOS `~/Library/Application Support/Driftless/Meldungen/`, Windows `%LOCALAPPDATA%\Driftless\Meldungen\`, sonst `$XDG_DATA_HOME/driftless/meldungen` bzw. `~/.local/share/driftless/meldungen`. Nichts wird hochgeladen.
- Scandaten, Meldungen, Bilder aus Projekten und Kundennamen kommen nie ins Repository (auch nicht als Testdaten); Tests nutzen nur synthetische Daten (`driftless_io::testscan`, `surface/testutil.rs`).
- Schwere Läufe mit echten Daten nur auf VM 101 über `~/.local/bin/driftless-vm` (6 Threads, `nice 10`), nicht auf dem Mac.
- Vor jedem Commit grün: `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, die Tests der berührten Crates (`cargo test -p <crate>`); vor der letzten Aufgabe zusätzlich `cargo test --workspace`, `cargo clippy --release -p driftless-app --bins -- -D warnings`, `cargo deny check`. Vorher `export PATH=$HOME/.cargo/bin:$PATH`.
- Commits mit den Zeilen `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` und `Claude-Session: https://claude.ai/code/session_01EaCFfju4adDubj5wkFzZX4`.

## Review Focus

1. **Stelle über einer Seiten- oder Kartengrenze:** derselbe Punkt liegt in mehreren Seiten; Glätten muss jede Kopie gleich verschieben, die Fläche bleibt geschlossen (Test in Aufgabe 3 und 4).
2. **Rohdaten fehlen** (Laufwerk abgezogen): „Bilder neu einspielen“ aus mit Hinweis, Glätten geht, Neuanwenden beim Laden bricht nicht ab und sagt, was fehlt (Test in Aufgabe 7 und 10).
3. **Projekt verschoben oder kopiert:** Stellen und Zwischenspeicher passen weiter, nichts wird neu gerechnet (Test in Aufgabe 7).
4. **Shift im Alltag:** Shift halten zum Verschieben/Fliegen, Großbuchstaben tippen, Shift+Klick lösen ⇧⇧ nie aus (Test in Aufgabe 11).
5. **Rückgängig über einen Neustart, Löschen einer Stelle, neue Oberfläche derselben Karte, „Aufräumen“:** die Nacharbeit folgt genau der Datei (Test in Aufgabe 1 und 10).

---

### Task 1: Stellen-Datei

Lies: `b-app.md` §5; Vorbild `crates/driftless-steps/src/pairs.rs`.

**Files:**
- Create: `crates/driftless-steps/src/marks.rs`
- Modify: `crates/driftless-steps/src/lib.rs` (Modul)
- Test: im Modul (`#[cfg(test)]`) und `crates/driftless-steps/tests/marks.rs`

**Interfaces:**
- Produces:
```rust
pub const MARKS_FILE: &str = "stellen.toml";
pub struct Dab { pub centre: [f64; 3], pub normal: [f32; 3], pub radius: f32, pub erase: bool, pub seq: u64 }
pub enum SmoothLevel { Light, Strong, Flat }                       // serde: "leicht" | "stark" | "eben"
pub enum EditStep { Smooth { level: SmoothLevel, seq: u64 }, Retexture { variant: u32, seq: u64 } }
pub enum Placement { Wall, Floor, Ceiling, Surface }               // Text: Wand | Boden | Decke | Fläche
pub struct Mark { pub nr: u32, pub dabs: Vec<Dab>, pub steps: Vec<EditStep>, /* unbekannte Schlüssel bleiben */ }
pub struct Marks { pub marks: Vec<Mark>, /* next_seq, next_nr */ }
impl Mark { pub fn placement(&self) -> Placement; pub fn label(&self) -> String /* "Stelle 1 · Wand" */; pub fn next_variant(&self) -> u32; }
impl Marks {
    pub fn load(owner_dir: &Path) -> Result<Marks, MarksError>;    // fehlende Datei = leer
    pub fn save(&self, owner_dir: &Path) -> Result<(), MarksError>; // atomic_write unter FileLock
    pub fn paint(&mut self, mark: Option<u32>, dabs: &[DabInput]) -> u32;  // neue Stelle, wenn None
    pub fn add_step(&mut self, mark: u32, step: StepInput) -> Result<(), MarksError>;
    pub fn undo(&mut self) -> Option<Undone>;                       // nimmt das Jüngste (Tupfer-Zug oder Schritt)
    pub fn remove(&mut self, mark: u32) -> bool;
    pub fn fingerprint(&self) -> String;                            // Schlüssel für Zwischenspeicher, sha256 hex
    pub fn of_owner(&self) -> ...;                                  // nach Bedarf
}
```
`DabInput`/`StepInput` sind dieselben Werte ohne `seq`; ein Pinselzug (Drücken bis Loslassen) bekommt **eine** `seq` für alle seine Tupfer, damit Rückgängig den ganzen Zug nimmt.

- [ ] **Step 1: Failing tests** (`tests/marks.rs`):
  - `roundtrip_keeps_everything`: zwei Stellen mit Tupfern und Schritten speichern, laden, `assert_eq!`; eine von Hand eingefügte unbekannte Zeile `notiz = "x"` in einer Stelle übersteht Laden+Speichern.
  - `missing_file_is_empty`, `undo_takes_the_youngest`: Zug A (3 Tupfer), Schritt Glätten, Zug B → `undo` nimmt Zug B ganz, dann Glätten, dann Zug A; danach `None`.
  - `undo_survives_restart`: nach Speichern/Laden nimmt `undo` dasselbe wie vorher.
  - `remove_takes_steps_along`, `next_variant_counts_retextures` (0, 1, 2 nach zwei „Bilder neu“).
  - `placement_from_normals`: mittlere Normale (0,0,1) → Boden, (0,0,−1) → Decke, (1,0,0) → Wand, (0.6,0,0.8) → Fläche (Grenzen: |z| ≥ 0,7 Boden/Decke, |z| ≤ 0,3 Wand).
  - `fingerprint_changes_with_content_only`: gleich nach Laden/Speichern, anders nach `paint`.
  - `file_name_is_safe`: `MARKS_FILE` beginnt nicht mit `oberflaeche` und endet nicht auf `.tmp`.
- [ ] **Step 2:** `cargo test -p driftless-steps --test marks` → FAIL (Modul fehlt).
- [ ] **Step 3:** `marks.rs` umsetzen wie `pairs.rs` (Kopfkommentar in der Datei, `#[serde(flatten)] unknown: toml::Table`, `atomic_write`, `FileLock`).
- [ ] **Step 4:** `cargo test -p driftless-steps --test marks` → PASS.
- [ ] **Step 5:** Commit `feat(stellen): Stellen je Karte und Modell speichern`.

### Task 2: Verschweißte Oberfläche, Strahl und Auswahl

Lies: `a-oberflaeche.md` §1, §3; `c-ansicht.md` §3.

**Files:**
- Create: `crates/driftless-map/src/surface/edit/mod.rs`, `edit/weld.rs`, `edit/bvh.rs`, `edit/select.rs`
- Modify: `crates/driftless-map/src/surface/mod.rs` (`pub mod edit;`)

**Interfaces:**
- Consumes: `driftless_export::mesh::{Mesh, MeshPage}` (`read_mesh`).
- Produces:
```rust
pub const EDIT_VERSION: u32 = 1;
pub struct WeldedSurface { /* positions f64, faces [u32;3] über verschweißte Punkte, je Fläche (Seite, Dreieck), je Punkt seine Kopien (Seite, Index) */ }
impl WeldedSurface {
    pub fn from_mesh(mesh: &Mesh) -> WeldedSurface;   // Flächen-Id = laufender Index über Seiten in Reihenfolge
    pub fn face_count(&self) -> usize;
    pub fn positions(&self) -> &[[f64; 3]];
    pub fn faces(&self) -> &[[u32; 3]];
    pub fn face_origin(&self, face: u32) -> (u32, u32); // (Seite, Dreieck)
    pub fn face_normal(&self, face: u32) -> [f64; 3];
    pub fn copies(&self, vertex: u32) -> &[(u32, u32)]; // alle Kopien eines Punkts
}
pub struct FaceBvh { /* LBVH, Morton-sortiert, 8 Flächen je Blatt */ }
impl FaceBvh {
    pub fn build(surface: &WeldedSurface) -> FaceBvh;
    pub fn raycast(&self, s: &WeldedSurface, origin: [f64; 3], dir: [f64; 3], z_range: Option<[f64; 2]>) -> Option<RayHit>;
    pub fn faces_within(&self, s: &WeldedSurface, centre: [f64; 3], radius: f64) -> Vec<u32>; // nächster Punkt ≤ r, sortiert
}
pub struct RayHit { pub face: u32, pub t: f64, pub pos: [f64; 3] }
pub struct SelectDab { pub centre: [f64; 3], pub normal: [f32; 3], pub radius: f64, pub erase: bool }
pub fn dab_normal(s: &WeldedSurface, bvh: &FaceBvh, hit: &RayHit, radius: f64) -> [f32; 3]; // flächengewichtet im halben Radius
pub fn select_faces(s: &WeldedSurface, bvh: &FaceBvh, dabs: &[SelectDab]) -> Vec<u32>;     // sortiert, ohne Doppel
```
- Auswahl je Tupfer: Startfläche = nächste Fläche zur Mitte; von dort über gemeinsame Kanten nur Flächen, deren nächster Punkt ≤ Radius liegt und deren Normale · Tupfer-Normale > 0,5; radierende Tupfer nehmen ihre Flächen wieder heraus, in Reihenfolge der Tupfer.

- [ ] **Step 1: Failing tests** (im Modul, synthetische Netze):
  - `weld_joins_copies`: zwei Seiten, die sich eine Kante teilen (bitgleiche Ecken, eine davon −0.0 statt +0.0) → gemeinsame Punkte, `copies` hat je 2 Einträge.
  - `bvh_matches_brute_force`: 2 000 zufällige Strahlen (festes LCG) auf einem gewellten Gitter mit 20 000 Dreiecken → gleiche Fläche und `t` wie Brute Force; ebenso `faces_within`; mit Ursprung bei (5e5, 5e6, 300) m.
  - `raycast_skips_cut`: Treffer unter `z_range` werden übersprungen.
  - `selection_on_noisy_wall_has_no_holes`: Wand 2 × 2 m, Normalen ±30° verrauscht, Tupfer r = 0,2 m → ≥ 98 % der Flächen mit Schwerpunkt ≤ 0,18 m gewählt.
  - `selection_stays_on_its_side`: dünne Wand 5 cm stark, Tupfer r = 0,2 m auf der Vorderseite → keine Fläche der Rückseite; an einer 90°-Ecke keine Fläche der anderen Wand.
  - `erase_removes`, `selection_is_deterministic` (zweimal → gleiche Bytes).
- [ ] **Step 2:** `cargo test -p driftless-map edit::` → FAIL.
- [ ] **Step 3:** umsetzen (Weld nach Bits wie PLY-Export; LBVH wie `c-ansicht.md` §3; Morton/`par_sort_unstable` nur auf eindeutigen Schlüsseln).
- [ ] **Step 4:** Tests PASS; Zeit für `from_mesh`+`build` bei 6,6 Mio. synthetischen Dreiecken in einem `#[ignore]`-Test messen und im Bericht nennen.
- [ ] **Step 5:** Commit `feat(nacharbeit): Oberfläche verschweißen, treffen und Stellen wählen`.

### Task 3: Glätten

Lies: `a-oberflaeche.md` §3; `crates/driftless-map/src/smooth.rs`, `clean.rs::spread_axes`.

**Files:**
- Create: `crates/driftless-map/src/surface/edit/smooth.rs`

**Interfaces:**
- Consumes: `WeldedSurface`, Auswahl aus Task 2.
- Produces:
```rust
pub enum EditError { NotFlat { rms_m: f64 }, NoFaces, Cancelled, Map(MapError) }  // in edit/mod.rs; spätere Aufgaben ergänzen Varianten
pub enum Smoothing { Light, Strong, Flat }   // eigener Typ der Karten-Crate; Task 7 setzt `marks::SmoothLevel` darauf um
pub fn smooth_region(s: &WeldedSurface, faces: &[u32], how: Smoothing) -> Result<Vec<(u32, [f64; 3])>, EditError>;
// verschobene verschweißte Punkte, nach Id sortiert
```
- Punkte auf dem Rand der Stelle (geteilt mit Flächen außerhalb) bewegen sich nie; innen steigt das Gewicht über 0,10 m vom Rand linear auf 1.
- „Eben“: Ebene durch die Punkte der Stelle (`spread_axes`); Abweichung (RMS) > 0,03 m oder kleinste/mittlere Streuung > 0,05 → `NotFlat`, nichts verschoben.

- [ ] **Step 1: Failing tests:** Gitter 1 × 1 m, 1 cm Kanten, z mit Rauschen σ = 5 mm (festes LCG), Stelle = Scheibe r = 0,3 m in der Mitte:
  - `light_calms`: RMS zu z = 0 der Punkte ≥ 0,15 m vom Rand sinkt um ≥ 40 %; `strong_calms_more`: um ≥ 70 %; `flat_is_flat`: ≤ 0,5 mm.
  - `border_and_outside_untouched`: Randpunkte und alle Punkte außerhalb nicht in der Ausgabe.
  - `corner_is_not_flat`: Stelle über einer 90°-Ecke → `Flat` gibt `NotFlat`, `Light` geht.
  - `deterministic`: zweimal gleiche Ausgabe; `empty_selection` → `NoFaces`.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen (Vorschlag: Taubin λ/μ über die 1-Nachbarschaft in f64, feste Durchgänge; Parameter so, dass die Schwellen halten). **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat(nacharbeit): Stellen glätten (leicht, stark, ganz eben)`.

### Task 4: Änderungen anwenden und speichern

Lies: `a-oberflaeche.md` §1, §5; `crates/driftless-export/src/mesh/`.

**Files:**
- Create: `crates/driftless-map/src/surface/edit/patch.rs`

**Interfaces:**
- Produces:
```rust
pub struct SurfacePatch { pub moved: Vec<(u32, [f64; 3])>, pub removed_faces: Vec<u32>, pub pages: Vec<PatchPage> }
pub struct PatchPage { pub positions: Vec<[f64; 3]>, pub uvs: Vec<[f32; 2]>, pub indices: Vec<[u32; 3]>,
                       pub faces: Vec<u32> /* verschweißte Id je Dreieck */, pub jpeg: Arc<[u8]> }
pub fn apply_patches(mesh: &Mesh, s: &WeldedSurface, patches: &[SurfacePatch]) -> Mesh;
pub fn write_patches(dir: &Path, patches: &[SurfacePatch]) -> Result<(), EditError>;  // atomar, deterministisch
pub fn read_patches(dir: &Path) -> Result<Vec<SurfacePatch>, EditError>;
pub struct PageEdit { pub page: u32, pub positions: Option<Vec<[f64; 3]>>, pub removed: Vec<u32> /* Dreiecke der Seite */ }
pub fn page_edits(s: &WeldedSurface, mesh: &Mesh, patches: &[SurfacePatch]) -> (Vec<PageEdit>, Vec<MeshPage>); // für die Ansicht
```
- Reihenfolge: jeder Patch verschiebt alle Kopien seiner Punkte (auch in früheren Zusatzseiten), nimmt seine Flächen aus der Seite, die sie gerade hält, und hängt seine Seiten an.

- [ ] **Step 1: Failing tests:**
  - `move_keeps_watertight`: verschobener Punkt auf einer Seitengrenze → alle Kopien bitgleich verschoben; `WeldedSurface::from_mesh(apply(..))` hat dieselbe Punktzahl wie vorher.
  - `retexture_replaces_faces`: Patch mit `removed_faces` + Seite → jede Fläche genau einmal im Ergebnis, die Bilder (`Arc`-Bytes) unberührter Seiten zeigergleich bzw. bytegleich.
  - `later_patch_wins`: zwei Patches über dieselbe Fläche → nur die Seite des zweiten hält sie.
  - `patches_roundtrip_and_bytes`: `write_patches` zweimal → gleiche Bytes; `read_patches` gibt Gleiches zurück.
  - `glb_after_apply_keeps_old_pages`: `write_glb` vor/nach einem Patch, der nur Seite 1 berührt → die Daten von Seite 0 im BIN-Block bytegleich.
- [ ] **Step 2:** FAIL. **Step 3:** umsetzen. **Step 4:** PASS. **Step 5:** Commit `feat(nacharbeit): Änderungen anwenden und zwischenspeichern`.

### Task 5: Bilder neu einspielen I – Quellen, Kandidaten, Sichtbarkeit

Lies: `a-oberflaeche.md` §2 (Schritte 1–3), §4; `crates/driftless-map/src/surface/views.rs`, `crates/driftless-io/src/images.rs`, `crates/driftless-steps/src/surface.rs:744-808`.

**Files:**
- Create: `crates/driftless-map/src/surface/edit/retexture.rs` (erster Teil)
- Modify: `crates/driftless-io/src/images.rs` (`for_each_image_where`), `crates/driftless-map/src/surface/views.rs` (`image_views` nimmt Kandidatenlisten; der volle Weg übergibt dieselben Listen wie heute), `crates/driftless-steps/src/surface.rs` (`texture_sources`)

**Interfaces:**
- Produces:
```rust
// driftless-io
pub fn for_each_image_where(bags: &[PathBuf], topics: &[&str], downscale: u32, want: impl Fn(u64 /*stamp ns*/) -> bool,
                            f: impl FnMut(ImageFrame) -> Result<(), IoError>) -> Result<(), IoError>;
// driftless-steps/src/surface.rs
pub fn texture_sources(surface_dir: &Path) -> Result<Option<Vec<TextureSource>>, StepError>;
// Karte oder Modell, ohne clean.bin; findet Rohdaten, Laufbahn und Kalibrierung so wie der Oberflächen-Schritt
// (Lauf → `lauf.toml`, Modell → `modell.json` + `member_transform`); `Ok(None)`, wenn Rohdaten fehlen
// driftless-map edit/retexture.rs
pub(crate) struct RegionView { pub source: u32, pub frame: u32, pub stamp: u64, /* Lage, Kamera */ pub faces: Vec<(u32, f32)> /* Fläche, Güte */ }
pub(crate) fn region_views(s: &WeldedSurface, faces: &[u32], sources: &[TextureSource], cfg: &SurfaceConfig) -> Vec<RegionView>;
pub(crate) fn decode_views(views: &[RegionView], sources: &[TextureSource], k: usize, progress: &Progress) -> Result<Vec<(usize, RgbImage)>, EditError>;
```
- Kandidaten nur über Zeitstempel und Lage (kein JPEG), Sichtbarkeit über einen Tiefenpuffer nur aus Flächen, die ins Bildfeld der Stelle fallen; höchstens K = 24 Bilder voll dekodiert.
- `SURFACE_VERSION` bleibt; die volle Oberfläche ist danach bytegleich (`one_source_pages_unchanged`, `files_unchanged_by_move`, `byte_identical_twice` grün).

- [ ] **Step 1: Failing tests:** mit `testscan::scan_with_images` (Karte + Oberfläche wie in den bestehenden Oberflächen-Tests):
  - `texture_sources_match_surface_step`: dieselben Quellen wie der Oberflächen-Schritt (Anzahl Bilder, Kameras).
  - `region_views_see_the_region`: für eine Wandstelle nur Bilder, deren Blickfeld die Stelle trifft; ein Bild hinter einer Wand der Szene ist nicht dabei.
  - `decode_only_k`: höchstens 24 dekodierte Bilder; gleiches Ergebnis zweimal.
  - Bestehende Tests der Oberfläche unverändert grün.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS (`cargo test -p driftless-io -p driftless-map -p driftless-steps`).
- [ ] **Step 5:** Commit `feat(nacharbeit): Bilder für eine Stelle finden`.

### Task 6: Bilder neu einspielen II – Wahl, eigene Seite, Rand angleichen

Lies: `a-oberflaeche.md` §2 (Schritte 4–9), §6; `atlas.rs` (`bake`, `split_chart`, `pack`, `absorb_islands`), `level.rs`, `sharpness.rs`.

**Files:**
- Modify: `crates/driftless-map/src/surface/edit/retexture.rs`, `atlas.rs` (`bake` in Aufteilen und Füllen trennen, ohne Änderung des vollen Wegs), `level.rs` (`level_region`)

**Interfaces:**
- Consumes: Task 5.
- Produces:
```rust
pub fn retexture_region(s: &WeldedSurface, current: &Mesh, faces: &[u32], sources: &[TextureSource],
                        variant: u32, cfg: &SurfaceConfig, progress: &Progress) -> Result<RetextureDone, EditError>;
pub struct RetextureDone { pub patch: SurfacePatch, pub variant_of: (u32, u32) /* (k+1, n) für „Variante 2 von 5“ */ }
```
- Wahl: Bilder nach Güte über die Stelle (Abdeckung ≥ 0,9 zuerst, dann Güte · Schärfe auf der Stelle selbst); `variant` k nimmt das k-te (k mod n); Flächen, die es nicht sieht, bekommen per ICM bei festgehaltenem Außen das nächstbeste.
- Helligkeit je Bild und Kanal gegen einen Ring alter Textur rund um die Stelle; am Rand gleicht nur die neue Seite an (voller Unterschied), innen wie heute.
- Neue Seite: kleinste Zweierpotenz ≥ nötig, 256–4096, Texelgröße wie die Oberfläche (aus dem Bericht); passt die Stelle nicht auf eine Seite, mehrere.
- Ohne Bild, das die Stelle sieht → `EditError::NoImages`; die Stelle behält ihre Textur.

- [ ] **Step 1: Failing tests** (synthetisch, `scan_with_images`):
  - `new_page_for_region`: Patch entfernt genau die Flächen der Stelle und bringt ≥ 1 Seite, deren Dreiecke genau diese Flächen tragen.
  - `border_matches_outside`: mittlerer Farbunterschied über die Randkanten (Texelpaare beidseits) nach dem Angleichen ≤ vor dem Angleichen und ≤ 6/255, wenn das Außen aus demselben Bild mit 20 % anderer Belichtung stammt.
  - `variants_cycle`: `variant` 0, 1, … nimmt verschiedene Bilder, `variant = n` wieder das erste; `variant_of` stimmt.
  - `deterministic`: zweimal gleiche Patch-Bytes (`write_patches`).
  - `full_surface_unchanged`: bestehende Byte-Tests der Oberfläche grün.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** Commit `feat(nacharbeit): Bilder einer Stelle neu einspielen`.

### Task 7: Nacharbeit-Schritt und Export

Lies: `a-oberflaeche.md` §5; `crates/driftless-steps/src/surface.rs` (`surface_fits`, Bericht), `export.rs` (`ExportSource`, `read_source`, `files`).

**Files:**
- Create: `crates/driftless-steps/src/touchup.rs`
- Modify: `crates/driftless-steps/src/lib.rs`, `crates/driftless-steps/src/export.rs` (`ExportSource::EditedSurface`)
- Test: `crates/driftless-steps/tests/touchup.rs`

**Interfaces:**
- Consumes: Task 1 (`Marks`), Tasks 2–6.
- Produces:
```rust
pub const TOUCHUP_DIR: &str = "nacharbeit";
pub struct TouchupRequest { pub surface_dir: PathBuf, pub owner_dir: PathBuf, pub sources: Option<Vec<TextureSource>> }
pub struct TouchupDone { pub steps: usize, pub hints: Vec<String> /* deutsch, für die App */, pub refused: Vec<(u32, String)> }
pub fn touchup_fits(surface_dir: &Path, marks: &Marks) -> bool;                   // Schlüssel: input_id + glb-Größe + fingerprint + EDIT_VERSION
pub fn run_touchup(req: &TouchupRequest, progress: &Progress) -> Result<TouchupDone, StepError>;
pub fn edited_mesh(surface_dir: &Path, progress: &Progress) -> Result<Option<Mesh>, StepError>; // None ohne passenden Zwischenspeicher
pub fn edited_pages(surface_dir: &Path, s: &WeldedSurface, mesh: &Mesh) -> Result<Option<(Vec<PageEdit>, Vec<MeshPage>)>, StepError>;
// export.rs
ExportSource::EditedSurface(PathBuf) // Oberflächenordner; liest edited_mesh, Standardname wie die Oberfläche
```
- Ablauf je Stelle in `seq`-Reihenfolge: Auswahl → je Schritt: Glätten (danach von selbst Bilder neu, Variante 0, falls Quellen da) oder Bilder neu (Variante aus dem Schritt). „Nicht eben genug“ und „keine Bilder“ landen in `refused`/`hints`, die übrigen Schritte laufen weiter.
- Ohne Quellen (`sources: None`): Glätten mit alter Textur, Hinweis „Rohdaten fehlen – Bilder nicht neu eingespielt“; „Bilder neu“ wird übersprungen mit demselben Hinweis.
- Abbruch: kein halber Zwischenspeicher (schreiben in `nacharbeit.tmp-…`, dann umbenennen; Reste entfernt der nächste Lauf).

- [ ] **Step 1: Failing tests** (`scan_with_images` → Karte → Oberfläche im Testordner):
  - `smooth_then_export_contains_it`: Stelle + Glätten stark → `run_touchup`, `edited_mesh` hat bewegte Punkte und ≥ 1 Seite mehr; Export glb/OBJ/PLY über `EditedSurface` enthalten sie; die Dateien des Oberflächenordners sind bytegleich wie vorher, `surface_fits` wahr.
  - `cache_is_reused`: zweiter Aufruf ohne Änderung → `touchup_fits` wahr, nichts neu geschrieben (mtime gleich).
  - `moved_project_keeps_cache`: Projekt kopieren (`copy_dir` wie `crates/driftless-app/tests/portable.rs`) → `touchup_fits` im neuen Ort wahr.
  - `new_surface_reapplies`: zweite Oberfläche derselben Karte (andere Feinheit) → `run_touchup` dort erzeugt eigene Patches aus denselben Stellen.
  - `no_raw_data`: Quellen `None` → Glätten wirkt, Hinweis gesetzt, kein Fehler.
  - `cancel_leaves_nothing`: Abbruch über `Progress` → kein `nacharbeit/`, keine Reste nach dem nächsten Lauf.
  - `cli_export_unchanged`: `ExportSource::Surface` exportiert weiter die gerechnete Oberfläche.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** Commit `feat(nacharbeit): Nacharbeit rechnen, zwischenspeichern und exportieren`.

### Task 8: Markierung und Vorher/Nachher zeichnen

Lies: `c-ansicht.md` §2, §6; `crates/driftless-app/src/render/mesh.rs`, `render/mesh.wgsl`, `ui/view3d.rs` (`FrameKey`).

**Files:**
- Modify: `crates/driftless-app/src/render/mesh.rs`, `render/mesh.wgsl`
- Create: `crates/driftless-app/src/render/mark.wgsl`
- Modify: `crates/driftless-app/src/ui/view3d.rs` (`FrameKey` um `marks: u64`, `edit: u64`, `show_edit: bool`)
- Test: `crates/driftless-app/tests/mesh.rs`

**Interfaces:**
- Consumes: `PageEdit`, `MeshPage` aus Task 4 (die App wandelt in `MeshVertex` relativ zum Ursprung).
- Produces:
```rust
pub struct MarkTriangle { pub pos: [[f32; 3]; 3], pub boundary: [bool; 3], pub color: [u8; 4] }
impl MeshRenderer {
    pub fn set_marks(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, tris: &[MarkTriangle]);
    pub fn set_edit(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, edit: Option<MeshEdit>);
    pub fn show_edit(&mut self, on: bool);
}
pub struct MeshEdit { pub pages: Vec<(u32, Option<Vec<MeshVertex>>, Option<Vec<u32>>)>, pub extra: Vec<(Vec<MeshVertex>, Vec<u32>, RgbaImage)> }
```
- Markierung: zweiter Durchgang über kopierte Dreiecke, `@invariant`-Position in beiden Shadern, `GreaterEqual` ohne Tiefe schreiben, Alpha-Mischung, Umriss aus Baryzentrik an Randkanten, Farben aus `SCAN_COLORS`.
- Vorher/Nachher: Ersatzpuffer je berührter Seite plus Zusatzseiten; Umschalten lädt nichts hoch.

- [ ] **Step 1: Failing tests** (Offscreen wie `tests/mesh.rs`, ohne Grafikkarte übersprungen):
  - `mark_tints_and_outlines`: Fläche mit Markierung → Pixel in der Mitte getönt, am Rand Umrissfarbe, außerhalb unverändert; kein Z-Flimmern (zwei Kamerawinkel).
  - `edit_toggle_swaps_without_upload`: `set_edit` mit verschobener Seite + Zusatzseite; `show_edit(true/false)` zeigt beide Stände; Byte-Zähler des Hochladens bleibt beim Umschalten gleich.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS (`cargo test -p driftless-app --test mesh`).
- [ ] **Step 5:** Commit `feat(ansicht): Stellen markieren und Vorher/Nachher zeichnen`.

### Task 9: Pinsel in der App

Lies: `b-app.md` §1–§5, §9; `c-ansicht.md` §1, §3–§5.

**Files:**
- Create: `crates/driftless-app/src/ui/brush.rs` (Leiste, Malen, Liste)
- Modify: `ui/tools.rs` (`Tool::Brush`, `TOOLS`, `floats`, `ScreenLayout.brush`, Hinweise), `ui/view3d.rs` (Maus: links malt, rechts dreht beim Pinsel, Doppelklick aus), `ui/mod.rs` (`AppState`: Stellen der gezeigten Karte/des Modells, aktive Stelle; `Action`s), `ui/app.rs` (`apply`, Ladeauftrag baut `WeldedSurface`+`FaceBvh`), `ui/steps/bilder.rs` + `tree.rs` (Liste „Markierte Stellen“), `surface.rs` (`LoadedSurface` trägt die Treffer-Daten), `camera.rs` (`frame_facing`), `ui/text.rs`
- Test: `crates/driftless-app/tests/brush.rs`; anpassen: `tests/ui_tools.rs` (`TOOLS`, `small_window_no_overlap`)

**Interfaces:**
- Consumes: Task 1, 2, 8.
- Produces (für Task 10 und 11):
```rust
Action::Paint { owner: PathBuf, mark: Option<u32>, dabs: Vec<DabInput> }   // ein ganzer Zug
Action::SelectMark(Option<u32>), Action::FrameMark(u32), Action::DeleteMark(u32), Action::UndoMark
pub fn Camera::frame_facing(&mut self, centre: [f32; 3], normal: [f32; 3], radius: f32, aspect: f32);
```
- Malen: Tupfer im Abstand r/4 entlang der Mausbahn, Treffer auf dem UI-Faden, Normale per `dab_normal`; Anzeige der aktiven Stelle wird bei jedem Tupfer neu gebaut (`select_faces`, `set_marks`); gespeichert wird beim Loslassen (ein Zug = ein `Action::Paint`, über `HelperTask`).
- Ohne Oberfläche: Werkzeug aus mit Hinweis „Pinsel braucht eine Oberfläche“; ohne Fehler auch bei nur gezeigter Fläche (kein Punktwolken-Zwang).
- Liste unter der Karte/dem Modell im Feld „Bilder“ (wie `tree::pair_list`): „Stelle 1 · Wand“, darunter der letzte Schritt; Klick wählt und zeigt (`frame_facing`), Rechtsklick „Löschen“.

- [ ] **Step 1: Failing tests** (`tests/brush.rs`, ganze App mit Grafik wie `tests/ui_tools.rs`, sonst übersprungen; Projekt mit kleiner Oberfläche wie `ui_surface.rs::project_with_map`):
  - `paint_creates_mark_and_file`: Pinsel wählen, Zug über die Fläche → `stellen.toml` im Kartenordner mit 1 Stelle, Tupfer ≥ 3, Liste zeigt „Stelle 1 · Wand“.
  - `alt_erases`, `shift_pans_not_paints`, `right_drag_orbits`.
  - `undo_and_restart`: Zug, ⌘Z → leer; Zug, Neustart der App, ⌘Z → leer.
  - `brush_off_without_surface` (Hinweistext), `delete_from_list`.
  - `ui_tools` grün mit sechs Werkzeugen, keine Überlappung im kleinen Fenster.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS (`cargo test -p driftless-app --test brush --test ui_tools`).
- [ ] **Step 5:** Commit `feat(app): Pinsel – Stellen malen, radieren, zurücknehmen`.

### Task 10: Nacharbeit in der App

Lies: `b-app.md` §4, §5; `c-ansicht.md` §2b.

**Files:**
- Modify: `ui/brush.rs` (Aktionsleiste: Glätten leicht/stark/ganz eben, Bilder neu einspielen, Vorher/Nachher, ↶, Melden … als Platzhalter-Aktion für Task 11), `ui/actions.rs` (`start_step` für Nacharbeit), `ui/mod.rs` (`Action::Touchup`, `step_folders`), `jobs.rs` (`JobOutput::Touchup`), `ui/app.rs` (`finish_step`, beim Laden einer Oberfläche: passt der Zwischenspeicher nicht → Auftrag einreihen; sonst `edited_pages` laden und `set_edit`), `ui/export.rs` (`shown_sources` → `EditedSurface`, wenn Nacharbeit da), `ui/text.rs`
- Test: `crates/driftless-app/tests/touchup_app.rs`

**Interfaces:**
- Consumes: Task 7, 8, 9.
- Produces: `Action::Smooth { mark: u32, level: SmoothLevel }`, `Action::Retexture { mark: u32 }`, `Action::ShowEdit(bool)`, `Action::OpenFeedback { mark: Option<u32> }` (Task 11 füllt den Dialog).
- Aktionen: Schritt in `stellen.toml` eintragen, dann `run_touchup` als Hintergrundauftrag mit Fortschritt und „Abbrechen“; danach Ansicht aktualisieren. Abgelehnte Schritte („nicht eben genug“, „keine Bilder“) als klare Meldung, Schritt wird wieder ausgetragen. Ohne Rohdaten ist „Bilder neu einspielen“ aus, mit Hinweis.
- Neue Oberfläche derselben Karte/desselben Modells (fertig gerechnet oder geladen) → Nacharbeit wird von selbst eingereiht.
- ↶/⌘Z auf einen Schritt (nicht nur einen Zug) und „Löschen“ einer Stelle rechnen die Nacharbeit ebenso neu.

- [ ] **Step 1: Failing tests** (ohne Fenster wie `tests/portable.rs`, mit `scan_with_images`):
  - `smooth_runs_and_shows`: Stelle malen (Action::Paint), `Action::Smooth{Strong}` → nach `settle` Zwischenspeicher passt, Ansicht hat eine Bearbeitung, `Action::ShowEdit(false)` zeigt das Gerechnete.
  - `refused_flat_is_taken_back`: Stelle über eine Ecke, `Flat` → Meldung „nicht eben genug“, kein Schritt in der Datei.
  - `no_raw_data_disables_retexture`: Rohdaten-Ordner umbenennen → Aktion aus, Glätten geht.
  - `new_surface_gets_touchup`: zweite Oberfläche rechnen → Nacharbeit eingereiht und fertig.
  - `export_contains_touchup`, `cleanup_keeps_marks` („Aufräumen“ lässt `stellen.toml` und `nacharbeit/` stehen).
  - `undo_step_recomputes`: Glätten, dann ⌘Z → Zwischenspeicher passt wieder zum Stand ohne Glätten.
  - `new_map_starts_empty`: Karte mit anderer Genauigkeit neu rechnen → neue `karte-NNN` ohne Stellen, die alte behält ihre.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS.
- [ ] **Step 5:** Commit `feat(app): Stellen glätten und Bilder neu einspielen`.

### Task 11: Melden – Ordner, ⇧⇧, Dialog, Liste

Lies: `b-app.md` §6–§8, §10; `c-ansicht.md` §4.

**Files:**
- Create: `crates/driftless-app/src/feedback.rs` (Ordner, Datei, Lesen, ⇧⇧-Erkennung), `crates/driftless-app/src/ui/feedback.rs` (Dialog und Liste)
- Modify: `ui/rail.rs` (☰ → „Meldungen“), `ui/app.rs` (Erkennung nach `tools::keys`, Bildschirmfoto, Kontext, `Zur Ansicht`), `camera.rs` (`CameraState` mit serde, `state(origin)`, `restore`), `ui/view3d.rs` (`view_origin()`), `ui/text.rs`, `lib.rs`
- Test: `crates/driftless-app/tests/feedback.rs`, Einheitstests in `feedback.rs`

**Interfaces:**
- Consumes: `Action::OpenFeedback { mark }` (Task 10), `Marks` (Task 1).
- Produces:
```rust
pub fn feedback::default_dir() -> Option<PathBuf>;                  // Pfade aus Global Constraints
pub enum FeedbackKind { Bug, Wish }                                 // „Fehler“ | „Wunsch“
pub struct Feedback { pub kind, pub text, pub created: String /* ISO, Ortszeit */, pub version, pub system, pub graphics,
                      pub project: Option<(String, PathBuf)>, pub selection: Option<String>, pub camera: Option<CameraState>,
                      pub status: String /* "offen" */, pub answer: String, /* unbekannte Schlüssel bleiben */ }
pub fn feedback::write(dir: &Path, f: &Feedback, screenshot: Option<&ColorImage>, marks: Option<&Marks>, log: &[String]) -> Result<PathBuf, FeedbackError>;
pub fn feedback::read_all(dir: &Path) -> Vec<(PathBuf, Feedback)>; // neueste zuerst, kaputte überspringen
pub struct DoubleShift { .. } impl DoubleShift { pub fn feed(&mut self, events: &[egui::Event], time: f64) -> bool; }
```
- Ordner je Meldung `JJJJ-MM-TT_hhmmss_fehler|wunsch` (bei Gleichstand `-2`, `-3`), darin `meldung.toml`, `fenster.png`, `stellen.toml` (Kopie), `protokoll.txt` (letzte 200 Zeilen).
- ⇧⇧ → Bildschirmfoto anfordern → Dialog öffnen, sobald das Bild da ist (das Bild zeigt das Fenster ohne Dialog).
- Liste: neueste zuerst, Art, Zeit, Projekt, Text, Stand, Antwort; „Ordner öffnen“, „Zur Ansicht“ (Projekt öffnen, Auswahl und Kamera nach dem Hochladen setzen). Stand und Antwort liest die Liste bei jedem Öffnen neu.

- [ ] **Step 1: Failing tests:**
  - Einheit `double_shift`: Tipp-Tipp (0,1 s / 0,2 s Abstand) → wahr; Shift 0,5 s gehalten → nie; Shift–A–Shift → nie; Shift+Klick → nie; zu langer Abstand (0,6 s) → nie; links+rechts gemischt → wahr.
  - Einheit `write_and_read`: zwei Meldungen in derselben Sekunde → zwei Ordner; `read_all` neueste zuerst; von Hand `status = "erledigt"`, `answer = "erledigt in 0.2.0: …"` und ein unbekannter Schlüssel → gelesen, nach erneutem Schreiben erhalten.
  - `dialog_saves` (kittest, `with_step_dt`): ⇧⇧ → Dialog, Text tippen (auch ein Großbuchstabe), „Melden“ → Ordner mit `meldung.toml` und `fenster.png` (mit Grafik), `protokoll.txt`; Esc schließt ohne zu speichern.
  - `list_shows_answer_and_goes_back`: Meldung mit Projekt und Kamera, Stand von außen geändert → Liste zeigt ihn; „Zur Ansicht“ öffnet das Projekt, Auswahl und Kamera wie gespeichert (Abstand < 1 mm, Winkel < 0,01°).
  - `brush_bar_reports_mark`: `Action::OpenFeedback{ mark: Some(1) }` hakt „Markierte Stellen anhängen (1)“ an.
- [ ] **Step 2–4:** FAIL → umsetzen → PASS (`cargo test -p driftless-app --lib feedback --test feedback`).
- [ ] **Step 5:** Commit `feat(app): Melden mit ⇧⇧ und Liste der Meldungen`.

### Task 12: Anleitung, Messung auf echten Daten, Bilder

Lies: `docs/anleitung.md`, `CHANGELOG.md`, `docs/ergebnisse/2026-10-28-version-0-1.md` (Form), `crates/driftless-app/tests/whole_run_real.rs` (Muster für einen `#[ignore]`-Lauf).

**Files:**
- Modify: `docs/anleitung.md` (Abschnitte „Stellen nacharbeiten“ und „Melden“, Ordner der Meldungen), `CHANGELOG.md` (`## [Unreleased]`)
- Create: `crates/driftless-app/tests/touchup_real.rs` (`#[ignore]`, `DRIFTLESS_TESTDATA`, `DRIFTLESS_WORK`, optional `DRIFTLESS_SHOTS`), `docs/ergebnisse/2026-10-29-pinsel.md`

**Interfaces:** Consumes alle vorigen.

- [ ] **Step 1:** `touchup_real.rs`: Projekt mit S1–S3 als Modell und Büro (wie `whole_run_real`), je eine Wandstelle von etwa 1 m² malen (per `Action::Paint` mit festen Tupfern), „ganz eben“, „stark“, „Bilder neu“ (Varianten 0 und 1); misst je Aktion Zeit und Speicher, RMS der Punkte zur Ebene vorher/nachher, Sprung am Rand (mm), Farbunterschied am Rand; rendert mit `DRIFTLESS_SHOTS` Vorher/Nachher-Bilder.
- [ ] **Step 2:** auf VM 101 laufen lassen: `driftless-vm sync`; in `/home/yannick/td/jmk6-2026-10-22/` fehlen die Verweise auf `20261022132520` und `20261022133028` (Ziel `/home/yannick/scans/<id>`, wie der vorhandene auf `20261022133554`) – anlegen; dann `driftless-vm run env DRIFTLESS_TESTDATA=/home/yannick/td DRIFTLESS_WORK=/home/yannick/work-pinsel cargo test -p driftless-app --release --test touchup_real -- --ignored --nocapture`, danach `/home/yannick/work-pinsel` wieder löschen; Zahlen in `docs/ergebnisse/2026-10-29-pinsel.md` (ohne Bilder, ohne Kundennamen). Ziele: Aktion auf 1 m² ≤ 5 s, Rand ohne sichtbaren Sprung (≤ 1 mm), Malen auf Büro ≥ 30 Bilder/s (Messung wie `tests/view3d.rs:419`).
- [ ] **Step 3:** Anleitung und CHANGELOG schreiben; `cargo test --workspace`, Release-Clippy, `cargo deny check` grün.
- [ ] **Step 4:** Commit `docs: Pinsel und Melden in der Anleitung, Messung auf echten Daten`.
