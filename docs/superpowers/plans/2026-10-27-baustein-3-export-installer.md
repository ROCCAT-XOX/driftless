# Baustein 3 – Export, Formate öffnen, Installer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Driftless exportiert Punktwolken (LAS 1.4, LAZ, E57, PLY, PCD), Oberflächen (glb, OBJ, PLY-Netz) und Laufbahnen (TUM, CSV) über `driftless export` und die App, öffnet dieselben Formate in der App und kommt als `.dmg` (macOS arm64) und `Driftless-<version>-setup.exe` (Windows x64) aus der CI.

**Architecture:**
- **Neuer Crate `driftless-export`** (unten im Baum, hängt nur an `driftless-io` und `driftless-traj`):
  - Ein Punkte-Modell `Points` (Spalten, xyz in f64, Farbe 16 Bit) und ein Lese-Merkmal `PointSource`, über das jeder Schreiber Punkte holt, ohne sie zu kopieren.
  - Je Format ein Modul mit Schreiber und Leser: `las` (auch LAZ), `ply`, `pcd`, `e57`, `mesh` (glb, OBJ, PLY-Netz), `traj` (TUM, CSV).
- **Kein zweiter Schreiber:** Die heutigen Schreiber ziehen um.
  - `write_las*`/`write_ply*` aus `driftless-map/src/export.rs` und `write_glb`/`write_obj` aus `driftless-map/src/surface/write.rs` werden in `driftless-map` zu dünnen Hüllen: `Cloud` bzw. `TexturedMesh` → `driftless-export`.
  - `driftless-steps/src/scan.rs::write_las` wird ebenfalls eine Hülle.
  - Der PLY-Leser der App (`driftless-app/src/ply.rs`) zieht nach `driftless-export/src/ply.rs`.
- **Schritt `driftless-steps/src/export.rs`:** `ExportRequest` → `run_export` mit `Progress`/Abbruch; die CLI (`driftless export`) und der App-Auftrag rufen ihn.
  - Die App liest Dateien direkt über `driftless-export`, so wie sie heute LAS direkt über `las` liest. Schreiben geht nur über den Schritt.
- **Installer:**
  - macOS: Skripte unter `packaging/macos/` (`Driftless.app` mit ad-hoc-Signatur, `hdiutil`). Lokal auf dem Mac bau- und prüfbar.
  - Windows: Inno Setup unter `packaging/windows/`, nur in der CI prüfbar.
  - Workflows: `package.yml` (PR-Artefakte) und `release.yml` (Tag `v*` → Release-**Entwurf**).

**Tech Stack:** Rust 2021, las 0.11 (Merkmal `laz`, laz 0.13), e57 0.11, gltf/gltf-json 1.4.1, image 0.25 (jpeg, png), png 0.18, nalgebra 0.33, egui/eframe 0.36, rfd 0.17, clap 4; Inno Setup 6 (auf `windows-latest` vorinstalliert), `hdiutil`/`codesign` (macOS); Python im Benutzerkonto auf VM 101 (laspy+lazrs, pye57, open3d, trimesh, scipy).

**Spec:** `docs/superpowers/specs/2026-10-27-export-installer-design.md` (Kontext: `docs/superpowers/specs/2026-10-20-driftless-design.md`, Ziel 4 und 6).

**Codeübersicht (Stand 27.10., Zweig `baustein-3` = 4e867f4):**
- `driftless-map/src/export.rs` schreibt LAS **1.2** (Format 3/1), Versatz = abgerundetes Minimum, Maßstab `LAS_SCALE = 0.0001`. PLY ist binär LE: `x y z intensity` float, `red green blue` uchar, `colored` uchar, `scan` ushort. Aufrufer:
  - `steps/map.rs` (dense/clean/removed/color),
  - `steps/merge.rs` (`write_whole` → `modell.las/.ply`),
  - viele Tests.
- `las::Writer::from_path` setzt `is_compressed` **nach der Dateiendung**. Eine temporäre Datei `x.laz.tmp` würde also unkomprimiert. LAZ muss über `Writer::new(BufWriter<File>, header)` mit gesetztem `point_format.is_compressed` geschrieben werden.
- `surface/write.rs`:
  - `write_glb(path, &TexturedMesh, &PageJpegs)` und `write_obj(dir, stem, …)`, deterministisch, je Datei `.tmp` + Umbenennen.
  - Die Tests dort (`glb_roundtrip`, `obj_files_complete`, `byte_identical_twice`, …) bleiben die Wache für unveränderte Bytes.
- App:
  - `cloud.rs::load_las` (Ursprung aus Kopf-Grenzen, 8-Bit-Erkennung über niedrige Bytes, Qualität aus `gps_time`), `ply.rs::load_ply` (663 Zeilen eigener Parser) und `surface.rs::load_glb` (nur eingebettete JPEG, keine Knoten-Transformation).
  - ☰ hat „Punktwolke öffnen …“ (`Action::PickOpenCloud`) und „Oberfläche öffnen …“ (`Action::PickOpenSurface`); `ui/app.rs::dropped` nimmt `.las/.ply/.glb`.
  - `actions::start_step` verlangt ein Projekt.
  - Ein Fenster „Über Driftless“ gibt es nicht, ebenso keine Versions- oder Commit-Anzeige und kein Programmsymbol.
- CI (`.github/workflows/ci.yml`): macOS-14 + windows-latest, lädt bei PRs `driftless-app.exe` als Artefakt hoch. Es gibt keinen Paket- und keinen Release-Workflow.

## Global Constraints

- **Lizenz** Apache-2.0 OR MIT, keine GPL/AGPL. Neue Abhängigkeiten nur diese:
  - `e57 = "0.11"` (MIT)
  - `las` mit Merkmal `laz` (laz 0.13, Apache-2.0)
  - `image` Merkmal `png`
  - `gltf`/`png` wie schon im Workspace

  `cargo deny check` bleibt grün. Inno Setup ist nur Bauwerkzeug.
- **Keine Scan-Daten, Kundennamen oder Zugangsdaten** im Repo, in Fixtures oder im Installer. Fixtures sind synthetisch (fester Zufallsstartwert). Tests mit echten Daten sind `#[ignore = "needs DRIFTLESS_TESTDATA"]` über `driftless_io::testdata::path`.
- **Koordinaten:** Meter, Rahmen der Karte bzw. des Modells, Z oben. Nur glTF hat Y oben: `(x, y, z)` → `(x, z, −y)`.
  - LAS/LAZ: LAS 1.4, Maßstab 0,0001 m, Versatz = je Achse `round((min+max)/2)` in ganzen Metern.
  - E57: f64.
  - PLY/PCD: `float`, aber `double` für xyz, sobald eine Koordinate |v| ≥ `MAX_FLOAT_COORD = 8192.0` m ist.
- **Farbe** intern 16 Bit. Aus 8-Bit-Formaten wird ×257 gelesen, in 8-Bit-Formate wird `round(v / 257)` geschrieben. Intensität intern 0–65535, in PLY/PCD als float 0–255 (`v / 257`), wie heute in `color.ply`.
- **Scan-Nummer:**
  - LAS/LAZ `point_source_id`,
  - PLY `scan` (ushort),
  - PCD Feld `scan` (U 2),
  - E57 ein `data3D`-Eintrag je Scan mit Name und Lage.
- **Atomar:** Jede Datei geht nach `<name>.tmp` daneben und wird erst nach der letzten Abbruchprüfung umbenannt. Fehler und Abbruch lassen keine `.tmp` zurück. Bei mehreren Dateien (OBJ) werden alle erst als `.tmp` geschrieben und dann umbenannt, die `.obj` zuletzt.
- **Kein stilles Überschreiben:** Die CLI braucht `--ueberschreiben`, die App fragt. Ziel = Quelle wird immer abgelehnt.
- **Speicher:** Ein Export hält die Wolke einmal als `Points`. Der Spitzen-Speicher bleibt ≤ 2 × `Points`-Bytes + 64 MB. Gelesen und geschrieben wird in Blöcken von `BLOCK = 1 << 20` Punkten, je Block Fortschritt und Abbruchprüfung.
- **Deterministisch:** gleiche Eingabe → gleiche Bytes. Kein Datum im Kopf; E57-GUIDs aus FNV-1a 64 über (Dateiname, Punktzahl, Scan-Nummer), als `xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx`.
- **Texte** für Nutzer deutsch und kurz, App-Texte in `crates/driftless-app/src/ui/text.rs`. Code-Kommentare englisch im Stil der Datei. README englisch, UI-Wörter deutsch zitiert.
- **CI** auf macOS und Windows grün:
  - `cargo fmt --all -- --check`
  - `cargo clippy --workspace --all-targets -- -D warnings`
  - `cargo test --workspace`
  - `cargo clippy --release -p driftless-app --bins -- -D warnings`
  - `cargo deny check`
- **Schwere Läufe** nur auf VM 101: `DRIFTLESS_RDIR=driftless-b3 ~/.local/bin/driftless-vm sync|run …`.
  - 6 Threads, nice 10, ein schwerer Job zur Zeit.
  - `sync` nimmt den **committeten** Stand.
  - Lange Läufe als `nohup setsid`-Skript mit Log; begrenzt nachfragen (alle 60 s, höchstens 30 min, dann melden).
  - Testdaten: `DRIFTLESS_TESTDATA=/home/yannick/td/b1`; Modell der drei Scans aus dem schwarzen Raum: `/home/yannick/td/b1fix/t9/m/`.
  - Python nur `pip install --user`, kein sudo.
- **Mac:** Tests und das `.dmg` dürfen lokal laufen, Exporte echter Daten nicht.
- **Push, PR, Merge, Tag, Release** nur nach Yannicks Okay. Ein Release bleibt Entwurf.
- **Commits** mit den Trailern der Sitzung, vor dem Bauen `export PATH=$HOME/.cargo/bin:$PATH`:
  ```
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01EaCFfju4adDubj5wkFzZX4
  ```
- **Signieren/Notarisieren** gehört nicht dazu. macOS bekommt nur eine ad-hoc-Signatur (`codesign -s -`, ohne Zugangsdaten), damit Rechtsklick → Öffnen geht.

## Review Focus

1. **Das Ziel gibt es schon oder es ist die Quelle selbst**, auch die Nebendateien eines OBJ (`.mtl`, `_seite_NN.jpg`).
   - Erwartet: Ohne `--ueberschreiben` bzw. Zustimmung wird nichts angefasst, und Ziel = Quelle wird immer abgelehnt.
   - Tests: Task 5 `existing_target_refused_without_overwrite`, `obj_side_file_exists_refused`, `target_is_source_refused`; Task 7 `export_asks_before_replacing_side_files`.
2. **Abbruch oder Fehler mitten im Schreiben**, auch ein mehrteiliges OBJ mit `--ueberschreiben`.
   - Erwartet: keine `.tmp` übrig, die alten Dateien unverändert bis zum gemeinsamen Umbenennen.
   - Tests: Task 1 `failed_write_leaves_no_tmp`; Task 5 `cancel_keeps_old_files`.
3. **Koordinaten weit vom Ursprung** (UTM, x = 500 000 m, y = 5 400 000 m).
   - Erwartet: LAS ≤ 0,1 mm Fehler, E57/PLY/PCD ≤ 1 µm.
   - Tests: Task 1 `las_far_coordinates`; Task 2 `ply_far_coordinates_use_double`, `pcd_far_coordinates_use_double`; Task 3 `e57_far_coordinates`; Task 6 `far_e57_opens_with_mm_precision`.
4. **Fremde Dateien mit Eigenheiten:**
   - LAS mit 8-Bit-Farben, PCD organisiert mit NaN-Zeilen, OBJ mit Vierecken, negativen Indizes und fehlender Textur, E57 nur sphärisch, glb mit Knoten-Transformation oder PNG-Textur.
   - Erwartet: Sie werden richtig gelesen oder ergeben eine klare deutsche Meldung, nie eine Panik.
   - Tests in Task 1–4 (je Format) und die Fixtures der Python-Bibliotheken in Task 5 (`foreign.rs`).
5. **Leere Wolke oder leeres Netz und Pfade mit Umlauten und Leerzeichen** (`Büro Scan/Prüfung ä.e57`).
   - Erwartet: eine gültige Datei bzw. eine klare Meldung.
   - Tests: Task 5 `empty_source_every_format`, `umlaut_paths_every_format`.

---

## Dateien

| Datei | Aufgabe |
|---|---|
| `crates/driftless-export/src/lib.rs` | Module, `pub use`, `ExportError` |
| `crates/driftless-export/src/points.rs` | `Points`, `PointSource`, `Layout`, `ScanInfo`, `PointFormat`, `read_points`, `write_points` |
| `crates/driftless-export/src/{las,ply,pcd,e57}.rs` | Schreiber und Leser je Punkteformat |
| `crates/driftless-export/src/mesh/{mod,glb,obj,ply}.rs` | `Mesh`, Schreiber/Leser glb und OBJ, PLY-Netz |
| `crates/driftless-export/src/traj.rs` | TUM (über `driftless_traj::write_tum`) und CSV |
| `crates/driftless-export/src/testutil.rs` | `synthetic_points` (Merkmal `test-util`) |
| `crates/driftless-io/src/atomic.rs` | dazu `TempPath` |
| `crates/driftless-io/src/testalloc.rs` | Zähl-Allokator aus `driftless-app/tests/counting` (Merkmal `test-util`) |
| `crates/driftless-map/src/export.rs`, `surface/write.rs` | Hüllen um `driftless-export` |
| `crates/driftless-steps/src/export.rs` | Schritt Export |
| `crates/driftless-cli/src/export.rs` | `driftless export` |
| `crates/driftless-app/src/ui/export.rs` | Dialog „Exportieren“ |
| `crates/driftless-app/src/ui/about.rs`, `src/icon.rs`, `build.rs` | „Über Driftless“, Symbol, Commit |
| `assets/icon/driftless.{png,ico,icns}` | erzeugtes Symbol (eingecheckt, Test prüft Gleichheit) |
| `packaging/build_commit.rs` | gemeinsamer Teil der `build.rs` von App und CLI |
| `packaging/macos/{Info.plist.in,build-dmg.sh,check-dmg.sh}` | macOS-Paket |
| `packaging/windows/{driftless.iss,build-setup.ps1,check-setup.ps1}` | Windows-Paket |
| `.github/workflows/{package,release}.yml` | Paket-Artefakte, Release-Entwurf |
| `tools/export/{check_export.py,check_mesh.py,make_foreign.py,README.md}` | unabhängige Gegenprüfung (VM 101) |
| `crates/driftless-export/tests/fixtures/foreign/` | von laspy/pye57/open3d/trimesh geschriebene Beispieldateien + `expected.json` |

---

### Task 1: Crate `driftless-export`, Punkte-Modell, LAS 1.4 und LAZ, Umzug der LAS-Schreiber

**Files:**
- Create: `crates/driftless-export/{Cargo.toml,src/lib.rs,src/points.rs,src/las.rs,src/testutil.rs}`, `crates/driftless-export/tests/las.rs`
- Modify:
  - `Cargo.toml` (Mitglied)
  - `crates/driftless-io/src/atomic.rs`, `src/lib.rs` (`TempPath`)
  - `crates/driftless-map/Cargo.toml`, `src/export.rs`, `src/lib.rs`: Hüllen, `LAS_SCALE` von `driftless_export::las` wieder ausgeben
  - `crates/driftless-steps/Cargo.toml`, `src/scan.rs` (`write_las` als Hülle), `src/error.rs` (`From<ExportError>`)
  - `crates/driftless-map/tests/export.rs`: Erwartungen Format/Versatz
- Test: `crates/driftless-export/tests/las.rs`, Tests in `driftless-io/src/atomic.rs`

**Interfaces:**
- Produces:
```rust
// driftless_io (pub use atomic::{atomic_write, TempPath})
pub struct TempPath { /* path, tmp = "<name>.tmp" daneben, committed */ }
impl TempPath {
    pub fn new(path: &Path) -> TempPath;
    pub fn tmp(&self) -> &Path;
    pub fn path(&self) -> &Path;
    pub fn commit(self) -> io::Result<()>;   // sync_all (Datei mit Schreibrecht öffnen, wegen Windows), dann rename
}
impl Drop for TempPath { /* nicht committet → tmp entfernen */ }

// driftless_export
#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("{path}: {source}")] Io { path: String, source: std::io::Error },
    #[error("{path}: {msg}")]    Format { path: String, msg: String },   // msg deutsch
    #[error("abgebrochen")]      Cancelled,
}
pub const BLOCK: usize = 1 << 20;
pub const MAX_FLOAT_COORD: f64 = 8192.0;
pub const PHASE_READ: &str = "Lesen";      // jeder Leser, Gesamtzahl = Punkte bzw. Seiten
pub const PHASE_WRITE: &str = "Schreiben"; // jeder Schreiber
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Layout { pub rgb: bool, pub intensity: bool, pub source: bool, pub time: bool, pub flags: bool }
#[derive(Clone, Debug, PartialEq)]
pub struct ScanInfo { pub source: u16, pub name: String, pub pose: Isometry3<f64> } // Scan-Rahmen → Datei-Rahmen
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Points {
    pub xyz: Vec<[f64; 3]>,
    pub rgb: Option<Vec<[u16; 3]>>,  // 16 Bit
    pub intensity: Option<Vec<u16>>, // 0–65535
    pub source: Option<Vec<u16>>,    // Scan-Nummer
    pub time: Option<Vec<f64>>,      // s, wie gespeichert (LAS gps_time)
    pub flags: Option<Vec<u8>>,      // LAS user_data (1 = ohne gemessene Farbe, bzw. Entfernungsgrund)
    pub scans: Vec<ScanInfo>,
}
impl Points {
    pub fn len(&self) -> usize; pub fn is_empty(&self) -> bool;
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])>;
    pub fn drop_empty_columns(&mut self); // Intensität alle 0 → None, flags alle 0 → None, source einheitlich → None
}
pub trait PointSource {
    fn len(&self) -> usize;
    fn layout(&self) -> Layout;
    fn xyz(&self, i: usize) -> [f64; 3];
    fn rgb(&self, i: usize) -> [u16; 3];
    fn intensity(&self, i: usize) -> f32;   // 0–255 (LAS: round(×257))
    fn source(&self, i: usize) -> u16;
    fn time(&self, i: usize) -> f64;
    fn flags(&self, i: usize) -> u8;
    fn scans(&self) -> &[ScanInfo];
}
impl PointSource for Points { /* intensity = v as f32 / 257 */ }
pub fn bounds_of(src: &dyn PointSource) -> Option<([f64; 3], [f64; 3])>;
pub fn needs_double(src: &dyn PointSource) -> bool;  // eine Koordinate |v| ≥ MAX_FLOAT_COORD
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointFormat { Las, Laz }               // Task 2: Ply, Pcd; Task 3: E57
impl PointFormat { pub fn from_path(path: &Path) -> Option<Self>; pub fn extension(self) -> &'static str }
pub fn read_points(path: &Path, progress: &Progress) -> Result<Points, ExportError>;   // Format nach Endung, dann drop_empty_columns
pub fn write_points(path: &Path, format: PointFormat, src: &dyn PointSource, progress: &Progress) -> Result<(), ExportError>;

// driftless_export::las
pub const LAS_SCALE: f64 = 0.0001;
pub fn las_offset(bounds: Option<([f64; 3], [f64; 3])>) -> [f64; 3]; // round((min+max)/2), leer → 0
pub fn write_las(path: &Path, src: &dyn PointSource, compress: bool, progress: &Progress) -> Result<(), ExportError>;
pub fn read_las(path: &Path, progress: &Progress) -> Result<Points, ExportError>;      // LAS und LAZ

// driftless_export::testutil (feature "test-util")
pub fn synthetic_points(n: usize, seed: u64, layout: Layout) -> Points;
```
- LAS-Schreiber:
  - LAS 1.4. Punktformat nach Spalten: Zeit + Farbe → 7, Zeit → 6, Farbe → 2, keines → 0. So bekommt eine Datei ohne Zeit keine falschen Zeiten.
  - `generating_software = "driftless"`, `date = None`.
  - Kompression über `point_format.is_compressed = compress` und `las::Writer::new(BufWriter::new(File::create(tmp)), header)`, **nicht** über `from_path`.
  - Eine Koordinate außerhalb des i32-Bereichs um den Versatz ergibt `Format("… zu weit ausgedehnt für LAS mit 0,1 mm")`.
- LAS-Leser:
  - liest blockweise mit Fortschritt (Phase „Lesen“) und Abbruch;
  - Farbe bzw. Intensität mit Maximum ≤ 255 über die ganze Datei gelten als 8 Bit und werden ×257 gerechnet (wie heute in der App);
  - ohne Zeit im Format ist `time = None`.
- Hüllen in `driftless-map`: ein privater Adapter `CloudPoints<'a> { cloud: &'a Cloud, sources: Option<&'a [u16]>, flags: Flags<'a> }` mit `Flags::Uncolored | Flags::Reasons(&[RemovalReason]) | Flags::None`. Er implementiert `PointSource` ohne Kopie: `time` = `t`, Intensität wie in `Cloud`. `write_las`, `write_las_sources` und `write_removed_las` behalten ihre Signaturen und rufen `driftless_export::las::write_las(…, false, &Progress::new())`.
- `MapError` bekommt `Export(String)` mit `From<ExportError>`, wobei `Cancelled` → `MapError::Cancelled`. `StepError` bekommt `From<ExportError>`, wobei `Cancelled` → `Cancelled` und alles andere → `Failed(e.to_string())`.

- [ ] **Step 1: Fehlschlagende Tests** (`driftless-export/tests/las.rs`, `atomic.rs`):
  - `las_round_trip_all_columns`: `synthetic_points(10_000, 1, alle Spalten)`.
    - Kopf: Version 1.4, Format 7, Maßstab 0.0001, Versatz = `las_offset`.
    - xyz < 0,05 mm, rgb/Intensität/source/time/flags gleich.
  - `point_format_follows_columns`: 7 / 6 / 2 / 0.
  - `laz_is_compressed_and_reads_back`: `.laz` hat `is_compressed`, ist kleiner als die LAS derselben 100 000 Punkte und liest gleich zurück; keine `.tmp` übrig.
  - `eight_bit_las_widened` (Farbe [255,128,0] → [65535,32896,0]) und `sixteen_bit_dark_stays` (wie der App-Test `sixteen_bit_dark_points_stay_dark`).
  - `las_far_coordinates`: x = 500 000,123 4, y = 5 400 000,567 8, Fehler ≤ 0,1 mm.
  - `las_deterministic`: zweimal schreiben, gleiche Bytes. `las_empty`: 0 Punkte, gültig, liest leer.
  - `failed_write_leaves_no_tmp`: Ziel ist ein Ordner → `Err`, keine `.tmp`.
  - `cancel_before_write`: `Progress::cancel()` → `ExportError::Cancelled`, kein Ziel, keine `.tmp`.
  - `temp_path_commit_and_drop` (in `atomic.rs`): commit benennt um; ohne commit ist die `.tmp` nach Drop weg.
- [ ] **Step 2:** `cargo test -p driftless-export -p driftless-io` schlägt fehl (Crate fehlt).
- [ ] **Step 3:** Crate anlegen (`las = { version = "0.11", features = ["laz"] }`, `nalgebra`, `thiserror`, `driftless-io`, `driftless-traj`), Modell, `las.rs`, `TempPath` bauen.
- [ ] **Step 4:** Hüllen in `driftless-map` und `driftless-steps/src/scan.rs` umstellen. `driftless-map/tests/export.rs` anpassen:
  - `las_round_trip_with_rgb`: Format 7, Versatz = gerundete Mitte
  - `las_without_rgb_is_format_1` → `las_without_rgb_is_format_6`
  - Die übrigen bleiben, ebenso die App-Tests, die `write_las*` nutzen.
- [ ] **Step 5:** Prüfen:
  - `cargo test --workspace` grün
  - `cargo clippy --workspace --all-targets -- -D warnings` sauber
  - `cargo deny check` grün (laz, byteorder `MIT OR Unlicense` passt über MIT)
- [ ] **Step 6:** Commit `feat(export): Crate driftless-export mit LAS 1.4 und LAZ, LAS-Schreiber umgezogen`.

### Task 2: PLY und PCD (Umzug des PLY-Lesers aus der App)

**Files:**
- Create: `crates/driftless-export/src/ply.rs` (Parser aus `driftless-app/src/ply.rs`), `src/pcd.rs`, `tests/ply.rs`, `tests/pcd.rs`
- Modify:
  - `crates/driftless-export/src/points.rs`: `PointFormat::{Ply, Pcd}`
  - `crates/driftless-map/src/export.rs`: `write_ply`, `write_ply_sources` als Hüllen
  - `crates/driftless-app/src/cloud.rs`: `load_ply` = `read_ply` + `point_set`; `src/ply.rs` löschen; `src/lib.rs`; `Cargo.toml` (`driftless-export`)
  - `crates/driftless-app/tests/cloud.rs`: Parser-Tests ziehen nach `export/tests/ply.rs`, die Umwandlungstests bleiben
- Test: `crates/driftless-export/tests/{ply,pcd}.rs`, `crates/driftless-map/tests/export.rs`, `crates/driftless-app/tests/cloud.rs`

**Interfaces:**
- Consumes: Task 1 `Points`, `PointSource`, `needs_double`, `TempPath`, `BLOCK`.
- Produces:
```rust
pub fn write_ply(path: &Path, src: &dyn PointSource, progress: &Progress) -> Result<(), ExportError>;
pub fn read_ply(path: &Path, progress: &Progress) -> Result<Points, ExportError>;
pub fn write_pcd(path: &Path, src: &dyn PointSource, progress: &Progress) -> Result<(), ExportError>;
pub fn read_pcd(path: &Path, progress: &Progress) -> Result<Points, ExportError>;
// driftless_app::cloud
pub fn point_set(points: Points, slam: Option<&SlamReport>) -> PointSet;   // Task 6 nutzt sie für alle Formate
```
- PLY schreiben, binär LE, Reihenfolge der Eigenschaften genau so (damit `color.ply` und `modell.ply` byte-gleich bleiben):
  - `x y z`: `float`, bei `needs_double` `double`;
  - `intensity`: float 0–255, wenn Layout `intensity`;
  - `red green blue`: uchar, `round(v/257)`;
  - `colored`: uchar `flags == 0`, wenn Layout `flags`;
  - `scan`: ushort, wenn Layout `source`.
- PLY lesen: alles, was `driftless-app/src/ply.rs` heute kann, mit denselben deutschen Meldungen. Die Ausgabe sind `Points` mit Farbe ×257, Intensität in 0–65535 (nach den Regeln von heute: 0–255 bleibt, float 0–1 ×255, größere auf das hellste = 255 skaliert, dann ×257) und `colored` = 0 → `flags` = 1.
- PCD schreiben, alle Kopfzeilen fest:
  - `# .PCD v0.7 - Point Cloud Data file format`, `VERSION 0.7`.
  - `FIELDS x y z [rgb] [intensity] [scan]`, `SIZE 4|8 4 4 2`, `TYPE F F F U`, `COUNT 1 …`.
  - `WIDTH n`, `HEIGHT 1`, `VIEWPOINT 0 0 0 1 0 0 0`, `POINTS n`, `DATA binary`.
  - `rgb` ist ein float mit den Bits `0x00RRGGBB` (PCL-Gewohnheit).
- PCD lesen:
  - `DATA ascii` und `binary`;
  - `binary_compressed` → `Format("PCD „binary_compressed“ wird nicht unterstützt – in CloudCompare oder PCL als „binary“ speichern")`;
  - Felder `x y z` (F 4/8), `rgb`/`rgba` (F 4 oder U 4, gepackt), `r g b` (U 1), `intensity` (jeder Typ, Regel wie PLY), `scan`; weitere Felder überspringen, auch `COUNT` > 1;
  - Zeilen mit nicht endlichem x/y/z (organisierte Wolken) überspringen;
  - kaputter Kopf → `Format("PCD nicht lesbar: …")`, keine Panik.
- `point_set`:
  - Ursprung = je Achse `round((min+max)/2)` der **tatsächlichen** Grenzen, für alle Formate gleich;
  - rgb `>> 8`, Intensität `>> 8`, ohne Intensität `MID_GRAY`;
  - `t`/`quality` aus `time` wie heute in `load_las`, `source` übernommen;
  - Spalten nacheinander umwandeln und jede gleich freigeben.

- [ ] **Step 1: Vor dem Umzug festhalten:** `color_ply_unchanged_by_move` in `driftless-map/tests/export.rs` mit dem `content_hash` der heutigen `write_ply`- und `write_ply_sources`-Dateien der Testwolke `cloud(true)` (Werte jetzt ausrechnen und eintragen). Der Test ist grün vor der Änderung.
- [ ] **Step 2: Fehlschlagende Tests:**
  - PLY:
    - die heutigen App-Tests `ply_round_trip_from_write_ply`, `ply_ascii_with_float_color`, `ply_float_color_0_255`, `ply_big_endian_double`, `ply_without_color`, `ply_skips_faces`, `ply_broken_header_is_error` und `ply_load_reports_phase_and_cancels`, nach `export/tests/ply.rs` übertragen, Erwartungen auf `Points` (Farbe ×257);
    - `ply_far_coordinates_use_double`: Kopf `property double x`, Fehler ≤ 1 µm.
  - PCD:
    - `pcd_binary_round_trip` (alle Spalten, rgb exakt bei 8-Bit-Herkunft) und `pcd_header_exact` (Kopf Zeile für Zeile wie oben);
    - `pcd_ascii_reads`, `pcd_separate_rgb_fields`;
    - `pcd_binary_compressed_refused` (genaue Meldung), `pcd_organized_nan_skipped` (WIDTH 4 HEIGHT 2, zwei NaN-Zeilen → 6 Punkte);
    - `pcd_far_coordinates_use_double`, `pcd_broken_header_is_error`.
  - App (`tests/cloud.rs`): `ply_origin_is_rounded_centre` ersetzt die festen Ursprünge der übertragenen Tests.
- [ ] **Step 3:** `cargo test -p driftless-export` schlägt fehl. **Step 4:** `ply.rs` (Parser umziehen, Ausgabe auf `Points`), `pcd.rs` und die Hüllen bauen; `driftless-app/src/ply.rs` löschen.
- [ ] **Step 5:**
  - `cargo test --workspace` grün, auch `color_ply_unchanged_by_move` und `ply_header_and_size`;
  - clippy sauber, auch `--release -p driftless-app --bins`.
- [ ] **Step 6:** Commit `feat(export): PLY und PCD schreiben und lesen, PLY-Leser aus der App umgezogen`.

### Task 3: E57

**Files:**
- Create: `crates/driftless-export/src/e57.rs`, `tests/e57.rs`
- Modify: `crates/driftless-export/Cargo.toml` (`e57 = "0.11"`), `src/points.rs` (`PointFormat::E57`)

**Interfaces:**
- Consumes: Task 1 `Points`, `PointSource::scans`, `ScanInfo`, `TempPath`.
- Produces:
```rust
pub fn write_e57(path: &Path, src: &dyn PointSource, progress: &Progress) -> Result<(), ExportError>;
pub fn read_e57(path: &Path, progress: &Progress) -> Result<Points, ExportError>;
```
- Schreiben:
  - Ein `data3D` je Scan:
    - Gibt es `scans()`, dann je `ScanInfo` mit `name`, Lage = `pose` (Quaternion + Verschiebung).
    - Sonst je vorkommender `source` (Name „Scan <n+1>“).
    - Ohne `source` ist es ein Scan mit dem Dateinamen ohne Endung und der Einheitslage.
  - Die Punkte eines Scans stehen in dessen eigenem Rahmen: `pose⁻¹ · p`, in der Reihenfolge der Quelle (stabil nach Scan gruppiert, über eine Indexliste `u32`).
  - Prototyp:
    - `cartesianX/Y/Z` als Float double;
    - `colorRed/Green/Blue` als Integer 0–255, wenn alle Werte Vielfache von 257 sind, sonst 0–65535, mit `ColorLimits` passend;
    - `intensity` als Integer 0–65535 mit `IntensityLimits`.
  - `set_creation(None)`, GUIDs nach Global Constraints. Kein Feld `scan`: die Scans sind die Einträge.
- Lesen:
  - `pointcloud_simple` mit `apply_pose(true)`, `spherical_to_cartesian(true)`, `intensity_to_color(false)`;
  - Farbe und Intensität (normiert 0–1) → `round(v × 65535)`;
  - alle Scans hintereinander, `source` = Index des Scans (bei einem Scan `None`), `scans` mit Name und Lage;
  - `Invalid`/`Direction`-Punkte überspringen;
  - ein Scan mit Datensätzen, aber ohne einen gültigen Punkt → `Format("E57: Scan „<name>“ hat keine kartesischen Punkte")`;
  - Bilder (`images2D`) werden nicht gelesen; das sagt eine Warnung über `progress.warn`.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `e57_round_trip_two_scans`: Zwei Scans mit `ScanInfo` (Lage 2: Gier 30°, t = (3, −1, 0,5)).
    - Zurück: Namen und Lagen gleich (1e-12), Punkte im Datei-Rahmen ≤ 1e-9 m.
    - Farbe 8 Bit exakt, Intensität exakt, `source` = {0, 1}.
  - `e57_sixteen_bit_colour`: Farbe 1000 → Limits 0–65535, liest 1000 zurück.
  - `e57_spherical_only_is_converted`: mit dem `e57`-Crate im Test geschrieben (nur sphärische Felder) → kartesisch ≤ 1e-9.
  - `e57_without_coordinates_is_error`: alle Punkte mit `cartesianInvalidState = 2` → genaue Meldung.
  - `e57_far_coordinates` ≤ 1 µm. `e57_deterministic`: gleiche Bytes. `e57_empty`: 0 Punkte schreibt und liest.
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen. **Step 4:** `cargo test -p driftless-export` grün, clippy sauber, `cargo deny check` grün.
- [ ] **Step 5:** Commit `feat(export): E57 mit einem Scan je Mitglied, Lage und Namen`.

### Task 4: Netze – glb und OBJ umziehen, Leser, PLY-Netz

**Files:**
- Create: `crates/driftless-export/src/mesh/{mod.rs,glb.rs,obj.rs,ply.rs}`, `tests/mesh.rs`
- Modify:
  - `crates/driftless-export/Cargo.toml`: `gltf-json`, `gltf`, `image` mit `jpeg`, `png`
  - `crates/driftless-map/src/surface/write.rs`: `write_glb`/`write_obj` als Hüllen; `encode_pages`, `PageJpegs` und die Tests bleiben dort
- Test: `crates/driftless-export/tests/mesh.rs`, die bestehenden Tests in `surface/write.rs`

**Interfaces:**
- Produces:
```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ImageKind { Jpeg, Png }
#[derive(Clone, Debug, PartialEq)] pub struct PageImage { pub kind: ImageKind, pub bytes: Vec<u8> } // kodiert
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshPage {
    pub positions: Vec<[f64; 3]>,  // Z oben, Meter
    pub uvs: Vec<[f32; 2]>,        // v nach unten (Zeile 0 bei v = 0), wie TexturePage
    pub indices: Vec<[u32; 3]>,
    pub image: Option<PageImage>,
    pub color: [u8; 3],            // ohne Bild: Grundfarbe
}
#[derive(Clone, Debug, Default, PartialEq)] pub struct Mesh { pub pages: Vec<MeshPage> }
impl Mesh { pub fn triangles(&self) -> usize }
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum MeshFormat { Glb, Obj, PlyMesh }
pub const GENERATOR: &str = "Driftless";
pub fn write_glb(path: &Path, mesh: &Mesh) -> Result<(), ExportError>;
pub fn write_obj(path: &Path /* <stem>.obj */, mesh: &Mesh) -> Result<(), ExportError>;
pub fn obj_files(path: &Path, mesh: &Mesh) -> Vec<PathBuf>; // .obj, .mtl, <stem>_seite_NN.<jpg|png>
pub fn write_ply_mesh(path: &Path, mesh: &Mesh, progress: &Progress) -> Result<(), ExportError>;
pub fn read_mesh(path: &Path, progress: &Progress) -> Result<Mesh, ExportError>; // .glb, .obj
// driftless_map::surface
impl TexturedMesh { pub fn to_mesh(&self, jpegs: &PageJpegs) -> Result<Mesh, MapError> }
```
- glb und OBJ schreiben: Der Code von `surface/write.rs` zieht um.
  - Seiten mit leerem `indices` lässt glb aus, wie heute. Eine Seite ohne Bild bekommt `baseColorFactor = color/255` und keine Textur. Eine PNG-Seite wird mit `image/png` eingebettet.
  - OBJ:
    - Seitenbild `<stem>_seite_NN.jpg|png`, Material `seite_NN`, ohne Bild `Kd` = Farbe;
    - `v` als f32-Kurzform, wenn `(v as f32) as f64 == v`, sonst f64-Kurzform. So bleiben heutige Dateien byte-gleich;
    - alle Dateien erst `.tmp`, umbenannt nach dem letzten Schreiben, die `.obj` zuletzt.
- glb lesen:
  - Zuerst `Gltf::from_slice_without_validation` und `extensions_required`:
    - `KHR_draco_mesh_compression` → `Format("glTF mit Draco-Kompression wird nicht unterstützt")`;
    - `EXT_meshopt_compression` → ebenso „mit meshopt-Kompression“;
    - mehr als eine Szene → `Format("glTF mit mehreren Szenen wird nicht unterstützt")`.
  - Dann validieren.
  - Die Knoten der Szene werden mit ihren Welt-Matrizen durchlaufen; ohne Szene werden alle Netze ohne Transformation genommen.
  - Je Primitiv eine Seite: nur Dreiecke, sonst eine Meldung wie heute in `load_glb`. Bild JPEG/PNG eingebettet; externer Puffer oder externe Bild-URI → Meldung wie heute.
  - Y oben → Z oben `(x, −z, y)`.
- OBJ lesen:
  - `v`, `vt`, `f` mit Vielecken (Fächer), negativen Indizes und `v//vn`;
  - `usemtl`, `mtllib` mit `Kd` und `map_Kd` (Pfad relativ zur `.mtl`, JPEG/PNG);
  - fehlende `.mtl` oder fehlendes Bild → Seite grau (128) und `progress.warn("Textur fehlt: <datei>")`;
  - gelesen wird als Z oben (so schreibt Driftless und Blender mit „Z up“).
- PLY-Netz:
  - binär LE: `element vertex` mit `x y z` (float, bei Bedarf double), `red green blue` (uchar); `element face` mit `property list uchar int vertex_indices`.
  - Ecken mit bitgleicher Position über alle Seiten verschweißt.
  - Farbe je Ecke = Mittel der bilinear abgetasteten Texel an allen UV-Stellen dieser Ecke. Ohne Bild gilt die Seitenfarbe.
  - Bilder einmal je Seite dekodiert, Fortschritt je Seite.

- [ ] **Step 1: Vor dem Umzug festhalten:** In `surface/write.rs` kommt `files_unchanged_by_move` dazu: `content_hash` von `x.glb`, `x.obj`, `x.mtl` aus `floor_and_ceiling()`, jetzt ausgerechnet und eingetragen, grün vor der Änderung.
- [ ] **Step 2: Fehlschlagende Tests** (`tests/mesh.rs`, kleine Netze im Test gebaut):
  - glb:
    - `glb_read_write_identical`: schreiben → lesen → schreiben ergibt gleiche Bytes;
    - `glb_node_transform_applied`: fremde glb mit Knoten-Verschiebung (0, 2, 0) im glTF-Rahmen → z + 2 in Driftless;
    - `glb_png_texture` (PNG-Seite bleibt PNG);
    - `glb_multiple_scenes_refused`, `glb_draco_refused`: JSON im Test zusammengesetzt, genaue Meldungen.
  - OBJ:
    - `obj_round_trip`: Positionen, UVs, Seitenbilder nach Namen; keine `.tmp`;
    - `obj_foreign_quads_negative_indices_missing_texture`: 1 Viereck → 2 Dreiecke, negative Indizes, fehlendes Bild → grau + Warnung.
  - PLY-Netz:
    - `ply_mesh_vertex_colors_from_texture`: Seite links rot, rechts blau, Ecken links ±2 rot;
    - `ply_mesh_untextured_uses_page_color`;
    - `ply_mesh_header_counts` (Ecken nach dem Verschweißen, Flächen = Dreiecke).
- [ ] **Step 3:** schlägt fehl. **Step 4:** bauen; `write_glb`/`write_obj` in `driftless-map` rufen `to_mesh` und den neuen Schreiber.
- [ ] **Step 5:**
  - `cargo test --workspace` grün, auch `files_unchanged_by_move`, `byte_identical_twice`, `glb_roundtrip`, `obj_files_complete` und die Oberflächen-Tests der Schritte und der App;
  - clippy sauber.
- [ ] **Step 6:** Commit `feat(export): Netze – glb/OBJ umgezogen, glb/OBJ lesen, PLY-Netz mit Farbe je Ecke`.

### Task 5: Schritt Export, Laufbahn als CSV, `driftless export`, unabhängige Gegenprüfung

**Files:**
- Create:
  - `crates/driftless-export/src/traj.rs`
  - `crates/driftless-steps/src/export.rs`, `crates/driftless-cli/src/export.rs`
  - `crates/driftless-io/src/testalloc.rs`; `crates/driftless-app/tests/counting/` entfällt, `lod_memory*.rs` nutzen `driftless_io::testalloc`
  - Tests: `crates/driftless-steps/tests/{export.rs,export_memory.rs}`, `crates/driftless-cli/tests/export.rs`
  - `tools/export/{check_export.py,check_mesh.py,make_foreign.py,README.md}`
  - `crates/driftless-export/tests/{foreign.rs,fixtures/foreign/*}`
- Modify: `crates/driftless-steps/src/lib.rs`, `Cargo.toml`; `crates/driftless-cli/src/main.rs` (`Command::Export`); `crates/driftless-io/src/lib.rs`

**Interfaces:**
- Consumes: Task 1–4 `read_points`, `write_points`, `PointFormat`, `ScanInfo`, `read_mesh`, `write_glb`, `write_obj`, `obj_files`, `write_ply_mesh`; `merge::{read_model, member_transform, MODEL_LAS, MODEL_REPORT}`; `driftless_traj::read_tum`.
- Produces:
```rust
// driftless_export::traj
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum TrajFormat { Tum, Csv }
pub fn write_traj(path: &Path, poses: &[Pose], format: TrajFormat) -> Result<(), ExportError>;
// CSV: Kopfzeile "zeit;x;y;z;qx;qy;qz;qw", je Pose "{:.9};…" wie TUM, Punkt als Dezimaltrenner, "\n"

// driftless_steps::export
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ExportKind { Points, Mesh, Trajectory }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExportFormat { Las, Laz, E57, Ply, Pcd, Glb, Obj, PlyMesh, Tum, Csv }
impl ExportFormat {
    pub const ALL: [ExportFormat; 10];
    pub fn kind(self) -> ExportKind;
    pub fn name(self) -> &'static str;       // --format: las laz e57 ply pcd glb obj ply-netz tum csv
    pub fn label(self) -> &'static str;      // App: "LAS 1.4", "LAZ", "E57", "PLY (binär)", "PCD (binär)",
                                             // "glb (Y oben)", "OBJ mit Seitenbildern", "PLY-Netz (Farbe je Ecke)", "TUM", "CSV"
    pub fn extension(self) -> &'static str;  // ply-netz → "ply"
    pub fn parse(name: &str) -> Option<Self>;
    pub fn for_path(path: &Path, kind: ExportKind) -> Option<Self>; // ".ply" → Ply bzw. PlyMesh nach kind
}
#[derive(Clone, Debug, PartialEq)]
pub enum ExportSource { Map(PathBuf), Model(PathBuf), Surface(PathBuf), Run(PathBuf), PointFile(PathBuf), MeshFile(PathBuf) }
impl ExportSource {
    pub fn detect(path: &Path) -> Result<Self, StepError>;
    pub fn kind(&self) -> ExportKind;
    pub fn default_name(&self) -> String;   // Vorschlag für den Dateidialog, ohne Endung
}
pub const POINT_EXTENSIONS: [&str; 5] = ["las", "laz", "e57", "ply", "pcd"];
pub const MESH_EXTENSIONS: [&str; 2] = ["glb", "obj"];
pub fn file_kind(path: &Path) -> Option<ExportKind>; // nach Endung, ohne Groß/klein; .ply = Points
#[derive(Clone, Debug, PartialEq)]
pub struct ExportRequest { pub source: ExportSource, pub out: PathBuf, pub format: ExportFormat, pub overwrite: bool }
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct ExportReport { pub files: Vec<PathBuf>, pub count: u64 /* Punkte, Dreiecke oder Lagen */, pub bytes: u64, pub seconds: f64 }
pub fn export_targets(req: &ExportRequest) -> Vec<PathBuf>;   // OBJ: .obj, .mtl und jede vorhandene <stem>_seite_*
pub fn existing_targets(req: &ExportRequest) -> Vec<PathBuf>;
pub fn run_export(req: &ExportRequest, progress: &Progress) -> Result<ExportReport, StepError>;
```
- `ExportSource::detect`:
  - Ordner mit `modell.json` → `Model`, mit `map.json` → `Map`, mit `oberflaeche.glb` → `Surface`, mit `traj.tum` → `Run`.
  - `modell.las` neben `modell.json` → `Model(Ordner)`.
  - Datei mit Endung aus `POINT_EXTENSIONS` → `PointFile`, aus `MESH_EXTENSIONS` → `MeshFile`.
  - Sonst `Failed("<pfad>: weder Karte, Modell, Oberfläche, Lauf noch Punktwolken- oder Netzdatei")`.
- `run_export`, in dieser Reihenfolge, Meldungen deutsch:
  1. `format.kind() != source.kind()` → `Failed("<label> passt nicht zu <Quelle>: möglich sind …")`.
  2. Das Ziel ist eine Datei der Quelle (kanonisch verglichen) → `Failed("Ziel ist die Quelle selbst: <pfad>")`.
  3. `!overwrite` und `existing_targets` nicht leer → `Failed("gibt es schon: <pfad>")`.
  4. Lesen:
     - `Map`: `color.las`, sonst `clean.las`, sonst `Failed("Karte ohne color.las oder clean.las")`.
     - `Model`: `modell.las` und `read_model`; `scans` = je Mitglied mit `included` eine `ScanInfo { source: Index, name: Ordnername des Scans, pose: member_transform }`.
     - `Surface`: `oberflaeche.glb`. `Run`: `traj.tum`.
  5. Abbruch prüfen, schreiben. Mit `overwrite` werden nach dem Umbenennen alte `<stem>_seite_*`, die dieses OBJ nicht geschrieben hat, entfernt.
  6. `ExportReport`, `bytes` = Summe der Dateigrößen.
- CLI:
  - Aufruf: `driftless export <QUELLE> -o <DATEI> [--format <name>] [--ueberschreiben]`.
  - Das Format kommt aus `--format`, sonst aus `for_path(-o, kind)`, sonst `Fehler: unbekannte Endung „.x“ – mit --format angeben: las, laz, e57, ply, pcd, glb, obj, ply-netz, tum, csv`.
  - Gibt es schon Ziele ohne `--ueberschreiben`, kommt vor dem Lauf `Fehler: <pfad> gibt es schon – mit --ueberschreiben ersetzen`.
  - Ausgabe: `<count> Punkte|Dreiecke|Lagen → <datei> (<MB> MB, <s> s)`.
  - Kurzhilfe im Stil von `main.rs`: `/// Exportiert Karte, Modell, Oberfläche, Laufbahn oder eine Punktwolken-Datei (LAS, LAZ, E57, PLY, PCD, glb, OBJ, TUM, CSV)`.

- [ ] **Step 1: Fehlschlagende Tests** (`steps/tests/export.rs`, Szenen aus `synthetic_points`, `write_las_sources` + kleinem `modell.json`, `Mesh` aus Task 4):
  - Quellen und Formate:
    - `map_exports_color_else_clean`;
    - `model_e57_has_named_scans_with_poses`: Namen = Ordnernamen, Lage = `member_transform`;
    - `detect_sources`, `format_from_extension_and_override`, `ply_extension_by_kind`, `kind_mismatch_is_refused`.
  - Überschreiben:
    - `existing_target_refused_without_overwrite`, `overwrite_replaces`, `target_is_source_refused`;
    - `obj_side_file_exists_refused`: nur `x_seite_00.jpg` liegt da;
    - `obj_overwrite_removes_stale_pages`.
  - `cancel_keeps_old_files`: altes OBJ da, `overwrite`, Abbruch nach dem Lesen über einen Beobachter auf die Phase „Schreiben“ → alte Dateien byte-gleich, keine `.tmp`.
  - Laufbahn: `run_exports_tum_and_csv`. CSV-Kopf genau `zeit;x;y;z;qx;qy;qz;qw`, Zeile 1 genau wie die TUM-Werte mit `;`.
  - `empty_source_every_format`: leere Wolke in alle fünf Punkteformate, leeres Netz in drei; jeweils gültig und zurücklesbar.
  - `umlaut_paths_every_format`: Ordner `Büro Scan`, Datei `Prüfung ä.<ext>`, alle Formate hin und zurück.
  - `export_memory.rs` (ein Test, eigener Binary, `driftless_io::testalloc`):
    - 2 Mio. Punkte mit allen Spalten als LAS in jedes Punkteformat;
    - Spitze ≤ 2 × `Points`-Bytes + 64 MB, wobei `Points`-Bytes = Summe der Spaltenlängen × Elementgröße.
  - CLI (`cli/tests/export.rs`): `export_writes_laz`, `export_needs_ueberschreiben` (Exit ≠ 0, Hinweis im stderr), `export_unknown_extension_lists_formats`, `export_format_overrides_extension`.
- [ ] **Step 2:** schlägt fehl. **Step 3:** `traj.rs`, `export.rs` (Schritt), CLI und `testalloc` bauen.
- [ ] **Step 4:** `cargo test --workspace` grün, clippy sauber. Commit `feat: Schritt Export und driftless export`.
- [ ] **Step 5: Gegenprüfung auf VM 101** (leicht, aber dort, wo die Python-Bibliotheken liegen):
  - Python einrichten: `pip install --user laspy lazrs pye57 open3d trimesh scipy numpy`. Gibt es für `pye57` kein Rad für die Python-Version der VM, das melden und E57 dort nur mit dem e57-Crate gegenprüfen.
  - `tools/export/check_export.py <bezug.las> <export>`:
    - Bezug mit laspy, Export mit laspy/lazrs, pye57, open3d bzw. einem eigenen numpy-Leser für PCD/PLY.
    - Geprüft werden: gleiche Zahl; Grenzen ± Genauigkeit (LAS 0,1 mm, sonst 1 µm bzw. float-Genauigkeit); 1000 Stichproben gleicher Farbe, Intensität und Scan-Nummer.
    - E57: Reihenfolge = Bezug stabil nach `point_source_id` sortiert, Punkte nach Lage.
    - Exit ≠ 0 bei Abweichung, eine Tabellenzeile je Datei.
  - `tools/export/check_mesh.py <oberflaeche.glb> <export>`: mit trimesh gleiche Ecken- und Flächenzahl, Grenzen nach Achsentausch, beim PLY-Netz Mittelfarbe ±3.
  - `tools/export/make_foreign.py <ordner>`: Szene mit festem Startwert (600 Punkte auf drei Ebenen, Farben je Ebene). Daraus:
    - laspy: LAS 1.2 mit 8-Bit-Farben, LAZ 1.4 Format 7;
    - pye57: zwei Scans mit Lage, ein Scan nur sphärisch, falls pye57 das schreibt;
    - open3d: PCD ascii/binary/binary_compressed, PLY;
    - trimesh: glb mit PNG-Textur und Knoten-Transformation, OBJ mit `.mtl` und PNG.

    Dazu `expected.json` (Zahl, Grenzen, Farbe der ersten 5 Punkte je Datei). Zusammen höchstens 300 KB.
  - Ablauf: `driftless-vm sync`, Release-CLI bauen, synthetische Exporte jedes Formats mit `check_export.py` und `check_mesh.py` prüfen, alle grün.
  - Fixtures holen (`driftless-vm fetch`) nach `crates/driftless-export/tests/fixtures/foreign/`.
- [ ] **Step 6:** `crates/driftless-export/tests/foreign.rs`: je Fixture `read_points` bzw. `read_mesh` gegen `expected.json` (`serde_json` als dev-Abhängigkeit). `open3d_binary_compressed.pcd` ergibt die Meldung aus Task 2. Grün auf dem Mac; Abweichungen in den Lesern beheben, nicht in den Erwartungen.
- [ ] **Step 7:** `tools/export/README.md` (Aufruf, Bibliotheken, was geprüft wird). Commit `test(export): unabhängige Gegenprüfung und fremde Beispieldateien`.

### Task 6: App – Dateien öffnen (LAZ, E57, PCD, glb, OBJ)

**Files:**
- Modify:
  - `crates/driftless-app/src/cloud.rs`: `load_points` ersetzt `load_las` und `load_ply`; `is_ply` entfällt; `load_cloud` und `load_pair_cloud`
  - `src/surface.rs`: `load_mesh` ersetzt `load_glb`, Seitenbilder JPEG/PNG
  - `Cargo.toml`: `image` mit `png`
  - `ui/mod.rs`: `Action::PickOpenFile` ersetzt `PickOpenCloud` und `PickOpenSurface`
  - `ui/rail.rs`, `ui/app.rs` (`dropped`, `PickOpenFile`), `ui/text.rs`
  - `tests/lod_memory_real.rs`
- Test: `crates/driftless-app/tests/{cloud.rs,mesh.rs,ui_layout.rs,ui_app.rs}`

**Interfaces:**
- Consumes: Task 1–4 `read_points`, `read_mesh`, `Mesh`; Task 2 `point_set`; Task 5 `file_kind`, `POINT_EXTENSIONS`, `MESH_EXTENSIONS`.
- Produces:
```rust
pub fn load_points(path: &Path, slam: Option<&SlamReport>, progress: &Progress) -> Result<PointSet, StepError>; // read_points + point_set
pub fn load_mesh(path: &Path, progress: &Progress) -> Result<LoadedSurface, StepError>;  // .glb/.obj oder Oberflächenordner
```
- `load_mesh`: `read_mesh` → `LoadedSurface`.
  - Ursprung wie heute (gerundete Mitte), Seiten parallel dekodiert (JPEG/PNG → RGBA) mit `mip_levels`.
  - Eine Seite ohne Bild wird 1 × 1 in `color`. `surface_glb` bleibt für Ordner.
- Texte:
  - `OPEN_FILE = "Datei öffnen …"`;
  - `DIALOG_OPEN_FILE = "Datei öffnen (LAS, LAZ, E57, PLY, PCD, GLB, OBJ)"`;
  - Filter `FILTER_ALL = "Punktwolken und Netze"`, `FILTER_CLOUD`, `FILTER_MESH = "Netz"`;
  - `drop_unsupported(name)` → `"Dateiformat nicht unterstützt: {name} (LAS, LAZ, E57, PLY, PCD, GLB oder OBJ)"`.
- Öffnen und Hineinziehen nach `file_kind`: `Points` → `Action::LoadCloud { path, run: None }`, `Mesh` → `Action::LoadSurface`. Lesefehler erscheinen wie heute als Meldung in der Statusleiste, mit dem Text des Lesers.

- [ ] **Step 1: Fehlschlagende Tests:**
  - Wolken:
    - `loads_laz`, `loads_e57_two_scans_as_sources` (zwei Scans → `source` {0,1}, Farbmodus „je Scan“ angeboten), `loads_pcd_ascii`;
    - `far_e57_opens_with_mm_precision`: Ursprung ganze Meter, Abweichung ≤ 1 mm;
    - `pcd_compressed_shows_message`: die Meldung aus Task 2 als `StepError::Failed`.
  - Netze (`mesh.rs`): `loads_obj_with_png_page`, `loads_glb_with_node_transform`, `draco_glb_shows_message`.
  - UI:
    - `ui_layout.rs`: ☰ zeigt `text::OPEN_FILE` und kein „Oberfläche öffnen …“ mehr;
    - `ui_app.rs`: `dropped_e57_loads_cloud`, `dropped_obj_loads_surface`, `dropped_unknown_shows_hint` mit neuer Liste.
  - Die bestehenden LAS-Tests in `tests/cloud.rs` (8-Bit, 16-Bit dunkel, Qualität aus Teilkarten, leere Datei, Abbruch, Phase) laufen über `load_points` und bleiben grün. Feste Ursprünge werden nach der Regel „gerundete Mitte der tatsächlichen Grenzen“ neu ausgerechnet und im Commit genannt.
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen.
- [ ] **Step 4:**
  - `cargo test -p driftless-app` grün; Schnappschüsse, wo das Menü sich ändert, bewusst neu (`UPDATE_SNAPSHOTS=1`) und im Commit nennen;
  - clippy sauber, auch `cargo clippy --release -p driftless-app --bins -- -D warnings`.
- [ ] **Step 5:** Commit `feat(app): LAZ, E57, PCD, glb und OBJ öffnen, „Datei öffnen …“`.

### Task 7: App – Exportieren

**Files:**
- Create: `crates/driftless-app/src/ui/export.rs`
- Modify:
  - `ui/mod.rs`: `AppState.export: Option<ExportDialog>`, Aktionen
  - `ui/steps/tree.rs`: Kontextmenü an Lauf, Karte, Modell, Oberfläche
  - `ui/rail.rs`: ☰ „Exportieren …“
  - `ui/app.rs`: Dateidialog, Nachfrage, Ergebnis
  - `ui/actions.rs`: `RunExport` ohne Projekt
  - `jobs.rs`: `JobOutput::Exported`
  - `ui/text.rs`
- Test: `crates/driftless-app/tests/{ui_export.rs,actions.rs}`

**Interfaces:**
- Consumes: Task 5 `ExportSource`, `ExportFormat`, `ExportKind`, `ExportRequest`, `ExportReport`, `existing_targets`, `run_export`.
- Produces:
```rust
pub struct ExportDialog {
    pub choices: Vec<(ExportSource, String)>, // Was: Quelle und Name („Karte 001 – color.las“, „Oberfläche 001“ …)
    pub chosen: usize,
    pub format: ExportFormat,                 // Vorgabe: LAZ, glb bzw. CSV
}
// Action
OpenExport(ExportSource),        // aus dem Baum: eine Wahl
OpenExportShown,                 // ☰: was die Ansicht zeigt (Wolke, Oberfläche oder beide zur Wahl)
PickExportTarget,                // „Ziel wählen …“: rfd-Speicherdialog, Vorschlag default_name + Endung
ConfirmExport(ExportRequest),    // nach der Nachfrage
RunExport(ExportRequest),
JobOutput::Exported(Box<ExportReport>)
```
- Ablauf:
  1. Das Fenster „Exportieren“ zeigt Was (ComboBox bei mehreren Wahlen), Format (ComboBox der `ExportFormat::ALL` passender Art, `label()`), „Ziel wählen …“ und „Abbrechen“.
  2. Nach dem Speicherdialog (`rfd::FileDialog::save_file`, Filter des Formats) werden die Ziele geprüft:
     - Die gewählte Datei hat der Speicherdialog schon bestätigt (macOS und Windows fragen selbst).
     - Gibt es weitere `existing_targets` (OBJ-Nebendateien), fragt das Fenster „Ersetzen?“ mit der Liste. Erst „Ersetzen“ gibt `ConfirmExport`, dann `overwrite = true`.
  3. `RunExport` läuft im Schritt-Runner wie die übrigen Schritte, auch ohne Projekt (Zweig vor der Projektprüfung in `start_step`), mit Fortschritt und Abbrechen in der Statusleiste.
  4. Fertig: Info `"Exportiert: <datei> (<count> …, <MB> MB)"`. Fehler wie bei den übrigen Schritten.
- Baum: Kontextmenü (Rechtsklick) „Exportieren …“ an Lauf (`Run`), Karte (`Map`), Modell (`Model`) und Oberfläche (`Surface`).
- ☰ „Exportieren …“ ist gesperrt, solange nichts angezeigt wird (Hinweis `EXPORT_NEEDS_VIEW = "Zuerst eine Punktwolke oder Oberfläche anzeigen"`).
  - Die Quelle der gezeigten Wolke ist `ExportSource::detect(path)`, damit `modell.las` die Scan-Namen mitnimmt.
- Texte: `EXPORT = "Exportieren …"`, `EXPORT_TITLE = "Exportieren"`, `EXPORT_WHAT = "Was"`, `EXPORT_FORMAT = "Format"`, `EXPORT_PICK = "Ziel wählen …"`, `EXPORT_REPLACE = "Ersetzen"`, `fn export_replace_question(files: &[PathBuf]) -> String` (`"Diese Dateien gibt es schon:\n…\nErsetzen?"`), `fn exported(report: &ExportReport, kind: ExportKind) -> String`.

- [ ] **Step 1: Fehlschlagende Tests** (`ui_export.rs`, kittest wie `ui_daten.rs`; Projekt mit einer Karte und einer Oberfläche aus den bestehenden Helfern):
  - `tree_context_menu_offers_export`: `click_secondary` auf „Karte 001“ → „Exportieren …“ → `Action::OpenExport(Map(..))`. Ebenso für Lauf, Modell und Oberfläche.
  - `dialog_lists_formats_of_kind`: Karte → 5 Punkteformate, Oberfläche → 3, Lauf → 2.
  - `menu_export_needs_view`: gesperrt ohne Anzeige; mit Wolke und Oberfläche zwei Wahlen.
  - `export_asks_before_replacing_side_files`: `x.mtl` liegt da → Nachfrage mit Liste; „Abbrechen“ startet nichts, „Ersetzen“ ergibt `RunExport` mit `overwrite`.
  - `actions.rs`: `run_export_without_project_starts` und `run_export_while_busy_refused`.
  - Job: `export_job_reports_file`: echter kleiner Export über `JobRunner`, `JobOutput::Exported` mit Datei.
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen. **Step 4:** `cargo test -p driftless-app` grün, Schnappschüsse bewusst aktualisieren, clippy sauber (auch Release-Bins).
- [ ] **Step 5:** Commit `feat(app): Exportieren aus dem Baum und aus ☰`.

### Task 8: Symbol, Version, „Über Driftless“, macOS-App und `.dmg` (lokal)

**Files:**
- Create:
  - `crates/driftless-app/src/icon.rs`, `crates/driftless-app/src/ui/about.rs`, `crates/driftless-app/build.rs`, `crates/driftless-cli/build.rs`, `packaging/build_commit.rs`
  - `assets/icon/driftless.{png,ico,icns}`
  - `packaging/macos/{Info.plist.in,build-dmg.sh,check-dmg.sh}`
  - Test: `crates/driftless-app/tests/icon.rs`
- Modify:
  - `crates/driftless-app/src/main.rs`: `with_icon`
  - `ui/app.rs::parse_args`: `--version` → `Args.version`; `main` gibt `Driftless <version> (<commit>)` aus und endet mit 0
  - `ui/rail.rs`: ☰ „Über Driftless …“
  - `ui/mod.rs`, `ui/text.rs`, `Cargo.toml` (`png`)
  - `crates/driftless-cli/src/main.rs`: `long_version`
- Test: `tests/icon.rs`, `tests/ui_layout.rs` (oder `ui_about.rs`), `tests/ui_app.rs` (`parse_args`), `cli/tests/inspect.rs` oder neu `cli/tests/version.rs`

**Interfaces:**
- Produces:
```rust
// driftless_app::icon
pub fn icon_rgba(size: u32) -> Vec<u8>;   // RGBA, size², deterministisch
pub fn icon_png(size: u32) -> Vec<u8>;
pub fn icon_ico() -> Vec<u8>;              // Größen 16 24 32 48 64 128 256, PNG-Einträge
pub fn icon_icns() -> Vec<u8>;             // icp4 16, icp5 32, icp6 64, ic07 128, ic08 256, ic09 512, ic10 1024, PNG-Einträge
// driftless_app::ui::about
pub const COMMIT: &str = env!("DRIFTLESS_COMMIT");   // aus build.rs; "unbekannt" ohne git
pub fn show(ctx: &egui::Context, open: &mut bool);  // AppState.about_open, ☰ setzt es
// Args { …, version: bool }
```
- Symbol, selbst gezeichnet:
  - abgerundetes Quadrat in `DARK.surface`, darauf eine geschwungene Laufbahn aus Punkten in `DARK.accent`, die sich schließt;
  - geglättete Kanten durch 4×4-Überabtastung, keine Schrift.
- `tests/icon.rs` vergleicht die eingecheckten `assets/icon/*` byteweise mit `icon_png(1024)`, `icon_ico()` und `icon_icns()`. Mit `UPDATE_ICONS=1` schreibt er sie neu (wie `UPDATE_SNAPSHOTS`).
- `build.rs` (beide über `include!("../../packaging/build_commit.rs")`):
  - `DRIFTLESS_COMMIT` = Umgebung `DRIFTLESS_COMMIT`, sonst `git rev-parse --short=9 HEAD`, sonst `"unbekannt"`;
  - `rerun-if-env-changed=DRIFTLESS_COMMIT`, `rerun-if-changed` auf `HEAD`, die Ref-Datei von `HEAD` und `packed-refs` des Git-Ordners (`git rev-parse --git-dir`), soweit vorhanden.
- CLI: `driftless --version` → `driftless <version> (<commit>)`.
- „Über Driftless“ (`egui::Window`, ☰ → „Über Driftless …“) zeigt:
  - `Driftless <version>`, `Commit <commit>`, `Lizenz: Apache-2.0 OR MIT`, Repository-Adresse aus `CARGO_PKG_REPOSITORY`;
  - `Kommandozeile: <pfad>`: `current_exe().parent()/driftless[.exe]`, wenn vorhanden, sonst `"Kommandozeile: nicht im Paket (cargo run -p driftless-cli)"`.
- `build-dmg.sh [--ohne-bauen]`:
  1. Bricht ab, wenn `uname -m` ≠ `arm64`.
  2. `cargo build --release --locked -p driftless-app -p driftless-cli`. Version aus `[workspace.package]` von `Cargo.toml`.
  3. Bündel `target/package/Driftless.app/Contents/`:
     - `Info.plist` aus der Vorlage: `@VERSION@`, `CFBundleExecutable driftless-app`, `CFBundleIdentifier io.github.roccat-xox.driftless`, `CFBundleIconFile driftless`, `LSMinimumSystemVersion 15.0`, `NSHighResolutionCapable true`;
     - `MacOS/driftless-app`, `MacOS/driftless`, `Resources/driftless.icns`.
  4. `codesign --force --deep --sign - Driftless.app`.
  5. Staging mit `Applications`-Verweis, dann `hdiutil create -volname "Driftless <v>" -fs HFS+ -format UDZO -ov target/package/Driftless-<v>-macos-arm64.dmg`.
- `check-dmg.sh <dmg>` (Exit ≠ 0 bei jedem Fehler, `trap` hängt aus):
  1. `hdiutil attach -nobrowse -readonly -mountpoint <tmp>`.
  2. Die Dateiliste des Bündels ist genau die erwartete (`Info.plist`, zwei Programme, `driftless.icns`, `_CodeSignature/*`).
  3. `plutil -lint`, `codesign --verify --deep --strict`.
  4. `Contents/MacOS/driftless --version` und `Contents/MacOS/driftless-app --version` enthalten die Version.
  5. `.dmg` < 500 MB. Größe ausgeben.

- [ ] **Step 1: Fehlschlagende Tests:**
  - `icons_match_committed` (schlägt ohne Dateien fehl);
  - `icon_has_transparent_corners_and_accent_centre`;
  - `parse_args_version`;
  - `about_shows_version_commit_license` (kittest);
  - CLI `version_has_commit`: Ausgabe passt auf `^driftless \d+\.\d+\.\d+ \(.+\)$`.
- [ ] **Step 2:** schlägt fehl. **Step 3:** bauen; Symbole mit `UPDATE_ICONS=1 cargo test -p driftless-app --test icon` erzeugen und einchecken. Das 1024er-PNG ansehen (Read-Werkzeug) und schlicht halten.
- [ ] **Step 4:** `cargo test --workspace` grün, clippy sauber (auch Release-Bins), `cargo deny check` grün.
- [ ] **Step 5: lokal auf dem Mac:** `packaging/macos/build-dmg.sh && packaging/macos/check-dmg.sh target/package/Driftless-*-macos-arm64.dmg`.
  - Erwartet: alle Prüfungen grün, Größe genannt.
  - Danach die App aus dem gemounteten `.dmg` einmal starten: Symbol im Dock, „Über Driftless“ nennt die Kommandozeile im Bündel. Bildschirmfoto für Yannick.
- [ ] **Step 6:** Commit `feat: Programmsymbol, „Über Driftless“, macOS-App und .dmg`.

### Task 9: Windows-Einrichtungsprogramm, Paket-Workflow, Release-Entwurf bei Tag `v*`

**Files:**
- Create:
  - `packaging/windows/{driftless.iss,build-setup.ps1,check-setup.ps1}`
  - `.github/workflows/package.yml`, `.github/workflows/release.yml`
- Modify: `.github/workflows/ci.yml` (die zwei Schritte „driftless-app.exe als Artefakt“ entfallen, das Einrichtungsprogramm ersetzt sie)

**Interfaces:**
- Consumes: Task 8 `assets/icon/driftless.ico`, `build-dmg.sh`, `check-dmg.sh`, `--version` beider Programme.
- `driftless.iss` (Inno Setup 6):
  - `AppName=Driftless`, `AppVersion={#AppVersion}`, `AppPublisher=Driftless`, `DefaultDirName={autopf}\Driftless`.
  - `PrivilegesRequired=lowest`, `PrivilegesRequiredOverridesAllowed=dialog commandline`.
  - `ArchitecturesAllowed=x64compatible`, `ArchitecturesInstallIn64BitMode=x64compatible`.
  - `OutputBaseFilename=Driftless-{#AppVersion}-setup`, `SetupIconFile=..\..\assets\icon\driftless.ico`, `UninstallDisplayIcon={app}\driftless.ico`.
  - `Compression=lzma2`, `SolidCompression=yes`.
  - Sprachen Deutsch und Englisch.
  - `[Files]`: `driftless-app.exe`, `driftless.exe`, `driftless.ico`, `LICENSE-APACHE`, `LICENSE-MIT`.
  - `[Icons]`: `{autoprograms}\Driftless` → `driftless-app.exe` mit `driftless.ico`.
  - Keine Dateizuordnung, kein PATH-Eintrag.
- `build-setup.ps1`: `cargo build --release --locked -p driftless-app -p driftless-cli`, Version aus `Cargo.toml`, `ISCC.exe /DAppVersion=<v> packaging\windows\driftless.iss`. Ausgabe nach `target\package\`.
- `check-setup.ps1 <setup.exe>`:
  1. `/VERYSILENT /SUPPRESSMSGBOXES /NORESTART /CURRENTUSER /DIR=<RUNNER_TEMP>\Driftless-Test`, auf das Ende warten.
  2. Prüfen: `driftless.exe --version` enthält die Version; `driftless-app.exe` liegt da; die Startmenü-Verknüpfung `Driftless.lnk` unter den Programmen des Benutzers ist da.
  3. `unins000.exe /VERYSILENT`. Danach sind Ordner und Verknüpfung weg.
  4. Setup < 500 MB, Größe ausgeben.
- `package.yml`:
  - `on: pull_request` und `workflow_call`;
  - Job `macos` (macos-14): `build-dmg.sh`, `check-dmg.sh`, `upload-artifact` `driftless-macos`, `retention-days: 14`;
  - Job `windows` (windows-latest): `build-setup.ps1`, `check-setup.ps1`, `upload-artifact` `driftless-windows`;
  - beide mit `DRIFTLESS_COMMIT: ${{ github.sha }}` und `Swatinem/rust-cache@v2`.
- `release.yml`:
  - `on: push: tags: ['v*']`, `permissions: contents: write`;
  - Job `package` über `uses: ./.github/workflows/package.yml`;
  - Job `draft` (ubuntu-latest):
    - Tag ohne `v` muss die Version aus `Cargo.toml` sein, sonst Fehler;
    - `download-artifact` beider;
    - `gh release create "$GITHUB_REF_NAME" --draft --verify-tag --title "Driftless <v>" --generate-notes <dmg> <setup.exe>` mit `GH_TOKEN: ${{ github.token }}`;
    - nie `--draft=false`.

- [ ] **Step 1: Lokal prüfen, was ohne Windows geht:**
  - `actionlint .github/workflows/*.yml`, falls vorhanden, sonst `python3 -c "import yaml,sys; [yaml.safe_load(open(f)) for f in sys.argv[1:]]" .github/workflows/*.yml`;
  - `bash -n packaging/macos/*.sh`;
  - die `.iss` auf die Pflichtschlüssel oben durchsehen.
- [ ] **Step 2:** `cargo test --workspace` unverändert grün. Commit `ci: Windows-Einrichtungsprogramm, Paket-Artefakte und Release-Entwurf`.
- [ ] **Step 3: Verified in PR CI**, erst nach Yannicks Okay zum Push:
  - Zweig pushen, PR öffnen.
  - Erwartet:
    - `CI` grün auf beiden Systemen;
    - `package / macos` und `package / windows` grün, mit stiller Installation und Deinstallation im Log;
    - Artefakte `driftless-macos` und `driftless-windows` < 500 MB.
  - Größen mit `gh run view` nennen.
  - Der Release-Workflow wird nicht ausgelöst (kein Tag ohne Okay).

### Task 10: Messlauf, Ergebnisbericht, README (Controller, nicht Subagent)

- [ ] **Step 1:** `DRIFTLESS_RDIR=driftless-b3 ~/.local/bin/driftless-vm sync`, dann `… run cargo build --release --locked -p driftless-cli` als `nohup setsid`-Skript mit Log, begrenzt nachfragen.
- [ ] **Step 2: Echte Daten**, je Lauf `/usr/bin/time -v`, ein schwerer Lauf zur Zeit:
  - Modell `/home/yannick/td/b1fix/t9/m/` → `modell.{las,laz,e57,ply,pcd}`.
  - Büro-Scan B: `$DRIFTLESS_TESTDATA/jmk6-2026-10-20/20261020173822/driftless/`; den Kartenordner vorher mit `ls` bestätigen → dieselben fünf Formate.
  - Je Datei: Größe, Zeit, Spitzen-Speicher, Verhältnis Spitze / `Points`-Bytes (Ziel ≤ 2).
  - `check_export.py` gegen `modell.las` bzw. `color.las`, alle grün. Das E57 des Modells hat drei Scans mit den Ordnernamen und den Lagen aus `modell.json`.
- [ ] **Step 3: Oberfläche und Laufbahn:**
  - Eine Oberfläche des Modells bzw. des Büros, falls vorhanden; sonst `driftless surface --model … --voxel 0.02` als eigener Lauf. Davon glb, OBJ und PLY-Netz mit `check_mesh.py`.
  - Laufbahn eines Laufs als TUM und CSV, CSV gegen TUM verglichen.
- [ ] **Step 4:** Auf dem Mac (leicht) ein E57, ein LAZ und ein OBJ aus Step 2/3 in der App öffnen (über „Datei öffnen …“ und Hineinziehen). Farbe „je Scan“ beim E57, Bildschirmfoto. `.dmg` aus Task 8 und die PR-Artefakte aus Task 9 mit Größe.
- [ ] **Step 5:** `docs/ergebnisse/<Datum>-baustein-3.md` im Stil von `2026-10-27-baustein-1.md`:
  - Was gebaut wurde;
  - Tabellen je Format (Größe, Zeit, Speicher, Gegenprüfung);
  - fremde Dateien;
  - Installer (Größen, Prüfungen, offen: Windows erst nach PR);
  - Abweichungen vom Plan.
- [ ] **Step 6:** README-Abschnitte (englisch, UI-Wörter deutsch):
  - „Export“: `driftless export` mit Beispielen, Formate, Achsen, LAS 0,1 mm, E57 je Scan;
  - „Opening files“;
  - „Installers“:
    - Download aus der Release bzw. den PR-Artefakten;
    - erster Start: macOS Rechtsklick → „Öffnen“, Windows „Weitere Informationen → Trotzdem ausführen“;
    - die Kommandozeile `Driftless.app/Contents/MacOS/driftless` bzw. im Installationsordner;
    - selbst bauen mit `packaging/…`.
- [ ] **Step 7:** Commit `docs: Ergebnisse Baustein 3, README Export und Installer`. Kein Push ohne Okay.
