# Baustein 2 – Fotoähnliche Oberfläche – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Aus einer fertigen Karte (`clean.bin` + Laufbahn + Kamerabilder) entsteht eine geschlossene, texturierte Oberfläche (`oberflaeche.glb`, `oberflaeche.obj`), die sich im Programm und in Blender ansehen lässt.

**Architecture:** Neues Modul `driftless-map/src/surface/` mit einer Kette reiner Teilschritte: TSDF aus den bereinigten Punkten mit ihrem Sensorursprung → Marching Cubes (nur in beobachteten Zellen) → Aufräumen (kleine Teile weg, kleine Löcher zu) → Vereinfachen (meshoptimizer) → je Dreieck bestes Bild (Punktzahl + Glättung über Nachbarn) → Karten (zusammenhängende Dreiecke gleichen Bilds) in Texturseiten packen und aus den Bildern in voller Auflösung füllen → Nahtausgleich → glb/obj. Bildauswahl, Belichtung und Randabfall nutzen die vorhandenen Teile von `color.rs`/`exposure.rs` (im selben Crate, `pub(crate)`). `driftless-steps::surface` macht daraus den Schritt (Eingabe-ID, Bericht, veraltet), `driftless surface` und das Programm rufen ihn auf; das Programm bekommt einen Mesh-Renderer.

**Tech Stack:** wie M5; neu `meshopt` (MIT, Vereinfachen), `gltf-json` (MIT/Apache, Schreiben), `gltf` (MIT/Apache, Lesen im Programm), `jpeg-encoder` (schon Dev-Abhängigkeit, wird normale Abhängigkeit von `driftless-map`). Marching-Cubes-Tabellen nach Paul Bourke (gemeinfrei, Quelle im Kommentar).

**Spec:** `docs/superpowers/specs/2026-10-22-fotooberflaeche-design.md` (Ziele 1–6, Abschnitt 6 Entscheidungen). Vorgänger: `docs/ergebnisse/2026-10-22-meilenstein-5.md`.

## Global Constraints

- Lizenz Apache-2.0 OR MIT; nur Abhängigkeiten aus `deny.toml` `allow`, `cargo deny check` grün; nichts unter GPL/AGPL/Inria, nichts aus RayStudio.
- macOS arm64 + Windows x64, CI grün (`fmt`, `clippy -D warnings`, `test --workspace`, `deny`).
- Deterministisch: gleiche Eingabe → byte-gleiche `oberflaeche.glb`/`.obj`/Texturen (feste Reihenfolgen, ganzzahlige Akkumulation, keine Zeit/Zufall in Dateien).
- „Nichts überschreiben“: neuer Ordner je Rechnung, Schreiben über `.tmp` + Umbenennen, Abbrechen hinterlässt keine halben Dateien; alte Ergebnisse werden als veraltet gemeldet, nie gelöscht.
- Grundlage ist `clean.bin` einer Karte; Geometrie weicht davon im Mittel ≤ 3 mm ab (gemessen, in Ergebnissen belegt). Nie gesehene Bereiche und Glaswände bleiben offen.
- Ein Raum-Scan (≈ 5 min, ≈ 2.000 Bilder) < 10 min, Spitzen-RAM < 4 GB nur für diesen Schritt; `.glb` eines Raums < 300 MB. Vor langen Läufen `df -h` (Mac-Platte fast voll).
- glTF: Meter, Y oben: Driftless `(x, y, z)` → glTF `(x, z, −y)`. OBJ bleibt Z oben (Blender-Import „Z up“), steht in `oberflaeche.json`.
- Kommentare Englisch, Meldungen Deutsch. Scan-Daten, Bilder, Kundennamen nie ins Repo; Tests mit echten Daten `#[ignore]` über `$DRIFTLESS_TESTDATA`.

## Review Focus

- Scan ohne Kamerabilder oder ohne `calib.json` → Fläche wird trotzdem gerechnet, grau aus der Intensität, mit Warnung statt Abbruch (Task 9: `surface_without_images_is_gray_with_warning`).
- Dreiecke, die kein Bild sieht (Decke über dem Gerät, Ecken) → bekommen Grau und zählen in `faces_untextured`, kein schwarzes Loch, kein Absturz (Task 6: `unseen_faces_get_gray_block`).
- Dünne Dinge (Leiterholme, Stuhlbeine, Kabelkanal) → bleiben als Fläche erhalten, werden nicht als „kleines Teil“ entfernt oder wegvereinfacht (Task 3: `keeps_thin_rod_attached_to_wall`, Task 4: `simplify_keeps_rod`).
- Glastür/Fensterband (Loch größer als `max_hole_edges`) → bleibt offen (Task 3: `large_hole_stays_open`).
- Karte veraltet oder `clean.bin` fehlt → klare Meldung „zuerst Karte rechnen“, keine Rechnung auf alten Daten (Task 9: `stale_map_is_refused`).

---

### Task 1: TSDF aus bereinigten Punkten

**Files:**
- Create: `crates/driftless-map/src/surface/mod.rs`, `crates/driftless-map/src/surface/tsdf.rs`
- Modify: `crates/driftless-map/src/lib.rs` (`pub mod surface;`), `crates/driftless-map/Cargo.toml`

**Interfaces:**
- Produces:
  - `pub struct SurfaceInput<'a> { pub xyz: &'a [[f32; 3]], pub sensor_of_point: &'a [u32], pub sensors: &'a [[f32; 3]] }` – Punkt `i` wurde von `sensors[sensor_of_point[i]]` aus gemessen (Welt). Für eine Karte: `sensors` = LiDAR-Position je Frame (`pose(frame_end) · lidar_to_imu`), `sensor_of_point` = `Cloud::frame`. Mehrere Scans später = aneinandergehängte Listen (Spec Ziel 6).
  - `pub fn sensors_for_frames(poses: &[Pose], lidar_to_imu: &Isometry3<f64>) -> Vec<[f32; 3]>`
  - `pub struct SurfaceConfig { pub voxel: f64 /* 0.01 */, pub truncation_voxels: f32 /* 4.0 */, pub min_weight: u32 /* 2 */, pub min_component_faces: usize /* 200 */, pub max_hole_edges: usize /* 64 */, pub simplify_error: f64 /* 0.002 m */, pub min_texel: f64 /* 0.002 m */, pub page_size: u32 /* 4096 */, pub views_per_face: usize /* 4 */, pub smooth_weight: f32 /* 0.5 */, pub jpeg_quality: u8 /* 90 */, pub color: ColorConfig /* downscale 1 */ }` mit `Default` und `fn check(&self) -> Result<(), MapError>` (deutsche Meldungen wie `ColorConfig::check`).
  - `pub struct Tsdf` mit `pub fn integrate(input: &SurfaceInput, cfg: &SurfaceConfig, progress: &Progress) -> Result<Tsdf, MapError>`, `pub fn sample(&self, key: [i32; 3]) -> Option<(f32 /* sdf m */, u32 /* weight */)>`, `pub fn voxel(&self) -> f64`, `pub fn blocks(&self) -> impl Iterator<Item = [i32; 3]>`.

Verfahren: dünnbesetzte Blöcke 8³ (Hash `FixedState`), je Punkt nur die Voxel entlang des Strahls im Band ±`truncation_voxels·voxel` um den Endpunkt. Akkumulation ganzzahlig (`AtomicI64` Summe sdf in 0,1 mm, `AtomicU32` Gewicht) → parallel und trotzdem deterministisch. Vorzeichen: positiv vor der Fläche (Sensorseite).

- [ ] **Step 1: Tests** in `tsdf.rs`: `plane_zero_crossing_at_wall` (Ebene x = 2 aus Strahlen vom Ursprung, 1 cm: Nulldurchgang zwischen den Voxeln bei 2,00 ± 0,005 m, sdf > 0 bei x < 2); `unobserved_is_none` (Voxel 1 m hinter der Wand → `None`); `deterministic_parallel` (zweimal integrieren → gleiche `sample` überall); `config_check_rejects` (`voxel = 0`, `page_size = 0` → `Err`).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-map surface::tsdf`).
- [ ] **Step 5: Commit** `feat(map): TSDF aus bereinigten Punkten`

### Task 2: Marching Cubes nur in beobachteten Zellen

**Files:**
- Create: `crates/driftless-map/src/surface/extract.rs`, `crates/driftless-map/src/surface/mc_tables.rs`

**Interfaces:**
- Consumes: `Tsdf` (Task 1)
- Produces: `#[derive(Clone, Debug, Default, PartialEq)] pub struct Mesh { pub positions: Vec<[f32; 3]>, pub indices: Vec<[u32; 3]> }` mit `fn face_normal(&self, f: usize) -> [f32; 3]`, `fn face_area(&self, f: usize) -> f32`; `pub fn extract(tsdf: &Tsdf, cfg: &SurfaceConfig) -> Mesh`.

Eine Zelle liefert Dreiecke nur, wenn alle 8 Ecken `weight >= min_weight` haben (sonst entstünden Flächen an der Rückseite des Bandes). Kantenpunkte über Kante-ID verschweißt (gemeinsame Ecken), Ausrichtung: Normale zeigt zur Sensorseite. Blöcke in sortierter Reihenfolge → deterministische Indizes.

- [ ] **Step 1: Tests:** `box_room_is_closed_inside` (synthetischer Würfelraum 3×3×2,5 m, Sensor in der Mitte, 1 cm: jede Kante gehört zu genau 2 Dreiecken, Normalen zeigen nach innen); `back_of_band_has_no_faces` (einzelne Wand: alle Dreiecke liegen bei x = 2 ± 1 cm, keine bei 2 + 4 cm); `extract_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Fläche aus dem TSDF (Marching Cubes)`

### Task 3: Aufräumen – kleine Teile, kleine Löcher

**Files:**
- Create: `crates/driftless-map/src/surface/cleanup.rs`

**Interfaces:**
- Consumes: `Mesh`
- Produces: `pub struct CleanupStats { pub components_removed: usize, pub faces_removed: usize, pub holes_filled: usize, pub holes_left: usize }`; `pub fn cleanup(mesh: &mut Mesh, cfg: &SurfaceConfig) -> CleanupStats`.

Zusammenhangskomponenten über gemeinsame Kanten; weg, wenn `< min_component_faces` Dreiecke. Randschleifen mit `≤ max_hole_edges` Kanten werden geschlossen (Fächer vom Schwerpunkt, Ausrichtung passend zur Nachbarschaft); größere bleiben offen.

- [ ] **Step 1: Tests:** `removes_floating_speck` (Wand + loses Teilchen mit 20 Dreiecken → Teilchen weg, Wand bleibt); `keeps_thin_rod_attached_to_wall` (Stab 2 cm Durchmesser an der Wand → bleibt, weil mit der Wand verbunden); `fills_small_hole` (Wand mit Loch 10 cm → geschlossen, `holes_filled == 1`); `large_hole_stays_open` (Loch 1 m → offen, `holes_left == 1`).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Fläche aufräumen`

### Task 4: Vereinfachen

**Files:**
- Create: `crates/driftless-map/src/surface/simplify.rs`
- Modify: `crates/driftless-map/Cargo.toml` (`meshopt = "0.4"`), `deny.toml` nur falls nötig

**Interfaces:**
- Produces: `pub fn simplify(mesh: &Mesh, cfg: &SurfaceConfig) -> Mesh` – `meshopt::simplify` mit absolutem Fehler `simplify_error` (umgerechnet in den relativen Fehler über die Ausdehnung), Rand gesperrt (`SimplifyOptions::LockBorder`), danach unbenutzte Ecken entfernen.

- [ ] **Step 1: Tests:** `flat_wall_gets_few_triangles` (Wand 3×2,5 m aus Task 2 → < 1 % der Dreiecke, alle Ecken weiter auf der Ebene ± 1 mm); `simplify_keeps_rod` (Stab aus Task 3 → Hausdorff-Abstand zum Original ≤ 2 mm); `border_unchanged` (Randkanten von `large_hole_stays_open` gleich lang).
- [ ] **Step 2–4:** FAIL → implementieren → PASS; `cargo deny check` grün, CI Windows baut `meshopt` (C++ über `cc`).
- [ ] **Step 5: Commit** `feat(map): Fläche vereinfachen`

### Task 5: Bilder für die Textur – Auswahl, Belichtung, Sichtbarkeit

**Files:**
- Create: `crates/driftless-map/src/surface/views.rs`
- Modify: `crates/driftless-map/src/color.rs` (nur Sichtbarkeit ändern: `quality_pass`, `ImageRecord`, `Status`, `angular_rate`, `frame_camera` → `pub(crate)`; kein Verhalten ändern, alle bestehenden Tests unverändert grün)

**Interfaces:**
- Consumes: `Mesh`, `ColorCamera`, `ImageSource`, `exposure::{Observations, ViewObs, estimate_exposure, Vignetting}`, `visibility::DepthBuffer`
- Produces:
  - `pub struct TextureSource { pub images: ImageSource, pub poses: Vec<Pose>, pub cams: Vec<ColorCamera> }`
  - `pub(crate) struct FaceView { pub image: u32 /* Rang in der Liste benutzter Bilder */, pub score: f32 }`
  - `pub(crate) struct ViewTable { pub images: Vec<UsedImage>, pub per_face: Vec<FaceView> /* views_per_face je Dreieck, leer = score 0 */, pub gains: Vec<[f32; 3]>, pub vignetting: Vignetting }`, `pub(crate) struct UsedImage { pub camera: usize, pub stamp: f64, pub world_from_cam: Isometry3<f64> }`
  - `pub(crate) fn collect_views(mesh: &Mesh, src: &TextureSource, cfg: &SurfaceConfig, progress: &Progress) -> Result<ViewTable, MapError>`

Ein Bilddurchlauf (`SHARPNESS_DOWNSCALE` reicht, nur Geometrie): je Bild Tiefenpuffer aus den Dreiecks-Schwerpunkten + Ecken (`DepthBuffer`), je sichtbarem Dreieck `score = projizierte Fläche in Pixeln (volle Auflösung) × sharpness_norm × cos(θ zur Normalen)`, nur θ_Achse < `rim_theta`, Abstand < `max_dist`; je Dreieck die besten `views_per_face` behalten (`keep_best`-Muster). Belichtung: für eine feste Stichprobe von Dreiecken (jedes k-te, ≤ 200.000) die Farben ihrer Ansichten sammeln → `estimate_exposure` (wie `colorize`).

- [ ] **Step 1: Tests** (synthetische Szene wie in `color.rs`-Tests, Kamerabilder über `testscan`): `front_view_beats_grazing` (Dreieck frontal 1,5 m vs. streifend 80° → frontales Bild hat höheren Score); `occluded_view_dropped` (Säule zwischen Kamera und Wandstück → dieses Bild nicht unter den Ansichten); `blurry_image_never_chosen` (Bild unter `min_sharpness` → nie gewählt); `gains_match_colorize` (gleiche Szene: Gains innerhalb 2 % der `colorize`-Gains).
- [ ] **Step 2–4:** FAIL → implementieren → PASS; `cargo test -p driftless-map` komplett grün (color-Tests unverändert).
- [ ] **Step 5: Commit** `feat(map): Ansichten je Dreieck für die Textur`

### Task 6: Bild je Dreieck festlegen und Texturseiten backen

**Files:**
- Create: `crates/driftless-map/src/surface/atlas.rs`
- Modify: `crates/driftless-map/Cargo.toml` (`jpeg-encoder` von dev nach normal)

**Interfaces:**
- Consumes: `ViewTable`, `Mesh`, `TextureSource`
- Produces:
  - `pub struct TexturePage { pub positions: Vec<[f32; 3]>, pub uvs: Vec<[f32; 2]> /* 0..1, v nach unten */, pub indices: Vec<[u32; 3]>, pub image: RgbImage /* sRGB */ }`
  - `pub struct TexturedMesh { pub pages: Vec<TexturePage> }`
  - `pub struct TextureStats { pub faces_textured: usize, pub faces_untextured: usize, pub charts: usize, pub pages: usize, pub texel_median_mm: f64 }`
  - `pub(crate) fn choose_labels(mesh: &Mesh, views: &ViewTable, cfg: &SurfaceConfig) -> Vec<Option<u32>>` und `pub(crate) fn bake(mesh: &Mesh, labels: &[Option<u32>], views: &ViewTable, src: &TextureSource, cfg: &SurfaceConfig, progress: &Progress) -> Result<(TexturedMesh, TextureStats), MapError>`

Labels: Start = bester Score; dann 5 Runden ICM über Kanten-Nachbarn mit Kosten `−score_norm + smooth_weight · #Nachbarn mit anderem Bild` (feste Reihenfolge der Dreiecke). Karten = zusammenhängende Dreiecke gleichen Labels; UV = Projektion der Ecken ins Bild (volle Auflösung, `Camera::project`), Rechteck + 2 px Rand, verkleinert, wenn feiner als `min_texel`. Packen: Regalverfahren nach Höhe sortiert in Seiten `page_size²`. Füllen: zweiter Bilddurchlauf mit `downscale = cfg.color.downscale` (1), Pixel × Gain × Randabfall-Korrektur → sRGB, nur innerhalb der Kartendreiecke, danach Rand 2 px ausweiten. Dreiecke ohne Label: gemeinsamer 8×8-Block in Seite 0, Grau aus der mittleren Intensität (`GRAY_MIN`-Regel wie `colorize`).

- [ ] **Step 1: Tests:** `neighbours_share_image` (Wand mit zwei fast gleich guten Bildern → nach ICM ≥ 95 % der Dreiecke aus einem Bild); `checkerboard_survives_bake` (synthetische Wand mit Schachbrett im Bild → gebackene Textur zeigt an der UV jedes Felds die richtige Farbe ± 8 sRGB-Stufen); `unseen_faces_get_gray_block` (Dreiecke ohne Ansicht → UV im Grau-Block, `faces_untextured` stimmt); `pages_respect_size` (viele Karten → mehrere Seiten, keine > `page_size`, keine Überlappung der Rechtecke).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Textur backen`

### Task 7: Nahtausgleich

**Files:**
- Create: `crates/driftless-map/src/surface/level.rs`

**Interfaces:**
- Produces: `pub(crate) fn level_seams(mesh: &Mesh, labels: &[Option<u32>], tex: &mut TexturedMesh) -> SeamStats`, `pub struct SeamStats { pub seam_jump_before: f64, pub seam_jump_after: f64 }` (mittlerer sRGB-Sprung entlang Kartengrenzen, Maß wie `seam_jumps` in `color.rs`).

Je Karte eine Farbkorrektur je Ecke (additiv, linear): an Grenzecken die Hälfte der Differenz der beiden Seiten, im Inneren über die Kantennachbarschaft geglättet (feste Zahl Jacobi-Runden), in die Texel der Dreiecke baryzentrisch eingerechnet.

- [ ] **Step 1: Tests:** `seam_jump_drops` (zwei Karten gleicher Wand, eine 20 % heller → `seam_jump_after < 0.3 · seam_jump_before`); `uniform_untouched` (keine Naht → Textur byte-gleich).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Nähte der Textur ausgleichen`

### Task 8: Schreiben (glb, obj) und Vorschaubild

**Files:**
- Create: `crates/driftless-map/src/surface/write.rs`, `crates/driftless-map/src/surface/preview.rs`
- Modify: `crates/driftless-map/Cargo.toml` (`gltf-json`), `crates/driftless-map/src/surface/mod.rs` (`pub fn build_surface(input: &SurfaceInput, sources: &[TextureSource], cfg: &SurfaceConfig, progress: &Progress) -> Result<(TexturedMesh, SurfaceStats), MapError>` = Kette Task 1–7; `sources` leer → nur Grau)

**Interfaces:**
- Produces: `pub fn write_glb(path: &Path, tex: &TexturedMesh, jpeg_quality: u8) -> Result<(), MapError>`, `pub fn write_obj(dir: &Path, stem: &str, tex: &TexturedMesh, jpeg_quality: u8) -> Result<(), MapError>` (`stem.obj`, `stem.mtl`, `stem_seite_NN.jpg`), `pub fn render_preview(tex: &TexturedMesh, eye: [f32; 3], target: [f32; 3], fov_deg: f32, size: [u32; 2]) -> RgbImage` (Software-Rasterer, Z-Puffer, für Ergebnisse und Abnahme), `pub struct SurfaceStats { tsdf_blocks, faces_raw, faces_final, vertices, cleanup: CleanupStats, texture: TextureStats, seams: SeamStats, accuracy_mean_mm: f64, accuracy_p95_mm: f64, seconds: BTreeMap<&'static str, f64> }`.

glb: ein Mesh, eine Primitive je Seite, Material `baseColorTexture` (JPEG eingebettet), `KHR_materials_unlit` (Fotos sind schon beleuchtet), Achsen `(x, z, −y)`. Genauigkeit: für jede Ecke Abstand zum nächsten Punkt von `input.xyz` (k-d-Baum), Mittel und 95 %-Quantil.

- [ ] **Step 1: Tests:** `glb_roundtrip` (schreiben → mit `gltf` lesen: gleiche Dreieckszahl, Positionen umgerechnet ± 1e-6, Textur dekodierbar); `glb_is_y_up` (Boden bei z = 0 → glTF-y = 0, Decke y = +2,5); `obj_files_complete` (obj + mtl + je Seite ein jpg, Verweise stimmen); `byte_identical_twice` (zweimal schreiben → gleiche Bytes); `preview_shows_wall_color` (rote Wand vor der Kamera → Bildmitte rot).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Oberfläche als glb und obj schreiben`

### Task 9: Schritt `surface` und Kommandozeile

**Files:**
- Create: `crates/driftless-steps/src/surface.rs`, `crates/driftless-cli/src/surface.rs`
- Modify: `crates/driftless-steps/src/lib.rs`, `crates/driftless-steps/src/map.rs` (`pub fn clean_stage_id(input: &MapInput) -> Result<String, StepError>` aus `dense_id`/`clean_id`; `pub(crate)` für `color_input`), `crates/driftless-cli/src/main.rs`, `README.md` (Befehlstabelle)

**Interfaces:**
- Consumes: `build_surface`, `write_glb`, `write_obj`, `MapInput`, `load_stage`
- Produces: `pub const SURFACE_REPORT: &str = "oberflaeche.json"`; `pub struct SurfaceRequest { pub map: MapInput, pub map_dir: PathBuf, pub out_dir: PathBuf, pub config: SurfaceConfig }`; `pub fn run_surface(req: &SurfaceRequest, progress: &Progress) -> Result<Value, StepError>`; `pub fn surface_fits(req: &SurfaceRequest) -> bool` (für „veraltet“).
- CLI: `driftless surface <scan> --slam <slam-out> [--map <karten-ordner>, Standard = slam-out] [-o <ziel>, Standard = <karte>/oberflaeche] [--voxel M] [--no-texture]`.

Eingabe-ID = Clean-ID + Bild-ID (`color_input`) + `SurfaceConfig` + `SURFACE_VERSION`. `clean.bin` passt nicht/fehlt → Fehler „Karte fehlt oder ist veraltet: zuerst `driftless map …` rechnen“. Ohne Bilder/Kalibrierung → Warnung, `--no-texture`-Verhalten. Bericht `oberflaeche.json`: Eingabe-ID, Einstellungen, `SurfaceStats`, Spitzen-RAM, Achsen-Hinweis („glb: Y oben; obj: Z oben“). Abbrechen vor dem Schreiben → nichts geschrieben.

- [ ] **Step 1: Tests** (synthetischer Scan wie in `map.rs`-Tests): `surface_end_to_end` (Scan → slam → map → surface: glb + obj + json da, `faces_textured > 0`); `stale_map_is_refused` (Laufbahn danach geändert → `Err` mit „veraltet“); `surface_without_images_is_gray_with_warning`; `cancel_writes_nothing`; `surface_fits_after_run` (true, nach Änderung der Laufbahn false).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-steps surface`, CLI-Test wie bestehende `driftless-cli`-Tests).
- [ ] **Step 5: Commit** `feat: Schritt Oberfläche (driftless surface)`

### Task 10: Echte Daten und Abnahme durch Yannick

**Files:**
- Create: `crates/driftless-steps/tests/surface_real.rs` (`#[ignore]`), `docs/ergebnisse/2026-10-2x-baustein-2.md` (Entwurf)
- Modify: `docs/testdaten.md` (Ordner `jmk6-2026-10-22/` mit den drei Scans des schwarzen Raums, ohne Ortsangabe)

- [ ] **Step 1:** `df -h`; die drei Scans als Verweise (Symlinks) aus `~/Downloads/2026102213*` nach `$DRIFTLESS_TESTDATA/jmk6-2026-10-22/` (keine Kopie, Platte knapp).
- [ ] **Step 2:** `driftless surface` auf `…133554` (und Büro-Scan), mit `/usr/bin/time -l`: Ziel < 10 min, < 4 GB, `.glb` < 300 MB, `accuracy_mean_mm ≤ 3`. Werte ins Ergebnisdokument.
- [ ] **Step 3:** Vergleichsbilder mit `render_preview` (gleiche Blickpunkte: Notausgangsschild, Tür, Leuchtstäbe, Leiter; Oberfläche gegen Punktwolke `color.las` als Punkte gerastert) nach `driftless-testdata/ergebnisse-b2/`.
- [ ] **Step 4: Abnahme:** Bilder an Yannick (`SendUserFile`), auf sein Okay warten. Bei Mängeln: Werte (`voxel`, `smooth_weight`, `min_texel`) nachstellen oder Task 5–7 nachbessern, bevor Task 11 beginnt.
- [ ] **Step 5: Commit** `test: Oberfläche mit echten Daten`

### Task 11: Oberfläche im Programm anzeigen

**Files:**
- Create: `crates/driftless-app/src/render/mesh.rs`, `crates/driftless-app/src/render/mesh.wgsl`, `crates/driftless-app/src/surface.rs` (glb laden)
- Modify: `crates/driftless-app/src/render/mod.rs`, `crates/driftless-app/Cargo.toml` (`gltf`), `crates/driftless-app/src/jobs.rs` (`JobOutput::Surface(Box<LoadedSurface>)`), `crates/driftless-app/src/ui/view3d.rs`

**Interfaces:**
- Produces: `pub struct LoadedSurface { pub origin: [f64; 3], pub pages: Vec<(Vec<MeshVertex>, Vec<u32>, RgbaImage)> }`, `pub fn load_glb(path: &Path, progress: &Progress) -> Result<LoadedSurface, StepError>` (glTF Y-oben zurück nach Z-oben, relativ zum selben `origin` wie die Wolke der Karte); `pub struct MeshRenderer` mit `new`, `upload(&mut self, device, queue, &LoadedSurface)`, `render(&self, pass, &FrameParams)`; `#[repr(C)] pub struct MeshVertex { pub pos: [f32; 3], pub uv: [f32; 2] }`; `pub enum ShowMode { Surface, Cloud, Both }` in `FrameParams`.

Textur unbeleuchtet (Fotos), Höhenschnitt `z_range` im Shader (`discard`), gleicher Tiefenpuffer wie die Punkte, Mipmaps je Seite.

- [ ] **Step 1: Tests** (`headless_device`, wie bestehende Render-Tests; ohne Adapter überspringen): `mesh_renders_texture` (rote Seite vor der Kamera → Mitte rot); `z_cut_hides_ceiling`; `load_glb_matches_written` (glb aus Task 8 → gleiche Dreieckszahl, Positionen ± 1e-5 nach Rückrechnung).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(app): Oberfläche anzeigen`

### Task 12: Oberfläche im Projekt – Schritt, Baum, Umschalter, veraltet

**Files:**
- Modify: `crates/driftless-app/src/project.rs` (`MapInfo.surfaces: Vec<SurfaceInfo>`, `SurfaceInfo { dir, label /* „Oberfläche 001“ */, stale: bool }`, `fn new_surface_dir(map_dir) -> StepDir` → `karte-NNN/oberflaeche-NNN/`), `crates/driftless-app/src/ui/{left,right,actions,app,text}.rs`, `README.md` (Programm-Abschnitt + Projektordner)

**Interfaces:**
- Consumes: `run_surface`, `surface_fits`, `load_glb`, `MeshRenderer`, `ShowMode`

Rechts bei gewählter Karte: Knopf „Oberfläche rechnen“ (Schalter: Voxel 1/1,5/2 cm, „ohne Fotos“), Fortschritt/Abbrechen wie „Karte rechnen“. Links unter der Karte die Oberflächen, veraltete mit Hinweis „veraltet – Karte oder Lauf neu gerechnet“. 3D-Ansicht: Umschalter „Oberfläche / Punkte / beides“ (Tastatur `O`). Alle Texte in `text.rs`, Deutsch.

- [ ] **Step 1: Tests** (`egui_kittest` wie bestehende UI-Tests): `surface_button_runs_step` (synthetisches Projekt → Knopf → Ordner `oberflaeche-001` mit glb); `stale_surface_marked` (Lauf neu rechnen → Eintrag „veraltet“); `toggle_cycles_show_mode` (`O` → Surface → Cloud → Both).
- [ ] **Step 2–4:** FAIL → implementieren → PASS; `cargo test --workspace`, `clippy -D warnings`.
- [ ] **Step 5:** Ergebnisdokument fertig (Zeiten, Speicher, Größen, Genauigkeit, Nahtsprung vorher/nachher, Bilder-Verweise, Grenzen), README Befehle/Programm; Handtest im Programm mit `…133554`.
- [ ] **Step 6: Commit** `feat(app): Oberfläche im Projekt` – danach PR „Baustein 2: Fotoähnliche Oberfläche“, CI Mac + Windows grün.
