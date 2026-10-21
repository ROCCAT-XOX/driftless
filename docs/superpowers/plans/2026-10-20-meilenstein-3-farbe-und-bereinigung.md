# Meilenstein 3 – Dichte Punktwolke, Bereinigung, Einfärben mit Belichtungsabgleich – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Aus Scan + SLAM-Ergebnis entsteht eine dichte, bereinigte (Geister, Ausreißer) und sauber eingefärbte Punktwolke (Fisheye-Projektion mit Verdeckungstest, Belichtungs-/Weißabgleich, unscharfe Bilder aussortiert), exportiert als LAS (RGB) und PLY.

**Architecture:** Neues Crate `driftless-map` mit drei getrennt aufrufbaren Schritten: `dense` (Odometrie deterministisch erneut laufen lassen, entzerrte Frames mit den SLAM-korrigierten Posen in ein feines Voxelraster legen), `clean` (Freiraum-Prüfung gegen Geister, statistischer Ausreißerfilter; Entferntes landet in einer eigenen Datei), `color` (Kameramodell, Sichtbarkeit je Bild, Bildgüte, Belichtungsabgleich, gewichtete Mischung). `driftless-io` bekommt Kamera-Modell und Bildstrom. CLI-Befehl `driftless map` mit Unterschritten.

**Tech Stack:** wie bisher, dazu ein JPEG-Decoder mit permissiver Lizenz (`zune-jpeg` bevorzugt, sonst `image` mit nur JPEG-Feature), `las` (Format 2/3 mit RGB), eigener PLY-Writer (binär, little endian).

**Spec:** `docs/superpowers/specs/2026-10-20-driftless-design.md` (Ziel 3 „Qualität und Optik“, Ziel 4 Export, Abschnitt 6 Zeitziele, Abschnitt 7 „Klären der Kalibrier-Konvention per Test-Projektion“, Abschnitt 8 Punkt 4).

## Global Constraints

- Lizenz Apache-2.0 OR MIT; nur MIT/Apache/BSD/Zlib-Abhängigkeiten, `cargo deny check` grün.
- macOS arm64 und Windows x64 nativ, CI grün auf beiden; `rust-version = "1.99"`.
- Deterministisch: zwei Läufe → byte-gleiche Ausgabedateien (feste Reduktionsreihenfolge, `FixedState`).
- Spec: „Nichts wird überschrieben, jede Korrektur ist ein rücknehmbarer Schritt.“ Jeder Schritt schreibt eigene Dateien (`dense.las`, `clean.las` + `removed.las`, `color.las`/`color.ply`); Bereinigung entfernt nichts endgültig.
- Spec-Zeitziel auf dem M4 Pro: volle Verarbeitung 5-min-Scan „< 10 min ohne Einfärben, < 20 min mit“. Messlatte hier: JMK6-Testscan (266 s, 1.741 Bilder à 12 MP) `dense + clean + color` < 15 min.
- Spec: „Ganze Aufnahmen dürfen nicht komplett in den RAM müssen.“ Bilder werden einzeln gestreamt und nach Verarbeitung verworfen; Spitzen-RAM < 8 GB auf dem Testscan.
- GPU: Spec verlangt wgpu (Metal/DX12/Vulkan), „Zuverlässigkeit vor Geschwindigkeit“. In diesem Meilenstein CPU mit rayon; die schweren Schleifen (Projektion je Bild, Freiraum-Strahlen) als reine Per-Element-Funktionen + feste Reduktion schreiben, damit sie später als wgpu-Compute laufen können (wie `icp.rs`).
- Kalibrier-Konvention (`transform_matrix` L→C oder C→L, Zeitversatz Kamera) wird in Task 2 gemessen und in `docs/entscheidungen/0002-kamera-kalibrierung.md` festgehalten; vorher nichts hart verdrahten.
- Code-Kommentare Englisch, Meldungen Deutsch. Scan-Daten, Bilder, Seriennummern nie ins Repo; Tests mit echten Daten `#[ignore]`.

## Review Focus

- Scan ohne `IMAGE_*.bag` oder ohne `calib.json` → `dense` und `clean` laufen, `color` meldet deutsch, was fehlt; kein Absturz (Task 7).
- Kaputtes/abgeschnittenes JPEG oder Bild mit falscher Größe → Bild wird übersprungen und gezählt (Task 1).
- Bilder außerhalb der Laufbahn-Zeit (vor erster/nach letzter Pose) → übersprungen, nicht extrapoliert (Task 5).
- Punkt, den keine Kamera sieht → bleibt drin, bekommt Kennzeichnung „ohne Farbe“ (Grau nach Intensität), nicht Schwarz (Task 5).
- Person läuft durchs Bild, aber steht auch kurz still → Freiraum-Prüfung entfernt keine statischen Wände, die nur einmal verdeckt waren (Task 4).

---

### Task 1: Kameramodell und Bildstrom (`driftless-io`)

**Files:**
- Create: `crates/driftless-io/src/camera.rs`, `crates/driftless-io/src/images.rs`
- Modify: `crates/driftless-io/src/lib.rs`, `crates/driftless-io/Cargo.toml`
- Test: `crates/driftless-io/tests/camera.rs`, `crates/driftless-io/tests/images.rs`

**Interfaces:**
- Produces:
  - `pub struct Camera { pub name: String, pub width: u32, pub height: u32, pub fx: f64, pub fy: f64, pub cx: f64, pub cy: f64, pub model: CameraModel }` mit `Camera::from_calib(name, &CameraCalib) -> Result<Camera, CalibError>`.
  - `pub fn project(&self, p_cam: &Vector3<f64>) -> Option<[f64; 2]>` (Kannala-Brandt: `θ = atan2(r, z)`, `θd = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)`, `u = fx·θd·x/r + cx`; `None` bei θ > `max_theta` oder außerhalb des Bildes) und `pub fn unproject(&self, uv: [f64; 2]) -> Option<Vector3<f64>>` (Newton auf θd, max. 20 Iterationen, Einheitsvektor). `pub max_theta: f64` = größtes θ, bei dem `θd` noch monoton steigt, begrenzt auf 100°.
  - `pub struct Frame { pub camera: String, pub stamp: f64, pub rgb: RgbImage }` mit `pub struct RgbImage { pub width: u32, pub height: u32, pub data: Vec<u8> }` (RGB8, zeilenweise) und `pub fn decode_jpeg(bytes: &[u8], downscale: u32) -> Result<RgbImage, ImageError>` (`downscale` ∈ {1, 2, 4}, 2×2- bzw. 4×4-Mittelung).
  - `pub fn for_each_image(bags: &[PathBuf], topics: &[String], downscale: u32, f: impl FnMut(Result<Frame, ImageError>) -> bool) -> Result<ImageStats, BagError>` – streamt, nie mehr als ein dekodiertes Bild je Arbeitsthread im Speicher; Kameraname = Topic-Teil `camera_left` → `left`.

- [ ] **Step 1: Tests** – `kb_project_unproject_round_trip` (Kalibrierwerte aus `tests/fixtures/calib_synthetic.json`, 1.000 Richtungen bis θ = 90°: Winkelfehler < 1e-9 rad). `kb_center_maps_to_principal_point`. `kb_rejects_behind_and_beyond_max_theta`. `decode_jpeg_downscale` (kleines im Test erzeugtes JPEG, Größe und Mittelwert stimmen). `corrupt_jpeg_is_error_not_panic` (abgeschnittene Bytes, Zufallsbytes). `image_stream_counts_bad_images` (synthetische Bag mit 3 gültigen, 1 kaputten Bild über den `test-util`-Writer → 3 Frames, `ImageStats.bad == 1`).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-io`), `cargo deny check` nach JPEG-Crate.
- [ ] **Step 5: Commit** `feat(io): Fisheye-Kameramodell und Bildstrom`

### Task 2: Kalibrier-Konvention und Zeitversatz messen (Spike mit Entscheidung)

**Files:**
- Create: `crates/driftless-cli/src/calib_check.rs`, `docs/entscheidungen/0002-kamera-kalibrierung.md`
- Modify: `crates/driftless-io/src/calib.rs` (nach der Messung: `lidar_to_cam_raw` → `pub lidar_to_cam: Option<Isometry3<f64>>` mit der gemessenen Richtung; Doku-Kommentar nennt ADR 0002), `crates/driftless-cli/src/main.rs`
- Test: `crates/driftless-cli/tests/calib_check.rs`, `crates/driftless-io/tests/project.rs` bzw. bestehende Calib-Tests anpassen

**Interfaces:**
- Produces: `driftless calib-check <scan> --slam <ziel> -o <ordner> [--images N]`: wählt N (Standard 12) scharfe Bilder je Kamera gleichmäßig über den Scan, projiziert die SLAM-Karte (Teilkarten-Punkte an korrigierten Posen, Kamerapose = Laufbahn interpoliert zur Bildzeit · LiDAR→Kamera) für beide Richtungen (`T` und `T⁻¹`) und Zeitversätze −0,20 … +0,20 s in 0,01-s-Schritten. Güte = mittlerer normierter Bildgradient an den projizierten Tiefensprung-Kanten (Punkte, deren Nachbar im Bild > 0,3 m weiter weg ist). Schreibt `calib_check.json` (Güte je Variante), Überlagerungsbilder (PNG, Punkte nach Tiefe eingefärbt) für die beste und die schlechteste Variante.
  - Hilfsfunktion `pub fn interpolate_pose(poses: &[Pose], t: f64) -> Option<Isometry3<f64>>` (Slerp/linear, `None` außerhalb) in `driftless-traj`.
- Entscheidung: beste Richtung + Zeitversatz kommen in ADR 0002 (Messwerte für Test-Scan und beide Scans aus `jmk6-2026-10-20`, Tabellen, Überlagerungsbilder nur lokal). Die Richtung wird im Code fest, der Zeitversatz ist `ColorConfig.time_offset` (Standard = gemessener Wert).

- [ ] **Step 1: Tests** – `interpolate_pose_midpoint_and_bounds`. `calib_check_finds_synthetic_truth` (synthetische Szene: Kanten-Bild aus bekannter Pose gerendert, bekannte Richtung und Versatz +0,05 s → `calib_check.json` nennt genau diese Variante als beste). `calib_check_without_images_errors` (deutsch).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Messen** auf `$DRIFTLESS_TESTDATA/Test` und den zwei `jmk6-2026-10-20`-Scans (vorher `driftless slam`), ADR 0002 schreiben, `lidar_to_cam` umstellen.
- [ ] **Step 6: Commit** `feat(calib): Kalibrier-Konvention per Test-Projektion bestimmt (ADR 0002)`

### Task 3: Dichte Punktwolke (`driftless-map`, Teil 1)

**Files:**
- Create: `crates/driftless-map/` (`Cargo.toml`, `src/lib.rs`, `src/dense.rs`, `src/cloud.rs`, `src/export.rs`), Workspace ergänzen
- Test: `crates/driftless-map/tests/dense.rs`, `crates/driftless-map/tests/export.rs`

**Interfaces:**
- Consumes: `run_odometry_with`, `FrameOutput`, `driftless_slam::SlamResult::poses()` bzw. `traj.tum` des SLAM-Laufs, `OdomConfig`.
- Produces:
  - `pub struct Cloud { pub xyz: Vec<[f32; 3]>, pub intensity: Vec<f32>, pub t: Vec<f64> /* Aufnahmezeit */, pub frame: Vec<u32>, pub rgb: Option<Vec<[u8; 3]>>, pub colored: Option<Vec<bool>> }` (Spalten gleich lang; `Cloud::len`, `Cloud::select(&[bool]) -> (Cloud, Cloud)` für behalten/entfernt).
  - `pub struct DenseConfig { pub voxel: f64 /* 0.02 m */, pub max_range: f64 /* 30 m */, pub min_range: f64 /* 0.5 m */ }`.
  - `pub fn build_dense(frames, imu, odom_cfg: &OdomConfig, corrected: &[Pose], cfg: &DenseConfig) -> Result<(Cloud, DenseStats), MapError>` – Odometrie erneut (deterministisch, gleiche Frames), jede `FrameOutput.body_points` mit `corrected[index]` in die Welt; Fehler, wenn die Posenzahl nicht zur Frame-Zahl passt (anderer Scan/Einstellungen). Je Voxel bleibt der Punkt mit dem kleinsten Abstand zum Sensor (Nahes ist genauer); Intensität und Zeit dieses Punkts.
  - `pub fn write_las(path, &Cloud)` (Format 3: XYZ, Intensität, GPS-Zeit, RGB falls vorhanden; Skalierung 0,1 mm) und `pub fn write_ply(path, &Cloud)` (binär LE: x y z float, intensity float, red green blue uchar falls vorhanden).
  - `FrameOutput` muss dafür die Zeit je Punkt nicht liefern: Zeit = Frame-Zeit.

- [ ] **Step 1: Tests** – `dense_reproduces_room_geometry` (synthetische Raumszene aus `driftless_odom::testutil`, korrigierte Posen = Wahrheit: 95 % der Punkte < 0,03 m von einer Wand). `dense_voxel_keeps_nearest`. `dense_rejects_pose_count_mismatch` (deutsch). `dense_is_deterministic`. `las_round_trip_with_rgb` (mit `las`-Reader zurücklesen). `ply_header_and_size`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-map`).
- [ ] **Step 5: Commit** `feat(map): dichte Punktwolke aus korrigierter Laufbahn`

### Task 4: Bereinigung – Geister und Ausreißer

**Files:**
- Create: `crates/driftless-map/src/clean.rs`
- Test: `crates/driftless-map/tests/clean.rs`

**Interfaces:**
- Produces:
  - `pub struct CleanConfig { pub ghost_voxel: f64 /* 0.1 m */, pub ray_step: usize /* jeder 4. Punkt je Frame */, pub ghost_ratio: f64 /* 3.0 */, pub min_passes: u32 /* 5 */, pub outlier_k: usize /* 16 */, pub outlier_sigma: f64 /* 2.5 */ }`.
  - `pub fn find_ghosts(cloud: &Cloud, rays: &RaySource, cfg: &CleanConfig) -> Vec<bool>` – Freiraum-Zählung: je Frame Strahlen Sensor → Messpunkt (3D-DDA im `ghost_voxel`-Raster), Zähler „durchquert“ je Voxel, das *vor* dem Endpunkt liegt (letzte 2 Voxel ausgenommen), und „getroffen“ je Endvoxel. Punkt ist Geist, wenn sein Voxel `durchquert ≥ min_passes` und `durchquert ≥ ghost_ratio × getroffen`. `RaySource` = Sensorposition + Endpunkte je Frame, beim Dichtelauf aus Task 3 mitgeschrieben (Datei `rays.bin` oder im Speicher als Teilmenge laut `ray_step`).
  - `pub fn find_outliers(cloud: &Cloud, cfg: &CleanConfig) -> Vec<bool>` – mittlerer Abstand zu `outlier_k` Nachbarn (kiddo) > Mittel + `outlier_sigma`·σ.
  - `pub fn clean(cloud: Cloud, ...) -> (Cloud /*behalten*/, Cloud /*entfernt*/, CleanStats)`.

- [ ] **Step 1: Tests** – `removes_person_walking_through` (synthetische Szene: Raum + Kiste, die sich während des Scans 3 m bewegt → > 90 % der Kistenpunkte an alten Positionen entfernt, < 1 % der Wandpunkte entfernt). `keeps_wall_seen_once_behind_occluder` (Wand nur kurz hinter einer stehenden Kiste sichtbar → bleibt). `removes_isolated_outliers` (zufällige Einzelpunkte in der Luft → entfernt; Wandkanten bleiben). `clean_is_deterministic`. `removed_plus_kept_equals_input`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Geister- und Ausreißerfilter`

### Task 5: Einfärben – Sichtbarkeit, Bildgüte, gewichtete Mischung

**Files:**
- Create: `crates/driftless-map/src/color.rs`, `crates/driftless-map/src/visibility.rs`, `crates/driftless-map/src/sharpness.rs`
- Test: `crates/driftless-map/tests/color.rs`

**Interfaces:**
- Consumes: `Camera`, `for_each_image`, `interpolate_pose`, `lidar_to_cam` (Task 2), `ColorConfig.time_offset`.
- Produces:
  - `pub struct ColorConfig { pub downscale: u32 /* 2 */, pub time_offset: f64, pub max_dist: f64 /* 12 m */, pub depth_tolerance: f64 /* 0.05 m + 2 % der Entfernung */, pub zbuffer_scale: u32 /* Tiefenpuffer = Bild / 4 */, pub min_sharpness: f64, pub max_angular_rate: f64 /* 1.0 rad/s aus der Laufbahn */, pub views_per_point: usize /* 3 */ }`.
  - `pub fn sharpness(img: &RgbImage) -> f64` – Varianz des Laplace-Filters auf Graustufen; `min_sharpness` Standard = 0,4 × Median der Szene (relativ, wird je Scan bestimmt).
  - Sichtbarkeit je Bild: Punkte im `max_dist`-Umkreis vorab über ein grobes Raster gesucht, in den Tiefenpuffer gesplattet (Minimum), Punkt sichtbar, wenn Tiefe ≤ Puffer + `depth_tolerance`.
  - Gewicht einer Ansicht: `w = cos(Einfallswinkel) / (1 + d²/4) × Randabfall (1 bei θ ≤ 60°, linear auf 0 bei max_theta) × sharpness_norm`. Je Punkt die `views_per_point` besten Ansichten behalten (feste Reihenfolge bei Gleichstand: Bildzeit, Kameraname), Farbe = gewichtetes Mittel in linearem RGB (sRGB ↔ linear).
  - `pub fn colorize(cloud: &mut Cloud, images: ImageSource, poses: &[Pose], cams: &[Camera], cfg: &ColorConfig) -> Result<ColorStats, MapError>`; `ColorStats { images_used, images_blurry, images_outside_trajectory, images_bad, points_colored, points_uncolored }`. Ungefärbte Punkte: Grau aus Intensität, `colored = false`.

- [ ] **Step 1: Tests** (synthetisch, gerenderte Bilder aus bekannter Szene mit farbigen Wänden) – `colors_match_rendered_walls` (> 95 % der Punkte auf ±8/255 der Wandfarbe). `occluded_points_take_no_color_from_occluder` (Wand hinter Kiste: Wandpunkte nicht in Kistenfarbe). `blurry_and_fast_images_are_skipped` (Bild mit Gauß-Unschärfe und Bild bei hoher Drehrate → gezählt, nicht genutzt). `images_outside_trajectory_skipped`. `uncolored_points_are_gray_not_black`. `colorize_is_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Einfärben mit Verdeckungstest und Bildgüte`

### Task 6: Belichtungs- und Weißabgleich

**Files:**
- Create: `crates/driftless-map/src/exposure.rs`
- Modify: `crates/driftless-map/src/color.rs` (Abgleich vor der Mischung anwenden)
- Test: `crates/driftless-map/tests/exposure.rs`

**Interfaces:**
- Produces:
  - Je Bild ein Faktor-Tripel `g_i = (r, g, b)` (linear). Schätzung in zwei Durchläufen über die Bilder: (1) Stichprobe von bis zu 50.000 Punkten, die in ≥ 2 Bildern sichtbar sind, sammelt die beobachteten linearen Farben je Bild; (2) kleinste Quadrate auf `log` je Kanal: `log c_ij − log g_i = log a_j` mit Glättung zwischen zeitlich benachbarten Bildern derselben Kamera (Gewicht `smooth`) und Verankerung Mittel `log g = 0`. `pub fn estimate_gains(obs: &Observations, cfg: &ExposureConfig) -> Vec<[f32; 3]>`; `ExposureConfig { pub smooth: f64 /* 1.0 */, pub max_gain: f64 /* 4.0 */, pub samples: usize /* 50_000 */ }`.
  - `ColorStats` neu: `gain_min`, `gain_max`, Farbstreuung zwischen Ansichten vorher/nachher (mittlere Standardabweichung der Ansichtsfarben je Punkt) – das ist die Messgröße „Farben passen über Kameras/Zeit zusammen“.

- [ ] **Step 1: Tests** – `recovers_injected_gains` (synthetisch: rechte Kamera 1,3× heller und blaustichig, linke über die Zeit 0,8 → 1,2: Faktoren auf 3 % genau, nach Abgleich Streuung < 25 % von vorher). `gains_are_clamped`. `single_camera_single_image_is_identity`. `exposure_is_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Belichtungs- und Weißabgleich zwischen Kameras und über die Zeit`

### Task 7: Befehl `driftless map` und Anzeige

**Files:**
- Create: `crates/driftless-cli/src/map.rs`
- Modify: `crates/driftless-cli/src/main.rs`, `crates/driftless-cli/src/view.rs` (`--cloud datei.las` zeigt eine LAS mit Farbe/Intensität ausgedünnt auf ≤ 5 Mio. Punkte), `README.md`
- Test: `crates/driftless-cli/tests/map.rs`

**Interfaces:**
- Produces: `driftless map <scan> --slam <ziel> [--step dense|clean|color|all] [--no-ghosts] [--no-outliers] [--voxel M] [--downscale 1|2|4]` → im Zielordner `dense.las`, `clean.las`, `removed.las`, `color.las`, `color.ply`, `map.json` (Einstellungen, Statistiken aller Schritte, Laufzeiten, Spitzen-RAM falls verfügbar). Jeder Schritt liest das Ergebnis des vorigen aus dem Zielordner, falls es zu Eingabe + Einstellungen passt (Kennung wie `submaps.bin`), sonst wird es neu erzeugt. Fehlt `IMAGE_*.bag` oder `calib.json`: `dense`/`clean` laufen, `color` endet mit deutscher Meldung und Exit-Code ≠ 0 nur bei `--step color`; bei `all` Warnung und `color.*` fehlt.
  - Konsole: Punktzahlen je Schritt, entfernt (Geister/Ausreißer), Bilder genutzt/unscharf/außerhalb/kaputt, Farbanteil, Laufzeit je Schritt.

- [ ] **Step 1: Tests** – `map_all_writes_outputs` (synthetische Bag mit Bildern + Kalibrierung), `map_without_images_skips_color`, `map_step_color_reuses_clean`, `map_is_deterministic` (byte-gleiche `color.las`), `view_cloud_dry_run`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test --workspace`).
- [ ] **Step 5: Commit** `feat(cli): driftless map (dicht, bereinigt, eingefärbt)`

### Task 8: Prüfung mit echten Daten, Ergebnisse

**Files:**
- Create: `crates/driftless-map/tests/real_data.rs` (`#[ignore]`), `docs/ergebnisse/2026-10-21-meilenstein-3.md`
- Modify: `docs/testdaten.md`, `README.md`; Standardwerte in `driftless-map` nur, wenn die Messung es verlangt (jede Änderung im Ergebnisdokument begründet)

- [ ] **Step 1: Messen** auf `Test` (Vergleich mit RayStudio `shading/smooth.las`: Punktzahl, Anteil gefärbt, Farbstreuung zwischen Ansichten vorher/nachher Abgleich, Laufzeit je Schritt, Spitzen-RAM) und auf beiden `jmk6-2026-10-20`-Scans. Dazu je Scan drei Ausschnitt-Ansichten (Rendering der Punktwolke von oben und zwei Innenansichten als PNG, lokal im Testdaten-Ordner, nicht im Repo) von Driftless und RayStudio nebeneinander.
- [ ] **Step 2: Tests mit gemessenen Schranken** – `test_scan_full_under_15_min`, `test_scan_colored_share` (≥ Messwert × 0,95), `exposure_reduces_spread` (Streuung nachher < vorher), `jmk6_scans_run_and_are_deterministic`.
- [ ] **Step 3: Ergebnisdokument** (Deutsch, Tabellen, ehrliche Grenzen) und Doku.
- [ ] **Step 4: Gesamtprüfung** – `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo deny check`, `cargo fmt --all --check`, `cargo test --release -- --ignored`.
- [ ] **Step 5: Commit** `test(map): Prüfung mit echten Daten, Ergebnisse Meilenstein 3`
