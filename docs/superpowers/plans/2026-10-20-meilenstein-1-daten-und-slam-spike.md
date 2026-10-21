# Meilenstein 1 – Rohdaten lesen, anzeigen, SLAM-Spike – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Driftless liest JMK6-Aufnahmen (ROS1-Bags + `calib.json`), zeigt sie an, misst Trajektorien und entscheidet per Spike, ob die Laufbahn mit GLIM oder einem eigenen Rust-Kern berechnet wird.

**Architecture:** Cargo-Workspace. `driftless-io` (Bag-Parser, Nachrichten, Kalibrierung, Projekt-Erkennung) ohne GUI-Abhängigkeiten; `driftless-traj` (TUM-Format, Metriken); `driftless-odom` (Rust-Grundlinie LiDAR-IMU-Odometrie für den Spike); `driftless-cli` (Befehle `inspect`, `view`, `compare`, `odom`). Anzeige im Spike über das rerun-SDK; die eigentliche egui/wgpu-Oberfläche ist Meilenstein 2+.

**Tech Stack:** Rust stable (≥ 1.99), `lz4_flex` (LZ4-Frames), `byteorder`, `serde`/`serde_json`, `nalgebra`, `kiddo`, `rerun`, `clap`, `thiserror`, `anyhow`, `image` (JPEG-Decode im Viewer), `las` (Export Grundlinie). Lizenzen aller Crates: MIT/Apache/BSD.

**Spec:** `docs/superpowers/specs/2026-10-20-driftless-design.md`

## Global Constraints

- Lizenz Apache-2.0 OR MIT; keine GPL/AGPL/Inria-3DGS-Abhängigkeit (auch nicht transitiv – `cargo deny check licenses` muss grün sein).
- macOS arm64 und Windows x64 nativ; CI baut und testet beide. Kein WSL, kein ROS.
- Scan-Daten nie ins Repo. Echte Testdaten liegen unter `$DRIFTLESS_TESTDATA` (Pfad lokal frei wählbar); Tests darauf sind `#[ignore]` und laufen mit `cargo test -- --ignored`.
- Nichts aus RayStudio übernehmen (Code, Dateien, Modelle, Texte). Referenzdateien des Testprojekts (`trj.txt`, `smooth.las`) nur lokal zum Vergleich.
- Ganze Aufnahmen nicht komplett in den RAM: Bags werden streamend gelesen (Chunk für Chunk).
- Feste Gerätedaten JMK6: Topics `/vanjee_722z` (PointCloud2, Felder x,y,z,intensity f32, ring u16, timestamp f64 relativ, point_step 26), `/vanjee_imu_packets` (Imu, 200 Hz, Orientierung = 0), `/camera_left/image_raw`, `/camera_right/image_raw` (CompressedImage, `jpeg`).
- Spike-Kriterium aus Spec: GLIM gilt nur als bestanden, wenn es ohne ROS auf macOS **und** Windows baut und läuft.

## Review Focus

- Abgebrochene/abgeschnittene Bag-Datei (Kopie unterbrochen) → klarer Fehler mit Byte-Position, alle vollständigen Chunks davor bleiben lesbar.
- Bag ohne Index (Aufnahme nicht sauber beendet) → wird trotzdem vollständig sequenziell gelesen.
- Punkte mit NaN/Inf oder (0,0,0) → werden verworfen und gezählt, nicht weitergereicht.
- Projektordner mit fehlender Kamera-Bag oder fehlender `calib.json` → `inspect` meldet genau was fehlt; LiDAR-Verarbeitung funktioniert ohne Bilder.
- Nicht monotone Zeitstempel (IMU/LiDAR) → werden erkannt und im `inspect`-Bericht ausgewiesen, Odometrie sortiert bzw. überspringt sie.

---

### Task 1: Workspace, CI, Testdaten bereitstellen

**Files:**
- Create: `Cargo.toml` (Workspace), `crates/driftless-io/`, `crates/driftless-traj/`, `crates/driftless-odom/`, `crates/driftless-cli/` (je `Cargo.toml`, `src/lib.rs` bzw. `src/main.rs`)
- Create: `.github/workflows/ci.yml`, `deny.toml`, `rust-toolchain.toml`, `docs/testdaten.md`

**Interfaces:**
- Produces: Crate-Namen `driftless_io`, `driftless_traj`, `driftless_odom`, Binärprogramm `driftless`; Umgebungsvariable `DRIFTLESS_TESTDATA`; Hilfsfunktion `driftless_io::testdata::path(rel: &str) -> Option<PathBuf>` (None, wenn Variable fehlt).

- [ ] **Step 1:** Workspace anlegen, `rust-toolchain.toml` mit `channel = "stable"`, Workspace-weite Lints (`clippy::all` warn).
- [ ] **Step 2:** CI: Matrix `macos-14` (arm64) und `windows-latest`; Jobs `cargo fmt --check`, `cargo clippy --all-targets -D warnings`, `cargo test --workspace`, `cargo deny check licenses` (erlaubt: MIT, Apache-2.0, BSD-2/3, ISC, Zlib, Unicode-3.0, MPL-2.0 nur für Abhängigkeiten ohne Codeübernahme).
- [ ] **Step 3:** Testdaten lokal kopieren (nicht ins Repo): vom Windows-Testrechner (RayStudio-Projektordner `Test`) → `$DRIFTLESS_TESTDATA/Test/` die Dateien `LIDAR_Test.bag`, `IMAGE_Test.bag`, `calibration/calib.json`, `project_parameters.json`, `Test.ini`, `shading/trj.txt`, `shading/smooth.las`. `docs/testdaten.md` beschreibt Aufbau und Variable, ohne persönliche Pfade.
- [ ] **Step 4:** `cargo test --workspace` grün lokal; Push; CI grün auf beiden Plattformen.
- [ ] **Step 5:** Commit `chore: Workspace, CI und Testdaten-Konvention`.

### Task 2: ROS1-Bag-Parser (streamend)

**Files:**
- Create: `crates/driftless-io/src/bag/{mod.rs,record.rs,reader.rs,writer.rs}`
- Test: `crates/driftless-io/tests/bag.rs`

**Interfaces:**
- Produces:
  - `pub struct BagReader<R: Read + Seek>`; `BagReader::open(path: &Path) -> Result<BagReader<BufReader<File>>, BagError>`
  - `fn connections(&self) -> &[Connection]` mit `Connection { id: u32, topic: String, msg_type: String, md5sum: String }`
  - `fn messages(&mut self, topics: &[&str]) -> impl Iterator<Item = Result<RawMessage, BagError>>` mit `RawMessage { conn_id: u32, topic: String, time: Time, data: Vec<u8> }`, `Time { sec: u32, nsec: u32 }` + `fn as_secs_f64()`
  - `pub enum BagError { Io, NotABag, Truncated { offset: u64 }, UnsupportedCompression(String), Malformed(String) }`
  - Test-Hilfe `bag::writer::BagWriter` (nur `#[cfg(any(test, feature = "test-util"))]`): schreibt Bags mit Kompression `none` oder `lz4`.

- [ ] **Step 1: Failing tests** in `tests/bag.rs`:
  - `reads_uncompressed_bag`: Writer erzeugt 2 Topics × 3 Nachrichten → Reader liefert 6 Nachrichten in Zeitreihenfolge, Topics/Typen stimmen.
  - `reads_lz4_bag`: wie oben mit `lz4` → identische Payloads.
  - `filters_by_topic`: nur `/a` → 3 Nachrichten.
  - `truncated_bag_reports_offset`: Datei nach 70 % abschneiden → alle vollständigen Chunks davor gelesen, letztes Element `Err(Truncated{offset})`.
  - `unindexed_bag_reads_sequentially`: Index-Records weglassen → alle Nachrichten gelesen.
  - `rejects_non_bag`: beliebige Datei → `NotABag`.
  - `#[ignore] real_lidar_bag_summary`: `LIDAR_Test.bag` → Topics `/vanjee_722z` (2.663 Nachrichten) und `/vanjee_imu_packets` (53.264).
- [ ] **Step 2:** `cargo test -p driftless-io --test bag` → FAIL (Typen fehlen).
- [ ] **Step 3:** Implementieren nach ROS-Bag-Format 2.0 (Magic `#ROSBAG V2.0\n`; Record = header_len u32 LE + Header-Felder `name=value` + data_len + data; op-Codes 0x02 Message, 0x03 Bag-Header, 0x04 Index, 0x05 Chunk, 0x06 Chunk-Info, 0x07 Connection). Chunks einzeln dekomprimieren (`lz4_flex::frame::FrameDecoder`), nie die ganze Datei. Reihenfolge: sequentiell durch die Datei; Index nicht nötig.
- [ ] **Step 4:** Tests grün; `#[ignore]`-Test mit `DRIFTLESS_TESTDATA` grün.
- [ ] **Step 5:** Commit `feat(io): streamender ROS1-Bag-Parser`.

### Task 3: Nachrichten dekodieren (PointCloud2, Imu, CompressedImage)

**Files:**
- Create: `crates/driftless-io/src/msg/{mod.rs,pointcloud2.rs,imu.rs,image.rs}`
- Test: `crates/driftless-io/tests/msg.rs`

**Interfaces:**
- Consumes: `RawMessage` (Task 2)
- Produces:
  - `pub struct LidarFrame { stamp: f64, points: Vec<LidarPoint>, dropped: u32 }`, `pub struct LidarPoint { xyz: [f32;3], intensity: f32, ring: u16, t_rel: f64 }`
  - `fn decode_pointcloud2(data: &[u8]) -> Result<LidarFrame, MsgError>` – liest Feldliste dynamisch (Name/Offset/Datentyp), verlangt mindestens x,y,z; ring/intensity/timestamp optional (Default 0)
  - `pub struct ImuSample { stamp: f64, gyro: [f64;3], accel: [f64;3] }`; `fn decode_imu(data: &[u8]) -> Result<ImuSample, MsgError>`
  - `pub struct CompressedImage { stamp: f64, format: String, data: Vec<u8> }`; `fn decode_compressed_image(data: &[u8]) -> Result<CompressedImage, MsgError>`

- [ ] **Step 1: Failing tests:**
  - `pointcloud2_vanjee_layout`: Testpuffer mit point_step 26 (x,y,z,intensity f32 @0/4/8/12, ring u16 @16, timestamp f64 @18), 3 Punkte → Werte exakt.
  - `pointcloud2_drops_nan_and_origin`: 1 NaN-Punkt + 1 (0,0,0) + 1 gültig → `points.len()==1`, `dropped==2`.
  - `pointcloud2_missing_optional_fields`: nur x,y,z → ring 0, t_rel 0.
  - `imu_roundtrip`: gyro (0.1,0.2,0.3), accel (0,0,9.81) → exakt.
  - `compressed_image_jpeg`: format `jpeg`, Daten unverändert.
  - `#[ignore] real_frames_have_4800_points_max`: erste 50 Frames aus `LIDAR_Test.bag` → je ≤ 16×300 Punkte, ring ∈ 1..=16, t_rel ∈ [0, 0.11].
- [ ] **Step 2:** FAIL. **Step 3:** Implementieren (ROS-Serialisierung little-endian: std_msgs/Header = seq u32, stamp sec/nsec u32, frame_id string mit u32-Länge; Arrays mit u32-Länge). **Step 4:** grün.
- [ ] **Step 5:** Commit `feat(io): PointCloud2-, Imu- und Bild-Dekodierung`.

### Task 4: Kalibrierung und Projekt-Erkennung

**Files:**
- Create: `crates/driftless-io/src/{calib.rs,project.rs}`
- Test: `crates/driftless-io/tests/project.rs`, Fixture `crates/driftless-io/tests/fixtures/calib_synthetic.json` (synthetisch, keine echten Werte)

**Interfaces:**
- Produces:
  - `pub struct Calibration { device_sn: Option<String>, device_model: Option<String>, cameras: BTreeMap<String, CameraCalib> }`
  - `pub struct CameraCalib { k: Matrix3<f64>, model: CameraModel, width: u32, height: u32, lidar_to_cam_raw: Option<Matrix4<f64>> }`, `pub enum CameraModel { FisheyeKB { k: [f64;4] }, Pinhole }`
    (`lidar_to_cam_raw` = `out_put.<cam>.transform_matrix` unverändert; Richtung wird in Meilenstein 2 geklärt – Feld heißt bewusst `_raw`)
  - `fn load_calibration(path: &Path) -> Result<Calibration, CalibError>`
  - `pub struct Project { root: PathBuf, lidar_bags: Vec<PathBuf>, image_bags: Vec<PathBuf>, calibration: Option<PathBuf>, problems: Vec<String> }`; `fn discover_project(root: &Path) -> Result<Project, ProjectError>` – erkennt `LIDAR_*.bag`, `IMAGE_*.bag`, `calibration/calib.json`; fehlende Teile landen in `problems`.

- [ ] **Step 1: Failing tests:** `parses_two_fisheye_cameras` (Fixture → Kameras `left`,`right`, KB-Koeffizienten), `missing_camera_bag_is_problem`, `missing_calib_is_problem`, `empty_dir_is_error`, `#[ignore] real_project_jmk6` (SN `<Seriennummer>`, model `JMK6`).
- [ ] **Step 2:** FAIL. **Step 3:** Implementieren mit serde; unbekannte Felder ignorieren. **Step 4:** grün.
- [ ] **Step 5:** Commit `feat(io): Kalibrierung und Projekt-Erkennung`.

### Task 5: `driftless inspect`

**Files:**
- Create: `crates/driftless-cli/src/{main.rs,inspect.rs}`
- Test: `crates/driftless-cli/tests/inspect.rs` (mit `assert_cmd`, synthetisches Projekt aus `BagWriter`)

**Interfaces:**
- Consumes: Tasks 2–4
- Produces: `driftless inspect <ordner> [--json]` – Bericht: Gerät, je Topic Anzahl/Rate (Hz)/Dauer, verworfene Punkte, nicht monotone Zeitstempel (Anzahl), fehlende Teile. `--json` gibt dieselben Werte als JSON (Schlüssel: `device`, `topics[]{topic,type,count,rate_hz,first,last,non_monotonic}`, `dropped_points`, `problems[]`).

- [ ] **Step 1: Failing tests:** `inspect_reports_counts_and_rates` (synthetisch 10 Hz LiDAR, 200 Hz IMU → ±1 %), `inspect_flags_non_monotonic`, `inspect_lists_problems`, `inspect_json_schema`.
- [ ] **Step 2–4:** FAIL → implementieren → grün; manuell `cargo run -- inspect $DRIFTLESS_TESTDATA/Test` → 266 s, 10 Hz, 200 Hz, 2 Kameras ≈ 3,3 Hz.
- [ ] **Step 5:** Commit `feat(cli): inspect`.

### Task 6: Fehlersuche-Viewer `driftless view` (rerun)

**Files:**
- Create: `crates/driftless-cli/src/view.rs`

**Interfaces:**
- Consumes: Tasks 2–4; optional Trajektorie aus Task 7
- Produces: `driftless view <ordner> [--traj <tum.txt>] [--every N]` – startet rerun-Viewer, loggt auf Zeitachse `stamp`: LiDAR-Frame als Punkte (Farbe nach intensity), IMU-Gyro/Accel als Zeitreihen, Kamerabilder (jedes N-te), Trajektorie als Linie.

- [ ] **Step 1:** Implementieren (rerun `RecordingStreamBuilder::spawn`); keine Unit-Tests für Darstellung, aber `view --dry-run` (loggt in Datei `.rrd`) mit Test `view_dry_run_writes_rrd` auf synthetischem Projekt.
- [ ] **Step 2:** Manuell auf Testdaten: Frames, IMU, Bilder sichtbar, flüssig scrubbar; Speicher < 4 GB (streamend).
- [ ] **Step 3:** Commit `feat(cli): Fehlersuche-Viewer mit rerun`.

### Task 7: Trajektorien und Messlatte (`driftless-traj`, `driftless compare`)

**Files:**
- Create: `crates/driftless-traj/src/{lib.rs,tum.rs,metrics.rs}`, `crates/driftless-cli/src/compare.rs`
- Test: `crates/driftless-traj/tests/metrics.rs`

**Interfaces:**
- Produces:
  - `pub struct Pose { t: f64, p: Vector3<f64>, q: UnitQuaternion<f64> }`; `fn read_tum(path) -> Result<Vec<Pose>>`, `fn write_tum(path, &[Pose])`
  - `fn align_umeyama(est: &[Pose], gt: &[Pose], max_dt: f64) -> Isometry3<f64>` (zeitliche Zuordnung nächster Nachbar ≤ `max_dt`, Standard 0.01 s)
  - `fn ate_rmse(est, gt, max_dt) -> f64` (nach Ausrichtung, Meter)
  - `fn start_end_gap(est) -> f64` (Abstand erster ↔ letzter Pose, Meter – Messlatte „Rundgang“)
  - `fn path_length(est) -> f64`
  - CLI `driftless compare <a.tum> <b.tum>` → ATE, Gap a/b, Länge a/b.

- [ ] **Step 1: Failing tests:** `identity_ate_zero`; `rigid_offset_removed_by_alignment` (gleiche Bahn, verschoben+gedreht → ATE < 1e-9); `noise_ate_matches` (σ=0.01 m Rauschen → ATE ≈ 0.01±0.002); `start_end_gap_square_loop` (Quadrat 10 m geschlossen → 0, offen um 0.5 m → 0.5); `tum_roundtrip`.
- [ ] **Step 2–4:** FAIL → implementieren → grün; manuell `compare` RayStudio `trj.txt` mit sich selbst → 0.
- [ ] **Step 5:** Commit `feat(traj): TUM, Ausrichtung und Messlatte`.

### Task 8: Spike A – GLIM auf macOS und Windows

**Files:**
- Create: `spikes/glim/README.md`, `spikes/glim/build-macos.sh`, `spikes/glim/build-windows.ps1`, `spikes/glim/config/` (Sensor-Konfig für Vanjee + IMU), `crates/driftless-cli/src/export_glim.rs` (falls GLIM eigenes Eingabeformat braucht)

**Interfaces:**
- Consumes: Tasks 2–3, 7
- Produces: Ergebnis-Trajektorie `out/glim_traj.tum` und Bewertung in `spikes/glim/README.md`.

- [ ] **Step 1:** Zeitlimit **2 Arbeitstage gesamt**. GLIM + Abhängigkeiten (GTSAM, Eigen, gtsam_points) ohne ROS auf macOS bauen (Homebrew); dann Windows (vcpkg/MSVC).
- [ ] **Step 2:** Eingabe ohne ROS: GLIM-Standalone-Weg nutzen bzw. Bag-Inhalte über unseren Parser in GLIMs Offline-Eingabe überführen.
- [ ] **Step 3:** Lauf auf `LIDAR_Test.bag`; `driftless compare out/glim_traj.tum trj.txt`; Laufzeit und Speicher notieren.
- [ ] **Step 4:** README: baut macOS ja/nein, Windows ja/nein, Aufwand, Ergebnisgüte, Lizenzen aller mitgezogenen Abhängigkeiten.
- [ ] **Step 5:** Commit `spike: GLIM-Bewertung`.

### Task 9: Spike B – Rust-Grundlinie LiDAR-IMU-Odometrie (`driftless-odom`)

**Files:**
- Create: `crates/driftless-odom/src/{lib.rs,imu_integrate.rs,deskew.rs,voxel_map.rs,icp.rs,pipeline.rs}`, `crates/driftless-cli/src/odom.rs`
- Test: `crates/driftless-odom/tests/{deskew.rs,icp.rs,pipeline.rs}`

**Interfaces:**
- Consumes: `LidarFrame`, `ImuSample` (Task 3), `Pose`, `write_tum` (Task 7)
- Produces: `pub struct OdomConfig { voxel_size: f64 /*0.3*/, map_voxel: f64 /*0.5*/, max_range: f64 /*60.0*/, min_range: f64 /*0.5*/, icp_max_iter: u32 /*30*/ }`; `fn run_odometry(frames: impl Iterator<Item=LidarFrame>, imu: &[ImuSample], cfg: &OdomConfig) -> OdomResult` mit `OdomResult { poses: Vec<Pose>, map_points: Vec<[f32;3]> }`; CLI `driftless odom <ordner> -o <out_dir>` → `traj.tum`, `map.las`.

- [ ] **Step 1: Failing tests:**
  - `imu_integration_constant_rotation`: 1 rad/s um z für 1 s → Yaw 1.0±1e-3.
  - `deskew_moves_points_by_motion`: Frame mit bekannter Drehung während 0.1 s → entzerrte Punkte auf Referenzgeometrie (Fehler < 1 cm).
  - `icp_recovers_known_transform`: synthetischer Raum (Wände/Boden), Transformation 0.2 m / 5° → zurückgewonnen < 1 cm / 0.1°.
  - `pipeline_straight_corridor`: synthetischer Flur 30 m, Bewegung 1 m/s → End-Abweichung < 0.3 m.
  - `#[ignore] real_test_runs_under_10_minutes`: `LIDAR_Test.bag` vollständig < 10 min auf M4 Pro, ATE gegen `trj.txt` protokollieren (kein harter Grenzwert im Spike).
- [ ] **Step 2:** FAIL. **Step 3:** Implementieren: IMU-Vorintegration für Startschätzung + Entzerrung (Punkte über `t_rel` auf Frame-Ende), Voxel-Downsampling, Punkt-zu-Ebene-ICP gegen lokale Voxelkarte (k-d-Baum `kiddo`), Karte nachführen. Bewusst ohne Schleifenschluss (kommt in Meilenstein 2). **Step 4:** grün.
- [ ] **Step 5:** Commit `feat(odom): Rust-Grundlinie LiDAR-IMU-Odometrie`.

### Task 10: Spike-Entscheidung

**Files:**
- Create: `docs/entscheidungen/0001-slam-basis.md`

- [ ] **Step 1:** Tabelle GLIM vs. Rust-Grundlinie: baut macOS/Windows, Laufzeit, Speicher, ATE gegen `trj.txt`, Start-Ende-Abstand, Lizenzen, Wartbarkeit, Weg zu Schleifenschluss und manueller Korrektur in Sekunden.
- [ ] **Step 2:** Empfehlung + Begründung; offene Punkte für Meilenstein 2 (Schleifenschluss, Pose-Graph, Drift-Anzeige).
- [ ] **Step 3:** Yannick vorlegen (Deutsch, kurz) – Entscheidung abwarten.
- [ ] **Step 4:** Commit `docs: Entscheidung SLAM-Basis`.
