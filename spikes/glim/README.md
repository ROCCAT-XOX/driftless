# Spike A: GLIM ohne ROS auf macOS und Windows

Frage: Taugt [GLIM](https://github.com/koide3/glim) (MIT) als externer Prozess für die
Trajektorienberechnung (LiDAR-IMU-SLAM) von Driftless – nativ, ohne ROS, auf macOS arm64
**und** Windows x64?

Stand: 2026-10-20. GLIM `1a197c9` (v1.2.2+), gtsam_points `8e94654` (v1.2.2+), GTSAM `4.3a1`.

## Ergebnis

**GLIM besteht das Spike-Kriterium: baut und läuft ohne ROS auf macOS arm64 und Windows x64.**
Auf Windows aber nur mit eigenen Patches an GLIM und gtsam_points (siehe unten) – ohne
den `long`-Patch rechnet GLIM dort still falsch.

| | macOS arm64 | Windows x64 |
|---|---|---|
| baut | ja (1 Mini-Patch) | ja (5 Patches, GTSAM + gtsam_points statisch) |
| läuft | ja | ja |
| Rechner | Mac (M-Serie, 14 Kerne), 8 Threads | VM 12 vCPU (Ryzen 7 9800X3D), 8 Threads |
| Laufzeit für 266 s Aufnahme | 7,4–8,0 s (≈ 35× Echtzeit) | 11,1–13,5 s (≈ 20× Echtzeit) |
| Spitzen-RAM | ≈ 235 MB | ≈ 175 MB |
| Bauzeit von null (`-j8`) | ≈ 1¾ min (gemessen) | vcpkg 5,7 min + Rest grob 15 min (nicht einzeln gemessen) |
| Platz | `.deps/` ≈ 0,4 GB (nach Aufräumen 0,2 GB) | VS Build Tools 8,2 GB, vcpkg 2,1 GB, Spike 2,8 GB |

Güte auf `Test/LIDAR_Test.bag` (2663 Frames, 53 264 IMU-Messungen), verglichen mit der
RayStudio-Trajektorie `shading/trj.txt` (**keine Ground Truth**) per `driftless compare`:

| Trajektorie | ATE (RMSE) | Start↔Ende GLIM / RayStudio | Länge GLIM / RayStudio |
|---|---|---|---|
| macOS, nach globaler Optimierung (`traj_imu.txt`) | **0,060 m** | 5,394 m / 5,382 m | 84,1 m / 98,9 m |
| macOS, nur Odometrie (`odom_imu.txt`) | 0,116 m | 5,691 m / 5,382 m | 84,1 m / 98,9 m |
| Windows, nach globaler Optimierung | **0,063 m** | 5,399 m / 5,382 m | 84,5 m / 98,9 m |
| Windows, nur Odometrie | 0,306 m | 6,255 m / 5,382 m | 84,4 m / 98,9 m |

- 2632 Posen zugeordnet. GLIM beginnt erst nach dem 3-s-Initialisierungsfenster (2,5 s nach RayStudio).
- Der Längenunterschied ist kein Fehler: RayStudio liefert 200 Hz mit Zittern, GLIM 10 Hz.
  RayStudio nur zu GLIMs Zeitstempeln abgetastet ergibt 87,9 m (GLIM 84,1 m).
- Start↔Ende stimmt mit RayStudio auf 1–2 cm überein; die Aufnahme endet also nicht am Start (≈ 5,4 m Abstand).
- Je Plattform reproduzierbar (3 Läufe, gleiche Zahlen). Zwischen den Plattformen weichen die Ergebnisse
  leicht ab (Zufallsauswahl beim Ausdünnen, andere Standardbibliothek), nach globaler Optimierung nur um 3 mm ATE.

### Was auf Windows kaputt war (und warum das zählt)

1. **Stiller Rechenfehler**: gtsam_points speichert Punkt-Indizes (Voxel-ID << 32 | Punkt-ID) in `long`.
   Unter Windows ist `long` 32 bit → Indizes abgeschnitten, GICP-Zuordnungen falsch. Folge:
   Initialisierung daneben (1,8 m Startversatz, Kreisel-Bias 0,64 rad/s; mit `LOOSE`-Init sogar
   Startlage bei 10⁸ m), ATE 3,9 m, Weglänge 348 m – **ohne Absturz oder Fehlermeldung**.
   Patch: `long` → `std::int64_t` in allen Faktoren (`gtsam_points-0001`).
2. Kompilierfehler unter MSVC: `M_PI`, fehlende Includes (`<numeric>`, `<string>`), Standardargumente
   in expliziten Instanziierungen, `regex_search` auf temporärem String, `/bigobj`.
3. Linkfehler: `struct`/`class`-Vorwärtsdeklarationen gemischt (MSVC kodiert das in Symbolnamen),
   statische Callback-Objekte ohne `dllexport`/`dllimport`, GTSAM-Templates mit `dllimport`.
   Lösung: GTSAM und gtsam_points statisch, GLIM als DLL mit Exportmakro (`glim-0002`, `-0004`).
4. GLIM lädt seine Module mit `dlopen` → `LoadLibrary`/`GetProcAddress` ergänzt (`glim-0002`).

Upstream testet nur Ubuntu (ROS 2). Windows ist für GLIM unerprobtes Gelände: Wer GLIM nutzt,
pflegt diese Patches selbst und muss mit weiteren 32/64-bit-Fehlern in nicht genutzten Pfaden rechnen
(z. B. CT-ICP, LOAM; die Aux-Datei-Erkennung in `PointCloudCPU::load` sucht `/aux_` und findet unter
Windows mit `\` nichts).

### Einschätzung

- Güte: sehr gut. Mit Standardparametern, ohne Feinabstimmung und mit angenommener Identitäts-Extrinsik
  liegt GLIM 6 cm (RMSE) neben RayStudio – das ist der Bereich, in dem wir die Referenz selbst nicht
  mehr beurteilen können.
- Tempo/RAM: unkritisch, CPU-only, deutlich schneller als Echtzeit.
- Risiken: Windows-Patches (Pflegeaufwand, stille Fehler wie oben), lange C++-Abhängigkeitskette
  (GTSAM-Alpha `4.3a1`, Boost, Eigen-Versionsbindung: vcpkg liefert Eigen 5, GTSAM braucht 3.4),
  8 GB MSVC-Toolchain zum Bauen, GLIM-Ausgaben enthalten Log-Fehler zu fehlenden ROS-/Viewer-Configs
  (harmlos).

## Nachtrag: Hilti 2022 `exp14_basement_2` (Task 10, nur macOS)

Objektive Referenz statt RayStudio. Vergleich mit der Rust-Grundlinie: `docs/entscheidungen/0001-slam-basis.md`.

- Hesai XT-32: Feld `timestamp` (f64) enthält **absolute** Zeiten. `export-glim` schreibt sie seit Task 10
  relativ zum Frame-Stempel (vorher f32 der absoluten Zeit = unbrauchbar).
- Extrinsik aus `calibration/calibration_files/lidar_calibration.yaml` (LiDAR-Lage im IMU-Rahmen, `qx qy qz qw`).
  GLIM braucht die Umkehrung: `T_lidar_imu = [-0.00855, -0.001, 0.055, 0.7071068, -0.7071068, 0, 0]`.
  Die Konfiguration wird lokal erzeugt (nicht im Repo, Datensatz CC BY-NC-SA):

```
mkdir -p spikes/glim/out/hilti && cp -r spikes/glim/config spikes/glim/out/hilti/config
# in out/hilti/config/config_sensors.json "T_lidar_imu" wie oben eintragen
driftless export-glim $DRIFTLESS_TESTDATA/hilti2022/projekt -o <tmp>/input
spikes/glim/.deps/prefix/bin/glim_driftless spikes/glim/out/hilti/config <tmp>/input <tmp>/dump
driftless compare <tmp>/dump/traj_imu.txt $DRIFTLESS_TESTDATA/hilti2022/exp14_basement_2_imu.txt --max-dt 0.05
```

Standard-Konfiguration (nur Extrinsik geändert), Treiber mit bis zu 10 Frames Rückstand:

| Lauf | ATE | Laufzeit | Spitzen-RAM |
|---|---|---|---|
| Standard, 8 Threads (3 Läufe) | 0,62 / 0,79 / 1,18 m | 3,9–4,1 s (+ Export 5,2 s) | 407–427 MB |
| 1 Thread (Vorverarbeitung + Odometrie) | 0,75 m | 9,5–10,3 s | ≈ 520 MB |
| 1 Thread, `initialization_mode` LOOSE | 0,46 m | 9,2 s | 537 MB |
| 1 Thread, Voxel 0,25 m, 20 000 Punkte | 1,04 m | 14,7 s | 813 MB |

Befund Initialisierung: Das Gerät steht ≈ 6 s still. `ROBUST` (Standard, 3-s-Fenster) schätzt trotzdem
`v_world_imu ≈ (0, 0, −0,23) m/s` und Bias 0; die ersten 3 s fehlen in der Ausgabe (Initialisierungsfenster).
Daher eingerichtet (zusätzlich zur Extrinsik):

- `config_odometry_cpu.json`: `"initialization_mode": "NAIVE"`, `"initialization_window_size": 1.0`
  (Schwerkraft aus dem Mittel der Beschleunigung, Geschwindigkeit 0).
- `config_sensors.json`: `"imu_bias": [0.0608, 0.0835, 0.0571, -0.00154, -0.00134, 0.00030]`
  (`bias_a`, `bias_g` aus `lidar_calibration.yaml`); `imu_acc_noise` 0,01, `imu_gyro_noise` 0,002
  (geschätzt, die Kalibrierung enthält keine Rauschwerte).
- Treiber: `DRIFTLESS_GLIM_MAX_LAG=0` (Umgebungsvariable). GLIMs Odometrie-Thread verarbeitet vor jedem Frame alle
  bis dahin eingespeisten IMU-Daten; mit Rückstand hängt das Ergebnis (z. B. auf welchem Frame die
  Initialisierung landet) vom Thread-Timing ab. Standard bleibt 10 (JMK6-Ergebnis unverändert, bytegleich).

| Lauf (`MAX_LAG=0`) | ATE | Start↔Ende (21,303) | Laufzeit | Spitzen-RAM |
|---|---|---|---|---|
| `ROBUST`, 1 Thread (2 Läufe gleich) | 0,746 m | 18,62 m | 15,6 s | 475 MB |
| `NAIVE` + Bias, 1 Thread (2 Läufe gleich) | 0,346 m | 21,43 m | 14,9 s | ≈ 450 MB |
| `NAIVE` + Bias + Rauschwerte, 1 Thread (2 Läufe gleich) | **0,326 m** | 21,48 m | 14,6 s | 462 MB |
| `NAIVE` + Bias, 8 Threads (3 Läufe) | 0,45 / 0,54 / 0,37 m | 20,6–21,5 m | 6,4–6,7 s | 387–417 MB |
| `NAIVE` + Bias + Rauschwerte, 8 Threads (2 Läufe) | 0,35 / 0,58 m | 21,38 m | 6,5 s | 387–414 MB |
| `ROBUST`, 8 Threads (2 Läufe) | 0,43 / 0,64 m | 20,2–21,1 m | 6,8 s | 391–396 MB |

- Mit 8 Threads bleibt das Ergebnis auch Frame für Frame **nicht reproduzierbar** (auf JMK6 war es das).
- Nach der Einrichtung hat der Start keinen Sprung mehr (0,1–0,4 m); der Restfehler wächst in den letzten ≈ 15 s bis 0,6 m.
  GLIM meldet weiter zeitweise „IMU prediction is not good“. Ursache offen; die Rust-Grundlinie kommt mit
  derselben Extrinsik auf 0,074 m.

## Aufbau

```
driftless export-glim <scan-ordner> -o out/input   # Rust, liest die Bag streamend
glim_driftless config out/input out/dump           # C++, linkt libglim, ohne ROS
driftless compare out/dump/traj_imu.txt <trj.txt>  # unsere Messlatte
```

- `driver/main.cpp` – kleiner Treiber (≈230 Zeilen). Er macht, was `glim_ros` macht, nur ohne ROS:
  `TimeKeeper` → `CloudPreprocessor` → `AsyncOdometryEstimation` → `AsyncSubMapping` →
  `AsyncGlobalMapping`, am Ende `GlobalMapping::save()`. IMU wird vor jedem Frame bis
  `Frame-Stempel + 0,2 s` eingespeist; der Treiber wartet, solange die Odometrie mehr als
  10 Frames im Rückstand ist (Offline-Wiedergabe so schnell wie möglich).
- `config/` – Kopie der GLIM-Standardkonfiguration (MIT, © koide3) mit diesen Änderungen:
  CPU-Module statt GPU, `T_lidar_imu` = Identität, `ring_field` = `ring`, `num_threads` 8,
  keine Logdateien. Die Modulnamen `lib*.so` übersetzt der Treiber in `.dylib`/`.dll`.
- `patches/` – nötige Änderungen an GLIM (siehe unten).
- `build-macos.sh`, `build-windows.ps1` – bauen alles in einen Präfix (`.deps/` bzw. `C:\Tools\glim-spike`).

### Zwischenformat (`driftless export-glim`)

Zwei Binärdateien, little-endian, in Bag-Reihenfolge. Ungültige Punkte (NaN/Inf, (0,0,0))
sind schon verworfen.

| Datei | Kopf (8 Byte) | Datensatz |
|---|---|---|
| `imu.bin` | `DLIMU01\0` | `f64 stamp, f64 ax, ay, az (m/s²), f64 gx, gy, gz (rad/s)` – 56 Byte |
| `lidar.bin` | `DLPTS01\0` | `f64 stamp, u32 n`, dann n × `f32 x, y, z, f32 intensity, f32 t_rel (s), u32 ring` – 24 Byte je Punkt |

`stamp` ist der Header-Zeitstempel (Sekunden), `t_rel` die Punktzeit relativ dazu (0–0,1 s).

### Sensorannahmen (JMK6)

- Vanjee WLR-722Z, 16 Ringe, 10 Hz, Punktzeit relativ in Sekunden.
- IMU 200 Hz im LiDAR-Körper, Beschleunigung in m/s² (|a| ≈ 9,84), Drehrate rad/s, Orientierung ungenutzt.
- **LiDAR–IMU-Extrinsik: Identität angenommen.** `calib.json` enthält nur Kamera-Extrinsiken,
  keine LiDAR–IMU-Lage. Für die Annahme sprechen: GLIM meldet auf macOS keine einzige
  IMU-Validierungswarnung („IMU prediction is not good“ / „T_lidar_imu is not accurate“), und die
  ATE ist klein. (Die gleiche Startlage wie RayStudio – Quaternion x ≈ 0,261 vs. 0,259 – zeigt nur,
  dass beide die Schwerkraft gleich ausrichten.)

## Bauen und laufen lassen

macOS (Homebrew, ≈ 2 min mit `-j8` auf M-Serie):

```
spikes/glim/build-macos.sh
cargo run --release -p driftless-cli -- export-glim $DRIFTLESS_TESTDATA/Test -o spikes/glim/out/input
spikes/glim/.deps/prefix/bin/glim_driftless spikes/glim/config spikes/glim/out/input spikes/glim/out/dump
cp spikes/glim/out/dump/traj_imu.txt spikes/glim/out/glim_traj.tum
cargo run -p driftless-cli -- compare spikes/glim/out/glim_traj.tum $DRIFTLESS_TESTDATA/Test/shading/trj.txt
```

Windows (VM `10.100.200.33`, alles unter `C:\Tools`):

```
# einmalig: VS 2022 Build Tools nach C:\Tools\VSBuildTools (Workload VCTools + CMake + Clang),
#           vcpkg nach C:\Tools\vcpkg (git clone + bootstrap-vcpkg.bat)
powershell -ExecutionPolicy Bypass -File build-windows.ps1            # alle Stufen
powershell -ExecutionPolicy Bypass -File build-windows.ps1 -Stage glim # einzelne Stufe
# Laufzeit-DLLs von vcpkg müssen im PATH liegen:
$env:PATH = "C:\Tools\glim-spike\prefix\bin;C:\Tools\vcpkg\installed\x64-windows\bin;" + $env:PATH
glim_driftless.exe <config> <input> <dump>
```

Die Eingabe (`imu.bin`, `lidar.bin`) ist plattformneutral und wurde auf dem Mac erzeugt.
Die Stufen wurden einzeln durchlaufen (vcpkg, eigen, gtsam, gtsam_points, glim, driver).

### Patches

| Patch | Plattform | Inhalt |
|---|---|---|
| `glim-0001-vector-bool-fmt` | macOS (fmt 12) | `std::vector<bool>`-Element vor `fmt::format` in `bool` wandeln |
| `glim-0002-windows-dll` | Windows | `GLIM_API` für statische Callback-Slots; `LoadLibrary` statt `dlopen` |
| `glim-0003-msvc-includes` | Windows | `#include <string>` |
| `glim-0004-msvc-struct-forward-decls` | Windows | Vorwärtsdeklarationen `struct` statt `class` |
| `gtsam_points-0001-msvc` | Windows | `long` → `std::int64_t` (Index-Abschneiden), `<numeric>`, Instanziierungen, `regex_search`, `add_library` ohne festes `SHARED` |

Alle Patches werden auf beiden Plattformen angewendet; macOS liefert damit dieselben Zahlen wie ohne.

## Lizenzen der mitgezogenen Abhängigkeiten

Geprüft an Quelltext/LICENSE-Dateien der gebauten Stände; keine GPL/AGPL im gelinkten Ergebnis.

| Komponente | Version | Lizenz | Bemerkung |
|---|---|---|---|
| GLIM | `1a197c9` | MIT | `thirdparty/json` = nlohmann/json (MIT) |
| gtsam_points | `8e94654` | MIT | enthält nanoflann (BSD-2-Clause); SPDX in den Dateien: MIT |
| GTSAM | 4.3a1 | BSD-2-Clause („simplified BSD“) | gebündelt und gelinkt: METIS + GKlib (Apache-2.0), CCOLAMD + SuiteSparse_config (BSD-3-Clause), cephes (MIT), ceres-Autodiff-Header (BSD-3-Clause), Spectra (MPL-2.0, Header); gebündeltes Eigen ungenutzt |
| Eigen | 3.4.1 | MPL-2.0 (Teile BSD/Apache/Minpack) | Header-only; `IncompleteLUT.h` war früher LGPL, ist relizenziert; ungenutzt. Absichern mit `EIGEN_MPL2_ONLY` |
| Boost | 1.92.0 | BSL-1.0 | serialization, filesystem, graph, thread, date_time, timer, chrono, regex, program_options, container, atomic, random, math, poly_collection u. a. |
| spdlog | 1.17.0 | MIT | |
| fmt | 12.2.0 | MIT | |
| nlohmann/json | 3.12.0 | MIT | |
| LLVM OpenMP (libomp, macOS) | 23.1.3 | Apache-2.0 WITH LLVM-exception | Windows: MSVC `/openmp:llvm` (libomp140 aus VC-Laufzeit) |
| MSVC-Laufzeit, Windows SDK | VS 2022 17.x | proprietär, weitergabefähig (VC-Redist) | nur Windows; Build Tools dürfen für Open-Source-Entwicklung genutzt werden |
| vcpkg | aktuell | MIT | nur Bauwerkzeug |

Nicht gebaut und damit nicht mitgezogen: CUDA, Iridescence-Viewer (MIT, aber mit ImGui/GLFW), OpenCV, TBB, ROS.
MPL-2.0 (Eigen, Spectra) ist dateibezogenes Copyleft: unverändert genutzt, keine Pflichten über
Quellangabe hinaus. Da GLIM als separater Prozess liefe, berührt keine dieser Lizenzen den Rust-Code.
