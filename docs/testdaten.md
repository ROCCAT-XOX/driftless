# Testdaten

Echte Scandaten gehoeren nicht ins Repository (Datenschutz, Groesse). Die
`.gitignore` schliesst `*.bag`, `*.las`, `*.ply` usw. aus. Tests, die echte
Daten brauchen, sind mit `#[ignore]` markiert und laufen nur lokal.

## Umgebungsvariable

`DRIFTLESS_TESTDATA` zeigt auf das Wurzelverzeichnis der Testdaten:

```sh
export DRIFTLESS_TESTDATA=/pfad/zu/driftless-testdata
cargo test --workspace -- --ignored
```

Im Code liefert `driftless_io::testdata::path("Test/LIDAR_Test.bag")` den
vollen Pfad, oder `None`, wenn die Variable nicht gesetzt ist.

## Aufbau

```
$DRIFTLESS_TESTDATA/
  Test/
    LIDAR_Test.bag        LiDAR-Rosbag (ROS1), ca. 300 MB
    IMAGE_Test.bag        Kamera-Rosbag (ROS1), ca. 2,9 GB
    project_parameters.json
    Test.ini
    calibration/
      calib.json          Kalibrierung des Scanners
    shading/
      trj.txt             Referenztrajektorie
      smooth.las          Referenz-Punktwolke
  hilti2022/
    exp14_basement_2.bag            Hilti SLAM Challenge 2022 (Hesai XT-32, Alphasense)
    exp14_basement_2_imu.txt        Referenz-Laufbahn (TUM, IMU-Rahmen, 10 Hz)
    projekt/LIDAR_exp14.bag         Symlink auf die Bag (`ln -s ../exp14_basement_2.bag LIDAR_exp14.bag`)
    exp04_construction_upper_level.bag   Rundgang-Sequenz (125,8 s, Hesai XT-32 + Alphasense-IMU 400 Hz)
    exp04_construction_upper_level.txt   Referenz: 7 Kontrollpunkte, nur Position (s. u.)
    projekt-exp04/LIDAR_exp04.bag   harter Link auf die Bag (`ln ../exp04_construction_upper_level.bag LIDAR_exp04.bag`)
    calibration/lidar_calibration.yaml   LiDAR-IMU-Extrinsik (von Hugging Face)
  ergebnisse-m3/                    lokale Renderings Meilenstein 3 (PNG), nicht im Repo
  jmk6-2026-10-20/
    <JJJJMMTThhmmss>/               eigene JMK6-Aufnahmen, Rohdaten von der MicroSD-Karte, ohne Referenz
      LIDAR_<JJJJMMTThhmmss>.bag    LiDAR + IMU
      IMAGE_<JJJJMMTThhmmss>.bag    Kamerabilder
      project_parameters.json
      calibration/calib.json
      camera/camera_left/, camera/camera_right/
      thumbnail/                    Vorschaubilder des Scanners
```

Zwei Aufnahmen liegen unter `jmk6-2026-10-20/`: `20261020143415` (63 s, ≈ 15 m) und
`20261020143623` (108 s, ≈ 21 m, endet nahe am Start: Start↔Ende schon in der Odometrie 2,6 cm). Ein RayStudio-Ergebnis gibt es dazu nicht.

**exp04 (Rundgang):** dieselbe Extrinsik wie exp14. Die Referenz ist keine dichte Laufbahn, sondern
7 Kontrollpunkte im Abstand von ≈ 20 s, je eine Zeile `t x y z 0 0 0 1` (TUM-Format, Lage nur als
Platzhalter); erster und letzter Punkt sind dieselbe Stelle. Gemessen wird die Abweichung an den
Kontrollpunkten nach starrer Ausrichtung (`driftless compare <traj.tum> exp04_construction_upper_level.txt
--max-dt 0.05`).

Hilti-Daten: CC BY-NC-SA 3.0, nur lokal zum Testen; nichts davon kommt ins Repo.
Beispiel:

```sh
driftless odom $DRIFTLESS_TESTDATA/hilti2022/projekt -o <ziel> \
  --lidar-to-imu -0.001 -0.00855 0.055 0.7071068 -0.7071068 0 0
driftless slam $DRIFTLESS_TESTDATA/hilti2022/projekt-exp04 -o <ziel> \
  --lidar-to-imu -0.001 -0.00855 0.055 0.7071068 -0.7071068 0 0
```

Die Tests mit echten Daten für SLAM: `cargo test --release -p driftless-slam -- --ignored --nocapture`
(Schranken und Messwerte: `docs/ergebnisse/2026-10-20-meilenstein-2.md`).

Für die Karte (dicht, bereinigt, eingefärbt) auf `Test` und beiden JMK6-Aufnahmen:
`cargo test --release -p driftless-map --test real_data -- --ignored --nocapture` (≈ 3 min;
Schranken und Messwerte: `docs/ergebnisse/2026-10-21-meilenstein-3.md`).

**`Test/shading/smooth.las` hat keine Farben:** LAS 1.3, Punktformat 2, 12,5 Mio. Punkte, aber
alle mit derselben Farbe (0, 166, 252) und Intensität 0. Vergleichbar sind nur Lage und Dichte.
`trj.txt` liegt im selben Rahmen wie unsere `traj.tum` (Ausrichtung dreht 0,4°, verschiebt 2 cm).

Renderings und Vergleichsbilder liegen nur lokal unter `$DRIFTLESS_TESTDATA/ergebnisse-m3/`
(nicht im Repo).

Die Dateien unter `Test/` stammen aus einem Testscan mit dem 3DMakerPro-Scanner. Die
Referenzdateien (`trj.txt`, `smooth.las`) dienen nur zum Vergleich; Software
des Scanner-Herstellers wird nicht ins Repository uebernommen.

## Fehlersuche-Viewer

`driftless view <ordner>` braucht den rerun-Viewer in passender Version:
`cargo install rerun-cli --version 0.38.1 --locked`.
Ohne Fenster: `--dry-run datei.rrd` schreibt nur die Aufzeichnung.
