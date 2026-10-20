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
    projekt/LIDAR_exp14.bag         harter Link auf die Bag (Symlinks werden nicht verfolgt)
    calibration/lidar_calibration.yaml   LiDAR-IMU-Extrinsik (von Hugging Face)
```

Hilti-Daten: CC BY-NC-SA 3.0, nur lokal zum Testen; nichts davon kommt ins Repo.
Beispiel:

```sh
driftless odom $DRIFTLESS_TESTDATA/hilti2022/projekt -o <ziel> \
  --lidar-to-imu -0.001 -0.00855 0.055 0.7071068 -0.7071068 0 0
```

Die Dateien unter `Test/` stammen aus einem Testscan mit dem 3DMakerPro-Scanner. Die
Referenzdateien (`trj.txt`, `smooth.las`) dienen nur zum Vergleich; Software
des Scanner-Herstellers wird nicht ins Repository uebernommen.

## Fehlersuche-Viewer

`driftless view <ordner>` braucht den rerun-Viewer in passender Version:
`cargo install rerun-cli --version 0.38.1 --locked`.
Ohne Fenster: `--dry-run datei.rrd` schreibt nur die Aufzeichnung.
