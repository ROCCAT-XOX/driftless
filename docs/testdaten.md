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
```

Die Dateien stammen aus einem Testscan mit dem 3DMakerPro-Scanner. Die
Referenzdateien (`trj.txt`, `smooth.las`) dienen nur zum Vergleich; Software
des Scanner-Herstellers wird nicht ins Repository uebernommen.
