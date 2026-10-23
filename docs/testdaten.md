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
  jmk6-2026-10-22/
    <JJJJMMTThhmmss>/               drei JMK6-Aufnahmen eines dunklen Raums (Aufbau wie oben),
                                    als Verweise (Symlinks) auf die Rohdaten, keine Kopie
      driftless/                    slam + map der Aufnahme; oberflaeche-b2*/ = `driftless surface`
  ergebnisse-b2/                    Vergleichsbilder und Messprotokolle Baustein 2 (nicht im Repo)
```

Zwei Aufnahmen liegen unter `jmk6-2026-10-20/`: `20261020143415` (63 s, ≈ 15 m) und
`20261020143623` (108 s, ≈ 21 m, endet nahe am Start: Start↔Ende schon in der Odometrie 2,6 cm). Ein RayStudio-Ergebnis gibt es dazu nicht.

Eine dritte, lange Aufnahme ist der Größentest für Speicher und Zeit: `20261020173822`
(„Büro“, 582 s, 3.831 Bilder): ein Büro mit Deckenleuchten und Fensterbändern (Spiegelungen
und Durchblicke, die Meilenstein 4 behandelt). Der Drift ohne Schleifenschluss beträgt 2,16 m und
mit automatischem Schleifenschluss 0,13 m. Bei 1 cm Voxelgröße hat die dichte Wolke 15,6 Mio.
Punkte. Eine Referenz gibt es nicht; die Ergebnisse (`slam`, `map`) liegen lokal unter
`$DRIFTLESS_TESTDATA/ergebnisse-buero/`, nicht im Repo.

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
`cargo test --release -p driftless-map --test real_data -- --ignored --nocapture` (ohne die Büro-Aufnahme ≈ 4 min;
Schranken und Messwerte: `docs/ergebnisse/2026-10-21-meilenstein-3.md`).

**`Test/shading/smooth.las` hat keine Farben:** LAS 1.3, Punktformat 2, 12,5 Mio. Punkte, aber
alle mit derselben Farbe (0, 166, 252) und Intensität 0. Vergleichbar sind nur Lage und Dichte.
`trj.txt` liegt im selben Rahmen wie unsere `traj.tum` (Ausrichtung dreht 0,4°, verschiebt 2 cm).

Renderings und Vergleichsbilder liegen nur lokal unter `$DRIFTLESS_TESTDATA/ergebnisse-m3/` und
`$DRIFTLESS_TESTDATA/ergebnisse-m4/` (nicht im Repo).

Seit Meilenstein 4 laufen die Karten-Tests auch auf der Büro-Aufnahme (zwei volle Läufe, bitgleich, dazu ein
Einfärben mit `--highlights`), zusammen ≈ 27 min. Speicher und Zeit der Büro-Aufnahme prüft die CLI, je
Schritt ein eigener Prozess (≈ 8 min):
`cargo test --release -p driftless-cli --test real_data -- --ignored --nocapture`
(Schranken und Messwerte: `docs/ergebnisse/2026-10-21-meilenstein-4.md`).

**Liegen die Testdaten in einem iCloud-Ordner** (z. B. unter `~/Documents` mit „Mac-Speicher
optimieren“), lagert macOS große Dateien bei knappem Speicher aus (`ls -lO` zeigt `dataless`). Das
Lesen bricht dann mit „Operation timed out (os error 60)“ ab. Vorher herunterladen:
`brctl download <datei>` und warten, bis `dataless` verschwindet.

Die Dateien unter `Test/` stammen aus einem Testscan mit dem 3DMakerPro-Scanner. Die
Referenzdateien (`trj.txt`, `smooth.las`) dienen nur zum Vergleich; Software
des Scanner-Herstellers wird nicht ins Repository uebernommen.

**`jmk6-2026-10-22/` („schwarzer Raum“):** drei Aufnahmen desselben dunklen Raums mit schwarz
gestrichenen Wänden, Leuchtstäben an den Wänden, einem beleuchteten Notausgangsschild und einer Leiter.
Wenig Licht, die Wände werfen wenig LiDAR zurück. Ohne Referenz.

| Aufnahme | Dauer | Bilder (beide Kameras) |
|---|---|---|
| `20261022132520` | 252 s | 1.639 |
| `20261022133028` | 264 s | 1.719 |
| `20261022133554` | 345 s | 2.267 |

`slam` und `map` liegen je Aufnahme unter `<aufnahme>/driftless/`, die Oberfläche unter
`<aufnahme>/driftless/oberflaeche-b2/`. Der Test mit echten Daten für die Oberfläche
(`crates/driftless-steps/tests/surface_real.rs`) liest diese Ausgaben (Schranken und Messwerte:
`docs/ergebnisse/2026-10-22-baustein-2.md`).

## Programm (Meilenstein 5)

Die Tests des Programms mit echten Daten lesen die Ausgaben des Release-CLI unter
`$DRIFTLESS_TESTDATA/ergebnisse-m5/` (nicht im Repo):

```
ergebnisse-m5/
  buero/          driftless slam <Büro> -o …/buero, dann driftless map <Büro> --slam …/buero
    karte/color.las   Kopie von buero/color.las (die Wolke, die die Tests laden)
  test/           dasselbe für Test
```

```sh
cargo test -p driftless-app --release --test real_data -- --ignored --nocapture --test-threads 1
cargo test -p driftless-app --release --test lod_memory_real -- --ignored --nocapture
cargo test -p driftless-app --release --test view3d -- --ignored --nocapture   # 50 Mio., GPU
DRIFTLESS_SHOTS=<ordner> cargo test -p driftless-app --release --test ui_screenshots -- --ignored --nocapture
```

`app_steps_match_cli` rechnet `Test` wie das Programm (≈ 3 min) und vergleicht alle Dateien mit
`ergebnisse-m5/test`; nach einer Änderung an SLAM oder Karte also zuerst die CLI-Ausgaben neu
erzeugen. Bildschirmfotos nur in einen Ordner außerhalb des Repos (Schranken und Messwerte:
`docs/ergebnisse/2026-10-22-meilenstein-5.md`).

## Fehlersuche-Viewer

`driftless view <ordner>` braucht den rerun-Viewer in passender Version:
`cargo install rerun-cli --version 0.38.1 --locked`.
Ohne Fenster: `--dry-run datei.rrd` schreibt nur die Aufzeichnung.
