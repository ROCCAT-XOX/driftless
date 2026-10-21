# Driftless – Design (Entwurf)

Stand 20.10.2026 · Arbeitstitel „Driftless“ (änderbar) · Autor: Yannick Stumpf mit Claude
Grundlage: eigene Analyse der Hersteller-Software und der Rohdaten (nicht veröffentlicht).

## 1. Worum es geht und warum

Yannick besitzt einen 3DMakerPro-LiDAR-Scanner (Gerät `JMK6`, Vanjee WLR-722Z mit IMU, zwei 12-MP-Fisheye-Kameras).
Die Hersteller-Software RayStudio ist unreif: nur Windows + NVIDIA, instabil, 10,7 GB, Drift lässt sich kaum
korrigieren, Farben und Wolken sind sichtbar unsauber, Pflicht-Login, Passwort im Klartext.

**Driftless** ist ein quelloffenes Desktop-Programm für **macOS (Apple Silicon) und Windows**, das die Rohdaten des
Scanners einliest, daraus eine saubere, driftfreie, schön eingefärbte Punktwolke rechnet, Korrekturen direkt in der
3D-Ansicht erlaubt und in Standardformate exportiert. Vorbild der Bedienung: **CloudCompare, aber moderner**.

Erfolg heißt: Ein Scan aus dem eigenen Gerät landet ohne RayStudio auf dem Mac als Punktwolke, die sichtbar
**weniger Drift** und **sauberere Farben** hat als das RayStudio-Ergebnis desselben Scans, und Drift lässt sich
in der Oberfläche erkennen und von Hand beheben.

## 2. Ziele

1. **Rohdaten lesen:** ROS1-Bags (`LIDAR_*.bag`: `/vanjee_722z` PointCloud2 mit x,y,z,intensity,ring,timestamp;
   `/vanjee_imu_packets` Imu 200 Hz; `IMAGE_*.bag`: `/camera_left|right/image_raw` CompressedImage JPEG) und
   `calibration/calib.json` (K, Fisheye Kannala-Brandt k1–k4, 4×4 LiDAR↔Kamera, `device.SN/model`).
2. **Drift abfangen** (Schwerpunkt):
   - LiDAR-IMU-Odometrie mit **automatischem Schleifenschluss** und globaler Pose-Graph-Optimierung.
   - **Unsicherheit sichtbar machen:** Laufbahn farbig nach Güte (z. B. Registrierungsfehler, Überlappung,
     Degeneration in langen Fluren).
   - **Manuelle Korrektur:** zwei Stellen als „gleich“ markieren → Schleifenschluss → Neuausrichtung;
     Abschnitte der Laufbahn verwerfen/neu rechnen; Vorher/Nachher-Vergleich.
3. **Qualität und Optik:**
   - Punktwolke: Bewegungsentzerrung, Entfernen dynamischer Objekte (Personen/Geister), Ausreißer-/Rauschfilter,
     Ausdünnen mit Kantenerhalt, doppelte Wände durch Drift verschwinden durch Ziel 2.
   - Farbe: Fisheye-Projektion mit Verdeckungstest, Belichtungs- und Weißabgleich zwischen den Kameras und über
     die Zeit, unscharfe Bilder aussortieren, je Punkt die bestgeeignete Ansicht wählen.
4. **Export:** LAS/LAZ, E57, PLY, PCD, Laufbahn (TUM), damit CloudCompare & Co. alles öffnen.
5. **Programm mit Oberfläche:** Projekte/Aufnahmen links, 3D-Ansicht Mitte (flüssig bis ~50 Mio. Punkte),
   Verarbeitungsschritte und Eigenschaften rechts. Jeder Schritt einzeln startbar, wiederholbar, vergleichbar.
   **Nichts wird überschrieben**, jede Korrektur ist ein rücknehmbarer Schritt.
6. **Open Source** von Tag 1: öffentliches Repo, Builds für macOS (arm64) und Windows (x64) automatisch.

## 3. Nicht-Ziele (vorerst)

- Live-Aufnahme / Steuerung des Scanners, Download per SFTP vom Gerät (später möglich).
- Mesh + Textur, Panorama, Gaussian Splatting, RTK/GNSS, Passpunkte – kommen **nach** dem Kern (Ausbau, Abschnitt 8).
- Das RayStudio-Format `.amap/.amx` lesen oder schreiben.
- Andere Scanner als JMK6 aktiv unterstützen (Struktur offen halten: Livox/RoboSense-Topics existieren bei
  anderen 3DMakerPro-Geräten).
- Cloud-Dienst, Konto, Telemetrie.

## 4. Anti-Ziele (wäre ein Fehlschlag, selbst wenn es „funktioniert“)

- Nur unter Windows/NVIDIA brauchbar – der Mac ist Hauptplattform.
- Ergebnisse, die schlechter aussehen oder mehr driften als RayStudio.
- Korrekturen, die Daten unwiderruflich verändern.
- Code, Dateien, Texte oder Modelle aus RayStudio im Repo; Scan-Daten von Yannick im Repo.
- GPL/AGPL/nicht-kommerzielle Bausteine im Programm, die die Lizenz des Projekts kippen.
- Ein Riesenpaket wie RayStudio: Ziel ist ein schlanker Installer.

## 5. Rahmenbedingungen

- **Recht:** Interoperabilität nach § 69e UrhG / EU 2009/24/EG – wir lesen offene Rohdaten (Bags, JSON) und
  beschreiben Formate; kein Dekompilieren von RayStudio-Code für die Umsetzung, keine Übernahme von Algorithmen
  daraus.
- **Lizenz des Projekts:** Apache-2.0 OR MIT (Doppellizenz). Folge: keine GPL/AGPL-Bausteine einbinden
  (FAST-LIO/FAST-LIVO, CGAL, OpenSplat, gnss-rtk) und keine Inria-3DGS-Abkömmlinge.
- **Plattformen:** macOS 15+ auf Apple Silicon (Entwicklung auf MacBook Pro M4 Pro, 48 GB) und Windows 11 x64
  (Test auf Windows-PC mit RTX 5080). Linux darf nebenbei laufen, wird aber nicht aktiv gepflegt.
- **GPU:** Rechenintensive Schritte und 3D-Ansicht nutzen die GPU auf Mac (Metal) und Windows (RTX, DX12/Vulkan)
  über wgpu; CUDA höchstens optional. Zuverlässigkeit vor Geschwindigkeit. (Vorgabe Yannick, 20.10.2026)
- **Datenmengen:** Testscan 266 s ≈ 2.663 LiDAR-Frames, 53.000 IMU-Werte, 1.741 Bilder à 12 MP, 12,5 Mio. Punkte
  im Ergebnis; Bags 3,2 GB. Ganze Aufnahmen dürfen nicht komplett in den RAM müssen.
- **Testdaten:** liegen nur lokal (nicht im Repo). Referenzergebnisse von RayStudio aus dem Testprojekt
  (`trj.txt`, `smooth.las`, `color_normals_result.ply`) dienen als Vergleich.

## 6. Entschiedene Teile des Wie

| Thema | Entscheidung | Wer |
|---|---|---|
| Plattform | macOS + Windows, Mac zuerst | Yannick |
| Form | Desktop-Programm mit Oberfläche, Korrekturen in der 3D-Ansicht | Yannick |
| Vorbild | CloudCompare, moderner | Yannick |
| Open Source | sofort öffentlich, GitHub-Konto `ROCCAT-XOX` | Yannick |
| Sprache | Rust für Kern und Oberfläche | Claude (empfohlen, von Yannick gewünscht „Rust oder C“) |
| Lizenz | Apache-2.0 OR MIT | Claude |
| Grafik | wgpu (Metal auf Mac, DX12/Vulkan unter Windows); CUDA höchstens optional | Claude |
| Oberfläche | egui (eframe) + eigener wgpu-Renderer für Punktwolken | Claude |
| SLAM-Basis | eigener LiDAR-IMU-Kern in Rust **oder** GLIM (MIT) als externer Prozess – Entscheidung nach Spike (Abschnitt 7) | Claude |
| Externe Werkzeuge | nur permissive (MIT/BSD/Apache) als eigener Prozess, z. B. GLIM, COLMAP, PoissonRecon | Claude |

### Ergänzende Festlegungen (Claude, nach Prüfung)

- **Beide Plattformen nativ, auch SLAM.** Kein WSL, keine zwei getrennten SLAM-Wege. Der SLAM-Spike gilt nur als
  bestanden, wenn der Weg auf macOS **und** Windows ohne ROS läuft; sonst eigener Rust-Kern.
- **Messlatte für Drift** (nicht RayStudio als Wahrheit): Rundgang mit identischem Start-/Endpunkt
  (Abstand Start↔Ende nach Verarbeitung), einige von Hand gemessene Strecken (Maßband/Laser) im Scan,
  Wanddicke/Doppelwände in der Wolke. RayStudio-Ergebnis desselben Scans wird mit derselben Messlatte gemessen.
- **Antwortzeit Korrekturen:** manueller Schleifenschluss aktualisiert die Laufbahn **in Sekunden** (Pose-Graph
  über gespeicherte Teilkarten, kein kompletter SLAM-Neulauf). Volle Verarbeitung eines 5-Minuten-Scans darf
  auf dem M4 Pro einige Minuten dauern (Ziel: < 10 min ohne Einfärben, < 20 min mit).
- **Externe Werkzeuge** werden mit dem Installer ausgeliefert; fehlt eines, wird nur die betroffene Funktion
  abgeschaltet. Ziel-Größe Installer < 500 MB (Kern ohne Ausbau-Werkzeuge).
- **Rohdaten-Weg:** Kopie von der MicroSD bzw. dem Projektordner (Bags + `calib.json`). Leser muss sowohl
  RayStudio-Projektordner als auch MicroSD-Rohdaten erkennen, sobald Beispiel vorliegt.
- **Version 1 fertig** = Abschnitt 8, Punkte 1–5 (Rohdaten → driftkorrigierte, eingefärbte, bereinigte Wolke →
  Export, mit Oberfläche und Installern). Zwischenstände werden ab Punkt 2 als Vorabversion veröffentlicht.

## 7. Dem Entwickler überlassen

- Projektstruktur (Cargo-Workspace mit z. B. `core`, `io`, `slam`, `color`, `export`, `app`).
- Projektformat auf der Platte (Vorschlag: Ordner mit `project.toml` + nachvollziehbare Schritt-Ergebnisse,
  jede Korrektur als eigener Eintrag).
- **SLAM-Spike zuerst:** GLIM mit den Bags (Vanjee, PointCloud2 mit Zeitstempel je Punkt, IMU ohne Orientierung)
  auf dem Mac zum Laufen bringen und gegen RayStudio `trj.txt` vergleichen. Gelingt das nicht sauber oder ist
  GLIM ohne ROS auf macOS zu sperrig → eigener Rust-Kern (Entzerrung, iterativer Kalman-Filter, Voxel-Karte,
  Pose-Graph mit Schleifenschluss).
- Klären der Kalibrier-Konvention (`transform_matrix` L→C oder C→L, Bedeutung von `extrinsics`) per Test-Projektion.
- Wahl der Crates (z. B. `rosbag`, `lz4`, `nalgebra`, `kiddo`, `las`/`laz-rs`, `e57`, `rerun` für Fehlersuche).
- Darstellung großer Wolken (Level-of-Detail/Octree).

## 8. Reihenfolge (grob)

1. Bags + Kalibrierung lesen, Rohdaten anzeigen (Fehlersuche-Viewer).
2. SLAM-Spike → Laufbahn + Punktwolke, Vergleich mit RayStudio.
3. Drift-Anzeige, Schleifenschluss automatisch + manuell.
4. Einfärben mit Belichtungsabgleich; Wolken-Bereinigung.
5. Export; Programm-Oberfläche vollständig; Installer macOS/Windows.
6. Ausbau: Mesh + Textur, Gaussian Splatting (Brush, Apache-2), RTK/GNSS + Passpunkte, Mehrfach-Scans,
   Download vom Gerät.

## 9. Offen

- Gerätename zu `JMK6` (Raven oder Hawk).
- Weitere Testdaten von Yannick: Aufnahme mit deutlich sichtbarer Drift (z. B. langer Flur, Treppenhaus,
  Rundgang), Aufnahme mit RTK, Rohdaten direkt von der MicroSD.
