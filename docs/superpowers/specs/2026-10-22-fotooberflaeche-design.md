# Baustein 2: Fotoähnliche Oberfläche (texturiertes Modell) – Design

Stand 22.10.2026 · Autor: Yannick Stumpf mit Claude · Reihenfolge laut Yannick: **2 zuerst**, dann 4
(Oberfläche optisch), 1 (Zusammenführen, Design `2026-10-22-zusammenfuehren-design.md`), 3 (Export/
Installer, M6), 5 (Planen).

## 1. Worum es geht und warum

Driftless liefert heute eine saubere, gefärbte **Punktwolke**. Aus der Nähe wirkt sie wie Sand, vor
allem in dunklen Räumen; Steckdosen, Schalter oder Schilder erkennt man kaum. Yannick will eine
**fotoähnliche Oberfläche wie bei Matterport**: eine geschlossene Fläche, auf der die Kamerabilder
liegen.

Sie ist die Grundlage für alles Spätere: Steckdosen und Anschlüsse erkennen und markieren, Räume
bemaßen, Objekte (z. B. LED-Wände) hineinsetzen und in Blender rendern, Ausgabe für CAD (DXF/DWG) und
Modelle. Planer arbeiten in AutoCAD, Vectorworks, SketchUp oder Blender (welche zuerst, ist offen).

Genutzt wird sie **beides**: zuerst im Programm zum Hindurchgehen und Anschauen, zugleich als Datei für
Blender/CAD. Eine Ansicht für Kunden im Browser kommt später (Baustein 3).

Erfolg heißt:
- In den Scans vom 22.10. (schwarzer Raum, Testdaten s. u.) sind im Programm auf der Oberfläche
  Steckdosen, Schilder (Notausgang), Türen und Leuchten aus 1–2 m Abstand klar erkennbar, deutlich
  besser als in der Punktwolke.
- Dieselbe Oberfläche öffnet sich als `.glb` in Blender mit Texturen, in richtiger Größe (Meter) und
  aufrecht.
- Wände sind glatt und gerade, Ecken scharf; keine sichtbare Treppenstufen- oder Blasenoptik.

## 2. Ziele

1. **Geschlossene Fläche (Mesh) aus dem Scan:** aus den LiDAR-Messungen mit der korrigierten Laufbahn
   eines SLAM-Laufs. Kleine Löcher werden geschlossen; was nie gesehen wurde, bleibt offen (nichts
   erfinden). Ebene Flächen werden mit wenigen großen Dreiecken dargestellt, Details (Kanten, Möbel)
   mit feinen.
2. **Fotos auf die Fläche:** Jeder Flächenteil bekommt seine Farbe aus dem dafür besten Kamerabild
   (nah, möglichst senkrecht, scharf, nicht verdeckt, richtig belichtet). Benachbarte Teile sollen
   möglichst aus demselben Bild kommen (kein Flickenteppich). Helligkeitssprünge an Nähten werden
   ausgeglichen; vorhandene Bausteine (Belichtungsabgleich, Schärfe, Randabfall, Sichtbarkeit aus
   `driftless-map`) werden genutzt. Zielauflösung: so fein wie die Kameras es hergeben, etwa
   2–4 mm je Bildpunkt auf Flächen in 1–2 m Abstand.
3. **Neuer Schritt** „Oberfläche rechnen“ neben „Karte rechnen“ – im Programm und als
   `driftless surface <scan> --slam <lauf>` (Name dem Bauenden überlassen). Wie bei der Karte: nichts
   überschreiben, Zwischenergebnisse wiederverwenden, Abbrechen ohne halbe Dateien, Laufzeiten und
   Kennzahlen in einer JSON-Datei.
4. **Ausgabe:** `oberflaeche.glb` (glTF binär, Texturen eingebettet) und `oberflaeche.obj` + `.mtl` +
   Texturbilder. Meter, Z oben im Driftless-Koordinatensystem; glTF erwartet Y oben – beim Schreiben
   richtig umrechnen, damit es in Blender aufrecht steht.
5. **Im Programm ansehen:** Die 3D-Ansicht zeigt die texturierte Oberfläche, umschaltbar zwischen
   Oberfläche / Punktwolke / beides. Bewegen wie bisher (Umkreisen, Fliegen mit W/A/S/D); Höhenschnitt
   („Decke ausblenden“) gilt auch für die Oberfläche. Flüssig auf dem M4 Pro und auf Windows.
6. **Vorbereitet für später (nicht bauen, nur nicht verbauen):** Die Oberfläche gehört zu einem
   Lauf/einer Karte. Später (Baustein 1) soll derselbe Schritt ein zusammengeführtes Modell aus
   mehreren Scans texturieren – also die Eingabe als „Punkte + eine oder mehrere Laufbahnen +
   Bildquellen“ denken, nicht fest an einen Scan.

## 3. Nicht-Ziele

- Gaussian Splats (wie RayStudio): sehen noch fotografischer aus, taugen aber nicht zum Messen und
  nicht für CAD. Höchstens später als zusätzliche Ansicht.
- Browser-Ansicht für Kunden, E57/LAZ, Installer → Baustein 3.
- Messen, Markieren, DXF → Baustein 5. Optische Überarbeitung des Programms → Baustein 4.
- Mehrere Scans zusammen → Baustein 1 (siehe Ziel 6).
- Bewegte Dinge (Personen) aus den Bildern entfernen.

## 4. Was es nicht werden darf

- Eine „schöne“ Fläche, die von der Messung abweicht (geglättete Wände, die nicht mehr da liegen, wo
  gemessen). Die Geometrie bleibt messgenau wie die bereinigte Wolke (Abweichung im Millimeterbereich,
  in den Ergebnissen belegen).
- Erfundene Flächen in ungesehenen Bereichen.
- Fremder Code unter GPL/AGPL/Inria-Lizenz oder unklarer Lizenz (Projekt ist Apache-2.0 OR MIT);
  nichts aus RayStudio.
- Scan-Daten oder Kundennamen im Repo.

## 5. Rahmen

- macOS (Apple Silicon) und Windows; CI beide grün; Tests wie bisher (synthetische Szenen + Prüfung mit
  echten Daten, Ergebnisse unter `docs/ergebnisse/`).
- Ein Raum-Scan (≈ 5 min, ≈ 2.000 Bilder) in unter 10 min auf dem M4 Pro; Spitzen-RAM unter 4 GB
  (Ziel aus M4); `.glb` eines Raums unter etwa 300 MB.
- Mac-Platte ist fast voll: vor langen Läufen `df -h` prüfen, Zwischenergebnisse klein halten.

## 6. Entscheidungen von Claude (nicht von Yannick bestätigt)

- **Fläche aus den LiDAR-Strahlen (TSDF-Verfahren: Abstandsfeld im Voxelgitter, aus jedem Frame mit
  der korrigierten Pose eingetragen, dann Marching Cubes)** statt aus der Punktwolke allein: nutzt das
  Wissen „hier war freier Raum“, schließt Löcher ehrlicher und passt zur vorhandenen Strahlverfolgung
  (`dense`, `through`). Danach Vereinfachen ebener Flächen. Wenn sich das als ungeeignet erweist
  (z. B. dünne Objekte gehen verloren), darf der Bauende auf eine lizenzreine Poisson-Rekonstruktion
  ausweichen und begründet das in den Ergebnissen.
- Textur je Flächenteil aus **einem** Bild (Auswahl mit Glättung über Nachbarn), nicht als Mittel
  vieler Bilder (Mittelwerte machen unscharf).
- Programm zeigt zuerst die Oberfläche eines Laufs; Ablage in dessen Ordner, z. B.
  `slam-001/oberflaeche-001/` (genauer Ort dem Bauenden überlassen).

- **Grundlage ist eine fertige Karte** (`karte-NNN`, `clean.bin`), nicht nur der SLAM-Lauf: Die
  Fläche entsteht aus den **bereinigten** Punkten, je Punkt als Strahl von der Sensorposition seines
  Frames (`Cloud::frame` + `traj.tum`). So bleiben Geister und Durchblicke draußen und die geglätteten
  Wände drin. Messgenauigkeit wird gegen `clean.bin` belegt. Ablage: `karte-NNN/oberflaeche-NNN/`.
- **Zeit/Speicher (10 min, 4 GB) gelten nur für den neuen Schritt**; die Karte liegt schon vor. Bilder
  werden für die Textur in voller Auflösung gelesen (`downscale 1`), soweit das Budget reicht.
- **Glaswände bleiben offen** (nichts erfinden). Fläche von Hand einziehen kommt mit Baustein 5.
- **Zuerst nur für Blender prüfen** (Yannick hat das Zielprogramm noch nicht genannt). CAD-taugliche
  leichte Fassung (weniger Dreiecke, kleinere Texturen) kommt mit Baustein 3/5, falls nötig.
- **Sichtprüfung durch Yannick:** Nachdem Fläche + Textur auf dem schwarzen Raum laufen und bevor
  Anzeige im Programm und Export fertig gebaut werden, bekommt Yannick Bilder (Oberfläche gegen
  Punktwolke) zur Abnahme.
- **Veraltet statt automatisch neu:** Wird der Lauf (`slam --reuse`) oder die Karte neu gerechnet, wird
  eine vorhandene Oberfläche als veraltet markiert (wie `stale` bei Karten), nicht still neu gerechnet.
- Eingabe des Schritts als eigener Typ „Punkte mit Strahlursprung + Bildquellen mit Posen“, damit das
  spätere zusammengeführte Modell (Baustein 1) ihn ohne Umbau nutzen kann.

## 7. Dem Bauenden überlassen

- Voxelgröße der Fläche (Startwert 1–2 cm), Vereinfachungsgrad, Lochgröße zum Schließen.
- Verfahren für Bildauswahl und Nahtausgleich, Atlas-Packen der Texturen, Bildformat der Texturen
  (JPEG/PNG/KTX2), Kachelung/Detailstufen für die Anzeige.
- Bibliotheken (z. B. für glTF-Schreiben, Vereinfachen, Atlas), solange MIT/Apache/BSD-artig.
- Zerlegung in Aufgaben und Reihenfolge.

## 8. Testdaten

- Schwarzer Raum, drei Scans vom 22.10.: `~/Downloads/20261022132520`, `…133028`, `…133554`, je mit
  SLAM-Lauf und Karte in `<scan>/driftless/`. Schwieriger Fall: dunkle Wände, helle Leuchtstäbe. Für die
  Prüfung nach `driftless-testdata/` kopieren, Ordnername neutral.
- Büro-Scan `driftless-testdata/jmk6-2026-10-20/20261020173822` (heller, mehrere Räume, Glaswände).
- Vergleich: RayStudio-Splat-Export des Büro-Scans (liegt nur lokal, nicht im Repo) und die
  Punktwolken aus M4.
