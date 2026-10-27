# Baustein 3: Export, Formate öffnen, Installer

Stand: 2026-10-27. Yannick: „einfach ausführen“. Es gab also keine Rückfragen. Die Entscheidungen unten sind
eigene Entscheidungen auf Grundlage des Grundentwurfs
(`2026-10-20-driftless-design.md`, Ziel 4 und 6, Anti-Ziele) und des Entwurfs zu B1, der Export und weitere
Formate auf B3 verschiebt.

## 1. Wozu

Ergebnisse aus Driftless müssen in den Programmen ankommen, mit denen danach gearbeitet wird:
- CloudCompare, Autodesk ReCap/Revit, Blender, Planungs- und CAD-Werkzeuge.

Umgekehrt muss Driftless Punktwolken aus anderen Quellen öffnen, um sie anzusehen oder zu vergleichen.

Der Installer soll Driftless ohne Rust und ohne Kommandozeile auf einen fremden Mac oder Windows-Rechner bringen.

## 2. Ziele

1. **Export Punktwolke:** LAS 1.4, LAZ, E57, PLY (binär) und PCD (binär) aus folgenden Quellen:
   - einer Karte (`color.las`, ersatzweise `clean.las`)
   - einem Modell (`modell.las`)
   - einer beliebigen geöffneten Punktwolke

   Übernommen werden Farbe, Intensität, soweit vorhanden, und beim Modell die Scan-Nummer:
   - LAS/LAZ: `point_source_id`
   - PLY: Eigenschaft `scan`
   - E57: ein E57-„scan“ je Mitglied mit Namen und Sensorlage
   - PCD: Feld `scan`
2. **Export Oberfläche:** glb (Y oben, wie heute) und OBJ mit `.mtl` und Seitenbildern (Z oben) in einen
   gewählten Ordner. Zusätzlich gibt es **PLY-Netz** mit Farbe je Ecke, gemittelt aus der Textur, für
   Werkzeuge ohne Texturen.
3. **Export Laufbahn:** TUM (wie heute) und CSV (`zeit;x;y;z;qx;qy;qz;qw`, Kopfzeile, Punkt als
   Dezimaltrenner).
4. **Öffnen:** Die App öffnet außer LAS und PLY auch:
   - **LAZ**
   - **E57**: alle Scans zusammen, mit Pose angewendet
   - **PCD**: ascii und binary
   - **glb und OBJ**, eigene und einfache fremde Netze

   Der Weg ist derselbe wie heute: ☰ „Punktwolke öffnen …“ (wird zu „Datei öffnen …“) und Ziehen und Ablegen.
   Was ein Format nicht hergibt, meldet die App deutsch und klar:
   - PCD `binary_compressed`
   - E57 ohne kartesische Punkte
   - glTF mit mehreren Szenen oder Draco

   Es gibt keinen stillen Unsinn.
5. **CLI:** `driftless export <quelle> -o <datei> [--format …] [--ueberschreiben]`.
   - Die Quelle ist ein Kartenordner, ein Modellordner, ein Oberflächenordner, ein SLAM-Lauf (dann die
     Laufbahn) oder eine Punktwolken-Datei.
   - Das Format wird aus der Endung von `-o` abgeleitet, `--format` sticht.
6. **App:**
   - „Exportieren …“ an Karte, Modell, Oberfläche und Lauf im Baum.
   - Im ☰-Menü für das, was gerade angezeigt wird.
   - Ein kleiner Dialog: Was, Format, Ziel über den Dateidialog von `rfd`.
   - Er läuft als Auftrag mit Fortschritt und Abbrechen, wie die übrigen Schritte.
7. **Installer:**
   - **macOS:** `.dmg` mit `Driftless.app` (arm64, wie im Grundentwurf), Symbol und `Info.plist`.
     `driftless` (CLI) liegt in der App unter `Contents/MacOS/` und ist im Bildschirm „Über Driftless“ genannt.
   - **Windows:** Einrichtungsprogramm `Driftless-<version>-setup.exe` (x64). Es legt einen Eintrag im
     Startmenü an, hat einen Deinstaller und enthält `driftless-app.exe` und `driftless.exe`.
   - Beide unter 500 MB. Gebaut in der GitHub-CI, bei jedem PR als Artefakt.
   - Bei einem Tag `v*` wird ein **Entwurf** einer GitHub-Release mit beiden Dateien angelegt. Veröffentlicht
     wird er nur von Hand, nach Yannicks Okay.
8. **Version und Symbol:** ein Programmsymbol (selbst gezeichnet, schlicht) für Fenster, Dock/Taskleiste und
   Installer. „Über Driftless“ zeigt Version, Commit und Lizenz.

## 3. Nicht-Ziele

- **Signieren und Notarisieren:**
  - macOS: Apple-Entwicklerkonto
  - Windows: Code-Signing-Zertifikat

  Beides braucht Zugangsdaten, die es noch nicht gibt. Die README sagt, wie man die Warnung beim ersten Start
  bestätigt: macOS nach dem ersten, blockierten Start Systemeinstellungen → „Datenschutz & Sicherheit“ →
  „Dennoch öffnen“ (oder `xattr -dr com.apple.quarantine /Applications/Driftless.app`), Windows
  „Weitere Informationen → Trotzdem ausführen“.
- Intel-Macs, Linux-Pakete, automatische Updates.
- Dateizuordnungen im System (Doppelklick auf `.las` öffnet Driftless). Sie würden CloudCompare und andere
  verdrängen.
- DWG/DXF, IFC, Revit-Formate: gehören zu Baustein 5.
- COPC, LAS-Wellenformen, Bilder in E57 (`images2D`), Gaussian Splats.
- Komprimiertes PCD schreiben.

## 4. Was es nicht werden darf

- **Eine Exportdatei, die anders aussieht als in Driftless:**
  - vertauschte Achsen
  - verschobener Ursprung
  - verlorene Farben
  - Farben in 8 statt 16 Bit falsch skaliert

  Jeder Exporter wird mit einem **unabhängigen Leser** gegengeprüft (siehe 6).
- Halbe Dateien nach Abbruch oder Fehler. Geschrieben wird in eine temporäre Datei daneben, dann umbenannt.
- Stilles Überschreiben. Die App fragt nach, die CLI braucht `--ueberschreiben`.
- Ein Installer, der Scan-Daten, Kundennamen oder Zugangsdaten enthält.
- GPL/AGPL-Abhängigkeiten. `cargo deny` bleibt grün.
  - `e57`: MIT.
  - `laz`: Apache-2.0, über das Merkmal `laz` von `las`.

## 5. Rahmen

- Rust-Workspace, egui 0.36 / wgpu wie bisher.
- Neuer Crate **`driftless-export`** mit Schreibern und Lesern für Punkte, Netz und Laufbahn. Die App und die
  CLI nutzen ihn über `driftless-steps`, nach dem Muster der übrigen Schritte.
- Vorhandene Schreiber ziehen dorthin um oder werden von dort aus aufgerufen; es gibt keine zweite Kopie:
  - `write_las`, `write_ply` in `driftless-map/src/export.rs`
  - `write_glb`, `write_obj`
  - `write_tum`
- Große Wolken bis etwa 50 Mio. Punkte, wie im Grundentwurf: Schreiben und Lesen in Blöcken, Speicher höchstens
  etwa doppelt so groß wie die Wolke im Speicher.
- Koordinaten: Meter, Rahmen der Karte bzw. des Modells, Z oben. Nur glTF hat Y oben (Norm).
  - LAS/LAZ: Maßstab 0,1 mm, Versatz auf die gerundete Mitte der Wolke.
  - E57: doppelte Genauigkeit.
- Schwere Läufe auf VM 101, nicht auf dem Mac (6 Threads, nice 10, ein schwerer Job zur Zeit).

## 6. Wie geprüft wird

- **Rundreise je Format:** schreiben, mit dem eigenen Leser lesen. Lage gleich bis auf die Genauigkeit des
  Formats, Farbe und Intensität gleich, Scan-Nummer gleich.
- **Unabhängige Leser auf VM 101:** Python im Benutzerkonto, ohne sudo:
  - `laspy` mit `lazrs` für LAS/LAZ
  - `pye57` für E57
  - `open3d` oder ein kleiner eigener Leser für PCD/PLY
  - `trimesh` für glb/OBJ/PLY-Netz

  Gleiche Zahl Punkte, gleiche Grenzen (± Genauigkeit), Stichproben gleicher Farbe. Dazu kommt das Öffnen
  fremder Beispieldateien, die diese Bibliotheken selbst schreiben.
- Echte Daten: Modell der drei Scans aus dem schwarzen Raum und Büro-Scan B als LAS/LAZ/E57/PLY/PCD.
  Gemessen werden Größe, Zeit und Spitzen-Speicher.
- **Installer:** Die CI baut beide. Ein Prüfschritt öffnet das `.dmg` bzw. installiert still in einen
  Testordner, startet `driftless --version` aus dem Paket und prüft die Größe.
- Ergebnisbericht `docs/ergebnisse/2026-10-2x-baustein-3.md` und README-Abschnitte.

## 7. Dem Bauenden überlassen

- Der Aufbau innerhalb von `driftless-export`, Blockgrößen, Fortschrittsschritte.
- Das Werkzeug für Windows: Inno Setup oder WiX. Gewählt wird, was auf `windows-latest` ohne Zugangsdaten
  verlässlich läuft. Für macOS `hdiutil` oder `create-dmg`.
- Die Gestaltung des Symbols, schlicht in den Farben der App.
- Die genaue Platzierung von „Exportieren …“ im Baum, nach dem Stil von B4.
