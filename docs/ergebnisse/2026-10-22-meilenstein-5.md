# Ergebnisse Meilenstein 5 – Oberfläche

Stand: 2026-10-22 · gemessen auf dem MacBook Pro M4 Pro (14 Kerne, 48 GB, Metal), Release-Build

Meilenstein 5 bringt das Programm `driftless-app`: Projekte mit Scans, Läufen und Karten, die
Schritte des CLI (Prüfen, SLAM, Neu rechnen, Karte), eine 3D-Ansicht mit Detailstufen und Höhenschnitt,
die Laufbahn nach Güte mit Schleifen, Schleifen per Klick, Vorher/Nachher und Verlauf. SLAM und Karte
sind gegenüber Meilenstein 4 unverändert (die Büro-Laufbahn ist bitgleich mit der aus Meilenstein 4).

## Kurzfassung

- **Laden der Büro-Wolke** (`color.las`, 14,96 Mio. Punkte): **0,95 s** lesen, **0,20 s**
  Detailstufen. Der Prozess braucht danach 0,93 GB. Der Aufbau der Detailstufen belegt höchstens
  40 B je Punkt (Schranke 64 B).
- **50 Mio. Punkte (synthetisch):** Detailstufen in 0,55 s, das Programm braucht 3,1 GB. Beim
  Umkreisen braucht ein Bild auf dem M4 Pro im Median **6–11 ms GPU-Zeit** (90–160 Bilder/s), bei
  Budget 6 Mio. wie bei 12 Mio. Das Standardbudget bleibt **6 Mio.**
- **Neu rechnen nach einer Schleife** (Büro, wie im Programm): **5,9 s** vom Eintragen der Schleife bis
  zur neuen Laufbahn (Schranke 15 s).
- **Programm = CLI:** SLAM und Karte aus dem Programm schreiben auf `Test` dieselben 13 Dateien wie
  `driftless slam` und `driftless map`, Byte für Byte (`map.json` ohne Laufzeiten).
- **RTX 5080 (Windows): offen.** Das Programm für Windows baut die CI bei jedem Pull Request als
  Artefakt `driftless-app-windows`; die Bildzeit misst ein Handtest (siehe unten). Ob die GPU-Zeit
  unter DX12/Vulkan gemessen wird, ist ungeprüft.
- **Güte der Teilkarten zu streng skaliert:** Im Büro liegt sie im Median bei 0,21 (höchstens 0,64),
  auf `Test` bei 0,16, obwohl SLAM den Drift im Büro von 2,16 auf 0,13 m senkt. Fast immer begrenzt die
  Beschränktheit (`degeneracy`) des schlechtesten Frames. Vorschlag für später unten; in M5 nicht
  geändert, weil die Güte auch die Kanten des Posengraphen gewichtet.
- **Ablauf von Hand auf beiden Plattformen: offen.** Abgedeckt sind die Schritte durch die
  Oberflächen-Tests und die Bilder des Bildschirmfoto-Tests; der Handtest mit Maus (Retina,
  Windows 150 %) steht aus.

## Vorbereitung (Release-CLI)

```sh
driftless slam <Büro> -o $DRIFTLESS_TESTDATA/ergebnisse-m5/buero
driftless map <Büro> --slam $DRIFTLESS_TESTDATA/ergebnisse-m5/buero
cp …/buero/color.las …/buero/karte/          # die Wolke, die die Tests laden
driftless slam $DRIFTLESS_TESTDATA/Test -o $DRIFTLESS_TESTDATA/ergebnisse-m5/test
driftless map $DRIFTLESS_TESTDATA/Test --slam $DRIFTLESS_TESTDATA/ergebnisse-m5/test
```

Spitzen-RAM mit `/usr/bin/time -l` („maximum resident set size“), je ein Lauf ohne andere Last.

| Aufnahme | Schritt | Laufzeit | Spitzen-RAM | Ergebnis |
|---|---|---|---|---|
| Büro (582 s) | `slam` | 25,0 s | 0,17 GB | 59 Teilkarten, 20 Schleifen akzeptiert, Start↔Ende 2,162 → 0,129 m; alle fünf Dateien bitgleich mit Meilenstein 4 |
| Büro | `map` (alle Schritte in einem Prozess) | 411,4 s | 4,09 GB | dicht 15,60 Mio., bereinigt und eingefärbt 14,96 Mio. Punkte, 97,6 % eingefärbt |
| `Test` (266 s) | `slam` | 9,1 s | 0,11 GB | Start↔Ende 5,41 m (kein Rundgang) |
| `Test` | `map` | 179,7 s | 2,04 GB | 6,94 Mio. Punkte, 98,5 % eingefärbt |

Die Büro-Karte in einem Prozess braucht 4,09 GB; in getrennten Prozessen je Schritt waren es in
Meilenstein 4 höchstens 3,49 GB (der CLI-Test prüft weiter die getrennten Schritte).

## Laden und Detailstufen (Büro)

`buero_scan_lod_build` und `buero_scan_octree_live_heap`: `color.las` mit der Güte aus `slam.json`
laden wie das Programm (`load_cloud`), dann die Detailstufen.

| | gemessen | Schranke |
|---|---|---|
| Punkte | 14.961.782 | |
| Lesen | 0,95 s | |
| Detailstufen | 0,20 s | ≤ 10 s |
| Speicher beim Aufbau (Zählender Allokator, nur der Aufbau) | 599 MB = 40,0 B/Punkt | ≤ 64 B/Punkt + 32 MiB |
| Spitzen-RSS des Testprozesses nach dem Laden | 934 MiB | |
| … nachdem dieselbe Wolke ein zweites Mal geladen wurde (beide im Speicher) | 1.648 MiB (+714 MiB) | |
| Knoten / Tiefe / größter Knoten / GPU-Blöcke | 1.521 / 6 / 38.516 Punkte / 4 | |

- Die Schranken haben viel Spielraum. 10 s für den Aufbau sind die Grenze, ab der das Warten beim
  Öffnen stört (gemessen 50-mal schneller). 64 B je Punkt sind 1,6-mal der Messwert: 16 B je Schlüssel
  (Morton-Code und Index), 20 B je GPU-Punkt, 4 B Rückverweis auf die Punktliste.
- Das Programm zeigt im Bildschirmfoto-Test dieselben Zeiten in der Statuszeile („Laden 0,9 s,
  Detailstufen 0,2 s“).
- Der Plan nannte ≈ 15,6 Mio. Punkte; das ist die dichte Wolke. Geladen wird die eingefärbte, die aus
  der bereinigten entsteht (14,96 Mio.).
- **Speicher beim Wechseln der Wolke:** Bis die neue Wolke fertig ist, bleibt die alte im Speicher.
  Bei der Büro-Wolke kostet das 0,7 GB zusätzlich, bei 50 Mio. Punkten (Programm 3,1 GB) geschätzt
  das Doppelte, also gut 6 GB (nicht gemessen). Bei 48 GB unkritisch; auf Rechnern mit 8 GB sollte
  das Programm die alte Wolke vorher freigeben (offen).

### Ausreißer und der Würfel der Wurzel

Die Wurzel des Octrees ist der kleinste Würfel um alle Punkte. Im Büro ist er 42,1 m groß, die Wolke
ohne die äußersten 0,5 % je Seite und Achse misst 18,6 × 27,0 × 4,5 m. 43.960 Punkte (0,29 %) liegen
mehr als 1 m außerhalb dieses Kastens (Punkte durch Fenster, die die Bereinigung stehen lässt). Der
Würfel ist dadurch 1,6-mal so groß wie nötig; das kostet höchstens eine Detailstufe mehr. Die Tiefe
ist 6, die Zahl der Knoten klein (1.521), der Aufbau schnell. **Kein Handlungsbedarf.** „Alles zeigen“
rahmt ohnehin ohne die äußersten 0,5 % ein.

## 50 Mio. Punkte (synthetisch)

`synthetic_50m_build` und `synthetic_50m_frame_time`: eine synthetische Wolke aus 50 Mio. Punkten,
Ansicht 1000 × 800 Punkte bei 2 Pixeln je Punkt (Retina), 60 Bilder je Messung beim Umkreisen
(6° je Bild), im Takt von 60 Hz. GPU-Zeit aus Zeitstempel-Abfragen.

| | gemessen |
|---|---|
| Erzeugen / Detailstufen | 0,89 s / 0,55 s |
| Spitzen-RSS des Tests | 2.968 MiB |
| RSS des Programms (`driftless-app --synthetic 50000000`, nach dem Hochladen) | 3,1 GB |
| Hochladen auf die GPU | 15 Bilder, längstes 26 ms |

Bildzeit (GPU) beim Umkreisen:

| Plattform / GPU | Budget | Abstand | gezeichnet | Median | 90 % | Max. |
|---|---|---|---|---|---|---|
| M4 Pro (Metal) | 6 Mio. | ganze Wolke | 0,79 Mio. | 6,2 ms | 6,6 ms | 24,7 ms (erstes Bild) |
| M4 Pro | 6 Mio. | × 0,3 | 5,26 Mio. | 10,0 ms | 10,3 ms | 41,4 ms |
| M4 Pro | 6 Mio. | × 0,1 | 5,98 Mio. | 10,7 ms | 10,7 ms | 11,2 ms |
| M4 Pro | 12 Mio. | ganze Wolke | 0,74 Mio. | 6,6 ms | 6,6 ms | 10,7 ms |
| M4 Pro | 12 Mio. | × 0,3 | 5,67 Mio. | 10,0 ms | 10,3 ms | 32,6 ms |
| M4 Pro | 12 Mio. | × 0,1 | 5,89 Mio. | 11,0 ms | 11,3 ms | 12,1 ms |
| RTX 5080 (Windows, DX12/Vulkan) | 6 Mio. / 12 Mio. | | | offen (Handtest) | | |

- **≥ 30 Bilder/s erreicht** (Median 6–11 ms, auch das 90-%-Bild unter 12 ms). Die Spitzen um
  30–40 ms sind einzelne Bilder nach dem Wechsel des Abstands (neue Knoten). Das Standardbudget bleibt
  **6 Mio.**
- Bei 12 Mio. werden kaum mehr Punkte gezeichnet, weil die Verfeinerung aufhört, sobald der
  Punktabstand auf dem Schirm unter der Punktgröße liegt. 12 Mio. kosten also fast nichts zusätzlich.
- Im Bericht zu Task 10 standen ≈ 16 ms je Bild bei derselben Messung; dazwischen wartet die Ansicht
  nicht mehr blockierend auf die GPU (Zeitstempel werden abgefragt, ohne zu warten).
- Die UI-Schritte (egui + Auswahl der Knoten, CPU) brauchen im Median 0,8–1,6 ms.

**Handtest RTX 5080** (offen): das Programm aus dem Artefakt `driftless-app-windows` eines Pull Requests
auf dem Windows-PC starten, ein Projekt mit dem Büro-Scan anlegen, den Lauf `ergebnisse-m5/buero`
hinzufügen und `karte/color.las` (oder `color.las`) laden. Mit Punktbudget 6 Mio. und 12 Mio. umkreisen,
einmal in der Gesamtansicht, einmal nah heran, und die Zeit oben in der Ansicht ablesen („gezeichnet …
· x ms GPU“). Steht dort „ms je Bild“ statt „ms GPU“, liefert der Adapter unter DX12/Vulkan keine
Zeitstempel; dann ist es die Zeit zwischen zwei Bildern. Für 50 Mio. Punkte: `driftless-app.exe
--synthetic 50000000`. Werte in die Tabelle oben eintragen.

## Neu rechnen nach einer Schleife

`buero_scan_reuse_in_seconds`: die SLAM-Dateien des Büro-Laufs in einen leeren Ordner kopieren, als
Lauf in ein Projekt aufnehmen, eine Schleife zwischen Start und der nächstgelegenen Teilkarte am Ende
eintragen (5,0 s ↔ 580,7 s, Teilkarten 0 ↔ 58) und neu rechnen wie das Programm (`run_slam_job` mit
`Reuse`, also mit Verlauf).

| | gemessen | Schranke |
|---|---|---|
| Schleife eintragen bis neue Laufbahn | 5,91 s | ≤ 15 s |
| davon `run_slam` (Zwischenspeicher laden, Graph lösen, schreiben) | 5,91 s | |
| Schleife | akzeptiert | |
| Stände danach | „übernommen“, „neu gerechnet“ | |

Im Programm kommt danach noch das Lesen des Laufs für die Ansicht hinzu („Lauf lesen“, nicht
gesondert gemessen). Mit Maus gestoppt ist die Zeit vom Klick bis zur Laufbahn noch nicht
(Handtest offen).

## Programm gegen CLI

`app_steps_match_cli`: `Test` als neues Projekt, „SLAM rechnen“ und „Karte rechnen“ (Alles,
Standardwerte) genau wie das Programm (`run_slam_job`, `run_map_job`, Karte in `karte-001/`), dann
jede Datei mit der Ausgabe des Release-CLI unter `ergebnisse-m5/test` verglichen.

| Schritt | Laufzeit | Dateien | Ergebnis |
|---|---|---|---|
| SLAM | 8,8 s | `traj.tum`, `traj_odom.tum`, `map.las`, `slam.json`, `submaps.bin` | alle bytegleich |
| Karte | 178,5 s | `dense.las`, `dense.bin`, `clean.las`, `removed.las`, `clean.bin`, `color.las`, `color.ply`, `map.json` | alle bytegleich, `map.json` ohne `seconds_total`, `peak_rss_bytes` und die Laufzeiten je Schritt |

Dazu schreibt das Programm `lauf.toml` und `verlauf/` (nicht im CLI). Die Binärdatei `driftless`
selbst vergleicht der Test nicht (sie ist aus Tests eines anderen Crates nicht zuverlässig
erreichbar); dass sie dieselben Dateien wie vor Meilenstein 5 schreibt, war in Task 2 mit `cmp`
geprüft.

## Güte der Teilkarten

Die Güte einer Teilkarte ist die **schlechteste** Güte ihrer Frames (≈ 100 Frames je Teilkarte). Die
Güte eines Frames ist das Produkt aus drei Faktoren, je auf 0…1 begrenzt
(`driftless-odom/src/quality.rs`):

- Überlappung: `inlier_ratio / 0,6`
- Beschränktheit: `degeneracy / 0,15` (`DEGENERACY_GOOD`, abgeleitet aus dem synthetischen Raum, in
  dem Wände, Boden und Decke in alle Richtungen sichtbar sind: 0,18–0,20 beim Gehen)
- Rest: `1 − residual_rms / 0,1 m`

Ausgewertet aus `submaps.bin` (Wegwerf-Skript, nicht im Repo):

| | Büro | `Test` |
|---|---|---|
| Teilkarten / Frames | 59 / 5.819 | 28 / 2.663 |
| `degeneracy` der Frames: 10 % / Median / 90 % | 0,028 / 0,100 / 0,183 | 0,036 / 0,092 / 0,190 |
| `inlier_ratio`: 10 % / Median / 90 % | 0,36 / 0,54 / 0,67 | 0,48 / 0,57 / 0,66 |
| `residual_rms`: Median | 12 mm | 11 mm |
| Faktor Beschränktheit / Überlappung / Rest, Median | 0,67 / 0,91 / 0,88 | 0,61 / 0,95 / 0,89 |
| Güte der Frames, Median | 0,47 | 0,50 |
| **Güte der Teilkarten (jetzt, schlechtester Frame): Median / Max.** | **0,21 / 0,64** | **0,16 / 0,70** |
| begrenzender Faktor im schlechtesten Frame | Beschränktheit 54×, Überlappung 5× | Beschränktheit 26×, Überlappung 1×, Rest 1× |
| Variante: 10-%-Frame statt schlechtestem | 0,31 / 0,78 | 0,24 / 0,85 |
| Variante: 10-%-Frame und `DEGENERACY_GOOD` 0,08 | 0,47 / 0,90 | 0,44 / 0,85 |
| Anteil Teilkarten < 0,3: jetzt → letzte Variante | 63 % → 37 % | 71 % → 29 % |

**Befund:** Die Skala ist für echte Aufnahmen zu streng, nicht die Laufbahn schlecht. Auf `Test` weicht
die Laufbahn nur 1,8 cm (ATE) von RayStudio ab (Meilenstein 2), und die Güte ist genauso niedrig.
Drei Gründe, nach Gewicht:

1. **Beschränktheit:** In echten Räumen und Fluren liegt `degeneracy` im Median bei 0,09–0,10, also
   unter dem „guten“ Wert 0,15 aus dem synthetischen Raum. Sie begrenzt in 54 von 59 Teilkarten.
2. **Schlechtester Frame:** Ein einziger schwacher Frame (z. B. in einer Tür oder beim Drehen)
   bestimmt die ganze Teilkarte. Der Median der Frames liegt bei 0,47, die Teilkarte bei 0,21.
3. **Rest:** `1 − rms / 0,1 m` gibt bei den üblichen 11–12 mm nur 0,88; mehr als ≈ 0,9 ist praktisch
   nicht erreichbar.

**Vorschlag (nicht umgesetzt):** `DEGENERACY_GOOD` an echten Aufnahmen neu festlegen (≈ 0,08) und je
Teilkarte den 10-%-Frame statt des schlechtesten nehmen. Das ändert die Gewichte der
Odometrie-Kanten im Posengraphen und damit die Laufbahnen; es braucht deshalb einen eigenen Schritt
mit den SLAM-Tests (Meilenstein 2) als Schranke. Alternativ nur die Anzeige umrechnen (die Farbe nach
Rang statt nach Wert), ohne SLAM zu ändern.

## Ablauf von Hand

Der Handtest mit Maus (Projekt anlegen, Büro-Scan hinzufügen, Prüfen, SLAM rechnen und abbrechen,
Karte rechnen, drei Farbmodi, Umkreisen/Fliegen, Punkt anklicken, Schleife per Klick, neu rechnen,
Vorher/Nachher, Eintrag abschalten, Stand wiederherstellen; Retina und Windows 150 %) ist **auf beiden
Plattformen offen**. Das Terminal, aus dem gemessen wurde, darf den Bildschirm nicht aufnehmen und
keine Eingaben schicken. Automatisch geprüft ist:

| Ablauf | wodurch |
|---|---|
| Projekt anlegen, Scan und Lauf hinzufügen, Projekt wieder öffnen | `project.rs`, `ui_app.rs`, `ui_left.rs` |
| Schritte starten, Abbrechen ohne Dateien (SLAM und Karte) | `project.rs` (`cancelled_jobs_leave_no_folder`), `ui_app.rs`, `jobs.rs` |
| Wolke laden, drei Farbmodi, Budget, Höhenschnitt, Punkt anklicken | `view3d.rs`, Bildschirmfoto-Test |
| Klick trifft bei 2 Pixeln je Punkt (Retina) | `pick_uses_physical_pixels` (Rechnung; nicht mit echter Maus) |
| Zwei Stellen auf der Laufbahn, Schleife eintragen, neu rechnen, Eintrag abschalten | `ui_correct.rs`, `buero_scan_reuse_in_seconds` |
| Vorher/Nachher, Verlauf, Stand wiederherstellen | `compare.rs`, Bildschirmfoto-Test |

## Bilder

Lokal geschrieben vom Bildschirmfoto-Test (`ui_screenshots`, nicht im Repo), Fenster 1400 × 900:

- **Büro in Farbe, von schräg oben:** die ganze Etage, natürliche Farben, die Dächer der Räume
  sichtbar; Statuszeile „color.las geladen: 15,0 Mio. Punkte (Laden 0,9 s, Detailstufen 0,2 s)“,
  gezeichnet 1,7 Mio. Punkte.
- **Büro ohne Decke von oben** („Decke ausblenden“, z max −0,78 m): die Räume mit Möbeln und Regalen,
  darüber die Laufbahn in den Farben der Güte: überwiegend rot und orange, einzelne gelbe und grüne
  Abschnitte (passend zur Auswertung oben), grüne Schleifenlinien.
- **Korrektur:** Lauf im Projekt gewählt, Odometrie (cyan) neben der korrigierten Laufbahn, Anker der
  Teilkarten als Punkte, Stelle A und B am Start (0,0 s und 579,9 s) magenta markiert, Felder
  „Stelle A/B“ ausgefüllt.
- **Vorher/Nachher:** Odometrie (A, orange) an der korrigierten Laufbahn (B, hellblau) ausgerichtet;
  rechts ATE 0,944 m, Start↔Ende 2,162 / 0,129 m, Länge 158,1 / 160,4 m, 5.819 Posen.
- Synthetische Wolke in Farbe, Intensität, Güte, mit Budget 1 Mio. und mit angeklicktem Punkt.

Die Zeit „ms je Bild“ in diesen Bildern ist ohne Aussage (der Test rendert ohne Zeitstempel und ohne
festen Takt).

## Tests

```sh
cargo test -p driftless-app --release --test real_data -- --ignored --nocapture --test-threads 1
cargo test -p driftless-app --release --test lod_memory_real -- --ignored --nocapture
cargo test -p driftless-app --release --test view3d -- --ignored --nocapture
```

| Test | prüft | gemessen |
|---|---|---|
| `buero_scan_lod_build` | Büro: Laden mit Güte, Detailstufen ≤ 10 s, zweites Laden | 0,95 s + 0,20 s |
| `buero_scan_octree_live_heap` (eigenes Testprogramm mit zählendem Allokator) | Speicher des Aufbaus ≤ 64 B/Punkt + 32 MiB | 40,0 B/Punkt |
| `buero_scan_reuse_in_seconds` | Büro: Schleife eintragen und neu rechnen wie das Programm ≤ 15 s, neuer Stand, `traj.vorher.tum` = alte Laufbahn | 5,9 s |
| `app_steps_match_cli` | `Test`: SLAM und Karte aus dem Programm = Release-CLI, alle Dateien | 13 von 13 gleich |
| `synthetic_50m_build` | 50 Mio.: Aufbau, RSS (nur Ausgabe) | 0,55 s, 2,97 GB |
| `synthetic_50m_frame_time` (`view3d.rs`) | 50 Mio.: Bildzeit bei 6 und 12 Mio. (nur Ausgabe) | siehe oben |
| `ui_screenshots` | Bilder (mit `DRIFTLESS_SHOTS`) | siehe Bilder |

Gesamtprüfung: `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
`cargo clippy --release -p driftless-app --bins -- -D warnings`, `cargo test --workspace` (524
bestanden, 33 ignoriert), `cargo deny check` und `cargo test --release -- --ignored` (alle Tests mit
echten Daten aller Crates: 33 bestanden, 35 min).

**CI** (`macos-14`, `windows-latest`): wie bisher Format, Clippy, Tests, Release-Clippy des Programms,
`cargo deny`. Neu baut der Windows-Lauf bei Pull Requests `driftless-app.exe` im Release und lädt sie
als Artefakt `driftless-app-windows` hoch (14 Tage). Der macOS-Läufer hat nicht immer einen
Metal-Adapter; dann überspringen sich die GPU-Tests selbst. Unter Linux gibt es kein Fenster, das
Programm wird dort nicht geprüft.

## Einschränkungen

- **Kein Cache der Detailstufen:** Sie werden bei jedem Laden neu gebaut. Bei 0,2 s (Büro) und 0,55 s
  (50 Mio.) ist das kein Problem; ein Cache ist für M6 nicht nötig, solange Wolken in dieser Größe
  bleiben.
- **Ein Schritt zur Zeit.** Während SLAM oder Karte läuft, startet kein zweiter Schritt.
- **Linien mit fester Breite auf dem Schirm** (Laufbahn 2,5, Odometrie und Schleifen 1,5, verworfene
  Kandidaten 1 Punkt), nicht mit der Entfernung skaliert. Die im Plan genannte Grenze „Linien 1 px“
  gilt seit Task 11 nicht mehr.
- **Export erst in Meilenstein 6** (LAZ, E57, PCD aus dem Programm).
- **Güte zu streng** (siehe oben): Die Laufbahn ist fast überall rot, auch wo SLAM gut ist.
- **Beim Wechseln der Wolke** sind kurz beide im Speicher (Büro +0,7 GB, 50 Mio. geschätzt +3 GB).
- **Nur auf dem Mac gemessen.** RTX 5080 und der Handtest unter Windows 150 % stehen aus; GPU-Zeit
  unter DX12/Vulkan ungeprüft.
- **Je ein Lauf** je Messung; die Zeiten schwanken zwischen Läufen (siehe Meilenstein 4).
