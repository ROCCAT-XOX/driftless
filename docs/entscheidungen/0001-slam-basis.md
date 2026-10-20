# 0001 – SLAM-Basis: GLIM oder eigener Rust-Kern

Stand: 2026-10-20 · Status: **Angenommen** (Yannick, 20.10.2026): eigener Rust-Kern, GLIM nur als Vergleich

## Frage

Welche Grundlage berechnet in Driftless die Laufbahn (LiDAR-IMU-SLAM)?

- **GLIM** (C++, MIT) als externer Prozess – Spike A, `spikes/glim/`
- **Eigene Rust-Grundlinie** (`crates/driftless-odom`, `driftless odom`) – Spike B, noch ohne Schleifenschluss

## Messung

Zwei Aufnahmen; die Rust-Grundlinie lief mit Standardeinstellungen, GLIM zuerst ebenfalls (Standard), auf Hilti danach zusätzlich eingerichtet (Fußnote ²); gemessen auf dem MacBook Pro M4 Pro.
ATE = mittlere Abweichung (RMSE) nach bester Ausrichtung (`driftless compare --max-dt 0.05`).

- **Eigener Scan JMK6** (`Test`, 266 s, Innenraum): verglichen mit RayStudio `trj.txt`.
  Das ist **keine Wahrheit**, nur Übereinstimmung mit einem anderen Programm.
- **Hilti SLAM Challenge 2022, `exp14_basement_2`** (74 s, Keller, Hesai XT-32 + Alphasense-IMU):
  verglichen mit der Referenz-Laufbahn des Datensatzes. Das ist eine **objektive Referenz**.
  Lizenz CC BY-NC-SA 3.0: nur lokal genutzt, nichts davon im Repo.

| | GLIM – JMK6 | Rust – JMK6 | GLIM – Hilti, Standard | GLIM – Hilti, eingerichtet² | Rust – Hilti |
|---|---|---|---|---|---|
| ATE | 0,060 m (nur Odometrie 0,116 m) | **0,018 m** | 0,62 / 0,79 / 1,18 m (3 Läufe); 0,75 m mit 1 Thread | **0,326 m** mit 1 Thread; 0,35 / 0,58 m mit 8 Threads | **0,074 m** |
| Start↔Ende (Referenz) | 5,394 m (5,382) | 5,407 m (5,382) | 20,0–20,6 m (21,303) | 21,38–21,48 m (21,303) | 21,419 m (21,303) |
| Weglänge (Referenz) | 84,1 m (98,9¹) | 86,9 m (98,9¹) | 36,5–39,9 m (37,8) | 36,3–37,2 m (37,8) | 38,8 m (37,8) |
| Laufzeit | 7,2–8,0 s + Export 1,1 s | 6,2–6,9 s | 3,9–4,1 s + Export 5,2 s | 14,6 s (1 Thread) / 6,5 s (8 Threads) + Export 5,2 s | 7,0–7,1 s |
| Spitzen-RAM | ≈ 235 MB | 63–72 MB | 410–430 MB (Export 89 MB) | ≈ 460 MB / 405–434 MB | 190–200 MB |
| gleiches Ergebnis bei jedem Lauf | ja | ja (bitgleich) | nein | mit 1 Thread ja, mit 8 Threads nein | ja (bitgleich) |

¹ RayStudio liefert 200 Hz mit Zittern; zu GLIMs Zeitstempeln abgetastet sind es 87,9 m.

² „Eingerichtet“ = nach der Prüfung (Fix-Runde 1): IMU-Initialisierung aus dem Stillstand (`NAIVE`, Fenster 1 s),
IMU-Bias aus der Hilti-Kalibrierung, kleinere IMU-Rauschwerte (0,01 / 0,002 statt 0,05 / 0,02), Treiber füttert
Frame für Frame (`DRIFTLESS_GLIM_MAX_LAG=0`). Extrinsik unverändert. Das ist eine kurze, zeitlich begrenzte Einrichtung, keine
gründliche Abstimmung: **Untergrenze für ein nicht abgestimmtes GLIM**.

Weitere Befunde:

- **Ohne Entzerrung** (Rust, `--no-deskew`): JMK6 0,040 m, Hilti 0,254 m. Die IMU-Entzerrung bringt also viel.
- **GLIM auf Hilti, Standard:** Der größte Fehler lag am Start. Das Gerät steht dort ≈ 6 s still, aber GLIMs
  Standard-Initialisierung (`ROBUST`, 3-s-Fenster) schätzt 0,23 m/s Geschwindigkeit nach unten und keinen Bias;
  die ersten 3 s fehlen in der Ausgabe (Initialisierungsfenster). Danach blieb ein Versatz von ≈ 0,4 m.
  Das war ein Einrichtungsfehler auf unserer Seite, nicht GLIMs Schuld.
- **GLIM auf Hilti, eingerichtet (Fußnote ²):** Start jetzt ohne Sprung (0,1–0,4 m nach Ausrichtung), ATE 0,33 m.
  Der Rest kommt vor allem aus den letzten ≈ 15 s (Fehler wächst bis 0,6 m). GLIM meldet weiterhin zeitweise
  „IMU prediction is not good“ (6× statt 9×). Versuche im Einzelnen: `spikes/glim/README.md`. Mit mehr Abstimmung
  wird GLIM vermutlich noch besser; die Ursache des Rests ist offen.
- **Unser Treiber war nicht reproduzierbar:** Er ließ GLIM bis zu 10 Frames hinterherlaufen. GLIM verarbeitet vor
  jedem Frame alle bis dahin eingespeisten IMU-Daten, so hing das Ergebnis vom Thread-Timing ab. Mit
  `DRIFTLESS_GLIM_MAX_LAG=0` und 1 Thread ist es reproduzierbar; mit 8 Threads streut es weiter (0,35–0,58 m).
- **Extrinsik Hilti:** aus `lidar_calibration.yaml`. Die Datei schreibt Quaternionen als `[qx, qy, qz, qw]`
  (die Identität steht dort als `[0, 0, 0, 1]`), also wörtlich übernommen; GLIM bekommt die Umkehrung.
  Gegenprobe Rust: andere Lesarten ergeben 1,5–3,8 m.
- **Laufzeit Rust** enthält das zweimalige Lesen der Bag (Hilti 6,2 GB mit Kamerabildern). GLIM braucht vorher
  den Export (`driftless export-glim`).

**Vorsicht beim Lesen der Zahlen:**

- Die Rust-Grundlinie wurde am JMK6-Scan abgestimmt. Ihre 1,8 cm dort sind nicht unabhängig.
- Hilti ist für beide Kandidaten unbekannt: Rust lief mit unveränderten Parametern, nur die Extrinsik wurde übergeben.
- Das ist **eine** Hilti-Sequenz (74 s, kein Rundgang). Schleifenschluss wird damit nicht geprüft.
  GLIMs Vorteil (globale Optimierung) kommt hier kaum zum Tragen.
- Rust hat noch keinen Schleifenschluss. Bei langen Aufnahmen wächst die Drift, das zeigen diese Daten nicht.

## Bewertung

| Kriterium | GLIM | Rust-Grundlinie |
|---|---|---|
| Baut auf macOS / Windows | ja / ja, aber nur mit 5 eigenen Patches und 8 GB MSVC-Werkzeugen | ja / ja (cargo, CI grün auf beiden) |
| Genauigkeit (Hilti = Wahrheit) | 0,6–1,2 m mit Standardeinstellungen; bestes Ergebnis nach Einrichtung 0,33 m | 0,074 m |
| Laufzeit | schnell (mit Export ähnlich wie Rust) | schnell, 10–40× schneller als Echtzeit |
| RAM | 235–430 MB | 63–200 MB |
| Gleiche Ergebnisse je Lauf | mit 1 Thread und Schritt-für-Schritt-Treiber ja; mit 8 Threads auf Hilti nein; zwischen Mac und Windows leicht anders | ja, bitgleich |
| Lizenzen | MIT/BSD/Apache/MPL – als eigener Prozess unkritisch | nur Abhängigkeiten unter MIT, Apache-2.0 (auch mit LLVM-Ausnahme), BSD-2/3, ISC, Zlib, Unicode-3.0, MPL-2.0, BSL-1.0, CDLA-Permissive-2.0 (so erlaubt es `deny.toml`); `cargo deny` grün |
| Wartung | hoch: Windows-Patches selbst pflegen; stiller 32-bit-`long`-Fehler gefunden (falsche Ergebnisse ohne Meldung); GTSAM ist eine Alpha (4.3a1); Eigen-Version festnageln; Upstream testet nur Ubuntu | eigener Code: wir pflegen alles selbst, aber in einer Sprache und mit Tests |
| **GPU auf Mac (Metal) und RTX (wgpu)** – Vorgabe Yannick | **nein**: GLIMs GPU-Teil braucht CUDA, also nur NVIDIA, nicht auf dem Mac | **ja, möglich**: der ICP-Kern ist schon als „je Punkt rechnen, dann fest summieren“ gebaut und kann als wgpu-Rechenschritt laufen (noch nicht umgesetzt) |
| Weg zu Schleifenschluss | vorhanden (globale Optimierung über Teilkarten) | fehlt: Teilkarten + Pose-Graph + Erkennung müssen gebaut werden |
| Manuelle Korrektur „in Sekunden“ | GTSAM-Graph liegt vor, aber Bedienung nur über GLIMs eigenen Viewer (C++/ImGui) oder eigene C++-Brücke | eigener Pose-Graph über Teilkarten: wenige hundert Knoten, sollte in Millisekunden lösen (noch nicht gebaut); voll in unserer Hand |
| Zuverlässigkeit | Windows-Fehler ohne Meldung; mit 8 Threads schwankt das Ergebnis je Lauf | auf beiden Aufnahmen stabil; aber lose gekoppelt (keine Beschleunigungs-Bias-Schätzung, langer LiDAR-Ausfall driftet schnell) |

## Empfehlung

**Eigener Rust-Kern als SLAM-Basis.** GLIM bleibt nur als Vergleichswerkzeug im Spike-Ordner (wird nicht ausgeliefert).

Gründe:

1. **Genauer auf der objektiven Referenz:** 7 cm gegen 33 cm (bestes GLIM-Ergebnis nach Einrichtung;
   mit Standardeinstellungen 60–120 cm). Keiner der beiden wurde auf Hilti abgestimmt. Der Abstand ist kleiner
   geworden und ein gut abgestimmtes GLIM käme wohl näher heran – ein klarer Vorsprung bleibt.
2. **GPU-Vorgabe:** Nur der Rust-Weg kann über wgpu auf Mac und RTX rechnen. GLIM ginge nur mit CUDA.
3. **Zuverlässigkeit vor Geschwindigkeit:** bitgleiche Ergebnisse, ein Werkzeugkasten (cargo), keine
   Windows-Patches mit stillen Rechenfehlern.
4. **Korrektur in Sekunden** verlangt ohnehin einen eigenen Pose-Graph in der Oberfläche. Den bauen wir
   besser gleich in Rust als eine Brücke zu GTSAM.

Was es kostet, wenn das falsch ist:

- Der Rust-Kern braucht noch Arbeit, die GLIM schon hat: Teilkarten, Pose-Graph, automatischer
  Schleifenschluss, eine engere IMU-Kopplung (Bias-Schätzung). Grob einige Wochen.
- Stellt sich in Meilenstein 2 heraus, dass das nicht reicht (z. B. große Außenszenen, lange Flure ohne Merkmale),
  ist der Rückweg offen: Das Austauschformat ist dasselbe (Laufbahn als TUM + Punktwolke), GLIM-Treiber,
  Patches und Bauskripte liegen in `spikes/glim/`. Verloren wäre die Zeit für den eigenen Schleifenschluss;
  dazu kämen GLIMs Wartungslast und der Verzicht auf GPU am Mac.

## Offene Punkte für Meilenstein 2

- **Offene GLIM-Einrichtungsfragen** (nur falls der GLIM-Weg doch gebraucht wird): Herkunft des Restfehlers am
  Ende von Hilti exp14, Rauschwerte der Alphasense-IMU (nur geschätzt, keine Kalibrier-Werte vorhanden),
  Streuung mit 8 Threads.

- **Pose-Graph über Teilkarten** (z. B. alle 5–10 m eine Teilkarte), Lösung in Millisekunden; Grundlage für die
  manuelle Korrektur „zwei Stellen gleich“.
- **Automatischer Schleifenschluss:** Kandidaten finden (Nähe + Ortserkennung), mit ICP prüfen, nur sichere übernehmen.
- **Drift-Anzeige:** Abweichung je Teilkarte nach dem Schleifenschluss sichtbar machen.
- **Engere IMU-Kopplung** (iterativer Kalman-Filter mit Bias-Schätzung), Verhalten bei LiDAR-Ausfall.
- **GPU-Pfad:** ICP-Schritt als wgpu-Rechenschritt, Ergebnis gegen die CPU-Version prüfen (gleiche Zahlen).
- **Mehr Testdaten mit Wahrheit:** weitere Hilti-Sequenzen mit Rundgang (Schleifenschluss), eigene Aufnahme mit
  Drift (langer Flur, Treppenhaus) und von Hand gemessenen Strecken.

Quelle Hilti-Daten: Hilti SLAM Challenge 2022 (Hilti Research), CC BY-NC-SA 3.0,
https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022
