# Ergebnisse Meilenstein 2 – Drift und Schleifenschluss

Stand: 2026-10-20 · gemessen auf dem MacBook Pro M4 Pro, Release-Build, Standardeinstellungen

## Kurzfassung

- `driftless slam` läuft auf allen fünf Aufnahmen durch. Zwei volle Läufe sind bitgleich (geprüft
  auf den drei JMK6-Aufnahmen; die Hilti-Aufnahmen liefen je einmal), und `--reuse` liefert dieselbe
  Laufbahn wie der volle Lauf (geprüft auf JMK6 `Test`).
- **Keine falsche Schleife** wurde übernommen. Alle übernommenen Schleifen verschieben die
  Odometrie um höchstens 1,3 cm und 0,2°.
- **Die Odometrie ist auf diesen Aufnahmen schon so gut, dass der Schleifenschluss kaum etwas
  zu korrigieren hat.** ATE bleibt gleich (Hilti exp14, exp04); bei JMK6 `Test`
  (Vergleich mit RayStudio) ist sie um 0,5 mm schlechter als die Odometrie (0,0178 → 0,0183 m).
  Das liegt unter der Aussagegrenze von ≈ 2 mm, ist also nicht messbar verschlechtert. Eine Verbesserung durch den Schleifenschluss lässt sich mit
  diesen Daten **nicht zeigen**; geprüft ist, dass er nichts verschlechtert.
- Schwellwert `min_inlier_ratio` von 0,5 auf **0,25** gesenkt (Begründung unten): mehr echte
  Schleifen (JMK6 `Test` 1 → 15, `143623` 3 → 10, `143415` 0 → 2), keine falsche dazu.
- `--reuse` braucht 0,01–2,4 s statt 1–32 s.

## Messung

Befehle (Extrinsik der Hilti-Aufnahmen aus `docs/testdaten.md`):

```sh
driftless slam <ordner> -o <ziel> [--lidar-to-imu …]
driftless slam <ordner> -o <ziel> [--lidar-to-imu …] --reuse
driftless compare <ziel>/traj_odom.tum <referenz> --max-dt 0.05   # vor dem Schleifenschluss
driftless compare <ziel>/traj.tum <referenz> --max-dt 0.05        # nachher
```

Laufzeit und Spitzen-RAM mit `/usr/bin/time -l`. ATE = mittlere Abweichung (RMSE) nach bester
starrer Ausrichtung, wie in Meilenstein 1.

| Aufnahme | Referenz | Dauer, Weg | ATE Odometrie | ATE SLAM | Start↔Ende vorher → nachher | Schleifen übernommen / verworfen | Laufzeit voll | `--reuse` | Spitzen-RAM voll / `--reuse` |
|---|---|---|---|---|---|---|---|---|---|
| Hilti 2022 exp14 | dichte Laufbahn (Wahrheit) | 74 s, 38,8 m | 0,0738 m | 0,0738 m | 21,419 → 21,419 m (Referenz 21,303) | 0 / 0 (kein Kandidat, kein Rundgang) | 8,9 s | 0,01 s | 213 MB / 21 MB |
| Hilti 2022 exp04 | 7 Kontrollpunkte (Wahrheit, nur Position) | 126 s, 76,6 m | 0,0553 m | 0,0553 m | 0,087 → 0,087 m | 0 / 9 | 32,2 s | 0,8 s | 811 MB / 195 MB |
| JMK6 `Test` | RayStudio `trj.txt` (keine Wahrheit) | 266 s, 86,9 m | 0,0178 m | 0,0183 m | 5,407 → 5,410 m (RayStudio 5,382) | 15 / 34 | 9,5 s | 2,4 s | 93 MB / 53 MB |
| JMK6 `20261020143415` | keine | 63 s, 14,8 m | – | – | 0,686 → 0,684 m | 2 / 6 | 1,1 s | 0,1 s | 33 MB / 14 MB |
| JMK6 `20261020143623` | keine | 108 s, 20,6 m | – | – | 0,026 → 0,024 m | 10 / 11 | 1,7 s | 0,25 s | 42 MB / 15 MB |

Hilti exp04, Abweichung je Kontrollpunkt nach Ausrichtung (Odometrie und SLAM gleich):

| Kontrollpunkt | 1 | 2 | 3 | 4 | 5 | 6 | 7 (= 1) |
|---|---|---|---|---|---|---|---|
| Abweichung | 0,061 m | 0,043 m | 0,069 m | 0,053 m | 0,038 m | 0,073 m | 0,038 m |

Abstand der Laufbahn zwischen Kontrollpunkt 1 und 7 (dieselbe Stelle): 0,087 m.

## Warum exp04 nicht besser wird

exp04 ist der einzige Rundgang mit Wahrheit. Die Odometrie liegt dort an allen 7 Kontrollpunkten
4–7 cm daneben, **ohne dass der Fehler mit der Zeit wächst** – das sieht nicht nach Drift aus.

- Die Schleifenkandidaten zwischen Anfang und Ende (Teilkarte 0 ↔ 14, 17, 18) passen per ICP
  bis auf 3–4 mm zur Odometrie. Die Scans selbst sehen also keine Lücke.
- Das Gerät steht an Kontrollpunkt 1 und 7 um 82° verdreht. Steht der Kontrollpunkt nicht
  genau unter dem IMU-Ursprung, sondern um einen Hebelarm versetzt, entsteht so ein scheinbarer
  Fehler. Probe: Ein zusätzlich geschätzter Hebelarm von 6,2 cm (im Gerät) senkt die Abweichung
  auf 0,024 m (RMSE) und den Abstand Kontrollpunkt 1 ↔ 7 auf 0,024 m. Der Hebelarm ist geschätzt,
  nicht aus dem Datensatz belegt; die Zahl ist nur ein Hinweis, keine Messgröße.
- Die Kandidaten haben nur 12–16 % passende Punkte. Mit dem neuen Schwellwert 0,25 bleiben sie
  verworfen; mit 0,15 bzw. 0,10 übernommen, ändern sie ATE um +0,2 bzw. +0,6 mm (siehe unten).

Folge: Der Test `loop_sequence_slam_not_worse` prüft auf exp04 „nicht schlechter als die
Odometrie“ statt „besser“ (der Plan sah `loop_sequence_slam_better` vor). Ein Datensatz, auf dem
die Odometrie messbar driftet, fehlt noch.

## Schwellwert `min_inlier_ratio`

Gemessen auf den Teilkarten (`submaps.bin`) aller Aufnahmen, nur der Graph-Teil neu gerechnet
(`solve`, bitgleich zu `--reuse`). Ob eine Schleife echt ist, zeigt die Korrektur gegenüber der
Odometrie: Echte Wiederbesuche passen bei so geringer Drift bis auf wenige Zentimeter zur
Odometrie; ein ICP, der in eine falsche Lage gerutscht ist, springt um Dezimeter bis Meter. Bei
JMK6 `Test` zusätzlich gegen die relative Lage aus RayStudio `trj.txt` geprüft.

### Echte und falsche Kandidaten

| Aufnahme | Art | Anzahl | Anteil passender Punkte | Restfehler (RMS) | Bestimmtheit | Korrektur gegen Odometrie | gegen RayStudio |
|---|---|---|---|---|---|---|---|
| JMK6 `Test` | echt | 41 | 0,087–0,560 | 7–26 mm | 0,046–0,154 | ≤ 3,7 cm, ≤ 0,4° | ≤ 5,3 cm, ≤ 0,8° |
| JMK6 `Test` | falsch (ICP gerutscht) | 8 | **0,009–0,065** | 28–83 mm | 0,025–0,236 | 0,7–3,7 m, bis 48° | 0,7–3,7 m |
| JMK6 `143415` | echt | 7 | 0,156–0,510 | 7–12 mm | 0,015–0,203 | ≤ 1,3 cm | – |
| JMK6 `143415` | vermutlich falsch | 1 (0 ↔ 3) | **0,177** | 28 mm | 0,025 | 0,98 m | – |
| JMK6 `143623` | echt | 21 | 0,127–0,557 | 6–14 mm | 0,008–0,251 | ≤ 1,1 cm | – |
| Hilti exp04 | wohl echt | 9 | 0,044–0,161 | 18–36 mm | 0,165–0,276 | ≤ 4,9 cm | – |

Restfehler und Bestimmtheit trennen echte und falsche Kandidaten **nicht**: Drei falsche bei
`Test` (Teilkarten 1 ↔ 11, 0 ↔ 12, 1 ↔ 12) hätten mit 28–29 mm Restfehler und Bestimmtheit
0,12–0,21 beide Prüfungen bestanden. Es trennt nur der Anteil passender Punkte. Die falschen
Kandidaten von `Test` betreffen alle die Teilkarten 11–13 (108–139 s des Scans); warum der ICP
gerade dort abrutscht (wiederholte Struktur wie in Ruling D befürchtet oder wenig gemeinsame
Fläche), ist nicht untersucht. Abgefangen hat sie der Überlappungs-Schwellwert.

### Wirkung verschiedener Schwellwerte

Übernommene Schleifen, ATE und Start↔Ende nach dem Schleifenschluss:

| `min_inlier_ratio` | exp04: Schleifen, ATE | `Test`: Schleifen, ATE | `143415`: Schleifen, Start↔Ende | `143623`: Schleifen, Start↔Ende |
|---|---|---|---|---|
| 0,50 (bisher) | 0, 0,0553 m | 1, 0,0180 m | 0, 0,686 m | 3, 0,025 m |
| 0,40 | 0, 0,0553 m | 3, 0,0183 m | 0, 0,686 m | 4, 0,025 m |
| 0,30 | 0, 0,0553 m | 11, 0,0188 m | 2, 0,684 m | 5, 0,024 m |
| **0,25 (neu)** | **0, 0,0553 m** | **15, 0,0183 m** | **2, 0,684 m** | **10, 0,024 m** |
| 0,20 | 0, 0,0553 m | 25, 0,0199 m | 3, 0,685 m | 13, 0,024 m |
| 0,15 | 3, 0,0555 m | 31, 0,0172 m | 4, 0,684 m | 17, 0,024 m |
| 0,10 | 7, 0,0559 m | 35, 0,0185 m | 4, 0,684 m | 18, 0,024 m |

Odometrie zum Vergleich: exp04 0,0553 m, `Test` 0,0178 m. exp14 hat keinen Kandidaten und bleibt
bei jedem Wert 0,0738 m. Bei keinem Wert wurde eine Schleife als „widerspricht anderen
Schleifen“ verworfen, und keine übernommene Schleife war falsch.

### Entscheidung: 0,25

- **Mehr echte Schleifen:** `Test` 1 → 15, `143623` 3 → 10, `143415` 0 → 2.
- **Keine falsche dazu, mit Abstand:** Der höchste Anteil eines falschen Kandidaten war 0,177
  (`143415`, 0 ↔ 3; dort zusätzlich von „schlecht bestimmt“ bzw. „nicht konvergiert“ abgefangen),
  bei `Test` 0,065. 0,25 hält diese allein über den Anteil heraus; 0,15 oder weniger verließe sich
  bei `143415` nur noch auf die anderen Prüfungen.
- **ATE:** exp14 und exp04 unverändert. `Test` gegenüber dem bisherigen Schwellwert 0,50:
  0,0180 → 0,0183 m (+0,3 mm); gegenüber der Odometrie: 0,0178 → 0,0183 m (+0,5 mm schlechter).
  Beides liegt unter der Aussagegrenze von ≈ 2 mm (Vergleich mit RayStudio, keine Wahrheit) und
  innerhalb der Streuung aller Varianten (0,0172–0,0199 m), ist also nicht messbar verschlechtert. Werte unter 0,25 bringen auf exp04 (Wahrheit) +0,2 bis
  +0,6 mm.
- Manuelle Schleifen haben ihren eigenen Schwellwert `manual_min_inlier_ratio` (0,25, unverändert).

Sonst wurde nichts verstellt.

## Falsche Schleifen und Widersprüche

- **Falsch übernommene Schleifen: keine** auf allen fünf Aufnahmen und bei allen geprüften
  Schwellwerten (0,10–0,50). Größte Korrektur einer übernommenen Schleife mit 0,25: 1,3 cm / 0,2°.
- **„widerspricht anderen Schleifen“: kein einziges Mal.** Die Prüfung über den Pose-Graphen
  wurde auf echten Daten also nie gebraucht; ihre Wirkung ist nur synthetisch belegt.
- Verworfen wurde mit 0,25 aus diesen Gründen: „zu wenig Überlappung“ 57×, „schlecht bestimmt“ 3×
  (`143415`, Teilkarte 6: Bestimmtheit 0,015–0,020).

## Tests

`crates/driftless-slam/tests/real_data.rs`, alle `#[ignore]`, überspringen sauber ohne Daten:
`cargo test --release -p driftless-slam -- --ignored --nocapture` (gemessen 37 s nach fertigem
Build). Die Tests laufen parallel, die Dauer bestimmt exp04; die Laufzeiten, die die Tests ausgeben,
sind deshalb länger als die Einzelläufe der Tabelle oben. Mit `--test-threads=1` dauert es länger.

| Test | prüft |
|---|---|
| `exp14_slam_not_worse` | ATE SLAM ≤ Odometrie × 1,05 und ≤ 0,074 m × 1,2 |
| `loop_sequence_slam_not_worse` | exp04: ATE an den Kontrollpunkten SLAM ≤ Odometrie × 1,05 und ≤ 0,0553 m × 1,2; gibt Abweichung je Kontrollpunkt aus |
| `jmk6_slam_not_worse_than_odom` | `Test` gegen `trj.txt`: ATE SLAM ≤ Odometrie × 1,05 und ≤ 0,0183 m × 1,2; zwei volle Läufe bitgleich (Laufbahn, Odometrie, Schleifenprüfungen) |
| `jmk6_reuse_under_10_s` | `Test`: Zwischenspeicher lesen, Graph lösen, Ausgaben bilden < 10 s (gemessen 2,8–3,1 s), gleiche Laufbahn wie voller Lauf |
| `jmk6_scan_143415_deterministic` | zwei Läufe bitgleich (Laufbahn und Schleifenprüfungen) |
| `jmk6_scan_143623_gap_not_worse` | zwei Läufe bitgleich, mindestens eine Schleife, Start↔Ende nach dem Schleifenschluss ≤ Odometrie + 2 mm und ≤ 0,024 m × 1,2 + 0,01 m |

## Einschränkungen

- **Verbesserung nicht belegt.** Auf keiner Aufnahme driftet die Odometrie so, dass der
  Schleifenschluss messbar helfen könnte. Dass er große Drift zurückholt, zeigen nur die
  synthetischen Tests (Meilenstein-2-Aufgaben 5 und 6).
- **exp04-Referenz:** nur 7 Punkte, nur Position; der Bezugspunkt am Gerät ist unklar (Hebelarm,
  siehe oben). Die 5,5 cm sind eher eine Obergrenze für den Fehler der Laufbahn.
- **JMK6 `Test`** wird gegen RayStudio verglichen, nicht gegen eine Wahrheit; Unterschiede unter
  ≈ 2 mm sind nicht aussagekräftig. RayStudio endet 5,38 m vom Start, der Rundgang ist dort nicht
  geschlossen.
- **`143415`:** Ob die Aufnahme am Start endet, ist offen; die 0,68 m Start↔Ende sind keine
  Fehlerangabe. Die übernommenen Schleifen bestätigen die Odometrie auf ≈ 1 cm.
- **Echt/falsch** ist bei den Aufnahmen ohne Referenz aus der Größe der Korrektur geschlossen,
  nicht gemessen. Bei stark driftender Odometrie wäre diese Unterscheidung schwieriger.
- **Spitzen-RAM** bei exp04 (811 MB) kommt vor allem von der Odometrie, die zusätzlich die ganze
  Karte im Speicher hält, und von den Teilkarten der dichten Hesai-Punktwolken.
- **Ruling D bleibt offen:** Wiederholte Strukturen wurden hier nur durch den Überlappungs-
  Schwellwert abgefangen. Eine eigene Eindeutigkeitsprüfung gibt es nicht.
- Laufzeiten aus je einem Lauf; das Lesen der Bag (exp04: 10,5 GB mit Kamerabildern) ist
  enthalten.
