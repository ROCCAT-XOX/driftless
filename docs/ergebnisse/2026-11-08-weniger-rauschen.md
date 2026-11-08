# Ergebnisse Weniger Rauschen – Prüfung auf echten Scans

Stand: 2026-11-08 · Zweig `rauschen`, Programm auf dem Stand `8ab0cf6`. Gerechnet auf VM 101 (Linux, NVIDIA
RTX 5080, 6 Threads, `nice 10`, Release-Build) mit `tools/qualitaet/run_rauschen.sh` nach Abschnitt 11 der
Spezifikation (`docs/superpowers/specs/2026-11-08-weniger-rauschen-design.md`). Das Skript lief auf
echten Daten zum ersten Mal und ohne Fehler durch: 26 Oberflächen, 2 h 38 min.

Vergleichsbilder liegen nur lokal, nicht im Repository.

## Das Wichtigste

- Die Form ist klar besser: Rauheit der Wände auf allen Sätzen um 83–95 % im Median, Abstand zur Ebene der
  Punkte um 54–67 %, Wände Median, Raummaße und Löcher bleiben.
- Das Bildrauschen ändert sich nicht: Der Filter (`entrauschen`) greift fast nirgends. Ziel −30 % im
  dunklen Raum verfehlt.
- Die Verdeckung findet bei 1 cm keine einzige Stelle, bei 2 cm 574–8.266. Bei 1 cm wirkt sie also gar nicht.
- Verfehlt sind außerdem: Zahl der Karten (mehr statt gleich), `seam_step` bei den meisten Sätzen,
  Speicher Büro 1 cm (9,75 GB) und Zeit Büro 2 cm (+17 %), `.glb` bei 2 cm in drei Sätzen.
- Keine Maßnahme verschlechtert Wände Median, Raummaße oder Löcher. Nach der Regel aus Abschnitt 11 muss also
  keine abgeschaltet werden.

## So ist gemessen

| Kurz | Scan-Ordner |
|---|---|
| S1 | `20261022133554` |
| S2 | `20261022132520` |
| S3 | `20261022133028` |
| Büro | `20261020173822` |
| Modell | `driftless merge` aus S1, S2, S3 |

Laufbahn und Karte (0,5 cm) entstehen einmal je Scan. „Vorher“ ist `--ohne alles`, „nachher“ die Vorgabe,
jeweils 1 cm und 2 cm auf allen fünf Sätzen. Jede Maßnahme einzeln abgeschaltet: S1, 1 cm. Prozessor: S1,
2 cm, `--gpu aus`. Zahlen aus `oberflaeche.json`, `q_table.py` und `images.log` (Bilder neben der Punktwolke).
Die `metrics.json` entstehen auf der VM; Tabelle und Protokolle sind außerhalb des Repositorys gesichert.

Die Python-Umgebung `~/td/quality-venv` fehlt auf der VM. Für die Messung lief ein eigenes `venv` mit numpy,
scipy, laspy (und pillow) im Arbeitsordner. Das Skript selbst blieb unverändert.

## Ergebnis gegen die Grenzen aus Abschnitt 11

| Messgröße | Ziel | Ergebnis | bestanden |
|---|---|---|---|
| Rauheit Wände, Median | mindestens halbiert | 1 cm −83 bis −94 %, 2 cm −83 bis −95 % | ja |
| Rauheit Wände, 95 % | mindestens halbiert | S1–S3 −64 bis −80 %; Modell 1 cm −56 %, 2 cm −44 %; Büro 1 cm −47 %, 2 cm −35 % | **nein** (Büro, Modell 2 cm) |
| Wand zur Ebene der Punkte, 95 % | mindestens −30 % | −54 bis −67 % | ja |
| dieselbe gegen die ungeglättete Wolke, Median | nicht schlechter als +0,3 mm | 0,21–0,50 mm besser; beim Modell nicht messbar (keine dichte Wolke) | ja |
| Wände Median | nicht schlechter als +0,2 mm | höchstens +0,07 mm (S2, 1 cm); Büro 1 cm −0,53 mm | ja |
| Raummaße Spanne S1–S3 | ≤ 2,5 mm | 0,8 / 1,1 / 2,0 mm (aus der Wolke, unverändert) | ja |
| Löcher von oben, Rundblick | nicht schlechter als +0,5 Prozentpunkte | höchstens +0,1 | ja |
| Flächen ohne Bild | nicht schlechter als +0,1 Prozentpunkte | höchstens +0,02 | ja |
| Karten | nicht mehr als vorher | 1 cm +5 bis +21 %, 2 cm +5 bis +25 %, überall mehr | **nein** |
| `seam_step` je m Naht | −10 % oder besser | 1 cm: S1 −11 %, S2 −11 %, Modell −12 %, S3 −7 %, Büro +1 %; 2 cm: −2 bis −9,5 %, keiner erreicht −10 % | **nein** |
| Bildrauschen, dunkler Raum | mindestens −30 % | `noise_before` = `noise_after` auf jedem Satz (0 %) | **nein** |
| Bildrauschen, Büro | nicht schlechter | gleich, 0 % | ja |
| `.glb` | nicht größer | 1 cm kleiner (−0,1 bis −11 %); 2 cm S1 +1,6 %, S2 +0,05 %, Modell +5,5 % | **nein** (2 cm) |
| Zeit Oberfläche | höchstens +15 % | 1 cm +5 bis +11 %; 2 cm +5 bis +15 %, Büro +17,2 % | **nein** (Büro 2 cm) |
| Spitzen-Speicher Büro, 1 cm | höchstens rund 8 GB | 8,26 → 9,75 GB | **nein** |
| Streifen, dunkler Raum | gegen die Wand daneben von 15,4 auf ≤ 8 Stufen | Oberfläche Modell 2 cm 12,6 → 8,2, 1 cm 12,6 → 9,0; „Bilder neu“ zeigt ihn weiterhin (siehe unten) | **nein** (knapp, bei „Bilder neu“ deutlich) |

### 1 cm

| Messgröße | S1 | S2 | S3 | Büro | Modell |
|---|---|---|---|---|---|
| Rauheit Wände, Median (mm) | 0,41 → 0,03 | 0,43 → 0,03 | 0,44 → 0,03 | 0,34 → 0,04 | 0,29 → 0,05 |
| Rauheit Wände, 95 % (mm) | 1,49 → 0,32 | 1,53 → 0,32 | 1,55 → 0,31 | 1,40 → 0,73 | 1,27 → 0,55 |
| Wand zur Ebene der Punkte, 95 % (mm) | 1,69 → 0,58 | 1,77 → 0,59 | 1,85 → 0,63 | 2,08 → 0,73 | 1,46 → 0,67 |
| Wand zur Ebene gegen dense, Median (mm) | 0,85 → 0,63 | 0,85 → 0,54 | 0,76 → 0,49 | 0,85 → 0,51 | – |
| Wände Median (mm) | 2,63 → 2,66 | 2,92 → 2,98 | 3,06 → 3,10 | 4,81 → 4,28 | 2,03 → 2,03 |
| Löcher von oben (%) | 2,0 → 2,0 | 9,7 → 9,7 | 3,5 → 3,5 | 31,1 → 31,1 | 1,5 → 1,5 |
| Löcher Rundblick, größter (%) | 0,6 → 0,6 | 5,5 → 5,5 | 2,8 → 2,8 | 12,3 → 12,3 | 0,2 → 0,3 |
| Flächen ohne Bild (%) | 0,110 → 0,111 | 0,129 → 0,134 | 0,121 → 0,127 | 1,300 → 1,319 | 0,183 → 0,185 |
| Karten | 71.326 → 84.563 | 59.788 → 72.120 | 73.306 → 82.121 | 688.640 → 723.764 | 85.894 → 100.130 |
| `seam_step` je m Naht | 16,23 → 14,41 | 13,96 → 12,35 | 14,75 → 13,68 | 17,10 → 17,21 | 16,86 → 14,76 |
| Bildrauschen `noise_after` (Stufen) | 0,37 → 0,49 | 0,37 → 0,43 | 0,43 → 0,49 | 0,37 → 0,43 | 0,37 → 0,49 |
| `.glb` (MB) | 109,6 → 102,1 | 101,9 → 92,5 | 108,9 → 96,8 | 614,1 → 587,0 | 104,0 → 103,9 |
| Zeit Oberfläche (s) | 245 → 264 | 212 → 232 | 219 → 244 | 1.284 → 1.366 | 341 → 359 |
| Spitzen-Speicher (GB) | 2,72 → 2,60 | 2,51 → 2,31 | 2,57 → 2,35 | 8,26 → 9,75 | 3,11 → 3,14 |
| Verdecker (nachher) | 0 | 0 | 0 | 0 | 0 |

### 2 cm

| Messgröße | S1 | S2 | S3 | Büro | Modell |
|---|---|---|---|---|---|
| Rauheit Wände, Median (mm) | 0,51 → 0,03 | 0,58 → 0,03 | 0,60 → 0,03 | 0,69 → 0,07 | 0,28 → 0,05 |
| Rauheit Wände, 95 % (mm) | 2,27 → 0,81 | 2,41 → 0,76 | 2,46 → 0,75 | 2,69 → 1,76 | 1,75 → 0,98 |
| Wand zur Ebene der Punkte, 95 % (mm) | 2,28 → 0,79 | 2,41 → 0,80 | 2,55 → 0,85 | 2,77 → 1,01 | 1,80 → 0,81 |
| Wand zur Ebene gegen dense, Median (mm) | 1,04 → 0,70 | 1,14 → 0,64 | 0,88 → 0,64 | 1,02 → 0,60 | – |
| Wände Median (mm) | 2,88 → 2,75 | 3,19 → 3,10 | 3,30 → 3,18 | 4,82 → 4,87 | 2,18 → 2,11 |
| Löcher von oben (%) | 1,5 → 1,5 | 3,8 → 3,8 | 2,2 → 2,2 | 20,2 → 20,3 | 1,1 → 1,1 |
| Löcher Rundblick, größter (%) | 0,3 → 0,3 | 1,5 → 1,6 | 1,3 → 1,3 | 4,3 → 4,3 | 0,1 → 0,1 |
| Flächen ohne Bild (%) | 0,145 → 0,146 | 0,163 → 0,171 | 0,160 → 0,160 | 2,875 → 2,891 | 0,259 → 0,254 |
| Karten | 31.730 → 39.320 | 27.891 → 34.951 | 32.908 → 37.481 | 284.369 → 297.397 | 39.137 → 47.599 |
| `seam_step` je m Naht | 14,10 → 12,81 | 12,03 → 11,14 | 13,24 → 12,61 | 15,47 → 15,17 | 14,36 → 13,00 |
| Bildrauschen `noise_after` (Stufen) | 0,37 → 0,49 | 0,37 → 0,43 | 0,43 → 0,43 | 0,37 → 0,37 | 0,37 → 0,49 |
| `.glb` (MB) | 48,8 → 49,6 | 45,5 → 45,6 | 48,7 → 47,9 | 280,3 → 272,0 | 49,7 → 52,4 |
| Zeit Oberfläche (s) | 121 → 127 | 97 → 103 | 98 → 112 | 407 → 477 | 214 → 228 |
| Spitzen-Speicher (GB) | 1,24 → 1,21 | 1,10 → 1,12 | 1,12 → 1,08 | 4,13 → 3,96 | 1,87 → 1,89 |
| Verdecker (nachher) | 1.270 | 574 | 738 | 8.266 | 5.043 |

## Die Maßnahmen einzeln (S1, 1 cm)

| Messgröße | Vorgabe | ohne Anlegen | ohne Belichtung | ohne Entrauschen | ohne Verdeckung | alles aus |
|---|---|---|---|---|---|---|
| Rauheit Wände, Median (mm) | 0,03 | 0,41 | 0,03 | 0,03 | 0,03 | 0,41 |
| Rauheit Wände, 95 % (mm) | 0,32 | 1,49 | 0,32 | 0,32 | 0,32 | 1,49 |
| Wand zur Ebene der Punkte, 95 % (mm) | 0,58 | 1,69 | 0,58 | 0,58 | 0,58 | 1,69 |
| Wand zur Ebene gegen dense, Median (mm) | 0,63 | 0,85 | 0,63 | 0,63 | 0,63 | 0,85 |
| Wände Median (mm) | 2,66 | 2,63 | 2,66 | 2,66 | 2,66 | 2,63 |
| Löcher von oben / Rundblick größter (%) | 2,0 / 0,6 | 2,0 / 0,6 | 2,0 / 0,6 | 2,0 / 0,6 | 2,0 / 0,6 | 2,0 / 0,6 |
| Flächen ohne Bild (%) | 0,111 | 0,109 | 0,111 | 0,111 | 0,111 | 0,110 |
| Karten | 84.563 | 88.923 | 68.532 | 84.563 | 84.563 | 71.326 |
| `seam_step` je m Naht | 14,41 | 13,38 | 17,35 | 14,42 | 14,41 | 16,23 |
| Bildrauschen `noise_after` (Stufen) | 0,49 | 0,49 | 0,37 | 0,49 | 0,49 | 0,37 |
| `.glb` (MB) | 102,1 | 115,8 | 96,1 | 102,2 | 102,1 | 109,6 |
| Zeit Oberfläche (s) | 264 | 249 | 262 | 264 | 263 | 245 |
| Spitzen-Speicher (GB) | 2,60 | 2,72 | 2,57 | 2,57 | 2,60 | 2,72 |

„Ohne Verdeckung“ ist Byte für Byte gleich der Vorgabe (`oberflaeche.glb`, SHA-256 `cc12a8d0…`): Bei 1 cm gibt
es keine Verdecker.

Befund je Maßnahme:

- **Anlegen (M1):** bringt die ganze Formwirkung (Rauheit, Abstand zur Ebene, `.glb` −12 %). Wände Median
  +0,03 mm, innerhalb der Toleranz. Sie hebt `seam_step` (13,38 → 14,41) und kostet 15 s (+6 %).
- **Belichtung (M2):** senkt `seam_step` deutlich (17,35 → 14,41), ist aber die Ursache der vielen Karten
  (68.532 → 84.563, +23 %). Dass die Karten überall zunehmen, geht auf sie zurück. Sie hebt auch die gemessene
  Körnung (0,37 → 0,49 Stufen). Die Bilder zeigen weniger dunkle Flecken an den Wänden.
- **Entrauschen (M3):** praktisch ohne Wirkung. Die Zahl `noise_before`/`noise_after` ändert sich nicht. Von
  allen Texeln ändern sich nur 0,6 % um mehr als 2 Stufen (S1, 1 cm, dunkle Texel genauso), 0,05 % um mehr als
  5. Die geschätzte Körnung je Karte liegt unter der Schwelle `SIGMA_OFF` = 1 Stufe (Median 0,4–0,5 Stufen;
  gerechnet direkt auf den fertigen Texturseiten, dunkle Texel: S1 mit und ohne Filter je 0,49). Nur wenige
  Karten liegen darüber. Die Maßnahme kostet nichts und schadet nicht, bringt aber auch den Zielwert nicht.
- **Verdeckung (M4):** bei 2 cm 574 bis 8.266 Verdecker, bei 1 cm auf keinem der fünf Sätze einer. Vermutlich
  zerfallen die Punkte dünner Teile bei 1-cm-Zellen in Gruppen unter 30 cm (`OCCLUDER_MIN_EXTENT`), wie beim
  Bau als Risiko notiert; geprüft ist das nicht. Bei 1 cm ändert die Maßnahme also nichts. Auf der Wand
  mit dem Streifen ändert sich bei 2 cm die Fläche (Abschnitt Streifen), Anteil der Verdeckung daran nicht
  einzeln gemessen.

**Regel aus Abschnitt 11:** Keine Maßnahme verschlechtert Wände Median, Raummaße oder Löcher über die Toleranz
(geprüft: alle vier zusammen auf allen fünf Sätzen, einzeln auf S1). Keine muss deshalb abgeschaltet werden.

### Prozessor gegen Grafikkarte (S1, 2 cm)

| | Grafikkarte (nachher) | Prozessor (`--gpu aus`, nachher) |
|---|---|---|
| Rauheit Median / Wand zur Ebene 95 % / Wände Median (mm) | 0,03 / 0,79 / 2,75 | 0,03 / 0,79 / 2,75 |
| Karten | 39.320 | 39.333 |
| `seam_step` je m Naht | 12,81 | 12,82 |
| Verdecker | 1.270 | 1.270 |
| `.glb` | 49,6 MB | 49,8 MB |
| Zeit, Speicher | 127 s, 1,21 GB | 408 s, 1,03 GB |

Beide Wege liefern gleichwertige Ergebnisse (nicht bytegleich, wie schon vor diesem Zweig).

## Streifen im dunklen Raum

Gemessen auf derselben Wand wie im Pinsel-Bericht (Modell S1–S3, Wandmitte `[-0,50; -6,79; 0,03]`), von vorn
gezeichnet, 1,24 mm je Pixel. Das Diagnose-Skript aus Aufgabe 14 liegt nicht mehr vor, die Messung ist von Hand
nachgebaut: Streifen 3 cm breit, die Wand 3 bis 8 cm links und rechts davon, zwölf Abschnitte in der Höhe, je
Abschnitt der größte Kanalunterschied, Median über die Abschnitte. Die absolute Lage zu den alten 15,4 Stufen
ist deshalb nur ungefähr vergleichbar.

| | Streifen / daneben (sRGB) | gegen die Wand daneben |
|---|---|---|
| Oberfläche Modell 2 cm, vorher | 19,4 / 30,7 | 12,6 |
| Oberfläche Modell 2 cm, nachher | 25,1 / 30,9 | 8,2 |
| Oberfläche Modell 1 cm, vorher | 18,9 / 30,5 | 12,6 |
| Oberfläche Modell 1 cm, nachher | 27,3 / 35,1 | 9,0 |
| S1 1 cm, vorher | 24,3 / 39,5 | 16,3 |
| S1 1 cm, nachher | 26,3 / 43,8 | 19,0 |

- Die gerechnete Fläche des Modells wird ruhiger (12,6 → 8,2), das Ziel von 8 Stufen ist knapp nicht
  erreicht. Bei 1 cm gibt es keine Verdecker, die Besserung dort stammt also nicht von der Verdeckung.
- Der Streifen ist auch in der Punktwolke zu sehen, das Band stammt aus den Fotos (wie in Aufgabe 14).
- **„Bilder neu“ Variante 0 auf dieser Stelle zeigt weiterhin einen scharfen dunklen Balken** neben dem Rohr
  (Vergleichsbilder der App, Stelle hell 36,6 gegen Balken 12,6 sRGB). Die Verdeckung ändert daran nichts. In
  Variante 1 ist er schwächer, dafür ein heller Keil daneben.

## Weg „aus“ gegen Version 0.1.0

S1 auf der VM: v0.1.0 (`9247d05`, per `git archive` gebaut) von Laufbahn bis Oberfläche gegen diesen Zweig. Oberfläche
einmal auf der Grafikkarte, einmal mit `--gpu aus`, im Zweig mit `--ohne alles`. Vergleich SHA-256.

| Datei | Größe | Zweig gegen 0.1.0 |
|---|---|---|
| `traj.tum`, `traj_odom.tum`, `submaps.bin`, `map.las`, `slam.json` | | gleich |
| `dense.bin`, `dense.las`, `clean.bin`, `clean.las`, `removed.las`, `color.las`, `color.ply` | | gleich |
| `oberflaeche.glb`, Grafikkarte | 48.804.680 B | gleich |
| `oberflaeche.glb`, Prozessor | 48.717.172 B | gleich |
| `map.json`, `oberflaeche.json` | | verschieden, nur Laufzeiten, Speicher und neue Felder (`noise`, `walls_plane`, `roughness`, `occluders`) |

Die Probe im Skript (`probe.txt`, Text „von 99284b9“ ist fest eingebaut) lief hier mit dem Programm von v0.1.0
und meldet ebenfalls „gleich“. Eine Abweichung gibt es nicht, der Vergleich gegen `99284b9` war nicht nötig.

## Nacharbeit (`touchup_real`, Stand dieses Zweigs)

Der erste Lauf brach nach dem Projektbau beim Öffnen des Fensters ab: `RequestDeviceError … Device(Lost)`. Kein
GPU-Fehler im Kernel-Log; Ursache unbekannt. Der zweite Lauf auf demselben Projekt ging ohne Änderung durch. Die
Zahlen stammen aus dem zweiten Lauf.

| Aktion | Modell S1–S3 | Büro | Stand Pinsel-Bericht (Modell / Büro) |
|---|---|---|---|
| Stelle malen | 0,09 s | 0,11 s | |
| „ganz eben“ | 2,69 s, Spitze 1,85 GB | 1,71 s, 3,46 GB | 2,82 / 1,84 s |
| „stark“ | 2,45 s, 1,87 GB | 1,69 s, 3,47 GB | 2,66 / 1,76 s |
| „Bilder neu“ Variante 0 (nicht im Cache) | 2,65 s, 1,87 GB | 1,51 s, 3,17 GB | 2,74 / 1,59 s |
| „Bilder neu“ Variante 1 | 2,56 s, 1,87 GB | 1,51 s, 3,17 GB | 2,72 / 1,56 s |
| ↶ | 0,02 s | 0,02 s | 0,02 / 0,01 s |

Ziel höchstens 5 s je Aktion: erfüllt (1,5–2,7 s). Malen im Büro: 285 Bilder/s im Median, 117 im 90-%-Wert, keines
unter 30/s (RTX 5080).

Glätten am Rand (Punkte neben dem Rand, 95 %): Modell „ganz eben“ 1,10 mm (vorher 1,08), Büro 0,82 mm (vorher
1,23); Rand selbst 0,000 mm. Farbunterschied am Rand der neuen Bilder, Mittel:

| | Modell, jetzt | Modell, Pinsel-Bericht | Büro, jetzt | Büro, Pinsel-Bericht |
|---|---|---|---|---|
| „ganz eben“ | 4,23 | 2,97 | 4,55 | 5,10 |
| „stark“ | 5,78 | 3,25 | 4,59 | 5,12 |
| „Bilder neu“ V0 | 4,23 | 3,27 | 4,58 | 5,35 |
| „Bilder neu“ V1 | 5,04 | 6,51 | 9,83 | 6,21 |

Das Modell ist am Rand etwas schlechter (+1,0 bis +2,5 Stufen), außer V1. Das Büro ist bei V0 besser, bei V1
schlechter (90 % bei 30 Stufen; gegen 15). In den Bildern ist am Rand keine Kante zu sehen; die Griffe im Büro sind
nicht zerschnitten.

**Varianten:** sichtbar verschieden. V0 und V1 auf der Modell-Wand zeigen den Balken unterschiedlich stark und
an anderer Stelle (andere Fotos, siehe oben). Die sechs Varianten stammen aus sechs verschiedenen Bildern und
Standorten:

| | Standort gegen die Varianten davor (m / Blickrichtung) |
|---|---|
| Modell V1–V5 | 0,97/13° · 0,68/19° · 0,32/27° · 0,57/5° · 0,35/11° |
| Büro V1–V5 | 0,90/7° · 0,48/16° · 0,29/5° · 0,07/2° · 0,13/6° |

Im Büro liegen V4 und V5 („so ähnlich wie V1/V2“) nur 7–13 cm neben früheren Bildern; dort ist der Unterschied
gering.

## Bodenstelle 3 × 3 m

Mit dem Test `f4_boden` (auf der VM in den Arbeitsstand kopiert, nicht im Repository). Die Tupfer ergeben
etwa 8,9 m²:

| | Modell | Büro |
|---|---|---|
| Flächen, Fläche | 18.585, 8,88 m² | 16.439, 8,93 m² |
| „Bilder neu“ V0 / V1 / V2 | 36,9 / 37,1 / 36,7 s | 12,1 / 12,1 / 11,9 s |
| davon „Bilder der Stelle suchen“ | 34,2 s | 9,4 s |
| Backen | 2,4–2,7 s | 2,3–2,5 s |
| „ganz eben“ (backt neu) | 37,0 s | 12,4 s |
| Speicher (Spitze) | 1,6–1,8 GB | 2,2–2,5 GB |
| Abbrechen dauert | 0,13 s (Suche), 0,02 s (Backen) | 0,06–0,08 s (Suche), 0,03 s (Backen) |

Die Sichtbarkeitsprüfung wächst mit der Fläche; bei 9 m² sind das 10–37 s statt unter 3 s auf 1 m². Abbrechen
geht auch dort sofort. Ein Fehler trat nicht auf.

## Speicher und Zeit im Büro

| | 1 cm vorher | 1 cm nachher | 2 cm vorher | 2 cm nachher |
|---|---|---|---|---|
| Zeit Oberfläche | 21:24 min | 22:46 min (+6 %) | 6:47 min | 7:57 min (+17 %) |
| Spitzen-Speicher | 8,26 GB | 9,75 GB | 4,13 GB | 3,96 GB |

Bei 1 cm kommen 1,5 GB dazu, das Ziel von höchstens rund 8 GB ist verfehlt. Welche Maßnahme das verursacht, ist
im Büro nicht einzeln gemessen.

## Offene Punkte für die Entscheidung

1. Bildrauschen: Der Filter schaltet kaum ein, weil die gemessene Körnung (Median 0,4–0,5 Stufen) unter der
   Einschaltschwelle von 1 Stufe liegt. Soll die Schwelle sinken, oder bleibt die Maßnahme so?
2. Verdeckung wirkt bei 1 cm nicht.
3. Karten +5 bis +25 % durch die Belichtung, `seam_step`-Ziel teilweise verfehlt.
4. Speicher Büro 1 cm 9,75 GB statt rund 8 GB; Zeit Büro 2 cm +17 %.
5. Bei einer Änderung an den Maßnahmen: Anleitung und CHANGELOG beschreiben ihre Wirkung, Abschnitt 5 der Anleitung
   nennt noch Zeiten und Speicher von 0.1.0.

## Bilder

Lokal auf dem Mac unter `~/Downloads/Driftless-Rauschen-Bilder/`, nicht im Repository: Vorher/Nachher nebeneinander
und Reliefbilder (1 cm und 2 cm, alle fünf Sätze), einzelne Maßnahmen, Prozessor, Nacharbeit, Nahaufnahmen der
Streifenwand und dunkler Wände (etwa 1:1, statt reiner Texturausschnitte), Messwerte.

## So lässt sich der Lauf wiederholen

```
driftless-vm sync   # mit DRIFTLESS_REPO=$PWD DRIFTLESS_RDIR=driftless-rauschen
DQ_ROOT=… DQ_PYTHON=… DQ_PROBE_BIN=<v0.1.0/driftless> tools/qualitaet/run_rauschen.sh r1 --abgekoppelt
DRIFTLESS_TESTDATA=… DRIFTLESS_WORK=… DRIFTLESS_SHOTS=… gpu-job -- nice -n 10 cargo test -p driftless-app --release --test touchup_real -- --ignored --nocapture touchup_on_real_scans
```
