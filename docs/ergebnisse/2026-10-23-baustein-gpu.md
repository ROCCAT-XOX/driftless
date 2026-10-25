# Ergebnisse Baustein GPU – Rechnen auf der Grafikkarte

Stand: 2026-10-25 · Branch `baustein-gpu` · Spec `docs/superpowers/specs/2026-10-23-gpu-design.md`

Einfärben der Karte (`driftless map --step color`) und die Ansichten je Dreieck der Oberfläche
(`driftless surface`) laufen jetzt auf der Grafikkarte, über `wgpu`-Compute-Shader: Vulkan auf der
RTX 5080 (VM 101, Linux), Metal auf dem Mac (M4 Pro). Schalter `--gpu auto|an|aus` an `map` und
`surface`, Standard `auto`. Ohne taugliche Grafikkarte rechnet alles wie vorher auf der CPU.

## Kurzfassung

- **Einfärben der Karte:** RTX 5080 **9,9×** schneller als die CPU derselben Maschine (schwarzer Raum
  516 s → 52 s), M4 Pro **5,7×** (315 s → 55 s). Ziel (5× RTX, 2× Mac) erreicht.
- **Oberfläche, Ansichten:** RTX **5,7×** (schwarzer Raum 250 s → 44 s), Büro etwa **12×**
  (860 s → 71 s). **Oberfläche gesamt:** schwarzer Raum nur **2,5–3,5×** (199–287 s → 79–81 s),
  Büro etwa 5× (987 s → 198 s). Ziel 5× für die ganze Oberfläche im schwarzen Raum **nicht erreicht**:
  Die Grafikkarte wartet jetzt fast nur noch auf das Dekodieren der JPEG-Bilder (siehe Grenzen).
- **Ergebnis gleichwertig, nicht byte-gleich:** dieselbe beste Ansicht für 99,8 % der Dreiecke,
  Kartenfarben 99,7 % der Punkte innerhalb ±5 sRGB-Stufen. **Fertige Texturseiten nur 96,7 %
  innerhalb ±5, mittleres |ΔY| 1,17:** Das vorgeschlagene Kriterium (99 %, ≤ 1) ist dort verfehlt.
  Ursache ist **nicht** der Belichtungsausgleich (seit 25.10. stabil, Oberflächen-Version 9), sondern
  die Lage der Karten auf den Seiten: Fast jede Karte liegt bei GPU und CPU an anderer Stelle, und die
  JPEG-Kodierung rundet dann anders (siehe unten). Entscheidung 25.10.: Das Kriterium gilt je
  Dreieck vor dem Packen der Seiten; dort stimmen die Farben zu 99,999 % überein.
- **`--gpu aus` war byte-gleich zum Stand vor dem Baustein** (Karte und Oberfläche, echte Daten, bis
  24.10.). Seit 25.10. ändert die stabile Belichtungsstichprobe die Oberfläche auch auf der CPU
  (Oberflächen-Version 9, ältere Ordner gelten als veraltet); die Karte bleibt unverändert.
- **Grafikspeicher:** Oberfläche schwarzer Raum 525 MiB, Büro 973 MiB; weit unter der Grenze 4 GB.
- **TSDF bleibt auf der CPU:** Anteil an der Oberflächenzeit 1,7 % (Schwelle laut Spec 20 %).

## Messung

VM 101: RTX 5080 (16 GB), NVIDIA-Treiber 580.173.02, Vulkan; CPU-Seite mit `RAYON_NUM_THREADS=6`
(halbe Last, Regel PVE Privat), GPU-Läufe über `gpu-job`. Mac: M4 Pro, Metal. Release-Build. Daten:
schwarzer Raum `jmk6-2026-10-22/20261022133554` (2.145 genutzte Bilder), Büro
`jmk6-2026-10-20/20261020173822`. Oberfläche mit `--voxel 0.02`-Standard des Bausteins 2
(`voxel 0.02`). Logs und Berichte: `driftless-testdata/ergebnisse-gpu/{gpu-color,surface,t5,expo}/`.

```sh
S=~/td/jmk6-2026-10-22/20261022133554
gpu-job --name … -- driftless map $S --slam <ordner> --step color --gpu an
gpu-job --name … -- driftless surface $S --slam $S/driftless -o <ziel> --gpu an
driftless surface $S --slam $S/driftless -o <ziel> --gpu aus
```

### Zeiten je Gerät

| Schritt | Aufnahme | CPU | GPU | Faktor |
|---|---|---|---|---|
| Einfärben (`--step color`) | schwarzer Raum, VM 101 | 516 s (6 Threads) | **52 s** (RTX 5080) | **9,9×** |
| Einfärben | schwarzer Raum, Mac | 315 s | **55 s** (M4 Pro) | **5,7×** |
| Einfärben | Büro, VM 101 | – | 88 s (RTX 5080) | – |
| Oberfläche, Ansichten | schwarzer Raum, VM 101 | 250 s (6 Thr.) | **44 s** | **5,7×** |
| Oberfläche, Bilddurchgang darin | schwarzer Raum, VM 101 | 226 s | 20 s (davon 18,8 s Warten auf Bilder) | 11,3× |
| Oberfläche gesamt | schwarzer Raum, VM 101 | 287 s (6 Thr.) / 199 s (12 Thr.) | 81 s / 79 s | 3,5× / 2,5× |
| Oberfläche, Ansichten | Büro, VM 101 | 860 s (6 Thr., früherer Lauf) | **71 s** | ≈ 12× |
| Oberfläche gesamt | Büro, VM 101 | 987 s (früherer Lauf) | 198 s | ≈ 5× |

Die Büro-CPU-Zahlen stammen aus einem früheren Lauf (gleicher Code der CPU-Seite), nicht vom selben
Tag. Die Oberfläche auf dem Mac (Metal) wurde nur in den Tests gerechnet, nicht auf echten Daten.

Nachlauf 24.10. (Stand `8434484`, für den Seitenvergleich): GPU 87,6 s (Ansichten 48,8 s), CPU mit 4
Threads 383 s, beide neben einem anderen Rechenjob auf der VM, daher nicht für die Zeiten genutzt. Die
CPU-Ausgabe (glb, obj, mtl, 6 Seiten) ist **byte-gleich** mit `oberflaeche-fixB2` vom 23.10.: Die
Umstellungen dieses Bausteins an der CPU-Seite (Zähltabelle der Verdeckungsmaske, Seiten nur einmal
kodiert) ändern nichts am Ergebnis.

### Speicher

| Lauf | Spitzen-RAM | Grafikspeicher |
|---|---|---|
| Einfärben schwarzer Raum, RTX | 1,47 GB (CPU 1,64 GB) | ≈ 0,8 GB (berechnet) |
| Einfärben Büro, RTX | 2,69 GB | – |
| Oberfläche schwarzer Raum, RTX | 1,21 GB (CPU 0,90 GB) | 525 MiB (gemessen, `nvidia-smi`) |
| Oberfläche Büro, RTX | 2,81 GB (CPU 2,51 GB) | 973 MiB |

Das Budget wird vor dem Hochladen geprüft (höchstens 4 GB, `DRIFTLESS_GPU_BUDGET` senkt es). Darüber
rechnet `auto` auf der CPU und nennt den Grund, `an` bricht mit „… Grafikspeicher …“ ab.

## Abweichungen GPU gegen CPU

Die GPU rechnet in `f32`, die CPU in `f64`. Gleiche Eingabe auf **demselben Gerät** gibt gleiche
Dateien (geprüft: zweiter GPU-Lauf identisch).

### Karte: Farben je Punkt (Mac, schwarzer Raum)

| ≤ 0 | ≤ 1 | ≤ 2 | ≤ 5 | ≤ 10 | mittleres \|ΔY\| |
|---|---|---|---|---|---|
| 43,4 % | 90,6 % | 96,2 % | **99,7 %** | 99,97 % | **0,41** |

Die gewählten Ansichten sind praktisch gleich. Die Abweichung kommt aus dem Belichtungsausgleich:
Rundungsunterschiede verschieben, welche Punkte in die Belichtungsstichprobe kommen; die Verstärkung
je Bild ändert sich um etwa 0,3 % und verschiebt ganze Bilder um 1–3 Stufen.

### Oberfläche: Ansichten je Dreieck (RTX, schwarzer Raum, 700.866 Dreiecke mit Bild)

| gleiche beste Ansicht | Bewertung der besten Ansicht > 1 % daneben | Farbe der besten Ansicht > 1 Stufe daneben | gleiche Ansichtsmenge | Jaccard im Mittel |
|---|---|---|---|---|
| **99,816 %** | 0,018 % (125 Dreiecke) | 0 | 99,672 % | 0,9992 |

„Bewertungen ± 1 %“ heißt hier und im Test `views_gpu_match_cpu`: **höchstens 1 % der Dreiecke liegen
außerhalb von ± 1 %**, nicht: jedes Dreieck liegt innerhalb.

**Ausreißer der Bewertung (125 Dreiecke, schlimmster 93 %):** Die Bewertung ist stetig in allen
Größen (projizierte Fläche, Deckel bei `min_texel`, Begrenzung `cos_s/cos_n` auf ½…2), ein Sprung über
eine Schwelle erklärt 93 % nicht. Wahrscheinliche Ursache sind streifend gesehene Dreiecke: Ihre
projizierte Fläche `½·|d1 × d2|` ist fast null und entsteht in `f32` als Differenz fast gleicher
Produkte (Auslöschung), dazu ist `cos_n` klein. Bei solchen Dreiecken ist der relative Fehler groß, der
Betrag klein; die beste Ansicht bleibt dieselbe. Nicht behoben: Das ginge nur mit `f64` oder einer
anderen Flächenformel im Shader; der Nutzen ist bei 0,018 % der Dreiecke ohne andere beste Ansicht
gering. Nicht einzeln nachgeprüft (dafür bräuchte es wieder einen Mitschnitt der Ansichtstabelle).

**Belichtung (bis 24.10., Version 8):** Die Verstärkungen je Bild wichen bis 4,7 % ab (größter
Unterschied über alle Bilder und Kanäle), aus demselben Grund wie bei der Karte.

**Belichtung ab 25.10. (Version 9, Stand `27d21c6`):** Die Stichprobe für den Belichtungsausgleich
hängt nicht mehr an knappen Bewertungsunterschieden:
- Je Stichproben-Dreieck gewinnen die Ansichten der besten halben Oktave der Bewertung
  (`⌊2·log₂ Bewertung⌋`, exakt aus den Bits). Innerhalb einer halben Oktave entscheidet ein Hash aus
  Dreieck und Bild.
- Es gibt höchstens so viele Stichproben-Dreiecke, wie der Ausgleich nimmt (50.000). Vorher nahm er
  jedes 4. von bis zu 200.000, und ein Dreieck mehr oder weniger verschob die ganze Auswahl.

Gemessen an der Ansichtstabelle (Mitschnitt, nicht eingecheckt), CPU gegen RTX:

| | Version 8 | nur Hash (verworfen) | **Version 9** |
|---|---|---|---|
| Stichproben-Dreiecke mit gleichen Bildern | – | 99,996 % | **99,79 %** |
| Verstärkung je Bild: Median / p99 / größter Unterschied | – / – / 4,7 % | 0,0009 / 0,03 / 1,6 % | **0,04 / 0,55 / 2,0 %** |
| `seam_step_after` (CPU) | 20,43 | 21,45 | **20,34** |
| `seam_step_before` (CPU) | 26,15 | 27,71 | **26,26** |
| `seam_jump_after` (CPU) | 12,86 | 13,65 | **12,83** |
| Dreiecke ohne Bild | 6.030 | 6.030 | **6.030** |

Nur der Hash, ohne Bewertung, hätte die Verstärkungen fast gleich gemacht, ließ aber streifende und
ferne Ansichten in den Ausgleich; die Nähte wurden 5–6 % schlechter. Mit der halben Oktave sind die
Nähte so gut wie vorher.

### Oberfläche: fertige Texturseiten (RTX, schwarzer Raum)

Bis 24.10. (Version 8) verglichen: `oberflaeche.glb` des CPU-Laufs (`oberflaeche-fixB2`, byte-gleich
mit `--gpu aus` am 24.10.) gegen den GPU-Lauf vom 24.10. (Stand `8434484`). Die Dreiecke sind in beiden gleich (706.896, alle
über ihre Eckpunkte zugeordnet); die Atlanten unterscheiden sich (54.217 gegen 54.191 Karten). Je
Dreieck 10 innere Punkte (weg vom Kartenrand), dort das Texel beider Seiten (nächstes Texel);
gewichtet mit der Texelfläche des Dreiecks, also etwa Anteile der Texel. Skript:
`driftless-testdata/ergebnisse-gpu/t5/cmp_pages.py`.

| innerhalb ±0 | ±1 | ±2 | **±5** | ±10 | **mittleres \|ΔY\|** | mittleres ΔY (GPU − CPU) |
|---|---|---|---|---|---|---|
| 18,1 % | 45,7 % | 72,8 % | **96,8 %** | 99,6 % | **1,16** | +0,01 |
| Version 9 (25.10., `27d21c6`) | | | | | | |
| 17,7 % | 45,1 % | 72,5 % | **96,7 %** | 99,6 % | **1,17** | −0,00 |
| Version 9 ohne Nahtausgleich (Versuch) | | | | | | |
| 19,1 % | 47,0 % | 73,8 % | **97,1 %** | 99,7 % | **1,14** | +0,00 |

- **Das vorgeschlagene Kriterium (≥ 99 % innerhalb ±5, |ΔY| ≤ 1) ist für die fertigen Seiten nicht
  erfüllt** (96,8 %, 1,16). Für die Kartenfarben ist es erfüllt (99,7 %, 0,41).
- **Kein Versatz in eine Richtung:** Im Mittel ist die GPU-Seite um 0,01 Stufen heller, die
  Abweichungen streuen in beide Richtungen.
- **Nicht der Atlas:** Auf den 83 % der Dreiecke, deren Texelraster in beiden Atlanten gleich liegt,
  sind es 97,6 % innerhalb ±5 und |ΔY| 1,09, fast dasselbe.
- **Die Belichtung ist es nicht** (Prüfung 25.10.): Mit stabiler Stichprobe (Version 9) bleibt das
  Ergebnis gleich (96,7 %, 1,17); auch mit fast gleichen Verstärkungen (nur Hash: Median 0,0009 %)
  waren es 96,3 % und 1,21. Ohne Nahtausgleich sind es 97,1 % und 1,14, also auch nicht der
  Nahtausgleich. Die Bildwahl je Dreieck (Labels) unterscheidet sich nur bei 0,26 % der Dreiecke.
- **Ursache ist die Lage der Karten auf den Seiten.** Kleine Unterschiede in Labels und Karten ändern
  Größen von Karten. Die Packung (Regale, nach Höhe sortiert) schiebt dann fast alle folgenden Karten
  an eine andere Stelle. An anderer Stelle tastet das Texelraster das Foto anders ab, und JPEG rundet
  in anderen 8×8-Blöcken. Aufgeteilt nach Lage (Stand `27d21c6`, Skript
  `driftless-testdata/ergebnisse-gpu/expo/cmp_pos.py`):

  | Dreiecke | Anteil | innerhalb ±5 | mittleres \|ΔY\| |
  |---|---|---|---|
  | gleiche Stelle auf der Seite | 1,2 % | **99,999 %** | **0,14** |
  | verschoben um ein Vielfaches von 16 Texeln (gleiche JPEG-Blöcke) | 0,4 % | 99,7 % | 0,24 |
  | sonst verschoben | 98,4 % | 96,7 % | 1,19 |

  Gegenprobe: Eine schon kodierte Seite um 3×5 Pixel verschoben und wieder kodiert gibt allein schon
  98,8 % innerhalb ±5 und |ΔY| 0,69.
- **Was helfen würde:** Wo die Karten liegen, müsste unabhängig von kleinen Größenänderungen sein,
  oder das Kriterium für die Seiten misst die Farben vor der Packung (z. B. je Dreieck den Mittelwert
  statt des Texels). **Yannick entscheidet**, ob das Kriterium für die Seiten gelockert oder die
  Packung stabilisiert wird.

## Grenzen

- **JPEG-Dekodieren ist jetzt der Flaschenhals.** Die GPU rechnet die 2.145 Bilder des schwarzen Raums
  in etwa 1,3 s, wartet aber 94 % des Bilddurchgangs auf dekodierte Bilder (Büro 88 %). Übrig von den
  Ansichten sind zwei Dekodierdurchgänge (Schärfe und Bilder). Nicht geändert: Yannick hat entschieden,
  dass Qualität vor Geschwindigkeit geht. Der nächste Hebel wäre ein gemeinsamer Dekodierdurchgang oder
  mehr Dekodier-Threads.
- **TSDF auf der CPU:** 1,39 s von 81 s (schwarzer Raum, 1,7 %), 3,29 s von 198 s (Büro, 1,7 %). Unter
  der Schwelle von 20 % aus der Spec, daher kein `tsdf.rs` auf der GPU.
- **Kein Ausschneiden nach Sichtkegel:** Jedes Dreieck wird gegen jedes Bild geprüft (nur eine grobe
  Entfernungsgrenze). Bei 4,3 Mio. Dreiecken kostet das etwa 1 s GPU-Zeit, für viel größere Scans wäre
  es der nächste Schritt.
- **Über 4 GB keine Stapelung:** Ein Scan über dem Budget rechnet unter `auto` auf der CPU.
- **Fehler während des Laufs:** Fällt die Grafikkarte mitten im Lauf aus (z. B. Win11-VM übernimmt sie),
  bricht der Schritt mit einer deutschen Meldung ab, auch unter `auto`; kein Wechsel auf die CPU mitten im
  Lauf. Vor dem ersten Bild (Gerät, Budget, Hochladen) fällt `auto` auf die CPU zurück; der Grund steht
  dann in `map.json` (`steps.color.gpu.fallback`) bzw. `oberflaeche.json` (`gpu.fallback`).
- **Im Programm** gibt es den Schalter noch nicht; er kommt mit dem neuen Einstellungsfenster
  (Baustein 4). Bis dahin nur `--gpu` in der Befehlszeile; das Programm rechnet mit `auto`.
- **CI** hat keine Grafikkarte: Die GPU-Tests überspringen sich dort, die CPU-Tests bleiben Pflicht.

## Entscheidungen, die noch offen sind

- **Kriterium für echte Daten** (Vorschlag, von Yannick zu bestätigen): ≥ 99 % innerhalb ±5 sRGB und
  mittleres |ΔY| ≤ 1, statt ±2 wie in den synthetischen Tests. Für die fertigen Seiten auch nach der
  stabilen Belichtungsstichprobe verfehlt (96,7 %, 1,17), Ursache die Packung der Karten (siehe oben).
- **Gesamtziel 5× für die Oberfläche** im schwarzen Raum: erreicht nur mit schnellerem Dekodieren.

## Programm

In ⚙ steht „Grafikkarte nutzen“ mit Auto · An · Aus (Standard Auto). Die Wahl gilt für Karte und
Oberfläche, auch für „Alle rechnen“, und bleibt über einen Neustart erhalten.
