# Ergebnisse Meilenstein 3 – dicht, bereinigt, eingefärbt

Stand: 2026-10-21 · gemessen auf dem MacBook Pro M4 Pro (14 Kerne, 48 GB), Release-Build

## Kurzfassung

- `driftless slam` + `driftless map` laufen auf allen drei JMK6-Aufnahmen durch, zwei volle Läufe
  sind bitgleich (`traj.tum`, `dense.las`, `clean.las`, `removed.las`, `color.las`, `color.ply`).
- **Zeit- und Speicherziel eingehalten:** Testscan `Test` (266 s, 1.741 Bilder) mit den neuen
  Standardwerten in **140 s** (SLAM 9 s + Karte 131 s), Spitzen-RAM **4,0 GB**. Ziel war < 15 min
  und < 8 GB.
- **Voxelgröße der dichten Wolke von 2 cm auf 1 cm gesenkt** (einzige geänderte Voreinstellung).
  Bei 2 cm sehen Nahansichten sichtbar löchrig aus. Mit 1 cm sind Pflanze, Tapetenstreifen und
  Kissen erkennbar. Kosten auf `Test`: 2,4× so viele Punkte, Zeit 84 → 140 s, RAM 1,4 → 4,0 GB.
- **Eingefärbt** werden 98,4 % (`Test`) und 99,2–99,3 % (JMK6-Aufnahmen) der Punkte.
- **Belichtungsabgleich:** Die neue Kennzahl „Kamera-Unterschied“ reagiert auf Belichtung und
  Weißabgleich. Der Abgleich senkt sie auf `Test` um 46 % (1,42 → 0,76 sRGB-Stufen) und auf den
  JMK6-Aufnahmen um 57–64 %. Der „Bild-Versatz“ (Helligkeit je Bild über die Zeit) sinkt
  um 44–55 %. Die alte Streuung sinkt nur um 4–10 %.
- **Die extremen Belichtungsfaktoren auf `Test` (0,28–3,98) sind echt.** Sie gehören zu sehr
  dunklen bzw. sehr hellen Bildern, also zur Belichtungsautomatik der Kamera. Ein Fehler der
  Anpassung ist das nicht. Der Grenzwert 4 wird aber in 6 von 1.677 Bildern erreicht.
- **RayStudio `smooth.las` hat keine Farben.** Alle 12,5 Mio. Punkte tragen dieselbe Farbe
  (0, 166, 252), die Intensität ist 0. Verglichen werden konnten deshalb nur Lage und Dichte,
  nicht die Farbe.

## Messung

Befehle (Ausgaben im Testdaten-Ordner bzw. einem Arbeitsordner, nicht im Repo):

```sh
driftless slam <scan> -o <ziel>
driftless map <scan> --slam <ziel> [--voxel 0.02|0.015|0.01]
```

Laufzeit und RAM stehen in `map.json` (`seconds` je Schritt, `peak_rss_bytes`). Gegengeprüft wurde
mit `/usr/bin/time -l`. Alle Zahlen stammen aus je einem Lauf, ohne andere Last.

### Laufzeit, Speicher, Punkte

| Aufnahme | Voxel | Punkte dicht | behalten | Dicht | Bereinigen | Einfärben | Karte gesamt | SLAM | Spitzen-RAM |
|---|---|---|---|---|---|---|---|---|---|
| `Test` (266 s) | 2 cm | 2.892.160 | 2.750.567 | 8,4 s | 1,2 s | 65,4 s | 74,9 s | 9,4 s | 1,41 GB |
| `Test` | 1,5 cm | 4.407.174 | 4.228.800 | 9,4 s | 2,0 s | 83,1 s | 94,5 s | 9,4 s | 2,51 GB |
| `Test` | **1 cm** | 7.189.956 | 6.955.244 | 9,1 s | 3,5 s | 118,1 s | **130,7 s** | 9,4 s | **3,99 GB** |
| `20261020143415` (63 s) | 2 cm | 517.025 | 484.977 | 1,1 s | 0,2 s | 9,6 s | 10,9 s | 0,9 s | 0,41 GB |
| `20261020143415` | **1 cm** | 1.425.860 | 1.372.747 | 1,2 s | 0,6 s | 13,4 s | **15,3 s** | 0,9 s | **0,86 GB** |
| `20261020143623` (108 s) | 2 cm | 584.209 | 557.708 | 1,7 s | 0,3 s | 17,3 s | 19,3 s | 1,5 s | 0,42 GB |
| `20261020143623` | 1,5 cm | 965.949 | 929.014 | 1,9 s | 0,4 s | 19,4 s | 21,7 s | 1,5 s | 0,58 GB |
| `20261020143623` | **1 cm** | 1.825.946 | 1.771.778 | 2,0 s | 0,7 s | 25,6 s | **28,4 s** | 1,5 s | **1,06 GB** |

Der Speicher wird vor allem beim Einfärben gebraucht: Bei `Test` stehen nach dem Bereinigen
1,23 GB, nach dem Einfärben 3,99 GB (1 cm). Das sind etwa 0,55 kB je Punkt. Die Bilder werden
dabei einzeln gestreamt, wie verlangt. Was mit der Punktzahl wächst, sind die Ansichten je Punkt,
die Normalen, der Suchbaum und das Raster der Sichtbarkeitsprüfung.

Dateigrößen `Test` bei 1 cm (2 cm): `dense.las` 201 MB (81), `dense.bin` 239 MB (119),
`clean.las` 195 MB (77), `clean.bin` 195 MB (77), `color.las` 236 MB (94), `color.ply` 132 MB (52),
`removed.las` 7 MB (4).

### Einfärben und Belichtung

| Aufnahme | Voxel | eingefärbt | Bilder genutzt / unscharf | Faktoren min–max | Streuung vorher → nachher | Kamera-Unterschied vorher → nachher (Punkte) | Bild-Versatz vorher → nachher |
|---|---|---|---|---|---|---|---|
| `Test` | 2 cm | 95,2 % | 1.677 / 64 | 0,28–3,06 | 6,52 → 6,34 | 0,92 → 0,19 (358.501) | 0,94 → 0,57 |
| `Test` | 1,5 cm | 97,0 % | 1.677 / 64 | 0,34–3,12 | 6,42 → 6,17 | 1,13 → 0,79 (547.472) | 0,97 → 0,57 |
| `Test` | **1 cm** | **98,4 %** | 1.677 / 64 | 0,28–3,98 | 6,24 → 5,97 | **1,42 → 0,76** (865.815) | **1,01 → 0,57** |
| `143415` | 2 cm | 96,8 % | 368 / 13 | 0,57–2,71 | 7,50 → 7,17 | 7,40 → 1,77 (21.858) | 0,82 → 0,43 |
| `143415` | **1 cm** | **99,2 %** | 368 / 13 | 0,45–1,90 | 6,76 → 6,37 | **8,22 → 2,98** (54.191) | **0,85 → 0,43** |
| `143623` | 2 cm | 96,3 % | 657 / 25 | 0,56–1,90 | 7,82 → 7,39 | 5,69 → 2,21 (29.764) | 0,93 → 0,47 |
| `143623` | **1 cm** | **99,3 %** | 657 / 25 | 0,50–1,85 | 6,83 → 6,21 | **7,26 → 3,12** (71.835) | **0,93 → 0,41** |

Alle Farbwerte sind sRGB-Stufen (0–255), Mittel über die drei Kanäle.

- **Streuung** (bisherige Kennzahl): Standardabweichung der Ansichten eines Punktes. Sie wird von
  Rauschen beherrscht (Kanten, Glanz, kleine Lagefehler) und reagiert kaum auf den Abgleich
  (−4 bis −10 %).
- **Kamera-Unterschied** (neu, `camera_difference_*` in `ColorStats` und `map.json`): Gemessen
  auf den Punkten, die beide Kameras sehen. Je Punkt wird die mittlere Farbe der linken minus die
  der rechten Kamera gebildet und dann über alle diese Punkte gemittelt. Das Rauschen mittelt sich
  heraus, ein Farbstich oder Helligkeitsunterschied einer Kamera bleibt stehen. Im synthetischen
  Test (rechte Kamera 1,3× heller und bläulich) fällt er von 17,3 auf 0,15.
- **Bild-Versatz** (neu, `image_offset_*`): Je Bild wird gemittelt, wie weit seine Ansichten vom
  Mittel des jeweiligen Punktes abweichen. Das erfasst auch Belichtungswechsel über die Zeit
  innerhalb einer Kamera (früh/spät). Der größte Teil der Faktoren auf `Test` ändert sich über die
  Zeit, nicht zwischen den Kameras (siehe unten).
- Auf `Test` ist der Kamera-Unterschied schon vorher klein (≈ 1 Stufe). Die beiden Kameras
  belichten dort fast gleich: Der Median der Faktoren liegt links bei 0,97 und rechts bei 0,82.
  Bei den JMK6-Aufnahmen ist er 5–8 Stufen. Das ist der Farbstich, der in Aufgabe 6 zu sehen war.
- Bei 1 cm sinkt der Kamera-Unterschied auf `Test` weniger stark als bei 2 cm (−46 % statt −79 %).
  Mehr Punkte in Ecken und an Kanten werden von beiden Kameras gesehen. Nicht untersucht ist, ob
  das an der Blickwinkel-Abhängigkeit der Oberflächen liegt oder an Restfehlern der Lage.

### Sind die extremen Faktoren auf `Test` echt?

Geprüft mit einem Wegwerf-Programm (nicht im Repo): Es hat die Faktoren je Bild zusammen mit der
mittleren Helligkeit und dem Anteil fast schwarzer bzw. weißer Pixel des Bildes ausgegeben.

| | Wert |
|---|---|
| Korrelation log(Faktor) ↔ log(mittlere Bildhelligkeit) | 0,75 |
| Faktor (geometrisches Mittel der Kanäle): 1 % / Median / 99 % | 0,41 / 0,89 / 2,91 |
| Bilder mit Faktor < 0,5 / > 2 | 95 / 318 |
| Bilder mit einem Kanal am Grenzwert (1/4 oder 4) | 6 von 1.677 (alle grün, linke Kamera) |
| Median der Faktoren je 15 s, erste 60 s | 2,1–2,6 |
| Median der Faktoren je 15 s, ab 60 s | 0,5–1,3 |
| größter Sprung zwischen zwei Bildern einer Kamera (log) | 0,67 links, 0,39 rechts |

- Die kleinsten Faktoren (≈ 0,25, 149–151 s) gehören zu Bildern eines dunklen Großraumbüros am
  Abend: mittlere Helligkeit 23–29 von 255, 41–51 % fast schwarze Pixel, die Fenster sind dunkel.
- Die größten Faktoren (≈ 3,2–3,3, 18–44 s) gehören zu Bildern eines kleinen weißen Büros direkt
  unter der Deckenleuchte: mittlere Helligkeit 97–118, weiße Wände. Im Bild ist außerdem der Arm
  des Bedieners zu sehen.
- Die Faktoren folgen also der Belichtungsautomatik von Raum zu Raum und sind keine Panne der
  Anpassung. Der Grenzwert `max_gain = 4` wird im grünen Kanal von 6 Bildern erreicht; dort wäre
  der wahre Faktor wohl noch kleiner. Geändert wurde nichts: 6 Bilder rechtfertigen keinen
  höheren Grenzwert, der Fehlanpassungen mehr Spielraum gäbe.

### Dichte und Aussehen: 2 cm, 1,5 cm, 1 cm

Renderings: siehe Abschnitt „Bilder“. Der Punktabstand wurde in einem gemeinsamen Ausschnitt
geschätzt (Laufbahn ± 1 m waagrecht, ± 1,5 m in der Höhe): Punkte je belegtem 5-cm-Voxel, daraus
`Abstand ≈ 5 cm / √(Punkte je Voxel)`. Die Bildfüllung ist der Anteil getroffener Pixel einer
Innenansicht (1000 × 700 px, 80°, Punkte 1 px groß).

| Aufnahme | Wolke | Punkte im Ausschnitt | Abstand | Bildfüllung innen 1 / 2 |
|---|---|---|---|---|
| `Test` | Driftless 2 cm | 2,12 Mio. | ≈ 20 mm | 8,7 % / 6,9 % |
| `Test` | Driftless 1,5 cm | 3,45 Mio. | ≈ 16 mm | 12,1 % / 9,2 % |
| `Test` | Driftless 1 cm | 6,02 Mio. | ≈ 12 mm | 17,3 % / 11,3 % |
| `Test` | RayStudio `smooth.las` | 11,33 Mio. | ≈ 8 mm | 23,6 % / 12,9 % |
| `143623` | Driftless 2 cm / 1,5 cm / 1 cm | 0,51 / 0,88 / 1,72 Mio. | ≈ 19 / 15 / 11 mm | 5,5 / 10,0 / 20,6 % · 12,6 / 16,6 / 21,6 % |
| `143415` | Driftless 2 cm / 1 cm | 0,42 / 1,28 Mio. | ≈ 19 / 11 mm | 6,3 / 17,1 % · 7,3 / 16,4 % |

RayStudio ist im gemeinsamen Ausschnitt fast doppelt so dicht wie Driftless bei 1 cm, belegt aber
weniger 5-cm-Voxel (302 Tsd. gegenüber 339 Tsd.). Die RayStudio-Wolke ist also dünner (geglättet,
der Dateiname sagt „smooth“), unsere Wände sind dicker. Wie viel davon Rauschen ist, wurde nicht
gemessen.

**Entscheidung: `DenseConfig.voxel` 0,02 → 0,01 m.**

- Deutlich besser im Bild: Bei `143623`, Innenansicht 1 ist die Zimmerpflanze bei 2 cm fast
  unsichtbar, bei 1 cm klar zu erkennen. Ebenso die Streifen der Tapete und der Fernseher mit
  Spiegelungen. Innenansicht 2 zeigt bei 1 cm Kissen, Blumentopf und Lamellen des Fensters
  geschlossen, bei 2 cm nur als Punktewolke. Die Bildfüllung steigt auf das 1,7- bis 3,7-Fache.
- 1,5 cm liegt sichtbar dazwischen und bringt nicht den Sprung, der bei 1 cm zu sehen ist.
- Ziel bleibt klar eingehalten: `Test` 140 s statt 15 min, 4,0 GB statt 8 GB.
- **Preis:** Speicher und Dateien etwa 2,5×, Einfärben 1,8× so lang. Ein Scan, der doppelt so lang
  ist wie `Test` (≈ 9 min in denselben Räumen), käme hochgerechnet auf ≈ 8 GB. Für solche Scans
  gibt es `--voxel 0.02` (RAM 1,4 GB auf `Test`).
- Mehr Punkte werden eingefärbt (95,2 → 98,4 % auf `Test`). Bei 1 cm kann ein Punkt seinen
  Nachbarn weniger verdecken, und die Sichtbarkeitsprüfung schließt weniger aus.

Sonst wurde keine Voreinstellung geändert.

### Geister und Ausreißer

| Aufnahme (1 cm) | Geister | Ausreißer | entfernt gesamt | davon ≤ 1 m vom Gerät zur Aufnahmezeit | davon > 5 m von der Laufbahn |
|---|---|---|---|---|---|
| `Test` | 128.029 (1,78 %) | 106.683 (1,48 %) | 3,3 % | 6 % (behaltene Punkte: 27 %) | 17 % |
| `143415` | 39.367 (2,76 %) | 13.746 (0,96 %) | 3,7 % | 62 % (behaltene: 38 %) | 3 % |
| `143623` | 31.872 (1,75 %) | 22.296 (1,22 %) | 3,0 % | 41 % (behaltene: 45 %) | 4 % |

Was entfernt wird, nach den Renderings von oben (`*_entfernt.png`: entfernte Punkte rot über der
bereinigten Wolke, Laufbahn blau):

- **`143415`: Spur des Bedieners.** Ein geschlossener roter Ring folgt der Laufbahn rund um den
  Tisch: der Mensch, der den Scanner trägt, gesehen aus der Gegenrichtung. Dazu kommen Punkte
  draußen vor dem Fenster (dünn verteilt, Bäume/Fassade durch das Glas) und Kanten der Stühle am
  Tisch. 62 % der entfernten Punkte lagen bei der Aufnahme höchstens 1 m vom Gerät weg, bei den
  behaltenen nur 38 %.
- **`143623`:** Die Bedienerspur ist nur auf einem Teil der Runde zu sehen, dazu wieder die Punkte
  vor dem Fenster und Stuhlkanten. Nach Abstand zum Gerät sind entfernte und behaltene Punkte
  nicht zu unterscheiden (41 % gegenüber 45 %).
- **`Test`:** Keine Bedienerspur zu erkennen (nur 6 % ≤ 1 m vom Gerät). Entfernt werden vor
  allem Punkte draußen vor den Fenstern (17 % liegen mehr als 5 m von der Laufbahn weg), Kanten
  von Türen, Schränken und Tischen und eine größere Gruppe in der Mitte des Großraums an einer
  Pflanzengruppe. Ob Türen während des Scans bewegt wurden, ist nicht bekannt. Bei den Pflanzen
  und Möbelkanten wird **wohl auch Feststehendes entfernt** (Blätter, schräg getroffene Kanten).
  Das bleibt in `removed.las` erhalten und kann mit `--no-ghosts` / `--no-outliers` abgeschaltet
  werden.
- Personen außer dem Bediener waren in keiner Aufnahme zu erkennen.

## Vergleich mit RayStudio (`Test`)

- **Rahmen:** RayStudio `trj.txt` und unsere `traj.tum` liegen im selben Rahmen. Die starre
  Ausrichtung (Umeyama, 2.658 Zeitpunkte) dreht nur um 0,39° und verschiebt um 2 cm, mit einem
  Restfehler von 1,8 cm. Das ist die ATE aus Meilenstein 2. Die RayStudio-Wolke wurde für die
  Renderings so in unseren Rahmen gebracht. Die Grundrisse von oben decken sich.
- **Farbe:** nicht vergleichbar, `smooth.las` ist einfarbig (siehe oben).
- **Punktzahl:** RayStudio 12,5 Mio. Punkte, davon 11,3 Mio. im gemeinsamen Ausschnitt. Driftless
  bei 1 cm hat 6,96 Mio. Punkte (6,0 Mio. im Ausschnitt). RayStudio reicht weiter nach draußen
  (bis ≈ 41 m vom Start, wir: höchstens 30 m Messweite).
- **Dichte:** RayStudio ≈ 8 mm Punktabstand, Driftless 1 cm ≈ 12 mm. Die Wände von RayStudio
  sind dünner (siehe oben).
- **Der Vergleich ist keine Wahrheit:** RayStudio ist die Software des Herstellers, kein
  Messnormal. „Dichter“ heißt nicht „genauer“, und eine geglättete Wolke kann Einzelheiten
  verwischen.

## Bilder

Lokal unter `$DRIFTLESS_TESTDATA/ergebnisse-m3/` (nicht im Repo). Gerendert mit einem
Wegwerf-Skript: Z-Puffer, feste Punktgröße 2 px (Dichte-Bilder 1 px, alle Punkte weiß),
Innenansichten waagrecht in Laufrichtung, 2 m vor der nächsten Fläche.

| Datei | Inhalt |
|---|---|
| `<scan>_oben.png` | Grundriss von oben ohne Decke (Höhe < Gerät + 0,6 m); 2 cm, 1,5 cm, 1 cm, bei `Test` RayStudio (grau nach Höhe) |
| `<scan>_innen1.png`, `_innen2.png` | zwei Innenansichten nebeneinander, in Farbe |
| `<scan>_innen1_dichte.png`, `_innen2_dichte.png` | dieselben Ansichten, nur Punktdichte |
| `<scan>_entfernt.png` | entfernte Punkte (rot), Laufbahn (blau), 1 cm |
| `Test_belichtung_extrem.png` | Kamerabild mit dem kleinsten und dem größten Belichtungsfaktor |

Was die Bilder zeigen:

- **Von oben (`Test`):** Die Grundrisse von Driftless und RayStudio decken sich. Die Möbel stehen
  an denselben Stellen, Türen und Wanddurchbrüche liegen gleich. Bei Driftless sieht man auf dem
  Boden des Großraums harte Farbgrenzen. Ein bräunlicher Keil am Eingang stößt gerade an einen
  blaugrauen Bereich. Dort stammen benachbarte Flächen aus verschiedenen Bildern mit
  unterschiedlichem Licht, und der Abgleich gleicht das nicht ganz aus. Bei 1 cm wirkt der
  Boden dunkler und gleichmäßiger.
- **Innen (`Test`):** Tür mit Rahmen, ein Kasten neben der Tür und die Reihe dunkler Felder an der
  Wand (Ansicht 2) liegen bei allen Voxelgrößen an derselben Stelle wie bei RayStudio. Die Szene ist dunkel (abends
  aufgenommen), deshalb sind die Farben gedämpft. Bei 2 cm ist die Wand gerade noch als Fläche zu
  lesen, bei 1 cm geschlossen.
- **Innen (`143623`):** siehe Begründung der Voxelgröße. Bei 2 cm wirkt die Pflanze fast schwarz
  und löchrig, bei 1 cm sind die Blätter grün und zusammenhängend. Die Farben stimmen in beiden
  Fällen: Fensterrahmen blau, Pflanze grün, Kissen dunkelgrün. Nichts färbt auf Nachbarflächen ab.
- **Dichte:** RayStudio ist auch bei 1 px am vollsten. Driftless bei 1 cm kommt nahe heran, 2 cm
  ist deutlich dünner.

## Tests

`crates/driftless-map/tests/real_data.rs`, alle `#[ignore]`, sie überspringen ohne Daten sauber:

```sh
cargo test --release -p driftless-map --test real_data -- --ignored --nocapture
```

Gemessen 181 s. Die Tests laufen parallel, ihre ausgegebenen Laufzeiten sind deshalb länger als
die Einzelläufe (`Test` gesamt 180 s statt 140 s).

| Test | prüft |
|---|---|
| `test_scan_full_under_15_min` | `Test`: SLAM + dicht + bereinigt + eingefärbt (Bibliothek, Standardwerte wie die CLI) < 15 min; > 1.000 Bilder genutzt |
| `test_scan_colored_share` | `Test`: Anteil eingefärbt ≥ 0,984 × 0,95 |
| `test_scan_exposure_reduces_camera_difference` | `Test`: Kamera-Unterschied nachher < 0,7 × vorher (gemessen 0,54), Bild-Versatz nachher < 0,75 × vorher (gemessen 0,56), Streuung nachher < vorher; > 100.000 Punkte von beiden Kameras gesehen |
| `jmk6_scans_run_and_are_deterministic` | `143415` und `143623` je zweimal voll: alle sechs Ausgabedateien bitgleich; > 100 Bilder genutzt, > 90 % eingefärbt, Bereinigung entfernt < 10 % |
| `jmk6_scan_143623_dense_cloud`, `_clean`, `_color` | schon vorhanden (Aufgaben 3–6), Einzelschritte auf `143623` |

Der geforderte Test `exposure_reduces_spread` heißt jetzt
`test_scan_exposure_reduces_camera_difference`. Er prüft die Streuung weiterhin, verlässt sich aber
auf die beiden neuen Kennzahlen, die tatsächlich auf die Faktoren reagieren. Synthetisch belegt
durch `camera_difference_drops_after_equalisation` und `image_offset_drops_after_equalisation` in
`tests/exposure.rs`.

Der RAM wird in den Tests nicht geprüft, weil mehrere Tests im selben Prozess laufen. Gemessen
wurde er mit der CLI (`map.json`, `/usr/bin/time -l`).

## Einschränkungen

- **Vergleich mit RayStudio ≠ Wahrheit.** Es gibt keine Referenzwolke mit bekannter Genauigkeit.
  Lage, Dichte und Abdeckung sind nur gegen die Herstellersoftware verglichen. Farbe gar nicht, weil
  die Referenz keine hat.
- **Farbtreue nicht gemessen.** Ob die Farben den echten Farben entsprechen, wurde nur im Bild
  beurteilt (Fensterrahmen, Pflanzen, Kissen). Eine Farbtafel war bei keiner Aufnahme dabei.
- **Kamera-Unterschied und Bild-Versatz messen Übereinstimmung, nicht Richtigkeit.** Sie stammen
  aus denselben Ansichten, an die der Abgleich angepasst wird. Sie zeigen, dass die Faktoren das
  tun, wofür sie da sind. Ob das Ergebnis „richtig hell“ ist, zeigen sie nicht. Die
  Gesamthelligkeit wird vom Mittel aller Bilder festgelegt (`mean(log g) = 0`).
- **Farbgrenzen auf großen Flächen** (Boden in `Test`) bleiben sichtbar. Der Abgleich hat einen
  Faktor je Bild. Ungleiches Licht innerhalb eines Bildes (Fenster gegen Raum) kann er nicht
  ausgleichen.
- **Grenzwert der Faktoren:** 6 Bilder erreichen ihn auf `Test`. Bei Scans mit noch größeren
  Helligkeitssprüngen werden mehr Bilder begrenzt und bleiben zu dunkel oder zu hell.
- **Bereinigung entfernt auch Feststehendes** an Pflanzen und schräg getroffenen Kanten. Wie viel,
  ist ohne Wahrheit nicht gemessen, nur im Bild gesehen. Ob Türen bewegt wurden, ist unbekannt.
- **Speicher wächst mit der Punktzahl** (≈ 0,55 kB je Punkt beim Einfärben). Sehr lange Scans mit
  1 cm könnten 8 GB überschreiten; dann `--voxel 0.015` oder `0.02`.
- **Nur drei Aufnahmen**, alle innen, zwei davon im selben Raum. Laufzeiten aus je einem Lauf.
- Die Windows-Messung des Spitzen-RAM ist nur gebaut (CI), nicht auf echten Daten gelaufen.
