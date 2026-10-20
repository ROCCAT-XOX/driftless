# 0002 – Kamera-Kalibrierung: Bilddrehung, Richtung der Matrix, Zeitversatz

Stand: 2026-10-20 · Status: **Angenommen** für Meilenstein 3 (gemessen). Bilddrehung und Richtung sind sicher,
der Zeitversatz ist vorläufig (siehe „Offen“).

## Frage

`calibration/calib.json` der JMK6 enthält je Kamera (`left`, `right`) ein Fisheye-Modell (`K`, 4 Koeffizienten,
Bildgröße 4000 × 3000) und `out_put.<cam>.transform_matrix` (4 × 4, Verschiebung ≈ 0,004 / −0,12 / −0,03 m).
Nicht dokumentiert ist:

1. Bildet die Matrix LiDAR-Punkte in den Kamerarahmen ab (LiDAR → Kamera) oder umgekehrt?
2. Auf welches Bild bezieht sich `K`? Der Bildhauptpunkt (≈ 1515 / 1966) passt nicht zu einem
   4000 × 3000-Bild (Mitte 2000 / 1500), sondern zu einem hochkant gedrehten 3000 × 4000-Bild.
3. Wie weit liegen die Zeitstempel der Bilder neben der LiDAR-Uhr?

## Messung

`driftless calib-check <scan> --slam <ziel> -o <ordner> [--images N]` (Test-Projektion):

- Je Kamera N Bilder: der Scan wird in N gleiche Zeitabschnitte geteilt, je Abschnitt das schärfste Bild
  (mittlerer Sobel-Gradient). Gerechnet wird auf 1000 × 750 Pixeln (Bild beim Dekodieren um 4 verkleinert).
- LiDAR-Punkte von ±0,5 s um jedes Bild, 0,5–30 m, mit der korrigierten Laufbahn (`traj.tum` von
  `driftless slam`) je Punkt zu seiner Messzeit in die Welt gesetzt (das entzerrt sie zugleich).
- Varianten: 4 Bilddrehungen (keine, 90° im / gegen den Uhrzeigersinn, 180°) × 2 Richtungen der Matrix ×
  41 Zeitversätze (−0,20 … +0,20 s, Schritt 0,01 s) = 328. Kamerapose = Laufbahn zur Zeit Bildstempel + Versatz
  · Kamera im LiDAR-Rahmen.
- Güte = mittlerer normierter Bildgradient (Mittel des Bildes = 1, also Zufall ≈ 1) an den sichtbaren
  Tiefensprung-Punkten: Punkte, deren Nachbar auf derselben LiDAR-Zeile mehr als 0,3 m weiter weg ist.
  Sichtbarkeit über einen Tiefenpuffer (2 × 2 Pixel je Zelle).
- **Warum Nachbarn auf der LiDAR-Zeile und nicht im Bild:** Zuerst wurden Tiefensprünge im Bild gesucht
  (Nachbarzelle im Tiefenpuffer). In der synthetischen Prüfszene lag das Ergebnis dann systematisch daneben
  (+0,045 s bzw. +0,06 s statt +0,05 s, je nach Drehrichtung): Zwischen den LiDAR-Zeilen bleiben Lücken im
  Tiefenpuffer, durch die Hintergrundpunkte „sichtbar“ werden, und wegen des Versatzes Kamera ↔ LiDAR
  geschieht das nur an einer Seite der Kanten. Mit Nachbarn auf der LiDAR-Zeile trifft die synthetische
  Prüfung genau +0,05 s, auch bei unterschiedlichen Kontrasten links und rechts der Kanten.
- Laufbahn: `driftless slam` mit Standardeinstellungen. Die Karte (`map.las`, 10 cm) ist für scharfe Kanten zu
  grob, deshalb werden die Rohpunkte genommen.

Gemessen auf dem MacBook Pro M4 Pro, Werte aus `calib_check.json`. Drei Aufnahmen desselben Geräts:
`Test` (266 s), `jmk6-2026-10-20/20261020143415` (63 s) und `…/20261020143623` (108 s).

### Bilddrehung und Richtung

Beste Variante insgesamt und die beste Variante mit anderer Bilddrehung bzw. anderer Richtung (Güte):

| Aufnahme | Bilder je Kamera | beste Variante | Güte | beste andere Drehung | beste andere Richtung |
|---|---|---|---|---|---|
| Test | 12 | gegen Uhrzeigersinn, LiDAR → Kamera | 1,841 | 1,304 | 1,249 |
| Test | 30 | gegen Uhrzeigersinn, LiDAR → Kamera | 1,982 | 1,263 | 1,206 |
| Test | 60 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,057 | 1,293 | 1,225 |
| 143415 | 12 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,100 | 1,380 | 1,380 |
| 143415 | 30 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,358 | 1,367 | 1,297 |
| 143415 | 60 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,382 | 1,339 | 1,307 |
| 143623 | 12 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,192 | 1,617 | 1,434 |
| 143623 | 30 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,446 | 1,436 | 1,357 |
| 143623 | 60 | gegen Uhrzeigersinn, LiDAR → Kamera | 2,430 | 1,377 | 1,325 |

Je Kamera einzeln gewinnt dieselbe Variante in 16 von 18 Fällen. Die zwei Ausnahmen: linke Kamera, 12 Bilder,
in beiden kurzen Aufnahmen (beste Güte 1,80 / 1,79, die richtige Variante 1,61 / 1,62). Die linke Kamera sah
dort meist flache Wände und Fenster mit wenigen Tiefensprüngen; ab 30 Bildern wählt auch sie die richtige
Variante (1,82–2,02 gegenüber höchstens 1,75 für die nächste).

### Zeitversatz

Bester Versatz der richtigen Drehung und Richtung (insgesamt; links / rechts) und wie stark die Güte über
−0,20 … +0,20 s schwankt (Spanne = (max − min) / max):

| Aufnahme | Bilder je Kamera | bester Versatz | links / rechts | Spanne der Güte |
|---|---|---|---|---|
| Test | 12 | +0,03 s | +0,02 / +0,05 s | 14,8 % |
| Test | 30 | +0,05 s | +0,05 / +0,07 s | 11,2 % |
| Test | 60 | +0,03 s | +0,01 / +0,05 s | 10,9 % |
| 143415 | 12 | +0,19 s | – | 7,4 % |
| 143415 | 30 | +0,14 s | +0,00 / +0,14 s | 4,6 % |
| 143415 | 60 | +0,13 s | −0,01 / −0,09 s | 4,1 % |
| 143623 | 12 | −0,18 s | – | 6,3 % |
| 143623 | 30 | −0,18 s | −0,05 / +0,17 s | 5,2 % |
| 143623 | 60 | +0,09 s | +0,05 / +0,09 s | 4,3 % |

- **Test:** deutliches Maximum, Kurve oben flach zwischen +0,01 und +0,05 s (60 Bilder: +0,00 → 2,045,
  +0,02 → 2,054, +0,04 → 2,053, ±0,20 s → 1,88 / 1,83). Ein Vorversuch über ±0,60 s fand kein anderes Maximum.
- **Kurze Aufnahmen:** Kurve fast flach (4–7 %), bester Wert springt zwischen −0,18 und +0,19 s. Dort ist der
  Versatz nicht messbar: Das Gerät dreht sich im Mittel nur ≈ 0,15 rad/s, 0,1 s Versatz verschieben die Punkte
  um ≈ 4 Pixel (bei 1000 Pixeln Breite), weniger als das Rauschen der Güte.

Laufzeit `calib-check` (12 Bilder je Kamera): Test 43 s, 143415 14 s, 143623 20 s; Spitzen-RAM ≈ 350 MB
(60 Bilder: ≈ 820 MB).

## Entscheidung

1. **`transform_matrix` bildet LiDAR-Punkte in den Kamerarahmen ab** (`p_kamera = M · p_lidar`).
2. **`K` gilt für das um 90° gegen den Uhrzeigersinn gedrehte Bild** (3000 × 4000). Gespeicherter Bildpunkt
   `(x, y)` = `(4000 − 1 − y', x')` des gedrehten Bildpunkts `(x', y')`.
3. **Zeitversatz +0,03 s**: Ein Bild mit Stempel `t` wurde zur LiDAR-Zeit `t + 0,03 s` aufgenommen
   (Mittel der drei Messungen auf `Test`: +0,03 / +0,05 / +0,03 s; die Kurve ist zwischen 0 und +0,05 s flach, Unsicherheit etwa ±0,05 s).

Im Code (`crates/driftless-io/src/calib.rs`): `CALIB_MATRIX_DIRECTION`, `CALIB_IMAGE_ROTATION`,
`CAMERA_TIME_OFFSET_S`. `load_calibration` rechnet `K` und die Matrix gleich auf das gespeicherte Bild um:
`CameraCalib.lidar_to_cam` bildet LiDAR-Punkte in einen Kamerarahmen ab, dessen x/y-Achsen den Achsen des
gespeicherten Bildes folgen, und `Camera::project` liefert Pixel des gespeicherten Bildes. Der Zeitversatz wird
Standardwert von `ColorConfig.time_offset` (Task 5).

Nicht genutzt: `camera_info.<cam>.extrinsics` (Drehvektor ≈ ±1,58 rad, Verschiebung ≈ 2,4 m – für ein
Handgerät unmöglich, kein Bezug zu LiDAR ↔ Kamera erkennbar) und `initial_guess` / `EulerFormResult`
(dieselben Zahlen wie die Verschiebung der Matrix in anderer Form).

## Offen

- Zeitversatz: nur auf einer Aufnahme deutlich messbar; er liegt bei 0 … +0,05 s, Unsicherheit etwa ±0,05 s, +0,03 s als Standard. Die rechte Kamera hat einen Doppelgipfel. Eine Aufnahme mit
  schnellen Drehungen vor Kanten (Türrahmen, Säulen) würde ihn genauer bestimmen.
- Die Kalibrierung selbst (Matrix, `K`) wird nicht verfeinert; die beste Güte liegt bei 1,8–2,4.
- Gilt für die JMK6. Andere Geräte mit anderem `calib.json` brauchen eine eigene Messung mit `calib-check`.
