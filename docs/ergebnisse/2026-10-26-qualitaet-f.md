# Ergebnisse Qualitätsrunde – Schritt F (Texturkarten und Nähte)

Stand: 2026-10-26 · Zweig `qualitaet-f`. Gerechnet auf VM 101 (RTX 5080, 6 Threads), Oberfläche neu
auf den Karten von A+B (1 cm: Lauf `b2`, 2 cm: Lauf `a2`).
Spec: `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md`, Plan:
`docs/superpowers/plans/2026-10-26-qualitaet-f.md`. Vorlagen wie in A+B: S1–S3 (dunkler Raum), B (Büro).

## Was sich geändert hat

- **Inseln** (`absorb_islands`): Eine Texturkarte mit weniger als 8 Flächen übernimmt das Bild der
  Nachbarkarte, mit der sie die meisten Kanten teilt. Voraussetzung ist, dass dessen Kamera sie
  geometrisch sieht: im Bild, in Reichweite, Umgebung zur Kamera gewandt. Einen Tiefentest gibt es nicht.
- **Kennzahl:** `texture.islands_absorbed`. Oberfläche v12, alte Oberflächen gelten als veraltet.
- Das Netz bleibt unverändert, die Wandgenauigkeit ist identisch.

## Diagnose vorher (S1, 1 cm)

- Von 192.456 Karten bestanden 110.586 aus einer einzigen Fläche.
- 75 % der Summe von `seam_step` entsteht an Karten mit 1–3 Flächen.
- Ursache ist das wellige 1-cm-Netz: Einzelne Flächen zeigen schräg weg, fallen bei ihrem Nachbarbild
  durch und behalten ein anderes Bild.
- Werkzeug: `q_f_seams.py`.

## 1 cm

| | S1 vorher | S1 neu | S2 vorher | S2 neu | S3 vorher | S3 neu | B vorher | B neu |
|---|---|---|---|---|---|---|---|---|
| Karten | 221.097 | 79.750 | 197.769 | 67.974 | 239.794 | 84.478 | 2.026.134 | 808.938 |
| Seiten | 8 | 7 | 7 | 6 | 7 | 6 | 36 | 30 |
| .glb MB | 111 | 96 | 105 | 91 | 113 | 98 | 723 | 609 |
| seam_step Mittel / Median | 21,9 / 6,1 | 20,6 / 4,6 | 19,7 / 5,2 | 17,2 / 3,6 | 20,6 / 5,5 | 18,8 / 4,1 | 21,8 / 6,8 | 19,0 / 4,8 |
| seam_jump | 17,4 | 14,4 | 15,9 | 12,1 | 16,4 | 13,2 | 18,5 | 14,7 |
| ohne Textur % | 1,14 | 1,14 | 1,33 | 1,33 | 1,21 | 1,21 | 4,12 | 4,12 |
| Wände Median mm | 2,85 | 2,85 | 3,13 | 3,13 | 3,39 | 3,39 | 5,42 | 5,42 |
| Zeit Oberfläche s | 225 | 230 | 193 | 198 | 203 | 200 | 1152 | 1146 |

## 2 cm

| | S1 vorher | S1 neu | S2 vorher | S2 neu | S3 vorher | S3 neu | B vorher | B neu |
|---|---|---|---|---|---|---|---|---|
| Karten | 59.142 | 35.902 | 51.154 | 31.057 | 60.454 | 36.519 | 599.769 | 331.831 |
| .glb MB | 47 | 44 | 44 | 41 | 48 | 45 | 313 | 284 |
| seam_step Mittel / Median | 20,5 / 5,4 | 17,9 / 4,4 | 18,0 / 4,5 | 14,7 / 3,5 | 19,4 / 5,1 | 16,4 / 4,2 | 19,2 / 5,3 | 16,6 / 4,2 |
| seam_jump | 13,2 | 10,0 | 11,7 | 8,3 | 12,4 | 9,2 | 13,6 | 10,3 |
| Wände Median mm | 4,08 | 4,08 | 4,32 | 4,32 | 4,38 | 4,38 | 6,38 | 6,38 |

## Ziel Z6 (S1 = 133554, Ausgangswert b = 1 cm)

| Messgröße | vorher | jetzt | Ziel | |
|---|---|---|---|---|
| Karten | 221.097 | 79.750 | ≤ 80.000 | erreicht |
| `seam_step` Mittel | 19,7–21,9 | 17,2–20,6 (2 cm: 14,7–17,9) | ≤ 15 | verfehlt |
| `.glb` | 111 MB | 96 MB | ≤ 60 MB | verfehlt bei 1 cm, bei 2 cm 44 MB |

**Grund `seam_step`:** Nach der Änderung sitzt der Rest vor allem auf Knicken über 45° (rund 50 % der
Summe) und auf Inhalt an der Naht (Seiten mehr als 3× verschieden, rund 18 %). Dort treffen zwei
Kamerabilder mit wenigen Millimetern Versatz auf eine scharfe Farbkante. Die Bildwahl löst das nicht,
nur eine genauere Registrierung (siehe C, ohne Bau abgeschlossen).

**Grund `.glb`:** Bei 1 cm sind allein Netz und UVs 79 MB groß (2,67 Mio. Flächen). Das Netz
zu glätten (Taubin, 5 Runden) brachte 63 MB. Der Wandmedian stieg dabei aber um 0,24 mm, über die
Toleranz für Z2 von 0,2 mm, deshalb wurde es nicht übernommen.

## Verworfene Varianten

| S1 | Karten 2 cm | seam_step 2 cm | Karten 1 cm | seam_step 1 cm |
|---|---|---|---|---|
| ohne Wächter (übernommen) | 35.902 | 17,9 | 79.750 | 20,6 |
| Einschluss ≥ 2/3 + eigene Normale | 49.926 | 20,0 | 155.613 | 23,6 |
| nur eigene Normale | 38.664 | 18,9 | 108.658 | 22,2 |
| nur Einschluss | 47.017 | 19,4 | 123.015 | 22,9 |
| Inselgrenze 64 statt 8 | – | – | 85.587 | 24,8 |

Die Wächter sollten verhindern, dass ein kleines Einzelobjekt das Bild der Wand dahinter bekommt. Die
Sichtprüfung (S1, 2 cm, die acht am stärksten geänderten Ausschnitte vorher / ohne / mit Wächter) zeigte
keine falschen Farben. Die Wächter nahmen aber den größten Teil des Gewinns weg. Das Restrisiko
steht im Code.

## Entscheidung

Die Inseln sind die Vorgabe für alle Feinheiten. Z2, Z3 und Z5 sind unverändert, die Geometrie ist dieselbe.
