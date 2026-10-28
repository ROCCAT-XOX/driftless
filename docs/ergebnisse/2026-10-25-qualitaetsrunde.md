# Ergebnisse Qualitätsrunde – Schritte A und B

Stand: 2026-10-25 · Zweig `qualitaetsrunde`. Gerechnet auf VM 101 (RTX 5080, 6 Threads).
Spec: `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md`,
Plan: `docs/superpowers/plans/2026-10-25-qualitaet-a-b.md`.

Vier Vorlagen: drei Aufnahmen desselben dunklen Raums (S1–S3) und ein großes Büro (B).

## Was sich geändert hat

- **A – Messung:** Die Karte schreibt Kennzahlen der dichten Punktwolke (`dense_quality`), die Oberfläche
  misst sich zusätzlich gegen die dichte Wolke (`accuracy_dense`). Die Vorlagen rechnet
  `tools/qualitaet/run_vorlagen.sh`, die Tabelle `q_table.py`.
- **B – zweistufige Oberfläche:** Ein Voxel mit zu wenig Messungen nimmt seinen Wert aus einem zweiten,
  doppelt so groben Raster (`coarse_fallback`, Vorgabe an). So schließen sich Wände, die der Scanner nur
  dünn erfasst hat.
- **Filter:** Eine Fläche aus dem groben Raster bleibt nur, wenn ein Messpunkt höchstens 1,5 cm entfernt
  liegt (`max_fill_gap`). Ohne ihn erfand das grobe Raster bei 2 cm Flächen ohne Messung
  (88 % der neuen Ecken ohne Punkt im Umkreis von 2 cm).

`tsdf_blocks` und `tsdf_bytes` zählen jetzt beide Raster.

## Löcher von oben (%)

| Feinheit | Stand | S1 | S2 | S3 | B |
|---|---|---|---|---|---|
| 1 cm | vorher | 9,8 | 44,4 | 23,4 | 72,5 |
| 1 cm | ohne Filter | 1,9 | 9,3 | 3,4 | 30,2 |
| 1 cm | **mit Filter** | **2,0** | **9,7** | **3,6** | **31,1** |
| 2 cm | vorher | 1,8 | 6,0 | 3,3 | 31,6 |
| 2 cm | ohne Filter | 1,3 | 2,4 | 2,0 | 18,6 |
| 2 cm | **mit Filter** | **1,5** | **2,8** | **2,4** | **22,9** |

## Genauigkeit der Oberfläche (mm)

Abstand jeder Ecke zum nächsten Messpunkt, Median / 95 %:

| Feinheit | Stand | Wände S1–S3 | Kanten S1–S3 95 % | Wände B |
|---|---|---|---|---|
| 1 cm | vorher | 2,99–3,29 / 7,4–7,9 | 8,1–8,2 | 3,67 |
| 1 cm | mit Filter | 2,85–3,39 / 11,7–13,6 | 10,1–10,4 | 5,42 |
| 2 cm | vorher | 4,08–4,42 / 9,0–9,9 | 15,8–18,1 | 5,98 |
| 2 cm | ohne Filter | 4,05–4,47 / 9,3–13,7 | 42,8–43,7 | 6,28 |
| 2 cm | mit Filter | 4,08–4,38 / 9,7–11,0 | 17,4–19,2 | 6,38 |

Dieser Wert wächst mit dem Punktabstand. In neu gefüllten Löchern liegen die Punkte weiter auseinander,
die Fläche ist dort aber nicht schlechter. Gegenprobe mit `q_fill_diag.py`: Abstand zur Ebene durch die
Punkte im Umkreis von 2 cm, Median / 95 %:

| | S2 1 cm | B 1 cm | S2 2 cm | B 2 cm |
|---|---|---|---|---|
| schon vorher abgedeckt | 1,37 / 6,73 | 1,72 / 6,91 | 1,37 / 5,39 | 1,94 / 6,91 |
| neu gefüllt | 1,59 / 6,32 | 2,10 / 7,52 | 1,72 / 7,77 | 1,16 / 5,04 |
| Anteil neu gefüllter Ecken | 27 % | 65 % | 4 % | 25 % |

## Raummaße und Aufwand

| Messgröße | 1 cm | 2 cm |
|---|---|---|
| Spanne der Raummaße S1–S3 kurz / lang / Höhe (mm) | 0,8 / 1,1 / 2,1 | 1,1 / 0,6 / 1,8 |
| Zeit Oberfläche S1–S3 vorher → jetzt (s) | 133–136 → 193–225 | 78–95 → 79–98 |
| Zeit Oberfläche B (s) | 1152 | 330 → 418 |
| Arbeitsspeicher Oberfläche B (GB) | 7,2 | 2,7 → 3,2 |
| Fläche ohne Textur B (%) | 4,1 | 4,4 → 6,5 |

SLAM und Karte ändern sich durch A und B nicht. Der Versatz zwischen Durchgängen (Ziel Z3) bleibt
Thema von Schritt C.

Die weiteren Schritte: [C und D](2026-10-26-qualitaet-c-d.md) (Durchgänge und Büro-Laufbahn, ohne
Änderung), [E](2026-10-26-qualitaet-e.md) (Glätten), [F](2026-10-26-qualitaet-f.md) (Texturkarten und
Nähte).

## Entscheidung

Zweistufige Oberfläche mit Filter ist die Vorgabe für alle Feinheiten. Die Hinweise zu 1 cm im Programm
nennen jetzt bis 8 GB Arbeitsspeicher und rund 20 Minuten bei großen Aufnahmen, statt „dunkle Wände
bleiben löchrig“.
