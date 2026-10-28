# Ergebnisse Qualitätsrunde – Schritte C und D (Durchgänge, Büro-Laufbahn)

Stand: 2026-10-26 · VM 101. Vorlagen wie in A+B: dunkler Raum S1 (133554), S2 (132520), S3 (133028), Büro B.
Nur Diagnose, ohne Änderung am Programm. Die Diagnose-Werkzeuge (`q_c0_diag.py`, `q_c0b_analyze.py`,
`q_c0b_patches.py`, `c0b_dump.rs`, `d1_loops.rs`) sind nicht im Repository; sie liegen nur auf dem lokalen
Diagnosezweig `diagnose/qualitaet-c-d`. Ziele Z3 und Z4: `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md`.

## C0 – Lage oder Sensor? (Z3, Versatz derselben Wand zwischen Durchgängen)

| Messgröße | Wert |
|---|---|
| Spanne max − min, 133554 / 132520 | 9,9 / 8,6 mm, vor allem Max − Min über 7–10 Durchgänge |
| echter Versatz je Durchgang | 3,4 mm RMS |
| die zwei größten Durchgänge | 1,9 / 2,7 mm |
| Rauschen allein | 1,3–1,8 mm |
| Anteil des Versatzes durch den Sensor (Entfernung, Intensität, Winkel) | ≤ 7 % |
| Anteil durch einen Zeitversatz | ≤ 2 % |
| Anteil durch einen Hebelarm | 3 % |
| starres Nachregistrieren je Teilkarte / je 3 s | real schlechter: 10,5–12,2 mm; bestenfalls 7 mm |
| dieselbe Wand, erste gegen zweite Hälfte desselben Durchgangs | wandert ~4 mm |

- Die Spanne max − min wächst mit der Zahl der Durchgänge; der Versatz je Durchgang ist kleiner.
- Sensor, Zeitversatz und Hebelarm erklären fast nichts. Eine Korrektur dafür lohnt den Bau nicht.
- Der Fehler ändert sich innerhalb von Sekunden. Eine starre Lage je Teilkarte oder je 3 s kann ihn
  deshalb nicht ausgleichen.

## C0b – Rohframes je Laserstrahl

| Messgröße | Wert |
|---|---|
| Versatz je Laserstrahl | real, ±2–3 mm (Ring 9 +3,6 mm) |
| Korrektur je Strahl: Versatz | 3,29 → 3,23 mm (erklärt ~2 %) |
| Korrektur je Strahl: Rauschen im Durchgang | 5,14 → 4,87 mm |
| Entzerrung | in Ordnung (R² 0,0001) |
| starre Korrektur je Frame | Versatz schlechter |
| Rest | flächenlokal, 0,3–1 s Dauer, dazu langsames Wandern im Durchgang (3,7 mm) |

- Der Versatz je Strahl mittelt sich im Durchgang weg.
- Die Ursache des Rests ist offen, vielleicht das Messverhalten des Sensors nach Azimut oder Winkel.

## D1 – Laufbahn im Büro (Z4)

| Messgröße | Wert |
|---|---|
| verworfene Schleifen im Büro | 115 |
| davon richtig, aber wenig gemeinsame Fläche (0,43) und schlechter passend (0,40) | 65 + 6 |
| davon falsche ICP-Becken | 44 |
| Versuch mit Schwellen 0,05 / 30 mm: angenommen | 260 / 265 richtige, 0 / 181 falsche |
| derselbe Versuch im Volllauf: 133028 Start↔Ende | 24,6 → 30,0 mm (über der Toleranz +5 mm) |
| derselbe Versuch im Volllauf: Schleifen-RMS max | +7 mm |
| derselbe Versuch im Volllauf: Büro | nicht besser: 132,7 mm, Wände 1,23° |
| Start↔Ende im Büro: Abstand laut 4 Direktregistrierungen | 128–130 mm |
| Start↔Ende im Büro: Rest im Graphen | 1,4 mm |

- Die gelockerten Schwellen sind verworfen. Die Umgebungsvariable dafür in `steps/slam.rs` ist wieder
  entfernt.
- Die 129 mm Start↔Ende sind die echte Strecke: Das Gerät endete 128–130 mm vom Start entfernt. Gemessen als
  „Restlücke“ war Z4 deshalb falsch definiert.
- Die 1,2° zwischen den Büro-Wänden sind unerklärt. Sie ändern sich nicht mit den Schleifen. Vielleicht ist
  das Gebäude so gebaut; klären lässt sich das nur mit einer Messung vor Ort.

## Ziele Z3 und Z4

| Ziel | Messgröße | Ausgang (Spec) | Ziel | Stand nach C und D |
|---|---|---|---|---|
| Z3 | Versatz derselben Wand zwischen Durchgängen (dense) | 6,6–8,8 mm | ≤ 5 mm | derzeit nicht erreichbar; Spanne max − min 9,9 / 8,6 mm bleibt im Bericht |
| Z3 | dazu neu: Versatz je Durchgang RMS | – | ≤ 2 mm (Vorschlag) | 3,4 mm |
| Z4 | Büro: Restlücke, neu gemessen als Rest der Schleife 0↔Ende | 129 mm (alte Definition) | ≤ 50 mm | 1,4 mm, erfüllt |
| Z4 | Büro: Gegenwände nicht parallel | 1,26° | ≤ 0,3° | 1,2°, unerklärt |

## Vor Ort prüfen (Yannick)

- Im Büro den Winkel der beiden Gegenwände mit einem Laser-Entfernungsmesser nachmessen: den Abstand
  Wand–Wand an beiden Enden des Raums. Sind die Wände parallel, sind beide Maße gleich. 1,2° ergäben
  rechnerisch rund 21 cm Unterschied je 10 m Wandlänge.
- Dabei einige Strecken (Wand–Wand, Türbreiten) mit „Messen“ im Programm vergleichen: Gegen ein echtes Maß
  wurde Driftless noch nie geprüft.

## Entscheidung

- **C ruht ohne Änderung.** Kein Bau: C1 bis C3 und eine Sensorkorrektur sind gestrichen. Die Tabelle je
  Laserstrahl bleibt ein Kandidat für später (Rauschen, Schritt E). Ist das falsch, wird C mit einer
  Azimut-Diagnose wieder aufgenommen.
- **D ist ohne Änderung abgeschlossen.**
- **Z3 ist derzeit nicht erreichbar.** Die Zahl bleibt im Bericht. Gegenüber 129 mm im Büro waren die ~3 mm
  nachrangig.
- **Z4 ist neu definiert:** Rest der Schleife 0↔Ende statt Abstand Start↔Ende. So gemessen ist es erfüllt.
- Die Runde ging mit F (Texturkarten und Nähte) weiter, danach E.
