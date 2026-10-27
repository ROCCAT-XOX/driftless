# Ergebnisse Qualitätsrunde – Schritt E (Glätten)

Stand: 2026-10-26 · VM 101. Dunkler Raum S1 (133554): Karte 0,5 cm, Oberfläche 1 cm, Stand nach F.
Einstellungen über Umgebungsvariablen auf dem lokalen Diagnosezweig `diagnose/qualitaet-e`, ohne Änderung am Programm.

| | heute: max_shift 3 cm, planarity 0,05 | max_shift 2 cm | 1 cm | 0,5 cm | radius 6 cm | planarity 0,03 | 0,02 | aus |
|---|---|---|---|---|---|---|---|---|
| Wanddicke nach Glätten mm | 0,95 | 0,95 | 1,59 | 3,12 | 4,20 | 1,29 | 3,48 | 6,02 |
| Oberfläche Wände Median / 95 % mm | 2,85 / 12,4 | 2,85 / 12,4 | 2,93 / 13,0 | 3,22 / 14,7 | 3,15 / 15,0 | 2,92 / 13,3 | 3,09 / 14,8 | 4,21 / 16,9 |
| gegen dense Wände Median / 95 % mm | 4,05 / 9,4 | 4,05 / 9,4 | 4,03 / 9,6 | 4,03 / 10,3 | 4,03 / 10,5 | 4,04 / 9,7 | 4,03 / 10,4 | 4,18 / 12,7 |
| gegen dense Kanten Median / 95 % mm | 4,75 / 9,8 | 4,75 / 9,8 | 4,70 / 9,6 | 4,56 / 9,3 | 4,57 / 9,3 | 4,68 / 9,6 | 4,58 / 9,3 | 4,39 / 8,9 |
| Dreiecke Mio | 2,67 | 2,67 | 2,81 | 3,32 | 3,48 | 2,87 | 3,35 | 4,48 |
| seam_step Mittel | 20,6 | 20,7 | 20,3 | 18,5 | 17,2 | 19,8 | 17,8 | 14,8 |
| .glb MB | 96 | 97 | 100 | 114 | 119 | 102 | 116 | 146 |

## Befund

- Unter 2 cm greift die Grenze `max_shift` nicht. Darunter wird nur weniger geglättet.
- Jede schwächere Einstellung macht die Wände dicker und das Netz größer:
  - Der Wandmedian wird um 0,07–1,4 mm schlechter.
  - Das `.glb` wächst um 4–50 MB.
- Gewonnen wird dabei nur an Kanten: dense p95 sinkt um 0,2–0,9 mm.
- Die Ziele von E sind mit der heutigen Einstellung schon erfüllt:
  - Wanddicke nach Glätten 0,95 mm, Ziel ≤ 3 mm.
  - `accuracy_dense` nicht schlechter.
- Für zusammengeschobene Doppelwände gibt es in den Messwerten keinen Hinweis.

## Entscheidung

E ist ohne Änderung abgeschlossen; Glätten bleibt bei radius 10 cm, planarity 0,05, max_shift 3 cm.
