# Auswertung der Vorlagen-Scans (Qualitätsrunde)

Entwicklerwerkzeug, keine Daten im Repo. Es lässt die vier Vorlagen-Scans durch `driftless slam`,
`map` und `surface` laufen, misst die Ergebnisse und schreibt eine Tabelle mit einer Spalte je Scan.
Die Messgrößen und Ziele stehen in `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md`.

| Datei | Aufgabe |
|---|---|
| `run_vorlagen.sh` | Lauf für alle Scans: slam, map (GPU), surface (GPU), Messung, Tabelle |
| `q_analyze.py` | `variant <Lauf-Ordner>` → `metrics.json`; `cross <Label-Ordner> <Ref> <S2> <S3>` → `cross.json`, `canon.json` |
| `q_table.py` | `<Label-Ordner> [Scan …]` → Markdown-Tabelle |
| `q_views.py` | `<Scan> <canon.json>` → Kamerapositionen (`QA_VIEWS`) des schwarzen Raums im Rahmen des jeweiligen Scans |

## Was gemessen wird (Python, `q_analyze.py`)

Aus `clean.las` und `dense.las` werden große Ebenen gefunden (Raumrahmen aus den Wandnormalen).
Auf jeder Ebene wird auf der ganzen Wolke angepasst, danach in Flecken von 30 × 30 cm (mindestens
300 Punkte, ohne Gegenstände) gemessen. Zeitlücken über 5 s trennen die Durchgänge.

| Größe | Bedeutung |
|---|---|
| Ebene lokal RMS / p95 | Streuung der Punkte um die Ebene des Flecks, Median über die Flecken |
| Ebene global RMS / p95 | dasselbe um die Ebene der ganzen Wand |
| Wanddicke 95 % | Abstand der 2,5- und 97,5-%-Punkte senkrecht zur Wand im Fleck |
| Rauschen je Durchgang | RMS der Punkte um den Median ihres Durchgangs im Fleck |
| Versatz zwischen Durchgängen | größter Unterschied der Durchgangs-Mediane im Fleck (Median / p90), nur Flecken mit mindestens 2 Durchgängen. Dieselbe Wand aus verschiedenen Durchgängen |
| Raummaße | Abstand der äußersten Gegenwände je Achse (x, y) und Boden–Decke (z); Nicht-Parallelität in Grad; Wandabstand in 3 Abschnitten (Spanne) |
| Läufe | Schleifen (akzeptiert/geprüft, ICP-RMS), Start↔Ende vorher/nachher, Weglänge, Korrektur durch den Schleifenschluss, aus `slam.json` und den Trajektorien |
| Oberfläche | Abstand der Dreiecke zu `clean` (`accuracy`: Wände, Kanten) und zu `dense` (`accuracy_dense`), Texturkarten, Seitenfüllung, `seam_step`, `.glb`-Größe |
| Zeit, RAM | aus `/usr/bin/time -v` je Schritt |

`q_analyze.py cross` richtet die Scans eines Raums aufeinander aus und misst Abdeckung und
Raummaß-Spanne (Ziel Z5). Die Tabelle zeigt die Spanne auch ohne `cross` (Zeile unter der Tabelle).

Die Zeilen „Programm: …“ der Tabelle sind die Kennwerte, die das Programm selbst berechnet
(`map.json` → `steps.clean.stats.dense_quality`, `oberflaeche.json` → `stats.accuracy_dense`), die
Zeilen „dense: …“ derselbe Wert nach der Python-Definition. Beide sollen innerhalb 1 mm liegen.

Nicht enthalten: „Löcher von oben“ und Rundblick (Ziel Z1) kommen aus dem Ansichtswerkzeug
(`surface_real.rs`), nicht aus diesen Skripten.

## Auf VM 101 ausführen

Halbe Last (6 von 12 vCPU, `nice 10`) und immer nur ein Lauf zur Zeit; `driftless-vm run` setzt
`RAYON_NUM_THREADS=6` und `nice -n 10` selbst.

```
driftless-vm sync
driftless-vm run cargo build --release -p driftless-cli
DQ_PYTHON=$HOME/td/quality-venv/bin/python driftless-vm run tools/qualitaet/run_vorlagen.sh b0
```

`DQ_PYTHON` muss ein Python mit numpy, scipy und laspy sein (auf der VM die venv
`~/td/quality-venv`); ohne Angabe gilt `python3`. Wird die Variable nicht an die VM durchgereicht,
im Aufruf mit `env` setzen: `driftless-vm run env DQ_PYTHON=$HOME/td/quality-venv/bin/python tools/qualitaet/run_vorlagen.sh b0`.

Optionen: `--map-voxel 0.005` (Karte, m) und `--surface-voxel 0.01` (Oberfläche, m) sind die
Vorgaben „Maximal“. Umgebung:

| Variable | Standard |
|---|---|
| `DQ_SCANS` | die vier Vorlagen auf der VM: `~/td/jmk6-2026-10-22/{20261022133554,20261022132520,20261022133028}` und `~/td/jmk6-2026-10-20/20261020173822` (leerzeichengetrennt) |
| `DQ_ROOT` | `~/td/quality` |
| `DQ_PYTHON` | `python3` |

Ergebnis je Scan in `~/td/quality/<label>/<scan>/` (slam-, map- und Oberflächen-Dateien, Logs
`slam.log`, `map.log`, `surface.log`, `analyze.log`, `metrics.json`) und die Tabelle in
`~/td/quality/<label>/tabelle.md`. Ein Scan, dessen Schritt scheitert, wird übersprungen; der Lauf
macht mit dem nächsten weiter. Die GPU-Schritte laufen über `gpu-job`.
Für Vergleiche (Laufrauschen, vorher/nachher) jeweils ein eigenes `<label>` wählen.
