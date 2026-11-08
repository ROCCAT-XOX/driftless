# Auswertung der Vorlagen-Scans (Qualitätsrunde)

Entwicklerwerkzeug, keine Daten im Repo. Es lässt die vier Vorlagen-Scans durch `driftless slam`,
`map` und `surface` laufen, misst die Ergebnisse und schreibt eine Tabelle mit einer Spalte je Scan.
Die Messgrößen und Ziele stehen in `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md`.

| Datei | Aufgabe |
|---|---|
| `run_vorlagen.sh` | Lauf für alle Scans: slam, map (GPU), surface (GPU), Messung, Tabelle |
| `run_rauschen.sh` | Prüflauf „weniger Rauschen“: alle Läufe der Spec, Abschnitt 11 (siehe unten) |
| `q_analyze.py` | `variant <Lauf-Ordner>` → `metrics.json`; `surface <Lauf-Ordner> [<metrics.json der Karte>]` → `metrics.json` einer Oberflächen-Variante (Wolkenwerte aus der Karte, die Wolke ändert sich zwischen den Varianten nicht); `cross <Label-Ordner> <Ref> <S2> <S3>` → `cross.json`, `canon.json` |
| `q_table.py` | `<Label-Ordner> [Scan …]` → Markdown-Tabelle |
| `q_images.sh` | `<Lauf-Ordner>` → Bilder in `bilder/` (bleiben auf der VM) und `images.log` mit den Zeilen `LOECHER <Ansicht> <Prozent>`; nutzt den ignorierten Test `crates/driftless-steps/tests/quality_real.rs` (baut ihn, falls nötig) |
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
Zeilen „dense: …“ der Wert nach der Python-Definition (`refine`). Das sind nicht dieselben Zahlen:

- Grundgesamtheit: Rust misst alle ebenen 30-cm-3D-Zellen der ganzen Wolke (auch Möbel), Python
  Flecken auf den höchstens 12 größten Raumebenen, nach dem 5°/1-cm-Objektfilter.
- Aggregation: Rust nimmt das p90 über Zellen, Python den Median über Ebenen des p90 je Ebene.
  Das Rauschen je Durchgang ist in beiden der Median über Flecken/Zellen.
- Eine Wand näher als etwa 1 cm an einer Zellfläche wird auf zwei Zellen verteilt; das kann ihren
  Versatz verdecken und drückt den Rust-Versatz nach unten.

Es gilt die Regel aus der Spezifikation: liegen beide innerhalb 1 mm, passt es, sonst zählt der
Python-Wert.

### Löcher (Ziel Z1, `q_images.sh`)

Der Test `quality_real` zeichnet die Oberfläche (`oberflaeche.glb`) und die Punktwolke (`color.las`)
aus derselben Kamera und zählt, wie viele Pixel mit Punkten keine Oberfläche zeigen. Ansichten:
`oben` (von oben, bis 0,6 m über der mittleren Bahnhöhe), `rundblick_000` bis `rundblick_315`
(8 Richtungen aus der Raummitte) und, wenn `<Label-Ordner>/canon.json` von `q_analyze.py cross`
existiert, die festen Ansichten des schwarzen Raums. Die Tabelle zeigt „Löcher von oben“ und die
Spanne min–max der Rundblick-Ansichten. Der Test ist `#[ignore]` und braucht Daten über
Umgebungsvariablen (`QA_A_SURF`, `QA_A_CLOUD`, `QA_TRAJ`, `QA_OUT`; optional `QA_B_*` für einen
zweiten Lauf nebeneinander), normale `cargo test` laufen ohne ihn.

## Auf VM 101 ausführen

Halbe Last (6 von 12 vCPU, `nice 10`) und immer nur ein Lauf zur Zeit; `driftless-vm run` setzt
`RAYON_NUM_THREADS=6` und `nice -n 10` selbst.

```
driftless-vm sync
driftless-vm run cargo build --release -p driftless-cli
driftless-vm run bash -c 'DQ_PYTHON=$HOME/td/quality-venv/bin/python tools/qualitaet/run_vorlagen.sh b0'
```

`DQ_PYTHON` muss ein Python mit numpy, scipy und laspy sein (auf der VM die venv
`~/td/quality-venv`); ohne Angabe gilt `python3`. Die Variable wird mit `bash -c` auf der VM gesetzt,
weil sie sonst nur lokal gälte und `~` dort nicht aufgelöst würde.

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

## Prüflauf „weniger Rauschen“ (`run_rauschen.sh`)

Rechnet auf VM 101 alle Läufe aus Abschnitt 11 der Spec
(`docs/superpowers/specs/2026-11-08-weniger-rauschen-design.md`). Laufbahn und Karte (0,5 cm) entstehen
einmal je Scan, das Modell S1–S3 mit `driftless merge`, danach die Oberflächen.

| Ordner unter `<label>/` | Inhalt |
|---|---|
| `basis/<scan>`, `basis/modell` | Laufbahn, Karte (mit `clean.las` und `dense.las`), Modell, Werte der Wolke |
| `vorher_2cm`, `vorher_1cm` | `--ohne alles`, alle fünf Sätze |
| `nachher_2cm`, `nachher_1cm` | Vorgabe, alle fünf Sätze |
| `ohne_anlegen_1cm`, `ohne_belichtung_1cm`, `ohne_entrauschen_1cm`, `ohne_verdeckung_1cm` | `--ohne` je eine Maßnahme, S1 |
| `cpu_2cm` | Vorgabe, `--gpu aus`, S1 |
| `probe_2cm` | Programm des Stands 99284b9, S1; `probe.txt` sagt, ob `oberflaeche.glb` Byte für Byte `vorher_2cm` entspricht |
| `tabelle.md` | eine Tabelle je Variante (`q_table.py`), eine Spalte je Scan |

Die Bilder von vorher und nachher liegen nebeneinander (vorher links) in `vorher_*/<scan>/bilder/`, die
Reliefbilder daneben als `*_relief.png`: die Form der Fläche in Grau mit Licht von schräg, ohne Foto. Bilder
bleiben auf der VM und kommen nie ins Repository. Die Modell-Oberfläche hat keine festen Ansichten des
dunklen Raums, nur `oben` und den Rundblick; die Laufbahn für ihre Ansichten ist die von S1, deren Rahmen
der Modellrahmen ist.

Einmal starten, das Skript baut selbst und koppelt sich mit `--abgekoppelt` vom Terminal ab. Das Protokoll
steht in `<label>/lauf.log`, die letzte Zeile beginnt mit `FERTIG`. Was fertig ist, hinterlässt `<schritt>.ok`
und wird bei einem Neustart mit demselben `<label>` übersprungen; für einen Lauf von vorn ein neues `<label>`
nehmen.

```
# Stand des Zweigs und, einmal, das Programm von 99284b9 für die Probe auf die VM
git worktree add --detach ../driftless-basis 99284b9
DRIFTLESS_REPO=$PWD/../driftless-basis DRIFTLESS_RDIR=driftless-basis driftless-vm sync
DRIFTLESS_RDIR=driftless-basis driftless-vm run cargo build --release -p driftless-cli
DRIFTLESS_REPO=$PWD DRIFTLESS_RDIR=driftless-rauschen driftless-vm sync
# Starten
DRIFTLESS_RDIR=driftless-rauschen driftless-vm run bash -c 'DQ_PYTHON=$HOME/td/quality-venv/bin/python DQ_PROBE_BIN=$HOME/driftless-basis/target/release/driftless tools/qualitaet/run_rauschen.sh r1 --abgekoppelt'
```

Ohne `DQ_PROBE_BIN` lässt das Skript die Probe aus und sagt das in `probe.txt`. `DQ_SCANS` braucht genau vier
Ordner in der Reihenfolge S1, S2, S3, Büro. `touchup_real` läuft getrennt, das Skript startet ihn nicht.

Neue Zeilen der Tabelle (aus `oberflaeche.json`):

| Zeile | Quelle |
|---|---|
| Rauheit Wände | `roughness`: Abstand jeder Wand-Ecke zum Mittel ihrer Nachbarn entlang der Normalen |
| Wand zur Ebene der Punkte | `accuracy.walls_plane`: Abstand zur Ebene der Punkte im Umkreis von 3 cm |
| Wand zur Ebene gegen dense | `accuracy_dense.walls_plane`: dasselbe gegen die ungeglättete Wolke |
| Anlegen | `snap`: Ecken, bewegt, ohne Ebene, zu weit weg, zurückgenommen; mittlere Verschiebung |
| Bildrauschen | `texture.noise_before`, `noise_after`: Median je Karte, nach Texeln gewichtet |
| Verdecker | `occluders` |
| ohne Bild (Fläche %) | `texture.untextured_area_share` |
| `seam_step` je m Naht | `seams.seam_step_per_m` |
