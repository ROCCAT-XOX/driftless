# Qualitätsrunde – Design

Stand: 2026-10-25 · Grundlage: Qualitätskampagne vom 24./25.10. (Bericht in
`.superpowers/quality/2026-10-24-qualitaet-report.md`, nicht im Repo) · Branch `qualitaetsrunde` von
`main` nach Baustein GPU.

## Warum

Yannick: „die bestmöglichen Ergebnisse aus den Vorlagen-Scans rausziehen. Es geht nicht um
Rechenleistung … Qualität der Scans hochhalten, Datenausgabe gut, sehr gute Werte am Schluss.“

Die Kampagne hat gezeigt: Die feinste Einstellung (Karte 0,5 cm, Oberfläche 1 cm) macht Wände und
Kanten deutlich genauer, reißt aber Löcher in die Oberfläche. Und unter allen Einstellungen liegt
dieselbe Wand aus verschiedenen Durchgängen 6–9 mm auseinander; das Glätten verdeckt das nur.
Diese Runde behebt die Ursachen, gereiht nach Wirkung, und misst nach jedem Schritt nach.

## Ziele (gemessen an den vier Vorlagen-Scans, Einstellung „Maximal“)

Vorlagen: schwarzer Raum 20261022133554, 20261022132520, 20261022133028; Büro 20261020173822.
Ausgangswerte = Variante b der Kampagne (Karte 0,5 cm, Oberfläche 1 cm, GPU).

| Nr | Messgröße | heute (b) | Ziel |
|---|---|---|---|
| Z1 | Löcher von oben, schwarzer Raum 133554 / 133028 / 132520 | 9,8 / 23,4 / 44,4 % | ≤ 2 / 4 / 6 % (so gut wie a) |
| Z1 | Löcher von oben, Büro | 72,5 % | ≤ 30 % |
| Z2 | Oberfläche Wände Median, alle Scans | 3,0–3,7 mm | nicht schlechter als +0,2 mm |
| Z3 | Versatz derselben Wand zwischen Durchgängen (dense), Median | 6,6–8,8 mm | ≤ 5 mm |
| Z3 | dense: Ebene lokal RMS, schwarzer Raum 133554 | 6,2 mm | ≤ 4,5 mm |
| Z4 | Büro: Restlücke Start↔Ende nach Schleifenschluss | 129 mm | ≤ 50 mm |
| Z4 | Büro: Gegenwände nicht parallel | 1,26° | ≤ 0,3° |
| Z5 | Raummaße der drei Schwarzraum-Scans, Spanne | 0,8 / 1,1 / 2,1 mm | bleibt ≤ 2,5 mm |
| Z6 | Texturkarten, schwarzer Raum 133554 | 167.424 | ≤ 80.000 |
| Z6 | `seam_step` Mittel | 20–23 | ≤ 15 |
| Z6 | `.glb` schwarzer Raum 133554 | 91 MB | ≤ 60 MB |

**Toleranzen für „nicht schlechter“** (Laufrauschen): Z2 +0,2 mm, Z3 +0,3 mm, Ebenen-RMS +0,3 mm,
Z5 bleibt ≤ 2,5 mm; für die kleinen Räume zusätzlich Start↔Ende höchstens +5 mm und Schleifen-RMS
max höchstens +3 mm gegenüber b. Schritt A misst das Laufrauschen einmal (derselbe Scan zweimal); ist
es größer als diese Toleranzen, gilt das gemessene Rauschen.

Ein Ziel, das ein Schritt nicht erreicht, wird im Ergebnisbericht mit Messwert und Grund benannt; die
Runde geht trotzdem weiter. **Kein Schritt wird übernommen, der einen der vier Scans in Z2, Z3 oder Z5
verschlechtert.**

## Nicht-Ziele

- Scans zusammenführen (Baustein 1), Planung/DWG (Baustein 5), Export/Installer (Baustein 3).
- Neue Bedienelemente im Programm. Neue Einstellungen nur, wo eine Messung zeigt, dass ein fester
  Wert nicht für alle Räume passt; dann automatisch gewählt, nicht vom Nutzer.
- Rechenzeit sparen. Längere Läufe sind erlaubt (Grenze: ein Scan „Maximal“ unter 30 min auf VM 101
  mit RTX; heute b: schwarzer Raum ≈ 6 min, Büro ≈ 11 min).
- Eigene Wege je Genauigkeitsstufe: Die Änderungen gelten für alle Stufen (eine Pipeline); die Ziele
  werden an „Maximal“ gemessen, „Genau“ darf nicht schlechter werden als heute (a).

## Anti-Ziele

- Werte, die nur auf dem Papier besser werden (z. B. stärker glätten, damit die Oberfläche näher an
  der geglätteten Wolke liegt). Deshalb kommt die ehrliche Messung zuerst (Schritt A).
- Scan-Daten, Bilder daraus oder Kundennamen im Repo.
- Abweichung von der halben Last auf PVE Privat (6 von 12 vCPU, `nice 10`, ein schwerer Lauf zur
  Zeit).

## Schritte

Reihenfolge fest: A → B → C → D → E → F. Nach jedem Schritt ein Messlauf der vier Scans auf VM 101,
Tabelle im Ergebnisbericht `docs/ergebnisse/2026-10-25-qualitaetsrunde.md` (nur Zahlen, keine
Bilder, keine Pfade mit Kundennamen).

### A – Ehrlich messen (Engpass 4, Messteil)

- `map.json` → `steps.clean` bekommt die Kennwerte **vor** dem Glätten: Wanddicke (heute schon
  `thickness_before`) und neu den **Versatz zwischen Durchgängen** (Median, p90; Durchgänge über
  Zeitlücken > 5 s wie in der Kampagne) sowie Ebene lokal RMS auf `dense`.
- `oberflaeche.json` → `accuracy` zusätzlich gegen die **ungeglättete** Wolke (`accuracy_dense`).
- Ein Messwerkzeug für die Tabelle oben: Löcher von oben und Rundblick (wie `surface_real.rs`),
  Raummaße (große Ebenen), Versatz, Laufbahnwerte aus `slam.json`.
- Kein Einfluss auf Ergebnisse; die Versionen der Schritte bleiben.
- Form: Die Kennwerte, die jeder Lauf braucht (Versatz, Dicke, Ebene lokal RMS vor dem Glätten;
  `accuracy_dense`), rechnet das Programm selbst in Rust und schreibt sie in `map.json` /
  `oberflaeche.json`. Die Auswertung der Vorlagen (Raumebenen, Raummaße, Löcher aus Ansichten,
  Tabelle) bleibt Python und kommt als Entwicklerwerkzeug nach `tools/qualitaet/` ins Repo, ohne
  Daten. Dazu ein Lauf-Skript für die vier Scans auf VM 101. Die Daten liegen dort unter
  `~/td/jmk6-2026-10-22/{20261022133554,20261022132520,20261022133028}` und
  `~/td/jmk6-2026-10-20/20261020173822`; normale `cargo test` brauchen sie nicht.
- **Definitionen = die der Kampagne** (Python in `.superpowers/quality/skripte/q-analyze.py`,
  `q-views.py`, Bericht `.superpowers/quality/2026-10-24-qualitaet-report.md`, beides lokal, nicht im
  Repo). Die Rust-Kennwerte müssen die Python-Werte der Kampagne auf denselben Daten bis auf 1 mm treffen
  (sonst stehen beide im Bericht und die Python-Werte gelten).

### B – Oberfläche ohne Löcher bei 1 cm (Engpass 1)

- Ursache: Bei 1 cm erreichen schwach getroffene Flächen (schwarz, Boden) nicht `min_weight = 2`;
  Marching Cubes lässt jede Zelle mit einer leeren Ecke weg.
- Vorgabe: **TSDF in zwei Stufen.** Neben dem feinen Gitter läuft ein grobes mit doppelter
  Kantenlänge. Wo das feine Gitter zu wenig Stimmen hat, kommt der Wert aus dem groben (trilinear),
  sodass die Fläche geschlossen bleibt; wo das feine genug hat, gilt das feine.
- Alternative, falls die Messung damit Z1 nicht erreicht: Stimmen je Strahl gewichtet auf die
  Nachbarvoxel verteilen. Entscheidung nach Messung, im Bericht begründet.
- Übergänge fein ↔ grob dürfen keine Risse oder Stufen > 1 Voxel im Netz erzeugen (Test).
- Oberflächen-Version steigt (ältere gelten als veraltet).

### C – Wände aus allen Durchgängen an derselben Stelle (Engpass 3)

Befund aus dem Code: Die Lagen der Frames innerhalb einer Teilkarte werden nach dem Schließen nie
mehr verändert (nur die Anker im Posengraph); die dichte Karte setzt die Frames mit ihrer
Odometrie-Entzerrung (bis Frame-Ende, IMU-Vorhersage) an die korrigierte Laufbahn, ohne neu zu
entzerren. Es gibt keinen Zeitversatz LiDAR↔IMU und als Extrinsik die Identität.

- **C0 Ursache eingrenzen (zuerst, billig):** Hängt der Versatz an der Lage (Durchgänge starr
  zueinander nachregistriert → Versatz sinkt) oder am Sensor (Versatz abhängig von Flächenfarbe/
  Intensität, Auftreffwinkel oder Entfernung)? Ist es der Sensor, ersetzt eine Entfernungs-/
  Intensitätskorrektur C1–C2; das Ergebnis steht im Bericht, bevor C1 gebaut wird.
- **C1 Frames nachziehen:** Nach dem Schleifenschluss jeden Frame per Punkt-zu-Ebene-ICP gegen die
  fertige Karte (aus allen Frames, ohne den eigenen Durchgang zu bevorzugen) registrieren; zwei
  Runden (Karte neu bauen, nochmal). Schutz: Korrektur je Frame begrenzt (Vorschlag 5 cm / 1°),
  schlecht bestimmte Frames behalten ihre Lage.
- **C2 Neu entzerren:** Punkte eines Frames mit der korrigierten Laufbahn (Bewegung zwischen den
  nachgezogenen Frame-Lagen) neu entzerren statt mit der IMU-Vorhersage.
- **C3 Extrinsik und Zeitversatz prüfen:** Aus den Daten schätzen, ob Hebelarm LiDAR↔IMU oder ein
  Zeitversatz den Versatz erklärt (Suche über kleinen Bereich, Maß = Versatz zwischen Durchgängen).
  Nur übernehmen, wenn alle vier Scans besser werden; dann als fester JMK6-Standard mit Beleg.
- Wo das lebt (SLAM-Schritt mit neuer `traj.tum`, oder Karten-Schritt): Sache des Bauplans; die
  Laufbahn, die die App anzeigt, und die Karte müssen dieselbe sein.
- SLAM-/Kartenversion steigt.

### D – Laufbahn im Büro (Engpass 2)

- Ziel Z4 ohne die kleinen Räume zu verschlechtern (in der Kampagne machte feinere Odometrie
  132520 schlechter: 11 → 77 mm).
- Hebel in dieser Reihenfolge, jeder einzeln gemessen:
  1. Verworfene Schleifen im Büro nach Grund auswerten (115 von 135, fast alle „zu wenig
     Überlappung“), Schwellen/Kandidatensuche danach anpassen.
  2. Odometrie-Auflösung an die Raumgröße bzw. Punktdichte koppeln statt fest 0,3 / 0,5 m.
  3. Eine Laufbahn nur übernehmen, wenn die Kartenwerte aus A nicht schlechter werden.
- Automatische Kopplung an Punktdichte/Raumgröße ist erlaubt; sie gilt für alle Scans und muss die
  Toleranzen der kleinen Räume einhalten.
- C wirkt hier mit (Frames nachziehen); D wird deshalb erst nach C gemessen.

### E – Glätten neu einstellen (Engpass 4, Rest)

- Nach C: `max_shift` (3 cm) und ggf. Radius so senken, dass das Glätten Rauschen nimmt, aber keine
  Doppelwände mehr zusammenschiebt. Maß: `accuracy_dense` aus A darf nicht schlechter werden,
  Wanddicke nach dem Glätten ≤ 3 mm.

### F – Nähte und Texturkarten (Engpass 5)

- Nachbardreiecke mit derselben besten Ansicht zu größeren Karten zusammenlegen.
- Nahtausgleich nach den großen Sprüngen (p95 von `seam_step`) regeln statt nach dem Mittel.
- Auf hellen Lichtquellen die Zahl der beteiligten Ansichten je Dreieck begrenzen.
- Ziel Z6; die Seitenfarben-Regel aus Baustein GPU (je Dreieck vor dem Packen) gilt weiter.

## Rahmen

- Schwere Läufe nur auf VM 101 über `driftless-vm` / `gpu-job`, halbe Last.
- Jeder Schritt: Tests (Rust, wie bisher), Messlauf, Zeile im Ergebnisbericht, eigener Commit-Satz.
- Neue Versionsnummern machen alte Karten/Oberflächen veraltet; das ist gewollt.
- Push und Merge nur nach Yannicks Okay.

## Entscheidungen

- **Yannick (24./25.10.):** Qualität vor Rechenzeit; RTX/Proxmox nutzen; Reihenfolge der Engpässe
  1 → 3 → 2 → 4 → 5; Zusammenführen erst danach.
- **Claude:**
  - Messteil von Engpass 4 (A) kommt zuerst, weil C und E sonst nicht ehrlich bewertbar sind; der
    Rest von 4 (E) erst nach C.
  - Zielwerte der Tabelle oben.
  - Grenze 30 min je Scan „Maximal“.
  - Zwei-Stufen-TSDF als erste Wahl für B.
  - C0 vor C1, C1 vor C3 (Extrinsik nur mit Beleg).
  - Toleranzen für „nicht schlechter“, Laufrauschen-Messung in A.
  - Laufkennwerte in Rust (`map.json`/`oberflaeche.json`), Vorlagen-Auswertung als Python-Werkzeug
    in `tools/qualitaet/`, Definitionen aus der Kampagne.
  - Änderungen für alle Genauigkeitsstufen; Büro-Ziel verfehlt → benennen, kein Lochfüllen.
