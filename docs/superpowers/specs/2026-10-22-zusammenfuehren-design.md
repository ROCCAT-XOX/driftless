# Baustein 1: Mehrere Scans zu einem Modell zusammenführen – Design

Stand 22.10.2026 · Autor: Yannick Stumpf mit Claude · Teil der Reihe nach Meilenstein 5.
Reihenfolge laut Yannick: 2 fotoähnliche Oberfläche → 4 Oberfläche optisch → **1 Zusammenführen +
Dateien öffnen** → 3 Export/Installer (M6) → 5 Planen. Das Modell muss dann direkt als Eingabe des
Oberflächen-Schritts taugen (siehe dessen Design, Abschnitt 6).

## 1. Worum es geht und warum

Ein Raum oder ein Gebäude wird selten in einem Scan erfasst. Yannick scannt **beides**: denselben Raum
mehrmals (hohe Überlappung) und **mehrere Räume, die nur über einen Flur oder eine Tür zusammenhängen**
(wenig Überlappung). Für alles Weitere (Oberfläche, Export, Bemaßen, Markieren, Planen) wird **ein**
gemeinsames Modell gebraucht.

Bisher konnte Driftless das nicht. Ein Wegwerf-Versuch am 22.10. (drei Scans desselben Raums,
Gier-Rastersuche + ICP aus `driftless-odom`, 1-cm-Raster) ergab 90–99 % Überlappung bei 3–4 mm
Restfehler in 40 s. Für den Flur-Fall reicht dieses Durchprobieren nicht sicher.

Außerdem kann das Programm bisher nur Projekte öffnen, keine einzelne Punktwolke. Yannick will
fertige Modelle (PLY, LAS) direkt ansehen.

Erfolg heißt:
- Die drei Scans vom 22.10. ergeben im Programm per Klick dasselbe Ergebnis wie der Versuch.
- Zwei Scans benachbarter Räume, die sich nur einen Flur teilen, werden richtig ausgerichtet, oder das
  Programm sagt klar „unsicher“ und lässt sich mit drei Punktpaaren von Hand helfen.
- `modell.ply` / `modell.las` lassen sich im Programm öffnen und ansehen.

## 2. Ziele

1. **Zusammenführen im Projekt:** mehrere Scans (genauer: je ein SLAM-Lauf mit Karte) auswählen →
   „Zusammenführen“ → ein **Modell** im Projekt, mit Punktwolke (`modell.las`, `modell.ply`) und den
   Lagen der Scans zueinander. Auch über die Kommandozeile (`driftless merge …`).
2. **Robuste automatische Ausrichtung** für beide Fälle:
   - Grobausrichtung über markante Stellen (Merkmale an Ecken/Kanten, Abgleich mit Ausreißer-festem
     Verfahren), gestützt darauf, dass alle Karten schon nach der Schwerkraft ausgerichtet sind
     (nur Drehung um die Hochachse + Verschiebung gesucht).
   - Danach Feinausrichtung (ICP, vorhandener Kern).
   - Die bisherige Rastersuche bleibt als zweiter Versuch, wenn die Merkmale nichts Sicheres liefern.
3. **Kette statt Stern:** Jeder Scan darf an irgendeinen schon platzierten Scan andocken, nicht nur an
   den ersten (Raum 1 → Flur → Raum 3). Bezug (Ursprung des Modells) ist der erste gewählte Scan.
4. **Ehrliche Güte:** je Paar Überlappung und Restfehler; unter einer Schwelle heißt das Paar
   „unsicher“ und wird nicht still übernommen. In der 3D-Ansicht lässt sich das Modell je Scan
   unterschiedlich einfärben (Sichtprüfung).
5. **Hilfe von Hand:** Scheitert die Automatik, klickt Yannick in beiden Scans **drei gleiche Stellen**
   an (z. B. Türecken); daraus wird grob ausgerichtet, dann automatisch verfeinert. Die Punktpaare
   werden gespeichert und sind wie Schleifen ab- und zuschaltbar (nichts wird gelöscht).
6. **Dateien öffnen:** „Datei → Öffnen …“ und Hineinziehen für PLY und LAS (auch fremde Dateien,
   z. B. das Ergebnis von heute). Ohne Projekt nur ansehen.
7. Wie bisher: nichts wird überschrieben, jedes Zusammenführen ist ein eigener Stand, Abbrechen
   hinterlässt keine halben Dateien.

## 3. Nicht-Ziele (später)

- Fotoähnliche Oberfläche (Baustein 2), Export E57/LAZ/Mesh und Installer (Baustein 3).
- Gemeinsame Optimierung aller Laufbahnen über Scan-Grenzen (ein Pose-Graph über alle Scans). Jeder
  Scan bleibt in sich, wie SLAM ihn gerechnet hat; nur die Lage als Ganzes wird bestimmt.
- Stockwerke ohne gemeinsamen Bereich (Treppe nicht mitgescannt): bleibt Handarbeit über Punktpaare.
- Weitere Dateiformate beim Öffnen (E57, PCD, Mesh): kommen mit Baustein 3.

## 4. Was es nicht werden darf

- Ein Modell, das falsch zusammengesetzt ist und trotzdem „ok“ meldet.
- Ein Werkzeug, das Scan-Daten oder Kundennamen ins Repo bringt (Testergebnisse nur in
  `driftless-testdata/`, Namen neutral).
- Fremder Code unter GPL/AGPL oder unklarer Lizenz (Projekt ist Apache-2.0 OR MIT).

## 5. Rahmen

- Mac (Apple Silicon) und Windows, wie bisher; CI beide grün.
- Speicher: die drei Scans vom 22.10. (≈ 17 Mio. Punkte zusammen) unter 4 GB; der Versuch brauchte 1,5 GB.
- Zusammenführen dichter Scans auf 1-cm-Raster; bei Doppelungen gewinnt ein gefärbter Punkt.
- Was zwischen Scans bewegt wurde (Kartons, Personen), darf im Modell auftauchen; Entfernen ist kein Ziel.

## 6. Dem Bauenden überlassen

- Welches Merkmalsverfahren (z. B. FPFH oder eigene, schwerkraftgestützte Ecken-/Kantenmerkmale) und
  welches Ausreißer-feste Abgleichsverfahren, solange lizenzrein und auf beiden Systemen schnell.
- Genaue Schwellen für „unsicher“ (Startwert aus den Testdaten ableiten und in den Ergebnissen belegen).
- Ablage im Projektordner (Vorschlag: `<projekt>/modelle/modell-001/` mit `modell.toml`, Punktwolken,
  Lagen, Punktpaaren, `verlauf/` wie bei Läufen) und Name/Form des CLI-Befehls.
- Reihenfolge des Andockens in der Kette (z. B. immer das Paar mit der besten Überlappung zuerst).

## 7. Testdaten

- Gleicher Raum: `~/Downloads/20261022132520`, `…133028`, `…133554` (je mit `driftless/`).
  Für das Repo nach `driftless-testdata/` kopieren, Ordnername neutral.
- Flur-Fall: noch kein echter Scan vorhanden. Bis dahin aus dem Büro-Scan oder Hilti-Datensatz zwei
  Teilstücke mit wenig Überlappung schneiden; ein echter Flur-Scan von Yannick folgt.
