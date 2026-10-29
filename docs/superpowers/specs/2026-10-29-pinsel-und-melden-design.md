# Pinsel und Melden – Design

Stand 29.10.2026 · Autor: Yannick Stumpf mit Claude · erster von drei Schritten nach Version 0.1.0.
Reihenfolge laut Gespräch vom 29.10.: 1 **Pinsel und Melden** → 2 weniger Rauschen (an den markierten
Stellen gemessen) → 3 Grundriss automatisch aus der Draufsicht (Start von Baustein 5).

Entwürfe aus dem Gespräch (nur lokal, nicht im Repository):
`.superpowers/brainstorm/97448-1793227738/content/nacharbeiten.html` und `melden.html`.

## 1. Worum es geht und warum

Yannick findet Driftless 0.1.0 gut. Auf den Oberflächen sieht er aber stellenweise noch Rauschen und
Stellen mit schlechten Bildern. Solche Stellen will er in der App selbst anmalen und dort nachbessern:
nochmal glätten oder die Bilder besser einspielen. Außerdem will er, wie in seinen anderen Programmen,
mit ⇧⇧ jederzeit einen Fehler oder Wunsch melden. Zentral gespeichert werden muss nichts, auf dem
Rechner reicht.

Die angemalten Stellen und die Meldungen zeigen Claude außerdem, wo die Automatik noch schwach ist.
Das ist die Grundlage für Schritt 2.

Erfolg heißt:
- Im Projekt „Abnahme“ malt Yannick eine rauschige Wandstelle an und wählt „Glätten: ganz eben“.
  Nach wenigen Sekunden ist die Fläche ruhig, und am Rand ist keine Kante zu sehen. „Vorher/Nachher“
  zeigt den Unterschied.
- Eine Stelle mit fleckigen oder unscharfen Bildern sieht nach „Bilder neu einspielen“ ruhiger und
  schärfer aus.
- Die Nacharbeit ist nach einem Neustart noch da. Sie steckt in den Exporten der Oberfläche (glb, OBJ,
  PLY) und lässt sich jederzeit zurücknehmen.
- ⇧⇧ öffnet überall „Melden“. Die Meldung liegt danach mit Bild und Zusammenhang auf dem Rechner,
  Claude findet sie in der nächsten Sitzung, und Yannick sieht unter ☰ → „Meldungen“, was daraus wurde.

## 2. Ziele

### Pinsel

1. **Neues Werkzeug „Pinsel“** in der unteren Leiste der 3D-Ansicht, neben „Messen“ und „Schnitt“.
   Es malt auf die **Oberfläche** einer Karte oder eines Modells. Ohne geladene Oberfläche ist es aus,
   mit einem Hinweis.
2. **Bedienung:**
   - Die linke Taste malt, mit ⌥ (Windows: Alt) radiert sie.
   - Die Pinselgröße steht in cm in der Leiste, Vorgabe 20 cm.
   - Drehen und Verschieben gehen mit den anderen Maustasten weiter.
   - Das Malen läuft flüssig, auch auf der Büro-Oberfläche mit 6,6 Mio. Dreiecken.
3. **Stellen:** Ein angemalter Bereich ist eine Stelle.
   - Links im Feld steht unter der Oberfläche die Liste der Stellen, zum Beispiel „Stelle 1 · Wand“.
     Wand, Boden oder Decke ergibt sich aus der Lage der Fläche.
   - In der Ansicht ist jede Stelle farbig markiert.
   - Malen ohne gewählte Stelle beginnt eine neue. Ist eine Stelle gewählt, malt der Pinsel in sie.
   - Ein Klick in der Liste zeigt die Stelle in der Ansicht.
   - Rechtsklick → „Löschen“ nimmt die Stelle samt ihrer Nacharbeit zurück.
4. **Aktionen** für die gewählte Stelle, in der Leiste über den Werkzeugen:
   - **Glätten: leicht · stark · ganz eben.** „Ganz eben“ legt die Stelle auf eine Ebene, gedacht für
     Wände, Boden und Decke. Ist die Stelle dafür nicht eben genug, sagt die App das in klaren Worten
     und ändert nichts. Der Rand geht weich in die Umgebung über.
   - **Bilder neu einspielen,** automatisch und ohne Fotoauswahl von Hand (Yannicks Entscheidung).
     Die App nimmt für die Stelle schärfere, nähere und geradere Fotos, möglichst eines für die ganze
     Stelle, und gleicht die Übergänge an. Jeder weitere Druck probiert die nächstbeste Variante.
   - Nach dem Glätten spielt die App die Bilder der Stelle selbst neu ein, damit sie auf der
     geglätteten Fläche sitzen.
   - **Melden … (⇧⇧)** öffnet „Melden“ mit dieser Stelle.
   - **Vorher/Nachher** schaltet zwischen gerechneter und nachgearbeiteter Oberfläche.
   - **↶** (auch ⌘Z, unter Windows Strg+Z) nimmt den letzten Schritt zurück.
5. **Nichts geht verloren:**
   - Die gerechnete Oberfläche bleibt unverändert.
   - Die Nacharbeit wird daneben im Projekt gespeichert, nach jedem Schritt.
   - Sie übersteht einen Neustart, das Verschieben oder Kopieren des Projekts und „Aufräumen“.
6. **Neu gerechnet:** Die Stellen gehören zur Karte oder zum Modell, nicht zu einer einzelnen
   Oberfläche.
   - Alle Oberflächen einer Karte oder eines Modells teilen dieselben Stellen. Löschen nimmt eine Stelle
     für alle zurück.
   - Bekommt dieselbe Karte oder dasselbe Modell eine neue Oberfläche (andere Feinheit, neue
     Programmversion), übernimmt sie die Stellen. Die App wendet die Nacharbeit dann wieder an, im
     Hintergrund und mit Fortschritt.
   - Eine neue Karte (korrigierte Laufbahn, andere Genauigkeit, neuer SLAM-Lauf) und ein neu
     zusammengeführtes Modell beginnen ohne Stellen. Die Stellen bleiben bei der alten Karte oder dem
     alten Modell und gehen nicht verloren (Yannick, 29.10.).
7. **Exporte** der Oberfläche (glb, OBJ, PLY) enthalten die Nacharbeit. Punktwolken und Karten
   bleiben unverändert.

### Melden

8. **⇧⇧ öffnet „Melden“,** überall in der App. ⇧⇧ heißt: zweimal kurz Shift, ohne eine andere Taste
   oder Maustaste dazwischen. Shift gedrückt halten löst es nie aus, etwa beim Verschieben oder beim
   schnelleren Fliegen.
9. **Inhalt des Dialogs:**
   - Fehler oder Wunsch, dazu ein Textfeld.
   - Angehakt: „Bild des Fensters anhängen“ und „Markierte Stellen anhängen (n)“.
   - Ohne Zutun kommen dazu:
     - die Ansicht (Kamera)
     - die Auswahl: Scan, Lauf, Karte, Oberfläche oder Modell
     - das Projekt
     - die Programmversion, das System und die Grafikkarte
     - die letzten Zeilen des Protokolls
10. **Nur auf dem Rechner:**
    - Jede Meldung ist ein eigener Ordner im Ordner der App:
      - macOS: `~/Library/Application Support/Driftless/Meldungen/`
      - Windows: `%LOCALAPPDATA%\Driftless\Meldungen\`
    - Die Stellen liegen als Kopie dabei. So bleibt die Meldung lesbar, auch wenn das Projekt
      verschoben oder gelöscht wird.
    - Nichts wird hochgeladen.
11. **Unter ☰ → „Meldungen“** stehen alle Meldungen des Rechners, die neueste zuerst. Jede zeigt Art,
    Zeit, Projekt, Text, Stand („offen“ oder „erledigt …“) und Antwort. „Zur Ansicht“ öffnet das
    Projekt und stellt Auswahl und Kamera wieder her.
12. **Stand und Antwort** stehen als Klartext in der Meldung. So kann Claude sie in einer Sitzung lesen
    und zum Beispiel „erledigt in 0.2.0: …“ eintragen. Die Liste zeigt das beim nächsten Öffnen.
    Claude darf die Meldungen samt Bildern lesen (Yannick, 29.10.). Den Ort merkt sich Claude im
    eigenen Gedächtnis, nicht im Repository; die Anleitung nennt den Ordner für die Nutzer.

## 3. Nicht-Ziele

- Fotos von Hand auswählen.
- Löcher füllen, Teile wegschneiden, Punktwolken oder Karten bearbeiten.
- Meldungen zentral sammeln, zwischen Rechnern abgleichen oder nachts automatisch auswerten.
- Nacharbeit über die Kommandozeile; die CLI bleibt, wie sie ist.
- Die automatische Glättung aller Flächen verbessern: Das ist Schritt 2.

## 4. Was es nicht werden darf

- Die gerechnete Oberfläche wird verändert oder geht verloren, oder eine Nacharbeit lässt sich nicht
  zurücknehmen.
- Am Rand einer bearbeiteten Stelle sieht man eine Kante, einen Sprung oder einen Farbbruch.
- Die Nacharbeit verschwindet nach einem Neustart, nach dem Neurechnen, nach dem Verschieben des
  Projekts oder nach „Aufräumen“.
- Das Malen ruckelt, oder eine Aktion blockiert die App. Aktionen laufen im Hintergrund, mit
  Fortschritt, und lassen sich abbrechen.
- ⇧⇧ löst beim Verschieben mit Shift oder beim Tippen von Großbuchstaben aus.
- Die App lädt Meldungen hoch oder teilt sie, oder Meldungen mit Bildern von Kundenräumen landen im
  Repository.

## 5. Rahmen

- Rust-Workspace wie bisher (egui/eframe 0.36, wgpu), für macOS und Windows. Die Texte sind deutsch
  und schlicht und stehen in `crates/driftless-app/src/ui/text.rs`.
- Fertige Ergebnisordner bleiben Byte für Byte so, wie sie gerechnet wurden; `surface_fits` prüft die
  Größen der Dateien im Bericht.
- Vorbild für Daten, die der Nutzer anlegt, ist `modelle/punktpaare.toml`: im Projekt, mit relativen
  Pfaden.
- Keine Datei der Nacharbeit endet auf `.tmp`. „Aufräumen“ und die Erkennung abgebrochener Läufe
  (`crates/driftless-app/src/leftovers.rs`) lassen sie stehen.
- Die Oberfläche entsteht in zwei Hälften: `build_mesh` und `texture_mesh` in
  `crates/driftless-map/src/surface`.
- „Bilder neu einspielen“ braucht die Kamerabilder und Lagen der Scans aus den Rohdaten. Fehlen die
  Rohdaten, etwa weil das Laufwerk abgezogen ist, ist die Aktion aus, mit Hinweis. Glätten geht
  trotzdem.
- Die Grafikkarte zeichnet die Ansicht, die Markierung und Vorher/Nachher. Die Nacharbeit selbst rechnet
  der Prozessor: Sie betrifft nur eine Stelle, ist damit klein, und so fällt sie auf jedem Rechner Byte
  für Byte gleich aus (die Grafikkarte rechnet nur bis auf Rundung gleich; Grenze siehe Nachtrag im
  nächsten Punkt).
- Gleiche Stellen und Schritte ergeben dasselbe Ergebnis, Byte für Byte wie der Rest der Kette.
  Nachtrag nach dem Gesamt-Review (29.10.): gleiche Bytes auf demselben System (Betriebssystem
  und Prozessor). Über Systeme hinweg können neu eingespielte Seiten in seltenen Texeln
  abweichen, wie die Seiten der Oberfläche selbst: Kamerastellung, Projektion und
  Helligkeitsangleich gehen durch `sin`, `atan2`, `exp` und `ln`, deren letztes Bit jede
  Mathematik-Bibliothek anders runden kann. Flächen und Punktlagen (Auswahl, Glätten) sind
  überall gleich. Ein mit dem Projekt verschobener Zwischenspeicher gilt weiter.
- Schwere Läufe mit echten Daten laufen auf VM 101 (6 Threads, `nice 10`).
- Scandaten, Meldungen und Kundennamen kommen nie ins Repository.

## 6. Wie geprüft wird

- **Tests ohne Fenster** (`egui_kittest`, wie `whole_run_real`):
  - Malen, Radieren, jede Aktion, Rückgängig, Vorher/Nachher
  - Neustart und verschobenes Projekt
  - eine neue Oberfläche übernimmt die Stellen
  - der Export enthält die Nacharbeit
  - ⇧⇧ gegen gehaltenes Shift
  - eine Meldung schreiben und wieder lesen
  - ein von außen eingetragenes „erledigt“ erscheint in der Liste
- **Messung auf echten Daten** (Modell „Abnahme“ und Büro):
  - Abstand der Punkte einer geglätteten Wandstelle zur Ebene, vorher und nachher
  - Sprung am Rand der Stelle
  - Zeit je Aktion für eine Stelle von etwa 1 m²; Ziel sind wenige Sekunden auf dem Mac
  - Bildfrequenz beim Malen auf der Büro-Oberfläche
- **Bilder vorher und nachher,** ohne Fenster gerendert, für Yannick.

## 7. Von Claude entschieden (zum Gegenlesen)

- Die Stellen gehören zur Karte oder zum Modell und gelten für alle ihre Oberflächen (Ziel 6).
- Wand, Boden oder Decke im Namen einer Stelle ergibt sich aus der Lage der Fläche.
- Jeder weitere Druck auf „Bilder neu einspielen“ probiert die nächstbeste Variante.
- Nach dem Glätten werden die Bilder der Stelle von selbst neu eingespielt.
- „Ganz eben“ ändert nichts, wenn die Stelle nicht eben genug ist, und sagt warum.
- Radieren mit ⌥ (Windows: Alt).
- Meldungen liegen im Ordner der App, nicht im Projekt, mit einer Kopie der Stellen.
- In der Liste der Meldungen führt „Zur Ansicht“ zurück zur gemeldeten Ansicht.
- Die Kommandozeile bleibt, wie sie ist.

## 8. Dem Bauenden überlassen

- Dateiformat und Ort der Stellen und der Nacharbeit im Projekt, und wo die nachgearbeitete Oberfläche
  zwischengespeichert wird.
- Wie der Pinsel die Fläche trifft (Strahl gegen Dreiecke oder Tiefenpuffer) und wie eine Stelle
  abgegrenzt wird.
- Das Glätten im Einzelnen: Verfahren, Stärke der Stufen, Schwelle für „nicht eben genug“, weicher Rand.
- Wie „Bilder neu einspielen“ die nächstbeste Variante findet und wie viele es gibt.
- Aufbau der Meldungsordner (Dateinamen, Format), solange Stand und Antwort als Klartext lesbar und
  schreibbar bleiben.
- Die genaue Anordnung in der App, im Rahmen der Entwürfe.
