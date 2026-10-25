# Baustein 4: Neue Ansicht – Design

Stand 24.10.2026 · Autor: Yannick Stumpf mit Claude · Grundlage: Entwurf „B · Leiste links“
(`.superpowers/brainstorm/…/layout-b-v2.html`), von Yannick freigegeben („optisch gefällt mir das sehr gut,
genauso umsetzen“).

## 1. Worum es geht und warum

Driftless rechnet schnell und gut. Yannick findet die Geschwindigkeit „sehr gut“. Das Programm sieht aber
technisch aus und hat zu viele Einstellungen. Das rechte Feld zeigt heute Punktgröße, Punktbudget, Ebenen,
Laufbahn-Vergleich, Korrekturen und Schnitt untereinander. Yannick will, dass es **moderner wirkt und einfacher
zu bedienen ist**. Die Arbeit soll in der Reihenfolge ablaufen, in der er sie macht:

1. **Daten:** Punktwolke laden und möglichst genau rechnen. Hier sieht man die Punktwolke.
2. **Bilder:** Foto-Oberfläche (Baustein 2). Hier sieht man die Oberfläche.
3. **Planung:** Grundriss und Maße, später DWG/DXF (Baustein 5), damit man die Maße sauber sieht.

Erfolg heißt: Yannick kann einen Scan ohne Anleitung von „Daten“ bis „Bilder“ bringen, sieht in jedem
Schritt nur, was er dafür braucht, und das Programm wirkt wie ein aktuelles Profi-Werkzeug.

## 2. Ziele

1. **Aufbau B:**
   - Links eine schmale Leiste (≈ 60 px) mit Symbolen: oben Projekt (☰), dann **1 Daten**, **2 Bilder**,
     **3 Planung**, unten Einstellungen (⚙). Der aktive Schritt ist farbig hervorgehoben.
   - Daneben ein Feld (≈ 230–280 px) mit dem Inhalt des aktiven Schritts.
   - Der Rest ist die 3D-Ansicht.
   - Werkzeuge (Drehen, Verschieben, Messen, Schnitt, Alles zeigen) schweben unten mittig als
     halbdurchsichtige Leiste über der Ansicht.
   - Oben rechts schwebt der Umschalter Punkte · Fläche · beides, oben links eine kleine Info (z. B.
     „Punktwolke · 6,0 Mio. Punkte“).
2. **Schritt 1 „Daten“:**
   - Liste der Scans im Projekt mit Status (neu, rechnet mit Fortschritt, fertig ✓, veraltet) und
     „+ Scan hinzufügen“.
   - Genauigkeit als dreiteiliger Umschalter **Schnell / Genau / Maximal**.
   - Ein Hauptknopf „Alle rechnen“ bzw. „Rechnen“ für den gewählten Scan.
   - Die Ansicht zeigt die Punktwolke.
   - Laufbahn-Vergleich, Korrekturen (Schleifen), Teilkarten und verworfene Kandidaten liegen unter
     „▸ Erweitert“.
3. **Schritt 2 „Bilder“:**
   - Je Scan bzw. Karte: Oberfläche rechnen (Feinheit als Umschalter, „ohne Fotos“ unter Erweitert),
     Status und veraltet-Hinweis.
   - Die Ansicht zeigt die Oberfläche, Umschalten auf Punkte bleibt möglich.
4. **Schritt 3 „Planung“:**
   - Der Platz und die Leiste sind schon da.
   - Das Feld zeigt einen klaren Hinweis, was hier kommt (Grundriss, Maße, DWG).
   - Das Messwerkzeug (Abstand zwischen zwei Punkten) ist schon nutzbar.
   - Die echten Planungsfunktionen sind Baustein 5.
5. **Weniger Einstellungen:**
   - Normal sichtbar ist nur, was eine Entscheidung braucht.
   - Punktgröße, Punktbudget, Farbe/Intensität/Güte, Ebenen und z-Grenzen liegen unter Erweitert oder
     im Einstellungsfenster ⚙.
   - Sinnvolle Vorgaben sind gesetzt, und das Programm merkt sie sich (persistence ist schon an).
6. **Modernes Aussehen:**
   - Dunkles Thema mit ruhigen Grautönen (≈ #0f1115 / #151920 / #1b1f27) und eine Akzentfarbe Blau
     (≈ #2f6fed).
   - Abgerundete Ecken (8–12 px), genug Abstand, klare Schriftgrößen (Überschrift, Text, gedämpft).
   - Halbdurchsichtige schwebende Leisten über der Ansicht.
   - Hintergrund der 3D-Ansicht als weicher Verlauf statt flachem Schwarz.
7. **Punktwolke wirkt wie ein Bild:**
   - Yannick kennt das aus dem Herstellerprogramm: Die farbigen Punkte sind kleine Bildflecken, und schon
     beim Bewegen sieht die Wolke aus wie ein gerendertes Bild.
   - Dafür werden Punkte als Flecken mit fester Größe in Metern gezeichnet statt mit fester Größe in Pixeln.
     Die Größe liegt etwa beim Punktabstand der Karte. Nah sind die Flecken groß, fern klein, und die Fläche
     wirkt geschlossen.
   - Die Flecken haben weiche runde Ränder und sind farbig: Farbe ist die Vorgabe in Schritt 1.
   - Die Größe wird nach unten und oben in Pixeln begrenzt, damit ferne Punkte nicht verschwinden und
     nahe nicht riesig werden.
   - Der heutige Pixel-Modus bleibt unter ⚙ wählbar.
   - Die Geschwindigkeit muss bleiben, siehe Abschnitt 4.
8. **Statusleiste:** unten schlank, mit letzter Meldung und laufendem Auftrag mit Fortschritt; das Protokoll
   lässt sich aufklappen wie heute.

## 3. Nicht-Ziele

- Neue Rechenfunktionen. Zusammenführen ist Baustein 1, Export Baustein 3, Planung/DWG Baustein 5.
- Ein helles Thema. Es wird nur vorbereitet, über Farben aus einer Stelle.
- Ein Wechsel des UI-Baukastens: Es bleibt bei egui 0.36 + wgpu.
- Änderungen an Rechenergebnissen.

## 4. Was es nicht werden darf

- **Langsamer:** Die Geschwindigkeit der Ansicht (Bildrate, Laden) bleibt wie heute. Yannick hat sie
  ausdrücklich gelobt.
- **Funktionen verloren:** Alles, was heute geht, ist weiter erreichbar, notfalls unter Erweitert:
  Korrekturen, Vergleich, Schnitt, Punkt anklicken, Oberfläche öffnen, Projekt anlegen/öffnen.
- **Englische oder technische Begriffe** in der Oberfläche. Alle Texte stehen auf Deutsch in `text.rs`.
- **Nur auf dem Mac gut:** Windows (DX12/Vulkan) und Bildschirme mit Skalierung 1× und 2× müssen sauber
  aussehen.

## 5. Rahmen

- Bestehender Code: `crates/driftless-app/src/ui/` (left, right, view3d, status, text, actions …).
  Die Schritt-Logik (Projekt, Aufträge, veraltet) bleibt. Neu angeordnet wird die Darstellung.
- Die UI-Tests mit `egui_kittest` bleiben grün und werden auf den neuen Aufbau angepasst. Für die drei
  Schritte kommen Bildvergleichs-Tests (Snapshots) dazu.
- Symbole: eine Symbolschrift mit MIT/Apache/OFL-Lizenz (z. B. Phosphor oder Lucide); `cargo deny` muss grün
  bleiben. Ohne passende Schrift: einfache eigene Zeichnungen.
- Die Schrift bleibt die eingebaute egui-Schrift, oder Inter (OFL), wenn das Bild dadurch deutlich ruhiger
  wird.
- Kein schwerer Rechenlauf auf dem Mac. Zum Ansehen reichen vorhandene Ergebnisse.

## 6. Entscheidungen von Claude (nicht von Yannick bestätigt)

- **Von Yannick bestätigt (24.10.):**
  - „Genauigkeit“ in Schritt 1 stellt nur das Raster der Punktwolke ein. Schritt 2 hat eine eigene
    Feinheit 1 / 1,5 / 2 cm.
  - „Alle rechnen“ in Schritt 1 rechnet nacheinander die Punktwolken aller neuen oder veralteten Scans.
    Schritt 2 hat einen eigenen Knopf „Alle Oberflächen rechnen“.
- Die Zuordnung der drei Stufen „Schnell / Genau / Maximal“ zu den heutigen Karten-Einstellungen legt der
  Bauende fest. Sie richtet sich nach den heute erlaubten Werten. „Genau“ entspricht der heutigen Vorgabe,
  sodass sich die Ergebnisse nicht ändern. Bei „Maximal“ steht ein Speicherhinweis.
- Projekt anlegen/öffnen liegt hinter dem Symbol ☰ (Menü bzw. Feld „Projekt“), nicht in einer eigenen
  Menüzeile.
- Wechselt man den Schritt, schaltet die Ansicht automatisch passend um (Daten → Punkte, Bilder → Fläche).
  Der Umschalter oben rechts kann das jederzeit ändern.
- Die Messfunktion in der Werkzeugleiste ist schon in Baustein 4 dabei (zwei Punkte anklicken → Abstand in m),
  als Vorgriff auf die Planung.
- Nur dunkles Thema; Farben zentral in einem Modul `theme.rs`.
- „Maximal“ wird nicht gesperrt, sondern zeigt nur den Speicherhinweis.
- Alte gespeicherte Fensterzustände werden beim ersten Start verworfen. Gespeicherte Vorgaben wie
  Genauigkeit oder Punktgröße werden übernommen, wo es sie noch gibt.
- **Ansicht beim Schrittwechsel:**
  - Bei jedem Schrittwechsel wird die passende Ansicht neu gesetzt. Ein Umschalten von Hand gilt nur
    innerhalb des Schritts.
  - Gibt es in „Bilder“ noch keine Oberfläche, bleibt die Punktwolke sichtbar. Dazu kommt ein Hinweis
    „Noch keine Oberfläche – rechts rechnen“.
- **Werkzeuge:**
  - Es ist immer genau ein Werkzeug aktiv: Drehen/Verschieben (Kamera), Punkt wählen, Messen oder Schnitt.
  - Messwerte stehen in der Ansicht und im Feld „Planung“. Sie werden erst mit Baustein 5 gespeichert.
- **Erweitert und ⚙:**
  - ⚙ enthält Einstellungen für das ganze Programm: Punktgröße, Punktbudget, Farbart und Bedienung.
  - „Erweitert“ im Feld enthält Dinge, die zu einem Schritt gehören: Ebenen, Vergleich, Korrekturen,
    Teilkarten und „ohne Fotos“.
  - Schnittgrenzen gehören zum Werkzeug Schnitt.
- **Snapshots:** Die Bildvergleiche laufen auf dem Mac mit Skalierung 1× und lockerer Pixel-Toleranz.
  Windows und 2× werden von Hand mit Bildschirmfotos geprüft.

## 7. Dem Bauenden überlassen

- Genaue Maße, Abstände, Schriftgrößen und Farbwerte (nah am Entwurf).
- Wie „Erweitert“ aussieht (aufklappbar im Feld oder eigenes Fenster ⚙) – Hauptsache, normal unsichtbar.
- Aufteilung in Aufgaben und welche heutigen Panels zusammengelegt werden.
- Ob die schwebenden Leisten echte Unschärfe bekommen (in egui nur angedeutet: halbdurchsichtig reicht).

## 8. Testdaten

Vorhandene Ergebnisse unter `~/Downloads/20261022133554/driftless/` (Karte und Oberflächen) und das
Testprojekt im Scratchpad. Bildschirmfotos der drei Schritte gehen an Yannick zur Abnahme.
