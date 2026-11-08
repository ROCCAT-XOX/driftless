# Baustein 5, Start: Grundriss automatisch – Design

Stand 08.11.2026 · Autor: Yannick Stumpf mit Claude · dritter Schritt nach Version 0.1.0.
Reihenfolge laut Gespräch vom 29.10.: 1 Pinsel und Melden → 2 weniger Rauschen → 3 **Grundriss automatisch aus
der Draufsicht** (Start von Baustein 5). Grundlage: Gespräche vom 22.10. und 29.10., Feld „Planung“ aus
`2026-10-24-ansicht-design.md`. Es gab keine Rückfragen; die Entscheidungen stehen mit Begründung in Abschnitt 10.

## 1. Worum es geht und warum

Aus einem Scan entsteht heute eine Punktwolke und eine Oberfläche. Wer einen Raum planen will, braucht aber einen
**Grundriss**: Wände, Türen, Fenster, Maße, und dazu Steckdosen, Kabel, Anschlüsse und Rohre als eigene Objekte. Das
Ganze muss in den Programmen ankommen, in denen weitergeplant wird, also als **DXF**.

Bisher kann Driftless nur zwei Punkte anklicken und den Abstand zeigen, und auch der geht beim Schließen verloren.
Das Feld „Planung“ ist ein Platzhalter.

Erfolg heißt:
- Ein Klick auf „Grundriss rechnen“ macht aus der Karte oder dem Modell eines Raums in Sekunden einen Plan mit
  geschlossenen, rechtwinkligen Wänden, Türen, Fenstern und Maßen. Die Maße des dunklen Raums (etwa 8,3 × 7,9 m)
  stimmen auf wenige Zentimeter, in jedem Einzelscan und im Modell.
- Was das Programm nicht sicher weiß, steht als „unklar“ im Plan, statt zu raten.
- Steckdose, Kabel, Anschluss und Rohr setzt man in der 3D-Ansicht. Sie erscheinen im Plan und im DXF.
- Das DXF öffnet ohne Fehler in einem freien CAD-Programm, mit Millimetern, deutschen Layern und lesbaren Umlauten.

## 2. Ziele

1. **Grundriss rechnen** für eine Karte oder ein Modell, in der App und mit `driftless plan`. Eingabe ist die
   Punktwolke (`color.las`, sonst `clean.las`; beim Modell `modell.las`) und die Laufbahn. Eine Oberfläche ist nicht
   nötig.
2. **Ein Stockwerk je Karte oder Modell.** Boden und Decke findet das Programm selbst. Hat die Karte mehrere
   Stockwerke, wird nichts geraten: Es nennt die gefundenen Stockwerke, und man wählt eines aus.
3. **Wände** als Linien der gemessenen Wandflächen. Rechte Winkel bleiben exakt rechte Winkel, schräge Wände bleiben
   schräg. Sind beide Seiten einer Wand gemessen (zwei Räume im Modell), ist auch die Wanddicke bekannt.
4. **Türen und Fenster** als Öffnungen in den Wänden, mit Breite, Höhe und Brüstung. Was nicht sicher ist, kommt auf
   einen eigenen Layer („Öffnung-unklar“) und sagt, warum.
5. **Rohre** werden in dieser Runde von Hand gesetzt. Der Layer „Rohr“ ist da, Liste und Anleitung sagen „von Hand ·
   automatisch später“ (Begründung: Abschnitt 3.5).
6. **Räume und Maße:** je Raum ein geschlossener Umriss mit Name, Fläche und Höhe, je Wand ihre Länge, je Öffnung ihre
   Größe. Die Maße stehen im Plan und im DXF.
7. **Objekte von Hand** in der 3D-Ansicht: Steckdose, Anschluss (Strom, Wasser, Abwasser, Heizung, Gas, Netzwerk,
   Sonstiges), Kabel als Linienzug, Rohr als Linienzug mit Durchmesser. Jedes Objekt hat Nummer, Name, Notiz und freie
   Eigenschaften. Die Datei hält fest, woher ein Objekt stammt („hand“), damit die automatische Erkennung später
   dieselben Objekte schreiben kann.
8. **Messungen werden gespeichert** (angekündigt in `2026-10-24-ansicht-design.md`): Sie gehören wie die Objekte zur
   Karte oder zum Modell, überstehen einen Neustart und stehen im Plan und im DXF.
9. **Plan in „Planung“:** 2D-Ansicht mit Draufsicht-Bild als Hintergrund, Layer ein und aus, Verschieben und Zoomen,
   Umschalter „Plan · 3D“. Export über „DXF …“.
10. **DXF** nach Abschnitt 4. Der Plan in der App und das DXF kommen aus derselben Zeichnung, sie können nicht
    auseinanderlaufen.
11. **Wie die anderen Schritte:** eigener Ergebnisordner `grundriss-NNN`, Versionsnummer, atomar geschrieben, Bericht
    zuletzt, „veraltet“-Erkennung, Abbrechen ohne halbe Dateien.

## 3. So findet Driftless den Grundriss

Alle Zahlen sind Startwerte für synthetische Räume. Angepasst wird nur in der Prüfung am Schluss.

### 3.1 Stockwerk

Für jede Höhe (1-cm-Stufen) wird gezählt, wie viel waagerechte Fläche dort Punkte hat (10-cm-Zellen). Boden und Decke
sind deutliche Spitzen, Wände fallen kaum ins Gewicht. Die **Laufbahn** sagt, wo man stand: Der Boden ist die tiefste
große Fläche 0,5 bis 2,2 m unter der Laufbahn (mindestens 1,5 m² und 40 % der größten Fläche in diesem Fenster), so
gilt ein Tisch nicht als Boden. Die Decke ist die größte Fläche über der Laufbahn. Ohne Decke (Dachschräge, offene
Halle) bleibt die Höhe unbekannt, und das Programm rechnet mit 2,2 m über dem Boden weiter.

Liegen die Böden verschiedener Abschnitte der Laufbahn mindestens 1,0 m auseinander, sind das **mehrere
Stockwerke**. Ein Absatz im Boden (5 bis 50 cm) ist ein Hinweis, kein Stockwerk. Eine leicht schiefe Karte wird über
die Ebene des Bodens gerade gestellt (ab 0,3°); der Bericht nennt die Schiefe.

### 3.2 Wände

Die Punkte zwischen Boden + 15 cm und Decke − 15 cm werden auf ein 2-cm-Raster von oben gelegt. Je Zelle zählt nicht
die Menge der Punkte, sondern **wie viel der Raumhöhe belegt ist**. Wände sind fast über die volle Höhe belegt, Tische
und niedrige Möbel nur unten. Starke Zellen (ab 55 % belegt) ziehen schwächere (ab 25 %) an, die an ihnen hängen, so
reißt eine Wand an einem Fenster nicht ab.

Die **Hauptrichtung** ist der Winkel, bei dem die belegten Zellen die schärfsten Streifen bilden. Entlang der
Hauptrichtung und senkrecht dazu werden die Wandlinien gesucht, danach übrig gebliebene schräge Wände. Liegt eine
Linie näher als 2° an der Hauptrichtung, wird sie genau auf sie gelegt. Die Lage kommt aus den Punkten selbst
(Median), nicht aus dem Raster, das ist genauer als die Zellgröße. Ecken werden geschlossen, wenn sich zwei Enden auf
25 cm treffen. Auf welcher Seite der Raum liegt, sagt die Laufbahn. Zwei gegenüberliegende Flächen, 5 bis 60 cm
auseinander, sind die zwei Seiten einer Wand.

### 3.3 Türen

Jede Wand wird auch von vorn angesehen: ein Bild der Wand (4-cm-Zellen, Länge × Höhe) aus den Punkten direkt auf ihr.
Ein Loch im Bild kann eine Öffnung sein, oder etwas steht davor. Entschieden wird über die **Sichtlinie**: Gibt es von
der Laufbahn aus eine freie Sicht auf die Stelle (Prüfung in einem 5-cm-Raster der Punkte), und trotzdem liegt dort
kein Punkt, dann ist sie wirklich offen. Ist die Sicht verstellt, zum Beispiel durch einen Schrank, ist es keine
Öffnung und die Wand läuft durch.

Eine **Tür** ist ein offenes Loch, das bis auf 10 cm zum Boden reicht, mindestens 1,7 m hoch und 0,55 bis 2,6 m breit
ist, **und** das man durchquert hat oder hinter dem Punkte liegen. Geht die Laufbahn durch die Wand, reicht das auch
bei teilweise verstellter Öffnung.

### 3.4 Fenster und Unklares

Ein **Fenster** ist ein offenes Loch mit Wand darunter (Brüstung, ab 20 cm über dem Boden, auf mindestens der halben
Breite), mindestens 25 × 25 cm groß. Glas liefert kaum Punkte, und die Punkte hinter dem Glas nimmt die Bereinigung
schon heraus, genau das macht das Loch sichtbar.

Auf den Layer **„Öffnung-unklar“**, mit Grund, kommt:
- bis zum Boden offen, aber weder durchquert noch etwas dahinter: Tür oder bodentiefes Fenster?
- offen, aber in Größe oder Form weder Tür noch Fenster, zum Beispiel sehr breit;
- ein Rechteck in Türgröße, das 1,5 bis 10 cm hinter der Wandfläche liegt: Tür geschlossen?

### 3.5 Rohre

Rohre werden in dieser Runde **nicht** automatisch gesucht. Ein Scan hat 1 cm Punktabstand und einige Millimeter
Rauschen, übliche Rohre sind 15 bis 110 mm dick. Zylinder zu suchen träfe bei dieser Auflösung ebenso Tischbeine,
Kabelkanäle und Türzargen, und es gibt keine gemessenen Beispiele, an denen sich das absichern ließe. Yannick will sie
nur, wenn es robust geht. Also setzt man sie von Hand (Linienzug mit Durchmesser), und Layer-Liste, Anleitung und
ein Kommentar im DXF sagen „automatisch später“.

### 3.6 Räume und Maße

Auf einem 4-cm-Raster ist frei, wo Boden gemessen wurde; Möbelflecken werden bis 20 cm geschlossen. Wände und die
Linien über Türen und unklare Öffnungen sind Grenzen. Eine Füllung von der Laufbahn aus ergibt je Raum eine Fläche (ab
1 m²). Ihr Umriss wird vereinfacht und auf die Wandlinien gelegt, die Ecken sind deren Schnittpunkte. Fehlt an mehr als
30 % des Umfangs die Wand, heißt der Raum „offen“, und ein Hinweis sagt das.

Gemaßt werden die Innenlängen je Wandstück ab 30 cm, Fläche und Höhe je Raum, Breite und Höhe je Öffnung (beim Fenster
mit Brüstung, „BRH“), bei gepaarten Wänden die Dicke. Zahlen stehen in Metern mit zwei Stellen und Komma („8,30 m“).

## 4. DXF

| Punkt | Festlegung |
|---|---|
| Version | **AutoCAD 2000 (AC1015)**, Text-DXF |
| Einheiten | Millimeter: `$INSUNITS = 4`, `$MEASUREMENT = 1` |
| Umlaute | `$DWGCODEPAGE = ANSI_1252`, Zeichen als Windows-1252; was es dort nicht gibt, als `\U+XXXX` |
| Lage | Vorgabe „an den Wänden“: Hauptrichtung = x-Achse, linke untere Ecke = (0, 0), Boden = 0. Wahlweise „wie die Punktwolke“: Lage der LAS-Datei, Höhe des Bodens |
| Bauteile | `LINE`, `LWPOLYLINE`, `CIRCLE`, `TEXT`, `BLOCK` + `INSERT` + `ATTRIB`, `XDATA` |
| Kein | `DIMENSION`, `HATCH`, `IMAGE`, `MTEXT`, Objekte in 3D |
| Eingebettet | Kommentarzeilen mit Programm, Version und Lage („Drehung, Verschiebung“) |

| Layer | Inhalt | Farbe (ACI) |
|---|---|---|
| `Wand` | Wandflächen als Linienzug, Endkappen an Öffnungen | 7 |
| `Tür` | Türöffnungen: Laibungen, Schwelle gestrichelt, Text | 30 |
| `Fenster` | Fensterlinien in der Wand, Text mit BRH | 4 |
| `Öffnung-unklar` | Lücke gestrichelt, Text mit Grund | 8 |
| `Raum` | Umriss geschlossen, Name, Fläche, Höhe | 3 |
| `Bemaßung` | Maßlinien und Maßzahlen, Wanddicken | 2 |
| `Messung` | Messungen mit dem Werkzeug „Messen“ | 6 |
| `Steckdose` | Block `Steckdose` mit Attributen | 1 |
| `Anschluss` | Block `Anschluss` mit Attributen | 200 |
| `Kabel` | Linienzug, Text | 5 |
| `Rohr` | Linienzug, Text mit Durchmesser | 34 |

Punktobjekte sind Blöcke mit den Attributen `NR`, `NAME`, `ART`, `HOEHE` (mm über dem Boden), `NOTIZ` und je freier
Eigenschaft einem weiteren. Kabel und Rohre tragen dieselben Angaben als `XDATA` unter `DRIFTLESS`. Ein in der App
ausgeblendeter Layer wird mitgeschrieben, ist im DXF aber aus.

## 5. Bedienung

1. Links „Planung“ wählen. Das Feld zeigt je Karte und Modell eine Karte mit Stand („nicht gerechnet“, „fertig“,
   „veraltet“, „rechnet“).
2. Karte wählen, „Grundriss rechnen“. Ein Auftrag mit Fortschritt und „Abbrechen“ wie überall. Bei mehreren
   Stockwerken erscheint „Mehrere Stockwerke gefunden“ mit einem Knopf je Stockwerk („Boden 0,00 m · Decke 2,60 m ·
   80 % der Laufbahn“).
3. Rechts erscheint der **Plan**. Mit gedrückter Taste verschieben, Mausrad zoomt zum Zeiger, `F` zeigt alles. Oben
   rechts schaltet „Plan · 3D“ um.
4. Im Feld: **Layer** mit Haken und Anzahl, „Draufsicht“ als eigener Haken, Hinweise des Plans, die Liste der Objekte,
   die Messungen, „DXF …“.
5. **Objekte setzen:** „3D“ wählen, unten das Werkzeug „Objekt“. Darüber die Leiste: Art, bei Anschluss die Sorte, bei
   Rohr der Durchmesser. Ein Klick setzt Steckdose oder Anschluss. Bei Kabel und Rohr setzt jeder Klick einen Punkt;
   Enter oder Doppelklick beendet, Backspace nimmt den letzten Punkt zurück, Esc verwirft. Getroffen wird wie beim
   „Messen“ die Punktwolke, wird nur die Oberfläche gezeigt, die Oberfläche.
6. In der Liste: Klick markiert im Plan, „In 3D zeigen“ führt hin, Rechtsklick „Umbenennen …“, „Notiz …“ und „Löschen“. ↶ (⌘Z,
   Strg+Z) nimmt bei gewähltem Werkzeug „Objekt“ die letzte Änderung zurück.
7. **„DXF …“** öffnet das bekannte Fenster „Exportieren“ mit der Wahl der Lage.

Die Objekte hängen an der Karte oder am Modell, nicht am Grundriss. Ein neu gerechneter Grundriss behält sie.

## 6. Nicht-Ziele (später)

- DWG (proprietär), IFC, Revit.
- Netz oder Mesh mit hineingesetzten Objekten, Rendern in Blender.
- Automatische Erkennung von Steckdosen, Kabeln, Anschlüssen und Rohren.
- Mehrere Stockwerke in einem Plan, Schnitte, Ansichten.
- Wände, Türen und Fenster in der App von Hand verändern; das geschieht im CAD-Programm. Unklares zeigt der Plan.
- Echte, mitlaufende Bemaßung (`DIMENSION`), Schraffuren.
- Das Draufsicht-Bild im DXF.
- Räume umbenennen. Freie Eigenschaften eines Objekts in der App bearbeiten (sie stehen vorerst nur in `objekte.toml`).

## 7. Was es nicht werden darf

- Ein Plan, der falsch ist und sicher aussieht: Unklares wird als unklar gezeichnet, mehrere Stockwerke werden gemeldet,
  offene Räume heißen „offen“.
- Eine Tür, wo nur ein Schrank steht.
- Ein DXF, das im CAD-Programm repariert werden muss, verschoben ist oder falsche Maßeinheiten hat.
- Ein Plan, der anders aussieht als sein DXF.
- Objekte, die beim Neurechnen, Verschieben des Projekts oder „Aufräumen“ verloren gehen.
- GPL/AGPL im Programm. Werkzeuge in der Prüfung (LibreCAD, Blender-Import) laufen nur daneben und werden nicht
  eingebunden.
- Scandaten, Bilder oder Kundennamen im Repository.

## 8. Rahmen

- **Mac und Windows**, wie bisher; CI grün. In Tests keine exakten Vergleiche von Gleitkommazahlen aus `sin`, `cos`,
  `atan2` usw., denn die Windows-Mathematik rundet anders. Rechte Winkel entstehen ohne Winkelfunktion aus (−y, x).
- **Gleiche Eingabe, gleiches Ergebnis** auf demselben System: keine Zufallszahlen, Sortierungen mit Gleichstandsregel,
  keine Reihenfolge aus einer HashMap.
- **Eigene Module, wegen des parallelen Zweigs `rauschen`:** Die Oberflächen-Rechnung (`driftless-map`,
  `driftless-steps/src/surface.rs`) bleibt unberührt. Die Erkennung wird eine neue Crate `driftless-plan`, der
  DXF-Schreiber ein neues Modul in `driftless-export`, die App bekommt eigene Module. In bestehenden Dateien nur
  Einhängepunkte.
- **Ablage:** `karte-NNN/grundriss-NNN/` bzw. `modell-NNN/grundriss-NNN/` mit `draufsicht.png` und `grundriss.json`
  (Plan und Bericht, zuletzt geschrieben). `objekte.toml` liegt in der Karte bzw. im Modell, gebaut wie `stellen.toml`:
  unter Sperre, atomar, unbekannte Schlüssel bleiben. Sie nennt keine Pfade und wandert mit dem Projekt. Das DXF entsteht
  erst beim Export, denn es hängt von den aktuellen Objekten ab.
- **Veraltet:** Die Kennung des Plans besteht aus der Kennung der bereinigten Wolke aus `map.json`, Name und Größe der
  gelesenen LAS-Datei, dem Inhalt der Laufbahn, den Einstellungen und `PLAN_VERSION`; beim Modell aus der Kennung des
  Modells und den Kennungen und Laufbahnen seiner Scans. So braucht ein Plan die Rohdaten nicht.
- **Speicher und Zeit:** Das Büro (rund 16 Mio. Punkte) soll in unter 60 s durchlaufen und höchstens das Doppelte der
  Wolke an Speicher brauchen. Das Raster wächst mit der Fläche; über 8192 Zellen je Seite wird die Zelle gröber, und der
  Bericht sagt das.
- Texte der App deutsch und schlicht in `ui/text.rs`. Im Programm heißen Höhen-Ebenen **„Stockwerk“** und Zeichnungs-
  Ebenen **„Layer“**, weil „Ebenen“ in ⚙ schon die Anzeige-Ebenen meint.
- Schwere Läufe mit echten Daten nur auf VM 101 über `driftless-vm`.

## 9. Abhängigkeiten

| Crate | Wofür | Lizenz | Geprüft |
|---|---|---|---|
| keine neue im Programm | DXF-Schreiber selbst gebaut, rund 600 Zeilen | – | – |
| `dxf` 0.6.2 | nur Test: unabhängiger Leser | MIT | `LICENSE.txt` im Paket und crates.io, Abhängigkeiten MIT/Apache/BSD |
| `png` | `draufsicht.png` schreiben, in `driftless-steps` bisher nur Test | MIT/Apache | schon im Projekt |

## 10. Von Claude entschieden (zum Gegenlesen)

| Entscheidung | Warum |
|---|---|
| DXF **AC1015 (AutoCAD 2000)** | Die älteste Fassung mit `$INSUNITS` und `LWPOLYLINE`, die alle genannten Programme lesen. R12 kennt `$INSUNITS` nicht. Neuere Fassungen bringen für einen 2D-Plan nichts und sind schwerer fehlerfrei zu schreiben. |
| DXF-Schreiber selbst | Klein, gleiche Bytes bei gleicher Eingabe, Windows-1252 unter Kontrolle, keine neue Abhängigkeit im Programm. `dxf` (MIT) prüft ihn als fremder Leser. |
| Maße als Linien + Text, kein `DIMENSION` | Blender und mancher Import ignorieren `DIMENSION`. Linien und Text kommen überall an. |
| Eingabe ist die Punktwolke, nicht die Oberfläche | Eine Oberfläche gibt es nicht immer, und so bleibt die Rechnung unabhängig vom Zweig `rauschen`. |
| Sichtlinie statt „etwas steht davor“ | Ein Schrank wirft einen schrägen Schatten. Nur die Sichtlinie trennt „offen“ von „verdeckt“ sicher. |
| Rohre von Hand | siehe 3.5. |
| Stockwerke: melden und wählen | Vorgabe „nicht raten“. Die Wahl kostet einen Knopf und erspart das Teilen des Scans. |
| Eigener Layer „Öffnung-unklar“ | Unsichere Fälle bleiben sichtbar, und im CAD-Programm ist schnell entschieden. |
| Objekte an der Karte, nicht am Grundriss | Sie bleiben beim Neurechnen erhalten, wie die Stellen. |
| Plan-Ansicht als eigene 2D-Fläche | Die 3D-Ansicht bleibt unangetastet. Eine Draufsicht-Kamera hätte die Größe der Punktflecken und damit den Renderer geändert. |
| Plan und DXF aus einer Zeichnung | Sie können nicht auseinanderlaufen, und die Symbole gibt es nur einmal. |
| Vorgabe „an den Wänden“ ausgerichtet | CAD-Arbeit braucht achsenparallele Wände. „Wie die Punktwolke“ bleibt für das Überlagern. |
| Tür ohne Aufschlag-Bogen | Die Bandseite ist nicht messbar; ein geratener Bogen wäre falsch. |
| Objekte im DXF flach bei z = 0, Höhe als Attribut | Fangen und Zeichnen im 2D-Plan funktionieren nur flach. Die echte Lage steht in `objekte.toml`. |
| Text „8,30 m“ mit Einheit | Klarer für Leser ohne Plan-Erfahrung, Millimeter sind die Zeichnungseinheit. |
| Hauptrichtung als Einheitsvektor | Rechter Winkel und Drehung in den Plan brauchen so keine Winkelfunktion, das Ergebnis hängt weniger an der Rundung von `sin` und `cos`. |

## 11. Dem Bauenden überlassen

- Genaue Form der Blöcke (schlicht, bei 1:50 lesbar, etwa 150 mm), Textgrößen um 120 mm, Strichstärken.
- Innerer Aufbau von `driftless-plan`, sofern Schnittstellen und Tests aus dem Bauplan halten.
- Schwellen im Rahmen von Abschnitt 3, wenn synthetische Tests es verlangen.
- Das Aussehen der Leiste „Objekt“ und der Plan-Ansicht im Stil von B4.

## 12. Prüfung am Schluss

Laut Yannick: schnell bauen, keine Mess- und Prüfläufe zwischendurch, ein Review und ein Prüflauf ganz am Schluss.
Zwischendurch laufen nur `cargo fmt`, `clippy` und die Unit-Tests mit synthetischen Daten.

**Vorbereitung auf VM 101** (6 Threads, `nice 10`; `DRIFTLESS_REPO=~/Developer/driftless-grundriss
DRIFTLESS_RDIR=driftless-grundriss`): Projekt mit `whole_run_real` nach `~/work-grundriss` rechnen (dunkler Raum S1–S3
= `20261022132520`, `20261022133028`, `20261022133554`, Büro `20261020173822`, Modell S1–S3), dann `driftless plan` für
jede Karte und das Modell. Für den Objekte-Test eine `objekte.toml` mit je einem Objekt jeder Art von Hand anlegen.
Danach `~/work-grundriss` löschen.

| Prüfung | Bestanden, wenn |
|---|---|
| Stockwerk S1–S3, Modell, Büro | genau eines, Raumhöhe plausibel gegen das Höhenprofil, keine falsche Meldung „mehrere Stockwerke“ |
| Maße dunkler Raum | etwa 8,3 × 7,9 m, Abweichung höchstens 10 cm; S1, S2, S3 und Modell untereinander höchstens 3 cm |
| Wände | Umriss geschlossen, rechte Winkel 90° ± 0,5°, keine doppelten Linien; im Modell keine versetzten Wände |
| Türen, Fenster, Unklares | Sichtprüfung gegen `draufsicht.png` und Fotos; jede Stelle, an der die Laufbahn durch eine Wand geht, ist eine Tür; Schränke sind keine Türen; Zahl der „unklar“ wird notiert |
| Büro | begangene Räume einzeln erkannt, Fensterbänder als Fenster, Zeit unter 60 s, Speicher höchstens doppelt so groß wie die Wolke |
| DXF allgemein | `ezdxf audit` (MIT) ohne Fehler, `$INSUNITS` = 4, alle Layer da, Umlaute richtig, Anzahl der Bauteile wie in `grundriss.json`, Längen in `ezdxf` gemessen gleich `grundriss.json` auf 1 mm |
| DXF in freiem CAD | LibreCAD (AppImage, ohne sudo) öffnet die Datei; `dxf2pdf` gibt ein PDF mit Wänden, Türen, Maßen, Umlauten |
| DXF in Blender | Import-Erweiterung auf VM 101: Linien auf den erwarteten Layern, Größe in Millimetern plausibel |
| Objekte | `INSERT` mit `ATTRIB` in `ezdxf` lesbar; `HOEHE` richtig; Kabel und Rohr mit `XDATA`; „Lage wie die Punktwolke“ deckt sich mit `color.las` auf 1 cm |
| Windows | CI grün, DXF-Tests laufen dort |

**Von Yannick von Hand**, weil auf der VM nicht prüfbar: öffnen in AutoCAD, Vectorworks, SketchUp, BricsCAD.

Dann das eine Gesamt-Review, danach die Zahlen in `docs/ergebnisse/2026-11-xx-grundriss.md` (ohne Bilder, ohne
Kundennamen). Bilder und PDF gehen an Yannick, nicht ins Repository.

## 13. Nachtrag 08.11.

- **Öffnungen nur mit Beleg in den Messdaten.** Von Yannick bestätigt: Ein Loch im Wandbild wird nur Tür oder
  Fenster, wenn die Laufbahn hindurchging, die andere Seite der Wand dort auch offen ist oder dahinter gemessen
  wurde. Eine geschlossene Tür ist für den Laser Wand, ein Fenster ohne Messung dahinter bleibt Wand.
- **„unklar“ im DXF.** Auf dem Layer „Öffnung-unklar“ steht im Plan nur der kurze Text „unklar“; der Grund steht als
  XDATA an der Linie (`GRUND=…`) und in `grundriss.json`.
- **Türen und Fenster von Hand.** Neue Objektarten „Tür“ und „Fenster“ (`art = "tuer"` / `"fenster"` in
  `objekte.toml`, zwei Punkte, mindestens 30 cm breit, in der Ebene gemessen; `hoehe_m` und `bruestung_m`
  freiwillig). Liegen beide Kanten bis 30 cm an einer Wandfläche des Plans, entlang der Wand und auf diesem Stockwerk, wird
  das Objekt auf diese Wand gelegt und wie eine erkannte Öffnung gezeichnet: Wand geschnitten (auch beide Flächen einer
  Wand mit zwei Flächen), Laibung, Beschriftung nur einmal, auf den Layern `Tür` und `Fenster`, in beiden Lagen. Es
  ersetzt erkannte Öffnungen (auch unklare), die es um mindestens 5 cm überlappt. Sonst bleibt es als Strich mit
  Beschriftung an seiner Stelle. Die Erkennung, die Räume und die Maße bleiben unberührt; der Plan wird dafür
  nicht neu gerechnet.
