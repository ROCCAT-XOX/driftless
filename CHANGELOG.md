# Changelog

Was sich in jeder Version von Driftless geändert hat. Die Form folgt
[Keep a Changelog](https://keepachangelog.com/de/1.1.0/), die Nummern folgen
[Semantic Versioning](https://semver.org/lang/de/).

## [0.1.0] – 2026-10-28

Die erste Version für echte Aufträge. Wie man sie installiert und benutzt, steht in der
[Anleitung](https://github.com/ROCCAT-XOX/driftless/blob/main/docs/anleitung.md).

### Neu

#### Scans lesen

- Liest die Aufnahmen des 3DMakerPro JMK6 so, wie sie auf der MicroSD-Karte liegen, oder aus einem
  RayStudio-Projektordner: `LIDAR_*.bag` (Laser und Bewegungssensor), `IMAGE_*.bag` (die beiden
  Kameras) und `calibration/calib.json`.
- „Prüfen“ (`driftless inspect`) nennt Gerät, Dauer, Datenströme und Probleme einer Aufnahme.
- Eine abgeschnittene Datei (Aufnahme unterbrochen oder Kopie unvollständig) wird bis zur Bruchstelle
  gelesen, mit einer Warnung, was zu tun ist.

#### Laufbahn und Korrekturen

- Berechnet die Laufbahn des Geräts aus Laser und Bewegungssensor. Kommt der Scan an eine schon gesehene
  Stelle zurück, zieht Driftless die Laufbahn dort zusammen (Schleifenschluss). Im Büro-Scan schrumpft
  so der Abstand zwischen Start und Ende von 2,16 m auf die echten 0,13 m.
- Die Laufbahn ist nach Güte gefärbt, mit Legende: „gut verortet“, „wenig Halt – prüfen“, „kaum Halt –
  Lage unsicher“.
- Schleifen von Hand: „Stellen wählen“, zwei Klicks auf die Laufbahn, „Schleife eintragen“, „Jetzt neu
  rechnen“; das dauert Sekunden (`driftless correct`, `driftless slam --reuse`).
- Jede automatische Schleife lässt sich einzeln abschalten („verwenden“), etwa eine, die zwei
  verschiedene Stellen verbindet.
- Vorher und nachher: „Vergleichen“, „Verlauf“ mit allen Ständen eines Laufs, „Diesen Stand
  wiederherstellen“ (`driftless compare`).

#### Karte und Farbe

- Dichte Punktwolke in drei Genauigkeiten: Schnell, Genau und Maximal (Punktabstand 2, 1 und 0,5 cm).
- Bereinigt: Bewegtes („Geister“), Ausreißer und durch Fenster gemessene Punkte werden entfernt, Wände
  geglättet.
- Farbe aus beiden Kameras mit Prüfung, was verdeckt ist; Belichtung und Weißabgleich werden angeglichen,
  unscharfe Bilder ausgelassen.
- Einfärben auf der Grafikkarte (Metal, Vulkan, DirectX 12), ohne taugliche Grafikkarte auf dem
  Prozessor; in ⚙ „Grafikkarte nutzen“.
- Eine Karte mit denselben Eingaben und Einstellungen wird nicht noch einmal gerechnet („Karte ist
  aktuell“). Jede andere Karte rechnet das Programm in einen neuen Ordner („Karte 002“); die alte bleibt,
  bis sie gelöscht wird.
- „Abbrechen“ und Strg+C halten auch das Glätten der Wände sofort an.

#### Fotoähnliche Oberfläche

- Eine geschlossene Fläche mit den Kamerabildern als Textur, in den Feinheiten 1, 1,5 und 2 cm. Löcher in
  Wänden und Böden bis 2 m Umfang werden eben geschlossen.
- Auf der Grafikkarte; große Räume in mehreren Durchläufen.
- `oberflaeche.json`, die Kommandozeile und das Desktop-Programm nennen Hinweise, zum Beispiel wenn ein
  großer Raum stellenweise offen bleibt. Im Programm steht der Hinweis klein unter der Oberfläche.

#### Scans zusammenführen

- Mehrere Scans desselben Orts werden zu einem Modell („Zusammenführen …“, `driftless merge`). Jeder
  Scan bekommt ein Urteil: „sicher“, „unsicher“ oder „gescheitert“.
- „Von Hand ausrichten“ mit Punktpaaren, auch bei wenig gemeinsamem Bereich (Tür, kurzer Flur).
- Die Farben der Scans werden angeglichen und ihre Wände gemeinsam geglättet. Die Oberfläche eines
  Modells entsteht aus den Bildern aller seiner Scans (`driftless surface --model`).
- Findet die automatische Ausrichtung keine Wand, an der sie den Versatz messen kann, ist der Scan
  „unsicher“ und der Grund heißt „Wände nicht messbar“. Mit Punktpaaren zählen die Paare, der Grund nennt
  es trotzdem.
- Ein Modell aus einer anderen Programmversion gilt als „veraltet“: in „Daten“ neu zusammenführen.
  `driftless surface --model` lehnt es ab.
- „Abbrechen“ und Strg+C halten auch das gemeinsame Glätten sofort an.

#### Export und Dateien öffnen

- Punktwolken als LAS, LAZ, E57, PLY und PCD, Oberflächen als glb, OBJ und PLY-Netz, die Laufbahn als
  TUM und CSV. Die CSV ist für deutsches Excel: Semikolon, Dezimalkomma, Spalte `zeit_s` (Sekunden ab
  der ersten Lage).
- Ein E57 eines Modells hält jeden Scan als eigenen Eintrag.
- Öffnet LAS, LAZ, E57, PLY, PCD, glb und OBJ anderer Programme, auch ohne Projekt („Datei öffnen …“
  oder die Datei ins Fenster ziehen).

#### Desktop-Programm

- Projekte mit den Schritten „Daten“, „Bilder“ und „Planung“; eine 3D-Ansicht, auf dem Mac flüssig bis
  etwa 50 Mio. Punkte; die Werkzeuge „Drehen“, „Verschieben“, „Punkt wählen“, „Messen“ und „Schnitt“.
- Projekte verwalten: Scans „Umbenennen …“, „Ordner neu zuordnen …“ und „Aus Projekt entfernen“; Läufe,
  Karten, Oberflächen und Modelle mit „Löschen …“ in den Papierkorb, nie ungefragt endgültig: Ist ein
  Ordner unter Windows zu groß für den Papierkorb, fragt Windows, ob er endgültig gelöscht werden soll –
  dann „Nein“ wählen. Was gerade rechnet, auch in einem anderen Fenster, lässt „Löschen …“ in Ruhe.
- Die Größe jedes Ergebnisses und der freie Platz; eine Warnung unter 10 GB frei.
- Eine Warnung bei Projekten in iCloud Drive, OneDrive und anderen Cloud-Ordnern; die Dialoge beginnen im
  Ordner „Driftless Projekte“.
- Reste abgebrochener Schritte heißen „unvollständig“, auch eine Oberfläche, die nach dem glb, aber vor
  ihrem Bericht abgebrochen wurde; „Aufräumen“ legt sie in den Papierkorb. Fertige Karten, Oberflächen
  und den Verlauf nimmt es nie mit: Ein Lauf oder eine Karte, deren Bericht fehlt, die aber Fertiges
  enthält, bleibt im Baum als „unvollständig – neu rechnen“.
- Schlägt „Jetzt neu rechnen“ fehl (zum Beispiel bei vollem Datenträger) oder wird es abgebrochen, bleibt
  der Lauf, wie er war; ebenso `driftless slam --reuse`.
- Ein Projektordner lässt sich umbenennen, verschieben und kopieren; Pfade werden auch relativ
  gespeichert.
- Zwei Fenster, oder Fenster und Kommandozeile, an einem Projekt verlieren keine Änderung des anderen.
- Genauigkeit, Feinheit und „ohne Fotos“ bleiben je Projekt gespeichert.
- Meldungen auf Deutsch, der Grund zuerst.

#### Kommandozeile

- `driftless` mit unter anderem `inspect`, `slam`, `correct`, `map`, `surface`, `merge`, `export`,
  `compare` und `calib-check`.
- Strg+C bricht `slam`, `map`, `surface`, `merge` und `export` sauber ab (Rückgabewert 130). Nichts halb
  Geschriebenes gilt als fertig; Reste räumt der nächste Lauf weg.
- `slam` ersetzt einen vorhandenen Lauf nur mit `--ueberschreiben` und nie den eines anderen Scans.
- Jedes Ergebnis (`slam.json`, `map.json`, `oberflaeche.json`, `modell.json`, `calib_check.json`) nennt
  unter `programm` Version und Commit des Programms.

#### Installer

- Mac: `.dmg` für Macs mit Apple-Chip ab macOS 14. Windows: Einrichtungsprogramm ohne
  Administratorrechte, mit Startmenü-Eintrag und Deinstallation. Die Kommandozeile ist in beiden dabei.
- Unter Windows ist die C-Laufzeit eingebaut; ein Visual-C++-Paket ist nicht nötig.
- „Über Driftless …“ nennt Version und Commit (wie `--version`) und öffnet die „Anleitung“ und die
  „Lizenzen Dritter …“. Die Lizenzen liegen beiden Installern bei.

#### Protokolldatei

- Jeder Start schreibt `driftless.log`: Version, System, Grafikkarte, alle Meldungen und bei einem Absturz
  dessen Ort. Mac: `~/Library/Logs/Driftless/`, Windows: `%LOCALAPPDATA%\Driftless\Logs\`. Die letzten
  fünf Starts bleiben.
- ☰ → „Protokoll-Ordner öffnen“ zeigt den Ordner.

### Bekannte Grenzen

- **Maße nicht geprüft:** Gegen einen Laser-Entfernungsmesser wurde noch nie verglichen. Drei Scans
  desselben Raums ergeben dieselben Raummaße bis auf 2,1 mm, ob sie stimmen, ist aber offen. Im Büro-Scan
  stehen zwei gegenüberliegende Wände 1,2° schief zueinander; warum, ist ungeklärt. Wichtige Maße
  nachmessen.
- **Verschiedene Räume zusammenführen ist unerprobt:** Zusammengeführt wurde mit echten Scans nur
  derselbe Raum dreimal. Räume, die nur über eine Tür oder einen Flur verbunden sind, sind auf echten
  Daten nicht erprobt.
- **Personen:** Wer durch den Raum geht oder darin steht, kann in den Fotos der Oberfläche zu sehen sein.
- **Glas und Spiegel:** Durch Fenster gemessene Punkte entfernt Driftless nur bei Fensterbändern mit Wand
  darüber und darunter. Was durch bodentiefe Fenster, Glastüren oder Glaswände gemessen wurde, bleibt
  stehen. Spiegel werden nicht erkannt.
- **Kein Wegschneiden:** Ein schlechtes Stück der Laufbahn lässt sich nicht verwerfen. Es gibt nur
  Schleifen von Hand und das Abschalten automatischer Schleifen.
- **Nicht signiert:** Die Programme sind nicht signiert und nicht bei Apple beglaubigt. Der erste Start
  braucht deshalb einen Schritt mehr (siehe Installieren).
- **Windows nur automatisch geprüft:** Bauen, Tests und Installieren laufen unter Windows in der CI. Von
  Hand auf einem echten Windows-PC ist Driftless noch nicht geprüft.
- **SharePoint-Bibliotheken:** Mit OneDrive abgeglichene SharePoint-Bibliotheken (unter
  `%USERPROFILE%\<Firmenname>`) erkennt Driftless nicht als Cloud-Ordner und warnt dort nicht; das tut
  es nur bei OneDrive selbst. Projekte nicht dort ablegen.
- **1 cm braucht viel Speicher:** Eine Oberfläche mit 1 cm braucht bei großen Aufnahmen bis 9 GB
  Arbeitsspeicher (Büro: 7,7 GB, 20 Minuten).
- **Verfehlte Qualitätsziele:**
  - Bei 1 cm bleiben in großen und in dunklen Räumen Stellen der Oberfläche offen: von oben gesehen im
    Büro 31 % (Ziel höchstens 30 %), in einem dunklen Raum 9,7 % (Ziel höchstens 6 %). 2 cm schließt
    mehr.
  - Die Übergänge zwischen zwei Fotos (Nähte) sind deutlicher zu sehen als geplant.
  - Die Oberflächen-Datei eines Raums ist bei 1 cm rund 100 MB groß statt höchstens 60 MB.
  - Dieselbe Wand, mehrmals abgelaufen, liegt vor dem Glätten einige Millimeter auseinander (Ziel
    höchstens 5 mm). Die Ursache ist offen; das Glätten legt sie wieder auf eine Fläche.

### Hinweis

- Ergebnisse aus Entwicklungsständen vor 0.1.0 gelten einmal als veraltet. Karten und Oberflächen rechnen
  „Alle rechnen“ und „Alle Oberflächen rechnen“ neu, in neue Ordner; die alten bleiben. Modelle in
  „Daten“ neu zusammenführen.
