# Driftless – Anleitung

Für Version 0.1.0. Driftless macht aus den Aufnahmen des Handscanners 3DMakerPro JMK6 eine Punktwolke,
eine fotoähnliche Oberfläche und Dateien für andere Programme. Es läuft auf dem Mac und unter Windows
und rechnet alles auf dem eigenen Rechner.

Wörter in „…“ stehen genau so im Programm.

1. [Installieren](#1-installieren)
2. [Daten vom Scanner holen](#2-daten-vom-scanner-holen)
3. [Aufnahme-Tipps](#3-aufnahme-tipps)
4. [Erste Schritte in der App](#4-erste-schritte-in-der-app)
5. [Welche Einstellung?](#5-welche-einstellung)
6. [Mehrere Scans zusammenführen](#6-mehrere-scans-zusammenführen)
7. [Projekte verwalten](#7-projekte-verwalten)
8. [Korrigieren](#8-korrigieren)
9. [Ergebnis weitergeben](#9-ergebnis-weitergeben)
10. [Bekannte Grenzen](#10-bekannte-grenzen)
11. [Fehler melden](#11-fehler-melden)

**Wörter, die im Programm vorkommen**

| Wort | Bedeutung |
|---|---|
| Laufbahn | Der Weg, den das Gerät während der Aufnahme gegangen ist. |
| SLAM | Die Berechnung der Laufbahn aus Laser und Bewegungssensor. Ein Ergebnis heißt „SLAM 001“, auch „Lauf“. |
| Karte | Die Punktwolke eines Scans, zum Beispiel „Karte 001“. |
| Oberfläche | Eine geschlossene Fläche mit den Fotos der Kameras darauf, zum Beispiel „Oberfläche 001“. |
| Modell | Mehrere Scans, zu einem zusammengeführt, zum Beispiel „Modell 001“. |
| Teilkarte | Ein kurzes Stück der Aufnahme. Die Laufbahn besteht aus solchen Stücken. |
| Schleife | Zwei Stellen der Laufbahn, die derselbe Ort sind. Driftless zieht sie zusammen. |

## 1. Installieren

Auf der Seite [github.com/ROCCAT-XOX/driftless/releases](https://github.com/ROCCAT-XOX/driftless/releases)
die neueste Version öffnen und die Datei für das eigene System laden.

| System | Datei |
|---|---|
| Mac | `Driftless-0.1.0-macos-arm64.dmg` |
| Windows | `Driftless-0.1.0-setup.exe` |

### Mac

1. Das `.dmg` öffnen, **Driftless** auf **Programme** ziehen und das Image auswerfen. Driftless nicht
   aus dem Image heraus starten.
2. Driftless in **Programme** doppelklicken. Beim ersten Start blockiert macOS das Programm, weil es
   nicht bei Apple beglaubigt ist: **Fertig** klicken.
3. **Systemeinstellungen → Datenschutz & Sicherheit**: Unten bei **Sicherheit** auf **Dennoch öffnen**
   klicken, mit Passwort oder Touch ID bestätigen und im nächsten Fenster noch einmal **Dennoch
   öffnen**.

Statt Schritt 2 und 3 geht auch dieser Befehl im Terminal (er gibt auch die Kommandozeile frei):

```sh
xattr -dr com.apple.quarantine /Applications/Driftless.app
```

Beides ist nach jeder neuen Version noch einmal nötig.

### Windows

1. Das Setup laden. Warnt der Browser, die Datei werde selten heruntergeladen: im Menü **…** auf
   **Beibehalten** klicken.
2. Das Setup starten. Meldet Windows **Der Computer wurde durch Windows geschützt**: **Weitere
   Informationen → Trotzdem ausführen**.
3. **Installation nur für Sie (empfohlen)** wählen. Driftless steht danach im Startmenü.

### Voraussetzungen

| | Mac | Windows |
|---|---|---|
| System | Apple-Chip (M1 oder neuer), macOS 14 oder neuer; Intel-Macs gehen nicht | Windows 10 oder 11, 64 Bit, Grafikkarte für DirectX 12 oder Vulkan |
| Arbeitsspeicher | 16 GB empfohlen | 16 GB empfohlen |
| Freier Platz | 1 bis 6 GB je Scan für die Ergebnisse, je nach Genauigkeit | ebenso |

| Einstellung | Arbeitsspeicher bei großen Aufnahmen |
|---|---|
| Karte mit „Maximal“ | über 4 GB |
| Oberfläche mit „1 cm“ | bis 9 GB |

### Kommandozeile

Für die meisten Aufträge ist sie nicht nötig. Sie ist in beiden Paketen dabei, aber nicht im Suchpfad:

| System | Pfad |
|---|---|
| Mac | `/Applications/Driftless.app/Contents/MacOS/driftless` |
| Windows | `%LOCALAPPDATA%\Programs\Driftless\driftless.exe` |

### Entfernen

Mac: Driftless aus **Programme** in den Papierkorb ziehen. Windows: über **Einstellungen → Apps**
deinstallieren. Projekte und Scans bleiben dabei, wo sie sind.

## 2. Daten vom Scanner holen

Der Scanner legt jede Aufnahme auf seiner MicroSD-Karte in einen eigenen Ordner. Der Ordner heißt nach
Datum und Uhrzeit der Aufnahme:

| Ordnername | heißt |
|---|---|
| `<JJJJMMTThhmmss>` | Jahr, Monat, Tag, Stunde, Minute, Sekunde |
| `20261022133554` | 22.10.2026, 13:35:54 Uhr |

So kommen die Daten auf den Rechner:

1. Die MicroSD-Karte über einen Kartenleser am Rechner öffnen.
2. Den ganzen Ordner der Aufnahme auf ein lokales Laufwerk kopieren, zum Beispiel in einen Ordner
   `Scans` neben den Projekten. Nicht in iCloud Drive, OneDrive oder einen anderen Ordner, der mit
   einer Cloud abgeglichen wird.
3. Warten, bis das Kopieren fertig ist. Eine abgeschnittene Kopie meldet Driftless später als
   abgeschnitten, mit dem Rat, die Datei neu vom Gerät zu kopieren.

Das liegt im Ordner und das braucht Driftless davon:

| Datei oder Ordner | Inhalt | Braucht Driftless |
|---|---|---|
| `LIDAR_<JJJJMMTThhmmss>.bag` | Laserpunkte und Bewegungssensor | immer |
| `IMAGE_<JJJJMMTThhmmss>.bag` | Bilder der beiden Kameras | für Farbe und Fotos auf der Oberfläche |
| `calibration/calib.json` | wie Kameras und Laser zueinander sitzen | für Farbe und Fotos auf der Oberfläche |
| `project_parameters.json`, `camera/`, `thumbnail/` | Angaben und Vorschaubilder des Scanners | nein, sie stören aber nicht |

Fehlen die Kamerabilder oder `calib.json`, rechnet Driftless die Karte ohne Farbe und die Oberfläche
grau, mit einer Warnung.

Was in einer Aufnahme steckt, zeigt „Prüfen“: in „Daten“ den Scan wählen, unten „Erweitert“ öffnen,
dann „Prüfen“. Es nennt Gerät, Dauer und Probleme.

## 3. Aufnahme-Tipps

| Tipp | Warum |
|---|---|
| Am Anfang das Gerät eine Sekunde ruhig halten. | In der ersten Sekunde misst Driftless, wo unten ist und wie der Drehsensor abweicht. Bewegt sich das Gerät schon, kann die Laufbahn leicht kippen; das Protokoll meldet dann „Gerät stand in der ersten Sekunde nicht still“. |
| Langsam drehen, nicht schneller als etwa 60° pro Sekunde. | Bilder aus schnellen Drehungen nutzt Driftless nicht für die Farbe. Schnelle Drehungen geben der Laufbahn außerdem wenig Halt (gelb oder rot in der Güte, siehe Abschnitt 8). |
| Am Ende zum Start zurückkehren, Runden gehen. | Kommt der Scan an eine schon gesehene Stelle zurück, zieht Driftless die Laufbahn dort zusammen. Im Büro-Scan lagen Start und Ende ohne Schleifen 2,16 m auseinander, mit Schleifen 0,13 m: so weit, wie das Gerät wirklich vom Start entfernt endete. |
| Stellen mit wenigen Punkten näher und langsamer scannen. | Wo der Scanner nur wenige Punkte misst (weit weg, dunkle Flächen, der Boden großer Räume), bleibt die Oberfläche offen. Schließen lässt sie sich nur durch dichteres Scannen, nicht durch eine feinere Einstellung. |
| Scans, die zusammengeführt werden sollen, mit genug gemeinsamem Bereich aufnehmen: etwa 1 m oder mehr. | Dann findet Driftless die Lage selbst. Bei weniger hilft „Von Hand ausrichten“ (Abschnitt 6). |
| Niemanden im Raum stehen lassen. | Wer länger steht oder sitzt, kann in den Fotos der Oberfläche zurückbleiben. Bewegtes nimmt Driftless aus der Punktwolke. |
| Auf gleichmäßiges Licht achten. | Bilder, die viel heller oder dunkler sind als ihre Nachbarn (Fenster, Gegenlicht, Lampen), gleicht Driftless nur bis zu einer Grenze an; dort können Farben fleckig wirken. |
| Glas beachten. | Punkte, die durch bodentiefe Fenster, Glastüren oder Glaswände gemessen wurden, bleiben in der Karte. |

## 4. Erste Schritte in der App

Das Fenster: Links eine Leiste mit ☰ (das Projektmenü) oben, darunter die Schritte „Daten“, „Bilder“ und
„Planung“, unten ⚙ („Einstellungen“). Daneben das Feld des gewählten Schritts, rechts die 3D-Ansicht,
unten die Statusleiste mit „Protokoll“ und „Abbrechen“.

1. **Projekt anlegen.** ☰ → „Neues Projekt …“ (oder der Knopf im leeren Feld). Einen leeren Ordner
   wählen oder anlegen; der Dialog beginnt im Ordner „Driftless Projekte“. Das Projekt heißt wie sein
   Ordner.
2. **Scan hinzufügen.** „Scan hinzufügen …“ und den Ordner der Aufnahme wählen (den mit der
   `LIDAR_….bag`). Für weitere Scans wiederholen; „Scan hinzufügen …“ steht auch unter den Kacheln der
   Scans. Jeder Scan bekommt eine Kachel mit seinem Stand, zuerst „neu“.
3. **Rechnen.** Bei „Genauigkeit“ „Genau“ lassen (siehe Abschnitt 5) und „Alle rechnen“ drücken.
   Driftless rechnet für jeden neuen Scan zuerst die Laufbahn, dann die Punktwolke. Der Fortschritt
   steht im Feld und rechts unten; „Abbrechen“ hält an, ohne halbe Dateien zu hinterlassen. Danach
   steht auf der Kachel „fertig“. Ist ein Scan gewählt, heißt der Knopf „Rechnen“ und rechnet nur
   diesen.
4. **Ansehen und prüfen.** Die Kachel eines Scans anklicken: Darunter erscheinen sein Lauf („SLAM 001“)
   und seine Karte („Karte 001“). Ein Klick auf die Karte zeigt die Punktwolke. Mit der linken Maustaste
   drehen, mit dem Rad zoomen, die Taste `F` zeigt alles. Die Laufbahn ist nach Güte gefärbt, die
   Legende steht unten links (Abschnitt 8). Gelbe und rote Stellen genauer ansehen: doppelte Wände,
   schiefe Räume?
5. **Oberfläche rechnen.** Links „Bilder“ wählen, bei „Feinheit“ „2 cm“ lassen und „Alle Oberflächen
   rechnen“ drücken. Danach unter der Karte „Oberfläche 001“ anklicken. Oben rechts in der Ansicht
   zwischen „Fläche“, „Punkte“ und „beides“ wechseln (Taste `O`). Bleibt die Oberfläche stellenweise
   offen, steht unter ihr klein ein „Hinweis“ („Stellenweise offen“); angeklickt erklärt „Diese
   Oberfläche“ den Grund.
6. **Messen, wenn nötig.** Unten in der Ansicht „Messen“ wählen und zwei Punkte anklicken: Driftless
   zeigt den Abstand in Metern. Die Messungen stehen in „Planung“; sie werden noch nicht gespeichert.
   Gegen ein Maßband oder einen Laser-Entfernungsmesser ist Driftless noch nicht geprüft (Abschnitt 10).
7. **Exportieren.** Rechtsklick auf eine Karte, eine Oberfläche oder ein Modell → „Exportieren …“. Im
   Fenster das „Format“ wählen, dann „Ziel wählen …“. Welches Format wofür: Abschnitt 9.

Unten in der Ansicht liegen die Werkzeuge „Drehen“, „Verschieben“, „Punkt wählen“, „Messen“ und
„Schnitt“ (blendet zum Beispiel die Decke aus); „Alles“ zeigt wieder alles. In ⚙ „Einstellungen“ stehen
„Farbart“ („Farbe“, „Intensität“, „Güte“, „je Scan“), „Grafikkarte nutzen“ („Auto“, „An“, „Aus“) und die
Ebenen, zum Beispiel „Laufbahn“ und „Schleifen“.

## 5. Welche Einstellung?

Zwei Schalter entscheiden über Zeit, Arbeitsspeicher und Platz:
- die „Genauigkeit“ der Karte in „Daten“
- die „Feinheit“ der Oberfläche in „Bilder“

Die Vorgaben „Genau“ und „2 cm“ passen für die meisten Räume.

Die Zahlen sind auf einem Testrechner gemessen: Linux mit der Grafikkarte NVIDIA RTX 5080, Version 0.1.0
([Ergebnisse](ergebnisse/2026-10-28-version-0-1.md)). Auf einem Büro-PC oder Laptop dauert es länger.
Auf dem Mac sind sie noch nicht nachgemessen.

**Karte** („Genauigkeit“ in „Daten“):

| „Genauigkeit“ | Punktabstand | gut für | Kleiner Raum (6 min Aufnahme): Zeit · Speicher · Platz | Büro (10 min Aufnahme): Zeit · Speicher · Platz |
|---|---|---|---|---|
| „Schnell“ | 2 cm | schnell ansehen; aus der Nähe löchrig | 1,0 min · 0,7 GB · 0,2 GB | 1,9 min · 1,4 GB · 0,7 GB |
| „Genau“ (Vorgabe) | 1 cm | die meisten Räume | 1,4 min · 1,4 GB · 0,6 GB | 2,7 min · 2,7 GB · 1,5 GB |
| „Maximal“ | 0,5 cm | genauere Wände und Kanten, feine Teile | 2,6 min · 2,3 GB · 1,2 GB | 3,8 min · 3,8 GB · 2,2 GB |

**Oberfläche** („Feinheit“ in „Bilder“), aus der Karte „Genau“:

| „Feinheit“ | gut für | Kleiner Raum: Zeit · Speicher · Datei | Büro: Zeit · Speicher · Datei |
|---|---|---|---|
| „2 cm“ (Vorgabe) | geschlossene Flächen, kleine Dateien | 1,9 min · 1,1 GB · 52 MB | 7,0 min · 3,6 GB · 293 MB |
| „1,5 cm“ | dazwischen | 2,7 min · 1,5 GB · 76 MB | 12,6 min · 5,6 GB · 431 MB |
| „1 cm“ | genauere Kanten; in großen und dunklen Räumen mehr offene Stellen | 4,3 min · 2,6 GB · 130 MB | 20,2 min · 7,7 GB · 631 MB |

Die Laufbahn vor der Karte braucht nur Sekunden: 9 s im kleinen Raum, 34 s im Büro. Den freien Platz für
die Ergebnisse nennt [Voraussetzungen](#voraussetzungen).

## 6. Mehrere Scans zusammenführen

Mehrere Scans desselben Orts werden zu einem Modell, zum Beispiel ein großer Raum in Teilen. Jeder Scan
braucht dafür eine fertige Karte.

1. In „Daten“ „Zusammenführen …“ drücken. Der Knopf erscheint, sobald zwei Scans eine fertige Karte
   haben.
2. Im Feld „Scans zusammenführen“ die Scans anhaken. Der erste angehakte Scan ist der „Bezug“: Die
   anderen werden an ihm ausgerichtet. Hat ein Scan mehrere Karten, die gewünschte wählen.
3. „Zusammenführen“ drücken. Das neue Modell steht unter „Modelle“, zum Beispiel „Modell 001“.
4. Unter dem Modell steht bei jedem Scan, wie sicher er liegt:

| Urteil | heißt | was tun |
|---|---|---|
| „sicher“ | Die Lage passt. | nichts |
| „unsicher“ | Eine Lage ist gefunden, aber nicht sicher genug, zum Beispiel zu wenig gemeinsamer Bereich, versetzte Wände, Wände nicht messbar oder eine zweite Lage fast so gut. Der Grund steht, wenn die Maus darauf zeigt. | „Von Hand ausrichten“, oder „Auch unsichere übernehmen“, wenn das Ergebnis selbst geprüft ist |
| „gescheitert“ | Keine brauchbare Lage. | „Von Hand ausrichten“ |

Unsichere und gescheiterte Scans sind „nicht im Modell“. „Auch unsichere übernehmen“ nimmt die
unsicheren mit auf, gescheiterte nie. Unter jedem Scan stehen „Überlappung“ (welcher Teil des Scans im
Modell wiederkehrt) und „Restversatz“ (wie weit seine Punkte im Mittel neben denen der anderen liegen).

**Punktpaare von Hand.** „Von Hand ausrichten“ steht unter einem unsicheren oder gescheiterten Scan.
Danach je Paar zuerst eine Stelle im Modell anklicken, dann dieselbe Stelle im Scan rechts daneben.
Mindestens drei Paare setzen, weit auseinander und nicht auf einer Linie. „Letzten Punkt zurück“
nimmt einen Klick zurück. „Übernehmen und neu zusammenführen“ speichert die Paare und legt ein neues
Modell an. Unter „Punktpaare“ lassen sich Paare ab- und wieder einschalten, gelöscht wird keines; ein
Paar, das nicht passt, heißt „nicht nutzbar“.

**Oberfläche des Modells.** In „Bilder“ steht das Modell unter den Karten. Anklicken und „Oberfläche
rechnen“ drücken: Die Oberfläche entsteht aus den Bildern aller Scans. In ⚙ „Einstellungen“ färbt
„Farbart“ „je Scan“ jeden Scan anders; so sieht man, wo welcher liegt.

Wird die Karte eines Scans neu gerechnet, ist das Modell „veraltet – Karte eines Scans neu gerechnet“:
in „Daten“ neu zusammenführen. Ein Modell aus einer anderen Programmversion heißt „veraltet – mit anderer
Programmversion zusammengeführt“; auch das in „Daten“ neu zusammenführen.

Drei Scans eines Raums, gemessen auf dem Testrechner mit RTX 5080
([Baustein 1](ergebnisse/2026-10-27-baustein-1.md)):

| Schritt | Zeit | Arbeitsspeicher | Datei |
|---|---|---|---|
| Zusammenführen | 37 s | 2,0 GB | – |
| Oberfläche des Modells, „2 cm“ | 3,6 min | 2,0 GB | 50 MB |
| Oberfläche des Modells, „1 cm“ | 5,6 min | 3,2 GB | 104 MB |

## 7. Projekte verwalten

**Wo Projekte liegen.** Die Dialoge beginnen im Ordner „Driftless Projekte“ im Benutzerordner (Mac:
`~/Driftless Projekte`, Windows: `%USERPROFILE%\Driftless Projekte`). Driftless legt ihn beim ersten
Mal an und merkt sich danach den zuletzt benutzten Ordner.

**Keine Projekte in Cloud-Ordnern.** Projekte und Scans nicht in iCloud Drive (auch nicht in
**Schreibtisch** und **Dokumente**, wenn sie mit iCloud abgeglichen werden), OneDrive oder einem anderen
Cloud-Ordner ablegen. Die Ergebnisse sind mehrere GB groß; sie werden hochgeladen, bei knappem Platz vom
Rechner genommen und laden dann langsam wieder herunter. Driftless warnt: „Projekt liegt in …“.

**Umbenennen, verschieben, kopieren.** Den Projektordner im Finder oder Explorer umbenennen,
verschieben oder kopieren, während Driftless ihn nicht geöffnet hat. Driftless findet Läufe, Karten und
Modelle danach wieder; eine Kopie liest nur ihre eigenen Ordner. Liegen die Scans außerhalb des
Projekts und sind nicht mitgewandert, zeigt die Kachel des Scans „fehlt“: dann „Ordner neu zuordnen …“
(siehe unten). So lässt sich ein Projekt auch an Kollegen weitergeben.

**Scans** (Rechtsklick auf die Kachel eines Scans):

| Eintrag | Was er tut |
|---|---|
| „Umbenennen …“ | Ändert nur den angezeigten Namen; Ordner und Ergebnisse bleiben. |
| „Ordner neu zuordnen …“ | Nur für einen Scan, dessen Ordner fehlt (verschoben, Laufwerk nicht angesteckt). Driftless nimmt nur denselben Scan: dieselben Laserdateien, aufgenommen zur selben Zeit. Modelle aus diesem Scan nehmen den neuen Ordner mit: die Fotos ihrer Oberfläche und ob sie „veraltet“ sind. |
| „Aus Projekt entfernen“ | Entfernt nur den Eintrag. Die Rohdaten und die Ergebnisse unter `scans/<id>` bleiben; wird der Scan wieder hinzugefügt, sind sie wieder da. Ein anderer Scan-Ordner mit demselben Namen bekommt eine eigene Kennung und übernimmt nichts davon. |

**Löschen.** Rechtsklick auf einen Lauf, eine Karte, eine Oberfläche oder ein Modell → „Löschen …“. Die
Frage nennt die Größe, was darin liegt und welche Modelle davon abhängen. „In den Papierkorb“ legt den
Ordner in den Papierkorb des Systems; von dort lässt er sich zurückholen. Ungefragt endgültig löscht
Driftless nie. Unter Windows haben Netzlaufwerke und USB-Sticks keinen Papierkorb: Dort lehnt Driftless
ab. Ist ein Ordner zu groß für den Papierkorb von Windows, fragt Windows, ob er endgültig gelöscht
werden soll: dann „Nein“ wählen, es wird nichts gelöscht. Das gilt auch für „Aufräumen“. Während ein
Schritt daran rechnet, auch in einem anderen Fenster, ist „Löschen …“ gesperrt. Zeigt die Ansicht den
Ordner gerade, erst „Aus der Ansicht nehmen“. Einen Lauf, der mit der Kommandozeile gerechnet und mit
„Lauf hinzufügen …“ geholt wurde, nimmt nur „Aus Projekt entfernen“ heraus; sein Ordner gehört dem
Nutzer.

**Größe und freier Platz.** Unter dem Projektnamen stehen die Größe des Projekts und der freie Platz
auf dem Laufwerk; jeder Lauf, jede Karte, Oberfläche und jedes Modell zeigt seine Größe.

| Wann | Was Driftless tut |
|---|---|
| unter 10 GB frei | Warnung unter dem Projektnamen und im Protokoll |
| ein Schritt braucht voraussichtlich mehr, als frei ist | Warnung beim Start des Schritts |

**Nach einem Absturz oder Stromausfall.** Was ein abgebrochener Schritt halb geschrieben hat, steht
unter dem Projektnamen als „unvollständig“, mit Größe. Dazu gehört eine Oberfläche ohne ihren Bericht
`oberflaeche.json`. „Aufräumen“ legt es in den Papierkorb. Ordner, an denen gerade ein Schritt schreibt,
auch in einem anderen Fenster, bleiben in Ruhe. Fertiges nimmt „Aufräumen“ nie mit: Fehlt einem Lauf
oder einer Karte der Bericht, enthält der Ordner aber fertige Karten, Oberflächen oder einen Verlauf,
bleibt er im Baum und heißt „unvollständig – neu rechnen“. Wenn die Maus darauf zeigt, steht, was zu tun
ist: einen Lauf mit „Neu rechnen (aus Teilkarten)“ oder einem Stand im Verlauf wieder vollständig
machen; statt einer Karte rechnet „Rechnen“ eine neue.

**Läufe der Kommandozeile.** ☰ → „Lauf hinzufügen …“ holt einen Lauf, der mit der Kommandozeile
gerechnet wurde, zu einem Scan. Dafür zuerst in „Daten“ den Scan wählen.

**Zwei Fenster.** Zwei Fenster, oder ein Fenster und die Kommandozeile, können am selben Projekt
arbeiten; keines verliert die Änderung des anderen.

Driftless merkt sich je Projekt die zuletzt gewählte „Genauigkeit“, „Feinheit“ und „ohne Fotos“.

## 8. Korrigieren

**Was die Farben der Laufbahn heißen.** Die Laufbahn ist nach Güte gefärbt; die Legende steht unten
links in der Ansicht. Die Güte sagt, wie fest die Umgebung jede Teilkarte gehalten hat, in ihrem
schwächsten Moment.

| Farbe | Legende | heißt |
|---|---|---|
| grün | „gut verortet“ | genug Wände und Kanten im Blick |
| gelb | „wenig Halt – prüfen“ | wenig Wände oder Kanten im Blick, etwa in einem leeren Flur, in einer Tür oder bei einer schnellen Drehung; die Karte an dieser Stelle prüfen |
| rot | „kaum Halt – Lage unsicher“ | dort kann die Laufbahn verrutscht sein |

Mit „Farbart“ „Güte“ in ⚙ „Einstellungen“ sind auch die Punkte so gefärbt.

**Woran man einen Fehler erkennt.** Doppelte oder versetzte Wände, ein Raum, der schief oder
verbogen aussieht. Oft liegt er an einer gelben oder roten Stelle.

**Schleife von Hand eintragen.** Zeigen zwei Stellen der Laufbahn denselben Ort, liegen aber
auseinander:

1. In „Daten“ den Lauf wählen („SLAM 001“ unter der Kachel des Scans) und unten „Erweitert“ öffnen. Dort
   steht „Korrekturen“.
2. „Stellen wählen“ drücken, dann in der Ansicht auf die Laufbahn klicken: Der erste Klick setzt „Stelle
   A“, der zweite „Stelle B“. Die Zeiten lassen sich auch eintippen (Sekunden ab Scan-Beginn). Eine
   „Notiz“ ist freiwillig.
3. „Schleife eintragen“, dann „Jetzt neu rechnen“. Das dauert Sekunden. Wird es abgebrochen oder schlägt
   es fehl (zum Beispiel bei vollem Datenträger), bleibt der Lauf, wie er war.
4. Danach ist die Karte „veraltet“: „Rechnen“ oder „Alle rechnen“ rechnet eine neue, zum Beispiel
   „Karte 002“. Die alte bleibt, bis sie gelöscht wird.

Jeder Eintrag in der Liste darunter lässt sich mit seinem Haken ab- und wieder einschalten, danach
„Jetzt neu rechnen“; gelöscht wird keiner.

**Automatische Schleife abschalten.** Unter „Korrekturen“ steht „Automatische Schleifen“: jede Schleife,
die Driftless selbst gefunden hat, mit „Teilkarten“, „Beginn (s)“ und „Überlappung“. Verbindet eine davon
zwei verschiedene Stellen (zum Beispiel zwei gleich aussehende Flure), den Haken bei „verwenden“
abnehmen. Der Lauf wird sofort neu gerechnet, in wenigen Sekunden. Haken wieder setzen, und sie zählt
wieder. Die Ebene „Schleifen“ in ⚙ „Einstellungen“ zeigt die Schleifen in der Ansicht.

**Vorher und nachher.** „Vergleichen“ legt zwei Laufbahnen eines Scans übereinander. „Verlauf“ listet
jeden Stand eines Laufs; ein gewählter Stand lässt sich mit „Diesen Stand wiederherstellen“
zurückholen. Dabei entsteht ein neuer Stand, die alten bleiben.

Ein schlechtes Stück der Laufbahn wegschneiden geht noch nicht (Abschnitt 10).

## 9. Ergebnis weitergeben

Rechtsklick auf das Ergebnis → „Exportieren …“, „Format“ wählen, „Ziel wählen …“. Eine vorhandene Datei
wird nur nach „Ersetzen“ überschrieben.

| Für | Format | Was exportieren |
|---|---|---|
| CloudCompare | E57 oder LAZ | eine Karte oder ein Modell (Punktwolke) |
| Autodesk ReCap | E57 | eine Karte oder ein Modell |
| Blender | glb | eine Oberfläche |
| Excel | CSV | einen Lauf („SLAM 001“): die Laufbahn |
| zum Verschicken | LAZ | die kleinste Datei für eine Punktwolke |

Keines dieser Programme ist bisher von Hand mit Dateien von Driftless geprüft. Geprüft sind alle
Formate mit Lese-Bibliotheken, die keinen Code mit Driftless teilen.

- Ein E57 eines Modells hält jeden Scan als eigenen Eintrag; in LAS und LAZ trägt jeder Punkt die
  Nummer seines Scans.
- Maße in Metern. Die Höhe zeigt nach oben (Z), nur bei glb nach Y, wie glb es verlangt.
- Die CSV ist für deutsches Excel: Semikolon zwischen den Spalten, Komma vor den Nachkommastellen;
  `zeit_s` sind die Sekunden ab der ersten Lage.
- OBJ besteht aus mehreren Dateien (`.obj`, `.mtl` und die Seitenbilder): immer alle zusammen
  weitergeben.

Dateigrößen, gemessen in [Baustein 3](ergebnisse/2026-10-27-baustein-3.md):

| Format | Modell aus 3 Scans eines Raums | Büro, „Maximal“ |
|---|---|---|
| Punkte | 5,4 Mio. | 22,9 Mio. |
| LAZ | 58 MB | 237 MB |
| E57 | 157 MB | 668 MB |
| LAS | 195 MB | 826 MB |
| PLY, PCD | 119 MB | 459 MB |

| Format | Oberfläche des Modells bei „1 cm“ |
|---|---|
| glb | 104 MB |
| OBJ mit Seitenbildern | 267 MB |
| PLY-Netz | 60 MB |

## 10. Bekannte Grenzen

Dieselbe Liste steht im [CHANGELOG](../CHANGELOG.md).

| Grenze | Was das heißt |
|---|---|
| Maße nicht geprüft | Gegen einen Laser-Entfernungsmesser wurde noch nie verglichen. Drei Scans desselben Raums ergeben dieselben Raummaße bis auf 2,1 mm, ob sie stimmen, ist aber offen. Im Büro-Scan stehen zwei gegenüberliegende Wände 1,2° schief zueinander; warum, ist ungeklärt. Wichtige Maße nachmessen. |
| Verschiedene Räume zusammenführen ist unerprobt | Zusammengeführt wurde mit echten Scans nur derselbe Raum dreimal. Räume, die nur über eine Tür oder einen Flur verbunden sind, sind auf echten Daten nicht erprobt. |
| Personen | Wer durch den Raum geht oder darin steht, kann in den Fotos der Oberfläche zu sehen sein. |
| Glas und Spiegel | Durch Fenster gemessene Punkte entfernt Driftless nur bei Fensterbändern mit Wand darüber und darunter. Was durch bodentiefe Fenster, Glastüren oder Glaswände gemessen wurde, bleibt stehen. Spiegel werden nicht erkannt. |
| Kein Wegschneiden | Ein schlechtes Stück der Laufbahn lässt sich nicht verwerfen. Es gibt nur Schleifen von Hand und das Abschalten automatischer Schleifen. |
| Nicht signiert | Die Programme sind nicht signiert und nicht bei Apple beglaubigt. Der erste Start braucht deshalb einen Schritt mehr (siehe Installieren). |
| Windows nur automatisch geprüft | Bauen, Tests und Installieren laufen unter Windows in der CI. Von Hand auf einem echten Windows-PC ist Driftless noch nicht geprüft. |
| SharePoint-Bibliotheken | Mit OneDrive abgeglichene SharePoint-Bibliotheken (unter `%USERPROFILE%\<Firmenname>`) erkennt Driftless nicht als Cloud-Ordner und warnt dort nicht; das tut es nur bei OneDrive selbst. Projekte nicht dort ablegen. |
| Mac ohne echte Scans | Mit dieser Version wurden echte Scans noch nicht auf einem Mac gerechnet, nur auf einem Linux-Rechner mit RTX 5080. |
| 1 cm braucht viel Speicher | Eine Oberfläche mit 1 cm braucht bei großen Aufnahmen bis 9 GB Arbeitsspeicher (Büro: 8,2 GB, 21 Minuten). |
| Verfehlte Qualitätsziele | Bei 1 cm bleiben in großen und in dunklen Räumen Stellen der Oberfläche offen: von oben gesehen im Büro 31 % (Ziel höchstens 30 %), in einem dunklen Raum 9,7 % (Ziel höchstens 6 %). 2 cm schließt mehr. Die Übergänge zwischen zwei Fotos (Nähte) sind deutlicher zu sehen als geplant. Die Oberflächen-Datei eines Raums ist bei 1 cm rund 100 MB groß statt höchstens 60 MB. Dieselbe Wand, mehrmals abgelaufen, liegt vor dem Glätten einige Millimeter auseinander (Ziel höchstens 5 mm); die Ursache ist offen, das Glätten legt sie wieder auf eine Fläche. |

## 11. Fehler melden

1. ☰ → „Protokoll-Ordner öffnen“: Dort liegt `driftless.log`, das Protokoll des letzten Starts. Die
   Starts davor stehen in `driftless.1.log` bis `driftless.4.log`.
2. ☰ → „Über Driftless …“: Version und Commit abschreiben.
3. Auf GitHub ein Issue mit der Vorlage **Fehler melden** anlegen:
   [github.com/ROCCAT-XOX/driftless/issues/new?template=fehler.md](https://github.com/ROCCAT-XOX/driftless/issues/new?template=fehler.md).
   Beschreiben, was gemacht wurde, was passiert ist und was erwartet war; das Betriebssystem nennen und
   `driftless.log` anhängen.

| Ordner des Protokolls | |
|---|---|
| Mac | `~/Library/Logs/Driftless/` |
| Windows | `%LOCALAPPDATA%\Driftless\Logs\` |

**Nie Kundendaten in ein Issue.** Das Projekt ist öffentlich, jeder kann die Issues lesen. Keine Scans,
Punktwolken, Oberflächen, Fotos oder Bildschirmfotos von Kunden anhängen und keine Kundennamen nennen.
Das Protokoll enthält Ordnernamen, zum Beispiel den des Projekts: vor dem Anhängen durchsehen und
Kundennamen ersetzen, etwa durch *Kunde A*. Werden Scandaten zur Fehlersuche gebraucht, gehen sie auf
anderem Weg, nie über das Issue.
