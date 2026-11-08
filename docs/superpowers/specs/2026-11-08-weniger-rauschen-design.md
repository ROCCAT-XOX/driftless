# Weniger Rauschen – Design

Stand 08.11.2026 · Autor: Yannick Stumpf mit Claude · zweiter von drei Schritten nach Version 0.1.0
(1 Pinsel und Melden → **2 weniger Rauschen** → 3 Grundriss). Zweig `rauschen`, Basis `pinsel-melden`
(99284b9). Bauplan: `docs/superpowers/plans/2026-11-08-weniger-rauschen.md`.

## 1. Worum es geht und warum

Yannick: „Rauschen weiter senken“, allgemein und automatisch, ohne dass er Stellen anmalt. Gemeint ist,
was man auf den fertigen Oberflächen sieht: unruhige Wände, Böden und Decken (Form) und fleckige oder
streifige Bilder (Textur), zum Beispiel der dunkle Streifen an dünnen Teilen vor der Wand und Nähte.

Die Pinsel-Spec wollte Schritt 2 an markierten Stellen messen. Es gibt aber noch keine markierten
Stellen und keine Meldungen. Deshalb richtet sich diese Runde nach den Befunden der Qualitätsrunde, den
Messungen zum Pinsel und den Vergleichsbildern.

Erfolg heißt:
- Auf den Vorlagen (dunkler Raum S1–S3, Büro, Modell S1–S3) wirken Wände und Böden ruhiger, die Körnung
  auf dunklen Flächen ist deutlich schwächer, der Streifen neben dem Rohr im dunklen Raum ist weg oder
  nur noch schwach zu sehen.
- Die Oberfläche liegt nicht schlechter an den Messpunkten als heute. Raummaße und Löcher bleiben.
- Alles läuft von selbst. Jede Maßnahme lässt sich zum Vergleich einzeln abschalten.

## 2. Was auf den Bildern stört, und woher es kommt

| Nr | Was man sieht | Beleg | Ursache |
|---|---|---|---|
| B1 | Wände und Böden sind unruhig | Die Punkte sind nach dem Glätten nur 0,95 mm dick (Schritt E, S1). Das Netz liegt aber im Median 1,4–1,7 mm und bei 95 % 6,7–6,9 mm neben der Ebene der Punkte (`q_fill_diag`, A+B, 1 cm) | Die Unruhe entsteht erst zwischen Punkten und Netz (Abstandsfeld, Marching Cubes), nicht in den Punkten |
| B2 | Viele kleine Bildstücke, Nähte auf Wänden | F: Das „wellige 1-cm-Netz“ macht aus einzelnen schräg stehenden Flächen eigene Karten. Rund 50 % des restlichen `seam_step` sitzen auf Knicken über 45° | Folge von B1: Über Knicke wird die Helligkeit nicht angeglichen (`SEAM_MIN_COS`), und auf Wänden sind das meist falsche Knicke |
| B3 | Körnung und farbige Sprenkel auf dunklen Flächen | Vergleichsbilder (Oberfläche gegen Punktwolke): Die Punkte mitteln mehrere Fotos, die Fläche nimmt ein Foto je Stück. Dunkle Fotos werden im Belichtungsabgleich aufgehellt, ihr Rauschen mit | Ein einzelnes dunkles Foto liegt ungefiltert in der Textur |
| B4 | Flecken in dunklen Räumen | Die Kamera regelt selbst nach. Mit einer Lampe im Bild wird eine schwarze Wand noch dunkler aufgenommen; aufgehellt zeigt sie Körnung und Stufen | Die Bildwahl achtet auf Größe, Schärfe und Winkel, nicht darauf, wie gut die Stelle belichtet war |
| B5 | Dunkler Streifen an dünnen Teilen vor der Wand (Ruling 16), Geisterbilder | Alle 24 Fotos zeigen den Streifen. Er wandert mit dem Standort, nahe Fotos liegen 3–4 cm neben fernen (Pinsel, Abschnitt Verdeckung) | Das Rohr fehlt im Netz oder liegt dort nur in Stücken. Sein Bild landet auf der Wand dahinter, je nach Standort an anderer Stelle. Der Tiefentest kennt nur das Netz und merkt das nicht |

Was diese Runde nicht löst: Der Versatz zwischen Durchgängen (3,4 mm, Schritt C) und langsame Wellen in
der Wand sind gemessen, aber ohne Weg zur Behebung. Fotos mit echtem Inhalt an einer Naht bleiben (F).

## 3. Was vom Pinsel sich auf große Flächen übertragen lässt

| Werkzeug | Automatisch auf alles? | Warum |
|---|---|---|
| „leicht“ / „stark“ (Taubin über Kanten) | nein | Ohne Maske rundet es Kanten und kleine Formen. F hat 5 Runden über das ganze Netz gemessen: `.glb` kleiner, Wandmedian +0,24 mm. Glätten rückt die Fläche von den Punkten weg; Maßnahme M1 rückt sie heran |
| „ganz eben“ (eine Ebene je Stelle) | nur im Kleinen | Eine ganze Wand ist nicht eben: Sie weicht 3,5 mm RMS von einer Ebene ab (global RMS). Eine Ebene je Wand würde Teile um rund 1 cm verschieben. Die Idee trägt aber im Kleinen: je Ecke die Ebene ihrer Umgebung (M1) |
| „Bilder neu“ (ein Foto je Stelle, Fotokonsistenz) | nein | Die Wahl ist ein Klick mit Varianten und keine Regel für alle Flächen. Die Konsistenz hat der Oberflächen-Schritt schon. Es bleibt nur, dass „Bilder neu“ dieselbe Körnungsdämpfung und dieselbe Verdeckung bekommt wie die Oberfläche (M3, M4), sonst sähe eine neu eingespielte Stelle anders aus als ihre Umgebung |
| Schritt E (Glätten der Punkte) | nicht neu einstellen | E zeigte: Der heutige Glätter ist optimal. Die Unruhe sitzt danach (B1) |

## 4. Die vier Maßnahmen

Die erwarteten Wirkungen sind Annahmen. Gemessen wird am Schluss (Abschnitt 11).

### M1 – Flächen an die Messpunkte legen (Form)

Nach `extract` und `drop_unsupported`, vor `cleanup` und `simplify`: Jede Ecke des rohen Netzes sucht die
bereinigten Punkte in ihrer Umgebung. Liegen diese auf einer Ebene, wandert die Ecke senkrecht auf diese
Ebene.

- Umkreis `max(2,5 Voxel, 3 cm)`; zu wenige Punkte: einmal mit doppeltem Umkreis.
- Rechnung wie beim Glätten der Punkte (`smooth::project`): mindestens 10 Nachbarn, Ebenheit 5 %, innere
  Ebene zuerst, Ausreißer weg.
- Weiter als 1 Voxel von der Ebene: Die Ecke bleibt (dort stimmt etwas anderes, etwa eine Kante).
- Ecken ohne ebene Umgebung (Kanten, dünne Teile) bleiben. Wie bei den Punkten bleibt so ein schmaler
  Saum an Kanten unverändert.
- Dreht sich eine Fläche durch die Verschiebung um mehr als 60°, behalten ihre drei Ecken die alte Lage.

Warum vor `simplify`: Ebene Flächen werden dann stärker zusammengefasst, das Netz wird kleiner und die
Dreiecke bleiben gerade. Warum das ehrlich ist: Die Ecken gehen auf die Messung zu, nicht von ihr weg (das
ist der Unterschied zu Taubin, F).

Erwartung: Rauheit der Wände mindestens halb so groß, falsche Knicke weg, dadurch weniger und größere
Karten, Nähte auf Wänden werden angeglichen (`seam_step` runter), `.glb` kleiner.

### M2 – Gut belichtete Bilder bevorzugen (Bild)

In `choose_labels` zählt zusätzlich, wie hell die Fläche im Foto tatsächlich aufgenommen wurde: Jede
Ansicht bekommt ein Gewicht aus der Helligkeit ihrer Farbe im Foto (`FaceView::colour`, roh, sRGB-Stufen):
`w = clamp(L / 64, 0,25, 1)`, ausgefressen (ein Kanal ≥ 250) `w = 0,5`. Wirksame Güte ist `score · w`; sie
ersetzt `score` in der Startwahl und in den Kosten von ICM.

Die übrigen Stufen (welche Ansichten bleiben, Inseln, Zusammenlegen, Belichtungsabgleich) bleiben.

Erwartung: Dunkle Räume bekommen ihre Textur öfter aus Fotos ohne Lampe im Bild, also weniger Körnung und
weniger Flecken. Helle Räume: fast nichts ändert sich, weil dort `L ≥ 64` ist.

### M3 – Bildrauschen dämpfen (Bild)

Nach `bake`, vor `level_seams`, je Karte für sich (nie über Kartengrenzen): Ein kantenerhaltender Filter
(geführter Filter) auf der Textur.

- Rechenraum: Helligkeit und zwei Farbanteile (YCoCg auf sRGB-Stufen, nur Grundrechenarten).
- Rauschen je Karte geschätzt: `σ = 1,4826 · Median |N∗Y| / 6` mit dem 3×3-Operator nach Immerkær, über die
  Texel der Karte mit voller 3×3-Umgebung. Karten mit weniger als 256 solchen Texeln nehmen den Median
  ihres Fotos, sonst den der Seite.
- Helligkeit: Fenster 5×5, `ε = (1,5 σ)²`. Farbe: Fenster 9×9, von der Helligkeit geführt, `ε = (2 σ)²`.
- Stärke folgt dem Rauschen: unter `σ = 1` bleibt die Karte, wie sie ist; von 1 bis 3 steigt die Mischung von
  0 auf 1. Helle, saubere Fotos bleiben so praktisch unverändert.
- Der Filter arbeitet nur mit den Texeln der Dreiecke der Karte, danach wird der Rand neu gefüllt
  (`fill_rest`).
- „Bilder neu“ (`bake_region`) nutzt denselben Filter, sonst wäre die neue Stelle körniger als ihre Umgebung.

Erwartung: Körnung im Dunkeln etwa halbiert; Schrift, Schalter und Kanten bleiben scharf; `.glb` etwas
kleiner, weil JPEG Rauschen teuer ist; Angleich an Nähten genauer, weil die Proben weniger rauschen.

### M4 – Teile ohne Fläche verdecken (Bild)

Beseitigt die Ursache von B5: Was gemessen wurde, aber im Netz fehlt, verdeckt beim Tiefentest die Wand
dahinter.

- **Verdecker finden** (nach `simplify`): Punkte, die weiter als `max(1,5 Voxel, 2 cm)` von jeder Fläche
  liegen, in Zellen von 1 Voxel sammeln. Zellen mit mindestens 3 Punkten zählen, ihr Mittelpunkt ist ein
  Verdecker. Zusammenhängende Gruppen mit kürzerer längster Seite als 30 cm fallen weg (Türgriffe, Krümel:
  Diese werden nicht verdeckt, sondern bleiben wie heute im Bild). Höchstens 1,5 Millionen, gleichmäßig
  ausgedünnt.
- **Wirkung auf die Ansichten:** Je Bild ein zweiter kleiner Tiefenpuffer nur aus Verdeckern, gleiche Zellen
  wie der Puffer der Flächen. Eine Ansicht gilt als verdeckt, wenn bei einem von 7 Proben (Mitte, 3 Ecken, 3
  Kantenmitten) ein Verdecker mehr als 2 cm näher ist.
- Eine verdeckte Ansicht wird nicht gestrichen, sondern herabgestuft: Sie zählt nur 4 % ihrer Güte und trägt
  das vorhandene Merkmal für „kann den Träger zeigen“. So fällt sie weg, sobald ein anderes Foto da ist, und
  bleibt, wenn es keines gibt: keine neuen grauen Flächen, keine neue Logik im Filter. Für die Belichtung
  zählt sie auch nicht.
- Beide Wege: auf dem Prozessor in `image_views` und auf der Grafikkarte (`faces.wgsl`, eigener Puffer,
  eigener Durchlauf).
- „Bilder neu“: Die Verdecker liegen als `verdecker.bin` neben der Oberfläche, `windowed_views` nutzt sie
  genauso.

Erwartung: Dünne Teile (Rohre, Leitern, Kabel) malen sich nicht mehr auf die Wand dahinter, der Streifen ist
weg oder deutlich schwächer. Auf großen Flächen ändert sich nichts. Grenze: Nur was weiter als 2 cm vor der
Fläche liegt, zählt, und nur, wenn es Punkte hat.

## 5. Schalter, Version, Kennzahlen

- **Schalter:** Neu `SurfaceConfig.noise` mit vier Feldern, alle Vorgabe an. Über die Kommandozeile `driftless
  surface --ohne anlegen,belichtung,entrauschen,verdeckung` oder `--ohne alles`. Die Felder stehen im Bericht
  unter `settings.noise` (fehlt der Eintrag, zählt die Vorgabe) und in der Eingabe-Kennung.
- **Mit `--ohne alles`** kommen dieselben Netze und Seiten heraus wie vor diesem Zweig. Die festen
  Prüfsummen-Tests bleiben so erhalten und sichern den alten Weg ab.
- **Version:** `SURFACE_VERSION` 13 → 14, alte Oberflächen gelten als veraltet. `EDIT_VERSION` 2 → 3, alte
  Zwischenspeicher der Nacharbeit gelten nicht mehr.
- **Neue Kennzahlen** in `oberflaeche.json` (nur Rust, in jedem Lauf):
  - `accuracy.walls_plane` und `accuracy_dense.walls_plane`: Abstand der Wand-Ecken zur Ebene der Punkte im
    Umkreis von 3 cm (Median, Mittel, 95 %; die ehrliche Messung für M1, wie `q_fill_diag`).
  - `roughness`: Rauheit der Wand-Ecken, Abstand zum Mittel der Nachbarn entlang der Normalen (wie Rauheit im
    Pinsel-Bericht).
  - `snap`: bewegte Ecken, mittlere Verschiebung, Ecken ohne Ebene, zu weit weg, zurückgenommen.
  - `texture.noise_before`, `texture.noise_after`: Rauschen (nach Texeln gewichteter Median der Karten).
  - `occluders`: Zahl der Verdecker.

## 6. Nicht-Ziele

- Neue Bedienelemente in der App. Der Vergleich läuft über die Kommandozeile und die Bilder am Schluss.
- Glätten der Punkte, SLAM, Karte, Zusammenführen ändern.
- Mittel aus mehreren Fotos je Texel (kostet Schärfe, braucht viel Speicher, Fotos liegen bis 8 cm auseinander).
- Dünne Teile ins Netz holen. Das wäre eine eigene Aufgabe; hier verhindern wir nur, dass sie die Wand verfärben.
- Bewegte Dinge aus den Bildern entfernen.

## 7. Was es nicht werden darf

- Eine ruhigere Fläche, die weiter von der Messung liegt als heute. Geprüft mit `walls_plane` und gegen die
  ungeglättete Wolke (`accuracy_dense`).
- Verwaschene Bilder: Schrift, Schalter, Türkanten müssen scharf bleiben.
- Mehr graue Flächen oder mehr Löcher.
- Sichtbare Kanten zwischen neu eingespielten Stellen und ihrer Umgebung.
- Fremder Code unter GPL/AGPL/Inria-Lizenz; Scan-Daten, Bilder oder Kundennamen im Repo.

## 8. Rahmen

- Rust-Workspace wie bisher, macOS und Windows. Tests nur mit synthetischen Daten, keine exakten
  Gleitkommavergleiche über transzendente Funktionen: Windows rundet anders. M1 bis M3 nutzen nur Grundrechenarten
  und Wurzeln.
- Prozessor- und Grafikkartenweg: M1, M2, M3 laufen in beiden gleich auf dem Prozessor, M4 ist auf beiden gebaut.
- Speicher Büro bei 1 cm: heute 8,16 GB (B1), Ziel höchstens rund 8 GB, also nicht mehr als heute. Dafür ein
  gemeinsamer k-d-Baum der Punkte statt mehrerer nacheinander; M3 arbeitet in Kacheln von höchstens 512² Texeln.
- Zeit: höchstens moderat länger, Ziel Oberfläche höchstens +15 %.
- App-Änderungen klein: nur `SurfaceSettings::from_report` in `crates/driftless-app/src/project.rs` liest
  `settings.noise`. Der parallele Zweig `grundriss` wird dadurch kaum berührt.

## 9. Von Claude entschieden (zum Gegenlesen)

| Entscheidung | Begründung |
|---|---|
| M1 als Lage an den Punkten statt Taubin | F hat Taubin gemessen und verworfen (Wandmedian +0,24 mm); M1 nähert die Fläche der Messung an, Kanten bleiben |
| M1 vor `simplify` | Ebene Flächen lassen sich dann weiter zusammenfassen, kleinerer `.glb`, weniger falsche Knicke |
| Keine Ebene je ganzer Wand | Wände sind 3,5 mm RMS uneben; das wäre geschönt (Anti-Ziel der Qualitätsrunde) |
| Rauschfilter je Karte, nicht je Foto | Büro: rund 350 Mio. Texel statt 46 Mrd. Bildpunkte (3.831 Fotos à 12 MP); Karten überlagern sich nicht |
| Geführter Filter statt Mittel mehrerer Fotos | Fotos liegen bis 8 cm auseinander, Mitteln verwischt |
| Keine neuen Merkmale im Ansichtsfeld: verdeckt = „Träger“-Merkmal, 4 % Güte | Kein neues Bit, derselbe Filter, und ohne Alternative bleibt das Foto (keine grauen Flächen) |
| Kleine Verdecker (unter 30 cm) zählen nicht | Griffe und Kleinteile sollen im Bild bleiben, auch wenn sie leicht versetzt sind |
| `--ohne alles` ergibt die alten Bytes | Sichert den Vergleich und behält die festen Tests |
| Kein Schalter in der App | Klein bleiben (Zweig `grundriss` läuft parallel) und der Vergleich findet am Schluss statt |
| „Bilder neu“ bekommt M3 und M4, nicht M2 | Sie sollen zur Umgebung passen; seine Bildwahl hat eine eigene Reihenfolge mit Varianten |
| `--ohne` als Liste, `alles` als Kurzform | Eine Option statt vier, wie `--ohne-glaetten` im Zusammenführen |
| Vorgaben ohne Messläufe festgelegt | Vorgabe Yannick: bauen, am Schluss prüfen; die Werte sind vorsichtig gewählt |

## 10. Dem Bauenden überlassen

- Innere Aufteilung der neuen Module (`surface/snap.rs`, `surface/denoise.rs`, `surface/occlude.rs`).
- Wie die Karten beim Filtern parallel laufen.
- Feinheiten der Grenzen, wenn ein synthetischer Test das zeigt; Änderungen an den Werten stehen in der
  Beschreibung der Änderung.

## 11. Prüfung am Schluss

Yannicks Vorgabe: ein Review und ein Prüflauf ganz am Schluss, nicht dazwischen. Nach dem Bau läuft einmal
`tools/qualitaet/run_rauschen.sh` auf VM 101 (Linux, RTX 5080, 6 Threads, `nice 10`, abgekoppelt starten).
Scans:

| Kurz | Ordner |
|---|---|
| S1 | `20261022133554` |
| S2 | `20261022132520` |
| S3 | `20261022133028` |
| Büro | `20261020173822` |
| Modell S1–S3 | `driftless merge` aus S1, S2, S3 |

**Läufe** (Laufbahn und Karte je Scan einmal, Karte 0,5 cm):

| Variante | Einstellung | Scans |
|---|---|---|
| vorher | `--ohne alles` | alle fünf, 1 cm und 2 cm |
| nachher | Vorgabe | alle fünf, 1 cm und 2 cm |
| einzeln | `--ohne` je eine Maßnahme | S1, 1 cm |
| CPU | Vorgabe, `--gpu aus` | S1, 2 cm |
| Probe | Stand 99284b9 gegen `--ohne alles` | S1, 2 cm: `oberflaeche.glb` Byte für Byte gleich |

**Messwerte, vorher → nachher** (`oberflaeche.json`, `tools/qualitaet/q_table.py`, `images.log`):

| Messgröße | Ziel |
|---|---|
| Rauheit Wände (`roughness`) Median / 95 % | mindestens halbiert |
| Abstand Wand zur Ebene der Punkte (`walls_plane`) 95 % | mindestens −30 % |
| Abstand gegen die ungeglättete Wolke (`accuracy_dense.walls_plane`, Median) | nicht schlechter als +0,3 mm |
| Wände Median (`accuracy.walls`, wie bisher) | nicht schlechter als +0,2 mm |
| Raummaße Spanne S1–S3 (Z5) | bleibt ≤ 2,5 mm |
| Löcher von oben, Rundblick | nicht schlechter als +0,5 Prozentpunkte |
| Flächen ohne Bild (Fläche %) | nicht schlechter als +0,1 Prozentpunkte |
| Karten | nicht mehr als vorher |
| `seam_step` je m Naht | −10 % oder besser |
| Bildrauschen (`noise_before` / `noise_after`) | dunkler Raum mindestens −30 %, Büro nicht schlechter |
| `.glb` | nicht größer |
| Zeit Oberfläche | höchstens +15 % |
| Spitzen-Speicher Büro 1 cm | höchstens rund 8 GB |
| Streifen im dunklen Raum (Verfahren aus Aufgabe 14, von Hand, gleiche Wand) | „gegen die Wand daneben“ von 15,4 auf höchstens 8 sRGB-Stufen, Bilder vorher/nachher |

Dazu `touchup_real` auf VM 101 (Modell S1–S3 und Büro, wie im Pinsel-Bericht): Zeit je Aktion bleibt unter
5 s, Rand und Verdeckung nicht schlechter, Griffe im Büro nicht zerschnitten.

**Regel:** Eine Maßnahme bleibt Vorgabe, wenn sie auf keinem der fünf Sätze Wände Median, Raummaße oder Löcher
verschlechtert (Toleranzen wie oben). Sonst steht sie auf aus und das steht im Bericht mit dem Messwert.

**Vergleichsbilder** (nur lokal auf dem Mac, nicht im Repo; Vorher und Nachher nebeneinander):

- `q_images.sh` mit `QA_B_SURF` für Vorher/Nachher: von oben, 8 Rundblicke, die festen Ansichten des dunklen
  Raums (Tür, Leuchtstäbe, Leiter, Notausgang-Tür).
- Reliefbilder (neu, `render_relief`: Fläche ohne Foto, mit Licht von schräg), weil die App ohne Licht zeichnet und
  Form nur so sichtbar wird: dieselben Ansichten.
- Ausschnitte 1:1 aus den Texturen dunkler Wände im Büro und im dunklen Raum (Körnung).
- Die Wand mit dem Streifen aus `touchup_real` (Modell S1–S3), Stelle und Griffe im Büro.

Ergebnis in `docs/ergebnisse/2026-11-08-weniger-rauschen.md` (Zahlen, keine Bilder, keine Kundennamen), danach ein
Gesamt-Review über den Zweig.
