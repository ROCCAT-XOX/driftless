# Ergebnisse Grundriss und DXF – Version 0.2.0

Stand: 2026-11-08. Gerechnet mit Stand `e94adb2` auf dem Mac (M4 Pro) mit der Kommandozeile
(`driftless plan <karte> -o <ziel> --dxf <datei>`), aus den fertigen Karten des Abnahme-Projekts. Der Code der
Erkennung (`crates/driftless-plan`) ist bis `v0.2.0` (`31251e6`) unverändert. Danach kamen nur die Türen und Fenster
von Hand in die Zeichnung des Plans dazu (`plan_drawing.rs`).

Kurz: Der Plan entsteht in wenigen Sekunden und das DXF ist formal sauber. Die Maße sind aber **nicht bestätigt**,
und der Prüflauf aus Abschnitt 12 der Spec ist nur zum Teil gelaufen (Tabelle unten).

## Ergebnis je Karte

| Karte | Punkte | Räume | Fläche (m²) | Raum 1 außen | Raumhöhe | Türen | Fenster | unklar | Zeit | Speicher |
|---|---|---|---|---|---|---|---|---|---|---|
| Dunkler Raum S1 | 5,23 Mio | 1 | 59,7 | 7,75 × 7,72 m | 5,76 m | 1 | 1 | 0 | 0,7 s | 0,61 GB |
| Dunkler Raum S2 | 5,17 Mio | 1 | 60,0 | 7,72 × 7,80 m | 5,76 m | 1 | 0 | 0 | 0,6 s | 0,64 GB |
| Dunkler Raum S3 | 6,34 Mio | 1 | 60,0 | 7,72 × 7,80 m | 5,76 m | 1 | 0 | 0 | 0,9 s | 0,75 GB |
| Modell S1–S3 | 5,45 Mio | 1 | 59,7 | 7,72 × 7,75 m | 5,75 m | 1 | 0 | 0 | 1,0 s | 0,68 GB |
| Büro | 14,96 Mio | 6 | 125,6 · 72,6 · 54,6 · 19,9 · 9,3 · 2,4 | 14,76 × 11,56 m | 2,77 m | 7 | 22 | 11 | 3,0 s | 1,35 GB |

- „Raum 1 außen“ ist das umschließende Rechteck des Umrisses, mit Nischen. Es ist kein sauberes Wandmaß.
- Die Tür des dunklen Raums ist in allen vier Läufen gleich erkannt: 1,46 bis 1,47 m breit, 2,46 m hoch.
- Das Fenster in S1 (1,45 m breit, 0,40 m hoch, Brüstung 4,75 m) gibt es nur dort. Im Modell liegt dieselbe Stelle mit
  14 % Punkten dahinter knapp unter der Schwelle von 20 %.
- Die zweite Tür des dunklen Raums, ein geschlossener Notausgang, fehlt im Plan. Für den Laser ist sie Wand. Sie wird
  mit dem Werkzeug „Objekt“ von Hand gesetzt.

## Stand der Prüfung aus Spec 12

| Prüfung | Soll | Gemessen | Stand |
|---|---|---|---|
| Stockwerk | genau eines, Höhe plausibel | je Karte eines, keine Meldung „mehrere Stockwerke“; nicht gegen das Höhenprofil verglichen | erfüllt |
| **Maße dunkler Raum** | etwa 8,3 × 7,9 m, höchstens 10 cm daneben; S1–S3 und Modell höchstens 3 cm auseinander | 7,72 × 7,75 bis 7,80 m; 5 cm auseinander | **nicht erfüllt, ungeklärt** |
| Wände | geschlossen, 90° ± 0,5°, keine doppelten Linien | alle 10 Umrisse geschlossen; schräg sind 0 von 6 Kanten (S1, Modell), 1 von 8 (S2), 2 von 9 (S3), 10 von 90 (Büro, bis 26,6°, längste 2,16 m); doppelte Linien nur von Auge geprüft | teilweise |
| Türen, Fenster, Unklares | Sichtprüfung gegen Draufsicht und Fotos | von Auge gegen die Draufsicht, nicht gegen Fotos oder Aufmaß | teilweise |
| Büro | Räume einzeln, Fensterbänder als Fenster, unter 60 s, Speicher höchstens doppelt so groß wie die Wolke | 6 Räume; 3,0 s; 1,35 GB bei 0,54 GB Wolkendatei (2,5-fach); von einer Reihe aus drei Fenstern ist nur das mittlere erkannt | Zeit erfüllt, Speicher und Fenster nicht |
| DXF allgemein | Audit ohne Fehler, Einheiten, Layer, Anzahl und Längen wie `grundriss.json` | `ezdxf audit`: 0 Fehler auf allen fünf Dateien; AC1015, `$INSUNITS` 4, alle Layer, Umlaute in den Layernamen richtig; Längen nicht verglichen | teilweise |
| DXF in LibreCAD, `dxf2pdf` | öffnet, PDF richtig | nicht gelaufen | offen |
| DXF in Blender | Layer und Größe plausibel | nicht gelaufen | offen |
| Objekte | `INSERT`, `XDATA`, Lage auf 1 cm | nur synthetische Tests, nicht auf echten Karten und nicht von Hand durchgeklickt | offen |
| Windows | CI grün, DXF-Tests laufen | CI auf `31251e6` grün (`windows-latest`, `macos-15`) | erfüllt |

Von Hand noch offen, weil automatisch nicht prüfbar: Öffnen in AutoCAD, Vectorworks, SketchUp und BricsCAD.

## Maße: Was offen ist

Die Spec nennt für den dunklen Raum etwa 8,3 × 7,9 m. Der Plan kommt auf 7,72 × 7,75 bis 7,80 m, auf der langen Seite
also rund 58 cm weniger. Ob der Wert der Spec oder der Plan stimmt, ist nicht geklärt. Es gibt noch keine Vergleichsmessung
mit dem Lasermessgerät. Bis dahin sind die Maße als unbestätigt zu behandeln.

## Bewusste Änderung gegenüber der Spec

Auf den echten Karten erkannte die erste Fassung schwarze Wände als Fenster, und im Büro fand sie keinen Boden. Daher gilt
jetzt (Spec, Nachtrag 13):

- **Tür oder Fenster nur mit Beleg:** Die Laufbahn ging hindurch, die andere Seite der Wand ist dort auch offen, oder es
  sind mindestens 16 Linien und 20 % Punkte dahinter gemessen, in einem Bereich, der nicht begangen wurde. Ohne Beleg
  bleibt nur eine klare Türform als „unklar“, sonst kein Eintrag.
- **Im Plan stehen nur Wände, die einen begangenen Raum begrenzen.** Hohe Möbel und Fassaden fallen weg.
- **Türen und Fenster von Hand** gehen vor erkannten Öffnungen am selben Ort.

## Bekannte Schwächen

- Die Schwelle von 20 % ist an denselben Karten eingestellt, an denen sie geprüft ist, und trennt nicht sauber. Ein echtes
  Fenster ohne Messung dahinter fehlt im Plan, die Wand läuft durch.
- Die 22 Fenster im Büro sind nicht einzeln belegt. Die meisten liegen auf sichtbaren Glasflächen an Außenwänden, einige an
  Innenwänden sind ungeklärt.
- Raum 1 im Büro hat eine Kerbe von etwa 2,2 × 1,6 m, wo kein Boden gemessen ist. Seine Fläche ist entsprechend kleiner.
- Wände nicht begangener Räume fehlen, auch wenn sie gemessen sind.
- Im Büro fehlen 5 von 87 Maßzahlen und 1 von 22 Fensterbeschriftungen, weil dort kein Platz für den Text ist.
- Ersetzt eine Tür von Hand eine unklare Öffnung, nennt der Hinweis die Öffnung weiter als unklar, bis neu gerechnet wird.
- Reicht eine Öffnung von Hand über zwei fluchtende, getrennte Wandstücke, wird nur das nächste geschnitten.

## Bilder und Pläne

Lokal auf dem Mac unter `~/Downloads/Driftless-Raumplan/`, nicht im Repository: je Karte DXF, Plan als PNG und
Überlagerung auf der Draufsicht.
