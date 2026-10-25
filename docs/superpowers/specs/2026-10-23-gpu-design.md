# Baustein GPU: Rechnen auf der Grafikkarte – Design

Stand 23.10.2026 · Autor: Yannick Stumpf mit Claude · Reihenfolge laut Yannick („sehr sehr wichtig“,
„direkt nachziehen“): nach Runde B von Baustein 2, vor der Anzeige im Programm (Baustein 2, Aufgaben 11–12).

## 1. Worum es geht und warum

Driftless rechnet SLAM, Karte und Oberfläche heute nur auf der CPU. Die Grafikkarte wird nur zum Anzeigen
genutzt. Yannick hat eine RTX 5080 (VM 101, Linux; Win11-VM 100) und will, dass sie mitrechnet. Auf dem Mac
soll dasselbe über Metal laufen.

Gemessen (Baustein 2, schwarzer Raum, 2 cm): die Ansichten je Dreieck kosten etwa 80 % der Oberflächen-Zeit;
das Einfärben der Karte ist der größte Teil der Kartenzeit (Büro 1 cm: ≈ 300 s). Beides ist „viele Punkte/
Dreiecke in viele Bilder projizieren, Tiefenpuffer, Auswahl“ – typische GPU-Arbeit.

Erfolg heißt:
- Karte (`--step color`) und Oberfläche auf der RTX 5080 mindestens 5× schneller als auf der CPU derselben
  Maschine, auf dem M4 Pro mindestens 2× schneller als heute.
- Ergebnis gleichwertig: dieselben Bilder gewählt bzw. Farben innerhalb kleiner, belegter Abweichung.
- Ohne passende Grafikkarte läuft alles wie heute auf der CPU, ohne Fehler.

## 2. Ziele

1. **Ein Weg für alle Systeme:** Compute-Shader über `wgpu` (WGSL) – Vulkan auf Linux/Windows (RTX), Metal auf
   dem Mac, DX12 als Rückfall unter Windows. Kein CUDA (liefe nur auf NVIDIA, nicht auf dem Mac).
2. **Zuerst die teuren Teile:** (a) Ansichten je Dreieck der Oberfläche (Projektion, Tiefenpuffer,
   Sichtbarkeit aller Ecken, Bewertung, beste k je Dreieck); (b) Einfärben der Karte (Projektion der Punkte,
   Tiefenpuffer, beste Ansichten je Punkt); (c) TSDF-Eintrag, falls messbar lohnend. Bilddekodierung bleibt
   auf der CPU, parallel zur GPU-Arbeit (Pipeline: Bild n+1 dekodieren, während Bild n gerechnet wird).
3. **Schalter:** `--gpu auto|an|aus` an `map` und `surface` (Standard `auto`: GPU, wenn eine taugliche da
   ist, sonst CPU mit Hinweis). Im Programm ein Schalter „Grafikkarte nutzen“. Bericht (`map.json`,
   `oberflaeche.json`) nennt Gerät, Treiber und ob GPU genutzt wurde.
4. **CPU bleibt Bezug:** Die heutigen CPU-Wege bleiben vollständig und getestet; Tests vergleichen GPU gegen
   CPU auf synthetischen Szenen.
5. **VM 101:** GPU-Läufe dort gehen über die vorhandene Warteschlange (`gpu-run`/`gpu-job`), damit Ollama vorher
   aus dem Grafikspeicher entladen wird. Läuft Win11 (hält die Karte), CPU-Rückfall.

## 3. Nicht-Ziele

- SLAM/Odometrie (ICP) auf der GPU – heute schon schnell (Sekunden).
- CUDA, OptiX, Gaussian Splatting.
- Rendern/Anzeige (läuft schon über wgpu).

## 4. Was es nicht werden darf

- Andere Ergebnisse ohne Hinweis: Weicht GPU von CPU ab, muss das gemessen und in den Ergebnissen belegt sein.
- Absturz oder Hängen ohne Grafikkarte, im Headless-Betrieb (VM ohne Bildschirm) oder in der CI.
- Mehr als 4 GB Grafikspeicher für einen Raum-Scan (RTX 5080 hat 16 GB, M4 Pro teilt sich den Speicher).
- Neue Lizenzprobleme (wgpu ist MIT/Apache, schon im Projekt).

## 5. Rahmen

- Determinismus: gleiche Eingabe auf **demselben Gerät** → gleiche Dateien (ganzzahlige/feste Reduktionen
  wie heute, keine Float-Atomics mit Reihenfolge-Abhängigkeit). CPU und GPU dürfen sich minimal
  unterscheiden (Rundung); Grenzen werden in Tests festgelegt.
- CI hat keine echte GPU: GPU-Tests laufen mit wgpu-Software-Adapter (lavapipe/WARP), wenn vorhanden, sonst
  übersprungen; die CPU-Tests bleiben Pflicht.
- Rechnen auf VM 101 nur mit halber CPU-Last (Regel PVE Privat); GPU-Last ist dort erlaubt.

## 6. Entscheidungen von Claude (nicht von Yannick bestätigt)

- wgpu-Compute statt CUDA (siehe Ziel 1).
- Reihenfolge der Umstellung: Oberflächen-Ansichten → Einfärben der Karte → TSDF (nur wenn > 20 % Gewinn).
- Ein gemeinsamer GPU-Kern (`driftless-gpu`: Gerät finden, Puffer, Fisheye-Projektion und Tiefenpuffer als
  WGSL) für Karte und Oberfläche statt zweier Einzellösungen.

## 7. Dem Bauenden überlassen

- Aufteilung in Stapel (Bilder je Durchgang, Punkte je Kachel), Puffergrößen, Workgroup-Größen.
- Wie die besten k Ansichten auf der GPU gehalten werden (z. B. gepackte u64 mit atomicMax je Slot, oder
  Sortieren je Kachel).
- Messung und Vergleich (Ergebnisdokument wie bei den Meilensteinen).

## 8. Testdaten

Schwarzer Raum `jmk6-2026-10-22/20261022133554`, Büro `jmk6-2026-10-20/20261020173822`; Läufe auf VM 101
(RTX 5080, Vulkan-ICD vorhanden) und auf dem Mac (M4 Pro, Metal).
