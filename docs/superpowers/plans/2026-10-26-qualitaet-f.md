# Qualitätsrunde F – Inseln übernehmen das Bild der Umgebung Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Kleine Texturkarten (Inseln), die kein Bild ihrer Nachbarn in ihren Ansichten haben, übernehmen das Bild der Umgebung, wenn dessen Kamera sie geometrisch sieht. Ergebnis: deutlich weniger Karten und Nähte.

**Architecture:** Neue Funktion `absorb_islands` in `atlas.rs` läuft in `texture_mesh` nach `choose_labels` und vor `bake`. Sie prüft je Insel die Nachbarbilder nach geteilten Kanten und nimmt das erste, dessen Kamera alle Inselflächen projiziert (Ecken im Bild innerhalb `rim_theta`, Schwerpunkt innerhalb `max_dist`, Summennormale von Insel und Kantenring zur Kamera). Es gibt keinen Tiefentest: Eine Insel mitten in einer Karte ist fast immer Rauschen des Netzes, keine Verdeckung.

**Tech Stack:** Rust, rayon, nalgebra (crate `driftless-map`, `driftless-steps`).

**Spec:** `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md`, Abschnitt F, erster Punkt: „Nachbardreiecke mit derselben besten Ansicht zu größeren Karten zusammenlegen“.

**Diagnose (Zweig `diagnose/qualitaet-f`, nur lokal):** Der dunkle Raum 133554 hat bei 1 cm nach dem Zusammenlegen 192.456 Karten. 110.586 davon bestehen aus nur einer Fläche, und Karten mit 1–3 Flächen tragen 75 % der Summe von `seam_step`. Der Spike mit `ISLAND_FACES` 8 ergab:

| | heute | Spike |
|---|---|---|
| Karten | 221.097 | 79.750 |
| Nahtproben | 3,9 Mio. | 2,0 Mio. |
| `seam_jump` nachher | 17,4 | 14,4 |
| `.glb` | 111 MB | 96 MB |

`seam_step` im Mittel blieb bei 20,6. Der Rest sitzt auf Knicken über 45° (50 %) und auf Inhalt; die Bildwahl löst das nicht. Mit 64 statt 8 waren es 85.587 Karten, und `seam_step` stieg auf 24,8.

Netz glätten (Taubin) wurde mitgeprüft und **nicht übernommen**: Bei 5 Runden lag `.glb` bei 63 MB, aber der Wandmedian stieg um 0,24 mm, über die Toleranz für Z2 von 0,2 mm.

## Global Constraints

- Kein Schritt wird übernommen, der einen der vier Scans in Z2, Z3 oder Z5 verschlechtert (Spec). Die Geometrie ändert sich hier nicht, Z2/Z3/Z5 müssen also identisch bleiben.
- Neue Versionsnummern machen alte Oberflächen veraltet; das ist gewollt: `SURFACE_VERSION` 11 → 12.
- Gleiche Eingabe ergibt byte-gleiche Seiten (wie `bake`/`level_seams`); parallel berechnen, in fester Reihenfolge anwenden.
- Die Seitenfarben-Regel aus Baustein GPU (je Dreieck vor dem Packen) gilt weiter.
- Schwere Läufe nur auf VM 101 (`driftless-vm`), halbe Last. Push und Merge nur nach Yannicks Okay.
- Commits mit den Trailern der Sitzung; noreply-Mail ist im Repo gesetzt.
- Kommentare und Doku im Stil der Datei: Englisch, genaue Zahlen, Modul-Doku von `atlas.rs` ergänzen.

## Review Focus

1. Eine Insel, deren Nachbarn nur ungesehene Flächen (Label `None`) sind, bleibt unverändert und führt nicht zu einer Panik. (Test in Task 1: `island_without_neighbour_image_stays`.)
2. Grenzen zwei Bilder an eine Insel, gewinnt das mit den meisten geteilten Kanten; bei Gleichstand der niedrigere Rang. (Test in Task 1: `island_takes_image_with_most_shared_edges`.)
3. `UsedImage::camera` außerhalb von `cams` ergibt keine Übernahme und keine Panik. (Test in Task 1: `unknown_camera_is_skipped`.)
4. Ungesehene Flächen (Label `None`) bekommen nie ein Label. (Assertion in `island_without_view_takes_surrounding_image`.)
5. Bei einem leeren Netz oder einer `ViewTable` ohne Bilder ist das Ergebnis 0 und `labels` bleibt unverändert. (Test in Task 1: `empty_input_absorbs_nothing`.)

---

### Task 1: `absorb_islands` in `atlas.rs`

**Files:**
- Modify: `crates/driftless-map/src/surface/atlas.rs`, mit Konstante, Funktion, Modul-Doku Schritt 1 und Tests im bestehenden `mod tests`.

**Interfaces:**
- Produces:
  - `pub const ISLAND_FACES: usize = 8;` Charts mit weniger Flächen gelten als Insel.
  - `pub(crate) fn absorb_islands(mesh: &Mesh, views: &ViewTable, cams: &[crate::ColorCamera], cfg: &SurfaceConfig, labels: &mut [Option<u32>]) -> usize`: Gibt die Zahl der übernommenen Inseln zurück, über alle Runden.

**Algorithmus** (wie im Spike, Commit `35b04f5` auf `diagnose/qualitaet-f`, Funktion `absorb_islands`; die `diag_charts`-Ausgabe nicht übernehmen):
- Es gibt bis zu `MERGE_ROUNDS` Runden. Jede Runde bildet zuerst die Karten mit `connected(&nb, n, |f| labels[f])`.
- Inseln sind Karten mit `< ISLAND_FACES` Flächen.
- Für jede Insel werden parallel die Nachbarbilder gezählt, nach geteilten Kanten absteigend, bei Gleichstand nach Rang aufsteigend. Gewählt wird das erste Bild `l`, für das `sees(l)` gilt.
- `sees(l)` verlangt für jede Fläche der Insel:
  - Jede Ecke liegt im Kamerasystem `w2c[l]` innerhalb `cfg.color.rim_theta` von der optischen Achse, und `cam.camera.project(..)` ist `Some`.
  - Der Abstand vom Kameraort `world_from_cam.translation` zum Flächenschwerpunkt ist `≤ cfg.color.max_dist`.
  - Die flächengewichtete Summennormale von Insel und allen angrenzenden Flächen anderer Karten zeigt mit positivem Skalarprodukt zur Kamera.
- Die Wahl wird in Kartenreihenfolge angewendet. Eine Runde ohne Änderung bricht ab.

- [ ] **Step 1: Failing tests schreiben.** Jeder Test baut sich selbst eine Kamera (`Camera` Pinhole aus dem bestehenden Testmodul) mit `UsedImage { world_from_cam }`, die auf `open_wall(20, 0.1)` blickt (Wand bei x ∈ 0…2, z ∈ 0…2; die Lage aus `open_wall` ablesen). Die Labels werden von Hand gesetzt, `per_face` bleibt leer (`FaceView::EMPTY`), denn `absorb_islands` liest keine Ansichten.
  - `island_without_view_takes_surrounding_image`: Alle Flächen haben Label 0, eine Fläche in der Mitte Label 1, zwei Flächen am Rand `None`.
    - Ergebnis `1`.
    - Die Mitte hat danach Label 0.
    - Die beiden `None` bleiben `None`.
  - `large_chart_stays`: Ein Block von `ISLAND_FACES` Flächen mit Label 1 in Bild 0. Ergebnis `0`, Labels unverändert.
  - `island_outside_image_stays`: Kamera 0 ist so gedreht, dass die Inselfläche nicht projiziert wird (zum Beispiel um 90° um z). Ergebnis `0`.
  - `island_facing_away_stays`: Die Kamera steht hinter der Wand (Summennormale zeigt weg). Ergebnis `0`.
  - `island_without_neighbour_image_stays`: Die Insel ist nur von `None`-Flächen umgeben. Ergebnis `0`.
  - `island_takes_image_with_most_shared_edges`: Eine Fläche mit Label 2 hat zwei Nachbarn mit Label 1 und einen mit Label 0. Beide Kameras sehen sie. Danach hat sie Label 1.
  - `unknown_camera_is_skipped`: `UsedImage::camera = 5` bei nur einer Kamera. Ergebnis `0`, keine Panik.
  - `empty_input_absorbs_nothing`: `Mesh::default()` mit leeren Labels ergibt `0`.
- [ ] **Step 2: Tests laufen lassen, sie scheitern.** `cargo test -p driftless-map absorb island` → Kompilierfehler `absorb_islands` nicht gefunden.
- [ ] **Step 3: `ISLAND_FACES` und `absorb_islands` implementieren** (siehe Algorithmus) und die Modul-Doku Schritt 1 um einen Absatz ergänzen: was die Funktion tut, warum kein Tiefentest, und die Zahlen 192.456 → 79.750 Karten im dunklen Raum bei 1 cm sowie halb so viele Nahtproben.
- [ ] **Step 4: Tests laufen lassen.** `cargo test -p driftless-map` → alles grün. `cargo clippy -p driftless-map --all-targets -- -D warnings` → sauber.
- [ ] **Step 5: Commit** `Qualität F: Inseln übernehmen das Bild der Umgebung`.

### Task 2: In die Pipeline einhängen, Kennzahl, Version

**Files:**
- Modify: `crates/driftless-map/src/surface/atlas.rs` (`TextureStats`), `crates/driftless-map/src/surface/mod.rs` (`texture_mesh`), `crates/driftless-steps/src/surface.rs` (`SURFACE_VERSION`, `stats_json`), `crates/driftless-steps/tests/surface.rs`.

**Interfaces:**
- Consumes: `absorb_islands` aus Task 1.
- Produces: `TextureStats.islands_absorbed: usize`, in `stats_json` unter `texture.islands_absorbed`.

- [ ] **Step 1: Failing test.** Im bestehenden Pipeline-Test in `crates/driftless-steps/tests/surface.rs`, der die Schlüssel von `stats.texture` prüft (dort wie `faces_unsupported` ergänzen), wird auch `texture.islands_absorbed` als Zahl gefordert.
- [ ] **Step 2: Laufen lassen, scheitert.** `cargo test -p driftless-steps --test surface`.
- [ ] **Step 3: Implementieren.**
  - In `texture_mesh` wird `labels` mutable und nach `choose_labels` (noch vor `views.per_face = Vec::new()`, weil `views.images` gebraucht wird) mit `atlas::absorb_islands(mesh, &views, &src.cams, cfg, &mut labels)` bearbeitet.
  - Das Ergebnis kommt nach `bake` in `stats.texture.islands_absorbed`.
  - `level_seams` bekommt dieselben (geänderten) `labels`.
  - `SURFACE_VERSION` 12, `stats_json` ergänzen.
  - Die Doku von `TextureStats` bekommt eine Zeile zum neuen Feld.
- [ ] **Step 4: Alles testen.** `cargo test --workspace` → grün. Der bekannte Wackeltest `ui_tools measure_two_points_gives_distance` darf unter Last einmal scheitern: wiederholen. Danach `cargo clippy --workspace --all-targets -- -D warnings`.
- [ ] **Step 5: Commit** `Qualität F: Inseln in der Pipeline, Oberfläche v12`.

### Task 3: Messlauf und Ergebnisbericht (Controller, nicht Subagent)

- [ ] `driftless-vm sync`, dann Oberfläche für alle vier Vorlagen bei 1 cm auf den Karten von `b2` und bei 2 cm auf denen von `a2` neu rechnen. Das ist dieselbe Befehlszeile wie in `tools/qualitaet/run_vorlagen.sh`, Schritt `surface`, Ziel `~/td/quality/f/<voxel>/<scan>`.
- [ ] Tabelle in `docs/ergebnisse/2026-10-26-qualitaet-f.md`:
  - Zeilen: Karten, Seiten, `.glb` MB, `seam_step` Mittel/Median, `seam_jump`, ohne Textur %, Wandmedian (muss gleich bleiben), Zeit Oberfläche.
  - Spalten: vorher/nachher je Scan, getrennt nach 1 cm und 2 cm.
  - Dazu Ziel Z6 mit Messwert und Grund, wo es verfehlt wird: `seam_step` auf Knicken und Inhalt, `.glb` bei 1 cm durch die Geometrie mit 79 MB.
- [ ] Sichtprüfung: Je Scan einen Ausschnitt mit vielen übernommenen Inseln vorher/nachher ansehen (Rundblick-Bilder wie in `q-images.sh`). Wirken Flächen falsch eingefärbt (Verdeckung), wird die Insel-Grenze gesenkt oder ein Tiefentest ergänzt.
- [ ] Commit des Berichts. Danach `finishing-a-development-branch`: Push und PR erst nach Yannicks Okay.
