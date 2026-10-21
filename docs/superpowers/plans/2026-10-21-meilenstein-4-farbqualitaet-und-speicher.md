# Meilenstein 4 – Farbqualität, Fenster-Durchblick, dünnere Wände, Speicher – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Die eingefärbte Wolke aus Meilenstein 3 wird sichtbar sauberer (keine Lampen-Spiegelungen auf Böden, kein Randabfall der Fisheyes, Schärfe unabhängig von der Helligkeit, weichere Übergänge zwischen Bildern, „ohne Farbe“ in den Dateien), die Bereinigung entfernt Punkte, die durch Fenster gemessen wurden, und glättet Wände, und die 9,7-min-Aufnahme Promedia läuft bei 1 cm mit < 4 GB Spitzen-RAM.

**Architecture:** Alles bleibt in `driftless-map` und hinter `driftless map`. `color` behält mehr Ansichten je Punkt (kompakt gespeichert), schätzt Randabfall je Kamera gemeinsam mit den Belichtungsfaktoren und verwirft vor der Mischung Ansichten, die heller sind als die robuste Referenz des Punktes. `clean` bekommt zwei neue Teilschritte: `through` (Strahl läuft durch ein Loch in einer senkrechten Wand mit Wand darüber und darunter = Fensterband) und `smooth` (Projektion auf die lokale Ebene, nur in ebenen Nachbarschaften). Der Speicher sinkt vor allem durch einen anderen Allokator: Gemessen war der lebende Speicher nie über 2,42 GB, der Rest (bis 9,9 GB) bleibt beim macOS-Allokator nach den Bild-für-Bild-Zwischenpuffern liegen.

**Tech Stack:** wie Meilenstein 3, neu `mimalloc` (MIT, nur im Binary `driftless-cli`).

**Spec:** `docs/superpowers/specs/2026-10-20-driftless-design.md` (Ziel 3 „Qualität und Optik“, Ziel 5 „Nichts wird überschrieben“, Abschnitt 5 Datenmengen/RAM, Abschnitt 6 Zeitziele). Ausgangslage und Grenzen: `docs/ergebnisse/2026-10-21-meilenstein-3.md`.

## Global Constraints

- Lizenz Apache-2.0 OR MIT; nur MIT/Apache/BSD/Zlib-Abhängigkeiten, `cargo deny check` grün.
- macOS arm64 und Windows x64 nativ, CI grün auf beiden; `rust-version = "1.99"`.
- Deterministisch: zwei Läufe → byte-gleiche Ausgabedateien (feste Reduktionsreihenfolge, `FixedState`, ganzzahlige Zähler).
- Spec: „Nichts wird überschrieben, jede Korrektur ist ein rücknehmbarer Schritt.“ Entfernte Punkte landen mit Grund in `removed.las`; jeder neue Teilschritt ist abschaltbar (`--no-through`, `--no-smooth`, `--no-highlights`); `dense.las` bleibt ungeglättet.
- Speicher: Promedia (`jmk6-2026-10-20/20261020173822`, 582 s, 3.831 Bilder) bei 1 cm in jedem Schritt Spitzen-RAM < 4 GB (M3: 9,94 GB); `Test` weiter < 8 GB.
- Zeit (M4 Pro): `Test` `slam + map` < 15 min (M3: 140 s); Promedia `map` < 15 min (M3: 338 s).
- GPU (wgpu, Spec) bleibt außerhalb dieses Meilensteins, ebenso Gaussian Splatting. Neue schwere Schleifen als reine Per-Element-Funktionen + feste Reduktion (wie `visibility.rs`, `clean.rs`).
- Code-Kommentare Englisch, Meldungen Deutsch. Scan-Daten, Bilder, Seriennummern, IPs, persönliche Pfade nie ins Repo; Tests mit echten Daten `#[ignore]`, über `$DRIFTLESS_TESTDATA`.

## Review Focus

- Punkt mit nur 1–2 Ansichten (Ecken, selten gesehene Flächen) → die Glanzlicht-Regel darf ihm die einzige Farbe nicht nehmen (Task 4: `few_views_are_never_all_rejected`).
- Glaswand oder Glastür, hinter der später weitergescannt wird → der Raum dahinter bleibt (Task 6: `keeps_room_behind_glass_once_entered`); offene Tür in einen nie betretenen Raum → bleibt (Task 6: `keeps_room_seen_through_door`).
- Kamera, deren Ansichten fast alle bei kleinem θ liegen (Randabfall nicht bestimmbar) → Koeffizienten bleiben 0, kein Aufschaukeln (Task 3: `vignetting_unobservable_stays_flat`).
- Ecken, Kanten, dünne Stäbe (Stuhlbeine, Blätter) → die Wandglättung verschiebt sie nicht (Task 7: `keeps_corners_and_thin_rods`).
- Fast schwarzes oder einfarbiges Bild → Schärfe endlich, kein NaN, Bild gilt als unscharf statt als Absturz (Task 2: `dark_and_flat_images_are_finite`).

---

### Task 1: Speicher – Allokator, kompakte Ansichten, Promedia als Testdaten

**Files:**
- Modify: `crates/driftless-cli/Cargo.toml`, `crates/driftless-cli/src/main.rs` (`#[global_allocator] static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;`, `mimalloc = { version = "0.1", default-features = false }`), `crates/driftless-map/src/color.rs`, `crates/driftless-map/src/exposure.rs`, `docs/testdaten.md`
- Create: `crates/driftless-map/tests/memory.rs` (eigener Test-Binary mit zählendem `#[global_allocator]`, genau ein Test)

**Interfaces:**
- Produces: `pub(crate) struct View { pub(crate) weight: f32, pub(crate) rank: u32, pub(crate) rgb: [u16; 3] /* linear × 65535 */, pub(crate) theta: u16 /* θ / π × 65535 */ }` (16 B statt 20 B) mit `View::linear(&self) -> [f32; 3]` und `View::theta(&self) -> f32`; `ColorConfig.views_per_point` wird zu `views_kept` und `views_blended` (hier beide 3, Verhalten wie M3; Task 4 setzt `views_kept = 6`). `slots` wird erst nach `local_geometry` angelegt; `local_geometry` baut den Baum auf `f32`.

Messung beim Planen (Promedia, 1 cm, je `--step` einzeln, Release, zählender Allokator; Zahlen für das Ergebnisdokument):

| Schritt | Spitzen-RSS | lebender Speicher Spitze | Zeit |
|---|---|---|---|
| dense (15,6 Mio. Punkte, 27,5 Mio. im Bereich, 6,9 Mio. Strahlen) | 1,71 GB | 1,54 GB | 33 s |
| clean (inkl. Laden `dense.bin`) | 2,35 GB | 1,80 GB | 8 s |
| color, Systemallokator | 8,9–9,9 GB | 2,42 GB (in `local_geometry`), Bildschleife 1,95–2,0 GB + ≤ 0,35 GB je Bild | 302 s |
| color, `mimalloc` | **2,96 GB** | wie oben | 312 s, `color.las`/`.ply` byte-gleich |

Lebend beim Einfärben (15 Mio. Punkte): Wolke 28 B/Punkt (0,42 GB), `slots` 3 × 20 B (0,90 GB), Normalen 16 B (0,24 GB), Abstand 4 B, `PointGrid` 4 B (+16 B beim Sortieren), `local_geometry` f64-Kopie 24 B + Baum (vorübergehend ≈ 0,7 GB), je Bild bis 13,6 Mio. Kandidaten × (4 B Index + 20 B Projektion). Schon der Qualitätsdurchlauf (lebend 0,59 GB) stand bei 2,25 GB RSS. Kacheln/Streaming der Wolke ist deshalb nicht nötig; mit 6 Ansichten × 16 B (1,44 GB) bleibt der lebende Speicher bei ≈ 2,7 GB.

- [ ] **Step 1: Tests** – `memory.rs`: `colorize_live_heap_per_point` (synthetische Szene `walk_data()`, zählender Allokator: Spitze des lebenden Speichers während `colorize` ≤ 200 B × Punkte + 64 MB). Bestehende `color.rs`/`exposure.rs`-Tests laufen mit `views_kept = 3, views_blended = 3` unverändert (Ergebnis byte-gleich zu vorher bis auf die u16-Rundung: Farben ±1/255). `view_round_trip` (Einheitstest: `rgb`/`theta` über u16 auf 1e-4 genau).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-map`), `cargo deny check` nach `mimalloc`.
- [ ] **Step 5: Messen** Promedia `--step dense|clean|color` je mit `/usr/bin/time -l` (Spitzen-RSS je < 4 GB), Werte notieren. `docs/testdaten.md`: Promedia-Aufnahme (Ordnername, 582 s, 3.831 Bilder, Büro mit Deckenleuchten und Fensterbändern, Drift ohne Schleifen 2,16 m → 0,13 m) ohne persönliche Daten.
- [ ] **Step 6: Commit** `perf(map): mimalloc und kompakte Ansichten, Promedia unter 4 GB`

### Task 2: Schärfe unabhängig von der Helligkeit

**Files:**
- Modify: `crates/driftless-map/src/sharpness.rs`, `crates/driftless-map/src/color.rs` (Statistik)

**Interfaces:**
- Produces: `pub fn sharpness(img: &RgbImage) -> f64` = `var(Laplace) / max(mittlerer Grauwert, SHARPNESS_MIN_MEAN)²` mit `pub const SHARPNESS_MIN_MEAN: f64 = 16.0` (dimensionslos). `ColorStats.sharpness_brightness_rank_corr: f64` (Spearman zwischen Schärfe und mittlerer Helligkeit der Bilder auf der Laufbahn).

- [ ] **Step 1: Tests** – `brightness_scaling_keeps_sharpness` (Testbild ×0,3 und ×1: Schärfe auf 5 % gleich; vorher Faktor ≈ 11). `blur_still_lowers_sharpness` (Gauß σ = 2 px → < 0,3 × ungefiltert). `dark_and_flat_images_are_finite`. `rank_corr_of_brightness_ramp` (synthetisch, Einheitstest der Korrelation).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `fix(map): Schärfe normiert, dunkle Bilder nicht mehr unscharf`

### Task 3: Randabfall (Vignettierung) gemeinsam mit dem Belichtungsabgleich

**Files:**
- Modify: `crates/driftless-map/src/exposure.rs`, `crates/driftless-map/src/color.rs`, `crates/driftless-map/src/lib.rs`
- Test: `crates/driftless-map/tests/exposure.rs`

**Interfaces:**
- Consumes: `View::theta` (Task 1).
- Produces:
  - `Observations::push_point(&mut self, views: &[ViewObs])` mit `pub struct ViewObs { pub image: u32, pub rgb: [f32; 3], pub theta: f32 }`; `Observations::new(cameras)` unverändert.
  - `pub struct Vignetting { pub coeffs: Vec<[[f64; 2]; 3]> /* je Kamera, je Kanal: b1, b2 */ }` mit `Vignetting::factor(&self, camera: usize, channel: usize, theta: f32) -> f32 = exp(b1·θ² + b2·θ⁴)`, begrenzt auf `[1/max_gain, 1]`.
  - `pub fn estimate_exposure(obs: &Observations, cfg: &ExposureConfig) -> (Vec<[f32; 3]>, Vignetting)` ersetzt `estimate_gains` (bleibt als Hülle, liefert nur Faktoren). Je Kanal abwechselnd `VIGNETTE_ROUNDS = 3`: (a) Faktoren wie bisher (CG + Huber) auf `log c − log V`, (b) je Kamera `b1, b2` per gewichteter kleinster Quadrate (2 × 2, Huber-Gewichte aus (a)) auf den Rest `log c − log g_i − log a_j`. Kein konstanter Term: `V(0) = 1`, die Gesamthelligkeit bleibt bei `mean(log g) = 0`. Hat eine Kamera weniger als `VIGNETTE_MIN_SPREAD = 0,35 rad` Streuung in θ oder < 1.000 Beobachtungen, bleibt ihr `V = 1`.
  - `ExposureConfig.vignetting: bool` (Standard an); `ColorStats.vignetting: Vignetting`, `ColorStats.vignette_residual_before/after: f64` = je Kamera größter Betrag des mittleren Luminanz-Rests (log, in %) über θ-Klassen von 10° mit ≥ 1.000 Beobachtungen, Mittel über die Kameras; `ColorStats.images_gain_clamped: usize`.
  - Anwendung vor der Mischung: Ansicht / (`g_i` · `V_cam(θ)`).

Befund beim Planen (Promedia, 1 cm): Faktoren stoßen an beide Grenzen (0,25 und 4,0), die Streuung steigt durch den Abgleich (6,71 → 6,94). Probelauf mit `max_gain = 8`: 387 von 3.701 Bildern mit Faktor < 0,25, fast alle in einem Block 342–371 s nach Start (beide Kameras, 0,16–0,30), dazu 6 Bilder bei 17 s mit 3,6–4,4; Kamera-Unterschied danach schlechter (0,58 → 0,73), Streuung 7,02. **Entscheidung: `max_gain` bleibt 4**, geklemmte Bilder werden gezählt (`images_gain_clamped`, Promedia heute 498 = 13 %) und gewarnt; Ursache des Blocks (heller Raum ohne Überlappung oder Überbelichtung) klärt Task 9 an den Bildern.

- [ ] **Step 1: Tests** – `recovers_injected_vignetting` (beide Kameras `V = 1 − 0,5·(θ/θmax)²` in die gerenderten Bilder, dazu die Faktoren aus `recovers_injected_gains`: `V` auf 3 % genau in allen θ-Klassen, Faktoren weiter auf 3 %; Rest nachher < 0,3 × vorher). `vignetting_unobservable_stays_flat` (alle Ansichten θ < 20°: `coeffs == 0`). `vignetting_off_matches_m3_gains` (`vignetting = false` → gleiche Faktoren wie `estimate_gains` vorher). `gains_at_clamp_are_counted`. `exposure_is_deterministic` erweitert.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Randabfall der Fisheyes aus den Daten geschätzt`

### Task 4: Glanzlichter verwerfen, weiche Mischung, Nahtsprung

**Files:**
- Modify: `crates/driftless-map/src/color.rs`, `crates/driftless-map/src/exposure.rs` (Kennzahlen)
- Test: `crates/driftless-map/tests/color.rs`, `crates/driftless-map/tests/common/mod.rs` (Glanzfleck in gerenderte Bilder)

**Interfaces:**
- Consumes: `View` (Task 1), Faktoren + `Vignetting` (Task 3).
- Produces:
  - Regel je Punkt nach Abgleich (lineare Luminanz `Y = 0,2126 R + 0,7152 G + 0,0722 B` je Ansicht): Referenz `Y_ref` = zweitdunkelste Ansicht bei ≥ 3 Ansichten, sonst die dunkelste. Ansicht verworfen, wenn `Y > Y_ref · (1 + highlight_ratio)` **und** `Y − Y_ref > highlight_abs`. Glanz ist additiv und betrifft oft mehrere aufeinanderfolgende Bilder; die zweitdunkelste Ansicht übersteht einen einzelnen zu dunklen Ausreißer (Schatten des Bedieners, Lagefehler) und bis zu `views_kept − 2` Glanzansichten. Die Referenz selbst wird nie verworfen.
  - `ColorConfig { highlight_ratio: f64 /* 0.3 */, highlight_abs: f64 /* 0.02 linear */, highlights: bool /* an */, views_kept: usize /* 6 */, views_blended: usize /* 3 */ }`. 6 statt 3 behaltene Ansichten: Bilder je Kamera ≈ 3,3/s, die 6 besten liegen ≈ 1 s auseinander; 8 kosteten 0,5 GB mehr (Task 1).
  - Weiche Mischung: von den übrigen Ansichten (nach Gewicht) die besten `views_blended` mit Gewicht `w_k − w_(views_blended+1)` (0, wenn es keine weitere gibt); sind alle Gewichte danach 0, die unveränderten Gewichte. So wechselt die Farbe stetig, wenn zwischen Nachbarpunkten die 3. und 4. Ansicht die Plätze tauschen (Ursache harter Nähte).
  - Abgleich-Reihenfolge: Faktoren + Randabfall auf allen behaltenen Ansichten (Huber dämpft Glanz), dann Glanz verwerfen, dann mischen.
  - Nähte (Ergebnis M3: Kamera-Unterschied bei 1 cm auf `Test` nur −46 %): Auf Promedia sinkt er bei 1 cm um 83 % (3,33 → 0,58). Die −46 % auf `Test` sind also kein 1-cm-Effekt, sondern kleine Absolutwerte (1,42 → 0,76 Stufen) einer dunklen Szene. Was an Nähten bleibt, kommt aus Randabfall (Task 3), Glanz (oben), dem harten Schnitt nach 3 Ansichten (weiche Mischung) und geklemmten Bildern (Task 3); gemessen mit `seam_jump_*`.
  - `ColorStats`: `views_rejected`, `points_with_rejected`, `floor_highlight_share_before/after` (Bodenpunkte: Normale `|n_z| > 0,9`, Höhe < Laufbahnhöhe − 0,8 m; Anteil mit `Y > 1,3 × Median Y` ihrer 0,5-m-Zelle; „vorher“ = ohne Verwerfen und harte Mischung, aus denselben Ansichten), `seam_jump_before/after` (bis 200.000 gleichmäßig gewählte ebene Punkte mit ihren 4 nächsten Nachbarn ≤ 2 Voxel, Normalen ≤ 10°: mittleres `|ΔY|` in sRGB-Stufen bei verschiedenem Hauptbild minus bei gleichem Hauptbild; Hauptbild = Ansicht mit dem größten Mischgewicht).

- [ ] **Step 1: Tests** – `rejects_view_dependent_highlights` (Boden der Testszene, in jedem 2. Bild ein heller Fleck +120 sRGB, der mit der Kamera wandert: > 95 % der Bodenpunkte auf ±`TOL` der Bodenfarbe; mit `highlights = false` < 85 %). `few_views_are_never_all_rejected` (1 und 2 Ansichten: Farbe bleibt, `colored = true`). `dark_outlier_does_not_become_reference` (5 gleiche Ansichten + 1 sehr dunkle: nichts verworfen). `soft_blend_is_continuous` (Einheitstest: Gewichte 4. ≈ 3. → Farbe ändert sich beim Tausch um < 1/255). `seam_jump_drops_with_soft_blend` (Szene mit zwei Bildgruppen unterschiedlicher Restbelichtung). Bestehende Farbtests grün.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Glanzlichter verwerfen und weiche Mischung`

### Task 5: Kennzeichnung „ohne Farbe“ und Entfernungsgrund in LAS/PLY

**Files:**
- Modify: `crates/driftless-map/src/export.rs`, `crates/driftless-map/src/clean.rs`, `crates/driftless-map/src/lib.rs`, `crates/driftless-cli/src/map.rs` (`removed.las` über `write_removed_las`)
- Test: `crates/driftless-map/tests/export.rs`, `crates/driftless-map/tests/clean.rs`

**Interfaces:**
- Produces:
  - `write_las`: LAS-Feld `user_data` = 1 für Punkte mit `colored == Some(false)`, sonst 0 (Format 3 bleibt; CloudCompare zeigt „User Data“ als Skalarfeld). `write_ply`: nach RGB `property uchar colored` (1 = gemessene Farbe), nur wenn `colored` vorhanden.
  - `#[repr(u8)] pub enum RemovalReason { Ghost = 1, Outlier = 2 }` (Task 6 ergänzt `Through = 3`); `clean` liefert `(Cloud, Cloud, Vec<RemovalReason>, CleanStats)`, die Gründe in der Reihenfolge der entfernten Punkte; `pub fn write_removed_las(path, &Cloud, &[RemovalReason])` schreibt den Grund in `user_data`.
- [ ] **Step 1: Tests** – `las_user_data_marks_uncolored`, `ply_has_colored_property` (Header und Größe), `removed_las_carries_reason` (mit `las`-Reader zurücklesen), `removed_plus_kept_equals_input` erweitert.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): „ohne Farbe“ und Entfernungsgrund in den Dateien`

### Task 6: Durchblick durch Fenster entfernen

**Files:**
- Create: `crates/driftless-map/src/through.rs`
- Modify: `crates/driftless-map/src/clean.rs` (gemeinsames Belegt-Raster mit Normalen, Aufruf), `crates/driftless-map/src/lib.rs`
- Test: `crates/driftless-map/tests/through.rs`

**Interfaces:**
- Consumes: `RaySource`, `Cloud.frame` (Sensorposition je Punkt = `rays.sensor[frame]`), Belegt-Raster aus `clean.rs` (0,1 m, Normale je Voxel), `walk_ray`.
- Produces:
  - `pub fn find_through(cloud: &Cloud, rays: &RaySource, cfg: &CleanConfig) -> Vec<bool>`. Wandvoxel = belegt, eben, `|n_z| < 0,3`. Je Säule (x, y) die sortierten z-Schlüssel ihrer Wandvoxel (`HashMap<(i32, i32), Vec<i32>, FixedState>`). Ein Strahl bzw. eine Sichtlinie Sensor → Punkt gilt als „durch ein Fenster“, wenn er ein freies Voxel durchläuft, dessen Säule innerhalb `window_reach` (1,5 m) darüber **und** darunter ein Wandvoxel hat, dessen waagrechte Normale zur Strahlrichtung passt (`|n·d| ≥ 0,5`), und danach noch ≥ `through_beyond` (0,5 m) weiterläuft. Je 0,1-m-Voxel der Wolke zählen (ganzzahlig, atomar) „durch“ und „direkt“ über alle Strahlen und alle Punkte (Sichtlinie aus ihrem eigenen Frame). Punkt ist Durchblick, wenn sein Voxel `durch ≥ 1` und `direkt == 0` hat.
  - Warum Wand darüber **und** darunter: Fensterband = Brüstung unten, Sturz oben. Eine Tür hat unten Boden (waagrechte Normale) und bleibt; Räume, die später betreten werden, bekommen „direkt“ und bleiben. Bodentiefe Glasflächen werden nicht erkannt (Grenze, im Ergebnis nennen).
  - `CleanConfig { through: bool /* an */, window_reach: f64 /* 1.5 */, through_beyond: f64 /* 0.5 */ }`; `CleanStats.through`; `RemovalReason::Through = 3`; Reihenfolge Geister → Durchblick → Ausreißer.
- [ ] **Step 1: Tests** (Szene aus Platten: Raum mit Fensterband in einer Wand, Brüstung 0,9 m, Sturz 2,1 m, draußen Kisten 8–15 m; offene Tür zu einem zweiten Raum) – `removes_points_seen_through_window` (> 90 % der Außenpunkte entfernt, < 0,5 % der Raumpunkte). `keeps_room_seen_through_door` (Laufbahn betritt Raum 2 nie: < 1 % entfernt). `keeps_room_behind_glass_once_entered` (Fensterband innen, Laufbahn geht später hinein: < 1 % entfernt). `through_is_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Punkte hinter Fenstern entfernen`

### Task 7: Dünnere Wände (Projektion auf die lokale Ebene)

**Files:**
- Create: `crates/driftless-map/src/smooth.rs`
- Modify: `crates/driftless-map/src/clean.rs`, `crates/driftless-map/src/lib.rs`
- Test: `crates/driftless-map/tests/smooth.rs`

**Interfaces:**
- Produces:
  - `pub struct SmoothConfig { pub enabled: bool /* an */, pub radius: f64 /* 0.10 m */, pub max_neighbours: usize /* 48 */, pub min_neighbours: usize /* 10 */, pub planarity: f64 /* λ0 < 0.05·λ1 */, pub max_shift: f64 /* 0.03 m */ }`, als `CleanConfig.smooth`.
  - `pub fn smooth_surfaces(xyz: &[[f32; 3]], cfg: &SmoothConfig) -> (Vec<[f32; 3]>, SmoothStats)`: je Punkt aus den **Eingangs**-Koordinaten (nicht den schon verschobenen; daher reihenfolgeunabhängig) Schwerpunkt und Kovarianz der Nachbarn ≤ `radius` (kiddo `f32`, bis `max_neighbours`), bei Ebene Projektion auf sie, Verschiebung auf `max_shift` begrenzt; sonst unverändert. `SmoothStats { moved, mean_shift, thickness_before, thickness_after }`; Wanddicke = Median über ebene 10-cm-Zellen mit ≥ 30 Punkten der RMS-Abstände zur Zellebene.
  - Läuft in `clean` nach dem Entfernen; `clean.las`/`clean.bin` tragen die geglätteten Koordinaten, `dense.las` bleibt roh. Warum nicht Mitteln im Voxel: 1-cm-Voxel sind kleiner als das Rauschband (3–5 cm), Mitteln darin macht die Wand nicht dünner.
- [ ] **Step 1: Tests** – `thins_noisy_wall` (Ebene mit ±2 cm Rauschen: Dicke nachher < 0,4 × vorher). `keeps_corners_and_thin_rods` (90°-Ecke, Stab Ø 2 cm: Punkte ≤ 5 mm von der Wahrheit wie vorher, Kanten nicht gerundet). `shift_is_capped`. `smooth_is_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(map): Wände auf die lokale Ebene projiziert`

### Task 8: Befehl `driftless map` – Schalter, Kennzahlen, Doku

**Files:**
- Modify: `crates/driftless-cli/src/map.rs`, `README.md`
- Test: `crates/driftless-cli/tests/map.rs`

**Interfaces:**
- Produces: neue Schalter `--no-through`, `--no-smooth`, `--no-highlights`, `--no-vignetting`; `map.json`: alle neuen Einstellungen unter `settings`, neue Statistiken (`through`, `smooth.*`, `views_rejected`, `floor_highlight_share_*`, `seam_jump_*`, `vignette_residual_*`, `vignetting` je Kamera, `images_gain_clamped`, `sharpness_brightness_rank_corr`). Konsole nennt Durchblick-Punkte, geglättete Punkte und Wanddicke vorher/nachher, verworfene Glanzansichten. Warnung, wenn `images_gain_clamped > 1 %` der Bilder. Die Kennungen der Schritte ändern sich über die Einstellungen; Ordner aus M3 werden dadurch neu gerechnet, nicht falsch gelesen.
- [ ] **Step 1: Tests** – `map_all_writes_outputs` erweitert (Kennzahlen in `map.json`, `user_data` in `color.las`), `map_no_smooth_keeps_dense_coordinates`, `m3_folder_is_rebuilt_not_misread` (`map.json`/`*.bin` mit alter Kennung → neu erzeugt, Warnung zu alten Dateien wie bisher), `map_is_deterministic`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test --workspace`).
- [ ] **Step 5: Commit** `feat(cli): Schalter und Kennzahlen für Meilenstein 4`

### Task 9: Prüfung mit echten Daten, Ergebnisse

**Files:**
- Modify: `crates/driftless-map/tests/real_data.rs` (`#[ignore]`), `docs/testdaten.md`, `README.md`; Standardwerte nur, wenn die Messung es verlangt (jede Änderung im Ergebnis begründet)
- Create: `crates/driftless-cli/tests/real_data.rs` (`#[ignore]`, startet `env!("CARGO_BIN_EXE_driftless")`), `docs/ergebnisse/YYYY-MM-DD-meilenstein-4.md` (Datum des Messtags)

- [ ] **Step 1: Messen** auf `Test`, `143415`, `143623` und Promedia, je M3-Stand (Commit `49d16ea`) gegen M4: Glanzanteil Boden, Kamera-Unterschied, Bild-Versatz, Nahtsprung, Randabfall-Rest, Bilder am Grenzwert, Schärfe-Helligkeit-Korrelation und unscharfe Bilder, Durchblick-Punkte (davon > 5 m von der Laufbahn), Wanddicke (auch RayStudio `smooth.las` auf `Test`), Spitzen-RAM je Schritt (`/usr/bin/time -l`), Laufzeit je Schritt. Ausschnitt-Renderings (Boden unter Deckenleuchten in Promedia, Fensterseite von oben, Wand im Schnitt) lokal unter `$DRIFTLESS_TESTDATA/ergebnisse-m4/`.
- [ ] **Step 2: Tests mit gemessenen Schranken** – CLI-Crate: `promedia_map_under_4_gb_and_15_min` (je `--step` ein Kindprozess, `peak_rss_bytes` aus `map.json`); Map-Crate: `floor_highlights_drop` (Promedia: nachher ≤ 0,5 × vorher), `vignette_residual_drops` (nachher ≤ 0,5 × vorher), `camera_difference_not_worse` (≤ M3-Wert × 1,05), `through_removes_outside_points` (Promedia: Durchblick > 0, davon ≥ 80 % > 3 m von der Laufbahn), `walls_thinner` (`Test`: Dicke nachher ≤ 0,7 × vorher), `jmk6_scans_run_and_are_deterministic` erweitert um Promedia.
- [ ] **Step 3: Ergebnisdokument** (Deutsch, Tabellen, ehrliche Grenzen: bodentiefe Fenster, Glanz in allen Ansichten, Wahrheit fehlt) und Doku.
- [ ] **Step 4: Gesamtprüfung** – `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo deny check`, `cargo fmt --all --check`, `cargo test --release -- --ignored`.
- [ ] **Step 5: Commit** `test(map): Prüfung mit echten Daten, Ergebnisse Meilenstein 4`
