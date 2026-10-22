# Baustein GPU – Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Das Einfärben der Karte und die Ansichten je Dreieck der Oberfläche rechnen auf der Grafikkarte (wgpu-Compute: Vulkan/RTX, Metal/Mac, DX12), mit vollständigem CPU-Rückfall.

**Architecture:** Neues Crate `driftless-gpu` (Gerät finden, Puffer, WGSL-Kerne für Fisheye-Projektion, Tiefenpuffer, Sichtbarkeit, beste k Ansichten). `driftless-map` bekommt je teurem Teilschritt einen GPU-Weg neben dem CPU-Weg (`color.rs` zuerst, `surface/views.rs` nach Runde B von Baustein 2). Bilder werden weiter auf der CPU dekodiert, überlappend mit der GPU-Arbeit. Auswahl `GpuPref { Auto, On, Off }` über `driftless-steps` bis CLI (`--gpu`) und Programm.

**Tech Stack:** `wgpu` (dieselbe Version wie `egui-wgpu` 0.36 im Programm, über den Workspace geteilt), `bytemuck`, `pollster`; WGSL.

**Spec:** `docs/superpowers/specs/2026-10-23-gpu-design.md`. Messbasis: `docs/ergebnisse/2026-10-21-meilenstein-4.md` (Einfärben), `docs/ergebnisse/2026-10-22-baustein-2.md` (Oberfläche).

## Global Constraints

- Lizenz Apache-2.0 OR MIT; nur Abhängigkeiten aus `deny.toml` (`cargo-deny check licenses` grün); kein CUDA.
- macOS arm64 + Windows x64 CI grün; CI hat keine GPU: GPU-Tests nutzen einen Software-Adapter, wenn wgpu einen findet, sonst werden sie mit Meldung übersprungen. CPU-Tests bleiben Pflicht und unverändert grün.
- Gleiche Eingabe auf demselben Gerät → gleiche Dateien (ganzzahlige Atomics, feste Reduktion; keine reihenfolgeabhängigen Float-Atomics). GPU gegen CPU: Grenzen je Test festgelegt.
- Kein Absturz/Hängen ohne Grafikkarte, headless oder wenn die Karte belegt ist: `Auto` fällt auf CPU zurück und meldet es (deutsch); `On` ohne taugliche GPU → klarer Fehler.
- Grafikspeicher für einen Raum-Scan < 4 GB; Arbeitsspeicher-Grenzen der Schritte bleiben (< 4 GB).
- Kommentare Englisch, Meldungen Deutsch. Scan-Daten nie ins Repo; Echtdaten-Läufe auf VM 101 (halbe CPU-Last, GPU über `gpu-run`-Warteschlange) und auf dem Mac nur kurz für Metal-Messung.

## Review Focus

- Kein Adapter / nur Software-Adapter / Karte von Win11 belegt → `Auto` rechnet auf CPU mit Hinweis, Ergebnis wie heute (Task 1: `auto_without_adapter_falls_back`).
- Sehr großer Scan (Büro 15 Mio. Punkte, 3.831 Bilder) → Puffer in Stapeln, Grafikspeicher < 4 GB (Task 3: `colorize_gpu_batches_large_cloud`).
- Punkte genau auf dem Rand des Fisheye-Gültigkeitsbereichs (`max_theta`) und hinter der Kamera → GPU und CPU entscheiden gleich (Task 1: `projection_matches_cpu_at_rim`).
- Gleichstand von Gewichten zweier Ansichten → gleiche Wahl wie CPU (Rang als Tie-Break) (Task 3: `ties_break_by_rank_like_cpu`).
- Abbrechen während eines GPU-Durchgangs → `Cancelled`, keine halben Dateien, Gerät wieder frei (Task 3: `cancel_during_gpu_pass`).

---

### Task 1: Crate `driftless-gpu` – Gerät und Fisheye-Projektion

**Files:**
- Create: `crates/driftless-gpu/{Cargo.toml, src/lib.rs, src/device.rs, src/project.rs, src/project.wgsl}`
- Modify: `Cargo.toml` (Workspace-Mitglied; `wgpu` als Workspace-Abhängigkeit in der Version, die `egui-wgpu` 0.36 nutzt, und `driftless-app` darauf umstellen, damit es nur eine wgpu-Version gibt), `deny.toml` nur falls nötig

**Interfaces:**
- Produces: `pub enum GpuPref { Auto, On, Off }` (`FromStr`: `auto|an|aus`); `pub struct GpuInfo { pub name: String, pub backend: String, pub driver: String, pub software: bool }`; `pub struct GpuContext` mit `pub fn new(pref: GpuPref) -> Result<Option<GpuContext>, GpuError>` (`Off` → `Ok(None)`, `Auto` ohne taugliches Gerät → `Ok(None)`, `On` ohne → `Err` deutsch), `pub fn info(&self) -> &GpuInfo`, `pub fn device(&self)`, `pub fn queue(&self)`; `pub struct GpuCamera` (Fisheye-Parameter wie `driftless_io::camera::Camera`, `Pod`); `pub fn project_points(ctx: &GpuContext, cam: &GpuCamera, world_from_cam: &Isometry3<f64>, xyz: &[[f32; 3]]) -> Vec<Option<[f32; 2]>>` (Testhilfe und Baustein für Task 2/3). Software-Adapter (lavapipe/WARP) zählt bei `Auto` nicht als tauglich, für Tests per `GpuContext::new_for_tests()` erlaubt.

- [ ] **Step 1: Tests:** `projection_matches_cpu` (10.000 zufällige Punkte vor der Kamera: GPU-uv = `Camera::project` ± 1e-2 px); `projection_matches_cpu_at_rim` (Punkte bei θ = max_theta ± 1e-4 und hinter der Kamera: gleiche Gültigkeit); `auto_without_adapter_falls_back` (`Off` → None; `Auto` mit erzwungen leerer Adapterliste → None); `pref_parses` (`auto|an|aus`).
- [ ] **Step 2–4:** FAIL → implementieren → PASS (`cargo test -p driftless-gpu`; ohne Adapter überspringen die GPU-Tests mit Meldung).
- [ ] **Step 5: Commit** `feat(gpu): Gerät und Fisheye-Projektion auf der Grafikkarte`

### Task 2: Tiefenpuffer und Sichtbarkeit auf der GPU

**Files:**
- Create: `crates/driftless-gpu/src/visibility.rs`, `crates/driftless-gpu/src/visibility.wgsl`

**Interfaces:**
- Consumes: `GpuContext`, `GpuCamera`
- Produces: `pub struct GpuCloud` (Punkte + Normalen + Abstand je Punkt einmal hochgeladen, in Stapeln ≤ 256 MB); `pub struct ImagePass` mit `pub fn depth_and_visibility(&mut self, cam: &GpuCamera, world_from_cam: &Isometry3<f64>, params: &VisParams) -> Visible` – Tiefenpuffer wie `driftless_map::visibility::DepthBuffer` (Zellgröße `zbuffer_scale`, Splat-Radius wie `MAX_SPLAT`, `atomicMin` auf Tiefe als u32), danach je Punkt sichtbar ja/nein nach `DEPTH_TOLERANCE_REL`/`depth_tolerance`. `VisParams` spiegelt die Werte aus `ColorConfig`. `Visible` bleibt auf der GPU (Puffer), mit Lesehilfe für Tests.

- [ ] **Step 1: Tests:** `visible_set_matches_cpu` (synthetische Szene aus den `color.rs`-Tests: sichtbare Punktmenge je Bild ≥ 99,9 % gleich der CPU-Menge, Unterschiede nur an Tiefenkanten); `occluder_hides_wall` (Säule vor Wand → Wand dahinter unsichtbar); `deterministic_on_same_device` (zweimal → gleiche Bits).
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Commit** `feat(gpu): Tiefenpuffer und Sichtbarkeit`

### Task 3: Einfärben der Karte auf der GPU

**Files:**
- Create: `crates/driftless-map/src/color_gpu.rs`, `crates/driftless-gpu/src/views.wgsl`
- Modify: `crates/driftless-map/src/color.rs` (`colorize_with` bekommt `gpu: Option<&GpuContext>`; CPU-Weg unverändert), `crates/driftless-map/Cargo.toml`, `crates/driftless-map/src/lib.rs`, `crates/driftless-steps/src/map.rs` (`MapSwitches.gpu: GpuPref`, Bericht `map.json`: `gpu: { used, name, backend, driver }`), `crates/driftless-cli/src/map.rs` (`--gpu auto|an|aus`, Standard `auto`)

**Interfaces:**
- Consumes: Task 1–2
- Produces: GPU-Weg, der je Bild auf der GPU projiziert, Sichtbarkeit prüft, Gewicht nach `view_weight` rechnet, die Farbe aus dem hochgeladenen Bild (lineare Tabelle wie `sample_linear`) liest und die besten `views_kept` Ansichten je Punkt in GPU-Slots hält (Tie-Break Rang wie CPU, ganzzahlig gepackt). Nach dem letzten Bild: Slots zurücklesen, Belichtungs-/Mischschritt bleibt CPU (unverändert). Bilddekodierung überlappt (Kanal mit 2 Bildern Vorlauf). `map.json` dokumentiert, ob GPU genutzt wurde. Die Eingabe-ID der Farbstufe enthält `gpu` NICHT (gleichwertiges Ergebnis), der Bericht schon.

- [ ] **Step 1: Tests:** `colorize_gpu_matches_cpu` (synthetische Szene: ≥ 99 % der Punkte Farbe ± 2 sRGB-Stufen, Anteil gefärbter Punkte gleich ± 0,1 %); `ties_break_by_rank_like_cpu`; `colorize_gpu_batches_large_cloud` (künstlich kleine Stapelgröße → gleiches Ergebnis wie ein Stapel); `cancel_during_gpu_pass` (Abbruch nach 2 Bildern → `Cancelled`, Wolke unverändert); `map_json_reports_gpu`; CLI `--gpu aus` → CPU-Weg byte-gleich zu heute.
- [ ] **Step 2–4:** FAIL → implementieren → PASS; `cargo test --workspace`, `cargo-deny check licenses`.
- [ ] **Step 5: Messen** auf VM 101 (RTX, über `gpu-run`, CPU halbe Last) und auf dem Mac (Metal, nur der Farbschritt `--step color`): schwarzer Raum und Büro, `--gpu an` gegen `--gpu aus`: Zeit, RAM, Grafikspeicher, Farbabweichung. Ziel RTX ≥ 5×, M4 Pro ≥ 2×.
- [ ] **Step 6: Commit** `feat(map): Einfärben auf der Grafikkarte`

### Task 4: Ansichten je Dreieck auf der GPU (nach Runde B von Baustein 2)

**Files:**
- Create: `crates/driftless-map/src/surface/views_gpu.rs`
- Modify: `crates/driftless-map/src/surface/views.rs` (`collect_views` mit `gpu: Option<&GpuContext>`), `crates/driftless-steps/src/surface.rs` (`--gpu`, Bericht), `crates/driftless-cli/src/surface.rs`

**Interfaces:**
- Consumes: Task 1–3 (Projektion, Tiefenpuffer aus Dreiecks-Stichproben, Slots), `ViewTable` unverändert
- Produces: gleiches `ViewTable` wie der CPU-Weg (Score-Formel, Ecken-und-Mitte-Sichtbarkeit, beste `views_per_face`), Belichtungs-Stichprobe wie CPU.

- [ ] **Step 1: Tests:** `views_gpu_match_cpu` (synthetische Szenen aus `views.rs`: gleiche beste Ansicht für ≥ 99 % der Dreiecke, Scores ± 1 %); `partly_hidden_face_rejected_gpu`; `simplified_wall_gets_views_gpu`.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5: Messen** wie Task 3 Step 5 für `driftless surface`.
- [ ] **Step 6: Commit** `feat(map): Ansichten der Oberfläche auf der Grafikkarte`

### Task 5: Schalter im Programm, TSDF-Entscheidung, Ergebnisse

**Files:**
- Modify: `crates/driftless-app/src/ui/{right,text,app}.rs` (Schalter „Grafikkarte nutzen“: Auto/An/Aus, Anzeige des Geräts), `README.md`, `docs/ergebnisse/2026-10-23-baustein-gpu.md`
- Optional Create: `crates/driftless-gpu/src/tsdf.rs` nur wenn die Messung zeigt, dass TSDF > 20 % der Oberflächenzeit nach Task 4 ausmacht

- [ ] **Step 1: Tests:** UI-Test Schalter setzt `GpuPref` im Auftrag; Bericht zeigt Gerät.
- [ ] **Step 2–4:** FAIL → implementieren → PASS.
- [ ] **Step 5:** Ergebnisdokument (Zeiten CPU/GPU je Gerät, Abweichungen, Grafikspeicher, Grenzen) und README.
- [ ] **Step 6: Commit** `feat(app): Grafikkarte im Programm wählbar` – danach PR „Baustein GPU“.
