# Qualitätsrunde A + B Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Honest quality figures in every run (step A) and a 1 cm surface without holes (step B).

**Architecture:** A adds a pass-offset/thickness measure on the cloud *before* cleaning (new module
`driftless-map/src/quality.rs`, reported under `steps.clean.dense_quality` in `map.json`) and the
surface accuracy against `dense.bin` (`accuracy_dense` in `oberflaeche.json`); the template
evaluation stays Python in `tools/qualitaet/`. B fuses a second TSDF at twice the voxel edge and lets
marching cubes take a fine voxel's value from the coarse grid (trilinear) where the fine one has
fewer than `min_weight` votes.

**Tech Stack:** Rust workspace (driftless-map, driftless-steps), rayon, kiddo, nalgebra; Python 3 +
numpy for `tools/qualitaet/`.

**Spec:** `docs/superpowers/specs/2026-10-25-qualitaetsrunde-design.md` (steps A and B; C–F get their
own plans after the measurements).

## Global Constraints

- No scan data, images from scans, customer names or absolute paths under `/Users/…` in the repo.
- Comments and identifiers in English, user-facing texts (errors, progress, JSON is neutral) German,
  like the surrounding code.
- `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo fmt`
  green; tests must not need scan data.
- A changes no result: `CLEAN_VERSION` (3) and the dense/colour versions stay; only the report gains
  keys. B raises `SURFACE_VERSION` 9 → 10.
- Old `map.json`/`oberflaeche.json` without the new keys must still load everywhere (the app reads
  them).
- Heavy runs only on VM 101 via `~/.local/bin/driftless-vm`, half load (6 threads, `nice 10`).

## Review Focus

1. A map whose points all have the same time (or `t` empty/NaN) → pass measure reports no
   multi-pass patches (`null`/0), never panics or divides by zero. Test in Task 1.
2. `dense.bin` missing, stale or of another input → `accuracy_dense` is `null`, the surface still
   succeeds. Test in Task 2.
3. A fine voxel at the edge of the coarse grid (some of the 8 coarse neighbours unobserved) → no value
   (no made-up surface), as today. Test in Task 4.
4. Where the fine grid has enough votes everywhere, the mesh is identical to the single-grid mesh
   (no quality loss on well-observed walls). Test in Task 4.
5. `min_weight = 1` or `coarse_fallback = false` → behaves exactly as before B. Test in Task 4.

---

### Task 1: Pass offset before cleaning (`map.json`)

**Files:**
- Create: `crates/driftless-map/src/quality.rs`
- Modify: `crates/driftless-map/src/lib.rs` (module + re-export), `crates/driftless-map/src/clean.rs`
  (`CleanStats` gains `dense_quality`, computed in `clean_with` on the input cloud before anything is
  removed), `crates/driftless-steps/src/map.rs` (`clean_json`)
- Test: unit tests in `quality.rs`; `crates/driftless-steps/tests/map.rs` (key present)

**Interfaces:**
- Produces:
  ```rust
  pub struct PassQuality {
      pub patches: usize,              // planar 30 cm cells evaluated
      pub patches_multi_pass: usize,   // of them with ≥ 2 passes
      pub local_rms_mm: Option<f64>,   // median over patches of the plane-fit RMS
      pub thickness95_mm: Option<f64>, // median over patches of p97.5 − p2.5 of the residuals
      pub within_pass_rms_mm: Option<f64>,
      pub pass_offset_median_mm: Option<f64>,
      pub pass_offset_p90_mm: Option<f64>,
  }
  pub fn pass_quality(xyz: &[[f32; 3]], t: &[f64]) -> PassQuality
  ```
  `CleanStats.dense_quality: PassQuality`; JSON `steps.clean.dense_quality` with exactly these keys
  (`null` for `None`).

Definition (from the campaign, `tools/qualitaet/q-analyze.py` `refine`, but on 3D cells instead of
room planes): cells of `PATCH = 0.30` m (3D voxel key); a cell counts with ≥ `MIN_PATCH_POINTS = 300`
points whose PCA plane is planar (smallest eigenvalue / middle < `0.05`, the planarity used by
`SmoothConfig`); residuals `r` = signed distance to that plane. Passes: sort the cell's points by `t`,
split where consecutive times differ by more than `PASS_GAP_S = 5.0`; a pass counts with ≥ 40 points;
its level = median of its `r`; the cell's offset = max level − min level (needs ≥ 2 passes);
within-pass = RMS of `r − level` over all counted passes. Medians/p90 over cells. Deterministic
(sort cells by key; rayon only over cells).

- [ ] **Step 1: Write the failing tests** in `quality.rs`:
  - `two_passes_8mm_apart`: a 2 × 2 m wall (points every 5 mm, Gaussian-free jitter ±1 mm from a fixed
    pattern), first half of the points at t = 0…10 s on plane x = 0, second half at t = 100…110 s on
    plane x = 0.008 → `pass_offset_median_mm` within 8.0 ± 0.5, `patches_multi_pass > 0`,
    `within_pass_rms_mm < 1.5`.
  - `one_pass_has_no_offset`: same wall, all t within 0…10 s → `pass_offset_median_mm == None`,
    `patches_multi_pass == 0`, `local_rms_mm.is_some()`.
  - `constant_or_bad_times_do_not_panic`: `t` all 0.0, and `t` containing NaN → no panic, no offset;
    empty input → `patches == 0`, all `None`.
  - `curved_cells_are_skipped`: points on a sphere of radius 0.2 m → `patches == 0`.
- [ ] **Step 2:** `cargo test -p driftless-map quality` → FAIL (module missing).
- [ ] **Step 3:** Implement `pass_quality` and wire `dense_quality` into `CleanStats` (computed even
  when every clean part is switched off) and `clean_json`.
- [ ] **Step 4:** Add to the existing map step test in `crates/driftless-steps/tests/map.rs` an
  assertion that `map.json["steps"]["clean"]["dense_quality"]["patches"]` is a number.
- [ ] **Step 5:** `cargo test -p driftless-map -p driftless-steps` → PASS; clippy, fmt.
- [ ] **Step 6: Commit** `feat(map): Versatz zwischen Durchgängen vor dem Glätten in map.json`

### Task 2: Surface accuracy against the unsmoothed cloud (`oberflaeche.json`)

**Files:**
- Modify: `crates/driftless-map/src/surface/mod.rs` (`SurfaceStats.accuracy_dense:
  Option<AccuracySplit>`; `build_mesh` gets the dense points through `SurfaceInput`),
  `crates/driftless-steps/src/surface.rs` (load `dense.bin` of the same map with the id `map.rs` uses
  for it; on any failure `None` plus one `progress.info` line; JSON `accuracy_dense` = same shape as
  `accuracy` or `null`)
- Test: `crates/driftless-map/src/surface/mod.rs` tests, `crates/driftless-steps/tests/surface.rs`

**Interfaces:**
- Consumes: `accuracy(mesh, xyz)` (existing, `surface/mod.rs`).
- Produces: `SurfaceInput { xyz, sensor_of_point, sensors, dense_xyz: Option<&[[f32; 3]]> }`;
  `SurfaceStats.accuracy_dense`.

- [ ] **Step 1: Failing tests:**
  - map: `accuracy_dense_measures_against_given_points` — build a mesh from the existing slab test
    input with `dense_xyz = Some(points shifted 5 mm along the normal)` → `accuracy_dense.unwrap()
    .walls.median_mm` within 5 ± 1.5 and `accuracy.walls.median_mm < 2`; with `None` →
    `accuracy_dense == None`.
  - steps: `surface_without_dense_bin_still_succeeds` — run the surface step test fixture after
    deleting `dense.bin` → Ok, `oberflaeche.json["accuracy_dense"].is_null()`; with it present →
    `["accuracy_dense"]["walls"]["median_mm"]` is a number.
- [ ] **Step 2:** run → FAIL.
- [ ] **Step 3:** Implement. Free the dense points right after `build_mesh` (as the clean points are).
  `accuracy_dense` is not part of the input id; no version change.
- [ ] **Step 4:** `cargo test -p driftless-map -p driftless-steps` → PASS; fix every other
  `SurfaceInput { … }` literal in the workspace (`dense_xyz: None`).
- [ ] **Step 5: Commit** `feat(surface): Genauigkeit auch gegen die ungeglättete Wolke`

### Task 3: Template evaluation tool `tools/qualitaet/`

**Files:**
- Create: `tools/qualitaet/README.md`, `tools/qualitaet/q_analyze.py`, `tools/qualitaet/q_views.py`,
  `tools/qualitaet/q_table.py`, `tools/qualitaet/run_vorlagen.sh`
- Source: `.superpowers/quality/skripte/` (local, git-ignored: `q-analyze.py`, `q-views.py`,
  `q-table.py`, `q-run.sh`, `q-images.sh`)

**Interfaces:**
- Produces: `run_vorlagen.sh <label> [--map-voxel 0.005] [--surface-voxel 0.01]` run on VM 101 inside
  the synced repo: for the four scans (paths from env `DQ_SCANS`, default the four VM paths of the
  spec) runs `driftless slam`, `map --gpu an`, `surface --gpu an` into
  `~/td/quality/<label>/<scan>/`, then `q_analyze.py` and `q_table.py` → `~/td/quality/<label>/
  tabelle.md` with one column per scan and the rows of the spec's goal table plus
  `dense_quality`/`accuracy_dense` from the JSON reports.

- [ ] **Step 1:** Copy the scripts, rename to snake_case, replace every hard-coded path, scan id
  list and user name by arguments/env; keep the metric code unchanged. No data, no images.
- [ ] **Step 2:** `README.md` (German): what each script measures (definitions as in the spec), how
  to run on VM 101 (`driftless-vm sync`, then `driftless-vm run tools/qualitaet/run_vorlagen.sh …`),
  half load (`RAYON_NUM_THREADS=6 nice -n 10`), one run at a time.
- [ ] **Step 3:** `python3 -m py_compile tools/qualitaet/*.py` and `bash -n
  tools/qualitaet/run_vorlagen.sh` → no output. `grep -rn "/Users/\|yannick" tools/qualitaet` →
  nothing.
- [ ] **Step 4: Commit** `chore(tools): Auswertung der Vorlagen-Scans`

**Controller after Task 3 (not a subagent task):** on VM 101 run `run_vorlagen.sh b0` twice for
133554 (run-to-run noise) and once for all four; check Rust `dense_quality` against the Python
`pass_offset_spread_mm` (≤ 1 mm, else rule per spec); write the baseline table into
`docs/ergebnisse/2026-10-25-qualitaetsrunde.md`.

### Task 4: Two-level TSDF (surface without holes at 1 cm)

**Files:**
- Modify: `crates/driftless-map/src/surface/tsdf.rs`, `crates/driftless-map/src/surface/extract.rs`,
  `crates/driftless-map/src/surface/mod.rs` (`SurfaceConfig.coarse_fallback: bool`, default `true`;
  `SurfaceStats.cells_from_coarse: usize`; `build_mesh` integrates both grids),
  `crates/driftless-steps/src/surface.rs` (`SURFACE_VERSION` 10, `settings_json` +
  `"coarse_fallback"`, `stats_json` + `"cells_from_coarse"`), every place that reads settings back
  (`SurfaceSettings::from_report` in the app: missing key = `true`)
- Test: `tsdf.rs` / `extract.rs` / `mod.rs` unit tests

**Interfaces:**
- Produces:
  ```rust
  /// A fine TSDF with an optional coarse one (twice the voxel edge, same `truncation_voxels`).
  pub struct FusedSdf { fine: Tsdf, coarse: Option<Tsdf> }
  impl FusedSdf {
      pub fn integrate(input: &SurfaceInput, cfg: &SurfaceConfig, progress: &Progress)
          -> Result<FusedSdf, MapError>;
      /// Fine value if the fine voxel has ≥ min_weight votes; else the coarse grid trilinearly
      /// at the fine voxel centre if all 8 coarse neighbours are observed; else None.
      pub fn sample(&self, key: [i32; 3]) -> Option<f32>;
      pub fn voxel(&self) -> f64;                       // the fine edge
      pub fn blocks(&self) -> impl Iterator<Item = [i32; 3]> + '_; // fine blocks
      pub fn voxel_bytes(&self) -> usize;               // both grids
  }
  pub fn extract(sdf: &FusedSdf) -> (Mesh, usize /* cells using ≥ 1 coarse corner */);
  ```
  `Tsdf::sample` keeps its signature. Fine blocks already cover every region a ray crossed, so
  iterating fine blocks finds the coarse-filled cells too.

Trilinear lookup: fine voxel `k` has centre `c = (k + 0.5)·v`; in coarse voxel units `g = c / (2v) −
0.5`; base `b = floor(g)`, weights `g − b`; the 8 coarse voxels `b + {0,1}³` must all be `Some`.
Because every corner value is a pure function of its key, neighbouring cells agree and the mesh stays
crack-free.

- [ ] **Step 1: Failing tests:**
  - `sparse_wall_closes_with_coarse_fallback` (mod.rs): a flat 2 × 2 m wall at 1 cm voxel sampled so
    that each fine voxel gets on average ~1 vote (points every 14 mm, one sensor per point 2 m in
    front) → with `coarse_fallback: false` the mesh covers < 50 % of the wall area; with `true` it
    covers > 95 % and `accuracy.walls.median_mm < 3`.
  - `dense_wall_is_unchanged` (Review Focus 4): a wall with ≥ 4 votes per fine voxel → the mesh with
    `coarse_fallback: true` equals (positions and indices) the mesh with `false`, and
    `cells_from_coarse == 0`.
  - `coarse_edge_gives_no_value` (Review Focus 3, tsdf.rs): a fine voxel with weight 1 whose coarse
    neighbourhood has one unobserved voxel → `FusedSdf::sample == None`.
  - `min_weight_one_ignores_coarse` (Review Focus 5): `min_weight = 1` → identical mesh with and
    without fallback.
  - `fallback_mesh_has_no_cracks` (extract.rs): on the sparse wall, every edge of the mesh interior
    (more than 5 cm from the wall's rim) is shared by exactly two faces.
- [ ] **Step 2:** `cargo test -p driftless-map surface` → FAIL.
- [ ] **Step 3:** Implement `FusedSdf`, switch `extract` and `build_mesh` to it; `tsdf_blocks` /
  `tsdf_bytes` count both grids; new progress phase text „Oberfläche: grobes Gitter“.
- [ ] **Step 4:** `SURFACE_VERSION` 10; settings/stats JSON keys. Old reports without
  `coarse_fallback` are stale by version but must still parse in the app (missing key = `true`).
- [ ] **Step 5:** `cargo test --workspace` → PASS (the existing 2 cm test comment about „1 cm left
  45 % open“ stays true for `coarse_fallback: false`; update the comment if the default test changes);
  clippy, fmt.
- [ ] **Step 6: Commit** `feat(surface): zweistufiges TSDF schließt schwach getroffene Flächen`

**Controller after Task 4:** on VM 101 `run_vorlagen.sh b1` for all four scans; compare against b0
(Z1, Z2, Z3, Z5 with the spec's tolerances); if Z1 is missed, rule per spec (vote spreading as the
alternative) and write the outcome into the results doc.
