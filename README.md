# Driftless

[![CI](https://github.com/ROCCAT-XOX/driftless/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/ROCCAT-XOX/driftless/actions/workflows/ci.yml)

**Anleitung auf Deutsch (user guide in German): [docs/anleitung.md](docs/anleitung.md)** – installing,
getting the data off the scanner, first steps in the app, which setting to choose, formats for other
programs, known limits, reporting a bug.

Open-source desktop software for processing handheld LiDAR scans — drift-corrected, cleanly colourised,
correctable by hand. Runs natively on **macOS (Apple Silicon)** and **Windows**.

> Status: **v0.1.0**, the first version for real jobs. What is in it and its known limits:
> [CHANGELOG.md](CHANGELOG.md) (German). The desktop program (`driftless-app`) and the command line
> (`driftless`) read the raw recordings of the 3DMakerPro JMK6, compute the trajectory with loop closure
> (correctable by hand: mark loops, switch off wrong automatic ones), a cleaned and coloured point cloud
> and a photo-like textured surface, merge several scans of one place into a model, and export to open
> formats (LAS/LAZ, E57, PLY, PCD, glb, OBJ, TUM, CSV). Projects can be renamed, moved, copied and
> cleaned up; nothing is deleted for good. Installers for macOS and Windows: see [Install](#install).
> Results on real scans (German): [export and installers](docs/ergebnisse/2026-10-27-baustein-3.md),
> [merging scans](docs/ergebnisse/2026-10-27-baustein-1.md),
> [quality round C and D](docs/ergebnisse/2026-10-26-qualitaet-c-d.md),
> [quality round E](docs/ergebnisse/2026-10-26-qualitaet-e.md),
> [quality round F](docs/ergebnisse/2026-10-26-qualitaet-f.md),
> [quality round A and B](docs/ergebnisse/2026-10-25-qualitaetsrunde.md),
> [new interface](docs/ergebnisse/2026-10-24-baustein-4.md),
> [graphics card](docs/ergebnisse/2026-10-23-baustein-gpu.md),
> [photo-like surface](docs/ergebnisse/2026-10-22-baustein-2.md),
> [milestone 5](docs/ergebnisse/2026-10-22-meilenstein-5.md),
> [milestone 4](docs/ergebnisse/2026-10-21-meilenstein-4.md),
> [milestone 3](docs/ergebnisse/2026-10-21-meilenstein-3.md),
> [milestone 2](docs/ergebnisse/2026-10-20-meilenstein-2.md).

## Why

The first target is the 3DMakerPro LiDAR scanner family (starting with device code `JMK6`: Vanjee WLR-722Z
LiDAR with built-in IMU, two fisheye cameras). The vendor software is Windows/NVIDIA-only and gives little
control over drift and colour quality. Driftless reads the scanner's raw recordings (ROS1 bags + calibration
JSON) and aims to produce better results on any machine.

## Goals

- Read raw recordings (ROS1 bags: LiDAR point clouds, IMU, camera JPEGs; calibration JSON)
- LiDAR-inertial odometry with automatic loop closure and global optimisation
- Show where the trajectory is uncertain; fix drift by hand (mark two places as "the same")
- Clean point clouds (dynamic objects, outliers, double walls) and consistent colours across cameras
- Export to LAS/LAZ, E57, PLY, PCD, TUM trajectory
- A modern, CloudCompare-inspired UI; every correction is a reversible step

Design: [docs/superpowers/specs/2026-10-20-driftless-design.md](docs/superpowers/specs/2026-10-20-driftless-design.md) (German)

## Install

From the [Releases page](https://github.com/ROCCAT-XOX/driftless/releases):
`Driftless-<version>-macos-arm64.dmg` for a Mac with Apple chip (macOS 14 or newer),
`Driftless-<version>-setup.exe` for Windows 10/11 (64 bit). Neither is signed, so the first start takes
one extra step (in detail, German: [guide](docs/anleitung.md#1-installieren)):

- **macOS:** drag `Driftless.app` onto `Applications` and start it. macOS blocks it: click „Fertig“, then
  System Settings → „Datenschutz & Sicherheit“ → „Dennoch öffnen“; again after every new version.
- **Windows:** run the setup. At „Der Computer wurde durch Windows geschützt“ click „Weitere Informationen“
  → „Trotzdem ausführen“; it installs for the current user, without admin rights, with a start menu entry.

On the Mac, `xattr -dr com.apple.quarantine /Applications/Driftless.app` in Terminal does the same and also
frees the command line. The command line is in both packages, but not on the `PATH`:
`/Applications/Driftless.app/Contents/MacOS/driftless`, or `driftless.exe` next to `driftless-app.exe` in
the Windows installation folder (`%LOCALAPPDATA%\Programs\Driftless` unless chosen otherwise).

### Build from source

Prerequisites:

- Rust 1.99 or newer (`rust-version` in `Cargo.toml`; `rust-toolchain.toml` picks the stable toolchain
  with rustfmt and clippy).
- macOS: the Xcode Command Line Tools (`xcode-select --install`).
- Windows: the Visual Studio Build Tools with the C++ workload (MSVC); some dependencies have C and C++
  parts, and the C runtime is linked statically (`.cargo/config.toml`).

```sh
cargo build --release      # target/release/driftless and target/release/driftless-app
```

The debug viewer `driftless view` (rerun) is neither in this build nor in the packages; build it with
`cargo build --release -p driftless-cli --features debug-viewer`. It also needs the rerun viewer itself:
`cargo install rerun-cli --version 0.38.1 --locked`.

## Commands

The binaries are `driftless` (command line) and `driftless-app` (desktop program, see below). Folder
arguments are a scan folder with `LIDAR_*.bag` (and optionally `IMAGE_*.bag`, `calibration/calib.json`).
Test data and how to set it up: [docs/testdaten.md](docs/testdaten.md).

| Command | What it does |
|---|---|
| `driftless-app [project-folder]` · `driftless-app --synthetic N` | The desktop program (see [Desktop program](#desktop-program)); opens the given project or the last one; `--synthetic N` shows a generated test cloud of N points |
| `driftless inspect <folder> [--json]` | Report on a scan: device, duration, topics with counts and rates, problems |
| `driftless view <folder> [--traj t.tum] [--slam <slam-out>] [--lidar-topic T] [--imu-topic T] [--every N] [--dry-run <file.rrd>]` | Debug viewer (rerun), only in a build with `cargo build --release -p driftless-cli --features debug-viewer` and with `rerun-cli` 0.38.1 installed (the packages leave it out and say so; the desktop program shows the clouds): point clouds, IMU, every N-th camera image, optional trajectory; `--dry-run` writes the recording to a file instead of opening the viewer; `--slam` adds quality-coloured trajectory, odometry, loops and submap anchors, and the timeline `ab_scan_beginn` (seconds since scan start, the times `driftless correct` takes) next to the bag timeline `stamp` |
| `driftless odom <folder> -o <dir> [--lidar-to-imu x y z qx qy qz qw]` | LiDAR-IMU odometry (Rust baseline); writes `traj.tum` and `map.las` |
| `driftless slam <folder> -o <dir> [--ueberschreiben] [--lidar-to-imu x y z qx qy qz qw] [--lidar-topic T] [--imu-topic T]` | SLAM with automatic loop closure and the manual loops of `<dir>/corrections.toml`; writes `traj.tum` (corrected), `traj_odom.tum` (before), `map.las`, `slam.json` (absolute `scan_start`, submaps, loop checks, graph) and `submaps.bin` (cache for `--reuse`). A `<dir>` that already holds a run (also an unfinished one) is refused unless `--ueberschreiben`: its trajectory, and a map made from it, would be replaced; `--reuse` recomputes it instead. A `<dir>` that holds the run of another scan (recorded at another time) is always refused, so one `<dir>` per scan, e.g. `<folder>/driftless` |
| `driftless slam <folder> -o <dir> --reuse` | Recomputes only the pose graph from `submaps.bin` with the current corrections (seconds instead of a full run) and rewrites all outputs; the previous `traj.tum` is kept as `traj.vorher.tum` (an older one is replaced) |
| `driftless correct <dir> list` · `loop <a_s> <b_s> [--note TEXT]` · `disable <nr>` · `enable <nr>` | Manual corrections in `<dir>/corrections.toml`: list them (with the automatic loops switched off in the app, `[[reject]]`), add a loop (the places at two times, seconds since scan start as in `slam.json`, are the same), switch entries off and on; entries are never removed |
| `driftless calib-check <scan> --slam <slam-out> -o <dir> [--images N] [--lidar-to-imu x y z qx qy qz qw] [--lidar-topic T]` | Checks the camera calibration (`calibration/calib.json`) by projecting the points of a SLAM run into the sharpest images, spread over the scan (`--images` per camera, default 12): the direction of the LiDAR↔camera matrix and the time offset of the images; writes `calib_check.json` and overlay images into `-o` |
| `driftless map <folder> --slam <slam-out> [--step dense\|clean\|color\|all] [--no-ghosts] [--no-outliers] [--no-through] [--no-smooth] [--highlights] [--no-vignetting] [--voxel M] [--downscale 1\|2\|4] [--gpu auto\|an\|aus] [--alle-dateien] [--neu]` | Map of a SLAM run, written into `<slam-out>`: dense cloud from the corrected `traj.tum`, cleaned of ghosts, points measured through windows and outliers (the removed points with their reason in `removed.las`; walls are projected onto their local plane), coloured from the camera images (`color.las`; points no camera saw are marked `user_data` 1; `removed.las` carries the reason in `user_data`: 1 ghost, 2 outlier, 3 window), and `map.json` (settings, statistics, runtime per step, peak memory). Of the clouds to look at, a run writes the one of its last step: `color.las`, or `clean.las` when the scan cannot be coloured (`dense.las` for `--step dense` alone). `--alle-dateien` writes every step's cloud as well: `dense.las` (unsmoothed), `clean.las` and `color.ply` (with a `colored` property, 1 = measured colour); `driftless export` writes the cloud of a map as PLY, LAZ, E57 or PCD as well. The requested step always runs; the step before it is taken from `dense.bin` / `clean.bin` (full-precision intermediate results; LAS is export only) if they were written for the same input and settings, otherwise rebuilt. Only a whole map (`--step all`, the default) that is current already – the same input and switches, every file there – is not computed again: it says „Karte ist aktuell“ (about a second instead of minutes); `--neu` computes it anyway. Outputs of earlier runs that no longer fit are never deleted, but warned about and listed under `stale` in `map.json`. Each cleaning and colour refinement can be switched off (`--no-through`, `--no-smooth`, `--no-vignetting` for the edge falloff of the cameras); `--highlights` additionally down-weights highlights but is experimental and off by default, because on a real scan it lowered floor reflections yet made the seams between images more visible. Quality figures (floor highlights, seam jump, edge-falloff residual, removed and smoothed points, wall thickness) are in `map.json` and on the console. Without `IMAGE_*.bag` or `calibration/calib.json`, `--step color` fails and `all` skips colouring with a warning. `--gpu` (default `auto`) colours on the graphics card (Vulkan, Metal, DX12) when a usable one is there, else on the CPU with a notice; `an` requires one and fails without, `aus` always uses the CPU. The result is equivalent, not byte-identical; `map.json` names the device, driver and, under `auto`, why the CPU was used (`steps.color.gpu`) |
| `driftless surface <folder> --slam <slam-out> [--map <map-folder>] [-o <dir>] [--voxel M] [--no-texture] [--gpu auto\|an\|aus] [--obj]` | Photo-like surface of a map (default `<map-folder>` = `<slam-out>`): a TSDF of the cleaned points (`clean.bin`) becomes a closed, simplified mesh, textured from the camera images (one image per face, texture pages with seam levelling); writes `oberflaeche.glb` (glTF, unlit, Y up, the texture pages inside) and `oberflaeche.json` (settings, face counts, mean and 95 % deviation to the points, seconds per phase, peak memory, the size of every file, and under `hinweise` short German hints for the user, e.g. why a large room stays open in places: from 15 % of the faces the coarse grid offered dropped for lack of a point within 1.5 cm, `max_fill_gap`) into `-o` (default `<map-folder>/oberflaeche`), the JSON last. `--obj` also writes `oberflaeche.obj` with `.mtl` and the page JPEGs (Z up); `driftless export` makes the OBJ from the glb as well. Needs a `clean.bin` that fits the trajectory and switches (run `driftless map` first), otherwise it stops with „Karte fehlt oder ist veraltet“. Without `IMAGE_*.bag` or `calibration/calib.json`, or with `--no-texture`, every face is gray (with a warning in the first case). `--voxel` (default 0.02 m) sets the fineness: smaller is finer and needs more memory and time (0.01 m: up to 9 GB and about 20 minutes on a large scan), and leaves more of a large or dark room open. Holes in walls and floors with a rim up to 2 m are closed flat; rims of open pieces, pipe ends and windows stay open. `--gpu` as for `map`: the views per face are found on the graphics card within 4 GB of graphics memory; a mesh too large for that runs in several passes (each decodes the images again, the result is the same as in one pass), and only if not even a pass of one face fits (all faces, images and depth buffers alone over the budget) does `auto` fall back to the CPU; `oberflaeche.json` reports it under `gpu`, the passes as `gpu_batches` |
| `driftless merge <scan>... -o <dir> [--slam <slam-out>]... [--map <map-folder>]... [--auch-unsichere] [--ohne-glaetten] [--voxel M] [--punktpaare <file>] [--ply]` | Merges several scans of one place into one model: each scan needs a SLAM run and a map with `clean.bin` (default `<scan>/driftless`; `--slam` and `--map` once per scan, in the same order). The first scan is the reference, its frame is the model frame. Every other scan is aligned automatically (features at corners and edges, RANSAC about the vertical, then ICP; a search over the turn if the features find nothing sure) to whichever placed scan fits best, so a chain room → corridor → room works. Each pair gets overlap, residual, wall residual and a word: „sicher“, „unsicher“ (e.g. too little overlap, edges that do not agree, residual over 5 mm, walls of one direction over 3 mm apart, a second pose almost as good) or „gescheitert“ (no pose, or one with almost no overlap or edges that almost all disagree); a scan placed surely against an unsure one is „unsicher“ too. Unsure and failed scans stay out unless `--auch-unsichere` (failed ones always). The colours of the scans are matched, the points thinned to `--voxel` (default 0.01 m, a coloured point wins) and the surfaces smoothed together (`--ohne-glaetten` keeps them as merged). Writes `modell.las` (scan number in `point_source_id`) and `modell.json` (per scan: its folders, absolute and relative to the model folder, pose, method, verdict, overlap, residual, wall residual, fit of the point pairs, colour gain; the warnings; runtime, peak memory) into `-o`, the JSON last, with `--ply` also `modell.ply` (property `scan`); a folder that already holds a model is refused. `--punktpaare` takes three or more points marked by hand in two scans (a `punktpaare.toml` as the desktop program writes it, map folders relative to the file); they replace the automatic alignment of those two scans and are refined by ICP; such a scan is „sicher“ when the pairs lie within 3 cm (RMS) of the refined pose and the residuals are within their limits, however little the scans overlap (a door, a short corridor). An entry the program cannot use (a hand edit), or a switched-on one that names no two of the maps, is skipped with a warning |
| `driftless surface --model <model-dir> [-o <dir>] [--voxel M] [--no-texture] [--gpu auto\|an\|aus] [--obj]` | The surface of a merged model: as `surface` above, but the mesh comes from the points of all scans in the model and the texture from the images of all of them, with one exposure levelling over all scans, so places one scan missed are filled. Default `-o` is `<model-dir>/oberflaeche`. Needs the maps the model was merged from unchanged, otherwise it stops with a message to merge again |
| `driftless export <source> -o <file> [--format NAME] [--ueberschreiben]` | Writes a map, model, surface or run, or a single cloud or mesh file, in an open format chosen by the extension of `-o` (or `--format`): LAS, LAZ, E57, PLY, PCD for points; glb, OBJ, PLY mesh (`--format ply-netz`) for a surface; TUM or CSV for a trajectory. An existing target is only replaced with `--ueberschreiben`, the source itself never. See [Export](#export) |
| `driftless view <folder> --cloud <file.las>` | Shows a point cloud (e.g. `color.las`) in the debug viewer (`--features debug-viewer`, see above), with its colours or gray by intensity, evenly thinned to at most 5 million points |
| `driftless export-glim <folder> -o <dir>` | Writes `imu.bin` and `lidar.bin` for the GLIM spike (`spikes/glim/`) |
| `driftless compare <a.tum> <b.tum> [--max-dt S] [--json]` | Trajectory yardstick: ATE after alignment, path length, start-to-end gap; poses of `a` and `b` are paired within `--max-dt` seconds (default 0.01; the comparisons with reference trajectories in the results use 0.05) |

Every result JSON (`slam.json`, `map.json`, `oberflaeche.json`, `modell.json`, `calib_check.json`) names
the program that made it under `programm`: version and commit, as `driftless --version` prints them. It is
informational only and part of no input id: a result of another version that fits stays current.

Ctrl+C stops `slam`, `map`, `surface`, `merge` and `export` cleanly: the step stops at its next check,
removes what it had begun (a step already writing its files finishes them) and the program exits with
code 130. A second Ctrl+C ends it at once; the `.tmp` files left then are removed by the next run of
the step, which also removes those of a crash or power loss. Each step writes its report (`slam.json`,
`map.json`, `oberflaeche.json`, `modell.json`) last and removes an old one before its first write, so
nothing half-written ever counts as done.

One folder per scan for its run, map and surface: `<scan>/driftless`, the default of `driftless merge`.
Typical correction loop:

```sh
driftless slam <scan> -o <scan>/driftless                 # full run, minutes at most
driftless correct <scan>/driftless loop 3.5 250.0         # the places at 3.5 s and 250.0 s are the same
driftless slam <scan> -o <scan>/driftless --reuse         # recompute from submaps.bin in seconds
```

Check the quality, the loops and the start-to-end gap between these steps in the desktop program: add the
scan to a project and the run with ☰ → „Lauf hinzufügen …“. The trajectory is drawn in the quality colours,
and „Stellen wählen“ under „Korrekturen“ fills in the two times by two clicks on it („Stelle A“, „Stelle B“,
seconds since scan start, as `correct loop` takes them); a loop can also be entered right there. In a
build with `--features debug-viewer`, `driftless view <scan> --slam <scan>/driftless` shows the same; read
the times there on the timeline `ab_scan_beginn` (`stamp` minus `scan_start` from `slam.json`), not on
`stamp`, the raw bag clock.

Then the map and the surface:

```sh
driftless map <scan> --slam <scan>/driftless              # dense, clean, colour → <scan>/driftless/color.las
driftless map <scan> --slam <scan>/driftless --step color --downscale 1   # recolour only, reuses clean.bin
driftless map <scan> --slam <scan>/driftless --no-smooth --no-through     # keep walls and window views as measured
driftless map <scan> --slam <scan>/driftless --voxel 0.02 # coarser cloud: about 1/3 of the memory, for long scans
driftless surface <scan> --slam <scan>/driftless          # needs clean.bin of a map run → <scan>/driftless/oberflaeche/oberflaeche.glb
driftless surface <scan> --slam <scan>/driftless --gpu aus   # the same on the CPU only (e.g. while the card is busy)
```

Several scans of one place, each with its run and map in `<scan>/driftless`:

```sh
driftless merge <scan1> <scan2> <scan3> -o modell         # the first is the reference
driftless surface --model modell                          # → modell/oberflaeche/oberflaeche.glb
```

Look at a cloud or surface in the desktop program: ☰ → „Datei öffnen …“, or drop `color.las` or
`oberflaeche.glb` on the window. With `--features debug-viewer`: `driftless view <scan> --cloud
<scan>/driftless/color.las`.

## Export

`driftless export <source> -o <file>` writes one file (an OBJ also its `.mtl` and page images). The
source is a folder or a file:

| Source | What is exported | Formats |
|---|---|---|
| map folder (`map.json`) | `color.las`, else `clean.las` | LAS, LAZ, E57, PLY, PCD |
| model folder (`modell.json`) | `modell.las`, one scan per member | LAS, LAZ, E57, PLY, PCD |
| surface folder (`oberflaeche.glb`) | the textured mesh | glb, OBJ, PLY mesh |
| run folder (`traj.tum`) | the corrected trajectory | TUM, CSV |
| a `.las .laz .e57 .ply .pcd .glb .obj` file | its points or mesh | as above for its kind |

```sh
driftless export <scan>/driftless -o buero.laz                  # map → LAZ
driftless export modell -o modell.e57                           # model → E57, one entry per scan
driftless export <scan>/driftless/oberflaeche -o raum.obj       # surface → raum.obj, raum.mtl, raum_seite_NN.jpg
driftless export <scan>/driftless/oberflaeche -o raum.ply --format ply-netz
driftless export <scan>/driftless -o laufbahn.csv --format csv  # a run folder that is also a map
driftless export buero.laz -o buero.e57 --ueberschreiben        # convert a file, replace an older result
```

- **Frame and units:** metres in the frame of the map or model, Z up. Only glb is Y up, as glTF
  requires: `(x, y, z)` is stored as `(x, z, −y)`.
- **LAS/LAZ:** LAS 1.4, point format 7 (time + colour), 6, 2 or 0 after the columns; coordinates at
  0.1 mm around the rounded middle of the cloud, colour 16 bit, the scan of a model in
  `point_source_id`, the mark for no measured colour and the removal reasons in `user_data`.
- **E57:** one `data3D` entry per scan of a model, with the scan's name and pose, xyz as `double`;
  colour 8 bit (what libE57-based tools expect), intensity 16 bit.
- **PLY/PCD:** binary; xyz as `float`, as `double` once a coordinate reaches 8192 m (UTM); colour
  8 bit, intensity 0–255, the scan in the property/field `scan`.
- **Meshes:** glb (one texture page per material, JPEG), OBJ with `.mtl` and the pages as
  `<name>_seite_NN.jpg`, or a PLY mesh with one colour per corner (no texture).
- **Trajectory:** TUM as `driftless slam` writes it, or CSV for German spreadsheets: the header
  `stempel;zeit_s;x;y;z;qx;qy;qz;qw`, `;` between the fields and a decimal comma; `stempel` is the
  stamp as in TUM, `zeit_s` the seconds since the first pose.
- Every file is written to `<name>.tmp` and renamed at the end: an error or a cancel leaves no half
  file and the old one unchanged. The same input gives the same bytes (no date in any header).
  Ctrl+C stops `driftless export` like the other steps (see above); a `<name>.tmp` left by a second
  Ctrl+C or a crash is removed by the next export to the same target.

In the desktop program: right click on a run, map, model or surface in the tree → „Exportieren …“, or
☰ → „Exportieren …“ for what the 3D view shows. The window „Exportieren“ offers the formats of the
source („Format“), „Ziel wählen …“ opens the save dialog, existing files are only replaced after
„Ersetzen“.

## Opening files

The desktop program opens point clouds (LAS, LAZ, E57, PLY, PCD) and surfaces (glb, OBJ) of other
programs too, also without a project: ☰ → „Datei öffnen …“ or drop the file on the window.

- An E57 with several scans keeps them apart: the colour mode „je Scan“ colours by scan.
- Colours and intensities of 8-bit files (some foreign LAS, PLY with `uchar`, PCD) are read as such.
- A PCD in an organised grid skips its empty (NaN) rows; an OBJ with quads, negative indices or a
  missing texture opens (a missing texture is gray, with „Textur fehlt“ in the log; a texture or
  `.mtl` outside the OBJ's folder, e.g. `../textures/x.png`, is not read: „… außerhalb des Ordners
  wird nicht gelesen“ – copy it next to the OBJ); a glb with node
  transforms or PNG textures opens.
- What cannot be read gives a German message in the status bar instead of a crash: a PCD saved
  „binary_compressed“ (save it as „binary“ in CloudCompare or PCL), a glb compressed with Draco, a
  broken file.

## Installers

Each pull request builds both installers as CI artifacts (`driftless-macos`, `driftless-windows`,
kept 14 days; `.github/workflows/package.yml`). A tag `v<version>` makes a **draft** release with
both files and `THIRD-PARTY-NOTICES.html` (`.github/workflows/release.yml`). Its text is
`packaging/release-notes.md` followed by the section of that version in [CHANGELOG.md](CHANGELOG.md)
(`packaging/release-body.sh`; the job stops if the section is missing). Publishing the draft is a manual
step; then the files are on the repository's Releases page (first start: see [Install](#install)).

- **macOS (Apple Silicon, macOS 14 or newer):** `Driftless-<version>-macos-arm64.dmg` holds `Driftless.app`
  with both programs and the licence notices (`Contents/Resources/THIRD-PARTY-NOTICES.html`), signed ad
  hoc, not notarised.
- **Windows (x64):** `Driftless-<version>-setup.exe` (Inno Setup) installs both programs (C runtime linked
  in), the icon, the licences and the notices for the current user without admin rights, with a start
  menu entry; not signed. Uninstall through „Apps“ in the Windows settings.
- „Über Driftless …“ in ☰ shows the version, the commit and the path of the command line, and opens the
  German guide („Anleitung“) and the licence notices („Lizenzen Dritter …“); `driftless --version` and
  `driftless-app --version` print the same version and commit. On Windows the desktop program has no
  console of its own: `driftless-app --version` shows its line only with the output redirected, e.g.
  `driftless-app --version | more`.

Build them yourself (version from `Cargo.toml`, result in `target/package/`):

```sh
packaging/macos/build-dmg.sh && packaging/macos/check-dmg.sh target/package/Driftless-*-macos-arm64.dmg
```

```powershell
# Windows, needs Inno Setup 6.3 or newer
pwsh packaging/windows/build-setup.ps1
pwsh packaging/windows/check-setup.ps1 target/package/Driftless-<version>-setup.exe   # installs silently into a temp folder and removes it again
```

## Desktop program

`cargo build --release -p driftless-app`, then `target/release/driftless-app` (Windows:
`driftless-app.exe`; every pull request also builds the installers as the CI artifacts `driftless-windows`
and `driftless-macos`, see `.github/workflows/package.yml`). The interface is German. It runs the same
steps as the command line (`inspect`, `slam`, `slam --reuse`, `correct`, `map`, `surface`, `merge`,
`surface --model`, `export`, `compare`) through the shared crate `driftless-steps`, and writes the same
files.

The window has a rail at the left edge, the field of the chosen step next to it, the 3D view and a
slim status bar:

- **Rail:** ☰ opens the project menu („Neues Projekt …“ in an empty folder, „Projekt öffnen …“,
  „Scan hinzufügen …“ for a scan folder as above, „Lauf hinzufügen …“ for a run made with the command
  line – it goes to the scan selected in „Daten“ (the scan, or one of its runs or maps) and is locked
  until one is selected –, „Datei öffnen …“ for any point cloud or surface file, „Exportieren …“ for what the view
  shows, „Protokoll-Ordner öffnen“ for the log file, „Über Driftless …“; see [Export](#export),
  [Opening files](#opening-files) and [Installers](#installers)). Below it the three steps, and ⚙ at the
  bottom. The dialogs open in `~/Driftless Projekte` (Windows `%USERPROFILE%\Driftless
  Projekte`, made when first needed), later in the folder each was last used in. A project is read in
  the background („Projekt wird gelesen …“), so a slow drive or share never stops the window; one in
  iCloud Drive (also `~/Documents` and `~/Desktop` with „Schreibtisch & Dokumente“), OneDrive or
  another cloud storage opens with a warning: large results get uploaded and moved off the disk, and
  load slowly again.
- **1 Daten:** under the project line the size of the project and the free space of its drive (below
  10 GB a warning, also in the Protokoll; a step that will need more than is free is warned about when it
  starts); every run, map, surface and model in the tree shows the size of its folder (measured in the
  background). A card per scan with its state (neu, wartet, rechnet, fertig, veraltet) and „Scan
  hinzufügen …“ below the cards. A right click on a scan's card: „Umbenennen …“ (only the name shown), „Ordner neu
  zuordnen …“ for a scan whose folder is missing (moved, drive not attached; only the same scan is
  taken: the same LiDAR bags by name and size, recorded at the times of its runs) and „Aus Projekt
  entfernen“ (only the entry: the raw data and the results under `scans/<id>` stay). A right click on a
  run, map, surface or model: „Löschen …“ asks with the size, what lies in the folder and which models
  use it, and moves the folder to the system's trash – never deleted for good; on Windows a network or
  removable drive, which has no trash, is refused. It is locked while a step works on it; while the
  view shows it, the question offers „Aus der Ansicht nehmen“ first. A run made with the command line
  and added by hand is only taken out of the project. What a job cut off by a crash, a force quit or
  a power cut left behind – a run, map, surface or model folder without its report, a stale
  `<name>.tmp` – is listed as „unvollständig“ (not as a run or map that waits) and „Aufräumen“ moves it
  to the trash; a folder a job still writes is marked `.in-arbeit` (touched every 10 s) and left
  alone, as is anything written to in the last two minutes. The tree is read again when the window
  gets the focus back (a scan drive attached again, results of another window). The precision
  Schnell · Genau · Maximal sets only the voxel size of the map (0.02 /
  0.01 / 0.005 m; Maximal needs over 4 GB of memory on large scans, another voxel shows „Eigene“). „Alle
  rechnen“ queues every new or stale scan and computes them one after the other; with a scan selected,
  „Rechnen“ computes only that one. The runs and maps of a scan unfold below its card; a click on a
  map shows its best cloud. „Zusammenführen …“ scrolls its field into view, start button included.
  „Erweitert“ at the bottom of the field holds the rest: the switches of `driftless map` with the voxel
  as a number, the single steps („Prüfen“, „SLAM rechnen“, „Neu rechnen (aus Teilkarten)“, „Karte
  rechnen“), the corrections (loops:
  „Stellen wählen“ and two clicks on the trajectory, „Schleife eintragen“, „Jetzt neu rechnen“ from
  `submaps.bin`; „Automatische Schleifen“ lists every loop SLAM accepted with a switch „verwenden“:
  off leaves a wrong one out of the pose graph – a `[[reject]]` entry in `corrections.toml` by the
  start times of its two submaps, which `driftless slam --reuse` respects too – and recomputes the
  run at once), before/after („Vergleichen“ of two states or of the odometry against the corrected
  trajectory, with the numbers of `driftless compare`; „Diesen Stand wiederherstellen“), and the layers
  „Teilkarten“ and „Verworfene Kandidaten“.
- **Models (in 1 Daten):** once two scans have a finished map, „Zusammenführen …“ merges the ticked
  scans into a new model (`modelle/modell-NNN`, the first ticked scan is the „Bezug“; „Auch unsichere
  übernehmen“ = `--auch-unsichere`). Under „Modelle“ each scan of a model shows its verdict, overlap
  and residual („Punktpaare“ in front if placed by hand; the fit of the pairs and the wall residual on hover); an unsure or failed one has „Von Hand ausrichten“: click three or more same places in
  the model and in the scan beside it, „Übernehmen und neu zusammenführen“ saves them in
  `modelle/punktpaare.toml` and merges again into a new model. Point pairs can be switched off and on,
  never deleted; one the merge cannot use is marked „nicht nutzbar“. A model's surface is computed in „2 Bilder“ like that of a map, from the images of all
  its scans; the colour mode „je Scan“ colours a model by scan. „Datei öffnen …“ and dropping a
  file show a point cloud (LAS, LAZ, E57, PLY, PCD; an E57 with several scans can be coloured „je
  Scan“) or a surface (glb, OBJ), also without a project. A variant that cannot be read (a PCD saved
  „binary_compressed“, a glb with Draco) gives a German message in the status bar.
- **2 Bilder:** a card per map with its surfaces below it (one that no longer fits its map is marked
  „veraltet – Karte oder Lauf neu gerechnet“); a click shows it (on a map card: its newest fitting
  surface, else its cloud). The fineness 1 · 1,5 · 2 cm applies to
  the next surface; „Alle Oberflächen rechnen“ queues every map without a current surface, „Oberfläche
  rechnen“ only the selected one, into `karte-NNN/oberflaeche-NNN`. „ohne Fotos“ (= `--no-texture`) is
  under „Erweitert“. Entering the step shows the surface; Daten and Planung show the points.
- **3 Planung:** the measurements of the tool „Messen“ (not saved yet) and „Messungen löschen“; floor
  plan, measures and DWG export follow in a later milestone.
- **3D view:** top left an info (what is shown, the points drawn and the frame time, what a click does
  now, the clicked point with ×, and „Schnitt aktiv: z … bis …“ while a height cut hides points); top
  right, with a surface loaded, Punkte · Fläche · beides. At the bottom the tools, exactly one active:
  „Drehen“ (left drag orbits), „Verschieben“ (left drag pans), „Punkt wählen“ (a click shows the time,
  colour and quality of a point), „Messen“ (two clicks: the distance in metres, drawn in the view),
  „Schnitt“ (z min / z max, „Decke ausblenden“, „Zurücksetzen“ in a bar above the tools); „Alles“ frames
  everything. With every tool the right or middle button (or Shift) pans and the wheel zooms; a click
  picks only with „Punkt wählen“ and „Messen“. Keys: `F` shows everything, `O` switches Fläche → Punkte →
  beides, Esc drops the first point of a measurement, W/A/S/D and Q/E fly in fly mode. The keys do
  nothing while a text field has the keyboard. While the trajectory, the submap anchors or the points
  (colour mode Güte) are drawn in the quality colours, a legend at the bottom left says what they
  mean: green „gut verortet“ (submap quality 0.25 and above), yellow „wenig Halt – prüfen“ (about
  0.05), red „kaum Halt – Lage unsicher“ (0.01 and below), on a logarithmic scale set on real scans.
- **⚙ (Einstellungen):** for the whole program, kept across starts: „Grafikkarte nutzen“ Auto · An · Aus
  for colouring the map and the views of a surface (as `--gpu auto|an|aus`), and how the view draws:
  the colour mode „Farbart“ (Farbe, Intensität, Güte, and „je Scan“ for the scans of a model or of an
  E57 with several scans), points as „Meter“ (default: round spots in world size, 1.5 × the map's
  voxel, at least the minimum size in pixels and at most 24 px, so that they close into an image up
  close) or „Pixel“ (every
  point the same size, with „rund“), the spot size „aus der Karte“ or by hand, the point budget
  (default 6 million points per frame), „Bedienung“ (Umkreisen or Fliegen) and the layers (Laufbahn,
  Odometrie, Schleifen, Teilkarten, Verworfene Kandidaten, Vergleich (A)).
- **Status bar:** „Protokoll“ with the log and the last message at the left; the running job, its
  progress and „Abbrechen“ at the right. It stays while switching steps. One step runs at a time;
  „Abbrechen“ leaves no half-written files and empties the queue. A surface that already fits its map
  and settings is not computed again, nor is a map („Karte ist aktuell“). Every computation of a run
  keeps a state under `verlauf/`.
- **Log file:** every start writes `driftless.log` in `~/Library/Logs/Driftless/` (macOS) or
  `%LOCALAPPDATA%\Driftless\Logs\` (Windows): a start line with version, commit, system, graphics
  adapter and project, then every Protokoll line and status message, warnings of the libraries (wgpu,
  eframe, winit), and after a crash its thread, place and backtrace. The four starts before stay as
  `driftless.1.log` … `driftless.4.log` (a file is also rotated at 10 MB). ☰ → „Protokoll-Ordner öffnen“
  shows the folder. Attach it to a bug report ([template](.github/ISSUE_TEMPLATE/fehler.md); no customer
  scans or names in a public issue). The command line writes its messages to stderr only.

Project folder:

```text
<project>/project.toml                           name and scans (paths relative if inside the project)
<project>/scans/<scan-id>/slam-001/              a SLAM run, the same files as `driftless slam -o`
                          slam-001/lauf.toml     scan and settings of the run
                          slam-001/verlauf/      states of the run
                          slam-001/karte-001/    a map of the run, one folder per set of switches
                          slam-001/karte-001/oberflaeche-001/
                                                 a surface of the map, one folder per set of settings
<project>/modelle/modell-001/                    scans merged into one model, the same files as `driftless merge -o`
                  modell-001/oberflaeche-001/    a surface of the model (`driftless surface --model`)
<project>/modelle/punktpaare.toml                point pairs marked by hand (only appended or switched off)
<project>/auswahl.toml                           the precision, fineness and „ohne Fotos“ chosen last in the project
```

Two windows, or the window and the command line, may work on one project: `project.toml`,
`corrections.toml`, `punktpaare.toml` and `auswahl.toml` are changed under a lock file
(`<name>.lock`, held for milliseconds, taken over after 30 s), and a change of `project.toml` is made
on what is on disk at that moment, so neither loses the other's change; while another holds the lock
the change is refused with a German message naming the holder.

Not yet: a cache for the levels of detail (they are rebuilt on every load), more than one step at a
time.

## Legal

Driftless is an independent project and not affiliated with 3DMakerPro. It reads open data formats
(ROS bags, JSON) for interoperability. It contains no code, files or assets from the vendor software.
Scan data is never committed to this repository.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option. Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion
in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
