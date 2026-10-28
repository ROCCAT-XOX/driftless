# Driftless

Open-source desktop software for processing handheld LiDAR scans — drift-corrected, cleanly colourised,
correctable by hand. Runs natively on **macOS (Apple Silicon)** and **Windows**.

> Status: early development. Milestone 5 done: a desktop program (`driftless-app`) with projects, the
> steps of the command line, a 3D view with levels of detail (the 14.96-million-point office scan loads in
> under 2 s), the trajectory coloured by quality, loops marked by two clicks and recomputed in seconds,
> before/after comparison and a history of states. Earlier: map in about 7 minutes with under 4 GB for a
> 9.7-minute scan, thinner walls, window views removed (milestone 4), dense, cleaned and coloured map
> (milestone 3), LiDAR-IMU odometry and SLAM with loop closure and manual corrections (milestone 2).
> Export to open formats (LAS/LAZ, E57, PLY, PCD, glb, OBJ, TUM, CSV), opening them in the desktop
> program, and installers for macOS and Windows are built (see [Export](#export) and
> [Installers](#installers)).
> Results on real scans (German): [export and installers](docs/ergebnisse/2026-10-27-baustein-3.md),
> [merging scans](docs/ergebnisse/2026-10-27-baustein-1.md),
> [new interface](docs/ergebnisse/2026-10-24-baustein-4.md),
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

## Commands

Build with `cargo build --release`; the binaries are `driftless` (command line) and `driftless-app`
(desktop program, see below). Folder arguments are a scan folder with
`LIDAR_*.bag` (and optionally `IMAGE_*.bag`, `calibration/calib.json`). Test data and how to set it up:
[docs/testdaten.md](docs/testdaten.md).

| Command | What it does |
|---|---|
| `driftless-app [project-folder]` · `driftless-app --synthetic N` | The desktop program (see [Desktop program](#desktop-program)); opens the given project or the last one; `--synthetic N` shows a generated test cloud of N points |
| `driftless inspect <folder> [--json]` | Report on a scan: device, duration, topics with counts and rates, problems |
| `driftless view <folder> [--traj t.tum] [--slam <slam-out>] [--lidar-topic T] [--imu-topic T]` | Debug viewer (rerun): point clouds, IMU, camera images, optional trajectory; `--slam` adds quality-coloured trajectory, odometry, loops and submap anchors, and the timeline `ab_scan_beginn` (seconds since scan start, the times `driftless correct` takes) next to the bag timeline `stamp` |
| `driftless odom <folder> -o <dir> [--lidar-to-imu x y z qx qy qz qw]` | LiDAR-IMU odometry (Rust baseline); writes `traj.tum` and `map.las` |
| `driftless slam <folder> -o <dir> [--lidar-to-imu x y z qx qy qz qw] [--lidar-topic T] [--imu-topic T]` | SLAM with automatic loop closure and the manual loops of `<dir>/corrections.toml`; writes `traj.tum` (corrected), `traj_odom.tum` (before), `map.las`, `slam.json` (absolute `scan_start`, submaps, loop checks, graph) and `submaps.bin` (cache for `--reuse`). A `<dir>` that holds the run of another scan (recorded at another time) is refused, so one `<dir>` per scan, e.g. `<folder>/driftless` |
| `driftless slam <folder> -o <dir> --reuse` | Recomputes only the pose graph from `submaps.bin` with the current corrections (seconds instead of a full run) and rewrites all outputs; the previous `traj.tum` is kept as `traj.vorher.tum` (an older one is replaced) |
| `driftless correct <dir> list` · `loop <a_s> <b_s> [--note TEXT]` · `disable <nr>` · `enable <nr>` | Manual corrections in `<dir>/corrections.toml`: list them, add a loop (the places at two times, seconds since scan start as in `slam.json`, are the same), switch entries off and on; entries are never removed |
| `driftless map <folder> --slam <slam-out> [--step dense\|clean\|color\|all] [--no-ghosts] [--no-outliers] [--no-through] [--no-smooth] [--highlights] [--no-vignetting] [--voxel M] [--downscale 1\|2\|4] [--gpu auto\|an\|aus] [--alle-dateien]` | Map of a SLAM run, written into `<slam-out>`: dense cloud from the corrected `traj.tum`, cleaned of ghosts, points measured through windows and outliers (the removed points with their reason in `removed.las`; walls are projected onto their local plane), coloured from the camera images (`color.las`; points no camera saw are marked `user_data` 1; `removed.las` carries the reason in `user_data`: 1 ghost, 2 outlier, 3 window), and `map.json` (settings, statistics, runtime per step, peak memory). Of the clouds to look at, a run writes the one of its last step: `color.las`, or `clean.las` when the scan cannot be coloured (`dense.las` for `--step dense` alone). `--alle-dateien` writes every step's cloud as well: `dense.las` (unsmoothed), `clean.las` and `color.ply` (with a `colored` property, 1 = measured colour); `driftless export` writes the cloud of a map as PLY, LAZ, E57 or PCD as well. The requested step always runs; the step before it is taken from `dense.bin` / `clean.bin` (full-precision intermediate results; LAS is export only) if they were written for the same input and settings, otherwise rebuilt. Outputs of earlier runs that no longer fit are never deleted, but warned about and listed under `stale` in `map.json`. Each cleaning and colour refinement can be switched off (`--no-through`, `--no-smooth`, `--no-vignetting` for the edge falloff of the cameras); `--highlights` additionally down-weights highlights but is experimental and off by default, because on a real scan it lowered floor reflections yet made the seams between images more visible. Quality figures (floor highlights, seam jump, edge-falloff residual, removed and smoothed points, wall thickness) are in `map.json` and on the console. Without `IMAGE_*.bag` or `calibration/calib.json`, `--step color` fails and `all` skips colouring with a warning. `--gpu` (default `auto`) colours on the graphics card (Vulkan, Metal, DX12) when a usable one is there, else on the CPU with a notice; `an` requires one and fails without, `aus` always uses the CPU. The result is equivalent, not byte-identical; `map.json` names the device, driver and, under `auto`, why the CPU was used (`steps.color.gpu`) |
| `driftless surface <folder> --slam <slam-out> [--map <map-folder>] [-o <dir>] [--voxel M] [--no-texture] [--gpu auto\|an\|aus] [--obj]` | Photo-like surface of a map (default `<map-folder>` = `<slam-out>`): a TSDF of the cleaned points (`clean.bin`) becomes a closed, simplified mesh, textured from the camera images (one image per face, texture pages with seam levelling); writes `oberflaeche.glb` (glTF, unlit, Y up, the texture pages inside) and `oberflaeche.json` (settings, face counts, mean and 95 % deviation to the points, seconds per phase, peak memory) into `-o` (default `<map-folder>/oberflaeche`), the JSON last. `--obj` also writes `oberflaeche.obj` with `.mtl` and the page JPEGs (Z up); `driftless export` makes the OBJ from the glb as well. Needs a `clean.bin` that fits the trajectory and switches (run `driftless map` first), otherwise it stops with „Karte fehlt oder ist veraltet“. Without `IMAGE_*.bag` or `calibration/calib.json`, or with `--no-texture`, every face is gray (with a warning in the first case). `--voxel` (default 0.02 m) sets the fineness: smaller is finer, needs more memory and time, and at 0.01 m leaves dark walls full of holes. Holes in walls and floors with a rim up to 2 m are closed flat; rims of open pieces, pipe ends and windows stay open. `--gpu` as for `map`: the views per face are found on the graphics card within 4 GB of graphics memory; a mesh too large for that runs in several passes (each decodes the images again, the result is the same as in one pass), and only if not even a pass of one face fits (all faces, images and depth buffers alone over the budget) does `auto` fall back to the CPU; `oberflaeche.json` reports it under `gpu`, the passes as `gpu_batches` |
| `driftless merge <scan>... -o <dir> [--slam <slam-out>]... [--map <map-folder>]... [--auch-unsichere] [--ohne-glaetten] [--voxel M] [--punktpaare <file>] [--ply]` | Merges several scans of one place into one model: each scan needs a SLAM run and a map with `clean.bin` (default `<scan>/driftless`; `--slam` and `--map` once per scan, in the same order). The first scan is the reference, its frame is the model frame. Every other scan is aligned automatically (features at corners and edges, RANSAC about the vertical, then ICP; a search over the turn if the features find nothing sure) to whichever placed scan fits best, so a chain room → corridor → room works. Each pair gets overlap, residual, wall residual and a word: „sicher“, „unsicher“ (e.g. too little overlap, edges that do not agree, residual over 5 mm, walls of one direction over 3 mm apart, a second pose almost as good) or „gescheitert“ (no pose, or one with almost no overlap or edges that almost all disagree); a scan placed surely against an unsure one is „unsicher“ too. Unsure and failed scans stay out unless `--auch-unsichere` (failed ones always). The colours of the scans are matched, the points thinned to `--voxel` (default 0.01 m, a coloured point wins) and the surfaces smoothed together (`--ohne-glaetten` keeps them as merged). Writes `modell.las` (scan number in `point_source_id`) and `modell.json` (per scan: its folders, absolute and relative to the model folder, pose, method, verdict, overlap, residual, wall residual, fit of the point pairs, colour gain; the warnings; runtime, peak memory) into `-o`, the JSON last, with `--ply` also `modell.ply` (property `scan`); a folder that already holds a model is refused. `--punktpaare` takes three or more points marked by hand in two scans (a `punktpaare.toml` as the desktop program writes it, map folders relative to the file); they replace the automatic alignment of those two scans and are refined by ICP; such a scan is „sicher“ when the pairs lie within 3 cm (RMS) of the refined pose and the residuals are within their limits, however little the scans overlap (a door, a short corridor). An entry the program cannot use (a hand edit), or a switched-on one that names no two of the maps, is skipped with a warning |
| `driftless surface --model <model-dir> [-o <dir>] [--voxel M] [--no-texture] [--gpu auto\|an\|aus] [--obj]` | The surface of a merged model: as `surface` above, but the mesh comes from the points of all scans in the model and the texture from the images of all of them, with one exposure levelling over all scans, so places one scan missed are filled. Default `-o` is `<model-dir>/oberflaeche`. Needs the maps the model was merged from unchanged, otherwise it stops with a message to merge again |
| `driftless export <source> -o <file> [--format NAME] [--ueberschreiben]` | Writes a map, model, surface or run, or a single cloud or mesh file, in an open format chosen by the extension of `-o` (or `--format`): LAS, LAZ, E57, PLY, PCD for points; glb, OBJ, PLY mesh (`--format ply-netz`) for a surface; TUM or CSV for a trajectory. An existing target is only replaced with `--ueberschreiben`, the source itself never. See [Export](#export) |
| `driftless view <folder> --cloud <file.las>` | Shows a point cloud (e.g. `color.las`) in the viewer, with its colours or gray by intensity, evenly thinned to at most 5 million points |
| `driftless export-glim <folder> -o <dir>` | Writes `imu.bin` and `lidar.bin` for the GLIM spike (`spikes/glim/`) |
| `driftless compare <a.tum> <b.tum> [--json]` | Trajectory yardstick: ATE after alignment, path length, start-to-end gap |

Typical correction loop:

```sh
driftless slam <scan> -o out                 # full run, minutes at most
driftless view <scan> --slam out             # check quality, loops, start-to-end gap
driftless correct out loop 3.5 250.0         # the places at 3.5 s and 250.0 s are the same
driftless slam <scan> -o out --reuse         # recompute from submaps.bin in seconds
```

Then the map:

```sh
driftless map <scan> --slam out              # dense, clean, colour → out/color.las
driftless map <scan> --slam out --step color --downscale 1   # recolour only, reuses clean.bin
driftless map <scan> --slam out --no-smooth --no-through   # keep walls and window views as measured
driftless map <scan> --slam out --voxel 0.02 # coarser cloud: about 1/3 of the memory, for long scans
driftless surface <scan> --slam out           # needs clean.bin of a map run → out/oberflaeche/oberflaeche.glb
driftless surface <scan> --slam out --gpu aus # the same on the CPU only (e.g. while the card is busy)
driftless view <scan> --cloud out/color.las
```

Read the times for `correct loop` in the viewer on the timeline `ab_scan_beginn`, not on `stamp`:
`stamp` is the raw bag clock, `ab_scan_beginn` is `stamp` minus `scan_start` from `slam.json`.

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
driftless export out/karte -o buero.laz                    # map → LAZ
driftless export modelle/modell-001 -o modell.e57           # model → E57, one entry per scan
driftless export out/karte/oberflaeche -o raum.obj          # surface → raum.obj, raum.mtl, raum_seite_NN.jpg
driftless export out/karte/oberflaeche -o raum.ply --format ply-netz
driftless export out -o laufbahn.csv --format csv           # a run folder that is also a map
driftless export buero.laz -o buero.e57 --ueberschreiben    # convert a file, replace an older result
```

- **Frame and units:** metres in the frame of the map or model, Z up. Only glb is Y up, as glTF
  requires: `(x, y, z)` is stored as `(x, z, −y)`.
- **LAS/LAZ:** LAS 1.4, point format 7 (time + colour), 6, 2 or 0 after the columns; coordinates at
  0.1 mm around the rounded middle of the cloud, colour 16 bit, the scan of a model in
  `point_source_id`, „no measured colour“ and removal reasons in `user_data`.
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
  Stopping `driftless export` with Ctrl-C also leaves the old file unchanged, but its `<name>.tmp`
  stays behind; delete it by hand.

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
both files (`.github/workflows/release.yml`); once it is published they are under the repository's
Releases.

- **macOS (Apple Silicon):** `Driftless-<version>-macos-arm64.dmg`. Open it and drag `Driftless.app`
  onto the `Applications` link. The app is signed ad hoc, not notarised, so macOS blocks the first
  start. After that first try, open System Settings → „Datenschutz & Sicherheit“, scroll down and
  click „Dennoch öffnen“ next to the note about Driftless (once). Or, in Terminal:
  `xattr -dr com.apple.quarantine /Applications/Driftless.app`. The command line is inside the app:
  `/Applications/Driftless.app/Contents/MacOS/driftless`.
- **Windows (x64):** `Driftless-<version>-setup.exe`. It installs for the current user without admin
  rights (to `%LOCALAPPDATA%\Programs\Driftless` unless chosen otherwise) with a start menu entry.
  It is not signed: SmartScreen shows „Der Computer wurde durch Windows geschützt“, click „Weitere
  Informationen“ → „Trotzdem ausführen“. The command line `driftless.exe` is in the installation
  folder next to `driftless-app.exe`; uninstall through „Apps“ in the Windows settings.
- „Über Driftless …“ in ☰ shows the version, the commit and the path of the command line;
  `driftless --version` and `driftless-app --version` print the same. On Windows the desktop
  program has no console of its own: `driftless-app --version` shows its line only with the output
  redirected, e.g. `driftless-app --version | more`.

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
steps as the command line (`inspect`, `slam`, `slam --reuse`, `correct`, `map`, `surface`, `compare`)
through the shared crate `driftless-steps`, and writes the same files.

The window has a rail at the left edge, the field of the chosen step next to it, the 3D view and a
slim status bar:

- **Rail:** ☰ opens the project menu („Neues Projekt …“ in an empty folder, „Projekt öffnen …“,
  „Scan hinzufügen …“ for a scan folder as above, „Lauf hinzufügen …“ for a run made with the command
  line, „Datei öffnen …“ for any point cloud or surface file, „Exportieren …“ for what the view shows,
  „Über Driftless …“; see [Export](#export) and [Opening files](#opening-files)). Below it the three steps, and
  ⚙ at the bottom.
- **1 Daten:** a card per scan with its state (neu, wartet, rechnet, fertig ✓, veraltet) and „+ Scan
  hinzufügen“; the precision Schnell · Genau · Maximal sets only the voxel size of the map (0.02 /
  0.01 / 0.005 m; Maximal needs over 4 GB of memory on large scans, another voxel shows „Eigene“). „Alle
  rechnen“ queues every new or stale scan and computes them one after the other; with a scan selected,
  „Rechnen“ computes only that one. The runs and maps of a scan unfold below its card. „▸ Erweitert“
  holds the rest: the switches of `driftless map` with the voxel as a number, the single steps
  („Prüfen“, „SLAM rechnen“, „Neu rechnen (aus Teilkarten)“, „Karte rechnen“), the corrections (loops:
  „Stellen wählen“ and two clicks on the trajectory, „Schleife eintragen“, „Jetzt neu rechnen“ from
  `submaps.bin`), before/after („Vergleichen“ of two states or of the odometry against the corrected
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
  „veraltet – Karte oder Lauf neu gerechnet“); a click shows it. The fineness 1 · 1,5 · 2 cm applies to
  the next surface; „Alle Oberflächen rechnen“ queues every map without a current surface, „Oberfläche
  rechnen“ only the selected one, into `karte-NNN/oberflaeche-NNN`. „ohne Fotos“ (= `--no-texture`) is
  under „▸ Erweitert“. Entering the step shows the surface; Daten and Planung show the points.
- **3 Planung:** the measurements of the tool „Messen“ (not saved yet) and „Messungen löschen“; floor
  plan, measures and DWG export follow in a later milestone.
- **3D view:** top left an info (what is shown, the points drawn and the frame time, what a click does
  now, the clicked point with ×, and „Schnitt aktiv: z … bis … m“ while a height cut hides points); top
  right, with a surface loaded, Punkte · Fläche · beides. At the bottom the tools, exactly one active:
  „Drehen“ (left drag orbits), „Verschieben“ (left drag pans), „Punkt wählen“ (a click shows the time,
  colour and quality of a point), „Messen“ (two clicks: the distance in metres, drawn in the view),
  „Schnitt“ (z min / z max, „Decke ausblenden“, „Zurücksetzen“ in a bar above the tools); „Alles“ frames
  everything. With every tool the right or middle button (or Shift) pans and the wheel zooms; a click
  picks only with „Punkt wählen“ and „Messen“. Keys: `F` shows everything, `O` switches Fläche → Punkte →
  beides, Esc drops the first point of a measurement, W/A/S/D and Q/E fly in fly mode. The keys do
  nothing while a text field has the keyboard.
- **⚙ (Einstellungen):** how the view draws, for the whole program: colour mode (Farbe, Intensität,
  Güte), points as „Meter“ (default: round spots in world size, 1.5 × the map's voxel, at least the
  minimum size in pixels and at most 24 px, so that they close into an image up close) or „Pixel“ (every
  point the same size, with „rund“), the spot size „aus der Karte“ or by hand, the point budget
  (default 6 million points per frame), „Bedienung“ (Umkreisen or Fliegen) and the layers (Laufbahn,
  Odometrie, Schleifen, Teilkarten, Verworfene Kandidaten, Vergleich (A)).
- **Status bar:** „▸ Protokoll“ with the log and the last message at the left; the running job, its
  progress and „Abbrechen“ at the right. It stays while switching steps. One step runs at a time;
  „Abbrechen“ leaves no half-written files and empties the queue. A surface that already fits its map
  and settings is not computed again. Every computation of a run keeps a state under `verlauf/`.

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
```

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
