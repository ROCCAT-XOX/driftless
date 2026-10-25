# Driftless

Open-source desktop software for processing handheld LiDAR scans — drift-corrected, cleanly colourised,
correctable by hand. Runs natively on **macOS (Apple Silicon)** and **Windows**.

> Status: early development. Milestone 5 done: a desktop program (`driftless-app`) with projects, the
> steps of the command line, a 3D view with levels of detail (the 14.96-million-point office scan loads in
> under 2 s), the trajectory coloured by quality, loops marked by two clicks and recomputed in seconds,
> before/after comparison and a history of states. Earlier: map in about 7 minutes with under 4 GB for a
> 9.7-minute scan, thinner walls, window views removed (milestone 4), dense, cleaned and coloured map
> (milestone 3), LiDAR-IMU odometry and SLAM with loop closure and manual corrections (milestone 2).
> Results on real scans (German): [new interface](docs/ergebnisse/2026-10-24-baustein-4.md),
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
| `driftless slam <folder> -o <dir> [--lidar-to-imu x y z qx qy qz qw] [--lidar-topic T] [--imu-topic T]` | SLAM with automatic loop closure and the manual loops of `<dir>/corrections.toml`; writes `traj.tum` (corrected), `traj_odom.tum` (before), `map.las`, `slam.json` (absolute `scan_start`, submaps, loop checks, graph) and `submaps.bin` (cache for `--reuse`) |
| `driftless slam <folder> -o <dir> --reuse` | Recomputes only the pose graph from `submaps.bin` with the current corrections (seconds instead of a full run) and rewrites all outputs; the previous `traj.tum` is kept as `traj.vorher.tum` (an older one is replaced) |
| `driftless correct <dir> list` · `loop <a_s> <b_s> [--note TEXT]` · `disable <nr>` · `enable <nr>` | Manual corrections in `<dir>/corrections.toml`: list them, add a loop (the places at two times, seconds since scan start as in `slam.json`, are the same), switch entries off and on; entries are never removed |
| `driftless map <folder> --slam <slam-out> [--step dense\|clean\|color\|all] [--no-ghosts] [--no-outliers] [--no-through] [--no-smooth] [--highlights] [--no-vignetting] [--voxel M] [--downscale 1\|2\|4]` | Map of a SLAM run, written into `<slam-out>`: dense cloud from the corrected `traj.tum` (`dense.las`), cleaned of ghosts, points measured through windows and outliers (`clean.las`, the removed points with their reason in `removed.las`; walls are projected onto their local plane, `dense.las` stays unsmoothed), coloured from the camera images (`color.las`, `color.ply`; points no camera saw are marked `user_data` 1 in the LAS; `color.ply` has a `colored` property, 1 = measured colour; `removed.las` carries the reason in `user_data`: 1 ghost, 2 outlier, 3 window), and `map.json` (settings, statistics, runtime per step, peak memory). The requested step always runs; the step before it is taken from `dense.bin` / `clean.bin` (full-precision intermediate results; LAS is export only) if they were written for the same input and settings, otherwise rebuilt. Outputs of earlier runs that no longer fit are never deleted, but warned about and listed under `stale` in `map.json`. Each cleaning and colour refinement can be switched off (`--no-through`, `--no-smooth`, `--no-vignetting` for the edge falloff of the cameras); `--highlights` additionally down-weights highlights but is experimental and off by default, because on a real scan it lowered floor reflections yet made the seams between images more visible. Quality figures (floor highlights, seam jump, edge-falloff residual, removed and smoothed points, wall thickness) are in `map.json` and on the console. Without `IMAGE_*.bag` or `calibration/calib.json`, `--step color` fails and `all` skips colouring with a warning |
| `driftless surface <folder> --slam <slam-out> [--map <map-folder>] [-o <dir>] [--voxel M] [--no-texture]` | Photo-like surface of a map (default `<map-folder>` = `<slam-out>`): a TSDF of the cleaned points (`clean.bin`) becomes a closed, simplified mesh, textured from the camera images (one image per face, texture pages with seam levelling); writes `oberflaeche.glb` (glTF, unlit, Y up), `oberflaeche.obj` with `.mtl` and the page JPEGs (Z up) and `oberflaeche.json` (settings, face counts, mean and 95 % deviation to the points, seconds per phase, peak memory) into `-o` (default `<map-folder>/oberflaeche`), the JSON last. Needs a `clean.bin` that fits the trajectory and switches (run `driftless map` first), otherwise it stops with „Karte fehlt oder ist veraltet“. Without `IMAGE_*.bag` or `calibration/calib.json`, or with `--no-texture`, every face is gray (with a warning in the first case). `--voxel` (default 0.02 m) sets the fineness: smaller is finer, needs more memory and time, and at 0.01 m leaves dark walls full of holes. Holes in walls and floors with a rim up to 2 m are closed flat; rims of open pieces, pipe ends and windows stay open |
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
driftless view <scan> --cloud out/color.las
```

Read the times for `correct loop` in the viewer on the timeline `ab_scan_beginn`, not on `stamp`:
`stamp` is the raw bag clock, `ab_scan_beginn` is `stamp` minus `scan_start` from `slam.json`.

## Desktop program

`cargo build --release -p driftless-app`, then `target/release/driftless-app` (Windows:
`driftless-app.exe`; every pull request also builds it as the CI artifact `driftless-app-windows`). The
interface is German. It runs the same steps as the command line (`inspect`, `slam`, `slam --reuse`,
`correct`, `map`, `surface`, `compare`) through the shared crate `driftless-steps`, and writes the same
files.

The window has a rail at the left edge, the field of the chosen step next to it, the 3D view and a
slim status bar:

- **Rail:** ☰ opens the project menu („Neues Projekt …“ in an empty folder, „Projekt öffnen …“,
  „Scan hinzufügen …“ for a scan folder as above, „Lauf hinzufügen …“ for a run made with the command
  line, „Oberfläche öffnen …“ for any `oberflaeche.glb`). Below it the three steps, and ⚙ at the bottom.
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
```

Not yet: export (milestone 6), a cache for the levels of detail (they are rebuilt on every load), more
than one step at a time.

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
