# Driftless

Open-source desktop software for processing handheld LiDAR scans — drift-corrected, cleanly colourised,
correctable by hand. Runs natively on **macOS (Apple Silicon)** and **Windows**.

> Status: early development. Milestone 5 done: a desktop program (`driftless-app`) with projects, the
> steps of the command line, a 3D view with levels of detail (the 15.6-million-point office scan loads in
> under 2 s), the trajectory coloured by quality, loops marked by two clicks and recomputed in seconds,
> before/after comparison and a history of states. Earlier: map in about 7 minutes with under 4 GB for a
> 9.7-minute scan, thinner walls, window views removed (milestone 4), dense, cleaned and coloured map
> (milestone 3), LiDAR-IMU odometry and SLAM with loop closure and manual corrections (milestone 2).
> Results on real scans (German): [milestone 5](docs/ergebnisse/2026-10-22-meilenstein-5.md),
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
driftless view <scan> --cloud out/color.las
```

Read the times for `correct loop` in the viewer on the timeline `ab_scan_beginn`, not on `stamp`:
`stamp` is the raw bag clock, `ab_scan_beginn` is `stamp` minus `scan_start` from `slam.json`.

## Desktop program

`cargo build --release -p driftless-app`, then `target/release/driftless-app` (Windows:
`driftless-app.exe`; every pull request also builds it as the CI artifact `driftless-app-windows`). The
interface is German. It runs the same steps as the command line (`inspect`, `slam`, `slam --reuse`,
`correct`, `map`, `compare`) through the shared crate `driftless-steps`, and writes the same files.

- **Project:** „Projekt → Neues Projekt …“ in an empty folder, then „Scan hinzufügen …“ (a scan folder
  as above). Runs made with the command line can be added with „Lauf hinzufügen …“.
- **Steps** (middle panel, for the selected scan, run or map): „Prüfen“, „SLAM rechnen“, „Neu rechnen
  (aus Teilkarten)“, „Karte rechnen“ with the switches of `driftless map`. One step runs at a time, with
  progress in the status bar; „Abbrechen“ leaves no half-written files.
- **3D view:** a map or cloud from the tree; colour, intensity or quality of the submap; orbit (left
  drag, wheel, F = show all) or fly (W/A/S/D, Q/E); click a point for its time, colour and quality; a
  height cut („Decke ausblenden“) to look into the rooms from above. A point budget (default 6 million)
  bounds the points drawn per frame; the view shows the points drawn and the frame time.
- **Corrections:** with a run selected, the trajectory is drawn by quality with its loops; „Stellen
  wählen“ and two clicks on the trajectory mark a loop, „Schleife eintragen“ writes it to
  `corrections.toml`, „Jetzt neu rechnen“ recomputes the run from `submaps.bin`. Entries can be switched
  off and on.
- **Before/after and history:** every computation of a run keeps a state under `verlauf/`; „Vergleichen“
  shows two states or the odometry against the corrected trajectory, with the numbers of
  `driftless compare`, and „Diesen Stand wiederherstellen“ brings back the switches of an earlier state.

Project folder:

```text
<project>/project.toml                           name and scans (paths relative if inside the project)
<project>/scans/<scan-id>/slam-001/              a SLAM run, the same files as `driftless slam -o`
                          slam-001/lauf.toml     scan and settings of the run
                          slam-001/verlauf/      states of the run
                          slam-001/karte-001/    a map of the run, one folder per set of switches
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
