# Driftless

Open-source desktop software for processing handheld LiDAR scans — drift-corrected, cleanly colourised,
correctable by hand. Runs natively on **macOS (Apple Silicon)** and **Windows**.

> Status: early development. Nothing usable yet.

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

Build with `cargo build --release`; the binary is `driftless`. Folder arguments are a scan folder with
`LIDAR_*.bag` (and optionally `IMAGE_*.bag`, `calibration/calib.json`). Test data and how to set it up:
[docs/testdaten.md](docs/testdaten.md).

| Command | What it does |
|---|---|
| `driftless inspect <folder> [--json]` | Report on a scan: device, duration, topics with counts and rates, problems |
| `driftless view <folder> [--traj t.tum] [--slam <slam-out>] [--lidar-topic T] [--imu-topic T]` | Debug viewer (rerun): point clouds, IMU, camera images, optional trajectory; `--slam` adds quality-coloured trajectory, odometry, loops and submap anchors |
| `driftless odom <folder> -o <dir> [--lidar-to-imu x y z qx qy qz qw]` | LiDAR-IMU odometry (Rust baseline); writes `traj.tum` and `map.las` |
| `driftless slam <folder> -o <dir> [--lidar-to-imu x y z qx qy qz qw] [--lidar-topic T] [--imu-topic T]` | SLAM with automatic loop closure and the manual loops of `<dir>/corrections.toml`; writes `traj.tum` (corrected), `traj_odom.tum` (before), `map.las`, `slam.json` (submaps, loop checks, graph) and `submaps.bin` (cache for `--reuse`) |
| `driftless slam <folder> -o <dir> --reuse` | Recomputes only the pose graph from `submaps.bin` with the current corrections (seconds instead of a full run) and rewrites all outputs; the previous `traj.tum` is kept as `traj.vorher.tum` (an older one is replaced) |
| `driftless correct <dir> list` · `loop <a_s> <b_s> [--note TEXT]` · `disable <nr>` · `enable <nr>` | Manual corrections in `<dir>/corrections.toml`: list them, add a loop (the places at two times, seconds since scan start as in `slam.json`, are the same), switch entries off and on; entries are never removed |
| `driftless export-glim <folder> -o <dir>` | Writes `imu.bin` and `lidar.bin` for the GLIM spike (`spikes/glim/`) |
| `driftless compare <a.tum> <b.tum> [--json]` | Trajectory yardstick: ATE after alignment, path length, start-to-end gap |

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
