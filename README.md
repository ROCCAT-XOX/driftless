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
