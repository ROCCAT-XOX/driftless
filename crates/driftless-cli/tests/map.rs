//! `driftless map` on a synthetic scan: the static room of the `slam` tests, plus camera
//! images rendered from the room and a calibration.

mod common;

use assert_cmd::Command;
use common::{room_bag, time, tmp};
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::calib::load_calibration;
use driftless_io::camera::Camera;
use driftless_io::msg::encode::encode_compressed_image;
use driftless_io::msg::CompressedImage;
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use predicates::prelude::*;
use std::fs;
use std::path::Path;

/// The room of `common::room_scan`.
const LO: [f64; 3] = [-4.0, -3.0, -1.5];
const HI: [f64; 3] = [6.0, 5.0, 2.0];
/// Colour of the +x wall, plain (the camera looks at it); the others carry a checkerboard.
const WALL_X: [u8; 3] = [200, 200, 60];
const IMAGE_W: u16 = 160;
const IMAGE_H: u16 = 120;
const IMAGES: usize = 5;

/// Camera pose in the LiDAR frame: looking along LiDAR +x, tilted 10° down, offset a bit.
fn lidar_cam() -> Isometry3<f64> {
    let base = UnitQuaternion::from_basis_unchecked(&[
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ]);
    let tilt = UnitQuaternion::from_euler_angles(10f64.to_radians(), 0.0, 0.0);
    Isometry3::from_parts(Translation3::new(0.05, -0.1, 0.03), base * tilt)
}

/// Colour where the ray from `o` along `d` leaves the room.
fn cast(o: &Vector3<f64>, d: &Vector3<f64>) -> [u8; 3] {
    let (mut t, mut axis) = (f64::INFINITY, 0);
    for i in 0..3 {
        if d[i].abs() > 1e-12 {
            let bound = if d[i] > 0.0 { HI[i] } else { LO[i] };
            let ti = (bound - o[i]) / d[i];
            if ti > 0.0 && ti < t {
                (t, axis) = (ti, i);
            }
        }
    }
    if axis == 0 && d[0] > 0.0 {
        return WALL_X;
    }
    let p = o + d * t;
    let checker = (0..3)
        .filter(|&i| i != axis)
        .map(|i| (p[i] * 2.0).floor() as i64)
        .sum::<i64>()
        .rem_euclid(2);
    let base = [[60, 60, 200], [200, 60, 200], [60, 200, 200]][axis];
    base.map(|c: u8| if checker == 0 { c } else { c / 2 })
}

fn write_calib(dir: &Path) {
    let m = lidar_cam().inverse().to_homogeneous();
    let rows: Vec<Vec<f64>> = (0..4)
        .map(|i| (0..4).map(|j| m[(i, j)]).collect())
        .collect();
    let json = serde_json::json!({
        "camera_info": {"left": {
            "K": [55.0, 0.0, 59.5, 0.0, 55.0, 79.5, 0.0, 0.0, 1.0],
            "camera_model": "fisheyeKB",
            "coeff": [0.0, 0.0, 0.0, 0.0],
            "image_width": IMAGE_W,
            "image_height": IMAGE_H
        }},
        "out_put": {"left": {"transform_matrix": rows}}
    });
    fs::create_dir_all(dir.join("calibration")).unwrap();
    fs::write(dir.join("calibration/calib.json"), json.to_string()).unwrap();
}

/// Stored image of the static device (at the origin) through the calibration of `dir` as
/// Driftless reads it.
fn render_from(dir: &Path) -> Vec<u8> {
    let calib = load_calibration(&dir.join("calibration/calib.json")).unwrap();
    let c = &calib.cameras["left"];
    let cam = Camera::from_calib("left", c).unwrap();
    let world_cam = c.lidar_to_cam.unwrap().inverse();
    let o = world_cam.translation.vector;
    let mut rgb = Vec::new();
    for y in 0..cam.height {
        for x in 0..cam.width {
            let px = cam
                .unproject([f64::from(x), f64::from(y)])
                .map_or([0; 3], |d| cast(&o, &(world_cam.rotation * d)));
            rgb.extend(px);
        }
    }
    rgb
}

/// Scan folder: static room (LiDAR + IMU), `IMAGES` camera images, calibration.
fn scan_with_images(dir: &Path) {
    room_bag(dir, true);
    write_calib(dir);
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 95)
        .encode(
            &render_from(dir),
            IMAGE_W,
            IMAGE_H,
            jpeg_encoder::ColorType::Rgb,
        )
        .unwrap();
    let mut w = BagWriter::new(Compression::None);
    let ic = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    for k in 0..IMAGES {
        let s = 1000.3 + 0.2 * k as f64;
        let img = CompressedImage {
            stamp: s,
            format: "jpeg".into(),
            data: jpeg.clone(),
        };
        w.write_message(ic, time(s), &encode_compressed_image(&img));
    }
    fs::write(dir.join("IMAGE_0.bag"), w.finish()).unwrap();
}

fn slam(dir: &Path, out: &Path) {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("slam")
        .arg(dir)
        .arg("-o")
        .arg(out)
        .assert()
        .success();
}

fn map(dir: &Path, out: &Path, args: &[&str]) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("map")
        .arg(dir)
        .arg("--slam")
        .arg(out)
        .args(args)
        .assert()
}

fn map_json(out: &Path) -> serde_json::Value {
    serde_json::from_str(&fs::read_to_string(out.join("map.json")).unwrap()).unwrap()
}

fn las_points(path: &Path) -> Vec<las::Point> {
    las::Reader::from_path(path)
        .unwrap()
        .read_all()
        .unwrap()
        .points()
        .map(Result::unwrap)
        .collect()
}

#[test]
fn map_all_writes_outputs() {
    let dir = tmp("map-all");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--step", "all"])
        .success()
        .stdout(predicate::str::contains("Dicht:"))
        .stdout(predicate::str::contains("Bereinigt:"))
        .stdout(predicate::str::contains("Geister"))
        .stdout(predicate::str::contains("Durchblick"))
        .stdout(predicate::str::contains("Wanddicke"))
        .stdout(predicate::str::contains("Eingefärbt:"))
        .stdout(predicate::str::contains("Glanzansichten"))
        .stdout(predicate::str::contains(format!("Bilder {IMAGES} genutzt")))
        .stdout(predicate::str::contains("Laufzeit"));
    for f in [
        "dense.las",
        "dense.bin",
        "clean.las",
        "removed.las",
        "clean.bin",
        "color.las",
        "color.ply",
        "map.json",
    ] {
        assert!(out.join(f).is_file(), "{f} missing");
    }

    let json = map_json(&out);
    let steps = &json["steps"];
    let dense = steps["dense"]["stats"]["points"].as_u64().unwrap();
    let kept = steps["clean"]["stats"]["kept"].as_u64().unwrap();
    let ghosts = steps["clean"]["stats"]["ghosts"].as_u64().unwrap();
    let through = steps["clean"]["stats"]["through"].as_u64().unwrap();
    let outliers = steps["clean"]["stats"]["outliers"].as_u64().unwrap();
    assert!(dense > 1000, "{dense}");
    assert_eq!(kept + ghosts + through + outliers, dense);
    let color = &steps["color"]["stats"];
    assert_eq!(color["images_used"], IMAGES);
    assert!(color["points_colored"].as_u64().unwrap() > 100, "{color}");
    assert_eq!(color["gains"]["per_camera"]["left"]["images"], IMAGES);
    for s in ["dense", "clean", "color"] {
        assert!(steps[s]["seconds"].is_number(), "{s}");
        assert_eq!(steps[s]["reused"], false, "{s}");
    }
    assert_eq!(json["settings"]["clean"]["ghosts"], true);
    assert_eq!(json["settings"]["clean"]["through"], true);
    assert_eq!(json["settings"]["color"]["downscale"], 2);
    assert_eq!(json["settings"]["clean"]["smooth"]["enabled"], true);
    assert_eq!(json["settings"]["color"]["highlights"], false);
    assert_eq!(json["settings"]["color"]["exposure"]["vignetting"], true);
    for k in ["view_gap", "highlight_ratio", "highlight_abs", "rim_theta"] {
        assert!(json["settings"]["color"][k].is_number(), "{k}");
    }
    let smooth = &steps["clean"]["stats"]["smooth"];
    for k in ["moved", "mean_shift", "thickness_before", "thickness_after"] {
        assert!(smooth[k].is_number(), "smooth.{k}");
    }
    for k in [
        "views_rejected",
        "points_with_rejected",
        "views_rim_excluded",
        "floor_highlight_share_before",
        "floor_highlight_share_after",
        "seam_jump_before",
        "seam_jump_after",
        "vignette_residual_before",
        "vignette_residual_after",
        "sharpness_brightness_rank_corr",
    ] {
        assert!(color[k].is_number(), "color.{k}");
    }
    assert!(color["gains"]["clamped_images"].is_number());
    assert!(color["vignetting"]["left"].is_array());
    if cfg!(any(unix, windows)) {
        assert!(json["peak_rss_bytes"].as_u64().unwrap() > 1_000_000);
    }

    assert_eq!(las_points(&out.join("dense.las")).len() as u64, dense);
    let removed = las_points(&out.join("removed.las")).len() as u64;
    assert_eq!(removed, ghosts + through + outliers);
    let points = las_points(&out.join("color.las"));
    assert_eq!(points.len() as u64, kept);
    // `user_data` marks the points without a measured colour.
    let uncolored = points.iter().filter(|p| p.user_data == 1).count() as u64;
    assert_eq!(uncolored, color["points_uncolored"].as_u64().unwrap());
    assert!(uncolored > 0 && uncolored < kept, "{uncolored} of {kept}");
    // The plain +x wall in front of the camera has its colour.
    let wall: Vec<las::Color> = points
        .iter()
        .filter(|p| p.x > 5.9 && p.y.abs() < 1.5 && p.z.abs() < 0.8)
        .map(|p| p.color.unwrap())
        .collect();
    assert!(wall.len() > 20, "{}", wall.len());
    let near = wall
        .iter()
        .filter(|c| {
            [c.red, c.green, c.blue]
                .iter()
                .zip(WALL_X)
                .all(|(&v, w)| (i32::from(v / 257) - i32::from(w)).abs() <= 12)
        })
        .count();
    assert!(near * 10 >= wall.len() * 9, "{near} of {}", wall.len());
}

#[test]
fn map_without_images_skips_color() {
    let dir = tmp("map-noimages");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &[])
        .success()
        .stdout(predicate::str::contains("Bereinigt:"))
        .stderr(predicate::str::contains("Warnung"))
        .stderr(predicate::str::contains("keine Kamerabilder"));
    assert!(out.join("clean.las").is_file());
    assert!(!out.join("color.las").exists());
    assert!(!out.join("color.ply").exists());
    let json = map_json(&out);
    assert!(json["steps"]["color"]["skipped"]
        .as_str()
        .unwrap()
        .contains("keine Kamerabilder"));
    assert_eq!(json["stale"], serde_json::json!([]));

    map(&dir, &out, &["--step", "color"])
        .failure()
        .stderr(predicate::str::contains("keine Kamerabilder"));

    // Images, but no calibration.
    let dir = tmp("map-nocalib");
    scan_with_images(&dir);
    fs::remove_dir_all(dir.join("calibration")).unwrap();
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--step", "color"])
        .failure()
        .stderr(predicate::str::contains("keine Kalibrierung"));
    assert!(!out.join("dense.las").exists(), "fails before any work");
}

#[test]
fn map_skipped_color_marks_old_color_stale() {
    let dir = tmp("map-stale-color");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &[]).success();
    let color = fs::read(out.join("color.las")).unwrap();
    fs::remove_dir_all(dir.join("calibration")).unwrap();
    map(&dir, &out, &[])
        .success()
        .stderr(predicate::str::contains(
            "nicht eingefärbt, keine Kalibrierung",
        ))
        .stderr(predicate::str::contains(
            "color.las, color.ply stammen aus einem früheren Lauf und passen nicht mehr zu \
             clean.las",
        ))
        .stderr(predicate::str::contains("fehlen").not());
    let json = map_json(&out);
    assert!(json["steps"]["color"]["skipped"].is_string());
    assert_eq!(json["stale"], serde_json::json!(["color.las", "color.ply"]));
    assert_eq!(
        fs::read(out.join("color.las")).unwrap(),
        color,
        "not deleted"
    );
}

#[test]
fn map_step_color_reuses_clean() {
    let dir = tmp("map-reuse");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--step", "all"]).success();
    let color = fs::read(out.join("color.las")).unwrap();
    let first = map_json(&out);

    map(&dir, &out, &["--step", "color"])
        .success()
        .stdout(predicate::str::contains("clean.bin wiederverwendet"))
        .stdout(predicate::str::contains("dense.bin").not())
        .stdout(predicate::str::contains("Eingefärbt:"));
    assert_eq!(fs::read(out.join("color.las")).unwrap(), color);
    let json = map_json(&out);
    assert_eq!(json["steps"]["clean"]["reused"], true);
    assert_eq!(
        json["steps"]["clean"]["stats"],
        first["steps"]["clean"]["stats"]
    );
    assert_eq!(json["steps"]["dense"]["reused"], true);
    assert_eq!(json["steps"]["color"]["reused"], false);

    // Other cleaning settings: clean.bin does not fit, dense.bin does.
    map(&dir, &out, &["--step", "color", "--no-outliers"])
        .success()
        .stdout(predicate::str::contains(
            "clean.bin gehört zu einer anderen Eingabe",
        ))
        .stdout(predicate::str::contains("dense.bin wiederverwendet"))
        .stdout(predicate::str::contains("Bereinigt:"));
    assert_eq!(map_json(&out)["settings"]["clean"]["outliers"], false);

    // Another trajectory: the dense cloud is rebuilt.
    let color = fs::read(out.join("color.las")).unwrap();
    let traj = fs::read_to_string(out.join("traj.tum")).unwrap();
    let shifted: String = traj
        .lines()
        .map(|l| {
            if l.starts_with('#') {
                return format!("{l}\n");
            }
            let mut v: Vec<String> = l.split(' ').map(str::to_string).collect();
            v[1] = format!("{:.9}", v[1].parse::<f64>().unwrap() + 0.01);
            v.join(" ") + "\n"
        })
        .collect();
    fs::write(out.join("traj.tum"), shifted).unwrap();
    map(&dir, &out, &["--step", "clean"])
        .success()
        .stdout(predicate::str::contains(
            "dense.bin gehört zu einer anderen Eingabe",
        ))
        .stdout(predicate::str::contains("Dicht:"))
        .stderr(predicate::str::contains(
            "Warnung: color.las, color.ply stammen aus einem früheren Lauf und passen nicht \
             mehr zu clean.las",
        ));
    let json = map_json(&out);
    assert_eq!(json["steps"]["dense"]["reused"], false);
    assert!(json["steps"]["color"].is_null(), "colour no longer fits");
    assert_eq!(json["stale"], serde_json::json!(["color.las", "color.ply"]));
    // Nothing is deleted.
    assert_eq!(fs::read(out.join("color.las")).unwrap(), color);

    // After `--step dense` with another voxel size the cleaned cloud is stale too.
    map(&dir, &out, &["--step", "dense", "--voxel", "0.05"])
        .success()
        .stderr(predicate::str::contains(
            "clean.las, removed.las, clean.bin stammen aus einem früheren Lauf und passen \
             nicht mehr zu dense.las",
        ));
    assert_eq!(
        map_json(&out)["stale"],
        serde_json::json!([
            "clean.las",
            "removed.las",
            "clean.bin",
            "color.las",
            "color.ply"
        ])
    );
}

#[test]
fn map_no_smooth_keeps_dense_coordinates() {
    let dir = tmp("map-nosmooth");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(
        &dir,
        &out,
        &["--step", "clean", "--no-smooth", "--no-through"],
    )
    .success()
    .stdout(predicate::str::contains("geglättet").not());
    let json = map_json(&out);
    assert_eq!(json["settings"]["clean"]["smooth"]["enabled"], false);
    assert_eq!(json["settings"]["clean"]["through"], false);
    let stats = &json["steps"]["clean"]["stats"];
    assert_eq!(stats["smooth"]["moved"], 0);
    assert_eq!(stats["through"], 0);
    // Every kept point is a dense point, unchanged (both LAS files round the same way).
    let key = |p: &las::Point| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
    let dense: std::collections::HashSet<_> =
        las_points(&out.join("dense.las")).iter().map(key).collect();
    let clean = las_points(&out.join("clean.las"));
    assert!(clean.len() > 1000);
    assert!(clean.iter().all(|p| dense.contains(&key(p))));

    // The switches change the step ids: the smoothed result is not reused.
    map(&dir, &out, &["--step", "clean"])
        .success()
        .stdout(predicate::str::contains("Wanddicke"));
    assert_eq!(
        map_json(&out)["settings"]["clean"]["smooth"]["enabled"],
        true
    );
}

#[test]
fn map_switches_are_recorded() {
    let dir = tmp("map-switches");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--highlights", "--no-vignetting"]).success();
    let s = map_json(&out)["settings"].clone();
    assert_eq!(s["color"]["highlights"], true);
    assert_eq!(s["color"]["exposure"]["vignetting"], false);
}

#[test]
fn m3_folder_is_rebuilt_not_misread() {
    let dir = tmp("map-m3");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &[]).success();
    let clean = fs::read(out.join("clean.las")).unwrap();

    // Make the folder look like Meilenstein 3: the step files carry another input id and
    // map.json has no entry that fits.
    for f in ["dense.bin", "clean.bin"] {
        // Layout: magic (4), version (4), hash of the input id (8).
        let mut bytes = fs::read(out.join(f)).unwrap();
        bytes[8..16].copy_from_slice(&driftless_map::content_hash(b"meilenstein-3").to_le_bytes());
        fs::write(out.join(f), bytes).unwrap();
    }
    let mut json = map_json(&out);
    for step in ["dense", "clean", "color"] {
        json["steps"][step]["input_id"] = "0000000000000000".into();
    }
    fs::write(out.join("map.json"), json.to_string()).unwrap();

    map(&dir, &out, &["--step", "clean"])
        .success()
        .stdout(predicate::str::contains(
            "dense.bin gehört zu einer anderen Eingabe",
        ))
        .stdout(predicate::str::contains("Dicht:"))
        .stdout(predicate::str::contains("Bereinigt:"))
        // The colour files are from the "old" run and no longer fit: warned, not deleted.
        .stderr(predicate::str::contains(
            "color.las, color.ply stammen aus einem früheren Lauf",
        ));
    assert_eq!(fs::read(out.join("clean.las")).unwrap(), clean);
    let json = map_json(&out);
    assert_eq!(json["steps"]["dense"]["reused"], false);
    assert_eq!(json["steps"]["clean"]["reused"], false);
    assert!(json["steps"]["color"].is_null());
    assert_eq!(json["stale"], serde_json::json!(["color.las", "color.ply"]));
}

#[test]
fn map_is_deterministic() {
    let dir = tmp("map-determinism");
    scan_with_images(&dir);
    let (a, b) = (dir.join("a"), dir.join("b"));
    for out in [&a, &b] {
        slam(&dir, out);
        map(&dir, out, &[]).success();
    }
    for f in [
        "dense.las",
        "dense.bin",
        "clean.las",
        "removed.las",
        "clean.bin",
        "color.las",
        "color.ply",
    ] {
        assert_eq!(
            fs::read(a.join(f)).unwrap(),
            fs::read(b.join(f)).unwrap(),
            "{f}"
        );
    }
}

#[test]
fn map_rejects_bad_settings() {
    let dir = tmp("map-bad");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--downscale", "3"])
        .failure()
        .stderr(predicate::str::contains("1, 2 oder 4"));
    map(&dir, &out, &["--voxel", "0"])
        .failure()
        .stderr(predicate::str::contains("Voxelgröße"));
    map(&dir, &dir.join("nowhere"), &[])
        .failure()
        .stderr(predicate::str::contains("traj.tum"));
}
