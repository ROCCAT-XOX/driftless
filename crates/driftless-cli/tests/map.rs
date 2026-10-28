//! `driftless map` on a synthetic scan: the static room of the `slam` tests, plus camera
//! images rendered from the room and a calibration.

mod common;

use assert_cmd::Command;
use common::{room_bag, scan_with_images, tmp, IMAGES, WALL_X};
use predicates::prelude::*;
use std::fs;
use std::path::Path;

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
    // One cloud to look at, beside the stages and the removed points.
    for f in [
        "dense.bin",
        "removed.las",
        "clean.bin",
        "color.las",
        "map.json",
    ] {
        assert!(out.join(f).is_file(), "{f} missing");
    }
    for f in ["dense.las", "clean.las", "color.ply"] {
        assert!(!out.join(f).exists(), "{f} written without --alle-dateien");
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

/// `--alle-dateien`: every step's cloud as LAS and `color.ply`, as before the first release.
#[test]
fn map_all_files_writes_every_cloud() {
    let dir = tmp("map-alle-dateien");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--alle-dateien"])
        .success()
        .stdout(predicate::str::contains("→ dense.las, dense.bin"))
        .stdout(predicate::str::contains("→ color.las, color.ply"));
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
    let dense = json["steps"]["dense"]["stats"]["points"].as_u64().unwrap();
    assert_eq!(las_points(&out.join("dense.las")).len() as u64, dense);
    assert_eq!(
        json["steps"]["color"]["files"],
        serde_json::json!(["color.las", "color.ply"])
    );
    // The same map as without the switch: it is part of no id.
    let plain = dir.join("ohne");
    slam(&dir, &plain);
    map(&dir, &plain, &[]).success();
    for step in ["dense", "clean", "color"] {
        assert_eq!(
            map_json(&plain)["steps"][step]["input_id"],
            json["steps"][step]["input_id"]
        );
    }
    assert_eq!(
        fs::read(plain.join("color.las")).unwrap(),
        fs::read(out.join("color.las")).unwrap()
    );
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
            "color.las stammt aus einem früheren Lauf und passt nicht mehr zu clean.bin",
        ))
        .stderr(predicate::str::contains("fehlen").not());
    let json = map_json(&out);
    assert!(json["steps"]["color"]["skipped"].is_string());
    assert_eq!(json["stale"], serde_json::json!(["color.las"]));
    // Without colour the cleaned cloud is the one to look at.
    assert!(out.join("clean.las").is_file());
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
            "Warnung: color.las stammt aus einem früheren Lauf und passt nicht mehr zu \
             clean.bin",
        ));
    let json = map_json(&out);
    assert_eq!(json["steps"]["dense"]["reused"], false);
    assert!(json["steps"]["color"].is_null(), "colour no longer fits");
    assert_eq!(json["stale"], serde_json::json!(["color.las"]));
    // Nothing is deleted.
    assert_eq!(fs::read(out.join("color.las")).unwrap(), color);

    // After `--step dense` with another voxel size the cleaned cloud is stale too.
    map(&dir, &out, &["--step", "dense", "--voxel", "0.05"])
        .success()
        .stderr(predicate::str::contains(
            "clean.las, removed.las, clean.bin stammen aus einem früheren Lauf und passen \
             nicht mehr zu dense.bin",
        ));
    assert_eq!(
        map_json(&out)["stale"],
        serde_json::json!(["clean.las", "removed.las", "clean.bin", "color.las"])
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
        &[
            "--step",
            "clean",
            "--no-smooth",
            "--no-through",
            "--alle-dateien",
        ],
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

    // The switches change the step ids: the unsmoothed result is not reused by a color run
    // with the default settings (`--step color` reuses a fitting clean.bin).
    let unsmoothed_id = json["steps"]["clean"]["input_id"].clone();
    map(&dir, &out, &["--step", "color"])
        .success()
        .stdout(predicate::str::contains("Wanddicke"));
    let json = map_json(&out);
    assert_eq!(json["settings"]["clean"]["smooth"]["enabled"], true);
    assert_eq!(json["steps"]["clean"]["reused"], false);
    assert_ne!(json["steps"]["clean"]["input_id"], unsmoothed_id);
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
    map(&dir, &out, &["--alle-dateien"]).success();
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
        // The colour files and `dense.las` (not written now) are from the "old" run and no
        // longer fit: warned, not deleted.
        .stderr(predicate::str::contains(
            "color.las, color.ply stammen aus einem früheren Lauf",
        ))
        .stderr(predicate::str::contains(
            "dense.las stammt aus einem früheren Lauf",
        ));
    assert_eq!(fs::read(out.join("clean.las")).unwrap(), clean);
    let json = map_json(&out);
    assert_eq!(json["steps"]["dense"]["reused"], false);
    assert_eq!(json["steps"]["clean"]["reused"], false);
    assert!(json["steps"]["color"].is_null());
    assert_eq!(
        json["stale"],
        serde_json::json!(["dense.las", "color.las", "color.ply"])
    );
}

#[test]
fn map_is_deterministic() {
    let dir = tmp("map-determinism");
    scan_with_images(&dir);
    let (a, b) = (dir.join("a"), dir.join("b"));
    for out in [&a, &b] {
        slam(&dir, out);
        map(&dir, out, &["--alle-dateien"]).success();
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

/// The colour of the folder `out` (a map of `dir`) computed by the library's CPU reference
/// `colorize` from its `clean.bin`, written as `color.las`/`color.ply` into `into`.
fn cpu_reference(dir: &Path, out: &Path, into: &Path) {
    use driftless_io::calib::load_calibration;
    use driftless_io::camera::Camera;
    use driftless_map::{colorize, load_stage, ColorCamera, ColorConfig, ImageSource};
    use driftless_steps::map::{clean_stage_id, MapInput, MapSettings};
    use driftless_traj::nalgebra::Isometry3;

    let input = MapInput {
        scan: dir.to_path_buf(),
        slam_dir: out.to_path_buf(),
        slam: driftless_steps::slam::SlamSettings::default(),
        map: MapSettings::default(),
    };
    let id = clean_stage_id(&input).unwrap();
    let (mut cloud, _) = load_stage(&out.join("clean.bin"), &id).unwrap();
    let calib = load_calibration(&dir.join("calibration/calib.json")).unwrap();
    let c = &calib.cameras["left"];
    let cams = [ColorCamera::new(
        Camera::from_calib("left", c).unwrap(),
        &c.lidar_to_cam.unwrap(),
        &Isometry3::identity(),
    )];
    let images = ImageSource {
        bags: vec![dir.join("IMAGE_0.bag")],
        topics: vec!["/camera_left/image_raw".into()],
    };
    let poses = driftless_traj::read_tum(&out.join("traj.tum")).unwrap();
    colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    fs::create_dir_all(into).unwrap();
    driftless_map::write_las(&into.join("color.las"), &cloud).unwrap();
    driftless_map::write_ply(&into.join("color.ply"), &cloud).unwrap();
}

#[test]
fn gpu_off_is_the_cpu_path() {
    let dir = tmp("map-gpu-off");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &["--gpu", "aus", "--alle-dateien"])
        .success()
        .stdout(predicate::str::contains("Grafikkarte").not());
    let reference = dir.join("reference");
    cpu_reference(&dir, &out, &reference);
    for f in ["color.las", "color.ply"] {
        assert_eq!(
            fs::read(out.join(f)).unwrap(),
            fs::read(reference.join(f)).unwrap(),
            "{f}: --gpu aus is byte-identical to the CPU reference"
        );
    }
    let gpu = &map_json(&out)["steps"]["color"]["gpu"];
    assert_eq!(gpu["used"], false, "{gpu}");
    assert!(gpu["name"].is_null(), "{gpu}");
}

#[test]
fn map_json_reports_gpu() {
    let dir = tmp("map-gpu-report");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    let has_gpu = driftless_gpu::GpuContext::new(driftless_gpu::GpuPref::Auto)
        .unwrap()
        .is_some();
    let run = map(&dir, &out, &[]).success();
    let gpu = map_json(&out)["steps"]["color"]["gpu"].clone();
    for k in ["used", "name", "backend", "driver"] {
        assert!(gpu.get(k).is_some(), "gpu.{k} missing: {gpu}");
    }
    assert_eq!(gpu["used"], has_gpu, "{gpu}");
    if has_gpu {
        assert!(gpu["name"].as_str().is_some_and(|n| !n.is_empty()), "{gpu}");
        assert!(gpu["backend"].is_string(), "{gpu}");
        run.stdout(predicate::str::contains("Einfärben auf der Grafikkarte"));
        // `--gpu an` uses it too and the colour id does not change; the CPU result stays close
        // (checked after the `--gpu aus` run below).
        let id = map_json(&out)["steps"]["color"]["input_id"].clone();
        map(&dir, &out, &["--step", "color", "--gpu", "an"]).success();
        let json = map_json(&out);
        assert_eq!(json["steps"]["color"]["gpu"]["used"], true);
        assert_eq!(json["steps"]["color"]["input_id"], id);
    } else {
        // `auto` says why the CPU computes; `an` fails without a GPU.
        run.stdout(predicate::str::contains("Einfärben auf der CPU"));
        map(&dir, &out, &["--step", "color", "--gpu", "an"])
            .failure()
            .stderr(predicate::str::contains("Grafikkarte"));
    }
    let on_gpu = las_points(&out.join("color.las"));
    map(&dir, &out, &["--step", "color", "--gpu", "aus"]).success();
    let json = map_json(&out);
    assert_eq!(json["steps"]["color"]["gpu"]["used"], false);
    assert_eq!(json["steps"]["color"]["reused"], false);
    if has_gpu {
        let on_cpu = las_points(&out.join("color.las"));
        assert_eq!(on_cpu.len(), on_gpu.len());
        // LAS colours are 16 bit (sRGB × 257): 2 sRGB levels.
        let close = on_cpu
            .iter()
            .zip(&on_gpu)
            .filter(|(a, b)| {
                let (a, b) = (a.color.unwrap(), b.color.unwrap());
                [(a.red, b.red), (a.green, b.green), (a.blue, b.blue)]
                    .iter()
                    .all(|&(x, y)| x.abs_diff(y) <= 2 * 257)
            })
            .count();
        assert!(
            close * 100 >= 99 * on_cpu.len(),
            "{close} of {}",
            on_cpu.len()
        );
    }
    map(&dir, &out, &["--gpu", "vielleicht"])
        .failure()
        .stderr(predicate::str::contains("auto|an|aus"));
}

#[test]
fn auto_falls_back_when_gpu_budget_exceeded() {
    let dir = tmp("map-gpu-budget");
    scan_with_images(&dir);
    let (cpu, small) = (dir.join("cpu"), dir.join("small"));
    for out in [&cpu, &small] {
        slam(&dir, out);
    }
    map(&dir, &cpu, &["--gpu", "aus", "--alle-dateien"]).success();
    // A budget of 1 kB: `auto` says why and colours on the CPU, `an` fails in German.
    Command::cargo_bin("driftless")
        .unwrap()
        .env("DRIFTLESS_GPU_BUDGET", "1000")
        .args(["map"])
        .arg(&dir)
        .arg("--slam")
        .arg(&small)
        .arg("--alle-dateien")
        .assert()
        .success()
        .stdout(predicate::str::contains("Einfärben auf der CPU"));
    for f in ["color.las", "color.ply"] {
        assert_eq!(
            fs::read(small.join(f)).unwrap(),
            fs::read(cpu.join(f)).unwrap(),
            "{f}"
        );
    }
    assert_eq!(map_json(&small)["steps"]["color"]["gpu"]["used"], false);
    if driftless_gpu::GpuContext::new(driftless_gpu::GpuPref::Auto)
        .unwrap()
        .is_some()
    {
        assert!(map_json(&small)["steps"]["color"]["gpu"]["fallback"]
            .as_str()
            .is_some_and(|w| w.contains("Grafikspeicher")));
        Command::cargo_bin("driftless")
            .unwrap()
            .env("DRIFTLESS_GPU_BUDGET", "1000")
            .args(["map"])
            .arg(&dir)
            .arg("--slam")
            .arg(&small)
            .args(["--step", "color", "--gpu", "an"])
            .assert()
            .failure()
            .stderr(predicate::str::contains("Grafikspeicher"));
    }
}

/// `--gpu auto` without a usable device writes why into `map.json` (no DX12 outside
/// Windows: `WGPU_BACKEND=dx12` hides every adapter); `--gpu aus` writes no reason.
#[cfg(not(windows))]
#[test]
fn map_json_says_why_auto_took_the_cpu() {
    let dir = tmp("map-gpu-none");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    Command::cargo_bin("driftless")
        .unwrap()
        .env("WGPU_BACKEND", "dx12")
        .arg("map")
        .arg(&dir)
        .arg("--slam")
        .arg(&out)
        .assert()
        .success()
        .stdout(predicate::str::contains("Einfärben auf der CPU"));
    let gpu = &map_json(&out)["steps"]["color"]["gpu"];
    assert_eq!(gpu["used"], false, "{gpu}");
    assert_eq!(gpu["fallback"], "keine Grafikkarte gefunden", "{gpu}");
    map(&dir, &out, &["--step", "color", "--gpu", "aus"]).success();
    assert!(map_json(&out)["steps"]["color"]["gpu"]["fallback"].is_null());
}

/// The same `driftless map` again is „aktuell“: nothing computed, nothing written. The GPU
/// switch does not matter (an equivalent map); `--neu` computes it anyway.
#[test]
fn map_twice_is_current() {
    let dir = tmp("map-zweimal");
    scan_with_images(&dir);
    let out = dir.join("out");
    slam(&dir, &out);
    map(&dir, &out, &[]).success();
    let json = fs::read(out.join("map.json")).unwrap();
    for args in [&[][..], &["--gpu", "aus"]] {
        map(&dir, &out, args)
            .success()
            .stdout(predicate::str::contains("Karte ist aktuell"))
            .stdout(predicate::str::contains("Dicht:").not());
    }
    assert_eq!(fs::read(out.join("map.json")).unwrap(), json);
    map(&dir, &out, &["--neu"])
        .success()
        .stdout(predicate::str::contains("Dicht:"))
        .stdout(predicate::str::contains("Eingefärbt:"))
        .stdout(predicate::str::contains("Karte ist aktuell").not());
}
