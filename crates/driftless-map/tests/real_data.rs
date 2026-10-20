//! Dense cloud of a real JMK6 scan in `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`): SLAM,
//! trajectory through a TUM file (as `traj.tum` of `driftless slam`), then the dense cloud
//! and LAS/PLY, its cleaning and colouring. Prints point counts and timings; skips when the
//! data is missing. `DRIFTLESS_COLOR_OUT=<dir>` keeps the coloured cloud (`color.las`,
//! `color.ply`) there.
//!
//! `cargo test --release -p driftless-map --test real_data -- --ignored --nocapture`

use driftless_io::bag::BagReader;
use driftless_io::calib::load_calibration;
use driftless_io::camera::Camera;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_io::testdata;
use driftless_map::{
    build_dense, clean, colorize, write_las, write_ply, CleanConfig, ColorCamera, ColorConfig,
    DenseConfig, ImageSource,
};
use driftless_slam::{run_slam, SlamConfig};
use driftless_traj::{read_tum, write_tum};
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Topic of the first connection of `msg_type` in the bag.
fn topic(bag: &Path, msg_type: &str) -> String {
    let r = BagReader::open(bag).unwrap();
    r.connections()
        .iter()
        .find(|c| c.msg_type == msg_type)
        .unwrap_or_else(|| panic!("{}: no {msg_type}", bag.display()))
        .topic
        .clone()
}

fn read_imu(bags: &[PathBuf]) -> Vec<ImuSample> {
    let mut imu = Vec::new();
    for bag in bags {
        let t = topic(bag, "sensor_msgs/Imu");
        let mut r = BagReader::open(bag).unwrap();
        for m in r.messages(&[t.as_str()]).map_while(Result::ok) {
            imu.extend(decode_imu(&m.data).ok());
        }
    }
    imu
}

/// Streams the LiDAR frames of `bags` from a reader thread into `consume`.
fn with_frames<R>(
    bags: &[PathBuf],
    consume: impl FnOnce(&mut dyn Iterator<Item = LidarFrame>) -> R,
) -> R {
    let (tx, rx) = std::sync::mpsc::sync_channel::<LidarFrame>(16);
    let reader_bags = bags.to_vec();
    let reader = std::thread::spawn(move || {
        for bag in &reader_bags {
            let t = topic(bag, "sensor_msgs/PointCloud2");
            let mut r = BagReader::open(bag).unwrap();
            for m in r.messages(&[t.as_str()]).map_while(Result::ok) {
                if let Ok(f) = decode_pointcloud2(&m.data) {
                    if tx.send(f).is_err() {
                        return;
                    }
                }
            }
        }
    });
    let out = consume(&mut rx.into_iter());
    reader.join().unwrap();
    out
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_scan_143623_dense_cloud() {
    let Some(folder) = testdata::path("jmk6-2026-10-20/20261020143623").filter(|p| p.exists())
    else {
        eprintln!("SKIPPED: {} not set or scan missing", testdata::ENV_VAR);
        return;
    };
    let bags = discover_project(&folder).unwrap().lidar_bags;
    let imu = read_imu(&bags);
    let cfg = SlamConfig::default();

    let start = Instant::now();
    let (slam, _) = with_frames(&bags, |frames| run_slam(frames, &imu, &cfg));
    let slam_s = start.elapsed().as_secs_f64();
    let out = std::env::temp_dir().join(format!("driftless-map-real-{}", std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    write_tum(&out.join("traj.tum"), &slam.poses()).unwrap();
    let corrected = read_tum(&out.join("traj.tum")).unwrap();

    let start = Instant::now();
    let (cloud, _, stats) = with_frames(&bags, |frames| {
        build_dense(frames, &imu, &cfg.odom, &corrected, &DenseConfig::default())
    })
    .unwrap();
    let dense_s = start.elapsed().as_secs_f64();
    let start = Instant::now();
    write_las(&out.join("dense.las"), &cloud).unwrap();
    write_ply(&out.join("dense.ply"), &cloud).unwrap();
    let write_s = start.elapsed().as_secs_f64();
    let size = |n: &str| std::fs::metadata(out.join(n)).unwrap().len() / 1_000_000;
    let (imin, imax) = cloud
        .intensity
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), &i| {
            (a.min(i), b.max(i))
        });
    println!(
        "143623: SLAM {slam_s:.1} s, dense {dense_s:.1} s, write {write_s:.1} s; {} frames, \
         {} points in range -> {} in cloud, {} rays, intensity {imin}..{imax}; \
         dense.las {} MB, dense.ply {} MB",
        stats.frames,
        stats.points_in_range,
        stats.points,
        stats.rays,
        size("dense.las"),
        size("dense.ply"),
    );
    assert_eq!(stats.frames, corrected.len());
    assert!(stats.points > 100_000);
    std::fs::remove_dir_all(&out).unwrap();
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_scan_143623_clean() {
    let Some(folder) = testdata::path("jmk6-2026-10-20/20261020143623").filter(|p| p.exists())
    else {
        eprintln!("SKIPPED: {} not set or scan missing", testdata::ENV_VAR);
        return;
    };
    let bags = discover_project(&folder).unwrap().lidar_bags;
    let imu = read_imu(&bags);
    let cfg = SlamConfig::default();
    let (slam, _) = with_frames(&bags, |frames| run_slam(frames, &imu, &cfg));
    let (cloud, rays, _) = with_frames(&bags, |frames| {
        build_dense(
            frames,
            &imu,
            &cfg.odom,
            &slam.poses(),
            &DenseConfig::default(),
        )
    })
    .unwrap();
    let points = cloud.len();

    let start = Instant::now();
    let (kept, removed, stats) = clean(cloud, &rays, &CleanConfig::default()).unwrap();
    let clean_s = start.elapsed().as_secs_f64();
    println!(
        "143623: clean {clean_s:.1} s; {points} points, {} rays -> {} kept, {} ghosts, {} \
         outliers",
        rays.ends.len(),
        stats.kept,
        stats.ghosts,
        stats.outliers,
    );
    let out = std::env::temp_dir().join(format!("driftless-map-clean-{}", std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    write_las(&out.join("clean.las"), &kept).unwrap();
    write_las(&out.join("removed.las"), &removed).unwrap();
    assert_eq!(kept.len() + removed.len(), points);
    assert!(
        stats.kept > points * 9 / 10,
        "cleaning removed more than 10 %"
    );
    std::fs::remove_dir_all(&out).unwrap();
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_scan_143623_color() {
    let Some(folder) = testdata::path("jmk6-2026-10-20/20261020143623").filter(|p| p.exists())
    else {
        eprintln!("SKIPPED: {} not set or scan missing", testdata::ENV_VAR);
        return;
    };
    let project = discover_project(&folder).unwrap();
    let bags = project.lidar_bags.clone();
    let imu = read_imu(&bags);
    let cfg = SlamConfig::default();
    let (slam, _) = with_frames(&bags, |frames| run_slam(frames, &imu, &cfg));
    let poses = slam.poses();
    let (cloud, rays, _) = with_frames(&bags, |frames| {
        build_dense(frames, &imu, &cfg.odom, &poses, &DenseConfig::default())
    })
    .unwrap();
    let (mut cloud, _, _) = clean(cloud, &rays, &CleanConfig::default()).unwrap();

    let calib = load_calibration(project.calibration.as_ref().unwrap()).unwrap();
    let cams: Vec<ColorCamera> = calib
        .cameras
        .iter()
        .map(|(name, c)| {
            ColorCamera::new(
                Camera::from_calib(name, c).unwrap(),
                &c.lidar_to_cam.unwrap(),
                &cfg.odom.lidar_to_imu,
            )
        })
        .collect();
    let mut topics: Vec<String> = Vec::new();
    for bag in &project.image_bags {
        let r = BagReader::open(bag).unwrap();
        topics.extend(
            r.connections()
                .iter()
                .filter(|c| c.msg_type == "sensor_msgs/CompressedImage")
                .map(|c| c.topic.clone()),
        );
    }
    topics.sort();
    topics.dedup();
    let images = ImageSource {
        bags: project.image_bags.clone(),
        topics,
    };

    let start = Instant::now();
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    let color_s = start.elapsed().as_secs_f64();
    println!(
        "143623: colour {color_s:.1} s; {} points, {} coloured ({:.1} %); images {} used, {} \
         blurry, {} outside, {} bad",
        cloud.len(),
        stats.points_colored,
        100.0 * stats.points_colored as f64 / cloud.len() as f64,
        stats.images_used,
        stats.images_blurry,
        stats.images_outside_trajectory,
        stats.images_bad,
    );
    println!(
        "143623: gains {:.3}…{:.3}; view spread {:.2} → {:.2} (sRGB 0–255)",
        stats.gain_min, stats.gain_max, stats.spread_before, stats.spread_after,
    );
    match std::env::var_os("DRIFTLESS_COLOR_OUT") {
        Some(dir) => {
            let out = PathBuf::from(dir);
            std::fs::create_dir_all(&out).unwrap();
            write_las(&out.join("color.las"), &cloud).unwrap();
            write_ply(&out.join("color.ply"), &cloud).unwrap();
        }
        None => {
            let out =
                std::env::temp_dir().join(format!("driftless-map-color-{}", std::process::id()));
            std::fs::create_dir_all(&out).unwrap();
            write_las(&out.join("color.las"), &cloud).unwrap();
            std::fs::remove_dir_all(&out).unwrap();
        }
    }
    assert!(stats.images_used > 0);
    assert!(stats.points_colored > cloud.len() / 2);
    assert!(stats.spread_after < stats.spread_before);
}
