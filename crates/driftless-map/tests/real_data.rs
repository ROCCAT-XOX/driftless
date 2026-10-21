//! The map of real JMK6 scans in `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`): SLAM,
//! trajectory through a TUM file (as `traj.tum` of `driftless slam`), then the dense cloud
//! and LAS/PLY, its cleaning and colouring, with the default settings of `driftless map`.
//! Prints point counts and timings; skips when the data is missing. `DRIFTLESS_COLOR_OUT=<dir>`
//! keeps the coloured cloud of 143623 (`color.las`, `color.ply`) there.
//!
//! The bounds are measured values with a margin, see `docs/ergebnisse/2026-10-21-meilenstein-3.md`.
//!
//! `cargo test --release -p driftless-map --test real_data -- --ignored --nocapture`

use driftless_io::bag::BagReader;
use driftless_io::calib::load_calibration;
use driftless_io::camera::Camera;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_io::testdata;
use driftless_map::{
    build_dense, clean, colorize, content_hash, write_las, write_ply, write_removed_las,
    CleanConfig, CleanStats, ColorCamera, ColorConfig, ColorStats, DenseConfig, ImageSource,
};
use driftless_slam::{run_slam, SlamConfig};
use driftless_traj::{read_tum, write_tum};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::Instant;

/// The full Test scan (266 s, 1741 images) must be mapped within this time (s).
const TEST_SCAN_TIME_LIMIT_S: f64 = 15.0 * 60.0;
/// Share of coloured points on the Test scan, measured with the default settings.
const TEST_SCAN_COLORED_SHARE: f64 = 0.984;
/// Exposure on the Test scan: camera difference after / before (measured 0.76 / 1.42 = 0.54)
/// and image offset after / before (measured 0.57 / 1.01 = 0.56); bounds with margin.
const TEST_SCAN_CAMERA_DIFFERENCE_RATIO: f64 = 0.7;
const TEST_SCAN_IMAGE_OFFSET_RATIO: f64 = 0.75;

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

/// `folder` of the test data, or `None` (with a message) if it is not there.
fn scan(rel: &str) -> Option<PathBuf> {
    let found = testdata::path(rel).filter(|p| p.exists());
    if found.is_none() {
        eprintln!("SKIPPED: {} not set or {rel} missing", testdata::ENV_VAR);
    }
    found
}

/// The cameras of the scan with their mounts and its image topics.
fn color_input(
    folder: &Path,
    lidar_to_imu: &driftless_traj::nalgebra::Isometry3<f64>,
) -> (Vec<ColorCamera>, ImageSource) {
    let project = discover_project(folder).unwrap();
    let calib = load_calibration(project.calibration.as_ref().unwrap()).unwrap();
    let cams: Vec<ColorCamera> = calib
        .cameras
        .iter()
        .map(|(name, c)| {
            ColorCamera::new(
                Camera::from_calib(name, c).unwrap(),
                &c.lidar_to_cam.unwrap(),
                lidar_to_imu,
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
        bags: project.image_bags,
        topics,
    };
    (cams, images)
}

/// One run of SLAM, dense, clean and colour with the default settings.
struct FullRun {
    /// Seconds of SLAM, dense, clean and colour.
    seconds: [f64; 4],
    dense_points: usize,
    clean: CleanStats,
    color: ColorStats,
    /// Content hash of every output file (`traj.tum`, `dense.las`, `clean.las`,
    /// `removed.las`, `color.las`, `color.ply`).
    hashes: Vec<(&'static str, u64)>,
}

impl FullRun {
    fn total_seconds(&self) -> f64 {
        self.seconds.iter().sum()
    }

    fn colored_share(&self) -> f64 {
        let c = &self.color;
        c.points_colored as f64 / (c.points_colored + c.points_uncolored).max(1) as f64
    }

    fn print(&self, name: &str) {
        let [s, d, c, k] = self.seconds;
        let x = &self.color;
        println!(
            "{name}: SLAM {s:.1} s, dense {d:.1} s, clean {c:.1} s, colour {k:.1} s, total \
             {:.1} s; {} dense points, {} kept, {} ghosts, {} outliers; {:.1} % coloured, \
             images {} used / {} blurry; gains {:.2}…{:.2}; spread {:.2} → {:.2}, camera \
             difference {:.2} → {:.2} ({} points), image offset {:.2} → {:.2} (sRGB)",
            self.total_seconds(),
            self.dense_points,
            self.clean.kept,
            self.clean.ghosts,
            self.clean.outliers,
            100.0 * self.colored_share(),
            x.images_used,
            x.images_blurry,
            x.gain_min,
            x.gain_max,
            x.spread_before,
            x.spread_after,
            x.camera_difference_before,
            x.camera_difference_after,
            x.camera_difference_points,
            x.image_offset_before,
            x.image_offset_after,
        );
    }
}

/// A temporary output folder that is removed on drop, also when a test panics.
struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// SLAM, dense, clean and colour of the scan in `folder`, as `driftless slam` and
/// `driftless map` run them with their defaults; the output files go to a temporary folder
/// and only their hashes are kept.
fn full_run(folder: &Path, tag: &str) -> FullRun {
    let bags = discover_project(folder).unwrap().lidar_bags;
    let cfg = SlamConfig::default();
    let guard = TempDir(
        std::env::temp_dir().join(format!("driftless-map-full-{tag}-{}", std::process::id())),
    );
    let out = guard.0.clone();
    std::fs::create_dir_all(&out).unwrap();

    let start = Instant::now();
    let imu = read_imu(&bags);
    let (slam, _) = with_frames(&bags, |frames| run_slam(frames, &imu, &cfg));
    write_tum(&out.join("traj.tum"), &slam.poses()).unwrap();
    drop(slam);
    let poses = read_tum(&out.join("traj.tum")).unwrap();
    let slam_s = start.elapsed().as_secs_f64();

    let start = Instant::now();
    let (cloud, rays, dense) = with_frames(&bags, |frames| {
        build_dense(frames, &imu, &cfg.odom, &poses, &DenseConfig::default())
    })
    .unwrap();
    drop(imu);
    write_las(&out.join("dense.las"), &cloud).unwrap();
    let dense_s = start.elapsed().as_secs_f64();

    let start = Instant::now();
    let (mut kept, removed, reasons, clean_stats) =
        clean(cloud, &rays, &CleanConfig::default()).unwrap();
    drop(rays);
    write_las(&out.join("clean.las"), &kept).unwrap();
    write_removed_las(&out.join("removed.las"), &removed, &reasons).unwrap();
    drop(removed);
    let clean_s = start.elapsed().as_secs_f64();

    let start = Instant::now();
    let (cams, images) = color_input(folder, &cfg.odom.lidar_to_imu);
    let color = colorize(&mut kept, images, &poses, &cams, &ColorConfig::default()).unwrap();
    write_las(&out.join("color.las"), &kept).unwrap();
    write_ply(&out.join("color.ply"), &kept).unwrap();
    let color_s = start.elapsed().as_secs_f64();

    let files = [
        "traj.tum",
        "dense.las",
        "clean.las",
        "removed.las",
        "color.las",
        "color.ply",
    ];
    let hashes = files
        .iter()
        .map(|&f| (f, content_hash(&std::fs::read(out.join(f)).unwrap())))
        .collect();
    drop(guard);
    FullRun {
        seconds: [slam_s, dense_s, clean_s, color_s],
        dense_points: dense.points,
        clean: clean_stats,
        color,
        hashes,
    }
}

/// The full run of the Test scan, shared by the tests that check it.
fn test_scan() -> Option<&'static FullRun> {
    static RUN: OnceLock<Option<FullRun>> = OnceLock::new();
    RUN.get_or_init(|| {
        let folder = scan("Test")?;
        let run = full_run(&folder, "test");
        run.print("Test");
        Some(run)
    })
    .as_ref()
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn test_scan_full_under_15_min() {
    let Some(run) = test_scan() else { return };
    assert!(
        run.total_seconds() < TEST_SCAN_TIME_LIMIT_S,
        "{:.0} s",
        run.total_seconds()
    );
    assert!(run.color.images_used > 1000, "{:?}", run.color.images_used);
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn test_scan_colored_share() {
    let Some(run) = test_scan() else { return };
    assert!(
        run.colored_share() >= TEST_SCAN_COLORED_SHARE * 0.95,
        "{:.3}",
        run.colored_share()
    );
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn test_scan_exposure_reduces_camera_difference() {
    let Some(run) = test_scan() else { return };
    let c = &run.color;
    assert!(c.camera_difference_points > 100_000, "{c:?}");
    assert!(
        c.camera_difference_after < TEST_SCAN_CAMERA_DIFFERENCE_RATIO * c.camera_difference_before,
        "camera difference {:.3} → {:.3}",
        c.camera_difference_before,
        c.camera_difference_after
    );
    assert!(
        c.image_offset_after < TEST_SCAN_IMAGE_OFFSET_RATIO * c.image_offset_before,
        "image offset {:.3} → {:.3}",
        c.image_offset_before,
        c.image_offset_after
    );
    assert!(
        c.spread_after < c.spread_before,
        "spread {:.3} → {:.3}",
        c.spread_before,
        c.spread_after
    );
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_scans_run_and_are_deterministic() {
    for name in ["20261020143415", "20261020143623"] {
        let Some(folder) = scan(&format!("jmk6-2026-10-20/{name}")) else {
            continue;
        };
        let a = full_run(&folder, &format!("{name}-a"));
        a.print(name);
        let b = full_run(&folder, &format!("{name}-b"));
        assert_eq!(a.hashes, b.hashes, "{name}: two runs differ");
        assert!(
            a.color.images_used > 100,
            "{name}: {:?}",
            a.color.images_used
        );
        assert!(a.colored_share() > 0.9, "{name}: {:.3}", a.colored_share());
        assert!(
            a.clean.kept > a.dense_points * 9 / 10,
            "{name}: cleaning removed more than 10 %"
        );
    }
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
    let (kept, removed, reasons, stats) = clean(cloud, &rays, &CleanConfig::default()).unwrap();
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
    write_removed_las(&out.join("removed.las"), &removed, &reasons).unwrap();
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
    let bags = discover_project(&folder).unwrap().lidar_bags;
    let imu = read_imu(&bags);
    let cfg = SlamConfig::default();
    let (slam, _) = with_frames(&bags, |frames| run_slam(frames, &imu, &cfg));
    let poses = slam.poses();
    let (cloud, rays, _) = with_frames(&bags, |frames| {
        build_dense(frames, &imu, &cfg.odom, &poses, &DenseConfig::default())
    })
    .unwrap();
    let (mut cloud, _, _, _) = clean(cloud, &rays, &CleanConfig::default()).unwrap();

    let (cams, images) = color_input(&folder, &cfg.odom.lidar_to_imu);

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
        "143623: gains {:.3}…{:.3} ({} at the bound); view spread {:.2} → {:.2} (sRGB 0–255); \
         edge falloff residual {:.2} → {:.2} %; {:?}",
        stats.gain_min,
        stats.gain_max,
        stats.images_gain_clamped,
        stats.spread_before,
        stats.spread_after,
        stats.vignette_residual_before,
        stats.vignette_residual_after,
        stats.vignetting.coeffs,
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
