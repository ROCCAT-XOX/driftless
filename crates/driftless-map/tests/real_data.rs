//! Dense cloud of a real JMK6 scan in `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`): SLAM,
//! trajectory through a TUM file (as `traj.tum` of `driftless slam`), then the dense cloud
//! and LAS/PLY. Prints point count and timings; skips when the data is missing.
//!
//! `cargo test --release -p driftless-map --test real_data -- --ignored --nocapture`

use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_io::testdata;
use driftless_map::{build_dense, write_las, write_ply, DenseConfig};
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
