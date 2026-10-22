//! Loading a LAS cloud into a [`PointSet`].

use driftless_app::cloud::{load_las, PointSet};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use driftless_map::{write_las, Cloud};
use driftless_slam::report::{BeforeAfter, SlamReport, SubmapReport};
use driftless_steps::StepError;
use std::path::Path;

/// A small cloud as the map step exports it: absolute Unix times, 4 points.
fn small_cloud(rgb: bool) -> Cloud {
    let xyz = vec![
        [10.0, 20.0, 1.0],
        [10.5, 20.25, 1.5],
        [11.0, 21.0, 2.0],
        [12.123_4, 19.876_5, 0.5],
    ];
    Cloud {
        intensity: vec![0.0, 100.0, 200.0, 255.0],
        t: vec![
            1_700_000_000.0,
            1_700_000_001.5,
            1_700_000_003.0,
            1_700_000_010.0,
        ],
        frame: vec![0, 1, 2, 3],
        rgb: rgb.then(|| vec![[255, 0, 0], [0, 255, 0], [0, 0, 255], [1, 2, 3]]),
        colored: rgb.then(|| vec![true, false, true, false]),
        xyz,
    }
}

fn absolute(p: &PointSet, i: usize) -> [f64; 3] {
    let r = p.xyz[i];
    [0, 1, 2].map(|k| p.origin[k] + f64::from(r[k]))
}

fn report(scan_start: f64, starts: &[f64], qualities: &[f32]) -> SlamReport {
    SlamReport {
        scan_start,
        submaps: starts
            .iter()
            .zip(qualities)
            .enumerate()
            .map(|(id, (&t_start, &quality))| SubmapReport {
                id,
                t_start,
                t_end: t_start + 1.0,
                quality,
                anchor: [0.0; 3],
                odom_anchor: [0.0; 3],
            })
            .collect(),
        loops: Vec::new(),
        graph: None,
        start_end_gap: BeforeAfter {
            before: 0.0,
            after: 0.0,
        },
        path_length: BeforeAfter {
            before: 0.0,
            after: 0.0,
        },
    }
}

/// LAS 1.2 format 1 with exact (f64) coordinates.
fn write_raw_las(path: &Path, points: &[[f64; 3]], offset: [f64; 3]) {
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(1).unwrap();
    let t = |offset| las::Transform {
        scale: 0.0001,
        offset,
    };
    builder.transforms = las::Vector {
        x: t(offset[0]),
        y: t(offset[1]),
        z: t(offset[2]),
    };
    let mut w = las::Writer::from_path(path, builder.into_header().unwrap()).unwrap();
    for (i, p) in points.iter().enumerate() {
        w.write_point(las::Point {
            x: p[0],
            y: p[1],
            z: p[2],
            intensity: 1000,
            gps_time: Some(i as f64),
            ..Default::default()
        })
        .unwrap();
    }
    w.close().unwrap();
}

#[test]
fn loads_color_las_from_export() {
    let dir = tmp("app-cloud-color");
    let path = dir.join("color.las");
    let cloud = small_cloud(true);
    write_las(&path, &cloud).unwrap();
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 4);
    // Origin: centre of the hull in whole metres.
    assert_eq!(p.origin, [11.0, 20.0, 1.0]);
    for i in 0..4 {
        let a = absolute(&p, i);
        for k in 0..3 {
            let want = f64::from(cloud.xyz[i][k]);
            assert!((a[k] - want).abs() <= 0.000_1, "point {i} axis {k}: {a:?}");
        }
    }
    assert_eq!(p.rgb.as_deref(), cloud.rgb.as_deref());
    assert_eq!(p.flags, vec![0, 1, 0, 1]);
    // 0..255 → LAS ×257 → >> 8.
    assert_eq!(p.intensity, vec![0, 100, 200, 255]);
    // Without a report: seconds since the first point, quality 255.
    assert_eq!(p.t, vec![0.0, 1.5, 3.0, 10.0]);
    assert_eq!(p.quality, vec![255; 4]);
    let (min, max) = p.bounds();
    for k in 0..3 {
        assert!(min[k] <= max[k]);
        for x in &p.xyz {
            assert!(min[k] <= x[k] && x[k] <= max[k]);
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn quality_from_submap_time() {
    let dir = tmp("app-cloud-quality");
    let path = dir.join("color.las");
    // Scan starts at 1_700_000_000.0; submaps start at 0, 2 and 5 s.
    write_las(&path, &small_cloud(true)).unwrap();
    let slam = report(1_700_000_000.0, &[0.0, 2.0, 5.0], &[0.2, 0.6, 1.0]);
    let p = load_las(&path, Some(&slam), &Progress::new()).unwrap();
    assert_eq!(p.t, vec![0.0, 1.5, 3.0, 10.0]);
    // Points at 0 s and 1.5 s → submap 0, 3 s → submap 1, 10 s → submap 2.
    let q = |x: f32| (x * 255.0).round() as u8;
    assert_eq!(p.quality, vec![q(0.2), q(0.2), q(0.6), q(1.0)]);
    assert!(p.quality_known);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn loads_las_without_color() {
    let dir = tmp("app-cloud-nocolor");
    let path = dir.join("map.las");
    write_las(&path, &small_cloud(false)).unwrap();
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 4);
    assert!(p.rgb.is_none());
    assert_eq!(p.flags, vec![0; 4]);
    assert_eq!(p.intensity, vec![0, 100, 200, 255]);
    std::fs::remove_dir_all(dir).unwrap();
}

/// `map.las` of a SLAM run is format 0 without intensity or time: mid gray, quality 255.
#[test]
fn run_cloud_without_intensity_is_mid_gray() {
    let dir = tmp("app-cloud-run");
    let path = dir.join("map.las");
    driftless_steps::scan::write_las(&path, &[[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]]).unwrap();
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 2);
    assert!(p.rgb.is_none());
    assert_eq!(p.intensity, vec![128, 128]);
    assert_eq!(p.t, vec![0.0, 0.0]);
    assert_eq!(p.quality, vec![255, 255]);
    // with the run's report, too: without point times no submap can be picked
    let slam = report(1_700_000_000.0, &[0.0], &[0.5]);
    let p = load_las(&path, Some(&slam), &Progress::new()).unwrap();
    assert!(!p.quality_known);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn loads_empty_las() {
    let dir = tmp("app-cloud-empty");
    let path = dir.join("leer.las");
    write_las(&path, &Cloud::default()).unwrap();
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 0);
    assert!(p.is_empty());
    assert_eq!(p.origin, [0.0; 3]);
    assert_eq!(p.bounds(), ([0.0; 3], [0.0; 3]));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn far_coordinates_keep_mm_precision() {
    let dir = tmp("app-cloud-far");
    let path = dir.join("far.las");
    let base = [512_345.678, 5_412_345.678, 312.345];
    let points: Vec<[f64; 3]> = (0..100)
        .map(|i| {
            let f = f64::from(i);
            [
                base[0] + f * 0.731,
                base[1] - f * 0.417,
                base[2] + f * 0.013,
            ]
        })
        .collect();
    write_raw_las(&path, &points, base.map(f64::floor));
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), points.len());
    for (k, o) in p.origin.iter().enumerate() {
        assert_eq!(o.fract(), 0.0, "origin axis {k} in whole metres");
    }
    for (i, want) in points.iter().enumerate() {
        let a = absolute(&p, i);
        for k in 0..3 {
            assert!(
                (a[k] - want[k]).abs() <= 0.001,
                "point {i} axis {k}: {} vs {}",
                a[k],
                want[k]
            );
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn load_cancelled_returns_cancelled() {
    let dir = tmp("app-cloud-cancel");
    let path = dir.join("color.las");
    write_las(&path, &small_cloud(true)).unwrap();
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == "Wolke laden" {
            handle.cancel();
        }
    });
    let r = load_las(&path, None, &progress);
    assert_eq!(r.err(), Some(StepError::Cancelled));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn load_reports_phase_and_total() {
    let dir = tmp("app-cloud-phase");
    let path = dir.join("color.las");
    write_las(&path, &small_cloud(true)).unwrap();
    let progress = Progress::new();
    load_las(&path, None, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, "Wolke laden");
    assert_eq!(s.total, Some(4));
    assert_eq!(s.done, 4);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn missing_file_is_a_german_error() {
    let r = load_las(Path::new("/gibt/es/nicht.las"), None, &Progress::new());
    match r {
        Err(StepError::Failed(m)) => assert!(m.contains("nicht.las"), "{m}"),
        other => panic!("{other:?}"),
    }
}

/// Foreign writers may leave the header bounds zero; the origin then comes from the first
/// point, so UTM-scale coordinates keep millimetres.
#[test]
fn zero_header_bounds_keep_mm_precision() {
    let dir = tmp("app-cloud-zero-bounds");
    let path = dir.join("fremd.las");
    let base = [512_345.678, 5_412_345.678, 312.345];
    let points: Vec<[f64; 3]> = (0..50)
        .map(|i| {
            let f = f64::from(i);
            [
                base[0] + f * 0.731,
                base[1] - f * 0.417,
                base[2] + f * 0.013,
            ]
        })
        .collect();
    write_raw_las(&path, &points, base.map(f64::floor));
    // LAS 1.2 header: max/min x, y, z as six doubles at bytes 179..227.
    let mut bytes = std::fs::read(&path).unwrap();
    bytes[179..227].fill(0);
    std::fs::write(&path, bytes).unwrap();
    let header = las::Reader::from_path(&path).unwrap().header().bounds();
    assert_eq!([header.min.x, header.max.y], [0.0, 0.0], "bounds zeroed");
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert!((p.origin[0] - base[0]).abs() < 1.0, "origin {:?}", p.origin);
    for (i, want) in points.iter().enumerate() {
        let a = absolute(&p, i);
        for k in 0..3 {
            assert!((a[k] - want[k]).abs() <= 0.001, "point {i} axis {k}");
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// LAS format 3 as some foreign writers produce it: colours and intensity in 0–255.
fn write_8bit_las(path: &Path, colors: &[[u16; 3]], intensity: &[u16]) {
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(3).unwrap();
    let mut w = las::Writer::from_path(path, builder.into_header().unwrap()).unwrap();
    for (i, (c, &it)) in colors.iter().zip(intensity).enumerate() {
        w.write_point(las::Point {
            x: i as f64,
            y: 0.0,
            z: 0.0,
            intensity: it,
            gps_time: Some(0.0),
            color: Some(las::Color::new(c[0], c[1], c[2])),
            ..Default::default()
        })
        .unwrap();
    }
    w.close().unwrap();
}

#[test]
fn eight_bit_colour_and_intensity_are_scaled() {
    let dir = tmp("app-cloud-8bit");
    let path = dir.join("fremd.las");
    write_8bit_las(&path, &[[255, 128, 0], [10, 20, 30]], &[200, 7]);
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(vec![[255, 128, 0], [10, 20, 30]]));
    assert_eq!(p.intensity, vec![200, 7]);
    std::fs::remove_dir_all(dir).unwrap();
}

/// 16-bit files keep `>> 8`, even for dark points, once any value exceeds 255.
#[test]
fn sixteen_bit_dark_points_stay_dark() {
    let dir = tmp("app-cloud-16bit");
    let path = dir.join("dunkel.las");
    write_8bit_las(
        &path,
        &[[255, 128, 0], [65_535, 25_700, 257]],
        &[200, 51_400],
    );
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(vec![[0, 0, 0], [255, 100, 1]]));
    assert_eq!(p.intensity, vec![0, 200]);
    std::fs::remove_dir_all(dir).unwrap();
}
