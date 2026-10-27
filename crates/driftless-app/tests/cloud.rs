//! Loading a cloud (LAS, LAZ, E57, PLY, PCD) into a [`PointSet`].

use driftless_app::cloud::{
    combine_pair_points, load_cloud, load_pair_cloud, load_points, point_set, PointSet, MID_GRAY,
    PAIR_GAP,
};
use driftless_app::render::ColorMode;
use driftless_app::ui::view3d::color_modes;
use driftless_export::{write_points, PointFormat, Points, ScanInfo, PHASE_READ};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use driftless_map::{write_las, write_las_sources, write_ply, write_ply_sources, Cloud};
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
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
    let p = load_points(&path, Some(&slam), &Progress::new()).unwrap();
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 2);
    assert!(p.rgb.is_none());
    assert_eq!(p.intensity, vec![128, 128]);
    assert_eq!(p.t, vec![0.0, 0.0]);
    assert_eq!(p.quality, vec![255, 255]);
    // with the run's report, too: without point times no submap can be picked
    let slam = report(1_700_000_000.0, &[0.0], &[0.5]);
    let p = load_points(&path, Some(&slam), &Progress::new()).unwrap();
    assert!(!p.quality_known);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn loads_empty_las() {
    let dir = tmp("app-cloud-empty");
    let path = dir.join("leer.las");
    write_las(&path, &Cloud::default()).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
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
        if s.phase == PHASE_READ {
            handle.cancel();
        }
    });
    let r = load_points(&path, None, &progress);
    assert_eq!(r.err(), Some(StepError::Cancelled));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn load_reports_phase_and_total() {
    let dir = tmp("app-cloud-phase");
    let path = dir.join("color.las");
    write_las(&path, &small_cloud(true)).unwrap();
    let progress = Progress::new();
    load_points(&path, None, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, PHASE_READ);
    assert_eq!(s.total, Some(4));
    assert_eq!(s.done, 4);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn missing_file_is_a_german_error() {
    let r = load_points(Path::new("/gibt/es/nicht.las"), None, &Progress::new());
    match r {
        Err(StepError::Failed(m)) => assert!(m.contains("nicht.las"), "{m}"),
        other => panic!("{other:?}"),
    }
}

/// Foreign writers may leave the header bounds zero; the origin comes from the points
/// themselves (rounded centre of their real bounds), so UTM-scale coordinates keep
/// millimetres.
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
    // x 512 345.678…512 381.497, y 5 412 325.245…5 412 345.678, z 312.345…312.982
    assert_eq!(p.origin, [512_364.0, 5_412_335.0, 313.0]);
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
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
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(vec![[0, 0, 0], [255, 100, 1]]));
    assert_eq!(p.intensity, vec![0, 200]);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn las_point_source_becomes_scan() {
    let dir = tmp("app-cloud-sources");
    let path = dir.join("modell.las");
    write_las_sources(&path, &small_cloud(true), &[0, 1, 1, 0]).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.source, vec![0, 1, 1, 0]);
    // one scan only: no scans to tell apart
    write_las_sources(&path, &small_cloud(true), &[0; 4]).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert!(p.source.is_empty());
    write_las(&path, &small_cloud(false)).unwrap();
    assert!(load_points(&path, None, &Progress::new())
        .unwrap()
        .source
        .is_empty());
    std::fs::remove_dir_all(dir).unwrap();
}

/// A PLY through `driftless_export` and [`point_set`]: the origin is the rounded centre of
/// the points (not the first point), colours and intensities 8 bit again, `colored` 0 →
/// flag 1, a scan number only if it differs; no times, so the quality is unknown.
#[test]
fn ply_origin_is_rounded_centre() {
    let dir = tmp("app-ply-centre");
    let path = dir.join("color.ply");
    let cloud = small_cloud(true);
    write_ply(&path, &cloud).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 4);
    // bounds x 10…12.12, y 19.88…21, z 0.5…2
    assert_eq!(p.origin, [11.0, 20.0, 1.0]);
    for i in 0..4 {
        let a = absolute(&p, i);
        for k in 0..3 {
            let want = f64::from(cloud.xyz[i][k]);
            assert!((a[k] - want).abs() <= 1e-5, "point {i} axis {k}: {a:?}");
        }
    }
    assert_eq!(p.rgb.as_deref(), cloud.rgb.as_deref());
    assert_eq!(p.intensity, vec![0, 100, 200, 255]);
    // `colored` 0 → flag 1, as `user_data` of the LAS
    assert_eq!(p.flags, vec![0, 1, 0, 1]);
    assert_eq!(p.t, vec![0.0; 4]);
    assert_eq!(p.quality, vec![255; 4]);
    assert!(!p.quality_known);
    assert!(p.source.is_empty());

    write_ply_sources(&path, &cloud, &[0, 1, 2, 1]).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.source, vec![0, 1, 2, 1]);
    assert_eq!(p.rgb.as_deref(), cloud.rgb.as_deref());
    write_ply_sources(&path, &cloud, &[3; 4]).unwrap();
    assert!(load_points(&path, None, &Progress::new())
        .unwrap()
        .source
        .is_empty());

    // without colour and intensity: mid gray; no points: origin 0
    let mut gray = small_cloud(false);
    gray.intensity = vec![0.0; 4];
    write_ply(&path, &gray).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert!(p.rgb.is_none());
    assert_eq!(p.intensity, vec![MID_GRAY; 4]);
    assert_eq!(p.flags, vec![0; 4]);
    let empty = small_cloud(false).select(&[false; 4]).0;
    write_ply(&path, &empty).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert!(p.is_empty());
    assert_eq!(p.origin, [0.0; 3]);

    // `load_cloud` takes the reader by the extension, in any case
    write_ply(&path, &cloud).unwrap();
    let upper = dir.join("MODELL.PLY");
    std::fs::copy(&path, &upper).unwrap();
    let c = load_cloud(&upper, None, &Progress::new()).unwrap();
    assert_eq!(c.points.len(), 4);
    assert_eq!(c.tree.vertices.len(), 4);

    // a broken file is a German error naming it, a cancel stays a cancel
    let broken = dir.join("kaputt.ply");
    std::fs::write(&broken, b"ply\nformat ascii 1.0\n").unwrap();
    match load_points(&broken, None, &Progress::new()) {
        Err(StepError::Failed(m)) => {
            assert!(
                m.contains("kaputt.ply") && m.contains("PLY nicht lesbar: "),
                "{m}"
            );
        }
        other => panic!("expected an error, got {other:?}"),
    }
    let progress = Progress::new();
    progress.cancel();
    assert_eq!(
        load_points(&path, None, &progress).err(),
        Some(StepError::Cancelled)
    );
    std::fs::remove_dir_all(dir).unwrap();
}

/// [`point_set`] of points with times: `t` since the scan start of the report (or the first
/// point) and the quality of its submap; far coordinates keep millimetres around the centre.
#[test]
fn point_set_times_quality_and_far_origin() {
    let points = Points {
        xyz: vec![
            [500_000.25, 5_400_000.5, 100.0],
            [500_010.75, 5_400_003.5, 104.0],
        ],
        rgb: Some(vec![[65535, 256, 255], [0, 32768, 1]]),
        intensity: Some(vec![65535, 257]),
        source: Some(vec![2, 2]),
        time: Some(vec![1000.5, 1002.5]),
        flags: Some(vec![1, 0]),
        scans: Vec::new(),
    };
    let set = point_set(points.clone(), None);
    assert_eq!(set.origin, [500_006.0, 5_400_002.0, 102.0]);
    for (i, p) in points.xyz.iter().enumerate() {
        let a = absolute(&set, i);
        for k in 0..3 {
            assert!((a[k] - p[k]).abs() <= 1e-3, "point {i} axis {k}");
        }
    }
    assert_eq!(set.rgb, Some(vec![[255, 1, 0], [0, 128, 0]]));
    assert_eq!(set.intensity, vec![255, 1]);
    assert_eq!(set.t, vec![0.0, 2.0]);
    assert_eq!(set.quality, vec![255; 2]);
    assert!(!set.quality_known);
    assert_eq!(set.flags, vec![1, 0]);
    // one scan for all: no sources
    assert!(set.source.is_empty());

    let slam = report(1000.0, &[0.0, 2.0], &[0.2, 1.0]);
    let set = point_set(points, Some(&slam));
    assert_eq!(set.t, vec![0.5, 2.5]);
    assert_eq!(set.quality, vec![51, 255]);
    assert!(set.quality_known);
}

/// „Von Hand ausrichten“: the model without the points of the scan to place, the scan beside
/// it along x; its points carry its number, and the shift turns a shown point back into the
/// frame of the scan.
#[test]
fn pair_cloud_side_by_side() {
    // model: scans 0 and 2 over x 0…4 m, a point of scan 1 (misplaced) at x = 2
    let mut model = PointSet::from_xyz(
        [100.0, 50.0, 0.0],
        vec![
            [0.0, 0.0, 0.0],
            [4.0, 1.0, 1.0],
            [2.0, 0.5, 0.5],
            [1.0, 2.0, 2.0],
        ],
    );
    model.rgb = Some(vec![[10, 20, 30]; 4]);
    model.source = vec![0, 2, 1, 0];
    // the scan in its own frame: x from 99 m on
    let scan = PointSet::from_xyz([99.0, 50.0, 0.0], vec![[0.0, 0.0, 0.0], [3.0, 1.0, 0.0]]);
    let (set, shift) = combine_pair_points(model.clone(), 0, scan.clone(), 1);
    assert_eq!(set.len(), 5);
    assert_eq!(set.origin, [100.0, 50.0, 0.0]);
    assert_eq!(set.source, vec![0, 2, 0, 1, 1]);
    // the scan starts 1 m beyond the model's end (104 m)
    assert!(
        (shift[0] - (104.0 + PAIR_GAP - 99.0)).abs() < 1e-9,
        "{shift:?}"
    );
    assert_eq!(&shift[1..], &[0.0, 0.0]);
    let shown = |i: usize| [0, 1, 2].map(|k| set.origin[k] + f64::from(set.xyz[i][k]));
    let back = |i: usize| [0, 1, 2].map(|k| shown(i)[k] - shift[k]);
    assert_eq!(shown(3)[0], 104.0 + PAIR_GAP);
    assert_eq!(back(4), [102.0, 51.0, 0.0]);
    // the scan has no colours: grays of its intensity
    let rgb = set.rgb.as_ref().unwrap();
    assert_eq!(rgb.len(), 5);
    assert_eq!(rgb[4], [MID_GRAY; 3]);
    assert!(set.quality.iter().all(|&q| q == 255) && !set.quality_known);
    assert_eq!(set.flags.len(), 5);
    // a model of one scan has no sources: all its points are `lone`
    model.source = Vec::new();
    let (set, _) = combine_pair_points(model, 0, scan, 1);
    assert_eq!(set.source, vec![0, 0, 0, 0, 1, 1]);
}

/// The pair cloud from files: `modell.las` with its sources and the scan's cloud.
#[test]
fn pair_cloud_loads_from_files() {
    let dir = tmp("app-pair-cloud");
    let model = dir.join("modell.las");
    let mut cloud = small_cloud(true);
    write_las_sources(&model, &cloud, &[0, 1, 1, 0]).unwrap();
    cloud.rgb = None;
    cloud.colored = None;
    let scan = dir.join("clean.las");
    write_las(&scan, &cloud).unwrap();
    let done = load_pair_cloud(dir.join("paar"), &model, 0, &scan, 1, &Progress::new()).unwrap();
    let points = &done.cloud.points;
    assert_eq!(points.len(), 2 + 4);
    assert_eq!(points.source, vec![0, 0, 1, 1, 1, 1]);
    assert!(done.shift[0] > 0.0);
    assert_eq!(done.cloud.tree.vertices.len(), 6);
    let progress = Progress::new();
    progress.cancel();
    let err = load_pair_cloud(dir.join("paar"), &model, 0, &scan, 1, &progress).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
}

/// Points of two scans with 16-bit colours of 8-bit origin, intensities and the two scans'
/// names and poses, around `base`.
fn two_scan_points(base: [f64; 3], n: usize) -> Points {
    let xyz: Vec<[f64; 3]> = (0..n)
        .map(|i| {
            let f = i as f64;
            [
                base[0] + f * 0.731,
                base[1] - f * 0.417,
                base[2] + f * 0.013,
            ]
        })
        .collect();
    let pose = |x: f64| nalgebra::Isometry3::translation(x, 0.0, 0.0);
    Points {
        rgb: Some(
            (0..n)
                .map(|i| [(i % 256) as u16 * 257, 128 * 257, 255 * 257])
                .collect(),
        ),
        intensity: Some((0..n).map(|i| (i as u16 + 1) * 257).collect()),
        source: Some((0..n).map(|i| (i % 2) as u16).collect()),
        time: None,
        flags: None,
        scans: vec![
            ScanInfo {
                source: 0,
                name: "Erdgeschoss".into(),
                pose: pose(base[0]),
            },
            ScanInfo {
                source: 1,
                name: "Büro Süd".into(),
                pose: pose(base[0] + 5.0),
            },
        ],
        xyz,
    }
}

/// The absolute positions of `p`, sorted (readers may group points by scan).
fn sorted_absolute(p: &PointSet) -> Vec<[f64; 3]> {
    let mut v: Vec<[f64; 3]> = (0..p.len()).map(|i| absolute(p, i)).collect();
    v.sort_by(|a, b| a[0].total_cmp(&b[0]));
    v
}

#[test]
fn loads_laz() {
    let dir = tmp("app-cloud-laz");
    let path = dir.join("Büro Scan.laz");
    let pts = two_scan_points([10.0, 20.0, 1.0], 100);
    write_points(&path, PointFormat::Laz, &pts, &Progress::new()).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 100);
    for (i, want) in pts.xyz.iter().enumerate() {
        let a = absolute(&p, i);
        for k in 0..3 {
            assert!((a[k] - want[k]).abs() <= 1e-4, "point {i} axis {k}");
        }
    }
    let rgb = p.rgb.as_ref().unwrap();
    assert_eq!(rgb[3], [3, 128, 255]);
    assert_eq!(p.intensity[3], 4);
    assert_eq!(&p.source[..4], &[0, 1, 0, 1]);
    // `load_cloud` takes the same reader, also for `.LAZ`
    let upper = dir.join("WOLKE.LAZ");
    std::fs::copy(&path, &upper).unwrap();
    let c = load_cloud(&upper, None, &Progress::new()).unwrap();
    assert_eq!(c.points.len(), 100);
    std::fs::remove_dir_all(dir).unwrap();
}

/// An E57 with two scans: each point keeps its scan as its source, so „je Scan“ is offered.
#[test]
fn loads_e57_two_scans_as_sources() {
    let dir = tmp("app-cloud-e57");
    let path = dir.join("Büro Scan").join("Prüfung ä.e57");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    let pts = two_scan_points([3.0, -2.0, 1.0], 40);
    write_points(&path, PointFormat::E57, &pts, &Progress::new()).unwrap();
    let c = load_cloud(&path, None, &Progress::new()).unwrap();
    let p = &c.points;
    assert_eq!(p.len(), 40);
    let mut sources = p.source.clone();
    sources.sort_unstable();
    sources.dedup();
    assert_eq!(sources, vec![0, 1]);
    let has_rgb = p.rgb.is_some();
    assert!(has_rgb);
    assert!(color_modes(has_rgb, !p.source.is_empty()).contains(&ColorMode::Scan));
    // the same positions (E57 groups the points by scan)
    let mut want = pts.xyz.clone();
    want.sort_by(|a, b| a[0].total_cmp(&b[0]));
    for (got, want) in sorted_absolute(p).iter().zip(&want) {
        for k in 0..3 {
            assert!((got[k] - want[k]).abs() <= 1e-5, "{got:?} vs {want:?}");
        }
    }
    // each point keeps its own colour and scan: point i lies at x = 3 + 0.731 i
    let rgb = p.rgb.as_ref().unwrap();
    for (k, s) in p.source.iter().enumerate() {
        let i = ((absolute(p, k)[0] - 3.0) / 0.731).round() as usize;
        assert_eq!(rgb[k], [i as u8, 128, 255], "point {i}");
        assert_eq!(usize::from(*s), i % 2, "point {i}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn loads_pcd_ascii() {
    let dir = tmp("app-cloud-pcd");
    let path = dir.join("fremd.pcd");
    std::fs::write(
        &path,
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\n\
         SIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\n\
         VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n\
         1.5 2 3 16711680\n2.5 2 3 65280\nnan nan nan 0\n",
    )
    .unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    // the point without a position is skipped
    assert_eq!(p.len(), 2);
    assert_eq!(p.origin, [2.0, 2.0, 3.0]);
    assert_eq!(absolute(&p, 1), [2.5, 2.0, 3.0]);
    assert_eq!(p.rgb, Some(vec![[255, 0, 0], [0, 255, 0]]));
    assert_eq!(p.intensity, vec![MID_GRAY; 2]);
    assert!(!p.quality_known);
    std::fs::remove_dir_all(dir).unwrap();
}

/// An E57 in UTM coordinates: the origin in whole metres, every point within 1 mm.
#[test]
fn far_e57_opens_with_mm_precision() {
    let dir = tmp("app-cloud-e57-far");
    let path = dir.join("fern.e57");
    let base = [512_345.678, 5_412_345.678, 312.345];
    let pts = two_scan_points(base, 200);
    write_points(&path, PointFormat::E57, &pts, &Progress::new()).unwrap();
    let p = load_points(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.len(), 200);
    for (k, o) in p.origin.iter().enumerate() {
        assert_eq!(o.fract(), 0.0, "origin axis {k} in whole metres");
    }
    // rounded centre of the real bounds
    let (min, max) = pts.bounds().unwrap();
    let centre: [f64; 3] = std::array::from_fn(|k| ((min[k] + max[k]) / 2.0).round());
    assert_eq!(p.origin, centre);
    let mut want = pts.xyz.clone();
    want.sort_by(|a, b| a[0].total_cmp(&b[0]));
    for (got, want) in sorted_absolute(&p).iter().zip(&want) {
        for k in 0..3 {
            assert!(
                (got[k] - want[k]).abs() <= 0.001,
                "{} vs {}",
                got[k],
                want[k]
            );
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// A PCD saved compressed is not read: the reader's German message, naming the file.
#[test]
fn pcd_compressed_shows_message() {
    let dir = tmp("app-cloud-pcd-compressed");
    let path = dir.join("gepackt.pcd");
    let mut bytes = b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n\
         WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n"
        .to_vec();
    bytes.extend([12, 0, 0, 0, 12, 0, 0, 0]);
    bytes.extend([0u8; 12]);
    std::fs::write(&path, bytes).unwrap();
    let want = "PCD „binary_compressed“ wird nicht unterstützt – in CloudCompare oder PCL als \
                „binary“ speichern";
    for r in [
        load_points(&path, None, &Progress::new()).map(|_| ()),
        load_cloud(&path, None, &Progress::new()).map(|_| ()),
    ] {
        match r {
            Err(StepError::Failed(m)) => {
                assert!(m.contains("gepackt.pcd") && m.contains(want), "{m}");
            }
            other => panic!("expected the message, got {other:?}"),
        }
    }
    // an unknown extension names what can be read
    let txt = dir.join("notiz.txt");
    std::fs::write(&txt, "x").unwrap();
    match load_points(&txt, None, &Progress::new()) {
        Err(StepError::Failed(m)) => assert!(m.contains(".e57") && m.contains(".pcd"), "{m}"),
        other => panic!("{other:?}"),
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// `n` points with every column around `base`: 16-bit colours, intensities, two scans, times
/// from 1.7e9 s and flags, in a fixed pseudo-random order.
fn every_column(base: [f64; 3], n: usize) -> Points {
    let mut s = 0x2545_f491_4f6c_dd1du64;
    let mut next = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        s
    };
    let mut xyz = Vec::with_capacity(n);
    let mut rgb = Vec::with_capacity(n);
    let mut intensity = Vec::with_capacity(n);
    for _ in 0..n {
        let r = next();
        xyz.push([
            base[0] + (r % 40_000) as f64 * 0.000_5 - 10.0,
            base[1] + (r >> 16) as f64 % 20_000.0 * 0.000_5 - 5.0,
            base[2] + (r >> 32) as f64 % 6_000.0 * 0.000_5,
        ]);
        let c = next();
        rgb.push([c as u16, (c >> 16) as u16, (c >> 32) as u16]);
        intensity.push((c >> 48) as u16);
    }
    let pose = |x: f64| nalgebra::Isometry3::translation(x, 0.0, 0.0);
    Points {
        xyz,
        rgb: Some(rgb),
        intensity: Some(intensity),
        source: Some((0..n).map(|i| u16::from(i % 7 == 0)).collect()),
        time: Some((0..n).map(|i| 1.7e9 + i as f64 * 1e-4).collect()),
        flags: Some((0..n).map(|i| (i % 3) as u8).collect()),
        scans: vec![
            ScanInfo {
                source: 0,
                name: "A".into(),
                pose: pose(0.0),
            },
            ScanInfo {
                source: 1,
                name: "B".into(),
                pose: pose(1.0),
            },
        ],
    }
}

/// LAS, LAZ and E57 go block by block straight into the [`PointSet`]; over several blocks,
/// with times, a SLAM report and far coordinates the result is the same as [`point_set`] of
/// the whole cloud. The E57 states no bounds, so its origin starts at the first point and is
/// shifted at the end.
#[test]
fn streamed_load_matches_point_set() {
    let dir = tmp("app-cloud-streamed");
    let n = 2 * driftless_export::SINK_BLOCK + 17;
    let slam = report(1.7e9 + 1.0, &[0.0, 5.0, 10.0], &[0.2, 0.6, 1.0]);
    for (base, name) in [
        ([500_000.3, 5_400_000.7, 300.2], "utm"),
        ([12.0, -3.4, 0.5], "nah"),
    ] {
        let pts = every_column(base, n);
        for format in [PointFormat::Las, PointFormat::Laz, PointFormat::E57] {
            let path = dir.join(format!("{name}.{}", format.extension()));
            write_points(&path, format, &pts, &Progress::new()).unwrap();
            for slam in [None, Some(&slam)] {
                let want = point_set(
                    driftless_export::read_points(&path, &Progress::new()).unwrap(),
                    slam,
                );
                let got = load_points(&path, slam, &Progress::new()).unwrap();
                let what = format!("{name}.{}", format.extension());
                assert_eq!(got.origin, want.origin, "{what}");
                assert_eq!(got.len(), want.len(), "{what}");
                for (a, b) in got.xyz.iter().zip(&want.xyz) {
                    for k in 0..3 {
                        assert!((a[k] - b[k]).abs() <= 2e-5, "{what}: {a:?} vs {b:?}");
                    }
                }
                assert_eq!(got.rgb, want.rgb, "{what}");
                assert_eq!(got.intensity, want.intensity, "{what}");
                assert_eq!(got.t, want.t, "{what}");
                assert_eq!(got.quality, want.quality, "{what}");
                assert_eq!(got.quality_known, want.quality_known, "{what}");
                assert_eq!(got.flags, want.flags, "{what}");
                assert_eq!(got.source, want.source, "{what}");
            }
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}
