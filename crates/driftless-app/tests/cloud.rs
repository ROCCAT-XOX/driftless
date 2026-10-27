//! Loading a LAS or PLY cloud into a [`PointSet`].

use driftless_app::cloud::{
    combine_pair_points, load_cloud, load_las, load_pair_cloud, load_ply, PointSet, MID_GRAY,
    PAIR_GAP,
};
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

#[test]
fn las_point_source_becomes_scan() {
    let dir = tmp("app-cloud-sources");
    let path = dir.join("modell.las");
    write_las_sources(&path, &small_cloud(true), &[0, 1, 1, 0]).unwrap();
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert_eq!(p.source, vec![0, 1, 1, 0]);
    // one scan only: no scans to tell apart
    write_las_sources(&path, &small_cloud(true), &[0; 4]).unwrap();
    let p = load_las(&path, None, &Progress::new()).unwrap();
    assert!(p.source.is_empty());
    write_las(&path, &small_cloud(false)).unwrap();
    assert!(load_las(&path, None, &Progress::new())
        .unwrap()
        .source
        .is_empty());
    std::fs::remove_dir_all(dir).unwrap();
}

/// `message` of a failed load, or a panic naming what came instead.
fn failed(r: Result<PointSet, StepError>) -> String {
    match r {
        Err(StepError::Failed(m)) => m,
        other => panic!("expected an error, got {other:?}"),
    }
}

fn ply_error(dir: &Path, name: &str, bytes: &[u8]) -> String {
    let path = dir.join(name);
    std::fs::write(&path, bytes).unwrap();
    let m = failed(load_ply(&path, &Progress::new()));
    assert!(m.starts_with("PLY nicht lesbar: "), "{name}: {m}");
    m
}

#[test]
fn ply_round_trip_from_write_ply() {
    let dir = tmp("app-ply-roundtrip");
    let path = dir.join("color.ply");
    let cloud = small_cloud(true);
    write_ply(&path, &cloud).unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 4);
    // the first point in whole metres
    assert_eq!(p.origin, [10.0, 20.0, 1.0]);
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
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.source, vec![0, 1, 2, 1]);
    assert_eq!(p.rgb.as_deref(), cloud.rgb.as_deref());
    assert_eq!(p.flags, vec![0, 1, 0, 1]);
    write_ply_sources(&path, &cloud, &[3; 4]).unwrap();
    assert!(load_ply(&path, &Progress::new()).unwrap().source.is_empty());

    // `load_cloud` takes the reader by the extension, in any case
    let upper = dir.join("MODELL.PLY");
    std::fs::copy(&path, &upper).unwrap();
    let c = load_cloud(&upper, None, &Progress::new()).unwrap();
    assert_eq!(c.points.len(), 4);
    assert_eq!(c.tree.vertices.len(), 4);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_ascii_with_float_color() {
    let dir = tmp("app-ply-ascii");
    let path = dir.join("fremd.ply");
    let text = "ply\r\nformat ascii 1.0\r\ncomment von Hand\r\nobj_info test\r\n\
                element vertex 3\r\nproperty float32 x\r\nproperty float32 y\r\n\
                property float32 z\r\nproperty float nx\r\nproperty float red\r\n\
                property float green\r\nproperty float blue\r\n\
                property float scalar_Intensity\r\nend_header\r\n\
                1.5 2 -3 0.1 1 0 0 10\r\n\
                -1e-1 0.25 4 0 0 0.5 0 20.5\r\n\
                7 8 9 0 0 0 1 255\r\n";
    std::fs::write(&path, text).unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 3);
    assert_eq!(p.origin, [2.0, 2.0, -3.0]);
    let want = [[1.5, 2.0, -3.0], [-0.1, 0.25, 4.0], [7.0, 8.0, 9.0]];
    for (i, w) in want.iter().enumerate() {
        let a = absolute(&p, i);
        for k in 0..3 {
            assert!((a[k] - w[k]).abs() < 1e-6, "point {i}: {a:?}");
        }
    }
    // all within 0–1: scaled to 0–255
    assert_eq!(p.rgb, Some(vec![[255, 0, 0], [0, 128, 0], [0, 0, 255]]));
    assert_eq!(p.intensity, vec![10, 21, 255]);
    assert_eq!(p.flags, vec![0; 3]);
    std::fs::remove_dir_all(dir).unwrap();
}

/// Float colours of a foreign file in 0–255: one value above 1 tells the range, so they are
/// taken as they are (rounded, clamped), not ×255.
#[test]
fn ply_float_color_0_255() {
    let dir = tmp("app-ply-float-255");
    let path = dir.join("fremd.ply");
    let text = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\n\
                property float z\nproperty double red\nproperty double green\n\
                property double blue\nend_header\n\
                0 0 0 255 128.4 0\n\
                1 0 0 1 0.6 0\n\
                2 0 0 300 -4 nan\n\
                3 0 0 0.2 0 1\n";
    std::fs::write(&path, text).unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(
        p.rgb,
        Some(vec![[255, 128, 0], [1, 1, 0], [255, 0, 0], [0, 0, 1]])
    );
    std::fs::remove_dir_all(dir).unwrap();
}

/// Binary PLY data of `rows`, each value written big- or little-endian as its type says.
fn binary_rows(rows: &[Vec<Value>], big: bool) -> Vec<u8> {
    let mut out = Vec::new();
    for row in rows {
        for v in row {
            match (*v, big) {
                (Value::F64(x), true) => out.extend(x.to_be_bytes()),
                (Value::F64(x), false) => out.extend(x.to_le_bytes()),
                (Value::F32(x), true) => out.extend(x.to_be_bytes()),
                (Value::F32(x), false) => out.extend(x.to_le_bytes()),
                (Value::U16(x), true) => out.extend(x.to_be_bytes()),
                (Value::U16(x), false) => out.extend(x.to_le_bytes()),
                (Value::I32(x), true) => out.extend(x.to_be_bytes()),
                (Value::I32(x), false) => out.extend(x.to_le_bytes()),
                (Value::U8(x), _) => out.push(x),
            }
        }
    }
    out
}

#[derive(Debug, Clone, Copy)]
enum Value {
    F64(f64),
    F32(f32),
    U16(u16),
    I32(i32),
    U8(u8),
}

#[test]
fn ply_big_endian_double() {
    use Value::*;
    let dir = tmp("app-ply-be");
    let path = dir.join("utm.ply");
    let base = [512_345.678, 5_412_345.678, 312.345];
    let header = "ply\nformat binary_big_endian 1.0\nelement vertex 3\n\
                  property double x\nproperty float64 y\nproperty double z\n\
                  property uchar red\nproperty uint8 green\nproperty uchar blue\n\
                  property ushort intensity\nproperty ushort scan\nend_header\n";
    let rows: Vec<Vec<Value>> = (0..3)
        .map(|i| {
            let f = f64::from(i);
            vec![
                F64(base[0] + f * 0.731),
                F64(base[1] - f * 0.417),
                F64(base[2] + f * 0.013),
                U8(10 * i as u8),
                U8(20),
                U8(30),
                U16(1000 * (i as u16 + 1)),
                U16(i as u16 % 2),
            ]
        })
        .collect();
    let mut bytes = header.as_bytes().to_vec();
    bytes.extend(binary_rows(&rows, true));
    std::fs::write(&path, bytes).unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 3);
    for i in 0..3 {
        let f = i as f64;
        let want = [
            base[0] + f * 0.731,
            base[1] - f * 0.417,
            base[2] + f * 0.013,
        ];
        let a = absolute(&p, i);
        for k in 0..3 {
            assert!((a[k] - want[k]).abs() <= 0.001, "point {i} axis {k}");
        }
    }
    assert_eq!(p.rgb, Some(vec![[0, 20, 30], [10, 20, 30], [20, 20, 30]]));
    // above 255: scaled so that the brightest is 255
    assert_eq!(p.intensity, vec![85, 170, 255]);
    assert_eq!(p.source, vec![0, 1, 0]);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_without_color() {
    use Value::*;
    let dir = tmp("app-ply-nocolor");
    let path = dir.join("grau.ply");
    let mut bytes = b"ply\nformat binary_little_endian 1.0\nelement vertex 2\n\
                      property float x\nproperty float y\nproperty float z\nend_header\n"
        .to_vec();
    bytes.extend(binary_rows(
        &[
            vec![F32(1.0), F32(2.0), F32(3.0)],
            vec![F32(4.0), F32(5.0), F32(6.0)],
        ],
        false,
    ));
    std::fs::write(&path, bytes).unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 2);
    assert!(p.rgb.is_none());
    assert_eq!(p.intensity, vec![MID_GRAY; 2]);
    assert!(p.source.is_empty());
    assert_eq!(absolute(&p, 1), [4.0, 5.0, 6.0]);

    // no points at all
    std::fs::write(
        &path,
        "ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\n\
         property float z\nend_header\n",
    )
    .unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert!(p.is_empty());
    assert_eq!(p.origin, [0.0; 3]);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_skips_faces() {
    use Value::*;
    let dir = tmp("app-ply-faces");
    // a mesh as many programs write it: vertices, then faces
    let path = dir.join("netz.ply");
    std::fs::write(
        &path,
        "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
         property float z\nproperty list uchar int vertex_indices\nelement face 1\n\
         property list uchar int vertex_indices\nend_header\n\
         0 0 0 2 7 8\n1 0 0 0\n0 1 0 1 5\n3 0 1 2\n",
    )
    .unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 3);
    assert_eq!(absolute(&p, 2), [0.0, 1.0, 0.0]);

    // binary, faces (with a list) and another element before the vertices
    let path = dir.join("vorne.ply");
    let mut bytes = b"ply\nformat binary_little_endian 1.0\nelement face 2\n\
                      property list uchar int vertex_indices\nproperty uchar flag\n\
                      element material 1\nproperty float shine\n\
                      element vertex 2\nproperty float x\nproperty float y\n\
                      property float z\nproperty uchar colored\nend_header\n"
        .to_vec();
    bytes.extend(binary_rows(
        &[
            vec![U8(3), I32(0), I32(1), I32(2), U8(9)],
            vec![U8(0), U8(9)],
            vec![F32(0.5)],
            vec![F32(1.0), F32(2.0), F32(3.0), U8(1)],
            vec![F32(-1.0), F32(-2.0), F32(-3.0), U8(0)],
        ],
        false,
    ));
    std::fs::write(&path, bytes).unwrap();
    let p = load_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 2);
    assert_eq!(absolute(&p, 0), [1.0, 2.0, 3.0]);
    assert_eq!(absolute(&p, 1), [-1.0, -2.0, -3.0]);
    assert_eq!(p.flags, vec![0, 1]);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_broken_header_is_error() {
    let dir = tmp("app-ply-broken");
    let good = "ply\nformat binary_little_endian 1.0\nelement vertex 2\n\
                property float x\nproperty float y\nproperty float z\nend_header\n";
    let mut valid = good.as_bytes().to_vec();
    valid.extend([0u8; 24]);
    // every cut of a valid file: an error (or the points), never a panic
    for n in 0..valid.len() {
        let path = dir.join("cut.ply");
        std::fs::write(&path, &valid[..n]).unwrap();
        assert!(load_ply(&path, &Progress::new()).is_err(), "cut at {n}");
    }
    ply_error(&dir, "leer.ply", b"");
    ply_error(&dir, "kopf.ply", &good.as_bytes()[..good.len() - 12]);
    ply_error(&dir, "fremd.ply", b"PK\x03\x04 zip");
    ply_error(
        &dir,
        "format.ply",
        good.replace("binary_little_endian", "binary_middle")
            .as_bytes(),
    );
    ply_error(&dir, "version.ply", good.replace("1.0", "2.0").as_bytes());
    ply_error(
        &dir,
        "typ.ply",
        good.replace("float y", "quad y").as_bytes(),
    );
    ply_error(
        &dir,
        "ohne-z.ply",
        good.replace("property float z\n", "").as_bytes(),
    );
    ply_error(
        &dir,
        "ohne-punkte.ply",
        good.replace("vertex", "face").as_bytes(),
    );
    ply_error(
        &dir,
        "zeile.ply",
        good.replace("element vertex 2", "element vertex")
            .as_bytes(),
    );
    ply_error(
        &dir,
        "vor-element.ply",
        b"ply\nformat ascii 1.0\nproperty float x\nend_header\n",
    );
    // a count larger than the data, also a huge one: an error, no allocation of it
    let mut more = good.replace("vertex 2", "vertex 3").into_bytes();
    more.extend([0u8; 24]);
    let m = ply_error(&dir, "mehr.ply", &more);
    assert!(m.contains("Anzahl"), "{m}");
    for count in ["18446744073709551615", "99999999999999999999999", "-1"] {
        let mut huge = good
            .replace("vertex 2", &format!("vertex {count}"))
            .into_bytes();
        huge.extend([0u8; 24]);
        ply_error(&dir, "riesig.ply", &huge);
        let ascii = format!(
            "ply\nformat ascii 1.0\nelement vertex {count}\nproperty float x\n\
             property float y\nproperty float z\nend_header\n1 2 3\n"
        );
        ply_error(&dir, "riesig-ascii.ply", ascii.as_bytes());
    }
    // too few values in ASCII, or one that is no number
    ply_error(
        &dir,
        "kurz.ply",
        b"ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n\
          property float z\nend_header\n1 2 3\n4 5\n",
    );
    ply_error(
        &dir,
        "wort.ply",
        b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\n\
          property float z\nend_header\n1 zwei 3\n",
    );
    // a face list longer than the file before the vertices
    let mut list = b"ply\nformat binary_little_endian 1.0\nelement face 1\n\
                     property list uint int vertex_indices\nelement vertex 1\n\
                     property float x\nproperty float y\nproperty float z\nend_header\n"
        .to_vec();
    list.extend(u32::MAX.to_le_bytes());
    list.extend([0u8; 12]);
    ply_error(&dir, "liste.ply", &list);
    // a missing file names it
    let m = failed(load_ply(&dir.join("weg.ply"), &Progress::new()));
    assert!(
        m.starts_with("PLY nicht lesbar: ") && m.contains("weg.ply"),
        "{m}"
    );
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_load_reports_phase_and_cancels() {
    let dir = tmp("app-ply-cancel");
    let path = dir.join("color.ply");
    write_ply(&path, &small_cloud(true)).unwrap();
    let progress = Progress::new();
    load_ply(&path, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, "Wolke laden");
    assert_eq!((s.total, s.done), (Some(4), 4));

    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == "Wolke laden" {
            handle.cancel();
        }
    });
    assert_eq!(load_ply(&path, &progress).err(), Some(StepError::Cancelled));
    std::fs::remove_dir_all(dir).unwrap();
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
