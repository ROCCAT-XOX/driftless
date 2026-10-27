//! LAS 1.4 and LAZ: round trip, point formats, foreign 8-bit files, far coordinates and the
//! atomic write.

use driftless_export::las::{las_offset, read_las, write_las, LAS_SCALE};
use driftless_export::testutil::synthetic_points;
use driftless_export::{
    read_points, write_points, ExportError, Layout, PointFormat, PointSource, Points,
};
use driftless_io::progress::Progress;
use std::path::{Path, PathBuf};

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-export-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

const ALL: Layout = Layout {
    rgb: true,
    intensity: true,
    source: true,
    time: true,
    flags: true,
};

fn header(path: &Path) -> las::Header {
    las::Reader::from_path(path).unwrap().header().clone()
}

/// Files in `dir` whose name ends in `.tmp`.
fn leftovers(dir: &Path) -> Vec<PathBuf> {
    std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.to_string_lossy().ends_with(".tmp"))
        .collect()
}

fn assert_xyz_close(a: &Points, b: &Points, tol: f64) {
    assert_eq!(a.len(), b.len());
    for (p, q) in a.xyz.iter().zip(&b.xyz) {
        for k in 0..3 {
            assert!((p[k] - q[k]).abs() <= tol, "{p:?} vs {q:?}");
        }
    }
}

#[test]
fn las_round_trip_all_columns() {
    let dir = temp_dir("round-trip");
    let path = dir.join("wolke.las");
    let pts = synthetic_points(10_000, 1, ALL);
    write_las(&path, &pts, false, &Progress::new()).unwrap();

    let h = header(&path);
    assert_eq!((h.version().major, h.version().minor), (1, 4));
    assert_eq!(h.point_format().to_u8().unwrap(), 7);
    assert!(!h.point_format().is_compressed);
    assert_eq!(h.number_of_points(), 10_000);
    assert_eq!(h.generating_software(), "driftless");
    assert_eq!(h.date(), None);
    let offset = las_offset(pts.bounds());
    let t = h.transforms();
    assert_eq!([t.x.scale, t.y.scale, t.z.scale], [LAS_SCALE; 3]);
    assert_eq!([t.x.offset, t.y.offset, t.z.offset], offset);

    let back = read_las(&path, &Progress::new()).unwrap();
    assert_xyz_close(&pts, &back, 0.5 * LAS_SCALE + 1e-9);
    assert_eq!(back.rgb, pts.rgb);
    assert_eq!(back.intensity, pts.intensity);
    assert_eq!(back.source, pts.source);
    assert_eq!(back.time, pts.time);
    assert_eq!(back.flags, pts.flags);
    assert_eq!(back.layout(), ALL);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn point_format_follows_columns() {
    let dir = temp_dir("formats");
    let path = dir.join("f.las");
    let base = Layout {
        intensity: true,
        source: true,
        flags: true,
        ..Layout::default()
    };
    for (rgb, time, want) in [
        (true, true, 7),
        (false, true, 6),
        (true, false, 2),
        (false, false, 0),
    ] {
        let layout = Layout { rgb, time, ..base };
        let pts = synthetic_points(100, 2, layout);
        write_las(&path, &pts, false, &Progress::new()).unwrap();
        assert_eq!(header(&path).point_format().to_u8().unwrap(), want);
        let back = read_las(&path, &Progress::new()).unwrap();
        assert_eq!(back.rgb.is_some(), rgb);
        assert_eq!(back.time.is_some(), time, "no made-up times");
        assert_eq!(back.intensity, pts.intensity);
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn laz_is_compressed_and_reads_back() {
    let dir = temp_dir("laz");
    let pts = synthetic_points(100_000, 3, ALL);
    let las_path = dir.join("wolke.las");
    let laz_path = dir.join("wolke.laz");
    write_points(&las_path, PointFormat::Las, &pts, &Progress::new()).unwrap();
    write_points(&laz_path, PointFormat::Laz, &pts, &Progress::new()).unwrap();
    assert!(header(&laz_path).point_format().is_compressed);
    let las_len = std::fs::metadata(&las_path).unwrap().len();
    let laz_len = std::fs::metadata(&laz_path).unwrap().len();
    assert!(laz_len < las_len, "laz {laz_len} B, las {las_len} B");

    let a = read_las(&las_path, &Progress::new()).unwrap();
    let b = read_las(&laz_path, &Progress::new()).unwrap();
    assert_eq!(a, b);
    assert!(leftovers(&dir).is_empty());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn formats_follow_the_extension() {
    assert_eq!(
        PointFormat::from_path(Path::new("a/b.LAS")),
        Some(PointFormat::Las)
    );
    assert_eq!(
        PointFormat::from_path(Path::new("Büro Scan/x.laz")),
        Some(PointFormat::Laz)
    );
    assert_eq!(PointFormat::from_path(Path::new("x.txt")), None);
    assert_eq!(PointFormat::from_path(Path::new("las")), None);
    assert_eq!(PointFormat::Las.extension(), "las");
    assert_eq!(PointFormat::Laz.extension(), "laz");
}

/// A LAS 1.2 file in format 2 (colour, no time) with the given colours and intensities as
/// they are, as a foreign writer that stores 8-bit values would.
fn write_raw_las(path: &Path, rgb: &[[u16; 3]], intensity: &[u16]) {
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(2).unwrap();
    let mut w = las::Writer::from_path(path, builder.into_header().unwrap()).unwrap();
    for (i, (c, v)) in rgb.iter().zip(intensity).enumerate() {
        w.write_point(las::Point {
            x: i as f64,
            y: 0.5,
            z: -1.0,
            intensity: *v,
            color: Some(las::Color::new(c[0], c[1], c[2])),
            ..Default::default()
        })
        .unwrap();
    }
    w.close().unwrap();
}

#[test]
fn eight_bit_las_widened() {
    let dir = temp_dir("8bit");
    let path = dir.join("fremd.las");
    write_raw_las(&path, &[[255, 128, 0], [10, 20, 30]], &[200, 7]);
    let p = read_las(&path, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(vec![[65535, 32896, 0], [2570, 5140, 7710]]));
    assert_eq!(p.intensity, Some(vec![200 * 257, 7 * 257]));
    assert_eq!(p.time, None);
    assert_eq!(p.xyz[1], [1.0, 0.5, -1.0]);
    std::fs::remove_dir_all(&dir).unwrap();
}

/// 16-bit files stay as they are, even for dark points, once any value exceeds 255.
#[test]
fn sixteen_bit_dark_stays() {
    let dir = temp_dir("16bit");
    let path = dir.join("dunkel.las");
    let rgb = [[255, 128, 0], [65_535, 25_700, 257]];
    write_raw_las(&path, &rgb, &[200, 51_400]);
    let p = read_las(&path, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(rgb.to_vec()));
    assert_eq!(p.intensity, Some(vec![200, 51_400]));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn las_far_coordinates() {
    let dir = temp_dir("far");
    let path = dir.join("utm.laz");
    let base = [500_000.123_4, 5_400_000.567_8, 312.0];
    let pts = Points {
        xyz: (0..1000)
            .map(|i| {
                let d = f64::from(i) * 0.0137;
                [base[0] + d, base[1] - 2.0 * d, base[2] + 0.5 * d]
            })
            .collect(),
        ..Points::default()
    };
    write_points(&path, PointFormat::Laz, &pts, &Progress::new()).unwrap();
    let h = header(&path);
    assert_eq!(h.transforms().x.offset, 500_007.0);
    assert_eq!(h.transforms().y.offset, 5_399_987.0);
    let back = read_points(&path, &Progress::new()).unwrap();
    assert_xyz_close(&pts, &back, 1e-4);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn las_too_wide_is_refused() {
    let dir = temp_dir("wide");
    let path = dir.join("weit.las");
    let pts = Points {
        xyz: vec![[0.0; 3], [500_000.0, 0.0, 0.0]],
        ..Points::default()
    };
    match write_las(&path, &pts, false, &Progress::new()) {
        Err(ExportError::Format { msg, .. }) => {
            assert!(
                msg.contains("zu weit ausgedehnt für LAS mit 0,1 mm"),
                "{msg}"
            )
        }
        other => panic!("expected a format error, got {other:?}"),
    }
    let nan = Points {
        xyz: vec![[0.0, f64::NAN, 0.0]],
        ..Points::default()
    };
    assert!(matches!(
        write_las(&path, &nan, false, &Progress::new()),
        Err(ExportError::Format { .. })
    ));
    assert!(!path.exists());
    assert!(leftovers(&dir).is_empty());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn las_deterministic() {
    let dir = temp_dir("det");
    let pts = synthetic_points(5_000, 4, ALL);
    for ext in ["las", "laz"] {
        let a = dir.join(format!("a.{ext}"));
        let b = dir.join(format!("b.{ext}"));
        let format = PointFormat::from_path(&a).unwrap();
        write_points(&a, format, &pts, &Progress::new()).unwrap();
        write_points(&b, format, &pts, &Progress::new()).unwrap();
        assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn las_empty() {
    let dir = temp_dir("empty");
    for ext in ["las", "laz"] {
        let path = dir.join(format!("leer.{ext}"));
        let format = PointFormat::from_path(&path).unwrap();
        write_points(&path, format, &Points::default(), &Progress::new()).unwrap();
        let h = header(&path);
        assert_eq!(h.number_of_points(), 0);
        assert_eq!(h.transforms().x.offset, 0.0);
        let back = read_points(&path, &Progress::new()).unwrap();
        assert!(back.is_empty());
        assert_eq!(back.bounds(), None);
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn read_points_drops_empty_columns() {
    let dir = temp_dir("drop");
    let path = dir.join("karte.las");
    let mut pts = synthetic_points(1_000, 5, ALL);
    pts.intensity = Some(vec![0; 1_000]);
    pts.flags = Some(vec![0; 1_000]);
    pts.source = Some(vec![3; 1_000]);
    write_las(&path, &pts, false, &Progress::new()).unwrap();
    let back = read_points(&path, &Progress::new()).unwrap();
    assert_eq!(
        back.layout(),
        Layout {
            rgb: true,
            time: true,
            ..Layout::default()
        }
    );
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn failed_write_leaves_no_tmp() {
    let dir = temp_dir("fail");
    // The target is a non-empty folder: the final rename fails.
    let path = dir.join("wolke.las");
    std::fs::create_dir_all(path.join("x")).unwrap();
    let pts = synthetic_points(1_000, 6, ALL);
    for compress in [false, true] {
        assert!(matches!(
            write_las(&path, &pts, compress, &Progress::new()),
            Err(ExportError::Io { .. })
        ));
        assert!(leftovers(&dir).is_empty());
        assert!(path.join("x").is_dir());
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn cancel_before_write() {
    let dir = temp_dir("cancel");
    let path = dir.join("wolke.laz");
    let progress = Progress::new();
    progress.cancel();
    let pts = synthetic_points(1_000, 7, ALL);
    assert!(matches!(
        write_las(&path, &pts, true, &progress),
        Err(ExportError::Cancelled)
    ));
    assert!(!path.exists());
    assert!(leftovers(&dir).is_empty());
    std::fs::remove_dir_all(&dir).unwrap();
}

/// A cancel while the points are written keeps an existing target as it was.
#[test]
fn cancel_while_writing_keeps_old_file() {
    let dir = temp_dir("cancel-mid");
    let path = dir.join("wolke.las");
    std::fs::write(&path, b"alt").unwrap();
    let pts = synthetic_points(2 * driftless_export::BLOCK + 10, 8, Layout::default());
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == driftless_export::PHASE_WRITE && s.done > 0 {
            handle.cancel();
        }
    });
    assert!(matches!(
        write_las(&path, &pts, false, &progress),
        Err(ExportError::Cancelled)
    ));
    assert_eq!(std::fs::read(&path).unwrap(), b"alt");
    assert!(leftovers(&dir).is_empty());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn reading_reports_progress_and_cancel() {
    let dir = temp_dir("read");
    let path = dir.join("wolke.las");
    let pts = synthetic_points(3_000, 9, ALL);
    write_las(&path, &pts, false, &Progress::new()).unwrap();
    let progress = Progress::new();
    read_las(&path, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, driftless_export::PHASE_READ);
    assert_eq!((s.done, s.total), (3_000, Some(3_000)));
    progress.cancel();
    assert!(matches!(
        read_las(&path, &progress),
        Err(ExportError::Cancelled)
    ));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn foreign_garbage_is_a_clear_error() {
    let dir = temp_dir("garbage");
    let path = dir.join("kaputt.las");
    std::fs::write(&path, b"kein LAS").unwrap();
    match read_points(&path, &Progress::new()) {
        Err(ExportError::Format { path: p, msg }) => {
            assert!(p.ends_with("kaputt.las"));
            assert!(msg.contains("LAS"), "{msg}");
        }
        other => panic!("expected a format error, got {other:?}"),
    }
    assert!(matches!(
        read_points(&dir.join("fehlt.las"), &Progress::new()),
        Err(ExportError::Io { .. })
    ));
    assert!(matches!(
        read_points(&dir.join("x.xyz"), &Progress::new()),
        Err(ExportError::Format { .. })
    ));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn point_source_of_points() {
    let pts = synthetic_points(50, 10, ALL);
    assert_eq!(PointSource::len(&pts), 50);
    assert_eq!(PointSource::layout(&pts), ALL);
    let i = 17;
    let v = pts.intensity.as_ref().unwrap()[i];
    assert_eq!(pts.intensity(i), v as f32 / 257.0);
    assert_eq!(pts.rgb(i), pts.rgb.as_ref().unwrap()[i]);
    assert_eq!(pts.source(i), pts.source.as_ref().unwrap()[i]);
    assert_eq!(driftless_export::bounds_of(&pts), pts.bounds());
    assert!(!driftless_export::needs_double(&pts));
    let far = Points {
        xyz: vec![[0.0, -8192.0, 0.0]],
        ..Points::default()
    };
    assert!(driftless_export::needs_double(&far));
    // Missing columns read as zero.
    let plain = Points {
        xyz: vec![[1.0, 2.0, 3.0]],
        ..Points::default()
    };
    assert_eq!(
        (
            plain.rgb(0),
            plain.intensity(0),
            plain.source(0),
            plain.time(0),
            plain.flags(0)
        ),
        ([0; 3], 0.0, 0, 0.0, 0)
    );
}

#[test]
fn drop_empty_columns_keeps_what_says_something() {
    let mut p = synthetic_points(10, 11, ALL);
    p.drop_empty_columns();
    assert_eq!(p.layout(), ALL);
    p.intensity = Some(vec![0; 10]);
    p.flags = Some(vec![0; 10]);
    p.source = Some(vec![2; 10]);
    p.drop_empty_columns();
    assert_eq!(
        p.layout(),
        Layout {
            rgb: true,
            time: true,
            ..Layout::default()
        }
    );
}
