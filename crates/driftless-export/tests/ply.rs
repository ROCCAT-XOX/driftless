//! PLY: the writer of `color.ply`/`modell.ply`, and the reader moved from the desktop program
//! (ASCII and binary, foreign colours and intensities, faces, broken headers).

use driftless_export::ply::{read_ply, write_ply};
use driftless_export::testutil::synthetic_points;
use driftless_export::{read_points, ExportError, Layout, Points, PHASE_READ};
use driftless_io::progress::Progress;
use std::path::{Path, PathBuf};

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "driftless-export-ply-{}-{name}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Files in `dir` whose name ends in `.tmp`.
fn leftovers(dir: &Path) -> Vec<PathBuf> {
    std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.to_string_lossy().ends_with(".tmp"))
        .collect()
}

/// The header of a PLY file, up to and with `end_header\n`.
fn header_of(bytes: &[u8]) -> &str {
    let end = b"end_header\n";
    let n = bytes
        .windows(end.len())
        .position(|w| w == end)
        .expect("end_header")
        + end.len();
    std::str::from_utf8(&bytes[..n]).unwrap()
}

/// 8-bit values widened as the reader does.
fn wide<const N: usize>(v: [u8; N]) -> [u16; N] {
    v.map(|c| u16::from(c) * 257)
}

/// A small cloud as the map step exports it: 4 points, colours and intensities of 8-bit
/// origin, two of them without measured colour.
fn small_points() -> Points {
    Points {
        xyz: vec![
            [10.0, 20.0, 1.0],
            [10.5, 20.25, 1.5],
            [11.0, 21.0, 2.0],
            [12.123_4, 19.876_5, 0.5],
        ],
        rgb: Some(vec![
            wide([255, 0, 0]),
            wide([0, 255, 0]),
            wide([0, 0, 255]),
            wide([1, 2, 3]),
        ]),
        intensity: Some(wide([0, 100, 200, 255]).to_vec()),
        flags: Some(vec![0, 1, 0, 1]),
        ..Points::default()
    }
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
fn ply_round_trip_from_write_ply() {
    let dir = temp_dir("roundtrip");
    let path = dir.join("color.ply");
    let pts = small_points();
    write_ply(&path, &pts, &Progress::new()).unwrap();
    let header = header_of(&std::fs::read(&path).unwrap()).to_string();
    assert!(header.contains("property float x\n"), "{header}");
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_xyz_close(&p, &pts, 1e-5);
    assert_eq!(p.rgb, pts.rgb);
    assert_eq!(p.intensity, pts.intensity);
    // flag 1 → `colored` 0 → flag 1
    assert_eq!(p.flags, Some(vec![0, 1, 0, 1]));
    assert_eq!(p.time, None);
    assert_eq!(p.source, None);
    assert!(p.scans.is_empty());

    let mut with_scan = pts.clone();
    with_scan.source = Some(vec![0, 1, 2, 1]);
    write_ply(&path, &with_scan, &Progress::new()).unwrap();
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.source, Some(vec![0, 1, 2, 1]));
    assert_eq!(p.rgb, pts.rgb);
    assert_eq!(p.flags, Some(vec![0, 1, 0, 1]));
    // one scan for all: the column says nothing, `read_points` drops it
    with_scan.source = Some(vec![3; 4]);
    write_ply(&path, &with_scan, &Progress::new()).unwrap();
    assert_eq!(
        read_ply(&path, &Progress::new()).unwrap().source,
        Some(vec![3; 4])
    );
    assert_eq!(read_points(&path, &Progress::new()).unwrap().source, None);
    // `read_points` takes the reader by the extension, in any case
    let upper = dir.join("MODELL.PLY");
    std::fs::copy(&path, &upper).unwrap();
    assert_eq!(read_points(&upper, &Progress::new()).unwrap().len(), 4);
    std::fs::remove_dir_all(dir).unwrap();
}

/// Full 16-bit columns come back as 8 bit widened; times are not stored, flags only as
/// „measured colour or not“.
#[test]
fn ply_round_trip_synthetic() {
    let dir = temp_dir("synthetic");
    let path = dir.join("wolke.ply");
    let all = Layout {
        rgb: true,
        intensity: true,
        source: true,
        time: true,
        flags: true,
    };
    let pts = synthetic_points(5_000, 21, all);
    write_ply(&path, &pts, &Progress::new()).unwrap();
    let header = header_of(&std::fs::read(&path).unwrap()).to_string();
    assert_eq!(
        header,
        "ply\nformat binary_little_endian 1.0\nelement vertex 5000\nproperty float x\n\
         property float y\nproperty float z\nproperty float intensity\nproperty uchar red\n\
         property uchar green\nproperty uchar blue\nproperty uchar colored\n\
         property ushort scan\nend_header\n"
    );
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_xyz_close(&p, &pts, 1e-5);
    let eight = |v: u16| ((f64::from(v) / 257.0).round() as u16) * 257;
    let want_rgb: Vec<[u16; 3]> = pts
        .rgb
        .as_ref()
        .unwrap()
        .iter()
        .map(|c| c.map(eight))
        .collect();
    assert_eq!(p.rgb.as_ref(), Some(&want_rgb));
    // Intensity is written as float 0–255 (v / 257) and comes back exactly, not as 8 bit.
    assert_eq!(p.intensity, pts.intensity);
    assert_eq!(p.source, pts.source);
    let want_flags: Vec<u8> = pts
        .flags
        .as_ref()
        .unwrap()
        .iter()
        .map(|&f| u8::from(f != 0))
        .collect();
    assert_eq!(p.flags, Some(want_flags));
    assert_eq!(p.time, None);

    // the same points give the same bytes
    let again = dir.join("nochmal.ply");
    write_ply(&again, &pts, &Progress::new()).unwrap();
    assert_eq!(
        std::fs::read(&path).unwrap(),
        std::fs::read(&again).unwrap()
    );

    // only xyz
    let bare = Points {
        xyz: pts.xyz.clone(),
        ..Points::default()
    };
    write_ply(&path, &bare, &Progress::new()).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let header = header_of(&bytes);
    assert!(
        header.ends_with("property float z\nend_header\n"),
        "{header}"
    );
    assert_eq!(bytes.len(), header.len() + 5_000 * 12);
    std::fs::remove_dir_all(dir).unwrap();
}

/// UTM coordinates: `double` for xyz, back within 1 µm.
#[test]
fn ply_far_coordinates_use_double() {
    let dir = temp_dir("far");
    let path = dir.join("utm.ply");
    let mut pts = synthetic_points(2_000, 22, Layout::default());
    for p in &mut pts.xyz {
        p[0] += 500_000.0;
        p[1] += 5_400_000.0;
    }
    write_ply(&path, &pts, &Progress::new()).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let header = header_of(&bytes);
    assert!(
        header.contains("property double x\nproperty double y\nproperty double z\n"),
        "{header}"
    );
    assert_eq!(bytes.len(), header.len() + 2_000 * 24);
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_xyz_close(&p, &pts, 1e-6);
    // just below the limit stays float
    let near = Points {
        xyz: vec![[8191.0, -8191.5, 0.0]],
        ..Points::default()
    };
    write_ply(&path, &near, &Progress::new()).unwrap();
    assert!(header_of(&std::fs::read(&path).unwrap()).contains("property float x\n"));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_ascii_with_float_color() {
    let dir = temp_dir("ascii");
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
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 3);
    let want = [[1.5, 2.0, -3.0], [-0.1, 0.25, 4.0], [7.0, 8.0, 9.0]];
    for (a, w) in p.xyz.iter().zip(&want) {
        for k in 0..3 {
            assert!((a[k] - w[k]).abs() < 1e-6, "{a:?}");
        }
    }
    // all within 0–1: scaled to 0–255
    assert_eq!(
        p.rgb,
        Some(vec![
            wide([255, 0, 0]),
            wide([0, 128, 0]),
            wide([0, 0, 255])
        ])
    );
    // 0–255 as float: × 257, without rounding to whole 8-bit steps first
    assert_eq!(p.intensity, Some(vec![2570, 5269, 65535]));
    assert_eq!(p.flags, None);
    std::fs::remove_dir_all(dir).unwrap();
}

/// Float colours of a foreign file in 0–255: one value above 1 tells the range, so they are
/// taken as they are (rounded, clamped), not ×255.
#[test]
fn ply_float_color_0_255() {
    let dir = temp_dir("float-255");
    let path = dir.join("fremd.ply");
    let text = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\n\
                property float z\nproperty double red\nproperty double green\n\
                property double blue\nend_header\n\
                0 0 0 255 128.4 0\n\
                1 0 0 1 0.6 0\n\
                2 0 0 300 -4 nan\n\
                3 0 0 0.2 0 1\n";
    std::fs::write(&path, text).unwrap();
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(
        p.rgb,
        Some(vec![
            wide([255, 128, 0]),
            wide([1, 1, 0]),
            wide([255, 0, 0]),
            wide([0, 0, 1])
        ])
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
    let dir = temp_dir("be");
    let path = dir.join("utm.ply");
    let base = [512_345.678, 5_412_345.678, 312.345];
    let header = "ply\nformat binary_big_endian 1.0\nelement vertex 3\n\
                  property double x\nproperty float64 y\nproperty double z\n\
                  property uchar red\nproperty uint8 green\nproperty uchar blue\n\
                  property ushort intensity\nproperty ushort scan\nend_header\n";
    let at = |i: usize| {
        let f = i as f64;
        [
            base[0] + f * 0.731,
            base[1] - f * 0.417,
            base[2] + f * 0.013,
        ]
    };
    let rows: Vec<Vec<Value>> = (0..3)
        .map(|i| {
            let p = at(i);
            vec![
                F64(p[0]),
                F64(p[1]),
                F64(p[2]),
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
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 3);
    for i in 0..3 {
        // doubles come through exactly
        assert_eq!(p.xyz[i], at(i), "point {i}");
    }
    assert_eq!(
        p.rgb,
        Some(vec![
            wide([0, 20, 30]),
            wide([10, 20, 30]),
            wide([20, 20, 30])
        ])
    );
    // above 255: scaled so that the brightest is 255
    assert_eq!(p.intensity, Some(wide([85, 170, 255]).to_vec()));
    assert_eq!(p.source, Some(vec![0, 1, 0]));
    std::fs::remove_dir_all(dir).unwrap();
}

/// `ushort` colours are 16 bit already.
#[test]
fn ply_ushort_color_is_sixteen_bit() {
    let dir = temp_dir("ushort");
    let path = dir.join("tief.ply");
    std::fs::write(
        &path,
        "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\n\
         property float z\nproperty ushort red\nproperty ushort green\nproperty ushort blue\n\
         end_header\n0 0 0 65535 4660 7\n",
    )
    .unwrap();
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(vec![[65535, 4660, 7]]));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_without_color() {
    use Value::*;
    let dir = temp_dir("nocolor");
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
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 2);
    assert_eq!(p.rgb, None);
    assert_eq!(p.intensity, None);
    assert_eq!(p.source, None);
    assert_eq!(p.flags, None);
    assert_eq!(p.xyz[1], [4.0, 5.0, 6.0]);

    // no points at all
    std::fs::write(
        &path,
        "ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\n\
         property float z\nend_header\n",
    )
    .unwrap();
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert!(p.is_empty());
    assert_eq!(p.bounds(), None);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_skips_faces() {
    use Value::*;
    let dir = temp_dir("faces");
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
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 3);
    assert_eq!(p.xyz[2], [0.0, 1.0, 0.0]);

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
    let p = read_ply(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 2);
    assert_eq!(p.xyz, vec![[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]]);
    assert_eq!(p.flags, Some(vec![0, 1]));
    std::fs::remove_dir_all(dir).unwrap();
}

/// The German message of a failed read, or a panic naming what came instead.
fn format_error(r: Result<Points, ExportError>) -> String {
    match r {
        Err(ExportError::Format { msg, .. }) => msg,
        other => panic!("expected a format error, got {other:?}"),
    }
}

fn ply_error(dir: &Path, name: &str, bytes: &[u8]) -> String {
    let path = dir.join(name);
    std::fs::write(&path, bytes).unwrap();
    let m = format_error(read_ply(&path, &Progress::new()));
    assert!(m.starts_with("PLY nicht lesbar: "), "{name}: {m}");
    m
}

#[test]
fn ply_broken_header_is_error() {
    let dir = temp_dir("broken");
    let good = "ply\nformat binary_little_endian 1.0\nelement vertex 2\n\
                property float x\nproperty float y\nproperty float z\nend_header\n";
    let mut valid = good.as_bytes().to_vec();
    valid.extend([0u8; 24]);
    // every cut of a valid file: an error, never a panic
    for n in 0..valid.len() {
        let path = dir.join("cut.ply");
        std::fs::write(&path, &valid[..n]).unwrap();
        assert!(read_ply(&path, &Progress::new()).is_err(), "cut at {n}");
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
    match read_ply(&dir.join("weg.ply"), &Progress::new()) {
        Err(e @ ExportError::Io { .. }) => {
            assert!(e.to_string().contains("weg.ply"), "{e}");
        }
        other => panic!("expected an I/O error, got {other:?}"),
    }
    // the path is part of the message shown to the user
    let e = read_ply(&dir.join("leer.ply"), &Progress::new()).unwrap_err();
    assert!(
        e.to_string().contains("leer.ply: PLY nicht lesbar: "),
        "{e}"
    );
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn ply_load_reports_phase_and_cancels() {
    let dir = temp_dir("cancel");
    let path = dir.join("color.ply");
    write_ply(&path, &small_points(), &Progress::new()).unwrap();
    let progress = Progress::new();
    read_ply(&path, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, PHASE_READ);
    assert_eq!((s.total, s.done), (Some(4), 4));

    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == PHASE_READ {
            handle.cancel();
        }
    });
    assert!(matches!(
        read_ply(&path, &progress),
        Err(ExportError::Cancelled)
    ));
    std::fs::remove_dir_all(dir).unwrap();
}

/// A cancel while writing leaves the old file as it was and no `.tmp`; so does a target in a
/// folder that does not exist.
#[test]
fn ply_cancel_keeps_old_file() {
    let dir = temp_dir("atomic");
    let path = dir.join("wolke.ply");
    std::fs::write(&path, b"alt").unwrap();
    let pts = synthetic_points(10, 23, Layout::default());
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == driftless_export::PHASE_WRITE {
            handle.cancel();
        }
    });
    assert!(matches!(
        write_ply(&path, &pts, &progress),
        Err(ExportError::Cancelled)
    ));
    assert_eq!(std::fs::read(&path).unwrap(), b"alt");
    assert!(leftovers(&dir).is_empty());
    assert!(matches!(
        write_ply(&dir.join("fehlt/wolke.ply"), &pts, &Progress::new()),
        Err(ExportError::Io { .. })
    ));
    assert!(leftovers(&dir).is_empty());
    std::fs::remove_dir_all(dir).unwrap();
}
