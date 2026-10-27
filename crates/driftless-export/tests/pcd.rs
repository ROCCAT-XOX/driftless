//! PCD: the fixed header of the writer, binary and ASCII files, the packed colour of PCL,
//! organised clouds with NaN rows, far coordinates and broken headers.

use driftless_export::pcd::{read_pcd, write_pcd};
use driftless_export::testutil::synthetic_points;
use driftless_export::{read_points, ExportError, Layout, Points, PHASE_READ, PHASE_WRITE};
use driftless_io::progress::Progress;
use std::path::{Path, PathBuf};

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "driftless-export-pcd-{}-{name}",
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

/// The header of a PCD file, up to and with the line `DATA …`.
fn header_of(bytes: &[u8]) -> &str {
    let at = bytes
        .windows(5)
        .position(|w| w == b"DATA ")
        .expect("DATA line");
    let end = at + bytes[at..].iter().position(|&b| b == b'\n').unwrap() + 1;
    std::str::from_utf8(&bytes[..end]).unwrap()
}

fn wide<const N: usize>(v: [u8; N]) -> [u16; N] {
    v.map(|c| u16::from(c) * 257)
}

/// A packed PCL colour: the bits `0x00RRGGBB` of a float.
fn packed(r: u8, g: u8, b: u8) -> f32 {
    f32::from_bits(u32::from(r) << 16 | u32::from(g) << 8 | u32::from(b))
}

fn assert_xyz_close(a: &Points, b: &Points, tol: f64) {
    assert_eq!(a.len(), b.len());
    for (p, q) in a.xyz.iter().zip(&b.xyz) {
        for k in 0..3 {
            assert!((p[k] - q[k]).abs() <= tol, "{p:?} vs {q:?}");
        }
    }
}

const ALL: Layout = Layout {
    rgb: true,
    intensity: true,
    source: true,
    time: true,
    flags: true,
};

/// Synthetic points whose colours and intensities have 8-bit origin (multiples of 257).
fn eight_bit_points(n: usize, seed: u64) -> Points {
    let mut pts = synthetic_points(n, seed, ALL);
    let eight = |v: u16| (v >> 8) * 257;
    for c in pts.rgb.as_mut().unwrap() {
        *c = c.map(eight);
    }
    for v in pts.intensity.as_mut().unwrap() {
        *v = eight(*v);
    }
    pts
}

#[test]
fn pcd_binary_round_trip() {
    let dir = temp_dir("round-trip");
    let path = dir.join("wolke.pcd");
    let pts = eight_bit_points(5_000, 31);
    write_pcd(&path, &pts, &Progress::new()).unwrap();
    let p = read_pcd(&path, &Progress::new()).unwrap();
    assert_xyz_close(&p, &pts, 1e-5);
    // colour exact for 8-bit origin, through the packed float
    assert_eq!(p.rgb, pts.rgb);
    assert_eq!(p.intensity, pts.intensity);
    assert_eq!(p.source, pts.source);
    // PCD keeps neither times nor flags
    assert_eq!((p.time, p.flags), (None, None));
    let bytes = std::fs::read(&path).unwrap();
    let header = header_of(&bytes);
    // 3 × 4 + 4 + 4 + 2 bytes per point
    assert_eq!(bytes.len(), header.len() + 5_000 * 22);

    // full 16-bit colours: round(v / 257) per channel
    let full = synthetic_points(100, 32, ALL);
    write_pcd(&path, &full, &Progress::new()).unwrap();
    let p = read_pcd(&path, &Progress::new()).unwrap();
    let eight = |v: u16| ((f64::from(v) / 257.0).round() as u16) * 257;
    let want: Vec<[u16; 3]> = full
        .rgb
        .as_ref()
        .unwrap()
        .iter()
        .map(|c| c.map(eight))
        .collect();
    assert_eq!(p.rgb.as_ref(), Some(&want));

    // the same points give the same bytes; `read_points` knows the extension
    let again = dir.join("nochmal.PCD");
    write_pcd(&again, &full, &Progress::new()).unwrap();
    assert_eq!(
        std::fs::read(&path).unwrap(),
        std::fs::read(&again).unwrap()
    );
    assert_eq!(read_points(&again, &Progress::new()).unwrap().len(), 100);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn pcd_header_exact() {
    let dir = temp_dir("header");
    let path = dir.join("wolke.pcd");
    let pts = eight_bit_points(7, 33);
    write_pcd(&path, &pts, &Progress::new()).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let lines: Vec<&str> = header_of(&bytes).lines().collect();
    assert_eq!(
        lines,
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z rgb intensity scan",
            "SIZE 4 4 4 4 4 2",
            "TYPE F F F F F U",
            "COUNT 1 1 1 1 1 1",
            "WIDTH 7",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            "POINTS 7",
            "DATA binary",
        ]
    );
    // the first point: x y z, then the packed colour
    let data = &bytes[header_of(&bytes).len()..];
    let x = f32::from_le_bytes(data[0..4].try_into().unwrap());
    assert_eq!(x, pts.xyz[0][0] as f32);
    let c = pts.rgb.as_ref().unwrap()[0].map(|v| (v / 257) as u8);
    assert_eq!(
        u32::from_le_bytes(data[12..16].try_into().unwrap()),
        packed(c[0], c[1], c[2]).to_bits()
    );

    // only xyz
    let bare = Points {
        xyz: vec![[1.0, 2.0, 3.0]],
        ..Points::default()
    };
    write_pcd(&path, &bare, &Progress::new()).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let header = header_of(&bytes);
    assert!(
        header.contains("\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\n"),
        "{header}"
    );
    assert_eq!(bytes.len(), header.len() + 12);

    // no points: a valid empty file
    write_pcd(&path, &Points::default(), &Progress::new()).unwrap();
    let header = header_of(&std::fs::read(&path).unwrap()).to_string();
    assert!(header.contains("\nWIDTH 0\nHEIGHT 1\n"), "{header}");
    assert!(header.contains("\nPOINTS 0\n"), "{header}");
    assert!(read_pcd(&path, &Progress::new()).unwrap().is_empty());
    std::fs::remove_dir_all(dir).unwrap();
}

/// An ASCII file as PCL writes it: the packed colour as a whole number, a field of several
/// values (skipped), intensity in 0–1.
#[test]
fn pcd_ascii_reads() {
    let dir = temp_dir("ascii");
    let path = dir.join("fremd.pcd");
    let rgb = |r: u32, g: u32, b: u32| r << 16 | g << 8 | b;
    let text = format!(
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION .7\n\
         FIELDS x y z normal rgb intensity\nSIZE 4 4 4 4 4 4\nTYPE F F F F F F\n\
         COUNT 1 1 1 3 1 1\nWIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\n\
         DATA ascii\n\
         1.5 2 -3 0 0 1 {} 0.5\n\
         -0.1 0.25 4 0 1 0 {} 1\n\
         7 8 9 1 0 0 {:e} 0\n",
        rgb(255, 0, 0),
        rgb(0, 128, 0),
        // a foreign writer may print the packed float itself
        f64::from(packed(1, 2, 3)),
    );
    std::fs::write(&path, text).unwrap();
    let p = read_pcd(&path, &Progress::new()).unwrap();
    // the values as written in the text
    assert_eq!(
        p.xyz,
        vec![[1.5, 2.0, -3.0], [-0.1, 0.25, 4.0], [7.0, 8.0, 9.0]]
    );
    assert_eq!(
        p.rgb,
        Some(vec![wide([255, 0, 0]), wide([0, 128, 0]), wide([1, 2, 3])])
    );
    // floats in 0–1: ×65535
    assert_eq!(p.intensity, Some(vec![32768, 65535, 0]));
    assert_eq!(p.source, None);

    // rgba as U 4, scan as U 2, intensity above 255 scaled to the brightest
    let text = "VERSION 0.7\nFIELDS x y z rgba intensity scan\nSIZE 8 8 8 4 2 2\n\
                TYPE F F F U U U\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n\
                1 2 3 4278255360 700 4\n\
                4 5 6 4294967295 2000 2\n";
    std::fs::write(&path, text).unwrap();
    let p = read_pcd(&path, &Progress::new()).unwrap();
    assert_eq!(p.xyz, vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    assert_eq!(p.rgb, Some(vec![wide([0, 255, 0]), wide([255, 255, 255])]));
    assert_eq!(p.intensity, Some(vec![22937, 65535]));
    assert_eq!(p.source, Some(vec![4, 2]));
    std::fs::remove_dir_all(dir).unwrap();
}

/// Colour in three fields `r g b` (U 1), binary; a padding field `_` in between.
#[test]
fn pcd_separate_rgb_fields() {
    let dir = temp_dir("rgb-fields");
    let path = dir.join("drei.pcd");
    let mut bytes = b"VERSION 0.7\nFIELDS x y z _ r g b\nSIZE 4 4 4 1 1 1 1\n\
                      TYPE F F F U U U U\nCOUNT 1 1 1 1 1 1 1\nWIDTH 2\nHEIGHT 1\n\
                      VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n"
        .to_vec();
    for (p, c) in [
        ([1.0f32, 2.0, 3.0], [10u8, 20, 30]),
        ([4.0, 5.0, 6.0], [255, 0, 7]),
    ] {
        for v in p {
            bytes.extend(v.to_le_bytes());
        }
        bytes.push(99);
        bytes.extend(c);
    }
    std::fs::write(&path, bytes).unwrap();
    let p = read_pcd(&path, &Progress::new()).unwrap();
    assert_eq!(p.xyz, vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    assert_eq!(p.rgb, Some(vec![wide([10, 20, 30]), wide([255, 0, 7])]));
    assert_eq!(p.intensity, None);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn pcd_binary_compressed_refused() {
    let dir = temp_dir("compressed");
    let path = dir.join("klein.pcd");
    let mut bytes = b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n\
                      WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n"
        .to_vec();
    bytes.extend([0u8; 20]);
    std::fs::write(&path, bytes).unwrap();
    match read_pcd(&path, &Progress::new()) {
        Err(ExportError::Format { msg, .. }) => assert_eq!(
            msg,
            "PCD „binary_compressed“ wird nicht unterstützt – in CloudCompare oder PCL als \
             „binary“ speichern"
        ),
        other => panic!("expected a format error, got {other:?}"),
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// An organised cloud (WIDTH 4 × HEIGHT 2) whose rows of invalid points are NaN: those rows
/// are left out.
#[test]
fn pcd_organized_nan_skipped() {
    let dir = temp_dir("organized");
    let path = dir.join("kamera.pcd");
    let mut bytes = b"# organisiert\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\n\
                      TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 4\nHEIGHT 2\n\
                      VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 8\nDATA binary\n"
        .to_vec();
    for i in 0..8u8 {
        let p = if i == 2 || i == 5 {
            [f32::NAN, f32::NAN, f32::NAN]
        } else {
            [f32::from(i), 0.5, -1.0]
        };
        for v in p {
            bytes.extend(v.to_le_bytes());
        }
        bytes.extend(packed(i, 0, 0).to_bits().to_le_bytes());
    }
    std::fs::write(&path, &bytes).unwrap();
    let p = read_pcd(&path, &Progress::new()).unwrap();
    assert_eq!(p.len(), 6);
    let xs: Vec<f64> = p.xyz.iter().map(|q| q[0]).collect();
    assert_eq!(xs, vec![0.0, 1.0, 3.0, 4.0, 6.0, 7.0]);
    let reds: Vec<u16> = p.rgb.as_ref().unwrap().iter().map(|c| c[0] / 257).collect();
    assert_eq!(reds, vec![0, 1, 3, 4, 6, 7]);
    std::fs::remove_dir_all(dir).unwrap();
}

/// UTM coordinates: `SIZE 8` for xyz, back within 1 µm.
#[test]
fn pcd_far_coordinates_use_double() {
    let dir = temp_dir("far");
    let path = dir.join("utm.pcd");
    let mut pts = synthetic_points(2_000, 34, Layout::default());
    for p in &mut pts.xyz {
        p[0] += 500_000.0;
        p[1] += 5_400_000.0;
    }
    write_pcd(&path, &pts, &Progress::new()).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let header = header_of(&bytes);
    assert!(header.contains("\nSIZE 8 8 8\nTYPE F F F\n"), "{header}");
    assert_eq!(bytes.len(), header.len() + 2_000 * 24);
    let p = read_pcd(&path, &Progress::new()).unwrap();
    assert_xyz_close(&p, &pts, 1e-6);
    std::fs::remove_dir_all(dir).unwrap();
}

fn pcd_error(dir: &Path, name: &str, bytes: &[u8]) -> String {
    let path = dir.join(name);
    std::fs::write(&path, bytes).unwrap();
    match read_pcd(&path, &Progress::new()) {
        Err(ExportError::Format { msg, .. }) => {
            assert!(msg.starts_with("PCD nicht lesbar: "), "{name}: {msg}");
            msg
        }
        other => panic!("{name}: expected a format error, got {other:?}"),
    }
}

#[test]
fn pcd_broken_header_is_error() {
    let dir = temp_dir("broken");
    let good = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\n\
                HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n";
    let mut valid = good.as_bytes().to_vec();
    valid.extend([0u8; 24]);
    assert_eq!(read_pcd_bytes(&dir, &valid).unwrap().len(), 2);
    // every cut of a valid file: an error, never a panic
    for n in 0..valid.len() {
        assert!(read_pcd_bytes(&dir, &valid[..n]).is_err(), "cut at {n}");
    }
    pcd_error(&dir, "leer.pcd", b"");
    pcd_error(&dir, "fremd.pcd", b"ply\nformat ascii 1.0\nend_header\n");
    pcd_error(&dir, "binaer.pcd", b"\xff\xfe\x00garbage\n");
    let broken = |from: &str, to: &str| good.replace(from, to).into_bytes();
    pcd_error(&dir, "ohne-data.pcd", &broken("DATA binary\n", ""));
    pcd_error(&dir, "data.pcd", &broken("DATA binary", "DATA zip"));
    pcd_error(&dir, "ohne-felder.pcd", &broken("FIELDS x y z\n", ""));
    pcd_error(&dir, "ohne-z.pcd", &broken("FIELDS x y z", "FIELDS x y w"));
    pcd_error(&dir, "size.pcd", &broken("SIZE 4 4 4", "SIZE 4 4"));
    pcd_error(&dir, "size-3.pcd", &broken("SIZE 4 4 4", "SIZE 4 4 3"));
    pcd_error(&dir, "type.pcd", &broken("TYPE F F F", "TYPE F F Q"));
    pcd_error(&dir, "type-u.pcd", &broken("TYPE F F F", "TYPE F F U"));
    pcd_error(&dir, "count.pcd", &broken("COUNT 1 1 1", "COUNT 1 1 x"));
    pcd_error(&dir, "count-0.pcd", &broken("COUNT 1 1 1", "COUNT 1 1 0"));
    pcd_error(&dir, "count-x.pcd", &broken("COUNT 1 1 1", "COUNT 1 1 3"));
    pcd_error(&dir, "width.pcd", &broken("WIDTH 2", "WIDTH -2"));
    pcd_error(&dir, "zeile.pcd", &broken("HEIGHT 1", "HÖHE 1"));
    pcd_error(&dir, "points.pcd", &broken("POINTS 2", "POINTS 3"));
    // a count larger than the data, also a huge one: an error, no allocation of it
    for count in ["3", "18446744073709551615", "99999999999999999999999"] {
        let mut huge = broken("WIDTH 2", &format!("WIDTH {count}"));
        let huge_text = String::from_utf8(huge.clone())
            .unwrap()
            .replace("POINTS 2", &format!("POINTS {count}"));
        huge = huge_text.into_bytes();
        huge.extend([0u8; 24]);
        pcd_error(&dir, "riesig.pcd", &huge);
        let ascii = String::from_utf8(huge)
            .unwrap()
            .replace("DATA binary", "DATA ascii");
        pcd_error(&dir, "riesig-ascii.pcd", ascii.as_bytes());
    }
    // too few values in ASCII, or one that is no number
    let ascii = good.replace("DATA binary", "DATA ascii");
    pcd_error(&dir, "kurz.pcd", format!("{ascii}1 2 3\n4 5\n").as_bytes());
    pcd_error(
        &dir,
        "wort.pcd",
        format!("{ascii}1 2 3\n4 fünf 6\n").as_bytes(),
    );
    // a missing file names it
    match read_pcd(&dir.join("weg.pcd"), &Progress::new()) {
        Err(e @ ExportError::Io { .. }) => assert!(e.to_string().contains("weg.pcd"), "{e}"),
        other => panic!("expected an I/O error, got {other:?}"),
    }
    std::fs::remove_dir_all(dir).unwrap();
}

fn read_pcd_bytes(dir: &Path, bytes: &[u8]) -> Result<Points, ExportError> {
    let path = dir.join("cut.pcd");
    std::fs::write(&path, bytes).unwrap();
    read_pcd(&path, &Progress::new())
}

#[test]
fn pcd_progress_cancel_and_atomic_write() {
    let dir = temp_dir("cancel");
    let path = dir.join("wolke.pcd");
    let pts = eight_bit_points(50, 35);
    write_pcd(&path, &pts, &Progress::new()).unwrap();
    let progress = Progress::new();
    read_pcd(&path, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, PHASE_READ);
    assert_eq!((s.total, s.done), (Some(50), 50));
    let cancelled = Progress::new();
    cancelled.cancel();
    assert!(matches!(
        read_pcd(&path, &cancelled),
        Err(ExportError::Cancelled)
    ));

    // a cancel while writing keeps the old file and leaves no `.tmp`
    let before = std::fs::read(&path).unwrap();
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == PHASE_WRITE {
            handle.cancel();
        }
    });
    assert!(matches!(
        write_pcd(&path, &synthetic_points(10, 36, ALL), &progress),
        Err(ExportError::Cancelled)
    ));
    assert_eq!(std::fs::read(&path).unwrap(), before);
    assert!(leftovers(&dir).is_empty());
    std::fs::remove_dir_all(dir).unwrap();
}
