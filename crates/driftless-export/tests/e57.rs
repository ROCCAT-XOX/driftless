//! E57: one `data3D` entry per scan with name and pose, colour always 8 bit,
//! foreign files with spherical or invalid coordinates, far coordinates and the atomic write.

use driftless_export::e57::{read_e57, write_e57};
use driftless_export::testutil::synthetic_points;
use driftless_export::{
    read_points, write_points, ExportError, Layout, PointFormat, Points, ScanInfo, PHASE_READ,
    PHASE_WRITE,
};
use driftless_io::progress::{LogLine, Progress};
use e57::{E57Reader, Record, RecordDataType, RecordName, RecordValue};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use std::path::{Path, PathBuf};

mod common;
use common::{double, edit_xml, foreign, integer};

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "driftless-export-e57-{}-{name}",
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

fn assert_close(a: [f64; 3], b: [f64; 3], tol: f64) {
    for k in 0..3 {
        assert!((a[k] - b[k]).abs() <= tol, "{a:?} vs {b:?}");
    }
}

fn assert_pose_close(a: &Isometry3<f64>, b: &Isometry3<f64>, tol: f64) {
    let (ma, mb) = (a.to_homogeneous(), b.to_homogeneous());
    for (x, y) in ma.iter().zip(mb.iter()) {
        assert!((x - y).abs() <= tol, "{a:?} vs {b:?}");
    }
}

fn pose_two() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(3.0, -1.0, 0.5),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 30f64.to_radians()),
    )
}

/// Six points of two scans, interleaved; colours of 8-bit origin, full 16-bit intensities.
fn two_scans() -> Points {
    let n = 6;
    Points {
        xyz: (0..n)
            .map(|i| {
                [
                    i as f64 * 0.5 - 1.0,
                    2.0 - i as f64 * 0.25,
                    1.0 + i as f64 * 0.1,
                ]
            })
            .collect(),
        rgb: Some(
            (0..n)
                .map(|i| [i as u16 * 40 * 257, 255 * 257, (255 - i as u16 * 7) * 257])
                .collect(),
        ),
        intensity: Some(vec![0, 1, 257, 12_345, 65_534, 65_535]),
        source: Some(vec![0, 1, 1, 0, 1, 0]),
        time: None,
        flags: None,
        scans: vec![
            ScanInfo {
                source: 0,
                name: "Erdgeschoss".into(),
                pose: Isometry3::identity(),
            },
            ScanInfo {
                source: 1,
                name: "Büro Süd".into(),
                pose: pose_two(),
            },
        ],
    }
}

/// The indices of `pts` grouped by scan in the order of `order`, stable inside a scan.
fn grouped(pts: &Points, order: &[u16]) -> Vec<usize> {
    let source = pts.source.as_ref().unwrap();
    order
        .iter()
        .flat_map(|&s| (0..pts.len()).filter(move |&i| source[i] == s))
        .collect()
}

#[test]
fn e57_round_trip_two_scans() {
    let dir = temp_dir("round-trip");
    let path = dir.join("modell.e57");
    let pts = two_scans();
    write_e57(&path, &pts, &Progress::new()).unwrap();

    // In the file: two entries, the points in the frame of their scan.
    let reader = E57Reader::from_file(&path).unwrap();
    let clouds = reader.pointclouds();
    assert_eq!(clouds.len(), 2);
    assert_eq!(clouds[0].records, 3);
    assert_eq!(clouds[1].records, 3);
    assert_eq!(clouds[1].name.as_deref(), Some("Büro Süd"));
    assert!(!clouds[0]
        .prototype
        .iter()
        .any(|r| matches!(r.name, RecordName::Unknown { .. })));
    let mut raw = E57Reader::from_file(&path).unwrap();
    let mut simple = raw.pointcloud_simple(&clouds[1]).unwrap();
    simple.apply_pose(false);
    let first = simple.next().unwrap().unwrap();
    let e57::CartesianCoordinate::Valid { x, y, z } = first.cartesian else {
        panic!("not cartesian");
    };
    let local = pose_two().inverse_transform_point(&Point3::from(pts.xyz[1]));
    assert_close([x, y, z], local.coords.into(), 1e-12);

    let back = read_e57(&path, &Progress::new()).unwrap();
    assert_eq!(back.scans.len(), 2);
    assert_eq!(back.scans[0].name, "Erdgeschoss");
    assert_eq!(back.scans[1].name, "Büro Süd");
    assert_eq!((back.scans[0].source, back.scans[1].source), (0, 1));
    assert_pose_close(&back.scans[0].pose, &Isometry3::identity(), 1e-12);
    assert_pose_close(&back.scans[1].pose, &pose_two(), 1e-12);

    let order = grouped(&pts, &[0, 1]);
    assert_eq!(back.len(), pts.len());
    for (k, &i) in order.iter().enumerate() {
        assert_close(back.xyz[k], pts.xyz[i], 1e-9);
        assert_eq!(back.rgb.as_ref().unwrap()[k], pts.rgb.as_ref().unwrap()[i]);
        assert_eq!(
            back.intensity.as_ref().unwrap()[k],
            pts.intensity.as_ref().unwrap()[i]
        );
        assert_eq!(
            back.source.as_ref().unwrap()[k],
            pts.source.as_ref().unwrap()[i]
        );
    }
    assert_eq!(back.time, None);
    assert_eq!(back.flags, None);
    // 8-bit colours are stored as 0–255.
    let limits = clouds[0].color_limits.clone().unwrap();
    assert_eq!(limits.red_max, Some(RecordValue::Integer(255)));
    let limits = clouds[0].intensity_limits.clone().unwrap();
    assert_eq!(limits.intensity_max, Some(RecordValue::Integer(65_535)));
    std::fs::remove_dir_all(dir).unwrap();
}

/// Colour is always stored as 8 bit (limits 0–255, `round(v / 257)`), also when it is not
/// of 8-bit origin: tools built on libE57 (pye57, …) expect 8-bit colours.
#[test]
fn e57_colour_always_eight_bit() {
    let dir = temp_dir("sixteen");
    let path = dir.join("farbe.e57");
    let pts = Points {
        xyz: vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [2.0, 0.0, 1.0]],
        rgb: Some(vec![
            [1000, 257, 0],
            [65_535, 514, 12_345],
            [128, 129, 65_400],
        ]),
        ..Points::default()
    };
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let mut reader = E57Reader::from_file(&path).unwrap();
    let cloud = reader.pointclouds()[0].clone();
    let limits = cloud.color_limits.clone().unwrap();
    for (min, max) in [
        (&limits.red_min, &limits.red_max),
        (&limits.green_min, &limits.green_max),
        (&limits.blue_min, &limits.blue_max),
    ] {
        assert_eq!(min, &Some(RecordValue::Integer(0)));
        assert_eq!(max, &Some(RecordValue::Integer(255)));
    }
    let red = cloud
        .prototype
        .iter()
        .find(|r| r.name == RecordName::ColorRed)
        .unwrap();
    assert!(
        matches!(red.data_type, RecordDataType::Integer { min: 0, max: 255 }),
        "{:?}",
        red.data_type
    );
    // stored round(v / 257): 1000 → 4, 12 345 → 48, 128 → 0, 129 → 1, 65 400 → 254
    let mut raw = Vec::new();
    for p in reader.pointcloud_raw(&cloud).unwrap() {
        let p = p.unwrap();
        raw.push(
            p[3..6]
                .iter()
                .map(|v| match v {
                    RecordValue::Integer(i) => *i,
                    other => panic!("{other:?}"),
                })
                .collect::<Vec<_>>(),
        );
    }
    assert_eq!(raw, vec![vec![4, 1, 0], vec![255, 2, 48], vec![0, 1, 254]]);
    // read back × 257
    let back = read_e57(&path, &Progress::new()).unwrap();
    let want: Vec<[u16; 3]> = pts
        .rgb
        .as_ref()
        .unwrap()
        .iter()
        .map(|c| c.map(|v| ((u32::from(v) + 128) / 257 * 257) as u16))
        .collect();
    assert_eq!(back.rgb, Some(want));
    assert_eq!(back.intensity, None);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn e57_spherical_only_is_converted() {
    let dir = temp_dir("spherical");
    let path = dir.join("kugel.e57");
    let prototype = vec![
        double(RecordName::SphericalRange),
        double(RecordName::SphericalAzimuth),
        double(RecordName::SphericalElevation),
        Record {
            name: RecordName::SphericalInvalidState,
            data_type: RecordDataType::Integer { min: 0, max: 2 },
        },
    ];
    let polar = [
        (2.0, 0.3, 0.1),
        (5.5, -2.0, -0.4),
        (1.0, 3.0, 1.2),
        (7.0, 1.0, 0.0),
    ];
    // The last one is only a direction and is left out.
    let rows: Vec<_> = polar
        .iter()
        .enumerate()
        .map(|(i, &(r, a, e))| {
            vec![
                RecordValue::Double(r),
                RecordValue::Double(a),
                RecordValue::Double(e),
                RecordValue::Integer(i64::from(i == 3)),
            ]
        })
        .collect();
    let pose = e57::Transform {
        rotation: e57::Quaternion::default(),
        translation: e57::Translation {
            x: 10.0,
            y: 20.0,
            z: 0.5,
        },
    };
    foreign(&path, None, prototype, &rows, Some(pose), true);

    let progress = Progress::new();
    let back = read_e57(&path, &progress).unwrap();
    assert_eq!(back.len(), 3);
    for (p, &(r, a, e)) in back.xyz.iter().zip(&polar) {
        let want = [
            10.0 + r * f64::cos(e) * f64::cos(a),
            20.0 + r * f64::cos(e) * f64::sin(a),
            0.5 + r * f64::sin(e),
        ];
        assert_close(*p, want, 1e-9);
    }
    assert_eq!(back.rgb, None);
    assert_eq!(back.intensity, None);
    assert_eq!(back.source, None);
    assert_eq!(back.scans.len(), 1);
    assert_eq!(back.scans[0].name, "Scan 1");
    // The preview image is not read, and the user is told.
    let warnings: Vec<_> = progress
        .take_log()
        .into_iter()
        .filter_map(|l| match l {
            LogLine::Warning(w) => Some(w),
            LogLine::Info(_) => None,
        })
        .collect();
    assert_eq!(warnings.len(), 1);
    assert!(warnings[0].contains("Bild"), "{warnings:?}");
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn e57_without_coordinates_is_error() {
    let dir = temp_dir("invalid");
    let path = dir.join("leer.e57");
    let prototype = vec![
        double(RecordName::CartesianX),
        double(RecordName::CartesianY),
        double(RecordName::CartesianZ),
        Record {
            name: RecordName::CartesianInvalidState,
            data_type: RecordDataType::Integer { min: 0, max: 2 },
        },
    ];
    let rows: Vec<_> = (0..5)
        .map(|i| {
            vec![
                RecordValue::Double(f64::from(i)),
                RecordValue::Double(0.0),
                RecordValue::Double(0.0),
                RecordValue::Integer(2),
            ]
        })
        .collect();
    foreign(&path, Some("Keller"), prototype, &rows, None, false);
    match read_e57(&path, &Progress::new()) {
        Err(ExportError::Format { msg, .. }) => {
            assert_eq!(msg, "E57: Scan „Keller“ hat keine kartesischen Punkte")
        }
        other => panic!("{other:?}"),
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn e57_far_coordinates() {
    let dir = temp_dir("far");
    let far = [500_000.0, 5_400_000.0, 312.0];
    // Without scans: one entry, the identity pose.
    let mut pts = synthetic_points(2_000, 7, Layout::default());
    for p in &mut pts.xyz {
        for k in 0..3 {
            p[k] += far[k];
        }
    }
    let path = dir.join("utm.e57");
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let back = read_e57(&path, &Progress::new()).unwrap();
    assert_eq!(back.len(), pts.len());
    for (p, q) in back.xyz.iter().zip(&pts.xyz) {
        assert_close(*p, *q, 1e-6);
    }

    // With scans whose poses are far out as well.
    let layout = Layout {
        source: true,
        ..Layout::default()
    };
    let mut pts = synthetic_points(2_000, 8, layout);
    for p in &mut pts.xyz {
        for k in 0..3 {
            p[k] += far[k];
        }
    }
    for s in &mut pts.scans {
        s.pose.translation.vector += nalgebra::Vector3::from(far);
    }
    let path = dir.join("utm-scans.e57");
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let back = read_e57(&path, &Progress::new()).unwrap();
    let order = grouped(&pts, &[0, 1, 2]);
    assert_eq!(back.len(), order.len());
    for (k, &i) in order.iter().enumerate() {
        assert_close(back.xyz[k], pts.xyz[i], 1e-6);
    }
    for (a, b) in back.scans.iter().zip(&pts.scans) {
        assert_pose_close(&a.pose, &b.pose, 1e-9);
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// The GUID form `xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx` (lower-case hex).
fn is_guid(s: &str) -> bool {
    let parts: Vec<_> = s.split('-').collect();
    parts.iter().map(|p| p.len()).collect::<Vec<_>>() == [8, 4, 4, 4, 12]
        && parts
            .iter()
            .all(|p| p.chars().all(|c| matches!(c, '0'..='9' | 'a'..='f')))
}

#[test]
fn e57_deterministic() {
    let dir = temp_dir("deterministic");
    let all = Layout {
        rgb: true,
        intensity: true,
        source: true,
        time: true,
        flags: true,
    };
    let pts = synthetic_points(5_000, 3, all);
    std::fs::create_dir_all(dir.join("a")).unwrap();
    std::fs::create_dir_all(dir.join("b")).unwrap();
    write_e57(&dir.join("a/modell.e57"), &pts, &Progress::new()).unwrap();
    write_e57(&dir.join("b/modell.e57"), &pts, &Progress::new()).unwrap();
    let a = std::fs::read(dir.join("a/modell.e57")).unwrap();
    assert_eq!(a, std::fs::read(dir.join("b/modell.e57")).unwrap());

    let reader = E57Reader::from_file(dir.join("a/modell.e57")).unwrap();
    assert!(reader.creation().is_none());
    assert!(is_guid(reader.guid()), "{}", reader.guid());
    let guids: Vec<_> = reader
        .pointclouds()
        .iter()
        .map(|pc| pc.guid.clone().unwrap())
        .collect();
    assert_eq!(guids.len(), 3);
    assert!(guids.iter().all(|g| is_guid(g)), "{guids:?}");
    let mut unique = guids.clone();
    unique.push(reader.guid().to_owned());
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), 4);

    // Another file name gives other GUIDs.
    write_e57(&dir.join("a/anders.e57"), &pts, &Progress::new()).unwrap();
    let other = E57Reader::from_file(dir.join("a/anders.e57")).unwrap();
    assert_ne!(other.guid(), reader.guid());
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn e57_empty() {
    let dir = temp_dir("empty").join("Büro Scan");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("Prüfung ä.e57");
    write_e57(&path, &Points::default(), &Progress::new()).unwrap();
    let reader = E57Reader::from_file(&path).unwrap();
    let clouds = reader.pointclouds();
    assert_eq!(clouds.len(), 1);
    assert_eq!(clouds[0].records, 0);
    assert_eq!(clouds[0].name.as_deref(), Some("Prüfung ä"));
    let back = read_e57(&path, &Progress::new()).unwrap();
    assert!(back.is_empty());
    assert_eq!(back.scans.len(), 1);
    assert_eq!(back.scans[0].name, "Prüfung ä");
    std::fs::remove_dir_all(dir.parent().unwrap()).unwrap();
}

/// Without a `source` column there is one scan, named after the file, at the identity – or
/// the one scan of [`Points::scans`]; sources without a [`ScanInfo`] are "Scan <n+1>".
#[test]
fn e57_scan_entries_follow_the_cloud() {
    let dir = temp_dir("entries");
    let mut pts = synthetic_points(30, 5, Layout::default());
    let path = dir.join("einzeln.e57");
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let back = read_e57(&path, &Progress::new()).unwrap();
    assert_eq!(back.source, None);
    assert_eq!(back.scans.len(), 1);
    assert_eq!(back.scans[0].name, "einzeln");
    assert_pose_close(&back.scans[0].pose, &Isometry3::identity(), 0.0);

    // One scan with a pose, no source column: its name and pose are kept.
    pts.scans = vec![ScanInfo {
        source: 0,
        name: "Halle".into(),
        pose: pose_two(),
    }];
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let back = read_e57(&path, &Progress::new()).unwrap();
    assert_eq!(back.scans[0].name, "Halle");
    assert_pose_close(&back.scans[0].pose, &pose_two(), 1e-12);
    for (p, q) in back.xyz.iter().zip(&pts.xyz) {
        assert_close(*p, *q, 1e-9);
    }

    // Sources 2 and 0 without scans: "Scan 1" and "Scan 3", in the order of the numbers.
    pts.scans.clear();
    pts.source = Some((0..30).map(|i| if i % 3 == 0 { 2 } else { 0 }).collect());
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let back = read_e57(&path, &Progress::new()).unwrap();
    let names: Vec<_> = back.scans.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Scan 1", "Scan 3"]);
    let order = grouped(&pts, &[0, 2]);
    for (k, &i) in order.iter().enumerate() {
        assert_close(back.xyz[k], pts.xyz[i], 1e-9);
    }
    assert_eq!(back.source.as_ref().unwrap()[0], 0);
    assert_eq!(back.source.as_ref().unwrap()[29], 1);

    // A scan source not listed in `scans` gets its own entry after the listed ones.
    pts.scans = vec![ScanInfo {
        source: 2,
        name: "Treppe".into(),
        pose: pose_two(),
    }];
    write_e57(&path, &pts, &Progress::new()).unwrap();
    let back = read_e57(&path, &Progress::new()).unwrap();
    let names: Vec<_> = back.scans.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Treppe", "Scan 1"]);
    let order = grouped(&pts, &[2, 0]);
    for (k, &i) in order.iter().enumerate() {
        assert_close(back.xyz[k], pts.xyz[i], 1e-9);
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn e57_progress_cancel_and_atomic_write() {
    let dir = temp_dir("atomic");
    let path = dir.join("wolke.e57");
    let pts = synthetic_points(100, 9, Layout::default());
    let progress = Progress::new();
    write_e57(&path, &pts, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, PHASE_WRITE);
    assert_eq!((s.total, s.done), (Some(100), 100));
    let progress = Progress::new();
    read_e57(&path, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, PHASE_READ);
    assert_eq!((s.total, s.done), (Some(100), 100));

    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == PHASE_READ {
            handle.cancel();
        }
    });
    assert!(matches!(
        read_e57(&path, &progress),
        Err(ExportError::Cancelled)
    ));

    let old = std::fs::read(&path).unwrap();
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == PHASE_WRITE {
            handle.cancel();
        }
    });
    let other = synthetic_points(10, 10, Layout::default());
    assert!(matches!(
        write_e57(&path, &other, &progress),
        Err(ExportError::Cancelled)
    ));
    assert_eq!(std::fs::read(&path).unwrap(), old);
    assert!(leftovers(&dir).is_empty());
    assert!(matches!(
        write_e57(&dir.join("fehlt/wolke.e57"), &pts, &Progress::new()),
        Err(ExportError::Io { .. })
    ));
    let mut nan = pts.clone();
    nan.xyz[3][1] = f64::NAN;
    assert!(matches!(
        write_e57(&dir.join("nan.e57"), &nan, &Progress::new()),
        Err(ExportError::Format { .. })
    ));
    assert!(leftovers(&dir).is_empty());
    assert!(!dir.join("nan.e57").exists());
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn e57_foreign_garbage_and_dispatch() {
    let dir = temp_dir("garbage");
    let path = dir.join("kaputt.E57");
    assert!(matches!(
        read_e57(&path, &Progress::new()),
        Err(ExportError::Io { .. })
    ));
    std::fs::write(&path, b"keine E57-Datei, nur Text").unwrap();
    assert!(matches!(
        read_e57(&path, &Progress::new()),
        Err(ExportError::Format { .. })
    ));
    // Cut in half: an error, not a panic.
    let good = dir.join("gut.e57");
    write_e57(
        &good,
        &synthetic_points(5_000, 11, Layout::default()),
        &Progress::new(),
    )
    .unwrap();
    let bytes = std::fs::read(&good).unwrap();
    for cut in [bytes.len() / 2, bytes.len() - 10, 48] {
        std::fs::write(&path, &bytes[..cut]).unwrap();
        assert!(
            matches!(
                read_e57(&path, &Progress::new()),
                Err(ExportError::Format { .. })
            ),
            "cut {cut}"
        );
    }

    assert_eq!(
        PointFormat::from_path(Path::new("Büro Scan/x.E57")),
        Some(PointFormat::E57)
    );
    assert_eq!(PointFormat::E57.extension(), "e57");
    let pts = synthetic_points(
        20,
        12,
        Layout {
            rgb: true,
            ..Layout::default()
        },
    );
    let via = dir.join("über.e57");
    write_points(&via, PointFormat::E57, &pts, &Progress::new()).unwrap();
    let back = read_points(&via, &Progress::new()).unwrap();
    assert_eq!(back.len(), 20);
    // 8 bit in the file, × 257 back
    let eight: Vec<[u16; 3]> = pts
        .rgb
        .as_ref()
        .unwrap()
        .iter()
        .map(|c| c.map(|v| ((u32::from(v) + 128) / 257 * 257) as u16))
        .collect();
    assert_eq!(back.rgb, Some(eight));
    std::fs::remove_dir_all(dir).unwrap();
}

/// A foreign header may claim any number of points: more than the file can hold is refused
/// at once, without reserving memory for them.
#[test]
fn e57_claimed_points_beyond_the_file_are_refused() {
    let dir = temp_dir("claimed");
    let path = dir.join("gross.e57");
    let prototype = vec![
        double(RecordName::CartesianX),
        double(RecordName::CartesianY),
        double(RecordName::CartesianZ),
    ];
    let rows = [
        vec![
            RecordValue::Double(1.0),
            RecordValue::Double(2.0),
            RecordValue::Double(3.0),
        ],
        vec![
            RecordValue::Double(4.0),
            RecordValue::Double(5.0),
            RecordValue::Double(6.0),
        ],
    ];
    foreign(&path, Some("Riese"), prototype, &rows, None, false);
    assert_eq!(read_e57(&path, &Progress::new()).unwrap().len(), 2);
    edit_xml(&path, |xml| {
        xml.replace("recordCount=\"2\"", "recordCount=\"1000000000000\"")
    });
    let start = std::time::Instant::now();
    let err = read_e57(&path, &Progress::new()).unwrap_err();
    assert!(start.elapsed().as_secs_f64() < 2.0, "{:?}", start.elapsed());
    let ExportError::Format { msg, .. } = err else {
        panic!("{err:?}");
    };
    assert!(
        msg.starts_with("E57: Scan „Riese“ gibt 1000000000000 Punkte an, die Datei kann höchstens"),
        "{msg}"
    );
    std::fs::remove_dir_all(dir).unwrap();
}

/// A coordinate with min = max takes no bits; the library would then make up points without
/// end from an empty packet. Such files are refused before anything is decoded.
#[test]
fn e57_zero_bit_coordinates_are_refused() {
    let dir = temp_dir("zero-bits");
    // x and y take 1 bit (0..=1), z 10 bits; the XML is then edited so that x and y (and z,
    // if `all`) have min = max = 0.
    let zero = |path: &Path, all: bool| {
        let prototype = vec![
            integer(RecordName::CartesianX, 0, 1),
            integer(RecordName::CartesianY, 0, 1),
            integer(RecordName::CartesianZ, 0, if all { 1 } else { 1000 }),
        ];
        let rows = [vec![
            RecordValue::Integer(0),
            RecordValue::Integer(0),
            RecordValue::Integer(0),
        ]];
        foreign(path, Some("Leer"), prototype, &rows, None, false);
        edit_xml(path, |xml| {
            let xml = xml.replace("maximum=\"1\"", "maximum=\"0\"");
            if all {
                xml.replace("recordCount=\"1\"", "recordCount=\"1000000000000\"")
            } else {
                xml
            }
        });
    };
    // Every field 0 bit, and a claimed count far beyond the file.
    let path = dir.join("null.e57");
    zero(&path, true);
    let start = std::time::Instant::now();
    let err = read_e57(&path, &Progress::new()).unwrap_err();
    assert!(start.elapsed().as_secs_f64() < 2.0, "{:?}", start.elapsed());
    let ExportError::Format { msg, .. } = err else {
        panic!("{err:?}");
    };
    assert_eq!(
        msg,
        "E57: Scan „Leer“ speichert cartesianX mit 0 Bit je Punkt – solche Dateien werden \
         nicht gelesen"
    );
    // Only x and y without bits, the true count: refused the same way.
    let path = dir.join("halb.e57");
    zero(&path, false);
    let err = read_e57(&path, &Progress::new()).unwrap_err().to_string();
    assert!(err.contains("speichert cartesianX mit 0 Bit"), "{err}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// Entries may point at the same data: the claimed points of all of them together are bounded
/// by the file, not each one on its own.
#[test]
fn e57_entries_sharing_one_section_are_refused() {
    let dir = temp_dir("shared");
    let path = dir.join("geteilt.e57");
    let prototype = vec![
        double(RecordName::CartesianX),
        double(RecordName::CartesianY),
        double(RecordName::CartesianZ),
    ];
    let rows: Vec<Vec<RecordValue>> = (0..50)
        .map(|i| vec![RecordValue::Double(f64::from(i)); 3])
        .collect();
    foreign(&path, Some("Kopie"), prototype, &rows, None, false);
    let len = std::fs::metadata(&path).unwrap().len();
    // One entry fits the file easily.
    assert!(50 * 192 < len * 8);
    edit_xml(&path, |xml| {
        let start = xml.find("<vectorChild type=\"Structure\">").unwrap();
        let data3d = &xml[start..];
        let end = start + data3d.find("</data3D>").unwrap();
        let entry = &xml[start..end];
        format!("{}{}{}", &xml[..start], entry.repeat(500), &xml[end..])
    });
    let start = std::time::Instant::now();
    let err = read_e57(&path, &Progress::new()).unwrap_err();
    assert!(start.elapsed().as_secs_f64() < 2.0, "{:?}", start.elapsed());
    let ExportError::Format { msg, .. } = err else {
        panic!("{err:?}");
    };
    assert!(
        msg.starts_with("E57: Scan „Kopie“ gibt 50 Punkte an, die Datei kann höchstens"),
        "{msg}"
    );
    std::fs::remove_dir_all(dir).unwrap();
}
