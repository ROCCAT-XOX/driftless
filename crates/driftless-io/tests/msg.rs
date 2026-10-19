use driftless_io::bag::BagReader;
use driftless_io::msg::encode::{encode_compressed_image, encode_imu, encode_pointcloud2};
use driftless_io::msg::{
    decode_compressed_image, decode_imu, decode_pointcloud2, CompressedImage, ImuSample,
    LidarPoint, MsgError,
};

fn pt(xyz: [f32; 3], intensity: f32, ring: u16, t_rel: f64) -> LidarPoint {
    LidarPoint {
        xyz,
        intensity,
        ring,
        t_rel,
    }
}

fn string(out: &mut Vec<u8>, s: &str) {
    out.extend_from_slice(&(s.len() as u32).to_le_bytes());
    out.extend_from_slice(s.as_bytes());
}

/// Hand-built PointCloud2 (independent of the encoder). `fields`: (name, offset, datatype).
fn raw_cloud(fields: &[(&str, u32, u8)], step: u32, big_endian: bool, data: &[u8]) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&7u32.to_le_bytes()); // seq
    b.extend_from_slice(&100u32.to_le_bytes()); // sec
    b.extend_from_slice(&500_000_000u32.to_le_bytes()); // nsec
    string(&mut b, "lidar");
    b.extend_from_slice(&1u32.to_le_bytes()); // height
    b.extend_from_slice(&(data.len() as u32 / step).to_le_bytes()); // width
    b.extend_from_slice(&(fields.len() as u32).to_le_bytes());
    for (name, off, dt) in fields {
        string(&mut b, name);
        b.extend_from_slice(&off.to_le_bytes());
        b.push(*dt);
        b.extend_from_slice(&1u32.to_le_bytes());
    }
    b.push(big_endian as u8);
    b.extend_from_slice(&step.to_le_bytes());
    b.extend_from_slice(&(data.len() as u32).to_le_bytes());
    b.extend_from_slice(&(data.len() as u32).to_le_bytes());
    b.extend_from_slice(data);
    b.push(1);
    b
}

fn vanjee_point(x: f32, y: f32, z: f32, i: f32, ring: u16, t: f64) -> Vec<u8> {
    let mut d = Vec::new();
    for v in [x, y, z, i] {
        d.extend_from_slice(&v.to_le_bytes());
    }
    d.extend_from_slice(&ring.to_le_bytes());
    d.extend_from_slice(&t.to_le_bytes());
    d
}

const VANJEE: [(&str, u32, u8); 6] = [
    ("x", 0, 7),
    ("y", 4, 7),
    ("z", 8, 7),
    ("intensity", 12, 7),
    ("ring", 16, 4),
    ("timestamp", 18, 8),
];

#[test]
fn pointcloud2_vanjee_layout() {
    // Hand-built bytes: encoder and decoder cannot share a bug here.
    let mut data = Vec::new();
    data.extend(vanjee_point(1.0, 2.0, 3.0, 10.0, 1, 0.0));
    data.extend(vanjee_point(-4.5, 5.25, 6.0, 20.0, 9, 0.05));
    data.extend(vanjee_point(7.0, -8.0, 0.5, 255.0, 16, 0.1));
    let f = decode_pointcloud2(&raw_cloud(&VANJEE, 26, false, &data)).unwrap();
    assert_eq!(f.stamp, 100.5);
    assert_eq!(f.dropped, 0);
    assert_eq!(
        f.points,
        vec![
            pt([1.0, 2.0, 3.0], 10.0, 1, 0.0),
            pt([-4.5, 5.25, 6.0], 20.0, 9, 0.05),
            pt([7.0, -8.0, 0.5], 255.0, 16, 0.1),
        ]
    );
}

#[test]
fn pointcloud2_roundtrip_via_encoder() {
    let pts = vec![
        pt([1.0, 2.0, 3.0], 10.0, 1, 0.0),
        pt([-1.5, 0.25, 9.0], 3.0, 16, 0.0999),
    ];
    let f = decode_pointcloud2(&encode_pointcloud2(1234.25, "lidar", &pts)).unwrap();
    assert_eq!(f.stamp, 1234.25);
    assert_eq!(f.points, pts);
    assert_eq!(f.dropped, 0);
}

#[test]
fn pointcloud2_drops_nan_and_origin() {
    let mut data = Vec::new();
    data.extend(vanjee_point(f32::NAN, 1.0, 1.0, 1.0, 1, 0.0));
    data.extend(vanjee_point(0.0, 0.0, 0.0, 1.0, 1, 0.0));
    data.extend(vanjee_point(1.0, 2.0, 3.0, 1.0, 2, 0.01));
    data.extend(vanjee_point(f32::INFINITY, 1.0, 1.0, 1.0, 1, 0.0));
    let f = decode_pointcloud2(&raw_cloud(&VANJEE, 26, false, &data)).unwrap();
    assert_eq!(f.points.len(), 1);
    assert_eq!(f.dropped, 3);
    assert_eq!(f.points[0].xyz, [1.0, 2.0, 3.0]);
}

#[test]
fn pointcloud2_missing_optional_fields() {
    let mut data = Vec::new();
    for v in [1.0f32, 2.0, 3.0] {
        data.extend_from_slice(&v.to_le_bytes());
    }
    let fields = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7)];
    let f = decode_pointcloud2(&raw_cloud(&fields, 12, false, &data)).unwrap();
    assert_eq!(f.points, vec![pt([1.0, 2.0, 3.0], 0.0, 0, 0.0)]);
}

#[test]
fn pointcloud2_converts_other_datatypes_and_offsets() {
    // step 16: pad u8 @0, x f64 @1 ... fields in odd order with int types.
    let mut d = vec![0xAA];
    d.extend_from_slice(&1.5f64.to_le_bytes()); // x f64 @1
    d.extend_from_slice(&(-2i16).to_le_bytes()); // y i16 @9
    d.extend_from_slice(&7u8.to_le_bytes()); // z u8 @11
    d.extend_from_slice(&300u32.to_le_bytes()); // intensity u32 @12
    let fields = [("z", 11, 2), ("y", 9, 3), ("x", 1, 8), ("intensity", 12, 6)];
    let f = decode_pointcloud2(&raw_cloud(&fields, 16, false, &d)).unwrap();
    assert_eq!(f.points, vec![pt([1.5, -2.0, 7.0], 300.0, 0, 0.0)]);
}

#[test]
fn pointcloud2_rejects_bad_input() {
    let d = vanjee_point(1.0, 2.0, 3.0, 1.0, 1, 0.0);
    assert!(matches!(
        decode_pointcloud2(&raw_cloud(&VANJEE, 26, true, &d)),
        Err(MsgError::Unsupported(_))
    ));
    let no_z = [("x", 0, 7), ("y", 4, 7)];
    assert!(matches!(
        decode_pointcloud2(&raw_cloud(&no_z, 26, false, &d)),
        Err(MsgError::Malformed(_))
    ));
    // field beyond point_step
    let bad = [("x", 0, 7), ("y", 4, 7), ("z", 24, 7)];
    assert!(matches!(
        decode_pointcloud2(&raw_cloud(&bad, 26, false, &d)),
        Err(MsgError::Malformed(_))
    ));
    let full = raw_cloud(&VANJEE, 26, false, &d);
    assert!(matches!(
        decode_pointcloud2(&full[..full.len() - 10]),
        Err(MsgError::Truncated { .. })
    ));
}

/// Cloud with explicit height/width/row_step and raw data, one x,y,z f32 point = 12 bytes.
fn shaped_cloud(
    height: u32,
    width: u32,
    step: u32,
    row_step: u32,
    data_len: u32,
    data: &[u8],
) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&[0u8; 12]);
    string(&mut b, "l");
    b.extend_from_slice(&height.to_le_bytes());
    b.extend_from_slice(&width.to_le_bytes());
    b.extend_from_slice(&3u32.to_le_bytes());
    for (name, off) in [("x", 0u32), ("y", 4), ("z", 8)] {
        string(&mut b, name);
        b.extend_from_slice(&off.to_le_bytes());
        b.push(7);
        b.extend_from_slice(&1u32.to_le_bytes());
    }
    b.push(0);
    b.extend_from_slice(&step.to_le_bytes());
    b.extend_from_slice(&row_step.to_le_bytes());
    b.extend_from_slice(&data_len.to_le_bytes());
    b.extend_from_slice(data);
    b.push(1);
    b
}

fn xyz_bytes(n: usize) -> Vec<u8> {
    (0..n * 3)
        .flat_map(|i| (i as f32 + 1.0).to_le_bytes())
        .collect()
}

#[test]
fn pointcloud2_rejects_zero_point_step() {
    let b = shaped_cloud(1, 2, 0, 0, 24, &xyz_bytes(2));
    assert!(matches!(
        decode_pointcloud2(&b),
        Err(MsgError::Malformed(_))
    ));
}

#[test]
fn pointcloud2_rejects_row_step_too_small() {
    let d = xyz_bytes(4);
    for rs in [0u32, 12] {
        let b = shaped_cloud(2, 2, 12, rs, 48, &d);
        assert!(
            matches!(decode_pointcloud2(&b), Err(MsgError::Malformed(_))),
            "row_step {rs}"
        );
    }
    // Sufficient row_step works.
    let ok = decode_pointcloud2(&shaped_cloud(2, 2, 12, 24, 48, &d)).unwrap();
    assert_eq!(ok.points.len(), 4);
}

#[test]
fn pointcloud2_huge_height_zero_width_is_fast_or_error() {
    let start = std::time::Instant::now();
    let b = shaped_cloud(u32::MAX, 0, 12, 12, 0, &[]);
    match decode_pointcloud2(&b) {
        Ok(f) => assert!(f.points.is_empty() && f.dropped == 0),
        Err(_) => {}
    }
    assert!(start.elapsed() < std::time::Duration::from_millis(500));
}

#[test]
fn pointcloud2_rejects_count_exceeding_data() {
    let b = shaped_cloud(1, 1_000_000, 12, 12_000_000, 24, &xyz_bytes(2));
    assert!(matches!(
        decode_pointcloud2(&b),
        Err(MsgError::Malformed(_))
    ));
    let b = shaped_cloud(u32::MAX, u32::MAX, 12, 0, 24, &xyz_bytes(2));
    assert!(decode_pointcloud2(&b).is_err());
}

#[test]
fn imu_hand_built_layout() {
    let mut b = Vec::new();
    b.extend_from_slice(&1u32.to_le_bytes());
    b.extend_from_slice(&50u32.to_le_bytes());
    b.extend_from_slice(&250_000_000u32.to_le_bytes());
    string(&mut b, "imu");
    let f64s =
        |b: &mut Vec<u8>, v: &[f64]| v.iter().for_each(|x| b.extend_from_slice(&x.to_le_bytes()));
    f64s(&mut b, &[9.0, 9.0, 9.0, 9.0]); // orientation
    f64s(&mut b, &[8.0; 9]);
    f64s(&mut b, &[0.5, -0.25, 2.0]); // angular velocity
    f64s(&mut b, &[7.0; 9]);
    f64s(&mut b, &[1.0, 2.0, 3.0]); // linear acceleration
    f64s(&mut b, &[6.0; 9]);
    let s = decode_imu(&b).unwrap();
    assert_eq!(s.stamp, 50.25);
    assert_eq!(s.gyro, [0.5, -0.25, 2.0]);
    assert_eq!(s.accel, [1.0, 2.0, 3.0]);
}

#[test]
fn compressed_image_truncated() {
    let img = CompressedImage {
        stamp: 1.0,
        format: "jpeg".into(),
        data: vec![1, 2, 3, 4],
    };
    let b = encode_compressed_image(&img);
    for cut in [1, 3, b.len() - 1] {
        assert!(
            matches!(
                decode_compressed_image(&b[..b.len() - cut]),
                Err(MsgError::Truncated { .. })
            ),
            "cut {cut}"
        );
    }
}

#[test]
fn imu_roundtrip() {
    let s = ImuSample {
        stamp: 42.125,
        gyro: [0.1, 0.2, 0.3],
        accel: [0.0, 0.0, 9.81],
    };
    assert_eq!(decode_imu(&encode_imu(&s)).unwrap(), s);
    assert!(matches!(
        decode_imu(&encode_imu(&s)[..60]),
        Err(MsgError::Truncated { .. })
    ));
}

#[test]
fn compressed_image_jpeg() {
    let img = CompressedImage {
        stamp: 5.5,
        format: "jpeg".into(),
        data: vec![0xFF, 0xD8, 1, 2, 3, 0xFF, 0xD9],
    };
    let d = decode_compressed_image(&encode_compressed_image(&img)).unwrap();
    assert_eq!(d.format, "jpeg");
    assert_eq!(d, img);
}

#[test]
#[ignore]
fn real_frames_have_4800_points_max() {
    let Some(path) = driftless_io::testdata::path("Test/LIDAR_Test.bag") else {
        eprintln!("SKIPPED: DRIFTLESS_TESTDATA not set");
        return;
    };
    let mut reader = BagReader::open(&path).unwrap();
    let mut n = 0;
    for m in reader.messages(&["/vanjee_722z"]).take(50) {
        let m = m.unwrap();
        let f = decode_pointcloud2(&m.data).unwrap();
        assert!(f.points.len() <= 16 * 300, "{} points", f.points.len());
        assert!(!f.points.is_empty());
        for p in &f.points {
            assert!((1..=16).contains(&p.ring), "ring {}", p.ring);
            assert!((0.0..=0.11).contains(&p.t_rel), "t_rel {}", p.t_rel);
        }
        n += 1;
    }
    assert_eq!(n, 50);
}
