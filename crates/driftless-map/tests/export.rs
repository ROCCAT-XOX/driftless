use driftless_io::testdir::TestDir;
use driftless_map::{write_las, write_las_sources, write_ply, write_ply_sources, Cloud};
use las::Reader;

fn temp_dir(name: &str) -> TestDir {
    TestDir::new(format!("driftless-map-{}-{name}", std::process::id()))
}

fn cloud(rgb: bool) -> Cloud {
    Cloud {
        xyz: vec![[1.25, -2.5, 0.0001], [101.3, 7.75, -3.2], [-0.5, 0.5, 12.0]],
        intensity: vec![0.0, 10.5, 255.0],
        t: vec![1792499801.25, 1792499801.35, 1792499802.0],
        frame: vec![0, 1, 2],
        rgb: rgb.then(|| vec![[255, 0, 0], [0, 128, 255], [7, 8, 9]]),
        colored: rgb.then(|| vec![true, true, false]),
    }
}

#[test]
fn las_round_trip_with_rgb() {
    let dir = temp_dir("las-rgb");
    let path = dir.join("color.las");
    let c = cloud(true);
    write_las(&path, &c).unwrap();

    let mut r = Reader::from_path(&path).unwrap();
    let h = r.header().clone();
    assert_eq!((h.version().major, h.version().minor), (1, 4));
    assert_eq!(h.point_format().to_u8().unwrap(), 7);
    assert_eq!(h.number_of_points(), 3);
    assert_eq!(h.transforms().x.scale, 0.0001);
    // x from -0.5 to 101.3, y from -2.5 to 7.75, z from -3.2 to 12.0.
    let t = h.transforms();
    assert_eq!(
        [t.x.offset, t.y.offset, t.z.offset],
        [50.0, 3.0, 4.0],
        "offset = rounded middle"
    );
    let points: Vec<las::Point> = r.read_all().unwrap().points().map(Result::unwrap).collect();
    for (i, p) in points.iter().enumerate() {
        let want = c.xyz[i];
        assert!(
            (p.x - f64::from(want[0])).abs() < 1e-4,
            "x {} vs {}",
            p.x,
            want[0]
        );
        assert!((p.y - f64::from(want[1])).abs() < 1e-4);
        assert!((p.z - f64::from(want[2])).abs() < 1e-4);
        assert_eq!(p.gps_time, Some(c.t[i]));
        let col = p.color.unwrap();
        let rgb = c.rgb.as_ref().unwrap()[i];
        assert_eq!(
            [col.red, col.green, col.blue],
            rgb.map(|v| u16::from(v) * 257)
        );
    }
    // 0..255 spread over the u16 range.
    assert_eq!(
        points.iter().map(|p| p.intensity).collect::<Vec<_>>(),
        vec![0, 2699, 65535]
    );
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn las_without_rgb_is_format_6() {
    let dir = temp_dir("las-plain");
    let path = dir.join("dense.las");
    write_las(&path, &cloud(false)).unwrap();
    let mut r = Reader::from_path(&path).unwrap();
    assert_eq!(r.header().point_format().to_u8().unwrap(), 6);
    let points: Vec<las::Point> = r.read_all().unwrap().points().map(Result::unwrap).collect();
    assert_eq!(points.len(), 3);
    assert!(points
        .iter()
        .all(|p| p.color.is_none() && p.gps_time.is_some()));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn ply_header_and_size() {
    let dir = temp_dir("ply");
    for rgb in [false, true] {
        let path = dir.join(format!("cloud-{rgb}.ply"));
        let c = cloud(rgb);
        write_ply(&path, &c).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let end = b"end_header\n";
        let header_len = bytes
            .windows(end.len())
            .position(|w| w == end)
            .expect("end_header")
            + end.len();
        let header = std::str::from_utf8(&bytes[..header_len]).unwrap();
        let mut want = String::from(
            "ply\nformat binary_little_endian 1.0\nelement vertex 3\nproperty float x\n\
             property float y\nproperty float z\nproperty float intensity\n",
        );
        if rgb {
            want += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
            want += "property uchar colored\n";
        }
        want += "end_header\n";
        assert_eq!(header, want);
        let stride = if rgb { 20 } else { 16 };
        assert_eq!(bytes.len(), header_len + 3 * stride);
        // Second vertex: x, then intensity, then colour.
        let v = &bytes[header_len + stride..header_len + 2 * stride];
        assert_eq!(f32::from_le_bytes(v[0..4].try_into().unwrap()), 101.3);
        assert_eq!(f32::from_le_bytes(v[12..16].try_into().unwrap()), 10.5);
        if rgb {
            assert_eq!(&v[16..19], &[0, 128, 255]);
            assert_eq!(v[19], 1);
            let last = &bytes[header_len + 2 * stride..];
            assert_eq!(last[19], 0, "third point has no measured colour");
        }
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn select_splits_kept_and_removed() {
    let c = cloud(true);
    let (kept, removed) = c.select(&[true, false, true]);
    assert_eq!(kept.len(), 2);
    assert_eq!(removed.len(), 1);
    assert_eq!(kept.frame, vec![0, 2]);
    assert_eq!(removed.xyz, vec![[101.3, 7.75, -3.2]]);
    assert_eq!(removed.rgb, Some(vec![[0, 128, 255]]));
    assert_eq!(kept.colored, Some(vec![true, false]));
    let (kept, removed) = cloud(false).select(&[false, false, false]);
    assert!(kept.is_empty() && kept.rgb.is_none());
    assert_eq!(removed.len(), 3);
}

#[test]
fn las_user_data_marks_uncolored() {
    let dir = temp_dir("las-ud");
    let path = dir.join("color.las");
    write_las(&path, &cloud(true)).unwrap();
    let mut r = Reader::from_path(&path).unwrap();
    let ud: Vec<u8> = r
        .read_all()
        .unwrap()
        .points()
        .map(|p| p.unwrap().user_data)
        .collect();
    assert_eq!(ud, vec![0, 0, 1]);
    write_las(&path, &cloud(false)).unwrap();
    let mut r = Reader::from_path(&path).unwrap();
    assert!(r
        .read_all()
        .unwrap()
        .points()
        .all(|p| p.unwrap().user_data == 0));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn sources_go_to_point_source_id_and_scan() {
    let dir = temp_dir("sources");
    let c = cloud(true);
    let sources = [0u16, 2, 65535];
    let las = dir.join("modell.las");
    write_las_sources(&las, &c, &sources).unwrap();
    let mut r = Reader::from_path(&las).unwrap();
    let points: Vec<las::Point> = r.read_all().unwrap().points().map(Result::unwrap).collect();
    assert_eq!(
        points.iter().map(|p| p.point_source_id).collect::<Vec<_>>(),
        sources
    );
    assert_eq!(
        points.iter().map(|p| p.user_data).collect::<Vec<_>>(),
        vec![0, 0, 1],
        "user data as in write_las"
    );
    // Without sources every point has source 0.
    write_las(&las, &c).unwrap();
    let mut r = Reader::from_path(&las).unwrap();
    assert!(r
        .read_all()
        .unwrap()
        .points()
        .all(|p| p.unwrap().point_source_id == 0));

    let ply = dir.join("modell.ply");
    write_ply_sources(&ply, &c, &sources).unwrap();
    let bytes = std::fs::read(&ply).unwrap();
    let end = b"end_header\n";
    let header_len = bytes.windows(end.len()).position(|w| w == end).unwrap() + end.len();
    let header = std::str::from_utf8(&bytes[..header_len]).unwrap();
    assert!(header.ends_with("property uchar colored\nproperty ushort scan\nend_header\n"));
    let stride = 22;
    assert_eq!(bytes.len(), header_len + 3 * stride);
    for (k, s) in sources.iter().enumerate() {
        let v = &bytes[header_len + k * stride..header_len + (k + 1) * stride];
        assert_eq!(u16::from_le_bytes([v[20], v[21]]), *s);
    }
    // Without colours the scan follows the intensity.
    write_ply_sources(&ply, &cloud(false), &sources).unwrap();
    let bytes = std::fs::read(&ply).unwrap();
    let header_len = bytes.windows(end.len()).position(|w| w == end).unwrap() + end.len();
    assert_eq!(bytes.len(), header_len + 3 * 18);
    assert_eq!(
        u16::from_le_bytes([bytes[header_len + 16], bytes[header_len + 17]]),
        0
    );
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The PLY bytes of `color.ply` and `modell.ply` stay as they were before the writer moved
/// to `driftless-export` (hashes taken from the writer of `driftless-map` before the move).
#[test]
fn color_ply_unchanged_by_move() {
    let dir = temp_dir("ply-move");
    let path = dir.join("color.ply");
    let hash = |path: &std::path::Path| driftless_map::content_hash(&std::fs::read(path).unwrap());
    write_ply(&path, &cloud(true)).unwrap();
    let plain = hash(&path);
    write_ply_sources(&path, &cloud(true), &[0, 2, 65535]).unwrap();
    let sources = hash(&path);
    write_ply(&path, &cloud(false)).unwrap();
    let gray = hash(&path);
    assert_eq!(
        (plain, sources, gray),
        (
            0x04f1_c6b7_2722_ce22,
            0x4df6_fd05_dd01_4823,
            0x4f16_34ba_8b77_f1b2
        )
    );
    std::fs::remove_dir_all(&dir).unwrap();
}
