//! `submaps.bin`: round trip and every way a damaged or foreign file must fail.

use driftless_odom::FrameQuality;
use driftless_slam::{load_cache, read_cache, save_cache, CacheError, Submap, SubmapFrame};
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::fs;
use std::path::PathBuf;

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-cache-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir.join("submaps.bin")
}

fn iso(x: f64, yaw: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, -0.1 * x, 1.0 / 3.0),
        UnitQuaternion::from_euler_angles(0.01, -0.02, yaw),
    )
}

/// Three submaps with awkward values (thirds, negative zero, a failed ICP frame).
fn submaps() -> (Vec<Submap>, Vec<Isometry3<f64>>) {
    let mut out = Vec::new();
    for id in 0..3 {
        let frames = (0..4)
            .map(|i| SubmapFrame {
                index: id * 4 + i,
                t: 1000.0 + (id * 4 + i) as f64 / 3.0,
                offset: iso(i as f64 * 0.37, i as f64 * 0.1),
                quality: FrameQuality {
                    residual_rms: 0.01 / 3.0,
                    inlier_ratio: 0.9,
                    degeneracy: 0.15,
                    icp_ok: i != 2,
                },
            })
            .collect();
        out.push(Submap {
            id,
            anchor: iso(id as f64 * 5.0, id as f64),
            frames,
            points: vec![[-0.0, 1.0 / 3.0, 7.5], [1e-30, -2.5, 1.0e6]],
        });
    }
    let odom = out
        .iter()
        .map(|m| m.anchor * iso(0.01, 0.001))
        .collect::<Vec<_>>();
    (out, odom)
}

#[test]
fn cache_round_trip() {
    let path = tmp("round-trip");
    let (s, odom) = submaps();
    save_cache(&path, &s, &odom, "LIDAR_0.bag 1234 cfg").unwrap();
    let first = fs::read(&path).unwrap();
    assert_eq!(&first[..4], b"DLSM");
    assert_eq!(u32::from_le_bytes(first[4..8].try_into().unwrap()), 1);

    let (s2, odom2) = load_cache(&path, "LIDAR_0.bag 1234 cfg").unwrap();
    assert_eq!(format!("{s:?}"), format!("{s2:?}"));
    assert_eq!(odom, odom2);
    // bit-identical: writing the loaded submaps again gives the same bytes
    save_cache(&path, &s2, &odom2, "LIDAR_0.bag 1234 cfg").unwrap();
    assert_eq!(first, fs::read(&path).unwrap());
    assert_eq!(read_cache(&path).unwrap().scan_start, 1000.0);
}

fn hint(e: &CacheError) {
    let msg = e.to_string();
    assert!(
        msg.contains("`driftless slam` ohne `--reuse` neu laufen lassen"),
        "{msg}"
    );
}

#[test]
fn cache_rejects_other_input() {
    let path = tmp("other");
    let (s, odom) = submaps();
    save_cache(&path, &s, &odom, "LIDAR_0.bag 1234 cfg").unwrap();
    let e = load_cache(&path, "LIDAR_0.bag 1235 cfg").unwrap_err();
    assert!(matches!(e, CacheError::OtherInput), "{e:?}");
    assert!(e.to_string().contains("anderen Scan"), "{e}");
    hint(&e);
}

#[test]
fn cache_rejects_missing_and_wrong_version() {
    let path = tmp("version");
    let e = load_cache(&path, "x").unwrap_err();
    assert!(matches!(e, CacheError::Missing(_)), "{e:?}");
    hint(&e);

    let (s, odom) = submaps();
    save_cache(&path, &s, &odom, "x").unwrap();
    let mut bytes = fs::read(&path).unwrap();
    bytes[4] = 2;
    fs::write(&path, &bytes).unwrap();
    let e = load_cache(&path, "x").unwrap_err();
    assert!(matches!(e, CacheError::WrongVersion(2)), "{e:?}");
    assert!(e.to_string().contains("Version 2"), "{e}");
    hint(&e);

    bytes[0] = b'X';
    fs::write(&path, &bytes).unwrap();
    assert!(matches!(
        load_cache(&path, "x").unwrap_err(),
        CacheError::NotACache
    ));
}

#[test]
fn cache_rejects_truncated() {
    let path = tmp("truncated");
    let (s, odom) = submaps();
    save_cache(&path, &s, &odom, "x").unwrap();
    let bytes = fs::read(&path).unwrap();
    for n in 0..bytes.len() {
        fs::write(&path, &bytes[..n]).unwrap();
        let e = load_cache(&path, "x").unwrap_err();
        assert!(matches!(e, CacheError::Truncated), "{n} bytes: {e:?}");
        hint(&e);
    }
    let mut longer = bytes.clone();
    longer.push(0);
    fs::write(&path, &longer).unwrap();
    assert!(matches!(
        load_cache(&path, "x").unwrap_err(),
        CacheError::Invalid(_)
    ));
}

/// Byte offsets of the length fields of the first submap: submap count, frame count, point
/// count.
const SUBMAP_COUNT: usize = 4 + 4 + 8 + 8;
const FRAME_COUNT: usize = SUBMAP_COUNT + 8 + 8 + 2 * 56;
const POINT_COUNT: usize = FRAME_COUNT + 8 + 4 * (8 + 8 + 56 + 24 + 1);

#[test]
fn cache_rejects_huge_length_field() {
    let path = tmp("huge");
    let (s, odom) = submaps();
    save_cache(&path, &s, &odom, "x").unwrap();
    let bytes = fs::read(&path).unwrap();
    assert_eq!(
        u64::from_le_bytes(bytes[POINT_COUNT..POINT_COUNT + 8].try_into().unwrap()),
        2,
        "layout of the test offsets"
    );
    for at in [SUBMAP_COUNT, FRAME_COUNT, POINT_COUNT] {
        for huge in [u64::MAX, u64::MAX / 12 + 1, 1 << 40] {
            let mut b = bytes.clone();
            b[at..at + 8].copy_from_slice(&huge.to_le_bytes());
            fs::write(&path, &b).unwrap();
            let e = load_cache(&path, "x").unwrap_err();
            assert!(matches!(e, CacheError::Truncated), "{at}/{huge}: {e:?}");
        }
    }
}

#[test]
fn cache_rejects_unusable_submaps() {
    let path = tmp("invalid");
    let (mut s, odom) = submaps();
    s[1].frames.clear();
    save_cache(&path, &s, &odom, "x").unwrap();
    let e = load_cache(&path, "x").unwrap_err();
    assert!(matches!(e, CacheError::Invalid(_)), "{e:?}");
    hint(&e);

    let (mut s, odom) = submaps();
    s[2].anchor.translation.x = f64::NAN;
    save_cache(&path, &s, &odom, "x").unwrap();
    assert!(matches!(
        load_cache(&path, "x").unwrap_err(),
        CacheError::Invalid(_)
    ));
}

/// Finite but absurd coordinates (beyond 1000 km) are damage, not data.
#[test]
fn cache_rejects_huge_coordinates() {
    let path = tmp("huge-coord");
    type Damage = fn(&mut Vec<Submap>, &mut Vec<Isometry3<f64>>);
    let damage: [Damage; 4] = [
        |s, _| s[0].anchor.translation.y = 1.0e7,
        |_, o| o[1].translation.z = -2.0e6,
        |s, _| s[2].frames[1].offset.translation.x = 1.0e300,
        |s, _| s[1].points[0][0] = -3.0e6,
    ];
    for (k, f) in damage.iter().enumerate() {
        let (mut s, mut odom) = submaps();
        f(&mut s, &mut odom);
        save_cache(&path, &s, &odom, "x").unwrap();
        let e = load_cache(&path, "x").unwrap_err();
        assert!(matches!(e, CacheError::Invalid(_)), "{k}: {e:?}");
        assert!(e.to_string().contains("beschädigt"), "{k}: {e}");
    }
}
