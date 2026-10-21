//! `dense.bin` / `clean.bin`: the cloud (and rays) between the steps of `driftless map`.

use driftless_map::{load_stage, save_stage, stage_fits, Cloud, RaySource, StageError};
use std::fs;
use std::path::PathBuf;

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-stage-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn cloud() -> Cloud {
    Cloud {
        xyz: vec![[1.0, -2.5, 0.125], [1e-7, 3.3, -999.0], [0.0, 0.0, 0.0]],
        intensity: vec![0.0, 17.25, 255.0],
        t: vec![1_760_000_000.123_456_7, 1_760_000_000.2, 1_760_000_001.0],
        frame: vec![0, 1, 4_000_000_000],
        rgb: None,
        colored: None,
    }
}

fn rays() -> RaySource {
    RaySource {
        sensor: vec![[0.0, 0.0, 1.5], [0.1, 0.0, 1.5]],
        ends: vec![[3.0, 0.0, 0.0], [0.0, 4.0, 0.0], [1.0, 1.0, 1.0]],
        frame_start: vec![0, 2, 3],
    }
}

#[test]
fn stage_round_trips_bit_exact() {
    let dir = tmp("roundtrip");
    let path = dir.join("dense.bin");
    save_stage(&path, "id a", &cloud(), Some(&rays())).unwrap();
    let (c, r) = load_stage(&path, "id a").unwrap();
    assert_eq!(c, cloud());
    assert_eq!(r, Some(rays()));

    let path = dir.join("clean.bin");
    save_stage(&path, "id b", &cloud(), None).unwrap();
    let (c, r) = load_stage(&path, "id b").unwrap();
    assert_eq!(c, cloud());
    assert_eq!(r, None);

    // Same input, same bytes.
    save_stage(&dir.join("again.bin"), "id b", &cloud(), None).unwrap();
    assert_eq!(
        fs::read(&path).unwrap(),
        fs::read(dir.join("again.bin")).unwrap()
    );
}

#[test]
fn stage_colours_are_not_stored() {
    let dir = tmp("colours");
    let path = dir.join("x.bin");
    let mut c = cloud();
    c.rgb = Some(vec![[1, 2, 3]; 3]);
    c.colored = Some(vec![true; 3]);
    save_stage(&path, "id", &c, None).unwrap();
    assert_eq!(load_stage(&path, "id").unwrap().0, cloud());
}

#[test]
fn stage_rejects_other_input_and_missing_file() {
    let dir = tmp("other");
    let path = dir.join("dense.bin");
    assert!(matches!(
        load_stage(&path, "id"),
        Err(StageError::Missing(_))
    ));
    save_stage(&path, "id", &cloud(), Some(&rays())).unwrap();
    assert!(matches!(
        load_stage(&path, "id2"),
        Err(StageError::OtherInput)
    ));
}

#[test]
fn stage_fits_reads_only_the_header() {
    let dir = tmp("fits");
    let path = dir.join("dense.bin");
    assert!(!stage_fits(&path, "id"));
    save_stage(&path, "id", &cloud(), Some(&rays())).unwrap();
    assert!(stage_fits(&path, "id"));
    assert!(!stage_fits(&path, "id2"));
    // the body is not read: a cut-off stage with an intact header still fits
    let bytes = fs::read(&path).unwrap();
    fs::write(&path, &bytes[..16]).unwrap();
    assert!(stage_fits(&path, "id"));
    fs::write(&path, &bytes[..15]).unwrap();
    assert!(!stage_fits(&path, "id"));
    fs::write(&path, b"nope").unwrap();
    assert!(!stage_fits(&path, "id"));
}

#[test]
fn stage_rejects_damage_without_panicking() {
    let dir = tmp("damage");
    let path = dir.join("dense.bin");
    save_stage(&path, "id", &cloud(), Some(&rays())).unwrap();
    let good = fs::read(&path).unwrap();
    let bad = dir.join("bad.bin");

    // Every cut fails cleanly.
    for cut in 0..good.len() {
        fs::write(&bad, &good[..cut]).unwrap();
        assert!(load_stage(&bad, "id").is_err(), "cut at {cut}");
    }
    // Trailing bytes.
    let mut long = good.clone();
    long.push(0);
    fs::write(&bad, &long).unwrap();
    assert!(matches!(
        load_stage(&bad, "id"),
        Err(StageError::Invalid(_))
    ));
    // Wrong magic and version.
    let mut v = good.clone();
    v[0] = b'X';
    fs::write(&bad, &v).unwrap();
    assert!(matches!(load_stage(&bad, "id"), Err(StageError::NotAStage)));
    let mut v = good.clone();
    v[4] = 99;
    fs::write(&bad, &v).unwrap();
    assert!(matches!(
        load_stage(&bad, "id"),
        Err(StageError::WrongVersion(99))
    ));

    // Header: magic, version, hash (16 bytes), then the point count.
    let count_at = 16;
    let mut v = good.clone();
    v[count_at..count_at + 8].copy_from_slice(&u64::MAX.to_le_bytes());
    fs::write(&bad, &v).unwrap();
    assert!(matches!(load_stage(&bad, "id"), Err(StageError::Truncated)));

    // Point records of 28 bytes: x y z intensity (f32), t (f64), frame (u32).
    let first = count_at + 8;
    let patch = |at: usize, bytes: &[u8]| {
        let mut v = good.clone();
        v[at..at + bytes.len()].copy_from_slice(bytes);
        fs::write(&bad, &v).unwrap();
        load_stage(&bad, "id")
    };
    for (at, bytes) in [
        (first, f32::NAN.to_le_bytes().to_vec()),
        (first + 4, 2e6f32.to_le_bytes().to_vec()),
        (first + 12, f32::INFINITY.to_le_bytes().to_vec()),
        (first + 16, f64::NAN.to_le_bytes().to_vec()),
    ] {
        assert!(
            matches!(patch(at, &bytes), Err(StageError::Invalid(_))),
            "byte {at}"
        );
    }

    // Rays: flag, frame count, sensors, then the frame starts.
    let rays_at = first + 3 * 28;
    let starts = rays_at + 1 + 8 + 2 * 12;
    assert!(matches!(
        patch(starts, &1u32.to_le_bytes()),
        Err(StageError::Invalid(_))
    ));
    assert!(matches!(
        patch(starts + 4, &9u32.to_le_bytes()),
        Err(StageError::Invalid(_))
    ));
    assert!(matches!(patch(rays_at, &[7]), Err(StageError::Invalid(_))));
}
