use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::images::{decode_jpeg, for_each_image, ImageError};
use driftless_io::msg::encode::encode_compressed_image;
use driftless_io::msg::CompressedImage;
use jpeg_encoder::{ColorType, Encoder};

/// Solid-colour JPEG (quality 100, no chroma subsampling issues for flat colour).
fn solid_jpeg(w: u16, h: u16, rgb: [u8; 3]) -> Vec<u8> {
    let data: Vec<u8> = (0..w as usize * h as usize).flat_map(|_| rgb).collect();
    let mut out = Vec::new();
    Encoder::new(&mut out, 100)
        .encode(&data, w, h, ColorType::Rgb)
        .unwrap();
    out
}

#[test]
fn decode_jpeg_downscale() {
    let jpg = solid_jpeg(64, 48, [200, 100, 50]);
    for (d, w, h) in [(1u32, 64u32, 48u32), (2, 32, 24), (4, 16, 12)] {
        let img = decode_jpeg(&jpg, d).unwrap();
        assert_eq!((img.width, img.height), (w, h));
        assert_eq!(img.data.len(), (w * h * 3) as usize);
        let n = (w * h) as f64;
        for c in 0..3 {
            let mean = img
                .data
                .iter()
                .skip(c)
                .step_by(3)
                .map(|&v| v as f64)
                .sum::<f64>()
                / n;
            let want = [200.0, 100.0, 50.0][c];
            assert!((mean - want).abs() < 3.0, "d={d} c={c} mean={mean}");
        }
    }
    assert!(matches!(
        decode_jpeg(&jpg, 3),
        Err(ImageError::Downscale(3))
    ));
}

#[test]
fn corrupt_jpeg_is_error_not_panic() {
    let jpg = solid_jpeg(64, 48, [10, 20, 30]);
    let _ = decode_jpeg(&jpg[..jpg.len() / 2], 1); // truncated scan data: must not panic
    assert!(decode_jpeg(&jpg[..10], 1).is_err());
    assert!(decode_jpeg(&[], 1).is_err());
    let mut x = 12345u32;
    let noise: Vec<u8> = (0..2000)
        .map(|_| {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            (x >> 24) as u8
        })
        .collect();
    assert!(decode_jpeg(&noise, 1).is_err());
    // Valid header followed by garbage must not panic either.
    let mut mixed = jpg[..30].to_vec();
    mixed.extend_from_slice(&noise);
    let _ = decode_jpeg(&mixed, 2);
}

#[test]
fn image_stream_counts_bad_images() {
    let mut w = BagWriter::new(Compression::None);
    let left = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    let right = w.add_connection("/camera_right/image_raw", "sensor_msgs/CompressedImage");
    let good = solid_jpeg(32, 24, [255, 0, 0]);
    let items: [(u32, u32, Vec<u8>); 4] = [
        (left, 1, good.clone()),
        (right, 2, good.clone()),
        (left, 3, vec![1, 2, 3, 4]),
        (right, 4, good),
    ];
    for (conn, sec, data) in items {
        let msg = encode_compressed_image(&CompressedImage {
            stamp: sec as f64,
            format: "jpeg".into(),
            data,
        });
        w.write_message(conn, Time { sec, nsec: 0 }, &msg);
    }
    let dir = std::env::temp_dir().join(format!("driftless_images_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("t.bag");
    std::fs::write(&path, w.finish()).unwrap();

    let topics = vec![
        "/camera_left/image_raw".to_string(),
        "/camera_right/image_raw".to_string(),
    ];
    let mut frames = Vec::new();
    let mut errors = 0;
    let stats = for_each_image(std::slice::from_ref(&path), &topics, 2, |r| {
        match r {
            Ok(f) => frames.push((f.camera, f.stamp, f.rgb.width)),
            Err(_) => errors += 1,
        }
        true
    })
    .unwrap();
    assert_eq!(stats.frames, 3);
    assert_eq!(stats.bad, 1);
    assert_eq!(errors, 1);
    // Bag order is preserved.
    assert_eq!(
        frames,
        vec![
            ("left".to_string(), 1.0, 16),
            ("right".to_string(), 2.0, 16),
            ("right".to_string(), 4.0, 16)
        ]
    );

    // Early stop.
    let mut n = 0;
    for_each_image(&[path], &topics, 1, |_| {
        n += 1;
        false
    })
    .unwrap();
    assert_eq!(n, 1);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn truncated_image_bag_keeps_images_before_the_cut() {
    let mut w = BagWriter::new(Compression::None).messages_per_chunk(1);
    let conn = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    let jpg = solid_jpeg(32, 24, [0, 255, 0]);
    for sec in 1..=6u32 {
        let msg = encode_compressed_image(&CompressedImage {
            stamp: sec as f64,
            format: "jpeg".into(),
            data: jpg.clone(),
        });
        w.write_message(conn, Time { sec, nsec: 0 }, &msg);
    }
    let mut bytes = w.finish();
    let cut = bytes.len() * 7 / 10;
    bytes.truncate(cut);
    let dir = std::env::temp_dir().join(format!("driftless_images_trunc_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("t.bag");
    std::fs::write(&path, bytes).unwrap();

    let topics = vec!["/camera_left/image_raw".to_string()];
    let mut stamps = Vec::new();
    let stats = for_each_image(&[path], &topics, 1, |r| {
        stamps.push(r.unwrap().stamp);
        true
    })
    .expect("a truncated bag is not an error");
    assert_eq!(stats.truncated, 1);
    assert_eq!(stats.frames, stamps.len());
    assert!(!stamps.is_empty() && stamps.len() < 6, "{stamps:?}");
    assert!(stamps.windows(2).all(|p| p[0] < p[1]));
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn oversized_header_is_rejected_without_allocating() {
    // Take a valid JPEG and patch the SOF0 dimensions to 16000x16000.
    let mut jpg = solid_jpeg(64, 48, [1, 2, 3]);
    let sof = jpg
        .windows(2)
        .position(|w| w == [0xFF, 0xC0])
        .expect("SOF0 marker");
    // marker(2) length(2) precision(1) height(2) width(2)
    jpg[sof + 5..sof + 7].copy_from_slice(&16000u16.to_be_bytes());
    jpg[sof + 7..sof + 9].copy_from_slice(&16000u16.to_be_bytes());
    let start = std::time::Instant::now();
    assert!(matches!(decode_jpeg(&jpg, 1), Err(ImageError::Jpeg(_))));
    assert!(start.elapsed().as_millis() < 500);
    // Zero dimensions are rejected as well.
    jpg[sof + 5..sof + 9].copy_from_slice(&[0, 0, 0, 0]);
    assert!(decode_jpeg(&jpg, 1).is_err());
}

#[test]
fn gray_uses_bt601_weights() {
    let img = driftless_io::images::RgbImage {
        width: 3,
        height: 1,
        data: vec![255, 255, 255, 255, 0, 0, 0, 0, 255],
    };
    assert_eq!(img.to_gray(), vec![255, 77, 29]);
}
