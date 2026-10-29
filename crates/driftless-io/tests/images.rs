use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::images::{
    decode_jpeg, for_each_image, for_each_image_where, ImageError, ImageHeader, Wanted,
};
use driftless_io::msg::encode::encode_compressed_image;
use driftless_io::msg::CompressedImage;
use driftless_io::testdir::TestDir;
use jpeg_encoder::{ColorType, Encoder};
use std::path::PathBuf;

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
    let dir = TestDir::new(format!("driftless_images_{}", std::process::id()));
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
    let dir = TestDir::new(format!("driftless_images_trunc_{}", std::process::id()));
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

/// A bag of five image messages, in this order: `left` and `right` at 1000.3 s, `left` at
/// 1000.5 s whose JPEG is broken, a `left` message too short for its header, `right` at
/// 1000.7 s. The good images are 32 × 24.
fn mixed_bag(dir: &TestDir) -> (PathBuf, Vec<String>) {
    let mut w = BagWriter::new(Compression::None);
    let left = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    let right = w.add_connection("/camera_right/image_raw", "sensor_msgs/CompressedImage");
    let good = solid_jpeg(32, 24, [0, 0, 255]);
    let image = |stamp: f64, data: &[u8]| {
        encode_compressed_image(&CompressedImage {
            stamp,
            format: "jpeg".into(),
            data: data.to_vec(),
        })
    };
    let messages = [
        (left, image(1000.3, &good)),
        (right, image(1000.3, &good)),
        (left, image(1000.5, &[1, 2, 3, 4])),
        (left, vec![0, 0, 0]),
        (right, image(1000.7, &good)),
    ];
    for (i, (conn, data)) in messages.into_iter().enumerate() {
        w.write_message(
            conn,
            Time {
                sec: i as u32,
                nsec: 0,
            },
            &data,
        );
    }
    let path = dir.join("bilder.bag");
    std::fs::write(&path, w.finish()).unwrap();
    let topics = vec![
        "/camera_left/image_raw".to_string(),
        "/camera_right/image_raw".to_string(),
    ];
    (path, topics)
}

/// What a header shows: (place in the stream, camera, stamp in ns).
fn shown(h: &ImageHeader) -> (usize, String, u64) {
    (h.index, h.camera.clone(), h.stamp_ns)
}

/// Listing the stream decodes nothing: every message with a readable header is shown with
/// its place, camera and stamp; the broken JPEG is not noticed, the broken header counts as
/// bad but keeps its place in the numbering.
#[test]
fn image_stream_lists_headers_without_decoding() {
    let dir = TestDir::new(format!("driftless_images_list_{}", std::process::id()));
    let (bag, topics) = mixed_bag(&dir);
    let mut headers = Vec::new();
    let mut delivered = 0;
    let stats = for_each_image_where(
        std::slice::from_ref(&bag),
        &topics,
        1,
        |h| {
            headers.push(shown(h));
            Wanted::No
        },
        |_| {
            delivered += 1;
            true
        },
    )
    .unwrap();
    assert_eq!(delivered, 0);
    assert_eq!(stats.frames, 0);
    assert_eq!(stats.bad, 1, "{stats:?}");
    assert_eq!(
        headers,
        [
            (0, "left".to_string(), 1_000_300_000_000),
            (1, "right".to_string(), 1_000_300_000_000),
            (2, "left".to_string(), 1_000_500_000_000),
            (4, "right".to_string(), 1_000_700_000_000),
        ]
    );
}

/// Only the wanted images are decoded, at the given size, in stream order; a broken one
/// comes as an error. The header's stamp is the frame's, bit for bit.
#[test]
fn image_stream_decodes_only_wanted_images() {
    let dir = TestDir::new(format!("driftless_images_wanted_{}", std::process::id()));
    let (bag, topics) = mixed_bag(&dir);
    let mut stamps = Vec::new();
    let mut got = Vec::new();
    let stats = for_each_image_where(
        std::slice::from_ref(&bag),
        &topics,
        2,
        |h| {
            if [1, 2, 4].contains(&h.index) {
                stamps.push(h.stamp);
                Wanted::Yes
            } else {
                Wanted::No
            }
        },
        |r| {
            got.push(r.map(|f| (f.camera, f.stamp, f.rgb.width, f.rgb.height)));
            true
        },
    )
    .unwrap();
    assert_eq!(stats.frames, 2);
    assert_eq!(stats.bad, 2, "the broken header and the broken JPEG");
    assert_eq!(got.len(), 3);
    let first = got[0].as_ref().unwrap();
    assert_eq!(first, &("right".to_string(), 1000.3, 16, 12));
    assert!(matches!(got[1], Err(ImageError::Jpeg(_))), "{:?}", got[1]);
    let last = got[2].as_ref().unwrap();
    assert_eq!(last, &("right".to_string(), 1000.7, 16, 12));
    assert_eq!(first.1.to_bits(), stamps[0].to_bits());
    assert_eq!(last.1.to_bits(), stamps[2].to_bits());

    // The same images as the full stream delivers at those places.
    let mut all = Vec::new();
    for_each_image(std::slice::from_ref(&bag), &topics, 2, |r| {
        all.push(r.ok().map(|f| f.rgb));
        true
    })
    .unwrap();
    let mut again = Vec::new();
    for_each_image_where(
        &[bag],
        &topics,
        2,
        |h| {
            if [0, 4].contains(&h.index) {
                Wanted::Yes
            } else {
                Wanted::No
            }
        },
        |r| {
            again.push(r.unwrap().rgb);
            true
        },
    )
    .unwrap();
    assert_eq!(Some(&again[0]), all[0].as_ref());
    assert_eq!(Some(&again[1]), all[4].as_ref());
}

/// `Stop` ends the stream after the images wanted so far; `f` returning `false` ends it too.
#[test]
fn image_stream_stops_where_wanted() {
    let dir = TestDir::new(format!("driftless_images_stop_{}", std::process::id()));
    let (bag, topics) = mixed_bag(&dir);
    let mut seen = Vec::new();
    let mut got = Vec::new();
    let stats = for_each_image_where(
        std::slice::from_ref(&bag),
        &topics,
        1,
        |h| {
            seen.push(h.index);
            match h.index {
                0 => Wanted::Yes,
                1 => Wanted::Stop,
                _ => Wanted::Yes,
            }
        },
        |r| {
            got.push(r.unwrap().stamp);
            true
        },
    )
    .unwrap();
    assert_eq!(seen, [0, 1]);
    assert_eq!(got, [1000.3]);
    assert_eq!(stats.frames, 1);

    let mut n = 0;
    for_each_image_where(
        &[bag],
        &topics,
        1,
        |_| Wanted::Yes,
        |_| {
            n += 1;
            false
        },
    )
    .unwrap();
    assert_eq!(n, 1);
}
