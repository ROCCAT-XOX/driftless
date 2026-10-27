//! The program symbol: the files checked in under `assets/icon/` are exactly what
//! `driftless_app::icon` draws (`UPDATE_ICONS=1` writes them anew), and the drawing has
//! transparent corners and the accent colour in the middle.

use driftless_app::icon::{icns_argb, icon_icns, icon_ico, icon_png, icon_rgba};
use driftless_app::ui::theme::DARK;
use std::path::PathBuf;

fn assets() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/icon")
}

#[test]
fn icons_match_committed() {
    let files = [
        ("driftless.png", icon_png(1024)),
        ("driftless.ico", icon_ico()),
        ("driftless.icns", icon_icns()),
    ];
    let update = std::env::var_os("UPDATE_ICONS").is_some_and(|v| v == "1");
    if update {
        std::fs::create_dir_all(assets()).unwrap();
    }
    for (name, bytes) in files {
        let path = assets().join(name);
        if update {
            std::fs::write(&path, &bytes).unwrap();
            continue;
        }
        let committed = std::fs::read(&path)
            .unwrap_or_else(|e| panic!("{}: {e} (UPDATE_ICONS=1 writes it)", path.display()));
        assert!(
            committed == bytes,
            "{name} differs from the drawing (UPDATE_ICONS=1 writes it anew)"
        );
    }
}

#[test]
fn icon_is_deterministic() {
    assert_eq!(icon_rgba(64), icon_rgba(64));
    assert_eq!(icon_png(48), icon_png(48));
}

#[test]
fn icon_has_transparent_corners_and_accent_centre() {
    for size in [16u32, 32, 256, 1024] {
        let rgba = icon_rgba(size);
        assert_eq!(rgba.len(), (size * size * 4) as usize);
        let px = |x: u32, y: u32| {
            let i = ((y * size + x) * 4) as usize;
            [rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]]
        };
        let last = size - 1;
        for (x, y) in [(0, 0), (last, 0), (0, last), (last, last)] {
            assert_eq!(px(x, y)[3], 0, "corner ({x}, {y}) of {size}");
        }
        let [r, g, b, _] = DARK.accent.to_array();
        assert_eq!(px(size / 2, size / 2), [r, g, b, 255], "centre of {size}");
        // the rounded square between the corners and the middle
        let [r, g, b, _] = DARK.surface.to_array();
        assert_eq!(px(size / 2, size / 6), [r, g, b, 255], "surface of {size}");
    }
}

/// A PNG of the icon decodes to its RGBA drawing.
fn decode(png: &[u8]) -> (u32, Vec<u8>) {
    let img = image::load_from_memory_with_format(png, image::ImageFormat::Png).unwrap();
    let img = img.to_rgba8();
    assert_eq!(img.width(), img.height());
    (img.width(), img.into_raw())
}

#[test]
fn png_decodes_to_the_drawing() {
    let (size, rgba) = decode(&icon_png(128));
    assert_eq!(size, 128);
    assert_eq!(rgba, icon_rgba(128));
}

#[test]
fn ico_has_all_sizes_as_png() {
    let ico = icon_ico();
    let u16_at = |i: usize| u16::from_le_bytes([ico[i], ico[i + 1]]);
    let u32_at = |i: usize| u32::from_le_bytes(ico[i..i + 4].try_into().unwrap());
    assert_eq!((u16_at(0), u16_at(2)), (0, 1), "ICONDIR of an icon");
    let sizes = [16u32, 24, 32, 48, 64, 128, 256];
    assert_eq!(u16_at(4) as usize, sizes.len());
    let mut end = 6 + 16 * sizes.len();
    for (n, size) in sizes.into_iter().enumerate() {
        let e = 6 + 16 * n;
        let stored = |b: u8| if b == 0 { 256 } else { u32::from(b) };
        assert_eq!((stored(ico[e]), stored(ico[e + 1])), (size, size));
        assert_eq!((u16_at(e + 4), u16_at(e + 6)), (1, 32), "planes, bits");
        let (len, offset) = (u32_at(e + 8) as usize, u32_at(e + 12) as usize);
        assert_eq!(offset, end, "entries follow each other");
        end = offset + len;
        let (s, rgba) = decode(&ico[offset..end]);
        assert_eq!(s, size);
        assert_eq!(rgba, icon_rgba(size));
    }
    assert_eq!(end, ico.len());
}

/// `rgba` with the colours premultiplied by alpha, as `ic04`/`ic05` hold them.
fn premultiplied(rgba: &[u8]) -> Vec<u8> {
    rgba.as_chunks::<4>()
        .0
        .iter()
        .flat_map(|p| {
            let a = u32::from(p[3]);
            let m = |v: u8| ((u32::from(v) * a + 127) / 255) as u8;
            [m(p[0]), m(p[1]), m(p[2]), p[3]]
        })
        .collect()
}

/// Unpacks an `ic04`/`ic05` entry (`ARGB` and four packed planes) to RGBA, premultiplied.
fn unpack_argb(data: &[u8], size: u32) -> Vec<u8> {
    assert_eq!(&data[..4], b"ARGB");
    let n = (size * size) as usize;
    let mut planes = Vec::new();
    let mut i = 4;
    for _ in 0..4 {
        let mut plane = Vec::with_capacity(n);
        while plane.len() < n {
            let c = data[i] as usize;
            if c < 128 {
                plane.extend_from_slice(&data[i + 1..i + 2 + c]);
                i += 2 + c;
            } else {
                plane.extend(std::iter::repeat_n(data[i + 1], c - 125));
                i += 2;
            }
        }
        assert_eq!(plane.len(), n, "a run crosses the end of a plane");
        planes.push(plane);
    }
    assert_eq!(i, data.len());
    (0..n)
        .flat_map(|p| [planes[1][p], planes[2][p], planes[3][p], planes[0][p]])
        .collect()
}

#[test]
fn icns_has_all_sizes() {
    let icns = icon_icns();
    let u32_at = |i: usize| u32::from_be_bytes(icns[i..i + 4].try_into().unwrap());
    assert_eq!(&icns[..4], b"icns");
    assert_eq!(u32_at(4) as usize, icns.len());
    // as Apple's iconutil writes them; 16 and 32 at 1x are ARGB, the rest PNG
    let expected: [(&[u8; 4], u32); 10] = [
        (b"ic04", 16),
        (b"ic05", 32),
        (b"ic11", 32),
        (b"ic12", 64),
        (b"ic07", 128),
        (b"ic13", 256),
        (b"ic08", 256),
        (b"ic14", 512),
        (b"ic09", 512),
        (b"ic10", 1024),
    ];
    let mut at = 8;
    for (kind, size) in expected {
        assert_eq!(&icns[at..at + 4], kind);
        let len = u32_at(at + 4) as usize;
        let data = &icns[at + 8..at + len];
        let (rgba, expected) = if kind == b"ic04" || kind == b"ic05" {
            (unpack_argb(data, size), premultiplied(&icon_rgba(size)))
        } else {
            let (s, rgba) = decode(data);
            assert_eq!(s, size);
            (rgba, icon_rgba(size))
        };
        assert!(rgba == expected, "{}", String::from_utf8_lossy(kind));
        at += len;
    }
    assert_eq!(at, icns.len());
}

#[test]
fn argb_packs_runs_and_literals() {
    // 2×2 pixels: alpha 255 ×4 (a run), red 1 2 3 4 (literal), green 0 ×4, blue 9 9 8 8
    let rgba = [1, 0, 9, 255, 2, 0, 9, 255, 3, 0, 8, 255, 4, 0, 8, 255];
    let packed = icns_argb(&rgba);
    let planes: [&[u8]; 4] = [&[129, 255], &[3, 1, 2, 3, 4], &[129, 0], &[3, 9, 9, 8, 8]];
    assert_eq!(packed, [b"ARGB".as_slice(), &planes.concat()].concat());
    assert_eq!(unpack_argb(&packed, 2), rgba);
    // colours are premultiplied: half covered white is grey
    assert_eq!(
        unpack_argb(&icns_argb(&[255, 255, 255, 128]), 1),
        [128, 128, 128, 128]
    );
    // long runs are cut at 130: 400 = 3 × 130 + 10 per plane
    let rgba: Vec<u8> = std::iter::repeat_n([7, 7, 7, 255], 400).flatten().collect();
    let packed = icns_argb(&rgba);
    let run = |v| [255, v, 255, v, 255, v, 135, v];
    assert_eq!(packed[4..], [run(255), run(7), run(7), run(7)].concat());
    assert_eq!(unpack_argb(&packed, 20), rgba);
    // long literals are cut at 128 (the red plane starts after the 8 bytes of alpha)
    let rgba: Vec<u8> = (0..400u32).flat_map(|i| [i as u8, 0, 0, 255]).collect();
    let packed = icns_argb(&rgba);
    assert_eq!(packed[12], 127);
    assert_eq!(packed[12 + 129], 127);
    assert_eq!(unpack_argb(&packed, 20), rgba);
}
