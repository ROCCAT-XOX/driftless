//! The program symbol, drawn here rather than in a drawing program: a rounded square in
//! [`DARK.surface`](crate::ui::theme::DARK) with a curved run of dots in the accent colour – a
//! figure eight that closes where it started, in the middle. Edges are smoothed by 4×4
//! samples per pixel; there is no text.
//!
//! The drawing uses only `+`, `−`, `×`, `÷` and `sqrt` (no `sin`/`cos` of the platform's maths
//! library), so it is the same byte for byte on every platform. `tests/icon.rs` checks that
//! `assets/icon/driftless.{png,ico,icns}` are exactly what it draws.

use crate::ui::theme::DARK;

/// Samples per pixel along each axis.
const SUB: u32 = 4;
/// Steps of the dense polyline the dots are spaced along.
const CURVE_STEPS: usize = 4096;
/// cos and sin of 2π / [`CURVE_STEPS`], written out so no maths library is asked.
const STEP_COS: f64 = 0.999_998_823_451_701_9;
const STEP_SIN: f64 = 0.001_533_980_186_284_765_5;
/// cos and sin of −20°: the eight leans a little, so the run looks under way.
const TILT_COS: f64 = 0.939_692_620_785_908_4;
const TILT_SIN: f64 = -0.342_020_143_325_668_7;
/// Half the width and the height of the eight (fractions of the symbol).
const EIGHT_X: f64 = 0.29;
const EIGHT_Y: f64 = 0.40;
/// Dots along the run (those under the dot in the middle are left out).
const DOTS: usize = 30;
/// Radius of a dot and of the dot in the middle, where the run starts and ends (fractions).
const DOT_R: f64 = 0.024;
const MIDDLE_R: f64 = 0.052;
/// Dots closer than this to the middle are left out: the dot in the middle covers them.
const MIDDLE_GAP: f64 = 0.08;
/// A dot and the dot in the middle never get smaller than this (pixels), so the run stays
/// visible at 16 px.
const MIN_DOT_PX: f64 = 0.95;
const MIN_MIDDLE_PX: f64 = 1.5;

/// Sizes in the Windows icon.
const ICO_SIZES: [u32; 7] = [16, 24, 32, 48, 64, 128, 256];
/// Entries of the macOS icon as Apple's `iconutil` writes them: type and size in pixels.
/// 16 and 32 at 1x are raw ARGB (`ic04`, `ic05`); macOS reads `icp4`–`icp6` as raw data too,
/// so PNG goes only into the types made for it.
const ICNS_ENTRIES: [(&[u8; 4], u32); 10] = [
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
/// The types of [`ICNS_ENTRIES`] that hold ARGB rather than PNG.
const ICNS_ARGB: [&[u8; 4]; 2] = [b"ic04", b"ic05"];

/// The symbol as RGBA (not premultiplied), `size` × `size`, rows from the top.
pub fn icon_rgba(size: u32) -> Vec<u8> {
    let n = size as usize;
    let s = f64::from(size);
    // The square leaves a margin as macOS symbols do (824 of 1024); small sizes leave one pixel.
    let margin = if size >= 128 {
        s * 100.0 / 1024.0
    } else {
        s / 16.0
    };
    let half = s / 2.0 - margin;
    let radius = 0.225 * 2.0 * half;
    let centre = s / 2.0;
    let in_square = |x: f64, y: f64| {
        let qx = ((x - centre).abs() - (half - radius)).max(0.0);
        let qy = ((y - centre).abs() - (half - radius)).max(0.0);
        // inside the straight parts, or within `radius` of the corner's centre
        (x - centre).abs() <= half
            && (y - centre).abs() <= half
            && qx * qx + qy * qy <= radius * radius
    };

    // Which samples of each pixel a dot covers, one bit per sample.
    let mut dot_mask = vec![0u16; n * n];
    for (cx, cy, r) in dots() {
        let (cx, cy) = (cx * s, cy * s);
        let min = if r == MIDDLE_R {
            MIN_MIDDLE_PX
        } else {
            MIN_DOT_PX
        };
        let r = (r * s).max(min);
        let lo = |v: f64| ((v - r).floor().max(0.0)) as usize;
        let hi = |v: f64| ((v + r).ceil().min(s)) as usize;
        for py in lo(cy)..hi(cy) {
            for px in lo(cx)..hi(cx) {
                let mut bits = 0u16;
                for (k, (x, y)) in samples(px, py).enumerate() {
                    let (dx, dy) = (x - cx, y - cy);
                    if dx * dx + dy * dy <= r * r {
                        bits |= 1 << k;
                    }
                }
                dot_mask[py * n + px] |= bits;
            }
        }
    }

    let [sr, sg, sb, _] = DARK.surface.to_array();
    let [ar, ag, ab, _] = DARK.accent.to_array();
    let mut out = vec![0u8; n * n * 4];
    for py in 0..n {
        for px in 0..n {
            let mut square = 0u16;
            for (k, (x, y)) in samples(px, py).enumerate() {
                if in_square(x, y) {
                    square |= 1 << k;
                }
            }
            let inside = square.count_ones();
            if inside == 0 {
                continue;
            }
            let accent = (dot_mask[py * n + px] & square).count_ones();
            let plain = inside - accent;
            let mix = |a: u8, s: u8| {
                let sum = u32::from(a) * accent + u32::from(s) * plain;
                ((2 * sum + inside) / (2 * inside)) as u8
            };
            let full = SUB * SUB;
            let o = (py * n + px) * 4;
            out[o..o + 4].copy_from_slice(&[
                mix(ar, sr),
                mix(ag, sg),
                mix(ab, sb),
                ((2 * 255 * inside + full) / (2 * full)) as u8,
            ]);
        }
    }
    out
}

/// The symbol as PNG (RGBA, 8 bit).
pub fn icon_png(size: u32) -> Vec<u8> {
    let mut bytes = Vec::new();
    let mut encoder = png::Encoder::new(&mut bytes, size, size);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    // fdeflate and one fixed filter: the same bytes whatever zlib the rest of the program uses
    encoder.set_deflate_compression(png::DeflateCompression::FdeflateUltraFast);
    encoder.set_filter(png::Filter::Paeth);
    let mut writer = encoder.write_header().expect("PNG header into memory");
    writer
        .write_image_data(&icon_rgba(size))
        .expect("PNG data into memory");
    writer.finish().expect("PNG end into memory");
    bytes
}

/// The Windows icon: sizes 16, 24, 32, 48, 64, 128 and 256 as PNG entries.
pub fn icon_ico() -> Vec<u8> {
    let images: Vec<(u32, Vec<u8>)> = ICO_SIZES.iter().map(|&s| (s, icon_png(s))).collect();
    let count = images.len() as u16;
    let mut out = Vec::new();
    // ICONDIR: reserved, type 1 (icon), number of images
    for v in [0u16, 1, count] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    let mut offset = 6 + 16 * images.len();
    for (size, png) in &images {
        // 256 is written as 0
        let side = if *size >= 256 { 0 } else { *size as u8 };
        out.extend_from_slice(&[side, side, 0, 0]);
        out.extend_from_slice(&1u16.to_le_bytes()); // planes
        out.extend_from_slice(&32u16.to_le_bytes()); // bits per pixel
        out.extend_from_slice(&(png.len() as u32).to_le_bytes());
        out.extend_from_slice(&(offset as u32).to_le_bytes());
        offset += png.len();
    }
    for (_, png) in &images {
        out.extend_from_slice(png);
    }
    out
}

/// The macOS icon, with the entries Apple's `iconutil` writes: `ic04` 16 and `ic05` 32 as
/// ARGB ([`icns_argb`]); `ic11` 32, `ic12` 64, `ic07` 128, `ic13` 256, `ic08` 256, `ic14` 512,
/// `ic09` 512 and `ic10` 1024 as PNG.
pub fn icon_icns() -> Vec<u8> {
    let mut body = Vec::new();
    for (kind, size) in ICNS_ENTRIES {
        let data = if ICNS_ARGB.contains(&kind) {
            icns_argb(&icon_rgba(size))
        } else {
            icon_png(size)
        };
        body.extend_from_slice(kind);
        body.extend_from_slice(&((data.len() + 8) as u32).to_be_bytes());
        body.extend_from_slice(&data);
    }
    let mut out = Vec::with_capacity(body.len() + 8);
    out.extend_from_slice(b"icns");
    out.extend_from_slice(&((body.len() + 8) as u32).to_be_bytes());
    out.extend_from_slice(&body);
    out
}

/// An `ic04`/`ic05` entry of the RGBA `rgba` (not premultiplied): `ARGB`, then the alpha,
/// red, green and blue planes one after the other, colours premultiplied by alpha (as macOS
/// reads them), each plane packed: a byte `n` < 128 is followed by `n + 1` bytes as they are,
/// a byte `n` ≥ 128 by one byte repeated `n − 125` times.
pub fn icns_argb(rgba: &[u8]) -> Vec<u8> {
    let mut out = b"ARGB".to_vec();
    for channel in [3, 0, 1, 2] {
        let plane: Vec<u8> = rgba
            .as_chunks::<4>()
            .0
            .iter()
            .map(|p| {
                let (v, a) = (u32::from(p[channel]), u32::from(p[3]));
                if channel == 3 {
                    p[3]
                } else {
                    ((v * a + 127) / 255) as u8
                }
            })
            .collect();
        let mut i = 0;
        let mut literal: Vec<u8> = Vec::new();
        let flush = |literal: &mut Vec<u8>, out: &mut Vec<u8>| {
            for chunk in literal.chunks(128) {
                out.push((chunk.len() - 1) as u8);
                out.extend_from_slice(chunk);
            }
            literal.clear();
        };
        while i < plane.len() {
            let run = plane[i..]
                .iter()
                .take(130)
                .take_while(|&&b| b == plane[i])
                .count();
            if run >= 3 {
                flush(&mut literal, &mut out);
                out.extend_from_slice(&[(run + 125) as u8, plane[i]]);
                i += run;
            } else {
                literal.push(plane[i]);
                i += 1;
            }
        }
        flush(&mut literal, &mut out);
    }
    out
}

/// The sample points of pixel (`px`, `py`), in pixels.
fn samples(px: usize, py: usize) -> impl Iterator<Item = (f64, f64)> {
    let step = 1.0 / f64::from(SUB);
    (0..SUB * SUB).map(move |k| {
        let (i, j) = (f64::from(k % SUB), f64::from(k / SUB));
        (px as f64 + (i + 0.5) * step, py as f64 + (j + 0.5) * step)
    })
}

/// The dots of the run: centre and radius as fractions of the symbol. The eight
/// `x = sin t, y = sin t · cos t` (tilted) is walked from the middle, the dots evenly
/// spaced along it; the last one is the larger dot in the middle where the run starts and
/// closes.
fn dots() -> Vec<(f64, f64, f64)> {
    // The dense polyline; (c, s) = (cos t, sin t) turned on step by step.
    let mut line = Vec::with_capacity(CURVE_STEPS + 1);
    let (mut c, mut s) = (1.0f64, 0.0f64);
    for _ in 0..=CURVE_STEPS {
        let (x, y) = (EIGHT_X * s, EIGHT_Y * s * c);
        line.push((
            0.5 + TILT_COS * x - TILT_SIN * y,
            0.5 + TILT_SIN * x + TILT_COS * y,
        ));
        (c, s) = (c * STEP_COS - s * STEP_SIN, s * STEP_COS + c * STEP_SIN);
    }
    let lengths: Vec<f64> = line
        .windows(2)
        .map(|w| {
            let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
            (dx * dx + dy * dy).sqrt()
        })
        .collect();
    let total: f64 = lengths.iter().sum();
    let mut out = Vec::with_capacity(DOTS);
    let (mut i, mut walked) = (0, 0.0);
    // half a step from the middle, so the dots on both sides of it are evenly spaced
    for k in 0..DOTS {
        let at = total * (k as f64 + 0.5) / DOTS as f64;
        while walked + lengths[i] < at {
            walked += lengths[i];
            i += 1;
        }
        let f = (at - walked) / lengths[i];
        let (a, b) = (line[i], line[i + 1]);
        let (x, y) = (a.0 + f * (b.0 - a.0), a.1 + f * (b.1 - a.1));
        let (dx, dy) = (x - 0.5, y - 0.5);
        if dx * dx + dy * dy >= MIDDLE_GAP * MIDDLE_GAP {
            out.push((x, y, DOT_R));
        }
    }
    out.push((0.5, 0.5, MIDDLE_R));
    out
}
