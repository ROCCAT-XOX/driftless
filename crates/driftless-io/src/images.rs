//! JPEG decoding and a streaming reader for camera images stored in bags.

use crate::bag::{BagError, BagReader, RawMessage};
use crate::msg::{decode_compressed_image, MsgError};
use rayon::prelude::*;
use std::path::PathBuf;
use zune_jpeg::zune_core::bytestream::ZCursor;
use zune_jpeg::zune_core::colorspace::ColorSpace;
use zune_jpeg::zune_core::options::DecoderOptions;
use zune_jpeg::JpegDecoder;

/// Maximum number of images decoded concurrently (and therefore held in memory).
const MAX_WORKERS: usize = 4;
/// Largest accepted JPEG side length; guards against huge allocations from crafted headers.
const MAX_DIMENSION: usize = 8192;

/// Errors produced while decoding an image.
#[derive(Debug, thiserror::Error)]
pub enum ImageError {
    #[error("ungültige Bildnachricht: {0}")]
    Message(#[from] MsgError),
    #[error("JPEG konnte nicht dekodiert werden: {0}")]
    Jpeg(String),
    #[error("ungültiger Verkleinerungsfaktor {0} (erlaubt: 1, 2, 4)")]
    Downscale(u32),
}

/// An 8-bit RGB image, row-major.
#[derive(Debug, Clone, PartialEq)]
pub struct RgbImage {
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

impl RgbImage {
    /// Gray values (BT.601 weights, integer arithmetic), row-major.
    pub fn to_gray(&self) -> Vec<u8> {
        self.data
            .as_chunks::<3>()
            .0
            .iter()
            .map(|p| {
                ((77 * u32::from(p[0]) + 150 * u32::from(p[1]) + 29 * u32::from(p[2]) + 128) >> 8)
                    as u8
            })
            .collect()
    }
}

/// One camera image from a bag.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Camera name derived from the topic (`/camera_left/image_raw` -> `left`).
    pub camera: String,
    pub stamp: f64,
    pub rgb: RgbImage,
}

/// Counters of an image stream run.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ImageStats {
    /// Images decoded and delivered.
    pub frames: usize,
    /// Images that could not be decoded (delivered as `Err`).
    pub bad: usize,
    /// Bags that ended in a cut (e.g. recording stopped abruptly): the images before the
    /// cut were delivered, the rest of that bag is missing.
    pub truncated: usize,
}

/// Decodes a JPEG to RGB8. `downscale` (1, 2 or 4) averages 1x1, 2x2 or 4x4 blocks;
/// partial blocks at the right/bottom edge are dropped.
pub fn decode_jpeg(bytes: &[u8], downscale: u32) -> Result<RgbImage, ImageError> {
    if !matches!(downscale, 1 | 2 | 4) {
        return Err(ImageError::Downscale(downscale));
    }
    let options = DecoderOptions::default()
        .jpeg_set_out_colorspace(ColorSpace::RGB)
        .set_max_width(MAX_DIMENSION)
        .set_max_height(MAX_DIMENSION);
    let mut dec = JpegDecoder::new_with_options(ZCursor::new(bytes), options);
    // Read the header first so oversized images are rejected before any pixel allocation.
    dec.decode_headers()
        .map_err(|e| ImageError::Jpeg(format!("{e:?}")))?;
    let (w, h) = dec
        .dimensions()
        .ok_or_else(|| ImageError::Jpeg("keine Bildgröße".into()))?;
    if w == 0 || h == 0 || w > MAX_DIMENSION || h > MAX_DIMENSION {
        return Err(ImageError::Jpeg(format!("Bildgröße {w}x{h} nicht erlaubt")));
    }
    let expected = w
        .checked_mul(h)
        .and_then(|n| n.checked_mul(3))
        .ok_or_else(|| ImageError::Jpeg("Bild zu groß".into()))?;
    let pixels = dec
        .decode()
        .map_err(|e| ImageError::Jpeg(format!("{e:?}")))?;
    if pixels.len() != expected {
        return Err(ImageError::Jpeg("unerwartete Pixelanzahl".into()));
    }
    if downscale == 1 {
        return Ok(RgbImage {
            width: w as u32,
            height: h as u32,
            data: pixels,
        });
    }
    let d = downscale as usize;
    let (ow, oh) = (w / d, h / d);
    let mut data = Vec::with_capacity(ow * oh * 3);
    let area = (d * d) as u32;
    for oy in 0..oh {
        for ox in 0..ow {
            let mut sum = [0u32; 3];
            for dy in 0..d {
                let row = ((oy * d + dy) * w + ox * d) * 3;
                for px in pixels[row..row + d * 3].as_chunks::<3>().0 {
                    for c in 0..3 {
                        sum[c] += px[c] as u32;
                    }
                }
            }
            data.extend(sum.iter().map(|s| ((s + area / 2) / area) as u8));
        }
    }
    Ok(RgbImage {
        width: ow as u32,
        height: oh as u32,
        data,
    })
}

/// `/camera_left/image_raw` -> `left`; topics without a `camera_` segment keep their name.
fn camera_name(topic: &str) -> String {
    topic
        .split('/')
        .find_map(|seg| seg.strip_prefix("camera_").filter(|s| !s.is_empty()))
        .map(str::to_string)
        .unwrap_or_else(|| topic.trim_matches('/').to_string())
}

fn decode_message(msg: &RawMessage, downscale: u32) -> Result<Frame, ImageError> {
    let img = decode_compressed_image(&msg.data)?;
    Ok(Frame {
        camera: camera_name(&msg.topic),
        stamp: img.stamp,
        rgb: decode_jpeg(&img.data, downscale)?,
    })
}

/// Streams all images of `topics` from `bags` (in the given bag order, file order within a bag)
/// to `f`. Decoding runs on a few rayon threads, but frames reach `f` strictly in bag order and
/// at most one decoded image per worker is held at a time. Undecodable images are passed to `f`
/// as `Err` and counted in `ImageStats::bad`. A truncated bag ends like a complete one (the
/// images before the cut are kept) and is counted in `ImageStats::truncated`; any other read
/// error fails. `f` returns `false` to stop early.
pub fn for_each_image(
    bags: &[PathBuf],
    topics: &[String],
    downscale: u32,
    mut f: impl FnMut(Result<Frame, ImageError>) -> bool,
) -> Result<ImageStats, BagError> {
    let workers = rayon::current_num_threads().clamp(1, MAX_WORKERS);
    let topic_refs: Vec<&str> = topics.iter().map(String::as_str).collect();
    let mut stats = ImageStats::default();
    let mut batch: Vec<RawMessage> = Vec::with_capacity(workers);

    // Decodes the batch in parallel and delivers results in order; false = stop.
    let mut flush = |batch: &mut Vec<RawMessage>, stats: &mut ImageStats| -> bool {
        let results: Vec<_> = batch
            .par_iter()
            .map(|m| decode_message(m, downscale))
            .collect();
        batch.clear();
        for r in results {
            match &r {
                Ok(_) => stats.frames += 1,
                Err(_) => stats.bad += 1,
            }
            if !f(r) {
                return false;
            }
        }
        true
    };

    for path in bags {
        let mut reader = BagReader::open(path)?;
        for msg in reader.messages(&topic_refs) {
            let msg = match msg {
                Ok(m) => m,
                Err(BagError::Truncated { .. }) => {
                    stats.truncated += 1;
                    break;
                }
                Err(e) => return Err(e),
            };
            batch.push(msg);
            if batch.len() == workers && !flush(&mut batch, &mut stats) {
                return Ok(stats);
            }
        }
    }
    flush(&mut batch, &mut stats);
    Ok(stats)
}
