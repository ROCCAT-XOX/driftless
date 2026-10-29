//! JPEG decoding and a streaming reader for camera images stored in bags; an index of the
//! images of a stream ([`index_images`]) to read just the ones asked for ([`read_indexed`]).

use crate::bag::{BagError, BagReader, ChunkCompression, MessagePlace, RawMessage};
use crate::msg::{compressed_image_stamp, decode_compressed_image, stamp_secs, MsgError};
use crate::progress::Progress;
use rayon::prelude::*;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::SystemTime;
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
    #[error(transparent)]
    Bag(#[from] BagError),
    /// A bag is no longer the one its images were indexed in (its size or modification
    /// time, or an image's header, changed).
    #[error("{0} hat sich geändert, seit ihre Bilder verzeichnet wurden")]
    Changed(String),
    #[error("abgebrochen")]
    Cancelled,
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

/// Decodes `batch` in parallel and delivers the results to `f` in order, counting them in
/// `stats`; empties the batch. `false` once `f` asks to stop.
fn deliver(
    batch: &mut Vec<RawMessage>,
    downscale: u32,
    stats: &mut ImageStats,
    f: &mut impl FnMut(Result<Frame, ImageError>) -> bool,
) -> bool {
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
            if batch.len() == workers && !deliver(&mut batch, downscale, &mut stats, &mut f) {
                return Ok(stats);
            }
        }
    }
    deliver(&mut batch, downscale, &mut stats, &mut f);
    Ok(stats)
}

/// An image message as its header tells it, before the image is decoded.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageHeader {
    /// Its place in the stream: the image messages of the topics over the bags in order,
    /// counted from 0, as [`for_each_image`] delivers them (the undecodable ones too).
    pub index: usize,
    /// Camera name derived from the topic, as [`Frame::camera`].
    pub camera: String,
    /// The header's stamp (s), as [`Frame::stamp`] of the decoded image, bit for bit.
    pub stamp: f64,
    /// The same stamp in nanoseconds, exact.
    pub stamp_ns: u64,
}

/// What [`for_each_image_where`] does with an image, decided by its [`ImageHeader`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wanted {
    /// Decode it and pass it on.
    Yes,
    /// Pass over it: neither decoded nor passed on.
    No,
    /// End the stream here: neither this image nor any after it is read; the images wanted
    /// before it are still decoded and passed on.
    Stop,
}

/// [`for_each_image`] for the images `want` picks: every image message of `topics` in `bags`,
/// in the same order, is first shown to `want` by its header ([`ImageHeader`]), without
/// decoding; only the images it wants are decoded (at 1/`downscale`, a few at a time on rayon
/// threads, as there) and passed to `f`, in stream order. So the images of a stream are listed
/// without decoding a single one (`want` says [`Wanted::No`] to all), and a few are picked from
/// it at the cost of reading the bags once ([`Wanted::Stop`] after the last one saves the
/// rest). A message whose header cannot be read is not shown to `want` and counts as bad; a
/// wanted image that cannot be decoded reaches `f` as `Err` and counts as bad as well
/// ([`ImageStats::frames`] counts the decoded ones). Bags end at a cut as there. `f` returns
/// `false` to stop early.
pub fn for_each_image_where(
    bags: &[PathBuf],
    topics: &[String],
    downscale: u32,
    mut want: impl FnMut(&ImageHeader) -> Wanted,
    mut f: impl FnMut(Result<Frame, ImageError>) -> bool,
) -> Result<ImageStats, BagError> {
    let workers = rayon::current_num_threads().clamp(1, MAX_WORKERS);
    let topic_refs: Vec<&str> = topics.iter().map(String::as_str).collect();
    let mut stats = ImageStats::default();
    let mut batch: Vec<RawMessage> = Vec::with_capacity(workers);
    let mut index = 0usize;

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
            let at = index;
            index += 1;
            let Ok((sec, nsec)) = compressed_image_stamp(&msg.data) else {
                stats.bad += 1;
                continue;
            };
            let header = ImageHeader {
                index: at,
                camera: camera_name(&msg.topic),
                stamp: stamp_secs(sec, nsec),
                stamp_ns: u64::from(sec) * 1_000_000_000 + u64::from(nsec),
            };
            match want(&header) {
                Wanted::Yes => batch.push(msg),
                Wanted::No => continue,
                Wanted::Stop => {
                    deliver(&mut batch, downscale, &mut stats, &mut f);
                    return Ok(stats);
                }
            }
            if batch.len() == workers && !deliver(&mut batch, downscale, &mut stats, &mut f) {
                return Ok(stats);
            }
        }
    }
    deliver(&mut batch, downscale, &mut stats, &mut f);
    Ok(stats)
}

/// Bytes of an image message that hold its header's stamp ([`compressed_image_stamp`]).
const STAMP_BYTES: usize = 12;

/// At most this many indices are kept by [`ImageIndices`]; the oldest goes first.
const KEPT_INDICES: usize = 32;

/// A bag as it was when its images were indexed.
#[derive(Debug, Clone, PartialEq, Eq)]
struct BagStamp {
    path: PathBuf,
    len: u64,
    modified: Option<SystemTime>,
}

impl BagStamp {
    fn of(path: &Path) -> Result<BagStamp, ImageError> {
        let meta = std::fs::metadata(path).map_err(BagError::Io)?;
        Ok(BagStamp {
            path: path.to_path_buf(),
            len: meta.len(),
            modified: meta.modified().ok(),
        })
    }

    /// Whether the file still has its size and modification time.
    fn current(&self) -> bool {
        BagStamp::of(&self.path).is_ok_and(|now| now == *self)
    }
}

/// One image message of an indexed stream.
#[derive(Debug, Clone, PartialEq)]
struct Indexed {
    /// Its header; `None` where it cannot be read (the message counts as bad).
    header: Option<ImageHeader>,
    /// Its bag (index into [`ImageIndex::bags`]) and its place there.
    bag: u32,
    topic: Arc<str>,
    place: MessagePlace,
}

/// Where the images of a stream – the image messages of some topics in some bags, in order, as
/// [`for_each_image`] goes through them – lie, found once ([`index_images`]): per image its
/// header and its place in its bag, so that [`read_indexed`] reads only the images asked for.
/// It belongs to the bags as they were (path, size, modification time;
/// [`ImageIndex::is_current`]).
#[derive(Debug, Clone, PartialEq)]
pub struct ImageIndex {
    topics: Vec<String>,
    bags: Vec<BagStamp>,
    images: Vec<Indexed>,
    truncated: usize,
}

impl ImageIndex {
    /// The headers of the images, in stream order, as [`for_each_image_where`] shows them: a
    /// message whose header cannot be read is left out, but keeps its place in the numbering
    /// ([`ImageHeader::index`]).
    pub fn headers(&self) -> impl Iterator<Item = &ImageHeader> + '_ {
        self.images.iter().filter_map(|i| i.header.as_ref())
    }

    /// The image messages of the stream, also those whose header cannot be read.
    pub fn len(&self) -> usize {
        self.images.len()
    }

    pub fn is_empty(&self) -> bool {
        self.images.is_empty()
    }

    /// The bags ended in a cut (as [`ImageStats::truncated`]): the images before it are
    /// indexed.
    pub fn truncated(&self) -> usize {
        self.truncated
    }

    /// Whether it is the index of the stream of `topics` in `bags`.
    pub fn is_of(&self, bags: &[PathBuf], topics: &[String]) -> bool {
        self.topics == topics
            && self.bags.len() == bags.len()
            && self.bags.iter().zip(bags).all(|(b, p)| b.path == *p)
    }

    /// Whether every bag still has the size and modification time it had when it was indexed.
    pub fn is_current(&self) -> bool {
        self.bags.iter().all(BagStamp::current)
    }
}

/// Indexes the images of `topics` in `bags` (see [`ImageIndex`]): the messages of the stream as
/// [`for_each_image_where`] shows them, with the same numbering, found without decoding an image
/// and mostly without reading the chunks whole ([`BagReader::message_places`]). Bags end at a cut
/// as there. Progress: one step per image message, in the phase the caller set; a cancel is
/// checked at each and returns [`ImageError::Cancelled`].
///
/// Fails if a bag cannot be read (as [`for_each_image`] fails).
pub fn index_images(
    bags: &[PathBuf],
    topics: &[String],
    progress: &Progress,
) -> Result<ImageIndex, ImageError> {
    let topic_refs: Vec<&str> = topics.iter().map(String::as_str).collect();
    let mut index = ImageIndex {
        topics: topics.to_vec(),
        bags: Vec::with_capacity(bags.len()),
        images: Vec::new(),
        truncated: 0,
    };
    for (b, path) in bags.iter().enumerate() {
        if progress.is_cancelled() {
            return Err(ImageError::Cancelled);
        }
        // Before it is read: a bag changed meanwhile is not current afterwards.
        index.bags.push(BagStamp::of(path)?);
        let mut reader = BagReader::open(path)?;
        let mut topic: Option<Arc<str>> = None;
        for msg in reader.message_places(&topic_refs, STAMP_BYTES) {
            if progress.is_cancelled() {
                return Err(ImageError::Cancelled);
            }
            let msg = match msg {
                Ok(m) => m,
                Err(BagError::Truncated { .. }) => {
                    index.truncated += 1;
                    break;
                }
                Err(e) => return Err(e.into()),
            };
            let at = index.images.len();
            let header = compressed_image_stamp(&msg.head)
                .ok()
                .map(|(sec, nsec)| ImageHeader {
                    index: at,
                    camera: camera_name(&msg.topic),
                    stamp: stamp_secs(sec, nsec),
                    stamp_ns: u64::from(sec) * 1_000_000_000 + u64::from(nsec),
                });
            let topic = match &topic {
                Some(t) if **t == *msg.topic => Arc::clone(t),
                _ => Arc::clone(topic.insert(Arc::from(msg.topic.as_str()))),
            };
            index.images.push(Indexed {
                header,
                bag: b as u32,
                topic,
                place: msg.place,
            });
            progress.advance(1);
        }
    }
    Ok(index)
}

/// Reads and decodes (at 1/`downscale`) the images of `index` at the places `frames`
/// ([`ImageHeader::index`]; any order, each once) and passes them to `f` in stream order, with
/// their place: the same frames, byte for byte, as [`for_each_image`] delivers there (an image
/// that cannot be decoded comes as `Err` and counts as bad). Only these images are read: the
/// message where it lies, or its chunk decompressed whole ([`BagReader::read_message`]). A few
/// are decoded at a time on rayon threads. `f` returns `false` to stop.
///
/// Fails with [`ImageError::Changed`] if a bag is no longer as it was indexed (its size,
/// modification time or the header of an image read), or if a bag cannot be read.
///
/// # Panics
/// If a place is not one of the index.
pub fn read_indexed(
    index: &ImageIndex,
    frames: &[usize],
    downscale: u32,
    mut f: impl FnMut(usize, Result<Frame, ImageError>) -> bool,
) -> Result<ImageStats, ImageError> {
    let mut wanted = frames.to_vec();
    wanted.sort_unstable();
    wanted.dedup();
    if let Some(&last) = wanted.last() {
        assert!(last < index.len(), "image {last} of {}", index.len());
    }
    if let Some(bag) = index.bags.iter().find(|b| !b.current()) {
        return Err(ImageError::Changed(bag.path.display().to_string()));
    }
    let workers = rayon::current_num_threads().max(1);
    let mut stats = ImageStats::default();
    let mut readers: Vec<Option<BagReader<std::io::BufReader<std::fs::File>>>> =
        (0..index.bags.len()).map(|_| None).collect();
    // The records of the last lz4 chunk read: (bag, chunk), records.
    let mut chunk: Option<((u32, u64), Vec<u8>)> = None;
    let mut batch: Vec<(usize, RawMessage)> = Vec::with_capacity(workers);
    for &i in &wanted {
        let image = &index.images[i];
        let bag = &index.bags[image.bag as usize];
        let reader = match &mut readers[image.bag as usize] {
            Some(r) => r,
            slot => slot.insert(BagReader::open(&bag.path)?),
        };
        let place = &image.place;
        let data = match place.compression {
            ChunkCompression::None => reader.read_message(place)?,
            ChunkCompression::Lz4 => {
                let key = (image.bag, place.chunk_pos);
                if chunk.as_ref().is_none_or(|c| c.0 != key) {
                    chunk = Some((key, reader.read_chunk(place)?));
                }
                let records = &chunk.as_ref().expect("the chunk just read").1;
                let range = place.offset as usize..(place.offset + place.len) as usize;
                records
                    .get(range)
                    .ok_or_else(|| ImageError::Changed(bag.path.display().to_string()))?
                    .to_vec()
            }
        };
        // Still the message indexed there: its header as it was.
        let now = compressed_image_stamp(&data).ok();
        let then = image.header.as_ref().map(|h| {
            (
                (h.stamp_ns / 1_000_000_000) as u32,
                (h.stamp_ns % 1_000_000_000) as u32,
            )
        });
        if now != then {
            return Err(ImageError::Changed(bag.path.display().to_string()));
        }
        batch.push((
            i,
            RawMessage {
                conn_id: 0,
                topic: image.topic.to_string(),
                time: crate::bag::Time { sec: 0, nsec: 0 },
                data,
            },
        ));
        if batch.len() == workers && !deliver_indexed(&mut batch, downscale, &mut stats, &mut f) {
            return Ok(stats);
        }
    }
    deliver_indexed(&mut batch, downscale, &mut stats, &mut f);
    Ok(stats)
}

/// [`deliver`] for [`read_indexed`]: with each image its place.
fn deliver_indexed(
    batch: &mut Vec<(usize, RawMessage)>,
    downscale: u32,
    stats: &mut ImageStats,
    f: &mut impl FnMut(usize, Result<Frame, ImageError>) -> bool,
) -> bool {
    let results: Vec<_> = batch
        .par_iter()
        .map(|(i, m)| (*i, decode_message(m, downscale)))
        .collect();
    batch.clear();
    for (i, r) in results {
        match &r {
            Ok(_) => stats.frames += 1,
            Err(_) => stats.bad += 1,
        }
        if !f(i, r) {
            return false;
        }
    }
    true
}

/// Indices of image streams kept between runs ([`index_images`]): one per stream (its bags
/// and topics), built the first time it is asked for and again once a bag has changed (its size
/// or modification time). At most 32 are kept (`KEPT_INDICES`). Shared between threads.
#[derive(Debug, Default)]
pub struct ImageIndices {
    kept: Mutex<Vec<Arc<ImageIndex>>>,
}

impl ImageIndices {
    /// The index of the stream of `topics` in `bags`: the one kept if its bags are as they
    /// were, else one built now ([`index_images`], with `progress`) and kept.
    pub fn get(
        &self,
        bags: &[PathBuf],
        topics: &[String],
        progress: &Progress,
    ) -> Result<Arc<ImageIndex>, ImageError> {
        let lock = || self.kept.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(found) = lock()
            .iter()
            .find(|i| i.is_of(bags, topics) && i.is_current())
        {
            return Ok(Arc::clone(found));
        }
        let built = Arc::new(index_images(bags, topics, progress)?);
        let mut kept = lock();
        kept.retain(|i| !i.is_of(bags, topics));
        if kept.len() >= KEPT_INDICES {
            kept.remove(0);
        }
        kept.push(Arc::clone(&built));
        Ok(built)
    }

    /// Whether the index of the stream of `topics` in `bags` is kept and current.
    pub fn holds(&self, bags: &[PathBuf], topics: &[String]) -> bool {
        let kept = self.kept.lock().unwrap_or_else(|e| e.into_inner());
        kept.iter().any(|i| i.is_of(bags, topics) && i.is_current())
    }
}
