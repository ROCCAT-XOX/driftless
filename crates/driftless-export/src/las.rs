//! LAS 1.4 and LAZ: the one writer for every LAS file of Driftless, and the reader for own
//! and foreign files.
//!
//! The writer picks the point format from the columns (time + colour → 7, time → 6, colour →
//! 2, neither → 0), so that a file without times gets no made-up times. Coordinates are
//! stored at 0.1 mm around the rounded middle of the cloud. The header has no date, so the
//! same cloud gives the same bytes.

use crate::points::Collect;
use crate::{
    check_cancel, ExportError, Layout, PointSink, PointSource, Points, ReadEnd, ReadStart, BLOCK,
    PHASE_READ, PHASE_WRITE, SINK_BLOCK,
};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use std::fs::File;
use std::io::{self, BufWriter, Seek, SeekFrom, Write};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// LAS coordinate resolution (m): 0.1 mm.
pub const LAS_SCALE: f64 = 0.0001;

/// The LAS offset of a cloud: per axis the middle of its bounds rounded to whole metres;
/// zero without points.
pub fn las_offset(bounds: Option<([f64; 3], [f64; 3])>) -> [f64; 3] {
    match bounds {
        Some((min, max)) => std::array::from_fn(|k| ((min[k] + max[k]) / 2.0).round()),
        None => [0.0; 3],
    }
}

/// The LAS point format for the columns of `layout`.
fn point_format(layout: Layout) -> u8 {
    match (layout.time, layout.rgb) {
        (true, true) => 7,
        (true, false) => 6,
        (false, true) => 2,
        (false, false) => 0,
    }
}

/// Intensity 0–255 → LAS intensity 0–65535.
fn las_intensity(i: f32) -> u16 {
    (f64::from(i) * 257.0).round().clamp(0.0, 65535.0) as u16
}

/// A `las` error while writing or reading `path`; I/O stays I/O, everything else is a
/// format error with a German lead.
fn las_error(path: &Path, lead: &str, e: ::las::Error) -> ExportError {
    match e {
        ::las::Error::Io(e) if e.kind() == io::ErrorKind::UnexpectedEof => {
            ExportError::format(path, "LAS-Datei ist unvollständig (zu früh zu Ende)")
        }
        ::las::Error::Io(e) => ExportError::io(path, e),
        e => ExportError::format(path, format!("{lead} ({e})")),
    }
}

/// The file under the `las` writer. Once `discard` is set (after an error or a cancel), it
/// accepts every write and seek without doing it: the writer closes itself on drop and would
/// panic if that failed.
struct Sink {
    inner: BufWriter<File>,
    discard: Arc<AtomicBool>,
}

impl Sink {
    fn discarding(&self) -> bool {
        self.discard.load(Ordering::Relaxed)
    }
}

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.discarding() {
            return Ok(buf.len());
        }
        self.inner.write(buf)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        if self.discarding() {
            return Ok(());
        }
        self.inner.write_all(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        if self.discarding() {
            return Ok(());
        }
        self.inner.flush()
    }
}

impl Seek for Sink {
    fn seek(&mut self, pos: SeekFrom) -> io::Result<u64> {
        if self.discarding() {
            return Ok(0);
        }
        self.inner.seek(pos)
    }
}

/// Per-axis minimum and maximum.
type Bounds = ([f64; 3], [f64; 3]);

/// Per-axis bounds of `src`, or a format error naming the first point without a finite
/// position.
fn finite_bounds(path: &Path, src: &dyn PointSource) -> Result<Option<Bounds>, ExportError> {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for i in 0..src.len() {
        let p = src.xyz(i);
        if !p.iter().all(|v| v.is_finite()) {
            return Err(ExportError::format(
                path,
                format!("Punkt {} hat keine gültige Lage ({p:?})", i + 1),
            ));
        }
        for k in 0..3 {
            min[k] = min[k].min(p[k]);
            max[k] = max[k].max(p[k]);
        }
    }
    Ok((!src.is_empty()).then_some((min, max)))
}

/// Writes `src` as LAS 1.4, or as LAZ with `compress` (whatever the extension of `path`),
/// through `<name>.tmp` next to `path`. Progress: phase [`PHASE_WRITE`] counting points, a
/// cancel check per [`BLOCK`]; an error or a cancel leaves `path` as it was and no `.tmp`.
///
/// Columns: intensity as `round(v × 257)`, colour 16 bit, the scan number as
/// `point_source_id`, the flags as `user_data`, the time as `gps_time` (as stored: raw
/// seconds, not GPS week time). Every point is return 1 of 1.
pub fn write_las(
    path: &Path,
    src: &dyn PointSource,
    compress: bool,
    progress: &Progress,
) -> Result<(), ExportError> {
    check_cancel(progress)?;
    let err = |e| las_error(path, "LAS-Datei nicht schreibbar", e);
    let bounds = finite_bounds(path, src)?;
    let offset = las_offset(bounds);
    if let Some((min, max)) = bounds {
        let fits = |v: f64, o: f64| ((v - o) / LAS_SCALE).round().abs() <= f64::from(i32::MAX);
        if (0..3).any(|k| !fits(min[k], offset[k]) || !fits(max[k], offset[k])) {
            return Err(ExportError::format(
                path,
                "Punktwolke ist zu weit ausgedehnt für LAS mit 0,1 mm (höchstens ±214 km um \
                 die Mitte je Achse)",
            ));
        }
    }

    let layout = src.layout();
    let mut builder = ::las::Builder::from((1, 4));
    let mut format = ::las::point::Format::new(point_format(layout)).map_err(err)?;
    format.is_compressed = compress;
    builder.point_format = format;
    builder.date = None;
    builder.generating_software = "driftless".into();
    let transform = |offset| ::las::Transform {
        scale: LAS_SCALE,
        offset,
    };
    builder.transforms = ::las::Vector {
        x: transform(offset[0]),
        y: transform(offset[1]),
        z: transform(offset[2]),
    };
    let header = builder.into_header().map_err(err)?;

    let temp = TempPath::new(path);
    let file = File::create(temp.tmp()).map_err(|e| ExportError::io(path, e))?;
    let discard = Arc::new(AtomicBool::new(false));
    let sink = Sink {
        inner: BufWriter::new(file),
        discard: discard.clone(),
    };
    // `Writer::new` with `is_compressed` set, not `Writer::from_path`: that one decides on
    // compression by the extension, and `x.laz.tmp` would come out uncompressed.
    let mut writer = ::las::Writer::new(sink, header).map_err(err)?;
    let written = write_body(path, &mut writer, src, progress)
        .and_then(|()| check_cancel(progress))
        .and_then(|()| writer.close().map_err(err));
    if let Err(e) = written {
        discard.store(true, Ordering::Relaxed);
        // The writer closes itself on drop and panics if that fails; with the sink discarding
        // it cannot fail on I/O, and a panic left in `las` must not take the program down.
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || drop(writer)));
        return Err(e);
    }
    let sink = writer.into_inner().map_err(err)?;
    let file = sink
        .inner
        .into_inner()
        .map_err(|e| ExportError::io(path, e.into_error()))?;
    drop(file);
    temp.commit().map_err(|e| ExportError::io(path, e))
}

fn write_body(
    path: &Path,
    writer: &mut ::las::Writer<Sink>,
    src: &dyn PointSource,
    progress: &Progress,
) -> Result<(), ExportError> {
    let err = |e| las_error(path, "LAS-Datei nicht schreibbar", e);
    let layout = src.layout();
    let format = *writer.header().point_format();
    let n = src.len();
    progress.phase(PHASE_WRITE, Some(n as u64));
    for start in (0..n).step_by(BLOCK) {
        check_cancel(progress)?;
        let end = (start + BLOCK).min(n);
        for i in start..end {
            let [x, y, z] = src.xyz(i);
            let point = ::las::Point {
                x,
                y,
                z,
                intensity: if layout.intensity {
                    las_intensity(src.intensity(i))
                } else {
                    0
                },
                return_number: 1,
                number_of_returns: 1,
                gps_time: format.has_gps_time.then(|| src.time(i)),
                color: format.has_color.then(|| {
                    let [r, g, b] = src.rgb(i);
                    ::las::Color::new(r, g, b)
                }),
                user_data: src.flags(i),
                point_source_id: src.source(i),
                ..Default::default()
            };
            writer.write_point(point).map_err(err)?;
        }
        progress.advance((end - start) as u64);
    }
    Ok(())
}

/// Reads a LAS or LAZ file (any version and point format `las` knows) into [`Points`].
/// Progress: phase [`PHASE_READ`] counting points, a cancel check per [`BLOCK`].
///
/// Colour, and separately intensity, whose maximum over the whole file is at most 255 count
/// as 8 bit (as some foreign writers store them) and are widened ×257. Without times in the
/// point format `time` is `None`; intensity, scan number and flags are always there (see
/// [`crate::read_points`], which drops them when they say nothing). `scans` stays empty.
pub fn read_las(path: &Path, progress: &Progress) -> Result<Points, ExportError> {
    let mut points = Collect::default();
    let end = read_las_into(path, &mut points, progress)?;
    Ok(points.finish(end))
}

/// [`read_las`] block by block into `sink`, without holding the whole cloud. The blocks
/// hold colour and intensity as stored; [`ReadEnd`] says whether they are 8 bit.
/// [`ReadStart::bounds`] are the header's bounds, unchecked.
pub fn read_las_into(
    path: &Path,
    sink: &mut dyn PointSink,
    progress: &Progress,
) -> Result<ReadEnd, ExportError> {
    check_cancel(progress)?;
    let err = |e| las_error(path, "keine lesbare LAS-Datei", e);
    let mut reader = ::las::Reader::from_path(path).map_err(err)?;
    let header = reader.header();
    let total = header.number_of_points();
    let format = *header.point_format();
    let b = header.bounds();
    // A broken header must not reserve memory for points the file cannot hold.
    let file_len = std::fs::metadata(path).map_or(0, |m| m.len());
    let capacity = usize::try_from(total.min(file_len.saturating_mul(8))).unwrap_or(0);
    let layout = Layout {
        rgb: format.has_color,
        intensity: true,
        source: true,
        time: format.has_gps_time,
        flags: true,
    };
    sink.start(&ReadStart {
        layout,
        capacity,
        bounds: Some(([b.min.x, b.min.y, b.min.z], [b.max.x, b.max.y, b.max.z])),
    });
    let mut block = Points::with_layout(layout, SINK_BLOCK.min(capacity));
    let (mut max_rgb, mut max_intensity) = (0u16, 0u16);
    progress.phase(PHASE_READ, Some(total));
    let mut batch = ::las::PointDataBuilder::new()
        .for_header(reader.header())
        .build();
    loop {
        check_cancel(progress)?;
        let read = reader.fill_points(BLOCK as u64, &mut batch).map_err(err)?;
        if read == 0 {
            break;
        }
        for p in batch.points() {
            let p = p.map_err(err)?;
            block.xyz.push([p.x, p.y, p.z]);
            if let Some(rgb) = &mut block.rgb {
                let c = p.color.unwrap_or_default();
                max_rgb = max_rgb.max(c.red).max(c.green).max(c.blue);
                rgb.push([c.red, c.green, c.blue]);
            }
            if let Some(time) = &mut block.time {
                time.push(p.gps_time.unwrap_or_default());
            }
            max_intensity = max_intensity.max(p.intensity);
            let columns = (&mut block.intensity, &mut block.source, &mut block.flags);
            if let (Some(intensity), Some(source), Some(flags)) = columns {
                intensity.push(p.intensity);
                source.push(p.point_source_id);
                flags.push(p.user_data);
            }
            if block.xyz.len() == SINK_BLOCK {
                sink.block(&block);
                block.clear();
            }
        }
        progress.advance(read);
    }
    if !block.is_empty() {
        sink.block(&block);
    }
    Ok(ReadEnd {
        scans: Vec::new(),
        rgb_eight_bit: format.has_color && max_rgb <= 255,
        intensity_eight_bit: max_intensity <= 255,
    })
}
