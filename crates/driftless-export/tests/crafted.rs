//! Crafted foreign files whose headers once made a reader allocate without bound (E57 without
//! coordinates or bits, LAS/LAZ counts and lengths beyond the file). Its own test binary with
//! a capped allocator, so that a regression fails at once instead of filling the memory.
//! Exactly one test: the cap is global.

use driftless_export::las::write_las;
use driftless_export::testutil::synthetic_points;
use driftless_export::{read_points, ExportError, Layout};
use driftless_io::progress::Progress;
use driftless_io::testalloc::{limited_during, Counting};
use driftless_io::testdir::TestDir;
use e57::{RecordName, RecordValue};
use std::path::Path;
use std::time::{Duration, Instant};

mod common;
use common::{edit_xml, foreign, integer};

#[global_allocator]
static ALLOC: Counting = Counting;

/// Most bytes a refused file may cost to read.
const CAP: usize = 64 << 20;

fn temp_dir(name: &str) -> TestDir {
    TestDir::new(format!(
        "driftless-export-crafted-{}-{name}",
        std::process::id()
    ))
}

/// The format error of reading `path`, within [`CAP`] bytes and a second.
fn refused(path: &Path) -> String {
    let start = Instant::now();
    let result = limited_during(CAP, || read_points(path, &Progress::new()));
    assert!(
        start.elapsed() < Duration::from_secs(1),
        "{}: {:?}",
        path.display(),
        start.elapsed()
    );
    match result {
        Err(ExportError::Format { msg, .. }) => msg,
        other => panic!("{}: {other:?}", path.display()),
    }
}

/// An E57 whose cartesian fields (0..=`max`) are renamed to colours, so that it has no
/// coordinates; with `max` = 0 no field takes a bit.
fn e57_colours_only(path: &Path, max: i64) {
    let prototype = vec![
        integer(RecordName::CartesianX, 0, 255),
        integer(RecordName::CartesianY, 0, 255),
        integer(RecordName::CartesianZ, 0, 255),
    ];
    let rows = [vec![RecordValue::Integer(0); 3]];
    foreign(path, Some("Bunt"), prototype, &rows, None, false);
    edit_xml(path, |xml| {
        xml.replace("cartesianX", "colorRed")
            .replace("cartesianY", "colorGreen")
            .replace("cartesianZ", "colorBlue")
            .replace("maximum=\"255\"", &format!("maximum=\"{max}\""))
    });
}

fn patch(path: &Path, at: usize, bytes: &[u8]) {
    let mut data = std::fs::read(path).unwrap();
    data[at..at + bytes.len()].copy_from_slice(bytes);
    std::fs::write(path, data).unwrap();
}

fn u32_at(path: &Path, at: usize) -> u32 {
    let data = std::fs::read(path).unwrap();
    u32::from_le_bytes(data[at..at + 4].try_into().unwrap())
}

fn u64_at(path: &Path, at: usize) -> u64 {
    let data = std::fs::read(path).unwrap();
    u64::from_le_bytes(data[at..at + 8].try_into().unwrap())
}

/// A LAS 1.2 file in point format 0 with five points: 227 bytes header, 100 bytes points.
fn las12(path: &Path) {
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(0).unwrap();
    let mut writer = las::Writer::from_path(path, builder.into_header().unwrap()).unwrap();
    for i in 0..5 {
        writer
            .write_point(las::Point {
                x: f64::from(i),
                ..Default::default()
            })
            .unwrap();
    }
    writer.close().unwrap();
    assert_eq!(std::fs::metadata(path).unwrap().len(), 327);
}

/// A LAS 1.4 file (`compress`: LAZ) of 2000 points written by Driftless.
fn las14(path: &Path, compress: bool) {
    let layout = Layout {
        rgb: true,
        intensity: true,
        source: false,
        time: false,
        flags: false,
    };
    let pts = synthetic_points(2_000, 3, layout);
    write_las(path, &pts, compress, &Progress::new()).unwrap();
}

/// A LAZ 1.4 file of 1000 points in point format 6 (layered chunks, as Driftless writes a
/// map with times): one chunk, about 7 KB.
fn laz14_layered(path: &Path) {
    let layout = Layout {
        intensity: true,
        time: true,
        ..Layout::default()
    };
    let pts = synthetic_points(1_000, 9, layout);
    write_las(path, &pts, true, &Progress::new()).unwrap();
    assert!(read_points(path, &Progress::new()).is_ok());
}

/// Byte offsets in the LAS header.
const OFFSET_TO_POINTS: usize = 96;
const VLR_COUNT: usize = 100;
const RECORD_LENGTH: usize = 105;
const LEGACY_COUNT: usize = 107;
const FIRST_EVLR: usize = 235;
const EVLR_COUNT: usize = 243;
const COUNT_14: usize = 247;

#[test]
fn crafted_headers_fail_fast() {
    let dir = temp_dir("all");

    // E57: no field with a bit – the library would fill its queue without end (2 GB in 0.3 s
    // before the fix).
    let path = dir.join("null-bit.e57");
    e57_colours_only(&path, 0);
    assert_eq!(
        refused(&path),
        "E57: Scan „Bunt“ speichert keinen Wert mit mindestens 1 Bit je Punkt – solche Dateien \
         werden nicht gelesen"
    );
    // E57: colours with bits but no coordinates.
    let path = dir.join("nur-farbe.e57");
    e57_colours_only(&path, 255);
    assert_eq!(
        refused(&path),
        "E57: Scan „Bunt“ hat keine kartesischen Punkte"
    );

    // LAS: 327 bytes claiming 2^20 records of 65 535 bytes (64 GiB).
    let path = dir.join("gross.las");
    las12(&path);
    patch(&path, RECORD_LENGTH, &u16::MAX.to_le_bytes());
    patch(&path, LEGACY_COUNT, &(1u32 << 20).to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAS-Datei gibt 1048576 Punkte an, die Datei kann höchstens 0 enthalten"
    );
    // LAS: more VLRs than the file holds.
    let path = dir.join("vlr.las");
    las12(&path);
    patch(&path, VLR_COUNT, &u32::MAX.to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAS-Datei ist beschädigt (VLR reicht über das Dateiende)"
    );
    // LAS 1.4: an EVLR claiming a terabyte.
    let path = dir.join("evlr.las");
    las14(&path, false);
    let len = std::fs::metadata(&path).unwrap().len();
    let mut evlr = vec![0u8; 60];
    evlr[20..28].copy_from_slice(&(1u64 << 40).to_le_bytes());
    let mut data = std::fs::read(&path).unwrap();
    data.extend_from_slice(&evlr);
    std::fs::write(&path, data).unwrap();
    patch(&path, FIRST_EVLR, &len.to_le_bytes());
    patch(&path, EVLR_COUNT, &1u32.to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAS-Datei ist beschädigt (EVLR reicht über das Dateiende)"
    );

    // LAZ: a chunk table claiming 2^32 − 1 chunks (64 GiB of entries).
    let path = dir.join("chunks.laz");
    las14(&path, true);
    assert!(read_points(&path, &Progress::new()).is_ok());
    let table = u64_at(&path, u32_at(&path, OFFSET_TO_POINTS) as usize) as usize;
    patch(&path, table + 4, &u32::MAX.to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAZ-Datei ist beschädigt (Chunk-Tabelle passt nicht in die Datei)"
    );
    // LAZ: the chunk table behind the end of the file.
    let path = dir.join("tabelle.laz");
    las14(&path, true);
    let start = u32_at(&path, OFFSET_TO_POINTS) as usize;
    let len = std::fs::metadata(&path).unwrap().len();
    patch(&path, start, &(len + 1_000_000).to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAZ-Datei ist beschädigt (Chunk-Tabelle liegt außerhalb der Datei)"
    );
    // LAZ: a user id that only starts with „laszip encoded“ is not the LASzip VLR for `las`
    // either, so the file is refused instead of checked against the wrong table.
    let path = dir.join("fremd-vlr.laz");
    las14(&path, true);
    let header_size = u16::from_le_bytes(std::fs::read(&path).unwrap()[94..96].try_into().unwrap());
    patch(&path, header_size as usize + 2, b"laszip encodedX\0");
    assert!(read_points(&path, &Progress::new()).is_err());

    // LAZ: a trillion points in a few kilobytes.
    let path = dir.join("viele.laz");
    las14(&path, true);
    patch(&path, LEGACY_COUNT, &0u32.to_le_bytes());
    patch(&path, COUNT_14, &(1u64 << 40).to_le_bytes());
    let msg = refused(&path);
    assert!(
        msg.starts_with("LAZ-Datei gibt 1099511627776 Punkte an, die Datei kann höchstens"),
        "{msg}"
    );

    // LAZ 1.4, layered: the first chunk starts after the 8-byte table offset with the raw first
    // point (30 bytes), its point count and nine layer sizes. An xy layer of 4 GiB in 7 KB
    // (1.75 GB zero-filled in 0.26 s before the fix).
    let path = dir.join("schicht.laz");
    laz14_layered(&path);
    let chunk = u32_at(&path, OFFSET_TO_POINTS) as usize + 8;
    patch(&path, chunk + 30 + 4, &0xFFFF_FFF0u32.to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAZ-Datei ist beschädigt (Chunk 1 ist länger als die Datei)"
    );
    // The same with the last layer (times) one byte longer: still inside the file, but longer
    // than the chunk table says.
    let path = dir.join("schicht-tabelle.laz");
    laz14_layered(&path);
    let last = chunk + 30 + 4 + 8 * 4;
    patch(&path, last, &(u32_at(&path, last) + 1).to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAZ-Datei ist beschädigt (Chunk 1 ist länger als in der Chunk-Tabelle angegeben)"
    );
    // A chunk claiming 2^32 − 1 points.
    let path = dir.join("schicht-punkte.laz");
    laz14_layered(&path);
    patch(&path, chunk + 30, &u32::MAX.to_le_bytes());
    assert_eq!(
        refused(&path),
        "LAZ-Datei ist beschädigt (Chunk 1 gibt 4294967295 Punkte an, mehr als er enthalten \
         kann)"
    );

    std::fs::remove_dir_all(dir).unwrap();
}
