//! The streaming readers (`read_las_into`, `read_e57_into`): blocks of at most `SINK_BLOCK`
//! points after one `start`, the same points as the plain readers, 8-bit columns as stored.

use driftless_export::e57::{read_e57, read_e57_into};
use driftless_export::las::{read_las, read_las_into};
use driftless_export::testutil::synthetic_points;
use driftless_export::{
    bounds_of, write_points, Layout, PointFormat, PointSink, Points, ReadStart, SINK_BLOCK,
};
use driftless_io::progress::Progress;
use std::path::PathBuf;

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "driftless-export-stream-{}-{name}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

const ALL: Layout = Layout {
    rgb: true,
    intensity: true,
    source: true,
    time: true,
    flags: true,
};

/// Records the start and the size of each block, and appends the blocks.
#[derive(Default)]
struct Record {
    start: Option<ReadStart>,
    sizes: Vec<usize>,
    all: Points,
}

impl PointSink for Record {
    fn start(&mut self, start: &ReadStart) {
        assert!(self.start.is_none(), "start twice");
        self.start = Some(start.clone());
        let l = start.layout;
        self.all = Points {
            rgb: l.rgb.then(Vec::new),
            intensity: l.intensity.then(Vec::new),
            source: l.source.then(Vec::new),
            time: l.time.then(Vec::new),
            flags: l.flags.then(Vec::new),
            ..Points::default()
        };
    }

    fn block(&mut self, block: &Points) {
        let start = self.start.as_ref().expect("block before start");
        let n = block.len();
        assert!(n > 0 && n <= SINK_BLOCK, "block of {n}");
        let l = start.layout;
        assert_eq!(
            [
                block.rgb.is_some(),
                block.intensity.is_some(),
                block.source.is_some(),
                block.time.is_some(),
                block.flags.is_some(),
            ],
            [l.rgb, l.intensity, l.source, l.time, l.flags]
        );
        assert!(block.scans.is_empty());
        self.sizes.push(n);
        self.all.xyz.extend_from_slice(&block.xyz);
        if let (Some(a), Some(b)) = (&mut self.all.rgb, &block.rgb) {
            a.extend_from_slice(b);
        }
        if let (Some(a), Some(b)) = (&mut self.all.intensity, &block.intensity) {
            a.extend_from_slice(b);
        }
        if let (Some(a), Some(b)) = (&mut self.all.source, &block.source) {
            a.extend_from_slice(b);
        }
        if let (Some(a), Some(b)) = (&mut self.all.time, &block.time) {
            a.extend_from_slice(b);
        }
        if let (Some(a), Some(b)) = (&mut self.all.flags, &block.flags) {
            a.extend_from_slice(b);
        }
    }
}

#[test]
fn las_streams_full_blocks_like_read_las() {
    let dir = temp_dir("las");
    let n = 2 * SINK_BLOCK + 123;
    let pts = synthetic_points(n, 7, ALL);
    for format in [PointFormat::Las, PointFormat::Laz] {
        let path = dir.join(format!("wolke.{}", format.extension()));
        write_points(&path, format, &pts, &Progress::new()).unwrap();
        let mut rec = Record::default();
        let end = read_las_into(&path, &mut rec, &Progress::new()).unwrap();
        let start = rec.start.clone().unwrap();
        assert_eq!(start.layout, ALL);
        assert_eq!(start.capacity, n);
        // the header's bounds, within the LAS step
        let (smin, smax) = start.bounds.unwrap();
        let (min, max) = bounds_of(&pts).unwrap();
        for k in 0..3 {
            assert!((smin[k] - min[k]).abs() <= 1e-4 && (smax[k] - max[k]).abs() <= 1e-4);
        }
        assert_eq!(rec.sizes, vec![SINK_BLOCK, SINK_BLOCK, 123]);
        assert!(!end.rgb_eight_bit && !end.intensity_eight_bit);
        assert!(end.scans.is_empty());
        let mut want = read_las(&path, &Progress::new()).unwrap();
        want.scans.clear();
        assert_eq!(rec.all, want);
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

/// A foreign LAS with 8-bit colours and intensities: the blocks hold them as stored, the end
/// says they are 8 bit, and `read_las` widens them.
#[test]
fn las_eight_bit_streams_as_stored() {
    let dir = temp_dir("las8");
    let path = dir.join("fremd.las");
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(2).unwrap();
    let mut w = las::Writer::from_path(&path, builder.into_header().unwrap()).unwrap();
    for (i, (c, v)) in [([255u16, 128, 0], 200u16), ([10, 20, 30], 7)]
        .into_iter()
        .enumerate()
    {
        w.write_point(las::Point {
            x: i as f64,
            intensity: v,
            color: Some(las::Color::new(c[0], c[1], c[2])),
            ..Default::default()
        })
        .unwrap();
    }
    w.close().unwrap();
    let mut rec = Record::default();
    let end = read_las_into(&path, &mut rec, &Progress::new()).unwrap();
    assert!(end.rgb_eight_bit && end.intensity_eight_bit);
    assert_eq!(rec.all.rgb, Some(vec![[255, 128, 0], [10, 20, 30]]));
    assert_eq!(rec.all.intensity, Some(vec![200, 7]));
    let p = read_las(&path, &Progress::new()).unwrap();
    assert_eq!(p.rgb, Some(vec![[65535, 32896, 0], [2570, 5140, 7710]]));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn e57_streams_full_blocks_like_read_e57() {
    let dir = temp_dir("e57");
    let n = 2 * SINK_BLOCK + 5;
    let pts = synthetic_points(n, 3, ALL);
    let path = dir.join("wolke.e57");
    write_points(&path, PointFormat::E57, &pts, &Progress::new()).unwrap();
    let mut rec = Record::default();
    let end = read_e57_into(&path, &mut rec, &Progress::new()).unwrap();
    let start = rec.start.clone().unwrap();
    assert_eq!(start.bounds, None);
    assert_eq!(
        start.layout,
        Layout {
            rgb: true,
            intensity: true,
            source: true,
            time: false,
            flags: false,
        }
    );
    assert!(start.capacity >= n);
    assert!(rec.sizes[..rec.sizes.len() - 1]
        .iter()
        .all(|&s| s == SINK_BLOCK));
    assert_eq!(rec.sizes.iter().sum::<usize>(), n);
    assert!(!end.rgb_eight_bit && !end.intensity_eight_bit);
    let want = read_e57(&path, &Progress::new()).unwrap();
    assert_eq!(end.scans, want.scans);
    assert_eq!(end.scans.len(), 3);
    rec.all.scans = end.scans;
    assert_eq!(rec.all, want);
    std::fs::remove_dir_all(&dir).unwrap();
}
