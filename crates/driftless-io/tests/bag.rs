use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::{BagError, BagReader, Time};
use std::io::Cursor;

fn payload(seed: u32) -> Vec<u8> {
    // Deterministic pseudo-random bytes (incompressible enough to keep chunks large).
    let mut x = seed.wrapping_mul(2654435761).wrapping_add(1);
    (0..1000)
        .map(|_| {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            (x >> 24) as u8
        })
        .collect()
}

/// Two topics x three messages, interleaved in time order. Returns bytes and expected payloads.
fn build(comp: Compression, indexed: bool, per_chunk: usize) -> (Vec<u8>, Vec<(String, Vec<u8>)>) {
    let mut w = BagWriter::new(comp)
        .indexed(indexed)
        .messages_per_chunk(per_chunk);
    let a = w.add_connection("/a", "pkg/A");
    let b = w.add_connection("/b", "pkg/B");
    let mut expected = Vec::new();
    for i in 0..6u32 {
        let (conn, topic) = if i % 2 == 0 { (a, "/a") } else { (b, "/b") };
        let data = payload(i);
        w.write_message(
            conn,
            Time {
                sec: 10 + i,
                nsec: 500,
            },
            &data,
        );
        expected.push((topic.to_string(), data));
    }
    (w.finish(), expected)
}

type Msg = Result<(String, Time, Vec<u8>), BagError>;

fn read_all(bytes: Vec<u8>, topics: &[&str]) -> Vec<Msg> {
    let mut r = BagReader::new(Cursor::new(bytes)).unwrap();
    r.messages(topics)
        .map(|m| m.map(|m| (m.topic, m.time, m.data)))
        .collect()
}

#[test]
fn reads_uncompressed_bag() {
    let (bytes, expected) = build(Compression::None, true, 2);
    let r = BagReader::new(Cursor::new(bytes.clone())).unwrap();
    let conns = r.connections();
    assert_eq!(conns.len(), 2);
    assert_eq!(
        (conns[0].topic.as_str(), conns[0].msg_type.as_str()),
        ("/a", "pkg/A")
    );
    assert_eq!(
        (conns[1].topic.as_str(), conns[1].msg_type.as_str()),
        ("/b", "pkg/B")
    );

    let got = read_all(bytes, &[]);
    assert_eq!(got.len(), 6);
    let mut last = 0.0;
    for (m, (topic, data)) in got.into_iter().zip(expected) {
        let (t, time, d) = m.unwrap();
        assert_eq!((t, d), (topic, data));
        assert!(time.as_secs_f64() >= last);
        last = time.as_secs_f64();
    }
}

#[test]
fn reads_lz4_bag() {
    let (bytes, expected) = build(Compression::Lz4, true, 3);
    let got: Vec<_> = read_all(bytes, &[])
        .into_iter()
        .map(|m| m.unwrap())
        .collect();
    assert_eq!(got.len(), 6);
    for ((t, _, d), (et, ed)) in got.into_iter().zip(expected) {
        assert_eq!((t, d), (et, ed));
    }
}

#[test]
fn filters_by_topic() {
    let (bytes, _) = build(Compression::Lz4, true, 2);
    let got = read_all(bytes, &["/a"]);
    assert_eq!(got.len(), 3);
    assert!(got.iter().all(|m| m.as_ref().unwrap().0 == "/a"));
}

#[test]
fn truncated_bag_reports_offset() {
    for comp in [Compression::None, Compression::Lz4] {
        let (mut bytes, _) = build(comp, true, 1);
        let full = bytes.len();
        bytes.truncate(full * 7 / 10);
        let cut = bytes.len() as u64;
        let got = read_all(bytes, &[]);
        let (last, ok) = got.split_last().unwrap();
        assert!(!ok.is_empty() && ok.len() < 6, "{comp:?}: {} ok", ok.len());
        assert!(ok.iter().all(Result::is_ok));
        match last {
            Err(BagError::Truncated { offset }) => assert!(*offset < cut && *offset > 4096),
            other => panic!("{comp:?}: expected Truncated, got {other:?}"),
        }
    }
}

#[test]
fn unindexed_bag_reads_sequentially() {
    for comp in [Compression::None, Compression::Lz4] {
        let (bytes, expected) = build(comp, false, 2);
        let r = BagReader::new(Cursor::new(bytes.clone())).unwrap();
        assert_eq!(r.connections().len(), 2);
        let got = read_all(bytes, &[]);
        assert_eq!(got.len(), 6);
        for (m, (t, d)) in got.into_iter().zip(expected) {
            let (gt, _, gd) = m.unwrap();
            assert_eq!((gt, gd), (t, d));
        }
    }
}

#[test]
fn rejects_non_bag() {
    let err = BagReader::new(Cursor::new(b"this is definitely not a bag file".to_vec()))
        .err()
        .unwrap();
    assert!(matches!(err, BagError::NotABag));
    let err = BagReader::new(Cursor::new(Vec::new())).err().unwrap();
    assert!(matches!(err, BagError::NotABag));
}

#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA"]
fn real_lidar_bag_summary() {
    let Some(path) = driftless_io::testdata::path("Test/LIDAR_Test.bag") else {
        return;
    };
    let mut r = BagReader::open(&path).unwrap();
    let topics: Vec<_> = r.connections().iter().map(|c| c.topic.clone()).collect();
    assert!(topics.iter().any(|t| t == "/vanjee_722z"), "{topics:?}");
    assert!(
        topics.iter().any(|t| t == "/vanjee_imu_packets"),
        "{topics:?}"
    );
    let (mut lidar, mut imu) = (0, 0);
    for m in r.messages(&[]) {
        match m.unwrap().topic.as_str() {
            "/vanjee_722z" => lidar += 1,
            "/vanjee_imu_packets" => imu += 1,
            _ => {}
        }
    }
    assert_eq!((lidar, imu), (2663, 53264));
}
