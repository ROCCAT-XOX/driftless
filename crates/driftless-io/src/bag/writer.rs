//! Minimal ROS1 bag writer used to build test fixtures.

use super::record::*;
use std::io::Write;

/// Chunk compression of written bags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    None,
    Lz4,
    /// Stores the chunk data uncompressed but labels it with this compression name
    /// (e.g. `"bz2"`), to test how readers handle unsupported compression.
    Labelled(&'static str),
}

struct Pending {
    conn: u32,
    time: Time,
    data: Vec<u8>,
}

struct Conn {
    topic: String,
    msg_type: String,
}

/// Builds a bag in memory. Messages are stored in call order.
pub struct BagWriter {
    compression: Compression,
    indexed: bool,
    messages_per_chunk: usize,
    conns: Vec<Conn>,
    pending: Vec<Pending>,
    body: Vec<u8>,
    chunk_infos: Vec<Vec<u8>>,
}

fn field(out: &mut Vec<u8>, name: &str, value: &[u8]) {
    out.extend_from_slice(&((name.len() + 1 + value.len()) as u32).to_le_bytes());
    out.extend_from_slice(name.as_bytes());
    out.push(b'=');
    out.extend_from_slice(value);
}

fn record(out: &mut Vec<u8>, header: &[u8], data: &[u8]) {
    out.extend_from_slice(&(header.len() as u32).to_le_bytes());
    out.extend_from_slice(header);
    out.extend_from_slice(&(data.len() as u32).to_le_bytes());
    out.extend_from_slice(data);
}

fn time_bytes(t: Time) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[..4].copy_from_slice(&t.sec.to_le_bytes());
    b[4..].copy_from_slice(&t.nsec.to_le_bytes());
    b
}

impl BagWriter {
    pub fn new(compression: Compression) -> Self {
        BagWriter {
            compression,
            indexed: true,
            messages_per_chunk: usize::MAX,
            conns: Vec::new(),
            pending: Vec::new(),
            body: Vec::new(),
            chunk_infos: Vec::new(),
        }
    }

    /// `false` omits index records, chunk infos and `index_pos` (unclean shutdown).
    pub fn indexed(mut self, indexed: bool) -> Self {
        self.indexed = indexed;
        self
    }

    /// Starts a new chunk after this many messages.
    pub fn messages_per_chunk(mut self, n: usize) -> Self {
        self.messages_per_chunk = n.max(1);
        self
    }

    pub fn add_connection(&mut self, topic: &str, msg_type: &str) -> u32 {
        self.conns.push(Conn {
            topic: topic.into(),
            msg_type: msg_type.into(),
        });
        (self.conns.len() - 1) as u32
    }

    pub fn write_message(&mut self, conn: u32, time: Time, data: &[u8]) {
        self.pending.push(Pending {
            conn,
            time,
            data: data.to_vec(),
        });
        if self.pending.len() >= self.messages_per_chunk {
            self.flush_chunk();
        }
    }

    fn conn_record(&self, out: &mut Vec<u8>, id: u32) {
        let c = &self.conns[id as usize];
        let mut h = Vec::new();
        field(&mut h, "op", &[OP_CONNECTION]);
        field(&mut h, "conn", &id.to_le_bytes());
        field(&mut h, "topic", c.topic.as_bytes());
        let mut d = Vec::new();
        field(&mut d, "topic", c.topic.as_bytes());
        field(&mut d, "type", c.msg_type.as_bytes());
        field(&mut d, "md5sum", b"0123456789abcdef0123456789abcdef");
        field(&mut d, "message_definition", b"");
        record(out, &h, &d);
    }

    fn flush_chunk(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let pending = std::mem::take(&mut self.pending);
        let mut used: Vec<u32> = pending.iter().map(|m| m.conn).collect();
        used.sort_unstable();
        used.dedup();

        let mut raw = Vec::new();
        // (conn, [(time, offset)]) for the index records.
        let mut index: Vec<(u32, Vec<(Time, u32)>)> =
            used.iter().map(|&c| (c, Vec::new())).collect();
        for &c in &used {
            self.conn_record(&mut raw, c);
        }
        for m in &pending {
            let offset = raw.len() as u32;
            index
                .iter_mut()
                .find(|(c, _)| *c == m.conn)
                .unwrap()
                .1
                .push((m.time, offset));
            let mut h = Vec::new();
            field(&mut h, "op", &[OP_MESSAGE]);
            field(&mut h, "conn", &m.conn.to_le_bytes());
            field(&mut h, "time", &time_bytes(m.time));
            record(&mut raw, &h, &m.data);
        }

        let (name, payload) = match self.compression {
            Compression::None => ("none", raw.clone()),
            Compression::Labelled(name) => (name, raw.clone()),
            Compression::Lz4 => {
                let mut enc = lz4_flex::frame::FrameEncoder::new(Vec::new());
                enc.write_all(&raw).unwrap();
                ("lz4", enc.finish().unwrap())
            }
        };

        let chunk_pos = (MAGIC.len() + 4096 + self.body.len()) as u64;
        let mut h = Vec::new();
        field(&mut h, "op", &[OP_CHUNK]);
        field(&mut h, "compression", name.as_bytes());
        field(&mut h, "size", &(raw.len() as u32).to_le_bytes());
        record(&mut self.body, &h, &payload);

        if self.indexed {
            for (conn, entries) in &index {
                let mut h = Vec::new();
                field(&mut h, "op", &[OP_INDEX]);
                field(&mut h, "ver", &1u32.to_le_bytes());
                field(&mut h, "conn", &conn.to_le_bytes());
                field(&mut h, "count", &(entries.len() as u32).to_le_bytes());
                let mut d = Vec::new();
                for (t, off) in entries {
                    d.extend_from_slice(&time_bytes(*t));
                    d.extend_from_slice(&off.to_le_bytes());
                }
                record(&mut self.body, &h, &d);
            }
            let start = pending.iter().map(|m| m.time).min().unwrap();
            let end = pending.iter().map(|m| m.time).max().unwrap();
            let mut h = Vec::new();
            field(&mut h, "op", &[OP_CHUNK_INFO]);
            field(&mut h, "ver", &1u32.to_le_bytes());
            field(&mut h, "chunk_pos", &chunk_pos.to_le_bytes());
            field(&mut h, "start_time", &time_bytes(start));
            field(&mut h, "end_time", &time_bytes(end));
            field(&mut h, "count", &(index.len() as u32).to_le_bytes());
            let mut d = Vec::new();
            for (conn, entries) in &index {
                d.extend_from_slice(&conn.to_le_bytes());
                d.extend_from_slice(&(entries.len() as u32).to_le_bytes());
            }
            let mut info = Vec::new();
            record(&mut info, &h, &d);
            self.chunk_infos.push(info);
        }
    }

    /// Flushes the last chunk and returns the complete bag file.
    pub fn finish(mut self) -> Vec<u8> {
        self.flush_chunk();
        let chunk_count = self.chunk_infos.len() as u32;
        let mut index_pos = 0u64;
        if self.indexed {
            index_pos = (MAGIC.len() + 4096 + self.body.len()) as u64;
            for id in 0..self.conns.len() as u32 {
                let mut tail = Vec::new();
                self.conn_record(&mut tail, id);
                self.body.extend_from_slice(&tail);
            }
            for info in std::mem::take(&mut self.chunk_infos) {
                self.body.extend_from_slice(&info);
            }
        }

        let mut h = Vec::new();
        field(&mut h, "op", &[OP_BAG_HEADER]);
        field(&mut h, "index_pos", &index_pos.to_le_bytes());
        field(
            &mut h,
            "conn_count",
            &(self.conns.len() as u32).to_le_bytes(),
        );
        field(&mut h, "chunk_count", &chunk_count.to_le_bytes());
        let pad = 4096 - 8 - h.len();
        let mut out = MAGIC.to_vec();
        record(&mut out, &h, &vec![b' '; pad]);
        out.extend_from_slice(&self.body);
        out
    }
}
