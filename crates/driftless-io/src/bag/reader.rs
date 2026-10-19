//! Sequential, chunk-by-chunk bag reader.

use super::record::*;
use super::BagError;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::Path;

/// A topic connection declared in the bag.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Connection {
    pub id: u32,
    pub topic: String,
    pub msg_type: String,
    pub md5sum: String,
}

/// One message with its raw (serialized) payload.
#[derive(Debug, Clone, PartialEq)]
pub struct RawMessage {
    pub conn_id: u32,
    pub topic: String,
    pub time: Time,
    pub data: Vec<u8>,
}

pub struct BagReader<R: Read + Seek> {
    r: R,
    conns: Vec<Connection>,
    by_id: HashMap<u32, usize>,
    /// First byte after the bag header record.
    data_start: u64,
    file_len: u64,
}

impl BagReader<BufReader<File>> {
    pub fn open(path: &Path) -> Result<Self, BagError> {
        BagReader::new(BufReader::new(File::open(path)?))
    }
}

/// Reads exactly `buf.len()` bytes; `Ok(false)` if the stream ended first.
fn read_full<R: Read>(r: &mut R, buf: &mut [u8]) -> Result<bool, BagError> {
    let mut n = 0;
    while n < buf.len() {
        match r.read(&mut buf[n..]) {
            Ok(0) => return Ok(false),
            Ok(k) => n += k,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e.into()),
        }
    }
    Ok(true)
}

/// A top-level record header; the data (`data_len` bytes) has not been consumed yet.
struct Head {
    fields: Fields,
    data_len: u64,
    data_pos: u64,
}

enum Next {
    Eof,
    Record(Head),
}

/// Reads the next record header at `pos`. A partial header is `Truncated { offset: pos }`.
fn read_head<R: Read>(r: &mut R, pos: u64, file_len: u64) -> Result<Next, BagError> {
    let trunc = BagError::Truncated { offset: pos };
    if pos >= file_len {
        return Ok(Next::Eof);
    }
    let mut len = [0u8; 4];
    if !read_full(r, &mut len)? {
        return Err(trunc);
    }
    let hlen = u64::from(u32::from_le_bytes(len));
    if pos + 4 + hlen + 4 > file_len {
        return Err(trunc);
    }
    let mut hbuf = vec![0u8; hlen as usize];
    if !read_full(r, &mut hbuf)? || !read_full(r, &mut len)? {
        return Err(trunc);
    }
    let data_len = u64::from(u32::from_le_bytes(len));
    let data_pos = pos + 8 + hlen;
    if data_pos + data_len > file_len {
        return Err(trunc);
    }
    Ok(Next::Record(Head {
        fields: Fields::parse(&hbuf)?,
        data_len,
        data_pos,
    }))
}

fn parse_connection(fields: &Fields, data: &[u8]) -> Result<Connection, BagError> {
    let d = Fields::parse(data)?;
    Ok(Connection {
        id: fields.u32("conn")?,
        topic: fields
            .string("topic")
            .or_else(|| d.string("topic"))
            .ok_or_else(|| BagError::Malformed("connection without topic".into()))?,
        msg_type: d.string("type").unwrap_or_default(),
        md5sum: d.string("md5sum").unwrap_or_default(),
    })
}

fn decompress(compression: &str, data: Vec<u8>, size_hint: usize) -> Result<Vec<u8>, BagError> {
    match compression {
        "none" => Ok(data),
        "lz4" => {
            let mut out = Vec::with_capacity(size_hint.min(1 << 28));
            lz4_flex::frame::FrameDecoder::new(data.as_slice())
                .read_to_end(&mut out)
                .map_err(|e| BagError::Malformed(format!("lz4 chunk: {e}")))?;
            Ok(out)
        }
        other => Err(BagError::UnsupportedCompression(other.to_string())),
    }
}

/// Events found inside a decompressed chunk.
enum ChunkItem {
    Message {
        conn: u32,
        time: Time,
        data: Vec<u8>,
    },
    Connection(Connection),
}

struct Chunk {
    buf: Vec<u8>,
    pos: usize,
}

impl Chunk {
    fn next(&mut self) -> Result<Option<ChunkItem>, BagError> {
        loop {
            if self.pos >= self.buf.len() {
                return Ok(None);
            }
            let bad = || BagError::Malformed("record exceeds chunk".into());
            let b = &self.buf;
            let hlen = read_u32(b, self.pos).ok_or_else(bad)? as usize;
            let hstart = self.pos + 4;
            let hend = hstart.checked_add(hlen).ok_or_else(bad)?;
            let dlen = read_u32(b, hend).ok_or_else(bad)? as usize;
            let dend = (hend + 4).checked_add(dlen).ok_or_else(bad)?;
            if dend > b.len() {
                return Err(bad());
            }
            let fields = Fields::parse(&b[hstart..hend])?;
            let data = &b[hend + 4..dend];
            self.pos = dend;
            match fields.op()? {
                OP_MESSAGE => {
                    return Ok(Some(ChunkItem::Message {
                        conn: fields.u32("conn")?,
                        time: fields.time("time")?,
                        data: data.to_vec(),
                    }))
                }
                OP_CONNECTION => {
                    return Ok(Some(ChunkItem::Connection(parse_connection(
                        &fields, data,
                    )?)))
                }
                _ => {}
            }
        }
    }
}

impl<R: Read + Seek> BagReader<R> {
    pub fn new(mut r: R) -> Result<Self, BagError> {
        let file_len = r.seek(SeekFrom::End(0))?;
        r.seek(SeekFrom::Start(0))?;
        let mut magic = [0u8; MAGIC.len()];
        if !read_full(&mut r, &mut magic)? || magic != *MAGIC {
            return Err(BagError::NotABag);
        }
        let pos = MAGIC.len() as u64;
        let head = match read_head(&mut r, pos, file_len)? {
            Next::Record(h) => h,
            Next::Eof => return Err(BagError::Truncated { offset: pos }),
        };
        if head.fields.op()? != OP_BAG_HEADER {
            return Err(BagError::Malformed(
                "first record is not a bag header".into(),
            ));
        }
        let index_pos = head.fields.u64("index_pos").unwrap_or(0);
        let data_start = head.data_pos + head.data_len;
        let mut reader = BagReader {
            r,
            conns: Vec::new(),
            by_id: HashMap::new(),
            data_start,
            file_len,
        };
        if index_pos > 0 && index_pos < file_len {
            // Errors here (damaged index) fall through to the sequential scan below.
            let _ = reader.read_index_connections(index_pos);
        }
        if reader.conns.is_empty() {
            reader.scan_connections();
        }
        Ok(reader)
    }

    pub fn connections(&self) -> &[Connection] {
        &self.conns
    }

    fn add_connection(&mut self, c: Connection) {
        if !self.by_id.contains_key(&c.id) {
            self.by_id.insert(c.id, self.conns.len());
            self.conns.push(c);
        }
    }

    fn read_index_connections(&mut self, index_pos: u64) -> Result<(), BagError> {
        let mut pos = index_pos;
        self.r.seek(SeekFrom::Start(pos))?;
        while let Next::Record(head) = read_head(&mut self.r, pos, self.file_len)? {
            let next = head.data_pos + head.data_len;
            if head.fields.op()? == OP_CONNECTION {
                let mut data = vec![0u8; head.data_len as usize];
                self.r.read_exact(&mut data)?;
                self.add_connection(parse_connection(&head.fields, &data)?);
            } else {
                self.r.seek(SeekFrom::Start(next))?;
            }
            pos = next;
        }
        Ok(())
    }

    /// Fallback for unindexed/damaged bags: collect connections by walking all chunks.
    fn scan_connections(&mut self) {
        let mut it = Messages::new(self, None);
        while let Some(Ok(_)) = it.next() {}
    }

    /// Iterates messages of the given topics (all if empty) in file order.
    pub fn messages(
        &mut self,
        topics: &[&str],
    ) -> impl Iterator<Item = Result<RawMessage, BagError>> + '_ {
        let filter = if topics.is_empty() {
            None
        } else {
            Some(topics.iter().map(|t| t.to_string()).collect())
        };
        Messages::new(self, filter)
    }
}

struct Messages<'a, R: Read + Seek> {
    reader: &'a mut BagReader<R>,
    filter: Option<Vec<String>>,
    pos: u64,
    chunk: Option<Chunk>,
    done: bool,
}

impl<'a, R: Read + Seek> Messages<'a, R> {
    fn new(reader: &'a mut BagReader<R>, filter: Option<Vec<String>>) -> Self {
        let pos = reader.data_start;
        Messages {
            reader,
            filter,
            pos,
            chunk: None,
            done: false,
        }
    }

    /// Reads the next top-level record; loads a chunk if it is one.
    fn advance(&mut self) -> Result<bool, BagError> {
        let r = &mut self.reader;
        r.r.seek(SeekFrom::Start(self.pos))?;
        let head = match read_head(&mut r.r, self.pos, r.file_len)? {
            Next::Eof => return Ok(false),
            Next::Record(h) => h,
        };
        let next = head.data_pos + head.data_len;
        match head.fields.op()? {
            OP_CHUNK => {
                let compression = head.fields.string("compression").unwrap_or_default();
                let size = head.fields.u32("size").unwrap_or(0) as usize;
                let mut data = vec![0u8; head.data_len as usize];
                r.r.read_exact(&mut data)?;
                self.chunk = Some(Chunk {
                    buf: decompress(&compression, data, size)?,
                    pos: 0,
                });
            }
            OP_CONNECTION => {
                let mut data = vec![0u8; head.data_len as usize];
                r.r.read_exact(&mut data)?;
                r.add_connection(parse_connection(&head.fields, &data)?);
            }
            _ => {}
        }
        self.pos = next;
        Ok(true)
    }

    fn step(&mut self) -> Result<Option<RawMessage>, BagError> {
        loop {
            if let Some(chunk) = &mut self.chunk {
                match chunk.next()? {
                    Some(ChunkItem::Connection(c)) => self.reader.add_connection(c),
                    Some(ChunkItem::Message { conn, time, data }) => {
                        let idx = self.reader.by_id.get(&conn).copied().ok_or_else(|| {
                            BagError::Malformed(format!("message for unknown connection {conn}"))
                        })?;
                        let topic = &self.reader.conns[idx].topic;
                        if let Some(f) = &self.filter {
                            if !f.iter().any(|t| t == topic) {
                                continue;
                            }
                        }
                        return Ok(Some(RawMessage {
                            conn_id: conn,
                            topic: topic.clone(),
                            time,
                            data,
                        }));
                    }
                    None => self.chunk = None,
                }
            } else if !self.advance()? {
                return Ok(None);
            }
        }
    }
}

impl<R: Read + Seek> Iterator for Messages<'_, R> {
    type Item = Result<RawMessage, BagError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        match self.step() {
            Ok(Some(m)) => Some(Ok(m)),
            Ok(None) => {
                self.done = true;
                None
            }
            Err(e) => {
                self.done = true;
                Some(Err(e))
            }
        }
    }
}
