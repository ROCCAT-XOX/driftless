//! Where the messages of a bag lie, found without reading the chunks whole
//! ([`BagReader::message_places`]), and the data of one message read at its place
//! ([`BagReader::read_message`]).
//!
//! The walk goes through the records as [`BagReader::messages`] does – the same messages in
//! the same order, the same errors, the same end at a cut – but reads of each chunk only the
//! headers of its records (and the data of its connection records):
//! - an uncompressed chunk at the places of its records;
//! - an lz4 chunk block by block from the start, only as far as its records go: a stored
//!   (uncompressed) block is read only where a header lies in it, a compressed one is
//!   decompressed on its own. Its checksums are not verified here; [`BagReader::read_message`]
//!   decompresses the chunk of a message whole and checks it as the sequential reader does.
//!
//! An lz4 frame of another kind (linked blocks, a dictionary, another version, anything after
//! its end) and anything that does not fit is decompressed whole, as the sequential reader
//! does it, so it gives the same records or the same error. So is a frame whose blocks hold
//! more than the chunk's header gives as its size, more than [`LAZY_BYTES`] or more than
//! [`LAZY_BLOCKS`] blocks: what the walk keeps of a chunk is never more than that, nor more
//! than the sequential reader holds of it.

use super::reader::{decompress, parse_connection, read_head, BagReader, Next};
use super::record::*;
use super::BagError;
use std::io::{Read, Seek, SeekFrom};

/// How the records of a chunk are stored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChunkCompression {
    None,
    Lz4,
}

/// Where a message lies in its bag: in the chunk whose data starts at byte `chunk_pos` of the
/// file (`chunk_len` bytes there, its records `chunk_size` bytes as the chunk's header says,
/// stored as `compression`), its data at `offset` … `offset + len` of the chunk's records.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessagePlace {
    pub chunk_pos: u64,
    pub chunk_len: u64,
    pub chunk_size: u32,
    pub compression: ChunkCompression,
    pub offset: u64,
    pub len: u64,
}

/// A message as [`BagReader::message_places`] finds it, without its data: its connection, its
/// topic, the time it was recorded, its place and the first bytes of its data.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacedMessage {
    pub conn_id: u32,
    pub topic: String,
    pub time: Time,
    pub place: MessagePlace,
    /// The first bytes of its data (all of it if it is shorter).
    pub head: Vec<u8>,
}

/// The magic number of an lz4 frame.
const LZ4_MAGIC: u32 = 0x184D_2204;
/// Bits of the lz4 frame descriptor (FLG): the version (01), independent blocks, block
/// checksums, the content size, the content checksum, a dictionary; reserved.
const FLG_VERSION_MASK: u8 = 0b1100_0000;
const FLG_VERSION: u8 = 0b0100_0000;
const FLG_INDEPENDENT: u8 = 0b0010_0000;
const FLG_BLOCK_CHECKSUMS: u8 = 0b0001_0000;
const FLG_CONTENT_SIZE: u8 = 0b0000_1000;
const FLG_CONTENT_CHECKSUM: u8 = 0b0000_0100;
const FLG_RESERVED: u8 = 0b0000_0010;
const FLG_DICTIONARY: u8 = 0b0000_0001;
/// An lz4 block stored without compression.
const BLOCK_STORED: u32 = 0x8000_0000;
/// Pieces of the file are read at least this many bytes at a time.
const PEEK: usize = 4096;
/// The blocks of an lz4 chunk read one by one hold at most this many bytes, decompressed (as
/// much as the sequential reader sets aside for a chunk up front), and never more than the
/// chunk's header gives as its size; a chunk with more is decompressed whole.
const LAZY_BYTES: u64 = 1 << 28;
/// … and they are at most this many (the blocks of a common encoder hold 64 KiB or more, so this
/// is as much again); a frame of more blocks is decompressed whole as well.
const LAZY_BLOCKS: usize = 4096;

/// Reads pieces of the file at given places, keeping the last [`PEEK`] bytes or more read; and
/// the buffers the compressed blocks of a frame are read into and decompressed into, one block
/// after the other.
#[derive(Default)]
struct Peeker {
    start: u64,
    buf: Vec<u8>,
    input: Vec<u8>,
    output: Vec<u8>,
}

impl Peeker {
    /// The `len` bytes at `pos`; the caller made sure they lie in the file.
    fn read<R: Read + Seek>(
        &mut self,
        r: &mut R,
        pos: u64,
        len: usize,
        file_len: u64,
    ) -> Result<&[u8], BagError> {
        let end = pos + len as u64;
        let held = self.start + self.buf.len() as u64;
        if pos < self.start || end > held {
            let want = (len.max(PEEK) as u64).min(file_len.saturating_sub(pos)) as usize;
            if want < len {
                return Err(BagError::Truncated { offset: pos });
            }
            r.seek(SeekFrom::Start(pos))?;
            self.buf.resize(want, 0);
            r.read_exact(&mut self.buf)?;
            self.start = pos;
        }
        let at = (pos - self.start) as usize;
        Ok(&self.buf[at..at + len])
    }
}

/// A block of an lz4 frame read so far.
struct Block {
    /// Its place in the decompressed data and its length there.
    start: u64,
    len: u64,
    /// Stored: where its bytes lie in the file. Compressed: `None`, its bytes in `data` (just
    /// those: `len` of them).
    stored_at: Option<u64>,
    data: Vec<u8>,
}

/// An lz4 frame read block by block from its start (see the module docs).
struct Frame {
    /// The largest block, and whether every block has a checksum.
    block_max: usize,
    block_checksums: bool,
    content_checksum: bool,
    /// The file position of the next block's header, and where the chunk's data ends.
    next: u64,
    end: u64,
    blocks: Vec<Block>,
    /// The decompressed bytes of the blocks so far, and the most they may be: the chunk's size
    /// by its header, at most [`LAZY_BYTES`].
    total: u64,
    limit: u64,
    /// The end mark was read.
    ended: bool,
}

/// The records of a chunk as the walk reads them.
enum Records {
    /// Stored as they are, at this place of the file.
    Plain { pos: u64, len: u64 },
    /// An lz4 frame, read as far as needed.
    Lz4(Frame),
    /// Decompressed whole.
    Whole(Vec<u8>),
}

/// One record of a chunk.
enum Item {
    Message {
        conn: u32,
        time: Time,
        offset: u64,
        len: u64,
    },
    Connection(super::Connection),
}

/// A chunk being walked.
struct Chunk {
    pos: u64,
    len: u64,
    size: u32,
    compression: ChunkCompression,
    records: Records,
    /// The next record within the records.
    at: u64,
}

impl Frame {
    /// The frame of the chunk data at `pos` (`len` bytes, `size` decompressed as the chunk's
    /// header says), if it is one this module reads block by block (see the module docs).
    fn open<R: Read + Seek>(
        r: &mut R,
        peek: &mut Peeker,
        pos: u64,
        len: u64,
        size: u32,
        file_len: u64,
    ) -> Result<Option<Frame>, BagError> {
        if len < 7 {
            return Ok(None);
        }
        let head = peek.read(r, pos, (len as usize).min(19), file_len)?;
        let magic = u32::from_le_bytes(head[..4].try_into().expect("4 bytes"));
        let (flg, bd) = (head[4], head[5]);
        let plain = magic == LZ4_MAGIC
            && flg & FLG_VERSION_MASK == FLG_VERSION
            && flg & (FLG_RESERVED | FLG_DICTIONARY) == 0
            && flg & FLG_INDEPENDENT != 0
            && bd & !0b0111_0000 == 0;
        let block_max = match (bd >> 4) & 0b111 {
            4 => 64 << 10,
            5 => 256 << 10,
            6 => 1 << 20,
            7 => 4 << 20,
            _ => return Ok(None),
        };
        if !plain {
            return Ok(None);
        }
        let header = 7 + if flg & FLG_CONTENT_SIZE != 0 { 8 } else { 0 };
        if header > len {
            return Ok(None);
        }
        Ok(Some(Frame {
            block_max,
            block_checksums: flg & FLG_BLOCK_CHECKSUMS != 0,
            content_checksum: flg & FLG_CONTENT_CHECKSUM != 0,
            next: pos + header,
            end: pos + len,
            blocks: Vec::new(),
            total: 0,
            limit: u64::from(size).min(LAZY_BYTES),
            ended: false,
        }))
    }

    /// Reads the next block (its header; a compressed one whole, decompressed). `false` where
    /// the frame does not go on as this module reads it, or holds more than it may (more bytes
    /// than its limit, more than [`LAZY_BLOCKS`] blocks): then it is decompressed whole.
    fn read_block<R: Read + Seek>(
        &mut self,
        r: &mut R,
        peek: &mut Peeker,
        file_len: u64,
    ) -> Result<bool, BagError> {
        if self.next + 4 > self.end {
            return Ok(false);
        }
        let head = peek.read(r, self.next, 4, file_len)?;
        let word = u32::from_le_bytes(head.try_into().expect("4 bytes"));
        if word == 0 {
            // The end mark, then the checksum of the content; nothing may follow.
            let after = self.next + 4 + if self.content_checksum { 4 } else { 0 };
            self.ended = true;
            return Ok(after == self.end);
        }
        let stored = word & BLOCK_STORED != 0;
        let size = u64::from(word & !BLOCK_STORED);
        let at = self.next + 4;
        let next = at + size + if self.block_checksums { 4 } else { 0 };
        if size == 0
            || size > self.block_max as u64
            || next > self.end
            || self.blocks.len() >= LAZY_BLOCKS
        {
            return Ok(false);
        }
        let block = if stored {
            if self.total + size > self.limit {
                return Ok(false);
            }
            Block {
                start: self.total,
                len: size,
                stored_at: Some(at),
                data: Vec::new(),
            }
        } else {
            // Read and decompressed in the walk's buffers; the block keeps just its bytes.
            peek.input.resize(size as usize, 0);
            r.seek(SeekFrom::Start(at))?;
            r.read_exact(&mut peek.input)?;
            if peek.output.len() < self.block_max {
                peek.output.resize(self.block_max, 0);
            }
            let out = &mut peek.output[..self.block_max];
            let Ok(n) = lz4_flex::block::decompress_into(&peek.input, out) else {
                return Ok(false);
            };
            if n == 0 || self.total + n as u64 > self.limit {
                return Ok(false);
            }
            Block {
                start: self.total,
                len: n as u64,
                stored_at: None,
                data: out[..n].to_vec(),
            }
        };
        self.total += block.len;
        self.blocks.push(block);
        self.next = next;
        Ok(true)
    }
}

impl Chunk {
    /// Decompresses the records whole, as the sequential reader does.
    fn whole<R: Read + Seek>(&mut self, r: &mut R, compression: &str) -> Result<(), BagError> {
        let mut data = vec![0u8; self.len as usize];
        r.seek(SeekFrom::Start(self.pos))?;
        r.read_exact(&mut data)?;
        self.records = Records::Whole(decompress(compression, data, self.size as usize)?);
        Ok(())
    }

    /// Whether the records are at least `n` bytes long.
    fn reaches<R: Read + Seek>(
        &mut self,
        r: &mut R,
        peek: &mut Peeker,
        n: u64,
        file_len: u64,
    ) -> Result<bool, BagError> {
        loop {
            match &mut self.records {
                Records::Plain { len, .. } => return Ok(n <= *len),
                Records::Whole(buf) => return Ok(n <= buf.len() as u64),
                Records::Lz4(frame) => {
                    if n <= frame.total {
                        return Ok(true);
                    }
                    if frame.ended {
                        return Ok(false);
                    }
                    if !frame.read_block(r, peek, file_len)? {
                        self.whole(r, "lz4")?;
                    }
                }
            }
        }
    }

    /// The `len` bytes of the records at `at`; the caller made sure they are there
    /// ([`Self::reaches`]).
    fn bytes<R: Read + Seek>(
        &mut self,
        r: &mut R,
        peek: &mut Peeker,
        at: u64,
        len: u64,
        file_len: u64,
    ) -> Result<Vec<u8>, BagError> {
        match &self.records {
            Records::Plain { pos, .. } => {
                Ok(peek.read(r, pos + at, len as usize, file_len)?.to_vec())
            }
            Records::Whole(buf) => Ok(buf[at as usize..(at + len) as usize].to_vec()),
            Records::Lz4(frame) => {
                let mut out = Vec::with_capacity(len as usize);
                let end = at + len;
                let first = frame.blocks.partition_point(|b| b.start + b.len <= at);
                for b in &frame.blocks[first..] {
                    if b.start >= end {
                        break;
                    }
                    let from = at.max(b.start) - b.start;
                    let to = end.min(b.start + b.len) - b.start;
                    match b.stored_at {
                        Some(pos) => out.extend_from_slice(peek.read(
                            r,
                            pos + from,
                            (to - from) as usize,
                            file_len,
                        )?),
                        None => out.extend_from_slice(&b.data[from as usize..to as usize]),
                    }
                }
                debug_assert_eq!(out.len() as u64, len);
                Ok(out)
            }
        }
    }

    /// The next record of interest, as `Chunk::next` of the sequential reader finds it (the
    /// same checks in the same order, so the same errors).
    fn next<R: Read + Seek>(
        &mut self,
        r: &mut R,
        peek: &mut Peeker,
        file_len: u64,
    ) -> Result<Option<Item>, BagError> {
        let bad = || BagError::Malformed("record exceeds chunk".into());
        let u32_at =
            |c: &mut Chunk, r: &mut R, peek: &mut Peeker, at: u64| -> Result<u64, BagError> {
                if !c.reaches(r, peek, at + 4, file_len)? {
                    return Err(bad());
                }
                let b = c.bytes(r, peek, at, 4, file_len)?;
                Ok(u64::from(u32::from_le_bytes(
                    b[..].try_into().expect("4 bytes"),
                )))
            };
        loop {
            let pos = self.at;
            if !self.reaches(r, peek, pos + 1, file_len)? {
                return Ok(None);
            }
            let hlen = u32_at(self, r, peek, pos)?;
            let hstart = pos + 4;
            let hend = hstart + hlen;
            let dlen = u32_at(self, r, peek, hend)?;
            let dend = hend + 4 + dlen;
            if !self.reaches(r, peek, dend, file_len)? {
                return Err(bad());
            }
            let fields = Fields::parse(&self.bytes(r, peek, hstart, hlen, file_len)?)?;
            self.at = dend;
            match fields.op()? {
                OP_MESSAGE => {
                    return Ok(Some(Item::Message {
                        conn: fields.u32("conn")?,
                        time: fields.time("time")?,
                        offset: hend + 4,
                        len: dlen,
                    }))
                }
                OP_CONNECTION => {
                    let data = self.bytes(r, peek, hend + 4, dlen, file_len)?;
                    return Ok(Some(Item::Connection(parse_connection(&fields, &data)?)));
                }
                _ => {}
            }
        }
    }
}

/// The walk of [`BagReader::message_places`].
pub struct Places<'a, R: Read + Seek> {
    reader: &'a mut BagReader<R>,
    filter: Option<Vec<String>>,
    head: usize,
    pos: u64,
    chunk: Option<Chunk>,
    peek: Peeker,
    done: bool,
}

impl<R: Read + Seek> Places<'_, R> {
    /// Reads the next top-level record; opens a chunk if it is one (as `Messages::advance`).
    fn advance(&mut self) -> Result<bool, BagError> {
        let file_len = self.reader.file_len;
        let r = &mut self.reader.r;
        r.seek(SeekFrom::Start(self.pos))?;
        let head = match read_head(r, self.pos, file_len)? {
            Next::Eof => return Ok(false),
            Next::Record(h) => h,
        };
        let next = head.data_pos + head.data_len;
        match head.fields.op()? {
            OP_CHUNK => {
                let compression = head.fields.string("compression").unwrap_or_default();
                let size = head.fields.u32("size").unwrap_or(0);
                let (pos, len) = (head.data_pos, head.data_len);
                let mut chunk = Chunk {
                    pos,
                    len,
                    size,
                    compression: ChunkCompression::None,
                    records: Records::Plain { pos, len },
                    at: 0,
                };
                match compression.as_str() {
                    "none" => {}
                    "lz4" => {
                        chunk.compression = ChunkCompression::Lz4;
                        match Frame::open(r, &mut self.peek, pos, len, size, file_len)? {
                            Some(frame) => chunk.records = Records::Lz4(frame),
                            None => chunk.whole(r, "lz4")?,
                        }
                    }
                    // fails as the sequential reader fails
                    other => chunk.whole(r, other)?,
                }
                self.chunk = Some(chunk);
            }
            OP_CONNECTION => {
                let mut data = vec![0u8; head.data_len as usize];
                r.read_exact(&mut data)?;
                let c = parse_connection(&head.fields, &data)?;
                self.reader.add_connection(c);
            }
            _ => {}
        }
        self.pos = next;
        Ok(true)
    }

    fn step(&mut self) -> Result<Option<PlacedMessage>, BagError> {
        let file_len = self.reader.file_len;
        loop {
            if let Some(chunk) = &mut self.chunk {
                match chunk.next(&mut self.reader.r, &mut self.peek, file_len)? {
                    Some(Item::Connection(c)) => self.reader.add_connection(c),
                    Some(Item::Message {
                        conn,
                        time,
                        offset,
                        len,
                    }) => {
                        let topic = self.reader.topic_of(conn).ok_or_else(|| {
                            BagError::Malformed(format!("message for unknown connection {conn}"))
                        })?;
                        if let Some(f) = &self.filter {
                            if !f.iter().any(|t| t == topic) {
                                continue;
                            }
                        }
                        let topic = topic.to_string();
                        let head = (self.head as u64).min(len);
                        let head = chunk.bytes(
                            &mut self.reader.r,
                            &mut self.peek,
                            offset,
                            head,
                            file_len,
                        )?;
                        return Ok(Some(PlacedMessage {
                            conn_id: conn,
                            topic,
                            time,
                            place: MessagePlace {
                                chunk_pos: chunk.pos,
                                chunk_len: chunk.len,
                                chunk_size: chunk.size,
                                compression: chunk.compression,
                                offset,
                                len,
                            },
                            head,
                        }));
                    }
                    None => self.chunk = None,
                }
            } else if !self.advance()? {
                // as the sequential reader: a header pointing past the end is a cut
                let r = &self.reader;
                if r.index_pos > 0 && r.index_pos >= r.file_len {
                    return Err(BagError::Truncated { offset: r.file_len });
                }
                return Ok(None);
            }
        }
    }
}

impl<R: Read + Seek> Iterator for Places<'_, R> {
    type Item = Result<PlacedMessage, BagError>;

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

impl<R: Read + Seek> BagReader<R> {
    /// The messages of `topics` (all if empty) in file order, as [`Self::messages`] gives them
    /// and with the same errors, but each without its data: its place in the file and its first
    /// `head` bytes (see the module docs).
    pub fn message_places(&mut self, topics: &[&str], head: usize) -> Places<'_, R> {
        let pos = self.data_start;
        Places {
            reader: self,
            filter: (!topics.is_empty()).then(|| topics.iter().map(|t| t.to_string()).collect()),
            head,
            pos,
            chunk: None,
            peek: Peeker::default(),
            done: false,
        }
    }

    /// The data of the message at `place` (found by [`Self::message_places`] in this bag): of
    /// an uncompressed chunk read where it lies, of an lz4 chunk with the chunk decompressed
    /// whole and checked as [`Self::messages`] does it. Fails if the place is not in the file or
    /// the chunk cannot be decompressed.
    pub fn read_message(&mut self, place: &MessagePlace) -> Result<Vec<u8>, BagError> {
        let outside = || BagError::Malformed("message outside its chunk".into());
        if place
            .chunk_pos
            .checked_add(place.chunk_len)
            .is_none_or(|e| e > self.file_len)
        {
            return Err(BagError::Truncated {
                offset: place.chunk_pos,
            });
        }
        match place.compression {
            ChunkCompression::None => {
                if place
                    .offset
                    .checked_add(place.len)
                    .is_none_or(|e| e > place.chunk_len)
                {
                    return Err(outside());
                }
                let mut data = vec![0u8; place.len as usize];
                self.r
                    .seek(SeekFrom::Start(place.chunk_pos + place.offset))?;
                self.r.read_exact(&mut data)?;
                Ok(data)
            }
            ChunkCompression::Lz4 => {
                let records = self.read_chunk(place)?;
                let range = place.offset as usize..(place.offset + place.len) as usize;
                records.get(range).map(<[u8]>::to_vec).ok_or_else(outside)
            }
        }
    }

    /// The records of the chunk holding the message at `place`, decompressed whole and checked
    /// as [`Self::messages`] does it.
    pub fn read_chunk(&mut self, place: &MessagePlace) -> Result<Vec<u8>, BagError> {
        if place
            .chunk_pos
            .checked_add(place.chunk_len)
            .is_none_or(|e| e > self.file_len)
        {
            return Err(BagError::Truncated {
                offset: place.chunk_pos,
            });
        }
        let mut data = vec![0u8; place.chunk_len as usize];
        self.r.seek(SeekFrom::Start(place.chunk_pos))?;
        self.r.read_exact(&mut data)?;
        let name = match place.compression {
            ChunkCompression::None => "none",
            ChunkCompression::Lz4 => "lz4",
        };
        decompress(name, data, place.chunk_size as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lz4_flex::frame::{BlockMode, BlockSize, FrameDecoder, FrameEncoder, FrameInfo};
    use std::io::{Cursor, Write};

    /// An lz4 frame of independent blocks of at most 64 KiB, with the checksum of its content:
    /// a block for every part (each at most 64 KiB).
    fn frame_of(parts: &[Vec<u8>]) -> Vec<u8> {
        let info = FrameInfo::new()
            .block_size(BlockSize::Max64KB)
            .block_mode(BlockMode::Independent)
            .content_checksum(true);
        let mut enc = FrameEncoder::with_frame_info(info, Vec::new());
        for part in parts {
            enc.write_all(part).unwrap();
            enc.flush().unwrap();
        }
        enc.finish().unwrap()
    }

    /// Reads the blocks of the frame `bytes` (said to hold `size` bytes, as a chunk's header says
    /// it) as the walk does: whether it got through to the end mark, and the frame as far as it
    /// got.
    fn read_blocks(bytes: &[u8], size: u32) -> (bool, Frame) {
        let len = bytes.len() as u64;
        let mut r = Cursor::new(bytes.to_vec());
        let mut peek = Peeker::default();
        let mut frame = Frame::open(&mut r, &mut peek, 0, len, size, len)
            .unwrap()
            .expect("a frame read block by block");
        while !frame.ended {
            if !frame.read_block(&mut r, &mut peek, len).unwrap() {
                return (false, frame);
            }
        }
        (true, frame)
    }

    /// `n` bytes that compress well.
    fn pattern(n: usize, k: u8) -> Vec<u8> {
        (0..n).map(|i| (i % 7) as u8 + k).collect()
    }

    /// `n` bytes that do not compress.
    fn noise(n: usize, seed: u32) -> Vec<u8> {
        let mut x = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        (0..n)
            .map(|_| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (x >> 24) as u8
            })
            .collect()
    }

    /// What the blocks read hold, put together (the stored ones from `bytes`).
    fn content(bytes: &[u8], frame: &Frame) -> Vec<u8> {
        let mut out = Vec::new();
        for b in &frame.blocks {
            match b.stored_at {
                Some(at) => out.extend_from_slice(&bytes[at as usize..(at + b.len) as usize]),
                None => out.extend_from_slice(&b.data),
            }
        }
        out
    }

    /// The bytes the blocks read keep in memory.
    fn kept(frame: &Frame) -> usize {
        frame.blocks.iter().map(|b| b.data.capacity()).sum()
    }

    /// A compressed block keeps just its bytes, a stored one none; together they hold the frame's
    /// content, as the frame decoder gives it.
    #[test]
    fn blocks_keep_just_their_bytes() {
        let parts = vec![
            pattern(65_536, 1),
            noise(65_536, 7),
            pattern(10_000, 3),
            noise(500, 9),
        ];
        let whole = parts.concat();
        let bytes = frame_of(&parts);
        let (through, frame) = read_blocks(&bytes, whole.len() as u32);
        assert!(through);
        assert_eq!(frame.blocks.len(), 4);
        let compressed: Vec<&Block> = frame
            .blocks
            .iter()
            .filter(|b| b.stored_at.is_none())
            .collect();
        assert_eq!(
            compressed.len(),
            2,
            "the patterns compress, the noise does not"
        );
        for b in compressed {
            assert_eq!(b.data.capacity() as u64, b.len);
        }
        assert_eq!(kept(&frame), 65_536 + 10_000);
        assert_eq!(content(&bytes, &frame), whole);
        let mut decoded = Vec::new();
        FrameDecoder::new(&bytes[..])
            .read_to_end(&mut decoded)
            .unwrap();
        assert_eq!(decoded, whole);
    }

    /// Blocks that hold more than the chunk's header says are not kept: reading stops at the
    /// block that would go beyond (the chunk is then decompressed whole), having kept no more than
    /// that size – also where a few bytes in the file stand for megabytes.
    #[test]
    fn no_more_than_the_size_is_kept() {
        // 12 MiB of zeros in 192 blocks: a few KiB in the file.
        let parts: Vec<Vec<u8>> = (0..192).map(|_| vec![0u8; 65_536]).collect();
        let bytes = frame_of(&parts);
        assert!(bytes.len() < 100_000, "{}", bytes.len());
        let (through, frame) = read_blocks(&bytes, 192 * 65_536);
        assert!(through);
        assert_eq!(frame.total, 192 * 65_536);
        let (through, frame) = read_blocks(&bytes, 1 << 20);
        assert!(!through);
        assert_eq!(frame.blocks.len(), 16);
        assert!(kept(&frame) <= 1 << 20, "{}", kept(&frame));
        // Stored blocks count as well.
        let parts: Vec<Vec<u8>> = (0..4).map(|i| noise(65_536, i)).collect();
        let bytes = frame_of(&parts);
        let (through, frame) = read_blocks(&bytes, 3 * 65_536 - 1);
        assert!(!through);
        assert_eq!(frame.blocks.len(), 2);
        assert_eq!(kept(&frame), 0);
    }

    /// A frame of more than [`LAZY_BLOCKS`] blocks is not read block by block.
    #[test]
    fn no_more_than_so_many_blocks_are_kept() {
        let parts: Vec<Vec<u8>> = (0..LAZY_BLOCKS + 10).map(|i| vec![i as u8]).collect();
        let bytes = frame_of(&parts);
        let (through, frame) = read_blocks(&bytes, parts.len() as u32);
        assert!(!through);
        assert_eq!(frame.blocks.len(), LAZY_BLOCKS);
    }
}
