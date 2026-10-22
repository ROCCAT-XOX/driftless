//! Low-level record and header-field parsing shared by reader and writer.

use super::BagError;
use std::collections::HashMap;

pub const MAGIC: &[u8] = b"#ROSBAG V2.0\n";

pub const OP_MESSAGE: u8 = 0x02;
pub const OP_BAG_HEADER: u8 = 0x03;
/// Only the test writer emits index records; the reader skips them.
#[cfg(any(test, feature = "test-util"))]
pub const OP_INDEX: u8 = 0x04;
pub const OP_CHUNK: u8 = 0x05;
pub const OP_CHUNK_INFO: u8 = 0x06;
pub const OP_CONNECTION: u8 = 0x07;

/// ROS timestamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Time {
    pub sec: u32,
    pub nsec: u32,
}

impl Time {
    pub fn as_secs_f64(&self) -> f64 {
        f64::from(self.sec) + f64::from(self.nsec) * 1e-9
    }
}

/// Parsed header fields of one record.
pub struct Fields(HashMap<String, Vec<u8>>);

fn malformed(msg: &str) -> BagError {
    BagError::Malformed(msg.to_string())
}

pub fn read_u32(buf: &[u8], pos: usize) -> Option<u32> {
    let b = buf.get(pos..pos.checked_add(4)?)?;
    Some(u32::from_le_bytes(b.try_into().ok()?))
}

impl Fields {
    pub fn parse(mut buf: &[u8]) -> Result<Fields, BagError> {
        let mut map = HashMap::new();
        while !buf.is_empty() {
            let len =
                read_u32(buf, 0).ok_or_else(|| malformed("header field length cut off"))? as usize;
            let body = buf
                .get(4..4usize.saturating_add(len))
                .ok_or_else(|| malformed("header field exceeds header"))?;
            let eq = body
                .iter()
                .position(|&b| b == b'=')
                .ok_or_else(|| malformed("header field without `=`"))?;
            let name = String::from_utf8_lossy(&body[..eq]).into_owned();
            map.insert(name, body[eq + 1..].to_vec());
            buf = &buf[4 + len..];
        }
        Ok(Fields(map))
    }

    pub fn get(&self, name: &str) -> Option<&[u8]> {
        self.0.get(name).map(Vec::as_slice)
    }

    pub fn op(&self) -> Result<u8, BagError> {
        match self.get("op") {
            Some([b]) => Ok(*b),
            _ => Err(malformed("record without valid `op` field")),
        }
    }

    pub fn u32(&self, name: &str) -> Result<u32, BagError> {
        self.get(name)
            .and_then(|v| <[u8; 4]>::try_from(v).ok())
            .map(u32::from_le_bytes)
            .ok_or_else(|| BagError::Malformed(format!("missing or invalid field `{name}`")))
    }

    pub fn u64(&self, name: &str) -> Result<u64, BagError> {
        self.get(name)
            .and_then(|v| <[u8; 8]>::try_from(v).ok())
            .map(u64::from_le_bytes)
            .ok_or_else(|| BagError::Malformed(format!("missing or invalid field `{name}`")))
    }

    pub fn string(&self, name: &str) -> Option<String> {
        self.get(name)
            .map(|v| String::from_utf8_lossy(v).into_owned())
    }

    pub fn time(&self, name: &str) -> Result<Time, BagError> {
        self.get(name)
            .filter(|v| v.len() == 8)
            .map(|v| Time {
                sec: u32::from_le_bytes(v[..4].try_into().unwrap()),
                nsec: u32::from_le_bytes(v[4..].try_into().unwrap()),
            })
            .ok_or_else(|| BagError::Malformed(format!("missing or invalid field `{name}`")))
    }
}
