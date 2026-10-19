//! `sensor_msgs/CompressedImage`.

use super::{Cursor, MsgError};

/// A compressed camera image (payload is left untouched, e.g. JPEG bytes).
#[derive(Debug, Clone, PartialEq)]
pub struct CompressedImage {
    pub stamp: f64,
    pub format: String,
    pub data: Vec<u8>,
}

pub fn decode_compressed_image(data: &[u8]) -> Result<CompressedImage, MsgError> {
    let mut c = Cursor::new(data);
    let stamp = c.header()?;
    let format = c.string()?;
    let n = c.u32()? as usize;
    let data = c.take(n)?.to_vec();
    Ok(CompressedImage {
        stamp,
        format,
        data,
    })
}
