//! Decoders for the ROS1 message types found in JMK6 recordings.
//!
//! ROS1 serialization is little-endian; strings and arrays carry a `u32` length prefix.

mod image;
mod imu;
mod pointcloud2;

#[cfg(any(test, feature = "test-util"))]
pub mod encode;

pub use image::{compressed_image_stamp, decode_compressed_image, CompressedImage};
pub use imu::{decode_imu, ImuSample};
pub use pointcloud2::{decode_pointcloud2, LidarFrame, LidarPoint};

/// Errors produced while decoding a message payload.
#[derive(Debug, thiserror::Error)]
pub enum MsgError {
    #[error("message is truncated: needed {needed} bytes at offset {offset}")]
    Truncated { offset: usize, needed: usize },
    #[error("malformed message: {0}")]
    Malformed(String),
    #[error("unsupported message: {0}")]
    Unsupported(String),
}

/// Little-endian cursor over a message payload.
pub(crate) struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    pub(crate) fn new(data: &'a [u8]) -> Self {
        Cursor { data, pos: 0 }
    }

    pub(crate) fn take(&mut self, n: usize) -> Result<&'a [u8], MsgError> {
        let end = self.pos.checked_add(n).filter(|&e| e <= self.data.len());
        match end {
            Some(end) => {
                let s = &self.data[self.pos..end];
                self.pos = end;
                Ok(s)
            }
            None => Err(MsgError::Truncated {
                offset: self.pos,
                needed: n,
            }),
        }
    }

    pub(crate) fn u8(&mut self) -> Result<u8, MsgError> {
        Ok(self.take(1)?[0])
    }

    pub(crate) fn u32(&mut self) -> Result<u32, MsgError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }

    pub(crate) fn f64(&mut self) -> Result<f64, MsgError> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    pub(crate) fn string(&mut self) -> Result<String, MsgError> {
        let n = self.u32()? as usize;
        Ok(String::from_utf8_lossy(self.take(n)?).into_owned())
    }

    /// `std_msgs/Header`: returns `stamp` in seconds ([`stamp_secs`]). `seq` and `frame_id`
    /// are skipped.
    pub(crate) fn header(&mut self) -> Result<f64, MsgError> {
        let _seq = self.u32()?;
        let sec = self.u32()?;
        let nsec = self.u32()?;
        let _frame_id = self.string()?;
        Ok(stamp_secs(sec, nsec))
    }
}

/// A header stamp of `sec` seconds and `nsec` nanoseconds in seconds, as every decoder here
/// gives it.
pub fn stamp_secs(sec: u32, nsec: u32) -> f64 {
    f64::from(sec) + f64::from(nsec) * 1e-9
}
