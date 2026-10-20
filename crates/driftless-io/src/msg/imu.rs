//! `sensor_msgs/Imu`.

use super::{Cursor, MsgError};

/// Angular velocity (rad/s) and linear acceleration (m/s^2). Orientation is ignored.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuSample {
    pub stamp: f64,
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
}

fn vec3(c: &mut Cursor) -> Result<[f64; 3], MsgError> {
    Ok([c.f64()?, c.f64()?, c.f64()?])
}

pub fn decode_imu(data: &[u8]) -> Result<ImuSample, MsgError> {
    let mut c = Cursor::new(data);
    let stamp = c.header()?;
    c.take(8 * (4 + 9))?; // orientation + covariance
    let gyro = vec3(&mut c)?;
    c.take(8 * 9)?; // angular_velocity_covariance
    let accel = vec3(&mut c)?;
    // linear_acceleration_covariance (72 bytes) is not needed.
    Ok(ImuSample { stamp, gyro, accel })
}
