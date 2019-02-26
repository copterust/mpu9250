/// Marker trait for mode
pub(crate) trait MpuMode {}
/// Accelerometer + Gyroscope + Temperature Sensor
pub struct Imu;
impl MpuMode for Imu {}

/// Magnetometer + Accelerometer + Gyroscope + Temperature Sensor
pub struct Marg;
impl MpuMode for Marg {}
