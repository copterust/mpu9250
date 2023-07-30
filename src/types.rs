/// Marker trait for mode
pub(crate) trait MpuMode {}
/// Accelerometer + Gyroscope + Temperature Sensor
#[derive(Copy, Clone, Debug)]
pub struct Imu;
impl MpuMode for Imu {}

/// Magnetometer + Accelerometer + Gyroscope + Temperature Sensor
#[derive(Copy, Clone, Debug)]
pub struct Marg;
impl MpuMode for Marg {}

/// Accelerometer + Gyroscope + Dmp
#[cfg(feature = "dmp")]
#[derive(Copy, Clone, Debug)]
pub struct Dmp;
#[cfg(feature = "dmp")]
impl MpuMode for Dmp {}
