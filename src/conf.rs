//! Configuration for MPU9250.

use core::default::Default;
use core::marker::PhantomData;

use super::types;

/// Controls the gyroscope and temperature sensor data rates and bandwidth.
/// Can be either set to one of two FChoices, or to one of the 8
/// digital low pass filter modes. If the DLPF mode is used rate and bandwith
/// can be further tweaked by Sample Rate Divisor.
/// See page 13 of [`Register map`] for details.
/// Default is dlpf with default dlpf mode.
#[derive(Copy, Clone, Debug)]
pub enum GyroTempDataRate {
    /// FChoice x0:
    /// Gyroscope bandwidth=8800Hz, delay=0.064ms, Fs=32kHz;
    /// Temperature sensor bandwidth=4000Hz, delay=0.04ms.
    FChoice0, // Fchoice_b: 01 (inverted bits)
    /// FChoice 01:
    /// Gyroscope bandwidth=3600Hz, delay=0.11ms, Fs=32kHz;
    /// Temperature sensor bandwidth=4000Hz, delay=0.04ms.
    FChoice1, // Fchoice_b: 10
    /// FChoice set to 11 and data rate and bandwidth are controlled
    /// by Dlpf.
    DlpfConf(Dlpf),
}
impl Default for GyroTempDataRate {
    fn default() -> Self {
        GyroTempDataRate::DlpfConf(Dlpf::default())
    }
}
impl GyroTempDataRate {
    pub(crate) fn fchoice_b_bits(&self) -> u8 {
        match self {
            GyroTempDataRate::FChoice0 => 0b01,
            GyroTempDataRate::FChoice1 => 0b10,
            GyroTempDataRate::DlpfConf(_) => 0b00,
        }
    }

    pub(crate) fn dlpf_bits(&self) -> u8 {
        match self {
            GyroTempDataRate::FChoice0 => 0b000,
            GyroTempDataRate::FChoice1 => 0b000,
            GyroTempDataRate::DlpfConf(dlpf) => *dlpf as u8,
        }
    }
}

/// Controls the accelerometer data rate and bandwidth.
/// Can be either set to FChoice, or to one of the 8
/// digital low pass filter modes. If the DLPF mode is used rate and bandwith
/// can be further tweaked by Sample Rate Divisor.
/// See page 13 of [`Register map`] for details.
/// Noise Density for all values is 300 μg/rtHz.
/// Default is dlpf with default dlpf mode.
#[derive(Copy, Clone, Debug)]
pub enum AccelDataRate {
    /// ACCEL_FCHOICE 0:
    /// 3dB BW=1046Hz, delay=0.503ms, rate=4kHz.
    FChoice0, // Fchoice_b: 1 (inverted bit)
    /// FChoice set to 1 and data rate and bandwidth are controlled
    /// by Dlpf; rate = 1kHz.
    DlpfConf(Dlpf),
}
impl Default for AccelDataRate {
    fn default() -> Self {
        AccelDataRate::DlpfConf(Dlpf::default())
    }
}
impl AccelDataRate {
    pub(crate) fn accel_config_bits(&self) -> u8 {
        match self {
            AccelDataRate::FChoice0 => 0b00010000,
            AccelDataRate::DlpfConf(dlpf) => 0b00000000 | (*dlpf as u8),
        }
    }
}

/// Digital low pass filter configuration; default: _0;
#[derive(Copy, Clone, Debug)]
pub enum Dlpf {
    /// Accelerometer: bandwitdh=218.Hz, delay=1.88ms;
    /// Gyroscope: bandwidth=250Hz, delay=0.97ms, Fs=8kHz;
    /// Temperature sensor: bandwidth=4000 Hz, delay=0.04ms.
    _0 = 0,
    /// Accelerometer: bandwidth=218.1Hz, delay=1.88ms;
    /// Gyroscope: bandwidth=184Hz, delay=2.9ms, Fs=1kHz;
    /// Temperature sensor: bandwidth=188Hz delay=1.9ms.
    _1 = 1,
    /// Accelerometer: bandwidth=99Hz, delay=2.88ms;
    /// Gyroscope: bandwidth=92Hz, delay=3.9ms, Fs=1kHz;
    /// Temperature sensor: bandwidth=92Hz, delay=2.8ms.
    _2 = 2,
    /// Accelerometer bandwidth=44.8Hz, delay=4.88ms;
    /// Gyroscope: bandwidth=41Hz, delay=5.9ms, Fs=1kHz;
    /// Temperature sensor: bandwidth=42Hz, delay=4.8ms.
    _3 = 3,
    /// Accelerometer: bandwidth=21.2Hz, delay=8.87ms;
    /// Gyroscope: bandwidth=20Hz, delay=9.9ms, Fs=1kHz;
    /// Temperature sensor: bandwidth=20Hz, delay=8.3ms.
    _4 = 4,
    /// Accelerometer: bandwidth=10.2Hz, delay=16.83ms;
    /// Gyroscope: bandwidth=10Hz, delay=17.85ms, Fs=1kHz;
    /// Temperature sensor: bandwidth=10Hz, delay=13.4ms.
    _5 = 5,
    /// Accelerometer: bandwidth=5.05Hz, delay=32.48ms;
    /// Gyroscope: bandwidth=5Hz, delay=33.48ms, Fs=1kHz;
    /// Temperature sensor: bandwidth=5Hz, delay=18.6ms.
    _6 = 6,
    /// Accelerometer: bandwidth=420Hz, delay=1.38ms;
    /// Gyroscope: bandwidth=3600Hz, delay=0.17ms, Fs=8kHz;
    /// Temperature sensor: bandwidth=4000Hz, delay=0.04ms.
    _7 = 7,
}
impl Default for Dlpf {
    fn default() -> Self {
        Dlpf::_0
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Gyroscope reading full scale configuration; default: +250 dps.
pub enum GyroScale {
    /// +250 dps
    _250DPS = 0,
    /// +500 dps
    _500DPS,
    /// +1000 dps
    _1000DPS,
    /// +2000 dps
    _2000DPS,
}
impl GyroScale {
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            GyroScale::_250DPS => 250.0 / 32768.0,
            GyroScale::_500DPS => 500.0 / 32768.0,
            GyroScale::_1000DPS => 1000.0 / 32768.0,
            GyroScale::_2000DPS => 2000.0 / 32768.0,
        }
    }
}
impl Default for GyroScale {
    fn default() -> Self {
        GyroScale::_250DPS
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Accelerometer reading full scale configuration, default: +2g.
pub enum AccelScale {
    /// +2g
    _2G = 0,
    /// +4g
    _4G,
    /// +8g
    _8G,
    /// +16g
    _16G,
}
impl AccelScale {
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            AccelScale::_2G => 2.0 / 32768.0,
            AccelScale::_4G => 4.0 / 32768.0,
            AccelScale::_8G => 8.0 / 32768.0,
            AccelScale::_16G => 16.0 / 32768.0,
        }
    }
}
impl Default for AccelScale {
    fn default() -> Self {
        AccelScale::_2G
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Gyroscope reading full scale configuration; default: 0.6 mG per LSB
pub enum MagScale {
    /// 0.6 mG per LSB
    _14BITS = 0,
    /// 0.15 mG per LSB
    _16BITS,
}
impl MagScale {
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            MagScale::_14BITS => 10. * 4912. / 8190.,
            MagScale::_16BITS => 10. * 4912. / 32760.0,
        }
    }
}
impl Default for MagScale {
    fn default() -> Self {
        MagScale::_14BITS
    }
}

/// Configuration of MPU9250
#[derive(Copy, Clone, Debug)]
pub struct MpuConfig<MODE> {
    pub(crate) gyro_scale: Option<GyroScale>,
    pub(crate) accel_scale: Option<AccelScale>,
    pub(crate) mag_scale: Option<MagScale>,
    pub(crate) accel_data_rate: Option<AccelDataRate>,
    pub(crate) gyro_temp_data_rate: Option<GyroTempDataRate>,
    pub(crate) sample_rate_divisor: Option<u8>,
    _mode: PhantomData<MODE>,
}

impl MpuConfig<types::Imu> {
    /// Creates configuration for [`Imu`] driver (accelerometer + gyroscope).
    /// with default [`Accel scale`], [`Gyro scale`], [`Mag scale`],
    /// [`AccelDataRate`], [`GyroTempDataRate`],
    /// and no sample rate divisor.
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    pub fn imu() -> Self {
        MpuConfig {
            gyro_scale: None,
            accel_scale: None,
            mag_scale: None,
            accel_data_rate: None,
            gyro_temp_data_rate: None,
            sample_rate_divisor: None,
            _mode: PhantomData
        }

    }
}

impl MpuConfig<types::Marg> {
    /// Creates configuration for [`Marg`] driver
    /// (accelerometer + gyroscope + magnetometer)
    /// with default [`Accel scale`], [`Gyro scale`], [`Mag scale`],
    /// [`AccelDataRate`], [`GyroTempDataRate`],
    /// and no sample rate divisor.
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    pub fn marg() -> Self {
        MpuConfig {
            gyro_scale: None,
            accel_scale: None,
            mag_scale: None,
            accel_data_rate: None,
            gyro_temp_data_rate: None,
            sample_rate_divisor: None,
            _mode: PhantomData
        }

    }
}

impl<MODE> MpuConfig<MODE> {
    /// Sets gyroscope full reading scale ([`Gyro scale`]).
    ///
    /// [`Gyro scale`]: ./enum.GyroScale.html
    pub fn gyro_scale(&mut self, scale: GyroScale) -> &mut Self {
        self.gyro_scale = Some(scale);
        self
    }

    /// Sets accelerometer full reading scale ([`Accel scale`]).
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    pub fn accel_scale(&mut self, scale: AccelScale) -> &mut Self {
        self.accel_scale = Some(scale);
        self
    }

    /// Sets accelerometer data rate config ([`AccelDataRate`]).
    ///
    /// [`AccelDataRate`]: ./conf/enum.AccelDataRate.html
    pub fn accel_data_rate(&mut self, data_rate: AccelDataRate)
                           -> &mut Self {
        self.accel_data_rate = Some(data_rate);
        self
    }

    /// Sets gyroscope and temperatures data rate config
    /// ([`GyroTempDataRate`]).
    ///
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    pub fn gyro_temp_data_rate(&mut self,
                               data_rate: GyroTempDataRate)
                               -> &mut Self {
        self.gyro_temp_data_rate = Some(data_rate);
        self
    }

    /// Sets sample rate divisor.
    /// Sample rate divisor divides the internal sample rate to generate
    /// the sample rate that controls sensor data output rate, FIFO sample
    /// rate. NOTE: This register is only effective when dlpf mode used for
    /// GyroTempDataRate see [`GyroTempDataRate`].
    /// SampleRate = InternalSampleRate / (1 + SMPLRT_DIV).
    ///
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    pub fn sample_rate_divisor(&mut self, smplrt_div: u8) -> &mut Self {
        self.sample_rate_divisor = Some(smplrt_div);
        self
    }
}

impl MpuConfig<types::Marg> {
    /// Sets magnetrometer full reading scale ([`MagScale`])
    ///
    /// [`Mag scale`]: ./enum.MagScale.html
    pub fn mag_scale(&mut self, scale: MagScale) -> &mut Self {
        self.mag_scale = Some(scale);
        self
    }
}
