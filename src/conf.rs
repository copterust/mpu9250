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
            AccelDataRate::FChoice0 => 0b0000_1000,
            // 0x40 for 1024 bit fifo
            AccelDataRate::DlpfConf(dlpf) => 0b0100_0000 | (*dlpf as u8),
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
    _500DPS = 1,
    /// +1000 dps
    _1000DPS = 2,
    /// +2000 dps
    _2000DPS = 3,
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
    _4G = 1,
    /// +8g
    _8G = 2,
    /// +16g
    _16G = 3,
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

#[derive(Copy, Clone, Debug)]
/// Dmp data output rate, default 100Hz
// rate = 200 / div - 1
pub enum DmpRate {
    /// Update data at 200Hz
    _200Hz = 0,
    /// Update data at 100Hz
    _100Hz = 1,
    /// Update data at 50Hz
    _50Hz = 3,
    /// Update data at 40Hz
    _40Hz = 4,
    /// Update data at 20Hz
    _25Hz = 8,
    /// Update data at 25Hz
    _20Hz = 9,
    /// Update data at 10Hz
    _10Hz = 19,
    /// Update data at 8Hz
    _8Hz = 24,
    /// Update data at 5Hz
    _5Hz = 39,
    /// Update data at 4Hz
    _4Hz = 49,
}
impl Default for DmpRate {
    fn default() -> Self {
        DmpRate::_100Hz
    }
}

#[derive(Copy, Clone, Debug)]
/// DMP base orienatation, default Z axe pointing up
pub enum Orientation {
    /// Z axe pointing up
    ZUp = 0x088,
    /// Z axe pointing down
    ZDown = 0x18c,
    /// X axe pointing up
    XUp = 0x00e,
    /// X axe pointing down
    XDown = 0x10a,
    /// Y axe pointing up
    YUp = 0x070,
    /// Y axe pointing down
    YDown = 0x150,
    /// X pointing forward
    XForward = 0x085,
    /// X pointing forward
    XBackward = 0x0a1,
}
impl Orientation {
    pub(crate) fn gyro_axes(&self) -> [u8; 3] {
        const AXES: [u8; 3] = [0x4c, 0xcd, 0x6c];
        [AXES[*self as usize & 3],
         AXES[(*self as usize >> 3) & 3],
         AXES[(*self as usize >> 6) & 3]]
    }

    pub(crate) fn accel_axes(&self) -> [u8; 3] {
        const AXES: [u8; 3] = [0x0c, 0xc9, 0x2c];
        [AXES[*self as usize & 3],
         AXES[(*self as usize >> 3) & 3],
         AXES[(*self as usize >> 6) & 3]]
    }

    pub(crate) fn gyro_signs(&self) -> [u8; 3] {
        let mut sign: [u8; 3] = [0x36, 0x56, 0x76];
        if *self as u16 & 0x002 != 0 {
            sign[0] |= 1;
        }
        if *self as u16 & 0x020 != 0 {
            sign[1] |= 1;
        }
        if *self as u16 & 0x100 != 0 {
            sign[2] |= 1;
        }
        sign
    }

    pub(crate) fn accel_signs(&self) -> [u8; 3] {
        let mut sign: [u8; 3] = [0x26, 0x46, 0x66];
        if *self as u16 & 0x002 != 0 {
            sign[0] |= 1;
        }
        if *self as u16 & 0x020 != 0 {
            sign[1] |= 1;
        }
        if *self as u16 & 0x100 != 0 {
            sign[2] |= 1;
        }
        sign
    }
}
impl Default for Orientation {
    fn default() -> Self {
        Orientation::ZUp
    }
}

#[derive(Copy, Clone, Debug)]
/// Dmp features to enable, default with raw gyro and accel and 6 axes
/// quaternion
pub struct DmpFeatures {
    /// DMP output raw gyro measurement
    pub raw_gyro: bool,
    /// DMP output raw accel measurement
    pub raw_accel: bool,
    /// DMP output tap motions detection
    pub tap: bool,
    /// DMP output orientation motions detection
    pub android_orient: bool,
    /// DMP output quaternion based on accelrometer and gyroscope
    pub quat6: bool,
    /// DMP output quaternion based on gyroscope only
    pub quat: bool,
}
impl DmpFeatures {
    pub(crate) fn packet_size(self) -> usize {
        let mut size = 0;
        if self.raw_gyro {
            size += 6
        }
        if self.raw_accel {
            size += 6
        }
        if self.quat | self.quat6 {
            size += 16
        }
        if self.android_orient | self.tap {
            size += 4
        }
        size
    }
}
impl Default for DmpFeatures {
    fn default() -> Self {
        DmpFeatures { raw_gyro: true,
                      raw_accel: true,
                      tap: false,
                      android_orient: false,
                      quat6: true,
                      quat: false }
    }
}

#[derive(Copy, Clone, Debug)]
/// Dmp configuration
pub struct DmpConfiguration {
    pub(crate) orientation: Orientation,
    pub(crate) features: DmpFeatures,
    pub(crate) rate: DmpRate,
}
impl Default for DmpConfiguration {
    fn default() -> Self {
        DmpConfiguration { orientation: Default::default(),
                           features: Default::default(),
                           rate: Default::default() }
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
    pub(crate) dmp_configuration: Option<DmpConfiguration>,
    _mode: PhantomData<MODE>,
}

impl MpuConfig<types::Imu> {
    /// Creates configuration for [`Imu`] driver (accelerometer + gyroscope).
    /// with default [`Accel scale`], [`Gyro scale`], [`Mag scale`],
    /// [`AccelDataRate`], [`GyroTempDataRate`],
    /// and no sample rate divisor.
    ///
    /// [`Imu`]: ./struct.Imu.html
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    pub fn imu() -> Self {
        MpuConfig { gyro_scale: None,
                    accel_scale: None,
                    mag_scale: None,
                    accel_data_rate: None,
                    gyro_temp_data_rate: None,
                    sample_rate_divisor: None,
                    dmp_configuration: None,
                    _mode: PhantomData }
    }
}

impl MpuConfig<types::Marg> {
    /// Creates configuration for [`Marg`] driver
    /// (accelerometer + gyroscope + magnetometer)
    /// with default [`Accel scale`], [`Gyro scale`], [`Mag scale`],
    /// [`AccelDataRate`], [`GyroTempDataRate`],
    /// and no sample rate divisor.
    ///
    /// [`Marg`]: ./struct.Marg.html
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    pub fn marg() -> Self {
        MpuConfig { gyro_scale: None,
                    accel_scale: None,
                    mag_scale: None,
                    accel_data_rate: None,
                    gyro_temp_data_rate: None,
                    sample_rate_divisor: None,
                    dmp_configuration: None,
                    _mode: PhantomData }
    }
}

impl MpuConfig<types::Dmp> {
    /// Creates configuration for [`Dmp`] driver
    /// (accelerometer + gyroscope + dmp)
    /// with recommended [`Accel scale`], [`Gyro scale`], [`Mag scale`],
    /// [`AccelDataRate`], [`GyroTempDataRate`], [`Dmp rate`]
    /// and a sample rate of 200Hz. The default [`features`] and the
    /// default [`orientation`] are used
    ///
    /// [`Dmp`]: ./struct.Dmp.html
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    /// [`GyroTempDataRate`]: ./enum.GyroTempDataRate.html
    /// [`Dmp rate`]: ./enum.DmpRate.html
    /// [`features`]: ./struct.DmpFeatures.html
    /// [`orientation`]: ./enum.Orientation.html
    pub fn dmp() -> Self {
        MpuConfig { gyro_scale: Some(GyroScale::_2000DPS),
                    accel_scale: Some(AccelScale::_8G),
                    mag_scale: None,
                    accel_data_rate: Some(AccelDataRate::DlpfConf(Dlpf::_1)),
                    gyro_temp_data_rate:
                        Some(GyroTempDataRate::DlpfConf(Dlpf::_1)),
                    sample_rate_divisor: Some(4),
                    dmp_configuration: None,
                    _mode: PhantomData }
    }

    /// Sets dmp data output rate [`Dmp rate`]
    ///
    /// [`Dmp rate`]: ./enum.DmpRate.html
    pub fn dmp_rate(&mut self, rate: DmpRate) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.rate = rate,
            None => {
                self.dmp_configuration =
                    Some(DmpConfiguration { rate,
                                            ..Default::default() })
            },
        }
        self
    }

    /// Sets dmp base [`orientation`]
    ///
    /// [`orientation`]: ./enum.Orientation.html
    pub fn dmp_orientation(&mut self, orientation: Orientation) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.orientation = orientation,
            None => {
                self.dmp_configuration =
                    Some(DmpConfiguration { orientation,
                                            ..Default::default() })
            },
        }
        self
    }

    /// Selects [`dmp features`] to send raw gyro measurement
    ///
    /// [`dmp features`]: ./struct.DmpFeatures.html
    pub fn dmp_features_raw_gyro(&mut self, feature: bool) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.features.raw_gyro = feature,
            None => self.dmp_configuration =
                Some(DmpConfiguration { features:
                                            DmpFeatures { raw_gyro: feature,
                                                          ..Default::default() },
                                        ..Default::default() }),
        }
        self
    }

    /// Selects [`dmp features`] to send raw accel measurement
    ///
    /// [`dmp features`]: ./struct.DmpFeatures.html
    pub fn dmp_features_raw_accel(&mut self, feature: bool) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.features.raw_accel = feature,
            None => self.dmp_configuration =
                Some(DmpConfiguration { features:
                                            DmpFeatures { raw_accel: feature,
                                                          ..Default::default() },
                                        ..Default::default() }),
        }
        self
    }

    /// Selects [`dmp features`] to detect tap
    ///
    /// [`dmp features`]: ./struct.DmpFeatures.html
    pub fn dmp_features_tap(&mut self, feature: bool) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.features.raw_gyro = feature,
            None => self.dmp_configuration =
                Some(DmpConfiguration { features:
                                            DmpFeatures { tap: feature,
                                                          ..Default::default() },
                                        ..Default::default() }),
        }
        self
    }

    /// Selects [`dmp features`] to send 3 axes quaternion
    ///
    /// [`dmp features`]: ./struct.DmpFeatures.html
    pub fn dmp_features_quat(&mut self, feature: bool) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.features.raw_gyro = feature,
            None => self.dmp_configuration =
                Some(DmpConfiguration { features:
                                            DmpFeatures { quat: feature,
                                                          ..Default::default() },
                                        ..Default::default() }),
        }
        self
    }

    /// Selects [`dmp features`] to send 6 axes quaternion
    ///
    /// [`dmp features`]: ./struct.DmpFeatures.html
    pub fn dmp_features_quat6(&mut self, feature: bool) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.features.raw_gyro = feature,
            None => self.dmp_configuration =
                Some(DmpConfiguration { features:
                                            DmpFeatures { quat6: feature,
                                                          ..Default::default() },
                                        ..Default::default() }),
        }
        self
    }

    /// Selects [`dmp features`] to detect orientation changement
    ///
    /// [`dmp features`]: ./struct.DmpFeatures.html
    pub fn dmp_features_orientation(&mut self, feature: bool) -> &mut Self {
        match self.dmp_configuration {
            Some(mut x) => x.features.raw_gyro = feature,
            None => self.dmp_configuration =
                Some(DmpConfiguration { features:
                                            DmpFeatures { android_orient:
                                                              feature,
                                                          ..Default::default() },
                                        ..Default::default() }),
        }
        self
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
    pub fn accel_data_rate(&mut self, data_rate: AccelDataRate) -> &mut Self {
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

bitflags! {
    /// Enable interrupt for:
    #[allow(non_camel_case_types)]
    pub struct InterruptEnable: u8 {
        /// Wake up on motion
        const WOM_EN = 0b0100_0000;
        /// FIFO overflow
        const FIFO_OVERFLOW_EN = 0b0001_0000;
        /// fsync
        const FSYNC_INT_EN = 0b0000_1000;
        /// raw sensor data ready
        const RAW_RDY_EN = 0b0000_0001;
    }
}

bitflags! {
    /// Interrupt configuration
    /// Defaults:
    /// active high, push-pull, 50 us pulse, cleared only by reading INT_STATUS
    #[allow(non_camel_case_types)]
    pub struct InterruptConfig: u8 {
        /// Sets logic level for INT pin is active low (high if not set)
        const ACL = 0b1000_0000;
        /// INT pin is configured as open drain (push pull if not set)
        const OPEN = 0b0100_0000;
        /// INT pin level held untilinterrupt status is cleared (cleared within 50us if not set)
        const LATCH_INT_EN = 0b0010_0000;
        /// Interrupt status is cleared if any read operation is performed (cleared only by reading INT_STATUS if not set)
        const INT_ANYRD_CLEAR = 0b0001_0000;
        /// The logic level for the FSYNC pin as an interrupt is active low (active high if not set)
        const ACTL_FSYNC = 0b0000_1000;
        /// This enables the FSYNC pin to be used as an interrupt.
        /// A transition to the active level described by the ACTL_FSYNC bit
        /// will cause an interrupt.  The status of the interrupt is read in
        /// the I2C Master Status register PASS_THROUGH bit (disabled if not set)
        const FSYNC_INT_MODE_EN = 0b0000_0100;
        /// When asserted, the i2c_master interface pins(ES_CL and ES_DA) will
        /// go into ‘bypass mode’ when the i2c master interface is disabled.
        /// The pins will float high due to the internal pull-up if not enabled
        /// and the i2c master interface is disabled
        const BYPASS_EN = 0b0000_0010;
    }
}
