//! no_std driver for the MPU9250 & onboard AK8963 (accelerometer + gyroscope +
//! magnetometer IMU)
//!
//! # Connections
//!
//! - NCS
//! - SCL = SCK
//! - SDA = SDI = MOSI
//! - AD0 = SDO = MISO
//!
//! # Usage
//!
//! Use embedded-hal implementation to get SPI, NCS, and delay, then create mpu
//! handle
//!
//! ```
//! // to create sensor with mag support and default configuration:
//! let mut mpu = Mpu9250::marg_default(spi, ncs, &mut delay)?;
//! // to create sensor without mag support and default configuration:
//! let mut mpu = Mpu9250::imu_default(spi, ncs, &mut delay)?;
//! // to get all supported measurements:
//! let all = mpu.all()?;
//! println!("{:?}", all);
//! // One can also use conf module to supply configuration:
//! let mut mpu =
//!     Mpu9250::marg(spi,
//!                   ncs,
//!                   &mut delay,
//!                   MpuConfig::marg().mag_scale(conf::MagScale::_14BITS))?;
//! ```
//!
//! More examples (for stm32) in [Proving ground] repo.
//!
//! # References
//!
//! - [Product specification][2]
//!
//! [1]: https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//!
//! - [Register map][2]
//!
//! [2]: https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
//!
//! - [AK8963 specification][3]
//!
//! [3]: https://www.akm.com/akm/en/file/datasheet/AK8963C.pdf
//!
//! - [Proving ground][4]
//!
//! [4]: https://github.com/copterust/proving-ground

#![deny(missing_docs)]
#![no_std]

#[macro_use]
extern crate bitflags;
extern crate cast;
extern crate embedded_hal as hal;

mod ak8963;
mod conf;
mod device;
mod types;

#[cfg(feature = "dmp")]
mod dmp_firmware;

#[cfg(feature = "dmp")]
pub use dmp_firmware::DMP_FIRMWARE;

use ak8963::AK8963;

use core::marker::PhantomData;

use cast::{f32, i32, u16};

use hal::blocking::delay::DelayMs;
use hal::spi::{Mode, Phase, Polarity};

pub use conf::*;
pub use types::*;

#[doc(hidden)]
pub use device::Releasable;
pub use device::{
    Device, I2CError, I2cDevice, NineDOFDevice, SpiDevice, SpiError
};

/// Suported MPUx devices
pub enum MpuXDevice {
    /// MPU 9250
    MPU9250 = 0x71,
    /// MPU 9255
    MPU9255 = 0x73,
    /// MPU 6500
    MPU6500 = 0x70,
}

impl MpuXDevice {
    fn imu_supported(b: u8) -> bool {
        b == (MpuXDevice::MPU9250 as u8)
        || b == (MpuXDevice::MPU9255 as u8)
        || b == (MpuXDevice::MPU6500 as u8)
    }

    fn marg_supported(b: u8) -> bool {
        b == (MpuXDevice::MPU9250 as u8) || b == (MpuXDevice::MPU9255 as u8)
    }
}

/// MPU9250 driver
pub struct Mpu9250<DEV, MODE> {
    // connections
    dev: DEV,
    // data; factory defaults.
    mag_sensitivity_adjustments: [f32; 3],
    raw_mag_sensitivity_adjustments: [u8; 3],
    // configuration
    gyro_scale: GyroScale,
    accel_scale: AccelScale,
    mag_scale: MagScale,
    gyro_temp_data_rate: GyroTempDataRate,
    accel_data_rate: AccelDataRate,
    sample_rate_divisor: Option<u8>,
    dmp_configuration: Option<DmpConfiguration>,
    packet_size: usize,
    // mode
    _mode: PhantomData<MODE>,
}

/// MPU Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// WHO_AM_I returned invalid value (returned value is argument).
    InvalidDevice(u8),
    /// Mode not supported by device (WHO_AM_I is argument)
    ModeNotSupported(u8),
    /// Underlying bus error.
    BusError(E),
    /// Calibration error (not enough data gathered)
    CalibrationError,
    /// Reinitialization error (user provided function was unable to re-init
    /// device)
    ReInitError,
    /// DMP read internal memory error
    DmpRead,
    /// DMP write internal memory error
    DmpWrite,
    /// DMP firmware loading error
    DmpFirmware,
    /// DMP data are not ready yet
    DmpDataNotReady,
    /// DMP data do not correspond to the expected format
    DmpDataInvalid,
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
const MMODE: u8 = 0x06;

/// G constant
pub const G: f32 = 9.807;
const PI_180: f32 = core::f32::consts::PI / 180.;
const TEMP_SENSITIVITY: f32 = 333.87;
const TEMP_DIFF: f32 = 21.0;
const TEMP_ROOM_OFFSET: f32 = 0.0;

/// SPI device definitions
#[cfg(not(feature = "i2c"))]
mod spi_defs {
    use super::*;
    use hal::blocking::spi;
    use hal::digital::v2::OutputPin;

    // SPI device, 6DOF
    impl<E, SPI, NCS> Mpu9250<SpiDevice<SPI, NCS>, Imu>
        where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
              NCS: OutputPin
    {
        /// Creates a new [`Imu`] driver from a SPI peripheral and a NCS pin
        /// with default configuration.
        pub fn imu_default<D>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            Self::imu(spi, ncs, delay, &mut MpuConfig::imu())
        }

        /// Creates a new Imu driver from a SPI peripheral and a NCS pin with
        /// provided configuration [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn imu<D>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D,
            config: &mut MpuConfig<Imu>)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = SpiDevice::new(spi, ncs);
            Self::new_imu(dev, delay, config)
        }

        /// Creates a new Imu driver from a SPI peripheral and a NCS pin with
        /// provided configuration [`Config`]. Reinit function can be used to
        /// re-initialize SPI bus. Usecase: change SPI speed for faster data
        /// transfer:
        /// "Communication with all registers of the device is
        ///    performed using either I2C at 400kHz or SPI at 1M Hz.
        ///    For applications requiring faster communications, the sensor and
        ///    interrupt registers may be read using SPI at 20MHz."
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn imu_with_reinit<D, F>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D,
            config: &mut MpuConfig<Imu>,
            reinit_fn: F)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>,
                  F: FnOnce(SPI, NCS) -> Option<(SPI, NCS)>
        {
            let dev = SpiDevice::new(spi, ncs);
            let mpu = Self::new_imu(dev, delay, config)?;
            mpu.reinit_spi_device(reinit_fn)
        }
    }

    // SPI device, 9 DOF
    impl<E, SPI, NCS> Mpu9250<SpiDevice<SPI, NCS>, Marg>
        where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
              NCS: OutputPin
    {
        /// Creates a new [`Marg`] driver from a SPI peripheral and a NCS pin
        /// with default [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg_default<D>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            Mpu9250::marg(spi, ncs, delay, &mut MpuConfig::marg())
        }

        /// Creates a new MARG driver from a SPI peripheral and a NCS pin
        /// with provided configuration [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg<D>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D,
            config: &mut MpuConfig<Marg>)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = SpiDevice::new(spi, ncs);
            Self::new_marg(dev, delay, config)
        }

        /// Creates a new MARG driver from a SPI peripheral and a NCS pin
        /// with provided configuration [`Config`]. Reinit function can be used
        /// to re-initialize SPI bus. Usecase: change SPI speed for
        /// faster data transfer:
        /// "Communication with all registers of the device is
        ///    performed using either I2C at 400kHz or SPI at 1M Hz.
        ///    For applications requiring faster communications, the sensor and
        ///    interrupt registers may be read using SPI at 20MHz."
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg_with_reinit<D, F>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D,
            config: &mut MpuConfig<Marg>,
            reinit_fn: F)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>,
                  F: FnOnce(SPI, NCS) -> Option<(SPI, NCS)>
        {
            let dev = SpiDevice::new(spi, ncs);
            let mpu = Self::new_marg(dev, delay, config)?;
            mpu.reinit_spi_device(reinit_fn)
        }
    }

    #[cfg(feature = "dmp")]
    impl<E, SPI, NCS> Mpu9250<SpiDevice<SPI, NCS>, Dmp>
        where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
              NCS: OutputPin
    {
        /// Create a new dmp device with default configuration
        pub fn dmp_default<D>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D,
            firmware: &[u8])
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = SpiDevice::new(spi, ncs);
            Self::new_dmp(dev, delay, &mut MpuConfig::dmp(), firmware)
        }

        /// Create a new dmp device
        pub fn dmp<D>(
            spi: SPI,
            ncs: NCS,
            delay: &mut D,
            config: &mut MpuConfig<Dmp>,
            firmware: &[u8])
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = SpiDevice::new(spi, ncs);
            Self::new_dmp(dev, delay, config, firmware)
        }
    }

    // SPI device, any mode
    impl<E, SPI, NCS, MODE> Mpu9250<SpiDevice<SPI, NCS>, MODE>
        where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
              NCS: OutputPin
    {
        /// Destroys the driver recovering the SPI peripheral and the NCS pin
        pub fn release(self) -> (SPI, NCS) {
            self.dev.release()
        }

        fn reinit_spi_device<F>(
            self,
            reinit_fn: F)
            -> Result<Self,
                      Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>
            where F: FnOnce(SPI, NCS) -> Option<(SPI, NCS)>
        {
            self.reset_device(|spidev| {
                    let (cspi, cncs) = spidev.release();
                    reinit_fn(cspi, cncs).map(|(nspi, nncs)| {
                                             SpiDevice::new(nspi, nncs)
                                         })
                })
        }
    }
}

#[cfg(not(feature = "i2c"))]
pub use spi_defs::*;

#[cfg(feature = "i2c")]
mod i2c_defs {
    use super::*;
    use hal::blocking::i2c;

    impl<E, I2C> Mpu9250<I2cDevice<I2C>, Imu>
        where I2C: i2c::Read<Error = E>
                  + i2c::Write<Error = E>
                  + i2c::WriteRead<Error = E>
    {
        /// Creates a new [`Imu`] driver from an I2C peripheral
        /// with default configuration.
        pub fn imu_default<D>(
            i2c: I2C,
            delay: &mut D)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            Mpu9250::imu(i2c, delay, &mut MpuConfig::imu())
        }

        /// Creates a new Imu driver from an I2C peripheral with the
        /// provided configuration [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn imu<D>(
            i2c: I2C,
            delay: &mut D,
            config: &mut MpuConfig<Imu>)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = I2cDevice::new(i2c);
            Mpu9250::new_imu(dev, delay, config)
        }

        /// Creates a new IMU driver from an I2C peripheral
        /// with provided configuration [`Config`]. Reinit function can be used
        /// to re-initialize I2C bus. Usecase: change I2C speed for
        /// faster data transfer.
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn imu_with_reinit<D, F>(
            i2c: I2C,
            delay: &mut D,
            config: &mut MpuConfig<Imu>,
            reinit_fn: F)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>,
                  F: FnOnce(I2C) -> Option<I2C>
        {
            let dev = I2cDevice::new(i2c);
            let mpu = Self::new_imu(dev, delay, config)?;
            mpu.reinit_i2c_device(reinit_fn)
        }
    }

    impl<E, I2C> Mpu9250<I2cDevice<I2C>, Marg>
        where I2C: i2c::Read<Error = E>
                  + i2c::Write<Error = E>
                  + i2c::WriteRead<Error = E>
    {
        /// Creates a new [`Marg`] driver from an I2C peripheral with
        /// default [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg_default<D>(
            i2c: I2C,
            delay: &mut D)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            Mpu9250::marg(i2c, delay, &mut MpuConfig::marg())
        }

        /// Creates a new MARG driver from an I2C peripheral
        /// with provided configuration [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg<D>(
            i2c: I2C,
            delay: &mut D,
            config: &mut MpuConfig<Marg>)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = I2cDevice::new(i2c);
            Self::new_marg(dev, delay, config)
        }

        /// Creates a new MARG driver from an I2C peripheral
        /// with provided configuration [`Config`]. Reinit function can be used
        /// to re-initialize I2C bus. Usecase: change I2C speed for
        /// faster data transfer.
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg_with_reinit<D, F>(
            i2c: I2C,
            delay: &mut D,
            config: &mut MpuConfig<Marg>,
            reinit_fn: F)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>,
                  F: FnOnce(I2C) -> Option<I2C>
        {
            let dev = I2cDevice::new(i2c);
            let mpu = Self::new_marg(dev, delay, config)?;
            mpu.reinit_i2c_device(reinit_fn)
        }
    }

    #[cfg(feature = "dmp")]
    impl<E, I2C> Mpu9250<I2cDevice<I2C>, Dmp>
        where I2C: i2c::Read<Error = E>
                  + i2c::Write<Error = E>
                  + i2c::WriteRead<Error = E>
    {
        /// Creates a new DMP driver from an I2C peripheral with default
        /// configuration
        pub fn dmp_default<D>(
            i2c: I2C,
            delay: &mut D,
            firmware: &[u8])
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = I2cDevice::new(i2c);
            Self::new_dmp(dev, delay, &mut MpuConfig::dmp(), firmware)
        }

        /// Creates a new DMP driver from an I2C peripheral
        pub fn dmp<D>(
            i2c: I2C,
            delay: &mut D,
            config: &mut MpuConfig<Dmp>,
            firmware: &[u8])
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where D: DelayMs<u8>
        {
            let dev = I2cDevice::new(i2c);
            Self::new_dmp(dev, delay, config, firmware)
        }
    }

    // I2C device, any mode
    impl<E, I2C, MODE> Mpu9250<I2cDevice<I2C>, MODE>
        where I2C: i2c::Read<Error = E>
                  + i2c::Write<Error = E>
                  + i2c::WriteRead<Error = E>
    {
        /// Destroys the driver, recovering the I2C peripheral
        pub fn release(self) -> I2C {
            self.dev.release()
        }

        fn reinit_i2c_device<F>(
            self,
            reinit_fn: F)
            -> Result<Self, Error<<I2cDevice<I2C> as device::Device>::Error>>
            where F: FnOnce(I2C) -> Option<I2C>
        {
            self.reset_device(|i2cdev| {
                    let i2c = i2cdev.release();
                    reinit_fn(i2c).map(|i2c| I2cDevice::new(i2c))
                })
        }
    }
}

#[cfg(feature = "i2c")]
pub use i2c_defs::*;

// Any device, 6DOF
impl<E, DEV> Mpu9250<DEV, Imu> where DEV: Device<Error = E>
{
    /// Private constructor that creates an IMU-based MPU with the
    /// specified device.
    fn new_imu<D>(dev: DEV,
                  delay: &mut D,
                  config: &mut MpuConfig<Imu>)
                  -> Result<Self, Error<E>>
        where D: DelayMs<u8>
    {
        let mut mpu9250 =
            Mpu9250 { dev,
                      raw_mag_sensitivity_adjustments: [0; 3],
                      mag_sensitivity_adjustments: [0.0; 3],
                      gyro_scale: config.gyro_scale.unwrap_or_default(),
                      accel_scale: config.accel_scale.unwrap_or_default(),
                      mag_scale: MagScale::default(),
                      accel_data_rate: config.accel_data_rate
                                             .unwrap_or_default(),
                      gyro_temp_data_rate: config.gyro_temp_data_rate
                                                 .unwrap_or_default(),
                      sample_rate_divisor: config.sample_rate_divisor,
                      dmp_configuration: config.dmp_configuration,
                      packet_size: 0,
                      _mode: PhantomData };
        mpu9250.init_mpu(delay)?;
        let wai = mpu9250.who_am_i()?;
        if MpuXDevice::imu_supported(wai) {
            Ok(mpu9250)
        } else {
            Err(Error::InvalidDevice(wai))
        }
    }

    /// Configures device using provided [`MpuConfig`].
    pub fn config(&mut self, config: &mut MpuConfig<Imu>) -> Result<(), E> {
        transpose(config.gyro_scale.map(|v| self.gyro_scale(v)))?;
        transpose(config.accel_scale.map(|v| self.accel_scale(v)))?;
        transpose(config.accel_data_rate.map(|v| self.accel_data_rate(v)))?;
        transpose(config.gyro_temp_data_rate
                        .map(|v| self.gyro_temp_data_rate(v)))?;
        transpose(config.sample_rate_divisor
                        .map(|v| self.sample_rate_divisor(v)))?;

        Ok(())
    }

    /// Reads and returns raw unscaled Accelerometer + Gyroscope + Thermometer
    /// measurements (LSB).
    pub fn unscaled_all<T>(&mut self) -> Result<UnscaledImuMeasurements<T>, E>
        where T: From<[i16; 3]>
    {
        let buffer = &mut [0; 15];
        self.dev.read_many(Register::ACCEL_XOUT_H, &mut buffer[..])?;
        let accel = self.to_vector(buffer, 0).into();
        let temp = ((u16(buffer[7]) << 8) | u16(buffer[8])) as i16;
        let gyro = self.to_vector(buffer, 8).into();

        Ok(UnscaledImuMeasurements { accel,
                                     gyro,
                                     temp })
    }

    /// Reads and returns Accelerometer + Gyroscope + Thermometer
    /// measurements scaled and converted to respective units.
    pub fn all<T>(&mut self) -> Result<ImuMeasurements<T>, E>
        where T: From<[f32; 3]>
    {
        let buffer = &mut [0; 15];
        self.dev.read_many(Register::ACCEL_XOUT_H, &mut buffer[..])?;

        let accel = self.scale_accel(buffer, 0).into();
        let temp = self.scale_temp(buffer, 6);
        let gyro = self.scale_gyro(buffer, 8).into();

        Ok(ImuMeasurements { accel,
                             gyro,
                             temp })
    }

    /// Calculates the average of the at-rest readings of accelerometer and
    /// gyroscope and then loads the resulting biases into gyro
    /// offset registers. Retunrs either Ok with accelerometer biases, or
    /// Err(Error), where Error::CalibrationError means soft error, and user
    /// can proceed on their own risk.
    ///
    /// Accelerometer biases should be processed separately.
    ///
    /// NOTE: MPU is able to store accelerometer biases, to apply them
    ///       automatically, but at this moment it does not work.
    pub fn calibrate_at_rest<D, T>(&mut self,
                                   delay: &mut D)
                                   -> Result<T, Error<E>>
        where D: DelayMs<u8>,
              T: From<[f32; 3]>
    {
        Ok(self._calibrate_at_rest(delay)?.into())
    }
}

// Any device, 9DOF
impl<E, DEV> Mpu9250<DEV, Marg>
    where DEV: Device<Error = E> + AK8963<Error = E> + NineDOFDevice
{
    // Private constructor that creates a MARG-based MPU with
    // the specificed device.
    fn new_marg<D>(dev: DEV,
                   delay: &mut D,
                   config: &mut MpuConfig<Marg>)
                   -> Result<Self, Error<E>>
        where D: DelayMs<u8>
    {
        let mut mpu9250 =
            Mpu9250 { dev,
                      raw_mag_sensitivity_adjustments: [0; 3],
                      mag_sensitivity_adjustments: [0.0; 3],
                      gyro_scale: config.gyro_scale.unwrap_or_default(),
                      accel_scale: config.accel_scale.unwrap_or_default(),
                      mag_scale: config.mag_scale.unwrap_or_default(),
                      accel_data_rate: config.accel_data_rate
                                             .unwrap_or_default(),
                      gyro_temp_data_rate: config.gyro_temp_data_rate
                                                 .unwrap_or_default(),
                      sample_rate_divisor: config.sample_rate_divisor,
                      dmp_configuration: config.dmp_configuration,
                      packet_size: 0,
                      _mode: PhantomData };
        mpu9250.init_mpu(delay)?;
        let wai = mpu9250.who_am_i()?;
        if MpuXDevice::marg_supported(wai) {
            mpu9250.init_ak8963(delay)?;
            mpu9250.check_ak8963_who_am_i()?;
            Ok(mpu9250)
        } else if wai == MpuXDevice::MPU6500 as u8 {
            Err(Error::ModeNotSupported(wai))
        } else {
            Err(Error::InvalidDevice(wai))
        }
    }

    /// Calculates the average of the at-rest readings of accelerometer and
    /// gyroscope and then loads the resulting biases into gyro
    /// offset registers. Retunrs either Ok with accelerometer biases, or
    /// Err(Error), where Error::CalibrationError means soft error, and user
    /// can proceed on their own risk.
    ///
    /// Accelerometer biases should be processed separately.
    ///
    /// NOTE: MPU is able to store accelerometer biases, to apply them
    ///       automatically, but at this moment it does not work.
    pub fn calibrate_at_rest<D, T>(&mut self,
                                   delay: &mut D)
                                   -> Result<T, Error<E>>
        where D: DelayMs<u8>,
              T: From<[f32; 3]>
    {
        let accel_biases = self._calibrate_at_rest(delay)?;
        self.init_ak8963(delay)?;
        Ok(accel_biases.into())
    }

    fn init_ak8963<D>(&mut self, delay: &mut D) -> Result<(), E>
        where D: DelayMs<u8>
    {
        AK8963::init(&mut self.dev, delay)?;
        delay.delay_ms(10);
        // First extract the factory calibration for each magnetometer axis
        AK8963::write(&mut self.dev, ak8963::Register::CNTL, 0x00)?;
        delay.delay_ms(10);

        AK8963::write(&mut self.dev, ak8963::Register::CNTL, 0x0F)?;
        delay.delay_ms(20);
        let mag_x_bias = AK8963::read(&mut self.dev, ak8963::Register::ASAX)?;
        let mag_y_bias = AK8963::read(&mut self.dev, ak8963::Register::ASAY)?;
        let mag_z_bias = AK8963::read(&mut self.dev, ak8963::Register::ASAZ)?;
        // Return x-axis sensitivity adjustment values, etc.
        self.raw_mag_sensitivity_adjustments =
            [mag_x_bias, mag_y_bias, mag_z_bias];
        self.mag_sensitivity_adjustments = [f32(mag_x_bias - 128) / 256. + 1.,
                                            f32(mag_y_bias - 128) / 256. + 1.,
                                            f32(mag_z_bias - 128) / 256. + 1.];
        AK8963::write(&mut self.dev, ak8963::Register::CNTL, 0x00)?;
        delay.delay_ms(10);
        // Set magnetometer data resolution and sample ODR
        self._mag_scale()?;
        delay.delay_ms(10);

        AK8963::finalize(&mut self.dev, delay)?;

        Ok(())
    }

    /// Configures device using provided [`MpuConfig`].
    pub fn config(&mut self, config: &mut MpuConfig<Marg>) -> Result<(), E> {
        transpose(config.gyro_scale.map(|v| self.gyro_scale(v)))?;
        transpose(config.accel_scale.map(|v| self.accel_scale(v)))?;
        transpose(config.mag_scale.map(|v| self.mag_scale(v)))?;
        transpose(config.accel_data_rate.map(|v| self.accel_data_rate(v)))?;
        transpose(config.gyro_temp_data_rate
                        .map(|v| self.gyro_temp_data_rate(v)))?;
        transpose(config.sample_rate_divisor
                        .map(|v| self.sample_rate_divisor(v)))?;

        Ok(())
    }

    /// Reads and returns raw unscaled Accelerometer + Gyroscope + Thermometer
    /// + Magnetometer measurements (LSB).
    pub fn unscaled_all<T>(&mut self) -> Result<UnscaledMargMeasurements<T>, E>
        where T: From<[i16; 3]>
    {
        let buffer = &mut [0; 21];
        NineDOFDevice::read_9dof(&mut self.dev,
                                 Register::ACCEL_XOUT_H,
                                 buffer)?;
        let accel = self.to_vector(buffer, 0).into();
        let temp = ((u16(buffer[7]) << 8) | u16(buffer[8])) as i16;
        let gyro = self.to_vector(buffer, 8).into();
        let mag = self.to_vector_inverted(buffer, 14).into();

        Ok(UnscaledMargMeasurements { accel,
                                      gyro,
                                      temp,
                                      mag })
    }

    /// Reads and returns Accelerometer + Gyroscope + Thermometer + Magnetometer
    /// measurements scaled and converted to respective units.
    pub fn all<T>(&mut self) -> Result<MargMeasurements<T>, E>
        where T: From<[f32; 3]>
    {
        let buffer = &mut [0; 21];
        NineDOFDevice::read_9dof(&mut self.dev,
                                 Register::ACCEL_XOUT_H,
                                 buffer)?;

        let accel = self.scale_accel(buffer, 0).into();
        let temp = self.scale_temp(buffer, 6);
        let gyro = self.scale_gyro(buffer, 8).into();
        let mag = self.scale_and_correct_mag(buffer, 14).into();

        Ok(MargMeasurements { accel,
                              gyro,
                              temp,
                              mag })
    }

    fn scale_and_correct_mag(&self, buffer: &[u8], offset: usize) -> [f32; 3] {
        let resolution = self.mag_scale.resolution();
        let raw = self.to_vector_inverted(buffer, offset);

        [raw[0] as f32 * resolution * self.mag_sensitivity_adjustments[0],
         raw[1] as f32 * resolution * self.mag_sensitivity_adjustments[1],
         raw[2] as f32 * resolution * self.mag_sensitivity_adjustments[2]]
    }

    /// Reads and returns raw unscaled Magnetometer measurements (LSB).
    pub fn unscaled_mag<T>(&mut self) -> Result<T, E>
        where T: From<[i16; 3]>
    {
        let buffer = &mut [0; 7];
        self.dev.read_xyz(buffer)?;
        Ok(self.to_vector_inverted(buffer, 0).into())
    }

    /// Read and returns Magnetometer measurements scaled, adjusted for factory
    /// sensitivities, and converted to microTeslas.
    pub fn mag<T>(&mut self) -> Result<T, E>
        where T: From<[f32; 3]>
    {
        let buffer = &mut [0; 7];
        self.dev.read_xyz(buffer)?;
        Ok(self.scale_and_correct_mag(buffer, 0).into())
    }

    /// Returns raw mag sensitivity adjustments
    pub fn raw_mag_sensitivity_adjustments<T>(&self) -> T
        where T: From<[u8; 3]>
    {
        self.raw_mag_sensitivity_adjustments.into()
    }

    /// Returns mag sensitivity adjustments
    pub fn mag_sensitivity_adjustments<T>(&self) -> T
        where T: From<[f32; 3]>
    {
        self.mag_sensitivity_adjustments.into()
    }

    /// Configures magnetrometer full reading scale ([`MagScale`])
    ///
    /// [`Mag scale`]: ./conf/enum.MagScale.html
    pub fn mag_scale(&mut self, scale: MagScale) -> Result<(), E> {
        self.mag_scale = scale;
        self._mag_scale()
    }

    fn _mag_scale(&mut self) -> Result<(), E> {
        // Set magnetometer data resolution and sample ODR
        let scale = self.mag_scale as u8;
        AK8963::write(&mut self.dev,
                      ak8963::Register::CNTL,
                      scale << 4 | MMODE)?;
        Ok(())
    }

    fn check_ak8963_who_am_i(&mut self) -> Result<(), Error<E>> {
        let ak8963_who_am_i = self.ak8963_who_am_i()?;
        if ak8963_who_am_i == 0x48 {
            Ok(())
        } else {
            Err(Error::InvalidDevice(ak8963_who_am_i))
        }
    }

    /// Reads the AK8963 (magnetometer) WHO_AM_I register; should return `0x48`
    pub fn ak8963_who_am_i(&mut self) -> Result<u8, E> {
        AK8963::read(&mut self.dev, ak8963::Register::WHO_AM_I)
    }
}

// Any device, DMP
#[cfg(feature = "dmp")]
impl<E, DEV> Mpu9250<DEV, Dmp> where DEV: Device<Error = E>
{
    /// Private constructor that creates a DMP-based MPU with the
    /// specified device.
    fn new_dmp<D>(dev: DEV,
                  delay: &mut D,
                  config: &mut MpuConfig<Dmp>,
                  firmware: &[u8])
                  -> Result<Self, Error<E>>
        where D: DelayMs<u8>
    {
        let mut mpu9250 =
            Mpu9250 { dev,
                      raw_mag_sensitivity_adjustments: [0; 3],
                      mag_sensitivity_adjustments: [0.0; 3],
                      gyro_scale: config.gyro_scale
                                        .unwrap_or(GyroScale::_2000DPS),
                      accel_scale: config.accel_scale
                                         .unwrap_or(AccelScale::_8G),
                      mag_scale: config.mag_scale.unwrap_or_default(),
                      accel_data_rate:
                          config.accel_data_rate
                                .unwrap_or(AccelDataRate::DlpfConf(Dlpf::_1)),
                      gyro_temp_data_rate:
                          config.gyro_temp_data_rate
                                .unwrap_or(GyroTempDataRate::DlpfConf(Dlpf::_1)),
                      sample_rate_divisor: config.sample_rate_divisor
                                                 .or(Some(4)),
                      dmp_configuration: Some(config.dmp_configuration
                                                    .unwrap_or_default()),
                      packet_size: config.dmp_configuration
                                         .unwrap_or_default()
                                         .features
                                         .packet_size(),
                      _mode: PhantomData };
        mpu9250.init_mpu(delay)?;
        mpu9250.init_dmp(delay, firmware)?;
        Ok(mpu9250)
    }

    /// Logic to init the dmp
    fn init_dmp<D>(&mut self,
                   delay: &mut D,
                   firmware: &[u8])
                   -> Result<(), Error<E>>
        where D: DelayMs<u8>
    {
        let conf = self.dmp_configuration.unwrap_or_default();
        // disable i2c master mode and enable fifo
        const FIFO_EN: u8 = 1 << 6;
        self.dev.write(Register::USER_CTRL, FIFO_EN)?;
        delay.delay_ms(3);

        // enable i2c bypass
        self.interrupt_config(InterruptConfig::LATCH_INT_EN
                              | InterruptConfig::INT_ANYRD_CLEAR
                              | InterruptConfig::ACL
                              | InterruptConfig::BYPASS_EN)?;

        // load firmware
        self.load_firmware(firmware)?;

        // load orientation
        self.write_mem(DmpMemory::FCFG_1, &conf.orientation.gyro_axes())?;
        self.write_mem(DmpMemory::FCFG_2, &conf.orientation.accel_axes())?;
        self.write_mem(DmpMemory::FCFG_3, &conf.orientation.gyro_signs())?;
        self.write_mem(DmpMemory::FCFG_7, &conf.orientation.accel_signs())?;

        // set dmp features
        self.set_dmp_feature(delay)?;

        let div = [0, conf.rate as u8];
        self.write_mem(DmpMemory::D_0_22, &div)?;

        self.write_mem(DmpMemory::CFG_6,
                       &[0xfe, 0xf2, 0xab, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
                         0xbb, 0xaf, 0xdf, 0xdf])?;

        // turn on the dmp
        self.dev.write(Register::INT_ENABLE, 0)?;
        self.dev.write(Register::FIFO_EN, 0)?;

        // enable i2c bypass
        self.dev.write(Register::USER_CTRL, FIFO_EN)?;
        delay.delay_ms(10);
        self.interrupt_config(InterruptConfig::LATCH_INT_EN
                              | InterruptConfig::INT_ANYRD_CLEAR
                              | InterruptConfig::ACL
                              | InterruptConfig::BYPASS_EN)?;

        self.dev.write(Register::FIFO_EN, 0)?;
        self.dev.write(Register::INT_ENABLE, 0x02)?;
        self.dev.write(Register::FIFO_EN, 0)?;
        self.reset_fifo(delay)?;

        // set interrupt mode
        self.write_mem(DmpMemory::CFG_FIFO_ON_EVENT,
                       &[0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6,
                         0x09, 0xb4, 0xd9])?;

        Ok(())
    }

    /// Load the provided firmware in the internal dmp memory
    fn load_firmware(&mut self, firmware: &[u8]) -> Result<(), Error<E>> {
        let mut buffer: [u8; 17] = [0; 17];
        let mut addr: u16 = 0;

        for chunk in firmware.chunks(16) {
            self.write_mem(addr, chunk)?;
            self.read_mem(addr, &mut buffer)?;
            if &buffer[1..chunk.len() + 1] != chunk {
                return Err(Error::DmpFirmware);
            }
            addr += chunk.len() as u16;
        }
        assert_eq!(addr as usize, firmware.len());

        const DMP_START_ADDR: [u8; 2] = [0x04, 0x00];
        self.dev.write_many(Register::DMP_START_ADDR, &DMP_START_ADDR)?;

        Ok(())
    }

    /// Write the provided slice at the specified address in dmp memory
    fn write_mem<T>(&mut self, addr: T, data: &[u8]) -> Result<(), Error<E>>
        where T: Into<u16> + Copy
    {
        self.dev.write(Register::BANK_SEL, (addr.into() >> 8) as u8)?;
        self.dev.write(Register::MEM_ADDR, (addr.into() & 0xff) as u8)?;
        self.dev.write_many(Register::MEM_RW, data)?;
        Ok(())
    }

    /// Read dmp memory at the specified address into data
    fn read_mem<T>(&mut self, addr: T, data: &mut [u8]) -> Result<(), Error<E>>
        where T: Into<u16> + Copy
    {
        self.dev.write(Register::BANK_SEL, (addr.into() >> 8) as u8)?;
        self.dev.write(Register::MEM_ADDR, (addr.into() & 0xff) as u8)?;
        self.dev.read_many(Register::MEM_RW, data)?;
        Ok(())
    }

    /// Select which dmp features should be enabled
    fn set_dmp_feature<D>(&mut self, delay: &mut D) -> Result<(), Error<E>>
        where D: DelayMs<u8>
    {
        let features = self.dmp_configuration.unwrap_or_default().features;
        const GYRO_SF: [u8; 4] = [(46_850_825 >> 24) as u8,
                                  (46_850_825 >> 16) as u8,
                                  (46_850_825 >> 8) as u8,
                                  (46_850_825 & 0xff) as u8];
        self.write_mem(DmpMemory::D_0_104, &GYRO_SF)?;

        let mut conf = [0xa3 as u8; 10];
        if features.raw_accel {
            conf[1] = 0xc0;
            conf[2] = 0xc8;
            conf[3] = 0xc2;
        }
        if features.raw_gyro {
            conf[4] = 0xc4;
            conf[5] = 0xcc;
            conf[6] = 0xc6;
        }
        self.write_mem(DmpMemory::CFG_15, &conf)?;

        let mut set_config = |address,
                              state,
                              config_enabled,
                              config_disabled|
         -> Result<(), Error<E>> {
            let config = if state {
                config_enabled
            } else {
                config_disabled
            };
            self.write_mem(address, config)?;

            Ok(())
        };

        set_config(DmpMemory::CFG_27,
                   features.tap | features.android_orient,
                   &[0x20],
                   &[0xd8])?;
        set_config(DmpMemory::CFG_MOTION_BIAS,
                   features.gyro_auto_calibrate,
                   &[0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d],
                   &[0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7])?;

        if features.raw_gyro {
            set_config(DmpMemory::CFG_GYRO_RAW_DATA,
                       features.gyro_auto_calibrate,
                       &[0xb2, 0x8b, 0xb6, 0x9b],
                       &[0xb0, 0x80, 0xb4, 0x90])?;
        }

        // TODO handle tap
        set_config(DmpMemory::CFG_20, features.tap, &[0xf8], &[0xd8])?;
        set_config(DmpMemory::CFG_ANDROID_ORIENT_INT,
                   features.android_orient,
                   &[0xd9],
                   &[0xd8])?;
        set_config(DmpMemory::CFG_LP_QUAT,
                   features.quat,
                   &[0xc0, 0xc2, 0xc4, 0xc6],
                   &[0x8b, 0x8b, 0x8b, 0x8b])?;
        set_config(DmpMemory::CFG_8,
                   features.quat6,
                   &[0x20, 0x28, 0x30, 0x38],
                   &[0xa3, 0xa3, 0xa3, 0xa3])?;

        self.reset_fifo(delay)?;

        Ok(())
    }

    /// Reads and returns raw unscaled DMP measurement depending on
    /// activated features(LSB).
    pub fn dmp_unscaled_all<T1, T2>(
        &mut self)
        -> Result<UnscaledDmpMeasurement<T1, T2>, Error<E>>
        where T1: From<[i16; 3]>,
              T2: From<[i32; 4]>
    {
        let features = self.dmp_configuration.unwrap_or_default().features;

        let mut buffer: [u8; 33] = [0; 33];
        let read = self.read_fifo(&mut buffer[..self.packet_size + 1])?;
        if read < 0 {
            return Err(Error::DmpDataNotReady);
        }

        let mut offset = 0;
        let mut measures: UnscaledDmpMeasurement<T1, T2> =
            UnscaledDmpMeasurement { quaternion: None,
                                     accel: None,
                                     gyro: None };
        if features.quat6 || features.quat {
            measures.quaternion = Some(self.to_quat(&buffer).into());
            offset += 16;
        }
        if features.raw_accel {
            measures.accel = Some(self.to_vector(&buffer, offset).into());
            offset += 6;
        }
        if features.raw_gyro {
            measures.gyro = Some(self.to_vector(&buffer, offset).into());
            //offset += 6;
        }
        Ok(measures)
    }

    /// Read all measurement from DMP
    /// Reads and returns DMP measurement scaled depending on
    /// activated features(LSB).
    pub fn dmp_all<T1, T2>(&mut self)
                           -> Result<DmpMeasurement<T1, T2>, Error<E>>
        where T1: From<[f32; 3]>,
              T2: From<[f64; 4]>
    {
        let features = self.dmp_configuration.unwrap_or_default().features;

        let mut buffer: [u8; 33] = [0; 33];
        let read = self.read_fifo(&mut buffer[..self.packet_size + 1])?;
        if read < 0 {
            return Err(Error::DmpDataNotReady);
        }

        let mut offset = 0;
        let mut measures: DmpMeasurement<T1, T2> =
            DmpMeasurement { quaternion: None,
                             accel: None,
                             gyro: None };
        if features.quat6 || features.quat {
            measures.quaternion = Some(self.to_norm_quat(&buffer).into());
            offset += 16;
        }
        if features.raw_accel {
            measures.accel = Some(self.scale_accel(&buffer, offset).into());
            offset += 6;
        }
        if features.raw_gyro {
            measures.gyro = Some(self.scale_gyro(&buffer, offset).into());
            //offset += 6;
        }
        Ok(measures)
    }

    /// Parse quaternion from fifo buffer
    fn to_quat(&self, buffer: &[u8]) -> [i32; 4] {
        [(buffer[1] as i32) << 24
         | (buffer[2] as i32) << 16
         | (buffer[3] as i32) << 8
         | buffer[4] as i32,
         (buffer[5] as i32) << 24
         | (buffer[6] as i32) << 16
         | (buffer[7] as i32) << 8
         | buffer[8] as i32,
         (buffer[9] as i32) << 24
         | (buffer[10] as i32) << 16
         | (buffer[11] as i32) << 8
         | buffer[12] as i32,
         (buffer[13] as i32) << 24
         | (buffer[14] as i32) << 16
         | (buffer[15] as i32) << 8
         | buffer[16] as i32]
    }

    /// Normalized the quaternion
    fn to_norm_quat(&self, buffer: &[u8]) -> [f64; 4] {
        let quat = self.to_quat(buffer);
        //TODO handle this better, here is an ugly map on fixed size array
        let quat = [f64::from(quat[0]),
                    f64::from(quat[1]),
                    f64::from(quat[2]),
                    f64::from(quat[3])];
        let sum =
            libm::sqrt(quat.iter().map(|x| libm::pow(*x, 2.0)).sum::<f64>());
        [quat[0] / sum, quat[1] / sum, quat[2] / sum, quat[3] / sum]
    }
}

// Any device, any mode
impl<E, DEV, MODE> Mpu9250<DEV, MODE> where DEV: Device<Error = E>
{
    fn init_mpu<D>(&mut self, delay: &mut D) -> Result<(), E>
        where D: DelayMs<u8>
    {
        // Stop all communication with peripherals (such as AK8963).
        // If the chip is already powered up and if the communication is already running
        // then resetting the MPU can result in a stuck peripheral that
        // cannot be recovered from, except by physically powering down the peripheral.
        self.dev.write(Register::I2C_SLV0_CTRL, 0).ok(); // Ignore error code.
        self.dev.write(Register::I2C_SLV4_CTRL, 0).ok(); // Ignore error code.
        delay.delay_ms(1); // Wait for transfer to finish.

        // wake up device
        self.dev.write(Register::PWR_MGMT_1, 0x80)?;
        delay.delay_ms(100); // Wait for all registers to reset

        // get stable time source
        // Auto select clock source to be PLL gyroscope reference if ready else
        // else use the internal oscillator, bits 2:0 = 001
        self.dev.write(Register::PWR_MGMT_1, 0x01)?;
        // Enable all sensors
        self.dev.write(Register::PWR_MGMT_2, 0x00)?;

        // Set gyroscope full scale range
        self._gyro_scale()?;
        // Disable FSYNC and set thermometer and gyro bandwidth
        self._gyro_temp_data_rate()?;

        // Set accelerometer full-scale range configuration
        self._accel_scale()?;
        // Set accelerometer sample rate configuration
        self._accel_data_rate()?;

        // Set smplrt_div if present
        self._sample_rate_divisor()?;

        // Reset interrupts state
        self.dev.write(Register::INT_ENABLE, 0x00)?;

        Ok(())
    }

    fn reset_device<F>(self, f: F) -> Result<Self, Error<E>>
        where F: FnOnce(DEV) -> Option<DEV>
    {
        let raw_mag_sensitivity_adjustments =
            self.raw_mag_sensitivity_adjustments;
        let mag_sensitivity_adjustments = self.mag_sensitivity_adjustments;
        let gyro_scale = self.gyro_scale;
        let accel_scale = self.accel_scale;
        let mag_scale = self.mag_scale;
        let accel_data_rate = self.accel_data_rate;
        let gyro_temp_data_rate = self.gyro_temp_data_rate;
        let sample_rate_divisor = self.sample_rate_divisor;
        let dmp_configuration = self.dmp_configuration;
        let packet_size = self.packet_size;
        let _mode = self._mode;
        if let Some(new_dev) = f(self.dev) {
            Ok(Mpu9250 { dev: new_dev,
                         raw_mag_sensitivity_adjustments,
                         mag_sensitivity_adjustments,
                         gyro_scale,
                         accel_scale,
                         mag_scale,
                         accel_data_rate,
                         gyro_temp_data_rate,
                         sample_rate_divisor,
                         dmp_configuration,
                         packet_size,
                         _mode })
        } else {
            Err(Error::ReInitError)
        }
    }

    fn scale_accel(&self, buffer: &[u8], offset: usize) -> [f32; 3] {
        let resolution = self.accel_scale.resolution();
        let scale = G * resolution;
        let raw = self.to_vector(buffer, offset);
        [raw[0] as f32 * scale, raw[1] as f32 * scale, raw[2] as f32 * scale]
    }

    fn scale_gyro(&self, buffer: &[u8], offset: usize) -> [f32; 3] {
        let resolution = self.gyro_scale.resolution();
        let scale = PI_180 * resolution;
        let raw = self.to_vector(buffer, offset);
        [raw[0] as f32 * scale, raw[1] as f32 * scale, raw[2] as f32 * scale]
    }

    fn scale_temp(&self, buffer: &[u8], offset: usize) -> f32 {
        let t = f32((u16(buffer[offset + 1]) << 8) | u16(buffer[offset + 2]));
        (t - TEMP_ROOM_OFFSET) / TEMP_SENSITIVITY + TEMP_DIFF
    }

    /// Get enabled interrupts
    pub fn get_enabled_interrupts(&mut self) -> Result<InterruptEnable, E> {
        let bits = self.dev.read(Register::INT_ENABLE)?;
        Ok(InterruptEnable::from_bits_truncate(bits))
    }

    /// Get interrupt status
    pub fn get_interrupt_status(&mut self) -> Result<InterruptEnable, E> {
        let bits = self.dev.read(Register::INT_STATUS)?;
        Ok(InterruptEnable::from_bits_truncate(bits))
    }

    /// Enable specific interrupts
    pub fn enable_interrupts(&mut self, ie: InterruptEnable) -> Result<(), E> {
        self.dev.modify(Register::INT_ENABLE, |r| r | ie.bits())
    }

    /// Disable specific interrupts
    pub fn disable_interrupts(&mut self, ie: InterruptEnable) -> Result<(), E> {
        self.dev.modify(Register::INT_ENABLE, |r| r & !ie.bits())
    }

    /// Get interrupt configurtion
    pub fn get_interrupt_config(&mut self) -> Result<InterruptConfig, E> {
        let bits = self.dev.read(Register::INT_PIN_CFG)?;
        Ok(InterruptConfig::from_bits_truncate(bits))
    }

    /// *Overwrites* current interrupt configuration
    pub fn interrupt_config(&mut self, ic: InterruptConfig) -> Result<(), E> {
        self.dev.write(Register::INT_PIN_CFG, ic.bits())
    }

    /// Reset the internal FIFO
    pub fn reset_fifo<D>(&mut self, delay: &mut D) -> Result<(), Error<E>>
        where D: DelayMs<u8>
    {
        self.dev.write(Register::INT_ENABLE, 0)?;
        self.dev.write(Register::FIFO_EN, 0)?;
        self.dev.write(Register::USER_CTRL, 0)?;
        self.dev.write(Register::USER_CTRL, 0x0c)?;
        delay.delay_ms(3);
        self.dev.write(Register::USER_CTRL, 0xc0)?;
        self.dev.write(Register::INT_ENABLE, 0x02)?;
        self.dev.write(Register::FIFO_EN, 0)?;

        Ok(())
    }

    /// Read internal FIFO into data. **The first byte must be discarded**.
    /// Return the number of byte left in the FIFO.
    /// - If the number is positive, bytes are left in the FIFO
    /// - If the number is negative, only `data.len() - 1 - size` bytes were
    ///   avilable and were not read
    /// - If the number is 0, the FIFO is empty and data has been filled fully
    pub fn read_fifo(&mut self, data: &mut [u8]) -> Result<isize, Error<E>> {
        let mut buffer: [u8; 3] = [0; 3];
        self.dev.read_many(Register::FIFO_COUNT_H, &mut buffer)?;
        let count = (buffer[1] as usize) << 8 | buffer[2] as usize;
        if data.len() <= count + 1 {
            self.dev.read_many(Register::FIFO_RW, &mut data[..])?;
        }
        Ok(count as isize - data.len() as isize + 1)
    }

    /// Reads and returns unscaled accelerometer measurements (LSB).
    pub fn unscaled_accel<T>(&mut self) -> Result<T, E>
        where T: From<[i16; 3]>
    {
        let buffer = &mut [0; 7];
        self.dev.read_many(Register::ACCEL_XOUT_H, buffer)?;
        Ok(self.to_vector(buffer, 0).into())
    }

    /// Reads and returns accelerometer measurements scaled and converted to g.
    pub fn accel<T>(&mut self) -> Result<T, E>
        where T: From<[f32; 3]>
    {
        let buffer = &mut [0; 7];
        self.dev.read_many(Register::ACCEL_XOUT_H, buffer)?;
        Ok(self.scale_accel(buffer, 0).into())
    }

    /// Reads and returns unsacled Gyroscope measurements (LSB).
    pub fn unscaled_gyro<T>(&mut self) -> Result<T, E>
        where T: From<[i16; 3]>
    {
        let buffer = &mut [0; 7];
        self.dev.read_many(Register::GYRO_XOUT_H, buffer)?;
        Ok(self.to_vector(buffer, 0).into())
    }

    /// Reads and returns gyroscope measurements scaled and converted to rad/s.
    pub fn gyro<T>(&mut self) -> Result<T, E>
        where T: From<[f32; 3]>
    {
        let buffer = &mut [0; 7];
        self.dev.read_many(Register::GYRO_XOUT_H, buffer)?;
        Ok(self.scale_gyro(buffer, 0).into())
    }

    /// Configures accelerometer data rate config ([`AccelDataRate`]).
    ///
    /// [`AccelDataRate`]: ./conf/enum.AccelDataRate.html
    pub fn accel_data_rate(&mut self,
                           accel_data_rate: AccelDataRate)
                           -> Result<(), E> {
        self.accel_data_rate = accel_data_rate;
        self._accel_data_rate()
    }

    fn _accel_data_rate(&mut self) -> Result<(), E> {
        let bits = self.accel_data_rate.accel_config_bits();
        self.dev.write(Register::ACCEL_CONFIG_2, bits)?;

        Ok(())
    }

    /// Configures accelerometer full reading scale ([`Accel scale`]).
    ///
    /// [`Accel scale`]: ./conf/enum.AccelScale.html
    pub fn accel_scale(&mut self, scale: AccelScale) -> Result<(), E> {
        self.accel_scale = scale;
        self._accel_scale()
    }

    fn _accel_scale(&mut self) -> Result<(), E> {
        let scale = self.accel_scale as u8;
        self.dev.modify(Register::ACCEL_CONFIG, |r|
                    // Clear AFS bits [4:3]
                    (r & !0x18)
                    // Set full scale range for accel
                    | (scale << 3))?;
        Ok(())
    }

    /// Configures gyroscope and temperatures data rate config
    /// ([`GyroTempDataRate`]).
    ///
    /// [`GyroTempDataRate`]: ./conf/enum.GyroTempDataRate.html
    pub fn gyro_temp_data_rate(&mut self,
                               data_rate: GyroTempDataRate)
                               -> Result<(), E> {
        self.gyro_temp_data_rate = data_rate;
        self._gyro_temp_data_rate()
    }

    fn _gyro_temp_data_rate(&mut self) -> Result<(), E> {
        let fchoice_bits = self.gyro_temp_data_rate.fchoice_b_bits();
        let dlpf_bits = self.gyro_temp_data_rate.dlpf_bits();
        // Clear Fchoice bits [1:0] and set them
        self.dev
            .modify(Register::GYRO_CONFIG, |r| (r & !0b11) | fchoice_bits)?;
        // Clear and update DLPF_CFG
        self.dev.modify(Register::CONFIG, |r| (r & !0b111) | dlpf_bits)?;

        Ok(())
    }

    /// Configures gyroscope full reading scale ([`Gyro scale`]).
    ///
    /// [`Gyro scale`]: ./conf/enum.GyroScale.html
    pub fn gyro_scale(&mut self, scale: GyroScale) -> Result<(), E> {
        self.gyro_scale = scale;
        self._gyro_scale()
    }

    fn _gyro_scale(&mut self) -> Result<(), E> {
        let scale = self.gyro_scale as u8;
        self.dev.modify(Register::GYRO_CONFIG, |r|
                    // Clear GFS bits [4:3]
                    (r & !0x18)
                    // Set full scale range for the gyro
                    | (scale << 3))?;

        Ok(())
    }

    /// Reads and returns raw unscaled temperature sensor measurements (LSB).
    pub fn raw_temp(&mut self) -> Result<i16, E> {
        let buffer = &mut [0; 3];
        self.dev.read_many(Register::TEMP_OUT_H, buffer)?;
        let t = (u16(buffer[1]) << 8) | u16(buffer[2]);
        Ok(t as i16)
    }

    /// Reads and returns adjusted temperature measurements converted to C.
    pub fn temp(&mut self) -> Result<f32, E> {
        let buffer = &mut [0; 3];
        self.dev.read_many(Register::TEMP_OUT_H, buffer)?;
        Ok(self.scale_temp(buffer, 0))
    }

    /// Configures sample rate divisor.
    /// Sample rate divisor divides the internal sample rate to generate
    /// the sample rate that controls sensor data output rate, FIFO sample
    /// rate. NOTE: This register is only effective when dlpf mode used for
    /// GyroTempDataRate see [`GyroTempDataRate`].
    /// SampleRate = InternalSampleRate / (1 + SMPLRT_DIV).
    ///
    /// [`GyroTempDataRate`]: ./conf/enum.GyroTempDataRate.html
    pub fn sample_rate_divisor(&mut self, smplrt_div: u8) -> Result<(), E> {
        self.sample_rate_divisor = Some(smplrt_div);
        self.dev.write(Register::SMPLRT_DIV, smplrt_div)?;
        Ok(())
    }

    fn _sample_rate_divisor(&mut self) -> Result<(), E> {
        if let Some(sample_rate_div) = self.sample_rate_divisor {
            self.dev.write(Register::SMPLRT_DIV, sample_rate_div)?;
        }
        Ok(())
    }

    fn _calibrate_at_rest<D>(&mut self,
                             delay: &mut D)
                             -> Result<[f32; 3], Error<E>>
        where D: DelayMs<u8>
    {
        // First save current values, as we reset them below
        let orig_gyro_scale = self.gyro_scale;
        let orig_accel_scale = self.accel_scale;
        let orig_gyro_temp_data_rate = self.gyro_temp_data_rate;
        let orig_accel_data_rate = self.accel_data_rate;
        let orig_sample_rate_divisor = self.sample_rate_divisor;
        // reset device
        self.dev.write(Register::PWR_MGMT_1, 0x80)?;
        // get stable time source;
        // Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        self.dev.write(Register::PWR_MGMT_1, 0x01)?;
        // Enable all sensors
        self.dev.write(Register::PWR_MGMT_2, 0x00)?;
        delay.delay_ms(200);

        // Configure device for bias calculation
        // Disable all interrupts
        self.dev.write(Register::INT_ENABLE, 0x00)?;
        // Disable FIFO
        self.dev.write(Register::FIFO_EN, 0x00)?;
        // Turn on internal clock source
        self.dev.write(Register::PWR_MGMT_1, 0x00)?;
        // Disable I2C
        self.dev.write(Register::I2C_MST_CTRL, 0x00)?;
        // Disable FIFO and I2C master modes
        self.dev.write(Register::USER_CTRL, 0x00)?;
        // Reset FIFO and DMP
        self.dev.write(Register::USER_CTRL, 0x0C)?;
        delay.delay_ms(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        // Set low-pass filter to 184Hz
        self.gyro_temp_data_rate(GyroTempDataRate::DlpfConf(Dlpf::_1))?;
        self.sample_rate_divisor(0)?;
        // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        // self.gyro_scale(GyroScale::_250DPS)?;
        // Set accelerometer low pass filter to
        self.accel_data_rate(AccelDataRate::DlpfConf(Dlpf::_1))?;
        // Set accelerometer full-scale to 2g, maximum sensitivity
        self.accel_scale(AccelScale::_2G)?;

        // Configure FIFO to capture accelerometer and gyro data for bias
        // calculation
        self.dev.write(Register::USER_CTRL, 0x40)?;
        // Enable FIFO
        self.dev.write(Register::FIFO_EN, 0x78)?;
        // Enable gyro and accelerometer sensors for FIFO
        // (max size 512 bytes in MPU-9150)
        // accumulate 40 samples in 40 milliseconds =480 bytes
        delay.delay_ms(40);

        // At end of sample accumulation, turn off FIFO sensor read
        // Disable gyro and accelerometer sensors for FIFO
        self.dev.write(Register::FIFO_EN, 0x00)?;
        // read FIFO sample count
        let buffer = &mut [0; 13]; // larger buffer is used later
        self.dev.read_many(Register::FIFO_COUNT_H, &mut buffer[0..3])?;
        let fifo_count = ((u16(buffer[1]) << 8) | u16(buffer[2])) as i16;
        // Aim for at least half
        // How many sets of full gyro and accelerometer data for averaging
        let packet_count = i32(fifo_count / 12);
        if packet_count < 20 {
            return Err(Error::CalibrationError);
        }
        let mut accel_biases = [0; 3];
        let mut gyro_biases = [0; 3];
        for _ in 0..packet_count {
            self.dev.read_many(Register::FIFO_RW, buffer)?;
            let accel_temp = self.to_vector(buffer, 0);
            let gyro_temp = self.to_vector(buffer, 6);
            for i in 0..3 {
                accel_biases[i] += i32(accel_temp[i]);
                gyro_biases[i] += i32(gyro_temp[i]);
            }
        }
        for i in 0..3 {
            accel_biases[i] /= packet_count;
            gyro_biases[i] /= packet_count;
        }

        // Construct the gyro biases and push them to the hardware gyro bias
        // registers, which are reset to zero upon device startup.
        // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias
        // input format.
        // Biases are additive, so change sign on
        // calculated average gyro biases

        self.set_unscaled_gyro_bias(false,
                                    [(gyro_biases[0] / -4) as i16,
                                     (gyro_biases[1] / -4) as i16,
                                     (gyro_biases[2] / -4) as i16])?;

        // Compute accelerometer biases to be returned
        let resolution = self.accel_scale.resolution();
        let scale = G * resolution;

        // Set original values back and re-init device
        self.gyro_scale = orig_gyro_scale;
        self.accel_scale = orig_accel_scale;
        self.gyro_temp_data_rate = orig_gyro_temp_data_rate;
        self.accel_data_rate = orig_accel_data_rate;
        self.sample_rate_divisor = orig_sample_rate_divisor;
        self.init_mpu(delay)?;

        Ok([accel_biases[0] as f32 * scale,
            accel_biases[1] as f32 * scale,
            accel_biases[2] as f32 * scale])
    }

    /// Get unscaled gyroscope biases.
    /// Output format is +-1000dps
    fn get_unscaled_gyro_bias(&mut self) -> Result<[i16; 3], Error<E>> {
        Ok([(self.dev.read(Register::XG_OFFSET_H)? as i16) << 8
            | self.dev.read(Register::XG_OFFSET_L)? as i16,
            (self.dev.read(Register::YG_OFFSET_H)? as i16) << 8
            | self.dev.read(Register::YG_OFFSET_L)? as i16,
            (self.dev.read(Register::ZG_OFFSET_H)? as i16) << 8
            | self.dev.read(Register::ZG_OFFSET_L)? as i16])
    }

    /// Get scaled gyroscope biases.
    pub fn get_gyro_bias<T>(&mut self) -> Result<T, Error<E>>
        where T: From<[f32; 3]>
    {
        let biases = self.get_unscaled_gyro_bias()?;
        let scale = GyroScale::_1000DPS.resolution();

        Ok([biases[0] as f32 * scale,
            biases[1] as f32 * scale,
            biases[2] as f32 * scale].into())
    }

    /// Set unscaled gyroscope biases.
    /// In relative mode it will add the new biases to the existing ones instead
    /// of replacing them. Input format is +-1000dps
    fn set_unscaled_gyro_bias(&mut self,
                              relative: bool,
                              biases: [i16; 3])
                              -> Result<(), Error<E>> {
        let mut new_biases = biases;

        if relative {
            let old_biases = self.get_unscaled_gyro_bias()?;

            for i in 0..3 {
                new_biases[i] += old_biases[i];
            }
        }

        self.dev.write(Register::XG_OFFSET_H,
                        ((new_biases[0] >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::XG_OFFSET_L, (new_biases[0] & 0xFF) as u8)?;
        self.dev.write(Register::YG_OFFSET_H,
                        ((new_biases[1] >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::YG_OFFSET_L, (new_biases[1] & 0xFF) as u8)?;
        self.dev.write(Register::ZG_OFFSET_H,
                        ((new_biases[2] >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::ZG_OFFSET_L, (new_biases[2] & 0xFF) as u8)?;

        Ok(())
    }

    /// Set scaled gyro biases.
    /// In relative mode it will add the new biases to the existing ones instead
    /// of replacing them.
    pub fn set_gyro_bias<T>(&mut self,
                            relative: bool,
                            biases: T)
                            -> Result<(), Error<E>>
        where T: Into<[f32; 3]>
    {
        let biases = biases.into();
        let scale = GyroScale::_1000DPS.resolution();

        self.set_unscaled_gyro_bias(relative,
                                    [(biases[0] / scale) as i16,
                                     (biases[1] / scale) as i16,
                                     (biases[2] / scale) as i16])
    }

    /// Get unscaled accelerometer biases.
    /// Output format is +-16G
    fn get_unscaled_accel_bias(&mut self) -> Result<[i16; 3], Error<E>> {
        Ok([(self.dev.read(Register::XA_OFFSET_H)? as i16) << 8
            | self.dev.read(Register::XA_OFFSET_L)? as i16,
            (self.dev.read(Register::YA_OFFSET_H)? as i16) << 8
            | self.dev.read(Register::YA_OFFSET_L)? as i16,
            (self.dev.read(Register::ZA_OFFSET_H)? as i16) << 8
            | self.dev.read(Register::ZA_OFFSET_L)? as i16])
    }

    /// Get scaled accelerometer biases.
    pub fn get_accel_bias<T>(&mut self) -> Result<T, Error<E>>
        where T: From<[f32; 3]>
    {
        let biases = self.get_unscaled_accel_bias()?;
        let scale = G * AccelScale::_16G.resolution();

        Ok([biases[0] as f32 * scale,
            biases[1] as f32 * scale,
            biases[2] as f32 * scale].into())
    }

    /// Set unscaled accelerometer biases.
    /// Keep in mind that the registers contain factory-supplied values after
    /// reset. In relative mode it will add the new biases to the existing
    /// ones instead of replacing them. Input format is +-16G.
    fn set_unscaled_accel_bias(&mut self,
                               relative: bool,
                               biases: [i16; 3])
                               -> Result<(), Error<E>> {
        let mut new_biases = self.get_unscaled_accel_bias()?;

        // Do not touch the last bit
        for i in 0..3 {
            if !relative {
                new_biases[i] &= 1
            }
            new_biases[i] += biases[i] & !1;
        }

        self.dev.write(Register::XA_OFFSET_H,
                        ((new_biases[0] >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::XA_OFFSET_L, (new_biases[0] & 0xFF) as u8)?;
        self.dev.write(Register::YA_OFFSET_H,
                        ((new_biases[1] >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::YA_OFFSET_L, (new_biases[1] & 0xFF) as u8)?;
        self.dev.write(Register::ZA_OFFSET_H,
                        ((new_biases[2] >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::ZA_OFFSET_L, (new_biases[2] & 0xFF) as u8)?;

        Ok(())
    }

    /// Set scaled accelerometer biases.
    /// Keep in mind that the registers contain factory-supplied values after
    /// reset. In relative mode it will add the new biases to the existing
    /// ones instead of replacing them.
    pub fn set_accel_bias<T>(&mut self,
                             relative: bool,
                             biases: T)
                             -> Result<(), Error<E>>
        where T: Into<[f32; 3]>
    {
        let biases = biases.into();
        let scale = G * AccelScale::_16G.resolution();

        self.set_unscaled_accel_bias(relative,
                                     [(biases[0] / scale) as i16,
                                      (biases[1] / scale) as i16,
                                      (biases[2] / scale) as i16])
    }

    fn to_vector(&self, buffer: &[u8], offset: usize) -> [i16; 3] {
        [((u16(buffer[offset + 1]) << 8) | u16(buffer[offset + 2])) as i16,
         ((u16(buffer[offset + 3]) << 8) | u16(buffer[offset + 4])) as i16,
         ((u16(buffer[offset + 5]) << 8) | u16(buffer[offset + 6])) as i16]
    }

    fn to_vector_inverted(&self, buffer: &[u8], offset: usize) -> [i16; 3] {
        [((u16(buffer[offset + 2]) << 8) + u16(buffer[offset + 1])) as i16,
         ((u16(buffer[offset + 4]) << 8) + u16(buffer[offset + 3])) as i16,
         ((u16(buffer[offset + 6]) << 8) + u16(buffer[offset + 5])) as i16]
    }

    /// Reads the WHO_AM_I register; should return `0x71`
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        self.dev.read(Register::WHO_AM_I)
    }
}

// Any device, any mode, no possible errors
impl<DEV, MODE> Mpu9250<DEV, MODE> {
    /// Returns Accelerometer resolution.
    pub fn accel_resolution(&self) -> f32 {
        self.accel_scale.resolution()
    }

    /// Returns Gyroscope resolution.
    pub fn gyro_resolution(&self) -> f32 {
        self.gyro_scale.resolution()
    }

    /// Returns Magnetometer resolution.
    pub fn mag_resolution(&self) -> f32 {
        self.mag_scale.resolution()
    }
}

/// SPI mode
pub const MODE: Mode = Mode { polarity: Polarity::IdleHigh,
                              phase: Phase::CaptureOnSecondTransition };

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
#[doc(hidden)]
pub enum Register {
    ACCEL_CONFIG = 0x1c,
    ACCEL_CONFIG_2 = 0x1d,
    ACCEL_XOUT_H = 0x3b,
    ACCEL_YOUT_H = 0x3d,
    CONFIG = 0x1a,
    XG_OFFSET_H = 0x13, // User-defined trim values for gyroscope
    XG_OFFSET_L = 0x14,
    YG_OFFSET_H = 0x15,
    YG_OFFSET_L = 0x16,
    ZG_OFFSET_H = 0x17,
    ZG_OFFSET_L = 0x18,
    EXT_SENS_DATA_00 = 0x49,
    EXT_SENS_DATA_01 = 0x4a,
    EXT_SENS_DATA_02 = 0x4b,
    EXT_SENS_DATA_03 = 0x4c,
    EXT_SENS_DATA_04 = 0x4d,
    GYRO_CONFIG = 0x1b,
    SMPLRT_DIV = 0x19,
    GYRO_XOUT_H = 0x43,
    FIFO_EN = 0x23,
    I2C_MST_CTRL = 0x24,
    I2C_MST_STATUS = 0x36,
    I2C_SLV0_ADDR = 0x25,
    I2C_SLV0_CTRL = 0x27,
    I2C_SLV0_DO = 0x63,
    I2C_SLV0_REG = 0x26,
    I2C_SLV4_ADDR = 0x31,
    I2C_SLV4_CTRL = 0x34,
    I2C_SLV4_DI = 0x35,
    I2C_SLV4_DO = 0x33,
    I2C_SLV4_REG = 0x32,
    INT_PIN_CFG = 0x37,
    INT_ENABLE = 0x38,
    INT_STATUS = 0x3a,
    PWR_MGMT_1 = 0x6b,
    PWR_MGMT_2 = 0x6c,
    TEMP_OUT_H = 0x41,
    USER_CTRL = 0x6a,
    BANK_SEL = 0x6d,
    MEM_ADDR = 0x6e,
    MEM_RW = 0x6f,
    DMP_START_ADDR = 0x70,
    FIFO_COUNT_H = 0x72,
    FIFO_RW = 0x74,
    WHO_AM_I = 0x75,
    XA_OFFSET_H = 0x77,
    XA_OFFSET_L = 0x78,
    YA_OFFSET_H = 0x7A,
    YA_OFFSET_L = 0x7B,
    ZA_OFFSET_H = 0x7D,
    ZA_OFFSET_L = 0x7E,
}

const R: u8 = 1 << 7;
const W: u8 = 0 << 7;

impl Register {
    fn read_address(&self) -> u8 {
        *self as u8 | R
    }

    fn write_address(&self) -> u8 {
        *self as u8 | W
    }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
#[doc(hidden)]
pub enum DmpMemory {
    CFG_LP_QUAT = 2712,
    END_ORIENT_TEMP = 1866,
    CFG_27 = 2742,
    CFG_20 = 2224,
    CFG_23 = 2745,
    CFG_FIFO_ON_EVENT = 2690,
    END_PREDICTION_UPDATE = 1761,
    CGNOTICE_INTR = 2620,
    X_GRT_Y_TMP = 1358,
    CFG_DR_INT = 1029,
    CFG_AUTH = 1035,
    UPDATE_PROP_ROT = 1835,
    END_COMPARE_Y_X_TMP2 = 1455,
    SKIP_X_GRT_Y_TMP = 1359,
    SKIP_END_COMPARE = 1435,
    FCFG_3 = 1088,
    FCFG_2 = 1066,
    FCFG_1 = 1062,
    END_COMPARE_Y_X_TMP3 = 1434,
    FCFG_7 = 1073,
    FCFG_6 = 1106,
    FLAT_STATE_END = 1713,
    SWING_END_4 = 1616,
    SWING_END_2 = 1565,
    SWING_END_3 = 1587,
    SWING_END_1 = 1550,
    CFG_8 = 2718,
    CFG_15 = 2727,
    CFG_16 = 2746,
    CFG_EXT_GYRO_BIAS = 1189,
    END_COMPARE_Y_X_TMP = 1407,
    DO_NOT_UPDATE_PROP_ROT = 1839,
    CFG_7 = 1205,
    FLAT_STATE_END_TEMP = 1683,
    END_COMPARE_Y_X = 1484,
    SKIP_SWING_END_1 = 1551,
    SKIP_SWING_END_3 = 1588,
    SKIP_SWING_END_2 = 1566,
    TILTG75_START = 1672,
    CFG_6 = 2753,
    TILTL75_END = 1669,
    END_ORIENT = 1884,
    CFG_FLICK_IN = 2573,
    TILTL75_START = 1643,
    CFG_MOTION_BIAS = 1208,
    X_GRT_Y = 1408,
    TEMPLABEL = 2324,
    CFG_ANDROID_ORIENT_INT = 1853,
    CFG_GYRO_RAW_DATA = 2722,
    X_GRT_Y_TMP2 = 1379,

    D_0_22 = 22 + 512,
    D_0_24 = 24 + 512,

    D_0_36 = 36,
    //D_0_52 = 52, // Same as D_TILT1_H
    D_0_96 = 96,
    D_0_104 = 104,
    D_0_108 = 108,
    D_0_163 = 163,
    D_0_188 = 188,
    D_0_192 = 192,
    D_0_224 = 224,
    D_0_228 = 228,
    D_0_232 = 232,
    D_0_236 = 236,

    D_1_2 = 256 + 2,
    D_1_4 = 256 + 4,
    D_1_8 = 256 + 8,
    D_1_10 = 256 + 10,
    D_1_24 = 256 + 24,
    D_1_28 = 256 + 28,
    D_1_36 = 256 + 36,
    D_1_40 = 256 + 40,
    D_1_44 = 256 + 44,
    D_1_72 = 256 + 72,
    D_1_74 = 256 + 74,
    D_1_79 = 256 + 79,
    D_1_88 = 256 + 88,
    D_1_90 = 256 + 90,
    D_1_92 = 256 + 92,
    D_1_96 = 256 + 96,
    D_1_98 = 256 + 98,
    D_1_106 = 256 + 106,
    D_1_108 = 256 + 108,
    D_1_112 = 256 + 112,
    D_1_128 = 256 + 144,
    D_1_152 = 256 + 12,
    D_1_160 = 256 + 160,
    D_1_176 = 256 + 176,
    D_1_178 = 256 + 178,
    D_1_218 = 256 + 218,
    D_1_232 = 256 + 232,
    D_1_236 = 256 + 236,
    D_1_240 = 256 + 240,
    D_1_244 = 256 + 244,
    D_1_250 = 256 + 250,
    D_1_252 = 256 + 252,
    D_2_12 = 512 + 12,
    D_2_96 = 512 + 96,
    D_2_108 = 512 + 108,
    D_2_208 = 512 + 208,
    D_2_224 = 512 + 224,
    //D_2_236 = 512 + 236, // Same as FLICK_UPPER
    D_2_244 = 512 + 244,
    D_2_248 = 512 + 248,
    D_2_252 = 512 + 252,

    CPASS_BIAS_X = 35 * 16 + 4,
    CPASS_BIAS_Y = 35 * 16 + 8,
    CPASS_BIAS_Z = 35 * 16 + 12,
    CPASS_MTX_00 = 36 * 16,
    CPASS_MTX_01 = 36 * 16 + 4,
    CPASS_MTX_02 = 36 * 16 + 8,
    CPASS_MTX_10 = 36 * 16 + 12,
    CPASS_MTX_11 = 37 * 16,
    CPASS_MTX_12 = 37 * 16 + 4,
    CPASS_MTX_20 = 37 * 16 + 8,
    CPASS_MTX_21 = 37 * 16 + 12,
    CPASS_MTX_22 = 43 * 16 + 12,
    D_EXT_GYRO_BIAS_X = 61 * 16,
    D_EXT_GYRO_BIAS_Y = 61 * 16 + 4,
    D_EXT_GYRO_BIAS_Z = 61 * 16 + 8,
    D_ACT0 = 40 * 16,
    D_ACSX = 40 * 16 + 4,
    D_ACSY = 40 * 16 + 8,
    D_ACSZ = 40 * 16 + 12,

    FLICK_MSG = 45 * 16 + 4,
    FLICK_COUNTER = 45 * 16 + 8,
    FLICK_LOWER = 45 * 16 + 12,
    FLICK_UPPER = 46 * 16 + 12,

    D_AUTH_OUT = 992,
    D_AUTH_IN = 996,
    D_AUTH_A = 1000,
    D_AUTH_B = 1004,

    D_PEDSTD_BP_B = 768 + 0x1C,
    D_PEDSTD_HP_A = 768 + 0x78,
    D_PEDSTD_HP_B = 768 + 0x7C,
    D_PEDSTD_BP_A4 = 768 + 0x40,
    D_PEDSTD_BP_A3 = 768 + 0x44,
    D_PEDSTD_BP_A2 = 768 + 0x48,
    D_PEDSTD_BP_A1 = 768 + 0x4C,
    D_PEDSTD_INT_THRSH = 768 + 0x68,
    D_PEDSTD_CLIP = 768 + 0x6C,
    D_PEDSTD_SB = 768 + 0x28,
    D_PEDSTD_SB_TIME = 768 + 0x2C,
    D_PEDSTD_PEAKTHRSH = 768 + 0x98,
    D_PEDSTD_TIML = 768 + 0x2A,
    D_PEDSTD_TIMH = 768 + 0x2E,
    D_PEDSTD_PEAK = 768 + 0x94,
    D_PEDSTD_STEPCTR = 768 + 0x60,
    D_PEDSTD_TIMECTR = 964,
    D_PEDSTD_DECI = 768 + 0xA0,

    //D_HOST_NO_MOT = 976, // Same as D_EXT_GYRO_BIAS_X
    D_ACCEL_BIAS = 660,

    D_ORIENT_GAP = 76,

    D_TILT0_H = 48,
    D_TILT0_L = 50,
    D_TILT1_H = 52,
    D_TILT1_L = 54,
    D_TILT2_H = 56,
    D_TILT2_L = 58,
    D_TILT3_H = 60,
    D_TILT3_L = 62,
}

impl Into<u16> for DmpMemory {
    fn into(self) -> u16 {
        self as u16
    }
}

/// Unscaled IMU measurements (LSB)
#[derive(Clone, Copy, Debug)]
pub struct UnscaledImuMeasurements<T> {
    /// Accelerometer measurements (LSB)
    pub accel: T,
    /// Gyroscope measurements (LSB)
    pub gyro: T,
    /// Temperature sensor measurement (LSB)
    pub temp: i16,
}

/// Scaled IMU measurements converted to units
#[derive(Clone, Copy, Debug)]
pub struct ImuMeasurements<T> {
    /// Accelerometer measurements (g)
    pub accel: T,
    /// Gyroscope measurements (rad/s)
    pub gyro: T,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

/// Unscaled MARG measurements (LSB)
#[derive(Copy, Clone, Debug)]
pub struct UnscaledMargMeasurements<T> {
    /// Accelerometer measurements (LSB)
    pub accel: T,
    /// Gyroscope measurements (LSB)
    pub gyro: T,
    /// Magnetometer measurements (LSB)
    pub mag: T,
    /// Temperature sensor measurement (LSB)
    pub temp: i16,
}

/// MARG measurements scaled with respective scales and converted
/// to appropriate units.
#[derive(Copy, Clone, Debug)]
pub struct MargMeasurements<T> {
    /// Accelerometer measurements (g)
    pub accel: T,
    /// Gyroscope measurements (rad/s)
    pub gyro: T,
    /// Magnetometer measurements (μT)
    pub mag: T,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

/// DMP measurement scaled with respective scales and converted
/// to appropriate units. Each measurement will be present only
/// if the corresponding features is activated in [`dmp features`]
///
/// [`dmp features`]: ./struct.DmpFeatures.html
#[derive(Copy, Clone, Debug)]
pub struct UnscaledDmpMeasurement<T1, T2> {
    /// raw quaternion (LSB)
    pub quaternion: Option<T2>,
    /// Accelerometer measurements (LSB)
    pub accel: Option<T1>,
    /// Gyroscope measurements (LSB)
    pub gyro: Option<T1>,
}

/// DMP measurement scaled with respective scales and converted
/// to appropriate units. Each measurement will be present only
/// if the corresponding features is activated in [`dmp features`]
///
/// [`dmp features`]: ./struct.DmpFeatures.html
#[derive(Copy, Clone, Debug)]
pub struct DmpMeasurement<T1, T2> {
    /// Normalized quaternion
    pub quaternion: Option<T2>,
    /// Accelerometer measurements (g)
    pub accel: Option<T1>,
    /// Gyroscope measurements (rad/s)
    pub gyro: Option<T1>,
}

fn transpose<T, E>(o: Option<Result<T, E>>) -> Result<Option<T>, E> {
    match o {
        Some(Ok(x)) => Ok(Some(x)),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}
