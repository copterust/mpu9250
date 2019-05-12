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
extern crate generic_array;
extern crate nalgebra;

mod ak8963;
mod conf;
mod device;
mod types;

use ak8963::AK8963;

use core::marker::PhantomData;

use cast::{f32, i32, u16};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};
use nalgebra::convert;
pub use nalgebra::Vector3;

use hal::blocking::delay::DelayMs;
use hal::spi::{Mode, Phase, Polarity};

pub use conf::*;
pub use types::*;

#[doc(hidden)]
pub use device::Releasable;
pub use device::{Device, NineDOFDevice, I2cDevice, SpiDevice};

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
    mag_sensitivity_adjustments: Vector3<f32>,
    raw_mag_sensitivity_adjustments: Vector3<u8>,
    // configuration
    gyro_scale: GyroScale,
    accel_scale: AccelScale,
    mag_scale: MagScale,
    gyro_temp_data_rate: GyroTempDataRate,
    accel_data_rate: AccelDataRate,
    sample_rate_divisor: Option<u8>,
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
        pub fn imu_default<D>(i2c: I2C, delay: &mut D) -> Result<Self, Error<E>>
            where D: DelayMs<u8>
        {
            Mpu9250::imu(i2c, delay, &mut MpuConfig::imu())
        }

        /// Creates a new Imu driver from an I2C peripheral with the
        /// provided configuration [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn imu<D>(i2c: I2C,
                      delay: &mut D,
                      config: &mut MpuConfig<Imu>)
                      -> Result<Self, Error<E>>
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
        pub fn imu_with_reinit<D, F>(i2c: I2C,
                                     delay: &mut D,
                                     config: &mut MpuConfig<Imu>,
                                     reinit_fn: F)
                                     -> Result<Self, Error<E>>
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
        pub fn marg_default<D>(i2c: I2C,
                               delay: &mut D)
                               -> Result<Self, Error<E>>
            where D: DelayMs<u8>
        {
            Mpu9250::marg(i2c, delay, &mut MpuConfig::marg())
        }

        /// Creates a new MARG driver from an I2C peripheral
        /// with provided configuration [`Config`].
        ///
        /// [`Config`]: ./conf/struct.MpuConfig.html
        pub fn marg<D>(i2c: I2C,
                       delay: &mut D,
                       config: &mut MpuConfig<Marg>)
                       -> Result<Self, Error<E>>
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
        pub fn marg_with_reinit<D, F>(i2c: I2C,
                                      delay: &mut D,
                                      config: &mut MpuConfig<Marg>,
                                      reinit_fn: F)
                                      -> Result<Self, Error<E>>
            where D: DelayMs<u8>,
                  F: FnOnce(I2C) -> Option<I2C>
        {
            let dev = I2cDevice::new(i2c);
            let mpu = Self::new_marg(dev, delay, config)?;
            mpu.reinit_i2c_device(reinit_fn)
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

        fn reinit_i2c_device<F>(self, reinit_fn: F) -> Result<Self, Error<E>>
            where F: FnOnce(I2C) -> Option<I2C>
        {
            self.reset_device(|i2cdev| {
                    let i2c = i2cdev.release();
                    reinit_fn(i2c).map(|i2c| {
                                             I2cDevice::new(i2c)
                                         })
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
                      raw_mag_sensitivity_adjustments: Vector3::zeros(),
                      mag_sensitivity_adjustments: Vector3::zeros(),
                      gyro_scale: config.gyro_scale.unwrap_or_default(),
                      accel_scale: config.accel_scale.unwrap_or_default(),
                      mag_scale: MagScale::default(),
                      accel_data_rate: config.accel_data_rate
                                             .unwrap_or_default(),
                      gyro_temp_data_rate: config.gyro_temp_data_rate
                                                 .unwrap_or_default(),
                      sample_rate_divisor: config.sample_rate_divisor,
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
    pub fn unscaled_all(&mut self) -> Result<UnscaledImuMeasurements, E> {
        let buffer = self.dev.read_many::<U15>(Register::ACCEL_XOUT_H)?;
        let accel = self.to_vector(buffer, 0);
        let temp = ((u16(buffer[7]) << 8) | u16(buffer[8])) as i16;
        let gyro = self.to_vector(buffer, 8);

        Ok(UnscaledImuMeasurements { accel,
                                     gyro,
                                     temp })
    }

    /// Reads and returns Accelerometer + Gyroscope + Thermometer
    /// measurements scaled and converted to respective units.
    pub fn all(&mut self) -> Result<ImuMeasurements, E> {
        let buffer = self.dev.read_many::<U15>(Register::ACCEL_XOUT_H)?;

        let accel = self.scale_accel(buffer, 0);
        let temp = self.scale_temp(buffer, 6);
        let gyro = self.scale_gyro(buffer, 8);

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
    pub fn calibrate_at_rest<D>(&mut self,
                                delay: &mut D)
                                -> Result<Vector3<f32>, Error<E>>
        where D: DelayMs<u8>
    {
        self._calibrate_at_rest(delay)
    }
}

// Any device, 9DOF
impl<E, DEV> Mpu9250<DEV, Marg> where DEV: Device<Error = E> + AK8963<Error = E> + NineDOFDevice,
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
                      raw_mag_sensitivity_adjustments: Vector3::zeros(),
                      mag_sensitivity_adjustments: Vector3::zeros(),
                      gyro_scale: config.gyro_scale.unwrap_or_default(),
                      accel_scale: config.accel_scale.unwrap_or_default(),
                      mag_scale: config.mag_scale.unwrap_or_default(),
                      accel_data_rate: config.accel_data_rate
                                             .unwrap_or_default(),
                      gyro_temp_data_rate: config.gyro_temp_data_rate
                                                 .unwrap_or_default(),
                      sample_rate_divisor: config.sample_rate_divisor,
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
    pub fn calibrate_at_rest<D>(&mut self,
                                delay: &mut D)
                                -> Result<Vector3<f32>, Error<E>>
        where D: DelayMs<u8>
    {
        let accel_biases = self._calibrate_at_rest(delay)?;
        self.init_ak8963(delay)?;
        Ok(accel_biases)
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
            Vector3::new(mag_x_bias, mag_y_bias, mag_z_bias);
        self.mag_sensitivity_adjustments =
            self.raw_mag_sensitivity_adjustments
                .map(|d| f32(d - 128) / 256. + 1.);
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
    pub fn unscaled_all(&mut self) -> Result<UnscaledMargMeasurements, E> {
        let buffer = NineDOFDevice::read_9dof(&mut self.dev, Register::ACCEL_XOUT_H)?;
        let accel = self.to_vector(buffer, 0);
        let temp = ((u16(buffer[7]) << 8) | u16(buffer[8])) as i16;
        let gyro = self.to_vector(buffer, 8);
        let mag = self.to_vector_inverted(buffer, 14);

        Ok(UnscaledMargMeasurements { accel,
                                      gyro,
                                      temp,
                                      mag })
    }

    /// Reads and returns Accelerometer + Gyroscope + Thermometer + Magnetometer
    /// measurements scaled and converted to respective units.
    pub fn all(&mut self) -> Result<MargMeasurements, E> {
        let buffer = NineDOFDevice::read_9dof(&mut self.dev, Register::ACCEL_XOUT_H)?;

        let accel = self.scale_accel(buffer, 0);
        let temp = self.scale_temp(buffer, 6);
        let gyro = self.scale_gyro(buffer, 8);
        let mag = self.scale_and_correct_mag(buffer, 14);

        Ok(MargMeasurements { accel,
                              gyro,
                              temp,
                              mag })
    }

    fn scale_and_correct_mag<N>(&self,
                                buffer: GenericArray<u8, N>,
                                offset: usize)
                                -> Vector3<f32>
        where N: ArrayLength<u8>
    {
        let resolution = self.mag_scale.resolution();
        let mut fraw: Vector3<f32> =
            convert(self.to_vector_inverted(buffer, offset));
        fraw *= resolution;
        fraw.component_mul_assign(&self.mag_sensitivity_adjustments);
        fraw
    }

    /// Reads and returns raw unscaled Magnetometer measurements (LSB).
    pub fn unscaled_mag(&mut self) -> Result<Vector3<i16>, E> {
        let buffer = self.dev.read_xyz()?;
        Ok(self.to_vector_inverted(buffer, 0))
    }

    /// Read and returns Magnetometer measurements scaled, adjusted for factory
    /// sensitivities, and converted to Teslas.
    pub fn mag(&mut self) -> Result<Vector3<f32>, E> {
        let buffer = self.dev.read_xyz()?;
        Ok(self.scale_and_correct_mag(buffer, 0))
    }

    /// Returns raw mag sensitivity adjustments
    pub fn raw_mag_sensitivity_adjustments(&self) -> Vector3<u8> {
        self.raw_mag_sensitivity_adjustments
    }

    /// Returns mag sensitivity adjustments
    pub fn mag_sensitivity_adjustments(&self) -> Vector3<f32> {
        self.mag_sensitivity_adjustments
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

// Any device, any mode
impl<E, DEV, MODE> Mpu9250<DEV, MODE> where DEV: Device<Error = E>
{
    fn init_mpu<D>(&mut self, delay: &mut D) -> Result<(), E>
        where D: DelayMs<u8>
    {
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
                         _mode })
        } else {
            Err(Error::ReInitError)
        }
    }

    fn scale_accel<N>(&self,
                      buffer: GenericArray<u8, N>,
                      offset: usize)
                      -> Vector3<f32>
        where N: ArrayLength<u8>
    {
        let resolution = self.accel_scale.resolution();
        let scale = G * resolution;
        let mut fraw: Vector3<f32> = convert(self.to_vector(buffer, offset));
        fraw *= scale;
        fraw
    }

    fn scale_gyro<N>(&self,
                     buffer: GenericArray<u8, N>,
                     offset: usize)
                     -> Vector3<f32>
        where N: ArrayLength<u8>
    {
        let resolution = self.gyro_scale.resolution();
        let scale = PI_180 * resolution;
        let mut fraw: Vector3<f32> = convert(self.to_vector(buffer, offset));
        fraw *= scale;
        fraw
    }

    fn scale_temp<N>(&self, buffer: GenericArray<u8, N>, offset: usize) -> f32
        where N: ArrayLength<u8>
    {
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

    /// Reads and returns unscaled accelerometer measurements (LSB).
    pub fn unscaled_accel(&mut self) -> Result<Vector3<i16>, E> {
        let buffer = self.dev.read_many::<U7>(Register::ACCEL_XOUT_H)?;
        Ok(self.to_vector(buffer, 0))
    }

    /// Reads and returns accelerometer measurements scaled and converted to g.
    pub fn accel(&mut self) -> Result<Vector3<f32>, E> {
        let buffer = self.dev.read_many::<U7>(Register::ACCEL_XOUT_H)?;
        Ok(self.scale_accel(buffer, 0))
    }

    /// Reads and returns unsacled Gyroscope measurements (LSB).
    pub fn unscaled_gyro(&mut self) -> Result<Vector3<i16>, E> {
        let buffer = self.dev.read_many::<U7>(Register::GYRO_XOUT_H)?;
        Ok(self.to_vector(buffer, 0))
    }

    /// Reads and returns gyroscope measurements scaled and converted to rad/s.
    pub fn gyro(&mut self) -> Result<Vector3<f32>, E> {
        let buffer = self.dev.read_many::<U7>(Register::GYRO_XOUT_H)?;
        Ok(self.scale_gyro(buffer, 0))
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
        let buffer = self.dev.read_many::<U3>(Register::TEMP_OUT_H)?;
        let t = (u16(buffer[1]) << 8) | u16(buffer[2]);
        Ok(t as i16)
    }

    /// Reads and returns adjusted temperature measurements converted to C.
    pub fn temp(&mut self) -> Result<f32, E> {
        let buffer = self.dev.read_many::<U3>(Register::TEMP_OUT_H)?;
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
                             -> Result<Vector3<f32>, Error<E>>
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
        let buffer = self.dev.read_many::<U3>(Register::FIFO_COUNT_H)?;
        let fifo_count = ((u16(buffer[1]) << 8) | u16(buffer[2])) as i16;
        // Aim for at least half
        // How many sets of full gyro and accelerometer data for averaging
        let packet_count = i32(fifo_count / 12);
        if packet_count < 20 {
            return Err(Error::CalibrationError);
        }
        let mut accel_biases: Vector3<i32> = Vector3::zeros();
        let mut gyro_biases: Vector3<i32> = Vector3::zeros();
        let mut accel_temp: Vector3<i32>;
        let mut gyro_temp: Vector3<i32>;
        for _ in 0..packet_count {
            let buffer = self.dev.read_many::<U13>(Register::FIFO_RW)?;
            accel_temp = convert(self.to_vector(buffer, 0));
            gyro_temp = convert(self.to_vector(buffer, 6));
            accel_biases += accel_temp;
            gyro_biases += gyro_temp;
        }
        accel_biases /= packet_count;
        gyro_biases /= packet_count;

        // Construct the gyro biases and push them to the hardware gyro bias
        // registers, which are reset to zero upon device startup.
        // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias
        // input format.
        // Biases are additive, so change sign on
        // calculated average gyro biases
        gyro_biases /= -4;
        self.dev.write(Register::XG_OFFSET_H,
                        ((gyro_biases.x >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::XG_OFFSET_L, (gyro_biases.x & 0xFF) as u8)?;
        self.dev.write(Register::YG_OFFSET_H,
                        ((gyro_biases.y >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::YG_OFFSET_L, (gyro_biases.y & 0xFF) as u8)?;
        self.dev.write(Register::ZG_OFFSET_H,
                        ((gyro_biases.z >> 8) & 0xFF) as u8)?;
        self.dev.write(Register::ZG_OFFSET_L, (gyro_biases.z & 0xFF) as u8)?;

        // Compute accelerometer biases to be returned
        let resolution = self.accel_scale.resolution();
        let scale = G * resolution;
        let mut faccel_biases: Vector3<f32> = convert(accel_biases);
        faccel_biases *= scale;

        // Set original values back and re-init device
        self.gyro_scale = orig_gyro_scale;
        self.accel_scale = orig_accel_scale;
        self.gyro_temp_data_rate = orig_gyro_temp_data_rate;
        self.accel_data_rate = orig_accel_data_rate;
        self.sample_rate_divisor = orig_sample_rate_divisor;
        self.init_mpu(delay)?;

        Ok(faccel_biases)
    }

    fn to_vector<N>(&self,
                    buffer: GenericArray<u8, N>,
                    offset: usize)
                    -> Vector3<i16>
        where N: ArrayLength<u8>
    {
        Vector3::new(((u16(buffer[offset + 1]) << 8) | u16(buffer[offset + 2]))
                     as i16,
                     ((u16(buffer[offset + 3]) << 8) | u16(buffer[offset + 4]))
                     as i16,
                     ((u16(buffer[offset + 5]) << 8) | u16(buffer[offset + 6]))
                     as i16)
    }

    fn to_vector_inverted<N>(&self,
                             buffer: GenericArray<u8, N>,
                             offset: usize)
                             -> Vector3<i16>
        where N: ArrayLength<u8>
    {
        Vector3::new(((u16(buffer[offset + 2]) << 8) + u16(buffer[offset + 1]))
                     as i16,
                     ((u16(buffer[offset + 4]) << 8) + u16(buffer[offset + 3]))
                     as i16,
                     ((u16(buffer[offset + 6]) << 8) + u16(buffer[offset + 5]))
                     as i16)
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

/// Unscaled IMU measurements (LSB)
#[derive(Clone, Copy, Debug)]
pub struct UnscaledImuMeasurements {
    /// Accelerometer measurements (LSB)
    pub accel: Vector3<i16>,
    /// Gyroscope measurements (LSB)
    pub gyro: Vector3<i16>,
    /// Temperature sensor measurement (LSB)
    pub temp: i16,
}

/// Scaled IMU measurements converted to units
#[derive(Clone, Copy, Debug)]
pub struct ImuMeasurements {
    /// Accelerometer measurements (g)
    pub accel: Vector3<f32>,
    /// Gyroscope measurements (rad/s)
    pub gyro: Vector3<f32>,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

/// Unscaled MARG measurements (LSB)
#[derive(Copy, Clone, Debug)]
pub struct UnscaledMargMeasurements {
    /// Accelerometer measurements (LSB)
    pub accel: Vector3<i16>,
    /// Gyroscope measurements (LSB)
    pub gyro: Vector3<i16>,
    /// Magnetometer measurements (LSB)
    pub mag: Vector3<i16>,
    /// Temperature sensor measurement (LSB)
    pub temp: i16,
}

/// MARG measurements scaled with respective scales and converted
/// to appropriate units.
#[derive(Copy, Clone, Debug)]
pub struct MargMeasurements {
    /// Accelerometer measurements (g)
    pub accel: Vector3<f32>,
    /// Gyroscope measurements (rad/s)
    pub gyro: Vector3<f32>,
    /// Magnetometer measurements (T, teslas)
    pub mag: Vector3<f32>,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

fn transpose<T, E>(o: Option<Result<T, E>>) -> Result<Option<T>, E> {
    match o {
        Some(Ok(x)) => Ok(Some(x)),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}
