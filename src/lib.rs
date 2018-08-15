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
#![deny(warnings)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;

mod ak8963;

use core::default::Default;
use core::marker::PhantomData;
use core::mem;
use core::ops::Mul;

use cast::{f32, u16};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

use hal::blocking::delay::DelayMs;
use hal::blocking::spi;
use hal::digital::OutputPin;
use hal::spi::{Mode, Phase, Polarity};

/// Marker trait for mode
trait MpuMode {}
/// Accelerometer + Gyroscope + Temperature Sensor
pub struct Imu;
impl MpuMode for Imu {}

/// Magnetometer + Accelerometer + Gyroscope + Temperature Sensor
pub struct Marg;
impl MpuMode for Marg {}

/// MPU9250 driver
pub struct Mpu9250<SPI, NCS, MODE> {
    spi: SPI,
    ncs: NCS,
    gyro_scale: GyroScale,
    accel_scale: AccelScale,
    mag_scale: MagScale,
    raw_mag_sensitivity_adjustments: [u8; 3],
    mag_sensitivity_adjustments: F32x3,
    _mode: PhantomData<MODE>,
}

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
const MMODE: u8 = 0x06;

const G: f32 = 9.807;
const PI_180: f32 = core::f32::consts::PI / 180.;
const TEMP_SENSITIVITY: f32 = 333.87;
const TEMP_DIFF: f32 = 21.0;
const TEMP_ROOM_OFFSET: f32 = 0.0;

impl<E, SPI, NCS> Mpu9250<SPI, NCS, Imu>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    /// Creates a new [`Imu`] driver from a SPI peripheral and a NCS pin with
    /// default [`Accel scale`], [`Gyro scale`], [`DLPF`], and
    /// with no sample rate divisor.
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`DLPF`]: ./enum.Dlpf.html
    pub fn imu_default<D>(spi: SPI, ncs: NCS, delay: &mut D) -> Result<Self, E>
        where D: DelayMs<u8>
    {
        Mpu9250::imu(spi, ncs, delay, None, None, None, None)
    }

    /// Creates a new Imu driver from a SPI peripheral and a NCS pin with
    /// optionally configured [`Accel scale`], [`Gyro scale`],
    /// [`DLPF`], and sample rate divisor.
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`DLPF`]: ./enum.Dlpf.html
    pub fn imu<D>(spi: SPI,
                  ncs: NCS,
                  delay: &mut D,
                  gyro_scale: Option<GyroScale>,
                  accel_scale: Option<AccelScale>,
                  digital_low_pass_filter_mode: Option<Dlpf>,
                  sample_rate_divisor: Option<u8>)
                  -> Result<Self, E>
        where D: DelayMs<u8>
    {
        let mut mpu9250 =
            Mpu9250 { spi,
                      ncs,
                      gyro_scale: gyro_scale.unwrap_or_default(),
                      accel_scale: accel_scale.unwrap_or_default(),
                      mag_scale: MagScale::default(),
                      raw_mag_sensitivity_adjustments: [0, 0, 0],
                      mag_sensitivity_adjustments: F32x3::default(),
                      _mode: PhantomData, };
        mpu9250.init_mpu(digital_low_pass_filter_mode,
                         sample_rate_divisor,
                         delay)?;
        Ok(mpu9250)
    }

    fn init_mpu<D>(&mut self,
                   dlpf: Option<Dlpf>,
                   smplrt_div: Option<u8>,
                   delay: &mut D)
                   -> Result<(), E>
        where D: DelayMs<u8>
    {
        // wake up device
        self.write(Register::PWR_MGMT_1, 0x00)?; // Clear sleep mode bit (6), enable all sensors
        delay.delay_ms(100); // Wait for all registers to reset

        // get stable time source
        // Auto select clock source to be PLL gyroscope reference if ready else
        self.write(Register::PWR_MGMT_1, 0x01)?;
        delay.delay_ms(200);

        // Set gyroscope full scale range
        let gscale = self.gyro_scale;
        self.gyro_scale(gscale)?;
        // Disable FSYNC and set thermometer and gyro bandwidth
        let dlpf = dlpf.unwrap_or_default();
        self.gyro_filter(dlpf.clone())?;

        // Set accelerometer full-scale range configuration
        let ascale = self.accel_scale;
        self.accel_scale(ascale)?;
        // Set accelerometer sample rate configuration
        self.accel_filter(dlpf)?;

        // Set smplrt_div if present
        if let Some(sample_rate_div) = smplrt_div {
            self.sample_rate_divisor(sample_rate_div)?;
        }

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level
        // HIGH until interrupt cleared, clear on read of INT_STATUS,
        // and enable I2C_BYPASS_EN so
        // additional chips can join the I2C bus
        self.write(Register::INT_PIN_CFG, 0x12)?; // INT is 50 microsecond pulse and any read to clear
        self.write(Register::INT_ENABLE, 0x01)?; // Enable data ready (bit 0) interrupt
        delay.delay_ms(100);
        // TODO: check .who_am_i and return Err

        Ok(())
    }

    /// Reads and returns raw unscaled Accelerometer + Gyroscope + Thermometer
    /// measurements (LSB).
    pub fn unscaled_all(&mut self) -> Result<UnscaledImuMeasurements, E> {
        let buffer = self.read_many::<U15>(Register::ACCEL_XOUT_H)?;

        let accel =
            I16x3 { x: ((u16(buffer[1]) << 8) | u16(buffer[2])) as i16,
                    y: ((u16(buffer[3]) << 8) | u16(buffer[4])) as i16,
                    z: ((u16(buffer[5]) << 8) | u16(buffer[6])) as i16, };

        let temp = (u16(buffer[7]) << 8) | u16(buffer[8]);

        let gyro =
            I16x3 { x: ((u16(buffer[9]) << 8) | u16(buffer[10])) as i16,
                    y: ((u16(buffer[11]) << 8) | u16(buffer[12])) as i16,
                    z: ((u16(buffer[13]) << 8) | u16(buffer[14])) as i16, };

        Ok(UnscaledImuMeasurements { accel,
                                     gyro,
                                     temp, })
    }

    /// Reads and returns Accelerometer + Gyroscope + Thermometer
    /// measurements scaled and converted to respective units.
    pub fn all(&mut self) -> Result<ImuMeasurements, E> {
        let buffer = self.read_many::<U15>(Register::ACCEL_XOUT_H)?;

        let accel = self.scale_accel(buffer, 0);
        let temp = self.scale_temp(buffer, 6);
        let gyro = self.scale_gyro(buffer, 8);

        Ok(ImuMeasurements { accel,
                             gyro,
                             temp, })
    }
}

impl<E, SPI, NCS> Mpu9250<SPI, NCS, Marg>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    /// Creates a new [`Marg`] driver from a SPI peripheral and a NCS pin with
    /// default [`Accel scale`], [`Gyro scale`], [`Mag scale`], [`DLPF`], and
    /// with no sample rate divisor.
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`DLPF`]: ./enum.Dlpf.html
    pub fn marg_default<D>(spi: SPI, ncs: NCS, delay: &mut D) -> Result<Self, E>
        where D: DelayMs<u8>
    {
        Mpu9250::marg(spi, ncs, delay, None, None, None, None, None)
    }

    /// Creates a new MARG driver from a SPI peripheral and a NCS pin with
    /// optionally configured [`Accel scale`], [`Gyro scale`],
    /// [`Mag scale`], [`DLPF`], and sample rate divisor.
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Gyro scale`]: ./enum.GyroScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`DLPF`]: ./enum.Dlpf.html
    pub fn marg<D>(spi: SPI,
                   ncs: NCS,
                   delay: &mut D,
                   gyro_scale: Option<GyroScale>,
                   accel_scale: Option<AccelScale>,
                   mag_scale: Option<MagScale>,
                   dlpf: Option<Dlpf>,
                   smplrt_div: Option<u8>)
                   -> Result<Self, E>
        where D: DelayMs<u8>
    {
        let mpu9250_in = Mpu9250::imu(spi,
                                      ncs,
                                      delay,
                                      gyro_scale,
                                      accel_scale,
                                      dlpf,
                                      smplrt_div)?;
        let mut mpu9250: Mpu9250<SPI, NCS, Marg> =
            Mpu9250 { spi: mpu9250_in.spi,
                      ncs: mpu9250_in.ncs,
                      gyro_scale: mpu9250_in.gyro_scale,
                      accel_scale: mpu9250_in.accel_scale,
                      mag_scale: mag_scale.unwrap_or_default(),
                      raw_mag_sensitivity_adjustments: [0, 0, 0],
                      mag_sensitivity_adjustments: F32x3::default(),
                      _mode: PhantomData, };
        mpu9250.init_ak8963(mag_scale, delay)?;
        Ok(mpu9250)
    }

    fn init_ak8963<D>(&mut self,
                      mag_scale: Option<MagScale>,
                      delay: &mut D)
                      -> Result<(), E>
        where D: DelayMs<u8>
    {
        // isolate the auxiliary master I2C bus (AUX_CL, AUX_DA)
        // disable the slave I2C bus, make serial interface SPI only
        // reset the master I2C bus
        self.write(Register::USER_CTRL, 0x32)?;

        // First extract the factory calibration for each magnetometer axis
        // Power down magnetometer
        self.ak8963_write(ak8963::Register::CNTL, 0x00)?;
        delay.delay_ms(10);
        // Enter Fuse ROM access mode
        self.ak8963_write(ak8963::Register::CNTL, 0x0F)?;
        delay.delay_ms(10);
        // Read the x-, y-, and z-axis calibration values
        let mag_x_bias = self.ak8963_read(ak8963::Register::ASAX)?;
        let mag_y_bias = self.ak8963_read(ak8963::Register::ASAY)?;
        let mag_z_bias = self.ak8963_read(ak8963::Register::ASAZ)?;
        // Return x-axis sensitivity adjustment values, etc.
        self.raw_mag_sensitivity_adjustments =
            [mag_x_bias, mag_y_bias, mag_z_bias];
        self.mag_sensitivity_adjustments =
            F32x3 { x: ((self.raw_mag_sensitivity_adjustments[0] - 128)
                        as f32)
                       / 256.
                       + 1.,
                    y: ((self.raw_mag_sensitivity_adjustments[1] - 128)
                        as f32)
                       / 256.
                       + 1.,
                    z: ((self.raw_mag_sensitivity_adjustments[2] - 128)
                        as f32)
                       / 256.
                       + 1., };
        self.ak8963_write(ak8963::Register::CNTL, 0x00)?;
        delay.delay_ms(10);
        // Set magnetometer data resolution and sample ODR
        self.mag_scale(mag_scale.unwrap_or_default())?;
        delay.delay_ms(10);

        // set aux I2C frequency to 400 KHz (should be configurable?)
        self.write(Register::I2C_MST_CTRL, 0x0d)?;

        // XXX: check who_am_i and return error
        // debug_assert_eq!(self.ak8963_read(ak8963::Register::WHO_AM_I)?,
        // 0x48); // FIXME this hangs when compiled in release mode
        // self.ak8963_write(ak8963::Register::CNTL_2, 0x01)?;

        delay.delay_ms(10);

        // configure sampling of magnetometer
        self.write(Register::I2C_SLV0_ADDR, ak8963::I2C_ADDRESS | ak8963::R)?;
        self.write(Register::I2C_SLV0_REG, ak8963::Register::XOUT_L.addr())?;
        self.write(Register::I2C_SLV0_CTRL, 0x87)?;

        delay.delay_ms(10);
        Ok(())
    }

    /// Reads and returns raw unscaled Accelerometer + Gyroscope + Thermometer
    /// + Magnetometer measurements (LSB).
    pub fn unscaled_all(&mut self) -> Result<UnscaledMargMeasurements, E> {
        let buffer = self.read_many::<U21>(Register::ACCEL_XOUT_H)?;

        let accel =
            I16x3 { x: ((u16(buffer[1]) << 8) | u16(buffer[2])) as i16,
                    y: ((u16(buffer[3]) << 8) | u16(buffer[4])) as i16,
                    z: ((u16(buffer[5]) << 8) | u16(buffer[6])) as i16, };

        let temp = (u16(buffer[7]) << 8) | u16(buffer[8]);

        let gyro =
            I16x3 { x: ((u16(buffer[9]) << 8) | u16(buffer[10])) as i16,
                    y: ((u16(buffer[11]) << 8) | u16(buffer[12])) as i16,
                    z: ((u16(buffer[13]) << 8) | u16(buffer[14])) as i16, };

        let mag =
            I16x3 { x: ((u16(buffer[16]) << 8) | u16(buffer[15])) as i16,
                    y: ((u16(buffer[18]) << 8) | u16(buffer[17])) as i16,
                    z: ((u16(buffer[20]) << 8) | u16(buffer[19])) as i16, };

        Ok(UnscaledMargMeasurements { accel,
                                      gyro,
                                      temp,
                                      mag, })
    }

    /// Reads and returns Accelerometer + Gyroscope + Thermometer + Magnetometer
    /// measurements scaled and converted to respective units.
    pub fn all(&mut self) -> Result<MargMeasurements, E> {
        let buffer = self.read_many::<U21>(Register::ACCEL_XOUT_H)?;

        let accel = self.scale_accel(buffer, 0);
        let temp = self.scale_temp(buffer, 6);
        let gyro = self.scale_gyro(buffer, 8);
        let mag = self.scale_and_correct_mag(buffer, 14);

        Ok(MargMeasurements { accel,
                              gyro,
                              temp,
                              mag, })
    }

    fn scale_and_correct_mag<N>(&self,
                                buffer: GenericArray<u8, N>,
                                offset: usize)
                                -> F32x3
        where N: ArrayLength<u8>
    {
        let resolution = self.mag_scale.resolution();
        let scaled = self.scale_x3_measurement(buffer, offset, resolution);
        scaled * self.mag_sensitivity_adjustments
    }

    /// Reads and returns raw unscaled Magnetometer measurements (LSB).
    pub fn unscaled_mag(&mut self) -> Result<I16x3, E> {
        let buffer = self.read_many::<U8>(Register::EXT_SENS_DATA_00)?;

        Ok(I16x3 { x: ((u16(buffer[2]) << 8) + u16(buffer[1])) as i16,
                   y: ((u16(buffer[4]) << 8) + u16(buffer[3])) as i16,
                   z: ((u16(buffer[6]) << 8) + u16(buffer[5])) as i16, })
    }

    /// Read and returns Magnetometer measurements scaled, adjusted for factory
    /// sensitivities, and converted to Teslas.
    pub fn mag(&mut self) -> Result<F32x3, E> {
        let buffer = self.read_many::<U8>(Register::EXT_SENS_DATA_00)?;
        Ok(self.scale_and_correct_mag(buffer, 0))
    }

    /// Returns raw mag sensitivity adjustments
    pub fn raw_mag_sensitivity_adjustments(&self) -> [u8; 3] {
        self.raw_mag_sensitivity_adjustments
    }

    /// Returns mag sensitivity adjustments
    pub fn mag_sensitivity_adjustments(&self) -> F32x3 {
        self.mag_sensitivity_adjustments
    }

    /// Sets magnetrometer full reading scale ([`MagScale`])
    ///
    /// [`Mag scale`]: ./enum.MagScale.html
    pub fn mag_scale(&mut self, scale: MagScale) -> Result<(), E> {
        // Set magnetometer data resolution and sample ODR
        self.ak8963_write(ak8963::Register::CNTL, (scale as u8) << 4 | MMODE)?;
        Ok(())
    }

    /// Reads the AK8963 (magnetometer) WHO_AM_I register; should return `0x48`
    pub fn ak8963_who_am_i(&mut self) -> Result<u8, E> {
        self.ak8963_read(ak8963::Register::WHO_AM_I)
    }
}

impl<E, SPI, NCS, MODE> Mpu9250<SPI, NCS, MODE>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    fn scale_x3_measurement<N>(&self,
                               buffer: GenericArray<u8, N>,
                               offset: usize,
                               scale: f32)
                               -> F32x3
        where N: ArrayLength<u8>
    {
        let x = (u16(buffer[offset + 1]) << 8) + u16(buffer[offset + 2]);
        let y = (u16(buffer[offset + 3]) << 8) + u16(buffer[offset + 4]);
        let z = (u16(buffer[offset + 5]) << 8) + u16(buffer[offset + 6]);
        F32x3 { x: f32(x) * scale,
                y: f32(y) * scale,
                z: f32(z) * scale, }
    }

    fn scale_accel<N>(&self,
                      buffer: GenericArray<u8, N>,
                      offset: usize)
                      -> F32x3
        where N: ArrayLength<u8>
    {
        let resolution = self.accel_scale.resolution();
        let scale = G * resolution;
        self.scale_x3_measurement(buffer, offset, scale)
    }

    fn scale_gyro<N>(&self, buffer: GenericArray<u8, N>, offset: usize) -> F32x3
        where N: ArrayLength<u8>
    {
        let resolution = self.gyro_scale.resolution();
        let scale = PI_180 * resolution;
        self.scale_x3_measurement(buffer, offset, scale)
    }

    fn scale_temp<N>(&self, buffer: GenericArray<u8, N>, offset: usize) -> f32
        where N: ArrayLength<u8>
    {
        let t = f32((u16(buffer[offset + 1]) << 8) | u16(buffer[offset + 2]));
        (t - TEMP_ROOM_OFFSET) / TEMP_SENSITIVITY + TEMP_DIFF
    }

    /// Reads and returns unscaled accelerometer measurements (LSB).
    pub fn unscaled_accel(&mut self) -> Result<I16x3, E> {
        let buffer = self.read_many::<U7>(Register::ACCEL_XOUT_H)?;

        Ok(I16x3 { x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
                   y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
                   z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16, })
    }

    /// Reads and returns accelerometer measurements scaled and converted to g.
    pub fn accel(&mut self) -> Result<F32x3, E> {
        let buffer = self.read_many::<U7>(Register::ACCEL_XOUT_H)?;
        Ok(self.scale_accel(buffer, 0))
    }

    /// Applies a digital low pass filter to the accelerometer data ([`Dlpf`]).
    ///
    /// [`DLPF`]: ./enum.Dlpf.html
    pub fn accel_filter(&mut self, dlpf: Dlpf) -> Result<(), E> {
        self.write(Register::ACCEL_CONFIG_2, dlpf as u8)?;

        Ok(())
    }

    /// Sets accelerometer full reading scale ([`Accel scale`]).
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    pub fn accel_scale(&mut self, scale: AccelScale) -> Result<(), E> {
        self.modify(Register::ACCEL_CONFIG, |r| {
                (r & !0b11000) | ((scale as u8) << 3)
            })?;
        Ok(())
    }

    /// Reads and returns unsacled Gyroscope measurements (LSB).
    pub fn unscaled_gyro(&mut self) -> Result<I16x3, E> {
        let buffer = self.read_many::<U7>(Register::GYRO_XOUT_H)?;

        Ok(I16x3 { x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
                   y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
                   z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16, })
    }

    /// Reads and returns gyroscope measurements scaled and converted to rad/s.
    pub fn gyro(&mut self) -> Result<F32x3, E> {
        let buffer = self.read_many::<U7>(Register::GYRO_XOUT_H)?;
        Ok(self.scale_gyro(buffer, 0))
    }

    /// Applies a digital low pass filter to the gyroscope data ([`Dlpf`]).
    ///
    /// [`DLPF`]: ./enum.Dlpf.html
    pub fn gyro_filter(&mut self, dlpf: Dlpf) -> Result<(), E> {
        self.modify(Register::CONFIG, |r| (r & !0b11) | dlpf as u8)?;

        Ok(())
    }

    /// Sets gyroscope full reading scale ([`Gyro scale`]).
    ///
    /// [`Gyro scale`]: ./enum.GyroScale.html
    pub fn gyro_scale(&mut self, scale: GyroScale) -> Result<(), E> {
        self.gyro_scale = scale;
        self.modify(Register::GYRO_CONFIG, |r| {
                (r & !0b11000) | ((scale as u8) << 3)
            })?;

        Ok(())
    }

    /// Reads and returns raw unscaled temperature sensor measurements (LSB).
    pub fn raw_temp(&mut self) -> Result<i16, E> {
        let buffer = self.read_many::<U3>(Register::TEMP_OUT_H)?;
        let t = (u16(buffer[1]) << 8) | u16(buffer[2]);
        Ok(t as i16)
    }

    /// Reads and returns adjusted temperature measurements converted to C.
    pub fn temp(&mut self) -> Result<f32, E> {
        let buffer = self.read_many::<U3>(Register::TEMP_OUT_H)?;
        Ok(self.scale_temp(buffer, 0))
    }

    /// Clears Fchoice bits and sets rate divisor (SMPLRT_DIV).
    pub fn sample_rate_divisor(&mut self, smplrt_div: u8) -> Result<(), E> {
        // Configure Gyro and Thermometer
        self.modify(Register::GYRO_CONFIG, |v| {
                // c = c & ~0xE0; // Clear self-test bits [7:5]
                // Clear Fchoice bits [1:0], to be able to set SMPLRT_DIV
                v & !0x03
            })?;
        self.write(Register::SMPLRT_DIV, smplrt_div)?;
        Ok(())
    }

    /// Destroys the driver recovering the SPI peripheral and the NCS pin
    pub fn release(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }

    /// Reads the WHO_AM_I register; should return `0x71`
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        self.read(Register::WHO_AM_I)
    }

    fn ak8963_read(&mut self, reg: ak8963::Register) -> Result<u8, E> {
        self.write(Register::I2C_SLV4_ADDR, ak8963::I2C_ADDRESS | ak8963::R)?;
        self.write(Register::I2C_SLV4_REG, reg.addr())?;

        // start transfer
        self.write(Register::I2C_SLV4_CTRL, 0x80)?;

        // wait until transfer is over
        while self.read(Register::I2C_MST_STATUS)? & (1 << 6) == 0 {}

        self.read(Register::I2C_SLV4_DI)
    }

    fn ak8963_write(&mut self,
                    reg: ak8963::Register,
                    val: u8)
                    -> Result<(), E> {
        self.write(Register::I2C_SLV4_ADDR, ak8963::I2C_ADDRESS | ak8963::W)?;
        self.write(Register::I2C_SLV4_REG, reg.addr())?;
        self.write(Register::I2C_SLV4_DO, val)?;

        // start transfer
        self.write(Register::I2C_SLV4_CTRL, 0x80)?;

        // wait until transfer is over
        while self.read(Register::I2C_MST_STATUS)? & (1 << 6) == 0 {}

        Ok(())
    }

    fn modify<F>(&mut self, reg: Register, f: F) -> Result<(), E>
        where F: FnOnce(u8) -> u8
    {
        let r = self.read(reg)?;
        self.write(reg, f(r))?;
        Ok(())
    }

    fn read(&mut self, reg: Register) -> Result<u8, E> {
        let buffer = self.read_many::<U2>(reg)?;
        Ok(buffer[1])
    }

    fn read_many<N>(&mut self, reg: Register) -> Result<GenericArray<u8, N>, E>
        where N: ArrayLength<u8>
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.read_address();
            self.ncs.set_low();
            self.spi.transfer(slice)?;
            self.ncs.set_high();
        }

        Ok(buffer)
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.write(&[reg.write_address(), val])?;
        self.ncs.set_high();
        Ok(())
    }
}

impl<SPI, NCS, MODE> Mpu9250<SPI, NCS, MODE> {
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
                              phase: Phase::CaptureOnSecondTransition, };

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
enum Register {
    ACCEL_CONFIG = 0x1c,
    ACCEL_CONFIG_2 = 0x1d,
    ACCEL_XOUT_H = 0x3b,
    ACCEL_YOUT_H = 0x3d,
    CONFIG = 0x1a,
    EXT_SENS_DATA_00 = 0x49,
    EXT_SENS_DATA_01 = 0x4a,
    EXT_SENS_DATA_02 = 0x4b,
    EXT_SENS_DATA_03 = 0x4c,
    EXT_SENS_DATA_04 = 0x4d,
    GYRO_CONFIG = 0x1b,
    SMPLRT_DIV = 0x19,
    GYRO_XOUT_H = 0x43,
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
    PWR_MGMT_1 = 0x6b,
    PWR_MGMT_2 = 0x6c,
    TEMP_OUT_H = 0x41,
    USER_CTRL = 0x6a,
    WHO_AM_I = 0x75,
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

/// XYZ triple of i16
#[derive(Clone, Copy, Debug)]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Vector in 3D space
#[derive(Clone, Copy, Debug)]
pub struct F32x3 {
    /// X component
    pub x: f32,
    /// Y component
    pub y: f32,
    /// Z component
    pub z: f32,
}

impl Mul for F32x3 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        F32x3 { x: self.x * rhs.x,
                y: self.y * rhs.y,
                z: self.z * rhs.z, }
    }
}

impl Default for F32x3 {
    fn default() -> Self {
        F32x3 { x: 0.,
                y: 0.,
                z: 0., }
    }
}

/// Unscaled IMU measurements (LSB)
#[derive(Clone, Copy, Debug)]
pub struct UnscaledImuMeasurements {
    /// Accelerometer measurements (LSB)
    pub accel: I16x3,
    /// Gyroscope measurements (LSB)
    pub gyro: I16x3,
    /// Temperature sensor measurement (LSB)
    pub temp: u16,
}

/// Scaled IMU measurements converted to units
#[derive(Clone, Copy, Debug)]
pub struct ImuMeasurements {
    /// Accelerometer measurements (g)
    pub accel: F32x3,
    /// Gyroscope measurements (rad/s)
    pub gyro: F32x3,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

/// Unscaled MARG measurements (LSB)
#[derive(Copy, Clone, Debug)]
pub struct UnscaledMargMeasurements {
    /// Accelerometer measurements (LSB)
    pub accel: I16x3,
    /// Gyroscope measurements (LSB)
    pub gyro: I16x3,
    /// Magnetometer measurements (LSB)
    pub mag: I16x3,
    /// Temperature sensor measurement (LSB)
    pub temp: u16,
}

/// MARG measurements scaled with respective scales and converted
/// to appropriate units.
#[derive(Copy, Clone, Debug)]
pub struct MargMeasurements {
    /// Accelerometer measurements (g)
    pub accel: F32x3,
    /// Gyroscope measurements (rad/s)
    pub gyro: F32x3,
    /// Magnetometer measurements (T, teslas)
    pub mag: F32x3,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

/// Digital low pass filter configuration; default:
/// Accelerometer = 41 Hz, Gyroscope: 41 Hz, Temperature sensor = 42 Hz.
#[derive(Copy, Clone, Debug)]
pub enum Dlpf {
    /// Accelerometer = 460 Hz, Gyroscope: 250 Hz, Temperature sensor = 4000 Hz
    _0 = 0,
    /// Accelerometer = 184 Hz, Gyroscope: 184 Hz, Temperature sensor = 188 Hz
    _1 = 1,
    /// Accelerometer = 92 Hz, Gyroscope: 92 Hz, Temperature sensor = 98 Hz
    _2 = 2,
    /// Accelerometer = 41 Hz, Gyroscope: 41 Hz, Temperature sensor = 42 Hz
    _3 = 3,
    /// Accelerometer = 20 Hz, Gyroscope: 20 Hz, Temperature sensor = 20 Hz
    _4 = 4,
    /// Accelerometer = 10 Hz, Gyroscope: 10 Hz, Temperature sensor = 10 Hz
    _5 = 5,
    /// Accelerometer = 5 Hz, Gyroscope: 5 Hz, Temperature sensor = 5 Hz
    _6 = 6,
    /// Accelerometer = 460 Hz, Gyroscope: 3600 Hz, Temperature sensor = 4000
    /// Hz
    _7 = 7,
}
impl Default for Dlpf {
    fn default() -> Self {
        Dlpf::_3
    }
}

/// Sensor reading scale
pub trait FullScale {
    /// Returns scale resolutino
    fn resolution(&self) -> f32;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Gyroscope reading full scale configuration; default: +250 dps.
pub enum GyroScale {
    /// +250 dps
    GFS_250DPS = 0,
    /// +500 dps
    GFS_500DPS,
    /// +1000 dps
    GFS_1000DPS,
    /// +2000 dps
    GFS_2000DPS,
}
impl FullScale for GyroScale {
    fn resolution(&self) -> f32 {
        match self {
            GyroScale::GFS_250DPS => 250.0 / 32768.0,
            GyroScale::GFS_500DPS => 500.0 / 32768.0,
            GyroScale::GFS_1000DPS => 1000.0 / 32768.0,
            GyroScale::GFS_2000DPS => 2000.0 / 32768.0,
        }
    }
}
impl Default for GyroScale {
    fn default() -> Self {
        GyroScale::GFS_250DPS
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Accelerometer reading full scale configuration, default: +2g.
pub enum AccelScale {
    /// +2g
    AFS_2G = 0,
    /// +4g
    AFS_4G,
    /// +8g
    AFS_8G,
    /// +16g
    AFS_16G,
}
impl FullScale for AccelScale {
    fn resolution(&self) -> f32 {
        match self {
            AccelScale::AFS_2G => 2.0 / 32768.0,
            AccelScale::AFS_4G => 4.0 / 32768.0,
            AccelScale::AFS_8G => 8.0 / 32768.0,
            AccelScale::AFS_16G => 16.0 / 32768.0,
        }
    }
}
impl Default for AccelScale {
    fn default() -> Self {
        AccelScale::AFS_2G
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Gyroscope reading full scale configuration; default: 0.6 mG per LSB
pub enum MagScale {
    /// 0.6 mG per LSB
    MFS_14BITS = 0,
    /// 0.15 mG per LSB
    MFS_16BITS,
}
impl FullScale for MagScale {
    fn resolution(&self) -> f32 {
        match self {
            MagScale::MFS_14BITS => 10. * 4912. / 8190.,
            MagScale::MFS_16BITS => 10. * 4912. / 32760.0,
        }
    }
}
impl Default for MagScale {
    fn default() -> Self {
        MagScale::MFS_14BITS
    }
}
