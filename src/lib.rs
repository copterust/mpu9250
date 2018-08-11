//! A WIP, no_std, generic driver for the MPU9250 (accelerometer + gyroscope + magnetometer IMU)
//!
//! # Connections
//!
//! - NCS
//! - SCL = SCK
//! - SDA = SDI = MOSI
//! - AD0 = SDO = MISO
//!
//! # References
//!
//! - [Product specification][1]
//!
//! [1]: https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//!
//! - [Register map][2]
//!
//! [2]: https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;

mod ak8963;

use core::any::{Any, TypeId};
use core::marker::PhantomData;
use core::mem;

use cast::u16;
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};
use hal::blocking::delay::DelayMs;
use hal::blocking::spi;
use hal::digital::OutputPin;
use hal::spi::{Mode, Phase, Polarity};

/// Accelerometer + Gyroscope
pub struct Imu;

/// Magnetometer + Accelerometer + Gyroscope
pub struct Marg;

/// MPU9250 driver
pub struct Mpu9250<SPI, NCS, MODE> {
    spi: SPI,
    ncs: NCS,
    mag_sensitivity_adj: F32x3,
    accel_resolution: f32,
    gyro_resolution: f32,
    mag_resolution: f32,
    _mode: PhantomData<MODE>,
}

#[allow(non_camel_case_types)]
/// GSCALE
pub enum Gscale {
    /// 250DPS
    GFS_250DPS = 0,
    /// 500DPS
    GFS_500DPS,
    /// 1000DPS
    GFS_1000DPS,
    /// 2000DPS
    GFS_2000DPS,
}
impl Gscale {
    fn resolution(&self) -> f32 {
        match self {
            Gscale::GFS_250DPS => 250.0 / 32768.0,
            Gscale::GFS_500DPS => 500.0 / 32768.0,
            Gscale::GFS_1000DPS => 1000.0 / 32768.0,
            Gscale::GFS_2000DPS => 2000.0 / 32768.0,
        }
    }
}
// default
const GSCALE: Gscale = Gscale::GFS_250DPS;

#[allow(non_camel_case_types)]
/// ASCALE
pub enum Ascale {
    /// 2G
    AFS_2G = 0,
    /// 4G
    AFS_4G,
    /// 8G
    AFS_8G,
    /// 16G
    AFS_16G,
}
impl Ascale {
    fn resolution(&self) -> f32 {
        match self {
            Ascale::AFS_2G => 2.0 / 32768.0,
            Ascale::AFS_4G => 4.0 / 32768.0,
            Ascale::AFS_8G => 8.0 / 32768.0,
            Ascale::AFS_16G => 16.0 / 32768.0,
        }
    }
}
// default
const ASCALE: Ascale = Ascale::AFS_2G;

#[allow(non_camel_case_types)]
/// MSCALE
pub enum Mscale {
    /// 0.6 mG per LSB
    MFS_14BITS = 0,
    /// 0.15 mG per LSB
    MFS_16BITS,
}
impl Mscale {
    fn resolution(&self) -> f32 {
        match self {
            Mscale::MFS_14BITS => 10. * 4912. / 8190.,
            Mscale::MFS_16BITS => 10. * 4912. / 32760.0,
        }
    }
}
// default
const MSCALE: Mscale = Mscale::MFS_16BITS;
// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
const MMODE: u8 = 0x06;

fn new<SPI, NCS, MODE, D, E>(
    spi: SPI,
    ncs: NCS,
    delay: &mut D,
) -> Result<Mpu9250<SPI, NCS, MODE>, E>
where
    D: DelayMs<u8>,
    MODE: Any,
    NCS: OutputPin,
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
{
    let mut mpu9250 = Mpu9250 {
        spi,
        ncs,
        mag_sensitivity_adj: F32x3 {
            x: 0.,
            y: 0.,
            z: 0.,
        },
        accel_resolution: ASCALE.resolution(),
        gyro_resolution: GSCALE.resolution(),
        mag_resolution: MSCALE.resolution(),
        _mode: PhantomData,
    };

    // wake up device
    mpu9250.write(Register::PWR_MGMT_1, 0x00)?; // Clear sleep mode bit (6), enable all sensors
    delay.delay_ms(100); // Wait for all registers to reset

    // get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    mpu9250.write(Register::PWR_MGMT_1, 0x01)?;
    delay.delay_ms(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    //           r fifo ext_sync_set  dlpf
    // 0x03 == 0b0 0    000           011
    // XXX: SHOULD BE CONFIGURABLE: see https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
    // and https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    mpu9250.write(Register::CONFIG, 0x03)?;

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate determined inset in CONFIG above
    mpu9250.write(Register::SMPLRT_DIV, 0x04)?;
    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    // get current GYRO_CONFIG register value
    let mut c: u8 = mpu9250.read(Register::GYRO_CONFIG)?;
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    // Clear Fchoice bits [1:0]
    c = c & !0x03;
    // Clear GFS bits [4:3]
    c = c & !0x18;
    // Set full scale range for the gyro
    c = c | (GSCALE as u8) << 3;
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    mpu9250.write(Register::GYRO_CONFIG, c)?; // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = mpu9250.read(Register::ACCEL_CONFIG)?;
    // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    // Clear AFS bits [4:3]
    c = c & !0x18;
    // Set full scale range for the accelerometer
    c = c | (ASCALE as u8) << 3;
    // Write new ACCEL_CONFIG register value
    mpu9250.write(Register::ACCEL_CONFIG, c)?;

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    // get current ACCEL_CONFIG2 register value
    c = mpu9250.read(Register::ACCEL_CONFIG_2)?;
    // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c & !0x0F;
    // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    c = c | 0x03;
    // Write new ACCEL_CONFIG2 register value
    mpu9250.write(Register::ACCEL_CONFIG_2, c)?;

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus
    mpu9250.write(Register::INT_PIN_CFG, 0x12)?; // INT is 50 microsecond pulse and any read to clear
    mpu9250.write(Register::INT_ENABLE, 0x01)?; // Enable data ready (bit 0) interrupt
    delay.delay_ms(100);

    if TypeId::of::<MODE>() == TypeId::of::<Marg>() {
        // isolate the auxiliary master I2C bus (AUX_CL, AUX_DA)
        // disable the slave I2C bus, make serial interface SPI only
        // reset the master I2C bus
        mpu9250.write(Register::USER_CTRL, 0x32)?;

        // First extract the factory calibration for each magnetometer axis
        // Power down magnetometer
        mpu9250.ak8963_write(ak8963::Register::CNTL, 0x00)?;
        delay.delay_ms(10);
        // Enter Fuse ROM access mode
        mpu9250.ak8963_write(ak8963::Register::CNTL, 0x0F)?;
        delay.delay_ms(10);
        // Read the x-, y-, and z-axis calibration values
        let mag_x_bias = mpu9250.ak8963_read(ak8963::Register::ASAX)?;
        let mag_y_bias = mpu9250.ak8963_read(ak8963::Register::ASAY)?;
        let mag_z_bias = mpu9250.ak8963_read(ak8963::Register::ASAZ)?;
        // Return x-axis sensitivity adjustment values, etc.
        mpu9250.mag_sensitivity_adj = F32x3 {
            x: ((mag_x_bias - 128) as f32) / 256. + 1.,
            y: ((mag_y_bias - 128) as f32) / 256. + 1.,
            z: ((mag_z_bias - 128) as f32) / 256. + 1.,
        };
        mpu9250.ak8963_write(ak8963::Register::CNTL, 0x00)?;
        delay.delay_ms(10);
        // Set magnetometer data resolution and sample ODR
        mpu9250.ak8963_write(ak8963::Register::CNTL, (MSCALE as u8) << 4 | MMODE)?;
        delay.delay_ms(10);

        // set aux I2C frequency to 400 KHz
        mpu9250.write(Register::I2C_MST_CTRL, 0x0d)?;

        // // sanity check that the aux I2C is working
        // debug_assert_eq!(mpu9250.ak8963_read(ak8963::Register::WIA)?, 0x48);

        // // software reset the magnetometer
        // // FIXME this hangs when compiled in release mode
        // mpu9250.ak8963_write(ak8963::Register::CNTL_2, 0x01)?;

        delay.delay_ms(10);

        // configure sampling of magnetometer
        mpu9250.write(Register::I2C_SLV0_ADDR, ak8963::I2C_ADDRESS | ak8963::R)?;
        mpu9250.write(Register::I2C_SLV0_REG, ak8963::Register::XOUT_L.addr())?;
        mpu9250.write(Register::I2C_SLV0_CTRL, 0x87)?;

        delay.delay_ms(10);
    }

    Ok(mpu9250)
}

impl<SPI, NCS, MODE> Mpu9250<SPI, NCS, MODE> {
    /// Returns accel resolution
    pub fn accel_resolution(&self) -> f32 {
        self.accel_resolution
    }

    /// Returns gyro resolution
    pub fn gyro_resolution(&self) -> f32 {
        self.gyro_resolution
    }

    /// Returns mag resolution
    pub fn mag_resolution(&self) -> f32 {
        self.mag_resolution
    }
}

impl<E, SPI, NCS> Mpu9250<SPI, NCS, Imu>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
{
    /// Creates a new IMU driver from a SPI peripheral and a NCS pin
    ///
    /// # Defaults
    ///
    /// - Accelerometer: 16 bits, range [-2g, +2g]
    /// - Gyroscope: 16 bits, range [-250 dps, +250 dps]
    pub fn imu<D>(spi: SPI, ncs: NCS, delay: &mut D) -> Result<Self, E>
    where
        D: DelayMs<u8>,
    {
        new(spi, ncs, delay)
    }

    /// Accelerometer + Gyroscope + Temperature sensor measurements
    pub fn all(&mut self) -> Result<ImuMeasurements, E> {
        let buffer = self.read_many::<U15>(Register::ACCEL_XOUT_H)?;

        let accel = I16x3 {
            x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
            y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
            z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16,
        };

        let temp = (u16(buffer[7]) << 8) + u16(buffer[8]);

        let gyro = I16x3 {
            x: ((u16(buffer[9]) << 8) + u16(buffer[10])) as i16,
            y: ((u16(buffer[11]) << 8) + u16(buffer[12])) as i16,
            z: ((u16(buffer[13]) << 8) + u16(buffer[14])) as i16,
        };

        Ok(ImuMeasurements { accel, gyro, temp })
    }
}

impl<E, SPI, NCS> Mpu9250<SPI, NCS, Marg>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
{
    /// Creates a new MARG driver from a SPI peripheral and a NCS pin
    ///
    /// # Defaults
    ///
    /// - Accelerometer: 16 bits, range [-2g, +2g]
    /// - Gyroscope: 16 bits, range [-250 dps, +250 dps]
    pub fn marg<D>(spi: SPI, ncs: NCS, delay: &mut D) -> Result<Self, E>
    where
        D: DelayMs<u8>,
    {
        new(spi, ncs, delay)
    }

    /// Returns factory mag sensitivity biases
    pub fn mag_sensitivity_adj(&self) -> F32x3 {
        self.mag_sensitivity_adj
    }

    /// Reads the AK8963 (magnetometer) WHO_AM_I register; should return `0x48`
    pub fn ak8963_who_am_i(&mut self) -> Result<u8, E> {
        self.ak8963_read(ak8963::Register::WHO_AM_I)
    }

    /// Accelerometer + Gyroscope + Temperature sensor measurements
    pub fn all(&mut self) -> Result<MargMeasurements, E> {
        let buffer = self.read_many::<U21>(Register::ACCEL_XOUT_H)?;

        let accel = I16x3 {
            x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
            y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
            z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16,
        };

        let temp = (u16(buffer[7]) << 8) + u16(buffer[8]);

        let gyro = I16x3 {
            x: ((u16(buffer[9]) << 8) + u16(buffer[10])) as i16,
            y: ((u16(buffer[11]) << 8) + u16(buffer[12])) as i16,
            z: ((u16(buffer[13]) << 8) + u16(buffer[14])) as i16,
        };

        let mag = I16x3 {
            x: ((u16(buffer[16]) << 8) + u16(buffer[15])) as i16,
            y: ((u16(buffer[18]) << 8) + u16(buffer[17])) as i16,
            z: ((u16(buffer[20]) << 8) + u16(buffer[19])) as i16,
        };

        Ok(MargMeasurements {
            accel,
            gyro,
            temp,
            mag,
        })
    }

    /// Magnetometer measurements
    pub fn mag(&mut self) -> Result<I16x3, E> {
        let buffer = self.read_many::<U8>(Register::EXT_SENS_DATA_00)?;

        Ok(I16x3 {
            x: ((u16(buffer[2]) << 8) + u16(buffer[1])) as i16,
            y: ((u16(buffer[4]) << 8) + u16(buffer[3])) as i16,
            z: ((u16(buffer[6]) << 8) + u16(buffer[5])) as i16,
        })
    }
}

impl<E, SPI, NCS, MODE> Mpu9250<SPI, NCS, MODE>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
{
    /// Accelerometer measurements
    pub fn accel(&mut self) -> Result<I16x3, E> {
        let buffer = self.read_many::<U7>(Register::ACCEL_XOUT_H)?;

        Ok(I16x3 {
            x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
            y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
            z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16,
        })
    }

    /// Applies a digital low pass filter to the accelerometer data
    pub fn a_filter(&mut self, dlpf: Dlpf) -> Result<(), E> {
        self.write(Register::ACCEL_CONFIG_2, dlpf as u8)?;

        Ok(())
    }

    /// Set accelerometer reading scale
    pub fn a_scale(&mut self, scale: FSScale) -> Result<(), E> {
        self.modify(Register::ACCEL_CONFIG, |r| {
            (r & !0b11000) | ((scale as u8) << 3)
        })?;
        Ok(())
    }

    /// Accelerometer YZ measurements
    pub fn aryz(&mut self) -> Result<(i16, i16), E> {
        let buffer = self.read_many::<U5>(Register::ACCEL_YOUT_H)?;

        let y = ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16;
        let z = ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16;

        Ok((y, z))
    }

    /// Accelerometer YZ + temperature sensor + gyroscope X measurements
    pub fn aryz_t_gx(&mut self) -> Result<(i16, i16, i16, i16), E> {
        let buffer = self.read_many::<U9>(Register::ACCEL_YOUT_H)?;

        let y = ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16;
        let z = ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16;
        let t = ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16;
        let x = ((u16(buffer[7]) << 8) + u16(buffer[8])) as i16;

        Ok((y, z, t, x))
    }

    /// Applies a digital low pass filter to the gyroscope data
    pub fn g_filter(&mut self, dlpf: Dlpf) -> Result<(), E> {
        self.modify(Register::CONFIG, |r| (r & !0b11) | dlpf as u8)?;

        Ok(())
    }

    /// Set gyroscope reading scale
    pub fn g_scale(&mut self, scale: FSScale) -> Result<(), E> {
        self.modify(Register::GYRO_CONFIG, |r| {
            (r & !0b11000) | ((scale as u8) << 3)
        })?;

        Ok(())
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, E> {
        let buffer = self.read_many::<U7>(Register::GYRO_XOUT_H)?;

        Ok(I16x3 {
            x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
            y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
            z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16,
        })
    }

    /// Gyroscope X measurement
    pub fn gx(&mut self) -> Result<i16, E> {
        let buffer = self.read_many::<U3>(Register::GYRO_XOUT_H)?;

        Ok(((u16(buffer[1]) << 8) + u16(buffer[2])) as i16)
    }

    /// Temperature sensor measurements
    pub fn temp(&mut self) -> Result<i16, E> {
        let buffer = self.read_many::<U3>(Register::TEMP_OUT_H)?;

        Ok((u16(buffer[1]) << 8 + u16(buffer[2])) as i16)
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

    fn ak8963_write(&mut self, reg: ak8963::Register, val: u8) -> Result<(), E> {
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
    where
        F: FnOnce(u8) -> u8,
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
    where
        N: ArrayLength<u8>,
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

/// SPI mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnSecondTransition,
};

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

/// XYZ triple
#[derive(Debug)]
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

/// IMU measurements
#[derive(Debug)]
pub struct ImuMeasurements {
    /// Accelerometer measurements
    pub accel: I16x3,
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Temperature sensor measurement
    pub temp: u16,
}

/// MARG measurements
#[derive(Debug)]
pub struct MargMeasurements {
    /// Accelerometer measurements
    pub accel: I16x3,
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Magnetometer measurements
    pub mag: I16x3,
    /// Temperature sensor measurement
    pub temp: u16,
}

/// Digital low pass filter configuration
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
    /// Accelerometer = 460 Hz, Gyroscope: 3600 Hz, Temperature sensor = 4000 Hz
    _7 = 7,
}

/// Sensor reading full scale configuration
pub enum FSScale {
    /// Accelerometer = +2g, Gyroscope: +250 dps
    _00 = 0b00,
    /// Accelerometer = +4g, Gyroscope: +500 dps
    _01 = 0b01,
    /// Accelerometer = +8g, Gyroscope: +1000 dps
    _02 = 0b10,
    /// Accelerometer = +16g, Gyroscope: +2000 dps
    _03 = 0b11,
}
