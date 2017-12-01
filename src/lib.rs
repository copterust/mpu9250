//! A WIP, no_std, generic driver for the MPU9250 (accelerometer + gyroscope + magnetometer IMU)

#![deny(missing_docs)]
#![deny(warnings)]
#![feature(unsize)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;

mod ak8963;

use core::marker::Unsize;
use core::mem;

use cast::u16;
use hal::blocking::spi::{self, Mode, Phase, Polarity};
use hal::digital::OutputPin;

/// MPU9250 driver
pub struct Mpu9250<SPI, NCS> {
    spi: SPI,
    ncs: NCS,
}

impl<SPI, NCS> Mpu9250<SPI, NCS>
where
    SPI: spi::FullDuplex<u8>,
    NCS: OutputPin,
{
    /// Creates a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPI, ncs: NCS) -> Result<Self, SPI::Error> {
        let mut mpu9250 = Mpu9250 { spi, ncs };

        // XXX this probably needs some delay to be reliable
        // soft reset the device
        mpu9250.write(Register::PwrMgmt1, 0x80)?;

        // use the best clock
        mpu9250.write(Register::PwrMgmt1, 0x01)?;

        // sanity check that both the accelerometer and gyroscope are enabled
        // (this should be the default after a soft reset)
        debug_assert_eq!(mpu9250.read(Register::PwrMgmt2)?, 0x00);

        // isolate the auxiliary master I2C bus (AUX_CL, AUX_DA)
        // disable the slave I2C bus, make serial interface SPI only
        // reset the master I2C bus
        mpu9250.write(Register::UserCtrl, 0x32)?;

        // set aux I2C frequency to 400 KHz
        mpu9250.write(Register::I2cMstCtrl, 0x0d)?;

        // sanity check that the aux I2C is working
        debug_assert_eq!(mpu9250.ak8963_read(ak8963::Register::Wia)?, 0x48);

        // software reset the magnetometer
        // FIXME this hangs when compiled in release mode
        mpu9250.ak8963_write(ak8963::Register::Cntl2, 0x01)?;

        // 100 Hz (?) continuous measurement, 16-bit
        mpu9250.ak8963_write(ak8963::Register::Cntl1, 0x16)?;

        // configure sampling of magnetometer
        mpu9250.write(Register::I2cSlv0Addr, ak8963::I2C_ADDRESS | ak8963::R)?;
        mpu9250.write(Register::I2cSlv0Reg, ak8963::Register::Hxl.addr())?;
        mpu9250.write(Register::I2cSlv0Ctrl, 0x87)?;

        Ok(mpu9250)
    }

    /// Accelerometer measurements
    pub fn accel(&mut self) -> Result<I16x3, SPI::Error> {
        let buffer: [u8; 7] = self.read_many(Register::AccelXoutH)?;

        Ok(I16x3 {
            x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
            y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
            z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16,
        })
    }

    /// Reads the AK8963 (magnetometer) WHO_AM_I register; should return `0x48`
    pub fn ak8963_who_am_i(&mut self) -> Result<u8, SPI::Error> {
        self.ak8963_read(ak8963::Register::Wia)
    }

    /// Accelerometer + Gyroscope + Temperature sensor measurements
    pub fn all(&mut self) -> Result<Measurements, SPI::Error> {
        let buffer: [u8; 22] = self.read_many(Register::AccelXoutH)?;

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

        Ok(Measurements { accel, gyro, temp, mag })
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, SPI::Error> {
        let buffer: [u8; 7] = self.read_many(Register::GyroXOutH)?;

        Ok(I16x3 {
            x: ((u16(buffer[1]) << 8) + u16(buffer[2])) as i16,
            y: ((u16(buffer[3]) << 8) + u16(buffer[4])) as i16,
            z: ((u16(buffer[5]) << 8) + u16(buffer[6])) as i16,
        })
    }

    /// Magnetometer measurements
    pub fn mag(&mut self) -> Result<I16x3, SPI::Error> {
        let buffer: [u8; 8] = self.read_many(Register::ExtSensData00)?;

        Ok(I16x3 {
            x: ((u16(buffer[2]) << 8) + u16(buffer[1])) as i16,
            y: ((u16(buffer[4]) << 8) + u16(buffer[3])) as i16,
            z: ((u16(buffer[6]) << 8) + u16(buffer[5])) as i16,
        })
    }

    /// Temperature sensor measurements
    pub fn temp(&mut self) -> Result<u16, SPI::Error> {
        let buffer: [u8; 3] = self.read_many(Register::TempOutH)?;

        Ok(u16(buffer[1]) << 8 + u16(buffer[2]))
    }

    /// Destroys the driver recovering the SPI peripheral and the NCS pin
    pub fn unwrap(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }

    /// Reads the WHO_AM_I register; should return `0x71`
    pub fn who_am_i(&mut self) -> Result<u8, SPI::Error> {
        self.read(Register::WhoAmI)
    }

    fn ak8963_read(&mut self, reg: ak8963::Register) -> Result<u8, SPI::Error> {
        self.write(Register::I2cSlv4Addr, ak8963::I2C_ADDRESS | ak8963::R)?;
        self.write(Register::I2cSlv4Reg, reg.addr())?;

        // start transfer
        self.write(Register::I2cSlv4Ctrl, 0x80)?;

        // wait until transfer is over
        while self.read(Register::I2cMstStatus)? & (1 << 6) == 0 {}

        self.read(Register::I2cSlv4Di)
    }

    fn ak8963_write(&mut self, reg: ak8963::Register, val: u8) -> Result<(), SPI::Error> {
        self.write(Register::I2cSlv4Addr, ak8963::I2C_ADDRESS | ak8963::W)?;
        self.write(Register::I2cSlv4Reg, reg.addr())?;
        self.write(Register::I2cSlv4Do, val)?;

        // start transfer
        self.write(Register::I2cSlv4Ctrl, 0x80)?;

        // wait until transfer is over
        while self.read(Register::I2cMstStatus)? & (1 << 6) == 0 {}

        Ok(())
    }

    fn read(&mut self, reg: Register) -> Result<u8, SPI::Error> {
        let buffer: [u8; 2] = self.read_many(reg)?;
        Ok(buffer[1])
    }

    fn read_many<B>(&mut self, reg: Register) -> Result<B, SPI::Error>
    where
        B: Unsize<[u8]>,
    {
        let mut buffer: B = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.read_address();
            self.ncs.set_low();
            self.spi.transfer(slice)?;
            self.ncs.set_high();
        }

        Ok(buffer)
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), SPI::Error> {
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
#[derive(Clone, Copy)]
enum Register {
    AccelXoutH = 0x3b,
    ExtSensData00 = 0x49,
    ExtSensData01 = 0x4a,
    ExtSensData02 = 0x4b,
    ExtSensData03 = 0x4c,
    ExtSensData04 = 0x4d,
    GyroXOutH = 0x43,
    I2cMstCtrl = 0x24,
    I2cMstStatus = 0x36,
    I2cSlv0Addr = 0x25,
    I2cSlv0Ctrl = 0x27,
    I2cSlv0Do = 0x63,
    I2cSlv0Reg = 0x26,
    I2cSlv4Addr = 0x31,
    I2cSlv4Ctrl = 0x34,
    I2cSlv4Di = 0x35,
    I2cSlv4Do = 0x33,
    I2cSlv4Reg = 0x32,
    IntPinCfg = 0x37,
    PwrMgmt1 = 0x6b,
    PwrMgmt2 = 0x6c,
    TempOutH = 0x41,
    UserCtrl = 0x6a,
    WhoAmI = 0x75,
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

/// Several measurements
#[derive(Debug)]
pub struct Measurements {
    /// Accelerometer measurements
    pub accel: I16x3,
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Magnetometer measurements
    pub mag: I16x3,
    /// Temperature sensor measurement
    pub temp: u16,
}
