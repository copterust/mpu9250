//! A WIP, no_std, generic driver for the MPU9250 (accelerometer + gyroscope + magnetometer IMU)

#![deny(missing_docs)]
#![deny(warnings)]
#![feature(unsize)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;

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
    pub fn new(spi: SPI, ncs: NCS) -> Self {
        Mpu9250 { spi, ncs }
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

    /// Accelerometer + Gyroscope + Temperature sensor measurements
    pub fn all(&mut self) -> Result<Measurements, SPI::Error> {
        let buffer: [u8; 15] = self.read_many(Register::AccelXoutH)?;

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

        Ok(Measurements { accel, gyro, temp })
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

    /// Temperature sensor measurements
    pub fn temp(&mut self) -> Result<u16, SPI::Error> {
        let buffer: [u8; 3] = self.read_many(Register::TempOutH)?;

        Ok(u16(buffer[1]) << 8 + u16(buffer[2]))
    }

    /// Destroys the driver recovering the SPI peripheral and the NCS pin
    pub fn unwrap(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }

    /// Reads the WHO_AM_I register; should return `0x73`
    pub fn who_am_i(&mut self) -> Result<u8, SPI::Error> {
        self.read(Register::WhoAmI)
    }

    fn read(&mut self, reg: Register) -> Result<u8, SPI::Error> {
        let buffer: [u8; 2] = self.read_many(reg)?;
        Ok(buffer[1])
    }

    fn read_many<B>(&mut self, reg: Register) -> Result<B, SPI::Error>
    where
        B: Unsize<[u8]>,
    {
        let mut buffer: B = unsafe { mem::uninitialized() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.read_address();
            self.ncs.set_low();
            self.spi.transfer(slice)?;
            self.ncs.set_high();
        }

        Ok(buffer)
    }

    #[allow(dead_code)]
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

#[derive(Clone, Copy)]
enum Register {
    AccelXoutH = 0x3b,
    GyroXOutH = 0x43,
    TempOutH = 0x41,
    WhoAmI = 0x75,
}

impl Register {
    fn read_address(&self) -> u8 {
        *self as u8 | R
    }

    fn write_address(&self) -> u8 {
        *self as u8 | W
    }
}

const R: u8 = 1 << 7;
const W: u8 = 1 << 7;

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
    /// Temperature sensor measurement
    pub temp: u16,
}
