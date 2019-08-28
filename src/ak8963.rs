//! AK8963, I2C magnetometer

use hal::blocking::delay::DelayMs;

use generic_array::typenum::consts::*;
use generic_array::GenericArray;

// I2C slave address
pub const I2C_ADDRESS: u8 = 0x0c;
pub const R: u8 = 1 << 7;
pub const W: u8 = 0 << 7;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
pub enum Register {
    WHO_AM_I = 0x00, // should return 0x48
    INFO = 0x01,
    ST1 = 0x02,    // data ready status bit 0
    XOUT_L = 0x03, // data
    XOUT_H = 0x04,
    YOUT_L = 0x05,
    YOUT_H = 0x06,
    ZOUT_L = 0x07,
    ZOUT_H = 0x08,
    ST2 = 0x09, // Data overflow bit 3 and data read error status bit 2
    CNTL = 0x0A, /* Power down (0000), single-measurement (0001), self-test
                 * (1000) and Fuse ROM (1111) modes on bits 3:0 */
    ASTC = 0x0C,   // Self test control
    I2CDIS = 0x0F, // I2C disable
    ASAX = 0x10,   // Fuse ROM x-axis sensitivity adjustment value
    ASAY = 0x11,   // Fuse ROM y-axis sensitivity adjustment value
    ASAZ = 0x12,   // Fuse ROM z-axis sensitivity adjustment value
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

/// Decribes a type that can communicate with the
/// MPU's on-board magnetometer, the AK8963
pub trait AK8963 {
    /// Associated error type
    type Error;

    /// Initialize the AK8963
    ///
    /// It may not make sense to call this more than once. However, it is
    /// absolutely necessary to call it at least once if you need the
    /// magnetometer
    fn init<D: DelayMs<u8>>(&mut self,
                            delay: &mut D)
                            -> Result<(), Self::Error>;

    /// Perform final initialization. Invoked after acquiring the magnetomter's
    /// calibration values and setting the sampling rate and resolution.
    fn finalize<D: DelayMs<u8>>(&mut self,
                                _: &mut D)
                                -> Result<(), Self::Error> {
        Ok(())
    }

    /// Read a magnetometer's register
    fn read(&mut self, reg: Register) -> Result<u8, Self::Error>;

    /// Write to a magnetometer's register
    fn write(&mut self, reg: Register, value: u8) -> Result<(), Self::Error>;

    /// Read the magnetometer's X,Y,Z triplet
    fn read_xyz(&mut self) -> Result<GenericArray<u8, U7>, Self::Error>;
}
