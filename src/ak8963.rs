//! AK8963, I2C magnetometer

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
