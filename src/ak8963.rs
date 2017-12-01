//! AK8963, I2C magnetometer

// I2C slave address
pub const I2C_ADDRESS: u8 = 0x0c;
pub const R: u8 = 1 << 7;
pub const W: u8 = 0 << 7;

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Register {
    Cntl1 = 0x0a,
    Cntl2 = 0x0b,
    Hxl = 0x03,
    Wia = 0x00,
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}
