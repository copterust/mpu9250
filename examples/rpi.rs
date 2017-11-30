//! Raspberry Pi demo
//!
//! # Connections
//!
//! - PIN1 = 3V3 = VCC
//! - PIN19 = BCM10 = MOSI
//! - PIN19 = BCM10 = MOSI
//! - PIN21 = BCM9 = MISO (SCL)
//! - PIN23 = BCM11 = SCLK
//! - PIN24 = BCM8 = NCS
//! - PIN6 = GND = GND

extern crate embedded_hal as hal;
extern crate mpu9250;
extern crate spidev;
extern crate sysfs_gpio;

use std::io::{self, Write};

use mpu9250::Mpu9250;
use spidev::{Spidev, SpidevOptions, SpidevTransfer};
use sysfs_gpio::{Direction, Pin};

pub struct MySpidev(Spidev);
pub struct MyPin(Pin);

impl hal::blocking::spi::FullDuplex<u8> for MySpidev {
    type Error = io::Error;

    fn transfer<'b>(&mut self, buffer: &'b mut [u8]) -> Result<&'b [u8], io::Error> {
        let tx = buffer.to_owned();
        self.0
            .transfer(&mut SpidevTransfer::read_write(&tx, buffer))?;
        Ok(buffer)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), io::Error> {
        self.0.write_all(bytes)
    }
}

impl hal::digital::OutputPin for MyPin {
    fn set_high(&mut self) {
        self.0.set_value(1).unwrap()
    }

    fn set_low(&mut self) {
        self.0.set_value(0).unwrap()
    }

    fn is_low(&self) -> bool {
        self.0.get_value().unwrap() == 0
    }

    fn is_high(&self) -> bool {
        self.0.get_value().unwrap() == 1
    }
}

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .max_speed_hz(1_000_000)
        .mode(spidev::SPI_MODE_3)
        .build();
    spi.configure(&options).unwrap();

    let ncs = Pin::new(8);
    ncs.export().unwrap();
    ncs.set_value(1).unwrap();
    ncs.set_direction(Direction::Out).unwrap();

    let mut mpu9250 = Mpu9250::new(MySpidev(spi), MyPin(ncs));

    let who_am_i = mpu9250.who_am_i().unwrap();

    println!("WHO_AM_I: {}", who_am_i);

    assert_eq!(who_am_i, 0x71);

    let measurements = mpu9250.all().unwrap();

    println!("{:?}", measurements);
}
