//! Raspberry Pi demo
//!
//! # Connections
//!
//! IMPORTANT: Do *not* use PIN24 / BCM8 / CE0 as the NCS pin
//!
//! - PIN1 = 3V3 = VCC
//! - PIN19 = BCM10 = MOSI (SDA)
//! - PIN21 = BCM9 = MISO (AD0)
//! - PIN23 = BCM11 = SCLK (SCL)
//! - PIN22 = BCM25 = NCS
//! - PIN6 = GND = GND

extern crate embedded_hal as hal;
extern crate mpu9250;
extern crate spidev;
extern crate sysfs_gpio;

use std::io::{self, Write};
use std::thread;
use std::time::Duration;

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

    let ncs = Pin::new(25);
    ncs.export().unwrap();
    while !ncs.is_exported() {}
    ncs.set_direction(Direction::Out).unwrap();
    ncs.set_value(1).unwrap();

    let mut mpu9250 = Mpu9250::new(MySpidev(spi), MyPin(ncs)).unwrap();

    let who_am_i = mpu9250.who_am_i().unwrap();
    let ak8963_who_am_i = mpu9250.ak8963_who_am_i().unwrap();

    println!("WHO_AM_I: 0x{:x}", who_am_i);
    println!("AK8963_WHO_AM_I: 0x{:x}", who_am_i);

    assert_eq!(who_am_i, 0x71);
    assert_eq!(ak8963_who_am_i, 0x48);

    println!("{:#?}", mpu9250.all().unwrap());

    thread::sleep(Duration::from_millis(100));

    println!("{:#?}", mpu9250.all().unwrap());
}
