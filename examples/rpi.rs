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

extern crate linux_embedded_hal as hal;
extern crate mpu9250;

use std::thread;
use std::time::Duration;

use mpu9250::Mpu9250;
use hal::spidev::{self, SpidevOptions};
use hal::{Delay, Pin, Spidev};
use hal::sysfs_gpio::Direction;

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

    let mut mpu9250 = Mpu9250::marg(spi, ncs, &mut Delay).unwrap();

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
