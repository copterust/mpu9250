//! BeagleBone Blue example
//!
//! MPU9250 connected on I2C2. This requires the i2c feature flag
//! for the mpu9250 crate.
//!
//! /!\ You will have to unexport the interrupt pin by yourself
//! `sudo echo 117 > /sys/class/gpio/unexport`

extern crate linux_embedded_hal as hal;
extern crate mpu9250;

use std::io::{self, Write};

use hal::sysfs_gpio;
use hal::Delay;
use hal::I2cdev;
use mpu9250::I2cAddress;
use mpu9250::DMP_FIRMWARE;
use mpu9250::{Error, Mpu9250};

fn main() {
    let i2c = I2cdev::new("/dev/i2c-2").expect("unable to open /dev/i2c-2");

    let pin = sysfs_gpio::Pin::new(117);
    pin.with_exported(|| {
        pin.set_direction(sysfs_gpio::Direction::In).unwrap();
        pin.set_edge(sysfs_gpio::Edge::FallingEdge).unwrap();
        let mut event = pin.get_poller().unwrap();

        let stdout = io::stdout();
        let mut stdout = stdout.lock();
        writeln!(&mut stdout, "  Normalized quaternion").unwrap();

        let mut mpu9250 =
            Mpu9250::dmp_default(i2c,I2cAddress::AD0Low, &mut Delay, &DMP_FIRMWARE).expect("unable to load firmware");

        loop {
            match event.poll(1000).unwrap() {
                Some(_) =>
                    match mpu9250.dmp_all::<[f32; 3], [f64; 4]>() {
                        Ok(measure) => {
                            write!(&mut stdout,
                                "\r{:?}",
                                measure).unwrap();
                            stdout.flush().unwrap();
                        },
                        Err(Error::DmpDataNotReady) => (),
                        Err(_) => (),
                    },
                None => {
                    write!(&mut stdout, "\nTimeout\n").unwrap();
                    mpu9250.reset_fifo(&mut Delay).unwrap();
                }
            }
        }
    }).unwrap();
}
