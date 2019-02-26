//! BeagleBone Blue example
//!
//! MPU9250 connected on I2C2. This requires the i2c feature flag
//! for the mpu9250 crate.

extern crate linux_embedded_hal as hal;
extern crate mpu9250;

use std::thread;
use std::time::Duration;

use hal::Delay;
use hal::I2cdev;
use mpu9250::Mpu9250;

fn main() {
    let i2c = I2cdev::new("/dev/i2c-2").expect("unable to open /dev/i2c-2");

    let mut mpu9250 = Mpu9250::marg_default(i2c, &mut Delay).expect("unable to make MPU9250");

    let who_am_i = mpu9250.who_am_i().expect("could not read WHO_AM_I");

    println!("WHO_AM_I: 0x{:x}", who_am_i);
    assert_eq!(who_am_i, 0x71);

    println!("{:#?}", mpu9250.all().expect("could not perform first read all"));

    thread::sleep(Duration::from_millis(100));

    println!("{:#?}", mpu9250.all().unwrap());
}