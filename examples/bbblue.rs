//! BeagleBone Blue example
//!
//! MPU9250 connected on I2C2. This requires the i2c feature flag
//! for the mpu9250 crate.

extern crate linux_embedded_hal as hal;
extern crate mpu9250;

use std::io::{self, Write};
use std::thread;
use std::time::Duration;

use hal::Delay;
use hal::I2cdev;
use mpu9250::Mpu9250;

fn main() -> io::Result<()> {
    let i2c = I2cdev::new("/dev/i2c-2").expect("unable to open /dev/i2c-2");

    let mut mpu9250 =
        Mpu9250::marg_default(i2c, &mut Delay).expect("unable to make MPU9250");

    let who_am_i = mpu9250.who_am_i().expect("could not read WHO_AM_I");
    let mag_who_am_i = mpu9250.ak8963_who_am_i()
                              .expect("could not read magnetometer's WHO_AM_I");
    println!("WHO_AM_I: 0x{:x}", who_am_i);
    println!("AK8963 WHO_AM_I: 0x{:x}", mag_who_am_i);
    assert_eq!(who_am_i, 0x71);

    let stdout = io::stdout();
    let mut stdout = stdout.lock();
    writeln!(&mut stdout,
             "   Accel XYZ(m/s^2)  |   Gyro XYZ (rad/s)  |  Mag Field XYZ(uT)  | Temp (C)")?;
    loop {
        let all = mpu9250.all().expect("unable to read from MPU!");
        write!(&mut stdout,
               "\r{:>6.2} {:>6.2} {:>6.2} |{:>6.1} {:>6.1} {:>6.1} |{:>6.1} {:>6.1} {:>6.1} | {:>4.1} ",
               all.accel[0],
               all.accel[1],
               all.accel[2],
               all.gyro[0],
               all.gyro[1],
               all.gyro[2],
               all.mag[0],
               all.mag[1],
               all.mag[2],
               all.temp)?;
        stdout.flush()?;
        thread::sleep(Duration::from_micros(100000));
    }
}
