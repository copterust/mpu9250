# `mpu9250`

> no_std driver for the MPU9250 & onboard AK8963 (accelerometer + gyroscope +  magnetometer IMU).

[![Build Status](https://travis-ci.org/copterust/mpu9250.svg?branch=master)](https://travis-ci.org/copterust/mpu9250)

## What works

- Reading the accelerometer, gyroscope, temperature sensor, and magnetrometer: both raw and scaled and converted values.
- Setting DLPF, reading scales, sample rate divisor.
- Reading the WHO_AM_I registers of mpu9250 and ak8963.
- Getting resolutions and factory sensitivities.

`Imu` mode also works with mpu6500 (see #1 for details).

## Basic usage

Include [library](https://crates.io/crates/mpu9250) as a dependency in your Cargo.toml
[![crates.io](http://meritbadge.herokuapp.com/mpu9250?style=flat-square)](https://crates.io/crates/mpu9250):

```
[dependencies.mpu9250]
version = "<version>"
```

Use embedded-hal implementation to get SPI, NCS, and delay, then create mpu handle:

```rust
extern crate mpu9250; // or just use mpu9250; if 2018 edition is used.

// to create sensor with mag support and default configuration:
let mut _imu = Mpu9250::marg_default(spi, ncs, &mut delay)?;
// to create sensor without mag support and default configuration:
let mut marg = Mpu9250::imu_default(spi, ncs, &mut delay)?;
// to get all supported measurements:
let all = marg.all()?;
println!("{:?}", all);
```

## More examples

Number of examples can be found in [proving-ground](https://github.com/copterust/proving-ground) repo.
Examples include: reading temperature, calibrating magnetrometer, reading all sensors.

## Documentation

API Docs available on [docs.rs](https://docs.rs/mpu9250).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Testimonials

Started off as a fork of [japaric's mpu9250 repo](https://github.com/japaric/mpu9250).
