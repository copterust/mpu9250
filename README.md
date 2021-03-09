# `mpu9250`

> no_std driver for the MPU9250 (and some MPU* devices) & onboard AK8963 (accelerometer + gyroscope +  magnetometer IMU).

[![Build Status](https://travis-ci.org/copterust/mpu9250.svg?branch=master)](https://travis-ci.org/copterust/mpu9250)

## What works

- Reading the accelerometer, gyroscope, temperature sensor, and magnetrometer: both raw and scaled and converted values.
- Setting DLPF, reading scales, sample rate divisor.
- Reading the WHO_AM_I registers of mpu9250 and ak8963.
- Getting resolutions and factory sensitivities.

## Supported chips

* `MPU9250` -- `Imu` and `Marg`;
* `MPU9255` -- `Imu` and `Marg`;
* `MPU6500` -- `Imu` only.

### Notes

> MPU9255 has some extra capability in the ASIC that allows some additional
> gesture control but otherwise this chip is identical to the MPU9250.

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
let mut marg = Mpu9250::marg_default(spi, ncs, &mut delay)?;
// to create sensor without mag support and default configuration:
let mut imu = Mpu9250::imu_default(spi, ncs, &mut delay)?;
// to get all supported measurements:
let all = marg.all::<[f32; 3]>()?;
println!("{:?}", all);
```

or use the new expiremntal builder pattern:

```rust
extern crate mpu9250; // or just use mpu9250; if 2018 edition is used.

// to create sensor with mag support and default configuration:
let mut marg = mpu9250::MpuConfig::marg().build(spi, ncs);
marg.init(&mut delay)?;
// to create sensor without mag support and default configuration:
let mut imu = mpu9250::MpuConfig::imu().build(spi, ncs);
imu.init(&mut Delay)?;
// to get all supported measurements:
let all = marg.all::<[f32;3]>()?;
println!("{:?}", all);
```

## More examples

Number of examples can be found in [proving-ground](https://github.com/copterust/proving-ground) repo.
Examples include: reading temperature, calibrating magnetrometer, reading all sensors.

## Experimental I2C support

Expiremntal I2C support is enabled via `i2c` feature flag. When enabled, SPI support will be deactivated
and type of mpu9250 driver will change from `Mpu9250<SpiDevice<SPI, NCS>, MODE>` will change to
`Mpu9250<I2cDevice<I2C>, Imu>`.

The MPU9250 currently supports an IMU-only configuration. See the [BeagleBone Blue example](examples/bbblue.rs)
for a demonstration. Support for the AK8963 is a WPI.

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
