# `mpu9250`

> no_std driver for the MPU9250 & onboard AK8963 (accelerometer + gyroscope +  magnetometer IMU).

## What works

- Reading the accelerometer, gyroscope, temperature sensor, and magnetrometer: both raw and scaled and converted values.
- Setting DLPF, reading scales, sample rate divisor.
- Reading the WHO_AM_I registers of mpu9250 and ak8963.
- Getting resolutions and factory sensitivities.

## Basic usage

Include [library](https://crates.io/crates/mpu9250) as a dependency in your Cargo.toml:

```
[dependencies.mpu9250]
version = "0.1.0"
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
