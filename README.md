# `mpu9250`

> A WIP, no_std, generic driver for the MPU9250 (accelerometer + gyroscope + magnetometer IMU)

## What works

- Reading the accelerometer, gyroscope and temperature sensor
- Reading the WHO_AM_I register

## TODO

- Access to magnetometer data
- Make sure this works with the `spidev` crate (i.e. with the Raspberry Pi)
- Configuration. e.g. selecting the accelerometer sensing range.
- How to make the API compatible with device specific features like DMA?
- Sensor fusion?
- ???

## Examples

You should find at least one example in the [blue-pill] repository. If that branch is gone, check
the master branch.

[blue-pill]: https://github.com/japaric/blue-pill/tree/singletons/examples

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
