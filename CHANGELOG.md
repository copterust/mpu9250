# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

## [v0.9.2] - 2019-02-26

### Changed

- Only IMU (6-axis) driver is available via I2c.

## [v0.9.0] - 2019-02-26

### Added

- Experimental I2C support.

### Changed

- `MPU9250` is now parametrized over device (`SpiDevice<SPI, NCS`, `I2cDevice<I2C>`) and mode (`Marg` and `Imu`).

## [v0.8.0] - 2019-02-24

### Added

- Ability to disable/enable/query interrupts.

## [v0.2.2] - 2018-08-15

- First published version.


[Unreleased]: https://github.com/rust-embedded/cortex-m/compare/v0.8.0...HEAD
[v0.8.0]: https://github.com/copterust/mpu9250/compare/v0.2.2...v0.8.0
