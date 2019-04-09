# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [v0.10.1]

### Fixed

- `Marg` device connected via SPI reads all measurement in one transaction.


## [v0.10.0]

### Added

- Enable I2C Bypass for AK8963 communications: `Marg` is now available for `I2c` connected MPUs.


## [v0.9.5]

### Fixed

- `calibrate_at_rest` now correctly re-initializes on-board magnetrometer when Marg is used.

## [v0.9.4]

### Added

- `imu_with_reinit` and `marg_with_reinit` consturctor for MPU9250 via SPI.
   These functions allow supplying callback that will be called after initialization is done
   to optionally reconfigure underlying bus, e.g. change speed.

## [v0.9.3]

### Added

- `INT_PIN_CFG` and `INT_STATUS` related routines

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


[Unreleased]: https://github.com/rust-embedded/cortex-m/compare/v0.10.0...HEAD
[v0.10.0]: https://github.com/copterust/mpu9250/compare/v0.9.5...v0.10.0
[v0.9.5]: https://github.com/copterust/mpu9250/compare/v0.9.4...v0.9.5
[v0.9.4]: https://github.com/copterust/mpu9250/compare/v0.9.3...v0.9.4
[v0.9.3]: https://github.com/copterust/mpu9250/compare/v0.9.2...v0.9.3
[v0.9.2]: https://github.com/copterust/mpu9250/compare/v0.9.0...v0.9.2
[v0.9.0]: https://github.com/copterust/mpu9250/compare/v0.8.0...v0.9.0
[v0.8.0]: https://github.com/copterust/mpu9250/compare/v0.2.2...v0.8.0
