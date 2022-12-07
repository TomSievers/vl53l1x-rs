# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0]
### Added
* Added examples & more documentation

### Changed
* The driver now consumes the I2C, making the API easier to use as it only has to be passed into the `new` function.
  When using multiple devices on the same I2C bus you should use the [shared-bus crate](https://crates.io/crates/shared-bus).

[Unreleased]: https://github.com/TomSievers/vl53l1x-rs/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/TomSievers/vl53l1x-rs/compare/v1.0.0...v2.0.0
