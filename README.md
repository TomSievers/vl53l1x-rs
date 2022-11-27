![build](https://github.com/TomSievers/vl53l1x-rs/actions/workflows/rust.yml/badge.svg)
[![docs](https://img.shields.io/badge/docs-stable-blue)](https://docs.rs/vl53l1x-uld/)
[![crates.io](https://img.shields.io/crates/v/vl53l1x-uld)](https://crates.io/crates/vl53l1x-uld)

# vl53l1x-uld
This crate is a port of the [STM VL53L1X IMG009][driver-page] driver to native rust.

[driver-page]: https://www.st.com/content/st_com/en/products/embedded-software/imaging-software/stsw-img009.html#overview

## Minimal example

Below is minimal example of using the current driver implementation.

```rust
use vl53l1x_uld;

let i2c : I2CType = ...; //Create hardware specific I2C
let vl53l1x : VL53L1X::new()

```
