![build](https://github.com/TomSievers/vl53l1x-rs/actions/workflows/rust.yml/badge.svg)
[![docs](https://img.shields.io/badge/docs-stable-blue)](https://docs.rs/vl53l1x-uld/)
[![crates.io](https://img.shields.io/crates/v/vl53l1x-uld)](https://crates.io/crates/vl53l1x-uld)

# vl53l1x-uld
This crate is a port of the [STM VL53L1X IMG009][driver-page] driver to native rust.

[driver-page]: https://www.st.com/content/st_com/en/products/embedded-software/imaging-software/stsw-img009.html#overview

## Minimal example

Below is minimal example for using this driver. Other more complex examples can be found in the examples folder.

```rust
use vl53l1x_uld::{self, VL53L1X, IOVoltage, RangeStatus};

// Create hardware specific I2C instance.
let i2c = ...;
// Create sensor with default address. 
let mut vl = VL53L1X::new(i2c, vl53l1x_uld::DEFAULT_ADDRESS);

const ERR : &str = "Failed to communicate";

// Check if the sensor id is correct.
if (vl.get_sensor_id().expect(ERR) == 0xEACC)
{
    // Initialize the sensor before any usage.
    // Set the voltage of the IO pins to be 2.8 volts
    vl.init(IOVoltage::Volt2_8).expect(ERR);

    // Start a ranging operation, needed to retrieve a distance
    vl.start_ranging().expect(ERR);

    // Wait until distance data is ready to be read.
    while !vl.is_data_ready().expect(ERR) {}

    // Check if distance measurement is valid.
    if (vl.get_range_status().expect(ERR) == RangeStatus::Valid)
    {
        // Retrieve measured distance.
        let distance = vl.get_distance().expect(ERR);
    }
}

```
