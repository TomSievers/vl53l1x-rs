use core::fmt::Debug;
use linux_embedded_hal::{i2cdev::linux::LinuxI2CError, I2cdev};
use vl53l1x_uld::{
    self,
    comm::{Read, Write},
    IOVoltage, RangeStatus, VL53L1X,
};

fn measure_continuous<I2C, E>(mut vl: VL53L1X<I2C>) -> Result<(), vl53l1x_uld::Error<E>>
where
    E: Debug,
    I2C: Read<Error = E> + Write<Error = E>,
{
    // Ensure the correct device is connected
    if vl.get_sensor_id()? == 0xEACC {
        // Initialize the device.
        vl.init(IOVoltage::Volt2_8)?;

        // Start the ranging operations.
        vl.start_ranging()?;

        loop {
            // Wait for data to be ready
            while !vl.is_data_ready()? {}

            // Read the measurement result in one operation.
            let res = vl.get_result()?;

            if res.status == RangeStatus::Valid {
                // Print distance.
                println!("{}", res.distance_mm);
            }

            // Clear interrupt to start next measurement
            vl.clear_interrupt()?;
        }
    }
    Ok(())
}

fn main() -> Result<(), vl53l1x_uld::Error<LinuxI2CError>> {
    let i2c = I2cdev::new("/dev/i2c")?;

    let bus = shared_bus::new_std!(I2cdev = i2c).expect("Failed to create bus manager");

    let vl1 = VL53L1X::new(bus.acquire_i2c(), vl53l1x_uld::DEFAULT_ADDRESS);

    let vl2 = VL53L1X::new(bus.acquire_i2c(), 0x50);

    std::thread::spawn(move || measure_continuous(vl1));

    measure_continuous(vl2)?;

    Ok(())
}
