use core::fmt::Debug;
use linux_embedded_hal::{i2cdev::linux::LinuxI2CError, I2cdev};
use vl53l1x_uld::{
    self,
    comm::{Read, Write},
    IOVoltage, RangeStatus, SENSOR_ID, VL53L1X,
};

fn measure_without_interrupt<I2C, E>(mut vl: VL53L1X<I2C>) -> Result<(), vl53l1x_uld::Error<E>>
where
    E: Debug,
    I2C: Read<Error = E> + Write<Error = E>,
{
    // Ensure the correct device is connected
    if vl.get_sensor_id()? == SENSOR_ID {
        // Initialize the device.
        vl.init(IOVoltage::Volt2_8)?;

        // Do a direct register write to the interrupt config register to disable data ready interrupts.
        vl.write_bytes(vl53l1x_uld::Register::SYSTEM__INTERRUPT_CONFIG_GPIO, &[0])?;

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

    let vl = VL53L1X::new(i2c, vl53l1x_uld::DEFAULT_ADDRESS);

    measure_without_interrupt(vl)?;

    Ok(())
}
