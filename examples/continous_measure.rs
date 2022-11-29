#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;
use cortex_m_rt::entry;
use hal::{rcc::RccExt, prelude::*, pwr::PwrExt, time::Hertz, i2c};
use core::fmt::Debug;
use stm32l4xx_hal::{self as hal, pac};
use vl53l1x_uld::{self, VL53L1X, IOVoltage, RangeStatus, comm::{Read, Write}};

fn measure_continuous<I2C, E>(mut vl : VL53L1X<I2C>) -> Result<(), vl53l1x_uld::Error<E>>
where
    E : Debug,
    I2C : Read<Error = E> + Write<Error = E>
{
    // Ensure the correct device is connected
    if vl.get_sensor_id()? == 0xEACC
    {
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
                defmt::println!("{}", res.distance_mm);
            }

            // Clear interrupt to start next measurement
            vl.clear_interrupt()?;
        }
    }
    Ok(())
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("");
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let scl = gpioa.pa9.into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let sda = gpioa.pa10.into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    let i2c = i2c::I2c::i2c1(dp.I2C1, (scl, sda), hal::i2c::Config::new(Hertz::Hz(100_000), clocks), &mut rcc.apb1r1);

    let vl = VL53L1X::new(i2c, vl53l1x_uld::DEFAULT_ADDRESS);

    let res = measure_continuous(vl);

    defmt::println!("{}", defmt::Debug2Format(&res));

    loop {}
}
