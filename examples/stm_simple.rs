use stm32l4xx_hal::{self as hal, pac};
use vl53l1x_uld;

fn main() {
    /*let dp = pac::Peripherals::take().expect("");
    let rcc = dp.RCC.constrain();
    let flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let scl = gpioa.pa9.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let sda = gpioa.pa10.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    let i2c = hal::i2c::I2c::new(dp.I2C2, (scl, sda), Hertz(100_000), clocks, &mut rcc.apb1);

    let vl = vl53l1x_uld::VL53L1X::new(i2c, vl53l1x_uld::DEFAULT_ADDRESS);*/

    loop {}
}
