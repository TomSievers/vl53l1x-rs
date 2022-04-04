#![no_std]
use embedded_hal::blocking::i2c::{Write, WriteRead, SevenBitAddress};
use measurement::Measurement;
use roi::Roi;
use threshold::ThresholdSettings;
pub mod threshold;
pub mod roi;
pub mod measurement;

pub enum IrqPolarity {
    ActiveHigh,
    ActiveLow
}

pub enum DistanceMode {
    Short,
    Medium,
    Long
}

pub struct VL53L1X<T : Write<SevenBitAddress> + WriteRead<SevenBitAddress>> {
    i2c : T
}

impl<T : Write + WriteRead> VL53L1X<T> {
    pub fn new(i2c : T) -> VL53L1X<T> {
        todo!()
    }

    pub fn clear_irq(&mut self) -> Result<(), ()> {
        todo!()
    }

    pub fn set_irq_polarity(&mut self, polarity : IrqPolarity) -> Result<(), ()> {
        todo!()
    }

    pub fn get_irq_polarity(&mut self) -> Result<IrqPolarity, ()> {
        todo!()
    }

    pub fn start_measuring(&mut self) -> Result<(), ()> {
        todo!()
    }

    pub fn stop_measuring(&mut self) -> Result<(), ()> {
        todo!()
    }

    pub fn is_data_ready(&mut self) -> Result<bool, ()> {
        todo!()
    }

    pub fn set_timing_budget_us(&mut self, microseconds : u32) -> Result<(), ()> {
        todo!()
    }

    pub fn get_timing_budget_us(&mut self) -> Result<u32, ()> {
        todo!()
    }

    pub fn set_inter_measurement_period_ms(&mut self, milliseconds : u32) -> Result<(), ()> {
        todo!()
    }

    pub fn get_inter_measurement_period_ms(&mut self) -> Result<u32, ()> {
        todo!()
    }

    pub fn set_roi(&mut self, roi : Roi) -> Result<(), ()> {
        todo!()
    }

    pub fn get_roi(&mut self) -> Result<Roi, ()> {
        todo!()
    }

    pub fn set_distance_mode(&mut self, distance_mode : DistanceMode) -> Result<(), ()> {
        todo!()
    }

    pub fn get_distance_mode(&mut self) -> Result<DistanceMode, ()> {
        todo!()
    }

    pub fn set_xtalk_compensation_enable(&mut self, enabled : bool) -> Result<(), ()> {
        todo!()
    }

    pub fn get_xtalk_compensation_enable(&mut self) -> Result<bool, ()> {
        todo!()
    }

    pub fn calibrate_xtalk(&mut self, cal_distance_millimeter : u32) -> Result<(), ()> {
        todo!()
    }

    pub fn calibrate_offset(&mut self, cal_distance_millimeter : u32) -> Result<(), ()> {
        todo!()
    }

    pub fn set_threshold_settings(&mut self, threshold : ThresholdSettings) -> Result<(), ()> {
        todo!()
    }

    pub fn get_threshold_settings(&mut self) -> Result<ThresholdSettings, ()> {
        todo!()
    }

    pub fn read_milimeter() -> Result<u16, ()> {
        todo!()
    }

    pub fn read() -> Result<Measurement, ()> {
        todo!()
    }
}