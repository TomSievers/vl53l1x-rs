//! Port of the STM IMG009 Ultra Lite Driver for the VL53L1X.
#![no_std]
#![warn(missing_docs)]
use core::marker::PhantomData;

use embedded_hal::blocking::i2c::{Write, WriteRead};
use roi::{ROI, ROICenter};
use threshold::{Threshold, Window};
use core::fmt::Debug;
pub mod threshold;
pub mod roi;

/// Structure of software version.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct SWVersion {
    /// Major version number
    pub major : u8,
    /// Minor version number
    pub minor : u8,
    /// Build version
    pub build : u8,
    /// Revision
    pub revision : u32,
}


/// Driver error.
#[derive(Debug)]
pub enum Error<T> 
where 
    T : Write + WriteRead,
    <T as Write>::Error : Debug,
    <T as WriteRead>::Error : Debug
{
    /// Error occured during write operation with underlying fault from I2C implementation.
    WriteError(<T as Write>::Error),
    /// Error occured during write read operation with underlying fault from I2C implementation.
    WriteReadError(<T as WriteRead>::Error),
    /// The timing budget given is an invalid value.
    InvalidTimingBudget,
    /// The sigma threshold given is an invalid value.
    InvalidDistanceMode,
    /// The sigma threshold given is an invalid value.
    InvalidSigmaThreshold,
}

/// Interrupt polarity.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Polarity {
    /// Interrupt pin is high if the interrupt is active.
    ActiveHigh = 1,
    /// Interrupt pin is high if the interrupt is active.
    ActiveLow = 0
}

impl From<u8> for Polarity {
    fn from(v: u8) -> Self {
        match v {
            0 => Polarity::ActiveHigh,
            _ => Polarity::ActiveLow
        }
    }
}

/// Distance measuring mode.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DistanceMode {
    /// Short mode max distance is limited to 1.3 m but has better ambient immunity.
    Short = 1,
    /// Long mode can range up to 4 m in the dark with a 200 ms timing budget.
    Long = 2
}

/// Status of a measurement.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum RangeStatus {
    /// Valid measurement.
    Valid = 0,
    /// Sigma is above threshold (possibly valid measurement).
    SigmaFailure = 1, 
    /// Signal is above threshold (possibly valid measurement).
    SignalFailure = 2,
    /// Target is below minimum detection threshold.
    MinRangeClipped = 3,
    /// Phase is out of bounds.
    OutOfBounds = 4,
    /// HW or VCSEL failure.
    HardwareFailure = 5,
    /// Valid range, but wraparound check has not been done.
    WrapCheckFail = 6,
    /// Wrapped target, non matching phases.
    Wraparound = 7,
    /// Internal algorithm underflow or overflow.
    ProcessingFailure = 8,
    /// Crosstalk between signals.
    CrosstalkSignal = 9,
    /// First interrupt when starting ranging in back to back mode. Ignore measurement.
    Synchronisation = 10,
    /// Valid measurement but result is from multiple merging pulses.
    MergedPulse = 11,
    /// Used by RQL as different to phase fail.
    LackOfSignal = 12,
    /// Target is below minimum detection threshold.
    MinRangeFail = 13,
    /// Measurement is invalid.
    InvalidRange = 14,
    /// No new data.
    None = 255
}

impl From<u8> for RangeStatus {
    fn from(v: u8) -> Self {
        match v {
            3 => RangeStatus::HardwareFailure,
            4 => RangeStatus::SignalFailure,
            5 => RangeStatus::OutOfBounds,
            6 => RangeStatus::SigmaFailure,
            7 => RangeStatus::Wraparound,
            8 => RangeStatus::MinRangeClipped,
            9 => RangeStatus::Valid,
            12 => RangeStatus::CrosstalkSignal,
            13 => RangeStatus::MinRangeFail,
            18 => RangeStatus::Synchronisation,
            19 => RangeStatus::WrapCheckFail,
            22 => RangeStatus::MergedPulse,
            23 => RangeStatus::LackOfSignal,
            _ => RangeStatus::None
        }
    }
}

/// Voltage of SDA, SCL and GPIO.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum IOVoltage {
    /// Use an IO voltage ov 1.8v
    Volt1_8,
    /// Use an IO voltage ov 2.8v
    Volt2_8
}

/// Result of a block measurement.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MeasureResult {
    /// The status of this result.
	pub status : RangeStatus,
    /// The distance measured for this result (may be 0).
	pub distance_mm : u16,
    /// The measured ambient signal.
	pub ambient : u16,
    /// The measured signal per SPAD.
	pub sig_per_spad : u16,
    /// The number of SPADs used for this measurement.
	pub spad_count : u16,
}

/// Default I2C address for VL53L1X.
pub const DEFAULT_ADDRESS : u8 = 0x29;

/// Instance of a single VL53L1X driver.
pub struct VL53L1X<T> 
where 
    T : Write + WriteRead,
    <T as Write>::Error : Debug,
    <T as WriteRead>::Error : Debug
{
    _i2c : PhantomData<T>,
    address : u8
}

impl<T> VL53L1X<T> 
where 
    T : Write + WriteRead,
    <T as Write>::Error : Debug,
    <T as WriteRead>::Error : Debug
{
    /// Create a new instance of the VL53L1X driver with the given address.
    /// 
    /// # Arguments
    /// 
    /// * `address` - The address to use for communication with the VL53L1X.
    pub fn new(address : u8) -> VL53L1X<T> {
        VL53L1X {
            _i2c : PhantomData{},
            address
        }
    }

    /// Get the driver version.
    pub fn sw_version() -> SWVersion {
        SWVersion{
            major : 3,
            minor : 5,
            build : 1,
            revision : 0
        }
    }

    const VL53L1_I2C_SLAVE_DEVICE_ADDRESS : u16 = 0x01;
    const SYSTEM_INTERRUPT_CLEAR : u16 = 0x86;
    const GPIO_HV_MUX_CTRL : u16 = 0x30;
    const SYSTEM_MODE_START : u16 = 0x87;
    const GPIO_TIO_HV_STATE : u16 = 0x31;
    const RANGE_CONFIG_TIMEOUT_MACROP_A_HI : u16 = 0x5E;
    const RANGE_CONFIG_TIMEOUT_MACROP_B_HI : u16 = 0x61;
    const PHASECAL_CONFIG_TIMEOUT_MACROP : u16 = 0x4B;
    const RANGE_CONFIG_VCSEL_PERIOD_A : u16 = 0x60;
    const RANGE_CONFIG_VCSEL_PERIOD_B : u16 = 0x63;
    const RANGE_CONFIG_VALID_PHASE_HIGH : u16 = 0x69;
    const SD_CONFIG_WOI_SD0 : u16 = 0x78;
    const SD_CONFIG_INITIAL_PHASE_SD0 : u16 = 0x7A;
    const VL53L1_RESULT_OSC_CALIBRATE_VAL : u16 = 0xDE;
    const VL53L1_SYSTEM_INTERMEASUREMENT_PERIOD : u16 = 0x6C;
    const VL53L1_FIRMWARE_SYSTEM_STATUS : u16 = 0xE5;
    const VL53L1_IDENTIFICATION_MODEL_ID : u16 = 0x10F;
    const VL53L1_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 : u16 = 0x96;
    const VL53L1_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 : u16 = 0x98;
    const VL53L1_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0 : u16 = 0x8C;
    const RESULT_AMBIENT_COUNT_RATE_MCPS_SD : u16 = 0x8C;
    const VL53L1_RESULT_RANGE_STATUS : u16 = 0x89;
    const ALGO_PART_TO_PART_RANGE_OFFSET_MM : u16 = 0x1E;
    const MM_CONFIG_INNER_OFFSET_MM : u16 = 0x20;
    const MM_CONFIG_OUTER_OFFSET_MM : u16 = 0x22;
    const ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS : u16 = 0x18;
    const ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS : u16 = 0x1A;
    const ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS : u16 = 0x16;
    const SYSTEM_INTERRUPT_CONFIG_GPIO : u16 = 0x46;
    const SYSTEM_THRESH_HIGH : u16 = 0x72;
    const SYSTEM_THRESH_LOW : u16 = 0x74;
    const VL53L1_ROI_CONFIG_MODE_ROI_CENTRE_SPAD : u16 = 0x13E;
    const ROI_CONFIG_USER_ROI_CENTRE_SPAD : u16 = 0x7F;
    const ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE : u16 = 0x80;
    const RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS : u16 = 0x66;
    const RANGE_CONFIG_SIGMA_THRESH : u16 = 0x64;
    const VL53L1_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND : u16 = 0x08;
    const I2C_CONFIG : u16 = 0x2d;
    const I2C_GPIO_CONFIG : u16 = 0x2e;
    const GPIO_CONFIG : u16 = 0x2f;

    const DEFAULT_CONFG_START_ADDR : u16 = 0x30;
    const DEFAULT_CONFG_END_ADDR : u16 = 0x87;

    const DEFAULT_CONFIG : [u8; 88] = [
        0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
        0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
        0x00, /* 0x32 : not user-modifiable */
        0x02, /* 0x33 : not user-modifiable */
        0x08, /* 0x34 : not user-modifiable */
        0x00, /* 0x35 : not user-modifiable */
        0x08, /* 0x36 : not user-modifiable */
        0x10, /* 0x37 : not user-modifiable */
        0x01, /* 0x38 : not user-modifiable */
        0x01, /* 0x39 : not user-modifiable */
        0x00, /* 0x3a : not user-modifiable */
        0x00, /* 0x3b : not user-modifiable */
        0x00, /* 0x3c : not user-modifiable */
        0x00, /* 0x3d : not user-modifiable */
        0xff, /* 0x3e : not user-modifiable */
        0x00, /* 0x3f : not user-modifiable */
        0x0F, /* 0x40 : not user-modifiable */
        0x00, /* 0x41 : not user-modifiable */
        0x00, /* 0x42 : not user-modifiable */
        0x00, /* 0x43 : not user-modifiable */
        0x00, /* 0x44 : not user-modifiable */
        0x00, /* 0x45 : not user-modifiable */
        0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
        0x0b, /* 0x47 : not user-modifiable */
        0x00, /* 0x48 : not user-modifiable */
        0x00, /* 0x49 : not user-modifiable */
        0x02, /* 0x4a : not user-modifiable */
        0x0a, /* 0x4b : not user-modifiable */
        0x21, /* 0x4c : not user-modifiable */
        0x00, /* 0x4d : not user-modifiable */
        0x00, /* 0x4e : not user-modifiable */
        0x05, /* 0x4f : not user-modifiable */
        0x00, /* 0x50 : not user-modifiable */
        0x00, /* 0x51 : not user-modifiable */
        0x00, /* 0x52 : not user-modifiable */
        0x00, /* 0x53 : not user-modifiable */
        0xc8, /* 0x54 : not user-modifiable */
        0x00, /* 0x55 : not user-modifiable */
        0x00, /* 0x56 : not user-modifiable */
        0x38, /* 0x57 : not user-modifiable */
        0xff, /* 0x58 : not user-modifiable */
        0x01, /* 0x59 : not user-modifiable */
        0x00, /* 0x5a : not user-modifiable */
        0x08, /* 0x5b : not user-modifiable */
        0x00, /* 0x5c : not user-modifiable */
        0x00, /* 0x5d : not user-modifiable */
        0x01, /* 0x5e : not user-modifiable */
        0xcc, /* 0x5f : not user-modifiable */
        0x0f, /* 0x60 : not user-modifiable */
        0x01, /* 0x61 : not user-modifiable */
        0xf1, /* 0x62 : not user-modifiable */
        0x0d, /* 0x63 : not user-modifiable */
        0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
        0x68, /* 0x65 : Sigma threshold LSB */
        0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
        0x80, /* 0x67 : Min count Rate LSB */
        0x08, /* 0x68 : not user-modifiable */
        0xb8, /* 0x69 : not user-modifiable */
        0x00, /* 0x6a : not user-modifiable */
        0x00, /* 0x6b : not user-modifiable */
        0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
        0x00, /* 0x6d : Intermeasurement period */
        0x0f, /* 0x6e : Intermeasurement period */
        0x89, /* 0x6f : Intermeasurement period LSB */
        0x00, /* 0x70 : not user-modifiable */
        0x00, /* 0x71 : not user-modifiable */
        0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
        0x00, /* 0x73 : distance threshold high LSB */
        0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
        0x00, /* 0x75 : distance threshold low LSB */
        0x00, /* 0x76 : not user-modifiable */
        0x01, /* 0x77 : not user-modifiable */
        0x0f, /* 0x78 : not user-modifiable */
        0x0d, /* 0x79 : not user-modifiable */
        0x0e, /* 0x7a : not user-modifiable */
        0x0e, /* 0x7b : not user-modifiable */
        0x00, /* 0x7c : not user-modifiable */
        0x00, /* 0x7d : not user-modifiable */
        0x02, /* 0x7e : not user-modifiable */
        0xc7, /* 0x7f : ROI center, use SetROI() */
        0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
        0x9B, /* 0x81 : not user-modifiable */
        0x00, /* 0x82 : not user-modifiable */
        0x00, /* 0x83 : not user-modifiable */
        0x00, /* 0x84 : not user-modifiable */
        0x01, /* 0x85 : not user-modifiable */
        0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
        0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
    ];

    fn write_bytes(&self, i2c : &mut T, address : u16, bytes : &[u8]) -> Result<(), Error<T>> {
        let mut v = [0; 6];

        for (dst, src) in v.iter_mut().zip(address.to_be_bytes()) {
            *dst = src;
        }

        for (dst, src) in v.iter_mut().skip(2).zip(bytes) {
            *dst = *src;
        }

        if let Err(e) = i2c.write(self.address, v.as_slice()) {
            return Err(Error::WriteError(e));
        }

        Ok(())
    }

    fn read_bytes(&self, i2c : &mut T, address : u16, bytes : &mut [u8]) -> Result<(), Error<T>> {

        if let Err(e) = i2c.write_read(self.address, &address.to_be_bytes(), bytes) {
            return Err(Error::WriteReadError(e));
        }

        Ok(())
    }


    /// Set the i2c address of the current device in case multiple devices with the same address exists on the same bus.
    /// 
    /// # Arguments
    /// 
    /// * `new_address` - The new address to set for the current device.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_address(&mut self, i2c : &mut T, new_address : u8) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::VL53L1_I2C_SLAVE_DEVICE_ADDRESS, &[new_address])
    }

    /// Load the 88 byte default values for sensor initialisation.
    /// 
    /// # Arguments
    /// 
    /// * `io_config` - The io voltage that will be configured for the device.
    /// * `i2c` - I2C instance used for communication.
    pub fn init(&self, i2c : &mut T, io_config : IOVoltage) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::I2C_CONFIG, &[0])?;

        let io = match io_config {
            IOVoltage::Volt1_8 => 0,
            IOVoltage::Volt2_8 => 1,
        };

        self.write_bytes(i2c, Self::GPIO_CONFIG, &[io])?;
        self.write_bytes(i2c, Self::I2C_GPIO_CONFIG, &[io])?;

        for addr in Self::DEFAULT_CONFG_START_ADDR..Self::DEFAULT_CONFG_END_ADDR {
            self.write_bytes(i2c, addr, &[Self::DEFAULT_CONFIG[(addr - Self::DEFAULT_CONFG_START_ADDR) as usize]])?;
        }

        self.start_ranging(i2c)?;

        while !self.is_data_ready(i2c)? {}

        self.clear_interrupt(i2c)?;
        self.stop_ranging(i2c)?;

        self.write_bytes(i2c, Self::VL53L1_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, &[0x09])?;
        self.write_bytes(i2c, 0x0B, &[0])?;

        Ok(())
    }

    /// Clear the interrupt flag on the device. 
    /// Should be called after reading ranging data from the device to start the next measurement.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn clear_interrupt(&self, i2c : &mut T) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::SYSTEM_INTERRUPT_CLEAR, &[0x01])
    }

    /// Set polarity of the interrupt.
    /// 
    /// # Arguments
    /// 
    /// * `polarity` - The polarity to set.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_interrupt_polarity(&self, i2c : &mut T, polarity : Polarity) -> Result<(), Error<T>> {
        
        let mut gpio_mux_hv = [0u8];

        self.read_bytes(i2c, Self::GPIO_HV_MUX_CTRL, &mut gpio_mux_hv)?;

        gpio_mux_hv[0] &= 0xEF;

        gpio_mux_hv[0] |= (polarity as u8) << 4;

        self.write_bytes(i2c, Self::GPIO_HV_MUX_CTRL, &gpio_mux_hv)
    }

    /// Get the currently set interrupt polarity
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_interrupt_polarity(&self, i2c : &mut T) -> Result<Polarity, Error<T>> {
        let mut gpio_mux_hv = [0u8];

        self.read_bytes(i2c, Self::GPIO_HV_MUX_CTRL, &mut gpio_mux_hv)?;


        Ok((gpio_mux_hv[0] & 0x10).into())
    }

    /// Start a distance ranging operation.
    /// The operation is continuous, the interrupt flag should be cleared between each interrupt to start a new distance measurement.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn start_ranging(&self, i2c : &mut T) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::SYSTEM_MODE_START, &[0x40])

    }

    /// Stop an ongoing ranging operation.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn stop_ranging(&self, i2c : &mut T) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::SYSTEM_MODE_START, &[0x00])
    }

    /// Check if new ranging data is available by polling the device.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn is_data_ready(&self, i2c : &mut T) -> Result<bool, Error<T>> {
        let polarity = self.get_interrupt_polarity(i2c)? as u8;

        let mut state = [0u8];
        
        self.read_bytes(i2c, Self::GPIO_TIO_HV_STATE, &mut state)?;

        if (state[0] & 0x01) == polarity {
            return Ok(true)
        }

        Ok(false)
    }

    /// Set the timing budget for a measurement operation in milliseconds.
    /// 
    /// # Arguments
    /// 
    /// * `milliseconds` - One of the following values = 15, 20, 33, 50, 100(default), 200, 500.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_timing_budget_ms(&self, i2c : &mut T, milliseconds : u16) -> Result<(), Error<T>> {
        let mode = self.get_distance_mode(i2c)?;

        let (a, b) = match mode {
            DistanceMode::Short => {
                match milliseconds {
                    15 => (0x01Du16, 0x0027u16),
                    20 => (0x051, 0x006E),
                    33 => (0x00D6, 0x006E),
                    50 => (0x1AE, 0x01E8),
                    100 => (0x02E1, 0x0388),
                    200 => (0x03E1, 0x0496),
                    500 => (0x0591, 0x05C1),
                    _ => return Err(Error::InvalidTimingBudget)
                }
            },
            DistanceMode::Long => {
                match milliseconds {
                    20 => (0x001E, 0x0022),
                    33 => (0x0060, 0x006E),
                    50 => (0x00AD, 0x00C6),
                    100 => (0x01CC, 0x01EA),
                    200 => (0x02D9, 0x02F8),
                    500 => (0x048F, 0x04A4),
                    _ => return Err(Error::InvalidTimingBudget)
                }
            },
        };

        self.write_bytes(i2c, Self::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, &a.to_be_bytes())?;
        self.write_bytes(i2c, Self::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, &b.to_be_bytes())?;

        Ok(())
    }

    /// Get the currently set timing budget of the device.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_timing_budget_ms(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut a = [0u8, 0];

        self.read_bytes(i2c, Self::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, &mut a)?;

        Ok(match u16::from_be_bytes(a) {
            0x001D => 15,
            0x0051 | 0x001E => 20,
            0x00D6 | 0x0060 => 33,
            0x1AE  | 0x00AD => 50,
            0x02E1 | 0x01CC => 100,
            0x03E1 | 0x02D9 => 200,
            0x0591 | 0x048F => 500,
            _ => return Err(Error::InvalidTimingBudget),
        })
    }

    /// Set the distance mode in which the device operates.
    /// 
    /// # Arguments
    /// 
    /// * `mode` - The distance mode to use.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_distance_mode(&self, i2c : &mut T, mode : DistanceMode) -> Result<(), Error<T>> {
        let tb = self.get_timing_budget_ms(i2c)?;

        let (timeout, vcsel_a, vcsel_b, phase, woi_sd0, phase_sd0) = match mode {
            DistanceMode::Short => (0x14u8, 0x07u8, 0x05u8, 0x38u8, 0x0705u16, 0x0606u16),
            DistanceMode::Long => (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),
        };

        self.write_bytes(i2c, Self::PHASECAL_CONFIG_TIMEOUT_MACROP, &timeout.to_be_bytes())?;
        self.write_bytes(i2c, Self::RANGE_CONFIG_VCSEL_PERIOD_A, &vcsel_a.to_be_bytes())?;
        self.write_bytes(i2c, Self::RANGE_CONFIG_VCSEL_PERIOD_B, &vcsel_b.to_be_bytes())?;
        self.write_bytes(i2c, Self::RANGE_CONFIG_VALID_PHASE_HIGH, &phase.to_be_bytes())?;
        self.write_bytes(i2c, Self::SD_CONFIG_WOI_SD0, &woi_sd0.to_be_bytes())?;
        self.write_bytes(i2c, Self::SD_CONFIG_INITIAL_PHASE_SD0, &phase_sd0.to_be_bytes())?;

        self.set_timing_budget_ms(i2c, tb)?;

        Ok(())
    }

    /// Get the currently set distance mode of the device.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_distance_mode(&self, i2c : &mut T) -> Result<DistanceMode, Error<T>> {
        let mut out =[0u8];
        self.read_bytes(i2c, Self::PHASECAL_CONFIG_TIMEOUT_MACROP, &mut out)?;

        if out[0] == 0x14 {
            Ok(DistanceMode::Short)
        } else if out[0] == 0x0A {
            Ok(DistanceMode::Long)
        } else {
            Err(Error::InvalidDistanceMode)
        }
    }

    /// Set the inter measurement period in milliseconds.
    /// This value must be greater or equal to the timing budget. This condition is not checked by this driver.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `milliseconds` - The number of milliseconds used for the inter measurement period.
    pub fn set_inter_measurement_period_ms(&self, i2c : &mut T, milliseconds : u16) -> Result<(), Error<T>> {
        let mut clock_pll = [0u8, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_OSC_CALIBRATE_VAL, &mut clock_pll)?;

        let clock_pll = u16::from_be_bytes(clock_pll) & 0x3FF;

        let val = ((clock_pll * milliseconds) as f32 * 1.075f32) as u32;

        self.write_bytes(i2c, Self::VL53L1_SYSTEM_INTERMEASUREMENT_PERIOD, &val.to_be_bytes())?;

        Ok(())
    }

    /// Get the currently set inter measurement period in milliseconds.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_inter_measurement_period_ms(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut clock_pll = [0u8, 0];
        let mut period = [0u8, 0, 0, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_OSC_CALIBRATE_VAL, &mut clock_pll)?;

        let clock_pll = u16::from_be_bytes(clock_pll) & 0x3FF;

        self.read_bytes(i2c, Self::VL53L1_SYSTEM_INTERMEASUREMENT_PERIOD, &mut period)?;

        let period = u32::from_be_bytes(period);

        Ok((period/(clock_pll as f32 * 1.065f32) as u32) as u16)
    }

    /// Check if the device is booted.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn is_booted(&self, i2c : &mut T) -> Result<bool, Error<T>> {
        let mut status = [0u8];
        self.read_bytes(i2c, Self::VL53L1_FIRMWARE_SYSTEM_STATUS, &mut status)?;

        Ok(status[0] == 1)
    }

    /// Get the sensor id of the sensor. This id must be 0xEACC.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_sensor_id(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        
        let mut id = [0u8, 0];

        self.read_bytes(i2c, Self::VL53L1_IDENTIFICATION_MODEL_ID, &mut id)?;

        Ok(u16::from_be_bytes(id))
    }

    /// Get the distance measured in millimeters.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_distance(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        
        let mut distance = [0u8, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &mut distance)?;

        Ok(u16::from_be_bytes(distance))
    }

    /// Get the returned signal per SPAD in kcps/SPAD where kcps stands for Kilo Count Per Second.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_signal_per_spad(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut signal = [0, 0];
        let mut spad_count = [0, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &mut signal)?;
        self.read_bytes(i2c, Self::VL53L1_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &mut spad_count)?;

        let signal = u16::from_be_bytes(signal);
        let spad_count = u16::from_be_bytes(spad_count);

        Ok((200.0f32*signal as f32/spad_count as f32) as u16)
    }

    /// Get the ambient signal per SPAD in kcps/SPAD.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_ambient_per_spad(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut spad_count = [0, 0];
        let mut ambient = [0, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &mut spad_count)?;
        self.read_bytes(i2c, Self::RESULT_AMBIENT_COUNT_RATE_MCPS_SD, &mut ambient)?;

        let spad_count = u16::from_be_bytes(spad_count);
        let ambient = u16::from_be_bytes(ambient);

        Ok((200.0f32*ambient as f32/spad_count as f32) as u16)
    }

    /// Get the returned signal in kcps.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_signal_rate(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut signal = [0, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &mut signal)?;

        Ok(u16::from_be_bytes(signal) * 8)
    }

    /// Get the count of currently enabled SPADs.77
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_spad_count(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut spad_count = [0, 0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &mut spad_count)?;

        Ok(u16::from_be_bytes(spad_count) >> 8)
    }

    /// Get the ambient signal in kcps.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_ambient_rate(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut ambient = [0, 0];

        self.read_bytes(i2c, Self::RESULT_AMBIENT_COUNT_RATE_MCPS_SD, &mut ambient)?;

        Ok(u16::from_be_bytes(ambient) * 8)
    }

    /// Get the ranging status.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_range_status(&self, i2c : &mut T) -> Result<RangeStatus, Error<T>> {
        
        let mut status = [0];

        self.read_bytes(i2c, Self::VL53L1_RESULT_RANGE_STATUS, &mut status)?;

        let status = u8::from_be_bytes(status) & 0x1F;

        Ok(status.into())
    }

    /// Get the measure result object which is read in a single access
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_result(&self, i2c : &mut T) -> Result<MeasureResult, Error<T>> {
        let mut result = [0; 17];

        self.read_bytes(i2c, Self::VL53L1_RESULT_RANGE_STATUS, &mut result)?;

        Ok(
            MeasureResult {
                status : (result[0] & 0x1F).into(),
                ambient : u16::from_be_bytes([result[7], result[8]]) * 8,
                spad_count : result[3] as u16,
                sig_per_spad : u16::from_be_bytes([result[15], result[16]]) * 8,
                distance_mm : u16::from_be_bytes([result[13], result[14]])
            }
        )
    }

    /// Set the offset correction in millimeters.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `offset` - The offset in millimeters.
    pub fn set_offset(&self, i2c : &mut T, offset : i16) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::ALGO_PART_TO_PART_RANGE_OFFSET_MM, &(offset*4).to_be_bytes())?;

        self.write_bytes(i2c, Self::MM_CONFIG_INNER_OFFSET_MM, &[0, 0])?;
        self.write_bytes(i2c, Self::MM_CONFIG_OUTER_OFFSET_MM, &[0, 0])?;
        
        Ok(())
    }

    /// Get the currently set offset correction in millimeters.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_offset(&self, i2c : &mut T) -> Result<i16, Error<T>> {
        
        let mut offset = [0, 0];

        self.read_bytes(i2c, Self::ALGO_PART_TO_PART_RANGE_OFFSET_MM, &mut offset)?;

        let mut offset = u16::from_be_bytes(offset) << 3;

        offset >>= 5;

        Ok(offset as i16)
    }

    /// Set the crosstalk correction value in cps (Count Per Second).
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `correction` - The number of photons reflected back from the cover glass in cps.
    pub fn set_cross_talk(&self, i2c : &mut T, correction : u16) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, &[0, 0])?;
        self.write_bytes(i2c, Self::ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, &[0, 0])?;

        let correction = (correction << 9) / 1000;

        self.write_bytes(i2c, Self::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &correction.to_be_bytes())?;

        Ok(())
    }

    /// Get the crosstalk correction value in cps.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_cross_talk(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        
        let mut correction = [0, 0];

        self.read_bytes(i2c, Self::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &mut correction)?;

        let correction = (u16::from_be_bytes(correction) * 1000) >> 9;

        Ok(correction)
    }

    /// Set a distance threshold.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `threshold` - The threshold to apply.
    pub fn set_distance_threshold(&self, i2c : &mut T, threshold : Threshold) -> Result<(), Error<T>> {
        let mut config = [0];

        self.read_bytes(i2c, Self::SYSTEM_INTERRUPT_CONFIG_GPIO, &mut config)?;

        let config = config[0] & 0x47 | (threshold.window as u8 & 0x07);

        self.write_bytes(i2c, Self::SYSTEM_INTERRUPT_CONFIG_GPIO, &[config])?;
        self.write_bytes(i2c, Self::SYSTEM_THRESH_HIGH, &threshold.high.to_be_bytes())?;
        self.write_bytes(i2c, Self::SYSTEM_THRESH_LOW, &threshold.low.to_be_bytes())?;

        Ok(())
    }

    /// Get the currently set distance threshold.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_distance_threshold(&self, i2c : &mut T) -> Result<Threshold, Error<T>> {
        let mut config = [0];

        self.read_bytes(i2c, Self::SYSTEM_INTERRUPT_CONFIG_GPIO, &mut config)?;

        let window : Window = (config[0] & 0x07).into();

        let mut high = [0, 0];
        let mut low = [0, 0];

        self.read_bytes(i2c, Self::SYSTEM_THRESH_HIGH, &mut high)?;
        self.read_bytes(i2c, Self::SYSTEM_THRESH_LOW, &mut low)?;

        Ok(Threshold {
            window,
            low : u16::from_be_bytes(low),
            high : u16::from_be_bytes(high),
        })
    }

    /// Set the region of interest of the sensor. The ROI is centered and only the size is settable.
    /// The smallest acceptable ROI size is 4.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `roi` - The ROI to apply.
    pub fn set_roi(&self, i2c : &mut T, mut roi : ROI) -> Result<(), Error<T>> {
        
        debug_assert!(roi.width >= 4);
        debug_assert!(roi.height >= 4);

        let mut center  = [0];

        self.read_bytes(i2c, Self::VL53L1_ROI_CONFIG_MODE_ROI_CENTRE_SPAD, &mut center)?;

        if roi.width > 16 {
            roi.width = 16;
        }

        if roi.height > 16 {
            roi.height = 16;
        }

        if roi.width > 10 || roi.height > 10 {
            center[0] = 199;
        }

        let config = ((roi.height - 1) << 4 | (roi.width - 1)) as u8;

        self.write_bytes(i2c, Self::ROI_CONFIG_USER_ROI_CENTRE_SPAD, &center)?;
        self.write_bytes(i2c, Self::ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &[config])?;

        Ok(())
    }

    /// Get the currenly set ROI.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_roi(&self, i2c : &mut T) -> Result<ROI, Error<T>> {
        
        let mut config = [0];

        self.read_bytes(i2c, Self::ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &mut config)?;

        Ok(ROI {
            width : ((config[0] & 0x0F) + 1) as u16,
            height : (((config[0] >> 4) & 0x0F) + 1) as u16,
        })
    }

    /// Set the new ROI center. 
    /// If the new ROI clips out of the border this function does not return an error but only when ranging is started will an error be returned
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `center` - Tne ROI center to apply.
    pub fn set_roi_center(&self, i2c : &mut T, center : ROICenter) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::ROI_CONFIG_USER_ROI_CENTRE_SPAD, &[center.spad])
    }

    /// Get the current ROI center.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_roi_center(&self, i2c : &mut T) -> Result<ROICenter, Error<T>> {

        let mut center = [0];

        self.read_bytes(i2c, Self::ROI_CONFIG_USER_ROI_CENTRE_SPAD, &mut center)?;

        Ok(ROICenter {
            spad : center[0]
        })    }

    /// Set a signal threshold in kcps. Default is 1024
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `threshold` - The signal threshold.
    pub fn set_signal_threshold(&self, i2c : &mut T, threshold : u16) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &(threshold >> 3).to_be_bytes())
    }

    /// Get the currently set signal threshold.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_signal_threshold(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut threshold = [0, 0];

        self.read_bytes(i2c, Self::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &mut threshold)?;

        Ok(u16::from_be_bytes(threshold) << 3)    }

    /// Set a sigma threshold in millimeter. Default is 15.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `threshold` - The sigma threshold.
    pub fn set_sigma_threshold(&self, i2c : &mut T, threshold : u16) -> Result<(), Error<T>> {

        if threshold > (0xFFFF >> 2) {
            return Err(Error::InvalidSigmaThreshold);
        }

        self.write_bytes(i2c, Self::RANGE_CONFIG_SIGMA_THRESH, &(threshold << 2).to_be_bytes())    }

    /// Get the currently set sigma threshold.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_sigma_threshold(&self, i2c : &mut T) -> Result<u16, Error<T>> {
        let mut threshold = [0, 0];

        self.read_bytes(i2c, Self::RANGE_CONFIG_SIGMA_THRESH, &mut threshold)?;

        Ok(u16::from_be_bytes(threshold) >> 2)    }

    /// Perform temperature calibration of the sensor. 
    /// It is recommended to call this function any time the temperature might have changed by more than 8 deg Celsius
    /// without sensor ranging activity for an extended period.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn calibrate_temperature(&self, i2c : &mut T) -> Result<(), Error<T>> {
        self.write_bytes(i2c, Self::VL53L1_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, &0x81u16.to_be_bytes())?;
        self.write_bytes(i2c, 0x0Bu16, &0x92u16.to_be_bytes())?;

        self.start_ranging(i2c)?;

        while !self.is_data_ready(i2c)? {}

        self.clear_interrupt(i2c)?;
        self.stop_ranging(i2c)?;

        self.write_bytes(i2c, Self::VL53L1_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, &0x09u16.to_be_bytes())?;
        self.write_bytes(i2c, 0x0Bu16, &0u16.to_be_bytes())?;

        Ok(())
    }

    /// Perform offset calibration. 
    /// The function returns the offset value found and sets it as the new offset.
    /// Target reflectance = grey17%
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `target_distance_mm` - Distance to the target in millimeters, ST recommends 100 mm.
    pub fn calibrate_offset(&self, i2c : &mut T, target_distance_mm : u16) -> Result<i16, Error<T>> {
        self.write_bytes(i2c, Self::ALGO_PART_TO_PART_RANGE_OFFSET_MM, &0u16.to_be_bytes())?;
        self.write_bytes(i2c, Self::MM_CONFIG_INNER_OFFSET_MM, &0u16.to_be_bytes())?;
        self.write_bytes(i2c, Self::MM_CONFIG_OUTER_OFFSET_MM, &0u16.to_be_bytes())?;

        self.start_ranging(i2c)?;

        let mut average_distance = 0;

        for _ in 0..50 {
            while self.is_data_ready(i2c)? {}

            average_distance += self.get_distance(i2c)?;
            self.clear_interrupt(i2c)?;
        }

        self.stop_ranging(i2c)?;

        average_distance /= 50;

        let offset = target_distance_mm as i16 - average_distance as i16;

        self.set_offset(i2c, offset)?;

        Ok(offset)    }

    /// Perform crosstalk calibration.
    /// The function returns the crosstalk value found and set it as the new crosstalk.
    /// Target reflectance = grey 17%
    /// 
    ///  Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    /// * `target_distance_mm` - Distance to the target in millimeters, .
    pub fn calibrate_cross_talk(&self, i2c : &mut T, target_distance_mm : u16) -> Result<u16, Error<T>> {
        self.write_bytes(i2c, Self::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &0u16.to_be_bytes())?;

        self.start_ranging(i2c)?;

        let mut average_distance = 0;
        let mut average_spad_cnt = 0;
        let mut average_signal_rate = 0;

        for _ in 0..50 {
            while self.is_data_ready(i2c)? {}

            average_distance += self.get_distance(i2c)?;
            average_signal_rate += self.get_signal_rate(i2c)?;
            self.clear_interrupt(i2c)?;
            average_spad_cnt += self.get_spad_count(i2c)?;
        }

        self.stop_ranging(i2c)?;

        average_distance /= 50;
        average_spad_cnt /= 50;
        average_signal_rate /= 50;

        let mut calibrate_val = 512*(average_signal_rate as u32*(1-(average_distance as u32/target_distance_mm as u32)))/average_spad_cnt as u32;

        if calibrate_val > 0xFFFF {
            calibrate_val = 0xFFFF;
        }

        let config = ((calibrate_val as u16 * 1000) >> 9).to_be_bytes();

        self.write_bytes(i2c, Self::ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &config)?;

        Ok(0)
    }
}