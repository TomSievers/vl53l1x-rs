//! Port of the STM IMG009 Ultra Lite Driver for the VL53L1X.
#![no_std]
#![warn(missing_docs)]
use core::marker::PhantomData;

use embedded_hal::blocking::i2c::{Write, WriteRead};
use roi::{ROI, ROICenter};
use threshold::{Threshold};
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
pub enum Error<T : Write + WriteRead> {
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
pub struct VL53L1X<T : WriteRead + Write> {
    _i2c : PhantomData<T>,
    address : u8
}

impl<T : WriteRead + Write> VL53L1X<T> {

    

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
    pub fn sw_version() -> Result<SWVersion, Error<T>> {
        Ok(SWVersion{
            major : 3,
            minor : 5,
            build : 1,
            revision : 0
        })
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


    /// Set the i2c address of the current device in case multiple devices with the same address exists on the same bus.
    /// 
    /// # Arguments
    /// 
    /// * `new_address` - The new address to set for the current device.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_address(&mut self, i2c : T, new_address : u8) -> Result<(), Error<T>> {
        todo!()
    }

    /// Load the 88 byte default values for sensor initialisation.
    /// 
    /// # Arguments
    /// 
    /// * `io_config` - The io voltage that will be configured for the device.
    /// * `i2c` - I2C instance used for communication.
    pub fn init(&self, i2c : T, io_config : IOVoltage) -> Result<(), Error<T>> {
        todo!()
    }

    /// Clear the interrupt flag on the device. 
    /// Should be called after reading ranging data from the device to start the next measurement.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn clear_interrupt(&self, i2c : T) -> Result<(), Error<T>> {
        todo!()
    }

    /// Set polarity of the interrupt.
    /// 
    /// # Arguments
    /// 
    /// * `polarity` - The polarity to set.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_interrupt_polarity(&self, i2c : T, polarity : Polarity) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set interrupt polarity
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_interrupt_polarity(&self, i2c : T) -> Result<Polarity, Error<T>> {
        todo!()
    }

    /// Start a distance ranging operation.
    /// The operation is continuous, the interrupt flag should be cleared between each interrupt to start a new distance measurement.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn start_ranging(&self, i2c : T) -> Result<(), Error<T>> {
        todo!()
    }

    /// Stop an ongoing ranging operation.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn stop_ranging(&self, i2c : T) -> Result<(), Error<T>> {
        todo!()
    }

    /// Check if new ranging data is available by polling the device.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn is_data_ready(&self, i2c : T) -> Result<bool, Error<T>> {
        todo!()
    }

    /// Set the timing budget for a measurement operation in milliseconds.
    /// 
    /// # Arguments
    /// 
    /// * `milliseconds` - One of the following values = 15, 20, 33, 50, 100(default), 200, 500.
    /// * `i2c` - I2C instance used for communication.
    pub fn set_timing_budget_ms(&self, i2c : T, milliseconds : u16) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set timing budget of the device.
    /// 
    /// # Arguments
    /// 
    /// * `i2c` - I2C instance used for communication.
    pub fn get_timing_budget_ms(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Set the distance mode in which the device operates.
    /// 
    /// # Arguments
    /// 
    /// * `mode` - The distance mode to use.
    pub fn set_distance_mode(&self, i2c : T, mode : DistanceMode) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set distance mode of the device.
    pub fn get_distance_mode(&self, i2c : T) -> Result<DistanceMode, Error<T>> {
        todo!()
    }

    /// Set the inter measurement period in milliseconds.
    /// This value must be greater or equal to the timing budget. This condition is not checked by this driver.
    /// 
    /// # Arguments
    /// 
    /// * `milliseconds` - The number of milliseconds used for the inter measurement period.
    pub fn set_inter_measurement_period_ms(&self, i2c : T, milliseconds : u16) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set inter measurement period in milliseconds.
    pub fn get_inter_measurement_period_ms(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Check if the device is booted.
    pub fn is_booted(&self, i2c : T) -> Result<bool, Error<T>> {
        todo!()
    }

    /// Get the sensor id of the sensor. This id must be 0xEACC
    pub fn get_sensor_id(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the distance measured in millimeters.
    pub fn get_distance(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the returned signal per SPAD in kcps/SPAD where kcps stands for Kilo Count Per Second
    pub fn get_signal_per_spad(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the ambient signal per SPAD in kcps/SPAD.
    pub fn get_ambient_per_spad(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the returned signal in kcps.
    pub fn get_signal_rate(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the count of currently enabled SPADs
    pub fn get_spad_count(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the ambient signal in kcps.
    pub fn get_ambient_rate(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Get the ranging status.
    pub fn get_range_status(&self, i2c : T) -> Result<RangeStatus, Error<T>> {
        todo!()
    }

    /// Get the measure result object which is read in a single access
    pub fn get_result(&self, i2c : T) -> Result<MeasureResult, Error<T>> {
        todo!()
    }

    /// Set the offset correction in millimeters.
    /// 
    /// # Arguments
    /// 
    /// * `offset` - The offset in millimeters.
    pub fn set_offset(&self, i2c : T, offset : i16) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set offset correction in millimeters
    pub fn get_offset(&self, i2c : T) -> Result<i16, Error<T>> {
        todo!()
    }

    /// Set the crosstalk correction value in cps (Count Per Second).
    /// 
    /// # Arguments
    /// 
    /// * `correction` - The number of photons reflected back from the cover glass in cps.
    pub fn set_cross_talk(&self, i2c : T, correction : u16) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the crosstalk correction value in cps.
    pub fn get_cross_talk(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Set a distance threshold.
    /// 
    /// # Arguments
    /// 
    /// * `threshold` - The threshold to apply.
    pub fn set_distance_threshold(&self, i2c : T, threshold : Threshold) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set distance threshold.
    pub fn get_distance_threshold(&self, i2c : T) -> Result<Threshold, Error<T>> {
        todo!()
    }

    /// Set the region of interest of the sensor. The ROI is centered and only the size is settable.
    /// The smallest acceptable ROI size is 4.
    /// 
    /// # Arguments
    /// 
    /// * `roi` - The ROI to apply.
    pub fn set_roi(&self, i2c : T, mut roi : ROI) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currenly set ROI.
    pub fn get_roi(&self, i2c : T) -> Result<ROI, Error<T>> {
        todo!()
    }

    /// Set the new ROI center. 
    /// If the new ROI clips out of the border this function does not return an error but only when ranging is started will an error be returned
    /// 
    /// # Arguments
    /// 
    /// * `center` - Tne ROI center to apply.
    pub fn set_roi_center(&self, i2c : T, center : ROICenter) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the current ROI center.
    pub fn get_roi_center(&self, i2c : T) -> Result<ROICenter, Error<T>> {
        todo!()
    }

    /// Set a signal threshold in kcps. Default is 1024
    /// 
    /// # Arguments
    /// 
    /// * `threshold` - The signal threshold.
    pub fn set_signal_threshold(&self, i2c : T, threshold : u16) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set signal threshold.
    pub fn get_signal_threshold(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Set a sigma threshold in millimeter. Default is 15.
    /// 
    /// # Arguments
    /// 
    /// * `threshold` - The sigma threshold.
    pub fn set_sigma_threshold(&self, i2c : T, threshold : u16) -> Result<(), Error<T>> {
        todo!()
    }

    /// Get the currently set sigma threshold.
    pub fn get_sigma_threshold(&self, i2c : T) -> Result<u16, Error<T>> {
        todo!()
    }

    /// Perform temperature calibration of the sensor. 
    /// It is recommended to call this function any time the temperature might have changed by more than 8 deg Celsius
    /// without sensor ranging activity for an extended period.
    pub fn calibrate_temperature(&self, i2c : T) -> Result<(), Error<T>> {
        todo!()
    }

    /// Perform offset calibration. 
    /// The function returns the offset value found and sets it as the new offset.
    /// Target reflectance = grey17%
    /// 
    /// # Arguments
    /// 
    /// * `target_distance_mm` - Distance to the target in millimeters, ST recommends 100 mm.
    pub fn calibrate_offset(&self, i2c : T, target_distance_mm : u16) -> Result<i16, Error<T>> {
        todo!()
    }

    /// Perform crosstalk calibration.
    /// The function returns the crosstalk value found and set it as the new crosstalk.
    /// Target reflectance = grey 17%
    /// 
    /// # Arguments
    /// 
    /// * `target_distance_mm` - Distance to the target in millimeters, .
    pub fn calibrate_cross_talk(&self, i2c : T, target_distance_mm : u16) -> Result<u16, Error<T>> {
        todo!()
    }
}