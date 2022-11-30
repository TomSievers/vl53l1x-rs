//! Port of the STM IMG009 Ultra Lite Driver for the VL53L1X.
//!
//! # Features
//!
//! This crate has one feature called `i2c-iter`.
//! This feature changes the communication implementation of the sensor.
//! With this feature enabled the I2C instance is expected to implement
//! the [`WriteIter`](embedded_hal::blocking::i2c::WriteIter) and [`WriteIterRead`](embedded_hal::blocking::i2c::WriteIterRead) traits instead of [`Write`](embedded_hal::blocking::i2c::Write) and [`WriteRead`]((embedded_hal::blocking::i2c::Write)) traits.
//! The iterator implementation has the advantage that a call to [`write_bytes`](crate::VL53L1X::write_bytes())
//! is not limited to a slice of 4 bytes.
//!
//! # Example
//!
//! ```
//! use vl53l1x_uld::{self, VL53L1X, IOVoltage, RangeStatus};
//! # use embedded_hal_mock::i2c::{Mock, Transaction};
//! # use cfg_if::cfg_if;
//! #
//! # fn create_i2c() -> Mock {
//! #    let mut expectations = vec![
//! #        Transaction::write_read(0x29, vec![0x01, 0x0F], vec![0xEA, 0xCC]),
//! #        Transaction::write(0x29, vec![0x00, 0x2D, 0x00]),
//! #        Transaction::write(0x29, vec![0x00, 0x2F, 0x01]),
//! #        Transaction::write(0x29, vec![0x00, 0x2E, 0x01]),
//! #    ];
//! #
//! #    cfg_if! {
//! #        if #[cfg(feature = "i2c-iter")] {
//! #            expectations.push(Transaction::write(0x29, vec![0x00, 0x30].iter().chain(VL53L1X::<Mock>::DEFAULT_CONFIG.iter()).cloned().collect()));
//! #        } else {
//! #            for (byte, address) in VL53L1X::<Mock>::DEFAULT_CONFIG.iter().zip(0x30u16..0x88) {
//! #                let adrs = address.to_be_bytes();
//! #                expectations.push(Transaction::write(0x29, vec![adrs[0], adrs[1], *byte]));
//! #            }
//! #        }
//! #    }
//! #
//! #    expectations.append(&mut vec![
//! #        Transaction::write(0x29, vec![0x00, 0x87, 0x40]),
//! #        Transaction::write_read(0x29, vec![0x00, 0x30], vec![0x00]),
//! #        Transaction::write_read(0x29, vec![0x00, 0x31], vec![0x01]),
//! #        Transaction::write(0x29, vec![0x00, 0x86, 0x01]),
//! #        Transaction::write(0x29, vec![0x00, 0x87, 0x00]),
//! #        Transaction::write(0x29, vec![0x00, 0x08, 0x09]),
//! #        Transaction::write(0x29, vec![0x00, 0x0B, 0x00]),
//! #        Transaction::write(0x29, vec![0x00, 0x87, 0x40]),
//! #        Transaction::write_read(0x29, vec![0x00, 0x30], vec![0x00]),
//! #        Transaction::write_read(0x29, vec![0x00, 0x31], vec![0x01]),
//! #        Transaction::write_read(0x29, vec![0x00, 0x89], vec![0x09]),
//! #        Transaction::write_read(0x29, vec![0x00, 0x96], vec![0x00, 0x0F]),
//! #    ]);
//! #
//! #    Mock::new(&expectations)
//! # }
//! #
//! // Create hardware specific I2C instance.
//! let i2c = create_i2c();
//! // Create sensor with default address.
//! let mut vl = VL53L1X::new(i2c, vl53l1x_uld::DEFAULT_ADDRESS);
//!
//! const ERR : &str = "Failed to communicate";
//!
//! // Check if the sensor id is correct.
//! if (vl.get_sensor_id().expect(ERR) == 0xEACC)
//! {
//!     // Initialize the sensor before any usage.
//!     // Set the voltage of the IO pins to be 2.8 volts
//!     vl.init(IOVoltage::Volt2_8).expect(ERR);
//!
//!     // Start a ranging operation, needed to retrieve a distance
//!     vl.start_ranging().expect(ERR);
//!     
//!     // Wait until distance data is ready to be read.
//!     while !vl.is_data_ready().expect(ERR) {}
//!
//!     // Check if ditance measurement is valid.
//!     if (vl.get_range_status().expect(ERR) == RangeStatus::Valid)
//!     {
//!         // Retrieve measured distance.
//!         let distance = vl.get_distance().expect(ERR);
//!     }
//! }
//!
//!
//! ```
#![no_std]
#![warn(missing_docs)]

use cfg_if::cfg_if;
use comm::{Read, Write};
use core::fmt::Debug;
use roi::{ROICenter, ROI};
use threshold::{Threshold, Window};
pub mod comm;
pub mod roi;
pub mod threshold;

/// Structure of software version.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct SWVersion {
    /// Major version number
    pub major: u8,
    /// Minor version number
    pub minor: u8,
    /// Build version
    pub build: u8,
    /// Revision
    pub revision: u32,
}

/// Driver error.
#[derive(Debug)]
pub enum Error<E>
where
    E: Debug,
{
    /// Error occured during communication holds the specific error from the communcation
    CommunicationError(E),
    /// The timing budget is invalid.
    InvalidTimingBudget,
    /// The distance mode is invalid.
    InvalidDistanceMode,
    /// The sigma threshold is invalid.
    InvalidSigmaThreshold,
}

impl<E> From<E> for Error<E>
where
    E: Debug,
{
    fn from(e: E) -> Self {
        Self::CommunicationError(e)
    }
}

/// Interrupt polarity.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Polarity {
    /// Interrupt pin is logic level high if the interrupt is active.
    ActiveHigh = 1,
    /// Interrupt pin is logic level low if the interrupt is active.
    ActiveLow = 0,
}

impl From<u8> for Polarity {
    fn from(v: u8) -> Self {
        match v {
            0 => Polarity::ActiveHigh,
            _ => Polarity::ActiveLow,
        }
    }
}

/// Distance measuring mode.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DistanceMode {
    /// Short distance mode.
    /// Maximum distance is limited to 1.3 m but has better ambient immunity.
    Short = 1,
    /// Long distance mode.
    /// Can range up to 4 m in the dark with a 200 ms timing budget.
    Long = 2,
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
    None = 255,
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
            _ => RangeStatus::None,
        }
    }
}

/// Voltage of SDA, SCL and GPIO.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum IOVoltage {
    /// Use an IO voltage of 1.8v
    Volt1_8,
    /// Use an IO voltage of 2.8v
    Volt2_8,
}

/// Result of a block measurement.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MeasureResult {
    /// The status of this result.
    pub status: RangeStatus,
    /// The distance measured for this result (may be 0).
    pub distance_mm: u16,
    /// The measured ambient signal.
    pub ambient: u16,
    /// The measured signal per SPAD.
    pub sig_per_spad: u16,
    /// The number of SPADs used for this measurement.
    pub spad_count: u16,
}

/// Default I2C address for VL53L1X.
pub const DEFAULT_ADDRESS: u8 = 0x29;

pub use vl53l1_reg::Index as Register;

/// Instance of a single VL53L1X driver.
pub struct VL53L1X<T>
where
    T: Write + Read,
    // <T as Write>::Error: Debug + PartialEq,
    // <T as WriteRead>::Error: Debug + PartialEq,
{
    i2c: T,
    address: u8,
}

impl<T, E> VL53L1X<T>
where
    E: Debug,
    T: Write<Error = E> + Read<Error = E>,
{
    /// Create a new instance of the VL53L1X driver with the given I2C address.
    ///
    /// # Arguments
    ///
    /// * `i2c` - Instance of object that implements I2C communication. Is used for further communication with the sensor
    /// * `address` - The address to use for communication with the VL53L1X.
    pub fn new(i2c: T, address: u8) -> VL53L1X<T> {
        VL53L1X { i2c, address }
    }

    /// Get the driver version.
    pub fn sw_version() -> SWVersion {
        SWVersion {
            major: 3,
            minor: 5,
            build: 1,
            revision: 0,
        }
    }

    /// Default configuration used during initialization.
    pub const DEFAULT_CONFIG: [u8; 88] = [
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
        0x00, /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
    ];

    /// Write bytes to register at address of variable length.
    /// (Number of bytes is limited to 4 for non iterator implementations)
    ///
    /// # Arguments
    ///
    /// * `address` - Address of the register to write to.
    /// * `bytes` - Bytes to write to the given register.
    pub fn write_bytes<R>(&mut self, address: R, bytes: &[u8]) -> Result<(), Error<E>>
    where
        R: Into<[u8; 2]>,
    {
        self.i2c
            .write_registers(self.address, address.into(), bytes)?;

        Ok(())
    }

    /// Read bytes from at the address into mutable slice
    ///
    /// # Arguments
    ///
    /// * `address` - Address of the register to read from.
    /// * `bytes` - Mutable slice that read data will be written to.
    pub fn read_bytes<R>(&mut self, address: R, bytes: &mut [u8]) -> Result<(), Error<E>>
    where
        R: Into<[u8; 2]>,
    {
        self.i2c
            .read_registers(self.address, address.into(), bytes)?;

        Ok(())
    }

    /// Set the I2C address of the current device in case multiple devices with the same address exists on the same bus.
    ///
    /// # Arguments
    ///
    /// * `new_address` - The new address to set for the current device.
    pub fn set_address(&mut self, new_address: u8) -> Result<(), Error<E>> {
        self.write_bytes(Register::I2C_SLAVE__DEVICE_ADDRESS, &[new_address])?;

        self.address = new_address;

        Ok(())
    }

    /// Load the 88 byte default values for sensor initialisation.
    ///
    /// # Arguments
    ///
    /// * `io_config` - The io voltage that will be configured for the device.
    pub fn init(&mut self, io_config: IOVoltage) -> Result<(), Error<E>> {
        self.write_bytes(Register::PAD_I2C_HV__CONFIG, &[0])?;

        let io = match io_config {
            IOVoltage::Volt1_8 => 0,
            IOVoltage::Volt2_8 => 1,
        };

        self.write_bytes(Register::GPIO_HV_PAD__CTRL, &[io])?;
        self.write_bytes(Register::PAD_I2C_HV__EXTSUP_CONFIG, &[io])?;

        cfg_if! {
            if #[cfg(feature = "i2c-iter")] {
                self.write_bytes(Register::GPIO_HV_MUX__CTRL, &Self::DEFAULT_CONFIG)?;
            } else {
                for (byte, address) in Self::DEFAULT_CONFIG.iter().zip(0x30u16..0x88) {
                    self.write_bytes(address.to_be_bytes(), &[*byte])?;
                }
            }
        }

        self.start_ranging()?;

        while !self.is_data_ready()? {}

        self.clear_interrupt()?;
        self.stop_ranging()?;

        self.write_bytes(Register::VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &[0x09])?;
        self.write_bytes(Register::VHV_CONFIG__INIT, &[0])?;

        Ok(())
    }

    /// Clear the interrupt flag on the device.
    /// Should be called after reading ranging data from the device to start the next measurement.
    pub fn clear_interrupt(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(Register::SYSTEM__INTERRUPT_CLEAR, &[0x01])
    }

    /// Set polarity of the interrupt.
    ///
    /// # Arguments
    ///
    /// * `polarity` - The polarity to set.
    pub fn set_interrupt_polarity(&mut self, polarity: Polarity) -> Result<(), Error<E>> {
        let mut gpio_mux_hv = [0u8];

        self.read_bytes(Register::GPIO_HV_MUX__CTRL, &mut gpio_mux_hv)?;

        gpio_mux_hv[0] &= 0xEF;

        gpio_mux_hv[0] |= (polarity as u8) << 4;

        self.write_bytes(Register::GPIO_HV_MUX__CTRL, &gpio_mux_hv)
    }

    /// Get the currently set interrupt polarity.
    pub fn get_interrupt_polarity(&mut self) -> Result<Polarity, Error<E>> {
        let mut gpio_mux_hv = [0u8];

        self.read_bytes(Register::GPIO_HV_MUX__CTRL, &mut gpio_mux_hv)?;

        Ok((gpio_mux_hv[0] & 0x10).into())
    }

    /// Start a distance ranging operation.
    /// This operation is continuous, the interrupt flag should be cleared between each interrupt to start a new distance measurement.
    pub fn start_ranging(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(Register::SYSTEM__MODE_START, &[0x40])
    }

    /// Stop an ongoing ranging operation.
    pub fn stop_ranging(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(Register::SYSTEM__MODE_START, &[0x00])
    }

    /// Check if new ranging data is available by polling the device.
    pub fn is_data_ready(&mut self) -> Result<bool, Error<E>> {
        let polarity = self.get_interrupt_polarity()? as u8;

        let mut state = [0u8];

        self.read_bytes(Register::GPIO__TIO_HV_STATUS, &mut state)?;

        if (state[0] & 0x01) == polarity {
            return Ok(true);
        }

        Ok(false)
    }

    /// Set the timing budget for a measurement operation in milliseconds.
    ///
    /// # Arguments
    ///
    /// * `milliseconds` - One of the following values = 15, 20, 33, 50, 100(default), 200, 500.
    pub fn set_timing_budget_ms(&mut self, milliseconds: u16) -> Result<(), Error<E>> {
        let mode = self.get_distance_mode()?;

        let (a, b) = match mode {
            DistanceMode::Short => match milliseconds {
                15 => (0x01Du16, 0x0027u16),
                20 => (0x051, 0x006E),
                33 => (0x00D6, 0x006E),
                50 => (0x1AE, 0x01E8),
                100 => (0x02E1, 0x0388),
                200 => (0x03E1, 0x0496),
                500 => (0x0591, 0x05C1),
                _ => return Err(Error::InvalidTimingBudget),
            },
            DistanceMode::Long => match milliseconds {
                20 => (0x001E, 0x0022),
                33 => (0x0060, 0x006E),
                50 => (0x00AD, 0x00C6),
                100 => (0x01CC, 0x01EA),
                200 => (0x02D9, 0x02F8),
                500 => (0x048F, 0x04A4),
                _ => return Err(Error::InvalidTimingBudget),
            },
        };

        self.write_bytes(
            Register::RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
            &a.to_be_bytes(),
        )?;
        self.write_bytes(
            Register::RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
            &b.to_be_bytes(),
        )?;

        Ok(())
    }

    /// Get the currently set timing budget of the device.
    pub fn get_timing_budget_ms(&mut self) -> Result<u16, Error<E>> {
        let mut a = [0u8, 0];

        self.read_bytes(Register::RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &mut a)?;

        Ok(match u16::from_be_bytes(a) {
            0x001D => 15,
            0x0051 | 0x001E => 20,
            0x00D6 | 0x0060 => 33,
            0x1AE | 0x00AD => 50,
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
    pub fn set_distance_mode(&mut self, mode: DistanceMode) -> Result<(), Error<E>> {
        let tb = self.get_timing_budget_ms()?;

        let (timeout, vcsel_a, vcsel_b, phase, woi_sd0, phase_sd0) = match mode {
            DistanceMode::Short => (0x14u8, 0x07u8, 0x05u8, 0x38u8, 0x0705u16, 0x0606u16),
            DistanceMode::Long => (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),
        };

        self.write_bytes(
            Register::PHASECAL_CONFIG__TIMEOUT_MACROP,
            &timeout.to_be_bytes(),
        )?;
        self.write_bytes(
            Register::RANGE_CONFIG__VCSEL_PERIOD_A,
            &vcsel_a.to_be_bytes(),
        )?;
        self.write_bytes(
            Register::RANGE_CONFIG__VCSEL_PERIOD_B,
            &vcsel_b.to_be_bytes(),
        )?;
        self.write_bytes(
            Register::RANGE_CONFIG__VALID_PHASE_HIGH,
            &phase.to_be_bytes(),
        )?;
        self.write_bytes(Register::SD_CONFIG__WOI_SD0, &woi_sd0.to_be_bytes())?;
        self.write_bytes(
            Register::SD_CONFIG__INITIAL_PHASE_SD0,
            &phase_sd0.to_be_bytes(),
        )?;

        self.set_timing_budget_ms(tb)?;

        Ok(())
    }

    /// Get the currently set distance mode of the device.
    pub fn get_distance_mode(&mut self) -> Result<DistanceMode, Error<E>> {
        let mut out = [0u8];
        self.read_bytes(Register::PHASECAL_CONFIG__TIMEOUT_MACROP, &mut out)?;

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
    /// * `milliseconds` - The number of milliseconds used for the inter measurement period.
    pub fn set_inter_measurement_period_ms(&mut self, milliseconds: u16) -> Result<(), Error<E>> {
        let mut clock_pll = [0u8, 0];

        self.read_bytes(Register::RESULT__OSC_CALIBRATE_VAL, &mut clock_pll)?;

        let clock_pll = u16::from_be_bytes(clock_pll) & 0x3FF;

        let val = ((clock_pll * milliseconds) as f32 * 1.075f32) as u32;

        self.write_bytes(
            Register::SYSTEM__INTERMEASUREMENT_PERIOD,
            &val.to_be_bytes(),
        )?;

        Ok(())
    }

    /// Get the currently set inter measurement period in milliseconds.
    pub fn get_inter_measurement_period_ms(&mut self) -> Result<u16, Error<E>> {
        let mut clock_pll = [0u8, 0];
        let mut period = [0u8, 0, 0, 0];

        self.read_bytes(Register::RESULT__OSC_CALIBRATE_VAL, &mut clock_pll)?;

        let clock_pll = u16::from_be_bytes(clock_pll) & 0x3FF;

        self.read_bytes(Register::SYSTEM__INTERMEASUREMENT_PERIOD, &mut period)?;

        let period = u32::from_be_bytes(period);

        Ok((period / (clock_pll as f32 * 1.065f32) as u32) as u16)
    }

    /// Check if the device is booted.
    pub fn is_booted(&mut self) -> Result<bool, Error<E>> {
        let mut status = [0u8];
        self.read_bytes(Register::FIRMWARE__SYSTEM_STATUS, &mut status)?;

        Ok(status[0] == 1)
    }

    /// Get the sensor id of the sensor. This id must be 0xEACC.
    pub fn get_sensor_id(&mut self) -> Result<u16, Error<E>> {
        let mut id = [0u8, 0];

        self.read_bytes(Register::IDENTIFICATION__MODEL_ID, &mut id)?;

        Ok(u16::from_be_bytes(id))
    }

    /// Get the distance measured in millimeters.
    pub fn get_distance(&mut self) -> Result<u16, Error<E>> {
        let mut distance = [0u8, 0];

        self.read_bytes(
            Register::RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
            &mut distance,
        )?;

        Ok(u16::from_be_bytes(distance))
    }

    /// Get the returned signal per SPAD in kcps/SPAD where kcps stands for Kilo Count Per Second.
    pub fn get_signal_per_spad(&mut self) -> Result<u16, Error<E>> {
        let mut signal = [0, 0];
        let mut spad_count = [0, 0];

        self.read_bytes(
            Register::RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0,
            &mut signal,
        )?;
        self.read_bytes(
            Register::RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0,
            &mut spad_count,
        )?;

        let signal = u16::from_be_bytes(signal);
        let spad_count = u16::from_be_bytes(spad_count);

        Ok((200.0f32 * signal as f32 / spad_count as f32) as u16)
    }

    /// Get the ambient signal per SPAD in kcps/SPAD where kcps stands for Kilo Count Per Second.
    pub fn get_ambient_per_spad(&mut self) -> Result<u16, Error<E>> {
        let mut spad_count = [0, 0];
        let mut ambient = [0, 0];

        self.read_bytes(
            Register::RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0,
            &mut spad_count,
        )?;
        self.read_bytes(Register::RESULT__AMBIENT_COUNT_RATE_MCPS_SD0, &mut ambient)?;

        let spad_count = u16::from_be_bytes(spad_count);
        let ambient = u16::from_be_bytes(ambient);

        Ok((200.0f32 * ambient as f32 / spad_count as f32) as u16)
    }

    /// Get the returned signal in kcps (Kilo Count Per Second).
    pub fn get_signal_rate(&mut self) -> Result<u16, Error<E>> {
        let mut signal = [0, 0];

        self.read_bytes(
            Register::RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0,
            &mut signal,
        )?;

        Ok(u16::from_be_bytes(signal) * 8)
    }

    /// Get the count of currently enabled SPADs.
    pub fn get_spad_count(&mut self) -> Result<u16, Error<E>> {
        let mut spad_count = [0, 0];

        self.read_bytes(
            Register::RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0,
            &mut spad_count,
        )?;

        Ok(u16::from_be_bytes(spad_count) >> 8)
    }

    /// Get the ambient signal in kcps (Kilo Count Per Second).
    pub fn get_ambient_rate(&mut self) -> Result<u16, Error<E>> {
        let mut ambient = [0, 0];

        self.read_bytes(Register::RESULT__AMBIENT_COUNT_RATE_MCPS_SD0, &mut ambient)?;

        Ok(u16::from_be_bytes(ambient) * 8)
    }

    /// Get the ranging status.
    pub fn get_range_status(&mut self) -> Result<RangeStatus, Error<E>> {
        let mut status = [0];

        self.read_bytes(Register::RESULT__RANGE_STATUS, &mut status)?;

        let status = u8::from_be_bytes(status) & 0x1F;

        Ok(status.into())
    }

    /// Get a measurement result object which is read in a single access.
    pub fn get_result(&mut self) -> Result<MeasureResult, Error<E>> {
        let mut result = [0; 17];

        self.read_bytes(Register::RESULT__RANGE_STATUS, &mut result)?;

        Ok(MeasureResult {
            status: (result[0] & 0x1F).into(),
            ambient: u16::from_be_bytes([result[7], result[8]]) * 8,
            spad_count: result[3] as u16,
            sig_per_spad: u16::from_be_bytes([result[15], result[16]]) * 8,
            distance_mm: u16::from_be_bytes([result[13], result[14]]),
        })
    }

    /// Set a offset in millimeters which is aplied to the distance.
    ///
    /// # Arguments
    ///
    /// * `offset` - The offset in millimeters.
    pub fn set_offset(&mut self, offset: i16) -> Result<(), Error<E>> {
        self.write_bytes(
            Register::ALGO__PART_TO_PART_RANGE_OFFSET_MM,
            &(offset * 4).to_be_bytes(),
        )?;

        self.write_bytes(Register::MM_CONFIG__INNER_OFFSET_MM, &[0, 0])?;
        self.write_bytes(Register::MM_CONFIG__OUTER_OFFSET_MM, &[0, 0])?;

        Ok(())
    }

    /// Get the current offset in millimeters.
    pub fn get_offset(&mut self) -> Result<i16, Error<E>> {
        let mut offset = [0, 0];

        self.read_bytes(Register::ALGO__PART_TO_PART_RANGE_OFFSET_MM, &mut offset)?;

        let mut offset = u16::from_be_bytes(offset) << 3;

        offset >>= 5;

        Ok(offset as i16)
    }

    /// Set the crosstalk correction value in cps (Count Per Second).
    ///
    /// # Arguments
    ///
    /// * `correction` - The number of photons reflected back from the cover glass in cps.
    pub fn set_cross_talk(&mut self, correction: u16) -> Result<(), Error<E>> {
        self.write_bytes(
            Register::ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
            &[0, 0],
        )?;
        self.write_bytes(
            Register::ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
            &[0, 0],
        )?;

        let correction = (correction << 9) / 1000;

        self.write_bytes(
            Register::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            &correction.to_be_bytes(),
        )?;

        Ok(())
    }

    /// Get the crosstalk correction value in cps.
    pub fn get_cross_talk(&mut self) -> Result<u16, Error<E>> {
        let mut correction = [0, 0];

        self.read_bytes(
            Register::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            &mut correction,
        )?;

        let correction = (u16::from_be_bytes(correction) * 1000) >> 9;

        Ok(correction)
    }

    /// Set a distance threshold.
    ///
    /// # Arguments
    ///
    /// * `threshold` - The threshold to apply.
    pub fn set_distance_threshold(&mut self, threshold: Threshold) -> Result<(), Error<E>> {
        let mut config = [0];

        self.read_bytes(Register::SYSTEM__INTERRUPT_CONFIG_GPIO, &mut config)?;

        let config = config[0] & 0x47 | (threshold.window as u8 & 0x07);

        self.write_bytes(Register::SYSTEM__INTERRUPT_CONFIG_GPIO, &[config])?;
        self.write_bytes(Register::SYSTEM__THRESH_HIGH, &threshold.high.to_be_bytes())?;
        self.write_bytes(Register::SYSTEM__THRESH_LOW, &threshold.low.to_be_bytes())?;

        Ok(())
    }

    /// Get the currently set distance threshold.
    pub fn get_distance_threshold(&mut self) -> Result<Threshold, Error<E>> {
        let mut config = [0];

        self.read_bytes(Register::SYSTEM__INTERRUPT_CONFIG_GPIO, &mut config)?;

        let window: Window = (config[0] & 0x07).into();

        let mut high = [0, 0];
        let mut low = [0, 0];

        self.read_bytes(Register::SYSTEM__THRESH_HIGH, &mut high)?;
        self.read_bytes(Register::SYSTEM__THRESH_LOW, &mut low)?;

        Ok(Threshold {
            window,
            low: u16::from_be_bytes(low),
            high: u16::from_be_bytes(high),
        })
    }

    /// Set the region of interest of the sensor. The ROI is centered and only the size is settable.
    /// The smallest acceptable ROI size is 4.
    ///
    /// # Arguments
    ///
    /// * `roi` - The ROI to apply.
    pub fn set_roi(&mut self, mut roi: ROI) -> Result<(), Error<E>> {
        debug_assert!(roi.width >= 4);
        debug_assert!(roi.height >= 4);

        let mut center = [0];

        self.read_bytes(Register::ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &mut center)?;

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

        self.write_bytes(Register::ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &center)?;
        self.write_bytes(
            Register::ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
            &[config],
        )?;

        Ok(())
    }

    /// Get the currenly set ROI.
    ///
    /// # Arguments
    ///
    pub fn get_roi(&mut self) -> Result<ROI, Error<E>> {
        let mut config = [0];

        self.read_bytes(
            Register::ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
            &mut config,
        )?;

        Ok(ROI {
            width: ((config[0] & 0x0F) + 1) as u16,
            height: (((config[0] >> 4) & 0x0F) + 1) as u16,
        })
    }

    /// Set the new ROI center.
    /// If the new ROI clips out of the border this function does not return an error
    /// but only when ranging is started will an error be returned.
    ///
    /// # Arguments
    ///
    /// * `center` - Tne ROI center to apply.
    pub fn set_roi_center(&mut self, center: ROICenter) -> Result<(), Error<E>> {
        self.write_bytes(Register::ROI_CONFIG__USER_ROI_CENTRE_SPAD, &[center.spad])
    }

    /// Get the current ROI center.
    pub fn get_roi_center(&mut self) -> Result<ROICenter, Error<E>> {
        let mut center = [0];

        self.read_bytes(Register::ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &mut center)?;

        Ok(ROICenter { spad: center[0] })
    }

    /// Set a signal threshold in kcps. Default is 1024
    ///
    /// # Arguments
    ///
    /// * `threshold` - The signal threshold.
    pub fn set_signal_threshold(&mut self, threshold: u16) -> Result<(), Error<E>> {
        self.write_bytes(
            Register::RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
            &(threshold >> 3).to_be_bytes(),
        )
    }

    /// Get the currently set signal threshold.
    pub fn get_signal_threshold(&mut self) -> Result<u16, Error<E>> {
        let mut threshold = [0, 0];

        self.read_bytes(
            Register::RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
            &mut threshold,
        )?;

        Ok(u16::from_be_bytes(threshold) << 3)
    }

    /// Set a sigma threshold in millimeter. Default is 15.
    ///
    /// # Arguments
    ///
    /// * `threshold` - The sigma threshold.
    pub fn set_sigma_threshold(&mut self, threshold: u16) -> Result<(), Error<E>> {
        if threshold > (0xFFFF >> 2) {
            return Err(Error::InvalidSigmaThreshold);
        }

        self.write_bytes(
            Register::RANGE_CONFIG__SIGMA_THRESH,
            &(threshold << 2).to_be_bytes(),
        )
    }

    /// Get the currently set sigma threshold.
    pub fn get_sigma_threshold(&mut self) -> Result<u16, Error<E>> {
        let mut threshold = [0, 0];

        self.read_bytes(Register::RANGE_CONFIG__SIGMA_THRESH, &mut threshold)?;

        Ok(u16::from_be_bytes(threshold) >> 2)
    }

    /// Perform temperature calibration of the sensor.
    /// It is recommended to call this function any time the temperature might have changed by more than 8 degrees Celsius
    /// without sensor ranging activity for an extended period.
    pub fn calibrate_temperature(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(
            Register::VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
            &0x81u16.to_be_bytes(),
        )?;
        self.write_bytes(Register::VHV_CONFIG__INIT, &0x92u16.to_be_bytes())?;

        self.start_ranging()?;

        while !self.is_data_ready()? {}

        self.clear_interrupt()?;
        self.stop_ranging()?;

        self.write_bytes(
            Register::VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
            &0x09u16.to_be_bytes(),
        )?;
        self.write_bytes(Register::VHV_CONFIG__INIT, &0u16.to_be_bytes())?;

        Ok(())
    }

    /// Perform offset calibration.
    /// The function returns the offset value found and sets it as the new offset.
    /// Target reflectance = grey 17%
    ///
    /// # Arguments
    ///
    /// * `target_distance_mm` - Distance to the target in millimeters, ST recommends 100 mm.
    pub fn calibrate_offset(&mut self, target_distance_mm: u16) -> Result<i16, Error<E>> {
        self.write_bytes(
            Register::ALGO__PART_TO_PART_RANGE_OFFSET_MM,
            &0u16.to_be_bytes(),
        )?;
        self.write_bytes(Register::MM_CONFIG__INNER_OFFSET_MM, &0u16.to_be_bytes())?;
        self.write_bytes(Register::MM_CONFIG__OUTER_OFFSET_MM, &0u16.to_be_bytes())?;

        self.start_ranging()?;

        let mut average_distance = 0;

        for _ in 0..50 {
            while self.is_data_ready()? {}

            average_distance += self.get_distance()?;
            self.clear_interrupt()?;
        }

        self.stop_ranging()?;

        average_distance /= 50;

        let offset = target_distance_mm as i16 - average_distance as i16;

        self.set_offset(offset)?;

        Ok(offset)
    }

    /// Perform crosstalk calibration.
    /// The function returns the crosstalk value found and set it as the new crosstalk correction.
    /// Target reflectance = grey 17%
    ///
    ///  Arguments
    ///
    /// * `target_distance_mm` - Distance to the target in millimeters, ST recommends 100 mm.
    pub fn calibrate_cross_talk(&mut self, target_distance_mm: u16) -> Result<u16, Error<E>> {
        self.write_bytes(
            Register::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            &0u16.to_be_bytes(),
        )?;

        self.start_ranging()?;

        let mut average_distance = 0;
        let mut average_spad_cnt = 0;
        let mut average_signal_rate = 0;

        for _ in 0..50 {
            while self.is_data_ready()? {}

            average_distance += self.get_distance()?;
            average_signal_rate += self.get_signal_rate()?;
            self.clear_interrupt()?;
            average_spad_cnt += self.get_spad_count()?;
        }

        self.stop_ranging()?;

        average_distance /= 50;
        average_spad_cnt /= 50;
        average_signal_rate /= 50;

        let mut calibrate_val = 512
            * (average_signal_rate as u32
                * (1 - (average_distance as u32 / target_distance_mm as u32)))
            / average_spad_cnt as u32;

        if calibrate_val > 0xFFFF {
            calibrate_val = 0xFFFF;
        }

        let config = ((calibrate_val as u16 * 1000) >> 9).to_be_bytes();

        self.write_bytes(
            Register::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            &config,
        )?;

        Ok(0)
    }
}

#[cfg(test)]
mod tests {

    extern crate std;

    use crate::{Register, SWVersion, DEFAULT_ADDRESS, VL53L1X};

    use embedded_hal_mock::{
        i2c::{Mock as I2cMock, Transaction as I2cTransaction},
        MockError,
    };
    use std::{io::ErrorKind, vec};

    type VL53L1XMock = VL53L1X<I2cMock>;

    #[test]
    fn sw_version() {
        assert!(
            VL53L1XMock::sw_version()
                == SWVersion {
                    major: 3,
                    minor: 5,
                    build: 1,
                    revision: 0
                }
        );
    }

    #[test]
    fn set_address() {
        let new_address = 0x55;

        let i2c_adr_cmd = (Register::I2C_SLAVE__DEVICE_ADDRESS as u16).to_be_bytes();
        let clr_irq_cmd = (Register::SYSTEM__INTERRUPT_CLEAR as u16).to_be_bytes();

        let expectations = [
            I2cTransaction::write(0x29, vec![i2c_adr_cmd[0], i2c_adr_cmd[1], new_address]),
            I2cTransaction::write(0x55, vec![clr_irq_cmd[0], clr_irq_cmd[1], 0x01]),
        ];

        let i2c = I2cMock::new(&expectations);

        let mut sensor = VL53L1XMock::new(i2c, DEFAULT_ADDRESS);

        assert!(sensor.set_address(new_address).is_ok());
        assert!(sensor.clear_interrupt().is_ok());
    }

    #[test]
    fn set_address_driver_failure() {
        let new_address = 0x55;

        let i2c_adr_cmd = (Register::I2C_SLAVE__DEVICE_ADDRESS as u16).to_be_bytes();
        let clr_irq_cmd = (Register::SYSTEM__INTERRUPT_CLEAR as u16).to_be_bytes();

        let expectations = [
            I2cTransaction::write(0x29, vec![i2c_adr_cmd[0], i2c_adr_cmd[1], new_address])
                .with_error(MockError::Io(ErrorKind::NotFound)),
            I2cTransaction::write(0x29, vec![clr_irq_cmd[0], clr_irq_cmd[1], 0x01]),
        ];

        let i2c = I2cMock::new(&expectations);

        let mut sensor = VL53L1XMock::new(i2c, DEFAULT_ADDRESS);

        assert!(sensor.set_address(new_address).is_err());
        assert!(sensor.clear_interrupt().is_ok());
    }

    #[test]
    fn write_error_propegate() {
        let clr_irq_cmd = (Register::SYSTEM__INTERRUPT_CLEAR as u16).to_be_bytes();

        let expectations =
            [
                I2cTransaction::write(0x29, vec![clr_irq_cmd[0], clr_irq_cmd[1], 0x01])
                    .with_error(MockError::Io(ErrorKind::NotFound)),
            ];

        let i2c = I2cMock::new(&expectations);

        let mut sensor = VL53L1XMock::new(i2c, DEFAULT_ADDRESS);

        let res = sensor.clear_interrupt();

        assert!(res.is_err());

        match res.unwrap_err() {
            crate::Error::CommunicationError(e) => {
                assert!(e == MockError::Io(ErrorKind::NotFound))
            }
            _ => panic!("Invalid error returned"),
        }
    }

    #[test]
    fn write_read_error_propegate() {
        let irq_dir_cmd = (Register::GPIO_HV_MUX__CTRL as u16).to_be_bytes();

        let expectations =
            [
                I2cTransaction::write_read(0x29, vec![irq_dir_cmd[0], irq_dir_cmd[1]], vec![0])
                    .with_error(MockError::Io(ErrorKind::NotFound)),
            ];

        let i2c = I2cMock::new(&expectations);

        let mut sensor = VL53L1XMock::new(i2c, DEFAULT_ADDRESS);

        let res = sensor.is_data_ready();

        assert!(res.is_err());

        match res.unwrap_err() {
            crate::Error::CommunicationError(e) => {
                assert!(e == MockError::Io(ErrorKind::NotFound))
            }
            _ => panic!("Invalid error returned"),
        }
    }
}
