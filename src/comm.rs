//! Traits and implementations needed for I2C communication.
use cfg_if::cfg_if;
use core::fmt::Debug;

/// Trait to write registers for the VL53L1X.
pub trait Write {
    /// Error return type.
    type Error: Debug;
    /// Write registers with the given bytes.
    /// (Number of bytes is limited to 4 for non iterator implementations)
    ///
    /// # Arguments
    ///
    /// * `address` - The i2c address.
    /// * `register` - The register to write to (Auto increments if more than 1 byte is written).
    /// * `bytes` - The bytes to be written at the given address.
    fn write_registers(
        &mut self,
        address: u8,
        register: [u8; 2],
        bytes: &[u8],
    ) -> Result<(), Self::Error>;
}

/// Trait to read a registers for the VL53L1X.
pub trait Read {
    /// Error return type.
    type Error: Debug;
    /// Read registers into the given bytes.
    ///
    /// * `address` - The i2c address.
    /// * `register` - The register to read from (Auto increments if more than 1 byte is read).
    /// * `bytes` - The slice into which the read data will be put.
    fn read_registers(
        &mut self,
        address: u8,
        register: [u8; 2],
        bytes: &mut [u8],
    ) -> Result<(), Self::Error>;
}

cfg_if! {
    if #[cfg(feature = "i2c-iter")] {
        use embedded_hal::blocking::i2c::{WriteIter, WriteIterRead};
        impl<I2C> Write for I2C
        where I2C : WriteIter,
            <I2C as WriteIter>::Error : Debug
        {
            type Error = <I2C as WriteIter>::Error;
            fn write_registers(&mut self, address: u8, register: [u8; 2], bytes: &[u8]) -> Result<(), Self::Error> {
                let iter = register.iter().chain(bytes.iter()).cloned();
                self.write(address, iter)
            }
        }

        impl<I2C> Read for I2C
        where I2C : WriteIterRead,
            <I2C as WriteIterRead>::Error : Debug
        {
            type Error = <I2C as WriteIterRead>::Error;
            fn read_registers(&mut self, address: u8, register: [u8; 2], bytes: &mut [u8]) -> Result<(), Self::Error> {
                self.write_iter_read(address, register.into_iter(), bytes)
            }
        }
    } else {
        use embedded_hal::blocking::i2c::{Write as I2CWrite, WriteRead};
        impl<I2C> Write for I2C
        where I2C : I2CWrite,
            <I2C as I2CWrite>::Error : Debug
        {
            type Error = <I2C as I2CWrite>::Error;
            fn write_registers(&mut self, address: u8, register: [u8; 2], bytes: &[u8]) -> Result<(), Self::Error> {
                assert!(bytes.len() <= 4);

                let mut tmp = [0u8; 6];

                for (src, dst) in register.iter().chain(bytes).zip(tmp.iter_mut()) {
                    *dst = *src;
                }

                self.write(address, &tmp[..bytes.len()+2])
            }
        }

        impl<I2C> Read for I2C
        where I2C : WriteRead,
            <I2C as WriteRead>::Error : Debug
        {
            type Error = <I2C as WriteRead>::Error;
            fn read_registers(&mut self, address: u8, register: [u8; 2], bytes: &mut [u8]) -> Result<(), Self::Error> {
                self.write_read(address, &register, bytes)
            }
        }
    }
}
