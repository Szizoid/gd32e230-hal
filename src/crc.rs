//! Hardware CRC calculation unit.

use core::marker::PhantomData;

use crate::pac;
use crate::rcu::{Enable, Rcu};

/// 32-bit polynomial size, selects [`Crc::new_32bit`].
pub struct B32;
/// 16-bit polynomial size, selects [`Crc::new_16bit`].
pub struct B16;
/// 8-bit polynomial size, selects [`Crc::new_8bit`].
pub struct B8;
/// 7-bit polynomial size, selects [`Crc::new_7bit`].
pub struct B7;

/// Input data bit-reversal granularity (`REV_I`).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReverseInput {
    /// No reversal.
    Disabled = 0b00,
    /// Reverse the bit order within each byte.
    Byte = 0b01,
    /// Reverse the bit order within each half-word.
    HalfWord = 0b10,
    /// Reverse the bit order within each word.
    Word = 0b11,
}

/// Output data bit reversal (`REV_O`).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReverseOutput {
    /// The result is read out as computed.
    Disabled = 0,
    /// The result is bit-reversed on read.
    Enabled = 1,
}

/// Settings shared by every polynomial size.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CrcConfig {
    reverse_input: ReverseInput,
    reverse_output: ReverseOutput,
}

impl CrcConfig {
    /// Creates a configuration with the given input/output reversal.
    pub const fn new(reverse_input: ReverseInput, reverse_output: ReverseOutput) -> Self {
        Self {
            reverse_input,
            reverse_output,
        }
    }
}

impl Default for CrcConfig {
    fn default() -> Self {
        Self {
            reverse_input: ReverseInput::Disabled,
            reverse_output: ReverseOutput::Disabled,
        }
    }
}

/// Writes the fields shared by every polynomial size.
fn configure(crc: &pac::Crc, ps: u8, poly: u32, config: CrcConfig) {
    let rev_o = matches!(config.reverse_output, ReverseOutput::Enabled);
    crc.ctl().modify(|_, w| {
        unsafe { w.ps().bits(ps) };
        w.rev_o()
            .bit(rev_o)
            .rev_i()
            .bits(config.reverse_input as u8)
    });
    crc.poly().write(|w| unsafe { w.bits(poly) });
}

/// Loads `idata` into the result register through a pulse of `RST`.
fn new_seed(crc: &pac::Crc, idata: u32) {
    crc.idata().write(|w| w.idata().bits(idata));
    crc.ctl().modify(|_, w| w.rst().reset());
}

/// Hardware CRC calculation unit.
///
/// `PS` ([`B32`]/[`B16`]/[`B8`]/[`B7`]) fixes the polynomial width for the
/// lifetime of the value, selected by which constructor built it.
pub struct Crc<PS> {
    crc: pac::Crc,
    _poly_size: PhantomData<PS>,
}

impl Crc<B32> {
    /// Enables the peripheral's clock and configures a 32-bit polynomial.
    pub fn new_32bit(rcu: &mut Rcu, crc: pac::Crc, poly: u32, config: CrcConfig) -> Self {
        <pac::Crc as Enable>::enable(rcu);
        configure(&crc, 0b00, poly, config);
        Self {
            crc,
            _poly_size: PhantomData,
        }
    }

    /// Feeds one 32-bit word into the running CRC, combining it with the
    /// current result rather than replacing it.
    pub fn write_32bit(&mut self, data: u32) {
        let data_reg = self.crc.data().as_ptr();
        unsafe { data_reg.write_volatile(data) };
    }
    /// Reads the current accumulated CRC result.
    pub fn read_32bit(&self) -> u32 {
        self.read()
    }

    /// Sets `IDATA` to `seed` and pulses `RST`, so the result reads back as
    /// `seed` until the next [`write_32bit`](Self::write_32bit).
    pub fn reset_with(&mut self, seed: u32) {
        new_seed(&self.crc, seed);
    }
}

impl Crc<B16> {
    /// Enables the peripheral's clock and configures a 16-bit polynomial.
    pub fn new_16bit(rcu: &mut Rcu, crc: pac::Crc, poly: u16, config: CrcConfig) -> Self {
        <pac::Crc as Enable>::enable(rcu);
        configure(&crc, 0b01, poly as u32, config);
        Self {
            crc,
            _poly_size: PhantomData,
        }
    }

    /// Feeds one 16-bit word into the running CRC, combining it with the
    /// current result rather than replacing it.
    pub fn write_16bit(&mut self, data: u16) {
        let data_reg = self.crc.data().as_ptr() as *mut u16;
        unsafe { data_reg.write_volatile(data) };
    }
    /// Reads the current accumulated CRC result.
    pub fn read_16bit(&self) -> u16 {
        self.read() as u16
    }

    /// Sets `IDATA` to `seed` and pulses `RST`, so the result reads back as
    /// `seed` until the next [`write_16bit`](Self::write_16bit).
    pub fn reset_with(&mut self, seed: u16) {
        new_seed(&self.crc, seed as u32);
    }
}

impl Crc<B8> {
    /// Enables the peripheral's clock and configures an 8-bit polynomial.
    pub fn new_8bit(rcu: &mut Rcu, crc: pac::Crc, poly: u8, config: CrcConfig) -> Self {
        <pac::Crc as Enable>::enable(rcu);
        configure(&crc, 0b10, poly as u32, config);
        Self {
            crc,
            _poly_size: PhantomData,
        }
    }

    /// Feeds one byte into the running CRC, combining it with the current
    /// result rather than replacing it.
    pub fn write_8bit(&mut self, data: u8) {
        let data_reg = self.crc.data().as_ptr() as *mut u8;
        unsafe { data_reg.write_volatile(data) };
    }
    /// Reads the current accumulated CRC result.
    pub fn read_8bit(&self) -> u8 {
        self.read() as u8
    }

    /// Sets `IDATA` to `seed` and pulses `RST`, so the result reads back as
    /// `seed` until the next [`write_8bit`](Self::write_8bit).
    pub fn reset_with(&mut self, seed: u8) {
        new_seed(&self.crc, seed as u32);
    }
}

impl Crc<B7> {
    /// Enables the peripheral's clock and configures a 7-bit polynomial.
    pub fn new_7bit(rcu: &mut Rcu, crc: pac::Crc, poly: u8, config: CrcConfig) -> Self {
        <pac::Crc as Enable>::enable(rcu);
        configure(&crc, 0b11, poly as u32, config);
        Self {
            crc,
            _poly_size: PhantomData,
        }
    }

    /// Feeds the low 7 bits of `data` into the running CRC, combining them
    /// with the current result rather than replacing it. The top bit is
    /// ignored by hardware.
    pub fn write_7bit(&mut self, data: u8) {
        let data_reg = self.crc.data().as_ptr() as *mut u8;
        unsafe { data_reg.write_volatile(data) };
    }
    /// Reads the current accumulated CRC result, in the low 7 bits.
    pub fn read_7bit(&self) -> u8 {
        self.read() as u8
    }

    /// Sets `IDATA` to `seed` and pulses `RST`, so the result reads back as
    /// `seed` until the next [`write_7bit`](Self::write_7bit).
    pub fn reset_with(&mut self, seed: u8) {
        new_seed(&self.crc, seed as u32);
    }
}

impl<PS> Crc<PS> {
    /// Reads the current accumulated CRC result.
    pub fn read(&self) -> u32 {
        self.crc.data().read().data().bits()
    }

    /// The clock is left enabled — a later `new_*bit()` re-enables it anyway.
    pub fn release(self) -> pac::Crc {
        self.crc
    }

    /// Writes an arbitrary byte to the scratch `FDATA` register. Unrelated to
    /// the CRC calculation — hardware never reads or modifies it on its own.
    pub fn set_fdata(&mut self, fdata: u8) {
        self.crc.fdata().write(|w| w.fdata().bits(fdata));
    }
    /// Reads back the byte last written with [`set_fdata`](Self::set_fdata).
    pub fn fdata(&self) -> u8 {
        self.crc.fdata().read().fdata().bits()
    }
}
