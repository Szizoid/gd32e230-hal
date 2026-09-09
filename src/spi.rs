//! SPI master.
//!
//! Covers both SPI0 and SPI1 in master, full-duplex, blocking mode with software
//! NSS (chip select is an ordinary GPIO the caller toggles). Frames are 8 or 16
//! bits wide, selected by a typestate parameter.
//!
//! Every operation is a simultaneous *exchange* — a word leaves on MOSI while
//! another arrives on MISO — so [`SpiBus::read`] sends zeros and
//! [`SpiBus::write`] discards what comes back.
//!
//! ```ignore
//! let sck = parts.pa5.into_alternate::<0>();
//! let miso = parts.pa6.into_alternate::<0>();
//! let mosi = parts.pa7.into_alternate::<0>();
//! let mut spi = Spi::new(&mut rcu, dp.spi0, sck, miso, mosi, SpiConfig::new(SpiPsc::Div8));
//! ```

use core::marker::PhantomData;

use embedded_hal::spi::{ErrorKind, ErrorType, MODE_0, Mode, Phase, Polarity, SpiBus};

use crate::gpio::{Alternate, Pin};
use crate::pac;
use crate::rcu::{Enable, Rcu, Reset};

/// SPI1 `DZ` encodes the frame length as (bits - 1); values below 0b0011 are
/// forced to 8-bit by hardware.
const DZ_8BIT: u8 = 0b0111;
const DZ_16BIT: u8 = 0b1111;

/// Named idle levels for the side of an exchange that carries no data.
///
/// Every transfer is simultaneous, so reading `n` words means sending `n` of
/// them. Which value drives the wire is the target's business, not the bus's:
/// most accept anything, SD cards want MOSI high. Purely a readability aid —
/// any value is legal.
#[allow(missing_docs)]
pub mod fill {
    pub const LOW: u8 = 0x00;
    pub const HIGH: u8 = 0xFF;
    pub const LOW_WORD: u16 = 0x0000;
    pub const HIGH_WORD: u16 = 0xFFFF;
}

/// SCK prescaler: divides `pclk` down to the serial clock.
///
/// Discriminants are the `PSC` register encoding. There is no universal default
/// — the right divider depends on `pclk` and the slave's maximum clock.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum SpiPsc {
    Div2 = 0b000,
    Div4 = 0b001,
    Div8 = 0b010,
    Div16 = 0b011,
    Div32 = 0b100,
    Div64 = 0b101,
    Div128 = 0b110,
    Div256 = 0b111,
}

/// Order in which the bits of a word are shifted onto the wire.
///
/// Both ends of the link must agree, or every word arrives bit-reversed with no
/// error reported. Most devices are MSB-first, which is the default.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    /// Most significant bit first.
    MsbFirst,
    /// Least significant bit first.
    LsbFirst,
}

/// Bus configuration passed to [`Spi::new`] / [`Spi::new_word`].
///
/// Built with [`SpiConfig::new`], which requires a prescaler; [`mode`](Self::mode)
/// and [`bit_order`](Self::bit_order) refine the defaults fluently.
///
/// ```ignore
/// SpiConfig::new(SpiPsc::Div16)
///     .mode(embedded_hal::spi::MODE_3)
///     .bit_order(BitOrder::LsbFirst)
/// ```
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiConfig {
    psc: SpiPsc,
    mode: Mode,
    bit_order: BitOrder,
}

impl SpiConfig {
    /// Creates a configuration with the given prescaler, Mode 0 and MSB-first.
    ///
    /// The prescaler is a required argument and this type has no `Default`: an
    /// SCK divider has no conventional value, so it must be chosen deliberately.
    pub const fn new(psc: SpiPsc) -> Self {
        Self {
            psc,
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
        }
    }
    /// Sets the clock polarity and phase (CPOL/CPHA), per the slave's datasheet.
    pub const fn mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }
    /// Sets the bit order. Defaults to [`BitOrder::MsbFirst`].
    pub const fn bit_order(mut self, bit_order: BitOrder) -> Self {
        self.bit_order = bit_order;
        self
    }
}

/// Marks a pin usable as `SCK` for `SPI`, in the right alternate function.
pub trait SckPin<SPI> {}
/// Marks a pin usable as `MISO` for `SPI`, in the right alternate function.
pub trait MisoPin<SPI> {}
/// Marks a pin usable as `MOSI` for `SPI`, in the right alternate function.
pub trait MosiPin<SPI> {}

macro_rules! spi_pins {
    ( $( $SPI:ty:
        SCK:  [ $($(#[$sck_cfg:meta])? $sck_p:literal  $sck_n:literal  : $sck_af:literal),* $(,)? ]
        MISO: [ $($(#[$miso_cfg:meta])? $miso_p:literal $miso_n:literal : $miso_af:literal),* $(,)? ]
        MOSI: [ $($(#[$mosi_cfg:meta])? $mosi_p:literal $mosi_n:literal : $mosi_af:literal),* $(,)? ]
    ),* $(,)? ) => {
        $(
            $( $(#[$sck_cfg])?  impl SckPin<$SPI>  for Pin<$sck_p,  $sck_n,  Alternate<$sck_af>>  {} )*
            $( $(#[$miso_cfg])? impl MisoPin<$SPI> for Pin<$miso_p, $miso_n, Alternate<$miso_af>> {} )*
            $( $(#[$mosi_cfg])? impl MosiPin<$SPI> for Pin<$mosi_p, $mosi_n, Alternate<$mosi_af>> {} )*
        )*
    };
}

// PB13/PB14/PB15 at AF0 belong to a *different* SPI depending on the chip
// variant (datasheet Table 2-14 footnotes): SPI0 on GD32E230x4, SPI1 on
// GD32E230x8. They are therefore listed in the gated blocks, not here.
//
// The `pads_ge_*` gates say the package bonds the pin at all, and match the ones in
// `gpio::Parts` — an entry for an unbonded pad would advertise in the docs a pin
// nobody can obtain.
spi_pins!(
    pac::Spi0:
        SCK: ['A' 5 : 0, #[cfg(pads_ge_28)] 'B' 3 : 0]
        MISO: ['A' 6 : 0, #[cfg(pads_ge_28)] 'B' 4 : 0]
        MOSI: ['A' 7 : 0, #[cfg(pads_ge_28)] 'B' 5 : 0]
);

// ---- (1) GD32E230x4 only: PB13/14/15 AF0 are SPI0 ----
#[cfg(chip_x4)]
spi_pins!(
    pac::Spi0:
        SCK: [#[cfg(pads_ge_48)] 'B' 13 : 0]
        MISO: [#[cfg(pads_ge_48)] 'B' 14 : 0]
        MOSI: [#[cfg(pads_ge_48)] 'B' 15 : 0]
);

// ---- (3) GD32E230x8 only: SPI1 exists, and PB13/14/15 AF0 belong to it ----
// NB: below 48 pins this leaves PB1 plus PA13/PA14 — the SWD pair, reachable only
// through the `activate_into_*` family, at the cost of the debug port.
#[cfg(chip_x8)]
spi_pins!(
    pac::Spi1:
        SCK: ['B' 1 : 6, #[cfg(pads_ge_48)] 'B' 10 : 7, #[cfg(pads_ge_48)] 'B' 13 : 0]
        MISO: ['A' 13 : 6, #[cfg(pads_ge_48)] 'B' 14 : 0]
        MOSI: ['A' 14 : 6, #[cfg(pads_ge_48)] 'B' 15 : 0]
);

/// Word-width marker: 8-bit frames ([`Spi::transfer_byte`], `SpiBus<u8>`).
pub struct Byte;
/// Word-width marker: 16-bit frames ([`Spi::transfer_word`], `SpiBus<u16>`).
pub struct Word;

/// An error the peripheral flagged in `STAT`.
///
/// Its own type rather than [`ErrorKind`], which has no variant for a CRC
/// mismatch; [`kind`] gives the portable classification.
///
/// [`kind`]: embedded_hal::spi::Error::kind
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A received word was overwritten before it had been read (`RXORERR`).
    Overrun,
    /// `NSS` was pulled low while configured as a master (`CONFERR`) — another
    /// master is driving the bus.
    ModeFault,
    /// The received CRC does not match the one computed locally (`CRCERR`).
    Crc,
    /// A frame boundary arrived where none was expected (`FERR`), which only
    /// the TI frame format can report.
    Framing,
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Overrun => ErrorKind::Overrun,
            Self::ModeFault => ErrorKind::ModeFault,
            // No CRC variant exists upstream; this is the loss our own type
            // is here to keep out of the driver-facing API.
            Self::Crc => ErrorKind::Other,
            Self::Framing => ErrorKind::FrameFormat,
        }
    }
}

/// A peripheral that [`Spi`] can drive.
///
/// SPI0 and SPI1 have distinct register block types whose bits do not line up —
/// frame width is `FF16` in `CTL0` on SPI0 but `DZ` in `CTL1` on SPI1, where that
/// position means something else. No generic bound over a shared block is
/// possible, so this trait abstracts the peripheral at the *operation* level:
/// every register access lives in the impls, and [`Spi`] touches none.
pub trait Instance: Enable + Reset {
    /// Writes the full master configuration, leaving the peripheral enabled.
    ///
    /// `wide` selects the frame width, and the impl handles what follows from it
    /// (on SPI1 the FIFO access size must match, or reception stalls).
    fn apply_config(&mut self, config: SpiConfig, wide: bool);
    /// Transmit buffer empty — ready to accept the next word.
    fn tbe(&self) -> bool;
    /// Receive buffer not empty — a word has arrived.
    fn rbne(&self) -> bool;
    /// Writes a word to the data register, which starts the clock in master mode.
    fn write_data(&mut self, word: u16);
    /// Reads the received word from the data register.
    fn read_data(&mut self) -> u16;
    /// Returns the first pending error, if any, clearing it as the manual requires.
    fn take_error(&mut self) -> Option<Error>;
    /// Enables or disables the peripheral (`SPIEN`).
    fn set_enabled(&mut self, on: bool);
    /// Enables or disables the receive interrupt (`RBNEIE`).
    fn set_rbneie(&mut self, on: bool);
    /// Reads back `RBNEIE`.
    fn rbneie(&self) -> bool;
    /// Enables or disables the transmit interrupt (`TBEIE`).
    fn set_tbeie(&mut self, on: bool);
    /// Reads back `TBEIE`.
    fn tbeie(&self) -> bool;
    /// Enables or disables the error interrupt (`ERRIE`).
    fn set_errie(&mut self, on: bool);
    /// Reads back `ERRIE`.
    fn errie(&self) -> bool;
}

impl Instance for pac::Spi0 {
    #[inline]
    fn apply_config(&mut self, config: SpiConfig, wide: bool) {
        self.ctl0().modify(|_, w| {
            let w = w
                .mstmod()
                .set_bit()
                .swnssen()
                .set_bit()
                .swnss()
                .set_bit()
                .lf()
                .bit(config.bit_order == BitOrder::LsbFirst)
                .ff16()
                .bit(wide)
                .spien()
                .set_bit()
                .ckpl()
                .bit(config.mode.polarity == Polarity::IdleHigh)
                .ckph()
                .bit(config.mode.phase == Phase::CaptureOnSecondTransition);
            unsafe { w.psc().bits(config.psc as u8) }
        });
    }
    #[inline]
    fn tbe(&self) -> bool {
        self.stat().read().tbe().bit_is_set()
    }
    #[inline]
    fn rbne(&self) -> bool {
        self.stat().read().rbne().bit_is_set()
    }
    #[inline]
    fn write_data(&mut self, word: u16) {
        self.data().write(|w| unsafe { w.data().bits(word) });
    }
    #[inline]
    fn read_data(&mut self) -> u16 {
        self.data().read().data().bits()
    }
    #[inline]
    fn take_error(&mut self) -> Option<Error> {
        let stat = self.stat().read();
        if stat.rxorerr().bit_is_set() {
            // clear: read DATA (done in transfer_byte) + read STAT (above)
            Some(Error::Overrun)
        } else if stat.conferr().bit_is_set() {
            // clear: read STAT (above) + write CTL0
            self.ctl0().modify(|_, w| w);
            Some(Error::ModeFault)
        } else if stat.crcerr().bit_is_set() {
            self.stat().modify(|_, w| w.crcerr().clear_bit());
            Some(Error::Crc)
        } else if stat.ferr().bit_is_set() {
            self.stat().modify(|_, w| w.ferr().clear_bit());
            Some(Error::Framing)
        } else {
            None
        }
    }
    #[inline]
    fn set_enabled(&mut self, on: bool) {
        self.ctl0().modify(|_, w| w.spien().bit(on));
    }
    #[inline]
    fn set_rbneie(&mut self, on: bool) {
        self.ctl1().modify(|_, w| w.rbneie().bit(on));
    }
    #[inline]
    fn rbneie(&self) -> bool {
        self.ctl1().read().rbneie().bit_is_set()
    }
    #[inline]
    fn set_tbeie(&mut self, on: bool) {
        self.ctl1().modify(|_, w| w.tbeie().bit(on));
    }
    #[inline]
    fn tbeie(&self) -> bool {
        self.ctl1().read().tbeie().bit_is_set()
    }
    #[inline]
    fn set_errie(&mut self, on: bool) {
        self.ctl1().modify(|_, w| w.errie().bit(on));
    }
    #[inline]
    fn errie(&self) -> bool {
        self.ctl1().read().errie().bit_is_set()
    }
}

impl Instance for pac::Spi1 {
    #[inline]
    fn apply_config(&mut self, config: SpiConfig, wide: bool) {
        self.ctl1().modify(|_, w| {
            let w = w.byten().bit(!wide);
            unsafe { w.dz().bits(if wide { DZ_16BIT } else { DZ_8BIT }) }
        });
        self.ctl0().modify(|_, w| {
            let w = w
                .mstmod()
                .set_bit()
                .swnssen()
                .set_bit()
                .swnss()
                .set_bit()
                .lf()
                .bit(config.bit_order == BitOrder::LsbFirst)
                .spien()
                .set_bit()
                .ckpl()
                .bit(config.mode.polarity == Polarity::IdleHigh)
                .ckph()
                .bit(config.mode.phase == Phase::CaptureOnSecondTransition);
            unsafe { w.psc().bits(config.psc as u8) }
        });
    }
    #[inline]
    fn tbe(&self) -> bool {
        self.stat().read().tbe().bit_is_set()
    }
    #[inline]
    fn rbne(&self) -> bool {
        self.stat().read().rbne().bit_is_set()
    }
    #[inline]
    fn write_data(&mut self, word: u16) {
        self.data().write(|w| unsafe { w.data().bits(word) });
    }
    #[inline]
    fn read_data(&mut self) -> u16 {
        self.data().read().data().bits()
    }
    #[inline]
    fn take_error(&mut self) -> Option<Error> {
        let stat = self.stat().read();
        if stat.rxorerr().bit_is_set() {
            // clear: read DATA (done in transfer_byte) + read STAT (above)
            Some(Error::Overrun)
        } else if stat.conferr().bit_is_set() {
            // clear: read STAT (above) + write CTL0
            self.ctl0().modify(|_, w| w);
            Some(Error::ModeFault)
        } else if stat.crcerr().bit_is_set() {
            self.stat().modify(|_, w| w.crcerr().clear_bit());
            Some(Error::Crc)
        } else if stat.ferr().bit_is_set() {
            self.stat().modify(|_, w| w.ferr().clear_bit());
            Some(Error::Framing)
        } else {
            None
        }
    }
    #[inline]
    fn set_enabled(&mut self, on: bool) {
        self.ctl0().modify(|_, w| w.spien().bit(on));
    }
    #[inline]
    fn set_rbneie(&mut self, on: bool) {
        self.ctl1().modify(|_, w| w.rbneie().bit(on));
    }
    #[inline]
    fn rbneie(&self) -> bool {
        self.ctl1().read().rbneie().bit_is_set()
    }
    #[inline]
    fn set_tbeie(&mut self, on: bool) {
        self.ctl1().modify(|_, w| w.tbeie().bit(on));
    }
    #[inline]
    fn tbeie(&self) -> bool {
        self.ctl1().read().tbeie().bit_is_set()
    }
    #[inline]
    fn set_errie(&mut self, on: bool) {
        self.ctl1().modify(|_, w| w.errie().bit(on));
    }
    #[inline]
    fn errie(&self) -> bool {
        self.ctl1().read().errie().bit_is_set()
    }
}

/// An SPI event that can raise an interrupt.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// A word arrived in `DATA`, cleared by reading it.
    Rbne,
    /// The transmit buffer is free — true at idle, so listening for this needs
    /// `unlisten` from inside the handler once nothing is left to send.
    Tbe,
    /// Any of the `STAT` error flags, which share this one enable. Cleared by
    /// [`take_error`](Instance::take_error), which a handler must call —
    /// nothing else drains them.
    Error,
}

/// A configured SPI master, owning the peripheral and its three pins.
///
/// `WORD` records the frame width, so methods of the wrong width don't exist:
/// [`transfer_byte`](Self::transfer_byte) and `SpiBus<u8>` are available only on
/// `Spi<.., Byte>`, [`transfer_word`](Self::transfer_word) and `SpiBus<u16>` only
/// on `Spi<.., Word>`. It defaults to [`Byte`], so the parameter can be omitted.
///
/// Chip select is not handled here — NSS is software-managed, so drive the
/// slave's CS with an ordinary output pin around each transaction.
pub struct Spi<SPIX, SCK, MISO, MOSI, WORD = Byte> {
    spi: SPIX,
    sck_pin: SCK,
    miso_pin: MISO,
    mosi_pin: MOSI,
    _word: PhantomData<WORD>,
}

impl<SPIX, SCK, MISO, MOSI> Spi<SPIX, SCK, MISO, MOSI, Byte>
where
    SPIX: Instance,
    SCK: SckPin<SPIX>,
    MISO: MisoPin<SPIX>,
    MOSI: MosiPin<SPIX>,
{
    /// Enables the peripheral's clock, resets it and configures 8-bit master mode.
    ///
    /// The pins must already be in this SPI's alternate function; the bounds
    /// reject anything else at compile time. [`release`](Spi::release) hands them
    /// back.
    pub fn new(
        rcu: &mut Rcu,
        mut spi: SPIX,
        sck_pin: SCK,
        miso_pin: MISO,
        mosi_pin: MOSI,
        config: SpiConfig,
    ) -> Self {
        SPIX::enable(rcu);
        SPIX::reset(rcu);
        spi.apply_config(config, false);
        Self {
            spi,
            sck_pin,
            miso_pin,
            mosi_pin,
            _word: PhantomData,
        }
    }
}

impl<SPIX, SCK, MISO, MOSI> Spi<SPIX, SCK, MISO, MOSI, Word>
where
    SPIX: Instance,
    SCK: SckPin<SPIX>,
    MISO: MisoPin<SPIX>,
    MOSI: MosiPin<SPIX>,
{
    /// Same as [`new`](Spi::new), but configures 16-bit frames.
    pub fn new_word(
        rcu: &mut Rcu,
        mut spi: SPIX,
        sck_pin: SCK,
        miso_pin: MISO,
        mosi_pin: MOSI,
        config: SpiConfig,
    ) -> Self {
        SPIX::enable(rcu);
        SPIX::reset(rcu);
        spi.apply_config(config, true);
        Self {
            spi,
            sck_pin,
            miso_pin,
            mosi_pin,
            _word: PhantomData,
        }
    }
}

impl<SPIX, SCK, MISO, MOSI, WORD> Spi<SPIX, SCK, MISO, MOSI, WORD>
where
    SPIX: Instance,
{
    /// Returns whether a received word is waiting in `DATA` (`RBNE`).
    ///
    /// Only guarantees that the next single read will not block.
    pub fn read_ready(&self) -> bool {
        self.spi.rbne()
    }
    /// Returns whether `DATA` can accept the next word (`TBE`).
    ///
    /// True at idle, the bus having nothing queued; it says nothing about a word
    /// still on its way back.
    pub fn write_ready(&self) -> bool {
        self.spi.tbe()
    }
    /// Returns the pending error, if any, and clears it.
    ///
    /// The acknowledge for [`Event::Error`]: those flags are what hold the
    /// interrupt request up, so a handler that skips this re-enters forever.
    /// One error per call — with two pending, the second survives for the next.
    /// Overrun also needs `DATA` drained, which the read a handler makes to do
    /// its work already does.
    pub fn take_error(&mut self) -> Option<Error> {
        self.spi.take_error()
    }

    /// Raises an interrupt on `event`, which still has to be unmasked in the
    /// NVIC.
    ///
    /// [`Event::Tbe`] is true whenever nothing is queued, so a handler that
    /// listens for it must call `unlisten` once it has nothing left to send.
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rbne => self.spi.set_rbneie(true),
            Event::Tbe => self.spi.set_tbeie(true),
            Event::Error => self.spi.set_errie(true),
        }
    }
    /// Stops raising an interrupt on `event`.
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rbne => self.spi.set_rbneie(false),
            Event::Tbe => self.spi.set_tbeie(false),
            Event::Error => self.spi.set_errie(false),
        }
    }
    /// Whether `event` is being listened for.
    pub fn is_listening(&self, event: Event) -> bool {
        match event {
            Event::Rbne => self.spi.rbneie(),
            Event::Tbe => self.spi.tbeie(),
            Event::Error => self.spi.errie(),
        }
    }

    /// Disables the peripheral and returns it along with the three pins.
    ///
    /// The clock is left enabled and no reset is performed — a later `new()`
    /// does both anyway.
    pub fn release(mut self) -> (SPIX, SCK, MISO, MOSI) {
        self.spi.set_enabled(false);
        (self.spi, self.sck_pin, self.miso_pin, self.mosi_pin)
    }
}

impl<SPIX, SCK, MISO, MOSI> Spi<SPIX, SCK, MISO, MOSI, Byte>
where
    SPIX: Instance,
{
    /// Hands one byte to `DATA` and returns at once, the clock now running.
    ///
    /// Half an exchange: the answer arrives eight clocks later, on
    /// [`Event::Rbne`]. Check [`write_ready`](Spi::write_ready) first — writing
    /// over a full transmit buffer loses the byte.
    pub fn write_byte(&mut self, byte: u8) {
        self.spi.write_data(byte as u16);
    }
    /// Takes the byte that arrived on MISO, clearing `RBNE`.
    ///
    /// The other half: meaningful only once [`read_ready`](Spi::read_ready)
    /// holds, and it reports no error — [`take_error`](Spi::take_error) drains
    /// those.
    pub fn read_byte(&mut self) -> u8 {
        self.spi.read_data() as u8
    }

    /// Exchanges one byte: sends `byte` on MOSI and returns what arrived on MISO.
    ///
    /// Blocks until the exchange has completed, so nothing is left pending on the
    /// bus when it returns.
    pub fn transfer_byte(&mut self, byte: u8) -> Result<u8, Error> {
        while !self.write_ready() {}
        self.write_byte(byte);
        while !self.read_ready() {}
        let received = self.read_byte();
        match self.take_error() {
            Some(e) => Err(e),
            None => Ok(received),
        }
    }

    /// Exchanges two buffers: byte `i` of `write` goes out and what comes back
    /// lands at byte `i` of `read`.
    ///
    /// # Panics
    ///
    /// If the lengths differ. The bus trades a word for a word, so a longer
    /// `read` would have nothing to clock out; what fills the wire is the
    /// target's business, and [`fill`] names the usual levels.
    pub fn transfer_bytes(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        assert!(
            read.len() == write.len(),
            "SPI transfer buffers must be the same length"
        );
        for (slot, &byte) in read.iter_mut().zip(write) {
            *slot = self.transfer_byte(byte)?;
        }
        Ok(())
    }
    /// Exchanges `words` against itself: each byte is replaced by what came back.
    pub fn transfer_bytes_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
        for word in words {
            *word = self.transfer_byte(*word)?;
        }
        Ok(())
    }
}

impl<SPIX, SCK, MISO, MOSI> Spi<SPIX, SCK, MISO, MOSI, Word>
where
    SPIX: Instance,
{
    /// Hands one 16-bit word to `DATA` and returns at once, the clock now running.
    ///
    /// Half an exchange: the answer arrives sixteen clocks later, on
    /// [`Event::Rbne`]. Check [`write_ready`](Spi::write_ready) first — writing
    /// over a full transmit buffer loses the word.
    pub fn write_word(&mut self, word: u16) {
        self.spi.write_data(word);
    }
    /// Takes the word that arrived on MISO, clearing `RBNE`.
    ///
    /// The other half: meaningful only once [`read_ready`](Spi::read_ready)
    /// holds, and it reports no error — [`take_error`](Spi::take_error) drains
    /// those.
    pub fn read_word(&mut self) -> u16 {
        self.spi.read_data()
    }

    /// Exchanges one 16-bit word: sends `word` on MOSI, returns what arrived on MISO.
    ///
    /// Blocks until the exchange has completed, so nothing is left pending on the
    /// bus when it returns.
    pub fn transfer_word(&mut self, word: u16) -> Result<u16, Error> {
        while !self.write_ready() {}
        self.write_word(word);
        while !self.read_ready() {}
        let received = self.read_word();
        match self.take_error() {
            Some(e) => Err(e),
            None => Ok(received),
        }
    }

    /// Exchanges two buffers: word `i` of `write` goes out and what comes back
    /// lands at word `i` of `read`.
    ///
    /// # Panics
    ///
    /// If the lengths differ. The bus trades a word for a word, so a longer
    /// `read` would have nothing to clock out; what fills the wire is the
    /// target's business, and [`fill`] names the usual levels.
    pub fn transfer_words(&mut self, read: &mut [u16], write: &[u16]) -> Result<(), Error> {
        assert!(
            read.len() == write.len(),
            "SPI transfer buffers must be the same length"
        );
        for (slot, &word) in read.iter_mut().zip(write) {
            *slot = self.transfer_word(word)?;
        }
        Ok(())
    }
    /// Exchanges `words` against itself: each word is replaced by what came back.
    pub fn transfer_words_in_place(&mut self, words: &mut [u16]) -> Result<(), Error> {
        for word in words {
            *word = self.transfer_word(*word)?;
        }
        Ok(())
    }
}

impl<SPIX, SCK, MISO, MOSI, WORD> ErrorType for Spi<SPIX, SCK, MISO, MOSI, WORD>
where
    SPIX: Instance,
{
    type Error = Error;
}

impl<SPIX, SCK, MISO, MOSI> SpiBus<u8> for Spi<SPIX, SCK, MISO, MOSI, Byte>
where
    SPIX: Instance,
{
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer_bytes(read, write)
    }
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer_bytes_in_place(words)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        // No-op: transfer_byte blocks until RBNE (the byte is fully exchanged),
        // so nothing is ever pending on the bus when a method returns.
        Ok(())
    }
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for slot in words {
            *slot = self.transfer_byte(fill::LOW)?;
        }
        Ok(())
    }
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &byte in words {
            self.transfer_byte(byte)?;
        }
        Ok(())
    }
}

impl<SPIX, SCK, MISO, MOSI> SpiBus<u16> for Spi<SPIX, SCK, MISO, MOSI, Word>
where
    SPIX: Instance,
{
    fn transfer(&mut self, read: &mut [u16], write: &[u16]) -> Result<(), Self::Error> {
        self.transfer_words(read, write)
    }
    fn transfer_in_place(&mut self, words: &mut [u16]) -> Result<(), Self::Error> {
        self.transfer_words_in_place(words)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        // No-op: transfer_word blocks until RBNE (the word is fully exchanged),
        // so nothing is ever pending on the bus when a method returns.
        Ok(())
    }
    fn read(&mut self, words: &mut [u16]) -> Result<(), Self::Error> {
        for slot in words {
            *slot = self.transfer_word(fill::LOW_WORD)?;
        }
        Ok(())
    }
    fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
        for &word in words {
            self.transfer_word(word)?;
        }
        Ok(())
    }
}
