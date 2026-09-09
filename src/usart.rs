//! Asynchronous serial (USART), blocking and non-blocking.
//!
//! Word width is a typestate: a [`Usart<.., Byte>`](Usart) moves `u8` values and
//! is what [`new`](Usart::new) builds, while [`new_word`](Usart::new_word) gives a
//! `Usart<.., Word>` for raw 9-bit frames carrying `u16`. Methods of the other
//! width don't exist on either, so the two can't be mixed up.
//!
//! ```ignore
//! let tx = parts.pa9.into_alternate::<1>();
//! let rx = parts.pa10.into_alternate::<1>();
//! let mut serial = Usart::new(&mut rcu, dp.usart0, tx, rx, UsartConfig::default());
//! serial.write_byte(b'x');
//! ```
//!
//! Which pins are valid depends on the chip variant — on the x8 part `PA2`/`PA3`
//! reach USART1, not USART0 — but that is settled at compile time by the pin
//! bounds, so a wrong pin simply fails to build.
//!
//! A receiver just built takes the first frame apart wrongly unless the line has
//! been idle for a frame beforehand, and with frames arriving back to back the
//! damage carries through the whole burst. Against a peer that comes up at the
//! same time this never shows: raising `TEN` sends an idle frame. It bites when
//! the peer is already transmitting — then the line has to be given that idle,
//! by waiting a frame time before the traffic starts.

use core::marker::PhantomData;
use core::ops::Deref;

use crate::gpio::{Alternate, Pin};
use crate::pac;
use crate::rcu::{Clocks, Enable, Rcu, Reset};
use crate::time::{Bps, Hertz};

/// 7 data bits: parity occupies bit 7 inside the u8 (`E7`/`O7`).
const DATA_7BIT_MASK: u8 = 0x7F;
/// 9 data bits: the full 9-bit word in `WL=1, PCEN=0` mode.
const DATA_9BIT_MASK: u32 = 0x1FF;

/// Marks a pin usable as `TX` for `USART`, in the right alternate function.
pub trait TxPin<USART> {}
/// Marks a pin usable as `RX` for `USART`, in the right alternate function.
pub trait RxPin<USART> {}

macro_rules! usart_pins {
    ( $( $USART:ty:
        TX: [ $($(#[$tx_cfg:meta])? $tx_p:literal $tx_n:literal : $tx_af:literal),* $(,)? ]
        RX: [ $($(#[$rx_cfg:meta])? $rx_p:literal $rx_n:literal : $rx_af:literal),* $(,)? ]
    ),* $(,)? ) => {
        $(
            $( $(#[$tx_cfg])? impl TxPin<$USART> for Pin<$tx_p, $tx_n, Alternate<$tx_af>> {} )*
            $( $(#[$rx_cfg])? impl RxPin<$USART> for Pin<$rx_p, $rx_n, Alternate<$rx_af>> {} )*
        )*
    };
}

// PA2/PA3/PA14/PA15 at AF1 belong to a *different* USART depending on the chip
// variant (datasheet Table 2-13 footnotes): USART0 on GD32E230x4, USART1 on
// GD32E230x8/6. They are therefore listed in the gated blocks, not here.
//
// The `pads_ge_*` gates say the package bonds the pin at all, and match the ones in
// `gpio::Parts` — an entry for an unbonded pad would advertise in the docs a pin
// nobody can obtain.
usart_pins! {
    pac::Usart0:
        TX: [ 'A' 9:1, #[cfg(pads_ge_24)] 'B' 6:0 ]
        RX: [ 'A' 10:1, #[cfg(pads_ge_24)] 'B' 7:0 ],
}

// ---- (1) GD32E230x4 only: PA2/PA3/PA14/PA15 AF1 are USART0 ----
#[cfg(chip_x4)]
usart_pins! {
    pac::Usart0:
        TX: [ 'A' 2:1, 'A' 14:1 ]
        RX: [ 'A' 3:1, #[cfg(pads_ge_28)] 'A' 15:1 ],
}

// ---- (2) GD32E230x8/6: PA2/PA3/PA14/PA15 AF1 are USART1; USART1 exists ----
#[cfg(any(chip_x6, chip_x8))]
usart_pins! {
    pac::Usart1:
        TX: [ 'A' 2:1, #[cfg(pads_ge_24)] 'A' 8:4, 'A' 14:1 ]
        RX: [ 'A' 3:1, #[cfg(pads_ge_28)] 'A' 15:1, #[cfg(pads_ge_24)] 'B' 0:4 ],
}

/// Supplies the clock frequency feeding a given USART.
///
/// USART0 can be reclocked away from its bus (see
/// [`Usart0Sel`](crate::rcu::Usart0Sel)), USART1 always runs off APB1. Resolving
/// it per peripheral type keeps the baud divisor off the wrong frequency.
pub trait BusClocks {
    /// Returns the frequency actually clocking this USART.
    fn clock(clocks: &Clocks) -> Hertz;
}

impl BusClocks for pac::Usart0 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.usart0()
    }
}

impl BusClocks for pac::Usart1 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk1()
    }
}

/// Named constants for the standard bit rates.
///
/// Purely a readability aid — [`UsartConfig::baud`] takes any [`Bps`], the
/// hardware divisor not being restricted to these values. `115_200.bps()` says
/// the same thing.
#[allow(missing_docs)]
pub mod baud {
    use crate::time::Bps;

    pub const B110: Bps = Bps::from_raw(110);
    pub const B300: Bps = Bps::from_raw(300);
    pub const B600: Bps = Bps::from_raw(600);
    pub const B1200: Bps = Bps::from_raw(1_200);
    pub const B2400: Bps = Bps::from_raw(2_400);
    pub const B4800: Bps = Bps::from_raw(4_800);
    pub const B9600: Bps = Bps::from_raw(9_600);
    pub const B14400: Bps = Bps::from_raw(14_400);
    pub const B19200: Bps = Bps::from_raw(19_200);
    pub const B38400: Bps = Bps::from_raw(38_400);
    pub const B57600: Bps = Bps::from_raw(57_600);
    pub const B115200: Bps = Bps::from_raw(115_200);
    pub const B230400: Bps = Bps::from_raw(230_400);
    pub const B460800: Bps = Bps::from_raw(460_800);
    pub const B921600: Bps = Bps::from_raw(921_600);
}

/// How many times each bit is sampled.
///
/// ×16 is the default and more tolerant of clock error; ×8 halves the sampling
/// rate, which allows higher bit rates from the same peripheral clock.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Oversampling {
    /// 8 samples per bit — allows twice the bit rate from the same clock.
    X8,
    /// 16 samples per bit — the default, more tolerant of clock error.
    X16,
}

/// Word length and parity, as a single setting.
///
/// Named for the frame as the caller sees it, not for the register bits:
/// `E7`/`O7` leave 7 data bits because parity replaces the top one, `E8`/`O8`
/// keep all 8 by widening the frame to 9 bits. No `N7` exists — without parity a
/// frame carries the full 8 bits, which is `N8`. For raw 9-bit words see
/// [`Usart::new_word`].
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FrameFormat {
    /// 8 data bits, no parity.
    N8,
    /// 8 data bits, even parity.
    E8,
    /// 8 data bits, odd parity.
    O8,
    /// 7 data bits, even parity.
    E7,
    /// 7 data bits, odd parity.
    O7,
}

/// Configuration for [`Usart::new`].
///
/// [`Default`] is 115200 baud, ×16 oversampling, [`FrameFormat::N8`] — i.e. the
/// usual "115200 8N1".
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsartConfig {
    baud: Bps,
    oversampling: Oversampling,
    frame_format: FrameFormat,
}

impl UsartConfig {
    /// Sets the bit rate. See the [`baud`] module for named constants.
    pub const fn baud(mut self, baud: Bps) -> Self {
        self.baud = baud;
        self
    }
    /// Sets the oversampling ratio.
    pub const fn oversampling(mut self, oversampling: Oversampling) -> Self {
        self.oversampling = oversampling;
        self
    }
    /// Sets the word length and parity.
    pub const fn frame_format(mut self, frame_format: FrameFormat) -> Self {
        self.frame_format = frame_format;
        self
    }
}

impl Default for UsartConfig {
    fn default() -> Self {
        Self {
            baud: baud::B115200,
            oversampling: Oversampling::X16,
            frame_format: FrameFormat::N8,
        }
    }
}

/// Configuration for [`Usart::new_word`].
///
/// Deliberately has no frame-format field: the 9-bit path is always
/// "9 data bits, no parity", so there would be nothing meaningful to choose.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsartConfig9 {
    baud: Bps,
    oversampling: Oversampling,
}

impl UsartConfig9 {
    /// Sets the bit rate. See the [`baud`] module for named constants.
    pub const fn baud(mut self, baud: Bps) -> Self {
        self.baud = baud;
        self
    }
    /// Sets the oversampling ratio.
    pub const fn oversampling(mut self, oversampling: Oversampling) -> Self {
        self.oversampling = oversampling;
        self
    }
}

impl Default for UsartConfig9 {
    fn default() -> Self {
        Self {
            baud: baud::B115200,
            oversampling: Oversampling::X16,
        }
    }
}

fn configure<USARTX>(rcu: &mut Rcu, usart: &USARTX, baud: Bps, oversampling: Oversampling)
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock> + Enable + Reset + BusClocks,
{
    let clocks = rcu.clocks();
    USARTX::enable(rcu);
    USARTX::reset(rcu);
    let pclk = USARTX::clock(&clocks).to_Hz();
    let baud = baud.to_raw();
    // round(pclk / baud) in integers: adding half the divisor before truncating rounds.
    let usartdiv = (pclk + baud / 2) / baud;
    usart.baud().write(|w| unsafe {
        match oversampling {
            Oversampling::X16 => w.bits(usartdiv),
            Oversampling::X8 => {
                let intdiv = usartdiv / 8;
                let fradiv_8 = usartdiv % 8;
                w.bits((intdiv << 4) | (fradiv_8 & 0x7))
            }
        }
    });

    // UEN, TEN and REN are all left off here: WL and parity must be written
    // while the USART is disabled, and the manual brings the three up in their
    // own order afterwards (UEN, then the transmitter and receiver).
    usart.ctl0().modify(|_, w| match oversampling {
        Oversampling::X16 => w.ovsmod().oversampling16(),
        Oversampling::X8 => w.ovsmod().oversampling8(),
    });
}

/// Brings the peripheral up, in the order the manual prescribes: `UEN` first,
/// the transmitter and receiver after it.
fn enable<USARTX>(usart: &USARTX)
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    usart.ctl0().modify(|_, w| w.uen().enabled());
    usart
        .ctl0()
        .modify(|_, w| w.ten().enabled().ren().enabled());
}

/// Word-width marker: 8-bit words ([`Usart::write_byte`], [`Usart::read_byte`]),
/// optionally with parity.
pub struct Byte;
/// Word-width marker: raw 9-bit words ([`Usart::write_word`],
/// [`Usart::read_word`]), no parity possible.
pub struct Word;

/// A line error the receiver reported for one frame.
///
/// Its own type rather than a foreign `ErrorKind`, so what `STAT` distinguishes
/// stays distinguishable. The portable classifications are one `kind` call away
/// per ecosystem ([`embedded_hal_nb::serial::Error::kind`],
/// [`embedded_io::Error::kind`]), both lossy — neither has a variant for every
/// line condition named here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A frame arrived before the previous one had been read, and was lost.
    Overrun,
    /// The sampling logic disagreed with itself about a bit's level.
    Noise,
    /// The stop bit was not where the configured frame said it would be —
    /// usually a baud rate or frame format mismatch between the two ends.
    Framing,
    /// The parity bit contradicts the data bits.
    Parity,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Overrun => write!(f, "receive overrun, a frame was lost"),
            Self::Noise => write!(f, "noise detected on the line"),
            Self::Framing => write!(f, "framing error, no stop bit where expected"),
            Self::Parity => write!(f, "parity check failed"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Framing | Self::Noise | Self::Parity => embedded_io::ErrorKind::InvalidData,
            Self::Overrun => embedded_io::ErrorKind::Other,
        }
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            Self::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Self::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
            Self::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Self::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
        }
    }
}

/// A USART event that can raise an interrupt.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// A byte arrived in `RDATA`, cleared by reading it.
    ///
    /// Also carries overrun: `ORERR` reaches the NVIC through this enable as
    /// well as through [`Error`](Self::Error), and
    /// [`read_byte`](Usart::read_byte) acknowledges it either way.
    Rbne,
    /// The transmit buffer is free — true at idle, so listening for this needs
    /// `unlisten` from inside the handler once nothing is left to send.
    Tbe,
    /// A framing, noise or overrun error.
    ///
    /// Reaches the NVIC **only while receiving through DMA**: in hardware the
    /// enable is ANDed with the DMA request line, so without it the interrupt
    /// never fires. Cleared by [`take_error`](Usart::take_error), which a
    /// handler must call — nothing else drains these flags on the DMA path.
    Error,
    /// The parity check failed.
    ///
    /// Its own enable, separate from [`Error`](Self::Error) and not gated by
    /// DMA. Cleared by [`take_error`](Usart::take_error).
    ParityError,
}

/// A configured USART, owning the peripheral and both pins.
///
/// `WORD` records the word width, so methods of the wrong width don't exist:
/// `write_byte`/`read_byte` are available only on `Usart<.., Byte>`, and
/// `write_word`/`read_word` only on `Usart<.., Word>`. It defaults to [`Byte`],
/// so the parameter can be omitted.
pub struct Usart<USARTX, TX, RX, WORD = Byte> {
    usart: USARTX,
    tx_pin: TX,
    rx_pin: RX,
    frame_format: FrameFormat,
    _word: PhantomData<WORD>,
}

impl<USARTX, TX, RX, WORD> Usart<USARTX, TX, RX, WORD>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    /// Returns the pending receive error, if any, and clears it.
    ///
    /// The acknowledge for [`Event::Error`] and [`Event::ParityError`]: those
    /// flags are what hold the interrupt request up, and on the DMA receive path
    /// nothing else drains them, so a handler that skips this re-enters forever.
    /// One error per call — with two pending, the second survives for the next.
    /// Also drains `RDATA`, the frame having been lost or corrupted either way.
    pub fn take_error(&mut self) -> Option<Error> {
        let stat = self.usart.stat().read();
        let error = if stat.orerr().bit_is_set() {
            self.usart.intc().write(|w| w.orec().clear());
            Some(Error::Overrun)
        } else if stat.nerr().bit_is_set() {
            self.usart.intc().write(|w| w.nec().clear());
            Some(Error::Noise)
        } else if stat.ferr().bit_is_set() {
            self.usart.intc().write(|w| w.fec().clear());
            Some(Error::Framing)
        } else if stat.perr().bit_is_set() {
            self.usart.intc().write(|w| w.pec().clear());
            Some(Error::Parity)
        } else {
            None
        };

        if error.is_some() {
            self.usart.rdata().read();
        }

        error
    }
    fn rbne(&self) -> bool {
        self.usart.stat().read().rbne().bit_is_set()
    }
    fn tbe(&self) -> bool {
        self.usart.stat().read().tbe().bit_is_set()
    }
    fn tc(&self) -> bool {
        self.usart.stat().read().tc().bit_is_set()
    }
    fn wait_tc(&self) {
        while !self.tc() {}
    }

    /// Returns whether a received word is waiting to be read.
    ///
    /// Only guarantees that the *next* single read will not block; a buffered
    /// read may still block once it has taken what was already there.
    pub fn read_ready(&self) -> bool {
        self.rbne()
    }
    /// Returns whether the transmit buffer can accept a word right now.
    ///
    /// Only guarantees that the *next* single write will not block.
    pub fn write_ready(&self) -> bool {
        self.tbe()
    }
    /// Returns whether everything handed to the peripheral has left the wire.
    ///
    /// `TC`, the flag [`flush`](Usart::flush) waits for, so this is the same
    /// question asked without blocking.
    pub fn flush_ready(&self) -> bool {
        self.tc()
    }
    /// Blocks until everything handed to the peripheral has left the wire.
    ///
    /// Waits for `TC`, not `TBE` — the latter only says `TDATA` reached the shift
    /// register while the byte is still being clocked out. Call this before
    /// cutting power to a transceiver or sleeping. Cannot fail.
    pub fn flush(&self) {
        self.wait_tc();
    }

    /// Lets `event` raise an interrupt.
    ///
    /// Half of what an interrupt takes: the request now reaches the NVIC, which
    /// still has the line masked. Unmasking it — `NVIC::unmask` on the
    /// peripheral's [`Interrupt`](crate::pac::Interrupt) — is the caller's, this
    /// crate does not touch core registers.
    ///
    /// No event needs a separate clear — each is acknowledged by the same call a
    /// handler makes to do its work: `Rbne` by [`read_byte`](Usart::read_byte),
    /// `Tbe` by [`write_byte`](Usart::write_byte), both error events by
    /// [`take_error`](Usart::take_error). `Tbe` is set whenever nothing is
    /// queued, so a handler that listens for it must call `unlisten` once it has
    /// nothing left to send, or it re-enters at once.
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rbne => self.usart.ctl0().modify(|_, w| w.rbneie().enabled()),
            Event::Tbe => self.usart.ctl0().modify(|_, w| w.tbeie().enabled()),
            Event::Error => self.usart.ctl2().modify(|_, w| w.errie().enabled()),
            Event::ParityError => self.usart.ctl0().modify(|_, w| w.perrie().enabled()),
        }
    }
    /// Stops `event` from raising an interrupt. Leaves the NVIC alone.
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rbne => self.usart.ctl0().modify(|_, w| w.rbneie().disabled()),
            Event::Tbe => self.usart.ctl0().modify(|_, w| w.tbeie().disabled()),
            Event::Error => self.usart.ctl2().modify(|_, w| w.errie().disabled()),
            Event::ParityError => self.usart.ctl0().modify(|_, w| w.perrie().disabled()),
        }
    }
    /// Whether `event` currently raises an interrupt.
    ///
    /// Every event here shares one NVIC line, so a handler needs this to tell
    /// which of them woke it: the flag alone is not enough, since `Tbe` is set
    /// at idle regardless of whether it is being listened for.
    pub fn is_listening(&self, event: Event) -> bool {
        match event {
            Event::Rbne => self.usart.ctl0().read().rbneie().is_enabled(),
            Event::Tbe => self.usart.ctl0().read().tbeie().is_enabled(),
            Event::Error => self.usart.ctl2().read().errie().is_enabled(),
            Event::ParityError => self.usart.ctl0().read().perrie().is_enabled(),
        }
    }

    /// Disables the peripheral and returns it along with both pins.
    ///
    /// The clock is left enabled and no reset is performed — a later `new()`
    /// does both anyway.
    pub fn release(self) -> (USARTX, TX, RX) {
        self.usart.ctl0().modify(|_, w| w.uen().disabled());
        (self.usart, self.tx_pin, self.rx_pin)
    }
}

impl<USARTX, TX, RX> Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn received_byte(&mut self) -> u8 {
        let raw = self.usart.rdata().read().bits() as u8;
        match self.frame_format {
            FrameFormat::E7 | FrameFormat::O7 => raw & DATA_7BIT_MASK,
            FrameFormat::N8 | FrameFormat::E8 | FrameFormat::O8 => raw,
        }
    }

    /// Sends one byte, blocking until the transmit buffer can accept it.
    ///
    /// Returning does not mean the byte has left the wire — for that, see
    /// [`flush`](Usart::flush).
    pub fn write_byte(&mut self, byte: u8) {
        while !self.tbe() {}
        self.usart.tdata().write(|w| unsafe { w.bits(byte as u32) });
    }
    /// Sends every byte of `buf`, blocking until the last one is handed over.
    ///
    /// Returning does not mean the buffer has left the wire — for that see
    /// [`flush`](Usart::flush). Cannot fail.
    pub fn write_bytes(&mut self, buf: &[u8]) {
        for &byte in buf {
            self.write_byte(byte);
        }
    }
    /// Receives one byte, blocking until one arrives.
    ///
    /// A line error consumes the offending frame and is reported instead of the
    /// data, so a damaged byte is never mistaken for a good one.
    pub fn read_byte(&mut self) -> Result<u8, Error> {
        while !self.rbne() {}
        if let Some(e) = self.take_error() {
            Err(e)
        } else {
            Ok(self.received_byte())
        }
    }
    /// Receives into `buf` and returns how many bytes were placed there.
    ///
    /// Blocks until at least one byte arrives, then takes whatever else is
    /// waiting and returns — it does *not* wait for `buf` to fill, which would
    /// deadlock against a peer waiting for its answer. Returns `0` only for an
    /// empty `buf`. A line error ends the call at once, losing the bytes copied
    /// before it: the count is not reported alongside an error.
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut index = 0;
        while !self.rbne() {}
        while index < buf.len() && self.rbne() {
            buf[index] = self.read_byte()?;
            index += 1;
        }
        Ok(index)
    }
}

impl<USARTX, TX, RX> Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock> + Enable + Reset + BusClocks,
    TX: TxPin<USARTX>,
    RX: RxPin<USARTX>,
{
    /// Enables the peripheral's clock, resets it and configures 8-bit words.
    ///
    /// The pins must already be in this USART's alternate function; the bounds
    /// reject anything else at compile time. [`release`](Usart::release) hands
    /// them back. The baud divisor comes from the frozen clocks in `rcu`.
    pub fn new(rcu: &mut Rcu, usart: USARTX, tx_pin: TX, rx_pin: RX, config: UsartConfig) -> Self {
        configure(rcu, &usart, config.baud, config.oversampling);

        usart.ctl0().modify(|_, w| match config.frame_format {
            FrameFormat::N8 => w.pcen().disabled().wl().bit8(),
            FrameFormat::E8 => w.pcen().enabled().pm().even().wl().bit9(),
            FrameFormat::O8 => w.pcen().enabled().pm().odd().wl().bit9(),
            FrameFormat::E7 => w.pcen().enabled().pm().even().wl().bit8(),
            FrameFormat::O7 => w.pcen().enabled().pm().odd().wl().bit8(),
        });
        enable(&usart);
        Self {
            usart,
            tx_pin,
            rx_pin,
            frame_format: config.frame_format,
            _word: PhantomData,
        }
    }
}

impl<USARTX, TX, RX> Usart<USARTX, TX, RX, Word>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    /// Sends one 9-bit word, blocking until the transmit buffer can accept it.
    ///
    /// Bits above the ninth are discarded.
    pub fn write_word(&mut self, word: u16) {
        while !self.tbe() {}
        self.usart
            .tdata()
            .write(|w| unsafe { w.bits(word as u32 & DATA_9BIT_MASK) });
    }
    /// Sends every word of `buf`, blocking until the last one is handed over.
    ///
    /// Returning does not mean the buffer has left the wire — for that see
    /// [`flush`](Usart::flush). Cannot fail.
    pub fn write_words(&mut self, buf: &[u16]) {
        for &word in buf {
            self.write_word(word);
        }
    }
    /// Receives one 9-bit word, blocking until one arrives.
    pub fn read_word(&mut self) -> Result<u16, Error> {
        while !self.rbne() {}
        if let Some(e) = self.take_error() {
            Err(e)
        } else {
            Ok((self.usart.rdata().read().bits() & DATA_9BIT_MASK) as u16)
        }
    }
    /// Receives into `buf` and returns how many words were placed there.
    ///
    /// Same blocking rule as [`read_bytes`](Usart::read_bytes): waits for the
    /// first word, then takes only what is already waiting.
    pub fn read_words(&mut self, buf: &mut [u16]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut index = 0;
        while !self.rbne() {}
        while index < buf.len() && self.rbne() {
            buf[index] = self.read_word()?;
            index += 1;
        }
        Ok(index)
    }
}

impl<USARTX, TX, RX> Usart<USARTX, TX, RX, Word>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock> + Enable + Reset + BusClocks,
    TX: TxPin<USARTX>,
    RX: RxPin<USARTX>,
{
    /// Same as [`new`](Usart::new), but configures raw 9-bit words with no parity.
    ///
    /// All nine bits carry data, so there is no frame format to choose and the
    /// peripheral moves `u16` rather than `u8`.
    pub fn new_word(
        rcu: &mut Rcu,
        usart: USARTX,
        tx_pin: TX,
        rx_pin: RX,
        config: UsartConfig9,
    ) -> Self {
        configure(rcu, &usart, config.baud, config.oversampling);
        usart.ctl0().modify(|_, w| w.pcen().disabled().wl().bit9());
        enable(&usart);
        Self {
            usart,
            tx_pin,
            rx_pin,
            // Never read: only `Byte`'s `received_byte` looks at `frame_format`.
            frame_format: FrameFormat::N8,
            _word: PhantomData,
        }
    }
}

impl<USARTX, TX, RX, WORD> embedded_hal_nb::serial::ErrorType for Usart<USARTX, TX, RX, WORD> {
    type Error = Error;
}

impl<USARTX, TX, RX, WORD> embedded_io::ErrorType for Usart<USARTX, TX, RX, WORD> {
    type Error = Error;
}

impl<USARTX, TX, RX> embedded_hal_nb::serial::Read<u8> for Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if !self.rbne() {
            return Err(nb::Error::WouldBlock);
        }
        if let Some(e) = self.take_error() {
            Err(nb::Error::Other(e))
        } else {
            Ok(self.received_byte())
        }
    }
}

impl<USARTX, TX, RX> embedded_io::Read for Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_bytes(buf)
    }
}

impl<USARTX, TX, RX> embedded_hal_nb::serial::Write<u8> for Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        if !self.tbe() {
            Err(nb::Error::WouldBlock)
        } else {
            self.usart.tdata().write(|w| unsafe { w.bits(byte as u32) });
            Ok(())
        }
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if self.tc() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<USARTX, TX, RX> embedded_io::Write for Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        match buf.first() {
            Some(&b) => {
                self.write_byte(b);
                Ok(1usize)
            }
            None => Ok(0usize),
        }
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_tc();
        Ok(())
    }
}

impl<USARTX, TX, RX> embedded_io::ReadReady for Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rbne())
    }
}

impl<USARTX, TX, RX> embedded_io::WriteReady for Usart<USARTX, TX, RX, Byte>
where
    USARTX: Deref<Target = pac::usart0::RegisterBlock>,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tbe())
    }
}
