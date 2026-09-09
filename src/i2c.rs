//! I²C master.
//!
//! Blocking, 7-bit addressing, both peripherals. Transactions are `write`,
//! `read` and `write_read` (the last joined by a repeated START); both lines
//! must be open-drain, which the pin bounds enforce.
//!
//! ```ignore
//! let sda = parts.pb7.into_alternate_open_drain::<1>();
//! let scl = parts.pb6.into_alternate_open_drain::<1>();
//! let mut i2c = I2c::new(&mut rcu, dp.i2c0, sda, scl, I2cMode::standard(100.kHz()));
//! ```
//!
//! # Driving a transaction by hand
//!
//! [`WriteTransfer`] and [`ReadTransfer`] are one way to do it; the primitives
//! they are built from are public, so a handler can be written instead. A
//! transaction is a sequence of phases, and the step a handler takes depends on
//! which one it is in — unlike a USART, where every step is the same.
//!
//! Writing: [`write_start`](I2c::write_start), then on
//! [`sbsend`](I2c::sbsend) [`write_addr`](I2c::write_addr), then on
//! [`addsend`](I2c::addsend) [`clear_addsend`](I2c::clear_addsend), then one
//! [`write_byte`](I2c::write_byte) per [`tbe`](I2c::tbe). `TBE` stays up with
//! nothing left to feed, so `unlisten(Event::Byte)` after the last one and end
//! on [`btc`](I2c::btc) with [`write_stop`](I2c::write_stop).
//!
//! Reading is the same up to `ADDSEND`, and then turns on the acknowledge:
//! [`set_acken`](I2c::set_acken) governs the byte *already* being clocked in, so
//! the NAK that ends the transaction has to be placed one byte early, and
//! [`set_poap`](I2c::set_poap) moves it one further still. Both must stand
//! before the START. Four lengths behave differently: one byte NAKs before
//! `ADDSEND` is cleared; two ride on `POAP` and are taken together on `BTC`;
//! three go straight to that tail; longer ones take bytes on
//! [`rbne`](I2c::rbne) until three remain. Getting this wrong does not fail
//! loudly — the target keeps driving the line and the bus hangs.

use core::mem::ManuallyDrop;
use core::ops::Deref;
use core::ptr;

use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};

use crate::gpio::{Alternate, OpenDrain, Pin};
use crate::pac;
use crate::rcu::{Enable, Rcu, Reset};
use crate::time::Hertz;

/// Smallest `CLKC` the hardware honours.
const CLKC_MIN_STANDARD: u32 = 4;
const CLKC_MIN_FAST: u32 = 1;

/// Slowest `pclk1` each mode can be driven from, per the manual.
const MIN_PCLK1_STANDARD_HZ: u32 = 2_000_000;
const MIN_PCLK1_FAST_HZ: u32 = 8_000_000;
const MIN_PCLK1_FAST_PLUS_HZ: u32 = 24_000_000;

/// Longest SCL rise time the I²C specification allows per mode, in nanoseconds
/// (NXP UM10204) — a property of the pulled-up line, not of the controller.
const RISE_TIME_STANDARD_NS: u32 = 1000;
const RISE_TIME_FAST_NS: u32 = 300;
const RISE_TIME_FAST_PLUS_NS: u32 = 120;

/// A peripheral that [`I2c`] can drive.
///
/// I2C0 and I2C1 share one register block layout, so the driver is written once
/// over [`Deref`]; the supertraits are what the constructor needs.
pub trait Instance: Deref<Target = pac::i2c0::RegisterBlock> + Enable + Reset {}

impl Instance for pac::I2c0 {}
impl Instance for pac::I2c1 {}

/// Marks a pin usable as `SDA` for `I2C`, in the right alternate function.
pub trait SdaPin<I2C> {}
/// Marks a pin usable as `SCL` for `I2C`, in the right alternate function.
pub trait SclPin<I2C> {}

macro_rules! i2c_pins {
    ( $( $I2C:ty:
        SDA: [ $($(#[$sda_cfg:meta])? $sda_p:literal $sda_n:literal : $sda_af:literal),* $(,)? ]
        SCL: [ $($(#[$scl_cfg:meta])? $scl_p:literal $scl_n:literal : $scl_af:literal),* $(,)? ]
    ),* $(,)? ) => {
        $(
            $($(#[$sda_cfg])? impl SdaPin<$I2C> for Pin<$sda_p, $sda_n, Alternate<$sda_af, OpenDrain>> {})*
            $($(#[$scl_cfg])? impl SclPin<$I2C> for Pin<$scl_p, $scl_n, Alternate<$scl_af, OpenDrain>> {})*
        )*
    };
}

// PB10/PB11 at AF1 belong to a *different* I²C depending on the chip variant
// (datasheet Table 2-14 footnotes): I2C0 on GD32E230x4, I2C1 on GD32E230x8.
// They are therefore listed in the gated blocks, not here.
//
// The `pads_ge_*` gates say the package bonds the pin at all, and match the ones in
// `gpio::Parts` — an entry for an unbonded pad would advertise in the docs a pin
// nobody can obtain.
i2c_pins!(
    pac::I2c0:
        SDA: ['A' 10 : 4, #[cfg(pads_ge_24)] 'B' 7 : 1, #[cfg(pads_ge_48)] 'B' 9 : 1, 'F' 0 : 1]
        SCL: ['A' 9 : 4, #[cfg(pads_ge_24)] 'B' 6 : 1, #[cfg(pads_ge_qfn32)] 'B' 8 : 1, 'F' 1 : 1],
);

// ---- (1) GD32E230x4 only: PB10/PB11 AF1 and PF6/PF7 AF0 are I2C0 ----
#[cfg(chip_x4)]
i2c_pins!(
    pac::I2c0:
        SDA: [#[cfg(pads_ge_48)] 'B' 11 : 1, #[cfg(pads_ge_48)] 'F' 7 : 0]
        SCL: [#[cfg(pads_ge_48)] 'B' 10 : 1, #[cfg(pads_ge_48)] 'F' 6 : 0],
);

// ---- (3) GD32E230x8 only: I2C1 exists, and PB10/PB11 AF1 belong to it ----
#[cfg(chip_x8)]
i2c_pins!(
    pac::I2c1:
        SDA: ['A' 1 : 4, #[cfg(pads_ge_lqfp32)] 'A' 12 : 5, #[cfg(pads_ge_48)] 'B' 11 : 1, #[cfg(pads_ge_48)] 'B' 14 : 5, #[cfg(pads_ge_48)] 'F' 7 : 0]
        SCL: ['A' 0 : 4, #[cfg(pads_ge_lqfp32)] 'A' 11 : 5, #[cfg(pads_ge_48)] 'B' 10 : 1, #[cfg(pads_ge_48)] 'B' 13 : 5, #[cfg(pads_ge_48)] 'F' 6 : 0]
);

/// Writes the bus timing to `CTL1`, `CKCFG` and `RT`, leaving the peripheral enabled.
///
/// `I2CCLK` takes `pclk1` in whole megahertz: `RISETIME` and the analog filter
/// are measured in real time, so the absolute frequency is needed, not a divider.
/// `CKCFG` and `I2CCLK` are only taken while `I2CEN` is clear, so the peripheral
/// is disabled for the duration.
///
/// # Panics
///
/// If `pclk1` is below the minimum the mode needs (2 / 8 / 24 MHz), or if the
/// requested frequency puts `CLKC` below what the hardware honours.
fn apply_config<I2C: Instance>(i2c: &I2C, mode: I2cMode, pclk1: Hertz) {
    let pclk1 = pclk1.to_Hz();
    let pclk1_mhz = pclk1 / 1_000_000;
    // t_rise / T_pclk1 + 1, and `1 ns × pclk1` is exactly `pclk1` in MHz.
    let risetime = |t_rise_ns: u32| (pclk1_mhz * t_rise_ns / 1_000 + 1) as u8;

    i2c.ctl0().modify(|_, w| w.i2cen().disabled());
    i2c.ctl1()
        .modify(|_, w| unsafe { w.i2cclk().bits(pclk1_mhz as u8) });

    match mode {
        // Both halves of the period are `CLKC` cycles long, so the period is
        // twice that.
        I2cMode::Standard { frequency } => {
            assert!(
                pclk1 >= MIN_PCLK1_STANDARD_HZ,
                "I2C standard mode needs pclk1 of at least 2 MHz"
            );
            let clkc = pclk1 / (2 * frequency.to_Hz());
            assert!(
                clkc >= CLKC_MIN_STANDARD,
                "I2C frequency too high for this pclk1"
            );
            i2c.rt()
                .write(|w| w.risetime().bits(risetime(RISE_TIME_STANDARD_NS)));
            i2c.ckcfg()
                .write(|w| w.fast().standard().clkc().bits(clkc as u16));
        }
        I2cMode::Fast {
            frequency,
            duty_cycle,
        } => {
            assert!(
                pclk1 >= MIN_PCLK1_FAST_HZ,
                "I2C fast mode needs pclk1 of at least 8 MHz"
            );
            i2c.rt()
                .write(|w| w.risetime().bits(risetime(RISE_TIME_FAST_NS)));
            write_fast_ckcfg(i2c, pclk1, frequency, duty_cycle);
        }
        I2cMode::FastPlus {
            frequency,
            duty_cycle,
        } => {
            assert!(
                pclk1 >= MIN_PCLK1_FAST_PLUS_HZ,
                "I2C fast mode plus needs pclk1 of at least 24 MHz"
            );
            i2c.rt()
                .write(|w| w.risetime().bits(risetime(RISE_TIME_FAST_PLUS_NS)));
            write_fast_ckcfg(i2c, pclk1, frequency, duty_cycle);
            i2c.fmpcfg().write(|w| w.fmpen().set_bit());
        }
    }

    i2c.ctl0().modify(|_, w| w.i2cen().enabled());
}

/// Writes `CKCFG` for the two fast modes, which share their formulas.
///
/// The duty cycle splits the period into `2 + 1` or `16 + 9` parts of `CLKC`
/// cycles each.
fn write_fast_ckcfg<I2C: Instance>(i2c: &I2C, pclk1: u32, frequency: Hertz, duty_cycle: DutyCycle) {
    let parts = match duty_cycle {
        DutyCycle::Ratio2to1 => 3,
        DutyCycle::Ratio16to9 => 25,
    };
    let clkc = pclk1 / (parts * frequency.to_Hz());
    assert!(
        clkc >= CLKC_MIN_FAST,
        "I2C frequency too high for this pclk1"
    );
    i2c.ckcfg().write(|w| {
        let w = match duty_cycle {
            DutyCycle::Ratio2to1 => w.dtcy().duty2(),
            DutyCycle::Ratio16to9 => w.dtcy().duty16_9(),
        };
        w.fast().fast().clkc().bits(clkc as u16)
    });
}

/// Ratio of the low to the high half of an SCL period (`DTCY`).
///
/// Only the fast modes can shape it, so it is a field of their variants alone.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

/// Bus speed, and whatever else that speed implies.
///
/// Each variant has its own minimum `pclk1`: 2 / 8 / 24 MHz.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cMode {
    /// Up to 100 kHz, duty cycle fixed at 1:1.
    Standard {
        /// SCL frequency.
        frequency: Hertz,
    },
    /// Up to 400 kHz.
    Fast {
        /// SCL frequency.
        frequency: Hertz,
        /// Shape of the SCL period.
        duty_cycle: DutyCycle,
    },
    /// Up to 1 MHz; also enables the stronger line driver (`FMPCFG`).
    FastPlus {
        /// SCL frequency.
        frequency: Hertz,
        /// Shape of the SCL period.
        duty_cycle: DutyCycle,
    },
}

impl I2cMode {
    /// Standard mode at `frequency`.
    pub const fn standard(frequency: Hertz) -> Self {
        Self::Standard { frequency }
    }
    /// Fast mode at `frequency`, with the given duty cycle.
    pub const fn fast(frequency: Hertz, duty_cycle: DutyCycle) -> Self {
        Self::Fast {
            frequency,
            duty_cycle,
        }
    }
    /// Fast mode plus at `frequency`, with the given duty cycle.
    pub const fn fast_plus(frequency: Hertz, duty_cycle: DutyCycle) -> Self {
        Self::FastPlus {
            frequency,
            duty_cycle,
        }
    }
}

/// An error the peripheral flagged in `STAT0` (manual Table 17-3).
///
/// Its own type rather than [`ErrorKind`], which has no variant for a PEC
/// mismatch or an SMBus alert; [`kind`] gives the portable classification.
///
/// [`kind`]: embedded_hal::i2c::Error::kind
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A START or STOP arrived where the protocol allows none (`BERR`).
    Bus,
    /// Another master won the bus; this one fell back to slave mode (`LOSTARB`).
    ArbitrationLoss,
    /// A byte was overwritten before being read, or needed before being written
    /// (`OUERR`). Slave mode only, with clock stretching disabled.
    Overrun,
    /// Nobody acknowledged (`AERR`) — the ordinary answer from an empty address,
    /// which is what bus scans rely on.
    ///
    /// One flag covers both cases, so the source comes from the step that ran.
    NoAcknowledge(NoAcknowledgeSource),
    /// The received PEC does not match the one computed locally (`PECERR`).
    Pec,
    /// An SMBus transaction exceeded its timeout (`SMBTO`).
    SmbusTimeout,
    /// An SMBus device pulled the alert line (`SMBALT`).
    SmbusAlert,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Bus => ErrorKind::Bus,
            Self::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            Self::Overrun => ErrorKind::Overrun,
            Self::NoAcknowledge(source) => ErrorKind::NoAcknowledge(*source),
            // Nothing upstream describes these three.
            Self::Pec | Self::SmbusTimeout | Self::SmbusAlert => ErrorKind::Other,
        }
    }
}

/// An I²C event that can raise an interrupt.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// The phase events `SBSEND`, `ADDSEND`, `ADD10SEND`, `STPDET` and `BTC`
    /// (`EVIE`). Each is cleared by the step of the transaction that acts on it.
    Protocol,
    /// The byte events `TBE` and `RBNE` (`BUFIE`), on top of the phase ones.
    ///
    /// Listening raises `EVIE` as well, `BUFIE` doing nothing without it in
    /// hardware. `TBE` is true whenever nothing is queued, so a handler feeding
    /// a buffer must `unlisten` this once the last byte is out — which leaves
    /// the phase events standing, and `BTC` then ends the transaction.
    Byte,
    /// Any of the `STAT0` error flags, which share one enable (`ERRIE`).
    /// Cleared by [`take_error`](I2c::take_error), which a handler must call —
    /// nothing else drains them.
    Error,
}

/// A configured I²C master, owning the peripheral and its two pins.
///
/// Both lines are open-drain and pulled up externally. There is no chip select:
/// the target is named by the first byte of every transaction.
pub struct I2c<I2CX, SDA, SCL> {
    i2c: I2CX,
    sda_pin: SDA,
    scl_pin: SCL,
}

impl<I2CX, SDA, SCL> I2c<I2CX, SDA, SCL>
where
    I2CX: Instance,
    SDA: SdaPin<I2CX>,
    SCL: SclPin<I2CX>,
{
    /// Enables the peripheral's clock, resets it and applies `mode`.
    ///
    /// The pins must already be open-drain in this I²C's alternate function; the
    /// bounds reject anything else at compile time. [`release`](Self::release)
    /// hands them back. `I2CCLK` comes from `pclk1` in the frozen clocks.
    ///
    /// # Panics
    ///
    /// If `pclk1` is too slow for `mode` (2 / 8 / 24 MHz), or if `mode`'s
    /// frequency is unreachable from it.
    pub fn new(rcu: &mut Rcu, i2c: I2CX, sda_pin: SDA, scl_pin: SCL, mode: I2cMode) -> Self {
        let pclk1 = rcu.clocks().pclk1();
        I2CX::enable(rcu);
        I2CX::reset(rcu);

        apply_config(&i2c, mode, pclk1);
        Self {
            i2c,
            sda_pin,
            scl_pin,
        }
    }
    /// Returns the peripheral and both pins.
    ///
    /// The clock is left enabled and no reset is performed — a later `new()`
    /// does both anyway.
    pub fn release(self) -> (I2CX, SDA, SCL) {
        (self.i2c, self.sda_pin, self.scl_pin)
    }
}

impl<I2CX, SDA, SCL> I2c<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    /// Whether the bus is in use, by another master or by a transaction of this
    /// one (`I2CBSY`).
    pub fn is_busy(&self) -> bool {
        self.i2c.stat1().read().i2cbsy().is_busy()
    }
    /// START has gone out and the bus is waiting for the address (`SBSEND`).
    pub fn sbsend(&self) -> bool {
        self.i2c.stat0().read().sbsend().bit_is_set()
    }
    /// The addressed device answered; the data phase opens on
    /// [`clear_addsend`](Self::clear_addsend) (`ADDSEND`).
    pub fn addsend(&self) -> bool {
        self.i2c.stat0().read().addsend().bit_is_set()
    }
    /// `DATA` will take the next byte (`TBE`).
    ///
    /// True whenever nothing is queued, so a handler listening for it must stop
    /// once it has nothing left to send.
    pub fn tbe(&self) -> bool {
        self.i2c.stat0().read().tbe().bit_is_set()
    }
    /// The byte behind the one in `DATA` has arrived whole, and the peripheral
    /// holds SCL until software catches up (`BTC`).
    pub fn btc(&self) -> bool {
        self.i2c.stat0().read().btc().bit_is_set()
    }
    /// A received byte is waiting in `DATA` (`RBNE`).
    pub fn rbne(&self) -> bool {
        self.i2c.stat0().read().rbne().bit_is_set()
    }
    /// Blocks until the bus is free.
    ///
    /// Only the first START needs this: `I2CBSY` stays set while this master owns
    /// the bus, so waiting again before a repeated START would wait forever.
    pub fn wait_idle(&self) {
        while self.is_busy() {}
    }

    /// Raises START, which goes out as soon as the bus is free.
    ///
    /// On a bus this master already holds it is a repeated START, and the
    /// transaction continues without a STOP in between.
    pub fn write_start(&mut self) {
        self.i2c.ctl0().modify(|_, w| w.start().start());
    }
    /// Puts the address into `DATA` with `read` as its direction bit.
    ///
    /// Also the second half of clearing `SBSEND`, the first being the read of
    /// `STAT0` that observed it.
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits.
    pub fn write_addr(&mut self, address: u8, read: bool) {
        assert!(address <= 0x7F, "I2C address must be 7-bit");
        self.i2c
            .data()
            .write(|w| w.trb().bits(address << 1 | read as u8));
    }
    /// Puts one byte into `DATA`, which also clears `TBE`.
    pub fn write_byte(&mut self, byte: u8) {
        self.i2c.data().write(|w| w.trb().bits(byte));
    }
    /// Takes the received byte out of `DATA`, which also clears `RBNE`.
    pub fn read_byte(&mut self) -> u8 {
        self.i2c.data().read().trb().bits()
    }
    /// Releases the bus with a STOP condition.
    ///
    /// Every path out of a transaction goes through here, failures included: a
    /// skipped STOP leaves the master holding SCL, and the bus never goes idle
    /// again.
    pub fn write_stop(&mut self) {
        self.i2c.ctl0().modify(|_, w| w.stop().stop());
    }
    /// Clears `ADDSEND`, which releases the bus into the data phase.
    ///
    /// Goes down on a read of `STAT0` then a read of `STAT1`, in that order;
    /// skipping either leaves the transfer stalled.
    pub fn clear_addsend(&mut self) {
        self.i2c.stat0().read();
        self.i2c.stat1().read();
    }
    /// Acknowledges or refuses the byte currently on the wire (`ACKEN`).
    ///
    /// It governs the byte already being clocked in, so the NAK that ends a read
    /// has to stand one byte early — before `ADDSEND` is cleared on a one-byte
    /// read, and before the last `BTC` otherwise.
    pub fn set_acken(&mut self, ack: bool) {
        self.i2c.ctl0().modify(|_, w| w.acken().bit(ack));
    }
    /// Moves the acknowledge one byte further along (`POAP`), for the two-byte
    /// read where both bytes must be taken after the NAK is already placed.
    ///
    /// Like `ACKEN` it must stand before the START that begins the read.
    pub fn set_poap(&mut self, next: bool) {
        self.i2c.ctl0().modify(|_, w| w.poap().bit(next));
    }

    /// Sends START and the address, then waits for the device to answer.
    ///
    /// On a bus this master already holds, the START is a repeated one — hence no
    /// [`wait_idle`](Self::wait_idle) here. Returns with `ADDSEND` still raised:
    /// a short read has to write `ACKEN` before
    /// [`clear_addsend`](Self::clear_addsend) opens the data phase. Failures
    /// release the bus, so callers can propagate with `?`.
    fn start(&mut self, address: u8, read: bool) -> Result<(), Error> {
        self.write_start();

        while self.i2c.stat0().read().sbsend().is_no_start() {
            if let Some(err) = self.take_error_from(NoAcknowledgeSource::Unknown) {
                self.write_stop();
                return Err(err);
            }
        }
        self.write_addr(address, read);

        while self.i2c.stat0().read().addsend().is_not_match() {
            if let Some(err) = self.take_error_from(NoAcknowledgeSource::Address) {
                self.write_stop();
                return Err(err);
            }
        }
        Ok(())
    }
    /// Waits for a received byte to reach `DATA`, releasing the bus on error.
    fn wait_rbne(&mut self) -> Result<(), Error> {
        while self.i2c.stat0().read().rbne().is_empty() {
            if let Some(err) = self.take_error_from(NoAcknowledgeSource::Data) {
                self.write_stop();
                return Err(err);
            }
        }
        Ok(())
    }
    /// Waits for the byte in the shift register to finish, releasing the bus on
    /// error.
    ///
    /// While receiving, `BTC` means a byte arrived behind the one still unread in
    /// `DATA`, and the peripheral holds SCL low until software catches up.
    fn wait_btc(&mut self) -> Result<(), Error> {
        while self.i2c.stat0().read().btc().is_not_finished() {
            if let Some(err) = self.take_error_from(NoAcknowledgeSource::Data) {
                self.write_stop();
                return Err(err);
            }
        }
        Ok(())
    }

    /// Returns the first pending error, if any, clearing its flag.
    ///
    /// `source` says which step is running: one `AERR` covers both a dead address
    /// and an unacknowledged data byte.
    ///
    /// `STAT0` flags are `rc_w0`, so clearing goes through `modify` — a `write`
    /// would zero the flags it does not name and wipe them all.
    fn take_error_from(&mut self, source: NoAcknowledgeSource) -> Option<Error> {
        let stat = self.i2c.stat0().read();
        if stat.berr().is_error() {
            self.i2c.stat0().modify(|_, w| w.berr().no_error());
            Some(Error::Bus)
        } else if stat.lostarb().is_lost() {
            self.i2c.stat0().modify(|_, w| w.lostarb().no_lost());
            Some(Error::ArbitrationLoss)
        } else if stat.aerr().is_error() {
            self.i2c.stat0().modify(|_, w| w.aerr().no_error());
            Some(Error::NoAcknowledge(source))
        } else if stat.ouerr().is_overrun() {
            self.i2c.stat0().modify(|_, w| w.ouerr().no_overrun());
            Some(Error::Overrun)
        } else if stat.pecerr().is_error() {
            self.i2c.stat0().modify(|_, w| w.pecerr().no_error());
            Some(Error::Pec)
        } else if stat.smbto().is_timeout() {
            self.i2c.stat0().modify(|_, w| w.smbto().no_timeout());
            Some(Error::SmbusTimeout)
        } else if stat.smbalt().is_alert() {
            self.i2c.stat0().modify(|_, w| w.smbalt().no_alert());
            Some(Error::SmbusAlert)
        } else {
            None
        }
    }

    /// Writes `bytes` to the device at `address`, framed by START and STOP.
    ///
    /// `address` is the plain 7-bit value from the datasheet; the direction bit
    /// is appended here. Blocks until the last byte is out and acknowledged, so
    /// the bus is idle on return. An unanswered address is
    /// [`Error::NoAcknowledge`] with [`NoAcknowledgeSource::Address`], which is
    /// how a bus scan finds devices.
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits (10-bit addressing is not
    /// supported). Checked before the bus is touched.
    pub fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        assert!(address <= 0x7F, "I2C address must be 7-bit");

        self.wait_idle();
        self.write_phase(address, bytes)?;
        self.write_stop();
        Ok(())
    }

    /// Addresses the device for writing and clocks `bytes` out, leaving the bus
    /// held: [`write`](Self::write) ends it with STOP,
    /// [`write_read`](Self::write_read) with a repeated START.
    fn write_phase(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        self.start(address, false)?;
        self.clear_addsend();
        self.write_bytes(bytes)?;

        // With nothing sent there is no transfer to complete, and BTC never comes.
        if !bytes.is_empty() {
            self.wait_btc()?;
        }
        Ok(())
    }

    /// Clocks `bytes` out of an already addressed device.
    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for byte in bytes {
            while self.i2c.stat0().read().tbe().is_not_empty() {
                if let Some(err) = self.take_error_from(NoAcknowledgeSource::Data) {
                    self.write_stop();
                    return Err(err);
                }
            }
            self.i2c.data().write(|w| w.trb().bits(*byte));
        }
        Ok(())
    }
    /// Reads `bytes.len()` bytes from the device at `address`, framed by START
    /// and STOP.
    ///
    /// A read ends by withholding the acknowledge of the last byte, and `ACKEN`
    /// governs the byte already on the wire — one ahead of the one being read.
    /// Hence separate sequences for one and two bytes; longer reads leave a byte
    /// unread so the peripheral stretches SCL while `ACKEN` is cleared (the
    /// manual's "Solution B" — "Solution A" has to react within the last byte's
    /// transfer, which an interrupt can break).
    ///
    /// An empty `bytes` is a no-op: a zero-byte transfer has no acknowledge to
    /// end on.
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits.
    pub fn read(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), Error> {
        assert!(address <= 0x7F, "I2C address must be 7-bit");
        if bytes.is_empty() {
            return Ok(());
        }

        self.wait_idle();
        self.read_phase(address, bytes)
    }

    /// Addresses the device for reading and fills `bytes`, ending with STOP.
    fn read_phase(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), Error> {
        let n = bytes.len();
        self.read_slots(address, n, bytes.iter_mut())
    }

    /// The read phase over `n` byte slots, wherever they live: [`read`](Self::read)
    /// passes one slice, [`transaction`] spreads one run of received bytes over
    /// several buffers. `n` has to be known upfront — it decides where the NAK
    /// falls.
    ///
    /// [`transaction`]: embedded_hal::i2c::I2c::transaction
    fn read_slots<'a>(
        &mut self,
        address: u8,
        n: usize,
        mut slots: impl Iterator<Item = &'a mut u8>,
    ) -> Result<(), Error> {
        // Every `next()` below is matched against a slot counted in `n`.
        match n {
            0 => Ok(()),
            1 => {
                self.start(address, true)?;
                // The byte starts arriving the moment ADDSEND goes down, so the
                // NAK has to stand before that.
                self.i2c.ctl0().modify(|_, w| w.acken().nak());
                self.clear_addsend();
                self.write_stop();

                self.wait_rbne()?;
                if let Some(slot) = slots.next() {
                    *slot = self.read_byte();
                }
                Ok(())
            }
            2 => {
                // POAP moves the acknowledge one byte along, so the NAK lands on
                // the second byte. Must be set before START.
                self.i2c.ctl0().modify(|_, w| w.poap().next());
                let result = self.read_two(address, slots);
                self.i2c.ctl0().modify(|_, w| w.poap().current());
                result
            }
            _ => {
                self.i2c.ctl0().modify(|_, w| w.acken().ack());
                self.start(address, true)?;
                self.clear_addsend();

                // Up to the last three bytes: take each one as it lands.
                for _ in 0..n - 3 {
                    self.wait_rbne()?;
                    if let Some(slot) = slots.next() {
                        *slot = self.read_byte();
                    }
                }

                // Byte N-2 is left in DATA on purpose: once N-1 arrives behind it
                // the peripheral holds SCL, so this ACKEN cannot be late.
                self.wait_btc()?;
                self.i2c.ctl0().modify(|_, w| w.acken().nak());
                if let Some(slot) = slots.next() {
                    *slot = self.read_byte();
                }

                // Reading N-2 let the last byte in; SCL is stretched again, and
                // STOP goes out before it is read.
                self.wait_btc()?;
                self.write_stop();
                for slot in slots {
                    *slot = self.read_byte();
                }
                Ok(())
            }
        }
    }

    /// The two-byte read, split out so `POAP` is restored on every path.
    fn read_two<'a>(
        &mut self,
        address: u8,
        slots: impl Iterator<Item = &'a mut u8>,
    ) -> Result<(), Error> {
        self.start(address, true)?;
        self.i2c.ctl0().modify(|_, w| w.acken().nak());
        self.clear_addsend();

        // Neither byte is read until both have arrived, and STOP goes out before
        // either leaves DATA.
        self.wait_btc()?;
        self.write_stop();
        for slot in slots {
            *slot = self.read_byte();
        }
        Ok(())
    }

    /// Writes `write`, then reads into `read`, joined by a repeated START.
    ///
    /// This is how a register-addressed device is read: the write phase moves its
    /// pointer, the read phase takes what the pointer names. A
    /// [`write`](Self::write) then a [`read`](Self::read) is a different
    /// operation — the STOP between them lets another master move the pointer,
    /// and the wrong register reads back as valid data.
    ///
    /// An empty `read` degenerates to [`write`](Self::write), an empty `write`
    /// to [`read`](Self::read).
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits.
    pub fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        assert!(address <= 0x7F, "I2C address must be 7-bit");

        self.wait_idle();
        self.write_phase(address, write)?;

        if read.is_empty() {
            self.write_stop();
            return Ok(());
        }
        // No STOP in between: the read phase's START is a repeated one.
        self.read_phase(address, read)
    }

    /// Returns the pending error, if any, and clears it.
    ///
    /// The acknowledge for [`Event::Error`]: the flag is what holds the
    /// interrupt request up, so a handler that skips this re-enters forever.
    /// One error per call — with two pending, the second survives for the next.
    /// A NACK reports [`NoAcknowledgeSource::Unknown`], the phase it arrived in
    /// being known only to whoever drives the transaction.
    pub fn take_error(&mut self) -> Option<Error> {
        self.take_error_from(NoAcknowledgeSource::Unknown)
    }

    /// Raises an interrupt on `event`, which still has to be unmasked in the
    /// NVIC.
    ///
    /// [`Event::Byte`] raises `EVIE` along with `BUFIE`, so it covers
    /// [`Event::Protocol`] too.
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Protocol => self.i2c.ctl1().modify(|_, w| w.evie().enabled()),
            Event::Byte => self
                .i2c
                .ctl1()
                .modify(|_, w| w.evie().enabled().bufie().enabled()),
            Event::Error => self.i2c.ctl1().modify(|_, w| w.errie().enabled()),
        }
    }
    /// Stops raising an interrupt on `event`.
    ///
    /// [`Event::Protocol`] takes `BUFIE` down with it: byte events cannot stand
    /// on their own in hardware. Dropping only those is
    /// `unlisten(Event::Byte)`.
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Protocol => self
                .i2c
                .ctl1()
                .modify(|_, w| w.evie().disabled().bufie().disabled()),
            Event::Byte => self.i2c.ctl1().modify(|_, w| w.bufie().disabled()),
            Event::Error => self.i2c.ctl1().modify(|_, w| w.errie().disabled()),
        }
    }
    /// Whether `event` is being listened for.
    pub fn is_listening(&self, event: Event) -> bool {
        let ctl1 = self.i2c.ctl1().read();
        match event {
            Event::Protocol => ctl1.evie().is_enabled(),
            Event::Byte => ctl1.evie().is_enabled() && ctl1.bufie().is_enabled(),
            Event::Error => ctl1.errie().is_enabled(),
        }
    }

    /// Starts a write of `buf` to `address`, to be driven by interrupts.
    ///
    /// Takes the peripheral with it; both come back from
    /// [`release`](WriteTransfer::release). An empty `buf` still addresses the
    /// device, which is how a bus scan probes for one. Blocks only until the bus
    /// is free, then raises START and returns.
    ///
    /// Arms the interrupts itself and keeps re-arming them as the phases change,
    /// so `listen` beforehand is neither needed nor respected. A handler written
    /// by hand out of the primitives arms its own instead.
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits.
    pub fn start_write(mut self, address: u8, buf: &'static [u8]) -> WriteTransfer<I2CX, SDA, SCL> {
        assert!(address <= 0x7F, "I2C address must be 7-bit");
        self.wait_idle();
        self.listen(Event::Error);
        self.listen(Event::Byte);
        self.write_start();
        WriteTransfer {
            i2c: self,
            address: address << 1,
            buf,
            pos: 0,
            state: WriteState::Sbsend,
        }
    }
    /// Starts a read of `buf.len()` bytes from `address`, to be driven by
    /// interrupts.
    ///
    /// Takes the peripheral with it; both come back from
    /// [`release`](ReadTransfer::release). An empty `buf` finishes immediately
    /// without touching the bus — a zero-byte read has no acknowledge to end on.
    /// Blocks only until the bus is free, then raises START and returns.
    ///
    /// Arms the interrupts itself and keeps re-arming them as the phases change,
    /// so `listen` beforehand is neither needed nor respected. A handler written
    /// by hand out of the primitives arms its own instead.
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits.
    pub fn start_read(
        mut self,
        address: u8,
        buf: &'static mut [u8],
    ) -> ReadTransfer<I2CX, SDA, SCL> {
        assert!(address <= 0x7F, "I2C address must be 7-bit");
        if buf.is_empty() {
            return ReadTransfer {
                i2c: self,
                address: address << 1 | 1,
                buf,
                pos: 0,
                state: ReadState::Done(Ok(())),
            };
        }

        self.wait_idle();
        // Both of these must stand before START: ACKEN governs the byte already
        // on the wire, and POAP moves it one further along.
        self.set_acken(buf.len() > 2);
        self.set_poap(buf.len() == 2);
        self.listen(Event::Error);
        self.listen(Event::Byte);
        self.write_start();
        ReadTransfer {
            i2c: self,
            address: address << 1 | 1,
            buf,
            pos: 0,
            state: ReadState::Sbsend,
        }
    }
}

/// Where a [`WriteTransfer`] stands: the flag it is waiting for.
#[derive(Clone, Copy)]
enum WriteState {
    /// START is out; `SBSEND` calls for the address.
    Sbsend,
    /// The address is out; `ADDSEND` opens the data phase.
    Addsend,
    /// Feeding `DATA`, one byte per `TBE`.
    Sending,
    /// Everything is fed; `BTC` says the wire is clear for STOP.
    Btc,
    /// Over, carrying the outcome the handler reached.
    Done(Result<(), Error>),
}

/// A write driven by interrupts, owning the peripheral and the buffer.
///
/// Built by [`I2c::start_write`]. The handler for both I²C vectors calls
/// [`on_interrupt`](Self::on_interrupt); the transfer is over once
/// [`is_done`](Self::is_done) says so, and [`release`](Self::release) takes it
/// apart.
pub struct WriteTransfer<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    i2c: I2c<I2CX, SDA, SCL>,
    address: u8,
    buf: &'static [u8],
    pos: usize,
    state: WriteState,
}

impl<I2CX, SDA, SCL> WriteTransfer<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    /// Advances the transfer by one step. Call from both I²C interrupt
    /// handlers.
    ///
    /// Reads the flags itself rather than trusting which vector it was entered
    /// from, so a wakeup this step has nothing to do with is simply ignored.
    pub fn on_interrupt(&mut self) {
        if self.is_done() {
            return;
        }
        if let Some(err) = self.i2c.take_error() {
            self.i2c.write_stop();
            self.finish(Err(err));
            return;
        }

        match self.state {
            WriteState::Sbsend if self.i2c.sbsend() => {
                self.i2c.write_byte(self.address);
                self.state = WriteState::Addsend;
            }
            WriteState::Addsend if self.i2c.addsend() => {
                self.i2c.clear_addsend();
                if self.buf.is_empty() {
                    // Addressing was the whole transfer; BTC never comes.
                    self.i2c.write_stop();
                    self.finish(Ok(()));
                } else {
                    self.state = WriteState::Sending;
                }
            }
            WriteState::Sending if self.i2c.tbe() => {
                self.i2c.write_byte(self.buf[self.pos]);
                self.pos += 1;
                if self.pos == self.buf.len() {
                    // TBE stays up with nothing left to feed, so stop listening
                    // for it and wait out the byte in the shift register.
                    self.i2c.unlisten(Event::Byte);
                    self.state = WriteState::Btc;
                }
            }
            WriteState::Btc if self.i2c.btc() => {
                self.i2c.write_stop();
                self.finish(Ok(()));
            }
            _ => {}
        }
    }
    /// Whether the transfer has ended, successfully or not.
    pub fn is_done(&self) -> bool {
        matches!(self.state, WriteState::Done(_))
    }
    /// Gives back the peripheral, the buffer and the outcome.
    ///
    /// The outcome rides alongside instead of wrapping the pair: a failed
    /// transfer still has to hand the peripheral back, or it is lost for good.
    /// `None` means the transfer was taken apart while still running, in which
    /// case the bus is released with a STOP first.
    pub fn release(
        self,
    ) -> (
        I2c<I2CX, SDA, SCL>,
        &'static [u8],
        Option<Result<(), Error>>,
    ) {
        let mut this = ManuallyDrop::new(self);
        let outcome = match this.state {
            WriteState::Done(result) => Some(result),
            _ => {
                this.abort();
                None
            }
        };
        // Safe: `this` is never dropped, so the fields are read exactly once.
        let i2c = unsafe { ptr::read(&this.i2c) };
        (i2c, this.buf, outcome)
    }
    /// Stops every interrupt and records the outcome. The bus is released by
    /// the caller, which knows whether a STOP has gone out already.
    fn finish(&mut self, result: Result<(), Error>) {
        self.i2c.unlisten(Event::Protocol);
        self.i2c.unlisten(Event::Error);
        self.state = WriteState::Done(result);
    }
    /// Cuts a running transfer short, leaving the bus idle.
    fn abort(&mut self) {
        self.i2c.write_stop();
        self.finish(Ok(()));
    }
}

impl<I2CX, SDA, SCL> Drop for WriteTransfer<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    fn drop(&mut self) {
        if !self.is_done() {
            self.abort();
        }
    }
}

/// Where a [`ReadTransfer`] stands: the flag it is waiting for.
///
/// The tail states exist because `ACKEN` governs the byte already on the wire,
/// one ahead of the one being read — the manual's "Solution B".
#[derive(Clone, Copy)]
enum ReadState {
    /// START is out; `SBSEND` calls for the address.
    Sbsend,
    /// The address is out; `ADDSEND` opens the data phase.
    Addsend,
    /// Three or more bytes to go: take each one as it lands on `RBNE`.
    Receiving,
    /// Three left. `BTC` means two of them are in, so the NAK cannot be late.
    Penultimate,
    /// Two left, both already arrived: STOP goes out before either is read.
    Tail,
    /// The one-byte read: its NAK stands, STOP is out, `RBNE` is all that's
    /// left.
    Single,
    /// Over, carrying the outcome the handler reached.
    Done(Result<(), Error>),
}

/// A read driven by interrupts, owning the peripheral and the buffer.
///
/// Built by [`I2c::start_read`]. The handler for both I²C vectors calls
/// [`on_interrupt`](Self::on_interrupt); the transfer is over once
/// [`is_done`](Self::is_done) says so, and [`release`](Self::release) takes it
/// apart.
pub struct ReadTransfer<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    i2c: I2c<I2CX, SDA, SCL>,
    address: u8,
    buf: &'static mut [u8],
    pos: usize,
    state: ReadState,
}

impl<I2CX, SDA, SCL> ReadTransfer<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    /// Advances the transfer by one step. Call from both I²C interrupt
    /// handlers.
    ///
    /// Reads the flags itself rather than trusting which vector it was entered
    /// from, so a wakeup this step has nothing to do with is simply ignored.
    pub fn on_interrupt(&mut self) {
        if self.is_done() {
            return;
        }
        if let Some(err) = self.i2c.take_error() {
            self.i2c.write_stop();
            self.finish(Err(err));
            return;
        }

        match self.state {
            ReadState::Sbsend if self.i2c.sbsend() => {
                self.i2c.write_byte(self.address);
                self.state = ReadState::Addsend;
            }
            ReadState::Addsend if self.i2c.addsend() => {
                self.state = match self.buf.len() {
                    1 => {
                        // The byte starts arriving the moment ADDSEND goes
                        // down, so the NAK has to stand before that.
                        self.i2c.set_acken(false);
                        self.i2c.clear_addsend();
                        self.i2c.write_stop();
                        ReadState::Single
                    }
                    2 => {
                        self.i2c.set_acken(false);
                        self.i2c.clear_addsend();
                        self.i2c.unlisten(Event::Byte);
                        ReadState::Tail
                    }
                    3 => {
                        // Nothing to take before the tail: the three bytes are
                        // exactly what the last two steps handle.
                        self.i2c.clear_addsend();
                        self.i2c.unlisten(Event::Byte);
                        ReadState::Penultimate
                    }
                    _ => {
                        self.i2c.clear_addsend();
                        ReadState::Receiving
                    }
                };
            }
            ReadState::Single if self.i2c.rbne() => {
                self.buf[0] = self.i2c.read_byte();
                self.pos = 1;
                self.finish(Ok(()));
            }
            ReadState::Receiving if self.i2c.rbne() => {
                self.buf[self.pos] = self.i2c.read_byte();
                self.pos += 1;
                if self.pos == self.buf.len() - 3 {
                    // From here on the steps wait for BTC, and RBNE would only
                    // wake the handler between them.
                    self.i2c.unlisten(Event::Byte);
                    self.state = ReadState::Penultimate;
                }
            }
            ReadState::Penultimate if self.i2c.btc() => {
                // Byte N-2 stayed in DATA on purpose: with N-1 behind it the
                // peripheral holds SCL, so this ACKEN cannot be late.
                self.i2c.set_acken(false);
                self.buf[self.pos] = self.i2c.read_byte();
                self.pos += 1;
                self.state = ReadState::Tail;
            }
            ReadState::Tail if self.i2c.btc() => {
                // Both remaining bytes have arrived; STOP goes out before
                // either leaves DATA.
                self.i2c.write_stop();
                while self.pos < self.buf.len() {
                    self.buf[self.pos] = self.i2c.read_byte();
                    self.pos += 1;
                }
                self.finish(Ok(()));
            }
            _ => {}
        }
    }
    /// Whether the transfer has ended, successfully or not.
    pub fn is_done(&self) -> bool {
        matches!(self.state, ReadState::Done(_))
    }
    /// Gives back the peripheral, the buffer and the outcome.
    ///
    /// The outcome rides alongside instead of wrapping the pair: a failed
    /// transfer still has to hand the peripheral back, or it is lost for good.
    /// `None` means the transfer was taken apart while still running, in which
    /// case the bus is released with a STOP first.
    pub fn release(
        self,
    ) -> (
        I2c<I2CX, SDA, SCL>,
        &'static mut [u8],
        Option<Result<(), Error>>,
    ) {
        let mut this = ManuallyDrop::new(self);
        let outcome = match this.state {
            ReadState::Done(result) => Some(result),
            _ => {
                this.abort();
                None
            }
        };
        // Safe: `this` is never dropped, so the fields are read exactly once.
        let i2c = unsafe { ptr::read(&this.i2c) };
        let buf = unsafe { ptr::read(&this.buf) };
        (i2c, buf, outcome)
    }
    /// Stops every interrupt, restores `POAP` and records the outcome. The bus
    /// is released by the caller, which knows whether a STOP has gone out
    /// already.
    fn finish(&mut self, result: Result<(), Error>) {
        self.i2c.set_poap(false);
        self.i2c.unlisten(Event::Protocol);
        self.i2c.unlisten(Event::Error);
        self.state = ReadState::Done(result);
    }
    /// Cuts a running transfer short, leaving the bus idle.
    fn abort(&mut self) {
        self.i2c.write_stop();
        self.finish(Ok(()));
    }
}

impl<I2CX, SDA, SCL> Drop for ReadTransfer<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    fn drop(&mut self) {
        if !self.is_done() {
            self.abort();
        }
    }
}

impl<I2CX, SDA, SCL> ErrorType for I2c<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    type Error = Error;
}

/// Whether two operations run in the same direction, and so belong to one
/// addressed phase.
fn same_direction(a: &Operation<'_>, b: &Operation<'_>) -> bool {
    matches!(
        (a, b),
        (Operation::Read(_), Operation::Read(_)) | (Operation::Write(_), Operation::Write(_))
    )
}

impl<I2CX, SDA, SCL> embedded_hal::i2c::I2c for I2c<I2CX, SDA, SCL>
where
    I2CX: Instance,
{
    /// Per the trait's contract: neighbouring operations of the same direction
    /// join into one addressed phase, a change of direction inserts a repeated
    /// START, and only the end carries a STOP.
    ///
    /// # Panics
    ///
    /// If `address` does not fit in seven bits, or if a `Read` is followed by
    /// further operations — ending a read early needs a repeated START in place
    /// of its STOP, which the read sequences here do not express. `[Write.., Read]`
    /// works.
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        assert!(address <= 0x7F, "I2C address must be 7-bit");
        if operations.is_empty() {
            return Ok(());
        }
        let phases = operations.chunk_by(same_direction).count();

        self.wait_idle();
        for (i, phase) in operations.chunk_by_mut(same_direction).enumerate() {
            let last = i + 1 == phases;
            match phase.first() {
                Some(Operation::Write(_)) => {
                    self.start(address, false)?;
                    self.clear_addsend();
                    let mut sent = false;
                    for op in phase.iter() {
                        if let Operation::Write(bytes) = op {
                            self.write_bytes(bytes)?;
                            sent |= !bytes.is_empty();
                        }
                    }
                    if sent {
                        self.wait_btc()?;
                    }
                    if last {
                        self.write_stop();
                    }
                }
                Some(Operation::Read(_)) => {
                    assert!(last, "I2C read must be the last operation of a transaction");
                    let n = phase
                        .iter()
                        .map(|op| match op {
                            Operation::Read(buf) => buf.len(),
                            Operation::Write(_) => 0,
                        })
                        .sum();
                    let slots = phase
                        .iter_mut()
                        .filter_map(|op| match op {
                            Operation::Read(buf) => Some(buf),
                            Operation::Write(_) => None,
                        })
                        .flat_map(|buf| buf.iter_mut());
                    self.read_slots(address, n, slots)?;
                }
                None => {}
            }
        }
        Ok(())
    }

    fn read(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), Error> {
        self.read(address, bytes)
    }
    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        self.write(address, bytes)
    }
    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        self.write_read(address, bytes, buffer)
    }
}
