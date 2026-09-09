//! Analog comparator.

use core::marker::PhantomData;

use crate::gpio::{Analog, Pin};
use crate::pac;
use crate::rcu::Rcu;

/// Hysteresis on the comparator output, suppressing chatter near the threshold.
///
/// Discriminants are the `CMPxHST` encoding.
#[allow(missing_docs)]
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hysteresis {
    None = 0b00,
    Low = 0b01,
    Medium = 0b10,
    High = 0b11,
}

/// A source the inverting input can be tied to.
///
/// `MSEL` is the `CMPxMSEL` encoding. Implemented by the four internal reference
/// taps and by the four pins the multiplexer reaches, each only in [`Analog`] —
/// the manual requires that mode before a pin is selected as an input.
///
/// `VREFINT` is 1.2 V, so the taps sit at 0.3, 0.6, 0.9 and 1.2 V.
pub trait InvertingInput {
    /// The `CMPxMSEL` encoding of this source.
    const MSEL: u8;
}

/// A quarter of the internal reference.
pub struct VrefintQuarter;
/// Half of the internal reference.
pub struct VrefintHalf;
/// Three quarters of the internal reference.
pub struct VrefintThreeQuarters;
/// The internal reference itself.
pub struct Vrefint;

impl InvertingInput for VrefintQuarter {
    const MSEL: u8 = 0b000;
}
impl InvertingInput for VrefintHalf {
    const MSEL: u8 = 0b001;
}
impl InvertingInput for VrefintThreeQuarters {
    const MSEL: u8 = 0b010;
}
impl InvertingInput for Vrefint {
    const MSEL: u8 = 0b011;
}
impl InvertingInput for Pin<'A', 4, Analog> {
    const MSEL: u8 = 0b100;
}
impl InvertingInput for Pin<'A', 5, Analog> {
    const MSEL: u8 = 0b101;
}
impl InvertingInput for Pin<'A', 0, Analog> {
    const MSEL: u8 = 0b110;
}
impl InvertingInput for Pin<'A', 2, Analog> {
    const MSEL: u8 = 0b111;
}

/// What the non-inverting input consists of.
///
/// `SW` is the `CMPxSW` state. `PA1` is the only pin wired to this input; handing
/// over `PA4` as well closes the switch and ties the two together, which is why
/// the pair is a separate implementor. Owning `PA4` here is what keeps it from
/// also serving as the [`InvertingInput`] and shorting both inputs together.
pub trait NonInvertingInput {
    /// Whether `CMPxSW` is closed for this source.
    const SW: bool;
}

impl NonInvertingInput for Pin<'A', 1, Analog> {
    const SW: bool = false;
}
impl NonInvertingInput for (Pin<'A', 1, Analog>, Pin<'A', 4, Analog>) {
    const SW: bool = true;
}

/// Where the comparator output is routed on top of the pin.
///
/// Discriminants are the `CMPxOSEL` encoding; `0b100` and `0b101` are reserved.
/// Enable the comparator before configuring the timer channel that captures it.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputSel {
    /// Nowhere.
    None = 0b000,
    /// TIMER0 break input.
    Timer0Break = 0b001,
    /// TIMER0 CH0 input capture.
    Timer0Ch0 = 0b010,
    /// TIMER0 `OCPRE_CLR` input.
    Timer0OcpreClr = 0b011,
    /// TIMER2 CH0 input capture.
    Timer2Ch0 = 0b110,
    /// TIMER2 `OCPRE_CLR` input.
    Timer2OcpreClr = 0b111,
}

/// Propagation delay traded against current draw.
///
/// Discriminants are the `CMPxM` encoding. Speed and power move together, so one
/// axis names the variant.
#[allow(missing_docs)]
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    High = 0b00,
    Medium = 0b01,
    Low = 0b10,
    VeryLow = 0b11,
}

/// Polarity of the comparator output.
///
/// Discriminants are the `CMPxPL` encoding. It affects the pin, EXTI and the
/// timer, but not [`CmpRunning::output`], which is read off the raw comparison.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// The output is high while the non-inverting input is the higher one.
    NotInverted = 0,
    /// The output is inverted.
    Inverted = 1,
}

/// Everything the comparator is set up with, apart from its inputs.
///
/// ```ignore
/// CmpConfig::new(Speed::High)
///     .hysteresis(Hysteresis::Medium)
///     .output_sel(OutputSel::Timer0Break)
/// ```
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CmpConfig {
    speed: Speed,
    hysteresis: Hysteresis,
    output_sel: OutputSel,
    polarity: Polarity,
}

impl CmpConfig {
    /// Creates a configuration with the given speed, no hysteresis, no output
    /// routing and a non-inverted output.
    ///
    /// Speed is a required argument and this type has no `Default`: it trades
    /// propagation delay against current draw, and neither choice is universal.
    pub const fn new(speed: Speed) -> Self {
        Self {
            speed,
            hysteresis: Hysteresis::None,
            output_sel: OutputSel::None,
            polarity: Polarity::NotInverted,
        }
    }
    /// Sets the hysteresis. Defaults to [`Hysteresis::None`].
    pub const fn hysteresis(mut self, hysteresis: Hysteresis) -> Self {
        self.hysteresis = hysteresis;
        self
    }
    /// Routes the output to a timer as well as the pin. Defaults to
    /// [`OutputSel::None`].
    pub const fn output_sel(mut self, output_sel: OutputSel) -> Self {
        self.output_sel = output_sel;
        self
    }
    /// Sets the output polarity. Defaults to [`Polarity::NotInverted`].
    pub const fn polarity(mut self, polarity: Polarity) -> Self {
        self.polarity = polarity;
        self
    }
}

/// Typestate: the control register still accepts writes.
pub struct Unlocked;
/// Typestate: the control register is frozen until the next system reset.
pub struct Locked;

/// The comparator, configured and stopped.
pub struct Cmp<POS, INV> {
    cmp: pac::Cmp,
    pos: POS,
    inv: INV,
}

/// The comparator, running.
///
/// Once `LOCKED` is [`Locked`] the control register is frozen for good: the
/// peripheral cannot be stopped and the pins cannot be taken back.
pub struct CmpRunning<POS, INV, LOCKED = Unlocked> {
    cmp: pac::Cmp,
    pos: POS,
    inv: INV,
    _locked: PhantomData<LOCKED>,
}

impl<POS: NonInvertingInput, INV: InvertingInput> Cmp<POS, INV> {
    /// Clocks the peripheral and applies `config`, leaving the comparator stopped.
    pub fn new(rcu: &mut Rcu, cmp: pac::Cmp, pos: POS, inv: INV, config: CmpConfig) -> Self {
        rcu.enable_cfgcmp();
        let mut cmp = Self { cmp, pos, inv };
        cmp.apply_config(config);
        cmp
    }
    /// Hands back the peripheral and both inputs. The clock stays on, since
    /// `CFGCMPEN` also feeds SYSCFG.
    pub fn release(self) -> (pac::Cmp, POS, INV) {
        (self.cmp, self.pos, self.inv)
    }

    /// Writes the whole control register, leaving `CMPxEN` clear.
    fn apply_config(&mut self, config: CmpConfig) {
        self.cmp.cs().modify(|_, w| {
            let w = w.cmphst().bits(config.hysteresis as u8);
            let w = w.cmposel().bits(config.output_sel as u8);
            let w = w.cmpm().bits(config.speed as u8);
            let w = w.cmppl().bit(config.polarity == Polarity::Inverted);
            let w = unsafe { w.cmpmsel().bits(INV::MSEL) };
            w.cmpsw().bit(POS::SW)
        });
    }

    /// Starts the comparator.
    pub fn enable(self) -> CmpRunning<POS, INV, Unlocked> {
        self.cmp.cs().modify(|_, w| w.cmpen().set_bit());
        CmpRunning {
            cmp: self.cmp,
            pos: self.pos,
            inv: self.inv,
            _locked: PhantomData,
        }
    }
}

impl<POS, INV> CmpRunning<POS, INV, Unlocked> {
    /// Stops the comparator, handing back the configured but idle peripheral.
    pub fn disable(self) -> Cmp<POS, INV> {
        self.cmp.cs().modify(|_, w| w.cmpen().clear_bit());
        Cmp {
            cmp: self.cmp,
            pos: self.pos,
            inv: self.inv,
        }
    }

    /// Freezes the control register until the next system reset.
    ///
    /// Nothing clears `CMPxLK`, so the comparator keeps running for good and the
    /// pins stay with it: the locked type has neither `disable` nor a way out.
    pub fn lock(self) -> CmpRunning<POS, INV, Locked> {
        self.cmp.cs().modify(|_, w| w.cmplk().set_bit());
        CmpRunning {
            cmp: self.cmp,
            pos: self.pos,
            inv: self.inv,
            _locked: PhantomData,
        }
    }
}

impl<POS, INV, LOCKED> CmpRunning<POS, INV, LOCKED> {
    /// Whether the non-inverting input is above the inverting one.
    ///
    /// `CMPxO` is taken before the polarity multiplexer, so [`Polarity`] does not
    /// show up here — only on the pin, EXTI and the timer.
    pub fn output(&self) -> bool {
        self.cmp.cs().read().cmpo().bit_is_set()
    }
}
