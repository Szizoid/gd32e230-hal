//! Reset and clock unit: the system clock tree and per-peripheral clock gating.
//!
//! [`RcuExt::constrain`] hands out an [`UnfrozenRcu`], whose only method applies a
//! [`ClockConfig`] and turns it into the [`Rcu`] every driver takes. Freezing
//! writes the registers once and consumes the unfrozen value, so the tree is
//! configured exactly once and the resulting [`Clocks`] are read-only afterwards.
//!
//! ```ignore
//! let mut fmc = dp.fmc.constrain();
//! let config = ClockConfig::default()
//!     .sysclk(SysClk::Pll(PllFreq::Mhz48))
//!     .adc_sel(AdcSel::Prescaled(AdcPsc::Apb2Div8));
//! let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);
//! let clocks = rcu.clocks();
//! ```
//!
//! Peripheral clocks are gated through the [`Enable`] trait, which drivers call
//! from their constructors, so a peripheral cannot be used unclocked. [`Reset`]
//! is separate because not every peripheral has a reset bit — DMA has none.
//!
//! `HXTAL` and `LXTAL` are not started — no crystal is fitted on the target board.

use crate::fmc::Fmc;
use crate::pac;
use crate::time::Hertz;

const IRC8M: u32 = 8_000_000;
const IRC28M: u32 = 28_000_000;
const LXTAL: u32 = 32_768;
const PLL_SRC: u32 = IRC8M / 2;
pub(crate) const IRC40K: u32 = 40_000;

const IRC28MDIV_DIV1: bool = true;
const IRC28MDIV_DIV2: bool = false;
const ADCSEL_IRC28M: bool = false;
const ADCSEL_PRESCALED: bool = true;
const ADCPSC_MSB_APB2: bool = false;
const ADCPSC_MSB_AHB: bool = true;

/// Target system clock produced by the PLL, in 4 MHz steps up to the 72 MHz limit.
///
/// Named by frequency rather than multiplier: the PLL source is fixed
/// (IRC8M/2 = 4 MHz), so the two map one-to-one. Only reachable frequencies
/// exist, so an impossible request is a compile error, not a silent rounding.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum PllFreq {
    Mhz8 = 8_000_000,
    Mhz12 = 12_000_000,
    Mhz16 = 16_000_000,
    Mhz20 = 20_000_000,
    Mhz24 = 24_000_000,
    Mhz28 = 28_000_000,
    Mhz32 = 32_000_000,
    Mhz36 = 36_000_000,
    Mhz40 = 40_000_000,
    Mhz44 = 44_000_000,
    Mhz48 = 48_000_000,
    Mhz52 = 52_000_000,
    Mhz56 = 56_000_000,
    Mhz60 = 60_000_000,
    Mhz64 = 64_000_000,
    Mhz68 = 68_000_000,
    Mhz72 = 72_000_000,
}

/// Source of the system clock.
///
/// [`Irc8m`](Self::Irc8m) means the reset state rather than a switch back to it:
/// [`freeze`](UnfrozenRcu::freeze) leaves `SCS` alone, because lowering the clock
/// after the flash wait states were already set for a higher one is exactly the
/// order that must not happen.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SysClk {
    /// IRC8M straight through at 8 MHz, PLL off.
    Irc8m,
    /// The PLL, at the frequency it is asked for.
    Pll(PllFreq),
}

/// AHB prescaler: divides the system clock down to `hclk`.
///
/// Named by the divider, not the resulting frequency, because `sysclk` varies
/// with configuration; every variant is legal at any `sysclk`.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum AhbPsc {
    Div1 = 1,
    Div2 = 2,
    Div4 = 4,
    Div8 = 8,
    Div16 = 16,
    Div64 = 64,
    Div128 = 128,
    Div256 = 256,
    Div512 = 512,
}

/// APB prescaler: divides `hclk` down to `pclk1` (APB1) or `pclk2` (APB2).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum ApbPsc {
    Div1 = 1,
    Div2 = 2,
    Div4 = 4,
    Div8 = 8,
    Div16 = 16,
}

/// Divider for the prescaled `CK_ADC` branch, including which bus it taps.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum AdcPsc {
    Apb2Div2,
    Apb2Div4,
    Apb2Div6,
    Apb2Div8,
    AhbDiv3,
    AhbDiv5,
    AhbDiv7,
    AhbDiv9,
}

/// Divider on the internal 28 MHz oscillator feeding `CK_ADC`.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum Irc28mDiv {
    Div1,
    Div2,
}

/// Source of the ADC clock.
///
/// Each branch carries its own divider inside the variant, so a divider can't be
/// specified for the branch it doesn't belong to.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcSel {
    /// Left as the reset state: IRC28M selected but not running, so `CK_ADC` is
    /// 0 Hz and [`Clocks::ck_adc`] reads zero.
    Off,
    /// The dedicated internal 28 MHz oscillator, which [`UnfrozenRcu::freeze`] starts.
    Irc28m(Irc28mDiv),
    /// A prescaled tap off APB2 or AHB.
    Prescaled(AdcPsc),
}

/// Source of the USART0 clock, independent of the APB2 bus clock.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Usart0Sel {
    /// The APB2 bus clock — the reset default.
    Apb2,
    /// The system clock, unaffected by the bus prescalers.
    Sysclk,
    /// The 32.768 kHz crystal.
    ///
    /// Selecting this without starting `LXTAL` leaves USART0 unclocked, and its
    /// blocking reads and writes will never return. The HAL cannot know what is
    /// fitted on a given board, so this is left to the caller.
    Lxtal,
    /// The internal 8 MHz RC oscillator, independent of the system clock.
    Irc8m,
}

/// Frozen clock frequencies, produced by [`UnfrozenRcu::freeze`].
///
/// Passed by value into the drivers that need it (USART for its baud divisor,
/// ADC for its calibration delay). There are no setters — once frozen, the tree
/// matches what was actually written to the registers.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    sysclk: Hertz,
    usart0: Hertz,
    ck_adc: Hertz,
    pclk1_tim: Hertz,
    pclk2_tim: Hertz,
}

impl Clocks {
    /// AHB clock, which also clocks the core.
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }
    /// APB1 bus clock.
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }
    /// APB2 bus clock.
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }
    /// System clock, before the AHB prescaler.
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }
    /// Clock actually feeding USART0, per [`Usart0Sel`].
    pub fn usart0(&self) -> Hertz {
        self.usart0
    }
    /// Clock feeding the ADC. Zero while [`AdcSel::Off`] stands.
    pub fn ck_adc(&self) -> Hertz {
        self.ck_adc
    }
    /// Clock feeding the timers on APB1 (TIMER2, TIMER5, TIMER13).
    ///
    /// Equal to [`pclk1`](Self::pclk1) only when the APB1 prescaler is
    /// [`ApbPsc::Div1`]; on every other divider the timers are fed twice the
    /// bus clock, so the bus clock cannot be used in a timer period formula.
    pub fn pclk1_tim(&self) -> Hertz {
        self.pclk1_tim
    }
    /// Clock feeding the timers on APB2 (TIMER0, TIMER14, TIMER15, TIMER16).
    ///
    /// Same doubling rule as [`pclk1_tim`](Self::pclk1_tim).
    pub fn pclk2_tim(&self) -> Hertz {
        self.pclk2_tim
    }
}

/// Builder for the clock tree, applied by [`freeze`](UnfrozenRcu::freeze).
///
/// [`Default`] holds the reset state in one place — undivided buses, IRC8M as the
/// system clock, USART0 on APB2 and no ADC clock — and every field is written to
/// its registers whether it was named or not.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockConfig {
    hclk: AhbPsc,
    pclk1: ApbPsc,
    pclk2: ApbPsc,
    sysclk: SysClk,
    usart0_sel: Usart0Sel,
    adc_sel: AdcSel,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            hclk: AhbPsc::Div1,
            pclk1: ApbPsc::Div1,
            pclk2: ApbPsc::Div1,
            sysclk: SysClk::Irc8m,
            usart0_sel: Usart0Sel::Apb2,
            adc_sel: AdcSel::Off,
        }
    }
}

impl ClockConfig {
    fn pll_mul(desired: PllFreq) -> u32 {
        (desired as u32) / PLL_SRC
    }

    /// Sets the AHB prescaler, dividing `sysclk` down to `hclk`.
    pub fn hclk(mut self, psc: AhbPsc) -> Self {
        self.hclk = psc;
        self
    }
    /// Sets the APB1 prescaler, dividing `hclk` down to `pclk1`.
    pub fn pclk1(mut self, psc: ApbPsc) -> Self {
        self.pclk1 = psc;
        self
    }
    /// Sets the APB2 prescaler, dividing `hclk` down to `pclk2`.
    pub fn pclk2(mut self, psc: ApbPsc) -> Self {
        self.pclk2 = psc;
        self
    }
    /// Picks the system clock source.
    pub fn sysclk(mut self, src: SysClk) -> Self {
        self.sysclk = src;
        self
    }
    /// Picks the USART0 clock source.
    pub fn usart0_sel(mut self, src: Usart0Sel) -> Self {
        self.usart0_sel = src;
        self
    }
    /// Picks the ADC clock source and starts it if needed.
    ///
    /// Left at [`AdcSel::Off`] the ADC has no clock and [`Clocks::ck_adc`] stays
    /// zero — constructing an [`Adc`](crate::adc::Adc) would then divide by zero
    /// rather than silently hang in calibration.
    pub fn adc_sel(mut self, sel: AdcSel) -> Self {
        self.adc_sel = sel;
        self
    }
}

/// The RCU before its clock tree is frozen, handed out by [`RcuExt::constrain`].
///
/// [`freeze`](Self::freeze) is the only thing it does, and it consumes this
/// value — so the tree is configured exactly once, and every driver, which takes
/// [`Rcu`], can only be built afterwards.
pub struct UnfrozenRcu {
    rcu: pac::Rcu,
}

impl UnfrozenRcu {
    /// Applies `config` and hands back the RCU at the resulting frequencies.
    ///
    /// Flash wait states are raised from the new `hclk` *before* the system clock
    /// switches over, so the flash is never read faster than it can respond.
    /// `fmc` is taken because those wait states live in a separate peripheral.
    pub fn freeze(self, fmc: &mut Fmc, config: ClockConfig) -> Rcu {
        let sysclk = match config.sysclk {
            SysClk::Irc8m => IRC8M,
            SysClk::Pll(desired) => {
                let mul = ClockConfig::pll_mul(desired);
                self.rcu.cfg0().modify(|_, w| {
                    let w = w.pllsel().irc8m_2();
                    match mul {
                        2 => w.pllmf().mul2().pllmf_msb().none(),
                        3 => w.pllmf().mul3().pllmf_msb().none(),
                        4 => w.pllmf().mul4().pllmf_msb().none(),
                        5 => w.pllmf().mul5().pllmf_msb().none(),
                        6 => w.pllmf().mul6().pllmf_msb().none(),
                        7 => w.pllmf().mul7().pllmf_msb().none(),
                        8 => w.pllmf().mul8().pllmf_msb().none(),
                        9 => w.pllmf().mul9().pllmf_msb().none(),
                        10 => w.pllmf().mul10().pllmf_msb().none(),
                        11 => w.pllmf().mul11().pllmf_msb().none(),
                        12 => w.pllmf().mul12().pllmf_msb().none(),
                        13 => w.pllmf().mul13().pllmf_msb().none(),
                        14 => w.pllmf().mul14().pllmf_msb().none(),
                        15 => w.pllmf().mul15().pllmf_msb().none(),
                        16 => w.pllmf().mul16().pllmf_msb().none(),
                        17 => w.pllmf().mul2().pllmf_msb().plus15(),
                        18 => w.pllmf().mul3().pllmf_msb().plus15(),
                        _ => unreachable!(),
                    }
                });
                self.rcu.ctl0().modify(|_, w| w.pllen().on());
                while self.rcu.ctl0().read().pllstb().is_not_ready() {}
                desired as u32
            }
        };

        let ahb_psc = config.hclk;
        let hclk = sysclk / (ahb_psc as u32);
        let apb1_psc = config.pclk1;
        let pclk1 = hclk / (apb1_psc as u32);
        let apb2_psc = config.pclk2;
        let pclk2 = hclk / (apb2_psc as u32);

        let usart0_sel = config.usart0_sel;
        let usart0 = match usart0_sel {
            Usart0Sel::Apb2 => pclk2,
            Usart0Sel::Sysclk => sysclk,
            Usart0Sel::Lxtal => LXTAL,
            Usart0Sel::Irc8m => IRC8M,
        };
        self.rcu.cfg2().modify(|_, w| match usart0_sel {
            Usart0Sel::Apb2 => w.usart0sel().apb2(),
            Usart0Sel::Sysclk => w.usart0sel().sys(),
            Usart0Sel::Lxtal => w.usart0sel().lxtal(),
            Usart0Sel::Irc8m => w.usart0sel().irc8m(),
        });

        let ck_adc = match config.adc_sel {
            AdcSel::Off => 0,
            AdcSel::Irc28m(div) => {
                self.rcu.ctl1().modify(|_, w| w.irc28men().on());
                while self.rcu.ctl1().read().irc28mstb().is_not_ready() {}
                self.rcu.cfg2().modify(|_, w| {
                    let w = match div {
                        Irc28mDiv::Div1 => w.irc28mdiv().bit(IRC28MDIV_DIV1),
                        Irc28mDiv::Div2 => w.irc28mdiv().bit(IRC28MDIV_DIV2),
                    };
                    w.adcsel().bit(ADCSEL_IRC28M)
                });
                match div {
                    Irc28mDiv::Div1 => IRC28M,
                    Irc28mDiv::Div2 => IRC28M / 2,
                }
            }
            AdcSel::Prescaled(psc) => {
                // ADCPSC = 3-bit code split CFG0[15:14] + CFG2[31] (like PLLMF+MSB)
                self.rcu.cfg0().modify(|_, w| match psc {
                    AdcPsc::Apb2Div2 | AdcPsc::AhbDiv3 => w.adcpsc().div2(),
                    AdcPsc::Apb2Div4 | AdcPsc::AhbDiv5 => w.adcpsc().div4(),
                    AdcPsc::Apb2Div6 | AdcPsc::AhbDiv7 => w.adcpsc().div6(),
                    AdcPsc::Apb2Div8 | AdcPsc::AhbDiv9 => w.adcpsc().div8(),
                });
                self.rcu.cfg2().modify(|_, w| {
                    let w = match psc {
                        AdcPsc::Apb2Div2
                        | AdcPsc::Apb2Div4
                        | AdcPsc::Apb2Div6
                        | AdcPsc::Apb2Div8 => w.adcpsc().bit(ADCPSC_MSB_APB2),
                        AdcPsc::AhbDiv3 | AdcPsc::AhbDiv5 | AdcPsc::AhbDiv7 | AdcPsc::AhbDiv9 => {
                            w.adcpsc().bit(ADCPSC_MSB_AHB)
                        }
                    };
                    w.adcsel().bit(ADCSEL_PRESCALED)
                });
                match psc {
                    AdcPsc::Apb2Div2 => pclk2 / 2,
                    AdcPsc::Apb2Div4 => pclk2 / 4,
                    AdcPsc::Apb2Div6 => pclk2 / 6,
                    AdcPsc::Apb2Div8 => pclk2 / 8,
                    AdcPsc::AhbDiv3 => hclk / 3,
                    AdcPsc::AhbDiv5 => hclk / 5,
                    AdcPsc::AhbDiv7 => hclk / 7,
                    AdcPsc::AhbDiv9 => hclk / 9,
                }
            }
        };

        fmc.set_ws(hclk);

        self.rcu.cfg0().modify(|_, w| {
            let w = match ahb_psc {
                AhbPsc::Div1 => w.ahbpsc().div1(),
                AhbPsc::Div2 => w.ahbpsc().div2(),
                AhbPsc::Div4 => w.ahbpsc().div4(),
                AhbPsc::Div8 => w.ahbpsc().div8(),
                AhbPsc::Div16 => w.ahbpsc().div16(),
                AhbPsc::Div64 => w.ahbpsc().div64(),
                AhbPsc::Div128 => w.ahbpsc().div128(),
                AhbPsc::Div256 => w.ahbpsc().div256(),
                AhbPsc::Div512 => w.ahbpsc().div512(),
            };
            let w = match apb1_psc {
                ApbPsc::Div1 => w.apb1psc().div1(),
                ApbPsc::Div2 => w.apb1psc().div2(),
                ApbPsc::Div4 => w.apb1psc().div4(),
                ApbPsc::Div8 => w.apb1psc().div8(),
                ApbPsc::Div16 => w.apb1psc().div16(),
            };
            let w = match apb2_psc {
                ApbPsc::Div1 => w.apb2psc().div1(),
                ApbPsc::Div2 => w.apb2psc().div2(),
                ApbPsc::Div4 => w.apb2psc().div4(),
                ApbPsc::Div8 => w.apb2psc().div8(),
                ApbPsc::Div16 => w.apb2psc().div16(),
            };
            match config.sysclk {
                SysClk::Pll(_) => w.scs().pll(),
                // SCS is left alone: switching down after the wait states were
                // set for a higher hclk would read the flash too fast.
                SysClk::Irc8m => w,
            }
        });
        let clocks = Clocks {
            hclk: Hertz::from_raw(hclk),
            pclk1: Hertz::from_raw(pclk1),
            pclk2: Hertz::from_raw(pclk2),
            sysclk: Hertz::from_raw(sysclk),
            usart0: Hertz::from_raw(usart0),
            ck_adc: Hertz::from_raw(ck_adc),
            pclk1_tim: Hertz::from_raw(match apb1_psc {
                ApbPsc::Div1 => hclk,              // == pclk1
                _ => hclk / (apb1_psc as u32 / 2), // == pclk1 * 2
            }),
            pclk2_tim: Hertz::from_raw(match apb2_psc {
                ApbPsc::Div1 => hclk,              // == pclk2
                _ => hclk / (apb2_psc as u32 / 2), // == pclk2 * 2
            }),
        };
        Rcu {
            rcu: self.rcu,
            clocks,
        }
    }
}

/// Divider on the PLL branch feeding `CK_OUT`, ahead of the source multiplexer.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum PllDiv {
    Div1,
    Div2,
}

/// Clock node to route out on the `CK_OUT` pin.
///
/// The PLL branch carries its own pre-multiplexer divider inside the variant, so
/// it cannot be set for a source it doesn't apply to. Selecting a source that
/// isn't running simply leaves the pin quiet.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CkOutSrc {
    /// Nothing driven out.
    None,
    /// The internal RC oscillator dedicated to the ADC.
    Irc14m,
    /// The internal low-speed RC oscillator.
    Lsi40k,
    /// The external low-speed crystal.
    Lxtal,
    /// The system clock.
    Sysclk,
    /// The internal 8 MHz RC oscillator.
    Irc8m,
    /// The external high-speed crystal.
    Hxtal,
    /// The PLL output, through its own divider.
    Pll(PllDiv),
}

/// Divider applied to `CK_OUT` after the source multiplexer, for any source.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum CkOutDiv {
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
    Div32,
    Div64,
    Div128,
}

/// What brought the chip up, as recorded in `RSTSCK`.
///
/// Independent flags rather than one cause: several can stand after a single
/// reset, and nothing clears any of them but
/// [`clear_reset_flags`](Rcu::clear_reset_flags).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetFlag {
    /// Deep-sleep or standby reset.
    LowPower,
    /// The window watchdog timed out.
    WindowWatchdog,
    /// The free watchdog timed out.
    FreeWatchdog,
    /// A software reset was requested.
    Software,
    /// Power reset.
    Power,
    /// The external reset pin was pulled.
    ExternalPin,
    /// The option byte loader ran.
    OptionByteLoader,
}

/// Owns the RCU peripheral with its clock tree already frozen; obtained from
/// [`UnfrozenRcu::freeze`].
///
/// Every driver takes this type, so none of them can be built before the tree is
/// configured — there is no other way to get one.
pub struct Rcu {
    rcu: pac::Rcu,
    clocks: Clocks,
}

impl Rcu {
    /// The frequencies the tree was frozen at.
    pub fn clocks(&self) -> Clocks {
        self.clocks
    }

    /// Routes an internal clock node out onto `PA8` (AF0) or `PA9` (AF5).
    ///
    /// The pin still has to be put into the matching alternate function. Applied
    /// immediately and not recorded in [`Clocks`] — nothing else needs it.
    pub fn ck_out(&mut self, src: CkOutSrc, div: CkOutDiv) {
        self.rcu.cfg0().modify(|_, w| {
            let w = match div {
                CkOutDiv::Div1 => w.ckoutdiv().div1(),
                CkOutDiv::Div2 => w.ckoutdiv().div2(),
                CkOutDiv::Div4 => w.ckoutdiv().div4(),
                CkOutDiv::Div8 => w.ckoutdiv().div8(),
                CkOutDiv::Div16 => w.ckoutdiv().div16(),
                CkOutDiv::Div32 => w.ckoutdiv().div32(),
                CkOutDiv::Div64 => w.ckoutdiv().div64(),
                CkOutDiv::Div128 => w.ckoutdiv().div128(),
            };
            match src {
                CkOutSrc::None => w.ckoutsel().none(),
                CkOutSrc::Irc14m => w.ckoutsel().irc14m(),
                CkOutSrc::Lsi40k => w.ckoutsel().lsi40k(),
                CkOutSrc::Lxtal => w.ckoutsel().lxtal(),
                CkOutSrc::Sysclk => w.ckoutsel().sysclk(),
                CkOutSrc::Irc8m => w.ckoutsel().irc8m(),
                CkOutSrc::Hxtal => w.ckoutsel().hxtal(),
                CkOutSrc::Pll(d) => {
                    let w = w.ckoutsel().pll();
                    match d {
                        PllDiv::Div1 => w.plldv().div1(),
                        PllDiv::Div2 => w.plldv().div2(),
                    }
                }
            }
        });
    }

    /// Starts the internal 40 kHz oscillator and blocks until it is stable.
    ///
    /// Its frequency is fixed and does not depend on the clock tree, so nothing
    /// is recorded in [`Clocks`].
    pub fn enable_irc40k(&mut self) {
        self.rcu.rstsck().modify(|_, w| w.irc40ken().on());
        while self.rcu.rstsck().read().irc40kstb().is_not_ready() {}
    }
    /// Stops the internal 40 kHz oscillator.
    ///
    /// FWDGT and the RTC run off it; both stop with it.
    pub fn disable_irc40k(&mut self) {
        self.rcu.rstsck().modify(|_, w| w.irc40ken().off());
    }

    /// Clocks SYSCFG and the comparator.
    ///
    /// `CFGCMPEN` gates both blocks at once, so this is not [`Enable`]: naming the
    /// pair is the only warning that the peripheral next door goes with it.
    pub fn enable_cfgcmp(&mut self) {
        self.rcu.apb2en().modify(|_, w| w.cfgcmpen().enabled());
    }
    /// Stops the clock of SYSCFG and the comparator.
    ///
    /// One bit gates both blocks: SYSCFG stops too.
    pub fn disable_cfgcmp(&mut self) {
        self.rcu.apb2en().modify(|_, w| w.cfgcmpen().disabled());
    }
    /// Pulses the reset line of SYSCFG and the comparator.
    ///
    /// One bit resets both blocks: the EXTI source selection and the remaps in
    /// SYSCFG return to their defaults too.
    pub fn reset_cfgcmp(&mut self) {
        self.rcu.apb2rst().modify(|_, w| w.cfgcmprst().reset());
        self.rcu.apb2rst().modify(|_, w| w.cfgcmprst().clear_bit());
    }

    /// Whether `flag` took part in the last reset.
    ///
    /// Reads the flag as it stands; more than one can answer `true`.
    pub fn reset_flag(&self, flag: ResetFlag) -> bool {
        let rstsck = self.rcu.rstsck().read();
        match flag {
            ResetFlag::LowPower => rstsck.lprstf().is_reset(),
            ResetFlag::WindowWatchdog => rstsck.wwdgtrstf().is_reset(),
            ResetFlag::FreeWatchdog => rstsck.fwdgtrstf().is_reset(),
            ResetFlag::Software => rstsck.swrstf().is_reset(),
            ResetFlag::Power => rstsck.porrstf().is_reset(),
            ResetFlag::ExternalPin => rstsck.eprstf().is_reset(),
            ResetFlag::OptionByteLoader => rstsck.oblrstf().is_reset(),
        }
    }
    /// Clears every reset flag at once; `RSTFC` has no per-flag granularity.
    ///
    /// Until this is called the flags accumulate across resets, so a later
    /// reading cannot tell which reset set them.
    pub fn clear_reset_flags(&mut self) {
        self.rcu.rstsck().modify(|_, w| w.rstfc().clear());
    }
}

/// Extension trait turning the raw RCU peripheral into the managed [`UnfrozenRcu`].
pub trait RcuExt {
    /// Consumes the raw peripheral and returns the managed wrapper, which
    /// [`freeze`](UnfrozenRcu::freeze) turns into an [`Rcu`].
    fn constrain(self) -> UnfrozenRcu;
}

impl RcuExt for pac::Rcu {
    fn constrain(self) -> UnfrozenRcu {
        UnfrozenRcu { rcu: self }
    }
}

/// Clock gating for a peripheral, implemented per peripheral type.
///
/// Drivers call [`enable`](Enable::enable) from their constructors, so a
/// peripheral cannot be used before its clock is running.
pub trait Enable {
    /// Switches the peripheral's clock on.
    fn enable(rcu: &mut Rcu);
    /// Switches the peripheral's clock off.
    fn disable(rcu: &mut Rcu);
}

/// Reset control for a peripheral, implemented per peripheral type.
pub trait Reset {
    /// Pulses the peripheral's reset line, returning its registers to defaults.
    fn reset(rcu: &mut Rcu);
}

macro_rules! bus_en {
    ($($Periph:ty => $en_reg:ident, $en_bit:ident,)+) => {
        $(
            impl Enable for $Periph {
                fn enable(rcu: &mut Rcu) {
                    rcu.rcu.$en_reg().modify(|_, w| w.$en_bit().enabled());
                }
                fn disable(rcu: &mut Rcu) {
                    rcu.rcu.$en_reg().modify(|_, w| w.$en_bit().disabled());
                }
            }
        )+
    };
}

macro_rules! bus_rst {
    ($($Periph:ty => $rst_reg:ident, $rst_bit:ident,)+) => {
        $(
            impl Reset for $Periph {
                fn reset(rcu: &mut Rcu) {
                    rcu.rcu.$rst_reg().modify(|_, w| w.$rst_bit().reset());
                    rcu.rcu.$rst_reg().modify(|_, w| w.$rst_bit().clear_bit());
                }
            }
        )+
    };
}

bus_en! {
    pac::Gpioa => ahben, paen,
    pac::Gpiob => ahben, pben,
    pac::Gpiof => ahben, pfen,
    pac::Dma => ahben, dmaen,
    pac::Crc => ahben, crcen,
    pac::Usart1 => apb1en, usart1en,
    pac::Spi1 => apb1en, spi1en,
    pac::Timer2 => apb1en, timer2en,
    pac::Timer5 => apb1en, timer5en,
    pac::Timer13 => apb1en, timer13en,
    pac::I2c0 => apb1en, i2c0en,
    pac::I2c1 => apb1en, i2c1en,
    pac::Wwdgt => apb1en, wwdgten,
    pac::Usart0 => apb2en, usart0en,
    pac::Adc => apb2en, adcen,
    pac::Spi0 => apb2en, spi0en,
    pac::Timer0 => apb2en, timer0en,
    pac::Timer15 => apb2en, timer15en,
    pac::Timer16 => apb2en, timer16en,
}

// TIMER14 is absent from the 20- and 24-pin parts whatever their flash size, so its
// gate is the part's own row in build.rs rather than the flash code.
#[cfg(has_timer14)]
bus_en! {
    pac::Timer14 => apb2en, timer14en,
}

// Port C bonds three pads on the 48-pin package and none anywhere else.
#[cfg(pads_ge_48)]
bus_en! {
    pac::Gpioc => ahben, pcen,
}

bus_rst! {
    pac::Gpioa => ahbrst, parst,
    pac::Gpiob => ahbrst, pbrst,
    pac::Gpiof => ahbrst, pfrst,
    pac::Usart1 => apb1rst, usart1rst,
    pac::Spi1 => apb1rst, spi1rst,
    pac::Timer2 => apb1rst, timer2rst,
    pac::Timer5 => apb1rst, timer5rst,
    pac::Timer13 => apb1rst, timer13rst,
    pac::I2c0 => apb1rst, i2c0rst,
    pac::I2c1 => apb1rst, i2c1rst,
    pac::Wwdgt => apb1rst, wwdgtrst,
    pac::Usart0 => apb2rst, usart0rst,
    pac::Adc => apb2rst, adcrst,
    pac::Spi0 => apb2rst, spi0rst,
    pac::Timer0 => apb2rst, timer0rst,
    pac::Timer15 => apb2rst, timer15rst,
    pac::Timer16 => apb2rst, timer16rst,
}

#[cfg(has_timer14)]
bus_rst! {
    pac::Timer14 => apb2rst, timer14rst,
}

#[cfg(pads_ge_48)]
bus_rst! {
    pac::Gpioc => ahbrst, pcrst,
}
