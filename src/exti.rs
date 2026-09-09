//! External interrupts and events.
//!
//! GPIO has no interrupt of its own: an edge on a pin becomes one only here.
//! The detector is asynchronous, so it works with the system clock stopped,
//! which is what makes EXTI the way out of deep sleep and standby.
//!
//! [`ExtiExt::split`] hands out one token per line, and only the lines that
//! exist — the reserved numbers have no field to name. Lines 0 to 15 arrive
//! without a source and reach the rest of their API by being given a pin, which
//! they then own; the internal lines have their source wired in silicon.
//!
//! One line serves every port: `PA0`, `PB0` and `PF0` all feed line 0, and the
//! single token is what keeps two of them from claiming it.
//!
//! The pin lines share three vectors — `EXTI0_1`, `EXTI2_3`, `EXTI4_15` — so a
//! handler serves several lines and has to ask which of them is pending. The
//! internal lines have no EXTI vector at all: each arrives on the vector of the
//! peripheral behind it, LVD on its own and the rest on RTC, ADC/CMP and
//! USART0.
//!
//! The two outputs are not symmetric in what they leave behind: only the
//! interrupt path latches `PD`, so a line used for events alone has no flag to
//! clear after `WFE`.

use crate::gpio::Pin;
use crate::pac;
use crate::syscfg::Syscfg;

/// A line that has no source yet, as [`ExtiExt::split`] hands it out.
pub struct PinSrc;

/// A line whose source is wired in silicon and needs no pin.
pub struct InternalSrc;

/// Marks a line whose source is settled, which is what the rest of the line's
/// API hangs off.
///
/// Implemented for [`InternalSrc`] and for every pin, but never for
/// [`PinSrc`]: a line that has not been given a port would otherwise listen to
/// whatever `EXTISS` happens to hold, which after reset is port A.
pub trait ExtiConfigured {}

impl ExtiConfigured for InternalSrc {}

/// Source selection code for gpioa
const PA_SS_CODE: u8 = 0b000;
/// Source selection code for gpiob
const PB_SS_CODE: u8 = 0b001;
/// Source selection code for gpioc
const PC_SS_CODE: u8 = 0b010;
/// Source selection code for gpiof
const PF_SS_CODE: u8 = 0b101;

/// The `EXTISS` code of a port, so the table below states a pin once rather
/// than repeating its port's encoding on every row.
const fn ss_code(port: char) -> u8 {
    match port {
        'A' => PA_SS_CODE,
        'B' => PB_SS_CODE,
        'C' => PC_SS_CODE,
        'F' => PF_SS_CODE,
        _ => unreachable!(),
    }
}

/// Marks a pin as a source line `N` can be pointed at, and carries the `EXTISS`
/// code that points it there.
///
/// Populated from the line map: pin number and line number are the same, so the
/// only thing left to state is which ports bond that number. A pin the package
/// does not bond has no impl and so cannot be handed to [`ExtiLine::source`].
pub trait ExtiPin: ExtiConfigured {
    /// Value written to this line's `EXTISS` field to select the pin's port.
    const SS_CODE: u8;
}

// One impl per row, so the gate can stay inside the optional fragment — unlike
// `pin_af!`, where a row expands to several impls and the attribute has to be
// replicated by recursion instead.
macro_rules! exti_pin {
    () => {};
    ( $(#[$cfg:meta])? $p:literal $n:literal $(, $($rest:tt)*)? ) => {
        $(#[$cfg])?
        #[doc = concat!("`P", $p, stringify!($n), "` on EXTI line ", stringify!($n), ".")]
        impl<MODE> ExtiPin for Pin<$p, $n, MODE> {
            const SS_CODE: u8 = ss_code($p);
        }
        $(#[$cfg])?
        impl<MODE> ExtiConfigured for Pin<$p, $n, MODE> {}
        $( exti_pin! { $($rest)* } )?
    };
}

// Which ports bond a given pin number, gated exactly as the pins themselves are
// in `gpio.rs`.
exti_pin! {
    // ---- Port A ----
    'A' 0, 'A' 1, 'A' 2, 'A' 3, 'A' 4, 'A' 5, 'A' 6, 'A' 7,
    #[cfg(pads_ge_24)] 'A' 8,
    'A' 9, 'A' 10,
    #[cfg(pads_ge_lqfp32)] 'A' 11,
    #[cfg(pads_ge_lqfp32)] 'A' 12,
    'A' 13, 'A' 14,
    #[cfg(pads_ge_28)] 'A' 15,
    // ---- Port B ----
    #[cfg(pads_ge_24)] 'B' 0,
    'B' 1,
    #[cfg(pads_ge_qfn32)] 'B' 2,
    #[cfg(pads_ge_28)] 'B' 3,
    #[cfg(pads_ge_28)] 'B' 4,
    #[cfg(pads_ge_28)] 'B' 5,
    #[cfg(pads_ge_24)] 'B' 6,
    #[cfg(pads_ge_24)] 'B' 7,
    #[cfg(pads_ge_qfn32)] 'B' 8,
    #[cfg(pads_ge_48)] 'B' 9,
    #[cfg(pads_ge_48)] 'B' 10,
    #[cfg(pads_ge_48)] 'B' 11,
    #[cfg(pads_ge_48)] 'B' 12,
    #[cfg(pads_ge_48)] 'B' 13,
    #[cfg(pads_ge_48)] 'B' 14,
    #[cfg(pads_ge_48)] 'B' 15,
    // ---- Port C ----
    #[cfg(pads_ge_48)] 'C' 13,
    #[cfg(pads_ge_48)] 'C' 14,
    #[cfg(pads_ge_48)] 'C' 15,
    // ---- Port F ----
    'F' 0, 'F' 1,
    #[cfg(pads_ge_48)] 'F' 6,
    #[cfg(pads_ge_48)] 'F' 7,
}

/// Which edges on the source raise the line.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EdgeTrigger {
    /// Neither edge; the line answers only to software.
    None,
    /// Low to high.
    Rising,
    /// High to low.
    Falling,
    /// Either direction.
    Both,
}

/// One EXTI line, identified by its number at the type level, holding whatever
/// drives it.
///
/// `SRC` is [`PinSrc`] before a port is picked, the pin itself afterwards, and
/// [`InternalSrc`] on the lines that have no pin. Cannot be constructed outside
/// this module: the only lines that exist come from [`ExtiExt::split`], which is
/// what makes a line a unique token.
pub struct ExtiLine<const N: u8, SRC> {
    src: SRC,
}

impl<const N: u8, SRC> ExtiLine<N, SRC> {
    fn reg(&self) -> &pac::exti::RegisterBlock {
        unsafe { &*pac::Exti::ptr() }
    }
}

impl<const N: u8> ExtiLine<N, PinSrc> {
    /// Points the line at `pin` and takes the pin in.
    ///
    /// The pin keeps working as it was — EXTI taps the pad and changes nothing
    /// about the port. What it loses is its mode transitions: `into_*` take the
    /// pin by value, and while the line holds it only references get out.
    pub fn source<const P: char, MODE>(
        self,
        syscfg: &mut Syscfg,
        pin: Pin<P, N, MODE>,
    ) -> ExtiLine<N, Pin<P, N, MODE>>
    where
        Pin<P, N, MODE>: ExtiPin,
    {
        syscfg.set_extiss(N, <Pin<P, N, MODE> as ExtiPin>::SS_CODE);
        ExtiLine { src: pin }
    }
}

impl<const N: u8, SRC> ExtiLine<N, SRC>
where
    SRC: ExtiConfigured,
{
    /// This line's bit, the same position in every per-line register.
    const MASK: u32 = 1 << N;

    /// `bits` with this line's bit set or cleared and the others left alone.
    ///
    /// Every per-line register is written through this: the registers are
    /// shared by all twenty-one lines, so only read-modify-write leaves the
    /// neighbours standing. The sequence is not atomic, so two lines configured
    /// from different contexts can still lose one another's edit — put such a
    /// call in a critical section.
    const fn with_bit(bits: u32, on: bool) -> u32 {
        if on {
            bits | Self::MASK
        } else {
            bits & !Self::MASK
        }
    }
    fn set_rten(&mut self, on: bool) {
        self.reg()
            .rten()
            .modify(|r, w| unsafe { w.bits(Self::with_bit(r.bits(), on)) });
    }
    fn set_ften(&mut self, on: bool) {
        self.reg()
            .ften()
            .modify(|r, w| unsafe { w.bits(Self::with_bit(r.bits(), on)) });
    }
    fn set_inten(&mut self, on: bool) {
        self.reg()
            .inten()
            .modify(|r, w| unsafe { w.bits(Self::with_bit(r.bits(), on)) });
    }
    fn set_even(&mut self, on: bool) {
        self.reg()
            .even()
            .modify(|r, w| unsafe { w.bits(Self::with_bit(r.bits(), on)) });
    }

    /// Detects rising edges, falling edges, both, or neither.
    ///
    /// `RTEN` and `FTEN` are independent, so a line with neither set is left
    /// firing only from software.
    pub fn edge(&mut self, edge: EdgeTrigger) {
        let (rising, falling) = match edge {
            EdgeTrigger::None => (false, false),
            EdgeTrigger::Rising => (true, false),
            EdgeTrigger::Falling => (false, true),
            EdgeTrigger::Both => (true, true),
        };
        self.set_rten(rising);
        self.set_ften(falling);
    }

    /// Raises this line's interrupt in the NVIC.
    ///
    /// One vector serves several lines, so a handler still has to ask which of
    /// them is pending.
    pub fn listen(&mut self) {
        self.set_inten(true);
    }
    /// Stops the line reaching the NVIC. The pending flag still latches.
    pub fn unlisten(&mut self) {
        self.set_inten(false);
    }
    /// Whether the line reaches the NVIC.
    pub fn is_listening(&self) -> bool {
        self.reg().inten().read().bits() & Self::MASK != 0
    }

    /// Raises this line as an event instead of an interrupt.
    ///
    /// An event enters no handler: it sets the core's event latch, which is
    /// what `WFE` waits on, and execution carries on from there.
    pub fn listen_event(&mut self) {
        self.set_even(true);
    }
    /// Stops the line raising events.
    pub fn unlisten_event(&mut self) {
        self.set_even(false);
    }
    /// Whether the line raises events.
    pub fn is_listening_event(&self) -> bool {
        self.reg().even().read().bits() & Self::MASK != 0
    }

    /// Raises the line from software, edge or no edge.
    ///
    /// `SWIEV` goes straight to the pending flag, so a line left on
    /// [`EdgeTrigger::None`] still fires this way.
    pub fn pend(&mut self) {
        self.reg()
            .swiev()
            .modify(|r, w| unsafe { w.bits(Self::with_bit(r.bits(), true)) });
    }

    /// Whether the line has fired and not been cleared since.
    ///
    /// Only the interrupt path latches this: a line raised while it is listening
    /// for events alone leaves the flag clear, so polling it without
    /// [`listen`](Self::listen) never sees anything.
    pub fn is_pending(&self) -> bool {
        self.reg().pd().read().bits() & Self::MASK != 0
    }

    /// Clears the pending flag.
    ///
    /// The request is a level, so a handler that returns without this is
    /// entered again at once. `PD` clears by writing a one, which is why this
    /// writes the bare mask instead of reading first: a zero elsewhere in the
    /// word leaves that line's flag alone.
    pub fn clear_interrupt(&mut self) {
        self.reg().pd().write(|w| unsafe { w.bits(Self::MASK) });
    }
}

impl<const P: char, const N: u8, MODE> ExtiLine<N, Pin<P, N, MODE>> {
    /// The pin the line listens to, for reading it.
    pub fn pin(&self) -> &Pin<P, N, MODE> {
        &self.src
    }
    /// The pin the line listens to, for driving it.
    ///
    /// A line watching a pin this code drives itself is legal, and this is how
    /// that pin is still reached.
    pub fn pin_mut(&mut self) -> &mut Pin<P, N, MODE> {
        &mut self.src
    }

    /// Gives the pin back and returns the line to its unsourced state.
    ///
    /// Disarms the line first — no edges, neither output, no pending flag — so
    /// the next owner of the pin does not inherit an interrupt on it. `EXTISS`
    /// keeps pointing at the port until another pin is handed in, which is
    /// harmless while nothing listens.
    pub fn release(mut self) -> (ExtiLine<N, PinSrc>, Pin<P, N, MODE>)
    where
        Pin<P, N, MODE>: ExtiConfigured,
    {
        self.edge(EdgeTrigger::None);
        self.unlisten();
        self.unlisten_event();
        self.clear_interrupt();
        (ExtiLine { src: PinSrc }, self.src)
    }
}

/// The EXTI lines, as handed out by [`ExtiExt::split`].
///
/// The reserved line numbers are absent: a number the hardware does not
/// implement cannot be named rather than being rejected later.
#[allow(missing_docs)]
pub struct ExtiLines {
    pub line0: ExtiLine<0, PinSrc>,
    pub line1: ExtiLine<1, PinSrc>,
    pub line2: ExtiLine<2, PinSrc>,
    pub line3: ExtiLine<3, PinSrc>,
    pub line4: ExtiLine<4, PinSrc>,
    pub line5: ExtiLine<5, PinSrc>,
    pub line6: ExtiLine<6, PinSrc>,
    pub line7: ExtiLine<7, PinSrc>,
    pub line8: ExtiLine<8, PinSrc>,
    pub line9: ExtiLine<9, PinSrc>,
    pub line10: ExtiLine<10, PinSrc>,
    pub line11: ExtiLine<11, PinSrc>,
    pub line12: ExtiLine<12, PinSrc>,
    pub line13: ExtiLine<13, PinSrc>,
    pub line14: ExtiLine<14, PinSrc>,
    pub line15: ExtiLine<15, PinSrc>,
    /// Low voltage detector, in PMU.
    pub line16: ExtiLine<16, InternalSrc>,
    /// RTC alarm.
    pub line17: ExtiLine<17, InternalSrc>,
    /// RTC tamper and timestamp.
    pub line19: ExtiLine<19, InternalSrc>,
    /// Comparator output.
    pub line21: ExtiLine<21, InternalSrc>,
    /// USART0 wakeup.
    pub line25: ExtiLine<25, InternalSrc>,
}

/// Splits the EXTI peripheral into its individual lines.
pub trait ExtiExt {
    /// The lines this peripheral hands out.
    type Lines;

    /// Hands out the lines.
    ///
    /// Consumes the peripheral, so the lines it returns are the only ones that
    /// will ever exist. Takes no clock: EXTI has no enable bit of its own, and
    /// the `EXTISS` half of the job lives in [`crate::syscfg::Syscfg`], which
    /// switches on the clock it does need.
    fn split(self) -> Self::Lines;
}

impl ExtiExt for pac::Exti {
    type Lines = ExtiLines;
    fn split(self) -> Self::Lines {
        ExtiLines {
            line0: ExtiLine { src: PinSrc },
            line1: ExtiLine { src: PinSrc },
            line2: ExtiLine { src: PinSrc },
            line3: ExtiLine { src: PinSrc },
            line4: ExtiLine { src: PinSrc },
            line5: ExtiLine { src: PinSrc },
            line6: ExtiLine { src: PinSrc },
            line7: ExtiLine { src: PinSrc },
            line8: ExtiLine { src: PinSrc },
            line9: ExtiLine { src: PinSrc },
            line10: ExtiLine { src: PinSrc },
            line11: ExtiLine { src: PinSrc },
            line12: ExtiLine { src: PinSrc },
            line13: ExtiLine { src: PinSrc },
            line14: ExtiLine { src: PinSrc },
            line15: ExtiLine { src: PinSrc },
            line16: ExtiLine { src: InternalSrc },
            line17: ExtiLine { src: InternalSrc },
            line19: ExtiLine { src: InternalSrc },
            line21: ExtiLine { src: InternalSrc },
            line25: ExtiLine { src: InternalSrc },
        }
    }
}
