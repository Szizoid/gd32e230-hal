//! Timers: the counter core shared by every TIMER peripheral.
//!
//! A timer counts `CK_TIMERx` ticks divided by `PSC + 1`, and restarts from zero
//! once the count reaches `CAR`. Everything else a timer can do — PWM, input
//! capture, triggering other peripherals — is built on top of that core.
//!
//! ```ignore
//! let timer = dp.timer5.constrain(&mut rcu);
//! let mut timer = timer.start_interval(500.millis());
//! timer.wait();
//! ```

use core::convert::Infallible;

use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::{ErrorType, SetDutyCycle};

use crate::gpio::{Alternate, Pin};
use crate::pac;
use crate::rcu::{Clocks, Enable, Rcu, Reset};
use crate::time::{Duration, Hertz, NanosDuration};

/// Counts one prescaler or counter cycle can span, both fields being 16-bit.
const MAX_COUNT: u32 = 1 << 16;

/// Splits a tick count into the `PSC` and `CAR` values whose cycle spans it.
///
/// The prescaler is kept as small as the counter width allows, since the
/// counter's step is the resolution everything inherits. Truncating division, so
/// the interval is at most one prescaler cycle short. Zero is raised to one.
fn dividers(ticks: u32) -> (u16, u16) {
    let ticks = ticks.max(1);
    let psc = ticks.div_ceil(MAX_COUNT);
    let car = ticks / psc;
    ((psc - 1) as u16, (car - 1) as u16)
}

/// Converts an interval of any scale into ticks of `clock`.
///
/// One `Duration` tick is `NOM / DENOM` seconds, so ticks are
/// `as_ticks() * NOM * clock / DENOM`. The division comes last, or every scale
/// finer than a second floors to zero. Saturating throughout: past `u32::MAX`
/// ticks is beyond what the dividers span anyway.
fn interval_to_ticks<const NOM: u64, const DENOM: u64>(
    interval: Duration<u32, NOM, DENOM>,
    clock: Hertz,
) -> u32 {
    let raw_time = u64::from(interval.as_ticks());
    let raw_freq = u64::from(clock.to_Hz());
    (raw_time.saturating_mul(raw_freq).saturating_mul(NOM) / DENOM).min(u32::MAX.into()) as u32
}

/// Converts a tick count of `clock`, divided by `psc`, into a duration.
///
/// The inverse of [`interval_to_ticks`]: one tick is `psc + 1` clock cycles, so
/// the duration is `ticks * (psc + 1) * DENOM / (clock * NOM)`. Division last,
/// in `u64`, saturating.
fn ticks_to_interval<const NOM: u64, const DENOM: u64>(
    ticks: u16,
    psc: u16,
    clock: Hertz,
) -> Duration<u32, NOM, DENOM> {
    let ticks = u64::from(ticks);
    let psc = u64::from(psc);
    let clock = u64::from(clock.to_Hz());
    Duration::<u32, NOM, DENOM>::from_ticks(
        (ticks.saturating_mul(psc + 1).saturating_mul(DENOM) / clock.saturating_mul(NOM))
            .min(u32::MAX.into()) as u32,
    )
}

/// Loads the dividers and sets the counter running.
///
/// `UPG` loads them out of their shadow registers; the update event it raises is
/// consumed here, so the first wait afterwards sees a real rollover.
fn start_counter<TIMERX: Instance>(timer: &mut TIMERX, psc: u16, car: u16) {
    timer.set_psc(psc);
    timer.set_car(car);
    timer.set_ups(true);
    timer.gen_update();
    timer.set_cen(true);
}

/// Blocks until the counter rolls over, then clears the flag it raised —
/// hardware never clears `UPIF` itself, so the next wait measures a fresh cycle.
fn wait_update<TIMERX: Instance>(timer: &mut TIMERX) {
    while !timer.read_upif() {}
    timer.clear_upif();
}

/// A timer event that can raise an interrupt.
///
/// Belongs to the counter itself, so every role that owns one takes it —
/// [`CountDownTimer`], [`Pwm`] and [`Capture`] alike. Channel events are not
/// here: a channel has exactly one, and it says so without an argument.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// The counter rolled over, ending one interval.
    Update,
}

// Every role of a timer answers the same way about its events, so the mapping
// from `Event` to registers lives here once rather than in each of them.
fn set_listening<TIMERX: Instance>(timer: &mut TIMERX, event: Event, on: bool) {
    match event {
        Event::Update => timer.set_upie(on),
    }
}

fn is_listening<TIMERX: Instance>(timer: &TIMERX, event: Event) -> bool {
    match event {
        Event::Update => timer.read_upie(),
    }
}

fn is_pending<TIMERX: Instance>(timer: &TIMERX, event: Event) -> bool {
    match event {
        Event::Update => timer.read_upif(),
    }
}

fn clear_pending<TIMERX: Instance>(timer: &mut TIMERX, event: Event) {
    match event {
        Event::Update => timer.clear_upif(),
    }
}

/// A timer peripheral, tying it to the bus that clocks it.
///
/// Which APB a timer sits on is fixed in silicon, so the frequency for the
/// period arithmetic comes from the type rather than from an argument.
pub trait Instance: Enable + Reset {
    /// The `CK_TIMERx` branch feeding this timer, taken from a frozen tree.
    fn clk(clocks: &Clocks) -> Hertz;
    /// Writes the prescaler, which reaches the counter on the next update event.
    fn set_psc(&mut self, psc: u16);
    /// Reads back the prescaler dividing the clock into counter ticks.
    fn read_psc(&self) -> u16;
    /// Writes the auto-reload value the counter rolls over at.
    fn set_car(&mut self, car: u16);
    /// Reads back the auto-reload value the counter rolls over at.
    fn read_car(&self) -> u16;
    /// Reads the counter, which advances on its own while the timer runs.
    fn read_cnt(&self) -> u16;
    /// Raises an update event in software, loading the shadowed dividers.
    fn gen_update(&mut self);
    /// Runs or halts the counter.
    fn set_cen(&mut self, on: bool);
    /// Restricts what counts as an update event to overflow/underflow — `UPG` no
    /// longer raises `UPIF`, only the counter rolling over does.
    fn set_ups(&mut self, on: bool);
    /// Lets the update event through to the NVIC. The flag is raised either way.
    fn set_upie(&mut self, on: bool);
    /// Whether the update event reaches the NVIC.
    fn read_upie(&self) -> bool;
    /// Update flag — set by hardware on every rollover, never cleared by it.
    fn read_upif(&self) -> bool;
    /// Clears the update flag.
    fn clear_upif(&mut self);
    /// Produces a second handle to the same peripheral.
    ///
    /// # Safety
    ///
    /// The HAL's typestate rests on one handle meaning one configuration in
    /// flight. Keeping two handles from contradicting each other is on the
    /// caller — disjoint registers, or no reconfiguration through both.
    unsafe fn steal(&self) -> Self;
}

macro_rules! timer_instance {
    ($($TIMERX:ty => $clk:ident,)+) => {
        $(
            impl Instance for $TIMERX {
                #[inline]
                fn clk(clocks: &Clocks) -> Hertz {
                    clocks.$clk()
                }
                #[inline]
                fn set_psc(&mut self, psc: u16) {
                    self.psc().write(|w| w.psc().bits(psc));
                }
                #[inline]
                fn read_psc(&self) -> u16 {
                    self.psc().read().psc().bits()
                }
                // The `CAR` writer is unsafe on TIMER2 only (unconstrained field
                // in the SVD), safe on the other six; one macro body serves all.
                // Every `u16` is a legal reload value in counting mode.
                #[allow(unused_unsafe)]
                #[inline]
                fn set_car(&mut self, car: u16) {
                    self.car().write(|w| unsafe { w.car().bits(car) });
                }
                #[inline]
                fn read_car(&self) -> u16 {
                    self.car().read().car().bits()
                }
                #[inline]
                fn read_cnt(&self) -> u16 {
                    self.cnt().read().cnt().bits()
                }
                #[inline]
                fn gen_update(&mut self) {
                    self.swevg().write(|w| w.upg().set_bit());
                }
                #[inline]
                fn set_cen(&mut self, on: bool) {
                    self.ctl0().modify(|_, w| w.cen().bit(on));
                }
                #[inline]
                fn set_ups(&mut self, on: bool) {
                    self.ctl0().modify(|_, w| w.ups().bit(on));
                }
                #[inline]
                fn set_upie(&mut self, on: bool) {
                    self.dmainten().modify(|_, w| w.upie().bit(on));
                }
                #[inline]
                fn read_upie(&self) -> bool {
                    self.dmainten().read().upie().bit_is_set()
                }
                #[inline]
                fn read_upif(&self) -> bool {
                    self.intf().read().upif().bit_is_set()
                }
                // In `INTF` zero clears and one leaves alone, and its reset value
                // is zero — so `write` would clear every flag it does not name.
                #[inline]
                fn clear_upif(&mut self) {
                    self.intf().modify(|_, w| w.upif().clear());
                }
                unsafe fn steal(&self) -> Self{
                    unsafe { Self::steal() }
                }
            }
        )+
    };
}

timer_instance! {
    pac::Timer2 => pclk1_tim,
    pac::Timer5 => pclk1_tim,
    pac::Timer13 => pclk1_tim,
    pac::Timer0 => pclk2_tim,
    pac::Timer15 => pclk2_tim,
    pac::Timer16 => pclk2_tim,
}

// TIMER14 is the one peripheral whose presence the flash code does not decide: it
// is absent from the 20- and 24-pin parts even at 64K (datasheet Table 1-1), so the
// gate comes from the part's own row in build.rs. Its every impl is gated, not just
// the pin table — the PAC has the register block regardless, and a timer needs no
// pins, so ungated it would configure silicon that isn't there.
#[cfg(has_timer14)]
timer_instance! {
    pac::Timer14 => pclk2_tim,
}

/// A stopped timer, holding the peripheral and the frequency feeding it.
///
/// Clocked and reset on construction, but the counter is not running: the
/// methods that wait on the count live on the running type, so waiting on a
/// timer that was never started cannot be expressed.
pub struct Timer<TIMERX> {
    timer: TIMERX,
    clk: Hertz,
}

impl<TIMERX: Instance> Timer<TIMERX> {
    /// Returns the peripheral.
    ///
    /// The clock is left enabled and no reset is performed — a later
    /// [`constrain`](TimerExt::constrain) does both anyway.
    pub fn release(self) -> TIMERX {
        self.timer
    }

    /// Starts the counter, which then rolls over every `car + 1` ticks of
    /// `clk / (psc + 1)`.
    ///
    /// The dividers reach the counter before it runs, so the first
    /// [`wait`](CountDownTimer::wait) already measures the full interval.
    pub fn start(mut self, psc: u16, car: u16) -> CountDownTimer<TIMERX> {
        start_counter(&mut self.timer, psc, car);
        CountDownTimer {
            timer: self.timer,
            clk: self.clk,
        }
    }
    /// Starts the counter, which then rolls over once per `interval`.
    ///
    /// Any scale works with no conversion at the call site — `5.secs()`,
    /// `500.millis()`, `100.micros()`. Truncating, so the realised interval is at
    /// most one prescaler cycle short. Intervals past what the 16-bit dividers
    /// span (just under a minute at 72 MHz) saturate; zero becomes one tick.
    pub fn start_interval<const NOM: u64, const DENOM: u64>(
        self,
        interval: Duration<u32, NOM, DENOM>,
    ) -> CountDownTimer<TIMERX> {
        let (psc, car) = dividers(interval_to_ticks(interval, self.clk));
        self.start(psc, car)
    }
    /// Hands the timer over to blocking delays; every [`delay`](Delay::delay)
    /// sets up its own interval and tears it down again.
    pub fn into_delay(self) -> Delay<TIMERX> {
        Delay {
            timer: self.timer,
            clk: self.clk,
        }
    }
    /// Hands the timer over to PWM and starts it on the given period.
    ///
    /// The counter cycle is shared by every channel, and `car + 1` is also their
    /// duty resolution: a larger reload buys finer steps at a lower frequency.
    pub fn into_pwm(mut self, psc: u16, car: u16) -> Pwm<TIMERX> {
        start_counter(&mut self.timer, psc, car);
        Pwm {
            timer: self.timer,
            clk: self.clk,
        }
    }
    /// Hands the timer over to PWM, taking the period as a duration.
    ///
    /// Splits it like [`start_interval`](Self::start_interval), keeping the
    /// reload large — here that is what leaves the duty the most steps.
    pub fn into_pwm_interval<const NOM: u64, const DENOM: u64>(
        self,
        interval: Duration<u32, NOM, DENOM>,
    ) -> Pwm<TIMERX> {
        let (psc, car) = dividers(interval_to_ticks(interval, self.clk));
        self.into_pwm(psc, car)
    }
    /// Sets the counter free running and hands out the input capture role.
    ///
    /// Only the prescaler is a choice: it trades resolution against the longest
    /// interval that still fits between two rollovers. Capture reads the counter
    /// as a clock rather than a period, so the reload is pinned to the maximum.
    pub fn into_capture(mut self, psc: u16) -> Capture<TIMERX> {
        start_counter(&mut self.timer, psc, u16::MAX);
        Capture {
            timer: self.timer,
            clk: self.clk,
        }
    }
}

/// A running timer, counting down the interval [`Timer::start`] was given.
///
/// Free-running: the counter reloads and starts over on every rollover, so the
/// interval repeats until the timer is stopped.
pub struct CountDownTimer<TIMERX> {
    timer: TIMERX,
    clk: Hertz,
}

impl<TIMERX: Instance> CountDownTimer<TIMERX> {
    /// Returns the current counter value, in timer ticks.
    ///
    /// Says where inside the current interval the timer is, not how many
    /// intervals passed. One tick is `psc() + 1` cycles of the timer clock.
    pub fn cnt(&self) -> u16 {
        self.timer.read_cnt()
    }
    /// Returns the value the counter rolls over at, in timer ticks.
    ///
    /// Read back from the hardware, so it holds for a raw
    /// [`start`](Timer::start) too. One interval is `car() + 1` ticks: `cnt()`
    /// reaching this value is the last tick before a rollover, not the rollover.
    pub fn car(&self) -> u16 {
        self.timer.read_car()
    }
    /// Returns the prescaler the counter is running on.
    ///
    /// One tick is `psc() + 1` cycles of the timer clock — what turns a
    /// [`cnt`](Self::cnt) reading into time. Read back from the hardware.
    pub fn psc(&self) -> u16 {
        self.timer.read_psc()
    }
    /// Returns how far into the current interval the counter is, as a duration.
    ///
    /// Restarts from zero at every rollover and never exceeds one interval;
    /// counting intervals is the caller's business. The scale comes from the
    /// binding (`let t: MillisDuration = timer.elapsed();`) — coarser than one
    /// tick floors to zero, finer than `u32` spans saturates. Resolution is one
    /// tick.
    pub fn elapsed<const NOM: u64, const DENOM: u64>(&self) -> Duration<u32, NOM, DENOM> {
        ticks_to_interval(self.cnt(), self.psc(), self.clk)
    }

    /// Lets `event` raise an interrupt.
    ///
    /// Half of what an interrupt takes: the request now reaches the NVIC, which
    /// still has the line masked. Unmasking it — `NVIC::unmask` on the
    /// peripheral's [`Interrupt`](crate::pac::Interrupt) — is the caller's, this
    /// crate does not touch core registers.
    pub fn listen(&mut self, event: Event) {
        set_listening(&mut self.timer, event, true);
    }
    /// Stops `event` from raising an interrupt. Leaves the NVIC alone.
    pub fn unlisten(&mut self, event: Event) {
        set_listening(&mut self.timer, event, false);
    }
    /// Whether `event` reaches the NVIC.
    pub fn is_listening(&self, event: Event) -> bool {
        is_listening(&self.timer, event)
    }
    /// Whether the flag `event` raises is standing.
    ///
    /// The enable and the flag are independent: this reports rollovers whether
    /// or not anything is listening for them.
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(&self.timer, event)
    }
    /// Clears the flag `event` raised.
    ///
    /// A handler must call this, first thing: the request is the flag being set,
    /// and hardware never clears it, so returning with it still set re-enters
    /// the handler at once and starves everything else.
    pub fn clear_interrupt(&mut self, event: Event) {
        clear_pending(&mut self.timer, event);
    }

    /// Blocks until the counter rolls over, then clears the update flag.
    ///
    /// Leaves the timer running, so calling this in a loop yields one full
    /// interval per call.
    pub fn wait(&mut self) {
        wait_update(&mut self.timer);
    }

    /// Halts the counter and hands the timer back in its stopped form.
    ///
    /// The counter keeps its current value; a later [`start`](Timer::start)
    /// reloads the dividers and restarts from zero.
    pub fn stop(mut self) -> Timer<TIMERX> {
        self.timer.set_cen(false);
        Timer {
            timer: self.timer,
            clk: self.clk,
        }
    }
    /// Halts the counter and returns the peripheral, skipping the stopped form.
    pub fn release(self) -> TIMERX {
        self.stop().timer
    }
}

/// A timer given over to blocking delays.
///
/// Unlike [`CountDownTimer`] it promises no interval: each call configures the
/// dividers, waits, and stops the counter again, so any length can be asked for
/// at any time.
pub struct Delay<TIMERX> {
    timer: TIMERX,
    clk: Hertz,
}

impl<TIMERX: Instance> Delay<TIMERX> {
    /// Blocks for `interval`, in whatever scale the caller wrote it in.
    ///
    /// Rounding and the saturation ceiling are the same as in
    /// [`Timer::start_interval`], the dividers being derived the same way.
    pub fn delay<const NOM: u64, const DENOM: u64>(&mut self, interval: Duration<u32, NOM, DENOM>) {
        let (psc, car) = dividers(interval_to_ticks(interval, self.clk));
        start_counter(&mut self.timer, psc, car);
        wait_update(&mut self.timer);
        self.timer.set_cen(false);
    }

    /// Takes the timer back out of delay duty, stopped and ready to be started.
    pub fn into_timer(self) -> Timer<TIMERX> {
        Timer {
            timer: self.timer,
            clk: self.clk,
        }
    }

    /// Returns the peripheral.
    ///
    /// The clock is left enabled and no reset is performed — a later `new()`
    /// does both anyway.
    pub fn release(self) -> TIMERX {
        self.timer
    }
}

/// A timer running as the period behind one or more PWM channels.
///
/// Owns the counter the channels compare against: channels carry their own duty,
/// the frequency they all share is set here.
pub struct Pwm<TIMERX> {
    timer: TIMERX,
    clk: Hertz,
}

impl<TIMERX: Instance> Pwm<TIMERX> {
    /// Configures one channel of this timer and hands it out on the given pin.
    ///
    /// Which channel it is follows from the pin — the silicon routes each pin to
    /// one channel of one timer. The channel comes out configured but not
    /// enabled, with no duty set, and holds the pin for as long as it lives.
    ///
    /// Several pins can reach one channel (`TIMER2` channel 0 answers on `PA6`
    /// and `PB4`), and handing over both yields two channels writing one compare
    /// register: same signal on both pins, and the last duty written wins.
    pub fn channel<PIN, const C: u8>(&mut self, pin: PIN) -> PwmChannel<TIMERX, PIN, C>
    where
        PIN: ChannelPin<TIMERX, C>,
        TIMERX: PwmOps<C>,
    {
        self.timer.apply_pwm_mode();
        PwmChannel {
            // Each channel reaches its own compare register and its own bits of
            // the shared ones — the obligation `steal` places on the caller.
            timer: unsafe { self.timer.steal() },
            pin,
        }
    }

    /// Lets `event` raise an interrupt.
    ///
    /// The counter's own event, shared by every channel: `Update` marks the
    /// period boundary, where the duties written since the last one take
    /// effect. Channels raise their own, through
    /// [`PwmChannel::listen`]. Unmasking the line in the NVIC is the caller's.
    pub fn listen(&mut self, event: Event) {
        set_listening(&mut self.timer, event, true);
    }
    /// Stops `event` from raising an interrupt. Leaves the NVIC alone.
    pub fn unlisten(&mut self, event: Event) {
        set_listening(&mut self.timer, event, false);
    }
    /// Whether `event` reaches the NVIC.
    pub fn is_listening(&self, event: Event) -> bool {
        is_listening(&self.timer, event)
    }
    /// Whether the flag `event` raises is standing.
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(&self.timer, event)
    }
    /// Clears the flag `event` raised, which hardware never does on its own.
    pub fn clear_interrupt(&mut self, event: Event) {
        clear_pending(&mut self.timer, event);
    }

    /// Changes the period without disturbing the running counter.
    ///
    /// Channels keep the duty they were given **in ticks**, so a new reload moves
    /// what fraction that is: half of a thousand ticks is all of five hundred.
    /// Set the duties again, or scale them by the new
    /// [`max_duty`](PwmChannel::max_duty). The prescaler reaches the counter at
    /// the next update event, so the cycle in flight runs on the old one.
    pub fn set_period(&mut self, psc: u16, car: u16) {
        self.timer.set_psc(psc);
        self.timer.set_car(car);
    }
    /// Changes the period, taking it as a duration.
    ///
    /// Splits it like [`into_pwm_interval`](Timer::into_pwm_interval).
    pub fn set_period_interval<const NOM: u64, const DENOM: u64>(
        &mut self,
        interval: Duration<u32, NOM, DENOM>,
    ) {
        let (psc, car) = dividers(interval_to_ticks(interval, self.clk));
        self.set_period(psc, car);
    }

    /// Halts the counter and takes the timer back out of PWM duty.
    ///
    /// Channels keep their stolen handles and pins, so they outlive this and go
    /// on reaching the registers; their outputs hold the level the counter
    /// stopped at.
    pub fn into_timer(mut self) -> Timer<TIMERX> {
        self.timer.set_cen(false);
        Timer {
            timer: self.timer,
            clk: self.clk,
        }
    }
    /// Halts the counter and returns the peripheral, skipping the stopped form.
    pub fn release(self) -> TIMERX {
        self.into_timer().timer
    }
}

impl<TIMERX> Pwm<TIMERX>
where
    TIMERX: PrimaryOutput,
{
    /// Lets the channels reach their pins.
    ///
    /// Timers with a `CCHP` register keep their outputs behind this switch, and
    /// it starts off — such a channel stays silent however it is configured until
    /// this is called. Timers without the register need nothing.
    pub fn enable_output(&mut self) {
        self.timer.set_poen(true);
    }
    /// Cuts every channel off from its pin at once, keeping them configured.
    pub fn disable_output(&mut self) {
        self.timer.set_poen(false);
    }
}

/// A free running timer whose channels timestamp edges on their pins.
///
/// The counter is only a time base: it runs to `u16::MAX` and wraps, while each
/// channel latches the count when its edge arrives. Intervals are differences
/// between latched values, so this role carries no period of its own.
pub struct Capture<TIMERX> {
    timer: TIMERX,
    clk: Hertz,
}

impl<TIMERX: Instance> Capture<TIMERX> {
    /// Points one channel of this timer at the given pin and hands it out.
    ///
    /// Which channel it is follows from the pin, as in [`Pwm::channel`]. The
    /// channel comes out configured but not enabled, latches nothing until it is,
    /// and holds the pin for as long as it lives. The edge is taken here so a
    /// channel is never half configured;
    /// [`select_edge`](CaptureChannel::select_edge) changes it later.
    pub fn channel<PIN, const C: u8>(
        &mut self,
        pin: PIN,
        edge: Edge,
    ) -> CaptureChannel<TIMERX, PIN, C>
    where
        PIN: ChannelPin<TIMERX, C>,
        TIMERX: CaptureOps<C>,
    {
        self.timer.apply_capture_mode();
        self.timer.select_edge(edge);
        CaptureChannel {
            // Each channel reaches its own capture register and its own bits of
            // the shared ones — the obligation `steal` places on the caller.
            timer: unsafe { self.timer.steal() },
            pin,
            clk: self.clk,
        }
    }

    /// Lets `event` raise an interrupt.
    ///
    /// The time base's own event, shared by every channel. `Update` is what
    /// extends the range here: the counter is only 16 bits wide, so an interval
    /// longer than one cycle needs the rollovers counted alongside the
    /// timestamps — [`interval`](CaptureChannel::interval) sees the latched
    /// values alone and cannot know how many cycles passed between them.
    /// Captures raise their own event, through [`CaptureChannel::listen`].
    /// Unmasking the line in the NVIC is the caller's.
    pub fn listen(&mut self, event: Event) {
        set_listening(&mut self.timer, event, true);
    }
    /// Stops `event` from raising an interrupt. Leaves the NVIC alone.
    pub fn unlisten(&mut self, event: Event) {
        set_listening(&mut self.timer, event, false);
    }
    /// Whether `event` reaches the NVIC.
    pub fn is_listening(&self, event: Event) -> bool {
        is_listening(&self.timer, event)
    }
    /// Whether the flag `event` raises is standing.
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(&self.timer, event)
    }
    /// Clears the flag `event` raised, which hardware never does on its own.
    pub fn clear_interrupt(&mut self, event: Event) {
        clear_pending(&mut self.timer, event);
    }

    /// Halts the counter and takes the timer back out of capture duty.
    ///
    /// Channels keep their stolen handles and pins and go on reaching the
    /// registers; they simply stop latching once the counter is halted.
    pub fn into_timer(mut self) -> Timer<TIMERX> {
        self.timer.set_cen(false);
        Timer {
            timer: self.timer,
            clk: self.clk,
        }
    }
    /// Halts the counter and returns the peripheral, skipping the stopped form.
    pub fn release(self) -> TIMERX {
        self.into_timer().timer
    }
}

/// Blocking delays for portable drivers, delegating to [`Delay::delay`].
///
/// Resolution is one timer tick — around 20 ns at 48 MHz — so a finer request is
/// served as a single tick. `delay_us`/`delay_ms` are the trait's own defaults,
/// splitting long waits into chunks of at most `u32::MAX` nanoseconds, well
/// inside what the dividers span.
impl<TIMERX: Instance> DelayNs for Delay<TIMERX> {
    fn delay_ns(&mut self, ns: u32) {
        self.delay(NanosDuration::from_nanos(ns));
    }
}

/// Entry point on the raw peripheral, mirroring `GpioExt` and `DmaExt`.
pub trait TimerExt: Sized {
    /// Clocks the peripheral, resets it, and records the clock feeding it.
    ///
    /// The counter is left stopped.
    fn constrain(self, rcu: &mut Rcu) -> Timer<Self>;
}

impl<TIMERX: Instance> TimerExt for TIMERX {
    fn constrain(self, rcu: &mut Rcu) -> Timer<Self> {
        let clocks = rcu.clocks();
        TIMERX::enable(rcu);
        TIMERX::reset(rcu);
        Timer {
            timer: self,
            clk: TIMERX::clk(&clocks),
        }
    }
}

/// The switch turning channel `C` on, whichever direction it points.
///
/// `CHxEN` serves both roles — releases the output on a compare channel, arms
/// the latch on a capture one — so it sits above them. Implemented only for
/// timer/channel pairs that exist, so a channel the hardware lacks cannot be
/// named: `TIMER13` has channel 0 alone, `TIMER5` none at all.
pub trait ChannelEnable<const C: u8>: Instance {
    /// Enables or disables the channel, leaving its setup in place.
    fn set_chxen(&mut self, on: bool);
    /// Lets this channel's event through to the NVIC. The flag is raised either
    /// way, and every channel shares the timer's one interrupt line.
    fn set_chxie(&mut self, on: bool);
    /// Whether this channel's event reaches the NVIC.
    fn read_chxie(&self) -> bool;
    /// Channel flag — a capture latched or a compare matched, depending on the
    /// role the channel is in. Set by hardware, never cleared by it.
    fn read_chxif(&self) -> bool;
    /// Clears the channel flag.
    fn clear_chxif(&mut self);
}

macro_rules! channel_enable {
    {$($Timer:ty => [$($Ch:literal$(,)?)+]$(,)?)+} => {
        paste::paste! { $($(impl ChannelEnable<$Ch> for $Timer {
            fn set_chxen(&mut self, on: bool) {
                self.chctl2().modify(|_, w| match on {
                    true => w.[<ch $Ch en>]().enabled(),
                    false => w.[<ch $Ch en>]().disabled(),
                })
            }
            fn set_chxie(&mut self, on: bool) {
                self.dmainten().modify(|_, w| w.[<ch $Ch ie>]().bit(on));
            }
            fn read_chxie(&self) -> bool {
                self.dmainten().read().[<ch $Ch ie>]().bit_is_set()
            }
            fn read_chxif(&self) -> bool {
                self.intf().read().[<ch $Ch if>]().bit_is_set()
            }
            // `INTF` is rc_w0, as for `UPIF`: `write` would clear the flags of
            // every other channel along with this one.
            fn clear_chxif(&mut self) {
                self.intf().modify(|_, w| w.[<ch $Ch if>]().clear())
            }
        })+)+ }
    };
}

channel_enable! {
    pac::Timer0 => [0, 1, 2, 3],
    pac::Timer2 => [0, 1, 2, 3],
    pac::Timer13 => [0],
    pac::Timer15 => [0],
    pac::Timer16 => [0]
}

#[cfg(has_timer14)]
channel_enable! {
    pac::Timer14 => [0, 1]
}

/// Register operations on compare channel `C` of a timer.
///
/// Channel registers differ from one channel to the next, which is why the
/// number lives in the type rather than in an argument.
pub trait PwmOps<const C: u8>: ChannelEnable<C> {
    /// Configures the channel as a PWM output and readies it for a duty value.
    ///
    /// Covers direction, compare mode and polarity at once: a channel in PWM mode
    /// while still pointed at its input drives nothing.
    fn apply_pwm_mode(&mut self);
    /// Writes the compare value the channel switches its output at.
    ///
    /// The output stays active while the counter is below this value, so one
    /// full period is `car() + 1` and duty is the ratio between the two.
    fn set_chxcv(&mut self, cv: u16);
}

macro_rules! pwm {
    {$($Timer:ty => [
        $(
            ($Ch:literal, $chctl_reg:ident)$(,)?
        )+]$(,)?)+
    } => {
        paste::paste! { $($(impl PwmOps<$Ch> for $Timer {
            fn apply_pwm_mode(&mut self) {
                self.chctl2().modify(|_, w| w.[<ch $Ch p>]().not_inverted());
                self.$chctl_reg().modify(|_, w| {
                    w.[<ch $Ch ms>]()
                        .output()
                        .[<ch $Ch comctl>]()
                        .pwm_mode0()
                        .[<ch $Ch comsen>]()
                        .enabled()
                });
            }
            #[allow(unused_unsafe)]
            fn set_chxcv(&mut self, cv: u16) {
                self.[<ch $Ch cv>]().write(|w| unsafe { w.[<ch $Ch val>]().bits(cv) });
            }
        })+)+ }
    };
}

// Channels 0 and 1 live in `CHCTL0`, 2 and 3 in `CHCTL1`, which is the one name
// that does not follow from the channel number.
pwm! {
    pac::Timer0 => [(0, chctl0_output), (1, chctl0_output),
        (2, chctl1_output), (3, chctl1_output)],
    pac::Timer2 => [(0, chctl0_output), (1, chctl0_output),
        (2, chctl1_output), (3, chctl1_output)],
    pac::Timer13 => [(0, chctl0_output)],
    pac::Timer15 => [(0, chctl0_output)],
    pac::Timer16 => [(0, chctl0_output)]
}

#[cfg(has_timer14)]
pwm! {
    pac::Timer14 => [(0, chctl0_output), (1, chctl0_output)]
}

/// The output switch shared by every channel of a timer that has one.
///
/// Only the timers carrying a `CCHP` register implement this: `TIMER0`,
/// `TIMER15`, `TIMER16`, and `TIMER14` where the part has it. It sits above the
/// per-channel enables —
/// with it off, the channels stay configured and the counter keeps running
/// while the pins show nothing.
pub trait PrimaryOutput: Instance {
    /// Enables or disables the outputs of all channels at once.
    fn set_poen(&mut self, on: bool);
}

macro_rules! poen {
    ($($Timer:ty$(,)?)+) => {
        $(impl PrimaryOutput for $Timer {
            fn set_poen(&mut self, on: bool) {
                self.cchp().modify(|_, w| match on {
                    true => w.poen().enabled(),
                    false => w.poen().disabled(),
                });
            }
        })+
    };
}

poen!(pac::Timer0, pac::Timer15, pac::Timer16);
#[cfg(has_timer14)]
poen!(pac::Timer14);

/// Marks a pin the silicon routes to channel `C` of `TIMERX`, in the right
/// alternate function.
pub trait ChannelPin<TIMERX, const C: u8> {}

macro_rules! channel_pins {
    ( $( $TIMERX:ty: $( $C:literal: [ $($(#[$cfg:meta])? $p:literal $n:literal : $af:literal),* $(,)? ] )* ),* $(,)? ) => {
        $($($( $(#[$cfg])? impl ChannelPin<$TIMERX, $C> for Pin<$p, $n, Alternate<$af>> {} )*)*)*
    };
}

// Complementary outputs (`CHx_ON`), break inputs and `ETI` share these same
// pins at other alternate functions; only the plain compare outputs are listed.
//
// The `pads_ge_*` gates say the package bonds the pin at all, and match the ones in
// `gpio::Parts`.
channel_pins! {
    pac::Timer0:
        0: [ #[cfg(pads_ge_24)] 'A' 8:2 ]
        1: [ 'A' 9:2 ]
        2: [ 'A' 10:2 ]
        3: [ #[cfg(pads_ge_lqfp32)] 'A' 11:2 ],
    pac::Timer2:
        0: [ 'A' 6:1, #[cfg(pads_ge_28)] 'B' 4:1 ]
        1: [ 'A' 7:1, #[cfg(pads_ge_28)] 'B' 5:1 ]
        2: [ #[cfg(pads_ge_24)] 'B' 0:1 ]
        3: [ 'B' 1:1 ],
    pac::Timer13:
        0: [ 'A' 4:4, 'A' 7:4, 'B' 1:2 ],
    pac::Timer15:
        0: [ 'A' 6:5, #[cfg(pads_ge_qfn32)] 'B' 8:2 ],
    pac::Timer16:
        0: [ 'A' 7:5, #[cfg(pads_ge_48)] 'B' 9:2 ],
}

#[cfg(has_timer14)]
channel_pins! {
    pac::Timer14:
        0: [ 'A' 2:0, #[cfg(pads_ge_48)] 'B' 14:1 ]
        1: [ 'A' 3:0, #[cfg(pads_ge_48)] 'B' 15:1 ],
}

/// One PWM output of a timer, configured and ready to take a duty value.
///
/// Channels of one timer share the period (`PSC`/`CAR` belong to their common
/// counter); duty is the channel's own.
///
/// Each channel carries its own handle to the peripheral, which is what lets four
/// of them exist while the timer is a single value. Duties reach separate
/// registers, but all the enables live in one: enabling a channel while another
/// is enabled elsewhere — from an interrupt, say — can lose one of the writes.
pub struct PwmChannel<TIMERX, PIN, const C: u8> {
    timer: TIMERX,
    pin: PIN,
}

impl<TIMERX: PwmOps<C>, PIN: ChannelPin<TIMERX, C>, const C: u8> PwmChannel<TIMERX, PIN, C> {
    /// Drives the pin from the comparison, keeping the duty already set.
    pub fn enable(&mut self) {
        self.timer.set_chxen(true);
    }
    /// Stops driving the pin, leaving the channel configured.
    pub fn disable(&mut self) {
        self.timer.set_chxen(false);
    }

    /// Sets the duty, in timer ticks of the period.
    ///
    /// Active for `cv` ticks out of [`max_duty`](Self::max_duty): zero is a
    /// permanently inactive pin, `max_duty` and above a permanently active one.
    /// The value reaches the output at the next rollover, not mid-period.
    pub fn set_duty(&mut self, cv: u16) {
        self.timer.set_chxcv(cv);
    }
    /// Returns the duty that corresponds to a fully active output.
    ///
    /// The period the counter runs on, `car() + 1` ticks, which is also the
    /// resolution. A period spanning the whole counter reports one tick short,
    /// being the only value that does not fit `u16`.
    pub fn max_duty(&self) -> u16 {
        self.timer.read_car().saturating_add(1)
    }

    /// Lets the compare match raise an interrupt.
    ///
    /// Fires at the point in the period where the output flips, not at the
    /// rollover — that one is [`CountDownTimer::listen`]. Takes no event: a
    /// channel has exactly one, and which it means is already in the type.
    ///
    /// The whole timer shares one NVIC line, so a handler serving several
    /// channels tells them apart by [`is_listening`](Self::is_listening) and
    /// [`is_pending`](Self::is_pending) together. The flag alone will not do:
    /// after reset an untouched channel compares against zero and raises its
    /// flag once per rollover. Unmasking the line is the caller's.
    pub fn listen(&mut self) {
        self.timer.set_chxie(true);
    }
    /// Stops the compare match from raising an interrupt. Leaves the NVIC alone.
    pub fn unlisten(&mut self) {
        self.timer.set_chxie(false);
    }
    /// Whether this channel's compare match reaches the NVIC.
    pub fn is_listening(&self) -> bool {
        self.timer.read_chxie()
    }
    /// Whether this channel's flag is raised.
    pub fn is_pending(&self) -> bool {
        self.timer.read_chxif()
    }
    /// Clears this channel's flag.
    ///
    /// A handler must call this: a compare match yields no value to consume, so
    /// unlike a capture there is nothing else that acknowledges it, and hardware
    /// never clears the flag on its own.
    pub fn clear_interrupt(&mut self) {
        self.timer.clear_chxif();
    }

    /// Gives the pin back, dropping the channel.
    ///
    /// The output is left as it was, still driving its duty; call
    /// [`disable`](Self::disable) first if the pin is wanted quiet.
    pub fn release(self) -> PIN {
        self.pin
    }
}

/// What can go wrong while reading a capture.
///
/// Its own type rather than a bare `Option`: a missing capture and a lost one are
/// different answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// An edge landed on a value that had not been read yet (`CHxOF`).
    ///
    /// The register holds the newer timestamp and the older one is gone, so an
    /// interval measured across this is wrong rather than merely late.
    Overcapture,
}

/// One capture input of a timer, configured and ready to be armed.
///
/// Channels of one timer share the time base (`PSC` belongs to their common
/// counter); the edge and the latched value are the channel's own.
///
/// Each channel carries its own handle to the peripheral, which is what lets four
/// of them exist while the timer is a single value. Latching reaches separate
/// registers, but all the enables live in one: enabling a channel while another
/// is enabled elsewhere — from an interrupt, say — can lose one of the writes.
pub struct CaptureChannel<TIMERX, PIN, const C: u8> {
    timer: TIMERX,
    pin: PIN,
    clk: Hertz,
}

impl<TIMERX: CaptureOps<C>, PIN: ChannelPin<TIMERX, C>, const C: u8>
    CaptureChannel<TIMERX, PIN, C>
{
    /// Arms the channel, from here on latching the counter on every edge.
    pub fn enable(&mut self) {
        self.timer.set_chxen(true);
    }
    /// Stops latching, leaving the channel configured and its last value intact.
    pub fn disable(&mut self) {
        self.timer.set_chxen(false);
    }

    /// Changes which edge the channel latches on.
    ///
    /// Takes effect immediately, so an edge arriving during the switch belongs to
    /// whichever setting won the race; discard measurements spanning the change.
    pub fn select_edge(&mut self, edge: Edge) {
        self.timer.select_edge(edge);
    }

    /// Lets a capture raise an interrupt.
    ///
    /// Turns waiting for an edge from a busy loop into a wake-up, and shrinks
    /// the window in which the next edge could overwrite an unread timestamp —
    /// see [`Error::Overcapture`]. Takes no event: a channel has exactly one,
    /// and which it means is already in the type.
    ///
    /// No separate clear: [`read`](Self::read) takes the flag down, being the
    /// same call a handler makes to collect the timestamp. The whole timer
    /// shares one NVIC line, so a handler serving several channels tells them
    /// apart by [`is_listening`](Self::is_listening) and
    /// [`is_pending`](Self::is_pending) together — the flag alone will not do,
    /// since an untouched channel still compares against zero after reset and
    /// raises its flag once per rollover. Unmasking the line is the caller's.
    pub fn listen(&mut self) {
        self.timer.set_chxie(true);
    }
    /// Stops captures from raising an interrupt. Leaves the NVIC alone.
    pub fn unlisten(&mut self) {
        self.timer.set_chxie(false);
    }
    /// Whether this channel's captures reach the NVIC.
    pub fn is_listening(&self) -> bool {
        self.timer.read_chxie()
    }
    /// Whether a timestamp is waiting to be read.
    pub fn is_pending(&self) -> bool {
        self.timer.read_chxif()
    }

    /// Takes the timestamp of the last edge, if one has arrived.
    ///
    /// Returns [`WouldBlock`](nb::Error::WouldBlock) while no edge has been
    /// latched since the previous read, so it can be polled or waited on with
    /// [`nb::block!`]. Flags are cleared on the way out. The value is the counter
    /// at the moment of the edge, not an interval — those come from feeding two
    /// of these to [`interval`](Self::interval).
    ///
    /// # Errors
    ///
    /// [`Error::Overcapture`] when a further edge arrived before this read: the
    /// timestamp belongs to the later edge, the earlier one is lost. The next
    /// read is sound again.
    pub fn read(&mut self) -> nb::Result<u16, Error> {
        if !self.timer.read_chxif() {
            return Err(nb::Error::WouldBlock);
        }
        let cv = self.timer.read_chxcv();
        let lost = self.timer.read_chxof();
        // Hardware only raises these flags; left standing, `CHxIF` would report
        // the same edge for ever.
        self.timer.clear_chxif();
        match lost {
            true => {
                self.timer.clear_chxof();
                Err(nb::Error::Other(Error::Overcapture))
            }
            false => Ok(cv),
        }
    }

    /// Converts the span between two timestamps into a duration.
    ///
    /// Takes them in capture order and counts forward from the first, so a
    /// rollover in between costs nothing — the subtraction wraps exactly as the
    /// counter does. Spans longer than one counter cycle come back wrong; keep
    /// the prescaler large enough that the signal fits. The scale comes from the
    /// binding, as in [`elapsed`](CountDownTimer::elapsed).
    pub fn interval<const NOM: u64, const DENOM: u64>(
        &self,
        from: u16,
        to: u16,
    ) -> Duration<u32, NOM, DENOM> {
        ticks_to_interval(to.wrapping_sub(from), self.timer.read_psc(), self.clk)
    }

    /// Gives the pin back, dropping the channel.
    ///
    /// Left as it was, still armed if it was armed; call
    /// [`disable`](Self::disable) first if it should stop latching.
    pub fn release(self) -> PIN {
        self.pin
    }
}

/// Writing a duty cannot fail: the value goes straight into a compare register
/// the hardware always accepts.
impl<TIMERX: PwmOps<C>, PIN: ChannelPin<TIMERX, C>, const C: u8> ErrorType
    for PwmChannel<TIMERX, PIN, C>
{
    type Error = Infallible;
}

/// Duty control for portable drivers, delegating to [`PwmChannel::set_duty`].
///
/// `set_duty_cycle_percent` and friends are the trait's own defaults over these
/// two, scaled against the period the timer currently runs on.
impl<TIMERX: PwmOps<C>, PIN: ChannelPin<TIMERX, C>, const C: u8> SetDutyCycle
    for PwmChannel<TIMERX, PIN, C>
{
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(duty);
        Ok(())
    }
}

/// Which edge on the pin makes a channel take its snapshot.
///
/// A runtime value rather than a typestate: no method signature depends on it.
/// Capturing on both edges is not offered — that encoding is reserved on every
/// timer of this part.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum Edge {
    Rising,
    Falling,
}

/// The input capture half of one channel, numbered by `C` like [`PwmOps`].
///
/// Mirrors the output side, with the register differences between timers and
/// channels kept inside the implementations.
pub trait CaptureOps<const C: u8>: ChannelEnable<C> {
    /// Points the channel at its pin and readies it to latch the counter.
    ///
    /// Covers direction, input filter and capture prescaler at once: a channel
    /// still pointing at its comparator captures nothing.
    fn apply_capture_mode(&mut self);
    /// Chooses the edge the channel takes its snapshot on.
    fn select_edge(&mut self, edge: Edge);
    /// Reads the counter value the last edge latched.
    ///
    /// Says nothing about whether an edge arrived — the value means something
    /// only once [`read_chxif`](ChannelEnable::read_chxif) confirms a capture.
    fn read_chxcv(&self) -> u16;
    /// Overcapture flag — set when an edge lands on a value not yet read.
    fn read_chxof(&self) -> bool;
    /// Clears the overcapture flag.
    fn clear_chxof(&mut self);
}

macro_rules! capture {
    {$($Timer:ty => [
        $(
            ($Ch:literal, $chctl_reg:ident $(, $chnp:ident)?)$(,)?
        )+]$(,)?)+
    } => {
        paste::paste! { $($(impl CaptureOps<$Ch> for $Timer {
            fn apply_capture_mode(&mut self) {
                // `CHxNP` exists only on channels with a complementary output;
                // channel 3 has neither.
                $(self.chctl2().modify(|_, w| w.$chnp().not_inverted());)?
                // `ci0` is the channel's own input on every channel — the PAC
                // names the variants relative to the channel, not by line number.
                self.$chctl_reg().modify(|_, w| {
                    w.[<ch $Ch ms>]()
                        .ci0()
                        .[<ch $Ch capflt>]()
                        .no_filter()
                        .[<ch $Ch cappsc>]()
                        .div1()
                })
            }
            fn select_edge(&mut self, edge: Edge) {
                self.chctl2().modify(|_, w| match edge {
                    Edge::Rising => w.[<ch $Ch p>]().not_inverted(),
                    Edge::Falling => w.[<ch $Ch p>]().inverted()
                })
            }
            fn read_chxcv(&self) -> u16 {
                self.[<ch $Ch cv>]().read().[<ch $Ch val>]().bits()
            }
            fn read_chxof(&self) -> bool {
                self.intf().read().[<ch $Ch of>]().bit_is_set()
            }
            fn clear_chxof(&mut self) {
                self.intf().modify(|_, w| w.[<ch $Ch of>]().clear())
            }
        })+)+ }
    };
}

// As in `pwm!`, the `CHCTL0`/`CHCTL1` split is the one name the channel number
// does not give. `CHxNP` is passed where the channel has one; channel 3 does not.
capture! {
    pac::Timer0 => [(0, chctl0_input, ch0np), (1, chctl0_input, ch1np),
        (2, chctl1_input, ch2np), (3, chctl1_input)],
    pac::Timer2 => [(0, chctl0_input, ch0np), (1, chctl0_input, ch1np),
        (2, chctl1_input, ch2np), (3, chctl1_input)],
    pac::Timer13 => [(0, chctl0_input, ch0np)],
    pac::Timer15 => [(0, chctl0_input, ch0np)],
    pac::Timer16 => [(0, chctl0_input, ch0np)]
}

#[cfg(has_timer14)]
capture! {
    pac::Timer14 => [(0, chctl0_input, ch0np), (1, chctl0_input, ch1np)]
}
