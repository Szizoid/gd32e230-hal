//! EXTI line 0 catching the edges this code makes itself, over RTT.
//!
//! No wiring: PA0 is an output, the line watches it, and `toggle` through the
//! pin the line holds is what raises the interrupt. Three parts — edges on a
//! rising-only line, a line with no edge selected raised from software, and a
//! line switched to the event output, which raises neither the handler nor
//! `PD`.
//!
//! Lines 0 and 1 share the `EXTI0_1` vector, so the handler asks the line
//! whether it is the pending one before clearing it.
//!
//! Covers: `ExtiExt::split`, `ExtiLine::source`, `edge`, `listen`/`unlisten`,
//! `listen_event`, `pend`, `is_pending`, `clear_interrupt`, `pin_mut`,
//! `release`.

#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::Mutex;
use defmt_rtt as _;
use panic_halt as _;

use gd32e2_hal::exti::{EdgeTrigger, ExtiLine};
use gd32e2_hal::gpio::{Output, Pin, PushPull};
use gd32e2_hal::pac::{self, interrupt};
use gd32e2_hal::prelude::*;
use gd32e2_hal::rcu::{ClockConfig, PllFreq, SysClk};

type Line = ExtiLine<0, Pin<'A', 0, Output<PushPull>>>;

static LINE: Mutex<RefCell<Option<Line>>> = Mutex::new(RefCell::new(None));
static COUNT: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// Long enough for the edge to be caught and the handler to run.
const SETTLE: u32 = 4_800;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut fmc = dp.fmc.constrain();
    let config = ClockConfig::default().sysclk(SysClk::Pll(PllFreq::Mhz48));
    let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);
    let mut syscfg = dp.syscfg.constrain(&mut rcu);

    let gpioa = dp.gpioa.split(&mut rcu);
    let pa0 = gpioa.pa0.into_push_pull_output();

    let lines = dp.exti.split();
    let mut line = lines.line0.source(&mut syscfg, pa0);
    line.edge(EdgeTrigger::Rising);
    line.listen();
    critical_section::with(|cs| LINE.borrow(cs).replace(Some(line)));
    unsafe { NVIC::unmask(pac::Interrupt::EXTI0_1) };

    // Rising only: two toggles per round, one edge each way, one interrupt.
    for round in 1..=3 {
        toggle();
        toggle();
        defmt::info!("round {}: {} interrupts", round, count());
    }

    // No edge selected, so only software can raise the line.
    with_line(|line| line.edge(EdgeTrigger::None));
    toggle();
    defmt::info!("after a toggle with no edge selected: {}", count());
    with_line(|line| line.pend());
    cortex_m::asm::delay(SETTLE);
    defmt::info!("after pend: {}", count());

    // Event output alone: nothing reaches the handler, and the flag stays
    // clear — the manual does not say so, this is what the board does.
    with_line(|line| {
        line.unlisten();
        line.listen_event();
        line.edge(EdgeTrigger::Rising);
    });
    toggle();
    toggle();
    let pending = with_line(|line| line.is_pending());
    defmt::info!(
        "event output: {} interrupts, PD {}",
        count(),
        if pending { "latched" } else { "clear" }
    );

    let line = critical_section::with(|cs| LINE.borrow(cs).take()).unwrap();
    let (_line, mut pa0) = line.release();
    pa0.set_low();
    defmt::info!("done");

    loop {
        cortex_m::asm::wfi();
    }
}

/// Drives one edge on the pin the line is watching.
fn toggle() {
    with_line(|line| line.pin_mut().toggle());
    cortex_m::asm::delay(SETTLE);
}

/// Borrows the line out of the static for the length of one call.
fn with_line<R>(f: impl FnOnce(&mut Line) -> R) -> R {
    critical_section::with(|cs| f(LINE.borrow(cs).borrow_mut().as_mut().unwrap()))
}

fn count() -> u32 {
    critical_section::with(|cs| COUNT.borrow(cs).get())
}

#[interrupt]
fn EXTI0_1() {
    critical_section::with(|cs| {
        if let Some(line) = LINE.borrow(cs).borrow_mut().as_mut() {
            // The vector is shared with line 1, so being here is not proof.
            if line.is_pending() {
                line.clear_interrupt();
                let count = COUNT.borrow(cs);
                count.set(count.get() + 1);
            }
        }
    });
}
