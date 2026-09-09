//! I2C0 driving its transfers from the interrupt handlers, over RTT.
//!
//! **Wiring: PB7 to the target's SDA, PB6 to its SCL, grounds tied together.**
//! The bench is an RP2040 in I²C target mode at address 0x42, holding an
//! eight-byte register file whose first written byte is the index. There are no
//! external pull-ups, so the bus runs at 50 kHz on the internal ones.
//!
//! Three transactions, none of them blocking on the bus: write index and value,
//! write the index alone to rewind the pointer, then read four bytes back. Each
//! one is handed to the statics, driven to completion by the handlers, and taken
//! apart with `release`.
//!
//! The target keeps its register file across resets of this board, so the values
//! read back shift after a write — that is the bench, not a fault.
//!
//! Covers: `I2c::start_write`/`start_read`, `WriteTransfer`/`ReadTransfer`,
//! `on_interrupt` entered from both the event and the error vector.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::Mutex;
use defmt_rtt as _;
use panic_halt as _;

use gd32e2_hal::gpio::{Alternate, OpenDrain, Pin};
use gd32e2_hal::i2c::{Error, I2c, I2cMode, ReadTransfer, WriteTransfer};
use gd32e2_hal::pac::{self, interrupt};
use gd32e2_hal::prelude::*;
use gd32e2_hal::rcu::{ClockConfig, PllFreq, SysClk};

type Sda = Pin<'B', 7, Alternate<1, OpenDrain>>;
type Scl = Pin<'B', 6, Alternate<1, OpenDrain>>;
type Bus = I2c<pac::I2c0, Sda, Scl>;
type Write = WriteTransfer<pac::I2c0, Sda, Scl>;
type Read = ReadTransfer<pac::I2c0, Sda, Scl>;

static WRITE: Mutex<RefCell<Option<Write>>> = Mutex::new(RefCell::new(None));
static READ: Mutex<RefCell<Option<Read>>> = Mutex::new(RefCell::new(None));

const ADDRESS: u8 = 0x42;
const REGISTER: u8 = 0;
const VALUE: u8 = 0xA5;

static SET_REGISTER: [u8; 2] = [REGISTER, VALUE];
static REWIND: [u8; 1] = [REGISTER];

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut fmc = dp.fmc.constrain();
    let config = ClockConfig::default().sysclk(SysClk::Pll(PllFreq::Mhz48));
    let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);

    let gpiob = dp.gpiob.split(&mut rcu);
    let sda = gpiob.pb7.into_alternate_open_drain::<1>();
    let scl = gpiob.pb6.into_alternate_open_drain::<1>();
    let bus = I2c::new(&mut rcu, dp.i2c0, sda, scl, I2cMode::standard(50.kHz()));

    // Both vectors: `on_interrupt` reads the flags itself, so either may wake it.
    unsafe {
        NVIC::unmask(pac::Interrupt::I2C0_EV);
        NVIC::unmask(pac::Interrupt::I2C0_ER);
    };

    let read_buf = cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap();

    let bus = run_write(bus.start_write(ADDRESS, &SET_REGISTER), "set register");
    let bus = run_write(bus.start_write(ADDRESS, &REWIND), "rewind pointer");

    let transfer = bus.start_read(ADDRESS, read_buf);
    critical_section::with(|cs| READ.borrow(cs).replace(Some(transfer)));
    while !critical_section::with(|cs| {
        READ.borrow(cs)
            .borrow()
            .as_ref()
            .is_some_and(|transfer| transfer.is_done())
    }) {
        cortex_m::asm::wfi();
    }
    let transfer = critical_section::with(|cs| READ.borrow(cs).take()).unwrap();
    let (_bus, buf, outcome) = transfer.release();
    report("read", outcome);
    defmt::info!("read back {=[u8]:#04x}", buf);

    loop {
        cortex_m::asm::wfi();
    }
}

/// Hands a write to the handlers, sleeps until it is over, and gives the
/// peripheral back.
fn run_write(transfer: Write, what: &str) -> Bus {
    critical_section::with(|cs| WRITE.borrow(cs).replace(Some(transfer)));
    while !critical_section::with(|cs| {
        WRITE
            .borrow(cs)
            .borrow()
            .as_ref()
            .is_some_and(|transfer| transfer.is_done())
    }) {
        cortex_m::asm::wfi();
    }
    let transfer = critical_section::with(|cs| WRITE.borrow(cs).take()).unwrap();
    let (bus, _buf, outcome) = transfer.release();
    report(what, outcome);
    bus
}

fn report(what: &str, outcome: Option<Result<(), Error>>) {
    match outcome {
        Some(Ok(())) => defmt::info!("{} ok", what),
        Some(Err(err)) => defmt::error!("{} failed: {}", what, err),
        None => defmt::error!("{} was taken apart while still running", what),
    }
}

#[interrupt]
fn I2C0_EV() {
    service();
}

#[interrupt]
fn I2C0_ER() {
    service();
}

/// Advances whichever transfer is in flight; at most one ever is.
fn service() {
    critical_section::with(|cs| {
        if let Some(transfer) = WRITE.borrow(cs).borrow_mut().as_mut() {
            transfer.on_interrupt();
        }
        if let Some(transfer) = READ.borrow(cs).borrow_mut().as_mut() {
            transfer.on_interrupt();
        }
    })
}
