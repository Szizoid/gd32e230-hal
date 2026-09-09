//! Hardware abstraction layer for the GD32E23x series (Cortex-M23), built on top
//! of the [`gd32e2`] peripheral access crate. Only the GD32E230 is implemented so
//! far; the crate is named for the family it is meant to grow into.
//!
//! The API leans on the type system: a pin's port, number and mode live in its
//! type, so an invalid alternate function or a method that makes no sense for the
//! current mode fails to compile rather than misbehaving on the board.
//!
//! # Chip variants
//!
//! One feature names the part being built for, and exactly one must be enabled;
//! zero or several is an error rather than a silently wrong pin map. There is no
//! default: which part sits on a board is not something this crate can assume.
//!
//! A feature is the part number with an `x` in each field the code cannot see, so
//! `gd32e230g8xx` is every G8 part: the letter is the bonded pin count (F 20, E 24,
//! G 28, K 32, C 48), the digit the flash code (4 = 16K flash and 4K SRAM, 6 = 32K
//! and 6K, 8 = 64K and 8K), and the last `x` the temperature grade. Only the 32-pin
//! parts spell their package out — a QFN32 (`gd32e230k8ux`) carries VSS on its
//! thermal pad and gives the two freed pins to `PB2` and `PB8`, an LQFP32
//! (`gd32e230k8tx`) does not.
//!
//! Every named field reaches the code. The flash code decides the
//! alternate-function map, where the same pin at the same AF number can reach a
//! different peripheral (`PA2` AF1 is `USART0_TX` on x4 but `USART1_TX` on x8);
//! the bonded pads decide which pins exist at all. `build.rs` also writes the
//! `memory.x` the linker needs, so a project using this HAL does not supply one —
//! though a `memory.x` in its own root still takes precedence.
//!
//! # Getting started
//!
//! Clocks come first: freezing the tree is what produces the [`Rcu`](rcu::Rcu)
//! every other module takes, and it enables each peripheral's clock as that
//! peripheral is constructed.
//!
//! ```ignore
//! let dp = pac::Peripherals::take().unwrap();
//! let mut fmc = dp.fmc.constrain();
//! let config = ClockConfig::default().sysclk(SysClk::Pll(PllFreq::Mhz48));
//! let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);
//! let parts = dp.gpioa.split(&mut rcu);
//! let mut led = parts.pa5.into_output();
//! led.set_high().unwrap();
//! ```

#![no_std]
#![warn(missing_docs)]

// Both halves of the choice are checked in build.rs, which counts the enabled
// features and knows their names. This guards the source against being compiled
// with no build script at all — the flags below are what every gate reads.
#[cfg(not(any(chip_x4, chip_x6, chip_x8)))]
compile_error!("select a chip: enable exactly one of the `gd32e230*` features");

/// Re-export of the peripheral access crate this HAL is built on. Referring to
/// it as `pac` keeps the door open to other parts of the family behind a single
/// alias, instead of naming a specific chip module throughout.
pub use gd32e2::gd32e230 as pac;

pub mod adc;
pub mod cmp;
pub mod crc;
pub mod dma;
pub mod exti;
pub mod fmc;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod rcu;
pub mod spi;
pub mod syscfg;
pub mod time;
pub mod timer;
pub mod usart;
pub mod watchdog;
