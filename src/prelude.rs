//! One import for the traits whose methods this HAL is used through.
//!
//! Traits are re-exported anonymously (`as _`), so the methods arrive without the
//! names and nothing can collide with the user's own items. Types are not
//! re-exported — import those from their own modules.
//!
//! ```ignore
//! use gd32e2_hal::prelude::*;          // everything
//! use gd32e2_hal::prelude::gpio::*;    // or one peripheral at a time
//! ```
//!
//! The glob at this level covers every peripheral and takes [`usart::io`] for the
//! serial traits. [`usart::nb`] is the other choice, never both: their `read` and
//! `write` land on the same type, and two traits of one name in scope make every
//! such call ambiguous (`E0034`).

/// ADC entry point.
pub mod adc {
    pub use crate::adc::AdcExt as _;
}

/// DMA entry point.
pub mod dma {
    pub use crate::dma::DmaExt as _;
}

/// EXTI entry point.
pub mod exti {
    pub use crate::exti::ExtiExt as _;
}

/// Flash controller entry point.
pub mod fmc {
    pub use crate::fmc::FmcExt as _;
}

/// Port entry point and pin state.
pub mod gpio {
    pub use crate::gpio::GpioExt as _;
    pub use embedded_hal::digital::{InputPin as _, OutputPin as _, StatefulOutputPin as _};
}

/// I²C transactions.
pub mod i2c {
    pub use embedded_hal::i2c::I2c as _;
}

/// Clock tree entry point.
pub mod rcu {
    pub use crate::rcu::RcuExt as _;
}

/// SPI transfers.
pub mod spi {
    pub use embedded_hal::spi::SpiBus as _;
}

/// System configuration entry point.
pub mod syscfg {
    pub use crate::syscfg::SyscfgExt as _;
}

/// Suffixes for durations and frequencies: `500.millis()`, `100.kHz()`.
pub mod time {
    pub use crate::time::{BpsExtU32 as _, ExtU32 as _, RateExtU32 as _};
}

/// Timer entry point, delays, PWM duty, and `block!` for capture reads.
pub mod timer {
    pub use crate::timer::TimerExt as _;
    pub use embedded_hal::delay::DelayNs as _;
    pub use embedded_hal::pwm::SetDutyCycle as _;
    pub use nb::block;
}

/// Watchdog entry points.
pub mod watchdog {
    pub use crate::watchdog::{FwdgtExt as _, WwdgtExt as _};
}

/// Serial traits, in two flavours — take one.
pub mod usart {
    /// Blocking byte streams, with `read_exact` / `write_all` / `write_fmt`.
    pub mod io {
        pub use embedded_io::{Read as _, ReadReady as _, Write as _, WriteReady as _};
    }

    /// Non-blocking single bytes, driven through `block!`.
    pub mod nb {
        pub use ::nb::block;
        pub use embedded_hal_nb::serial::{Read as _, Write as _};
    }
}

pub use self::adc::*;
pub use self::dma::*;
pub use self::exti::*;
pub use self::fmc::*;
pub use self::gpio::*;
pub use self::i2c::*;
pub use self::rcu::*;
pub use self::spi::*;
pub use self::syscfg::*;
pub use self::time::*;
pub use self::timer::*;
pub use self::usart::io::*;
pub use self::watchdog::*;
