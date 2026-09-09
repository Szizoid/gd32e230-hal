//! Flash memory controller.
//!
//! The wait states are not set from here: they have to be raised before the
//! system clock speeds up, so
//! [`UnfrozenRcu::freeze`](crate::rcu::UnfrozenRcu::freeze) borrows this type and
//! writes them itself.
//!
//! Erasing and programming need `CTL` unlocked, which happens for the body of
//! [`Fmc::with_unlocked`] and no longer. The option bytes are covered too, as a
//! block: [`OptionBytes`] is read, changed and written back whole, because
//! erasing them takes every byte at once. A written block only takes effect on
//! the next load, [`Fmc::reload_option_bytes`] or a power-up.
//!
//! ```ignore
//! let mut fmc = dp.fmc.constrain();
//! let config = ClockConfig::default().sysclk(SysClk::Pll(PllFreq::Mhz48));
//! let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);
//!
//! fmc.with_unlocked(|f| {
//!     f.erase_page(Page::P63)?;
//!     f.program(Page::P63, 0, 0xDEAD_BEEF)
//! })?;
//! ```

use crate::pac;

/// Highest `hclk` each wait-state setting can be read at.
const WS0_MAX_HCLK: u32 = 24_000_000;
const WS1_MAX_HCLK: u32 = 48_000_000;
const WS2_MAX_HCLK: u32 = 72_000_000;
const UNLOCK_KEY1: u32 = 0x45670123;
const UNLOCK_KEY2: u32 = 0xCDEF89AB;

const BASE: u32 = 0x0800_0000;
const PAGE_SIZE: u32 = 0x400;
/// Pages covered by one `OB_WP` bit (user manual, Table 2-4).
const PAGES_PER_WP_BIT: u32 = 4;
/// Bytes per programmed word, `PGW` being left at its reset width of 32 bits.
const WORD_SIZE: u32 = 4;

/// `OB_WP` with every group free, protection being active low.
const WP_ALL_FREE: u32 = 0xFFFF_FFFF;
/// Start of the option byte block (user manual, Table 2-3).
const OB_BASE: u32 = 0x1FFF_F800;

/// Bit positions inside `OB_USER` (user manual, Table 2-3). Bits 3 and 7 are
/// reserved and are carried through untouched.
const BIT_NWDG_SW: u8 = 0;
const BIT_NRST_DPSLP: u8 = 1;
const BIT_NRST_STDBY: u8 = 2;
const BIT_BOOT1_N: u8 = 4;
const BIT_VDDA_VISOR: u8 = 5;
const BIT_SRAM_PARITY: u8 = 6;

/// Packs two option bytes and their complements into one programmed word.
///
/// Every byte of the block is stored next to its inverse, which is what `OBERR`
/// checks on load; the pair is written out here rather than left to the
/// controller.
fn ob_word(low: u8, high: u8) -> u32 {
    let pair = |byte: u8| byte as u32 | ((!byte as u32) << 8);
    pair(low) | (pair(high) << 16)
}

/// The `OB_WP` bit covering `page`, one bit standing for four pages.
const fn wp_bit(page: Page) -> u32 {
    let number = (page as u32 - BASE) / PAGE_SIZE;
    1 << (number / PAGES_PER_WP_BIT)
}

/// Whether an `OB_WP` mask protects the group holding `page`; a protected group
/// reads 0.
const fn wp_protected(wp: u32, page: Page) -> bool {
    wp & wp_bit(page) == 0
}

/// Reads one bit of `OB_USER`.
const fn user_bit(user: u8, bit: u8) -> bool {
    user & (1 << bit) != 0
}

/// Returns `OB_USER` with one bit set to `on`, the rest untouched.
const fn set_user_bit(user: u8, bit: u8, on: bool) -> u8 {
    match on {
        true => user | (1 << bit),
        false => user & !(1 << bit),
    }
}

macro_rules! pages {
    ($($n:literal),* $(,)?) => {
        paste::paste! {
            /// An erasable 1 KB page of the main flash.
            ///
            /// The discriminant is the address the page starts at, so it goes
            /// into `ADDR` as it is. How many pages exist follows the flash of
            /// the part being built for: 16, 32 or 64.
            #[allow(missing_docs)]
            #[derive(Clone, Copy, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[repr(u32)]
            pub enum Page {
                $([<P $n>] = BASE + $n * PAGE_SIZE),*
            }
        }
    };
}

#[cfg(chip_x4)]
#[rustfmt::skip]
pages!(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);

#[cfg(chip_x6)]
#[rustfmt::skip]
pages!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
);

#[cfg(chip_x8)]
#[rustfmt::skip]
pages!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
);

/// An FMC event that can raise an interrupt.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// An erase or a program operation finished (`ENDF`).
    End,
    /// An operation failed (`ERRIE`); which way is [`Fmc::take_error`].
    Error,
}

/// What an erase or a program operation failed on.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The page is protected by the option bytes.
    WriteProtected,
    /// The address is not aligned to the programming width.
    ProgramAlignment,
    /// The cell was not erased before programming.
    Program,
}

/// How far the option bytes lock the flash against being read out.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtectionLevel {
    /// No protection: the debugger reads and writes the flash freely.
    None,
    /// Debugger access is refused. Leaving this level mass-erases the main
    /// flash, so the contents cannot be recovered by lowering it.
    Low,
    /// As `Low`, and the option bytes themselves can no longer be erased or
    /// reprogrammed — the part stays this way for good.
    High,
}

impl ProtectionLevel {
    /// The `OB_SPC` code standing for this level.
    ///
    /// A separate type rather than a discriminant of this one: the same level is
    /// spelled one way in `OB_SPC` and another in the `PLEVEL` field this enum is
    /// also read from.
    const fn spc(self) -> Spc {
        match self {
            ProtectionLevel::None => Spc::None,
            ProtectionLevel::Low => Spc::Low,
            ProtectionLevel::High => Spc::High,
        }
    }
}

/// The `OB_SPC` byte itself (user manual, Table 2-3), the discriminant being the
/// code that goes into the option byte.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Spc {
    /// No protection.
    None = 0xA5,
    /// Low protection. Every code other than the two named here means this one
    /// as well, so reading the byte back can yield a different number.
    Low = 0x00,
    /// High protection, which cannot be undone.
    High = 0xCC,
}

/// Which free watchdog the part starts with (`nWDG_SW`).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FreeWatchdog {
    /// Started by hardware: the watchdog runs from reset and has to be fed from
    /// the first instruction on.
    Hardware,
    /// Started by software, which is what
    /// [`Fwdgt::start`](crate::watchdog::FwdgtRunning) expects.
    Software,
}

/// What entering deep-sleep or standby does (`nRST_DPSLP`, `nRST_STDBY`).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LowPowerEntry {
    /// The part resets instead of entering the mode.
    Reset,
    /// The part enters the mode.
    Enter,
}

/// The level `BOOT1` is taken to have (`BOOT1_n`, stored inverted).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Boot1 {
    /// `BOOT1` reads as 0.
    Low,
    /// `BOOT1` reads as 1.
    High,
}

/// Whether the supply monitor watches VDDA (`VDDA_VISOR`).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VddaMonitor {
    /// Not watched.
    Disabled,
    /// Watched; the part is held in reset while VDDA is too low.
    Enabled,
}

/// Whether the SRAM parity check is on (`SRAM_PARITY_CHECK`, stored inverted).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SramParity {
    /// Not checked.
    Disabled,
    /// Checked.
    Enabled,
}

/// The option byte block as a whole, read with
/// [`Fmc::read_option_bytes`] and written back with
/// [`UnlockedFmc::write_option_bytes`].
///
/// Writing goes through the whole block because erasing does: option bytes lose
/// every byte at once, so a single field cannot be changed on its own. Read the
/// block, change what you need, write it back.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OptionBytes {
    protection: ProtectionLevel,
    user: u8,
    data: u16,
    wp: u32,
}

impl Default for OptionBytes {
    /// The state the part leaves the factory in: no protection, and every other
    /// byte erased.
    fn default() -> Self {
        Self {
            protection: ProtectionLevel::None,
            user: 0xFF,
            data: 0xFFFF,
            wp: WP_ALL_FREE,
        }
    }
}

impl OptionBytes {
    /// Leaves the flash readable by a debugger.
    pub const fn no_protection(mut self) -> Self {
        self.protection = ProtectionLevel::None;
        self
    }
    /// Refuses debugger access. Going back to
    /// [`no_protection`](Self::no_protection) later mass-erases the main flash.
    pub const fn protection_low(mut self) -> Self {
        self.protection = ProtectionLevel::Low;
        self
    }
    /// Refuses debugger access **for good**: after this the option bytes
    /// themselves can no longer be erased or reprogrammed, so no later call can
    /// undo it and the part keeps this setting for the rest of its life.
    pub const fn protection_high_forever(mut self) -> Self {
        self.protection = ProtectionLevel::High;
        self
    }
    /// Picks which free watchdog the part starts with.
    pub const fn free_watchdog(mut self, watchdog: FreeWatchdog) -> Self {
        self.user = set_user_bit(
            self.user,
            BIT_NWDG_SW,
            matches!(watchdog, FreeWatchdog::Software),
        );
        self
    }
    /// Says what entering deep-sleep does.
    pub const fn deep_sleep(mut self, entry: LowPowerEntry) -> Self {
        self.user = set_user_bit(
            self.user,
            BIT_NRST_DPSLP,
            matches!(entry, LowPowerEntry::Enter),
        );
        self
    }
    /// Says what entering standby does.
    pub const fn standby(mut self, entry: LowPowerEntry) -> Self {
        self.user = set_user_bit(
            self.user,
            BIT_NRST_STDBY,
            matches!(entry, LowPowerEntry::Enter),
        );
        self
    }
    /// Sets the level `BOOT1` is taken to have.
    pub const fn boot1(mut self, boot1: Boot1) -> Self {
        self.user = set_user_bit(self.user, BIT_BOOT1_N, matches!(boot1, Boot1::Low));
        self
    }
    /// Turns the VDDA monitor on or off.
    pub const fn vdda_monitor(mut self, monitor: VddaMonitor) -> Self {
        self.user = set_user_bit(
            self.user,
            BIT_VDDA_VISOR,
            matches!(monitor, VddaMonitor::Enabled),
        );
        self
    }
    /// Turns the SRAM parity check on or off.
    pub const fn sram_parity(mut self, parity: SramParity) -> Self {
        self.user = set_user_bit(
            self.user,
            BIT_SRAM_PARITY,
            matches!(parity, SramParity::Disabled),
        );
        self
    }
    /// Sets the two user data bytes, which the silicon never reads itself.
    pub const fn data(mut self, data: u16) -> Self {
        self.data = data;
        self
    }
    /// Closes `page` to erasing and programming.
    ///
    /// One `OB_WP` bit covers four pages, so this closes the three neighbours of
    /// `page` along with it.
    pub const fn protect(mut self, page: Page) -> Self {
        self.wp &= !wp_bit(page);
        self
    }
    /// Opens `page`, and with it the three pages sharing its `OB_WP` bit.
    pub const fn unprotect(mut self, page: Page) -> Self {
        self.wp |= wp_bit(page);
        self
    }

    /// The protection level this block asks for.
    pub const fn protection(&self) -> ProtectionLevel {
        self.protection
    }
    /// Which free watchdog the part starts with.
    pub const fn get_free_watchdog(&self) -> FreeWatchdog {
        match user_bit(self.user, BIT_NWDG_SW) {
            true => FreeWatchdog::Software,
            false => FreeWatchdog::Hardware,
        }
    }
    /// What entering deep-sleep does.
    pub const fn get_deep_sleep(&self) -> LowPowerEntry {
        match user_bit(self.user, BIT_NRST_DPSLP) {
            true => LowPowerEntry::Enter,
            false => LowPowerEntry::Reset,
        }
    }
    /// What entering standby does.
    pub const fn get_standby(&self) -> LowPowerEntry {
        match user_bit(self.user, BIT_NRST_STDBY) {
            true => LowPowerEntry::Enter,
            false => LowPowerEntry::Reset,
        }
    }
    /// The level `BOOT1` is taken to have.
    pub const fn get_boot1(&self) -> Boot1 {
        match user_bit(self.user, BIT_BOOT1_N) {
            true => Boot1::Low,
            false => Boot1::High,
        }
    }
    /// Whether the VDDA monitor is on.
    pub const fn get_vdda_monitor(&self) -> VddaMonitor {
        match user_bit(self.user, BIT_VDDA_VISOR) {
            true => VddaMonitor::Enabled,
            false => VddaMonitor::Disabled,
        }
    }
    /// Whether the SRAM parity check is on.
    pub const fn get_sram_parity(&self) -> SramParity {
        match user_bit(self.user, BIT_SRAM_PARITY) {
            true => SramParity::Disabled,
            false => SramParity::Enabled,
        }
    }
    /// The two user data bytes.
    pub const fn data_bytes(&self) -> u16 {
        self.data
    }
    /// Whether this block closes `page` to erasing and programming.
    pub const fn is_protected(&self, page: Page) -> bool {
        wp_protected(self.wp, page)
    }
}

/// The flash controller with `CTL` unlocked, borrowed for the body of
/// [`Fmc::with_unlocked`] and locked again when that call returns.
pub struct UnlockedFmc<'a> {
    fmc: &'a mut Fmc,
}

impl<'a> UnlockedFmc<'a> {
    fn lock(self) {
        self.fmc.fmc.ctl().modify(|_, w| w.lk().lock());
    }

    /// Erases one page, blocking until it is done.
    ///
    /// The whole page reads back as `0xFF`, so a page holding code or data still
    /// in use has to be picked by the caller, not by us — nothing here checks
    /// what is in it.
    pub fn erase_page(&mut self, page: Page) -> Result<(), Error> {
        self.fmc.fmc.ctl().modify(|_, w| w.per().page_erase());
        self.fmc.fmc.addr().write(|w| w.addr().bits(page as u32));
        self.fmc.fmc.ctl().modify(|_, w| w.start().start());
        let result = self.fmc.wait_busy();
        self.fmc.fmc.ctl().modify(|_, w| w.per().clear_bit());
        result
    }
    /// Erases the whole main flash, blocking until it is done.
    ///
    /// That includes the code running the call, so this only makes sense from
    /// SRAM or from a debugger.
    pub fn mass_erase(&mut self) -> Result<(), Error> {
        self.fmc.fmc.ctl().modify(|_, w| w.mer().mass_erase());
        self.fmc.fmc.ctl().modify(|_, w| w.start().start());
        let result = self.fmc.wait_busy();
        self.fmc.fmc.ctl().modify(|_, w| w.mer().clear_bit());
        result
    }
    /// Programs one 32-bit word, `index` counting words from the start of
    /// `page`, and blocks until it is done.
    ///
    /// Programming only clears bits, so the word has to be erased first: writing
    /// over anything but `0xFFFF_FFFF` returns [`Error::Program`].
    pub fn program(&mut self, page: Page, index: u8, word: u32) -> Result<(), Error> {
        self.fmc.fmc.ctl().modify(|_, w| w.pg().program());
        let addr = page as u32 + index as u32 * WORD_SIZE;
        // The write itself is the command: `PG` makes the FMC latch the address
        // and the data off the bus, so there is no `ADDR` and no `START` here.
        // The address is in the flash and a multiple of four by construction —
        // `Page` gives the base and 256 words is exactly what `u8` counts.
        unsafe {
            core::ptr::write_volatile(addr as *mut u32, word);
        };
        let result = self.fmc.wait_busy();
        self.fmc.fmc.ctl().modify(|_, w| w.pg().clear_bit());
        result
    }

    /// Erases the option byte block and programs `ob` in its place, blocking
    /// until it is done.
    ///
    /// The whole block goes at once because erasing takes it all: read it with
    /// [`Fmc::read_option_bytes`], change what you need, write it back.
    ///
    /// Nothing here takes effect until the option bytes are loaded again, by
    /// [`Fmc::reload_option_bytes`] or by the next power-up.
    pub fn write_option_bytes(&mut self, ob: &OptionBytes) -> Result<(), Error> {
        // `OBER` and `OBPG` answer to a second lock of their own.
        self.fmc.fmc.obkey().write(|w| w.obkey().bits(UNLOCK_KEY1));
        self.fmc.fmc.obkey().write(|w| w.obkey().bits(UNLOCK_KEY2));
        self.fmc.fmc.ctl().modify(|_, w| w.obwen().set_bit());
        let result = self
            .erase_option_bytes()
            .and_then(|()| self.program_option_bytes(ob));
        self.fmc.fmc.ctl().modify(|_, w| w.obwen().disable());
        result
    }
    /// Erases the whole option byte block, every byte going back to `0xFF`.
    fn erase_option_bytes(&mut self) -> Result<(), Error> {
        self.fmc
            .fmc
            .ctl()
            .modify(|_, w| w.ober().option_byte_erase());
        self.fmc.fmc.ctl().modify(|_, w| w.start().start());
        let result = self.fmc.wait_busy();
        self.fmc.fmc.ctl().modify(|_, w| w.ober().clear_bit());
        result
    }
    /// Programs the erased block, one word per pair of option bytes.
    fn program_option_bytes(&mut self, ob: &OptionBytes) -> Result<(), Error> {
        let data = ob.data.to_le_bytes();
        let wp = ob.wp.to_le_bytes();
        let words = [
            ob_word(ob.protection.spc() as u8, ob.user),
            ob_word(data[0], data[1]),
            ob_word(wp[0], wp[1]),
            ob_word(wp[2], wp[3]),
        ];

        self.fmc
            .fmc
            .ctl()
            .modify(|_, w| w.obpg().option_byte_programming());
        let mut result = Ok(());
        for (index, word) in words.iter().enumerate() {
            let addr = OB_BASE + index as u32 * WORD_SIZE;
            // As in `program`, the write is the command. The address is a word
            // of the option byte block, which the loop cannot leave.
            unsafe { core::ptr::write_volatile(addr as *mut u32, *word) };
            result = self.fmc.wait_busy();
            if result.is_err() {
                break;
            }
        }
        self.fmc.fmc.ctl().modify(|_, w| w.obpg().clear_bit());
        result
    }

    /// Raises an interrupt on `event`, which still has to be unmasked in the
    /// NVIC.
    ///
    /// `ENDIE` and `ERRIE` sit in `CTL`, which the lock covers whole, so this
    /// can only be done from inside [`Fmc::with_unlocked`]. The interrupt itself
    /// outlives the call: locking `CTL` again leaves both bits standing.
    pub fn listen(&mut self, event: Event) {
        self.fmc.fmc.ctl().modify(|_, w| match event {
            Event::End => w.endie().enabled(),
            Event::Error => w.errie().enabled(),
        });
    }
    /// Stops `event` from raising an interrupt.
    pub fn unlisten(&mut self, event: Event) {
        self.fmc.fmc.ctl().modify(|_, w| match event {
            Event::End => w.endie().disabled(),
            Event::Error => w.errie().disabled(),
        });
    }
}

/// Owns the flash memory controller.
pub struct Fmc {
    fmc: pac::Fmc,
}

impl Fmc {
    /// Blocks until the running operation ends, then clears `ENDF` and reports
    /// how it went.
    fn wait_busy(&mut self) -> Result<(), Error> {
        while self.fmc.stat().read().busy().is_active() {}
        self.clear_interrupt(Event::End);
        match self.take_error() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    fn unlock(&mut self) -> UnlockedFmc<'_> {
        self.fmc.key().write(|w| w.key().bits(UNLOCK_KEY1));
        self.fmc.key().write(|w| w.key().bits(UNLOCK_KEY2));
        UnlockedFmc { fmc: self }
    }

    /// Unlocks `CTL`, runs `f`, locks it again, and returns what `f` returned.
    ///
    /// The unlocked handle cannot outlive the call, so the flash is never left
    /// writable.
    pub fn with_unlocked<R>(&mut self, f: impl FnOnce(&mut UnlockedFmc) -> R) -> R {
        let mut unlocked = self.unlock();
        let result = f(&mut unlocked);
        unlocked.lock();
        result
    }

    /// Returns the peripheral.
    pub fn release(self) -> pac::Fmc {
        self.fmc
    }

    /// Sets the wait states `hclk` can be read at, `hclk` given in Hz.
    ///
    /// Must run before the system clock rises and after it falls, so the flash
    /// is never read faster than it responds.
    pub(crate) fn set_ws(&mut self, hclk: u32) {
        self.fmc.ws().modify(|_, w| {
            if hclk <= WS0_MAX_HCLK {
                w.wscnt().ws0()
            } else if hclk <= WS1_MAX_HCLK {
                w.wscnt().ws1()
            } else if hclk <= WS2_MAX_HCLK {
                w.wscnt().ws2()
            } else {
                unreachable!()
            }
        });
    }

    /// Whether the option bytes protect `page` from being erased or programmed.
    ///
    /// This is what stands behind [`Error::WriteProtected`]. Protection comes in
    /// groups of four pages, so neighbours share the answer.
    pub fn is_protected(&self, page: Page) -> bool {
        wp_protected(self.fmc.wp().read().bits(), page)
    }
    /// The security protection level the option bytes ask for.
    pub fn protection_level(&self) -> ProtectionLevel {
        let plevel = self.fmc.obstat().read().plevel();
        if plevel.is_none() {
            ProtectionLevel::None
        } else if plevel.is_low() {
            ProtectionLevel::Low
        } else {
            ProtectionLevel::High
        }
    }
    /// Whether the option bytes failed their checksum, in which case the
    /// defaults were loaded instead of them.
    pub fn option_error(&self) -> bool {
        self.fmc.obstat().read().oberr().is_error()
    }
    /// The user option byte, raw: this HAL does not write the option bytes, so
    /// its bits are left to the caller to read against the manual.
    pub fn user_option(&self) -> u8 {
        self.fmc.obstat().read().ob_user().bits()
    }
    /// The two user data bytes of the option block, raw — the silicon attaches
    /// no meaning to them.
    pub fn data_option(&self) -> u16 {
        self.fmc.obstat().read().ob_data().bits()
    }
    /// The product ID code, fixed in silicon and read-only.
    pub fn product_id_code(&self) -> u32 {
        self.fmc.pid().read().pid().bits()
    }

    /// The option bytes as they were loaded at reset.
    ///
    /// This is the block to change and hand to
    /// [`UnlockedFmc::write_option_bytes`]; it reads what is in force, so a
    /// block written since the last reset is not what comes back.
    pub fn read_option_bytes(&self) -> OptionBytes {
        OptionBytes {
            protection: self.protection_level(),
            user: self.user_option(),
            data: self.data_option(),
            wp: self.fmc.wp().read().bits(),
        }
    }

    /// Reloads the option bytes, which resets the part and therefore never
    /// returns.
    ///
    /// This is how a written block takes effect without cycling the power.
    /// `OBRLD` is the one bit of `CTL` the lock leaves writable, so no
    /// unlocking is needed.
    pub fn reload_option_bytes(&mut self) -> ! {
        self.fmc.ctl().modify(|_, w| w.obrld().set_bit());
        loop {
            cortex_m::asm::nop();
        }
    }

    /// Turns the prefetch buffer on or off; it comes up on after reset.
    ///
    /// Off, every fetch waits out the wait states of its own, which is slower on
    /// average but takes the buffer out of the timing.
    pub fn set_prefetch(&mut self, on: bool) {
        self.fmc.ws().modify(|_, w| w.pfen().bit(on));
    }
    /// Whether the prefetch buffer is on.
    pub fn is_prefetch_enabled(&self) -> bool {
        self.fmc.ws().read().pfen().bit_is_set()
    }

    /// Returns the error the last operation ended with, clearing its flag.
    ///
    /// `ENDF` is left alone: it marks success and never stands together with an
    /// error.
    pub fn take_error(&mut self) -> Option<Error> {
        let stat = self.fmc.stat().read();
        if stat.wperr().is_error() {
            self.fmc.stat().write(|w| w.wperr().clear());
            Some(Error::WriteProtected)
        } else if stat.pgaerr().bit_is_set() {
            self.fmc.stat().write(|w| w.pgaerr().bit(true));
            Some(Error::ProgramAlignment)
        } else if stat.pgerr().is_error() {
            self.fmc.stat().write(|w| w.pgerr().clear());
            Some(Error::Program)
        } else {
            None
        }
    }

    /// Whether `event` currently raises an interrupt.
    ///
    /// Reading `CTL` is not covered by the lock, so a handler can ask without
    /// unlocking anything.
    pub fn is_listening(&self, event: Event) -> bool {
        let ctl = self.fmc.ctl().read();
        match event {
            Event::End => ctl.endie().is_enabled(),
            Event::Error => ctl.errie().is_enabled(),
        }
    }

    /// Clears the flag behind `event`, which is what stops it re-entering the
    /// handler.
    ///
    /// For [`Event::Error`] this drops every error flag at once; use
    /// [`take_error`](Self::take_error) instead to learn which one it was, it
    /// clears the flag as well.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.fmc.stat().write(|w| match event {
            Event::End => w.endf().clear(),
            Event::Error => w.wperr().clear().pgaerr().bit(true).pgerr().clear(),
        });
    }
}

/// Entry point on the raw peripheral, mirroring [`GpioExt`](crate::gpio::GpioExt).
pub trait FmcExt {
    /// Takes the peripheral.
    ///
    /// Nothing is written and no clock is gated — the flash controller is always
    /// clocked.
    fn constrain(self) -> Fmc;
}

impl FmcExt for pac::Fmc {
    fn constrain(self) -> Fmc {
        Fmc { fmc: self }
    }
}
