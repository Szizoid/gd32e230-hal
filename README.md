# gd32e2-hal

[English](#english) · [Русский](#русский)

---

## English

A hardware abstraction layer for the **GD32E230K8U6** (Cortex-M23), written in
Rust from scratch on top of the [`gd32e2`](https://crates.io/crates/gd32e2) PAC.

> ⚠️ **Work in progress.** Written by hand, incrementally; the API is unstable.
> The package is a library (`src/lib.rs` → `adc`, `cmp`, `crc`, `dma`, `exti`,
> `fmc`, `gpio`, `i2c`, `prelude`, `rcu`, `spi`, `syscfg`, `time`, `timer`,
> `usart`, `watchdog`) plus
> binaries in `examples/`, all of which are run on the board — RCU, GPIO, USART
> (8/9-bit and parity), SPI0/SPI1, ADC, a one-shot DMA transfer, TIMER, delays,
> PWM, input capture, I²C and its interrupts, CRC, FMC, EXTI, both watchdogs,
> RTT.
> Unverified: the option-byte fields other than the data bytes.

### Principles

- **Errors at compile time, not on the board.** Pin identity and mode live in the
  type: `Pin<'A', 5, Input>` has no `set_high`, an invalid AF number does not
  compile, and ownership prevents reconfiguring a pin twice or using a port
  before its clock is on.
- **A method that changes hardware takes `&mut self`.** `&self` is for reads that
  leave the peripheral as it was, so the borrow checker rejects two concurrent
  users of one peripheral in safe code.
- **Zero-cost.** The same register writes as hand-written PAC code; `Pin` is a ZST.
- **`#![no_std]`, no heap.**

### Chip variants

One feature names the part, and exactly one must be enabled — zero or several is
an error rather than a silently truncated pin map. **There is no default**: which
part is on a board is not something this crate can assume. A feature is the part
number with an `x` in each field the code cannot see: the letter is the bonded pin
count, the digit the flash code, and the trailing `x` is the temperature grade.

| feature | pins | flash | SRAM |
| --- | --- | --- | --- |
| `gd32e230f4xx` / `f6xx` / `f8xx` | 20 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230e8xx` | 24 | 64K | 8K |
| `gd32e230g4xx` / `g6xx` / `g8xx` | 28 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4tx` / `k6tx` / `k8tx` | 32, LQFP | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4ux` / `k6ux` / `k8ux` | 32, QFN | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230c4xx` / `c6xx` / `c8xx` | 48 | 16K / 32K / 64K | 4K / 6K / 8K |

The 32-pin parts are the one place the package matters: a QFN32 carries VSS on its
thermal pad and gives the two freed pins to `PB2` and `PB8`, an LQFP32 does not.

Development targets the `GD32E230K8U6`, i.e. `gd32e230k8ux`; the examples get it
from the crate's own `[dev-dependencies]` entry. The published documentation is
built for `gd32e230c8xx`, the largest part, so the page shows pins and peripherals
a smaller part does not have. `build.rs` turns the choice into the `memory.x` the
linker needs and into the cfg flags the source gates on. The AF map differs by
flash code — the same pin at the same AF number reaching a different peripheral
(`PA2` AF1 is `USART0_TX` on x4, `USART1_TX` on x8), datasheet Table 2-13/2-14
notes (1) x4, (2) x6/x8, (3) x8 — while the bonded pads decide which pins exist.

An orthogonal feature, **`defmt`**, derives `defmt::Format` on the public enums
and error types. Off by default; also enables `embedded-hal/defmt-03`.

### What's implemented

Method-level detail is in the API docs; this is the shape of each module and what
it does not cover.

**GPIO** (`src/gpio.rs`) — const-generic `Pin<P, N, MODE>`, a ZST; ports A, B, F.
Modes as typestate: `Input`, `Analog`, `Output<PushPull>` / `Output<OpenDrain>`,
`Alternate<AF, OTYPE>`, `Debugger` (`PA13`/`PA14`, left through
`activate_into_*()`), `Locked<MODE>` (terminal — hardware has no `unlock`).
`dp.gpioa.split(&mut rcu)` (`GpioExt`) enables the port clock and hands out pins;
`into_*` transitions check the AF number at compile time against a per-pin
`ValidAf` map. State (`set_high` / `toggle` / `is_high` …) is inherent and
`Result`-free, with `embedded-hal` 1.0 `OutputPin` / `InputPin` /
`StatefulOutputPin` on the same helpers. `erase()` gives `ErasedPin<MODE>` — port
and number as fields, mode still in the type, one way only. `Parts` holds only the
pads the package bonds. Port C (`PC13`–`PC15`, 48-pin only) is not implemented.

**RCU** (`src/rcu.rs`) — `dp.rcu.constrain()` (`RcuExt`) gives `UnfrozenRcu`,
whose only method `freeze(&mut fmc, config)` consumes it and returns the `Rcu`
every driver takes, `Clocks` inside. The tree is frozen exactly once, and no
driver can be built before it is; per-peripheral `Enable` / `Reset` traits are
called from every constructor, so nothing runs unclocked. `ClockConfig::default()`
is the reset state, and every field reaches its registers whether named or not.
PLL from IRC8M (`PllFreq`, 8–72 MHz) and bus prescalers are typed enums; flash
wait states are set from the new `hclk` before the source switch. Also `ck_out`
onto `PA8`/`PA9`, `enable_irc40k`, and the reset flags (`RSTFC` clears all seven
at once). `HXTAL` and `LXTAL` are out of scope — neither crystal is fitted.

**FMC** (`src/fmc.rs`) — `dp.fmc.constrain()` (`FmcExt`), always clocked.
`with_unlocked(|f| ...)` unlocks `CTL` for the body of the call only, and carries
`erase_page(Page)`, `mass_erase()`, `program(Page, index, word)`,
`write_option_bytes` and `listen` / `unlisten`. `Fmc` itself carries the reads and
the acknowledges: `take_error`, `clear_interrupt`, `read_option_bytes`,
`reload_option_bytes` (a system reset, returns `!`), `is_protected(Page)`,
`protection_level`, `option_error`, `user_option`, `data_option`,
`product_id_code`, `set_prefetch`. `Page` enumerates the pages of the part
(16 / 32 / 64) and `index` the 256 words of one page, so an address outside the
flash or off a word boundary is not expressible. Programming is 32-bit, `PGW` left
at its reset value; there is no slice variant. Option bytes are read and written
whole, erasing them taking every byte at once; the protection level is set by
`no_protection` / `protection_low` / `protection_high_forever` — the last cannot
be undone, and leaving `protection_low` mass-erases the flash. Only the data bytes
are exercised on hardware.

**USART** (`src/usart.rs`) — `Usart<USARTX, TX, RX, WORD = Byte>` owns the
peripheral and both pins; pin markers reject a wrong pin or AF at compile time,
and `BusClocks` picks the bus frequency per instance. `UsartConfig` (fluent,
`Default` = 115200 / ×16 / `N8`) carries `baud` as `time::Bps`, `Oversampling` and
`FrameFormat` — one source of truth for `WL`/`PCEN`/`PM`. Blocking byte API
(`write_byte` / `write_bytes` / `read_byte` / `read_bytes` / `flush`) plus
readiness flags, with `embedded-hal-nb` and `embedded-io` on the same width;
where a name exists in both layers the inherent one wins on an owned or `&`
receiver, the trait one on `&mut`. Interrupts: `Rbne`, `Tbe`, `Error`,
`ParityError`, all four on one NVIC line and none needing a separate clear.
`Event::Error` is ANDed in hardware with the DMA request line, so without DMA
reception it never fires. 9-bit words live on the `Word` typestate, blocking only.
Errors are `usart::Error`, cleared by `take_error`. No `CTS`/`RTS`.

**ADC** (`src/adc.rs`) — `dp.adc.constrain(rcu)` (`AdcExt`) runs the manual's
calibration sequence. `read(&pin, SampTime)` is one blocking software-triggered
conversion, `Channel` being implemented only for `Pin<P, N, Analog>`; `start` and
`result` are its halves, for driving conversions from an `Event::Eoc` handler.
`read_vref()` returns `VDDA` in mV (falling back to the typical VREFINT when the
factory calibration is blank) and `read_temperature()` tenths of °C, `None` when
`CK_ADC` is too fast for the sensor. Scan mode needs DMA and is deferred.

**SPI** (`src/spi.rs`) — SPI0 and SPI1: master, full-duplex, blocking, 8- or
16-bit, software NSS. Word width is a typestate, so `transfer_word` does not exist
on a byte-wide bus and back; `BitOrder` and `Mode` are runtime values in
`SpiConfig` (no `Default` — an SCK divider has no universal value). An `Instance`
trait abstracts the two peripherals at the operation level, their register blocks
and bit layouts differing. `transfer_bytes` takes buffers of equal length and
panics otherwise, `spi::fill` naming the usual idle levels; `write_byte` /
`read_byte` are the halves of an exchange, for handlers, alongside `read_ready` /
`write_ready` / `take_error`. `SpiBus` from `embedded-hal` sits on top.
Interrupts: `Rbne`, `Tbe`, `Error`; in an interrupt-driven exchange `Tbe` starts
the run and `Rbne` paces it, the receive buffer being one word deep. Hardware NSS,
CRC, half-duplex, TI mode and slave are not implemented. SPI1 exists on x8 parts
only, and below 48 pins its bonded pins are `PB1` plus the SWD pair, hence
`examples/spi1-word.rs` is `required-features = ["gd32e230c8xx"]`.

**DMA** (`src/dma.rs`) — one-shot transfers. `dp.dma.split(&mut rcu)` (`DmaExt`)
hands out `Channel<0>`…`Channel<4>`, each a unique ZST token. `write_to` /
`read_from` take the channel, the peripheral and the buffer by value and return a
`Transfer`; `wait()` is the only way back, and buffers are `&'static`.
`DmaSrc<N>` / `DmaDst<N>` encode the request map (Table 8-3), so a peripheral
paired with the wrong channel does not compile, and the associated `Word` derives
the transfer width from the peripheral's typestate. The request line is raised and
dropped by `dma` itself, so the drivers know nothing about it. Circular mode,
`M2M`, interrupts and `embedded-dma` are deferred.

**TIMER** (`src/timer.rs`) — all seven timers, `dp.timerX.constrain(rcu)`
(`TimerExt`). Roles are separate types with no way to confuse them: `Timer` →
`CountDownTimer` (`start`, `wait`, `stop`), `Delay` (`delay`, `DelayNs`), `Pwm`
(`channel(pin)` → `PwmChannel`, `set_duty`, `SetDutyCycle`, `set_period` for every
channel at once) and `Capture` (`channel(pin, edge)` → `CaptureChannel`, `read()`
as `nb::Result` with `Overcapture`, `interval(from, to)`). Intervals are `fugit`
durations of any scale, derived against this timer's own clock in `u64` and
saturating; `cnt`, `car` and `psc` are read back from the hardware. A pin binds to
a channel through `ChannelPin<TIMERX, C>`, implemented only for the routes the
silicon has, and channel operations only exist on timers that have that channel.
`enable_output()` exists only on the timers with a `CCHP` register. `TIMER14`
exists only on 28-pin and larger parts at 64K flash, and every impl of it is gated
on that. Interrupts cover the rollover and both channel roles; all of a timer's
events share one NVIC line, so a handler pairs `is_listening` with `is_pending`.
Complementary outputs, break inputs, dead time and encoder mode are not
implemented.

**I²C** (`src/i2c.rs`) — master, blocking, 7-bit addressing, both peripherals.
`I2c::new(rcu, i2c, sda, scl, mode)` takes `I2cMode::{standard, fast, fast_plus}`
and derives the timing from `pclk1`, panicking on a too-slow bus or an unreachable
frequency; both pins must be `Alternate<AF, OpenDrain>`. `write` / `read` /
`write_read` are inherent, reads following the manual's "Solution B";
`embedded_hal::i2c::I2c` sits on top. Errors are `i2c::Error`, one variant per
`STAT0` flag. The phase flags and the single steps a transaction is made of are
public, so an interrupt handler can be written by hand; `start_write` /
`start_read` take the peripheral and a `'static` buffer and return a transfer type
whose `on_interrupt` advances a state machine, `release` giving back the
peripheral, the buffer and the outcome. There is no interrupt-driven `write_read`.
10-bit addressing, SMBus, slave mode and DMA are not implemented. The bench is an
RP2040 in I²C target mode at 50 kHz; fast and fast plus are implemented but
unverified.

**CRC** (`src/crc.rs`) — `Crc<PS>` generic over the polynomial width
(`B32`/`B16`/`B8`/`B7`), fixed by the constructor, which also sets `POLY` and the
reversal options. `write_*bit` feeds one word at the bus width matching `PS` and
combines it with the result already there; `read` returns it, `reset_with(seed)`
loads `IDATA` and pulses `RST`. `set_fdata` / `fdata` reach the unrelated scratch
byte.

**Watchdogs** (`src/watchdog/`) — neither can be stopped once started, so each is
a pair of types with no way back: `constrain` then `start` into a running type
whose only method is `feed()`. FWDGT runs off IRC40K, takes either dividers or a
duration (`start_timeout` picks the smallest prescaler that spans it, saturating
at 26 s), and has no window mode. WWDGT is clocked from `PCLK1 / 4096 / psc`,
takes period and window in counter ticks, panics on a window outlasting the
period, and feeding early resets the chip just as a missed deadline does; its
early-wakeup interrupt has no `unlisten`, and the flag re-arms while the counter
sits at `0x40`. `FWDGT_HOLD`/`WWDGT_HOLD` are in the DBG block, which is not
implemented, so both keep counting while a debugger holds the core.

**CMP** (`src/cmp.rs`) — the whole block, one comparator in one register.
`Cmp::new(rcu, cmp, pos, inv, config)` takes both inputs by value, each only in
`Analog`: `InvertingInput` is implemented for the four `VREFINT` taps (0.3 / 0.6
/ 0.9 / 1.2 V) and for `PA4`, `PA5`, `PA0`, `PA2`; `NonInvertingInput` for `PA1`
alone or for the `(PA1, PA4)` pair, which is what closes `CMPSW` — owning `PA4`
there keeps it from also being the inverting input. `CmpConfig` carries speed,
hysteresis, output routing and polarity. `enable` goes to `CmpRunning`, whose
`output()` is read before the polarity multiplexer, so `Polarity` shows only on
the pin, EXTI and the timer. `lock` freezes the register until the next system
reset and leaves a type with no `disable` and no `release`. The clock is shared
with SYSCFG (`CFGCMPEN`), so `release` leaves it on. The output reaches EXTI
line 21, which has not been tried on the board.

**EXTI** (`src/exti.rs`) — external interrupts and events, 21 lines.
`ExtiExt::split` consumes the peripheral and hands out one token per line; the
reserved numbers have no field. Lines 0 to 15 arrive as `ExtiLine<N, PinSrc>`
and take a pin through `source(syscfg, pin)`, which writes `EXTISS` and keeps
the pin — `pin` / `pin_mut` reach it, `release` gives it back with the line
disarmed. Lines 16, 17, 19, 21 and 25 are `InternalSrc` and need no pin.
`edge(EdgeTrigger)` sets `RTEN` / `FTEN`; `listen` / `unlisten` / `is_listening`
drive the interrupt output and `listen_event` / `unlisten_event` /
`is_listening_event` the event output; `pend` raises the line from software,
`is_pending` / `clear_interrupt` work the flag. Only the interrupt path latches
`PD`. The pin lines share three vectors (`EXTI0_1`, `EXTI2_3`, `EXTI4_15`), so a
handler asks which line is pending; the internal lines arrive on the vector of
the peripheral behind them. Port C is absent here as it is in `gpio`.

**SYSCFG** (`src/syscfg.rs`) — `constrain(rcu)` takes the peripheral and
switches on `CFGCMPEN`, the clock shared with CMP. Only `EXTISS` is covered, and
not publicly: the port is picked through `ExtiLine::source`. The `PA11` / `PA12`
and DMA remaps are not implemented.

**Prelude** (`src/prelude.rs`) — split per peripheral (`prelude::gpio`, `::rcu`,
`::dma`, `::fmc`, `::adc`, `::spi`, `::i2c`, `::timer`, `::watchdog`, `::exti`,
`::syscfg`, `::time`, `::usart`). `usart` has `io` and `nb` for the two serial flavours — one or the
other, their `read`/`write` landing on the same type and two same-named traits in
scope making the call ambiguous (`E0034`). `use gd32e2_hal::prelude::*;` takes
everything with `usart::io`. Traits are re-exported as `_`; types are not
included.

### Usage

```rust
use gd32e2_hal::gpio::GpioExt;
use gd32e2_hal::pac;
use gd32e2_hal::rcu::{ClockConfig, PllFreq, RcuExt, SysClk};
use gd32e2_hal::usart::{Usart, UsartConfig};

let dp = pac::Peripherals::take().unwrap();
let mut fmc = dp.fmc.constrain();
let config = ClockConfig::default()
    .sysclk(SysClk::Pll(PllFreq::Mhz48));    // PLL from IRC8M -> 48 MHz
let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);
let clocks = rcu.clocks();
let parts = dp.gpioa.split(&mut rcu);        // enables the GPIOA clock

let mut led = parts.pa5.into_output();
led.set_high();

let tx = parts.pa9.into_alternate::<1>();    // USART0_TX; ::<3>() wouldn't compile
let rx = parts.pa10.into_alternate::<1>();
let mut usart0 = Usart::new(&mut rcu, dp.usart0, tx, rx, UsartConfig::default());
if let Ok(byte) = usart0.read_byte() {
    usart0.write_byte(byte);                 // verified on hardware: echoes back
}
```

### Constraints

- **PAC-only base, no third-party HAL.**
- **Flashing and debugging over SWD** (ST-Link V2 + `probe-rs`, `PA13`/`PA14`).
  Runtime output goes over RTT on the same probe (`defmt` + `defmt-rtt`); no
  USB-serial adapter, and `PA9`/`PA10` are used only by the USART examples.
- Target `thumbv8m.base-none-eabi`; flash 64K, RAM 8K.
- `gd32e2` is generated from patched SVDs — verify field names against
  `docs/GD32E23x_User_Manual.pdf` (PDFs are kept locally, not committed).

### Building

`build.rs` writes the `memory.x` the linker needs from the selected chip feature,
so there is nothing to copy first. A `memory.x` in the project root still wins —
the linker looks there before the search paths — which is the way out for a board
this table does not describe.

```sh
cargo lib                      # library only, alias for build --features gd32e230c8xx
cargo be usart-echo            # compile-check one example, needs no probe
cargo bre usart-echo           # same, release profile
```

The library alone needs the part named, since nothing supplies a default and
`[dev-dependencies]` does not apply to it; an example gets one either way, from
the crate's entry on itself. For another part, name it directly:

```sh
cargo build --release --features gd32e230g6xx
```

To flash, with an ST-Link on `PA13`/`PA14`:

```sh
cargo re usart-echo   # build + flash over SWD, then stay attached
```

`re` is `cargo run --release --example`; `.cargo/config.toml` points the target's
`runner` at `probe-rs run --chip GD32E230K8`, which flashes the ELF directly (no
`objcopy`, no `.bin`) and then stays attached, printing the RTT log until Ctrl-C.
The log level is fixed at compile time by `DEFMT_LOG`. The chip keeps running
without the probe, so the freed probe can read any register live:

```sh
probe-rs reset --chip GD32E230K8                    # run the firmware from reset
probe-rs read  --chip GD32E230K8 b32 0x48000014 1   # e.g. GPIOA_OCTL
```

Firmware that resets the board on purpose — the watchdogs, `reload_option_bytes`
— makes `probe-rs run` report an exception; `probe-rs attach --chip GD32E230K8
<elf>` picks the RTT log back up without reflashing.

> `rust-toolchain.toml` pins the channel and installs the target. On Windows
> without Visual Studio the host toolchain has to be GNU, otherwise build
> scripts fail for want of the MSVC linker:
> `rustup default stable-x86_64-pc-windows-gnu`.

### Roadmap

**Peripherals not covered at all**

- [ ] `PMU` — sleep / deep-sleep / standby, the wakeup pin, LDO.
- [ ] The rest of `SYSCFG` — the `PA11`/`PA12` remap onto the `PA9`/`PA10` pads
      below 32 pins and the second DMA channel map.
- [ ] `embedded-storage` (`ReadNorFlash` / `NorFlash`) over the FMC.
- [ ] `RTC` — the calendar. Runs off IRC40K without a crystal, at IRC40K accuracy.

**Finishing what is there**

- [ ] Interrupts for DMA — the `Event` / `listen` shape is settled, so this is
      mostly mechanical.
- [ ] DMA: circular mode, `M2M`, and `embedded-dma` for the buffers.
- [ ] ADC: scan mode across several channels, which needs DMA to keep the
      intermediate results.
- [ ] Timers: complementary outputs, break and dead time; `TRGO` on `TIMER5`, so
      it can trigger DMA.
- [ ] I²C: 10-bit addressing, SMBus, slave, DMA.
- [ ] SPI: half-duplex / single-wire modes (`BDEN`/`BDOEN`/`RO`); hardware NSS,
      CRC, TI mode and slave are low priority.
- [ ] USART: hardware flow control (`CTS`/`RTS`); a non-blocking API for 9-bit
      words.
- [ ] FWDGT: window mode (`WND`); `FWDGT_HOLD` and `WWDGT_HOLD` in the DBG
      module, so neither watchdog runs while a debugger holds the core.
- [ ] GPIO: port C (`PC13`–`PC15`, 48-pin parts only) — needs its own `Parts`;
      port F alternate functions (no `AFSEL` register — needs its own study).
- [ ] RCU: `HXTAL` and `LXTAL` (neither crystal is fitted on this board).

**Infrastructure**

- [ ] Async: `embedded-hal-async` / `embedded-io-async`, starting with a time
      driver over one of the timers, which is what Embassy needs from a HAL.
- [ ] Tests on the host and CI — there are none; the crate does not build for the
      host, so this needs mocks.
- [ ] Extract the HAL into its own standalone crate/repo.

### License

Dual-licensed, at your option:

- Apache License 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT ([LICENSE-MIT](LICENSE-MIT))

Any contribution submitted for inclusion in this work is licensed the same way,
without additional terms, unless stated otherwise (Apache-2.0, section 5).

---

## Русский

HAL для **GD32E230K8U6** (Cortex-M23), написанный с нуля на Rust поверх PAC-крейта
[`gd32e2`](https://crates.io/crates/gd32e2).

> ⚠️ **В работе.** Пишется руками, по частям; API нестабилен. Пакет — библиотека
> (`src/lib.rs` → `adc`, `cmp`, `crc`, `dma`, `exti`, `fmc`, `gpio`, `i2c`,
> `prelude`, `rcu`, `spi`, `syscfg`, `time`, `timer`, `usart`, `watchdog`) плюс
> бинарники в `examples/`, все прогнаны на плате: RCU, GPIO, USART (8/9 бит и
> чётность), SPI0/SPI1, ADC, разовая передача DMA, TIMER, задержки, PWM, input
> capture, I²C и прерывания по нему, CRC, FMC, EXTI, оба сторожа, RTT.
> Не проверено: поля option bytes кроме байтов данных.

### Принципы

- **Ошибки на компиляции, а не на плате.** Идентичность и режим ноги живут в
  типе: у `Pin<'A', 5, Input>` нет `set_high`, неверный номер AF не собирается, а
  владение не даёт настроить ногу дважды или взять порт до включения такта.
- **Метод, меняющий железо, берёт `&mut self`.** `&self` — только для чтений,
  оставляющих периферию как была, поэтому borrow checker отсекает двух
  одновременных пользователей одной периферии в safe-коде.
- **Zero-cost.** Те же записи в регистры, что и у ручного PAC-кода; `Pin` — ZST.
- **`#![no_std]`, без кучи.**

### Варианты чипа

Партномер называет фича, и включена должна быть ровно одна — ноль или несколько
это ошибка, а не молча урезанная карта пинов. **Дефолта нет**: какой чип на плате,
крейт знать не может. Имя фичи — партномер с `x` в тех полях, которых код не
видит: буква — число разваренных выводов, цифра — код флеша, хвостовой `x` —
температурный класс.

| фича | выводы | флеш | SRAM |
| --- | --- | --- | --- |
| `gd32e230f4xx` / `f6xx` / `f8xx` | 20 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230e8xx` | 24 | 64K | 8K |
| `gd32e230g4xx` / `g6xx` / `g8xx` | 28 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4tx` / `k6tx` / `k8tx` | 32, LQFP | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4ux` / `k6ux` / `k8ux` | 32, QFN | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230c4xx` / `c6xx` / `c8xx` | 48 | 16K / 32K / 64K | 4K / 6K / 8K |

32 вывода — единственное место, где корпус имеет значение: у QFN32 земля уходит на
брюшко, и два освободившихся вывода достаются `PB2` и `PB8`, у LQFP32 нет.

Разработка идёт под `GD32E230K8U6`, то есть `gd32e230k8ux`; примеры берут фичу из
записи крейта на себя в `[dev-dependencies]`. Опубликованная документация собрана
для `gd32e230c8xx`, самой жирной детали, поэтому страница показывает ноги и
периферию, которых у мелкой нет. `build.rs` превращает выбор в нужный линкеру
`memory.x` и в cfg-флаги, по которым гейтится исходник. Карта AF зависит от кода
флеша — одна и та же нога на одном номере AF ведёт к разной периферии (`PA2` AF1
это `USART0_TX` на x4 и `USART1_TX` на x8), сноски (1) x4, (2) x6/x8, (3) x8 в
Table 2-13/2-14 даташита, — а набор разваренных площадок решает, какие ноги вообще
существуют.

Ортогональная фича **`defmt`** вешает `defmt::Format` на публичные enum и типы
ошибок. По умолчанию выключена; заодно включает `embedded-hal/defmt-03`.

### Что уже есть

Подробности по методам — в документации API; здесь форма каждого модуля и то,
чего в нём нет.

**GPIO** (`src/gpio.rs`) — const-generic `Pin<P, N, MODE>`, ZST; порты A, B, F.
Режимы как typestate: `Input`, `Analog`, `Output<PushPull>` / `Output<OpenDrain>`,
`Alternate<AF, OTYPE>`, `Debugger` (`PA13`/`PA14`, выход через
`activate_into_*()`), `Locked<MODE>` (терминальный — `unlock` нет и в железе).
`dp.gpioa.split(&mut rcu)` (`GpioExt`) включает такт порта и раздаёт ноги;
переходы `into_*` проверяют номер AF на компиляции по карте `ValidAf` своей ноги.
Состояние (`set_high` / `toggle` / `is_high` …) инхерентное и без `Result`, сверху
`embedded-hal` 1.0 `OutputPin` / `InputPin` / `StatefulOutputPin` на тех же
хелперах. `erase()` даёт `ErasedPin<MODE>` — порт и номер полями, режим остаётся в
типе, только в одну сторону. `Parts` держит лишь те площадки, что разварены в
корпусе. Порт C (`PC13`–`PC15`, только 48 выводов) не реализован.

**RCU** (`src/rcu.rs`) — `dp.rcu.constrain()` (`RcuExt`) отдаёт `UnfrozenRcu`, у
которого единственный метод `freeze(&mut fmc, config)` съедает значение и
возвращает `Rcu`, который берут все драйверы, с `Clocks` внутри. Дерево
замораживается ровно один раз, и до этого ни один драйвер не собрать;
`Enable` / `Reset` на каждую периферию зовутся из каждого конструктора, поэтому
ничего не заработает без такта. `ClockConfig::default()` — состояние после сброса,
и каждое поле доезжает до регистров независимо от того, назвали его или нет. PLL
от IRC8M (`PllFreq`, 8–72 МГц) и делители шин — типизированные enum; wait states
флеша выставляются от нового `hclk` до переключения источника. Ещё `ck_out` на
`PA8`/`PA9`, `enable_irc40k` и флаги сброса (`RSTFC` гасит все семь разом).
`HXTAL` и `LXTAL` вне скоупа — ни один кварц на плате не запаян.

**FMC** (`src/fmc.rs`) — `dp.fmc.constrain()` (`FmcExt`), такт всегда есть.
`with_unlocked(|f| ...)` снимает замок с `CTL` только на тело вызова и несёт
`erase_page(Page)`, `mass_erase()`, `program(Page, index, word)`,
`write_option_bytes` и `listen` / `unlisten`. На самом `Fmc` — чтения и
подтверждения: `take_error`, `clear_interrupt`, `read_option_bytes`,
`reload_option_bytes` (системный сброс, возвращает `!`), `is_protected(Page)`,
`protection_level`, `option_error`, `user_option`, `data_option`,
`product_id_code`, `set_prefetch`. `Page` перечисляет страницы этого партномера
(16 / 32 / 64), `index` — 256 слов страницы, поэтому адрес вне флеша или не по
границе слова не выражается. Программирование 32-битное, `PGW` остаётся в
сбросовом значении; варианта по срезу нет. Option bytes читаются и пишутся целым
блоком, потому что стирание берёт все байты сразу; уровень защиты ставится
методами `no_protection` / `protection_low` / `protection_high_forever` —
последний необратим, а выход из `protection_low` стирает весь флеш. На железе
прогнаны только байты данных.

**USART** (`src/usart.rs`) — `Usart<USARTX, TX, RX, WORD = Byte>` владеет
периферией и обеими ногами; маркеры пинов отсекают неверную ногу или AF на
компиляции, `BusClocks` берёт частоту шины под конкретный инстанс. `UsartConfig`
(fluent, `Default` = 115200 / ×16 / `N8`) несёт `baud` величиной `time::Bps`,
`Oversampling` и `FrameFormat` — один источник правды для `WL`/`PCEN`/`PM`.
Блокирующий байтовый API (`write_byte` / `write_bytes` / `read_byte` /
`read_bytes` / `flush`) плюс флаги готовности, сверху `embedded-hal-nb` и
`embedded-io` на той же ширине; где имя есть в обоих слоях, инхерентный побеждает
на владении и `&`, трейтовый — на `&mut`. Прерывания: `Rbne`, `Tbe`, `Error`,
`ParityError`, все четыре на одной линии NVIC, и отдельного сброса не требует ни
одно. `Event::Error` в железе объединён по И с линией запроса DMA, поэтому без
приёма по DMA не срабатывает. 9-битные слова живут на typestate `Word`, только
блокирующие. Ошибки — `usart::Error`, снимаются `take_error`. `CTS`/`RTS` нет.

**ADC** (`src/adc.rs`) — `dp.adc.constrain(rcu)` (`AdcExt`) проводит калибровку по
мануалу. `read(&pin, SampTime)` — одна блокирующая конверсия по программному
триггеру, `Channel` реализован только для `Pin<P, N, Analog>`; `start` и `result`
— его половинки, для конверсий из обработчика по `Event::Eoc`. `read_vref()`
возвращает `VDDA` в мВ (с фолбэком на типовой VREFINT, если заводская калибровка
пуста), `read_temperature()` — десятые доли °C, `None` при слишком быстром
`CK_ADC`. Scan-режим требует DMA и отложен.

**SPI** (`src/spi.rs`) — SPI0 и SPI1: master, full-duplex, блокирующий, 8 или 16
бит, программный NSS. Ширина слова — typestate, поэтому `transfer_word` не
существует на байтовой шине и наоборот; `BitOrder` и `Mode` — рантайм-значения в
`SpiConfig` (без `Default`: у делителя SCK универсального значения нет). Трейт
`Instance` абстрагирует обе периферии на уровне операций — регистровые блоки и
раскладка бит у них разные. `transfer_bytes` требует буферы равной длины и иначе
паникует, уровни добивки названы в `spi::fill`; `write_byte` / `read_byte` —
половинки обмена для обработчиков, рядом `read_ready` / `write_ready` /
`take_error`. Сверху `SpiBus` из `embedded-hal`. Прерывания: `Rbne`, `Tbe`,
`Error`; в обмене по прерываниям `Tbe` запускает, `Rbne` задаёт темп — приёмный
буфер глубиной в одно слово. Аппаратный NSS, CRC, half-duplex, TI mode и slave не
реализованы. SPI1 есть только на деталях x8, и ниже 48 выводов из его ног
разварены `PB1` и пара SWD, поэтому `examples/spi1-word.rs` идёт с
`required-features = ["gd32e230c8xx"]`.

**DMA** (`src/dma.rs`) — разовые передачи. `dp.dma.split(&mut rcu)` (`DmaExt`)
раздаёт `Channel<0>`…`Channel<4>`, каждый — уникальный ZST-токен. `write_to` /
`read_from` забирают канал, периферию и буфер по значению и отдают `Transfer`;
`wait()` — единственный путь назад, буферы `&'static`. `DmaSrc<N>` / `DmaDst<N>`
кодируют карту запросов (Table 8-3), поэтому неверная пара «периферия — канал» не
собирается, а ассоциированный `Word` выводит ширину передачи из typestate
периферии. Линию запроса поднимает и гасит сам `dma`, драйверы о ней не знают.
Циклический режим, `M2M`, прерывания и `embedded-dma` отложены.

**TIMER** (`src/timer.rs`) — все семь таймеров, `dp.timerX.constrain(rcu)`
(`TimerExt`). Роли — отдельные типы, перепутать нельзя: `Timer` →
`CountDownTimer` (`start`, `wait`, `stop`), `Delay` (`delay`, `DelayNs`), `Pwm`
(`channel(pin)` → `PwmChannel`, `set_duty`, `SetDutyCycle`, `set_period` сразу на
все каналы) и `Capture` (`channel(pin, edge)` → `CaptureChannel`, `read()` как
`nb::Result` с `Overcapture`, `interval(from, to)`). Интервалы — длительности
`fugit` любой шкалы, пересчитываются от собственного такта таймера в `u64` с
насыщением; `cnt`, `car` и `psc` читаются из железа. Нога привязывается к каналу
через `ChannelPin<TIMERX, C>`, реализованный только для существующих в кремнии
маршрутов, а операции канала есть только у таймеров, у которых такой канал есть.
`enable_output()` есть только у таймеров с регистром `CCHP`. `TIMER14` существует
только на деталях от 28 выводов с флешем 64K, и на это гейтится каждая его
реализация. Прерывания покрывают переполнение и обе роли каналов; все события
таймера делят одну линию NVIC, поэтому обработчик сверяет `is_listening` с
`is_pending`. Комплементарных выходов, break, dead time и энкодера нет.

**I²C** (`src/i2c.rs`) — master, блокирующий, 7-битная адресация, обе периферии.
`I2c::new(rcu, i2c, sda, scl, mode)` берёт `I2cMode::{standard, fast, fast_plus}`
и считает тайминги от `pclk1`, паникуя при слишком медленной шине или
недостижимой частоте; обе ноги обязаны быть `Alternate<AF, OpenDrain>`.
`write` / `read` / `write_read` инхерентные, чтение идёт по «Solution B» из
мануала; сверху `embedded_hal::i2c::I2c`. Ошибки — `i2c::Error`, по варианту на
флаг `STAT0`. Флаги фаз и отдельные шаги транзакции публичны, поэтому обработчик
прерывания можно написать руками; `start_write` / `start_read` забирают периферию
и `'static`-буфер и отдают тип передачи, чей `on_interrupt` двигает автомат, а
`release` возвращает периферию, буфер и исход. `write_read` по прерываниям нет.
10-битная адресация, SMBus, slave и DMA не реализованы. Стенд — RP2040 в режиме
I²C target на 50 кГц; fast и fast plus написаны, но не проверены.

**CRC** (`src/crc.rs`) — `Crc<PS>` дженерик по ширине полинома
(`B32`/`B16`/`B8`/`B7`), которую фиксирует конструктор, он же ставит `POLY` и
опции разворота. `write_*bit` подаёт слово на шине той же ширины, что и `PS`, и
комбинирует его с уже накопленным результатом; `read` возвращает результат,
`reset_with(seed)` кладёт `IDATA` и импульсит `RST`. `set_fdata` / `fdata` — не
связанный со счётом байт-черновик.

**Сторожа** (`src/watchdog/`) — ни один нельзя остановить после запуска, поэтому
каждый — пара типов без пути назад: `constrain`, затем `start` в работающий тип,
единственный метод которого `feed()`. FWDGT тактуется от IRC40K, принимает либо
делители, либо длительность (`start_timeout` берёт наименьший подходящий
предделитель, насыщаясь на 26 с), оконного режима нет. WWDGT тактуется от
`PCLK1 / 4096 / psc`, период и окно задаются в тиках счётчика, окно длиннее
периода — паника, а кормление раньше окна сбрасывает чип так же, как пропущенный
срок; у прерывания раннего пробуждения нет `unlisten`, а флаг взводится заново,
пока счётчик стоит на `0x40`. `FWDGT_HOLD`/`WWDGT_HOLD` живут в блоке DBG,
которого нет, поэтому оба сторожа считают и под остановленным отладчиком.

**CMP** (`src/cmp.rs`) — блок целиком, один компаратор в одном регистре.
`Cmp::new(rcu, cmp, pos, inv, config)` забирает оба входа по значению и только в
`Analog`: `InvertingInput` реализован для четырёх отводов `VREFINT` (0.3 / 0.6 /
0.9 / 1.2 В) и для `PA4`, `PA5`, `PA0`, `PA2`; `NonInvertingInput` — для `PA1`
либо для пары `(PA1, PA4)`, которая и замыкает `CMPSW`: владение `PA4` там же
запрещает отдать её инвертирующим входом. `CmpConfig` несёт скорость,
гистерезис, маршрут выхода и полярность. `enable` даёт `CmpRunning`, чей
`output()` читается до мультиплексора полярности, поэтому `Polarity` видна
только на ноге, EXTI и таймере. `lock` замораживает регистр до системного сброса
и оставляет тип без `disable` и без `release`. Такт общий с SYSCFG
(`CFGCMPEN`), поэтому `release` его не гасит. Выход заведён на линию 21 EXTI, на
плате это не проверялось.

**EXTI** (`src/exti.rs`) — внешние прерывания и события, 21 линия.
`ExtiExt::split` съедает периферию и отдаёт по токену на линию; зарезервированных
номеров в структуре нет. Линии 0–15 приходят как `ExtiLine<N, PinSrc>` и берут
ногу через `source(syscfg, pin)`, который пишет `EXTISS` и оставляет ногу у
себя — до неё ведут `pin` / `pin_mut`, а `release` возвращает её, погасив линию.
Линии 16, 17, 19, 21 и 25 — `InternalSrc`, ноги не требуют. `edge(EdgeTrigger)`
ставит `RTEN` / `FTEN`; `listen` / `unlisten` / `is_listening` управляют выходом
в NVIC, `listen_event` / `unlisten_event` / `is_listening_event` — событийным;
`pend` поднимает линию программно, `is_pending` / `clear_interrupt` работают с
флагом. `PD` взводится только на тракте прерывания. Линии от ног делят три
вектора (`EXTI0_1`, `EXTI2_3`, `EXTI4_15`), поэтому обработчик спрашивает, чей
флаг; внутренние линии приходят на вектор своей периферии. Порта C здесь нет,
как и в `gpio`.

**SYSCFG** (`src/syscfg.rs`) — `constrain(rcu)` забирает периферию и включает
`CFGCMPEN`, такт общий с CMP. Покрыт только `EXTISS`, и непублично: порт
выбирается через `ExtiLine::source`. Ремапы `PA11` / `PA12` и DMA не реализованы.

**Прелюдия** (`src/prelude.rs`) — разбита по периферии (`prelude::gpio`, `::rcu`,
`::dma`, `::fmc`, `::adc`, `::spi`, `::i2c`, `::timer`, `::watchdog`, `::exti`,
`::syscfg`, `::time`, `::usart`). У `usart` есть `io` и `nb` под два стиля последовательного API — одно
или другое: их `read`/`write` ложатся на один тип, и два одноимённых трейта в
области видимости делают вызов неоднозначным (`E0034`).
`use gd32e2_hal::prelude::*;` берёт всё и `usart::io`. Трейты реэкспортированы под
`_`; типы не входят.

### Пример

```rust
use gd32e2_hal::gpio::GpioExt;
use gd32e2_hal::pac;
use gd32e2_hal::rcu::{ClockConfig, PllFreq, RcuExt, SysClk};
use gd32e2_hal::usart::{Usart, UsartConfig};

let dp = pac::Peripherals::take().unwrap();
let mut fmc = dp.fmc.constrain();
let config = ClockConfig::default()
    .sysclk(SysClk::Pll(PllFreq::Mhz48));    // PLL от IRC8M -> 48 МГц
let mut rcu = dp.rcu.constrain().freeze(&mut fmc, config);
let clocks = rcu.clocks();
let parts = dp.gpioa.split(&mut rcu);        // включает такт GPIOA

let mut led = parts.pa5.into_output();
led.set_high();

let tx = parts.pa9.into_alternate::<1>();    // USART0_TX; ::<3>() не соберётся
let rx = parts.pa10.into_alternate::<1>();
let mut usart0 = Usart::new(&mut rcu, dp.usart0, tx, rx, UsartConfig::default());
if let Ok(byte) = usart0.read_byte() {
    usart0.write_byte(byte);                 // проверено на железе: эхо возвращается
}
```

### Ограничения

- **База — только PAC, без сторонних HAL.**
- **Прошивка и отладка по SWD** (ST-Link V2 + `probe-rs`, `PA13`/`PA14`). Вывод из
  прошивки идёт по RTT через тот же зонд (`defmt` + `defmt-rtt`); USB-serial
  адаптера нет, `PA9`/`PA10` заняты только примерами с USART.
- Таргет `thumbv8m.base-none-eabi`; флеш 64K, RAM 8K.
- `gd32e2` собран из патченных SVD — имена полей сверять с
  `docs/GD32E23x_User_Manual.pdf` (PDF лежат локально, в репозиторий не входят).

### Сборка

`build.rs` пишет нужный линкеру `memory.x` из выбранной фичи чипа, копировать
ничего не надо. `memory.x` в корне проекта всё равно перебивает сгенерированный —
линкер смотрит туда раньше путей поиска, — и это штатный выход для платы, которой
нет в таблице.

```sh
cargo lib                      # только библиотека, алиас для build --features gd32e230c8xx
cargo be usart-echo            # проверить сборку одного примера, зонд не нужен
cargo bre usart-echo           # то же в release
```

Библиотеке партномер нужно называть явно: дефолта нет, а `[dev-dependencies]` к
ней не применяются; примеру фича достаётся в любом случае — из записи крейта на
себя. Под другую деталь:

```sh
cargo build --release --features gd32e230g6xx
```

Прошивка, с ST-Link на `PA13`/`PA14`:

```sh
cargo re usart-echo   # сборка + прошивка по SWD, дальше остаётся подключённым
```

`re` — это `cargo run --release --example`; `.cargo/config.toml` направляет
`runner` цели на `probe-rs run --chip GD32E230K8`, который заливает ELF напрямую
(без `objcopy` и `.bin`) и остаётся подключённым, печатая RTT-лог до Ctrl-C.
Уровень лога фиксируется на компиляции переменной `DEFMT_LOG`. Чипу зонд не нужен,
поэтому освободившимся зондом можно читать любой регистр вживую:

```sh
probe-rs reset --chip GD32E230K8                    # запустить прошивку со сброса
probe-rs read  --chip GD32E230K8 b32 0x48000014 1   # например GPIOA_OCTL
```

Прошивка, которая сбрасывает плату намеренно (сторожа, `reload_option_bytes`),
выглядит для `probe-rs run` как исключение; `probe-rs attach --chip GD32E230K8
<elf>` подхватывает RTT-лог обратно без перепрошивки.

> `rust-toolchain.toml` фиксирует канал и ставит таргет. На Windows без Visual
> Studio host-toolchain должен быть GNU, иначе build-скрипты падают из-за
> отсутствия компоновщика MSVC: `rustup default stable-x86_64-pc-windows-gnu`.

### Roadmap

**Периферия, не покрытая вовсе**

- [ ] `PMU` — sleep / deep-sleep / standby, wakeup-нога, LDO.
- [ ] Остаток `SYSCFG` — ремап `PA11`/`PA12` на площадки `PA9`/`PA10` ниже
      32 выводов и вторая карта каналов DMA.
- [ ] `embedded-storage` (`ReadNorFlash` / `NorFlash`) поверх FMC.
- [ ] `RTC` — календарь. Без кварца пойдёт от IRC40K, с его же точностью.

**Доделать имеющееся**

- [ ] Прерывания DMA — форма `Event` / `listen` устоялась, работа механическая.
- [ ] DMA: циклический режим, `M2M`, `embedded-dma` для буферов.
- [ ] ADC: scan по нескольким каналам, для чего нужен DMA.
- [ ] Таймеры: комплементарные выходы, break и dead time; `TRGO` у `TIMER5`, чтобы
      он мог запускать DMA.
- [ ] I²C: 10-битная адресация, SMBus, slave, DMA.
- [ ] SPI: half-duplex и однопроводные режимы (`BDEN`/`BDOEN`/`RO`); аппаратный
      NSS, CRC, TI mode и slave — низкий приоритет.
- [ ] USART: аппаратное управление потоком (`CTS`/`RTS`); неблокирующий API для
      9-битных слов.
- [ ] FWDGT: оконный режим (`WND`); `FWDGT_HOLD` и `WWDGT_HOLD` в модуле DBG,
      чтобы сторожа не считали под остановленным отладчиком.
- [ ] GPIO: порт C (`PC13`–`PC15`, только 48 выводов) — нужен свой `Parts`;
      альтернативные функции порта F (регистра `AFSEL` нет — нужен отдельный
      разбор).
- [ ] RCU: `HXTAL` и `LXTAL` (ни один кварц на плате не запаян).

**Инфраструктура**

- [ ] Async: `embedded-hal-async` / `embedded-io-async`, начиная с драйвера
      времени поверх одного из таймеров — именно его Embassy ждёт от HAL.
- [ ] Тесты на хосте и CI — их нет вовсе; крейт не собирается под хост, поэтому
      нужны моки.
- [ ] Вынос HAL в отдельный крейт/репозиторий.

### Лицензия

Двойное лицензирование, на выбор:

- Apache License 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT ([LICENSE-MIT](LICENSE-MIT))

Любой вклад, отправленный для включения в эту работу, лицензируется так же, без
дополнительных условий, если не оговорено иное (Apache-2.0, раздел 5).
