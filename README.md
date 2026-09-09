# gd32e2-hal

[English](#english) · [Русский](#русский)

---

## English

Hardware abstraction layer (HAL) for the gd32e230x and gd32e231x microcontrollers (Cortex-M23), written on top of the PAC crate: [github](https://github.com/gd32-rust/gd32-rs), [crates.io](https://crates.io/crates/gd32e2).

> **Work in progress.** Written piece by piece, the API is unstable. Implemented modules: `adc`, `cmp`, `crc`, `dma`, `exti`, `fmc`, `gpio`, `i2c`, `rcu`, `spi`, `syscfg`, `time`, `timer`, `usart`, `watchdog`. Usage examples live in `examples/`, all of them run on the board. Not verified: option byte fields other than the data bytes.

### Principles

- **Errors at compile time**. The identity of an entity (a pin, a peripheral, a peripheral channel and so on) determines the list of available methods — an input pin has no `.set_high()` to call;
- Thanks to Rust **ownership and borrowing**, the same peripheral cannot be initialized twice or taken before its clock is enabled, and hardware cannot be modified from two places at once;
- **Zero-cost.** The same register writes as less abstract PAC code; most of the abstractions are ZSTs.

### Chip variants

A feature name is the part number with `x` in the fields the code does not see: the letter is the number of bonded pads, the digit is the flash code, the trailing `x` is the temperature grade.

| feature | pads | flash | SRAM |
| --- | --- | --- | --- |
| `gd32e230f4xx` / `f6xx` / `f8xx` | 20 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230e8xx` | 24 | 64K | 8K |
| `gd32e230g4xx` / `g6xx` / `g8xx` | 28 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4tx` / `k6tx` / `k8tx` | 32, LQFP | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4ux` / `k6ux` / `k8ux` | 32, QFN | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230c4xx` / `c6xx` / `c8xx` | 48 | 16K / 32K / 64K | 4K / 6K / 8K |

32 pads is the only place where the package matters: on QFN32 ground moves to the belly pad, and the two freed pins go to `PB2` and `PB8`; LQFP32 has neither.

> Development targets `GD32E230K8U6`, that is `gd32e230k8ux`; the examples take the feature from the crate depending on itself in `[dev-dependencies]`. The published documentation is built for `gd32e230c8xx`, the largest part, so the page shows pins and peripherals the smaller parts do not have. `build.rs` turns the choice into the `memory.x` the linker needs and into cfg flags that gate the source. The AF map depends on the flash code — the same pin on the same AF number may lead to a different peripheral.

The orthogonal **`defmt`** feature derives `defmt::Format` on public enums and error types. Off by default; it also enables `embedded-hal/defmt-03`.

### Documentation

Method details are in the [documentation](https://docs.rs/gd32e2-hal/latest/gd32e2_hal/). A shorter view of most of the public API is in `examples/`.

### Example

```rust
use gd32e2_hal::prelude::*;
use gd32e2_hal::pac;
use gd32e2_hal::rcu::{ClockConfig, PllFreq, SysClk};
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

let tx = parts.pa9.into_alternate::<1>();    // USART0_TX; ::<3>() does not compile
let rx = parts.pa10.into_alternate::<1>();
let mut usart0 = Usart::new(&mut rcu, dp.usart0, tx, rx, UsartConfig::default());
if let Ok(byte) = usart0.read_byte() {
    usart0.write_byte(byte);                 // verified on hardware: the echo comes back
}
```

### Building

> **Flashing and debugging over SWD** (ST-Link V2 + `probe-rs`, `PA13` / `PA14`). Firmware output goes over RTT through the same probe (`defmt` + `defmt-rtt`).

- Target `thumbv8m.base-none-eabi`;
- `build.rs` writes the `memory.x` the linker needs from the selected library feature, nothing has to be copied. (A `memory.x` in the project root still overrides the generated one — the linker looks there before the search paths — and that is the supported way out for a board that is not in the table);

#### cargo aliases

```shell
cargo lib             # the library only, an alias for build --features gd32e230c8xx
cargo be usart-echo   # check that one example builds, no probe needed
cargo bre usart-echo  # the same in release
cargo re usart-echo   # build + flash over SWD, stays attached afterwards
```

### Roadmap

**Peripherals not covered at all**
- [ ] `PMU` — sleep / deep-sleep / standby, wakeup pin, LDO.
- [ ] The rest of `SYSCFG` — remapping `PA11`/`PA12` onto the `PA9`/`PA10` pads below 32 pins, and the second DMA channel map.
- [ ] `RTC` — calendar. With no crystal it runs off IRC40K, with the accuracy of IRC40K.

**Finishing what is there**
- [ ] DMA interrupts — the `Event` / `listen` shape has settled, the work is mechanical.
- [ ] DMA: circular mode, `M2M`, `embedded-dma` for buffers.
- [ ] ADC: scan over several channels, which needs DMA.
- [ ] Timers: complementary outputs, break and dead time; `TRGO` on `TIMER5` so it can drive DMA.
- [ ] I²C: 10-bit addressing, SMBus, slave, DMA.
- [ ] SPI: half-duplex and single-wire modes (`BDEN`/`BDOEN`/`RO`); hardware NSS, CRC, TI mode and slave — low priority.
- [ ] USART: hardware flow control (`CTS`/`RTS`); a non-blocking API for 9-bit words.
- [ ] FWDGT: window mode (`WND`); `FWDGT_HOLD` and `WWDGT_HOLD` in a DBG module, so that the watchdogs do not count under a halted debugger.
- [ ] RCU: `HXTAL` and `LXTAL` (no crystal is fitted on the board).
- [ ] FMC: `embedded-storage` (`ReadNorFlash` / `NorFlash`) on top of the existing program and erase.

**Infrastructure**
- [ ] gd32e231x support: part number features, pin map, gates. The PAC module exists, the chip is not at hand.
- [ ] Async: `embedded-hal-async` / `embedded-io-async`, starting with a time driver on one of the timers — that is what Embassy expects from a HAL.
- [ ] Host tests and CI — there are none at all; the crate does not build for the host, so mocks are needed.
- [ ] Moving the HAL into its own crate/repository.

### License

Dual licensed, at your option:
- Apache License 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT ([LICENSE-MIT](LICENSE-MIT))

Any contribution submitted for inclusion in this work shall be licensed the same way, without additional terms, unless stated otherwise (Apache-2.0, section 5).

---

## Русский

Hardware abstraction layer (HAL) для микроконтроллеров семейства gd32e230x и gd32e231x (Cortex-M23), написанный поверх PAC-крейта: [github](https://github.com/gd32-rust/gd32-rs), [crates.io](https://crates.io/crates/gd32e2).

> **В работе.** Пишется частями, API нестабилен. Реализованные модули: `adc`, `cmp`, `crc`, `dma`, `exti`, `fmc`, `gpio`, `i2c`, `rcu`, `spi`, `syscfg`, `time`, `timer`, `usart`, `watchdog`. Имеются примеры использования в `examples/`, все прогнаны на плате. Из непроверенного: поля option bytes кроме байтов данных.

### Принципы

- **Ошибки на этапе компиляции**. Идентичность сущности (пин, элемент периферии, канал периферии и пр.) определяет список доступных методов (например, input-пин не позволит вызвать у него метод `.set_high()`);
- Благодаря концепциям **владения и заимствования** языка Rust одну и ту же периферию не удастся инициализировать дважды или взять до включения тактования, а также не получится менять железо из двух разных участков кода одновременно;
- **Zero-cost.** Те же записи в регистры, что и у менее абстрактного PAC-кода; большая часть абстракций — ZST.

### Варианты чипа

Имя фичи — партномер с `x` в тех полях, которых код не видит: буква — число разваренных выводов, цифра — код флеша, хвостовой `x` — температурный класс.

| фича | выводы | флеш | SRAM |
| --- | --- | --- | --- |
| `gd32e230f4xx` / `f6xx` / `f8xx` | 20 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230e8xx` | 24 | 64K | 8K |
| `gd32e230g4xx` / `g6xx` / `g8xx` | 28 | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4tx` / `k6tx` / `k8tx` | 32, LQFP | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230k4ux` / `k6ux` / `k8ux` | 32, QFN | 16K / 32K / 64K | 4K / 6K / 8K |
| `gd32e230c4xx` / `c6xx` / `c8xx` | 48 | 16K / 32K / 64K | 4K / 6K / 8K |

32 вывода — единственное место, где корпус имеет значение: у QFN32 земля уходит на брюшко, и два освободившихся вывода достаются `PB2` и `PB8`, у LQFP32 нет.

> Разработка идёт под `GD32E230K8U6`, то есть `gd32e230k8ux`; примеры берут фичу из записи крейта на себя в `[dev-dependencies]`. Опубликованная документация собрана для `gd32e230c8xx`, самой жирной детали, поэтому страница показывает ноги и периферию, которых у мелкой нет. `build.rs` превращает выбор в нужный линкеру `memory.x` и в cfg-флаги, по которым гейтится исходник. Карта AF зависит от кода флеша — одна и та же нога на одном номере AF может вести к разной периферии.

Ортогональная фича **`defmt`** вешает `defmt::Format` на публичные enum и типы ошибок. По умолчанию выключена; заодно включает `embedded-hal/defmt-03`.

### Документация

Подробности по методам можно найти в [документации](https://docs.rs/gd32e2-hal/latest/gd32e2_hal/). Более краткое отражение большей части публичного API библиотеки в `examples/`.

### Пример

```rust
use gd32e2_hal::prelude::*;
use gd32e2_hal::pac;
use gd32e2_hal::rcu::{ClockConfig, PllFreq, SysClk};
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

### Сборка

> **Прошивка и отладка по SWD** (ST-Link V2 + `probe-rs`, `PA13` / `PA14`). Вывод из прошивки идёт по RTT через тот же зонд (`defmt` + `defmt-rtt`).

- Таргет `thumbv8m.base-none-eabi`;
- `build.rs` пишет нужный линкеру `memory.x` из выбранной фичи библиотеки, копировать ничего не надо. (`memory.x` в корне проекта всё равно перебивает сгенерированный — линкер смотрит туда раньше путей поиска, — и это штатный выход для платы, которой нет в таблице);

#### Алиасы cargo

```shell
cargo lib             # только библиотека, алиас для build --features gd32e230c8xx
cargo be usart-echo   # проверить сборку одного примера, зонд не нужен
cargo bre usart-echo  # то же в release
cargo re usart-echo   # сборка + прошивка по SWD, дальше остаётся подключённым
```

### Roadmap

**Периферия, не покрытая вовсе**
- [ ] `PMU` — sleep / deep-sleep / standby, wakeup-нога, LDO.
- [ ] Остаток `SYSCFG` — ремап `PA11`/`PA12` на площадки `PA9`/`PA10` ниже 32 выводов и вторая карта каналов DMA.
- [ ] `RTC` — календарь. Без кварца пойдёт от IRC40K, с его же точностью.

**Доделать имеющееся**
- [ ] Прерывания DMA — форма `Event` / `listen` устоялась, работа механическая.
- [ ] DMA: циклический режим, `M2M`, `embedded-dma` для буферов.
- [ ] ADC: scan по нескольким каналам, для чего нужен DMA.
- [ ] Таймеры: комплементарные выходы, break и dead time; `TRGO` у `TIMER5`, чтобы он мог запускать DMA.
- [ ] I²C: 10-битная адресация, SMBus, slave, DMA.
- [ ] SPI: half-duplex и однопроводные режимы (`BDEN`/`BDOEN`/`RO`); аппаратный NSS, CRC, TI mode и slave — низкий приоритет.
- [ ] USART: аппаратное управление потоком (`CTS`/`RTS`); неблокирующий API для 9-битных слов.
- [ ] FWDGT: оконный режим (`WND`); `FWDGT_HOLD` и `WWDGT_HOLD` в модуле DBG, чтобы сторожа не считали под остановленным отладчиком.
- [ ] RCU: `HXTAL` и `LXTAL` (ни один кварц на плате не запаян).
- [ ] FMC: `embedded-storage` (`ReadNorFlash` / `NorFlash`) поверх готовых записи и стирания.

**Инфраструктура**
- [ ] Поддержка gd32e231x: фичи партномеров, карта пинов, гейты. PAC-модуль есть, чипа на руках нет.
- [ ] Async: `embedded-hal-async` / `embedded-io-async`, начиная с драйвера времени поверх одного из таймеров — именно его Embassy ждёт от HAL.
- [ ] Тесты на хосте и CI — их нет вовсе; крейт не собирается под хост, поэтому нужны моки.
- [ ] Вынос HAL в отдельный крейт/репозиторий.

### Лицензия

Двойное лицензирование, на выбор:
- Apache License 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT ([LICENSE-MIT](LICENSE-MIT))

Любой вклад, отправленный для включения в эту работу, лицензируется так же, без дополнительных условий, если не оговорено иное (Apache-2.0, раздел 5).
