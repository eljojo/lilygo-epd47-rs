# LilyGo EPD47 Rust HAL

![Demo](_docs/hello-world.jpg)

Simple driver for
the [LilyGo T5 4.7 Inch E-Paper display](https://www.lilygo.cc/en-pl/products/t5-4-7-inch-e-paper-v2-3)
and the [M5Stack PaperS3](https://docs.m5stack.com/en/core/papers3).
The LilyGo backend only supports the V2.3 hardware variant (ESP32-S3).

It should also work on the LilyGo touch version, but I don't have the necessary hardware to validate that claim.

> **This is a fork.** It is based on [`fridolin-koch/lilygo-epd47-rs`] at commit [`352dbd5`] (which
> already carries the esp-hal 1.1 upgrade, display rotation, and the battery-read fix) and adds
> M5Stack PaperS3 support on top — see [Fork changes](#fork-changes) below.

[`fridolin-koch/lilygo-epd47-rs`]: https://github.com/fridolin-koch/lilygo-epd47-rs
[`352dbd5`]: https://github.com/fridolin-koch/lilygo-epd47-rs/commit/352dbd55570c69404a4cad14e4d0632743f7dde0

This library depends on `alloc` and requires you to set up the global allocator for the PSRAM. This is mainly due to
space requirements of the framebuffer and the lut (~325kb).

Built using [`esp-hal`] and [`embedded-graphics`]

[`esp-hal`]: https://github.com/esp-rs/esp-hal

[`embedded-graphics`]: https://docs.rs/embedded-graphics/

**WARNING:**

This is an experimental port of the C library. I ported the basic functionality and tried to simplify it as much as
possible. I give no guarantee that this is the correct usage of the hardware, use at your own risk!

## Usage

1. Prepare your development requirement according to
   this [guide](https://docs.esp-rs.org/book/installation/riscv-and-xtensa.html).
2. Create a new project, I recommend using `cargo-generate` and
   the [template](https://docs.esp-rs.org/book/writing-your-own-application/generate-project/index.html) provided
   by `esp-rs` (i.e. `cargo generate esp-rs/esp-template`)
3. Use the following template for your application and adopt for your needs.

```rust
#![no_std]
#![no_main]
extern crate alloc;

use embedded_graphics::{
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
};
use embedded_graphics_core::pixelcolor::{Gray4, GrayColor};
use esp_backtrace as _;
use esp_hal::{delay::Delay, main};
use lilygo_epd47::{pin_config, Display, DrawMode};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    // Create PSRAM allocator (octal mode for the LilyGo T5 V2.3)
    let psram_config = esp_hal::psram::PsramConfig {
        mode: esp_hal::psram::PsramMode::OctalSpi,
        ..Default::default()
    };
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram, psram_config);
    // Initialise the display. `pin_config!` builds the fixed LilyGo T5 V2.3 wiring;
    // for the M5PaperS3 construct `PinConfig::M5PaperS3(M5PaperS3Pins { .. })` instead.
    let mut display = Display::new(
        pin_config!(peripherals),
        peripherals.DMA_CH0,
        peripherals.LCD_CAM,
        peripherals.RMT,
    )
    .expect("to initialize display");
    // Turn the display on
    display.power_on();
    delay.delay_millis(10);
    // clear the screen
    display.clear().unwrap();
    // Draw a circle with a 3px wide stroke in the center of the screen
    // TODO: Adapt to your requirements (i.e. draw whatever you want)
    Circle::new(display.bounding_box().center() - Point::new(100, 100), 200)
        .into_styled(PrimitiveStyle::with_stroke(Gray4::BLACK, 3))
        .draw(&mut display)
        .unwrap();
    // Flush the framebuffer to the screen
    display.flush(DrawMode::BlackOnWhite).unwrap();
    // Turn the display off again
    display.power_off();
    // do nothing
    loop {}
}
```

## Examples

Run examples like this ` cargo run --release --example <name>`.

- `counter` - Simple counter that updates every second. Only refreshes the screen partially
- `grayscale` - Alternating loop between a horizontal/vertical "gradient" of all the available colors. You may notice
  that the darker colors are harder to distinguish. This is probably due to the waveforms not being used (yet).
- `hello-world` - [`embedded-graphics`] demo. The bmp images used have been converted using
  imagemagick
  `convert <source>.png -size 200x200 -background white -flatten -alpha off -type Grayscale -depth 4 <output>.bmp`
- `screen-repair` - Showcases how to use the repair
  methodology [provided by lilygo](https://github.com/Xinyuan-LilyGO/LilyGo-EPD47/blob/master/examples/screen_repair/screen_repair.ino).
- `rotation` - Demonstrates the four display orientations via `Display::set_rotation`.
- `simple` - Boilerplate, same as the example above.
- `deepsleep` - Deep sleep example. Note: my board suffered from occasional brownouts, I fixed it
  using [this](https://github.com/Xinyuan-LilyGO/LilyGo-EPD47/issues/98#issuecomment-1715584471) modification. I
  measured ~230μA on average during deep sleep using the Nordic PPKII.

## Fork changes

This fork is based on upstream [`fridolin-koch/lilygo-epd47-rs`] at [`352dbd5`]. That base already
includes the **esp-hal 1.1 upgrade**, **display rotation** (`DisplayRotation` /
`Display::set_rotation`, authored by Pierre-Yves Aillet, [@pyaillet](https://github.com/pyaillet),
with the `rotation` example), and the **LilyGo battery-read fix** — those are upstream, not part of
this fork.

What this fork adds on top of that base:

- **M5Stack PaperS3 backend** — `PinConfig` is now an enum (`LilyGoT5V23` / `M5PaperS3`). The
  PaperS3 path adds `M5PaperS3Pins` (type-erased `AnyPin`, so you map your own GPIOs) and a
  direct-GPIO `Bus_EPD` control modeled on M5GFX, the 2bpp pixel-order swizzle, and PaperS3
  scanline padding, all alongside the existing LilyGo T5 backend. `pin_config!` still builds the
  fixed LilyGo T5 V2.3 wiring; build `PinConfig::M5PaperS3` by hand for the PaperS3. (Ported from
  [徐辰 / windoze's fork](https://github.com/windoze/lilygo-epd47-rs).)
- **Calibrated PaperS3 grayscale + landscape-drift fixes** — `Display::flush_waveform(range_idx)`
  replays the vendored epdiy ED047TC2 GC16 waveform (mode 5) from a cleared/white start for true
  vendor-calibrated grays (caller must `clear()` first, same as a full repaint). Because the
  PaperS3 holds output-enable static-high for the whole render pass, the partial-update path now
  drives *every* physical row with its real charge-balanced waveform instead of bare CKV skips —
  this fixes the DC build-up that grayed/corrupted the panel over repeated landscape partials. The
  unused mode-1/mode-2 LUT tables (~8k lines) were dropped.
- **FastEPD-derived fast paths + perf (M5PaperS3)** — backported from
  [FastEPD (bitbank2)](https://github.com/bitbank2/FastEPD), which drives this exact panel:
  - **2-bpp (4-gray) flicker-free updates.** A persistent `current`/`previous` 2-bpp framebuffer
    (lazily allocated) with `flush_gray2()` (full: 5 black/white + 1 gray pass) and `partial_gray2()`
    (FastEPD's `bbep2BppPartial` diff — pushes only the changed pixels via a 16-entry old×new
    transition LUT, no `clear_rect` flash), plus `set_pixel_gray2`/`fill_gray2` and a `Gray2`-colour
    `gray2()` embedded-graphics target. `sync_gray2_from_framebuffer()` re-anchors the 2-bpp buffer
    from the 16-gray framebuffer so a fast 4-gray partial leaves a 16-gray image beneath it untouched.
  - **8-pass matrix grayscale** (`flush_matrix()`, FastEPD's `u8M5Matrix`) — present but **not the
    default**: on this panel it ghosts on large/orientation changes (it is a from-white painter with
    no previous-state ghost cancellation, and relies entirely on a strong flash-clear). `flush_waveform`
    (the epdiy GC16) remains the recommended full-repaint path; the matrix is kept for a future retry
    paired with a stronger clear.
  - **Async DMA pipeline** — `output_frame()` double-buffers scanlines and overlaps pixel-prep with the
    in-flight DMA (FastEPD's `iDMAOff` ping-pong), and the M5 parallel bus runs at **20 MHz** (FastEPD's
    M5 speed) instead of 16.
  - **Panel quirk vs FastEPD.** FastEPD's no-field / neutral code is `0b00`; on *this* M5PaperS3 wiring
    `0b00` faintly **darkens** clocked rows (a slow background "boom" under sustained partials), so the
    inert / no-op / skip code here is `0b11` (`0xFF`) and there is no `0b00` discharge frame. Everything
    else (drive-every-row, 2-bpp diff, matrix) matches FastEPD.

## Todos

- [ ] Basic examples and docs
- [ ] Compare performance to original implementation
- [x] Implement Waveforms / LUT — calibrated GC16 mode-5 waveform wired for the PaperS3
      (`flush_waveform`); the LilyGo bit-plane `flush` path is still hand-tuned

## Credits

This project is largely based on the C implementations provided by:

- [Official LilyGo Driver](https://github.com/Xinyuan-LilyGO/LilyGo-EPD47)
- [epdiy](https://github.com/vroland/epdiy)
- [M5GFX](https://github.com/m5stack/M5GFX) — reference for the M5PaperS3 EPD bus control
- [FastEPD (bitbank2)](https://github.com/bitbank2/FastEPD) — source of the 2-bpp diff, the 8-pass
  grayscale matrix, the async-DMA pipeline, and the 20 MHz M5 bus speed

Rust upstream / forks:

- [`fridolin-koch/lilygo-epd47-rs`] — upstream driver this fork is based on (already includes the
  esp-hal 1.1 upgrade, [@pyaillet](https://github.com/pyaillet)'s display rotation, and the battery fix)
- [windoze/lilygo-epd47-rs](https://github.com/windoze/lilygo-epd47-rs) — M5PaperS3 backend pulled into this fork

## License

Unless otherwise stated the provided code is licensed under the terms of the GNU General Public License v3.0.
