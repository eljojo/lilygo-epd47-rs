use alloc::{boxed::Box, vec};

use esp_hal::{delay::Delay, peripherals};
use log::*;

use crate::{ed047tc1, waveform, Error, Result};

const CONTRAST_CYCLES_4BPP: &[u16; 15] = &[
    30, 30, 20, 20, 30, 30, 30, 40, 40, 50, 50, 50, 100, 200, 300,
];
const CONTRAST_CYCLES_4BPP_WHITE: &[u16; 15] =
    &[10, 10, 8, 8, 8, 8, 8, 10, 10, 15, 15, 20, 20, 100, 300];

/// FastEPD (bitbank2) `u8M5Matrix`, backported verbatim from its `src/FastEPD.inl`. One row per gray
/// level (row 0 = black .. row 15 = white — the same convention as our framebuffer), 8 columns = 8
/// passes. Each entry is a per-pass push: `1` = push toward black, `2` = push toward white. (M5's matrix
/// has no `0`/skip entries, so every pass drives every pixel.) This is the fast 8-pass alternative to
/// the 46-phase epdiy waveform, exposed via [`Display::flush_matrix`] for A/B comparison.
const M5_MATRIX: [[u8; 8]; 16] = [
    [1, 1, 1, 1, 1, 1, 1, 1],
    [2, 2, 1, 1, 2, 1, 1, 1],
    [2, 2, 1, 1, 1, 1, 2, 1],
    [2, 2, 1, 1, 2, 2, 1, 1],
    [2, 2, 2, 2, 1, 1, 2, 1],
    [2, 2, 1, 1, 1, 2, 2, 1],
    [2, 2, 1, 1, 2, 1, 1, 2],
    [2, 2, 2, 1, 2, 1, 1, 2],
    [2, 2, 2, 2, 2, 1, 2, 1],
    [1, 1, 1, 1, 1, 1, 2, 2],
    [2, 2, 1, 1, 1, 1, 2, 2],
    [1, 1, 1, 1, 2, 1, 2, 2],
    [2, 2, 1, 1, 2, 1, 2, 2],
    [2, 1, 1, 2, 2, 1, 2, 2],
    [2, 2, 1, 2, 2, 1, 2, 2],
    [2, 2, 2, 2, 2, 2, 2, 2],
];

/// Per-row CKV dwell for the matrix path. FastEPD adds no per-row dwell — each row's drive time is just
/// the DMA transfer (~12.4 µs/line at 20 MHz) — so we keep this DMA-bound (any value whose dwell, in
/// `output_time × 24` CPU cycles, is below the ~12.4 µs DMA leaves the line DMA-bound). The real
/// inter-frame settle is the 230 µs between passes (see `draw_matrix`), matching FastEPD.
const MATRIX_DWELL: u16 = 30;

/// Per-phase CKV dwell for the calibrated waveform path. The epdiy waveform uses uniform phase timing
/// (its `phase_times` is NULL) and the calibrated phase counts assume ~15.5 µs/line.
///
/// At the original 16 MHz bus this was DMA-bound: the 248-byte scanline took ~15.5 µs and a small dwell
/// (30) was subsumed. Now the bus runs at 20 MHz (FastEPD's M5 speed) so DMA is only ~12.4 µs/line; to
/// keep the waveform calibration valid we hold the line at ~15.5 µs by making it *dwell*-bound instead:
/// 15.5 µs × 240 MHz ÷ 24 cycles/tick ≈ 155 ticks. The faster bus still speeds up the non-calibrated
/// `clear`/`draw` paths; only this grayscale path is deliberately pinned back to its calibrated timing.
const WF_DWELL: u16 = 155;

/// Display rotation, only 90° increments supported
#[derive(Clone, Copy, Default)]
pub enum DisplayRotation {
    /// No rotation
    #[default]
    Rotate0,
    /// Rotate by 90 degrees clockwise
    Rotate90,
    /// Rotate by 180 degrees clockwise
    Rotate180,
    /// Rotate 270 degrees clockwise
    Rotate270,
}

#[derive(Clone, Copy, Debug)]
pub enum DrawMode {
    BlackOnWhite,
    WhiteOnWhite,
    WhiteOnBlack,
}

#[derive(Clone, Copy, Debug)]
pub struct Rectangle {
    pub x: u16,
    pub y: u16,
    pub width: u16,
    pub height: u16,
}

impl DrawMode {
    fn lut_default(&self) -> u8 {
        match self {
            Self::BlackOnWhite => 0x55,
            Self::WhiteOnBlack | Self::WhiteOnWhite => 0xAA,
        }
    }

    fn contrast_cycles(&self) -> &[u16; 15] {
        match self {
            Self::WhiteOnBlack => CONTRAST_CYCLES_4BPP_WHITE,
            Self::BlackOnWhite | Self::WhiteOnWhite => CONTRAST_CYCLES_4BPP,
        }
    }
}

const FRAMEBUFFER_SIZE: usize = (Display::WIDTH / 2) as usize * Display::HEIGHT as usize;
const BYTES_PER_LINE: usize = Display::WIDTH as usize / 4;
const LINE_BYTES_4BPP: usize = Display::WIDTH as usize / 2;

/// 2-bpp (4-gray) framebuffer: 4 pixels per byte, level 0=black, 1=dark, 2=light, 3=white. One byte row
/// is exactly [`BYTES_PER_LINE`] (= WIDTH/4) bytes — the same width as a packed scanline.
const FB2_SIZE: usize = BYTES_PER_LINE * Display::HEIGHT as usize;

/// FastEPD 2-bpp partial-update transition LUTs, indexed by `(new << 2) | old` over the 4 gray levels.
/// `GRAY2_BW16` is the first phase (drive toward the black/white anchor), `GRAY2_GRAY16` the final
/// gray-split phase. Values are panel 2-bit push codes: `0b01` push black, `0b10` push white, and `0b11`
/// for "do nothing". (FastEPD uses `0b00` as its no-field code, but on THIS M5PaperS3 wiring `0b00`
/// faintly DARKENS — confirmed on-hardware as a slow background "boom" under sustained partials — so the
/// inert code here is `0b11`, the true no-op. This is the one spot we deviate from FastEPD's `0b00`.)
const GRAY2_BW16: [u8; 16] = [
    0b11, 0b01, 0b01, 0b01, 0b11, 0b11, 0b11, 0b01, 0b10, 0b11, 0b11, 0b11, 0b10, 0b10, 0b10, 0b11,
];
const GRAY2_GRAY16: [u8; 16] = [
    0b11, 0b11, 0b11, 0b11, 0b10, 0b11, 0b01, 0b10, 0b01, 0b10, 0b11, 0b01, 0b11, 0b11, 0b10, 0b11,
];

pub struct Display<'a> {
    epd: ed047tc1::ED047TC1<'a>,
    skipping: u16,
    framebuffer: Box<[u8; FRAMEBUFFER_SIZE]>,
    rotation: DisplayRotation,
    /// 2-bpp current/previous frames for the FastEPD-style 4-gray path ([`flush_gray2`]/[`partial_gray2`]).
    /// Lazily allocated on first 2-bpp use so 4-bpp-only callers pay no extra RAM.
    fb2: Option<Box<[u8; FB2_SIZE]>>,
    prev2: Option<Box<[u8; FB2_SIZE]>>,
}

impl<'a> Display<'a> {
    /// Width of the screen.
    pub const WIDTH: u16 = 960;
    /// Height of the screen
    pub const HEIGHT: u16 = 540;
    /// Bounding Box of the screen.
    pub const BOUNDING_BOX: Rectangle = Rectangle {
        x: 0,
        y: 0,
        width: Self::WIDTH,
        height: Self::HEIGHT,
    };
    pub fn new(
        pins: ed047tc1::PinConfig<'a>,
        dma: peripherals::DMA_CH0<'a>,
        lcd_cam: peripherals::LCD_CAM<'a>,
        rmt: peripherals::RMT<'a>,
    ) -> Result<Self> {
        Ok(Display {
            epd: ed047tc1::ED047TC1::new(pins, dma, lcd_cam, rmt)?,
            skipping: 0,
            framebuffer: Box::new([0xFF; FRAMEBUFFER_SIZE]),
            rotation: DisplayRotation::default(),
            fb2: None,
            prev2: None,
        })
    }

    /// Set the rotation
    pub fn set_rotation(&mut self, rotation: DisplayRotation) {
        self.rotation = rotation;
    }

    /// Get rotation
    pub fn rotation(&self) -> DisplayRotation {
        self.rotation
    }

    /// Turn the display on.
    pub fn power_on(&mut self) {
        debug!("Display power on");
        self.epd.power_on()
    }

    /// Turn the display off.
    pub fn power_off(&mut self) {
        debug!("Display power off");
        self.epd.power_off()
    }

    /// Sets a single pixel in the framebuffer without updating the display.
    ///
    /// If the provided coordinates are outside the screen, this method returns
    /// [Error::OutOfBounds]. If the provided color is greater than 0x0F,
    /// this method returns [Error::InvalidColor].
    pub fn set_pixel(&mut self, x: u16, y: u16, color: u8) -> Result<()> {
        if x >= Self::WIDTH || y >= Self::HEIGHT {
            return Err(Error::OutOfBounds);
        }
        if color > 0x0F {
            return Err(Error::InvalidColor);
        }
        // Calculate the index in the framebuffer.
        let index: usize = x as usize / 2 + y as usize * (Self::WIDTH as usize / 2);
        let value = self.framebuffer[index];
        if x % 2 == 1 {
            self.framebuffer[index] = (value & 0x0F) | ((color << 4) & 0xF0);
        } else {
            self.framebuffer[index] = (value & 0xF0) | (color & 0x0F);
        }
        Ok(())
    }

    /// Fill the whole framebuffer with the same color.
    pub fn fill(&mut self, color: u8) -> Result<()> {
        debug!("display fill");
        if color > 0x0F {
            return Err(Error::InvalidColor);
        }
        self.framebuffer.fill(color << 4 | color);
        Ok(())
    }

    /// Flush updates the display with the contents of the framebuffer. The
    /// method clears the framebuffer. The provided mode should match the
    /// contents of your framebuffer.
    pub fn flush(&mut self, mode: DrawMode) -> Result<()> {
        debug!("display flush");
        self.draw(mode)?;
        self.framebuffer.fill(0xFF);
        Ok(())
    }

    /// Calibrated grayscale flush: replays the vendored epdiy ED047TC2 GC16 waveform (mode 5) for the
    /// given temperature bucket (`range_idx` 0 = cold .. 6 = hot), from a WHITE start — the caller must
    /// `clear()` first, exactly like the normal full-repaint path. Wipes the framebuffer afterward.
    ///
    /// Unlike [`flush`] (a hand-tuned bit-plane darken-from-white), this drives the real vendor-calibrated
    /// per-gray phase sequence: each pixel's target gray indexes the waveform's white-source column, and
    /// each phase emits darken / lighten / no-op so the NET drive lands the calibrated gray level.
    pub fn flush_waveform(&mut self, range_idx: usize) -> Result<()> {
        debug!("display flush_waveform");
        self.draw_waveform(waveform::mode5_bucket(range_idx))?;
        self.framebuffer.fill(0xFF);
        Ok(())
    }

    /// Fast grayscale flush via FastEPD's 8-pass [`M5_MATRIX`], from a WHITE start (caller must `clear()`
    /// first, like [`flush_waveform`]). This is the A/B counterpart to [`flush_waveform`]: 8 passes
    /// instead of 46, no temperature compensation, ~5–6× fewer full-screen scans. Wipes the framebuffer
    /// afterward. Compare the two on the same image to judge the speed/fidelity trade-off.
    pub fn flush_matrix(&mut self) -> Result<()> {
        debug!("display flush_matrix");
        self.draw_matrix()?;
        self.framebuffer.fill(0xFF);
        Ok(())
    }

    /// Allocate the 2-bpp current/previous buffers (initialised white) on first 2-bpp use.
    fn ensure_gray2(&mut self) {
        if self.fb2.is_none() {
            self.fb2 = Some(Box::new([0xFF; FB2_SIZE]));
        }
        if self.prev2.is_none() {
            self.prev2 = Some(Box::new([0xFF; FB2_SIZE]));
        }
    }

    /// Copy the 2-bpp current frame into the previous frame (call after a full or partial 2-bpp update so
    /// `previous` tracks what is now on the panel).
    fn snapshot_gray2(&mut self) {
        if let (Some(prev), Some(cur)) = (self.prev2.as_mut(), self.fb2.as_ref()) {
            prev[..].copy_from_slice(&cur[..]);
        }
    }

    /// Re-anchor the 2-bpp world from the 16-gray framebuffer: quantise every 4-bpp pixel (luma >> 2) into
    /// `fb2`, then set `prev2 = fb2`. Call this right after rendering a 16-gray frame and BEFORE
    /// [`flush_matrix`](Self::flush_matrix) wipes the framebuffer. It makes the just-painted image read as
    /// "unchanged" to the next [`partial_gray2`](Self::partial_gray2) (which then emits `0b00` over the
    /// image region and leaves the 16-gray pixels untouched), so a fast 4-gray partial preserves the
    /// 16-gray image beneath it.
    pub fn sync_gray2_from_framebuffer(&mut self) {
        self.ensure_gray2();
        let Self {
            framebuffer,
            fb2,
            prev2,
            ..
        } = self;
        let fb2 = fb2.as_mut().expect("ensure_gray2");
        // fb2 byte b (4 pixels, leftmost at LSB) <- framebuffer bytes 2b/2b+1 (2 pixels each, even=low nibble).
        for (b, dst) in fb2.iter_mut().enumerate() {
            let lo = framebuffer[b * 2]; // screen pixels 4b (low nibble) + 4b+1 (high nibble)
            let hi = framebuffer[b * 2 + 1]; // screen pixels 4b+2 (low) + 4b+3 (high)
            *dst = ((lo & 0x0F) >> 2)
                | (((lo >> 4) >> 2) << 2)
                | (((hi & 0x0F) >> 2) << 4)
                | (((hi >> 4) >> 2) << 6);
        }
        if let Some(prev) = prev2.as_mut() {
            prev[..].copy_from_slice(&fb2[..]);
        }
    }

    /// Set one pixel in the 2-bpp (4-gray) framebuffer: `level` 0=black, 1=dark, 2=light, 3=white. Raw
    /// coordinates (no rotation) — the [`gray2`](Self::gray2) embedded-graphics target applies rotation.
    pub fn set_pixel_gray2(&mut self, x: u16, y: u16, level: u8) -> Result<()> {
        if x >= Self::WIDTH || y >= Self::HEIGHT {
            return Err(Error::OutOfBounds);
        }
        if level > 3 {
            return Err(Error::InvalidColor);
        }
        self.ensure_gray2();
        let fb = self.fb2.as_mut().expect("ensure_gray2");
        let idx = x as usize / 4 + y as usize * BYTES_PER_LINE;
        let shift = (x % 4) as usize * 2;
        fb[idx] = (fb[idx] & !(0b11 << shift)) | (level << shift);
        Ok(())
    }

    /// Fill the whole 2-bpp framebuffer with one gray level (0..=3).
    pub fn fill_gray2(&mut self, level: u8) -> Result<()> {
        if level > 3 {
            return Err(Error::InvalidColor);
        }
        self.ensure_gray2();
        let byte = level | level << 2 | level << 4 | level << 6;
        self.fb2.as_mut().expect("ensure_gray2").fill(byte);
        Ok(())
    }

    /// Full 4-gray repaint via FastEPD's 2-bpp update: clear to white, then 5 black/white passes + 1 gray
    /// pass. Establishes `previous = current` so subsequent [`partial_gray2`](Self::partial_gray2) calls
    /// diff correctly — call this once to anchor before driving partials.
    pub fn flush_gray2(&mut self) -> Result<()> {
        debug!("display flush_gray2");
        self.ensure_gray2();
        self.clear()?; // white anchor on the panel
        // White-start push tables per level 0..3: BW pushes blacks only (rest already white -> 0b11 inert),
        // GRAY splits. (0b11 = no-op on this panel; see GRAY2_BW16 — never 0b00.)
        let bw = expand_gray2_lut(&[0b01, 0b01, 0b11, 0b11]);
        let gray = expand_gray2_lut(&[0b01, 0b10, 0b01, 0b10]);
        let delay = Delay::new();
        {
            let Self { epd, fb2, .. } = self;
            let fb2 = fb2.as_ref().expect("ensure_gray2");
            for (lut, count) in [(&bw, 5usize), (&gray, 1usize)] {
                for _ in 0..count {
                    epd.frame_start()?;
                    epd.output_frame(Self::HEIGHT, MATRIX_DWELL, |y, line| {
                        let off = y as usize * BYTES_PER_LINE;
                        let row = &fb2[off..off + BYTES_PER_LINE];
                        for (dst, &src) in line.iter_mut().zip(row.iter()) {
                            *dst = lut[src as usize];
                        }
                    })?;
                    epd.frame_end()?;
                    delay.delay_micros(230);
                }
            }
        }
        self.snapshot_gray2();
        Ok(())
    }

    /// Flicker-free 4-gray partial update: diffs the 2-bpp framebuffer against the previously displayed
    /// frame and pushes only the per-pixel transitions (FastEPD's `bbep2BppPartial`: a 16-entry old×new
    /// transition LUT driving 5 BW passes + 1 gray pass). Updates `previous = current`. Requires a prior
    /// [`flush_gray2`](Self::flush_gray2) to anchor `previous`. Ends with the neutral [`discharge`], as
    /// FastEPD does.
    pub fn partial_gray2(&mut self) -> Result<()> {
        debug!("display partial_gray2");
        self.ensure_gray2();
        let delay = Delay::new();
        {
            let Self {
                epd, fb2, prev2, ..
            } = self;
            let fb2 = fb2.as_ref().expect("ensure_gray2");
            let prev2 = prev2.as_ref().expect("ensure_gray2");
            for (table, count) in [(&GRAY2_BW16, 5usize), (&GRAY2_GRAY16, 1usize)] {
                for _ in 0..count {
                    epd.frame_start()?;
                    epd.output_frame(Self::HEIGHT, MATRIX_DWELL, |y, line| {
                        let off = y as usize * BYTES_PER_LINE;
                        build_gray2_transition(
                            &fb2[off..off + BYTES_PER_LINE],
                            &prev2[off..off + BYTES_PER_LINE],
                            table,
                            line,
                        );
                    })?;
                    epd.frame_end()?;
                    delay.delay_micros(230);
                }
            }
        }
        self.snapshot_gray2();
        Ok(())
    }

    /// Borrow the display as a 2-bpp (4-gray) embedded-graphics `DrawTarget` (colour [`Gray2`]). Draw into
    /// it, then call [`flush_gray2`](Self::flush_gray2) (first paint) or
    /// [`partial_gray2`](Self::partial_gray2) (subsequent updates).
    #[cfg(feature = "embedded-graphics")]
    pub fn gray2(&mut self) -> crate::graphics::Gray2Canvas<'_, 'a> {
        crate::graphics::Gray2Canvas(self)
    }

    /// Clears the screen.
    pub fn clear(&mut self) -> Result<()> {
        debug!("display clear");
        self.clear_area(Self::BOUNDING_BOX)
    }

    /// Performs the screen repair routine as described here
    /// https://github.com/Xinyuan-LilyGO/LilyGo-EPD47/blob/master/examples/screen_repair/screen_repair.ino
    pub fn repair(&mut self, delay: Delay) -> Result<()> {
        debug!("display repair");
        self.clear()?;
        for _ in 0..20 {
            self.push_pixels(Self::BOUNDING_BOX, 50, 0)?;
            delay.delay_millis(500);
        }
        self.clear()?;
        for _ in 0..40 {
            self.push_pixels(Self::BOUNDING_BOX, 50, 1)?;
            delay.delay_millis(500);
        }
        self.clear()
    }

    pub fn clear_area(&mut self, area: Rectangle) -> Result<()> {
        self.clear_cycles(area, 4, 50)
    }

    /// Flashing clear: drive the area hard to black then white, repeated, to slam all particles to the
    /// rails and reset state (the e-ink "full refresh" erase). Also wipes accumulated DC drift from the
    /// 2-bpp partials, so it's the balanced full-refresh the drift-wipe cadence relies on (see firmware
    /// display.rs GHOST_*). This is the original (fast) recipe — the self-erasing GC16 waveform that
    /// follows it needs no more; the deeper FastEPD CLEAR_SLOW was only for the (parked) matrix path.
    fn clear_cycles(&mut self, area: Rectangle, cycles: u16, cycle_time: u16) -> Result<()> {
        for _ in 0..cycles {
            for _ in 0..4 {
                self.push_pixels(area, cycle_time, 0)?;
            }
            for _ in 0..4 {
                self.push_pixels(area, cycle_time, 1)?;
            }
        }
        Ok(())
    }

    fn push_pixels(&mut self, area: Rectangle, time: u16, color: u16) -> Result<()> {
        let mut row = [0u8; BYTES_PER_LINE];

        for i in 0..area.width {
            let pos = i + area.x % 4;
            let mask = match color {
                1 => 0b10101010,
                _ => 0b01010101,
            } & (0b00000011 << (2 * (pos % 4)));
            row[(area.x / 4 + pos / 4) as usize] |= mask;
        }
        line_buffer_reorder(&mut row);
        self.skipping = 0;
        self.epd.frame_start()?;

        // Clock rows up to the bottom of the rect, then end the frame early: rows BELOW the rect are
        // left un-driven (the gate re-homes via SPV at the next frame_start) — that's both correct
        // (they keep their image) and fast. Rows ABOVE the rect must be inert-clocked (row_skip) to
        // advance the gate to the rect; rows below it need not be touched at all.
        let last = (area.y + area.height).min(Self::HEIGHT);
        for i in 0..last {
            if i < area.y {
                self.row_skip()?;
            } else if i == area.y {
                self.epd.set_buffer(&row)?;
                self.row_write(time)?;
            } else {
                self.row_write(time)?;
            }
        }
        self.epd.frame_end()?;

        Ok(())
    }

    // Inert gate-advance row. We must DMA-stream a real line (a bare CKV-only skip with no DMA desyncs
    // this panel's continuously-clocked gate and corrupts the image); the streamed data is the panel's
    // no-op code 0b11 (= 0xFF), NOT 0x00. On THIS M5PaperS3 wiring 0x00 faintly DARKENS the clocked rows
    // (confirmed on-hardware), so an inert row must be 0xFF to leave the advance region truly untouched.
    // The buffer is loaded once per skip run (`skipping` resets to 0 on every driven row); output_row(1):
    // the data is inert so no CKV dwell beyond the DMA is needed, keeping the inert clock cheap.
    fn row_skip(&mut self) -> Result<()> {
        if self.skipping == 0 {
            self.epd.set_buffer(&[0xFFu8; BYTES_PER_LINE])?;
        }
        self.epd.output_row(1)?;
        self.skipping += 1;

        Ok(())
    }

    fn row_write(&mut self, output_time: u16) -> Result<()> {
        self.skipping = 0;
        self.epd.output_row(output_time)?;

        Ok(())
    }

    const DRAW_IMAGE_FRAME_COUNT: usize = 15;
    fn draw(&mut self, mode: DrawMode) -> Result<()> {
        // let start = esp_hal::time::current_time();

        // init lut
        let mut lut = vec![mode.lut_default(); 1 << 16];

        // Reference invariant (M5GFX / epdiy LCD): drive EVERY physical row every subframe with its
        // real LUT-resolved line — never skip/advance rows. Under static-high OE every clocked row
        // DRIVES, and charge balance only holds if each row carries balanced waveform content; an
        // inert constant "skip" line injects net DC that integrates into the column-axis drift + the
        // threshold "boom". Untainted rows are white in the framebuffer and resolve to a balanced
        // no-op within the waveform, so they cost time but don't change visually or accumulate charge.
        let Self {
            epd, framebuffer, ..
        } = self;
        for k in 0..Self::DRAW_IMAGE_FRAME_COUNT {
            // update lut
            update_lut(&mut lut, k, mode);
            // start draw
            epd.frame_start()?;
            // build line — every row, real content; pipelined so pixel-prep overlaps the DMA.
            let dwell = mode.contrast_cycles()[k];
            epd.output_frame(Self::HEIGHT, dwell, |y, line| {
                let start = y as usize * LINE_BYTES_4BPP;
                prepare_dma_buffer(&framebuffer[start..start + LINE_BYTES_4BPP], &lut, line);
            })?;
            epd.frame_end()?;
        }
        // println!(
        //     "draw_fb {}",
        //     (esp_hal::time::current_time() - start).to_millis()
        // );
        Ok(())
    }

    /// Replay a calibrated epdiy waveform `[phase][16 target-gray][4 bytes]` from a WHITE source. For each
    /// phase, the per-target-gray op (white-source column) is mapped to the panel's 2-bit codes and
    /// expanded into the 16-bit→byte conversion LUT, then every one of the 540 rows is driven (reference
    /// invariant — see `draw`: under static-high OE every clocked row must carry balanced content).
    fn draw_waveform(&mut self, phases: &[[[u8; 4]; 16]]) -> Result<()> {
        let mut lut = vec![0u8; 1 << 16];
        for phase in phases {
            // White-source op per target gray g (framebuffer white = 0x0F), mapped epdiy -> panel codes:
            //   1 darken -> 0b01, 2 lighten -> 0b10, 0 no-op -> 0b11 (true inert; 0b00 faintly darkens).
            let mut op = [0u8; 16];
            for (g, slot) in op.iter_mut().enumerate() {
                *slot = match epdiy_op(&phase[g], 0x0F) {
                    1 => 0b01,
                    2 => 0b10,
                    _ => 0b11,
                };
            }
            // Expand the 16-entry op table to the 4-pixel (16-bit index -> packed byte) conversion LUT
            // that prepare_dma_buffer expects.
            for (idx, cell) in lut.iter_mut().enumerate() {
                *cell = op[idx & 0xF]
                    | (op[(idx >> 4) & 0xF] << 2)
                    | (op[(idx >> 8) & 0xF] << 4)
                    | (op[(idx >> 12) & 0xF] << 6);
            }
            let Self {
                epd, framebuffer, ..
            } = self;
            epd.frame_start()?;
            epd.output_frame(Self::HEIGHT, WF_DWELL, |y, line| {
                let start = y as usize * LINE_BYTES_4BPP;
                prepare_dma_buffer(&framebuffer[start..start + LINE_BYTES_4BPP], &lut, line);
            })?;
            epd.frame_end()?;
        }
        Ok(())
    }

    /// Replay FastEPD's 8-pass [`M5_MATRIX`] from a WHITE source. Each pass maps every gray level to its
    /// per-pass push code (`1` darken -> `0b01`, `2` lighten -> `0b10`) and drives every row through the
    /// same LUT/pipeline as [`draw_waveform`]; passes are separated by a 230 µs settle, as FastEPD does.
    fn draw_matrix(&mut self) -> Result<()> {
        let mut lut = vec![0u8; 1 << 16];
        let delay = Delay::new();
        for pass in 0..M5_MATRIX[0].len() {
            // Per-gray op for this pass -> panel 2-bit codes (framebuffer gray indexes the matrix row).
            let mut op = [0u8; 16];
            for (g, slot) in op.iter_mut().enumerate() {
                *slot = match M5_MATRIX[g][pass] {
                    1 => 0b01,
                    2 => 0b10,
                    _ => 0b11, // no-op (0b11 is the inert code on this panel; M5 matrix has no such entry)
                };
            }
            // Expand to the 4-pixel (16-bit index -> packed byte) LUT prepare_dma_buffer expects.
            for (idx, cell) in lut.iter_mut().enumerate() {
                *cell = op[idx & 0xF]
                    | (op[(idx >> 4) & 0xF] << 2)
                    | (op[(idx >> 8) & 0xF] << 4)
                    | (op[(idx >> 12) & 0xF] << 6);
            }
            let Self {
                epd, framebuffer, ..
            } = self;
            epd.frame_start()?;
            epd.output_frame(Self::HEIGHT, MATRIX_DWELL, |y, line| {
                let start = y as usize * LINE_BYTES_4BPP;
                prepare_dma_buffer(&framebuffer[start..start + LINE_BYTES_4BPP], &lut, line);
            })?;
            epd.frame_end()?;
            // Inter-pass settle, matching FastEPD's `delayMicroseconds(230)`.
            delay.delay_micros(230);
        }
        Ok(())
    }
}

/// Decode the epdiy waveform 2-bit op for source gray `from` (0..15) from one `[16 target-gray][4 byte]`
/// phase row's 4-byte source-gray pack. 16 source grays are packed 2 bits each across the 4 bytes,
/// MSB-first within a byte (verified by decoding the vendored ED047TC2 table). Returns 0 = no-op,
/// 1 = darken, 2 = lighten.
fn epdiy_op(quad: &[u8; 4], from: usize) -> u8 {
    let byte = quad[from / 4];
    let shift = (3 - (from % 4)) * 2;
    (byte >> shift) & 0b11
}

fn line_buffer_reorder(data: &mut [u8]) {
    // Iterate over the data in chunks of 4 bytes (size of a u32)
    for chunk in data.chunks_exact_mut(4) {
        // Convert the 4-byte chunk to a u32, swap the high and low 16 bits, and then
        // write it back
        let val = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        let swapped = (val >> 16) | ((val & 0x0000FFFF) << 16);
        chunk.copy_from_slice(&swapped.to_le_bytes());
    }
}

fn prepare_dma_buffer(line_data: &[u8], conversion_lut: &[u8], out: &mut [u8; BYTES_PER_LINE]) {
    // The input is 4bpp (2 pixels per byte), which we process in 8-byte chunks:
    // 8 bytes => 4 u16s => 16 pixels.
    //
    // The output is 2bpp packed (16 pixels => 32 bits => 4 bytes).
    for (j, chunk) in line_data.chunks_exact(8).enumerate() {
        let v1 = u16::from_le_bytes([chunk[0], chunk[1]]);
        let v2 = u16::from_le_bytes([chunk[2], chunk[3]]);
        let v3 = u16::from_le_bytes([chunk[4], chunk[5]]);
        let v4 = u16::from_le_bytes([chunk[6], chunk[7]]);

        let pixel: u32 = (conversion_lut[v1 as usize] as u32)
            | (conversion_lut[v2 as usize] as u32) << 8
            | (conversion_lut[v3 as usize] as u32) << 16
            | (conversion_lut[v4 as usize] as u32) << 24;

        out[j * 4..(j + 1) * 4].copy_from_slice(&pixel.to_le_bytes());
    }
}

/// Expand a per-level (0..3) 2-bit push-code table into a 256-entry source-byte -> push-byte LUT,
/// position-preserving (each of the 4 packed pixels keeps its slot). Used by the 2-bpp full update.
fn expand_gray2_lut(code: &[u8; 4]) -> [u8; 256] {
    let mut t = [0u8; 256];
    for (b, slot) in t.iter_mut().enumerate() {
        *slot = code[b & 3]
            | (code[(b >> 2) & 3] << 2)
            | (code[(b >> 4) & 3] << 4)
            | (code[(b >> 6) & 3] << 6);
    }
    t
}

/// Build one packed 2-bpp push line from the new/previous gray rows via a 16-entry `(new<<2)|old`
/// transition table. Position-preserving; the result is swizzled to panel order in `output_frame`.
fn build_gray2_transition(
    new_row: &[u8],
    old_row: &[u8],
    table: &[u8; 16],
    out: &mut [u8; BYTES_PER_LINE],
) {
    for ((dst, &nb), &ob) in out.iter_mut().zip(new_row.iter()).zip(old_row.iter()) {
        *dst = table[((nb & 3) << 2 | (ob & 3)) as usize]
            | (table[(((nb >> 2) & 3) << 2 | ((ob >> 2) & 3)) as usize] << 2)
            | (table[(((nb >> 4) & 3) << 2 | ((ob >> 4) & 3)) as usize] << 4)
            | (table[(((nb >> 6) & 3) << 2 | ((ob >> 6) & 3)) as usize] << 6);
    }
}

fn update_lut(conversion_lut: &mut [u8], k: usize, mode: DrawMode) {
    let k = match mode {
        DrawMode::BlackOnWhite | DrawMode::WhiteOnWhite => Display::DRAW_IMAGE_FRAME_COUNT - k,
        DrawMode::WhiteOnBlack => k,
    };
    // reset the pixels which are not to be lightened / darkened
    // any longer in the current frame
    for l in (k..1 << 16).step_by(16) {
        conversion_lut[l] &= 0xFC;
    }
    for l in ((k << 4)..(1 << 16)).step_by(1 << 8) {
        for p in 0..16 {
            conversion_lut[l + p] &= 0xF3
        }
    }
    for l in ((k << 8)..(1 << 16)).step_by(1 << 12) {
        for p in 0..(1 << 8) {
            conversion_lut[l + p] &= 0xCF
        }
    }
    for l in (k << 12)..((k + 1) << 12) {
        conversion_lut[l] &= 0x3F;
    }
}
