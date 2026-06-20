use esp_hal::{
    dma::DmaTxBuf,
    dma_buffers,
    gpio::{AnyPin, Level, Output, OutputConfig, OutputPin},
    lcd_cam::{
        lcd::{i8080, i8080::Command},
        LcdCam,
    },
    peripherals,
    rmt::PulseCode,
    time::Rate,
    Blocking,
};

use crate::rmt;

macro_rules! pulse {
    ($high:expr, $low:expr) => {
        if $high > 0 {
            [
                PulseCode::new(Level::High, $high, Level::Low, $low),
                PulseCode::end_marker(),
            ]
        } else {
            [
                PulseCode::new(Level::High, $low, Level::Low, 0),
                PulseCode::end_marker(),
            ]
        }
    };
}

/// Bytes of pixel data per scanline (960px @ 2bpp).
const LINE_BYTES: usize = 240;
/// The M5PaperS3 `Bus_EPD` clocks a few extra dummy bytes after each scanline.
const PAPER_S3_LINE_PADDING: usize = 8;
const PAPER_S3_LINE_BYTES: usize = LINE_BYTES + PAPER_S3_LINE_PADDING;
/// The DMA buffer must fit the largest scanline of any supported board.
const DMA_BUFFER_SIZE: usize = PAPER_S3_LINE_BYTES;

struct ConfigRegister {
    latch_enable: bool,
    power_disable: bool,
    pos_power_enable: bool,
    neg_power_enable: bool,
    stv: bool,
    power_enable: bool, /* scan_direction, see https://github.com/vroland/epdiy/blob/main/src/board/epd_board_lilygo_t5_47.c#L199 */
    mode: bool,
    output_enable: bool,
}

impl Default for ConfigRegister {
    fn default() -> Self {
        ConfigRegister {
            latch_enable: false,
            power_disable: true,
            pos_power_enable: false,
            neg_power_enable: false,
            stv: true,
            power_enable: false,
            mode: false,
            output_enable: false,
        }
    }
}

/// LilyGo T5 4.7" config-register shift register (TPS65185-style control bits).
struct ConfigWriter<'a> {
    pin_data: Output<'a>,
    pin_clk: Output<'a>,
    pin_str: Output<'a>,
    config: ConfigRegister,
}

impl<'a> ConfigWriter<'a> {
    fn new(data: impl OutputPin + 'a, clk: impl OutputPin + 'a, str: impl OutputPin + 'a) -> Self {
        ConfigWriter {
            pin_data: Output::new(data, Level::High, OutputConfig::default()),
            pin_clk: Output::new(clk, Level::High, OutputConfig::default()),
            pin_str: Output::new(str, Level::Low, OutputConfig::default()),
            config: ConfigRegister::default(),
        }
    }

    fn write(&mut self) {
        self.pin_str.set_low();
        self.write_bool(self.config.output_enable);
        self.write_bool(self.config.mode);
        self.write_bool(self.config.power_enable);
        self.write_bool(self.config.stv);
        self.write_bool(self.config.neg_power_enable);
        self.write_bool(self.config.pos_power_enable);
        self.write_bool(self.config.power_disable);
        self.write_bool(self.config.latch_enable);
        self.pin_str.set_high();
    }

    #[inline(always)]
    fn write_bool(&mut self, v: bool) {
        self.pin_clk.set_low();
        self.pin_data.set_level(match v {
            true => Level::High,
            false => Level::Low,
        });
        self.pin_clk.set_high();
    }
}

/// M5PaperS3 direct-GPIO panel control, modeled on M5GFX `Bus_EPD`.
///
/// The PaperS3 wires the EPD timing pins (PWR/SPV/OE/LE/CKV) straight to GPIOs
/// instead of going through a config shift register + RMT like the LilyGo T5.
struct M5PaperS3Control<'d> {
    pin_pwr: Output<'d>,
    pin_spv: Output<'d>,
    pin_oe: Output<'d>,
    pin_le: Output<'d>,
    pin_ckv: Output<'d>,
}

impl<'d> M5PaperS3Control<'d> {
    fn new(
        pwr: AnyPin<'d>,
        spv: AnyPin<'d>,
        oe: AnyPin<'d>,
        le: AnyPin<'d>,
        ckv: AnyPin<'d>,
    ) -> Self {
        let output_cfg = OutputConfig::default();

        // Match the M5GFX `Bus_EPD` expectations:
        // - PWR: LOW = off, HIGH = on
        // - OE : LOW = off, HIGH = on
        // - SPV: LOW by default; pulsed in begin_transaction
        // - LE : HIGH between scanlines, LOW during transfer
        // - CKV: toggled around scanline transfer
        Self {
            pin_pwr: Output::new(pwr, Level::Low, output_cfg),
            pin_spv: Output::new(spv, Level::Low, output_cfg),
            pin_oe: Output::new(oe, Level::Low, output_cfg),
            pin_le: Output::new(le, Level::High, output_cfg),
            pin_ckv: Output::new(ckv, Level::High, output_cfg),
        }
    }

    fn power_control(&mut self, on: bool) {
        if on {
            self.pin_oe.set_high();
            busy_delay(100 * 240);
            self.pin_pwr.set_high();
            busy_delay(100 * 240);
            self.pin_spv.set_high();
            busy_delay(1_000 * 240);
        } else {
            busy_delay(1_000 * 240);
            self.pin_pwr.set_low();
            busy_delay(10 * 240);
            self.pin_oe.set_low();
            busy_delay(100 * 240);
            self.pin_spv.set_low();
        }
    }

    fn begin_transaction(&mut self) {
        // Mirrors M5GFX `Bus_EPD::beginTransaction`.
        self.pin_le.set_low();

        self.pin_spv.set_low();
        busy_delay(240);

        self.pin_ckv.set_low();
        busy_delay(3 * 240);

        self.pin_ckv.set_high();
        busy_delay(240);

        self.pin_spv.set_high();

        for _ in 0..3 {
            busy_delay(3 * 240);
            self.pin_ckv.set_low();
            busy_delay(3 * 240);
            self.pin_ckv.set_high();
        }
    }

    fn end_transaction(&mut self) {
        // Mirrors M5GFX `Bus_EPD::endTransaction`.
        self.pin_le.set_low();
        self.pin_ckv.set_high();
    }

    fn scanline_begin(&mut self) -> u32 {
        // Mirrors M5GFX `Bus_EPD::writeScanLine` prelude.
        self.pin_le.set_low();
        self.pin_ckv.set_high();
        cycles()
    }

    fn scanline_end(&mut self, output_time: u16, start_cycles: u32) {
        // Ensure we keep CKV asserted for at least `output_time` ticks.
        // In the original LilyGo driver, `output_time` is expressed in RMT
        // ticks with `clk_divider=8` at 80MHz => 10MHz => 0.1µs.
        //
        // At 240MHz CPU clock, 0.1µs ~= 24 cycles.
        let desired_cycles = output_time as u32 * 24;
        let elapsed = cycles().wrapping_sub(start_cycles);
        if elapsed < desired_cycles {
            busy_delay(desired_cycles - elapsed);
        }

        // Mirrors M5GFX `notify_line_done`.
        self.pin_ckv.set_low();
        self.pin_le.set_high();
    }

    // Bare CKV-only skip (no DMA). INTENTIONALLY UNUSED on the M5PaperS3: with OE static-high a bare
    // skip drives the stale source latch onto the skipped row and desyncs the gate over long runs
    // (grayscale corruption in landscape). The render paths now clock every row with a real DMA line
    // instead. Kept for reference / the LilyGo (RMT) backend's skip path.
    #[allow(dead_code)]
    fn skip_line(&mut self) {
        // Similar to the original LilyGo `skip()` CKV pulse, but using GPIO.
        self.pin_le.set_low();
        self.pin_ckv.set_high();
        busy_delay(45 * 24);
        self.pin_ckv.set_low();
        self.pin_le.set_high();
    }
}

/// Pin configuration for supported boards.
///
/// - [`PinConfig::LilyGoT5V23`] matches the original upstream crate (LilyGo T5
///   4.7" V2.3); build it with the [`pin_config!`](crate::pin_config) macro.
/// - [`PinConfig::M5PaperS3`] matches M5Stack PaperS3 (ESP32-S3) wiring.
pub enum PinConfig<'a> {
    LilyGoT5V23(LilyGoT5V23Pins<'a>),
    M5PaperS3(M5PaperS3Pins<'a>),
}

/// LilyGo T5 4.7" V2.3 pin assignment (fixed GPIOs, see [`pin_config!`]).
pub struct LilyGoT5V23Pins<'a> {
    pub data0: peripherals::GPIO8<'a>,
    pub data1: peripherals::GPIO1<'a>,
    pub data2: peripherals::GPIO2<'a>,
    pub data3: peripherals::GPIO3<'a>,
    pub data4: peripherals::GPIO4<'a>,
    pub data5: peripherals::GPIO5<'a>,
    pub data6: peripherals::GPIO6<'a>,
    pub data7: peripherals::GPIO7<'a>,
    pub cfg_data: peripherals::GPIO13<'a>,
    pub cfg_clk: peripherals::GPIO12<'a>,
    pub cfg_str: peripherals::GPIO0<'a>,
    pub lcd_dc: peripherals::GPIO40<'a>,
    pub lcd_wrx: peripherals::GPIO41<'a>,
    pub rmt: peripherals::GPIO38<'a>,
}

/// M5Stack PaperS3 pin assignment. Pins are type-erased ([`AnyPin`]) so the
/// caller maps their own GPIOs (use `peripherals.GPIOn.degrade()`).
pub struct M5PaperS3Pins<'a> {
    pub data0: AnyPin<'a>,
    pub data1: AnyPin<'a>,
    pub data2: AnyPin<'a>,
    pub data3: AnyPin<'a>,
    pub data4: AnyPin<'a>,
    pub data5: AnyPin<'a>,
    pub data6: AnyPin<'a>,
    pub data7: AnyPin<'a>,
    /// SPH (start pulse horizontal) - LCD_CAM `CS` pin.
    pub sph: AnyPin<'a>,
    /// CL (pixel clock) - LCD_CAM `WRX` pin.
    pub cl: AnyPin<'a>,
    /// CKV (vertical clock).
    pub ckv: AnyPin<'a>,
    /// SPV / STV (start pulse vertical).
    pub spv: AnyPin<'a>,
    /// LE (latch enable).
    pub le: AnyPin<'a>,
    /// OE (output enable).
    pub oe: AnyPin<'a>,
    /// EPD power enable.
    pub pwr: AnyPin<'a>,
}

/// Board-specific timing control. The i8080 data bus + DMA buffer are shared;
/// only the per-board scanline/power control differs.
enum Backend<'a> {
    LilyGo {
        cfg_writer: ConfigWriter<'a>,
        rmt: rmt::Rmt<'a>,
    },
    M5PaperS3 {
        ctrl: M5PaperS3Control<'a>,
    },
}

pub(crate) struct ED047TC1<'a> {
    i8080: Option<i8080::I8080<'a, Blocking>>,
    backend: Backend<'a>,
    dma_buf: Option<DmaTxBuf>,
    line_bytes: usize,
}

#[inline(always)]
fn swizzle_papers3_byte(b: u8) -> u8 {
    // M5GFX / Panel_EPD packs pixels in the opposite 2bpp order compared to the
    // original LilyGo driver:
    // - our pipeline produces: [p0|p1|p2|p3] as 2-bit pairs from LSB→MSB
    // - PaperS3 expects:       [p0|p1|p2|p3] as 2-bit pairs from MSB→LSB
    //
    // Reverse the order of the 2-bit pairs inside each byte:
    // bits 1:0 ↔ 7:6, 3:2 ↔ 5:4.
    ((b & 0x03) << 6) | ((b & 0x0C) << 2) | ((b & 0x30) >> 2) | ((b & 0xC0) >> 6)
}

impl<'a> ED047TC1<'a> {
    pub(crate) fn new(
        pins: PinConfig<'a>,
        dma: peripherals::DMA_CH0<'a>,
        lcd_cam: peripherals::LCD_CAM<'a>,
        rmt: peripherals::RMT<'a>,
    ) -> crate::Result<Self> {
        // init lcd
        let lcd_cam = LcdCam::new(lcd_cam);

        let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
        let dma_buf =
            Some(DmaTxBuf::new(tx_descriptors, tx_buffer).map_err(crate::Error::DmaBuffer)?);

        let (i8080, backend, line_bytes) = match pins {
            PinConfig::LilyGoT5V23(pins) => {
                let config = i8080::Config::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_cd_idle_edge(false)
                    .with_cd_cmd_edge(true)
                    .with_cd_dummy_edge(false)
                    .with_cd_data_edge(false);
                // Data lines are physically swizzled on the LilyGo T5 V2.3.
                let i8080 = i8080::I8080::new(lcd_cam.lcd, dma, config)
                    .map_err(crate::Error::I8080Config)?
                    .with_dc(pins.lcd_dc)
                    .with_wrx(pins.lcd_wrx)
                    .with_data0(pins.data6)
                    .with_data1(pins.data7)
                    .with_data2(pins.data4)
                    .with_data3(pins.data5)
                    .with_data4(pins.data2)
                    .with_data5(pins.data3)
                    .with_data6(pins.data0)
                    .with_data7(pins.data1);

                let mut cfg_writer = ConfigWriter::new(pins.cfg_data, pins.cfg_clk, pins.cfg_str);
                cfg_writer.write();

                // GPIO38 is reserved for the RMT CKV channel; `rmt::Rmt` claims it.
                let _ = pins.rmt;
                let rmt = rmt::Rmt::new(rmt);

                (i8080, Backend::LilyGo { cfg_writer, rmt }, LINE_BYTES)
            }
            PinConfig::M5PaperS3(pins) => {
                let M5PaperS3Pins {
                    data0,
                    data1,
                    data2,
                    data3,
                    data4,
                    data5,
                    data6,
                    data7,
                    sph,
                    cl,
                    ckv,
                    spv,
                    le,
                    oe,
                    pwr,
                } = pins;

                // Match M5GFX `Bus_EPD`:
                // - SPH is wired to LCD_CS, not LCD_DC.
                // - There is no meaningful DC line on this bus.
                let config = i8080::Config::default().with_frequency(Rate::from_mhz(16));

                let i8080 = i8080::I8080::new(lcd_cam.lcd, dma, config)
                    .map_err(crate::Error::I8080Config)?
                    .with_cs(sph)
                    .with_wrx(cl)
                    .with_data0(data0)
                    .with_data1(data1)
                    .with_data2(data2)
                    .with_data3(data3)
                    .with_data4(data4)
                    .with_data5(data5)
                    .with_data6(data6)
                    .with_data7(data7);

                let ctrl = M5PaperS3Control::new(pwr, spv, oe, le, ckv);

                // RMT is unused on PaperS3 but still owned by the caller's pin set.
                let _ = rmt;

                (i8080, Backend::M5PaperS3 { ctrl }, PAPER_S3_LINE_BYTES)
            }
        };

        Ok(ED047TC1 {
            i8080: Some(i8080),
            backend,
            dma_buf,
            line_bytes,
        })
    }

    pub(crate) fn power_on(&mut self) {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, .. } => {
                cfg_writer.config.power_enable = true;
                cfg_writer.config.power_disable = false;
                cfg_writer.write();
                busy_delay(100 * 240);
                cfg_writer.config.neg_power_enable = true;
                cfg_writer.write();
                busy_delay(500 * 240);
                cfg_writer.config.pos_power_enable = true;
                cfg_writer.write();
                busy_delay(100 * 240);
                cfg_writer.config.stv = true;
                cfg_writer.write();
            }
            Backend::M5PaperS3 { ctrl } => ctrl.power_control(true),
        }
    }

    pub(crate) fn power_off(&mut self) {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, .. } => {
                cfg_writer.config.pos_power_enable = false;
                cfg_writer.write();
                busy_delay(10 * 240);
                cfg_writer.config.neg_power_enable = false;
                cfg_writer.write();
                busy_delay(100 * 240);
                cfg_writer.config.power_disable = true;
                cfg_writer.write();
                cfg_writer.config.stv = false;
                cfg_writer.write();
            }
            Backend::M5PaperS3 { ctrl } => ctrl.power_control(false),
        }
    }

    pub(crate) fn frame_start(&mut self) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, rmt } => {
                cfg_writer.config.mode = true;
                cfg_writer.write();

                let data = pulse!(10, 10);
                rmt.pulse(&data, true)?;

                cfg_writer.config.stv = false;
                cfg_writer.write();

                busy_delay(240);
                let data = pulse!(100, 100);
                let rmt_tx = rmt.pulse(&data, false)?;

                cfg_writer.config.stv = true;
                cfg_writer.write();

                if let Some(rmt_tx) = rmt_tx {
                    rmt.reclaim_channel(rmt_tx)?;
                }

                let data = pulse!(0, 100);
                rmt.pulse(&data, true)?;

                cfg_writer.config.output_enable = true;
                cfg_writer.write();

                let data = pulse!(10, 10);
                rmt.pulse(&data, true)?;

                Ok(())
            }
            Backend::M5PaperS3 { ctrl } => {
                ctrl.begin_transaction();
                Ok(())
            }
        }
    }

    // INTENTIONALLY UNUSED — see `skip_line`: a bare CKV-only advance is unsafe on the M5PaperS3 (drives
    // stale latch data / desyncs the gate), so the render paths clock every row with a real DMA line.
    // Retained for the LilyGo RMT backend and reference.
    #[allow(dead_code)]
    pub(crate) fn skip(&mut self) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { rmt, .. } => {
                let data = pulse!(45, 5);
                rmt.pulse(&data, false)?;
            }
            Backend::M5PaperS3 { ctrl } => ctrl.skip_line(),
        }
        Ok(())
    }

    pub(crate) fn output_row(&mut self, output_time: u16) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, rmt } => {
                cfg_writer.config.latch_enable = true;
                cfg_writer.write();
                cfg_writer.config.latch_enable = false;
                cfg_writer.write();

                let data = pulse!(output_time, 50);
                let rmt_tx = rmt.pulse(&data, false)?;
                let i8080 = self.i8080.take().ok_or(crate::Error::Unknown)?;
                let dma_buf = self.dma_buf.take().ok_or(crate::Error::Unknown)?;
                let tx = i8080
                    .send(Command::<u8>::One(0), 0, dma_buf)
                    .map_err(|(err, i8080, buf)| {
                        self.dma_buf = Some(buf);
                        self.i8080 = Some(i8080);
                        crate::Error::Dma(err)
                    })?;
                let (r, i8080, dma_buf) = tx.wait();
                if let Some(rmt_tx) = rmt_tx {
                    rmt.reclaim_channel(rmt_tx)?;
                }
                r.map_err(crate::Error::Dma)?;
                self.i8080 = Some(i8080);
                self.dma_buf = Some(dma_buf);

                Ok(())
            }
            Backend::M5PaperS3 { ctrl } => {
                let start_cycles = ctrl.scanline_begin();

                let i8080 = self.i8080.take().ok_or(crate::Error::Unknown)?;
                let dma_buf = self.dma_buf.take().ok_or(crate::Error::Unknown)?;
                let tx = i8080
                    .send(Command::<u8>::None, 0, dma_buf)
                    .map_err(|(err, i8080, buf)| {
                        self.dma_buf = Some(buf);
                        self.i8080 = Some(i8080);
                        crate::Error::Dma(err)
                    })?;
                let (r, i8080, dma_buf) = tx.wait();
                r.map_err(crate::Error::Dma)?;
                self.i8080 = Some(i8080);
                self.dma_buf = Some(dma_buf);

                ctrl.scanline_end(output_time, start_cycles);
                Ok(())
            }
        }
    }

    pub(crate) fn frame_end(&mut self) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, rmt } => {
                cfg_writer.config.output_enable = false;
                cfg_writer.write();
                cfg_writer.config.mode = false;
                cfg_writer.write();
                let data = pulse!(10, 10);
                rmt.pulse(&data, true)?;
                rmt.pulse(&data, true)?;

                Ok(())
            }
            Backend::M5PaperS3 { ctrl } => {
                ctrl.end_transaction();
                Ok(())
            }
        }
    }

    pub(crate) fn set_buffer(&mut self, data: &[u8]) -> crate::Result<()> {
        if data.len() > LINE_BYTES {
            return Err(crate::Error::OutOfBounds);
        }

        let mut dma_buf = self.dma_buf.take().ok_or(crate::Error::Unknown)?;
        dma_buf.as_mut_slice().fill(0);
        match &self.backend {
            Backend::M5PaperS3 { .. } => {
                for (dst, &src) in dma_buf.as_mut_slice()[..data.len()]
                    .iter_mut()
                    .zip(data.iter())
                {
                    *dst = swizzle_papers3_byte(src);
                }
            }
            Backend::LilyGo { .. } => {
                dma_buf.as_mut_slice()[..data.len()].copy_from_slice(data);
            }
        }
        dma_buf.set_length(self.line_bytes);
        self.dma_buf = Some(dma_buf);
        Ok(())
    }
}

#[inline(always)]
fn busy_delay(wait_cycles: u32) {
    // `get_cycle_count` is a 32-bit counter and wraps; use the platform helper
    // that handles wrapping arithmetic correctly.
    esp_hal::xtensa_lx::timer::delay(wait_cycles);
}

#[inline(always)]
fn cycles() -> u32 {
    esp_hal::xtensa_lx::timer::get_cycle_count()
}
