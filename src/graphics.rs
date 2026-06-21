use embedded_graphics_core::{
    pixelcolor::{Gray2, Gray4},
    prelude::*,
};

use crate::{
    display::{Display, DisplayRotation},
    Error,
};

impl<'a> DrawTarget for Display<'a> {
    type Color = Gray4;

    type Error = Error;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            let (x, y) = translate_coord_rotation(coord.x as u16, coord.y as u16, &self.rotation());
            let result = self.set_pixel(x, y, color.luma());
            if matches!(result, Err(Error::OutOfBounds)) {
                continue;
            }
            result?;
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.fill(color.luma())
    }
}

#[inline(always)]
fn translate_coord_rotation(x: u16, y: u16, rotation: &DisplayRotation) -> (u16, u16) {
    match rotation {
        DisplayRotation::Rotate0 => (x, y),
        DisplayRotation::Rotate90 => (Display::WIDTH - 1 - y, x),
        DisplayRotation::Rotate180 => (Display::WIDTH - 1 - x, Display::HEIGHT - 1 - y),
        DisplayRotation::Rotate270 => (y, Display::HEIGHT - 1 - x),
    }
}

impl<'a> OriginDimensions for Display<'a> {
    fn size(&self) -> Size {
        match self.rotation() {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
                Size::new(Self::WIDTH as u32, Self::HEIGHT as u32)
            }
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
                Size::new(Self::HEIGHT as u32, Self::WIDTH as u32)
            }
        }
    }
}

/// A 2-bpp (4-gray) `DrawTarget` view over a [`Display`], obtained via [`Display::gray2`]. Writes into the
/// 2-bpp framebuffer ([`Display::set_pixel_gray2`]); present it with `flush_gray2`/`partial_gray2`.
/// [`Gray2`] luma maps 0=black .. 3=white, the same convention as the 2-bpp framebuffer.
pub struct Gray2Canvas<'d, 'a>(pub(crate) &'d mut Display<'a>);

impl DrawTarget for Gray2Canvas<'_, '_> {
    type Color = Gray2;
    type Error = Error;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let rotation = self.0.rotation();
        for Pixel(coord, color) in pixels.into_iter() {
            let (x, y) = translate_coord_rotation(coord.x as u16, coord.y as u16, &rotation);
            let result = self.0.set_pixel_gray2(x, y, color.luma());
            if matches!(result, Err(Error::OutOfBounds)) {
                continue;
            }
            result?;
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.0.fill_gray2(color.luma())
    }
}

impl OriginDimensions for Gray2Canvas<'_, '_> {
    fn size(&self) -> Size {
        match self.0.rotation() {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
                Size::new(Display::WIDTH as u32, Display::HEIGHT as u32)
            }
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
                Size::new(Display::HEIGHT as u32, Display::WIDTH as u32)
            }
        }
    }
}

impl From<embedded_graphics_core::primitives::Rectangle> for crate::display::Rectangle {
    fn from(val: embedded_graphics_core::primitives::Rectangle) -> Self {
        crate::display::Rectangle {
            x: val.top_left.x as u16,
            y: val.top_left.y as u16,
            width: val.size.width as u16,
            height: val.size.height as u16,
        }
    }
}
