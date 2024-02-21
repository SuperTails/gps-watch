use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use stm32l4xx_hal::hal::{
    digital::v2::OutputPin,
    blocking::spi::Write as SpiWrite,
};
use stm32l4xx_hal::spi;
use core::convert::Infallible;

const WIDTH: usize = 400;
const HEIGHT: usize = 240;
const WIDTH_BYTES: usize = WIDTH.div_ceil(8);
const HEIGHT_BYTES: usize = HEIGHT.div_ceil(8);

const UPDATE_BIT: u8 = 0b0000_0001;
const VCOM_BIT: u8 = 0b0000_0010;
const CLEAR_BIT: u8 = 0b0000_0100;

// #[derive(thiserror::Error, Debug)]
// pub enum DisplayError {
//     #[error("SPI write failed")]
//     Spi,
//     #[error("GPIO failed")]
//     Gpio
// }

// type Result<T> = core::result::Result<T, DisplayError>;

struct SpiTransaction<'a, SPI, CS> {
    spi: &'a mut SPI,
    cs: &'a mut CS
}

impl<'a, SPI, CS> SpiTransaction<'a, SPI, CS>
    where SPI: SpiWrite<u8, Error = spi::Error>,
          CS:  OutputPin<Error = Infallible>
{
    fn start(disp: &'a mut SharpMemDisplayDriver<SPI, CS>, command: u8) -> Self {
        disp.vcom = !disp.vcom;
        disp.cs.set_high().unwrap();
        disp.spi.write(&[
            command | if disp.vcom { VCOM_BIT } else { 0 }
        ]).unwrap();
        Self {
            spi: &mut disp.spi,
            cs: &mut disp.cs
        }
    }

    fn send(self, data: &[u8]) -> Self {
        self.spi.write(data).unwrap();
        self
    }

    fn finish(self) {
        self.cs.set_low().unwrap();
    }
}


pub struct SharpMemDisplayDriver<SPI, CS> {
    spi: SPI,
    cs: CS,
    vcom: bool
}

impl<SPI, CS> SharpMemDisplayDriver<SPI, CS>
    where SPI: SpiWrite<u8, Error = spi::Error>,
          CS:  OutputPin<Error = Infallible>
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi,
            cs,
            vcom: false
        }
    }

    fn start(&mut self, command: u8) -> SpiTransaction<SPI, CS> {
        SpiTransaction::start(self, command)
    }

    pub fn clear(&mut self) {
        self.start(CLEAR_BIT).send(&[0x00]).finish();
    }

    pub fn write_line(&mut self, line: u8, data: &[u8; WIDTH_BYTES]) {
        self.start(UPDATE_BIT).send(&[line]).send(data).send(&[0x00, 0x00]).finish();
    }
}

pub struct SharpMemDisplay<SPI, CS> {
    buf: [[u8; WIDTH_BYTES]; HEIGHT],
    dirty: [u8; HEIGHT_BYTES],
    dirty_any: bool,
    driver: SharpMemDisplayDriver<SPI, CS>
}

impl<SPI, CS> SharpMemDisplay<SPI, CS>
    where SPI: SpiWrite<u8, Error = spi::Error>,
          CS:  OutputPin<Error = Infallible>
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            buf: [[0; WIDTH_BYTES]; HEIGHT],
            dirty: [0; HEIGHT_BYTES],
            dirty_any: false,
            driver: SharpMemDisplayDriver::new(spi, cs)
        }
    }

    pub fn draw_pixel(&mut self, x: usize, y: usize, state: bool) {
        if x >= WIDTH || y >= HEIGHT { return }
        if state {
            self.buf[y][x / 8] |= 1u8 << (x % 8);
        } else {
            self.buf[y][x / 8] &= !(1u8 << (x % 8));
        }
        self.dirty[y / 8] |= 1u8 << (y % 8);
        self.dirty_any = true;
    }

    pub fn flush(&mut self) {
        // Don't send anything if there's nothing to flush
        if !self.dirty_any {
            return
        }
        // For each line
        self.buf.iter().enumerate()
            // If the line is dirty
            .filter(|(y, row)| (self.dirty[y / 8] & (1u8 << (y % 8))) != 0u8)
            // Send it
            .fold(
                self.driver.start(UPDATE_BIT), // command byte
                |trn, (y, row)|
                    trn.send(&[y as u8]) // address byte
                        .send(row) // row data
                        .send(&[0x00]) // spacing byte
            )
            .send(&[0x00]) // termination byte
            .finish();

        // Clear dirty bits
        self.dirty = [0; HEIGHT_BYTES];
        self.dirty_any = false;
    }

    pub fn clear(&mut self) {
        self.driver.clear();
        self.buf = [[0xFF; WIDTH_BYTES]; HEIGHT];
        self.dirty = [0; HEIGHT_BYTES];
        self.dirty_any = false;
    }
}

impl<SPI, CS> DrawTarget for SharpMemDisplay<SPI, CS>
    where SPI: SpiWrite<u8, Error = spi::Error>,
          CS:  OutputPin<Error = Infallible>
{
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error> where I: IntoIterator<Item=Pixel<Self::Color>> {
        for Pixel(pos, color) in pixels {
            self.draw_pixel(pos.x as usize, pos.y as usize, match color {
                BinaryColor::Off => true,
                BinaryColor::On => false
            });
        }
        Ok(())
    }
}

impl<SPI, CS> Dimensions for SharpMemDisplay<SPI, CS> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle {
            top_left: Point {x: 0, y: 0},
            size: Size {width: WIDTH as u32, height: HEIGHT as u32}
        }
    }
}