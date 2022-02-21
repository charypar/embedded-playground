use core::cmp::{max, min};

use atsamd_hal as hal;
use hal::clock::Sercom1CoreClock;
use hal::gpio::v2::{Alternate, Disabled, Floating, Pin, D};
use hal::gpio::v2::{PA00, PA01};
use hal::pac::{PM, SERCOM1};
use hal::prelude::*;
use hal::sercom::v2::{spi, Sercom1};
use hal::typelevel::NoneT;

use apa102_spi::Apa102;
use smart_leds::SmartLedsWrite;
use smart_leds_trait::RGB8;

pub struct DotStar {
    pub enabled: bool,
    brightness: u8,
    color_index: usize,
    out_color: RGB8,
    // FIXME There's probably a way more abstract and portable way of doing this
    spi: Apa102<
        spi::Spi<
            spi::Config<
                spi::Pads<Sercom1, NoneT, Pin<PA00, Alternate<D>>, Pin<PA01, Alternate<D>>>,
            >,
            spi::Tx,
        >,
    >,
}

impl DotStar {
    #[rustfmt::skip]
    const COLORS: [RGB8; 7] = [
        RGB8 { r: 148, g: 0, b: 211 },
        RGB8 { r: 75, g: 0, b: 130 },
        RGB8 { r: 0, g: 0, b: 255 },
        RGB8 { r: 0, g: 255, b: 0 },
        RGB8 { r: 255, g: 255, b: 0 },
        RGB8 { r: 255, g: 127, b: 0 },
        RGB8 { r: 255, g: 0, b: 0 },
    ];

    pub fn new(
        pin_do: Pin<PA00, Disabled<Floating>>,
        pin_sclk: Pin<PA01, Disabled<Floating>>,
        pm: &PM,
        sercom: SERCOM1,
        spi_clock: Sercom1CoreClock,
    ) -> Self {
        let spi_pads = spi::Pads::<Sercom1>::default()
            .data_out(pin_do)
            .sclk(pin_sclk);

        let spi = spi::Config::new(pm, sercom, spi_pads, spi_clock)
            .spi_mode(apa102_spi::MODE)
            .baud(24.mhz())
            .enable();

        let mut inst = Self {
            color_index: 4,
            enabled: true,
            brightness: 31,
            out_color: RGB8 { r: 0, g: 0, b: 0 },
            spi: Apa102::new(spi),
        };
        inst.out_color = inst.recompute();

        inst
    }

    pub fn brightness(&self) -> u8 {
        self.brightness
    }

    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = max(0, min(brightness, 255));
    }

    pub fn color(&self) -> usize {
        self.color_index
    }

    pub fn set_color(&mut self, color: usize) {
        self.color_index = max(0, min(color, Self::COLORS.len() - 1));
    }

    pub fn display(&mut self) {
        let c = self.recompute();
        if c == self.out_color {
            return;
        }

        self.out_color = c;
        self.spi.write([c].iter().cloned()).ok();
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
    }

    fn recompute(&self) -> RGB8 {
        if !self.enabled {
            return RGB8 { r: 0, g: 0, b: 0 };
        }

        let color = Self::COLORS[self.color_index];
        let brightness = self.brightness;

        RGB8 {
            r: (color.r as u32 * brightness as u32 / 255) as u8,
            g: (color.g as u32 * brightness as u32 / 255) as u8,
            b: (color.b as u32 * brightness as u32 / 255) as u8,
        }
    }
}
