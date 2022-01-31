#![no_std]
#![no_main]

use bsp::hal;

use panic_halt as _;

use apa102_spi as apa102;
use trinket_m0 as bsp; // TODO work out how to use the HAL and PAC crates directly

use cortex_m_rt::entry;
use hal::clock::GenericClockController;
use hal::gpio::v2::Pins;
use hal::pac::Peripherals;
use hal::prelude::*;
use hal::sercom::v2::{spi, Sercom1};

use apa102::Apa102;
use smart_leds::SmartLedsWrite;
use smart_leds_trait::RGB8;

#[entry]
fn main() -> ! {
    let mut peri = Peripherals::take().unwrap();
    let pins = Pins::new(peri.PORT);

    // Input matrix

    let mut rows = (
        pins.pa06.into_push_pull_output(),
        pins.pa07.into_push_pull_output(),
    );
    let cols = (
        pins.pa08.into_pull_down_input(),
        pins.pa02.into_pull_down_input(),
        pins.pa09.into_pull_down_input(),
    );

    // DotStar LED control over SPI on SERCOM1

    // Set up the SERCOM1 pad on the DotStar LED pins (CI - PA01 and DI - PA00)
    let pads = spi::Pads::<Sercom1>::default()
        .data_out(pins.pa00)
        .sclk(pins.pa01);

    // Configure GCLK for SERCOM1
    let mut clocks = GenericClockController::with_internal_32kosc(
        peri.GCLK,
        &mut peri.PM,
        &mut peri.SYSCTRL,
        &mut peri.NVMCTRL,
    );
    let gclk0 = clocks.gclk0();
    let clock = clocks.sercom1_core(&gclk0).unwrap();

    // Create the SPI interface and wrap it in Apa102 LED driver
    let spi = spi::Config::new(&peri.PM, peri.SERCOM1, pads, clock)
        .spi_mode(apa102_spi::MODE)
        .baud(24.mhz())
        .enable();
    let mut dostar = Apa102::new(spi);

    #[rustfmt::skip]
    let colours: [RGB8; 7] = [
        RGB8 {r: 148, g: 0, b: 211},
        RGB8 {r: 75, g: 0, b: 130},
        RGB8 {r: 0, g: 0, b: 255},
        RGB8 {r: 0, g: 255, b: 0},
        RGB8 {r: 255, g: 255, b: 0},
        RGB8 {r: 255, g: 127, b: 0},
        RGB8 {r: 255, g: 0, b: 0},
    ];

    // Input

    let mut toggle_out: Option<bool>;
    let mut toggle_state = false;

    let mut btn_a_out: bool;
    let mut btn_a_state = false;

    let mut btn_b_out: bool;
    let mut btn_b_state = false;

    let mut enc_out: EncoderOut;
    let mut enc_state = Enc::NEUTRAL;

    // Program state

    let mut led_on = true;
    let mut led_colour: usize = 5;
    let mut led_brightness: u8 = 31;
    let mut colour = adjust_brightess(&colours[0], led_brightness);

    loop {
        // Read inputs

        rows.0.set_high().unwrap();

        (enc_state, enc_out) = read_encoder(
            cols.1.is_high().unwrap(),
            cols.2.is_high().unwrap(),
            enc_state,
        );

        rows.0.set_low().unwrap();

        rows.1.set_high().unwrap();

        (btn_a_state, btn_a_out) = read_button(cols.0.is_high().unwrap(), btn_a_state);
        (btn_b_state, btn_b_out) = read_button(cols.1.is_high().unwrap(), btn_b_state);
        (toggle_state, toggle_out) = read_toggle(cols.2.is_low().unwrap(), toggle_state);

        rows.1.set_low().unwrap();

        // Update state

        match toggle_out {
            Some(true) => led_on = true,
            Some(false) => led_on = false,
            _ => (),
        }

        match enc_out {
            EncoderOut::CW if led_brightness < 247 => led_brightness += 8,
            EncoderOut::CCW if led_brightness > 8 => led_brightness -= 8,
            _ => (),
        }

        if btn_a_out && led_colour < 6 {
            led_colour = led_colour + 1;
        }

        if btn_b_out && led_colour > 0 {
            led_colour = led_colour - 1;
        }

        // Update LED

        let c = if led_on {
            adjust_brightess(&colours[led_colour], led_brightness)
        } else {
            RGB8 { r: 0, g: 0, b: 0 }
        };

        if colour != c {
            dostar.write([c].iter().cloned()).unwrap();
            colour = c;
        }
    }
}

fn adjust_brightess(colour: &RGB8, brightness: u8) -> RGB8 {
    RGB8 {
        r: (colour.r as u32 * brightness as u32 / 255) as u8,
        g: (colour.g as u32 * brightness as u32 / 255) as u8,
        b: (colour.b as u32 * brightness as u32 / 255) as u8,
    }
}

// Input

fn read_button(input: bool, state: bool) -> (bool, bool) {
    match (state, input) {
        (true, false) => (input, true),
        _ => (input, false),
    }
}

fn read_toggle(input: bool, state: bool) -> (bool, Option<bool>) {
    match (state, input) {
        (false, true) => (input, Some(true)),
        (true, false) => (input, Some(false)),
        _ => (input, None),
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
struct Enc(u8);

impl Enc {
    pub const NEUTRAL: Enc = Enc(0b0000);
    pub const CW_A: Enc = Enc(0b0010);
    pub const CW_B: Enc = Enc(0b1011);
    pub const CW_C: Enc = Enc(0b1101);
    pub const CW_D: Enc = Enc(0b0100);
    pub const CCW_A: Enc = Enc(0b0001);
    pub const CCW_B: Enc = Enc(0b0111);
    pub const CCW_C: Enc = Enc(0b1110);
    pub const CCW_D: Enc = Enc(0b1000);
}

#[derive(Clone, Copy)]
enum EncoderOut {
    None,
    CW,
    CCW,
}

fn read_encoder(a: bool, b: bool, state: Enc) -> (Enc, EncoderOut) {
    let mux = ((a as u8) << 1) | (b as u8);
    let input = Enc(((state.0 as u8) << 2 | mux) & 0xF);

    match (state, input) {
        (Enc::NEUTRAL, Enc::CW_A) => (Enc::CW_A, EncoderOut::None),
        (Enc::NEUTRAL, Enc::CCW_A) => (Enc::CCW_A, EncoderOut::None),

        (Enc::CW_A, Enc::CW_B) => (Enc::CW_B, EncoderOut::None),
        (Enc::CW_B, Enc::CW_C) => (Enc::CW_C, EncoderOut::None),
        (Enc::CW_C, Enc::CW_D) => (Enc::NEUTRAL, EncoderOut::CW),

        (Enc::CCW_A, Enc::CCW_B) => (Enc::CCW_B, EncoderOut::None),
        (Enc::CCW_B, Enc::CCW_C) => (Enc::CCW_C, EncoderOut::None),
        (Enc::CCW_C, Enc::CCW_D) => (Enc::NEUTRAL, EncoderOut::CCW),

        (_, Enc::NEUTRAL) => (Enc::NEUTRAL, EncoderOut::None),
        (s, _) => (s, EncoderOut::None),
    }
}
