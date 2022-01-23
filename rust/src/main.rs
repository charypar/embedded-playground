#![no_std]
#![no_main]

use bsp::hal;
use panic_halt as _;
use trinket_m0 as bsp; // TODO work out how to use the HAL and PAC crates directly

use cortex_m_rt::entry;
use hal::clock::GenericClockController;
use hal::gpio::v2::{Pins, E};
use hal::pac::Peripherals;
use hal::prelude::*;
use hal::pwm::{Channel, Pwm1};

#[entry]
fn main() -> ! {
    // Configure peripherals

    let mut peri = Peripherals::take().unwrap();

    let pins = Pins::new(peri.PORT);

    let enc_a = pins.pa02.into_pull_up_input();
    let enc_b = pins.pa09.into_pull_up_input();
    let button = pins.pa07.into_pull_up_input();

    pins.pa06.into_alternate::<E>();

    let mut clocks = GenericClockController::with_internal_32kosc(
        peri.GCLK,
        &mut peri.PM,
        &mut peri.SYSCTRL,
        &mut peri.NVMCTRL,
    );
    let gclk0 = clocks.gclk0();

    let mut pwm1 = Pwm1::new(
        &clocks.tcc0_tcc1(&gclk0).unwrap(),
        1.khz(),
        peri.TCC1,
        &mut peri.PM,
    );
    let max_duty = pwm1.get_max_duty();

    // Program state

    let mut led_brightness = 31;
    let mut led_state = true;

    let mut button_state = false;
    let mut enc_state = Enc::NEUTRAL;

    loop {
        let (es, e_out) = read_encoder(enc_a.is_low().unwrap(), enc_b.is_low().unwrap(), enc_state);
        enc_state = es;
        let (bs, b_out) = read_button(button.is_high().unwrap(), button_state);
        button_state = bs;

        if b_out {
            led_state = !led_state;

            if !led_state {
                pwm1.set_duty(Channel::_0, 0);
            }
        }

        if !led_state {
            continue;
        }

        led_brightness = match e_out {
            EncoderOut::CW if led_brightness < 63 => led_brightness + 1,
            EncoderOut::CCW if led_brightness > 0 => led_brightness - 1,
            _ => led_brightness,
        };
        let duty = LED_BRIGHTNESS_MAP[led_brightness] * max_duty / 255;

        pwm1.set_duty(Channel::_0, duty);
    }
}

// Output

const LED_BRIGHTNESS_MAP: [u32; 64] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 26, 27,
    29, 31, 33, 35, 37, 39, 42, 44, 47, 50, 53, 56, 60, 63, 67, 71, 76, 80, 85, 90, 95, 101, 107,
    114, 120, 128, 135, 143, 152, 161, 170, 180, 191, 203, 215, 227, 241, 255,
];

// Input

fn read_button(input: bool, state: bool) -> (bool, bool) {
    match (state, input) {
        (true, false) => (input, true),
        _ => (input, false),
    }
}

#[derive(PartialEq, Eq)]
struct Enc(u8);

impl Enc {
    pub const NEUTRAL: Enc = Enc(0b1111);
    pub const CW_A: Enc = Enc(0b1101);
    pub const CW_B: Enc = Enc(0b0100);
    pub const CW_C: Enc = Enc(0b0010);
    pub const CW_D: Enc = Enc(0b1011);
    pub const CCW_A: Enc = Enc(0b1110);
    pub const CCW_B: Enc = Enc(0b1000);
    pub const CCW_C: Enc = Enc(0b0001);
    pub const CCW_D: Enc = Enc(0b0111);
}

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
