#![no_std]
#![no_main]

use arduino_hal::pac::TC0;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let enc_a = pins.d16.into_pull_up_input();
    let enc_b = pins.d14.into_pull_up_input();
    let button = pins.d2.into_pull_up_input();

    let tc0 = dp.TC0;
    d3_pwm_enable(&tc0, true);
    pins.d3.into_output();

    let mut led_brightness = 31;
    let mut led_state = true;

    let mut button_state = false;
    let mut enc_state = Enc::NEUTRAL;

    loop {
        let (es, e_out) = read_encoder(enc_a.is_low(), enc_b.is_low(), enc_state);
        enc_state = es;
        let (bs, b_out) = read_button(button.is_high(), button_state);
        button_state = bs;

        if b_out {
            led_state = !led_state;

            d3_pwm_enable(&tc0, led_state);
        }

        if !led_state {
            continue;
        }

        led_brightness = match e_out {
            EncoderOut::CW if led_brightness < 63 => led_brightness + 1,
            EncoderOut::CCW if led_brightness > 0 => led_brightness - 1,
            _ => led_brightness,
        };
        let duty = LED_BRIGHTNESS_MAP[led_brightness];

        d3_pwm_write(&tc0, duty);
    }
}

// Output

fn d3_pwm_enable(tc0: &TC0, enable: bool) {
    if enable {
        tc0.tccr0a
            .write(|w| w.wgm0().pwm_fast().com0b().match_clear());
        tc0.tccr0b.write(|w| w.cs0().prescale_256());
    } else {
        tc0.tccr0a
            .write(|w| w.wgm0().normal_top().com0b().disconnected());
    }
}

fn d3_pwm_write(tc0: &TC0, v: u8) {
    tc0.ocr0b.write(|w| unsafe { w.bits(v) });
}

const LED_BRIGHTNESS_MAP: [u8; 64] = [
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
