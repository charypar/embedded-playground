use stm32f1xx_hal as hal;
// FIXME this can be written for embedded_hal

use hal::gpio::{ErasedPin, Input, PullUp};

use cross::debounce::Debounced;
use cross::rotary::Rotary;

pub trait PanelReport {
    fn new() -> Self;

    fn set_button(&mut self, n: usize, value: bool);
}

pub struct Panel<const B: usize, const E: usize> {
    // TODO can the panel structure be encoded as types?
    pub buttons: [Button; B],
    pub encoders: [Encoder; E],
}

impl<const B: usize, const E: usize> Panel<B, E> {
    pub fn scan(&mut self) {
        for btn in self.buttons.iter_mut() {
            btn.scan();
        }

        for enc in self.encoders.iter_mut() {
            enc.scan();
        }
    }

    pub fn report<R: PanelReport>(&self) -> R {
        let mut report = R::new();

        for (idx, button) in self.buttons.iter().enumerate() {
            report.set_button(idx, button.get());
        }

        let base = B;

        for (idx, encoder) in self.encoders.iter().enumerate() {
            report.set_button(base + 2 * idx, encoder.get_ccw());
            report.set_button(base + 2 * idx + 1, encoder.get_cw());
        }

        report
    }
}

pub struct Encoder {
    pin_a: ErasedPin<Input<PullUp>>,
    pin_b: ErasedPin<Input<PullUp>>,
    state: Rotary,
}

impl Encoder {
    pub fn new(pin_a: ErasedPin<Input<PullUp>>, pin_b: ErasedPin<Input<PullUp>>, hold: u8) -> Self {
        Self {
            pin_a,
            pin_b,
            state: Rotary::new(hold),
        }
    }

    pub fn scan(&mut self) {
        self.state.update(self.pin_a.is_low(), self.pin_b.is_low());
    }

    pub fn get_cw(&self) -> bool {
        self.state.get_cw()
    }

    pub fn get_ccw(&self) -> bool {
        self.state.get_ccw()
    }
}

pub struct Button {
    pin: ErasedPin<Input<PullUp>>,
    state: Debounced<bool>,
}

impl Button {
    pub fn new(pin: ErasedPin<Input<PullUp>>, init: bool, hold: usize) -> Self {
        Self {
            pin,
            state: Debounced::new(init, hold),
        }
    }

    pub fn scan(&mut self) -> Option<bool> {
        self.state.update(self.pin.is_low())
    }

    pub fn get(&self) -> bool {
        self.state.get()
    }
}
