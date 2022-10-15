use stm32f1xx_hal as hal;

use hal::gpio::{ErasedPin, Input, PullUp};

use cross::debounce::Debounced;
use cross::rotary::Rotary;

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
