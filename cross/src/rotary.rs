#[derive(PartialEq, Eq, Clone, Copy)]
struct State(u8);

impl State {
    pub const NEUTRAL: State = State(0b0000);
    pub const CW_A: State = State(0b0010);
    pub const CW_B: State = State(0b1011);
    pub const CW_C: State = State(0b1101);
    pub const CW_D: State = State(0b0100);
    pub const CCW_A: State = State(0b0001);
    pub const CCW_B: State = State(0b0111);
    pub const CCW_C: State = State(0b1110);
    pub const CCW_D: State = State(0b1000);
}

#[derive(Clone, Copy, PartialEq)]
pub enum Out {
    None,
    CW,
    CCW,
}

pub struct Rotary {
    hold_count: u8, // how long should the output stay on/off
    state: State,
    out_value: Out,
    out_count: u8,
    out_counter: u8,
}

impl Rotary {
    pub fn new(hold_count: u8) -> Self {
        Self {
            hold_count,
            state: State::NEUTRAL,
            out_value: Out::None,
            out_count: 0,
            out_counter: 0,
        }
    }

    pub fn update(&mut self, a: bool, b: bool) -> Out {
        let mux = ((a as u8) << 1) | (b as u8);
        let input = State(((self.state.0 as u8) << 2 | mux) & 0xF);

        // Encoder state update

        let (state, output) = match (self.state, input) {
            (State::NEUTRAL, State::CW_A) => (State::CW_A, Out::None),
            (State::NEUTRAL, State::CCW_A) => (State::CCW_A, Out::None),

            (State::CW_A, State::CW_B) => (State::CW_B, Out::None),
            (State::CW_B, State::CW_C) => (State::CW_C, Out::None),
            (State::CW_C, State::CW_D) => (State::NEUTRAL, Out::CW),

            (State::CCW_A, State::CCW_B) => (State::CCW_B, Out::None),
            (State::CCW_B, State::CCW_C) => (State::CCW_C, Out::None),
            (State::CCW_C, State::CCW_D) => (State::NEUTRAL, Out::CCW),

            (_, State::NEUTRAL) => (State::NEUTRAL, Out::None),
            (s, _) => (s, Out::None),
        };

        self.state = state;

        // Output buffering state update

        match (output, self.out_value) {
            (Out::CW, Out::CW) => self.out_count += 1,
            (Out::CCW, Out::CCW) => self.out_count += 1,
            (Out::None, _) => (),
            (cw_or_ccw, _different_than_output) => {
                self.out_value = cw_or_ccw;
                self.out_count = 1;
                self.out_counter = self.hold_count * 2;
            }
        }

        // Output timer/counter state update

        self.out_counter -= 1; // decrement by 1 tick
        if self.out_counter < 1 {
            self.out_count -= 1; // decrement one press

            if self.out_count > 0 {
                // reset timer counter
                self.out_counter = self.hold_count * 2;
            } else {
                // stop, set value to None
                self.out_value = Out::None;
            }
        }

        output
    }

    pub fn get(&self) -> Out {
        if self.out_value == Out::None || self.out_counter < self.hold_count {
            return Out::None;
        }

        return self.out_value;
    }

    pub fn get_cw(&self) -> bool {
        if self.out_value == Out::CW && self.out_counter >= self.hold_count {
            return true;
        }

        false
    }

    pub fn get_ccw(&self) -> bool {
        if self.out_value == Out::CCW && self.out_counter >= self.hold_count {
            return true;
        }

        false
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test() {
        todo!();
    }
}
