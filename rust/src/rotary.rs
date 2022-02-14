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

#[derive(Clone, Copy)]
pub enum Out {
    None,
    CW,
    CCW,
}

pub struct Rotary {
    state: State,
}

impl Rotary {
    pub fn new() -> Self {
        Self {
            state: State::NEUTRAL,
        }
    }

    pub fn update(&mut self, a: bool, b: bool) -> Out {
        let mux = ((a as u8) << 1) | (b as u8);
        let input = State(((self.state.0 as u8) << 2 | mux) & 0xF);

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

        output
    }
}
