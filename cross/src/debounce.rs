pub struct Debounced<T> {
    state: T,
    candidate: T,
    stable_for: usize,
    stable_minimum: usize,
}

impl<T: Clone> Debounced<T> {
    pub fn new(init: T, n: usize) -> Self {
        Self {
            state: init.clone(),
            candidate: init,
            stable_for: 0,
            stable_minimum: n,
        }
    }

    // Get the current stable value of the input
    pub fn get(&self) -> T {
        self.state.clone()
    }
}

impl<T: PartialEq + Clone> Debounced<T> {
    // Update the state with the latest input
    // Returns Some(value) if the value has changed and stabilised,
    // which can be used as a rising or falling edge trigger
    pub fn update(&mut self, input: T) -> Option<T> {
        if self.state == input {
            self.stable_for = 0;
            return None;
        }

        if self.candidate != input {
            self.candidate = input;

            self.stable_for = 1;
        } else {
            self.stable_for += 1;
        }

        if self.stable_for <= self.stable_minimum {
            return None;
        }

        self.state = self.candidate.clone();
        self.stable_for = 0;

        Some(self.candidate.clone())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test() {
        todo!();
    }
}
