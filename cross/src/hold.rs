pub struct Held<T> {
    state: T,
    hold_for: usize,
    remaining: usize,
}

impl<T: Clone> Held<T> {
    pub fn new(init: T, n: usize) -> Self {
        Self {
            state: init,
            hold_for: n,
            remaining: 0,
        }
    }

    // Get the current stable value of the input
    pub fn get(&self) -> T {
        self.state.clone()
    }
}

impl<T: PartialEq + Clone> Held<T> {
    // Update the state with the latest input
    // Returns Some(value) if the output has changed,
    // which can be used as a rising or falling edge trigger
    pub fn update(&mut self, input: T) -> Option<T> {
        if self.state == input {
            return None;
        }

        if self.remaining < 1 {
            self.remaining = self.hold_for - 1;
            self.state = input.clone();

            return Some(input);
        } else {
            self.remaining -= 1;

            if self.remaining > 0 {
                return None;
            }

            if input != self.state {
                self.remaining = self.hold_for - 1;
            }

            self.state = input.clone();

            return Some(input);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Held;

    #[test]
    fn test_holds() {
        let mut debouncer = Held::new(false, 3);
        let input = [false, true, false, true, false, true, false, true];

        let expected = vec![
            (None, false),
            (Some(true), true),
            (None, true),
            (None, true),
            (Some(false), false),
            (None, false),
            (None, false),
            (Some(true), true),
        ];
        let actual = feed(&mut debouncer, &input);

        assert_eq!(actual, expected);
    }

    fn feed<T>(debouncer: &mut Held<T>, input: &[T]) -> Vec<(Option<T>, T)>
    where
        T: PartialEq + Clone,
    {
        input
            .into_iter()
            .map(|i| {
                let out = debouncer.update(i.clone());
                let state = debouncer.get();

                (out, state)
            })
            .collect()
    }
}
