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

        if self.stable_for < self.stable_minimum {
            return None;
        }

        self.state = self.candidate.clone();
        self.stable_for = 0;

        Some(self.candidate.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::Debounced;

    #[test]
    fn test_changes_state_when_stable() {
        let mut debouncer = Debounced::new(false, 3);
        let input = [false, false, true, true, true];

        let expected = vec![
            (None, false),
            (None, false),
            (None, false),
            (None, false),
            (Some(true), true),
        ];
        let actual = feed(&mut debouncer, &input);

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_ignores_blips() {
        let mut debouncer = Debounced::new(false, 3);
        let input = [false, false, true, false];

        let expected = vec![(None, false), (None, false), (None, false), (None, false)];
        let actual = feed(&mut debouncer, &input);

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_makes_multiple_changes() {
        let mut debouncer = Debounced::new(false, 2);
        let input = [false, true, true, false, false];

        let expected = vec![
            (None, false),
            (None, false),
            (Some(true), true),
            (None, true),
            (Some(false), false),
        ];
        let actual = feed(&mut debouncer, &input);

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_complex_scenario() {
        let mut debouncer = Debounced::new(false, 2);
        let input = [
            false, true, false, true, true, false, true, false, false, true,
        ];

        let expected = vec![
            (None, false),
            (None, false),
            (None, false),
            (None, false),
            (Some(true), true),
            (None, true),
            (None, true),
            (None, true),
            (Some(false), false),
            (None, false),
        ];
        let actual = feed(&mut debouncer, &input);

        assert_eq!(actual, expected);
    }

    fn feed<T>(debouncer: &mut Debounced<T>, input: &[T]) -> Vec<(Option<T>, T)>
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
