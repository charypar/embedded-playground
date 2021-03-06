use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = JOYSTICK) = {
        (collection = PHYSICAL, usage = JOYSTICK) = {
            (usage_page = BUTTON, usage_min = BUTTON_1, usage_max = 0x06) = {
                #[packed_bits 6] #[item_settings data,variable,absolute] buttons=input;
            };
            (usage_page = GENERIC_DESKTOP,) = {
                (usage = 0x37,) = { // 0x37 is a Dial
                    #[item_settings data,variable,relative] wheel=input;
                }
            };
        }
    }
)]
pub struct JoystickReport {
    pub buttons: u8,
    pub wheel: i8,
}

impl JoystickReport {
    pub fn new() -> Self {
        Self {
            buttons: 0,
            wheel: 0,
        }
    }

    pub fn set_button(&mut self, n: usize, value: bool) {
        if n > 5 {
            return;
        }

        if value {
            self.buttons |= 1 << n
        } else {
            self.buttons &= !(1 << n)
        }
    }
}

// FIXME work out testing
// #[cfg(test)]
// mod tess {
//     use super::JoystickReport;

//     #[test]
//     fn test_set_true() {
//         let mut report = JoystickReport::new();

//         report.set_button(3, true);

//         assert_eq!(report.buttons, 0x04);
//     }
// }
