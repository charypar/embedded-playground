#![no_main]
#![no_std]

use panic_halt as _;
use rtic::app;

mod device;
mod usb;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use stm32f1xx_hal as hal;

    use hal::gpio::gpioc::PC13;
    use hal::gpio::{Output, PushPull};
    use hal::usb::{Peripheral, UsbBus};

    use systick_monotonic::{fugit::Duration, Systick};

    use usb_device::class_prelude::UsbBusAllocator;
    use usbd_hid::descriptor::SerializedDescriptor;

    use crate::device::{Bootloader, Device, InputPins};
    use crate::usb::{Report, Usb};

    use cross::debounce::Debounced;
    use cross::rotary::Rotary;

    #[monotonic(binds = SysTick, default = true)]
    type RtcMonotonic = Systick<1000>;

    pub struct InputsState {
        buttons: [Debounced<bool>; 4],
        encoder: Rotary,
        encoder_button: Debounced<bool>,
    }

    #[shared]
    struct Shared {
        usb: Usb<'static, UsbBus<Peripheral>, Report>,
        led: PC13<Output<PushPull>>,
    }

    #[local]
    struct Local {
        bootloader: Bootloader,
        pins: InputPins,
        inputs_state: InputsState,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Begin hardware initialisation
        // FIXME pull all of this out

        let device = Device::new(ctx.device);

        let usb_bus = ctx.local.usb_bus.insert(device.usb);
        let usb = Usb::new(usb_bus, Report::desc());

        // Set up the real-time clock, using the 32K oscillator clock
        // TODO this can be clocked by a subdivided clock for lower power consumption (probably)
        let rtc = Systick::new(ctx.core.SYST, 36_000_000);

        // End Hardware initialisation

        // Input
        // TODO factor this out

        let inputs_state = InputsState {
            buttons: [
                Debounced::new(false, 20),
                Debounced::new(false, 20),
                Debounced::new(false, 20),
                Debounced::new(false, 20),
            ],
            encoder: Rotary::new(20),
            encoder_button: Debounced::new(false, 20),
        };

        // Program state

        let mut shared = Shared {
            led: device.led,
            usb,
        };

        let local = Local {
            bootloader: device.bootloader,
            inputs_state,
            pins: device.pins,
        };

        shared.led.set_low();

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().ok();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [usb, led], local = [pins, inputs_state])]
    fn tick(ctx: tick::Context) {
        let pins = ctx.local.pins;
        let state = ctx.local.inputs_state;
        let mut led = ctx.shared.led;

        // Read inputs and update state

        let pin_state = pins.scan();

        // FIXME centralise the pin mapping and make it less error prone
        // Ideally map pin type straight to an Encoder, Button or Switch
        state.encoder_button.update(pin_state[2]);
        state.encoder.update(pin_state[6], pin_state[5]);

        state.buttons[0].update(pin_state[0]);
        state.buttons[1].update(pin_state[1]);
        state.buttons[2].update(pin_state[3]);
        state.buttons[3].update(pin_state[4]);

        led.lock(|led| led.set_low());

        // Report state as HID joystick

        let buttons = [
            state.buttons[0].get(),
            state.buttons[1].get(),
            state.buttons[2].get(),
            state.buttons[3].get(),
            state.encoder.get_cw(),
            state.encoder.get_ccw(),
            state.encoder_button.get(),
        ];

        // Debug LED
        if buttons.iter().any(|b| *b) {
            led.lock(|led| led.set_high());
        }

        let mut report = Report::new();

        for (button, state) in buttons.iter().enumerate() {
            report.set_button(button, *state);
        }

        let mut usb = ctx.shared.usb;
        usb.lock(|usb| usb.report_input(report)).ok();

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn_after(Duration::<u64, 1, 1000>::millis(1)).ok();
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 2, shared = [usb])]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let mut usb = ctx.shared.usb;

        usb.lock(|usb| {
            match usb.poll() {
                Ok(Some(b'R')) => reset_to_bootloader::spawn().ok(),
                _ => None,
            };
        });
    }

    #[task(local = [bootloader])]
    fn reset_to_bootloader(ctx: reset_to_bootloader::Context) {
        let bootloader = ctx.local.bootloader;

        bootloader.reset();
    }
}
