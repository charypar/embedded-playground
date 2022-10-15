#![no_main]
#![no_std]

use panic_halt as _;
use rtic::app;

mod device;
mod inputs;
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

    use crate::device::{Bootloader, Device};
    use crate::inputs::{Button, Encoder};
    use crate::usb::{Report, Usb};

    #[monotonic(binds = SysTick, default = true)]
    type RtcMonotonic = Systick<1000>;

    pub struct Inputs {
        buttons: [Button; 5],
        encoder: Encoder,
    }

    #[shared]
    struct Shared {
        usb: Usb<'static, UsbBus<Peripheral>, Report>,
        led: PC13<Output<PushPull>>,
    }

    #[local]
    struct Local {
        bootloader: Bootloader,
        inputs: Inputs,
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

        let inputs = Inputs {
            buttons: [
                Button::new(device.pb3.erase(), false, 20),
                Button::new(device.pb4.erase(), false, 20),
                Button::new(device.pb6.erase(), false, 20),
                Button::new(device.pb7.erase(), false, 20),
                Button::new(device.pb5.erase(), false, 20),
            ],
            encoder: Encoder::new(device.pb9.erase(), device.pb8.erase(), 20),
        };

        // Program state

        let mut shared = Shared {
            led: device.led,
            usb,
        };

        let local = Local {
            bootloader: device.bootloader,
            inputs,
        };

        shared.led.set_low();

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().ok();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [usb, led], local = [inputs])]
    fn tick(ctx: tick::Context) {
        let inputs = ctx.local.inputs;
        let mut led = ctx.shared.led;

        // Read inputs and update state

        led.lock(|led| led.set_low());

        // Scan the inputs

        inputs.buttons[0].scan();
        inputs.buttons[1].scan();
        inputs.buttons[2].scan();
        inputs.buttons[3].scan();
        inputs.buttons[4].scan();
        inputs.encoder.scan();

        // Report state as HID joystick

        let buttons = [
            inputs.buttons[0].get(),
            inputs.buttons[1].get(),
            inputs.buttons[2].get(),
            inputs.buttons[3].get(),
            inputs.buttons[4].get(),
            inputs.encoder.get_cw(),
            inputs.encoder.get_ccw(),
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
