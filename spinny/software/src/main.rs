#![no_main]
#![no_std]

use panic_halt as _;
use rtic::app;

use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;

use inputs::PanelReport;

mod device;
mod inputs;
mod usb;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = JOYSTICK) = {
        (collection = PHYSICAL, usage = JOYSTICK) = {
            (usage_page = BUTTON, usage_min = BUTTON_1, usage_max = 0x07) = {
                #[packed_bits 7] #[item_settings data,variable,absolute] buttons=input;
            };
        }
    }
)]
#[derive(PartialEq)]
pub struct Report {
    pub buttons: u8,
}

impl PanelReport for Report {
    fn new() -> Self {
        Self { buttons: 0 }
    }

    fn set_button(&mut self, n: usize, value: bool) {
        if n > 6 {
            return;
        }

        if value {
            self.buttons |= 1 << n
        } else {
            self.buttons &= !(1 << n)
        }
    }
}

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
    use crate::inputs::{Button, Encoder, Panel};
    use crate::usb::Usb;

    use super::Report;

    #[monotonic(binds = SysTick, default = true)]
    type RtcMonotonic = Systick<1000>;

    #[shared]
    struct Shared {
        usb: Usb<'static, UsbBus<Peripheral>, Report>,
        led: PC13<Output<PushPull>>,
    }

    #[local]
    struct Local {
        bootloader: Bootloader,
        panel: Panel<5, 1>,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Set up the real-time clock, using the 32K oscillator clock
        // TODO this can be clocked by a subdivided clock for lower power consumption (probably)
        let rtc = Systick::new(ctx.core.SYST, 36_000_000);

        let device = Device::new(ctx.device);

        let usb_bus = ctx.local.usb_bus.insert(device.usb);
        let usb = Usb::new(usb_bus, Report::desc());

        let panel = Panel {
            buttons: [
                Button::new(device.pb3.erase(), false, 20),
                Button::new(device.pb4.erase(), false, 20),
                Button::new(device.pb6.erase(), false, 20),
                Button::new(device.pb7.erase(), false, 20),
                Button::new(device.pb5.erase(), false, 20),
            ],
            encoders: [Encoder::new(device.pb8.erase(), device.pb9.erase(), 20)],
        };

        // Program state

        let shared = Shared {
            led: device.led,
            usb,
        };

        let local = Local {
            bootloader: device.bootloader,
            panel,
        };

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().ok();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [usb, led], local = [panel])]
    fn tick(ctx: tick::Context) {
        let panel = ctx.local.panel;
        let mut led = ctx.shared.led;

        led.lock(|led| led.set_low());

        panel.scan();
        let report = panel.report::<Report>();

        if report.buttons > 0 {
            led.lock(|led| led.set_high());
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
