#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod descriptor;

use panic_halt as _;

use rtic::app;

#[app(device = atsamd_hal::pac, peripherals = true, dispatchers = [EVSYS])]
mod app {
    use atsamd_hal as hal;
    use atsamd_hal::gpio::PushPullOutput;
    use hal::prelude::*;

    use hal::clock::GenericClockController;
    use hal::gpio::{Pin, Pins, PullUpInput};
    use hal::gpio::{PA04, PA05, PA10, PA11, PA14, PA27, PB02, PB03, PB09};
    use hal::rtc::{Count32Mode, Duration, Rtc};
    use hal::usb::UsbBus;

    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
    use usbd_hid::descriptor::SerializedDescriptor;
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

    use super::descriptor::JoystickReport;

    use cross::debounce::Debounced;
    use cross::rotary::Rotary;

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    pub struct InputsState {
        buttons: [Debounced<bool>; 4],
        encoder: Rotary,
        encoder_button: Debounced<bool>,
    }

    pub struct Inputs {
        buttons: (
            Pin<PB09, PullUpInput>,
            Pin<PA04, PullUpInput>,
            Pin<PA05, PullUpInput>,
            Pin<PB02, PullUpInput>,
        ),
        encoder_button: Pin<PA11, PullUpInput>,
        encoder_a: Pin<PA10, PullUpInput>,
        encoder_b: Pin<PA14, PullUpInput>,
    }

    #[shared]
    struct Shared {
        serial_port: SerialPort<'static, UsbBus>,
        hid_device: HIDClass<'static, UsbBus>,
    }

    #[local]
    struct Local {
        usb_device: UsbDevice<'static, UsbBus>,
        inputs: Inputs,
        inputs_state: InputsState,
        leds: (Pin<PA27, PushPullOutput>, Pin<PB03, PushPullOutput>),
    }

    #[init(local = [usb_allocator: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Begin hardware initialisation
        // FIXME pull all of this out

        let mut peri = ctx.device;
        let pins = Pins::new(peri.PORT);

        // Configure clocks

        let mut clocks = GenericClockController::with_internal_32kosc(
            peri.GCLK,
            &mut peri.PM,
            &mut peri.SYSCTRL,
            &mut peri.NVMCTRL,
        );
        let gclk0 = clocks.gclk0(); // OSC48Mhz
        let gclk1 = clocks.gclk1(); // OSC32K

        let rtc_clock = clocks.rtc(&gclk1).unwrap();
        let usb_clock = clocks.usb(&gclk0).unwrap();

        // Set up the real-time clock, using the 32K oscillator clock
        // TODO this can be clocked by a subdivided clock for lower power consumption (probably)
        let rtc = Rtc::count32_mode(peri.RTC, rtc_clock.freq(), &mut peri.PM);

        // Enable USB

        let usb_bus = UsbBus::new(&usb_clock, &mut peri.PM, pins.pa24, pins.pa25, peri.USB);
        let usb_allocator = ctx
            .local
            .usb_allocator
            .insert(UsbBusAllocator::new(usb_bus)); // Make usb_allocator 'static

        // End Hardware initialisation

        let (usb_device, serial_port, hid_device) = init_usb(usb_allocator);

        // Input

        let inputs = Inputs {
            buttons: (
                pins.pb09.into(),
                pins.pa04.into(),
                pins.pa05.into(),
                pins.pb02.into(),
            ),
            encoder_button: pins.pa11.into(),
            encoder_a: pins.pa10.into(),
            encoder_b: pins.pa14.into(),
        };

        let inputs_state = InputsState {
            buttons: [
                Debounced::new(false, 10),
                Debounced::new(false, 10),
                Debounced::new(false, 10),
                Debounced::new(false, 10),
            ],
            encoder: Rotary::new(20),
            encoder_button: Debounced::new(false, 10),
        };

        // Program state

        let shared = Shared {
            serial_port,
            hid_device,
        };

        let mut local = Local {
            usb_device,
            inputs,
            inputs_state,
            leds: (pins.pa27.into(), pins.pb03.into()),
        };

        local.leds.0.set_high().unwrap();
        local.leds.1.set_high().unwrap();

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().unwrap();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [serial_port, hid_device], local = [inputs, inputs_state, leds])]
    fn tick(ctx: tick::Context) {
        let pins = ctx.local.inputs;
        let state = ctx.local.inputs_state;
        let leds = ctx.local.leds;

        let mut _serial_port = ctx.shared.serial_port;

        // Read inputs and update state

        state
            .encoder_button
            .update(pins.encoder_button.is_low().unwrap());
        let (a, b) = (
            pins.encoder_a.is_low().unwrap(),
            pins.encoder_b.is_low().unwrap(),
        );
        state.encoder.update(a, b);

        state.buttons[0].update(pins.buttons.0.is_low().unwrap());
        state.buttons[1].update(pins.buttons.1.is_low().unwrap());
        state.buttons[2].update(pins.buttons.2.is_low().unwrap());
        state.buttons[3].update(pins.buttons.3.is_low().unwrap());

        // reset LEDs

        leds.0.set_high().unwrap();
        leds.1.set_high().unwrap();

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

        // Debug LEDs

        if buttons[4] {
            leds.0.set_low().unwrap();
        }

        if buttons[5] {
            leds.1.set_low().unwrap();
        }

        if buttons[0] || buttons[1] || buttons[2] || buttons[3] || buttons[6] {
            leds.0.set_low().unwrap();
            leds.1.set_low().unwrap();
        }

        let mut report = JoystickReport::new();

        for (button, state) in buttons.iter().enumerate() {
            report.set_button(button, *state);
        }

        let mut hid_device = ctx.shared.hid_device;

        hid_device.lock(|hid| hid.push_input(&report).ok());

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn_after(Duration::millis(1)).unwrap();
    }

    #[task(binds = USB, priority = 2, local = [usb_device], shared = [serial_port, hid_device])]
    fn usb_poll(ctx: usb_poll::Context) {
        let usb_device = ctx.local.usb_device;
        let serial_port = ctx.shared.serial_port;
        let hid_device = ctx.shared.hid_device;

        (serial_port, hid_device).lock(|serial, hid| {
            usb_device.poll(&mut [serial, hid]);

            // Prevent writes into the serial port locking everthing up
            // I am honestly not sure why this happens.
            let mut buf = [0u8; 64];
            serial.read(&mut buf).ok();
        });
    }

    fn init_usb<'a>(
        allocator: &'a UsbBusAllocator<UsbBus>,
    ) -> (
        UsbDevice<'a, UsbBus>,
        SerialPort<'a, UsbBus>,
        HIDClass<'a, UsbBus>,
    ) {
        let serial = SerialPort::new(allocator);
        let hid_device = HIDClass::new_ep_in(allocator, JoystickReport::desc(), 10);

        // https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
        let usb_device = UsbDeviceBuilder::new(allocator, UsbVidPid(0x16c0, 0x27dc))
            .manufacturer("Niche http://niche.london/")
            .product("Spinny development board")
            .serial_number("niche.london:Spinny-v0.1")
            .composite_with_iads()
            .build();

        (usb_device, serial, hid_device)
    }
}
