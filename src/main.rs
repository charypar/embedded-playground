#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod debounce;
mod dotstar;
mod hold;
mod rotary;

use panic_halt as _;

use rtic::app;

// DotStar

#[app(device = atsamd_hal::pac, peripherals = true, dispatchers = [EVSYS])]
mod app {
    use atsamd_hal as hal;
    use hal::prelude::*;

    use hal::clock::GenericClockController;
    use hal::gpio::v2::{Input, Output, Pin, Pins, PullDown, PushPull};
    use hal::gpio::v2::{PA02, PA06, PA07, PA08, PA09};
    use hal::rtc::{Count32Mode, Duration, Rtc};
    use hal::usb::UsbBus;

    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
    use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

    use super::debounce::Debounced;
    use super::dotstar::DotStar;
    use super::hold::Held;
    use super::rotary::{self, Rotary};

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    pub struct Inputs {
        encoder: Rotary,
        toggle: Debounced<bool>,
        button_a: Debounced<bool>,
        button_b: Debounced<bool>,
        encoder_button: Debounced<bool>,
        encoder_up: Held<bool>,
        encoder_down: Held<bool>,
    }

    pub struct PinMatrix {
        rows: (Pin<PA06, Output<PushPull>>, Pin<PA07, Output<PushPull>>),
        columns: (
            Pin<PA08, Input<PullDown>>,
            Pin<PA02, Input<PullDown>>,
            Pin<PA09, Input<PullDown>>,
        ),
    }

    #[shared]
    struct Shared {
        serial_port: SerialPort<'static, UsbBus>,
        hid_device: HIDClass<'static, UsbBus>,
    }

    #[local]
    struct Local {
        usb_device: UsbDevice<'static, UsbBus>,
        dotstar: DotStar,
        matrix: PinMatrix,
        inputs: Inputs,
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
        let spi_clock = clocks.sercom1_core(&gclk0).unwrap();
        let usb_clock = clocks.usb(&gclk0).unwrap();

        // Set up the real-time clock, using the 32K oscillator clock
        // TODO this can be clocked by a subdivided clock for lower power consumption (probably)
        let rtc = Rtc::count32_mode(peri.RTC, rtc_clock.freq(), &mut peri.PM);

        let mut dotstar = DotStar::new(pins.pa00, pins.pa01, &peri.PM, peri.SERCOM1, spi_clock);
        dotstar.display();

        // Enable USB

        let usb_bus = UsbBus::new(&usb_clock, &mut peri.PM, pins.pa24, pins.pa25, peri.USB);
        let usb_allocator = ctx
            .local
            .usb_allocator
            .insert(UsbBusAllocator::new(usb_bus)); // Make usb_allocator 'static

        // End Hardware initialisation

        let (usb_device, serial_port, hid_device) = init_usb(usb_allocator);

        // Input matrix

        let matrix = PinMatrix {
            rows: (
                pins.pa06.into_push_pull_output(),
                pins.pa07.into_push_pull_output(),
            ),
            columns: (
                pins.pa08.into_pull_down_input(),
                pins.pa02.into_pull_down_input(),
                pins.pa09.into_pull_down_input(),
            ),
        };

        // Input

        let inputs = Inputs {
            encoder: Rotary::new(),
            toggle: Debounced::new(false, 20),
            button_a: Debounced::new(false, 10),
            button_b: Debounced::new(false, 10),
            encoder_button: Debounced::new(false, 5),
            encoder_up: Held::new(false, 6),
            encoder_down: Held::new(false, 6),
        };

        // Program state

        let shared = Shared {
            serial_port,
            hid_device,
        };

        let local = Local {
            usb_device,
            dotstar,
            matrix,
            inputs,
        };

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().unwrap();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [serial_port, hid_device], local = [matrix, inputs, dotstar])]
    fn tick(ctx: tick::Context) {
        let PinMatrix { rows, columns } = ctx.local.matrix;
        let inputs = ctx.local.inputs;

        let mut serial_port = ctx.shared.serial_port;

        rows.0.set_high().unwrap();

        inputs.encoder_button.update(columns.0.is_high().unwrap());
        let (a, b) = (columns.1.is_high().unwrap(), columns.2.is_high().unwrap());

        let enc_out = inputs.encoder.update(a, b);

        rows.0.set_low().unwrap();
        rows.1.set_high().unwrap();

        let btn_a_out = inputs.button_a.update(columns.0.is_high().unwrap());
        let btn_b_out = inputs.button_b.update(columns.1.is_high().unwrap());
        inputs.toggle.update(columns.2.is_low().unwrap());

        rows.1.set_low().unwrap();

        // Update state

        let mut dotstar = ctx.local.dotstar;

        dotstar.enabled = inputs.toggle.get();

        match enc_out {
            rotary::Out::CW => {
                dotstar.set_brightness(dotstar.brightness() + 8);
                inputs.encoder_up.update(true);
                serial_port.lock(|serial| serial.write("UP\n\r".as_bytes()).ok());
            }
            rotary::Out::CCW => {
                dotstar.set_brightness(dotstar.brightness() - 8);
                inputs.encoder_down.update(true);
                serial_port.lock(|serial| serial.write("DOWN\n\r".as_bytes()).ok());
            }
            _ => {
                inputs.encoder_up.update(false);
                inputs.encoder_down.update(false);
            }
        }

        match btn_a_out {
            Some(true) => {
                dotstar.set_color(dotstar.color() + 1);
                serial_port.lock(|serial| serial.write("A\n\r".as_bytes()).ok());
            }
            Some(false) => {
                serial_port.lock(|serial| serial.write("!A\n\r".as_bytes()).ok());
            }
            _ => (),
        }

        match btn_b_out {
            Some(true) => {
                dotstar.set_color(dotstar.color() - 1);
                serial_port.lock(|serial| serial.write("B\n\r".as_bytes()).ok());
            }
            Some(false) => {
                serial_port.lock(|serial| serial.write("!B\n\r".as_bytes()).ok());
            }
            _ => (),
        }

        // Update LED

        dotstar.display();

        // HID keyboard

        let mut keycodes = [0; 6];

        let mapping = [
            (inputs.button_a.get(), 0x04),
            (inputs.button_b.get(), 0x05),
            (inputs.encoder_button.get(), 0x08),
            (inputs.encoder_up.get(), 0x18),
            (inputs.encoder_down.get(), 0x07),
        ];

        for (input, code) in mapping {
            if !input {
                continue;
            }

            if let Some(idx) = keycodes.iter().position(|&x| x == 0) {
                keycodes[idx] = code;
            }
        }

        let modifier = if inputs.toggle.get() { 0b10 } else { 0 };

        let report = KeyboardReport {
            modifier,
            reserved: 0,
            leds: 0,
            keycodes,
        };

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
        let hid_device = HIDClass::new_ep_in(allocator, KeyboardReport::desc(), 5);

        let usb_device = UsbDeviceBuilder::new(allocator, UsbVidPid(0x16c0, 0x27db))
            .manufacturer("Niche http://niche.london")
            .product("Blinky development board")
            .serial_number("0000")
            .composite_with_iads()
            .build();

        (usb_device, serial, hid_device)
    }
}
