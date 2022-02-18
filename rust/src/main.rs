#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod debounce;
mod rotary;

use panic_halt as _;

use rtic::app;

use smart_leds_trait::RGB8;

#[app(device = atsamd_hal::pac, peripherals = true, dispatchers = [EVSYS])]
mod app {
    use atsamd_hal as hal;
    use hal::prelude::*;
    use hal::typelevel::NoneT;

    use hal::clock::GenericClockController;
    use hal::gpio::v2::{Alternate, Input, Output, Pin, Pins, PullDown, PushPull, D};
    use hal::gpio::v2::{PA00, PA01, PA02, PA06, PA07, PA08, PA09, PA10};
    use hal::rtc::{Count32Mode, Duration, Rtc};
    use hal::sercom::v2::{spi, Sercom1};
    use hal::usb::UsbBus;

    use apa102_spi::Apa102;
    use smart_leds::SmartLedsWrite;
    use smart_leds_trait::RGB8;
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
    use usb_device::UsbError;
    use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

    use super::debounce::Debounced;
    use super::rotary::{self, Rotary};
    use super::{adjust_brightess, COLOURS};

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    type DotStar = Apa102<
        spi::Spi<
            spi::Config<
                spi::Pads<Sercom1, NoneT, Pin<PA00, Alternate<D>>, Pin<PA01, Alternate<D>>>,
            >,
            spi::Tx,
        >,
    >;

    pub struct Inputs {
        toggle: Debounced<bool>,
        button_a: Debounced<bool>,
        button_b: Debounced<bool>,
        encoder: Rotary,
    }

    pub struct PinMatrix {
        rows: (Pin<PA06, Output<PushPull>>, Pin<PA07, Output<PushPull>>),
        columns: (
            Pin<PA08, Input<PullDown>>,
            Pin<PA02, Input<PullDown>>,
            Pin<PA09, Input<PullDown>>,
        ),
    }

    pub struct LED {
        enabled: bool,
        colour: usize,
        brightness: u8,
    }

    #[shared]
    struct Shared {
        // serial: SerialPort<'static, UsbBus>,
        hid_device: HIDClass<'static, UsbBus>,
        red_led: Pin<PA10, Output<PushPull>>,
    }

    #[local]
    struct Local {
        usb_device: UsbDevice<'static, UsbBus>,
        dotstar: DotStar,
        matrix: PinMatrix,
        inputs: Inputs,
        led: LED,
        colour: RGB8,
    }

    #[init(local = [usb_allocator: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut peri = ctx.device;
        let pins = Pins::new(peri.PORT);

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

        // DotStar LED control over SPI on SERCOM1

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

        // Create the SPI interface and wrap it in Apa102 LED driver

        let spi_pads = spi::Pads::<Sercom1>::default()
            .data_out(pins.pa00)
            .sclk(pins.pa01);

        let spi = spi::Config::new(&peri.PM, peri.SERCOM1, spi_pads, spi_clock)
            .spi_mode(apa102_spi::MODE)
            .baud(24.mhz())
            .enable();
        let mut dotstar = Apa102::new(spi);

        // Enable USB

        let usb_bus = UsbBus::new(&usb_clock, &mut peri.PM, pins.pa24, pins.pa25, peri.USB);
        let alloc = UsbBusAllocator::new(usb_bus);

        // Make usb_allocator 'static
        let usb_allocator = ctx.local.usb_allocator.insert(alloc);
        // let serial = SerialPort::new(usb_allocator);
        let hid_device = HIDClass::new_ep_in(usb_allocator, KeyboardReport::desc(), 100);

        let usb_device = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x16c0, 0x27db))
            .manufacturer("Niche http://niche.london")
            .product("Blinky development board")
            .serial_number("0000")
            .build();

        // Input

        let inputs = Inputs {
            toggle: Debounced::new(false, 20),
            button_a: Debounced::new(false, 10),
            button_b: Debounced::new(false, 10),
            encoder: Rotary::new(),
        };

        // Program state

        let led = LED {
            enabled: true,
            colour: 5,
            brightness: 31,
        };

        let d13 = pins.pa10.into_push_pull_output();

        let colour = adjust_brightess(&COLOURS[5], 31);
        dotstar.write([colour].iter().cloned()).ok();

        let shared = Shared {
            // serial,
            hid_device,
            red_led: d13,
        };

        let local = Local {
            usb_device,
            dotstar,
            matrix,
            inputs,
            led,
            colour,
        };

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().unwrap();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [hid_device, red_led], local = [matrix, inputs, led, dotstar, colour])]
    fn tick(ctx: tick::Context) {
        let PinMatrix { rows, columns } = ctx.local.matrix;
        let inputs = ctx.local.inputs;
        let mut led = ctx.local.led;
        let dotstar = ctx.local.dotstar;
        let colour = ctx.local.colour;

        // let mut serial = ctx.shared.serial;
        let mut hid_device = ctx.shared.hid_device;
        let mut red_led = ctx.shared.red_led;

        rows.0.set_high().unwrap();

        let (a, b) = (columns.1.is_high().unwrap(), columns.2.is_high().unwrap());

        let enc_out = inputs.encoder.update(a, b);

        rows.0.set_low().unwrap();
        rows.1.set_high().unwrap();

        let btn_a_out = inputs.button_a.update(columns.0.is_high().unwrap());
        let btn_b_out = inputs.button_b.update(columns.1.is_high().unwrap());
        let toggle_out = inputs.toggle.update(columns.2.is_low().unwrap());

        rows.1.set_low().unwrap();

        // Update state

        led.enabled = inputs.toggle.get();

        // match toggle_out {
        //     Some(true) => {
        //         serial.lock(|serial| serial.write("ON\n\r".as_bytes()).ok());
        //     }
        //     Some(false) => {
        //         serial.lock(|serial| serial.write("OFF\n\r".as_bytes()).ok());
        //     }
        //     _ => (),
        // }

        match enc_out {
            rotary::Out::CW => {
                if led.brightness < 247 {
                    led.brightness += 8;
                }
                // serial.lock(|serial| serial.write("UP\n\r".as_bytes()).ok());
            }
            rotary::Out::CCW => {
                if led.brightness > 8 {
                    led.brightness -= 8;
                }
                // serial.lock(|serial| serial.write("DOWN\n\r".as_bytes()).ok());
            }
            _ => (),
        }

        match btn_a_out {
            Some(true) => {
                if led.colour < 6 {
                    led.colour = led.colour + 1;
                }
                // serial.lock(|serial| serial.write("A\n\r".as_bytes()).ok());
            }
            Some(false) => {
                // serial.lock(|serial| serial.write("!A\n\r".as_bytes()).ok());
            }
            _ => (),
        }

        match btn_b_out {
            Some(true) => {
                if led.colour > 0 {
                    led.colour = led.colour - 1;
                }
                // serial.lock(|serial| serial.write("B\n\r".as_bytes()).ok());
            }
            Some(false) => {
                // serial.lock(|serial| serial.write("!B\n\r".as_bytes()).ok());
            }
            _ => (),
        }

        // Update LED

        let c = if led.enabled {
            adjust_brightess(&COLOURS[led.colour], led.brightness)
        } else {
            RGB8 { r: 0, g: 0, b: 0 }
        };

        if *colour != c {
            dotstar.write([c].iter().cloned()).ok();
            *colour = c;
        }

        // HID keyboard

        let keycodes = match (inputs.button_a.get(), inputs.button_b.get()) {
            (true, true) => [04, 05, 0, 0, 0, 0],
            (true, false) => [04, 0, 0, 0, 0, 0],
            (false, true) => [05, 0, 0, 0, 0, 0],
            _ => [0; 6],
        };

        let modifier = if inputs.toggle.get() { 0b10 } else { 0 };

        let report = KeyboardReport {
            modifier,
            reserved: 0,
            leds: 0,
            keycodes,
        };

        match hid_device.lock(|hid| hid.push_input(&report)) {
            Ok(_) => red_led.lock(|red_led| red_led.set_low().ok()),
            Err(_) => red_led.lock(|red_led| red_led.set_high().ok()),
        };

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn_after(Duration::millis(1)).unwrap();
    }

    #[task(binds = USB, priority = 2, local = [usb_device], shared = [ hid_device, red_led])]
    fn usb_poll(ctx: usb_poll::Context) {
        let usb_device = ctx.local.usb_device;
        // let mut serial = ctx.shared.serial;
        let mut hid_device = ctx.shared.hid_device;
        // let mut red_led = ctx.shared.red_led;

        // serial.lock(|serial| usb_device.poll(&mut [serial]));
        hid_device.lock(|hid| usb_device.poll(&mut [hid]));
    }

    fn print_error(err: UsbError, s: &mut SerialPort<UsbBus>) -> Result<usize, UsbError> {
        match err {
            usb_device::UsbError::WouldBlock => s.write("would block\n\r".as_bytes()),
            usb_device::UsbError::ParseError => s.write("parse error\n\r".as_bytes()),
            usb_device::UsbError::BufferOverflow => s.write("buffer ovrfl\n\r".as_bytes()),
            usb_device::UsbError::EndpointOverflow => s.write("endp ovrfl\n\r".as_bytes()),
            usb_device::UsbError::EndpointMemoryOverflow => {
                s.write("endp mem ovrfl\n\r".as_bytes())
            }
            usb_device::UsbError::InvalidEndpoint => s.write("inv endp\n\r".as_bytes()),
            usb_device::UsbError::Unsupported => s.write("unsupp\n\r".as_bytes()),
            usb_device::UsbError::InvalidState => s.write("inv state\n\r".as_bytes()),
        }
    }
}

#[rustfmt::skip]
const COLOURS: [RGB8; 7] = [
    RGB8 { r: 148, g: 0, b: 211 },
    RGB8 { r: 75, g: 0, b: 130 },
    RGB8 { r: 0, g: 0, b: 255 },
    RGB8 { r: 0, g: 255, b: 0 },
    RGB8 { r: 255, g: 255, b: 0 },
    RGB8 { r: 255, g: 127, b: 0 },
    RGB8 { r: 255, g: 0, b: 0 },
];

fn adjust_brightess(colour: &RGB8, brightness: u8) -> RGB8 {
    RGB8 {
        r: (colour.r as u32 * brightness as u32 / 255) as u8,
        g: (colour.g as u32 * brightness as u32 / 255) as u8,
        b: (colour.b as u32 * brightness as u32 / 255) as u8,
    }
}

// Input
