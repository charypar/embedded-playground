#![no_main]
#![no_std]

use panic_halt as _;
use rtic::app;

mod bootloader;
mod usb;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use hal::prelude::*;
    use stm32f1xx_hal as hal;

    use hal::gpio::gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9};
    use hal::gpio::gpioc::PC13;
    use hal::gpio::{Input, Output, PullUp, PushPull};
    use hal::usb::{Peripheral, UsbBus};

    use systick_monotonic::{fugit::Duration, Systick};

    use usb_device::class_prelude::UsbBusAllocator;
    use usbd_hid::descriptor::SerializedDescriptor;

    use super::bootloader::Bootloader;
    use super::usb::{Report, Usb};

    use cross::debounce::Debounced;
    use cross::rotary::Rotary;

    #[monotonic(binds = SysTick, default = true)]
    type RtcMonotonic = Systick<1000>;

    pub struct InputsState {
        buttons: [Debounced<bool>; 4],
        encoder: Rotary,
        encoder_button: Debounced<bool>,
    }

    pub struct Inputs {
        buttons: (
            PB3<Input<PullUp>>,
            PB4<Input<PullUp>>,
            PB6<Input<PullUp>>,
            PB7<Input<PullUp>>,
        ),
        encoder_button: PB5<Input<PullUp>>,
        encoder_a: PB9<Input<PullUp>>,
        encoder_b: PB8<Input<PullUp>>,
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
        inputs_state: InputsState,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Begin hardware initialisation
        // FIXME pull all of this out

        let rcc = ctx.device.RCC;
        let pwr = ctx.device.PWR;
        let flash = ctx.device.FLASH;
        let afio = ctx.device.AFIO;

        let bootloader = Bootloader::new(ctx.device.BKP, &rcc, &pwr);

        // Configure clocks
        let rcc = rcc.constrain();
        let mut acr = flash.constrain().acr;
        rcc.cfgr
            .use_hse(8.MHz()) // External crystal is on 8 Mhz
            .sysclk(72.MHz()) // Clock system on 72 Mhz, used also by USB, divided by 1.5 to 48 MHz
            .freeze(&mut acr);

        // Set up the real-time clock, using the 32K oscillator clock
        // TODO this can be clocked by a subdivided clock for lower power consumption (probably)
        let rtc = Systick::new(ctx.core.SYST, 36_000_000);

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();
        let mut gpioc = ctx.device.GPIOC.split();

        // Disable JTAG to allow access to PB3 and PB4
        let (_, pb3, pb4) = afio
            .constrain()
            .mapr
            .disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // Enable USB

        // FIXME why is this a thing again?
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();

        let usb = UsbBus::new(Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        });
        let usb_bus = ctx.local.usb_bus.insert(usb);

        let usb = Usb::new(usb_bus, Report::desc());

        // End Hardware initialisation

        // Input
        // TODO factor this out

        let inputs = Inputs {
            buttons: (
                pb3.into_pull_up_input(&mut gpiob.crl),
                pb4.into_pull_up_input(&mut gpiob.crl),
                gpiob.pb6.into_pull_up_input(&mut gpiob.crl),
                gpiob.pb7.into_pull_up_input(&mut gpiob.crl),
            ),
            encoder_button: gpiob.pb5.into_pull_up_input(&mut gpiob.crl),
            encoder_a: gpiob.pb9.into_pull_up_input(&mut gpiob.crh),
            encoder_b: gpiob.pb8.into_pull_up_input(&mut gpiob.crh),
        };

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

        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // Program state

        let mut shared = Shared { usb, led };

        let local = Local {
            bootloader,
            inputs,
            inputs_state,
        };

        shared.led.set_low();

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().ok();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [usb, led], local = [inputs, inputs_state])]
    fn tick(ctx: tick::Context) {
        let pins = ctx.local.inputs;
        let state = ctx.local.inputs_state;
        let mut led = ctx.shared.led;

        // Read inputs and update state

        state.encoder_button.update(pins.encoder_button.is_low());
        let (a, b) = (pins.encoder_a.is_low(), pins.encoder_b.is_low());
        state.encoder.update(a, b);

        state.buttons[0].update(pins.buttons.0.is_low());
        state.buttons[1].update(pins.buttons.1.is_low());
        state.buttons[2].update(pins.buttons.2.is_low());
        state.buttons[3].update(pins.buttons.3.is_low());

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

        // Debug LEDs
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
