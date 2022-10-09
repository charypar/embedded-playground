#![no_main]
#![no_std]

use panic_halt as _;
use rtic::app;

mod descriptor;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use hal::prelude::*;
    use stm32f1xx_hal as hal;

    use hal::gpio::gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9};
    use hal::gpio::gpioc::PC13;
    use hal::gpio::{Input, Output, PullUp, PushPull};
    use hal::usb::{Peripheral, UsbBus};

    use hal::pac;
    use pac::{BKP, PWR, RCC, SCB};

    use systick_monotonic::{fugit::Duration, Systick};

    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
    use usbd_hid::descriptor::SerializedDescriptor;
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

    use super::descriptor::JoystickReport;

    use cross::debounce::Debounced;
    use cross::rotary::Rotary;

    const RESET_TO_BOOTLOADER: u32 = 0x4F42;

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
        microseconds: u32,
        usb_device: UsbDevice<'static, UsbBus<Peripheral>>,
        serial_port: SerialPort<'static, UsbBus<Peripheral>>,
        hid_device: HIDClass<'static, UsbBus<Peripheral>>,
        bkp: BKP,
    }

    #[local]
    struct Local {
        inputs: Inputs,
        inputs_state: InputsState,
        led: PC13<Output<PushPull>>,
        last_report: JoystickReport,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Begin hardware initialisation
        // FIXME pull all of this out

        let rcc = ctx.device.RCC;
        let pwr = ctx.device.PWR;
        let flash = ctx.device.FLASH;
        let afio = ctx.device.AFIO;

        enable_write_to_bkp(&rcc, &pwr);

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

        // Enable USB

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();
        let mut gpioc = ctx.device.GPIOC.split();

        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        };

        let usb_bus = ctx.local.usb_bus.insert(UsbBus::new(usb));

        // End Hardware initialisation

        let (usb_device, serial_port, hid_device) = init_usb(usb_bus);

        // Input

        // Disable JTAG to allow access to PB3 and PB4
        let (_, pb3, pb4) = afio
            .constrain()
            .mapr
            .disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

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

        // Program state

        let shared = Shared {
            microseconds: 0,
            usb_device,
            serial_port,
            hid_device,
            bkp: ctx.device.BKP,
        };

        let mut local = Local {
            inputs,
            inputs_state,
            led: gpioc.pc13.into_push_pull_output(&mut gpioc.crh),
            last_report: JoystickReport::new(),
        };

        local.led.set_high();

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn().unwrap();

        (shared, local, init::Monotonics(rtc))
    }

    #[task(shared = [serial_port, hid_device, microseconds], local = [inputs, inputs_state, led, last_report])]
    fn tick(ctx: tick::Context) {
        let pins = ctx.local.inputs;
        let state = ctx.local.inputs_state;
        let led = ctx.local.led;
        let last_report = ctx.local.last_report;

        // Read inputs and update state

        state.encoder_button.update(pins.encoder_button.is_low());
        let (a, b) = (pins.encoder_a.is_low(), pins.encoder_b.is_low());
        state.encoder.update(a, b);

        state.buttons[0].update(pins.buttons.0.is_low());
        state.buttons[1].update(pins.buttons.1.is_low());
        state.buttons[2].update(pins.buttons.2.is_low());
        state.buttons[3].update(pins.buttons.3.is_low());

        led.set_low();

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
            led.set_high()
        }

        let mut report = JoystickReport::new();

        for (button, state) in buttons.iter().enumerate() {
            report.set_button(button, *state);
        }

        let mut hid_device = ctx.shared.hid_device;

        if report != *last_report {
            hid_device.lock(|hid| hid.push_input(&report).ok());
            *last_report = report;
        }

        // TODO see if we can do this on a periodic timer instead,
        // so the task execution isn't fallible
        tick::spawn_after(Duration::<u64, 1, 1000>::millis(1)).unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, priority = 2, shared = [usb_device, serial_port, hid_device, bkp])]
    fn usb_tx(ctx: usb_tx::Context) {
        let usb_device = ctx.shared.usb_device;
        let serial_port = ctx.shared.serial_port;
        let hid_device = ctx.shared.hid_device;
        let bkp = ctx.shared.bkp;

        (usb_device, serial_port, hid_device, bkp).lock(|device, serial, hid, bkp| {
            usb_poll(device, serial, hid, bkp);
        });
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 2, shared = [usb_device, serial_port, hid_device, bkp])]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let usb_device = ctx.shared.usb_device;
        let serial_port = ctx.shared.serial_port;
        let hid_device = ctx.shared.hid_device;
        let bkp = ctx.shared.bkp;

        (usb_device, serial_port, hid_device, bkp).lock(|device, serial, hid, bkp| {
            usb_poll(device, serial, hid, bkp);
        });
    }

    fn usb_poll(
        device: &mut UsbDevice<UsbBus<Peripheral>>,
        serial: &mut SerialPort<UsbBus<Peripheral>>,
        hid: &mut HIDClass<UsbBus<Peripheral>>,
        bkp: &mut BKP,
    ) {
        device.poll(&mut [serial, hid]);

        // Prevent writes into the serial port locking everthing up
        // I am honestly not sure why this happens.
        let mut buf = [0u8; 1];
        match serial.read(&mut buf) {
            Ok(0) => (),
            Ok(count) => {
                // TODO better reset command
                if buf[0] == b'R' {
                    if signal_bootloader(bkp) {
                        serial.write(b"resetting to bootloader!").ok();

                        SCB::sys_reset();
                    } else {
                        serial.write(b"reset failed").ok();
                    }
                } else {
                    serial.write(&buf[0..count]).ok();
                }
            }
            Err(_) => (),
        }
    }

    // Enable writing into the BKP registers for software reset to bootloader
    fn enable_write_to_bkp(rcc: &RCC, pwr: &PWR) {
        // From stm32 reference manual, pg 81:
        // 1. enable the power and backup interface clocks
        // 2. set the DBP bit in the Power control register

        rcc.apb1enr.write(|w| w.pwren().set_bit().bkpen().set_bit());
        pwr.cr.write(|w| w.dbp().set_bit());
    }

    fn signal_bootloader(bkp: &mut BKP) -> bool {
        // from dapboot documentation/source code
        // https://github.com/devanlai/dapboot#switching-to-the-bootloader
        // https://github.com/devanlai/dapboot/blob/master/src/stm32f103/backup.c#L25
        //
        // 3. write the bootloader signal
        bkp.dr[0].write(|w| unsafe { w.bits(RESET_TO_BOOTLOADER) });

        // check the write succeeded
        return bkp.dr[0].read().bits() == RESET_TO_BOOTLOADER;
    }

    fn init_usb<'a>(
        allocator: &'a UsbBusAllocator<UsbBus<Peripheral>>,
    ) -> (
        UsbDevice<'a, UsbBus<Peripheral>>,
        SerialPort<'a, UsbBus<Peripheral>>,
        HIDClass<'a, UsbBus<Peripheral>>,
    ) {
        let serial = SerialPort::new(allocator);
        let hid_device = HIDClass::new_ep_in(allocator, JoystickReport::desc(), 4);

        // https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
        let usb_device = UsbDeviceBuilder::new(allocator, UsbVidPid(0x16c0, 0x27dc))
            .manufacturer("Niche http://niche.london/")
            .product("Spinny development board")
            .serial_number("niche.london:Spinny-v0.1")
            .build();

        (usb_device, serial, hid_device)
    }
}
