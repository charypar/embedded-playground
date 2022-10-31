use hal::prelude::*;
use stm32f1xx_hal as hal;

use hal::pac;
use pac::{BKP, PWR, RCC, SCB};

use hal::gpio::gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9};
use hal::gpio::gpioc::PC13;
use hal::gpio::{GpioExt, Input, Output, PullUp, PushPull};
use hal::pac::Peripherals;
use hal::usb::{Peripheral, UsbBus};

use usb_device::class_prelude::UsbBusAllocator;

pub struct Device {
    pub bootloader: Bootloader,
    pub pb3: PB3<Input<PullUp>>,
    pub pb4: PB4<Input<PullUp>>,
    pub pb5: PB5<Input<PullUp>>,
    pub pb6: PB6<Input<PullUp>>,
    pub pb7: PB7<Input<PullUp>>,
    pub pb8: PB8<Input<PullUp>>,
    pub pb9: PB9<Input<PullUp>>,
    pub led: PC13<Output<PushPull>>,
    pub usb: UsbBusAllocator<UsbBus<Peripheral>>,
}

impl Device {
    pub fn new(device: Peripherals) -> Self {
        let rcc = device.RCC;
        let pwr = device.PWR;
        let flash = device.FLASH;
        let afio = device.AFIO;

        let bootloader = Bootloader::new(device.BKP, &rcc, &pwr);

        // Configure clocks
        let rcc = rcc.constrain();
        let mut acr = flash.constrain().acr;
        rcc.cfgr
            .use_hse(8.MHz()) // External crystal is on 8 Mhz
            .sysclk(72.MHz()) // Clock system on 72 Mhz, used also by USB, divided by 1.5 to 48 MHz
            .freeze(&mut acr);

        let mut gpioa = device.GPIOA.split();
        let mut gpiob = device.GPIOB.split();
        let mut gpioc = device.GPIOC.split();

        // Disable JTAG to allow access to PB3 and PB4
        let (_, pb3, pb4) = afio
            .constrain()
            .mapr
            .disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // Enable USB

        // FIXME why is this a thing again?
        // and do we need to sleep after?
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();

        let usb = UsbBus::new(Peripheral {
            usb: device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        });

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_low();

        Device {
            pb3: pb3.into_pull_up_input(&mut gpiob.crl),
            pb4: pb4.into_pull_up_input(&mut gpiob.crl),
            pb5: gpiob.pb5.into_pull_up_input(&mut gpiob.crl),
            pb6: gpiob.pb6.into_pull_up_input(&mut gpiob.crl),
            pb7: gpiob.pb7.into_pull_up_input(&mut gpiob.crl),
            pb8: gpiob.pb8.into_pull_up_input(&mut gpiob.crh),
            pb9: gpiob.pb9.into_pull_up_input(&mut gpiob.crh),
            bootloader,
            led,
            usb,
        }
    }
}

// A convenient dapboot bootloader interface
pub struct Bootloader {
    bkp: BKP,
}

impl Bootloader {
    const RESET_TO_BOOTLOADER: u32 = 0x4F42;

    // Create a bootloader inteface and enable signaling
    pub fn new(bkp: BKP, rcc: &RCC, pwr: &PWR) -> Self {
        // From stm32 reference manual, pg 81:
        // 1. enable the power and backup interface clocks
        // 2. set the DBP bit in the Power control register

        rcc.apb1enr.write(|w| w.pwren().set_bit().bkpen().set_bit());
        pwr.cr.write(|w| w.dbp().set_bit());

        Self { bkp }
    }

    // Restart the device into bootloader
    pub fn reset(&self) {
        // from dapboot documentation/source code
        // https://github.com/devanlai/dapboot#switching-to-the-bootloader
        // https://github.com/devanlai/dapboot/blob/master/src/stm32f103/backup.c#L25
        //
        // 3. write the bootloader signal
        self.bkp.dr[0].write(|w| unsafe { w.bits(Self::RESET_TO_BOOTLOADER) });

        // check the write succeeded
        if self.bkp.dr[0].read().bits() == Self::RESET_TO_BOOTLOADER {
            SCB::sys_reset();
        }
    }
}
