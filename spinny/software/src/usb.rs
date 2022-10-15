use usb_device::class_prelude::{UsbBus, UsbBusAllocator};
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::UsbError;

use usbd_serial::SerialPort;

use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::AsInputReport;
use usbd_hid::hid_class::HIDClass;

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

impl Report {
    pub fn new() -> Self {
        Self { buttons: 0 }
    }

    pub fn set_button(&mut self, n: usize, value: bool) {
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

// USB Interface
pub struct Usb<'a, B, IR>
where
    IR: AsInputReport + PartialEq<IR>,
    B: UsbBus,
{
    device: UsbDevice<'a, B>,
    serial: SerialPort<'a, B>,
    hid: HIDClass<'a, B>,
    last_report: Option<IR>, // Last input report sent
}

impl<'a, B, IR> Usb<'a, B, IR>
where
    IR: AsInputReport + PartialEq<IR>,
    B: UsbBus,
{
    pub fn new(
        allocator: &'a UsbBusAllocator<B>,
        report_descriptor: &'static [u8],
    ) -> Usb<'a, B, IR> {
        let serial_port = SerialPort::new(allocator);
        let hid_device = HIDClass::new_ep_in(allocator, report_descriptor, 4);

        // https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
        // TODO expose the string settings
        let usb_device = UsbDeviceBuilder::new(allocator, UsbVidPid(0x16c0, 0x27dc))
            .manufacturer("Niche http://niche.london/")
            .product("Spinny development board")
            .serial_number("niche.london:Spinny-v0.1")
            .build();

        Usb {
            device: usb_device,
            serial: serial_port,
            hid: hid_device,
            last_report: None,
        }
    }

    pub fn report_input(&mut self, report: IR) -> Result<usize, UsbError> {
        // Skip the report if it's the same as the last one
        if let Some(last_report) = &self.last_report {
            if *last_report == report {
                return Ok(0);
            }
        }

        let bytes = self.hid.push_input(&report)?;
        self.last_report = Some(report);

        Ok(bytes)
    }

    // FIXME think this interface through properly...
    pub fn poll(&mut self) -> Result<Option<u8>, UsbError> {
        self.device.poll(&mut [&mut self.serial, &mut self.hid]);

        // Prevent writes into the serial port locking everthing up
        // I am honestly not sure why this happens.
        let mut buf = [0u8; 1];

        match self.serial.read(&mut buf) {
            Ok(0) => Ok(None),
            Ok(count) => {
                self.serial.write(&buf[0..count])?;

                return Ok(Some(buf[0]));
            }
            Err(_) => Ok(None),
        }
    }
}
