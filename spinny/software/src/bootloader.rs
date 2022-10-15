use stm32f1xx_hal as hal;

use hal::pac;
use pac::{BKP, PWR, RCC, SCB};

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
