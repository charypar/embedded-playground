

.PHONY: blinky
blinky:
	cd blinky && cargo build --release

.PHONY: spinny
spinny:
	cd spinny/software && cargo build --release

.PHONY: bootloader
bootloader:
	screen -dmS spinny /dev/tty.usbmodem14201 9600
	screen -S spinny -p 0 -X stuff 'R'
	sleep 0.2

.PHONY: dfu-spinny
dfu-spinny:
	cd spinny/software/target/thumbv7m-none-eabi/release && \
		arm-none-eabi-objcopy -O binary spinny spinny.bin && \
		dfu-util -D spinny.bin

.PHONY: upload-spinny
upload-spinny: bootloader dfu-spinny

.PHONY: blinky
flash-blinky: blinky
	openocd -f debug-config/openocd.cfg -c "program blinky/target/thumbv6m-none-eabi/release/blinky verify reset exit"

.PHONY: flash-spinny
flash-spinny: spinny
	openocd -f debug-config/openocd.cfg -c "program spinny/software/target/thumbv7m-none-eabi/release/spinny verify reset exit"

.PHONY: flash-bootloader
flash-bootloader: bootloader/dapboot.elf
	openocd -f debug-config/openocd.cfg -c "program bootloader/dapboot.elf verify reset exit"

.PHONY: test-probe
test-probe:
	openocd -f debug-config/openocd.cfg -c "targets exit"
