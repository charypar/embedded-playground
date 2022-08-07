

.PHONY: blinky
blinky:
	cd blinky && cargo build --release

.PHONY: spinny
spinny:
	cd spinny/software && cargo build --release

.PHONY: blinky
flash-blinky: blinky
	openocd -f debug-config/openocd.cfg -c "program blinky/target/thumbv6m-none-eabi/release/blinky verify reset exit"

.PHONY: flash-spinny
flash-spinny: spinny
	openocd -f debug-config/openocd.cfg -c "program spinny/software/target/thumbv7m-none-eabi/release/spinny verify reset exit"
