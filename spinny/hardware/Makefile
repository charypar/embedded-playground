.PHONY: fab
fab: clean
	kikit fab jlcpcb --assembly --schematic Spinny.kicad_sch Spinny.kicad_pcb production/pcb/

.PHONY: clean
clean:
	rm -rf production/pcb/*
