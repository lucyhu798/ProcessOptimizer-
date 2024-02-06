.PHONY: clean all rams syn sta

configs=
ifneq (clean,$(MAKECMDGOALS))
include configs.mk
endif

ram_targets=$(configs:%=build/sram_%_freepdk45_analytical_TT_1p0V_25C.lib)

all: netlist.v

mips_core.v: ../mips_core/*.sv
	$(CSE148_TOOLS)/sv2v-Linux/sv2v -Imips_core ../mips_core/*.sv > $@

hierarchy.json: mips_core.v
	bash -c "source $(CSE148_TOOLS)/oss-cad-suite/environment && yosys -s hierarchy.ys -l hierarchy.log -t"

configs.mk synthesis.ys sram.v timing.sta: hierarchy.json generate.py
	python3 generate.py

rams: $(ram_targets)

build/sram_%_freepdk45_analytical_TT_1p0V_25C.lib : config.py
	env RAM_CONFIG="$*" \
		FREEPDK45=$(CSE148_TOOLS)/FreePDK45 \
		OPENRAM_HOME=$(CSE148_TOOLS)/OpenRAM/compiler \
		OPENRAM_TECH=$(CSE148_TOOLS)/OpenRAM/technology \
		   python3 $(CSE148_TOOLS)/OpenRAM/compiler/openram.py config

netlist.v: mips_core.v $(ram_targets) synthesis.ys
	bash -c "source $(CSE148_TOOLS)/oss-cad-suite/environment && yosys -s synthesis.ys -l synthesis.log -t"

syn: netlist.v

sta: netlist.v timing.sta $(ram_targets)
	$(CSE148_TOOLS)/OpenSTA/app/sta timing.sta

clean:
	rm -rf build
	rm -f *.log
	rm -f *.json
	rm -f *.v
	rm -f configs.mk
	rm -f synthesis.ys
	rm -f timing.sta