.PHONY: clean verilate simulate dump wave

verilate:
	bash -c "source ~/oss-cad-suite/environment && verilator --cc --exe --build --trace-fst -DSIMULATION -Imips_core -f verilator_files --top-module mips_core verilator_main.cpp memory.cpp memory_driver.cpp -Wno-fatal"

simulate:
	obj_dir/Vmips_core

dump:
	obj_dir/Vmips_core -d

wave:
	bash -c "source ~/oss-cad-suite/environment && gtkwave simx.fst"

clean:
	rm -rf obj_dir/
	rm -f *.txt