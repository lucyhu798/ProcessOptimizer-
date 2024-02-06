import json
import os

liberty = f"{os.environ['CSE148_TOOLS']}/FreePDK45/osu_soc/lib/files/gscl45nm.lib"


def parse_hierarchy(file_name):
    with open(file_name) as f:
        h = json.load(f)

    configs = set()
    for module_name, module in h["modules"].items():
        for cell in module["cells"].values():
            if cell["type"] == "sram":
                addr_width = int(cell["parameters"]["ADDR_WIDTH"], 2)
                data_width = int(cell["parameters"]["DATA_WIDTH"], 2)
                depth = 1 << addr_width
                print(f"Found sram {data_width}x{depth} in {module_name}")
                configs.add((addr_width, data_width, depth))
    return configs

def generate_makefile(configs):
    with open("configs.mk", "w") as f:
        f.write("configs=")
        f.write(" ".join(f"{w}_{h}" for _, w, h in configs))

def generate_sram_mapping(configs):
    with open("sram.v", "w") as f:
        f.write(f"""

module sram(
// Port 0: W
        clk0,csb0,addr0,din0,
// Port 1: R
        clk1,csb1,addr1,dout1
    );

    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 4;

    input  clk0, csb0;
    input  [ADDR_WIDTH-1:0] addr0;
    input  [DATA_WIDTH-1:0] din0;
    input  clk1, csb1;
    input  [ADDR_WIDTH-1:0] addr1;
    output [DATA_WIDTH-1:0] dout1;

    generate
""")
        for a, w, h in configs:
            f.write(f"""
        if (ADDR_WIDTH == {a} && DATA_WIDTH == {w})
            sram_{w}_{h}_freepdk45_analytical core (.*);
""")

        f.write("""
    endgenerate
endmodule

""")

def generate_yosys_script(configs):
    with open("synthesis.ys", "w") as f:
        f.write(f"read_liberty -lib {liberty}\n")
        for _, w, h in configs:
            f.write(f"read_liberty -lib build/sram_{w}_{h}_freepdk45_analytical_TT_1p0V_25C.lib\n")
        f.write(f"""
read_verilog mips_core.v
read_verilog -sv sram.v
synth -top mips_core -flatten
dfflibmap -liberty {liberty}
abc -liberty {liberty} -constr mips_core.constr -D 5000
opt_clean

write_verilog -attr2comment netlist.v
stat -liberty {liberty}""")
        for _, w, h in configs:
            f.write(f" -liberty build/sram_{w}_{h}_freepdk45_analytical_TT_1p0V_25C.lib")

def generate_sta_script(configs):
    with open("timing.sta", "w") as f:
        f.write(f"read_liberty {liberty}\n")
        for _, w, h in configs:
            f.write(f"read_liberty build/sram_{w}_{h}_freepdk45_analytical_TT_1p0V_25C.lib\n")
        f.write(f"""
read_verilog netlist.v
link_design mips_core
read_sdc mips_core.sdc
report_checks
report_clock_min_period -include_port_paths
exit
""")

if __name__ == "__main__":
    configs = parse_hierarchy("hierarchy.json")

    generate_makefile(configs)
    generate_sram_mapping(configs)
    generate_yosys_script(configs)
    generate_sta_script(configs)