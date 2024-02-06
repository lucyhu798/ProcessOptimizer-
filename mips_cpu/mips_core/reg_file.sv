/*
 * reg_file.sv
 * Author: Zinsser Zhang
 * Last Revision: 04/09/2018
 *
 * A 32-bit wide, 32-word deep register file with two asynchronous read port
 * and one synchronous write port.
 *
 * Register file needs to output '0 if uses_r* signal is low. In this case,
 * either reg zero is requested for read or the register is unused.
 *
 * See wiki page "Branch and Jump" for details.
 */
`include "mips_core.svh"

interface reg_file_output_ifc ();
	logic [`DATA_WIDTH - 1 : 0] rs_data;
	logic [`DATA_WIDTH - 1 : 0] rt_data;
	modport in  (input rs_data, rt_data);
	modport out (output rs_data, rt_data);
endinterface

module reg_file (
	input clk,    // Clock
	//input logic i_done,
	// Input from decoder
	decoder_output_ifc.in i_decoded,
	// Input from write back stage
	write_back_ifc.in i_wb,
	input logic runahead_mode,
	input logic runahead_done,
	// Output data
	reg_file_output_ifc.out out
);

	logic [`DATA_WIDTH - 1 : 0] regs [32];
	logic [`DATA_WIDTH - 1 : 0] shadow_regs [32];
	always_comb
	begin
		if(runahead_mode)
		begin
			out.rs_data = i_decoded.uses_rs ? shadow_regs[i_decoded.rs_addr] : '0;
			out.rt_data = i_decoded.uses_rt ? shadow_regs[i_decoded.rt_addr] : '0;
		end
		else
		begin
			out.rs_data = i_decoded.uses_rs ? regs[i_decoded.rs_addr] : '0;
			out.rt_data = i_decoded.uses_rt ? regs[i_decoded.rt_addr] : '0;
		end
	end
	always_ff @(posedge clk) begin
		if(i_wb.uses_rw && ~runahead_mode)
		begin
			regs[i_wb.rw_addr] <= i_wb.rw_data;
		end
		if(i_wb.uses_rw)
		begin
			shadow_regs[i_wb.rw_addr] <= i_wb.rw_data;
		end
		if(runahead_done)
		begin
			for (int i = 0; i < 32; i++) begin
				shadow_regs[i] <= regs[i];
			end
		end
	end

endmodule

