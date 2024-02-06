/* mips_core.sv
* Author: Pravin P. Prabhu, Dean Tullsen, and Zinsser Zhang
* Last Revision: 03/13/2022
* Abstract:
*   The core module for the MIPS32 processor. This is a classic 5-stage
* MIPS pipeline architecture which is intended to follow heavily from the model
* presented in Hennessy and Patterson's Computer Organization and Design.
* All addresses used in this scope are byte addresses (26-bit)
*/
`include "mips_core.svh"

`ifdef SIMULATION
import "DPI-C" function void pc_event (input int pc);
import "DPI-C" function void wb_event (input int addr, input int data);
import "DPI-C" function void ls_event (input int op, input int addr, input int data);
`endif

module mips_core (
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low
	output done,  // Execution is done

	// AXI interfaces
	input AWREADY,
	output AWVALID,
	output [3:0] AWID,
	output [3:0] AWLEN,
	output [`ADDR_WIDTH - 1 : 0] AWADDR,

	input WREADY,
	output WVALID,
	output WLAST,
	output [3:0] WID,
	output [`DATA_WIDTH - 1 : 0] WDATA,

	output BREADY,
	input BVALID,
	input [3:0] BID,

	input ARREADY,
	output ARVALID,
	output [3:0] ARID,
	output [3:0] ARLEN,
	output [`ADDR_WIDTH - 1 : 0] ARADDR,

	output RREADY,
	input RVALID,
	input RLAST,
	input [3:0] RID,
	input [`DATA_WIDTH - 1 : 0] RDATA
);

	// Interfaces
	// |||| IF Stage
	pc_ifc if_pc_current();
	pc_ifc if_pc_next();
	cache_output_ifc if_i_cache_output();
	// ==== IF to DEC
	pc_ifc i2d_pc();
	cache_output_ifc i2d_inst();

	// |||| DEC Stage
	decoder_output_ifc dec_decoder_output();
	reg_file_output_ifc dec_reg_file_output();
	reg_file_output_ifc dec_shadow_reg_file_output();
	reg_file_output_ifc dec_forward_unit_output();
	branch_decoded_ifc dec_branch_decoded();
	alu_input_ifc dec_alu_input();
	alu_pass_through_ifc dec_alu_pass_through();
	logic dec_load_output;

	// ==== DEC to EX
	pc_ifc d2e_pc();
	alu_input_ifc d2e_alu_input();
	alu_pass_through_ifc d2e_alu_pass_through();
	logic ex_load_input;

	// |||| EX Stage
	alu_output_ifc ex_alu_output();
	branch_result_ifc ex_branch_result();
	d_cache_input_ifc ex_d_cache_input();
	d_cache_pass_through_ifc ex_d_cache_pass_through();

	// ==== EX to MEM
	pc_ifc e2m_pc();
	d_cache_input_ifc e2m_d_cache_input();
	d_cache_pass_through_ifc e2m_d_cache_pass_through();

	// |||| MEM Stage
	cache_output_ifc mem_d_cache_output();
	logic mem_done;
	write_back_ifc mem_write_back();
	logic mem_load_input;

	// ==== MEM to WB
	write_back_ifc m2w_write_back();

	// xxxx Hazard control
	logic lw_hazard;
	hazard_control_ifc i2i_hc();
	hazard_control_ifc i2d_hc();
	hazard_control_ifc d2e_hc();
	hazard_control_ifc e2m_hc();
	hazard_control_ifc m2w_hc();
	load_pc_ifc load_pc();
	logic mem_load_output;
	logic runahead_mode;
	logic state_controller;
	logic state_ready;
	logic run; 
	logic run_done_output;
	logic runahead_done;
	logic run_if_i;
	logic run_dec_i;
	logic run_mode_if_pass_input;
	logic run_mode_dec_pass_input;
	logic run_mode_exe_pass_input;
	logic run_mode_mem_pass_input;
	logic [`DATA_WIDTH - 1 : 0] pc_checkpoint;	// Recover pc 
	logic exit_runahead;
	// xxxx Memory
	axi_write_address axi_write_address();
	axi_write_data axi_write_data();
	axi_write_response axi_write_response();
	axi_read_address axi_read_address();
	axi_read_data axi_read_data();

	axi_write_address mem_write_address[1]();
	axi_write_data mem_write_data[1]();
	axi_write_response mem_write_response[1]();
	axi_read_address mem_read_address[2]();
	axi_read_data mem_read_data[2]();


	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// |||| IF Stage
	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	fetch_unit FETCH_UNIT(
		.clk, .rst_n,

		.i_hc         (i2i_hc),
		.i_load_pc    (load_pc),
		.o_pc_current (if_pc_current),
		.run_i (runahead_mode),
		.run_o(run_if_i),
		.o_pc_next    (if_pc_next)
	);

	i_cache I_CACHE(
		.clk, .rst_n,

		.mem_read_address(mem_read_address[0]),
		.mem_read_data   (mem_read_data[0]),

		.i_pc_current (if_pc_current),
		.i_pc_next    (if_pc_next),

		.out          (if_i_cache_output)
	);
	// If you want to change the line size and total size of instruction cache,
	// uncomment the following two lines and change the parameter.

	// defparam D_CACHE.INDEX_WIDTH = 9,
	// 	D_CACHE.BLOCK_OFFSET_WIDTH = 2;

	// ========================================================================
	// ==== IF to DEC
	// ========================================================================
	pr_i2d PR_I2D(
		.clk, .rst_n,
		.i_hc(i2d_hc),
		.i_pc   (if_pc_current),     .o_pc   (i2d_pc),
		.run_i (run_if_i),
		.run_o (run_dec_i),
		.i_inst (if_i_cache_output), .o_inst (i2d_inst)
	);

	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// |||| DEC Stage
	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	decoder DECODER(
		.i_pc(i2d_pc),
		.i_inst(i2d_inst),
		.runahead_mode(run_dec_i),
		.out(dec_decoder_output),
		.dec_load_output
	);

	reg_file REG_FILE(
		.clk,
		.i_decoded(dec_decoder_output),
		.runahead_done,
		.i_wb(m2w_write_back), // WB stage
		.runahead_mode(run_dec_i),
		.out(dec_reg_file_output)
	);
	
	forward_unit FORWARD_UNIT(
		.decoded     (dec_decoder_output),
		.reg_data    (dec_reg_file_output),

		.ex_ctl      (d2e_alu_pass_through),
		.ex_data     (ex_alu_output),
		.mem         (mem_write_back),
		.wb          (m2w_write_back),
		.out         (dec_forward_unit_output),
		.o_lw_hazard (lw_hazard)
	);

	decode_stage_glue DEC_STAGE_GLUE(
		.i_decoded          (dec_decoder_output),
		.i_reg_data         (dec_forward_unit_output),


		.branch_decoded     (dec_branch_decoded),

		.o_alu_input        (dec_alu_input),
		.o_alu_pass_through (dec_alu_pass_through)
	);

	// ========================================================================
	// ==== DEC to EX
	// ========================================================================
	pr_d2e PR_D2E(
		.clk, .rst_n,
		.i_hc(d2e_hc),
		.i_dec				(dec_load_output),	// pass over if find load instruction, else 0
		.i_pc(i2d_pc), .o_pc(d2e_pc),
		.i_alu_input        (dec_alu_input),
		.o_alu_input        (d2e_alu_input),
		.i_alu_pass_through (dec_alu_pass_through),
		.o_alu_pass_through (d2e_alu_pass_through),
		.o_dec				(ex_load_input)		// pass through to exe
	);

	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// |||| EX Stage
	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	alu ALU(
		.in(d2e_alu_input),
		.out(ex_alu_output),
		.done
	);

	ex_stage_glue EX_STAGE_GLUE (
		.i_alu_output           (ex_alu_output),
		.i_alu_pass_through     (d2e_alu_pass_through),

		.o_branch_result        (ex_branch_result),
		.o_d_cache_input        (ex_d_cache_input),
		.o_d_cache_pass_through (ex_d_cache_pass_through)
	);

	// ========================================================================
	// ==== EX to MEM
	// ========================================================================
	pr_e2m PR_E2M (
		.clk, .rst_n,
		.i_hc(e2m_hc),
		.i_exe(ex_load_input),
		.i_pc(d2e_pc), .o_pc(e2m_pc),
		.i_d_cache_input       (ex_d_cache_input),
		.i_d_cache_pass_through(ex_d_cache_pass_through),
		.o_d_cache_input       (e2m_d_cache_input),
		.o_d_cache_pass_through(e2m_d_cache_pass_through),
		.o_exe(mem_load_input)
	);

	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// |||| MEM Stage
	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	d_cache D_CACHE (
		.clk, .rst_n,
		.in(e2m_d_cache_input),
		.out(mem_d_cache_output),
		.exit_runahead,
		.mem_read_address(mem_read_address[1]),
		.mem_read_data   (mem_read_data[1]),

		.mem_write_address(mem_write_address[0]),
		.mem_write_data(mem_write_data[0]),
		.mem_write_response(mem_write_response[0])
	);
	// If you want to change the line size and total size of data cache,
	// uncomment the following two lines and change the parameter.

	// defparam D_CACHE.INDEX_WIDTH = 9,
	// 	D_CACHE.BLOCK_OFFSET_WIDTH = 2;

	mem_stage_glue MEM_STAGE_GLUE (
		.i_d_cache_output      (mem_d_cache_output),
		.i_d_cache_pass_through(e2m_d_cache_pass_through),
		.i_mem				   (mem_load_input),
		.o_done                (mem_done),
		.o_write_back          (mem_write_back),
		.o_mem				   (mem_load_output)
	);

	// ========================================================================
	// ==== MEM to WB
	// ========================================================================
	pr_m2w PR_M2W (
		.clk, .rst_n,
		.i_hc (m2w_hc),
		.i_wb (mem_write_back),
		.o_wb (m2w_write_back)
	);

	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// |||| WB Stage
	// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// NO LOGIC

	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	// xxxx Hazard Controller
	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	hazard_controller HAZARD_CONTROLLER (
		.clk, .rst_n,

		.if_i_cache_output,
		.dec_pc(i2d_pc),
		.dec_branch_decoded,
		.ex_pc(d2e_pc),
		.lw_hazard,
		.ex_branch_result,
		.mem_done,
		.i_pc(e2m_pc),
		.i_mem(mem_load_output),
		.axi_read_address,
		.axi_read_data,
		.delay1,
		.exit_runahead,
		.run_dec_i,

		.i2i_hc,
		.i2d_hc,
		.d2e_hc,
		.e2m_hc,
		.m2w_hc,
		.load_pc,
		.runahead_mode,
		.pc_checkpoint,
		.runahead_done
	);

	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	// xxxx Memory Arbiter
	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	memory_arbiter #(.WRITE_MASTERS(1), .READ_MASTERS(2)) MEMORY_ARBITER (
		.clk, .rst_n,
		.axi_write_address,
		.axi_write_data,
		.axi_write_response,
		.axi_read_address,
		.axi_read_data,

		.mem_write_address,
		.mem_write_data,
		.mem_write_response,
		.mem_read_address,
		.mem_read_data
	);

	assign axi_write_address.AWREADY = AWREADY;
	assign AWVALID = axi_write_address.AWVALID;
	assign AWID = axi_write_address.AWID;
	assign AWLEN = axi_write_address.AWLEN;
	assign AWADDR = axi_write_address.AWADDR;

	assign axi_write_data.WREADY = WREADY;
	assign WVALID = axi_write_data.WVALID;
	assign WLAST = axi_write_data.WLAST;
	assign WID = axi_write_data.WID;
	assign WDATA = axi_write_data.WDATA;

	assign axi_write_response.BVALID = BVALID;
	assign axi_write_response.BID = BID;
	assign BREADY = axi_write_response.BREADY;

	assign axi_read_address.ARREADY = ARREADY;
	assign ARVALID = axi_read_address.ARVALID;
	assign ARID = axi_read_address.ARID;
	assign ARLEN = axi_read_address.ARLEN;
	assign ARADDR = axi_read_address.ARADDR;

	assign RREADY = axi_read_data.RREADY;
	assign axi_read_data.RVALID = RVALID;
	assign axi_read_data.RLAST = RLAST;
	assign axi_read_data.RID = RID;
	assign axi_read_data.RDATA = RDATA;

	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	// xxxx Debug and statistic collect logic (Not synthesizable)
	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	logic delay1;
	logic delay2;
	logic delay3;
`ifdef SIMULATION
	always_ff @(posedge clk)
	begin
		/*
			* If an instruction goes into d2e pipeline register and is not a
			* nop, we count it as an instruction we executed.
			*/
		
		if(~rst_n)
		begin
			delay1<='0;
			delay2<='0;
		end
		/*
		if(runahead_done)
			delay1 <= runahead_done;
		else if(delay1)
		begin
			delay2 <= delay1;
			delay1 <= runahead_done;
		end
		else if(delay2)
		begin
			delay2 <= delay1;
		end
		*/
		delay1 <= run_dec_i;
		delay2 <= delay1;
		/*
		if (!i2d_hc.stall
			&& !d2e_hc.flush
			&& dec_decoder_output.valid
			&& i2d_inst.data)
		begin
			if(runahead_mode == '0 && run_mode_dec_pass_input == '0 && run_mode_if_pass_input == '0 && (pc_checkpoint != i2d_pc.pc || pc_checkpoint == '0))
			begin
				pc_event(i2d_pc.pc);
			end
		end
		*/
		if (!i2d_hc.stall
			&& !d2e_hc.flush
			&& dec_decoder_output.valid
			&& i2d_inst.data)
		begin
			if(runahead_mode == '0 && run_dec_i == '0)
			begin
				//pc_event(i2d_pc.pc);
			end
		end
		if (m2w_write_back.uses_rw)
		begin 
			if(runahead_mode == '0 && run_dec_i == '0)
			begin
				wb_event(m2w_write_back.rw_addr, m2w_write_back.rw_data);
			end
		end
		
		if (!e2m_hc.stall
			&& !m2w_hc.flush
			&& mem_d_cache_output.valid)
		begin
			if (e2m_d_cache_input.mem_action == READ)
			begin
				if(runahead_mode == '0 && run_dec_i == '0)
				ls_event(e2m_d_cache_input.mem_action, e2m_d_cache_input.addr, mem_d_cache_output.data);
			end
			else
			begin
				if(runahead_mode == '0 && run_dec_i == '0)
				ls_event(e2m_d_cache_input.mem_action, e2m_d_cache_input.addr, e2m_d_cache_input.data);
			end
		end
		
	end
`endif
endmodule
