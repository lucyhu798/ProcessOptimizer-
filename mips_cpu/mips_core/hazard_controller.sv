/*
 * hazard_controller.sv
 * Author: Zinsser Zhang
 * Last Revision: 03/13/2022
 *
 * hazard_controller collects feedbacks from each stage and detect whether there
 * are hazards in the pipeline. If so, it generate control signals to stall or
 * flush each stage. It also contains a branch_controller, which talks to
 * a branch predictor to make a prediction when a branch instruction is decoded.
 *
 * It also contains simulation only logic to report hazard conditions to C++
 * code for execution statistics collection.
 *
 * See wiki page "Hazards" for details.
 * See wiki page "Branch and Jump" for details of branch and jump instructions.
 */
`include "mips_core.svh"

`ifdef SIMULATION
import "DPI-C" function void stats_event (input string e);
`endif

module hazard_controller (
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// Feedback from IF
	cache_output_ifc.in if_i_cache_output,
	// Feedback from DEC
	pc_ifc.in dec_pc,
	branch_decoded_ifc.hazard dec_branch_decoded,
	// Feedback from EX
	pc_ifc.in ex_pc,
	pc_ifc.in i_pc,
	input lw_hazard,
	branch_result_ifc.in ex_branch_result,
	// Feedback from MEM

	input mem_done,
	axi_read_address.master axi_read_address,
	axi_read_data.master axi_read_data,
	input logic i_mem,
	input logic exit_runahead,
	input logic run_dec_i,
	
	// Hazard control output
	hazard_control_ifc.out i2i_hc,
	hazard_control_ifc.out i2d_hc,
	hazard_control_ifc.out d2e_hc,
	hazard_control_ifc.out e2m_hc,
	hazard_control_ifc.out m2w_hc,

	// Load pc output
	load_pc_ifc.out load_pc,
	output logic [`DATA_WIDTH - 1 : 0] pc_checkpoint,	// Recover pc 
	output logic runahead_mode,
	input logic delay1,
	output logic runahead_done
);

	branch_controller BRANCH_CONTROLLER (
		.clk, .rst_n,
		.dec_pc,
		.dec_branch_decoded,
		.ex_pc,
		.ex_branch_result
	);

	// We have total 6 potential hazards
	logic ic_miss;			// I cache miss
	logic ds_miss;			// Delay slot miss
	logic dec_overload;		// Branch predict taken or Jump
	logic ex_overload;		// Branch prediction wrong
	//    lw_hazard;		// Load word hazard (input from forward unit)
	logic dc_miss;			// D cache miss
	
	logic valid;
	// Determine if we have these hazards
	always_comb
	begin
		ic_miss = ~if_i_cache_output.valid;
		ds_miss = ic_miss & dec_branch_decoded.valid;
		dec_overload = dec_branch_decoded.valid
			& (dec_branch_decoded.is_jump
				| (dec_branch_decoded.prediction == TAKEN));
		ex_overload = ex_branch_result.valid &
			& (ex_branch_result.prediction != ex_branch_result.outcome);	
		// lw_hazard is determined by forward unit.
		dc_miss = ~mem_done;
		//recover = mem_done && ~valid;	// mem_done is valid and is invalid, you want to recover
	end
	// Control signals
	logic if_stall, if_flush;
	logic dec_stall, dec_flush;
	logic ex_stall, ex_flush;
	logic mem_stall, mem_flush;
	// wb doesn't need to be stalled or flushed
	// i.e. any data goes to wb is finalized and waiting to be commited

	/*
	 * Now let's go over the solution of all hazards
	 * ic_miss:
	 *     if_stall, if_flush
	 * ds_miss:
	 *     dec_stall, dec_flush (if_stall and if_flush handled by ic_miss)
	 * dec_overload:
	 *     load_pc
	 * ex_overload:
	 *     load_pc, ~if_stall, if_flush
	 * lw_hazard:
	 *     dec_stall, dec_flush
	 * dc_miss:
	 *     mem_stall, mem_flush
	 *
	 * The only conflict here is between ic_miss and ex_overload.
	 * ex_overload should have higher priority than ic_miss. Because i cache
	 * does not register missed request, it's totally fine to directly overload
	 * the pc value.
	 *
	 * In addition to above hazards, each stage should also stall if its
	 * downstream stage stalls (e.g., when mem stalls, if & dec & ex should all
	 * stall). This has the highest priority.
	 */

	always_comb
	begin : handle_if
		if_stall = 1'b0;
		if_flush = 1'b0;
		
		if (ic_miss)
		begin
			if_stall = 1'b1;
			if_flush = 1'b1;
		end

		if (ex_overload)
		begin
			if_stall = 1'b0;
			if_flush = 1'b1;
		end

		if (dec_stall)
			if_stall = 1'b1;
		
		if(runahead_done || (exit_runahead == '1 && runahead_mode == '1))
		begin
			if_flush = 1'b1;
			if_stall = 1'b0;
		end
			
	end

	always_comb
	begin : handle_dec
		dec_stall = 1'b0;
		dec_flush = 1'b0;

		if (ds_miss | lw_hazard)
		begin
			dec_stall = 1'b1;
			dec_flush = 1'b1;
		end

		if (ex_stall)
			dec_stall = 1'b1;

		if(runahead_done || (exit_runahead == '1 && runahead_mode == '1))
		begin
			dec_flush = 1'b1;
			dec_stall = 1'b0;
		end

	end

	always_comb
	begin : handle_ex
		ex_stall = mem_stall;
		ex_flush = 1'b0;

		if(runahead_done || (exit_runahead == '1 && runahead_mode == '1))
		begin
			ex_flush = 1'b1;
			ex_stall = 1'b0;
		end
	end

	always_comb
	begin : handle_mem
		mem_stall = dc_miss;
		mem_flush = dc_miss;
		if(runahead_mode)
		begin
			mem_stall = '0;
			mem_flush = '0;
		end
		
		if(runahead_done || (exit_runahead == '1 && runahead_mode == '1))
		begin
			mem_flush = 1'b1;
			mem_stall = '0;
		end
		
	end

	// Now distribute the control signals to each pipeline registers
	always_comb
	begin
		i2i_hc.stall = 1'b0;
		i2i_hc.stall = if_stall;
		i2d_hc.flush = if_flush;
		i2d_hc.stall = dec_stall;
		d2e_hc.flush = dec_flush;
		d2e_hc.stall = ex_stall;
		e2m_hc.flush = ex_flush;
		e2m_hc.stall = mem_stall;
		m2w_hc.flush = mem_flush;
		m2w_hc.stall = 1'b0;
	end

	logic prev;
	// Derive the load_pc
	always_comb
	begin
		load_pc.we = dec_overload | ex_overload;
		if (dec_overload)
		begin
			load_pc.we = 1'b1;
			load_pc.new_pc = dec_branch_decoded.target;
		end
		//else if(ic_miss && run_mode_exe_pass_input == '1 && run_mode_dec_pass_input == '0)
		//begin
		//	load_pc.we = 1'b1;
		//	load_pc.new_pc = pc_checkpoint + + `ADDR_WIDTH'd4;
			//load_pc.new_pc = pc_checkpoint == runahead_address ? pc_checkpoint : runahead_address;
			//$display("checkpoint");
		//end
		//else if(state_ready && runahead_mode == 1'b1 && ~axi_read_data.RVALID && axi_read_data.RID == '0 && ~ic_miss)
		else if(runahead_done)
		begin
			load_pc.we = 1'b1;
			load_pc.new_pc = pc_checkpoint;
			//load_pc.new_pc = pc_checkpoint == runahead_address ? pc_checkpoint : runahead_address;
			//$display("checkpoint");
		end
		else
			load_pc.new_pc = ex_branch_result.recovery_target;
	end	
	always_ff @(posedge clk)
	begin
		if(!rst_n)
		begin
			runahead_mode <= 1'b0;
			runahead_done <= '0;
			pc_checkpoint <= 1'b1;
		end
		else if(i_mem && ~mem_done && runahead_mode == 1'b0)	// When there is a mem miss and entering runahead
		begin
			runahead_mode <= 1'b1;
			pc_checkpoint <= i_pc.pc;
			runahead_done <= '0;
			//$display("Enter runahead, pc: %h", i_pc.pc);
		end
		else if(runahead_mode == 1'b1 && exit_runahead == '1)
		begin
			runahead_done <= runahead_mode;
			runahead_mode <= '0;
			//$display("Exit runahead");
		end
		else
		begin
			runahead_done <= '0;
		end
	end


`ifdef SIMULATION
	always_ff @(posedge clk)
	begin
		if (ic_miss) stats_event("ic_miss");
		if (ds_miss) stats_event("ds_miss");
		if (dec_overload) stats_event("dec_overload");
		if (ex_overload) stats_event("ex_overload");
		if (lw_hazard) stats_event("lw_hazard");
		if (dc_miss) stats_event("dc_miss");
		if (if_stall) stats_event("if_stall");
		if (if_flush) stats_event("if_flush");
		if (dec_stall) stats_event("dec_stall");
		if (dec_flush) stats_event("dec_flush");
		if (ex_stall) stats_event("ex_stall");
		if (ex_flush) stats_event("ex_flush");
		if (mem_stall) stats_event("mem_stall");
		if (mem_flush) stats_event("mem_flush");
		//if(i_mem) stats_event("loads");
	end
`endif

endmodule
