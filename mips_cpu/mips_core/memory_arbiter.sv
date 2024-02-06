/*
 * memory_arbiter.sv
 * Author: Zinsser Zhang
 * Last Revision: 03/13/2022
 *
 * This is a priority arbiter to merge/split multiple axi master interfaces
 * to a single axi interface to external memory.
 *
 * Thread id, or AWID/WID/BID/ARID/RID fields in axi protocol should match
 * the interface offset in the input/output interface array
 * Baseline configuration:
 *   Read masters x2:
 *     id=0 i_cache read => mem_read_*[0]
 *     id=1 d_cache read => mem_read_*[1]
 *   Write masters x1:
 *     id=0 d_cache read => mem_write_*[0]
 *
 * For egress interfaces, priority is given to the master with lowest id
 * E.g., in baseline configuration, if both i_cache and d_cache request to read
 * on the same cycle, i_cache will send its request out first.
 *
 * Note that this does not guarantee i_cache read response to come back before
 * d_cache read response. AXI protocol only guarantees ordered responses for
 * the same thread id. It is ok for the memory to response to different threads
 * out of order.
 *
 * Typical line sizes are from 2 words to 8 words. The memory interfaces only
 * support up to 8 words line size.
 *
 * For ingress interfaces, it simply flop the packet and assert the valid signal
 * for the corresponding master.
 *
 * The arbiter is adding 1 cycle latency to both egress and ingress interfaces.
 */

`include "mips_core.svh"

module memory_arbiter #(
		parameter WRITE_MASTERS = 1,
		parameter READ_MASTERS = 2
	)(
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// External AXI interfaces
	axi_write_address.master axi_write_address,
	axi_write_data.master axi_write_data,
	axi_write_response.master axi_write_response,
	axi_read_address.master axi_read_address,
	axi_read_data.master axi_read_data,

	// Masters
	axi_write_address.slave mem_write_address[WRITE_MASTERS],
	axi_write_data.slave mem_write_data[WRITE_MASTERS],
	axi_write_response.slave mem_write_response[WRITE_MASTERS],
	axi_read_address.slave mem_read_address[READ_MASTERS],
	axi_read_data.slave mem_read_data[READ_MASTERS]
	);

	genvar i;

	// ========================================================================
	// ==== write_address
	// ========================================================================

	logic write_address_entity_ready[WRITE_MASTERS];
	logic write_address_entity_valid[WRITE_MASTERS];
	logic [8+`ADDR_WIDTH-1:0] write_address_entity_payload[WRITE_MASTERS];

	generate
	for (i = 0; i < WRITE_MASTERS; i++) begin : write_address_connection
		always_comb begin
			mem_write_address[i].AWREADY = write_address_entity_ready[i];
			write_address_entity_valid[i] = mem_write_address[i].AWVALID;
			write_address_entity_payload[i] = {
				mem_write_address[i].AWID,
				mem_write_address[i].AWLEN,
				mem_write_address[i].AWADDR
			};
		end
	end
	endgenerate

	egress_priority_arbiter #(.COUNT(WRITE_MASTERS), .WIDTH(8+`ADDR_WIDTH)) write_address_arbiter (
		.clk, .rst_n,
		.ready(axi_write_address.AWREADY),
		.valid(axi_write_address.AWVALID),
		.payload({axi_write_address.AWID, axi_write_address.AWLEN, axi_write_address.AWADDR}),

		.entity_ready(write_address_entity_ready),
		.entity_valid(write_address_entity_valid),
		.entity_payload(write_address_entity_payload)
	);

	// ========================================================================
	// ==== write_data
	// ========================================================================

	logic write_data_entity_ready[WRITE_MASTERS];
	logic write_data_entity_valid[WRITE_MASTERS];
	logic [5+`DATA_WIDTH-1:0] write_data_entity_payload[WRITE_MASTERS];

	generate
	for (i = 0; i < WRITE_MASTERS; i++) begin : write_data_connection
		always_comb begin
			mem_write_data[i].WREADY = write_data_entity_ready[i];
			write_data_entity_valid[i] = mem_write_data[i].WVALID;
			write_data_entity_payload[i] = {
				mem_write_data[i].WID,
				mem_write_data[i].WLAST,
				mem_write_data[i].WDATA
			};
		end
	end
	endgenerate

	egress_priority_arbiter #(.COUNT(WRITE_MASTERS), .WIDTH(5+`DATA_WIDTH)) write_data_arbiter (
		.clk, .rst_n,
		.ready(axi_write_data.WREADY),
		.valid(axi_write_data.WVALID),
		.payload({axi_write_data.WID, axi_write_data.WLAST, axi_write_data.WDATA}),

		.entity_ready(write_data_entity_ready),
		.entity_valid(write_data_entity_valid),
		.entity_payload(write_data_entity_payload)
	);

	// ========================================================================
	// ==== write_response
	// ========================================================================

	logic write_response_entity_ready[WRITE_MASTERS];
	logic write_response_entity_valid[WRITE_MASTERS];
	logic [4-1:0] write_response_entity_payload[WRITE_MASTERS];

	generate
	for (i = 0; i < WRITE_MASTERS; i++) begin : write_response_connection
		always_comb begin
			mem_write_response[i].BVALID = write_response_entity_valid[i];
			write_response_entity_ready[i] = mem_write_response[i].BREADY;
			{
				mem_write_response[i].BID
			} = write_response_entity_payload[i];
		end
	end
	endgenerate

	ingress_splitter #(.COUNT(WRITE_MASTERS), .WIDTH(4)) write_response_splitter (
		.clk, .rst_n,
		.ready(axi_write_response.BREADY),
		.valid(axi_write_response.BVALID),
		.id(axi_write_response.BID),
		.payload({axi_write_response.BID}),

		.entity_ready(write_response_entity_ready),
		.entity_valid(write_response_entity_valid),
		.entity_payload(write_response_entity_payload)
	);

	// ========================================================================
	// ==== read_address
	// ========================================================================

	logic read_address_entity_ready[READ_MASTERS];
	logic read_address_entity_valid[READ_MASTERS];
	logic [8+`ADDR_WIDTH-1:0] read_address_entity_payload[READ_MASTERS];

	generate
	for (i = 0; i < READ_MASTERS; i++) begin : read_address_connection
		always_comb begin
			mem_read_address[i].ARREADY = read_address_entity_ready[i];
			read_address_entity_valid[i] = mem_read_address[i].ARVALID;
			read_address_entity_payload[i] = {
				mem_read_address[i].ARID,
				mem_read_address[i].ARLEN,
				mem_read_address[i].ARADDR
			};
		end
	end
	endgenerate

	egress_priority_arbiter #(.COUNT(READ_MASTERS), .WIDTH(8+`ADDR_WIDTH)) read_address_arbiter (
		.clk, .rst_n,
		.ready(axi_read_address.ARREADY),
		.valid(axi_read_address.ARVALID),
		.payload({axi_read_address.ARID, axi_read_address.ARLEN, axi_read_address.ARADDR}),

		.entity_ready(read_address_entity_ready),
		.entity_valid(read_address_entity_valid),
		.entity_payload(read_address_entity_payload)
	);

	// ========================================================================
	// ==== read_data
	// ========================================================================

	logic read_data_entity_ready[READ_MASTERS];
	logic read_data_entity_valid[READ_MASTERS];
	logic [5+`DATA_WIDTH-1:0] read_data_entity_payload[READ_MASTERS];

	generate
	for (i = 0; i < READ_MASTERS; i++) begin : read_data_connection
		always_comb begin
			mem_read_data[i].RVALID = read_data_entity_valid[i];
			read_data_entity_ready[i] = mem_read_data[i].RREADY;
			{
				mem_read_data[i].RID,
				mem_read_data[i].RLAST,
				mem_read_data[i].RDATA
			} = read_data_entity_payload[i];
		end
	end
	endgenerate

	ingress_splitter #(.COUNT(READ_MASTERS), .WIDTH(5+`DATA_WIDTH)) read_data_splitter (
		.clk, .rst_n,
		.ready(axi_read_data.RREADY),
		.valid(axi_read_data.RVALID),
		.id(axi_read_data.RID),
		.payload({axi_read_data.RID, axi_read_data.RLAST, axi_read_data.RDATA}),

		.entity_ready(read_data_entity_ready),
		.entity_valid(read_data_entity_valid),
		.entity_payload(read_data_entity_payload)
	);
endmodule

module egress_priority_arbiter #(
		parameter COUNT = 2,
		parameter WIDTH = 32
	)(
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// External interface
	input ready,
	output logic valid,
	output logic [WIDTH-1:0] payload,

	// Masters
	output logic entity_ready[COUNT],
	input entity_valid[COUNT],
	input [WIDTH-1:0] entity_payload[COUNT]
	);

	logic [WIDTH-1:0] merged_payload;
	logic merged_valid, merged_ready;
	integer gnt_id;

	always_comb begin
		merged_valid = 1'b0;
		merged_payload = '0;
		gnt_id = 0;
		for (int i = COUNT-1; i >=0; i--)
			if (entity_valid[i]) begin
				merged_valid = 1'b1;
				merged_payload = entity_payload[i];
				gnt_id = i;
			end
	end

	always_comb begin
		for (int i = COUNT-1; i >=0; i--)
			entity_ready[i] = 1'b0;
		entity_ready[gnt_id] = merged_ready;
	end

	always_comb begin
		merged_ready = ready || !valid;
	end

	always_ff @(posedge clk) begin
		if (~rst_n) begin
			valid <= 1'b0;
		end else begin
			valid <= merged_ready ? merged_valid : valid;
			payload <= (merged_ready && merged_valid) ? merged_payload : payload;
		end
	end
endmodule

module ingress_splitter #(
		parameter COUNT = 2,
		parameter WIDTH = 32
	)(
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low

	// External interface
	output logic ready,
	input valid,
	input [3:0] id,
	input [WIDTH-1:0] payload,

	// Masters
	input entity_ready[COUNT],
	output logic entity_valid[COUNT],
	output logic [WIDTH-1:0] entity_payload[COUNT]
	);

	logic [WIDTH-1:0] pipe_payload;
	logic pipe_valid, pipe_ready;
	logic [3:0] pipe_id;

	always_comb begin
		ready = !pipe_valid || entity_ready[pipe_id];
	end

	always_ff @(posedge clk) begin
		if (~rst_n) begin
			pipe_valid <= 1'b0;
		end else begin
			pipe_valid <= ready ? valid : pipe_valid;
			pipe_id <= (ready && valid) ? id : pipe_id;
			pipe_payload <= (ready && valid) ? payload : pipe_payload;
		end
	end

	always_comb begin
		for (int i = 0; i < COUNT; i++) begin
			entity_valid[i] = pipe_valid && (pipe_id == i);
			entity_payload[i] = pipe_payload;
		end
	end

endmodule