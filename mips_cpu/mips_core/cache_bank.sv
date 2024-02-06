/*
 * cache_bank.sv
 * Author: Zinsser Zhang
 * Last Revision: 03/13/2022
 *
 * TODO update doc
 *
 * cache_bank provides a storage with one synchronous read and one synchronous
 * write port. When reading and writing to the same address, new data is
 * presented at the read port.
 *
 * cache_bank_core is a hint to Quartus's compiler to synthesis it to block ram.
 * Block rams in FPGA only support synchronous read and write (old data is
 * presented at the read port). So we need extra logic in cache_bank to forward
 * the new data.
 *
 * See wiki page "Synchronous Caches" for details.
 */
module cache_bank #(parameter DATA_WIDTH = 32, parameter ADDR_WIDTH = 4)(
	input clk,	// Clock
	input i_we,	// Write enable
	input logic [DATA_WIDTH - 1 : 0] i_wdata,			// Write data
	input logic [ADDR_WIDTH - 1 : 0] i_waddr, i_raddr,	// Write/read address
	output logic [DATA_WIDTH - 1 : 0] o_rdata			// Read data
);

	// A register to store new_data
	logic [DATA_WIDTH - 1 : 0] new_data;

	// The registered output of cache_bank_core
	logic [DATA_WIDTH - 1 : 0] old_data;

	// A flag to determine whether the last cycle write data (new_data) or
	// the read output (old_data) should be presented at the output port.
	logic new_data_flag;

	generate
		if (ADDR_WIDTH < 4) begin
			// OpenRAM requires at least 16 rows
			// Use register based structure for shallow RAMs
			cache_bank_core #(DATA_WIDTH, ADDR_WIDTH)
				BANK_CORE (
					.clk, .i_we, .i_waddr, .i_raddr, .i_wdata,
					.o_rdata(old_data)
			);
		end else begin
			sram #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) BANK_CORE (
				// Port 0: W
				.clk0(clk), .csb0(~i_we), .addr0(i_waddr), .din0(i_wdata),
				// Port 1: R
				.clk1(clk), .csb1('0), .addr1(i_raddr), .dout1(old_data)
			);
		end
	endgenerate

	assign o_rdata = new_data_flag ? new_data : old_data;

	always_ff @(posedge clk)
	begin
		new_data <= i_wdata;
		new_data_flag <= i_we & (i_raddr == i_waddr);
	end
endmodule

module cache_bank_core #(parameter DATA_WIDTH = 32, parameter ADDR_WIDTH = 4)(
	input clk,	// Clock
	input i_we,	// Write enable
	input logic [DATA_WIDTH - 1 : 0] i_wdata,			// Write data
	input logic [ADDR_WIDTH - 1 : 0] i_waddr, i_raddr,	// Write/read address
	output logic [DATA_WIDTH - 1 : 0] o_rdata			// Read data
);
	localparam DEPTH = 1 << ADDR_WIDTH;

	logic [DATA_WIDTH - 1 : 0] data [DEPTH];

	always_ff @(posedge clk)
	begin
		o_rdata <= data[i_raddr];
		if (i_we)
			data[i_waddr] <= i_wdata;
	end
endmodule

`ifdef SIMULATION
// OpenRAM SRAM model
module sram(
// Port 0: W
		clk0,csb0,addr0,din0,
// Port 1: R
		clk1,csb1,addr1,dout1
	);

	parameter DATA_WIDTH = 32 ;
	parameter ADDR_WIDTH = 4 ;
	parameter RAM_DEPTH = 1 << ADDR_WIDTH;
	// FIXME: This delay is arbitrary.
	// parameter DELAY = 3 ;

	input  clk0; // clock
	input   csb0; // active low chip select
	input [ADDR_WIDTH-1:0]  addr0;
	input [DATA_WIDTH-1:0]  din0;
	input  clk1; // clock
	input   csb1; // active low chip select
	input [ADDR_WIDTH-1:0]  addr1;
	output [DATA_WIDTH-1:0] dout1;

	reg  csb0_reg;
	reg [ADDR_WIDTH-1:0]  addr0_reg;
	reg [DATA_WIDTH-1:0]  din0_reg;

	// All inputs are registers
	always @(posedge clk0)
	begin
		csb0_reg = csb0;
		addr0_reg = addr0;
		din0_reg = din0;
		// if ( !csb0_reg )
		// 	$display($time," Writing %m addr0=%b din0=%b",addr0_reg,din0_reg);
	end

	reg  csb1_reg;
	reg [ADDR_WIDTH-1:0]  addr1_reg;
	reg [DATA_WIDTH-1:0]  dout1;

	// All inputs are registers
	always @(posedge clk1)
	begin
		csb1_reg = csb1;
		addr1_reg = addr1;
		// if (!csb0 && !csb1 && (addr0 == addr1))
		// 		 $display($time," WARNING: Writing and reading addr0=%b and addr1=%b simultaneously!",addr0,addr1);
		// dout1 = 32'bx;
		// if ( !csb1_reg )
		// 	$display($time," Reading %m addr1=%b dout1=%b",addr1_reg,mem[addr1_reg]);
	end

reg [DATA_WIDTH-1:0]    mem [0:RAM_DEPTH-1];

	// Memory Write Block Port 0
	// Write Operation : When web0 = 0, csb0 = 0
	always @ (negedge clk0)
	begin : MEM_WRITE0
		if (!csb0_reg)
				mem[addr0_reg] = din0_reg;
	end

	// Memory Read Block Port 1
	// Read Operation : When web1 = 1, csb1 = 0
	always @ (negedge clk1)
	begin : MEM_READ1
		if (!csb1_reg)
			 dout1 <= /* #(DELAY) */ mem[addr1_reg];
	end

endmodule
`endif
