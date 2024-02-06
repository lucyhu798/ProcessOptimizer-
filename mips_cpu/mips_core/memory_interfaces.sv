/*
 * memory_interfaces.sv
 * Author: Zinsser Zhang
 * Last Revision: 03/13/2022
 *
 * This file defines the memory interface for reading from and writing to DRAM.
 * Detailed documentation can be found in the wiki page "Memory Interfaces"
 */

`include "mips_core.svh"

interface axi_write_address ();
	logic AWVALID, AWREADY;
	logic [3:0] AWID;
	logic [3:0] AWLEN;
	logic [`ADDR_WIDTH - 1 : 0] AWADDR;

	modport master (input AWREADY, output AWVALID, AWID, AWLEN, AWADDR);
	modport slave (output AWREADY, input AWVALID, AWID, AWLEN, AWADDR);
endinterface

interface axi_write_data ();
	logic WVALID, WREADY, WLAST;
	logic [3:0] WID;
	logic [`DATA_WIDTH - 1 : 0] WDATA;

	modport master (input WREADY, output WVALID, WLAST, WID, WDATA);
	modport slave (output WREADY, input WVALID, WLAST, WID, WDATA);
endinterface

interface axi_write_response ();
	logic BVALID, BREADY;
	logic [3:0] BID;

	modport master (input BVALID, BID, output BREADY);
	modport slave (output BVALID, BID, input BREADY);
endinterface

interface axi_read_address ();
	logic ARVALID, ARREADY;
	logic [3:0] ARID;
	logic [3:0] ARLEN;
	logic [`ADDR_WIDTH - 1 : 0] ARADDR;

	modport master (input ARREADY, output ARVALID, ARID, ARLEN, ARADDR);
	modport slave (output ARREADY, input ARVALID, ARID, ARLEN, ARADDR);
endinterface

interface axi_read_data ();
	logic RVALID, RREADY, RLAST;
	logic [3:0] RID;
	logic [`DATA_WIDTH - 1 : 0] RDATA;

	modport master (output RREADY, input RVALID, RLAST, RID, RDATA);
	modport slave (input RREADY, output RVALID, RLAST, RID, RDATA);
endinterface

