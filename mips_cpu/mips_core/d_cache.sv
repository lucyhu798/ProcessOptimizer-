	`include "mips_core.svh"
	
	interface d_cache_input_ifc ();
	logic valid;
	mips_core_pkg::MemAccessType mem_action;
	logic [`ADDR_WIDTH - 1 : 0] addr;
	logic [`ADDR_WIDTH - 1 : 0] addr_next;
	logic [`DATA_WIDTH - 1 : 0] data;
	
	modport in  (input valid, mem_action, addr, addr_next, data);
	modport out (output valid, mem_action, addr, addr_next, data);
	endinterface
	
	module d_cache #(
	parameter INDEX_WIDTH = 4,
	parameter BLOCK_OFFSET_WIDTH = 2
	)(
	// General signals
	input clk,    // Clock
	input rst_n,  // Synchronous reset active low
	
	// Request
	d_cache_input_ifc.in in,
	
	// Response
	cache_output_ifc.out out,
	output logic exit_runahead,
	// AXI interfaces
	axi_write_address.master mem_write_address,
	axi_write_data.master mem_write_data,
	axi_write_response.master mem_write_response,
	axi_read_address.master mem_read_address,
	axi_read_data.master mem_read_data
	);
	localparam TAG_WIDTH = `ADDR_WIDTH - INDEX_WIDTH - BLOCK_OFFSET_WIDTH - 2;
	localparam LINE_SIZE = 1 << BLOCK_OFFSET_WIDTH;
	localparam DEPTH = 1 << INDEX_WIDTH;
	
	// Check if the parameters are set correctly
	generate
		if(TAG_WIDTH <= 0 || LINE_SIZE > 16)
		begin
			INVALID_D_CACHE_PARAM invalid_d_cache_param ();
		end
	endgenerate
	
	// Parsing
	logic [TAG_WIDTH - 1 : 0] i_tag;
	logic [INDEX_WIDTH - 1 : 0] i_index;
	logic [BLOCK_OFFSET_WIDTH - 1 : 0] i_block_offset;
	
	logic [INDEX_WIDTH - 1 : 0] i_index_next;
	
	assign {i_tag, i_index, i_block_offset} = in.addr[`ADDR_WIDTH - 1 : 2];
	assign i_index_next = in.addr_next[BLOCK_OFFSET_WIDTH + 2 +: INDEX_WIDTH];
	// Above line uses +: slice, a feature of SystemVerilog
	// See https://stackoverflow.com/questions/18067571
	
	// States
	enum logic [2:0] {
		STATE_READY,            // Ready for incoming requests
		STATE_FLUSH_REQUEST,    // Sending out memory write request
		STATE_FLUSH_DATA,       // Writes out a dirty cache line
		STATE_REFILL_REQUEST,   // Sending out memory read request
		STATE_REFILL_DATA       // Loads a cache line from memory
	} state, next_state;
	logic pending_write_response;
	
	// Registers for flushing and refilling
	logic [INDEX_WIDTH - 1:0] r_index;
	logic [TAG_WIDTH - 1:0] r_tag;
	
	// databank signals
	logic [LINE_SIZE - 1 : 0] databank_select;
	logic [LINE_SIZE - 1 : 0] databank_we[4];
	logic [`DATA_WIDTH - 1 : 0] databank_wdata[4];
	logic [INDEX_WIDTH - 1 : 0] databank_waddr;
	logic [INDEX_WIDTH - 1 : 0] databank_raddr;
	logic [`DATA_WIDTH - 1 : 0] databank_rdata[4] [LINE_SIZE];
	
	logic [1:0] n_set;    // decides which one of the N-way cache was  accessed and where it should be placed.
	logic [1:0] new_set; // only used to see the set that still have space.
	logic [1:0] dirty_set;
	
	/*logic [TAG_WIDTH: 0] tagbank_newdata;
	assign tagbank_newdata = {tagbank_wdata[n_set], n_set};*/
	
	logic [1:0] index_list[16][4]; // an index list for each index with the n-way cache inside. index[index][order for nset]
	//logic[1:0] LRU_set[4]; // the LRU cache set in the index
	logic has_tag;
	
	logic [9:0] PSEL_ctr;
	logic [4:0] BIP_ctr;
	logic BIP; // used to check if the current policiy is a BIP
	logic LRU; // checks to see if we need to insert at LRU or at MRU;
	
	
	// databanks
	genvar h;
	genvar g;
	generate
		for(h = 0; h < 4; h++)
		begin: Ndatabanks
			for (g = 0; g < LINE_SIZE; g++)
			begin : databanks
				cache_bank #(
					.DATA_WIDTH (`DATA_WIDTH),
					.ADDR_WIDTH (INDEX_WIDTH)
				) databank (
					.clk,
					.i_we (databank_we[h][g]),
					.i_wdata(databank_wdata[h]),
					.i_waddr(databank_waddr),
					.i_raddr(databank_raddr),
					.o_rdata(databank_rdata[h][g])
				);
			end
		end
	endgenerate
	
	
	// tagbank signals
	logic tagbank_we[4];
	logic [TAG_WIDTH - 1 : 0] tagbank_wdata[4];
	logic [INDEX_WIDTH - 1 : 0] tagbank_waddr;
	logic [INDEX_WIDTH - 1 : 0] tagbank_raddr;
	logic [TAG_WIDTH - 1 : 0] tagbank_rdata[4];
	
	genvar j;
	generate
		for(j = 0; j < 4; j++)
		begin: tagbanks
			cache_bank #(
				.DATA_WIDTH (TAG_WIDTH),
				.ADDR_WIDTH (INDEX_WIDTH)
			) tagbank (
				.clk,
				.i_we    (tagbank_we[j]),
				.i_wdata (tagbank_wdata[j]),
				.i_waddr (tagbank_waddr),
				.i_raddr (tagbank_raddr),
				.o_rdata (tagbank_rdata[j])
			);
			end
	endgenerate
	
	// Valid bits
	logic [DEPTH - 1 : 0] valid_bits[4]; // valid_bits[n_set][index]
	logic [DEPTH - 1 : 0] total_valid_bits; // to see if there are any open spaces in cache
	logic [1:0] choosen_set; // the set the tag was found in when hit;
	logic [1:0] position;
	// Dirty bits
	logic [DEPTH - 1 : 0] dirty_bits[4];
	logic [DEPTH -1 : 0 ] total_dirty_bits;
	
	// Shift registers for flushing
	logic [`DATA_WIDTH - 1 : 0] shift_rdata[LINE_SIZE];
	
	// Intermediate signals
	logic hit, miss;
	logic last_flush_word;
	logic last_refill_word;
	assign exit_runahead = last_refill_word;
	
	// need to get the next open space in the set
	always_comb
	begin
		for(int i = 0; i < 4; i++)
		begin
			if(valid_bits[i][i_index] == 0) begin
				new_set = i;
				total_valid_bits[i_index] = 0;
				break;
			end
			else
				total_valid_bits[i_index] = 1;
		end
	end
	
	always_comb
	begin
		for(int i = 0; i < 4; i++)
		begin
			if(dirty_bits[i][i_index] == 0) begin
				dirty_set = i;
				total_dirty_bits[i_index] = 0;
				break;
			end
			else
				total_dirty_bits[i_index] = 1;
		end
	end
	
	always_comb
	begin
		for(int i = 0; i < 4; i++) begin 
			if(i_tag == tagbank_rdata[i]) begin
				choosen_set = i;
				has_tag = 1;
				break;
			end
			else
				has_tag = 0;
		end
	end
	
	always_comb
	begin
		hit = in.valid
			& valid_bits[choosen_set][i_index]
			& (has_tag)
			& (state == STATE_READY);
		miss = in.valid & ~hit;
		last_flush_word = databank_select[LINE_SIZE - 1] & mem_write_data.WVALID;
		last_refill_word = databank_select[LINE_SIZE - 1] & mem_read_data.RVALID;
	end
	
	always_comb
	begin
		mem_write_address.AWVALID = state == STATE_FLUSH_REQUEST;
		mem_write_address.AWID = 0;
		mem_write_address.AWLEN = LINE_SIZE;
		mem_write_address.AWADDR = {tagbank_rdata[n_set], i_index, {BLOCK_OFFSET_WIDTH + 2{1'b0}}};
		mem_write_data.WVALID = state == STATE_FLUSH_DATA;
		mem_write_data.WID = 0;
		mem_write_data.WDATA = shift_rdata[0];
		mem_write_data.WLAST = last_flush_word;
	
		// Always ready to consume write response
		mem_write_response.BREADY = 1'b1;
	end
	
	always_comb begin
		mem_read_address.ARADDR = {r_tag, r_index, {BLOCK_OFFSET_WIDTH + 2{1'b0}}};
		mem_read_address.ARLEN = LINE_SIZE;
		mem_read_address.ARVALID = state == STATE_REFILL_REQUEST;
		mem_read_address.ARID = 4'd1;
	
		// Always ready to consume data
		mem_read_data.RREADY = 1'b1;
	end
	
	always_comb
	begin
		databank_we[n_set] = '0;
		if (mem_read_data.RVALID)               // We are refilling data
			databank_we[n_set] = databank_select;
		else if (hit & (in.mem_action == WRITE))    // We are storing a word
			databank_we[n_set][i_block_offset] = 1'b1;
	end
	
	always_comb
	begin
		if (state == STATE_READY)
		begin
			databank_wdata[n_set] = in.data;
			databank_waddr = i_index;
			if (next_state == STATE_FLUSH_DATA)
				databank_raddr = i_index;
			else
				databank_raddr = i_index_next;
		end
		else
		begin
			databank_wdata[n_set] = mem_read_data.RDATA;
			databank_waddr = r_index;
			if (next_state == STATE_READY)
				databank_raddr = i_index_next;
			else
				databank_raddr = r_index;
		end
	end
	
	always_comb
	begin
		tagbank_we[n_set] = last_refill_word;
		tagbank_wdata[n_set] = r_tag;
		tagbank_waddr = r_index;
		tagbank_raddr = i_index_next;
	end
	
	always_comb
	begin
		out.valid = hit;
		out.data = databank_rdata[n_set][i_block_offset];
	end
	
	always_comb
	begin
		next_state = state;
		unique case (state)
			STATE_READY:
				if (miss) begin
					if(total_valid_bits[i_index] & total_dirty_bits[i_index])
						next_state = STATE_FLUSH_REQUEST;
					else
						next_state = STATE_REFILL_REQUEST;
				end
			STATE_FLUSH_REQUEST:
				if (mem_write_address.AWREADY)
					next_state = STATE_FLUSH_DATA;
	
			STATE_FLUSH_DATA:
				if (last_flush_word && mem_write_data.WREADY)
					next_state = STATE_REFILL_REQUEST;
	
			STATE_REFILL_REQUEST:
				if (mem_read_address.ARREADY)
					next_state = STATE_REFILL_DATA;
	
			STATE_REFILL_DATA:
				if (last_refill_word)
					next_state = STATE_READY;
		endcase
	end

		always_comb
		begin
			if(i_index == 'b1111)
				BIP = 0; 
			else if(i_index == 'b1000)
				BIP = 1;
			else if(PSEL_ctr >= 'b1000000000)
				BIP = 1; 
			else  
				BIP = 0; 
		end

		always_comb
		begin 
			if(BIP_ctr == 'b00000)
				LRU = 0;
			else
				LRU = 1;
		end 
		
		
	always_comb
	begin
		if(next_state == STATE_READY)
			if(hit)
				n_set = choosen_set;
		if(state == STATE_FLUSH_REQUEST)
			n_set = index_list[i_index][0];
	end
	
	always_comb
	begin
		for(int i = 0; i < 4; i++)
			if(i != n_set)begin
				databank_we[i] = 0;
				databank_wdata[i] = 0;
			end
	end
	
	
	/*always_ff @(posedge clk) begin // LRU 
		if(state == STATE_READY) begin
			if(hit)begin
				for(int i = 0; i < 4; i++)begin
					if(index_list[i_index][i] == choosen_set)begin
						for(int j = i; j < 3; j++)
							index_list[i_index][j] = index_list[i_index][j+1];
						index_list[i_index][3] = choosen_set;
						break;
					end
				end
			end
			else if(miss)
				if(next_state == STATE_REFILL_REQUEST) begin 
					if(~total_valid_bits[i_index])
						n_set = new_set;
					else if(~total_dirty_bits[i_index])
						n_set = dirty_set;
	
					for(int i = 0; i < 4; i++)begin
						if(index_list[i_index][i] == n_set)begin
							position = i;
							break;
						end
					end
				end
		end
		if(state == STATE_FLUSH_REQUEST)begin
			n_set = index_list[i_index][0];
			position = 0;
		end
		if(state == STATE_REFILL_REQUEST)begin
			for(int i = position; i < 3; i++) begin
				index_list[i_index][i] = index_list[i_index][i+1];
			end
			index_list[i_index][3] = n_set;
	
		end 
	end
	*/

	/* always_ff @(posedge clk) begin // LIP only 
		if(state == STATE_READY) begin
			if(hit)begin
				for(int i = 0; i < 4; i++)begin
					if(index_list[i_index][i] == choosen_set)begin
						for(int j = i; j < 3; j++)
							index_list[i_index][j] = index_list[i_index][j+1];
						index_list[i_index][3] = choosen_set;
						break;
					end
				end
			end
			else if(miss)
				if(next_state == STATE_REFILL_REQUEST) begin 
					if(~total_valid_bits[i_index])
						n_set = new_set;
					else if(~total_dirty_bits[i_index])
						n_set = dirty_set;
	
					for(int i = 0; i < 4; i++)begin
						if(index_list[i_index][i] == n_set)begin
							position = i;
							break;
						end
					end
				end
		end
		if(state == STATE_FLUSH_REQUEST)begin
			n_set = index_list[i_index][0];
			position = 0;
		end
		if(state == STATE_REFILL_REQUEST)begin
			if(position != 0)
					for(int i = 1; i <= position ; i++) 
						index_list[i_index][i] = index_list[i_index][i-1];
			index_list[i_index][0] = n_set;
		end 
	end
	*/


		/*always_ff @(posedge clk) begin // BIP only 
			if(state == STATE_READY) begin
				if(hit)begin
					if(~LRU)
						for(int i = 0; i < 4; i++)begin
							if(index_list[i_index][i] == choosen_set)begin
								for(int j = i; j < 3; j++)
									index_list[i_index][j] = index_list[i_index][j+1];
								index_list[i_index][3] = choosen_set;
								break;
							end
						end
					else
						for(int i = 0; i < 4; i++)begin
							if(index_list[i_index][i] == choosen_set)begin
								for(int j = 1; j <= i; j++)
									index_list[i_index][j] = index_list[i_index][j-1];
								index_list[i_index][0] = choosen_set;
								break;
							end
						end

				end
				else if(miss)
					BIP_ctr = BIP_ctr + 1; 
					if(next_state == STATE_REFILL_REQUEST) begin 
						if(~total_valid_bits[i_index])
							n_set = new_set;
						else if(~total_dirty_bits[i_index])
							n_set = dirty_set;
		
						for(int i = 0; i < 4; i++)begin
							if(index_list[i_index][i] == n_set)begin
								position = i;
								break;
							end
						end
					end
			end
			if(state == STATE_FLUSH_REQUEST)begin
				n_set = index_list[i_index][0];
				position = 0;
			end
			if(state == STATE_REFILL_REQUEST)begin
				if(position != 0)
					if(~LRU)
						for(int i = position; i < 3; i++) begin
							index_list[i_index][i] = index_list[i_index][i+1];
						end
					else
						for(int i = 1; i <= position; i++) begin
							index_list[i_index][i] = index_list[i_index][i-1];
						end
				if(~LRU)
					index_list[i_index][3] = n_set;
				else 
					index_list[i_index][0] = n_set;
			end 
		end
		*/



		always_ff @(posedge clk) begin  // LIP and BIP
			if(state == STATE_READY) begin
				if(BIP)begin 
					if(hit)begin
						if(~LRU)
							for(int i = 0; i < 4; i++)begin
								if(index_list[i_index][i] == choosen_set)begin
									for(int j = i; j < 3; j++)
										index_list[i_index][j] = index_list[i_index][j+1];
									index_list[i_index][3] = choosen_set;
									break;
								end
							end
						else
							for(int i = 0; i < 4; i++)begin
								if(index_list[i_index][i] == choosen_set)begin
									for(int j = 1; j <= i; j++)
										index_list[i_index][j] = index_list[i_index][j-1];
									index_list[i_index][0] = choosen_set;
									break;
								end
							end

					end
					else if(miss)
						PSEL_ctr = PSEL_ctr - 1; 
						BIP_ctr = BIP_ctr + 1; 
						if(next_state == STATE_REFILL_REQUEST) begin 
							if(~total_valid_bits[i_index])
								n_set = new_set;
							else if(~total_dirty_bits[i_index])
								n_set = dirty_set;
			
							for(int i = 0; i < 4; i++)begin
								if(index_list[i_index][i] == n_set)begin
									position = i;
									break;
								end
							end
						end
				end 
				else begin
					if(hit)begin
						for(int i = 0; i < 4; i++)begin
							if(index_list[i_index][i] == choosen_set)begin
								for(int j = i; j < 3; j++)
									index_list[i_index][j] = index_list[i_index][j+1];
								index_list[i_index][3] = choosen_set;
								break;
							end
						end
					end
					else if(miss)
						PSEL_ctr = PSEL_ctr + 1; 
						if(next_state == STATE_REFILL_REQUEST) begin 
							if(~total_valid_bits[i_index])
								n_set = new_set;
							else if(~total_dirty_bits[i_index])
								n_set = dirty_set;
			
							for(int i = 0; i < 4; i++)begin
								if(index_list[i_index][i] == n_set)begin
									position = i;
									break;
								end
							end
						end
				end 
			end
			if(state == STATE_FLUSH_REQUEST)begin
			n_set = index_list[i_index][0];
			position = 0;
			end
			if(state == STATE_REFILL_REQUEST)begin
				if(BIP)begin
					if(position != 0)
						if(~LRU)
							for(int i = position; i < 3; i++) begin
								index_list[i_index][i] = index_list[i_index][i+1];
							end
						else
							for(int i = 1; i <= position; i++) begin
								index_list[i_index][i] = index_list[i_index][i-1];
							end
					if(~LRU)
						index_list[i_index][3] = n_set;
					else 
						index_list[i_index][0] = n_set;
				end 
				else begin 
					if(position != 0)
						for(int i = 1; i <= position ; i++) 
							index_list[i_index][i] = index_list[i_index][i-1];
					index_list[i_index][0] = n_set;
				end 
			end 
		end
	
	always_ff @(posedge clk) begin
		if (~rst_n)
			pending_write_response <= 1'b0;
		else if (mem_write_address.AWVALID && mem_write_address.AWREADY)
			pending_write_response <= 1'b1;
		else if (mem_write_response.BVALID && mem_write_response.BREADY)
			pending_write_response <= 1'b0;
	end
	
	always_ff @(posedge clk)
	begin
		if (state == STATE_FLUSH_DATA && mem_write_data.WREADY)
			for (int i = 0; i < LINE_SIZE - 1; i++)
				shift_rdata[i] <= shift_rdata[i+1];
	
		if (state == STATE_FLUSH_REQUEST && next_state == STATE_FLUSH_DATA) begin
			for (int i = 0; i < LINE_SIZE; i++)
				shift_rdata[i] <= databank_rdata[n_set][i];
		end
	end
	
	always_ff @(posedge clk)
	begin
		if(~rst_n)
		begin
			state <= STATE_READY;
			databank_select <= 1;
			for(int i = 0 ; i < 4; i++)
				valid_bits[i] <= '0;
			n_set = 0;
			position = 0;
			PSEL_ctr = 'b1000000000;
			BIP_ctr = 0;
		end
		else
		begin
			state <= next_state;
	
			case (state)
				STATE_READY:
				begin
					if (miss)
					begin
						r_tag <= i_tag;
						r_index <= i_index;
					end
					else if (in.mem_action == WRITE)
						dirty_bits[choosen_set][i_index] <= 1'b1;
				end
	
				STATE_FLUSH_DATA:
				begin
					if (mem_write_data.WREADY)
						databank_select <= {databank_select[LINE_SIZE - 2 : 0],
							databank_select[LINE_SIZE - 1]};
				end
	
				STATE_REFILL_DATA:
				begin
					if (mem_read_data.RVALID)
						databank_select <= {databank_select[LINE_SIZE - 2 : 0],
							databank_select[LINE_SIZE - 1]};
	
					if (last_refill_word)
					begin
						valid_bits[n_set][r_index] <= 1'b1;
						dirty_bits[n_set][r_index] <= 1'b0;
					end
				end
			endcase
		end
	end
	endmodule