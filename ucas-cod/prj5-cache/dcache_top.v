`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

  //TODO: Please add your D-Cache code here
        localparam WAIT         = 18'b000000000000000001,
                   WR           = 18'b000000000000000010,
                   RD           = 18'b000000000000000100,
                   BYPASS_WR    = 18'b000000000000001000,
                   BYPASS_RD    = 18'b000000000000010000,
                   CACHE_WR     = 18'b000000000000100000,
                   CACHE_RD     = 18'b000000000001000000,
                   EVICT        = 18'b000000000010000000,
                   EVICT_WR_REQ = 18'b000000000100000000,
                   EVICT_WR     = 18'b000000001000000000,
                   MEM_RD       = 18'b000000010000000000,
                   RECV         = 18'b000000100000000000,
                   REFILL       = 18'b000001000000000000,
                   RESP         = 18'b000010000000000000,
                   BP_RD_WAIT   = 18'b000100000000000000,
                   BP_WR_WAIT   = 18'b001000000000000000,
                   WR_READY     = 18'b010000000000000000,
                   BP_RD_REQ    = 18'b100000000000000000;

        reg [17:0] current_state;
        reg [17:0] next_state;
        reg [31:0] req_addr;

        always @(posedge clk) begin
                if (rst)
                        req_addr <= 32'b0;
                else if (from_cpu_mem_req_valid && current_state[0])
                        req_addr <= from_cpu_mem_req_addr;
        end

        wire miss, hit;

        wire no_cache = ~(|from_cpu_mem_req_addr[31:5]) || (from_cpu_mem_req_addr[30] && from_cpu_mem_req_addr[29] || from_cpu_mem_req_addr[31]);
        wire bypass = ~(|req_addr[31:5]) || (req_addr[30] && req_addr[29] || req_addr[31]);

        wire [7:0] valid_array0 = valid_array[0];
        wire [7:0] valid_array1 = valid_array[1];
        wire [7:0] valid_array2 = valid_array[2];
        wire [7:0] valid_array3 = valid_array[3];

        wire [7:0] dirty_array0 = dirty_array[0];
        wire [7:0] dirty_array1 = dirty_array[1];
        wire [7:0] dirty_array2 = dirty_array[2];
        wire [7:0] dirty_array3 = dirty_array[3];

        always @(posedge clk) begin
                if (rst)
                        current_state <= WAIT;
                else
                        current_state <= next_state;
        end

        always @(*) begin
                case (current_state)
                        WAIT: begin
                                if (~from_cpu_mem_req && from_cpu_mem_req_valid && ~no_cache)
                                        next_state = RD;
                                else if (from_cpu_mem_req && from_cpu_mem_req_valid && ~no_cache)
                                        next_state = WR;
                                else if (~from_cpu_mem_req && from_cpu_mem_req_valid && no_cache)
                                        next_state = BP_RD_REQ;
                                else if (from_cpu_mem_req && from_cpu_mem_req_valid && no_cache)
                                        next_state = BYPASS_WR;
                                else
                                        next_state = WAIT;
                        end
                        WR: begin
                                if (hit)
                                        next_state = CACHE_WR;
                                else
                                        next_state = EVICT;
                        end
                        RD: begin
                                if (hit && from_cpu_mem_req_valid)
                                        next_state = CACHE_RD;
                                else if (miss && from_cpu_mem_req_valid)
                                        next_state = EVICT;
                                else
                                        next_state = RD;
                        end
                        BYPASS_RD: begin
                                if (from_mem_rd_req_ready)
                                        next_state = BP_RD_WAIT;
                                else
                                        next_state = BYPASS_RD;
                        end
                        BYPASS_WR:begin
                                if (from_mem_wr_req_ready && to_mem_wr_req_valid)
                                        next_state = BP_WR_WAIT;
                                else
                                        next_state = BYPASS_WR;
                        end
                        CACHE_WR: begin
                                next_state = WAIT;
                        end
                        CACHE_RD: begin
                                next_state = RESP;
                        end
                        RESP: begin
                                if (from_cpu_cache_rsp_ready)
                                        next_state = WAIT;
                                else
                                        next_state = RESP;
                        end
                        EVICT: begin
                                if (dirty)
                                        next_state = EVICT_WR_REQ;
                                else
                                        next_state = MEM_RD;
                        end
                        EVICT_WR_REQ: begin
                                if (from_mem_wr_req_ready)
                                        next_state = EVICT_WR;
                                else
                                        next_state = EVICT_WR_REQ;
                        end
                        EVICT_WR: begin
                                if (to_mem_wr_data_last && from_mem_wr_data_ready)
                                        next_state = MEM_RD;
                                else
                                        next_state = EVICT_WR;
                        end
                        MEM_RD: begin
                                if (from_mem_rd_req_ready)
                                        next_state = RECV;
                                else
                                        next_state = MEM_RD;
                        end
                        RECV: begin
                                if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last)
                                        next_state = REFILL;
                                else
                                        next_state = RECV;
                        end
                        REFILL: begin
                                if (write_miss)
                                        next_state = CACHE_WR;
                                else
                                        next_state = RESP;
                        end
                        BP_RD_WAIT: begin
                                if (from_mem_rd_rsp_valid)
                                        next_state = RESP;
                                else
                                        next_state = BP_RD_WAIT;
                        end
                        BP_WR_WAIT: begin
                                if (from_mem_wr_data_ready && to_mem_wr_data_valid)
                                        next_state = WR_READY;
                                else
                                        next_state = BP_WR_WAIT;
                        end
                        WR_READY: begin
                                if (from_cpu_mem_req_valid && to_cpu_mem_req_ready)
                                        next_state = WAIT;
                                else
                                        next_state = WR_READY;
                        end
                        BP_RD_REQ: begin
                                if (from_cpu_mem_req_valid)
                                        next_state = BYPASS_RD;
                                else
                                        next_state = BP_RD_REQ;
                        end
                        default: next_state = WAIT;
                endcase
        end

        assign to_cpu_mem_req_ready = current_state[2] || current_state[16] || current_state[5] || current_state[17] || rst;
        assign to_cpu_cache_rsp_valid = current_state[13] && ~rst;
        assign to_cpu_cache_rsp_data = (bypass) ? cache_evict_data[31:0] : read_hit_data;


        reg write_miss;
        always @(posedge clk) begin
                if (rst)
                        write_miss <= 1'b0;
                else if (current_state[1] && miss)
                        write_miss <= 1'b1;
                else if (current_state[5])
                        write_miss <= 1'b0;
        end

        wire [31:0] evict_wr_data = {32{mem_counter == 3'b000}} & evict_data[ 31:0]
                                  | {32{mem_counter == 3'b001}} & evict_data[ 63:32]
                                  | {32{mem_counter == 3'b010}} & evict_data[ 95:64]
                                  | {32{mem_counter == 3'b011}} & evict_data[127:96]
                                  | {32{mem_counter == 3'b100}} & evict_data[159:128]
                                  | {32{mem_counter == 3'b101}} & evict_data[191:160]
                                  | {32{mem_counter == 3'b110}} & evict_data[223:192]
                                  | {32{mem_counter == 3'b111}} & evict_data[255:224];

        assign to_mem_wr_req_valid = (current_state[8] || current_state[3]) && ~rst;
        assign to_mem_wr_req_addr = (current_state[8]) ? {evict_tag, cache_set_addr, 5'b0} : req_addr;
        assign to_mem_wr_req_len = (current_state[8]) ? 8'b111 : 8'b0;

        assign to_mem_wr_data_last = (mem_counter == 3'b111) || current_state[15];
        assign to_mem_wr_data = (current_state[9]) ? evict_wr_data : wdata;
        assign to_mem_wr_data_valid = (current_state[15] || current_state[9]) && ~rst;
        assign to_mem_wr_data_strb = (current_state[9]) ? 4'hF : wstrb;

        assign to_mem_rd_req_valid = (current_state[10] || current_state[4]) && ~rst;
        assign to_mem_rd_req_addr = (current_state[4]) ? req_addr : {req_addr[31:5], 5'b0};
        assign to_mem_rd_rsp_ready = (current_state[11] || current_state[14]) || rst;
        assign to_mem_rd_req_len = (current_state[10]) ? 8'b111 : 8'b0;

        reg [2:0] mem_counter;
        always @(posedge clk) begin
                if (rst)
                        mem_counter <= 3'b0;
                else if (from_mem_rd_rsp_valid && to_mem_rd_rsp_ready && current_state[11])
                        mem_counter <= mem_counter + 1'b1;
                else if (to_mem_wr_data_valid && from_mem_wr_data_ready && current_state[9])
                        mem_counter <= mem_counter + 1'b1;
        end

        wire [31:0] mask = {{8{wstrb[3]}}, {8{wstrb[2]}}, {8{wstrb[1]}}, {8{wstrb[0]}}};

        reg [31:0] wdata;
        always @(posedge clk) begin
                if (rst)
                        wdata <= 32'b0;
                else if (from_cpu_mem_req_valid && current_state[0] && from_cpu_mem_req)
                        wdata <= from_cpu_mem_req_wdata;
        end

        wire [31:0] to_cache_wr_data = (wdata & mask) | (read_hit_data & ~mask);

        reg [3:0] wstrb;
        always @(posedge clk) begin
                if (rst)
                        wstrb <= 4'b0;
                else if (from_cpu_mem_req_valid && current_state[0] && from_cpu_mem_req)
                        wstrb <= from_cpu_mem_req_wstrb;
        end

        reg [255:0] cache_evict_data;
        always @(posedge clk) begin
                if (rst)
                        cache_evict_data <= 256'b0;
                else if (from_mem_rd_rsp_valid && to_mem_rd_rsp_ready) begin
                        case (mem_counter)
                                3'b000: cache_evict_data[ 31:  0] <= from_mem_rd_rsp_data;
                                3'b001: cache_evict_data[ 63: 32] <= from_mem_rd_rsp_data;
                                3'b010: cache_evict_data[ 95: 64] <= from_mem_rd_rsp_data;
                                3'b011: cache_evict_data[127: 96] <= from_mem_rd_rsp_data;
                                3'b100: cache_evict_data[159:128] <= from_mem_rd_rsp_data;
                                3'b101: cache_evict_data[191:160] <= from_mem_rd_rsp_data;
                                3'b110: cache_evict_data[223:192] <= from_mem_rd_rsp_data;
                                3'b111: cache_evict_data[255:224] <= from_mem_rd_rsp_data;
                        endcase
                end
        end

        integer i;
        reg [`CACHE_SET - 1 : 0] valid_array [`CACHE_WAY - 1 : 0];
        always @(posedge clk) begin
                if (rst) begin
                        for (i = 0; i < `CACHE_SET; i = i + 1)
                                valid_array[i] <= 8'b0;
                end
                else if (current_state[7]) begin
                        case (full_cache_addr & {4{block_full}})
                                4'b0001: valid_array[0][req_addr[7:5]] <= 1'b0;
                                4'b0010: valid_array[1][req_addr[7:5]] <= 1'b0;
                                4'b0100: valid_array[2][req_addr[7:5]] <= 1'b0;
                                4'b1000: valid_array[3][req_addr[7:5]] <= 1'b0;
                                default: begin;
                                        valid_array[0] <= valid_array[0];
                                        valid_array[1] <= valid_array[1];
                                        valid_array[2] <= valid_array[2];
                                        valid_array[3] <= valid_array[3];
                                end
                        endcase
                end
                else if (current_state[12]) begin
                        case (evict_wen)
                                4'b0001: valid_array[0][req_addr[7:5]] <= 1'b1;
                                4'b0010: valid_array[1][req_addr[7:5]] <= 1'b1;
                                4'b0100: valid_array[2][req_addr[7:5]] <= 1'b1;
                                4'b1000: valid_array[3][req_addr[7:5]] <= 1'b1;
                                default: begin
                                        valid_array[0] <= valid_array[0];
                                        valid_array[1] <= valid_array[1];
                                        valid_array[2] <= valid_array[2];
                                        valid_array[3] <= valid_array[3];
                                end
                        endcase
                end
        end

        integer j;
        reg [`CACHE_SET - 1 : 0] dirty_array [`CACHE_WAY - 1 : 0];
        always @(posedge clk) begin
                if (rst) begin
                        for (j = 0; j < `CACHE_SET; j = j + 1)
                                dirty_array[j] <= 8'b0;
                end
                else if (current_state[7]) begin
                        case (full_cache_addr & {4{block_full}})
                                4'b0001: dirty_array[0][req_addr[7:5]] <= 1'b0;
                                4'b0010: dirty_array[1][req_addr[7:5]] <= 1'b0;
                                4'b0100: dirty_array[2][req_addr[7:5]] <= 1'b0;
                                4'b1000: dirty_array[3][req_addr[7:5]] <= 1'b0;
                                default: begin;
                                        dirty_array[0] <= dirty_array[0];
                                        dirty_array[1] <= dirty_array[1];
                                        dirty_array[2] <= dirty_array[2];
                                        dirty_array[3] <= dirty_array[3];
                                end
                        endcase
                end
                else if (current_state[5]) begin
                        case (hit_way)
                                4'b0001: dirty_array[0][req_addr[7:5]] <= 1'b1;
                                4'b0010: dirty_array[1][req_addr[7:5]] <= 1'b1;
                                4'b0100: dirty_array[2][req_addr[7:5]] <= 1'b1;
                                4'b1000: dirty_array[3][req_addr[7:5]] <= 1'b1;
                                default: begin
                                        dirty_array[0] <= dirty_array[0];
                                        dirty_array[1] <= dirty_array[1];
                                        dirty_array[2] <= dirty_array[2];
                                        dirty_array[3] <= dirty_array[3];
                                end
                        endcase
                end
        end

        wire [23:0] rd_tag [3:0];
        wire [255:0] rd_data [3:0];
        wire [3:0] evict_wen;

        // the write_data to mem
        wire [255:0] mem_wr_data = {256{set_offset[0]}} & {valid_rd_data[255: 32], to_cache_wr_data}
                                 | {256{set_offset[1]}} & {valid_rd_data[255: 64], to_cache_wr_data, valid_rd_data[31:0]}
                                 | {256{set_offset[2]}} & {valid_rd_data[255: 96], to_cache_wr_data, valid_rd_data[63:0]}
                                 | {256{set_offset[3]}} & {valid_rd_data[255:128], to_cache_wr_data, valid_rd_data[95:0]}
                                 | {256{set_offset[4]}} & {valid_rd_data[255:160], to_cache_wr_data, valid_rd_data[127:0]}
                                 | {256{set_offset[5]}} & {valid_rd_data[255:192], to_cache_wr_data, valid_rd_data[159:0]}
                                 | {256{set_offset[6]}} & {valid_rd_data[255:224], to_cache_wr_data, valid_rd_data[191:0]}
                                 | {256{set_offset[7]}} & {to_cache_wr_data, valid_rd_data[223:0]};
        
        wire [255:0] cache_wr_data = {256{current_state[ 5]}} & mem_wr_data
                                   | {256{current_state[12]}} & cache_evict_data;

        genvar way_i;
        generate 
                for (way_i = 0; way_i < `CACHE_WAY; way_i = way_i + 1) begin: tag_way
                        tag_array tag(
                                .clk(clk),
                                .waddr(req_addr[7:5]),
                                .raddr(req_addr[7:5]),
                                .wen(evict_wen[way_i] && current_state[12]),
                                .wdata(req_addr[31:8]),
                                .rdata(rd_tag[way_i])
                        );
                end
        endgenerate

        genvar way_j;
        generate 
                for (way_j = 0; way_j < `CACHE_WAY; way_j = way_j + 1) begin: data_way
                        data_array data(
                                .clk(clk),
                                .waddr(req_addr[7:5]),
                                .raddr(req_addr[7:5]),
                                .wen(evict_wen[way_j] && current_state[12] || hit_way[way_j] && current_state[5]),
                                .wdata(cache_wr_data),
                                .rdata(rd_data[way_j])
                        );
                end
        endgenerate

        wire [2:0] cache_set_addr;
        wire [4:0] cache_in_addr;
        wire [23:0] cache_tag;

        assign {cache_tag, cache_set_addr, cache_in_addr} = req_addr;

        wire [3:0] valid_way = {valid_array[3][cache_set_addr], 
                                valid_array[2][cache_set_addr],
                                valid_array[1][cache_set_addr],
                                valid_array[0][cache_set_addr]};
        
        wire [3:0] hit_way = {rd_tag[3] == cache_tag && valid_way[3],
                              rd_tag[2] == cache_tag && valid_way[2],
                              rd_tag[1] == cache_tag && valid_way[1],
                              rd_tag[0] == cache_tag && valid_way[0]};

        assign hit = |hit_way;
        assign miss = ~hit;

        wire [255:0] valid_rd_data = {256{hit_way[0]}} & rd_data[0] 
                                   | {256{hit_way[1]}} & rd_data[1]
                                   | {256{hit_way[2]}} & rd_data[2]
                                   | {256{hit_way[3]}} & rd_data[3];

        wire [7:0] set_offset = {
                cache_in_addr[4:2] == 3'b111,
                cache_in_addr[4:2] == 3'b110,
                cache_in_addr[4:2] == 3'b101,
                cache_in_addr[4:2] == 3'b100,
                cache_in_addr[4:2] == 3'b011,
                cache_in_addr[4:2] == 3'b010,
                cache_in_addr[4:2] == 3'b001,
                cache_in_addr[4:2] == 3'b000
        };
        
        wire [31:0] read_hit_data = {32{set_offset[0]}} & valid_rd_data[ 31: 0]
                                  | {32{set_offset[1]}} & valid_rd_data[ 63:32]
                                  | {32{set_offset[2]}} & valid_rd_data[ 95:64]
                                  | {32{set_offset[3]}} & valid_rd_data[127:96]
                                  | {32{set_offset[4]}} & valid_rd_data[159:128]
                                  | {32{set_offset[5]}} & valid_rd_data[191:160]
                                  | {32{set_offset[6]}} & valid_rd_data[223:192]
                                  | {32{set_offset[7]}} & valid_rd_data[255:224];
        
        // LRU-2

        integer matrix_i, matrix_j;
        reg [3:0] visit_matrix [7:0][3:0];
        always @(posedge clk) begin
                if (rst) begin
                        for (matrix_i = 0; matrix_i < 8; matrix_i = matrix_i + 1) begin
                                for (matrix_j = 0; matrix_j < 3; matrix_j = matrix_j + 1) begin
                                        visit_matrix[matrix_i][matrix_j] <= 4'b0;
                                end
                        end
                end
                else if ((current_state[13] && ~bypass && from_cpu_cache_rsp_ready) || current_state[5]) begin
                        case (hit_way)
                                4'b0001: begin
                                        visit_matrix[req_addr[7:5]][0] <= 4'b1110;
                                        visit_matrix[req_addr[7:5]][1] <= visit_matrix[req_addr[7:5]][1] & 4'b1110;
                                        visit_matrix[req_addr[7:5]][2] <= visit_matrix[req_addr[7:5]][2] & 4'b1110;
                                        visit_matrix[req_addr[7:5]][3] <= visit_matrix[req_addr[7:5]][3] & 4'b1110;
                                end
                                4'b0010: begin
                                        visit_matrix[req_addr[7:5]][0] <= visit_matrix[req_addr[7:5]][0] & 4'b1101;
                                        visit_matrix[req_addr[7:5]][1] <= 4'b1101;
                                        visit_matrix[req_addr[7:5]][2] <= visit_matrix[req_addr[7:5]][2] & 4'b1101;
                                        visit_matrix[req_addr[7:5]][3] <= visit_matrix[req_addr[7:5]][3] & 4'b1101;
                                end
                                4'b0100: begin
                                        visit_matrix[req_addr[7:5]][0] <= visit_matrix[req_addr[7:5]][0] & 4'b1011;
                                        visit_matrix[req_addr[7:5]][1] <= visit_matrix[req_addr[7:5]][1] & 4'b1011;
                                        visit_matrix[req_addr[7:5]][2] <= 4'b1011;
                                        visit_matrix[req_addr[7:5]][3] <= visit_matrix[req_addr[7:5]][3] & 4'b1011;
                                end
                                4'b1000: begin
                                        visit_matrix[req_addr[7:5]][0] <= visit_matrix[req_addr[7:5]][0] & 4'b0111;
                                        visit_matrix[req_addr[7:5]][1] <= visit_matrix[req_addr[7:5]][1] & 4'b0111;
                                        visit_matrix[req_addr[7:5]][2] <= visit_matrix[req_addr[7:5]][2] & 4'b0111;
                                        visit_matrix[req_addr[7:5]][3] <= 4'b0111;
                                end
                                default: begin
                                        visit_matrix[req_addr[7:5]][0] <= visit_matrix[req_addr[7:5]][0];
                                        visit_matrix[req_addr[7:5]][1] <= visit_matrix[req_addr[7:5]][1];
                                        visit_matrix[req_addr[7:5]][2] <= visit_matrix[req_addr[7:5]][2];
                                        visit_matrix[req_addr[7:5]][3] <= visit_matrix[req_addr[7:5]][3];
                                end
                        endcase
                end
        end

        wire [3:0] empty_block = {
                ~valid_array[3][cache_set_addr],
                ~valid_array[2][cache_set_addr],
                ~valid_array[1][cache_set_addr],
                ~valid_array[0][cache_set_addr]
        };

        wire block_full = ~(|empty_block);

        // write way when cache is empty
        wire [3:0] empty_cache_addr = {
                empty_block[3],
                ~empty_block[3] && empty_block[2],
                ~empty_block[3] && ~empty_block[2] && empty_block[1],
                ~empty_block[3] && ~empty_block[2] && ~empty_block[1] && empty_block[0]
        };

        // write way when cache is full
        wire [3:0] full_cache_addr = {
                ~(|visit_matrix[cache_set_addr][3]),
                ~(|visit_matrix[cache_set_addr][2]),
                ~(|visit_matrix[cache_set_addr][1]),
                ~(|visit_matrix[cache_set_addr][0])
        };

        assign evict_wen = empty_cache_addr;
        wire [255:0] evict_data = {256{evict_wen[0]}} & rd_data[0]
                                | {256{evict_wen[1]}} & rd_data[1]
                                | {256{evict_wen[2]}} & rd_data[2]
                                | {256{evict_wen[3]}} & rd_data[3];

        // deal with dirty data
        wire dirty = (full_cache_addr[0] && dirty_array[0][cache_set_addr]
                   || full_cache_addr[1] && dirty_array[1][cache_set_addr]
                   || full_cache_addr[2] && dirty_array[2][cache_set_addr]
                   || full_cache_addr[3] && dirty_array[3][cache_set_addr]) && block_full;
        
        wire [23:0]evict_tag = {24{evict_wen[0]}} & rd_tag[0]
                             | {24{evict_wen[1]}} & rd_tag[1]
                             | {24{evict_wen[2]}} & rd_tag[2]
                             | {24{evict_wen[3]}} & rd_tag[3];

        // bypass and io
        

endmodule

