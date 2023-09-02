`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here

        localparam WAIT      = 8'b00000001,
                   TAG_RD    = 8'b00000010,
                   CACHE_RD  = 8'b00000100,
                   EVICT     = 8'b00001000,
                   MEM_RD    = 8'b00010000,
                   RECV      = 8'b00100000,
                   REFILL    = 8'b01000000,
                   RESP      = 8'b10000000;

        reg [7:0] current_state;
        reg [7:0] next_state;
        reg [31:0] req_pc;

        always @(posedge clk) begin
                if (rst)
                        req_pc <= 32'b0;
                else if (from_cpu_inst_req_valid && to_cpu_inst_req_ready)
                        req_pc <= from_cpu_inst_req_addr;
        end

        wire read_miss, read_hit;
        always @(posedge clk) begin
                if (rst)
                        current_state <= WAIT;
                else
                        current_state <= next_state;
        end

        always @(*) begin
                case (current_state)
                        WAIT: begin
                                if (from_cpu_inst_req_valid)
                                        next_state = TAG_RD;
                                else
                                        next_state = WAIT;
                        end
                        TAG_RD: begin
                                if (read_miss)
                                        next_state = EVICT;
                                else
                                        next_state = CACHE_RD;
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
                                next_state = MEM_RD;
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
                                next_state = RESP;
                        end
                        default: next_state = WAIT;
                endcase
        end

        assign to_cpu_inst_req_ready = current_state[0] || rst;
        assign to_cpu_cache_rsp_valid = current_state[7] && ~rst;
        assign to_cpu_cache_rsp_data = read_hit_data;

        assign to_mem_rd_req_valid = current_state[4] && ~rst;
        assign to_mem_rd_req_addr = {req_pc[31:5], 5'b0};
        assign to_mem_rd_rsp_ready = current_state[5] || rst;

        reg [2:0] from_mem_rd_counter;
        always @(posedge clk) begin
                if (rst)
                        from_mem_rd_counter <= 3'b0;
                else if (from_mem_rd_rsp_valid && to_mem_rd_rsp_ready)
                        from_mem_rd_counter <= from_mem_rd_counter + 1'b1;
        end

        reg [255:0] write_data;
        always @(posedge clk) begin
                if (rst)
                        write_data <= 256'b0;
                else if (from_mem_rd_rsp_valid && to_mem_rd_rsp_ready) begin
                        case (from_mem_rd_counter)
                                3'b000: write_data[ 31:  0] <= from_mem_rd_rsp_data;
                                3'b001: write_data[ 63: 32] <= from_mem_rd_rsp_data;
                                3'b010: write_data[ 95: 64] <= from_mem_rd_rsp_data;
                                3'b011: write_data[127: 96] <= from_mem_rd_rsp_data;
                                3'b100: write_data[159:128] <= from_mem_rd_rsp_data;
                                3'b101: write_data[191:160] <= from_mem_rd_rsp_data;
                                3'b110: write_data[223:192] <= from_mem_rd_rsp_data;
                                3'b111: write_data[255:224] <= from_mem_rd_rsp_data;
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
                else if (current_state[3] && block_full) begin
                        case (full_cache_addr)
                                4'b0001: valid_array[0][req_pc[7:5]] <= 1'b0;
                                4'b0010: valid_array[1][req_pc[7:5]] <= 1'b0;
                                4'b0100: valid_array[2][req_pc[7:5]] <= 1'b0;
                                4'b1000: valid_array[3][req_pc[7:5]] <= 1'b0;
                                default: begin;
                                        valid_array[0] <= valid_array[0];
                                        valid_array[1] <= valid_array[1];
                                        valid_array[2] <= valid_array[2];
                                        valid_array[3] <= valid_array[3];
                                end
                        endcase
                end
                else if (current_state[6]) begin
                        case (cache_wen)
                                4'b0001: valid_array[0][req_pc[7:5]] <= 1'b1;
                                4'b0010: valid_array[1][req_pc[7:5]] <= 1'b1;
                                4'b0100: valid_array[2][req_pc[7:5]] <= 1'b1;
                                4'b1000: valid_array[3][req_pc[7:5]] <= 1'b1;
                                default: begin
                                        valid_array[0] <= valid_array[0];
                                        valid_array[1] <= valid_array[1];
                                        valid_array[2] <= valid_array[2];
                                        valid_array[3] <= valid_array[3];
                                end
                        endcase
                end
        end

        wire [23:0] rd_tag [3:0];
        wire [255:0] rd_data [3:0];
        wire [3:0] cache_wen;

        genvar way_i;
        generate 
                for (way_i = 0; way_i < `CACHE_WAY; way_i = way_i + 1) begin: tag_way
                        tag_array tag(
                                .clk(clk),
                                .waddr(req_pc[7:5]),
                                .raddr(req_pc[7:5]),
                                .wen(cache_wen[way_i] && current_state[6]),
                                .wdata(req_pc[31:8]),
                                .rdata(rd_tag[way_i])
                        );
                end
        endgenerate

        genvar way_j;
        generate 
                for (way_j = 0; way_j < `CACHE_WAY; way_j = way_j + 1) begin: data_way
                        data_array data(
                                .clk(clk),
                                .waddr(req_pc[7:5]),
                                .raddr(req_pc[7:5]),
                                .wen(cache_wen[way_j] && current_state[6]),
                                .wdata(write_data),
                                .rdata(rd_data[way_j])
                        );
                end
        endgenerate

        wire [2:0] cache_set_addr;
        wire [4:0] cache_in_addr;
        wire [23:0] cache_tag;

        assign {cache_tag, cache_set_addr, cache_in_addr} = req_pc;

        wire [3:0] valid_way = {valid_array[3][cache_set_addr], 
                                valid_array[2][cache_set_addr],
                                valid_array[1][cache_set_addr],
                                valid_array[0][cache_set_addr]};
        
        wire [3:0] hit_way = {rd_tag[3] == cache_tag && valid_way[3],
                              rd_tag[2] == cache_tag && valid_way[2],
                              rd_tag[1] == cache_tag && valid_way[1],
                              rd_tag[0] == cache_tag && valid_way[0]};

        wire [255:0] valid_data = {256{hit_way[0]}} & rd_data[0] 
                                | {256{hit_way[1]}} & rd_data[1]
                                | {256{hit_way[2]}} & rd_data[2]
                                | {256{hit_way[3]}} & rd_data[3]
                                | {256{read_miss }} & write_data;

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
        
        assign read_hit = |hit_way;
        assign read_miss = ~read_hit;
        wire [31:0] read_hit_data = {32{set_offset[0]}} & valid_data[ 31: 0]
                                  | {32{set_offset[1]}} & valid_data[ 63:32]
                                  | {32{set_offset[2]}} & valid_data[ 95:64]
                                  | {32{set_offset[3]}} & valid_data[127:96]
                                  | {32{set_offset[4]}} & valid_data[159:128]
                                  | {32{set_offset[5]}} & valid_data[191:160]
                                  | {32{set_offset[6]}} & valid_data[223:192]
                                  | {32{set_offset[7]}} & valid_data[255:224];
        
        // LRU-2
        // reg [1:0] visit_num [7:0][3:0];

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
                else if (current_state[7] && from_cpu_cache_rsp_ready) begin
                        case (hit_way)
                                4'b0001: begin
                                        visit_matrix[req_pc[7:5]][0] <= 4'b1110;
                                        visit_matrix[req_pc[7:5]][1] <= visit_matrix[req_pc[7:5]][1] & 4'b1110;
                                        visit_matrix[req_pc[7:5]][2] <= visit_matrix[req_pc[7:5]][2] & 4'b1110;
                                        visit_matrix[req_pc[7:5]][3] <= visit_matrix[req_pc[7:5]][3] & 4'b1110;
                                end
                                4'b0010: begin
                                        visit_matrix[req_pc[7:5]][0] <= visit_matrix[req_pc[7:5]][0] & 4'b1101;
                                        visit_matrix[req_pc[7:5]][1] <= 4'b1101;
                                        visit_matrix[req_pc[7:5]][2] <= visit_matrix[req_pc[7:5]][2] & 4'b1101;
                                        visit_matrix[req_pc[7:5]][3] <= visit_matrix[req_pc[7:5]][3] & 4'b1101;
                                end
                                4'b0100: begin
                                        visit_matrix[req_pc[7:5]][0] <= visit_matrix[req_pc[7:5]][0] & 4'b1011;
                                        visit_matrix[req_pc[7:5]][1] <= visit_matrix[req_pc[7:5]][1] & 4'b1011;
                                        visit_matrix[req_pc[7:5]][2] <= 4'b1011;
                                        visit_matrix[req_pc[7:5]][3] <= visit_matrix[req_pc[7:5]][3] & 4'b1011;
                                end
                                4'b1000: begin
                                        visit_matrix[req_pc[7:5]][0] <= visit_matrix[req_pc[7:5]][0] & 4'b0111;
                                        visit_matrix[req_pc[7:5]][1] <= visit_matrix[req_pc[7:5]][1] & 4'b0111;
                                        visit_matrix[req_pc[7:5]][2] <= visit_matrix[req_pc[7:5]][2] & 4'b0111;
                                        visit_matrix[req_pc[7:5]][3] <= 4'b0111;
                                end
                                default: begin
                                        visit_matrix[req_pc[7:5]][0] <= visit_matrix[req_pc[7:5]][0];
                                        visit_matrix[req_pc[7:5]][1] <= visit_matrix[req_pc[7:5]][1];
                                        visit_matrix[req_pc[7:5]][2] <= visit_matrix[req_pc[7:5]][2];
                                        visit_matrix[req_pc[7:5]][3] <= visit_matrix[req_pc[7:5]][3];
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

        assign cache_wen = empty_cache_addr;

endmodule
