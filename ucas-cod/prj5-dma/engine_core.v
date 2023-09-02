`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output [31:0]       src_base,
	output [31:0]       dest_base,
	output [31:0]       tail_ptr,
	output [31:0]       head_ptr,
	output [31:0]       dma_size,
	output [31:0]       ctrl_stat,

	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output              intr,
  
	output [31:0]       rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output [31:0]       wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);
	// TODO: Please add your logic design here
  
        wire EN = ctrl_stat[0];
        assign intr = ctrl_stat[31];
        reg [2:0] RD_current_state, RD_next_state;
        reg [2:0] WR_current_state, WR_next_state;

        localparam IDLE   = 3'b001;
        localparam RD_REQ = 3'b010;
        localparam RD     = 3'b100;
        localparam WR_REQ = 3'b010;
        localparam WR     = 3'b100;

        // FSM 1
        always @(posedge clk) begin
                if (rst) 
                        RD_current_state <= IDLE;
                else 
                        RD_current_state <= RD_next_state;
        end

        always @(posedge clk) begin
                if (rst) 
                        WR_current_state <= IDLE;
                else 
                        WR_current_state <= WR_next_state;
        end

        // FSM 2
        always @(*) begin
                case(RD_current_state) 
                        IDLE : begin
                                if (EN && head_ptr != tail_ptr && WR_current_state[0])
                                        RD_next_state = RD_REQ;
                                else
                                        RD_next_state = IDLE;
                        end
                        RD_REQ : begin
                                // if there is only rd_req_ready, memory will always waiting for valid signal
                                if (rd_req_ready && rd_req_valid) 
                                        RD_next_state = RD;
                                else if (rd_complete)
                                        RD_next_state = IDLE;
                                else
                                        RD_next_state = RD_REQ;
                        end
                        RD: begin
                                if (rd_valid && rd_last && !fifo_is_full)
                                        RD_next_state = RD_REQ;
                                else
                                        RD_next_state = RD;
                        end
                        default :
                                RD_next_state = IDLE;
                endcase
        end

        always @(*) begin
                case(WR_current_state) 
                        IDLE : begin
                                if (EN && RD_current_state[0] && head_ptr != tail_ptr)
                                        WR_next_state = WR_REQ;
                                else
                                        WR_next_state = IDLE;
                        end
                        WR_REQ : begin
                                // if there is only wr_req_ready, memory will always waiting for valid signal
                                if (wr_req_ready && wr_req_valid) 
                                        WR_next_state = WR;
                                else if (wr_complete)
                                        WR_next_state = IDLE;
                                else
                                        WR_next_state = WR_REQ;
                        end
                        WR: begin
                                if (wr_last)
                                        WR_next_state = WR_REQ;
                                else
                                        WR_next_state = WR;
                        end
                        default :
                                WR_next_state = IDLE;
                endcase
        end

        // the condition for starting dma
        wire start = RD_current_state[0] && WR_current_state[0] && EN && head_ptr != tail_ptr;

        // record the burst num
        reg [27:0] rd_counter;

        // to judge the last burst
        wire [27:0] rd_counter_plus = rd_counter + 1;
        always @(posedge clk) begin
                if (start || rst)
                        rd_counter <= 0;
                else if (rd_valid && rd_last && !fifo_is_full)
                        rd_counter <= rd_counter_plus;
                else
                        rd_counter <= rd_counter;
        end

        reg [27:0] wr_counter;
        wire [27:0] wr_counter_plus = wr_counter + 1;
        always @(posedge clk) begin
                if (start || rst)
                        wr_counter <= 0;
                else if (wr_last)
                        wr_counter <= wr_counter_plus;
                else
                        wr_counter <= wr_counter;
        end

        // to jugde the last data of wr
        reg [2:0] wr_last_counter;
        always @(posedge clk) begin
                if (rd_burst_start)
                        wr_last_counter <= wr_req_len[2:0];
                else if (wr_ready && wr_valid)
                        wr_last_counter <= wr_last_counter - 1;
                else
                        wr_last_counter <= wr_last_counter;
        end

        // calculate the tatal burst times and the last burst length
        wire [27:0] burst_num = dma_size[31:5] + (|dma_size[4:0]);
        wire [ 2:0] burst_last_len = dma_size[4:2] - {2'b0, !(|dma_size[2:0])};

        wire rd_complete = (rd_counter == burst_num) && (rd_counter != 0);
        wire wr_complete = (wr_counter == burst_num) && (wr_counter != 0);

        wire rd_burst_start = rd_req_ready && rd_req_valid;
        wire wr_burst_start = wr_req_ready && wr_req_valid;

        assign rd_req_addr = src_base + tail_ptr + {rd_counter, 5'b0};
        assign rd_req_valid = RD_current_state[1] && fifo_is_empty && !rd_complete;
        assign rd_req_len = (rd_counter_plus == burst_num) ? {2'b0, burst_last_len} : 5'b00111;
        assign rd_ready = RD_current_state[2] && !fifo_is_full;
        assign fifo_wdata = rd_rdata;
        assign fifo_wen = RD_current_state[2] && rd_valid && rd_ready;

        assign wr_req_addr = dest_base + tail_ptr + {wr_counter, 5'b0};
        assign wr_req_len = (wr_counter_plus == burst_num) ? {2'b0, burst_last_len} : 5'b00111;
        assign wr_req_valid = WR_current_state[1] && !fifo_is_empty && !wr_complete;
        assign wr_data = (last_fifo_rden) ? fifo_rdata : wr_data_reg;
        assign wr_valid = wr_valid_reg && WR_current_state[2];
        assign wr_last = WR_current_state[2] && (wr_last_counter == 0) && wr_valid;

        // get data from fifo only if the data in reg now is invalid or it is valid and will be written
        assign fifo_rden = wr_burst_start || WR_current_state[2] && ( ~wr_valid || wr_valid && wr_ready && !wr_last);

        reg wr_valid_reg;
        always @(posedge clk) begin
                if (rst) 
                        wr_valid_reg <= 0;
                // 上一周期拉高fifo读并且非空，读数据才有效
                else if (fifo_rden && !fifo_is_empty)
                        wr_valid_reg <= 1;
                // 上一周期数据被写走并且fifo读为低以及fifo空的时候无效
                else if (wr_valid && wr_ready && !fifo_rden || fifo_rden && fifo_is_empty)
                        wr_valid_reg <= 0;
                else
                        wr_valid_reg <= wr_valid_reg;
        end

        // record last_fifo_rden to judge whether fifo_rdata is valid
        reg last_fifo_rden;
        always @(posedge clk) begin
                last_fifo_rden <= fifo_rden;
        end

        // if fifo_rdata is valid but the memory is not ready, we need a register to record the data
        // because the data is only valid in one cycle
        reg [31:0] wr_data_reg;
        always @(posedge clk) begin
                if (rst)
                        wr_data_reg <= 32'b0;
                else if (last_fifo_rden)
                        wr_data_reg <= fifo_rdata;
                else
                        wr_data_reg <= wr_data_reg;
        end

        // six registers controlled by CPU
        reg [31:0] src_base_reg;
        always @(posedge clk) begin
                if (rst)
                        src_base_reg <= 32'b0;
                else if (reg_wr_en[0])
                        src_base_reg <= reg_wr_data;
                else
                        src_base_reg <= src_base_reg;
        end
        assign src_base = src_base_reg;

        reg [31:0] dest_base_reg;
        always @(posedge clk) begin
                if (rst)
                        dest_base_reg <= 32'b0;
                else if (reg_wr_en[1])
                        dest_base_reg <= reg_wr_data;
                else
                        dest_base_reg <= dest_base_reg;
        end
        assign dest_base = dest_base_reg;

        reg [31:0] tail_ptr_reg;
        always @(posedge clk) begin
                if (rst)
                        tail_ptr_reg <= 32'b0;
                else if (reg_wr_en[2])
                        tail_ptr_reg <= reg_wr_data;
                // wr_complete means a move is complete
                else if (WR_current_state[1] && wr_complete)
                        tail_ptr_reg <= tail_ptr_reg + dma_size;
                else
                        tail_ptr_reg <= tail_ptr_reg;
        end
        assign tail_ptr = tail_ptr_reg;

        reg [31:0] head_ptr_reg;
        always @(posedge clk) begin
                if (rst)
                        head_ptr_reg <= 32'b0;
                else if (reg_wr_en[3])
                        head_ptr_reg <= reg_wr_data;
                else
                        head_ptr_reg <= head_ptr_reg;
        end
        assign head_ptr = head_ptr_reg;

        reg [31:0] dma_size_reg;
        always @(posedge clk) begin
                if (rst)
                        dma_size_reg <= 32'b0;
                else if (reg_wr_en[4])
                        dma_size_reg <= reg_wr_data;
                else
                        dma_size_reg <= dma_size_reg;
        end
        assign dma_size = dma_size_reg;

        reg [31:0] ctrl_stat_reg;
        always @(posedge clk) begin
                if (rst)
                        ctrl_stat_reg <= 32'b1;
                else if (reg_wr_en[5])
                        ctrl_stat_reg <= reg_wr_data;
                // send a intr when completing a move
                else if (WR_current_state[1] && wr_complete)
                        ctrl_stat_reg <= {1'b1, ctrl_stat_reg[30:0]};
        end
        assign ctrl_stat = {ctrl_stat_reg[31], src_base[29:0], ctrl_stat_reg[0]};

endmodule

