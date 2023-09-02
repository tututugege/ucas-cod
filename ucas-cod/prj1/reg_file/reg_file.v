`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5
`define REG_NUM 32

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	// TODO: Please add your logic design here

	reg [`DATA_WIDTH - 1:0] rf [`REG_NUM - 1:0];

	always @(posedge clk) begin
		if (wen == 1'b1 && waddr != `ADDR_WIDTH'b0) begin
			rf[waddr] <= wdata;
		end
	end

	// Judgment signal for whether the address is 0
	wire not_zero_adder1 = | raddr1;
	wire not_zero_adder2 = | raddr2;

	assign rdata1 = {`DATA_WIDTH{not_zero_adder1}} & rf[raddr1];
	assign rdata2 = {`DATA_WIDTH{not_zero_adder2}} & rf[raddr2];

endmodule
