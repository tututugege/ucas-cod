`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
	input  [`DATA_WIDTH - 1:0] A,
	input  [              4:0] B,
	input  [              1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);
	// TODO: Please add your logic code here
        parameter SLL = 2'b00;
        parameter SRA = 2'b11;
        parameter SRL = 2'b10;

        wire op_sll = (Shiftop == SLL);
        wire op_sra = (Shiftop == SRA);
        wire op_srl = (Shiftop == SRL);

        wire [`DATA_WIDTH - 1:0] result_sll = A << B;
        wire [`DATA_WIDTH - 1:0] result_srl = A >> B;
        wire [`DATA_WIDTH - 1:0] result_sra = ($signed(A)) >>> B;

        assign Result = {{`DATA_WIDTH{op_sll}} & result_sll}
                      | {{`DATA_WIDTH{op_srl}} & result_srl}
                      | {{`DATA_WIDTH{op_sra}} & result_sra};

endmodule
