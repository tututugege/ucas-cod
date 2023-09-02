`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	// TODO: Please add your logic design here
	parameter AND  = 3'b000;
	parameter OR   = 3'b001;
	parameter XOR  = 3'b100;
	parameter NOR  = 3'b101;
	parameter ADD  = 3'b010;
	parameter SUB  = 3'b110;
	parameter SLT  = 3'b111;
	parameter SLTU = 3'b011;
	parameter MSB = `DATA_WIDTH - 1;

	// a decoder to generate one-hot code
	wire op_and  = (ALUop == AND);
	wire op_or   = (ALUop == OR );
	wire op_xor  = (ALUop == XOR);
	wire op_nor  = (ALUop == NOR);
	wire op_add  = (ALUop == ADD);
	wire op_sub  = (ALUop == SUB);
	wire op_slt  = (ALUop == SLT);
	wire op_sltu = (ALUop == SLTU);

	wire [MSB:0] add_A, add_B; // two operands in adder
	wire [MSB:0] add_result;   // get the results of
	wire [MSB:0] slt_result;   // add/sub, slt, and, or
	wire [MSB:0] sltu_result;
	wire [MSB:0] and_result;
	wire [MSB:0] or_result;
	wire [MSB:0] nor_result;
	wire [MSB:0] xor_result;
	wire cin, cout; 	   // cin is used to select add or sub
	wire sign_A, sign_B; 	   // the signs of add_A, "add_B" and result
	wire sign_result;

	assign cin = ALUop[2] || op_sltu; 		// do subtraction if ALUop is SUB or SLT, that is ALUop[2] = 1
	assign add_B = (B ^ {32{cin}}); // If the cin is 1, add_B will be the inverse of B.
	assign add_A = A;
	assign sign_A = add_A[MSB]; 	// get the signs of add_A, "add_B" and result
	assign sign_B = add_B[MSB];
	assign sign_result = add_result[MSB];
	assign and_result = A & B;
	assign or_result  = A | B;
        assign nor_result = ~ or_result;
        assign xor_result = A ^ B;

	// create an adder with cin and cout
	assign {cout, add_result} = add_A + add_B + cin;

	// judge the result of slt from the signs
	// add_result is negetive if and only if
	// the sign of add_A and add_B(not B) is 1
	// or they have different signs(won't overflow) and
	// the sign of result is 1
	assign slt_result = {31'b0, (sign_A && sign_B) || ((sign_A ^ sign_B) && sign_result)};
	assign sltu_result = {31'b0, CarryOut};

	// a MUX to select which operation result we want to output
	assign Result = (and_result  & {`DATA_WIDTH{op_and}})
		      | (or_result   & {`DATA_WIDTH{op_or }})
		      | (nor_result  & {`DATA_WIDTH{op_nor}})
		      | (xor_result  & {`DATA_WIDTH{op_xor}})
		      | (add_result  & ({`DATA_WIDTH{op_add}} | {`DATA_WIDTH{op_sub}}))
		      | (slt_result  & {`DATA_WIDTH{op_slt}})
		      | (sltu_result & {`DATA_WIDTH{op_sltu}});

	// if CarryOut occurs, it is obvious that cout will be 1 for addition
	// for subtraction, carryout means A < B
	// because of B+~B = 2^k-1, A+~B+1 < 2^k, which means cout = 0
	assign CarryOut = cout ^ cin;

	// two positive operands are added to get a negetive result or
	// two negetive operands are added to get a positive result
	assign Overflow = (sign_A && sign_B && !sign_result) || (!sign_A && !sign_B && sign_result);

	assign Zero = ~(| Result);

endmodule
