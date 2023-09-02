`timescale 10ns / 1ns

module simple_cpu(
	input             clk,
	input             rst,

	output [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output            MemRead
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;
        wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	// TODO: PLEASE ADD YOUR CODE BELOW

        // split the instruction in to diffrent parts
        wire [ 5:0] op, func;
        wire [ 4:0] rs, rt, rd, sa;
        wire [15:0] imm = {rd, sa, func};
        wire [25:0] instr_index = {rs, rt, rd, sa, func};
        wire R_type, J_type, REGIMM;
        assign {op, rs, rt, rd, sa, func} = Instruction;

        // PC module *****************************************************************
        wire [31:0] PC_plus = PC + 4;

        // the PC result of branch instruction
        wire [31:0] PC_branch = PC_plus + {{14{imm[15]}}, imm, 2'b00};;
        wire branchValid = (REGIMM             & (Zero ^ ~rt[0])
                         | (~op[1] && ~REGIMM) & (Zero ^ op[0])
                         | op[1]               & (Zero & (|RF_rdata1) ^ ~op[0])) & branch ;

        // the PC result of J-type instruction or jr and jalr
        wire [31:0] PC_jmp = {32{ J_type}} & {PC[31:28], instr_index, 2'b00}
                           | {32{~J_type}} & RF_rdata1;
        // select the next PC
	wire [31:0] PC_next = {32{branchValid}}          & PC_branch
                            | {32{jmp}}                  & PC_jmp
                            | {32{~jmp && ~branchValid}} & PC_plus;
        PC_mod my_PC(
                .clk(clk),
                .rst(rst),
                .PC_next(PC_next),
                .PC(PC)
        );

        // CU module *****************************************************************
        // generate this control signals and some signals about the type of instruction
        wire RegDst, ALUSrc, RegWrite, MemtoReg, branch, lui, ImmDst;
        wire [2:0] ALUOp, ALUop0;
        CU my_CU(
                .J_type(J_type),
                .R_type(R_type),
                .REGIMM(REGIMM),
                .op(op),
                .RegDst(RegDst),
                .ALUSrc(ALUSrc),
                .MemtoReg(MemtoReg),
                .branch(branch),
                .RegWrite(RegWrite),
                .MemRead(MemRead),
                .MemWrite(MemWrite),
                .ALUop0(ALUop0),
                .link(link),
                .lui(lui),
                .ImmDst(ImmDst)
        );

        // mov
        wire mov;
        wire movValid = ((|RF_rdata2) ^ ~func[0]) && mov;
        assign mov = R_type && (func[5:1] == 5'b00101);
        // link
        wire link = J_type && op[0] || R_type && (func == 6'b001001);
        // jmp
        wire jmp = J_type || R_type && (func[5:1] == 5'b00100);

        // Alu module ***********************************************************************************
        ALU_controler my_ALU_controler(
                .func(func),
                .ALUop0(ALUop0),
                .ALUOp(ALUOp)
        );
        // judge if we should select ALU_result to write in register
        wire ALUValid = (R_type && func[5] || ~op[5] && op[3]) && ~lui;
        wire CarryOut, Overflow, Zero;
        wire [31:0] ALU_A, ALU_B, ALU_result;

        assign ALU_A = RF_rdata1;
        assign ALU_B = {32{~ALUSrc}} & RF_rdata2
                     | {32{ ALUSrc && ~op[2]}} & {{16{imm[15]}}, imm}
                     | {32{ ALUSrc &&  op[2]}} & {16'b0, imm};

        alu my_ALU(
                .ALUop(ALUOp),
                .A(ALU_A),
                .B(ALU_B),
                .Result(ALU_result),
                .Zero(Zero),
                .CarryOut(CarryOut),
                .Overflow(Overflow)
        );

        // shifter module ******************************************************************
        wire [31:0] Shfter_result, Shfter_A;
        wire [ 1:0] Shfter_op = func[1:0];
        wire [ 4:0] Shfter_B = {5{ saValid}} & sa
                             | {5{~saValid}} & ALU_A[4:0];
        wire ShfterValid = R_type & ~func[5] & ~func[3] && ~lui;
        wire saValid = ~func[2];
        assign Shfter_A = ALU_B;

        shifter my_shifter(
                .A(Shfter_A),
                .B(Shfter_B),
                .Result(Shfter_result),
                .Shiftop(Shfter_op)
        );

        // RegFile module **********************************************************************
        wire [31:0] Result, RF_rdata1, RF_rdata2;
        wire [4:0] RF_raddr1 = rs;
        wire [4:0] RF_raddr2 = {5{ REGIMM}} & 5'b0
                             | {5{~REGIMM}} & rt;

        assign RF_waddr = {5{RegDst && ~mov || movValid}} & rd
                        | {5{ImmDst}}                     & rt
                        | {5{J_type && op[0]}}            & {5'd31};

        assign RF_wdata = {32{ MemtoReg}} & MemRdata
                        | {32{~MemtoReg}} & Result;

        assign Result = {32{ShfterValid}} & Shfter_result
                      | {32{link}}        & PC + 8
                      | {32{lui}}         & {imm, 16'b0}
                      | {32{movValid}}    & RF_rdata1
                      | {32{ALUValid}}    & ALU_result;

        wire jr = R_type && (func == 6'b001000);
        assign RF_wen = RegWrite && ~jr && ~mov || movValid;
        reg_file my_rf(
                .clk(clk),
                .raddr1(RF_raddr1),
                .raddr2(RF_raddr2),
                .wen(RF_wen),
                .waddr(RF_waddr),
                .wdata(RF_wdata),
                .rdata1(RF_rdata1),
                .rdata2(RF_rdata2)
        );

        // memory load and store module *******************************************
        // load
        wire [3:0] offset;
        assign Address = {ALU_result[31:2], 2'b0};
        assign offset[0] = (ALU_result[1:0] == 2'b00);
        assign offset[1] = (ALU_result[1:0] == 2'b01);
        assign offset[2] = (ALU_result[1:0] == 2'b10);
        assign offset[3] = (ALU_result[1:0] == 2'b11);

        // generate mask by op[1:0]
        wire [31:0] mask, validRdata;
        wire wr = (op[2:1] == 2'b11);
        wire wl = op[2:0] == 3'b010;
        wire align = ~(op[1:0] == 2'b10);
        assign mask[ 7: 0] =  8'hFF;
        assign mask[15: 8] =  8'hFF   & { 8{op[0]}};
        assign mask[31:16] = 16'hFFFF & {16{op[1]}};

        wire [31:0] MemRdata, extenMemRdata, lwr_data, lwl_data;
        assign validRdata = lwr_data & mask;

        // get RdataSignal by op[0]
        wire [31:0] RdataSignal = {32{~op[0]}} & {{24{validRdata[ 7]}},  8'b0}
                                | {32{ op[0]}} & {{16{validRdata[15]}}, 16'b0};

        assign extenMemRdata = {32{op[1] || op[2]}}   &  validRdata
                             | {32{~op[1] && ~op[2]}} & (validRdata | RdataSignal);

        assign lwl_data = {32{offset[0]}} & {Read_data[ 7: 0],RF_rdata2[23:0]}
                        | {32{offset[1]}} & {Read_data[15: 0],RF_rdata2[15:0]}
                        | {32{offset[2]}} & {Read_data[23: 0],RF_rdata2[ 7:0]}
                        | {32{offset[3]}} & {Read_data[31: 0]};

        assign lwr_data = {32{offset[0]}} & {Read_data}
                        | {32{offset[1]}} & {RF_rdata2[31:24], Read_data[31: 8]}
                        | {32{offset[2]}} & {RF_rdata2[31:16], Read_data[31:16]}
                        | {32{offset[3]}} & {RF_rdata2[31: 8], Read_data[31:24]};

        assign MemRdata = {32{wl  }} & lwl_data
                        | {32{wr  }} & lwr_data
                        | {32{align}} & extenMemRdata;

        // Store
        wire [31:0] validWdata, swl_data;
        wire [3:0] WriteWidth, Write_strb_l, Write_strb_r, Write_strb_align;
        assign WriteWidth[0] = 1'b1;
        assign WriteWidth[1] = op[0] || op[1];
        assign WriteWidth[3:2] = {2{op[1]}} & 2'b11;

        assign Write_strb_align = {4{offset[0]}} & {WriteWidth}
                                | {4{offset[1]}} & {WriteWidth[2:0], 1'b0}
                                | {4{offset[2]}} & {WriteWidth[1:0], 2'b0}
                                | {4{offset[3]}} & {WriteWidth[0]  , 3'b0};

        assign Write_strb_l[0] = 1'b1;
        assign Write_strb_l[1] = ALU_result[0] || ALU_result[1];
        assign Write_strb_l[2] = ALU_result[1];
        assign Write_strb_l[3] = ALU_result[0] && ALU_result[1];

        assign Write_strb_r = {Write_strb_l[0], ~Write_strb_l[3], ~Write_strb_l[2], ~Write_strb_l[1]};

        assign Write_strb = {4{wl   }} & Write_strb_l
                          | {4{wr   }} & Write_strb_r
                          | {4{align}} & Write_strb_align;

        assign validWdata = {32{offset[0]}} & {RF_rdata2}
                          | {32{offset[1]}} & {RF_rdata2[23: 0],  8'b0}
                          | {32{offset[2]}} & {RF_rdata2[15: 0], 16'b0}
                          | {32{offset[3]}} & {RF_rdata2[ 7: 0], 24'b0};

        assign swl_data = {32{offset[0]}} & {24'b0, RF_rdata2[31:24]}
                        | {32{offset[1]}} & {16'b0, RF_rdata2[31:16]}
                        | {32{offset[2]}} & { 8'b0, RF_rdata2[31: 8]}
                        | {32{offset[3]}} & RF_rdata2;

        assign Write_data = {32{ wl}} & swl_data
                          | {32{~wl}} & validWdata;

endmodule

module PC_mod(
        input clk,
        input rst,
        input [31:0] PC_next,
        output [31:0] PC
);

        reg [31:0] PC_reg;

        assign PC = PC_reg;
	always @(posedge clk) begin
		if (rst) begin
			PC_reg <= 32'b0;
		end
                else begin
                        PC_reg <= PC_next;
                end
	end

endmodule

module ALU_controler(
        input [2:0] ALUop0,
        input [5:0] func,
        output [2:0] ALUOp
);
        wire R_type = (ALUop0 == 3'b101);
        wire [2:0] funcOP;
        assign funcOP[2] =  func[1] && (~func[3] || ~func[0]);
        assign funcOP[1] = ~func[2] && (~func[3] || func[1]);
        assign funcOP[0] =  func[2] && func[0] || func[3] && func[1];

        assign ALUOp = {3{ R_type}} & funcOP
                     | {3{~R_type}} & ALUop0;

endmodule

module CU(
        input [5:0] op,
        output RegDst, ALUSrc, MemtoReg, branch, RegWrite, MemRead, MemWrite,
        output link, lui, ImmDst,
        output J_type, R_type, REGIMM,
        output [2:0] ALUop0
);

        wire I_type_branch = (op[5:2] == 4'b0001);

        assign J_type = (op[5:1] == 5'b00001);
        assign R_type = ~(|op);
        assign REGIMM = (op == 6'b000001);

        assign lui = (op == 6'b001111);
        assign RegDst   = R_type;
        assign ImmDst =  ALUSrc;
        assign ALUSrc   = op[5] | op[3];
        assign MemtoReg = op[5];
        assign RegWrite = (op[5] ^ op[3]) || R_type || (J_type && op[0]);
        assign branch   = I_type_branch | REGIMM;
        assign MemRead  = op[5] & ~op[3];
        assign MemWrite = op[5] &  op[3];

        // decode to generate ALUop0
        // 110表示Rtype运算类型 111表示branch(slt) 010表示l和s(add)
        // 010表示立即数加 000表示立即数与 001表示立即数或 100表示立即数异或
        // 111表示立即数比较 011表示无符号立即数比较
        // 即只要不是110就按给定的ALUop输出 否则需要结合funct进一步译码
        assign ALUop0[2] = ~op[5] && (~op[3] || op[1] && ~op[0]);
        assign ALUop0[1] = op[5] || (op[3] ^ op[2]) || REGIMM;
        assign ALUop0[0] = R_type || REGIMM || ~op[5] && (~op[2] && op[1] || ~op[3] && op[2] && op[1] || op[3] && op[2] && op[0]);

endmodule