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

        // split the instruction IR in to diffrent parts
        wire [ 5:0] op, func;
        wire [ 4:0] rs, rt, rd, sa;
        wire [15:0] imm = {rd, sa, func};
        wire [25:0] instr_index = {rs, rt, rd, sa, func};
        wire R_type, J_type, REGIMM;
        assign {op, rs, rt, rd, sa, func} = IR;

        // PC module *****************************************************************

        // the PC result of branch instruction
        wire branchValid = (REGIMM             & (Zero ^ ~rt[0])
                         | (~op[1] && ~REGIMM) & (Zero ^ op[0])
                         | op[1]               & (Zero & (|RF_rdata1) ^ ~op[0])) & PCWriteCond ;

        // the PC result of J-type instruction or jr and jalr
        wire [31:0] PC_jmp = {32{ J_type}} & {PC[31:28], instr_index, 2'b00}
                           | {32{~J_type}} & RF_rdata1;
        // select the next PC
        reg  [31:0] PC_reg;
        wire [31:0] PC_next;

        assign PC = PC_reg;
        assign PC_next = {32{PCsource == 2'b01}} & ALUOut
                       | {32{PCsource == 2'b00}} & ALU_result
                       | {32{PCsource == 2'b10}} & PC_jmp;

	always @(posedge clk) begin
		if (rst) begin
			PC_reg <= 32'b0;
		end
                else if (PCWrite || branchValid) begin
                        PC_reg <= PC_next;
                end
                else begin
                        PC_reg <= PC_reg;
                end
	end

        // IR
        reg [31:0] IR;
        always @(posedge clk) begin
                if (IRWrite) begin
                        IR <= Instruction;
                end
                else begin
                        IR <= IR;
                end
        end

        // MDR
        reg [31:0] MDR;
        always @(posedge clk) begin
                MDR <= MemRdata;
        end

        // CU module *****************************************************************


        // generate this control signals and some signals about the type of instruction
        wire RegDst, ALUSrcA, RegWrite, MemtoReg, IRWrite, PCWrite, PCWriteCond, lui, ImmDst;
        wire [2:0] ALUcontrol, ALUop;
        wire [1:0] ALUSrcB, PCsource;
        wire NOP = ~(|IR);
        // FSM *****************************************************************;
        CU my_CU(
                .IRWrite(IRWrite),
                .NOP(NOP),
                .PCsource(PCsource),
                .jmp_r(jmp_r),
                .clk(clk),
                .rst(rst),
                .PCWrite(PCWrite),
                .PCWriteCond(PCWriteCond),
                .J_type(J_type),
                .R_type(R_type),
                .REGIMM(REGIMM),
                .op(op),
                .RegDst(RegDst),
                .ALUSrcA(ALUSrcA),
                .ALUSrcB(ALUSrcB),
                .MemtoReg(MemtoReg),
                .RegWrite(RegWrite),
                .MemRead(MemRead),
                .MemWrite(MemWrite),
                .ALUop(ALUop),
                .link(link),
                .lui(lui),
                .ImmDst(ImmDst)
        );

        // mov
        wire mov = R_type && (func[5:1] == 5'b00101);
        wire movValid = ((|RF_rdata2) ^ ~func[0]) && mov;
        // link
        wire link = J_type && op[0] || R_type && (func == 6'b001001);
        // jmp
        wire jmp = J_type || R_type && jmp_r;
        wire jmp_r = (func[5:1] == 5'b00100);

        // Alu module ***********************************************************************************
        ALU_controler my_ALU_controler(
                .func(func),
                .ALUop(ALUop),
                .ALUcontrol(ALUcontrol)
        );
        // judge if we should select ALU_result to write in register
        wire ALUValid = (R_type && func[5] || ~op[5] && op[3]) && ~lui;
        wire CarryOut, Overflow, Zero;
        wire [31:0] extenImm = (op[2]) ? {16'b0, imm} : {{16{imm[15]}}, imm};
        wire [31:0] ALU_A, ALU_B, ALU_result;

        assign ALU_A = {32{ ALUSrcA}} & A
                     | {32{~ALUSrcA}} & PC;

        assign ALU_B = {32{ALUSrcB == 2'b00}} & B
                     | {32{ALUSrcB == 2'b01}} & 32'b100
                     | {32{ALUSrcB == 2'b10}} & extenImm
                     | {32{ALUSrcB == 2'b11}} & {{14{imm[15]}}, imm, 2'b0};

        alu my_ALU(
                .ALUop(ALUcontrol),
                .A(ALU_A),
                .B(ALU_B),
                .Result(ALU_result),
                .Zero(Zero),
                .CarryOut(CarryOut),
                .Overflow(Overflow)
        );

        reg [31:0] ALUOut;
        always @(posedge clk) begin
                ALUOut <= ALU_result;
        end

        // shifter module ******************************************************************
        wire [31:0] Shifter_result, Shfter_A;
        wire [ 1:0] Shfter_op = func[1:0];
        wire [ 4:0] Shfter_B = {5{ saValid}} & sa
                             | {5{~saValid}} & ALU_A[4:0];
        wire ShifterValid = R_type & ~func[5] & ~func[3] && ~lui;
        wire saValid = ~func[2];
        assign Shfter_A = ALU_B;

        shifter my_shifter(
                .A(Shfter_A),
                .B(Shfter_B),
                .Result(Shifter_result),
                .Shiftop(Shfter_op)
        );

        reg [31:0] ShifterOut;
        always @(posedge clk) begin
                ShifterOut <= Shifter_result;
        end

        // RegFile module **********************************************************************
        wire [31:0] Result, RF_rdata1, RF_rdata2;
        wire [4:0] RF_raddr1 = rs;
        wire [4:0] RF_raddr2 = {5{ REGIMM}} & 5'b0
                             | {5{~REGIMM}} & rt;

        assign RF_waddr = {5{RegDst && ~mov || movValid}} & rd
                        | {5{ImmDst}}                     & rt
                        | {5{J_type && op[0]}}            & {5'd31};

        assign RF_wdata = {32{ MemtoReg}} & MDR
                        | {32{~MemtoReg}} & Result;

        assign Result = {32{ShifterValid}}     & ShifterOut
                      | {32{lui}}              & {imm, 16'b0}
                      | {32{movValid}}         & A
                      | {32{ALUValid || link}} & ALUOut;

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

        reg [31:0] A, B;
        always @(posedge clk) begin
                A <= RF_rdata1;
                B <= RF_rdata2;
        end

        // memory load and store module *******************************************
        // load
        wire [3:0] offset;
        assign Address = {ALUOut[31:2], 2'b0};
        assign offset[0] = (ALUOut[1:0] == 2'b00);
        assign offset[1] = (ALUOut[1:0] == 2'b01);
        assign offset[2] = (ALUOut[1:0] == 2'b10);
        assign offset[3] = (ALUOut[1:0] == 2'b11);

        // generate mask by op[1:0]
        wire [31:0] mask, validRdata;
        wire wr = (op[2:1] == 2'b11);
        wire wl = op[2:0] == 3'b010;
        wire ailgn = ~(op[1:0] == 2'b10);
        assign mask[ 7: 0] =  8'hFF;
        assign mask[15: 8] =  8'hFF   & { 8{op[0]}};
        assign mask[31:16] = 16'hFFFF & {16{op[1]}};

        wire [31:0] MemRdata, extenMemRdata, lwr_data, lwl_data;
        assign validRdata = lwr_data & mask;

        // get RdataSignal by op[0]
        wire [31:0] RdataSignal = {32{~op[0]}} & {{24{validRdata[ 7]}},  8'b0}
                                | {32{ op[0]}} & {{16{validRdata[15]}}, 16'b0};

        assign extenMemRdata = {32{op[1] || op[2]}} &  validRdata
                             | {32{~op[1] && ~op[2]}} & (validRdata | RdataSignal);

        assign lwl_data = {32{offset[0]}} & {Read_data[ 7: 0],B[23:0]}
                        | {32{offset[1]}} & {Read_data[15: 0],B[15:0]}
                        | {32{offset[2]}} & {Read_data[23: 0],B[ 7:0]}
                        | {32{offset[3]}} & {Read_data[31: 0]};

        assign lwr_data = {32{offset[0]}} & {Read_data}
                        | {32{offset[1]}} & {B[31:24], Read_data[31: 8]}
                        | {32{offset[2]}} & {B[31:16], Read_data[31:16]}
                        | {32{offset[3]}} & {B[31: 8], Read_data[31:24]};

        assign MemRdata = {32{wl  }} & lwl_data
                        | {32{wr  }} & lwr_data
                        | {32{ailgn}} & extenMemRdata;

        // Store
        wire [31:0] validWdata, swl_data;
        wire [3:0] WriteWidth, Write_strb_l, Write_strb_r, Write_strb_ailgn;
        assign WriteWidth[0] = 1'b1;
        assign WriteWidth[1] = op[0] || op[1];
        assign WriteWidth[3:2] = {2{op[1]}} & 2'b11;

        assign Write_strb_ailgn = {4{offset[0]}} & {WriteWidth}
                                | {4{offset[1]}} & {WriteWidth[2:0], 1'b0}
                                | {4{offset[2]}} & {WriteWidth[1:0], 2'b0}
                                | {4{offset[3]}} & {WriteWidth[0]  , 3'b0};

        assign Write_strb_l[0] = 1'b1;
        assign Write_strb_l[1] = ALUOut[0] || ALUOut[1];
        assign Write_strb_l[2] = ALUOut[1];
        assign Write_strb_l[3] = ALUOut[0] && ALUOut[1];

        assign Write_strb_r = {Write_strb_l[0], ~Write_strb_l[3], ~Write_strb_l[2], ~Write_strb_l[1]};

        assign Write_strb = {4{wl   }} & Write_strb_l
                          | {4{wr   }} & Write_strb_r
                          | {4{ailgn}} & Write_strb_ailgn;

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

module ALU_controler(
        input [2:0] ALUop,
        input [5:0] func,
        output [2:0] ALUcontrol
);
        wire R_type = (ALUop == 3'b101);
        wire [2:0] funcOP;
        assign funcOP[2] =  func[1] && (~func[3] || ~func[0]);
        assign funcOP[1] = ~func[2] && (~func[3] || func[1]);
        assign funcOP[0] =  func[2] && func[0] || func[3] && func[1];

        assign ALUcontrol = {3{ R_type}} & funcOP
                          | {3{~R_type}} & ALUop;

endmodule

module CU(
        input  [5:0] op,
        input  clk, jmp_r, NOP, rst,
        output RegDst, ALUSrcA, MemtoReg, RegWrite, MemRead, MemWrite, PCWrite, PCWriteCond, IRWrite,
        output link, lui, ImmDst,
        output J_type, R_type, REGIMM,
        output [2:0] ALUop,
        output [1:0] ALUSrcB, PCsource
);
        // FSM *****************************************************************
        reg [4:0] current_state;
        reg [4:0] next_state;
        localparam IF  = 5'b00001,
                   ID  = 5'b00010,
                   EX  = 5'b00100,
                   MEM = 5'b01000,
                   WB  = 5'b10000;

        // FSM 1.
        always @(posedge clk) begin
                if (~rst)
                        current_state <= next_state;
                else
                        current_state <= IF;
        end

        // FSM 2.
        always @(*) begin
                case(current_state)
                        IF : next_state = ID;
                        ID : begin
                                if (NOP) begin
                                        next_state = IF;
                                end
                                else begin
                                        next_state = EX;
                                end
                        end
                        EX : begin
                                if (J_type && ~link || REGIMM || I_type_branch) begin
                                        next_state = IF;
                                end
                                else if (op[5]) begin
                                        next_state = MEM;
                                end
                                else begin
                                        next_state = WB;
                                end
                        end
                        MEM: begin
                                if (op[3]) begin
                                        next_state = IF;
                                end
                                else begin
                                        next_state = WB;
                                end
                        end
                        WB : next_state = IF;
                        default: next_state = IF;
                endcase
        end

        wire I_type_branch = (op[5:2] == 4'b0001);
        wire jmp = R_type && jmp_r || J_type;
        wire branch = REGIMM || I_type_branch;
        assign J_type = (op[5:1] == 5'b00001);
        assign R_type = ~(|op);
        assign REGIMM = (op == 6'b000001);
        assign lui = (op == 6'b001111);

        // FSM 3.
        assign PCWrite     = current_state[0] || current_state[2] && jmp;
        assign PCWriteCond = current_state[2] && (REGIMM || I_type_branch);
        assign MemRead     = current_state[3] && ~op[3];
        assign MemWrite    = current_state[3] &&  op[3];
        assign PCsource[1] = current_state[2] && jmp;
        assign PCsource[0] = current_state[2] && I_type_branch;
        assign ALUSrcB[1]  = current_state[3] || current_state[2] && (op[3] || op[5]) || current_state[1] && branch;
        assign ALUSrcB[0]  = current_state[3] || current_state[0] || current_state[2] && link || current_state[1] && branch;
        assign ALUSrcA     = (current_state[2] && ~link) || (current_state[1] && ~branch);
        assign RegWrite    = current_state[4];
        assign IRWrite     = current_state[0];
        assign MemtoReg    = current_state[4] && op[5] && ~op[3];
        assign RegDst      = current_state[4] && R_type;
        assign ImmDst      = current_state[4] && (op[5] || op[3]);

        wire [2:0] ALUEx;

        assign ALUEx[2] = ~op[5] && (~op[3] || op[1] && ~op[0]);
        assign ALUEx[1] = op[5] || (op[3] ^ op[2]) || REGIMM;
        assign ALUEx[0] = R_type || REGIMM || ~op[5] && (~op[2] && op[1] || ~op[3] && op[2] && op[1] || op[3] && op[2] && op[0]);

        assign ALUop = (current_state[0] || current_state[1] || current_state[2] && link) ? 3'b010
                                                                                          : ALUEx;

endmodule