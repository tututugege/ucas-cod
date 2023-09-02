`timescale 10ns / 1ns

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/

// TODO: Please add your custom CPU code here

        assign inst_retire[69]    = RegWrite;
        assign inst_retire[68:64] = RF_waddr;
        assign inst_retire[63:32] = RF_wdata;
        assign inst_retire[31: 0] = PC;

        // IR**************************************
        wire [ 6:0] func7, opcode;
        wire [ 4:0] rs2, rs1, rd;
        wire [ 2:0] func3;
        wire [31:0] U_imm, B_imm, J_imm, S_imm, I_imm;
        reg  [31:0] IR;
        wire IRWrite;

        always @(posedge clk) begin
                if (IRWrite) begin
                        IR <= Instruction;
                end
                else begin
                        IR <= IR;
                end
        end
        assign {func7, rs2, rs1, func3, rd, opcode} = IR;
        assign I_imm = {{20{IR[31]}}, IR[31:20]};
        assign S_imm = {{20{IR[31]}}, IR[31:25], IR[11:7]};
        assign U_imm = {IR[31:12], 12'b0};
        assign B_imm = {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};
        assign J_imm = {{12{IR[31]}}, IR[19:12], IR[20], IR[30:21], 1'b0};

        // PC********************************************
        reg  [31:0] PC_reg;
        wire [31:0] PC_next, PC_jmp;
        wire PCWrite;
        wire branchValid = PCWriteCond && (Zero ^ func3[0] ^ func3[2]);

        assign PC = PC_reg;
        assign PC_next = {32{PCWriteCond && ~branchValid}} & PC_plus
                       | {32{(branchValid || ~PCWriteCond) && !JALR}} & ALUOut
                       | {32{(branchValid || ~PCWriteCond) &&  JALR}} & {ALUOut[31:1], 1'b0};
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

        reg  [31:0] PC_plus;
        always @(posedge clk) begin
                if (current_state[1])
                        PC_plus <= ALU_result;
                else
                        PC_plus <= PC_plus;
        end

        // RegFile******************************************************************
        wire [31:0] Result, RF_rdata1, RF_rdata2, RF_wdata;
        wire [ 4:0] RF_raddr1 = rs1;
        wire [ 4:0] RF_raddr2 = rs2;
        wire [ 4:0] RF_waddr  = rd;
        wire lui = U_type && opcode[5];

        assign Result = {32{ lui}}          & U_imm
                      | {32{link}}          & PC_plus
                      | {32{~lui && ~link}} & ALUOut;

        assign RF_wdata = {32{ MemtoReg}} & MDR
                        | {32{~MemtoReg}} & Result;

        reg_file my_rf(
                .clk(clk),
                .raddr1(RF_raddr1),
                .raddr2(RF_raddr2),
                .wen(RegWrite),
                .waddr(RF_waddr),
                .wdata(RF_wdata),
                .rdata1(RF_rdata1),
                .rdata2(RF_rdata2)
        );

        // ALU***********************************************************************
        wire func30 = func3[0] || R_type && func7[5];
        wire [2:0] ALUEx;
        assign ALUEx[0] = ~func3[2] &&  func3[1] ||  func3[2] && (func3[1] ^ func30);
        assign ALUEx[1] = ~func3[2];
        assign ALUEx[2] =  func3[2] && ~func3[1] || ~func3[2] && (func3[1] ^ func30);
        wire [2:0] ALUop = {3{current_state[3] || I_type_load || S_type || current_state [1]}} & 3'b010
                         | {3{current_state[4] && (R_type || I_type_cal)}}           & ALUEx
                         | {3{current_state[4] && B_type && ~func3[2]}}              & 3'b110
                         | {3{current_state[4] && B_type &&  func3[2] && ~func3[1]}} & 3'b111
                         | {3{current_state[4] && B_type &&  func3[2] &&  func3[1]}} & 3'b011;
        wire CarryOut, Overflow, Zero;
        wire [31:0] extenImm;
        wire [31:0] ALU_A, ALU_B, ALU_result, Imm;

        assign Imm = {32{I_type}} & I_imm
                   | {32{B_type}} & B_imm
                   | {32{S_type}} & S_imm
                   | {32{U_type}} & U_imm
                   | {32{J_type}} & J_imm;

        assign ALU_A = {32{~ALUSrcA}} & RF_rdata1
                     | {32{ ALUSrcA}} & PC;

        assign ALU_B = {32{ALUSrcB == 2'b00}} & RF_rdata2
                     | {32{ALUSrcB == 2'b01}} & 32'b100
                     | {32{ALUSrcB == 2'b10}} & Imm;

        alu my_ALU(
                .ALUop(ALUop),
                .A(ALU_A),
                .B(ALU_B),
                .Result(ALU_result),
                .Zero(Zero),
                .CarryOut(CarryOut),
                .Overflow(Overflow)
        );

        reg [31:0] ALUOut;
        always @(posedge clk) begin
                if (current_state[5] || current_state[6] || current_state[7])
                        ALUOut <= ALUOut;
                else
                        ALUOut <= {32{ ShifterValid &&  current_state[4]}} & Shifter_result
                                | {32{~ShifterValid || ~current_state[4]}} & ALU_result;
        end

        // shifter module ******************************************************************
        wire [31:0] Shifter_result, Shfter_A;
        wire [ 1:0] Shfter_op = {func3[2], func7[5]};
        wire [ 4:0] shamt = rs2;
        wire [ 4:0] Shfter_B = {5{ I_type}} & shamt
                             | {5{~I_type}} & ALU_B[4:0];
        wire ShifterValid = (I_type_cal || R_type) && ~func3[1] && func3[0];
        assign Shfter_A = ALU_A;

        shifter my_shifter(
                .A(Shfter_A),
                .B(Shfter_B),
                .Result(Shifter_result),
                .Shiftop(Shfter_op)
        );

        // memory load and store module *******************************************
        // load
        wire [3:0] offset;
        assign Address   = {ALUOut[31:2], 2'b0};
        assign offset[0] = (ALUOut[1:0] == 2'b00);
        assign offset[1] = (ALUOut[1:0] == 2'b01);
        assign offset[2] = (ALUOut[1:0] == 2'b10);
        assign offset[3] = (ALUOut[1:0] == 2'b11);

        // generate mask by op[1:0]
        wire [31:0] mask, offsetRdata, validRdata, MemRdata;
        assign mask[31:8] = {24{func3[1]}} & 24'hFFFFFF
                          | {24{func3[0]}} & 24'h0000FF; 
        assign mask[7:0] = 8'hFF;

        // get RdataSignal by op[0]
        wire [31:0] RdataSignal = {32{~func3[1] && ~func3[0]}} & {{24{validRdata[ 7]}}, 8'b0}
                                | {32{ func3[0]}}              & {{16{validRdata[15]}}, 16'b0};

        assign offsetRdata = {32{offset[0]}} & {Read_data}
                           | {32{offset[1]}} & { 8'b0, Read_data[31: 8]}
                           | {32{offset[2]}} & {16'b0, Read_data[31:16]}
                           | {32{offset[3]}} & {24'b0, Read_data[31:24]};
        assign validRdata = offsetRdata & mask;
        assign MemRdata = {32{ func3[2]}} &  validRdata
                        | {32{~func3[2]}} & (validRdata | RdataSignal);

        reg [31:0] MDR;
        always @(posedge clk) begin
                if (current_state[7] && Read_data_Ready && Read_data_Valid)
                        MDR <= MemRdata;
                else
                        MDR <= MDR;
        end

        // Store
        wire [31:0] validWdata, swl_data;
        wire [3:0] WriteWidth;
        assign WriteWidth[0] = 1'b1;
        assign WriteWidth[1] = func3[0] || func3[1];
        assign WriteWidth[3:2] = {2{func3[1]}};

        assign Write_strb = {4{offset[0]}} & {WriteWidth}
                          | {4{offset[1]}} & {WriteWidth[2:0], 1'b0}
                          | {4{offset[2]}} & {WriteWidth[1:0], 2'b0}
                          | {4{offset[3]}} & {WriteWidth[0]  , 3'b0};

        assign Write_data = {32{offset[0]}} & {RF_rdata2}
                          | {32{offset[1]}} & {RF_rdata2[23: 0],  8'b0}
                          | {32{offset[2]}} & {RF_rdata2[15: 0], 16'b0}
                          | {32{offset[3]}} & {RF_rdata2[ 7: 0], 24'b0};

        // CU*************************************************************************
        wire R_type = (opcode[6:4] == 3'b011);
        wire I_type = (opcode[6:5] == 2'b00) && (opcode[3:2] == 2'b00) || JALR;
        wire S_type = (opcode[6:4] == 3'b010);
        wire B_type = (opcode[6:2] == 5'b11000);
        wire U_type = (opcode[4:2] == 3'b101);
        wire J_type = (opcode[6:5] == 2'b11) && (opcode[3]);
        wire JALR   = (opcode == 7'b1100111);
        wire link   = (opcode[6:4] == 3'b110 && opcode[2]);

        wire I_type_load = I_type && ~opcode[4] && ~JALR;
        wire I_type_cal  = I_type &&  opcode[4];
        // FSM *****************************************************************
        reg [8:0] current_state;
        reg [8:0] next_state;
        localparam INIT = 9'b000000001,
                   IF   = 9'b000000010,
                   IW   = 9'b000000100,
                   ID   = 9'b000001000,
                   EX   = 9'b000010000,
                   ST   = 9'b000100000,
                   LD   = 9'b001000000,
                   RDW  = 9'b010000000,
                   WB   = 9'b100000000;

        // FSM 1.
        always @(posedge clk) begin
                if (~rst)
                        current_state <= next_state;
                else
                        current_state <= INIT;
        end

        // FSM 2.
        always @(*) begin
                case(current_state)
                        INIT  : next_state = IF;
                        IF    : begin
                                if (Inst_Req_Ready)
                                        next_state = IW;
                                else
                                        next_state = IF;
                        end
                        IW    : begin
                                if (Inst_Valid)
                                        next_state = ID;
                                else
                                        next_state = IW;
                        end
                        ID    : next_state = EX;
                        EX    : begin
                                if (B_type)
                                        next_state = IF;
                                else if (S_type)
                                        next_state = ST;
                                else if (I_type_load)
                                        next_state = LD;
                                else
                                        next_state = WB;
                        end
                        ST    : begin
                                if (Mem_Req_Ready)
                                        next_state = IF;
                                else 
                                        next_state = ST;
                        end
                        LD    : begin
                                if (Mem_Req_Ready)
                                        next_state = RDW;
                                else
                                        next_state = LD;
                        end
                        RDW   : begin
                                if (Read_data_Valid)
                                        next_state = WB;
                                else
                                        next_state = RDW;
                        end
                        WB    : next_state = IF;
                        default: next_state = INIT;
                endcase
        end

        // FSM 3
        wire PCWriteCond, ALUSrcA, RegWrite, MemtoReg, PCSource;
        wire [1:0] ALUSrcB;

        assign Inst_Req_Valid  = ~current_state[0] && current_state[1];
        assign Inst_Ready      =  current_state[0] || current_state[2];
        assign Read_data_Ready =  current_state[0] || current_state[7];

        assign PCWrite     = current_state[4];
        assign PCWriteCond = current_state[4] && B_type;
        assign MemRead     = current_state[6];
        assign MemWrite    = current_state[5];
        assign ALUSrcA     = current_state[4] && U_type || current_state[3] && (!JALR) || current_state[1];
        assign ALUSrcB[1]  = current_state[3] && (link || B_type) || current_state[4] && (I_type || S_type || U_type);
        assign ALUSrcB[0]  = current_state[3] && ~link && ~B_type || current_state[1];
        assign RegWrite    = current_state[8];
        assign IRWrite     = current_state[2] && Inst_Ready && Inst_Valid;
        assign MemtoReg    = current_state[8] && I_type_load;


        // counter ******************************************************************
        reg [31:0] cycle_cnt, IW_cnt, memR_cnt, memW_cnt, jmp_times, branch_times, branchValid_times, Inst_num, NOP_num;

        always @(posedge clk) begin
                if (rst) begin
                        cycle_cnt <= 32'b0;
                end
                else begin
                        cycle_cnt <= cycle_cnt + 32'b1;
                end
        end
        assign cpu_perf_cnt_0 = cycle_cnt;

        always @(posedge clk) begin
                if (rst) begin
                        IW_cnt <= 32'b0;
                end
                else if (current_state[2]) begin
                        IW_cnt <= IW_cnt + 32'b1;
                end
                else begin
                        IW_cnt <= IW_cnt;
                end
        end
        assign cpu_perf_cnt_1 = IW_cnt;

        always @(posedge clk) begin
                if (rst) begin
                        jmp_times <= 32'b0;
                end
                else if (link && current_state[3]) begin
                        jmp_times <= jmp_times + 32'b1;
                end
                else begin
                        jmp_times <= jmp_times;
                end
        end
        assign cpu_perf_cnt_2 = jmp_times;

        always @(posedge clk) begin
                if (rst) begin
                        branch_times <= 32'b0;
                end
                else if (B_type && current_state[3]) begin
                        branch_times <= branch_times + 32'b1;
                end
                else begin
                        branch_times <= branch_times;
                end
        end
        assign cpu_perf_cnt_3 = branch_times;

        always @(posedge clk) begin
                if (rst) begin
                        branchValid_times <= 32'b0;
                end
                else if (branchValid && current_state[4]) begin
                        branchValid_times <= branchValid_times + 32'b1;
                end
                else begin
                        branchValid_times <= branchValid_times;
                end
        end
        assign cpu_perf_cnt_4 = branchValid_times;

        always @(posedge clk) begin
                if (rst) begin
                        memR_cnt <= 32'b0;
                end
                else if (current_state[6] || current_state[7]) begin
                        memR_cnt <= memR_cnt + 32'b1;
                end
                else begin
                        memR_cnt <= memR_cnt;
                end
        end
        assign cpu_perf_cnt_5 = memR_cnt;

        always @(posedge clk) begin
                if (rst) begin
                        memW_cnt <= 32'b0;
                end
                else if (current_state[5]) begin
                        memW_cnt <= memW_cnt + 32'b1;
                end
                else begin
                        memW_cnt <= memW_cnt;
                end
        end
        assign cpu_perf_cnt_6 = memW_cnt;

        always @(posedge clk) begin
                if (rst) begin
                        Inst_num <= 32'b0;
                end
                else if (current_state[3]) begin
                        Inst_num <= Inst_num + 32'b1;
                end
                else begin
                        Inst_num <= Inst_num;
                end
        end
        assign cpu_perf_cnt_7 = Inst_num;

        wire NOP = (IR == 32'h00000013);

        always @(posedge clk) begin
                if (rst) begin
                        NOP_num <= 32'b0;
                end
                else if (current_state[3] && NOP) begin
                        NOP_num <= NOP_num + 32'b1;
                end
                else begin
                        NOP_num <= NOP_num;
                end
        end
        assign cpu_perf_cnt_8 = NOP_num;


endmodule


