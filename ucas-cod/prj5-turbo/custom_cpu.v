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

        assign inst_retire[69] = RF_wen;
        assign inst_retire[68:64] = RF_waddr;
        assign inst_retire[63:32] = RF_wdata;
        assign inst_retire[31: 0] = RDW_WB_PC;

        wire Go = IF_complete && IW_complete && RDW_complete && MEM_complete;

        // IF ******************************************************************

        // determine if the request has been completed
        reg IF_complete_reg;
        always @(posedge clk) begin
                if (rst || Go)
                        IF_complete_reg <= 1'b0;
                else if (Inst_Req_Ready && Inst_Req_Valid)
                        IF_complete_reg <= 1'b1;
        end
        wire IF_complete = IF_complete_reg || load_hazard_control;

        assign Inst_Req_Valid = ~rst && ~IF_complete && ~Inst_Ready;

        reg  [31:0] PC_reg;
        wire [31:0] PC_plus = PC + 4;
        wire PCWrite;

	always @(posedge clk) begin
		if (rst) begin
			PC_reg <= 32'b0;
		end
                else if (Go && (load_hazard_control))
                        PC_reg <= PC_reg;
                else if (Go || branchValid && ~load_hazard_control || jump) begin
                        PC_reg <= PC_plus;
                end
	end

        assign PC = {32{~branchValid && ~jump || branchValid && load_hazard_control}} & PC_reg
                  | {32{ branchValid && ~load_hazard_control ||  jump}} & PC_branch;

        reg [31:0] IF_IW_PC;
        always @(posedge clk) begin
                if (rst)
                        IF_IW_PC <= 32'b0;
                else if (Go && (load_hazard_control))
                        IF_IW_PC <= IF_IW_PC;
                else if (Go)
                        IF_IW_PC <= PC;
        end

        // IW ******************************************************************
        // localparam INIT = 3'b001,        
        //            WAIT = 3'b010,        
        //            CPLT = 3'b100;        

        // reg [2:0] IW_current_state;
        // reg [2:0] IW_next_state;
        // always @(posedge clk) begin
        //         // pull up Inst_Ready when rst
        //         if (rst)      
        //                 IW_current_state <= 3'b1;
        //         else
        //                 IW_current_state <= IW_next_state;
        // end

        // always @(*) begin
        //         case (IW_current_state) 
        //                 INIT: begin
        //                         if (Inst_Req_Ready && Inst_Req_Valid)
        //                                 IW_next_state = WAIT;
        //                         else
        //                                 IW_next_state = INIT;
        //                 end
        //                 WAIT: begin
        //                         if (Inst_Ready && Inst_Valid)
        //                                 IW_next_state = CPLT;
        //                         else
        //                                 IW_next_state = WAIT;
        //                 end
        //                 CPLT: begin
        //                         if (~stop)
        //                                 IW_next_state = WAIT;
        //                         else
        //                                 IW_next_state = CPLT;
        //                 end
        //                 default :       IW_next_state = INIT;
        //         endcase
        // end

        reg IW_start;
        always @(posedge clk) begin
                if (rst)
                        IW_start <= 1'b0;
                else if (Go)
                        IW_start <= 1'b1;
        end

        reg IW_complete_reg;
        always @(posedge clk) begin
                if (rst || Go && ~load_hazard_control)
                        IW_complete_reg <= 1'b0;
                else if (Inst_Ready && Inst_Valid)
                        IW_complete_reg <= 1'b1;
        end
        wire IW_complete = IW_complete_reg || ~IW_start;

        assign Inst_Ready = IW_start && ~IW_complete_reg || rst;

        reg [31:0] IR;
        always @(posedge clk) begin
                if (rst)
                        IR <= 32'b0;
                else if (Inst_Ready && Inst_Valid)
                        IR <= Instruction;
        end

        // store instruction
        reg [31:0] IW_ID_IR;
        always @(posedge clk) begin
                if (rst)
                        IW_ID_IR <= 32'b0;
                else if ((load_hazard_control) && Go)
                        IW_ID_IR <= IW_ID_IR;
                else if ((branchValid || jump) && Go) 
                        IW_ID_IR <= 32'b0;
                else if (Inst_Ready && Inst_Valid && Go)
                        IW_ID_IR <= Instruction;
                else if (Go)
                        IW_ID_IR <= IR;
        end

        // store PC to calculate jump addr or AUIPC
        reg [31:0] IW_ID_PC;
        always @(posedge clk) begin
                if (rst)
                        IW_ID_PC <= 32'b0;
                else if ((load_hazard_control) && Go)
                        IW_ID_PC <= IW_ID_PC;
                else if (Go)
                        IW_ID_PC <= IF_IW_PC;
        end

        // ID ********************************************************************
        wire [ 6:0] func7, opcode;
        wire [ 4:0] rs2, rs1, rd;
        wire [ 2:0] func3;
        wire [31:0] U_imm, B_imm, J_imm, S_imm, I_imm;
        wire IRWrite;
        wire func30 = func3[0] || R_type && func7[5];
        wire [2:0] ALUEx;
        wire R_type = (opcode[6:4] == 3'b011);
        wire I_type = (opcode[6:5] == 2'b00) && (opcode[3:1] == 3'b001) || JALR;
        wire S_type = (opcode[6:4] == 3'b010);
        wire B_type = (opcode[6:2] == 5'b11000);
        wire U_type = (opcode[4:2] == 3'b101);
        wire J_type = (opcode[6:5] == 2'b11) && (opcode[3]);
        wire JALR   = (opcode == 7'b1100111);
        wire MUL    = R_type && func7[0];
        wire link   = (opcode[6:4] == 3'b110 && opcode[2]);

        wire I_type_load = I_type && ~opcode[4] && ~JALR;
        wire I_type_cal  = I_type &&  opcode[4];

        assign {func7, rs2, rs1, func3, rd, opcode} = IW_ID_IR;
        assign I_imm = {{20{IW_ID_IR[31]}}, IW_ID_IR[31:20]};
        assign S_imm = {{20{IW_ID_IR[31]}}, IW_ID_IR[31:25], IW_ID_IR[11:7]};
        assign U_imm = {IW_ID_IR[31:12], 12'b0};

        wire [31:0] PC_imm;
        assign B_imm = {{20{IW_ID_IR[31]}}, IW_ID_IR[7], IW_ID_IR[30:25], IW_ID_IR[11:8], 1'b0};
        assign J_imm = {{12{IW_ID_IR[31]}}, IW_ID_IR[19:12], IW_ID_IR[20], IW_ID_IR[30:21], 1'b0};
        assign PC_imm = {32{B_type}} & B_imm
                      | {32{J_type}} & J_imm;

        // check branch and jump immediately after receiving instructions
        // so we must solve data hazard

        wire branch = B_type;
        wire cout;
        wire [31:0] ID_result;
        assign {cout, ID_result} = RF_rdata1_valid + ~RF_rdata2_valid + 1'b1;

        wire ID_slt = RF_rdata1_valid[31] && ~RF_rdata2_valid[31] || ((RF_rdata1_valid[31] ^ ~RF_rdata2_valid[31]) && ID_result[31]);
	wire ID_sltu = cout;
        wire ID_Zero = ~(|ID_result);
        wire branchValid = branch && (~func3[2] && (ID_Zero ^ func3[0]) || func3[2] && ~func3[1] && (ID_slt ^ func3[0]) || func3[1] && (ID_sltu ^ func3[0]));
        wire jump = J_type || JALR;
        wire [31:0] PC_JALR = RF_rdata1_valid + I_imm;
        wire [31:0] PC_branch = (JALR) ? {PC_JALR[31:1], 1'b0} : IW_ID_PC + PC_imm;


        assign ALUEx[0] = ~func3[2] &&  func3[1] ||  func3[2] && (func3[1] ^ func30);
        assign ALUEx[1] = ~func3[2];
        assign ALUEx[2] =  func3[2] && ~func3[1] || ~func3[2] && (func3[1] ^ func30);
        wire [2:0] ALUop = {3{R_type || I_type_cal}}  & ALUEx
                         | {3{I_type_load || S_type || J_type || JALR}} & 3'b010;
        
        // four cases: EX/MEM forward, RDW/WB forward, PC and rdata1
        // four cases: EX/MEM forward, RDW/WB forward, Imm and rdata2
        // assign ALUSrcB[1]  = (link || B_type) || I_type || S_type || U_type;

        // RegFile
        wire [31:0] RF_rdata1, RF_rdata2, RF_wdata;
        wire RF_wen;
        wire [ 4:0] RF_raddr1 = rs1;
        wire [ 4:0] RF_raddr2 = rs2;
        wire [ 4:0] RF_waddr;
        wire lui = U_type && opcode[5];

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
        
        reg [31:0] ID_EX_rdata1, ID_EX_rdata2, ID_EX_PC, ID_EX_Imm;
        reg [ 4:0] ID_EX_rd;
        wire PCWriteCond, MemtoReg, PCSource, ALUSrcA, ALUSrcB;
        reg [4:0] ID_EX_rs1, ID_EX_rs2; // two index of read registers to deal with forwarding
        reg ID_EX_ALU_SrcA, ID_EX_ALU_SrcB, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_RegWrite, ID_EX_PCSrc, ID_EX_ShifterOp;
        reg [2:0] ID_EX_ALUOp, ID_EX_func3;
        reg [1:0] ID_EX_ALUOutSrc;

        reg MemWrite_delay;

        always @(posedge clk) begin
                if (rst)
                        MemWrite_delay <= 1'b1;
                else if (S_type && Go)
                        MemWrite_delay <= 1'b0;
                else if (Go)
                        MemWrite_delay <= 1'b1;
        end

        // control signal
        // EX control signal *********

        // always @(posedge clk) begin
        //         if (rst)
        //                 ID_EX_jump <= 1'b0;
        //         else if (!stop)
        //                 ID_EX_jump <= J_type;
        // end

        // always @(posedge clk) begin
        //         if (rst)
        //                 ID_EX_branch <= 1'b0;
        //         else if (!stop)
        //                 ID_EX_branch <= B_type;
        // end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_func3 <= 3'b0;
                else if (Go)
                        ID_EX_func3 <= func3;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_ALUOp <= 3'b0;
                else if (Go)
                        ID_EX_ALUOp <= ALUop;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_ShifterOp <= 1'b0;
                else if (Go)
                        ID_EX_ShifterOp <= func7[5];
        end

        wire ShifterValid = (I_type_cal || R_type) && ~func3[1] && func3[0];
        always @(posedge clk) begin
                if (rst)
                        ID_EX_ALUOutSrc <= 2'b0;
                else if (Go)
                        ID_EX_ALUOutSrc <= {MUL || lui, ShifterValid || lui};
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_ALU_SrcA <= 1'b0;
                else if (Go)
                        ID_EX_ALU_SrcA <= U_type || jump;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_ALU_SrcB <= 1'b0;
                else if (Go)
                        ID_EX_ALU_SrcB <= R_type;
        end

        // forward control signal **********
        always @(posedge clk) begin
                if (rst)
                        ID_EX_rs1 <= 5'b0;
                else if (Go)
                        ID_EX_rs1 <= rs1;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_rs2 <= 5'b0;
                else if (Go)
                        ID_EX_rs2 <= rs2;
        end

        // memory control signal ****************
        always @(posedge clk) begin
                if (rst)
                        ID_EX_MemRead <= 1'b0;
                else if (Go && load_hazard_control)
                        ID_EX_MemRead <= 1'b0;
                else if (Go)
                        ID_EX_MemRead <= I_type_load;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_MemWrite <= 1'b0;
                else if (Go && load_hazard_control)
                        ID_EX_MemWrite <= 1'b0;
                else if (Go)
                        ID_EX_MemWrite <= S_type;
        end

        // WB control signal *****************
        wire RegWrite = R_type || I_type_cal || I_type_load || J_type || JALR || U_type;
        always @(posedge clk) begin
                if (rst)
                        ID_EX_RegWrite <= 1'b0;
                else if (Go && load_hazard_control)
                        ID_EX_RegWrite <= 1'b0;
                else if (Go)
                        ID_EX_RegWrite <= RegWrite;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_PCSrc <= 1'b0;
                else if (Go)
                        ID_EX_PCSrc <= J_type || JALR;
        end

        // PC and rdata, Imm
        always @(posedge clk) begin
                if (rst)
                        ID_EX_PC <= 32'b0;
                else if (Go)
                        ID_EX_PC <= IW_ID_PC;
        end

        // four possible cases can lead to data hazard
        // one-hot

        // to judge whether raddr1 == waddr or raddr2 == waddr;
        wire data1_hazard_rw, data2_hazard_rw;
        assign data1_hazard_rw = (rs1 == RF_waddr) && RDW_WB_RegWrite && |rs1;
        assign data2_hazard_rw = (rs2 == RF_waddr) && RDW_WB_RegWrite && |rs2;

        wire [3:0] data1_hazard, data2_hazard;

        assign data1_hazard[2:0] = {rs1 == MEM_RDW_rd && MEM_RDW_RegWrite && |rs1,
                                    rs1 == EX_MEM_rd  && EX_MEM_RegWrite  && |rs1,
                                    rs1 == ID_EX_rd   && ID_EX_RegWrite   && |rs1};
        assign data1_hazard[3] = (data1_hazard[2:0] == 3'b0);

        assign data2_hazard[2:0] = {rs2 == MEM_RDW_rd && MEM_RDW_RegWrite && |rs2,
                                    rs2 == EX_MEM_rd  && EX_MEM_RegWrite  && |rs2,
                                    rs2 == ID_EX_rd   && ID_EX_RegWrite   && |rs2};
        assign data2_hazard[3] = (data2_hazard[2:0] == 3'b0);

        // the data hazard could not be solved by forwarding
        // read a reg which will be written by a load instruction
        wire [2:0] load_hazard;
        assign load_hazard[2] = MEM_RDW_MemRead && (MEM_RDW_rd == rs1 && |rs1 || MEM_RDW_rd == rs2 && |rs2);
        assign load_hazard[1] = EX_MEM_MemRead  && (EX_MEM_rd  == rs1 && |rs1 || EX_MEM_rd  == rs2 && |rs2);
        assign load_hazard[0] = ID_EX_MemRead   && (ID_EX_rd   == rs1 && |rs1 || ID_EX_rd   == rs2 && |rs2);
        wire load_hazard_control = |load_hazard || (MemWrite_delay && S_type);

        reg load_hazard2;
        always @(posedge clk) begin
                if (rst || load_hazard2 && Go)
                        load_hazard2 <= 1'b0;
                else if (load_hazard[1] && Go)
                        load_hazard2 <= 1'b1;
        end

        wire [31:0] RF_rdata1_valid, RF_rdata2_valid;

        assign RF_rdata1_valid = {32{data1_hazard[0]}} & Result
                               | {32{data1_hazard[1:0] == 2'b10}} & EX_MEM_ALUOut
                               | {32{data1_hazard[2:0] == 3'b100}} & MEM_RDW_ALUOut
                               | {32{data1_hazard[3] &&  data1_hazard_rw}} & RDW_WB_MDR
                               | {32{data1_hazard[3] && ~data1_hazard_rw}} & RF_rdata1;

        assign RF_rdata2_valid = {32{data2_hazard[0]}} & Result
                               | {32{data2_hazard[1:0] == 2'b10}} & EX_MEM_ALUOut
                               | {32{data2_hazard[2:0] == 3'b100}} & MEM_RDW_ALUOut
                               | {32{data2_hazard[3] &&  data2_hazard_rw}} & RDW_WB_MDR
                               | {32{data2_hazard[3] && ~data2_hazard_rw}} & RF_rdata2;


        always @(posedge clk) begin
                if (rst)
                        ID_EX_rdata1 <= 32'b0;
                else if (Go)
                        ID_EX_rdata1 <= RF_rdata1_valid;
        end

        always @(posedge clk) begin
                if (rst)
                        ID_EX_rdata2 <= 32'b0;
                else if (Go)
                        ID_EX_rdata2 <= RF_rdata2_valid;
        end

        assign Imm = {32{I_type}} & I_imm
                   | {32{S_type}} & S_imm
                   | {32{U_type}} & U_imm
                   | {32{jump  }} & 32'b100;

        always @(posedge clk) begin
                if (rst)
                        ID_EX_Imm <= 32'b0;
                else if (Go)
                        ID_EX_Imm <= Imm;
        end

        // store waddr used in WR stage
        always @(posedge clk) begin
                if (rst)
                        ID_EX_rd <= 5'b0;
                else if (Go)
                        ID_EX_rd <= rd;
        end

        // EX ***********************************************************************************
        // ALU

        wire CarryOut, Overflow, Zero;
        wire [31:0] ALU_A, ALU_B, ALU_result, Imm;

        // the data hazard could be solved by forwarding

        assign ALU_A = {32{~ID_EX_ALU_SrcA}} & ID_EX_rdata1
                     | {32{ ID_EX_ALU_SrcA}} & ID_EX_PC;

        assign ALU_B = {32{ ID_EX_ALU_SrcB}} & ID_EX_rdata2
                     | {32{~ID_EX_ALU_SrcB}} & ID_EX_Imm;

        alu val_ALU(
                .ALUop(ID_EX_ALUOp),
                .A(ALU_A),
                .B(ALU_B),
                .Result(ALU_result),
                .Zero(Zero),
                .CarryOut(CarryOut),
                .Overflow(Overflow)
        );
        wire [31:0] mul_result = RF_rdata1 * RF_rdata2;

        // Directly determine whether to branch or jump

        // shifter
        wire [31:0] Shifter_result, Shfter_A;
        wire [ 4:0] Shfter_B = {5{~ID_EX_ALU_SrcB}} & ID_EX_rs2
                             | {5{ ID_EX_ALU_SrcB}} & ID_EX_rdata2[4:0];
        assign Shfter_A = ID_EX_rdata1;

        shifter my_shifter(
                .A(Shfter_A),
                .B(Shfter_B),
                .Result(Shifter_result),
                .Shiftop({ID_EX_func3[2], ID_EX_ShifterOp})
        );

        reg [31:0] EX_MEM_ALUOut, EX_MEM_rdata2, EX_MEM_PC;
        reg [ 4:0] EX_MEM_rd, EX_MEM_rs1, EX_MEM_rs2;
        reg EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_PCSrc;
        reg [ 2:0] EX_MEM_func3;

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_PC <= 32'b0;
                else if (Go)
                        EX_MEM_PC <= ID_EX_PC;
        end

        // forward control signal **********
        always @(posedge clk) begin
                if (rst)
                        EX_MEM_rs1 <= 5'b0;
                else if (Go)
                        EX_MEM_rs1 <= ID_EX_rs1;
        end

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_rs2 <= 5'b0;
                else if (Go)
                        EX_MEM_rs2 <= ID_EX_rs2;
        end

        // memory control signal ***********
        always @(posedge clk) begin
                if (rst)
                        EX_MEM_MemRead <= 1'b0;
                else if (Go)
                        EX_MEM_MemRead <= ID_EX_MemRead;
        end

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_MemWrite <= 1'b0;
                else if (Go)
                        EX_MEM_MemWrite <= ID_EX_MemWrite;
        end

        // WB control signal *****************
        always @(posedge clk) begin
                if (rst)
                        EX_MEM_func3 <= 3'b0;
                else if (Go)
                        EX_MEM_func3 <= ID_EX_func3;
        end

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_RegWrite <= 1'b0;
                else if (Go)
                        EX_MEM_RegWrite <= ID_EX_RegWrite;
        end

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_PCSrc <= 1'b0;
                else if (Go)
                        EX_MEM_PCSrc <= ID_EX_PCSrc;
        end

        // PC_branch, ALUOut, wdata and waddr

        wire [31:0] Result = {32{ID_EX_ALUOutSrc == 2'b00}} & ALU_result
                           | {32{ID_EX_ALUOutSrc == 2'b01}} & Shifter_result
                           | {32{ID_EX_ALUOutSrc == 2'b10}} & mul_result
                           | {32{ID_EX_ALUOutSrc == 2'b11}} & ID_EX_Imm;

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_ALUOut <= 32'b0;
                else if (Go)
                        EX_MEM_ALUOut <= Result;
        end

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_rdata2 <= 32'b0;
                else if (Go)
                        EX_MEM_rdata2 <= ID_EX_rdata2;
        end

        always @(posedge clk) begin
                if (rst)
                        EX_MEM_rd <= 5'b0;
                else if (Go)
                        EX_MEM_rd <= ID_EX_rd;
        end


        // MEM *********************************************************************
        // load
        reg MEM_complete_reg;
        always @(posedge clk) begin
                if (rst || Go)
                        MEM_complete_reg <= 1'b0;
                else if ((EX_MEM_MemRead|| EX_MEM_MemWrite) && Mem_Req_Ready)
                        MEM_complete_reg <= 1'b1;
        end

        wire MEM_complete = ~EX_MEM_MemRead && ~EX_MEM_MemWrite || MEM_complete_reg;
        assign MemRead = EX_MEM_MemRead && ~MEM_complete_reg && ~rst;
        assign MemWrite = EX_MEM_MemWrite && ~MEM_complete_reg && ~rst;
        assign Address   = {EX_MEM_ALUOut[31:2], 2'b0};

        wire [3:0] offset;
        assign offset[0] = (EX_MEM_ALUOut[1:0] == 2'b00);
        assign offset[1] = (EX_MEM_ALUOut[1:0] == 2'b01);
        assign offset[2] = (EX_MEM_ALUOut[1:0] == 2'b10);
        assign offset[3] = (EX_MEM_ALUOut[1:0] == 2'b11);

        wire [3:0] Write_width;
        assign Write_width[0] = 1'b1;
        assign Write_width[1] = EX_MEM_func3[0] || EX_MEM_func3[1];
        assign Write_width[3:2] = {2{EX_MEM_func3[1]}};

        // Store
        assign Write_strb = {4{offset[0]}} & {Write_width}
                          | {4{offset[1]}} & {Write_width[2:0], 1'b0}
                          | {4{offset[2]}} & {Write_width[1:0], 2'b0}
                          | {4{offset[3]}} & {Write_width[0]  , 3'b0};

        assign Write_data = {32{offset[0]}} & {EX_MEM_rdata2}
                          | {32{offset[1]}} & {EX_MEM_rdata2[23: 0],  8'b0}
                          | {32{offset[2]}} & {EX_MEM_rdata2[15: 0], 16'b0}
                          | {32{offset[3]}} & {EX_MEM_rdata2[ 7: 0], 24'b0};

        reg [31:0] MEM_RDW_ALUOut, MEM_RDW_PC;
        reg [ 4:0] MEM_RDW_rd;
        reg MEM_RDW_RegWrite, MEM_RDW_MemRead;
        reg [ 2:0] MEM_RDW_func3;
        reg [ 3:0] MEM_RDW_offset;

        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_offset <= 4'b0;
                else if (Go)
                        MEM_RDW_offset <= offset;
        end
                
        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_PC <= 32'b0;
                else if (Go)
                        MEM_RDW_PC <= EX_MEM_PC;
        end

        // WB control signal *****************
        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_func3 <= 3'b0;
                else if (Go)
                        MEM_RDW_func3 <= EX_MEM_func3;
        end

        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_RegWrite <= 1'b0;
                else if (Go)
                        MEM_RDW_RegWrite <= EX_MEM_RegWrite;
        end

        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_rd <= 5'b0;
                else if (Go)
                        MEM_RDW_rd <= EX_MEM_rd;
        end

        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_ALUOut <= 32'b0;
                else if (Go)
                        MEM_RDW_ALUOut <= EX_MEM_ALUOut;
        end

        always @(posedge clk) begin
                if (rst)
                        MEM_RDW_MemRead <= 1'b0;
                else if (Go)
                        MEM_RDW_MemRead <= EX_MEM_MemRead;
        end

        // RDW ******************************************************************
        reg RDW_complete_reg;
        always @(posedge clk) begin
                if (rst || Go)
                        RDW_complete_reg <= 1'b0;
                else if (Read_data_Ready && Read_data_Valid)
                        RDW_complete_reg <= 1'b1;
        end
        assign Read_data_Ready = MEM_RDW_MemRead && ~RDW_complete_reg || rst;
        wire RDW_complete = ~MEM_RDW_MemRead || RDW_complete_reg;

        reg [31:0] RDW_WB_MDR, RDW_WB_PC;
        reg [ 4:0] RDW_WB_rd;
        reg RDW_WB_RegWrite;
        reg [31:0] MDR;

        // generate mask by op[1:0]
        wire [31:0] mask, offsetRdata, validRdata, MemRdata;
        assign mask[31:8] = {24{MEM_RDW_func3[1]}} & 24'hFFFFFF
                          | {24{MEM_RDW_func3[0]}} & 24'h0000FF; 
        assign mask[7:0] = 8'hFF;

        // get RdataSignal by op[0]
        wire [31:0] RdataSignal = {32{~MEM_RDW_func3[1] && ~MEM_RDW_func3[0]}} & {{24{validRdata[ 7]}}, 8'b0}
                                | {32{ MEM_RDW_func3[0]}}                     & {{16{validRdata[15]}}, 16'b0};

        assign offsetRdata = {32{MEM_RDW_offset[0]}} & {Read_data}
                           | {32{MEM_RDW_offset[1]}} & { 8'b0, Read_data[31: 8]}
                           | {32{MEM_RDW_offset[2]}} & {16'b0, Read_data[31:16]}
                           | {32{MEM_RDW_offset[3]}} & {24'b0, Read_data[31:24]};
        assign validRdata = offsetRdata & mask;
        assign MemRdata = {32{ MEM_RDW_func3[2]}} &  validRdata
                        | {32{~MEM_RDW_func3[2]}} & (validRdata | RdataSignal);

        always @(posedge clk) begin
                if (rst)
                        MDR <= 32'b0;
                else if (Read_data_Ready && Read_data_Valid)
                        MDR <= MemRdata;
        end

        always @(posedge clk) begin
                if (rst)
                        RDW_WB_PC <= 32'b0;
                else if (Go)
                        RDW_WB_PC <= MEM_RDW_PC;
        end

        always @(posedge clk) begin
                if (rst)
                        RDW_WB_rd <= 5'b0;
                else if (Go)
                        RDW_WB_rd <= MEM_RDW_rd;
        end

        always @(posedge clk) begin
                if (rst)
                        RDW_WB_MDR <= 32'b0;
                else if (~MEM_RDW_MemRead && Go)
                        RDW_WB_MDR <= MEM_RDW_ALUOut;
                else if (RDW_complete_reg && Go)
                        RDW_WB_MDR <= MDR;
                else if (Read_data_Ready && Read_data_Valid && Go)
                        RDW_WB_MDR <= MemRdata;
        end

        always @(posedge clk) begin
                if (rst)
                        RDW_WB_RegWrite <= 1'b0; 
                else if (Go) 
                        RDW_WB_RegWrite <= MEM_RDW_RegWrite;
        end

        // WB ******************************************************************************
        assign RF_wdata = RDW_WB_MDR;
        assign RF_waddr = RDW_WB_rd;
        assign RF_wen = RF_wen_reg;

        reg RF_wen_reg;
        always @(posedge clk) begin
                if (rst)
                        RF_wen_reg <= 1'b0;
                else if (Go)
                        RF_wen_reg <= MEM_RDW_RegWrite;
                else
                        RF_wen_reg <= 1'b0;
        end


        // CU*************************************************************************


        // FSM *****************************************************************
        // reg [8:0] current_state;
        // reg [8:0] next_state;
        // localparam INIT = 9'b000000001,
        //            IF   = 9'b000000010,
        //            IW   = 9'b000000100,
        //            ID   = 9'b000001000,
        //            EX   = 9'b000010000,
        //            ST   = 9'b000100000,
        //            LD   = 9'b001000000,
        //            RDW  = 9'b010000000,
        //            WB   = 9'b100000000;

        // // FSM 1.
        // always @(posedge clk) begin
        //         if (~rst)
        //                 current_state <= next_state;
        //         else
        //                 current_state <= INIT;
        // end

        // // FSM 2.
        // always @(*) begin
        //         case(current_state)
        //                 INIT  : next_state = IF;
        //                 IF    : begin
        //                         if (Inst_Req_Ready)
        //                                 next_state = IW;
        //                         else
        //                                 next_state = IF;
        //                 end
        //                 IW    : begin
        //                         if (Inst_Valid)
        //                                 next_state = ID;
        //                         else
        //                                 next_state = IW;
        //                 end
        //                 ID    : next_state = EX;
        //                 EX    : begin
        //                         if (B_type)
        //                                 next_state = IF;
        //                         else if (S_type)
        //                                 next_state = ST;
        //                         else if (I_type_load)
        //                                 next_state = LD;
        //                         else
        //                                 next_state = WB;
        //                 end
        //                 ST    : begin
        //                         if (Mem_Req_Ready)
        //                                 next_state = IF;
        //                         else 
        //                                 next_state = ST;
        //                 end
        //                 LD    : begin
        //                         if (Mem_Req_Ready)
        //                                 next_state = RDW;
        //                         else
        //                                 next_state = LD;
        //                 end
        //                 RDW   : begin
        //                         if (Read_data_Valid)
        //                                 next_state = WB;
        //                         else
        //                                 next_state = RDW;
        //                 end
        //                 WB    : next_state = IF;
        //                 default: next_state = INIT;
        //         endcase
        // end

        // // FSM 3


        // counter ******************************************************************
        // reg [31:0] cycle_cnt, IW_cnt, memR_cnt, memW_cnt, jmp_times, branch_times, branchValid_times, Inst_num, NOP_num;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 cycle_cnt <= 32'b0;
        //         end
        //         else begin
        //                 cycle_cnt <= cycle_cnt + 32'b1;
        //         end
        // end
        // assign cpu_perf_cnt_0 = cycle_cnt;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 IW_cnt <= 32'b0;
        //         end
        //         else if (current_state[2]) begin
        //                 IW_cnt <= IW_cnt + 32'b1;
        //         end
        //         else begin
        //                 IW_cnt <= IW_cnt;
        //         end
        // end
        // assign cpu_perf_cnt_1 = IW_cnt;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 jmp_times <= 32'b0;
        //         end
        //         else if (link && current_state[3]) begin
        //                 jmp_times <= jmp_times + 32'b1;
        //         end
        //         else begin
        //                 jmp_times <= jmp_times;
        //         end
        // end
        // assign cpu_perf_cnt_2 = jmp_times;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 branch_times <= 32'b0;
        //         end
        //         else if (B_type && current_state[3]) begin
        //                 branch_times <= branch_times + 32'b1;
        //         end
        //         else begin
        //                 branch_times <= branch_times;
        //         end
        // end
        // assign cpu_perf_cnt_3 = branch_times;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 branchValid_times <= 32'b0;
        //         end
        //         else if (branchValid && current_state[4]) begin
        //                 branchValid_times <= branchValid_times + 32'b1;
        //         end
        //         else begin
        //                 branchValid_times <= branchValid_times;
        //         end
        // end
        // assign cpu_perf_cnt_4 = branchValid_times;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 memR_cnt <= 32'b0;
        //         end
        //         else if (current_state[6] || current_state[7]) begin
        //                 memR_cnt <= memR_cnt + 32'b1;
        //         end
        //         else begin
        //                 memR_cnt <= memR_cnt;
        //         end
        // end
        // assign cpu_perf_cnt_5 = memR_cnt;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 memW_cnt <= 32'b0;
        //         end
        //         else if (current_state[5]) begin
        //                 memW_cnt <= memW_cnt + 32'b1;
        //         end
        //         else begin
        //                 memW_cnt <= memW_cnt;
        //         end
        // end
        // assign cpu_perf_cnt_6 = memW_cnt;

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 Inst_num <= 32'b0;
        //         end
        //         else if (current_state[3]) begin
        //                 Inst_num <= Inst_num + 32'b1;
        //         end
        //         else begin
        //                 Inst_num <= Inst_num;
        //         end
        // end
        // assign cpu_perf_cnt_7 = Inst_num;

        // wire NOP = (IW_ID_IR == 32'h00000013);

        // always @(posedge clk) begin
        //         if (rst) begin
        //                 NOP_num <= 32'b0;
        //         end
        //         else if (current_state[3] && NOP) begin
        //                 NOP_num <= NOP_num + 32'b1;
        //         end
        //         else begin
        //                 NOP_num <= NOP_num;
        //         end
        // end
        // assign cpu_perf_cnt_8 = NOP_num;

        // reg [31:0] intr_reg;
        // always @(posedge clk) begin
        //         intr_reg <= Instruction;
        // end


endmodule



