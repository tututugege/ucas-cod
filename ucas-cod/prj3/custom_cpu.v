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
        assign inst_retire[69]    = RF_wen;
        assign inst_retire[68:64] = RF_waddr;
        assign inst_retire[63:32] = RF_wdata;
        assign inst_retire[31: 0] = PC;
        wire			RF_wen;
        wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

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
                // Change the MDR only when the handshake is successful
                if (current_state[7] && Read_data_Ready && Read_data_Valid)
                        MDR <= MemRdata;
                else
                        MDR <= MDR;
        end

        // Alu module ***********************************************************************************
        ALU_controler my_ALU_controler(
                .func(func),
                .ALUop(ALUop),
                .ALUcontrol(ALUcontrol)
        );
        wire CarryOut, Overflow, Zero;
        wire [31:0] extenImm = (op[2]) ? {16'b0, imm} : {{16{imm[15]}}, imm};
        wire [31:0] ALU_A, ALU_B, ALU_result;

        assign ALU_A = {32{ ALUSrcA}} & RF_rdata1
                     | {32{~ALUSrcA}} & PC;

        assign ALU_B = {32{ALUSrcB == 2'b00}} & RF_rdata2
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
                // ALUOut is unchanged while waiting for memory response
                if (current_state[5] || current_state[6] || current_state[7])
                        ALUOut <= ALUOut;
                else
                        ALUOut <= {32{~ShifterValid}} & ALU_result
                                | {32{ ShifterValid}} & Shifter_result;
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

        assign Result = {32{lui}}               & {imm, 16'b0}
                      | {32{movValid}}          & RF_rdata1
                      | {32{~lui && ~movValid}} & ALUOut;

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
        assign Address   = {ALUOut[31:2], 2'b0};
        assign offset[0] = (ALUOut[1:0] == 2'b00);
        assign offset[1] = (ALUOut[1:0] == 2'b01);
        assign offset[2] = (ALUOut[1:0] == 2'b10);
        assign offset[3] = (ALUOut[1:0] == 2'b11);

        // generate mask by op[1:0]
        // the mask set the invaild high bits to 0
        wire [31:0] mask, validRdata;
        wire wr = (op[2:1] == 2'b11);
        wire wl = op[2:0] == 3'b010;
        wire ailgn = ~(op[1:0] == 2'b10);
        assign mask[ 7: 0] =  8'hFF;
        assign mask[15: 8] =  8'hFF   & { 8{op[0]}};
        assign mask[31:16] = 16'hFFFF & {16{op[1]}};

        wire [31:0] MemRdata, extenMemRdata, lwr_data, lwl_data;
        // the low valid bits of the data of lwr and lw, lh, lb is the same 
        // we just need to mask the high invalid bits of lwr_data
        assign validRdata = lwr_data & mask;
        // get RdataSignal by op[0]
        wire [31:0] RdataSignal = {32{~op[0]}} & {{24{validRdata[ 7]}},  8'b0}
                                | {32{ op[0]}} & {{16{validRdata[15]}}, 16'b0};

        // to do sign-extended or not
        assign extenMemRdata = {32{op[1] || op[2]}} &  validRdata
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

        // swr_data is the same as validWdata
        assign swl_data = {32{offset[0]}} & {24'b0, RF_rdata2[31:24]}
                        | {32{offset[1]}} & {16'b0, RF_rdata2[31:16]}
                        | {32{offset[2]}} & { 8'b0, RF_rdata2[31: 8]}
                        | {32{offset[3]}} & RF_rdata2;

        assign Write_data = {32{ wl}} & swl_data
                          | {32{~wl}} & validWdata;


        // CU module *****************************************************************
        // generate this control signals and some signals about the type of instruction
        wire RegDst, ALUSrcA, RegWrite, MemtoReg, IRWrite, PCWrite, PCWriteCond, lui, ImmDst;
        wire [2:0] ALUcontrol, ALUop;
        wire [1:0] ALUSrcB, PCsource;
        wire NOP = ~(|IR);

        // mov
        wire mov = R_type && (func[5:1] == 5'b00101);
        wire movValid = ((|RF_rdata2) ^ ~func[0]) && mov;
        // link
        wire link = J_type && op[0] || R_type && (func == 6'b001001);
        // jmp
        wire jmp = J_type || R_type && jmp_r;
        wire jmp_r = (func[5:1] == 5'b00100);
        // load and store
        wire I_type_branch = (op[5:2] == 4'b0001);
        wire branch = REGIMM || I_type_branch;
        wire load  = op[5] && ~op[3];
        wire store = op[5] &&  op[3];

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
                        ID    : begin
                                if (NOP) 
                                        next_state = IF;
                                else
                                        next_state = EX;
                        end
                        EX    : begin
                                if (J_type && ~link || REGIMM || I_type_branch)
                                        next_state = IF;
                                else if (store)
                                        next_state = ST;
                                else if (load)
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


        // FSM 3.
        assign Inst_Req_Valid  = ~current_state[0] && current_state[1];
        assign Inst_Ready      =  current_state[0] || current_state[2];
        assign Read_data_Ready =  current_state[0] || current_state[7];

        assign J_type = (op[5:1] == 5'b00001);
        assign R_type = ~(|op);
        assign REGIMM = (op == 6'b000001);
        assign lui = (op == 6'b001111);

        // unconditional jump
        assign PCWrite     = current_state[2] && Inst_Valid || current_state[4] && jmp;
        // conditional jump
        assign PCWriteCond = current_state[4] && (REGIMM || I_type_branch);
        assign MemRead     = current_state[6];
        assign MemWrite    = current_state[5];
        // to select the source of PC
        assign PCsource[1] = current_state[4] && jmp;
        assign PCsource[0] = current_state[4] && branch;
        // to select the second oprand of ALU
        assign ALUSrcB[1]  = current_state[5] || current_state[6] || current_state[4] && (op[3] || op[5]) || current_state[3] && branch;
        assign ALUSrcB[0]  = current_state[5] || current_state[6] || current_state[2] || current_state[4] && link || current_state[3] && branch;
        // to select the first oprand of ALU, RF_rdata1 or PC
        assign ALUSrcA     = (current_state[4] && ~link) || (current_state[3] && ~branch);
        assign RegWrite    = current_state[8];
        assign IRWrite     = current_state[2] && Inst_Ready && Inst_Valid;
        assign MemtoReg    = current_state[8] && load;
        assign RegDst      = current_state[8] && R_type;
        assign ImmDst      = current_state[8] && (op[5] || op[3]);

        // ALUEx is only used in EX state
        wire [2:0] ALUEx;

        // first decode of ALUop by opcode, then we still need to use the ALUcontroler module to decode
        assign ALUEx[2] = ~op[5] && (~op[3] || op[1] && ~op[0]);
        assign ALUEx[1] = op[5] || (op[3] ^ op[2]) || REGIMM;
        assign ALUEx[0] = R_type || REGIMM || ~op[5] && (~op[2] && op[1] || ~op[3] && op[2] && op[1] || op[3] && op[2] && op[0]);

        assign ALUop = (current_state[2] || current_state[3] || current_state[4] && link) ? 3'b010
                                                                                          : ALUEx;

        // counter ******************************************************************
        reg [31:0] cycle_cnt, IW_cnt, memR_cnt, memW_cnt, jmp_times, branch_times, branchValid_times, Inst_num;

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
                else if (jmp && current_state[3]) begin
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
                else if (branch && current_state[3]) begin
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

endmodule


// if ALUop is not 101, we can get ALUop only by op. So the input ALUop is the real ALUop
// if ALUop is 101, we must get ALUop by func because it is R-type instruction
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

