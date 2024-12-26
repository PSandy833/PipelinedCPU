// Version with comments!

// Template for Northwestern - CompEng 361 - Lab4
// Groupname: RISCitAll

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE 7'b0110011
`define OPCODE_BRANCH 7'b1100011
`define OPCODE_LOAD 7'b0000011
`define OPCODE_STORE 7'b0100011 
`define AUX_FUNC_ADD 7'b0000000
`define AUX_FUNC_SUB 7'b0100000
`define FUNC_SUB 3'b000
`define SIZE_BYTE 2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD 2'b10

`define OPCODE_IMMEDIATE 7'b0010011
`define OPCODE_JAL 7'b1101111
`define OPCODE_JALR 7'b1100111
`define OPCODE_LUI 7'b0110111
`define OPCODE_AUIPC 7'b0010111

`define INST_NOP 32'h00000013

// Basic instructions
`define ADD_OPCODE 3'b000
`define SUB_OPCODE 3'b000
`define SLL_OPCODE 3'b001
`define SLT_OPCODE 3'b010
`define SLTU_OPCODE 3'b011
`define XOR_OPCODE 3'b100
`define SRL_OPCODE 3'b101
`define SRA_OPCODE 3'b101
`define OR_OPCODE 3'b110
`define AND_OPCODE 3'b111

// Multiply extension
`define MUL_OPCODE 3'b000
`define MULH_OPCODE 3'b001
`define MULHSU_OPCODE 3'b010
`define MULHU_OPCODE 3'b011
`define DIV_OPCODE 3'b100
`define DIVU_OPCODE 3'b101
`define REM_OPCODE 3'b110
`define REMU_OPCODE 3'b111

// Immediate computation
`define ADDI_OPCODE 3'b000
`define SLTI_OPCODE 3'b010
`define SLTIU_OPCODE 3'b011
`define XORI_OPCODE 3'b100
`define ORI_OPCODE 3'b110
`define ANDI_OPCODE 3'b111
`define SLLI_OPCODE 3'b001
`define SRLI_OPCODE 3'b101
`define SRAI_OPCODE 3'b101

// Load instruction opcodes
`define LB_OPCODE 3'b000
`define LH_OPCODE 3'b001
`define LW_OPCODE 3'b010
`define LBU_OPCODE 3'b100
`define LHU_OPCODE 3'b101

// Store instruction opcodes
`define SB_OPCODE 3'b000
`define SH_OPCODE 3'b001
`define SW_OPCODE 3'b010

// Branch instruction opcodes
`define BEQ_OPCODE 3'b000
`define BNE_OPCODE 3'b001
`define BLT_OPCODE 3'b100
`define BGE_OPCODE 3'b101
`define BLTU_OPCODE 3'b110
`define BGEU_OPCODE 3'b111


module PipelinedCPU(halt, clk, rst);
 output halt;
 input clk, rst;

 // Assign halt if there is a halt at the end of the pipeline to ensure prior instructions are executed 
 assign halt = MEM_WB_halt;

 // IF to ID
 reg [`WORD_WIDTH-1:0] IF_ID_PC, IF_ID_InstWord;

 // ID to EX
 reg [`WORD_WIDTH-1:0] ID_EX_PC, ID_EX_Rdata1, ID_EX_Rdata2, ID_EX_Imm;
 reg [4:0] ID_EX_Rsrc1, ID_EX_Rsrc2, ID_EX_Rdst;
 reg [6:0] ID_EX_Opcode, ID_EX_Funct7;
 reg [2:0] ID_EX_Funct3;
 reg ID_EX_halt;

 // EX to MEM
 reg [`WORD_WIDTH-1:0] EX_MEM_ALUOut, EX_MEM_Rdata2;
 reg [4:0] EX_MEM_Rdst;
 reg [6:0] EX_MEM_Opcode;
 reg EX_MEM_MemWrEn, EX_MEM_RWrEn;
 reg [1:0] EX_MEM_MemSize;
 reg EX_MEM_halt;
 reg [2:0] EX_MEM_Funct3;

 // MEM to WB
 reg [`WORD_WIDTH-1:0] MEM_WB_MemData, MEM_WB_ALUOut;
 reg [4:0] MEM_WB_Rdst;
 reg [6:0] MEM_WB_Opcode;
 reg MEM_WB_RWrEn;
 reg MEM_WB_halt;

 // FLUSH ON BRANCH CHANGE
 wire control_flow_change;
 assign control_flow_change = (ID_EX_Opcode == `OPCODE_BRANCH && branch_out != (ID_EX_PC + 4)) ||
 (ID_EX_Opcode == `OPCODE_JAL) ||
 (ID_EX_Opcode == `OPCODE_JALR);

 // STALL ON LOAD-USE HAZARD
 wire uses_Rsrc1, uses_Rsrc2;

 // Instructions that use Rsrc1
 assign uses_Rsrc1 = (opcode == `OPCODE_COMPUTE) ||
 (opcode == `OPCODE_IMMEDIATE) ||
 (opcode == `OPCODE_BRANCH) ||
 (opcode == `OPCODE_STORE) ||
 (opcode == `OPCODE_LOAD) ||
 (opcode == `OPCODE_JALR);

 // Instructions that use Rsrc2
 assign uses_Rsrc2 = (opcode == `OPCODE_COMPUTE) ||
 (opcode == `OPCODE_BRANCH) ||
 (opcode == `OPCODE_STORE);

 wire load_stall;
 assign load_stall = (ID_EX_Opcode == `OPCODE_LOAD) &&
 ((Rsrc1 == ID_EX_Rdst && uses_Rsrc1) || (Rsrc2 == ID_EX_Rdst && uses_Rsrc2));



 // This ensures halting logic will not trigger when the pipeline is being loaded
 reg IF_ID_valid; // Register to track valid instructions
 always @(posedge clk or negedge rst) begin
 if (!rst) begin
 IF_ID_valid <= 0;
 end else begin
 IF_ID_valid <= 1; // Becomes valid after the first cycle
 end
 end


 // STALL FOR MULTIPLY AND DIVIDE
 wire is_long_latency;
 wire is_div;
 reg [4:0] stall_count;

 // Detect if the instruction is a divide or remainder operation
 assign is_long_latency = (ID_EX_Opcode == `OPCODE_COMPUTE && ID_EX_Funct7 == 7'h1);
 assign is_div = is_long_latency && (ID_EX_Funct3 == 3'b100 || ID_EX_Funct3 == 3'b101 || ID_EX_Funct3 == 3'b110 || ID_EX_Funct3 == 3'b111);

 // block to update stall_count
 always @(posedge clk or negedge rst) begin
 if (!rst) begin
 stall_count <= 0;
 end else if (stall_count > 0) begin
 stall_count <= stall_count - 1;
 end else if (is_long_latency) begin
 stall_count <= is_div ? 19 : 3;
 end
 end


 // SYSTEM STATE

 // PC file
 Reg PC_REG(.Din(NPC), .Qout(PC), .WE(!(load_stall || (stall_count != 0))), .CLK(clk), .RST(rst));

 // Memory file
 Mem MEM(.InstAddr(PC), .InstOut(InstWord), .DataAddr(EX_MEM_ALUOut), .DataSize(EX_MEM_MemSize), .DataIn(EX_MEM_Rdata2), .DataOut(data_out), .WE(EX_MEM_MemWrEn), .CLK(clk));
 
 // Register file
 RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1),
 .AddrB(Rsrc2), .DataOutB(Rdata2),
 .AddrW(MEM_WB_Rdst), .DataInW(RF_WriteData), 
 .WenW(MEM_WB_RWrEn), .CLK(clk));



 // STAGE 1: Instruction Fetch (IF)
 wire [`WORD_WIDTH-1:0] PC, NPC, InstWord, PC_Plus_4;
 assign PC_Plus_4 = PC + 4;
 assign NPC = (control_flow_change) ? ALUOut : PC_Plus_4;

 // Transfer values, hold if there are stalls
 always @(posedge clk or negedge rst) begin
 if (!rst || control_flow_change) begin
 IF_ID_PC <= 0;
 IF_ID_InstWord <= `INST_NOP;
 end else if (!(load_stall || (stall_count != 0))) begin
 IF_ID_PC <= PC;
 IF_ID_InstWord <= InstWord;
 end
 end



 // STAGE 2: Instruction Decode (ID)
 wire [4:0] Rsrc1, Rsrc2, Rdst;
 wire [6:0] opcode;
 wire [2:0] funct3;
 wire [6:0] funct7;
 wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, imm_I, imm_S, imm_B, imm_U, imm_J, imm_actual;
 wire invalid_op, mem_alignment_error;

 assign opcode = IF_ID_InstWord[6:0];
 assign Rdst = IF_ID_InstWord[11:7];
 assign Rsrc1 = IF_ID_InstWord[19:15];
 assign Rsrc2 = IF_ID_InstWord[24:20];
 assign funct3 = IF_ID_InstWord[14:12];
 assign funct7 = IF_ID_InstWord[31:25];

 assign imm_I = {{20{IF_ID_InstWord[31]}}, IF_ID_InstWord[31:20]};
 assign imm_S = {{20{IF_ID_InstWord[31]}}, IF_ID_InstWord[31:25], IF_ID_InstWord[11:7]};
 assign imm_B = {{19{IF_ID_InstWord[31]}}, IF_ID_InstWord[31], IF_ID_InstWord[7], IF_ID_InstWord[30:25], IF_ID_InstWord[11:8], 1'b0};
 assign imm_U = {IF_ID_InstWord[31:12], 12'b0};
 assign imm_J = {{11{IF_ID_InstWord[31]}}, IF_ID_InstWord[31], IF_ID_InstWord[19:12], IF_ID_InstWord[20], IF_ID_InstWord[30:21], 1'b0};

 assign imm_actual = (opcode == `OPCODE_LOAD || opcode == `OPCODE_IMMEDIATE) ? imm_I :
 (opcode == `OPCODE_STORE) ? imm_S :
 (opcode == `OPCODE_BRANCH) ? imm_B :
 (opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC) ? imm_U :
 (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? imm_J :
 0;

 assign invalid_op = IF_ID_valid && !(
 (opcode == `OPCODE_COMPUTE && 
 (funct7 == 7'b0000000 || funct7 == 7'b0100000 || funct7 == 7'b0000001)) ||
 (opcode == `OPCODE_BRANCH) ||
 (opcode == `OPCODE_LOAD) ||
 (opcode == `OPCODE_STORE) ||
 (opcode == `OPCODE_IMMEDIATE) ||
 (opcode == `OPCODE_JAL) ||
 (opcode == `OPCODE_JALR) ||
 (opcode == `OPCODE_LUI) ||
 (opcode == `OPCODE_AUIPC)
 );


 wire [31:0] effective_addr = (opcode == `OPCODE_LOAD) ? (Rdata1 + imm_I) :
 (opcode == `OPCODE_STORE) ? (Rdata1 + imm_S) :
 32'b0;

 assign mem_alignment_error = 
 ((opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) &&
 (
 ((funct3 == `LH_OPCODE || funct3 == `LHU_OPCODE || funct3 == `SH_OPCODE) && (effective_addr[0] != 1'b0)) ||
 ((funct3 == `LW_OPCODE || funct3 == `SW_OPCODE) && (effective_addr[1:0] != 2'b00))
 ));

 // Transfer values, reset if there is control flow change or a load stall, hold if there is a multiply/divide stall
 always @(posedge clk or negedge rst) begin
 if (!rst || control_flow_change || load_stall) begin
 ID_EX_PC <= 0;
 ID_EX_Rdata1 <= 0;
 ID_EX_Rdata2 <= 0;
 ID_EX_Rsrc1 <= 0;
 ID_EX_Rsrc2 <= 0;
 ID_EX_Imm <= 0;
 ID_EX_Opcode <= 7'b0010011; // Opcode for ADDI
 ID_EX_Funct3 <= 0;
 ID_EX_Funct7 <= 0;
 ID_EX_Rdst <= 0;
 ID_EX_halt <= 0;
 end else if (!stall_count) begin
 ID_EX_PC <= IF_ID_PC;
 ID_EX_Rdata1 <= Rdata1;
 ID_EX_Rdata2 <= Rdata2;
 ID_EX_Rsrc1 <= Rsrc1;
 ID_EX_Rsrc2 <= Rsrc2;
 ID_EX_Imm <= imm_actual;
 ID_EX_Opcode <= opcode;
 ID_EX_Funct3 <= funct3;
 ID_EX_Funct7 <= funct7;
 ID_EX_Rdst <= Rdst;
 ID_EX_halt <= invalid_op || mem_alignment_error;
 end
 end




 // STAGE 3: Execute (EX) 
 wire [31:0] ALUOut;
 wire [1:0] MemSize;
 wire MemWrEn;
 wire RWrEn;
 wire [1:0] load_MemSize;
 wire [1:0] store_MemSize;
 wire [`WORD_WIDTH-1:0] ALU_Rdata1, ALU_Rdata2;

 wire load_forward_Rdata1, load_forward_Rdata2, forward_Rdata1, forward_Rdata2;

 // Detect forwarding conditions from the MEM stage (for general forwarding)
 assign load_forward_Rdata1 = ((MEM_WB_Rdst != 0) && (MEM_WB_Opcode == `OPCODE_LOAD) && (MEM_WB_Rdst == ID_EX_Rsrc1));
 assign load_forward_Rdata2 = ((MEM_WB_Rdst != 0) && (MEM_WB_Opcode == `OPCODE_LOAD) && (MEM_WB_Rdst == ID_EX_Rsrc2));

 // Detect general forwarding conditions from the WB stage
 assign forward_Rdata1 = (EX_MEM_Rdst != 0) && (EX_MEM_RWrEn) && (EX_MEM_Rdst == ID_EX_Rsrc1);
 assign forward_Rdata2 = (EX_MEM_Rdst != 0) && (EX_MEM_RWrEn) && (EX_MEM_Rdst == ID_EX_Rsrc2);

 // Forwarding logic for ALU inputs (ALU_Rdata1 and ALU_Rdata2)
 assign ALU_Rdata1 = load_forward_Rdata1 ? MEM_WB_MemData : 
 forward_Rdata1 ? EX_MEM_ALUOut : 
 ID_EX_Rdata1;

 assign ALU_Rdata2 = load_forward_Rdata2 ? MEM_WB_MemData : 
 forward_Rdata2 ? EX_MEM_ALUOut : 
 ID_EX_Rdata2;



 wire [31:0] eu_out, ieu_out, branch_out, load_out, store_out, lui_out, auipc_out, jal_out, jalr_out;

 ExecutionUnit EU(.out(eu_out), .opA(ALU_Rdata1), .opB(ALU_Rdata2), .func(ID_EX_Funct3), .auxFunc(ID_EX_Funct7));
 ImmediateExecutionUnit IEU(.out(ieu_out), .opA(ALU_Rdata1), .imm(ID_EX_Imm), .func(ID_EX_Funct3), .auxFunc(ID_EX_Funct7));

 BranchUnit BU(.out(branch_out), .opA(ALU_Rdata1), .opB(ALU_Rdata2), .pc(ID_EX_PC), .imm(ID_EX_Imm[11:0]), .func(ID_EX_Funct3), .auxFunc(ID_EX_Opcode));
 LoadUnit LU(.out(load_out), .MemSize(load_MemSize), .base(ALU_Rdata1), .offset(ID_EX_Imm[11:0]), .func(ID_EX_Funct3));
 StoreUnit SU(.out(store_out), .MemSize(store_MemSize), .base(ALU_Rdata1), .offset(ID_EX_Imm[11:0]), .func(ID_EX_Funct3));

 LuiUnit LUI(.out(lui_out), .imm(ID_EX_Imm[31:12]), .pc(ID_EX_PC));
 AuipcUnit AUIPC(.out(auipc_out), .imm(ID_EX_Imm[31:12]), .pc(ID_EX_PC));
 JalUnit JAL(.out(jal_out), .offset(ID_EX_Imm[20:0]), .pc(ID_EX_PC));
 JalrUnit JALR(.out(jalr_out), .base(ALU_Rdata1), .offset(ID_EX_Imm[11:0]), .pc(ID_EX_PC));

 assign MemWrEn = (ID_EX_Opcode == `OPCODE_STORE) ? 1'b1 : 1'b0;
 assign MemSize = (ID_EX_Opcode == `OPCODE_LOAD) ? load_MemSize :
 (ID_EX_Opcode == `OPCODE_STORE) ? store_MemSize : 
 2'b00;

 assign ALUOut = (ID_EX_Opcode == `OPCODE_COMPUTE) ? eu_out : // Arithmetic/Logic operations
 (ID_EX_Opcode == `OPCODE_IMMEDIATE) ? ieu_out : // Immediate ALU operations
 (ID_EX_Opcode == `OPCODE_BRANCH) ? branch_out : // Branch target address
 (ID_EX_Opcode == `OPCODE_LOAD) ? load_out : // Load memory address
 (ID_EX_Opcode == `OPCODE_STORE) ? store_out : // Store memory address
 (ID_EX_Opcode == `OPCODE_LUI) ? lui_out : // LUI result
 (ID_EX_Opcode == `OPCODE_AUIPC) ? auipc_out : // AUIPC result
 (ID_EX_Opcode == `OPCODE_JAL) ? jal_out : // JAL target address
 (ID_EX_Opcode == `OPCODE_JALR) ? jalr_out : // JALR target address
 32'b0; // Default for unsupported opcodes

 assign RWrEn = (ID_EX_Opcode == `OPCODE_COMPUTE) || 
 (ID_EX_Opcode == `OPCODE_IMMEDIATE) || 
 (ID_EX_Opcode == `OPCODE_LOAD) || 
 (ID_EX_Opcode == `OPCODE_JAL) ||
 (ID_EX_Opcode == `OPCODE_JALR) ||
 (ID_EX_Opcode == `OPCODE_LUI) ||
 (ID_EX_Opcode == `OPCODE_AUIPC);

 // Transfer values, hold if multiply/divide is stalling
 always @(posedge clk or negedge rst) begin
 if (!rst) begin
 EX_MEM_ALUOut <= 0;
 EX_MEM_Rdata2 <= 0;
 EX_MEM_Opcode <= 0;
 EX_MEM_Rdst <= 0;
 EX_MEM_MemWrEn <= 0;
 EX_MEM_MemSize <= 0;
 EX_MEM_RWrEn <= 0;
 EX_MEM_halt <= 0;
 EX_MEM_Funct3 <= 0;
 end else if (!stall_count) begin
 EX_MEM_ALUOut <= ALUOut;
 EX_MEM_Rdata2 <= ALU_Rdata2;
 EX_MEM_Opcode <= ID_EX_Opcode;
 EX_MEM_Rdst <= ID_EX_Rdst;
 EX_MEM_MemWrEn <= (ID_EX_halt) ? 0: MemWrEn ; // Don't allow writes during a stall
 EX_MEM_MemSize <= MemSize;
 EX_MEM_RWrEn <= RWrEn;
 EX_MEM_halt <= ID_EX_halt;
 EX_MEM_Funct3 <= ID_EX_Funct3;
 end
 end





 // STAGE 4: Memory (MEM)
 wire [31:0] data_out;

 // Need to sign extend or zero extend data based on the instruction
 wire [31:0] extended_data;
 assign extended_data = (EX_MEM_Opcode == `OPCODE_LOAD) ? 
 (EX_MEM_Funct3 == `LB_OPCODE) ? {{24{data_out[7]}}, data_out[7:0]} :
 (EX_MEM_Funct3 == `LH_OPCODE) ? {{16{data_out[15]}}, data_out[15:0]} : 
 (EX_MEM_Funct3 == `LW_OPCODE) ? data_out :
 (EX_MEM_Funct3 == `LBU_OPCODE) ? {24'b0, data_out[7:0]} : 
 (EX_MEM_Funct3 == `LHU_OPCODE) ? {16'b0, data_out[15:0]} : 
 data_out : data_out; 

 // Transfer values
 always @(posedge clk or negedge rst) begin
 if (!rst) begin
 MEM_WB_MemData <= 0;
 MEM_WB_ALUOut <= 0;
 MEM_WB_Opcode <= 0;
 MEM_WB_Rdst <= 0;
 MEM_WB_RWrEn <= 0;
 MEM_WB_halt <= 0;
 end else begin
 MEM_WB_MemData <= extended_data;
 MEM_WB_ALUOut <= EX_MEM_ALUOut;
 MEM_WB_Opcode <= EX_MEM_Opcode;
 MEM_WB_Rdst <= EX_MEM_Rdst;
 MEM_WB_RWrEn <= EX_MEM_RWrEn;
 MEM_WB_halt <= EX_MEM_halt;
 end
 end




 // STAGE 5: Writeback (WB) 

 // Data to write should be data from memory for load instruction, otherwise ALU Out
 wire [31:0] RF_WriteData;
 assign RF_WriteData = (MEM_WB_Opcode == `OPCODE_LOAD) ? MEM_WB_MemData : MEM_WB_ALUOut; 





endmodule // PipelinedCPU









module ExecutionUnit(
 output [31:0] out,
 input [31:0] opA,
 input [31:0] opB,
 input [2:0] func,
 input [6:0] auxFunc);

 // Basic instructions
 wire [31:0] add_output = opA + opB;
 wire [31:0] sub_output = opA - opB;
 wire [31:0] sll_output = opA << opB[4:0];
 wire [31:0] slt_output = {31'b0, ($signed(opA) < $signed(opB)) ? 1'b1 : 1'b0};
 wire [31:0] sltu_output = {31'b0, (opA < opB) ? 1'b1 : 1'b0};
 wire [31:0] xor_output = opA ^ opB;
 wire [31:0] srl_output = opA >> opB[4:0];
 wire [31:0] sra_output = $signed(opA) >>> opB[4:0];
 wire [31:0] or_output = opA | opB;
 wire [31:0] and_output = opA & opB;

 // Multiply extension
 wire [31:0] mul_output = opA * opB;
 wire [63:0] mulh_temp = ($signed(opA) * $signed(opB));
 wire [31:0] mulh_output = mulh_temp [63:32];
 wire [63:0] mulhsu_temp = ($signed(opA) * opB);
 wire [31:0] mulhsu_output = mulhsu_temp [63:32];
 wire [63:0] mulhu_temp = (opA * opB);
 wire [31:0] mulhu_output = mulhu_temp [63:32];
 wire [31:0] div_output = (opB != 0) ? $signed(opA) / $signed(opB) : 32'b0;
 wire [31:0] divu_output = (opB != 0) ? opA / opB : 32'b0;
 wire [31:0] rem_output = (opB != 0) ? $signed(opA) % $signed(opB) : 32'b0;
 wire [31:0] remu_output = (opB != 0) ? opA % opB : 32'b0;

 assign out = (func == `SUB_OPCODE && auxFunc == 7'b0100000) ? sub_output :
 (func == `ADD_OPCODE && auxFunc == 7'b0000000) ? add_output :
 (func == `SLL_OPCODE && auxFunc == 7'b0000000) ? sll_output :
 (func == `SLT_OPCODE && auxFunc == 7'b0000000) ? slt_output :
 (func == `SLTU_OPCODE && auxFunc == 7'b0000000) ? sltu_output :
 (func == `XOR_OPCODE && auxFunc == 7'b0000000) ? xor_output :
 (func == `SRA_OPCODE && auxFunc == 7'b0100000) ? sra_output :
 (func == `SRL_OPCODE && auxFunc == 7'b0000000) ? srl_output :
 (func == `OR_OPCODE && auxFunc == 7'b0000000) ? or_output :
 (func == `AND_OPCODE && auxFunc == 7'b0000000) ? and_output :
 (func == `MUL_OPCODE && auxFunc == 7'b0000001) ? mul_output :
 (func == `DIV_OPCODE && auxFunc == 7'b0000001) ? div_output :
 (func == `DIVU_OPCODE && auxFunc == 7'b0000001) ? divu_output :
 (func == `REM_OPCODE && auxFunc == 7'b0000001) ? rem_output :
 (func == `REMU_OPCODE && auxFunc == 7'b0000001) ? remu_output :
 (func == `MULH_OPCODE && auxFunc == 7'b0000001) ? mulh_output :
 (func == `MULHSU_OPCODE && auxFunc == 7'b0000001) ? mulhsu_output :
 (func == `MULHU_OPCODE && auxFunc == 7'b0000001) ? mulhu_output :
 32'b0;

endmodule

module ImmediateExecutionUnit(
 output [31:0] out,
 input [31:0] opA,
 input [31:0] imm,
 input [2:0] func,
 input [6:0] auxFunc);

 wire [31:0] addi_output = opA + imm;
 wire [31:0] slti_output = {31'b0, ($signed(opA) < $signed(imm)) ? 1'b1 : 1'b0};
 wire [31:0] sltiu_output = {31'b0, (opA < imm) ? 1'b1 : 1'b0};
 wire [31:0] xori_output = opA ^ imm;
 wire [31:0] ori_output = opA | imm;
 wire [31:0] andi_output = opA & imm;
 wire [31:0] slli_output = opA << imm[4:0];
 wire [31:0] srli_output = opA >> imm[4:0];
 wire [31:0] srai_output = $signed(opA) >>> imm[4:0];

 assign out = (func == `ADDI_OPCODE) ? addi_output :
 (func == `SLTI_OPCODE) ? slti_output :
 (func == `SLTIU_OPCODE) ? sltiu_output :
 (func == `XORI_OPCODE) ? xori_output :
 (func == `ORI_OPCODE) ? ori_output :
 (func == `ANDI_OPCODE) ? andi_output :
 (func == `SLLI_OPCODE) ? slli_output :
 (func == `SRLI_OPCODE && auxFunc == 7'b0000000) ? srli_output : 
 (func == `SRAI_OPCODE && auxFunc == 7'b0100000) ? srai_output : 
 32'b0;

endmodule 


module BranchUnit(
 output [31:0] out,
 input [31:0] opA,
 input [31:0] opB,
 input [31:0] pc,
 input [11:0] imm,
 input [2:0] func,
 input [6:0] auxFunc);
 
 wire [31:0] extended_imm = {{20{imm[11]}}, imm};
 wire [31:0] branch_offset = extended_imm;

 wire condition_met = (func == `BEQ_OPCODE) ? (opA == opB) :
 (func == `BNE_OPCODE) ? (opA != opB) :
 (func == `BLT_OPCODE) ? ($signed(opA) < $signed(opB)) :
 (func == `BGE_OPCODE) ? ($signed(opA) >= $signed(opB)) :
 (func == `BLTU_OPCODE) ? (opA < opB) :
 (func == `BGEU_OPCODE) ? (opA >= opB) :
 1'b0;

 assign out = (condition_met) ? (pc + branch_offset) : (pc + 4);

endmodule

module LoadUnit(
 output [31:0] out,
 output [1:0] MemSize,
 input [31:0] base,
 input [11:0] offset,
 input [2:0] func
);
 wire [31:0] address = base + {{20{offset[11]}}, offset};
 
 assign out = address;
 assign MemSize = (func == `LB_OPCODE || func == `LBU_OPCODE) ? `SIZE_BYTE :
 (func == `LH_OPCODE || func == `LHU_OPCODE) ? `SIZE_HWORD :
 `SIZE_WORD;

endmodule

module StoreUnit(
 output [31:0] out,
 output [1:0] MemSize,
 input [31:0] base,
 input [11:0] offset,
 input [2:0] func
);
 wire [31:0] address = base + {{20{offset[11]}}, offset};
 
 assign out = address;
 assign MemSize = (func == `SB_OPCODE) ? `SIZE_BYTE :
 (func == `SH_OPCODE) ? `SIZE_HWORD :
 `SIZE_WORD;

endmodule

module LuiUnit(
 output [31:0] out,
 input [19:0] imm,
 input [31:0] pc
);
 assign out = {imm, 12'b0}; 
endmodule

module AuipcUnit(
 output [31:0] out,
 input [19:0] imm,
 input [31:0] pc
);
 assign out = pc + {imm, 12'b0}; // Add upper imm to pc
endmodule

module JalUnit(
 output [31:0] out,
 input [20:0] offset,
 input [31:0] pc
);
 wire [31:0] target = pc + {11'b0, offset};
 assign out = target;
endmodule

module JalrUnit(
 output [31:0] out,
 input [31:0] base,
 input [11:0] offset,
 input [31:0] pc
);
 wire [31:0] target = base + {20'b0, offset};
 assign out = {target[31:1], 1'b0}; // Zero lsb
endmodule
