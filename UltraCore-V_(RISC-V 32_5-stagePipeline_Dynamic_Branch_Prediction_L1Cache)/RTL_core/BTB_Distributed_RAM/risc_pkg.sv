// RISC-V RV32I Processor Package
package risc_pkg;

  parameter XLEN = 32;

  typedef enum logic [6:0] {
    OPCODE_R_TYPE = 7'b0110011,
    OPCODE_I_LOAD = 7'b0000011,
    OPCODE_I_ALU  = 7'b0010011,
    OPCODE_I_JALR = 7'b1100111,
    OPCODE_S_TYPE = 7'b0100011,
    OPCODE_B_TYPE = 7'b1100011,
    OPCODE_LUI    = 7'b0110111,
    OPCODE_AUIPC  = 7'b0010111,
    OPCODE_JAL    = 7'b1101111
  } opcode_t;

  typedef enum logic [3:0] {
    ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
  } alu_op_t;

  typedef enum logic [1:0] {
    BYTE      = 2'b00,
    HALF_WORD = 2'b01,
    WORD      = 2'b11
  } mem_size_t;

  typedef enum logic [2:0] {
    B_BEQ  = 3'h0,
    B_BNE  = 3'h1,
    B_BLT  = 3'h4,
    B_BGE  = 3'h5,
    B_BLTU = 3'h6,
    B_BGEU = 3'h7
  } b_type_instr_t;

  typedef enum logic [3:0] {
    R_ADD  = 4'h0,
    R_SUB  = 4'h8,
    R_SLL  = 4'h1,
    R_SLT  = 4'h2,
    R_SLTU = 4'h3,
    R_XOR  = 4'h4,
    R_SRL  = 4'h5,
    R_SRA  = 4'hD,
    R_OR   = 4'h6,
    R_AND  = 4'h7
  } r_type_instr_t;

  typedef enum logic [3:0] {
    I_LB        = 4'h0,
    I_LH        = 4'h1,
    I_LW        = 4'h2,
    I_LBU       = 4'h4,
    I_LHU       = 4'h5,
    I_ADDI      = 4'h8,
    I_SLTI      = 4'hA,
    I_SLTIU     = 4'hB,
    I_XORI      = 4'hC,
    I_ORI       = 4'hE,
    I_ANDI      = 4'hF,
    I_SLLI      = 4'h9,
    I_SRLI_SRAI = 4'hD
  } i_type_instr_t;

  typedef enum logic [2:0] {
    S_SB = 3'h0,
    S_SH = 3'h1,
    S_SW = 3'h2
  } s_type_instr_t;

  typedef enum logic [1:0] {
    WB_SRC_ALU = 2'b00,
    WB_SRC_MEM = 2'b01,
    WB_SRC_IMM = 2'b10,
    WB_SRC_PC  = 2'b11
  } wb_src_t;

  typedef struct packed {
    logic      mem_valid;
    logic      mem_write;
    mem_size_t mem_size;
    logic      load_zero_extend;
    logic      rf_write_enable;
    logic      pc_src_select;
    logic      alu_src_a_select;
    logic      alu_src_b_select;
    wb_src_t   wb_src;
    alu_op_t   alu_op;
  } control_t;

endpackage
