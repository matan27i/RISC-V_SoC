// Control Unit - RTL
import risc_pkg::*;

module control (
  input  logic       r_type,
  input  logic       i_type,
  input  logic       s_type,
  input  logic       b_type,
  input  logic       u_type,
  input  logic       j_type,

  input  logic [2:0] funct3,
  input  logic       funct7_bit5, // Distinguishes between certain instructions (e.g., SRLI vs SRAI)
  input  logic [6:0] opcode,

  output logic        pc_sel,
  output logic        op1_sel,
  output logic        op2_sel,
  output alu_op_t     alu_op,
  output wb_src_t     rf_wr_data_sel,
  output logic        dmem_req,
  output mem_size_t   dmem_size,
  output logic        dmem_wr_en,
  output logic        dmem_zero_extend,
  output logic        rf_wr_en
);

  logic [3:0] funct_r;
  logic [3:0] opcode_i;
  

 
  assign funct_r     = {funct7_bit5, funct3};
  assign opcode_i    = {opcode[4], funct3};

  control_t cntrl_r, cntrl_i, cntrl_s, cntrl_b, cntrl_u, cntrl_j, cntrl;

  // R-type
  always_comb begin
    cntrl_r = '0;
    cntrl_r.rf_write_enable = 1'b1;
    case (funct_r)
      R_ADD : cntrl_r.alu_op = ADD;
      R_SUB : cntrl_r.alu_op = SUB;
      R_AND : cntrl_r.alu_op = AND;
      R_OR  : cntrl_r.alu_op = OR;
      R_XOR : cntrl_r.alu_op = XOR;
      R_SLL : cntrl_r.alu_op = SLL;
      R_SRL : cntrl_r.alu_op = SRL;
      R_SRA : cntrl_r.alu_op = SRA;
      R_SLT : cntrl_r.alu_op = SLT;
      R_SLTU: cntrl_r.alu_op = SLTU;
      default: cntrl_r.alu_op = ADD;
    endcase
  end

  // I-type
  always_comb begin
    cntrl_i = '0;
    cntrl_i.rf_write_enable  = 1'b1;
    cntrl_i.alu_src_b_select = 1'b1;

    case (opcode_i)
      I_ADDI : cntrl_i.alu_op = ADD;
      I_ANDI : cntrl_i.alu_op = AND;
      I_ORI  : cntrl_i.alu_op = OR;
      I_XORI : cntrl_i.alu_op = XOR;
      I_SLLI : cntrl_i.alu_op = SLL;
      I_SRLI_SRAI: cntrl_i.alu_op = funct7_bit5 ? SRA : SRL;
      I_SLTI : cntrl_i.alu_op = SLT;
      I_SLTIU: cntrl_i.alu_op = SLTU;

      I_LB : {cntrl_i.mem_valid, cntrl_i.mem_size, cntrl_i.wb_src, cntrl_i.load_zero_extend} = {1'b1, BYTE,      WB_SRC_MEM, 1'b0};
      I_LH : {cntrl_i.mem_valid, cntrl_i.mem_size, cntrl_i.wb_src, cntrl_i.load_zero_extend} = {1'b1, HALF_WORD, WB_SRC_MEM, 1'b0};
      I_LW : {cntrl_i.mem_valid, cntrl_i.mem_size, cntrl_i.wb_src, cntrl_i.load_zero_extend} = {1'b1, WORD,      WB_SRC_MEM, 1'b0};
      I_LBU: {cntrl_i.mem_valid, cntrl_i.mem_size, cntrl_i.wb_src, cntrl_i.load_zero_extend} = {1'b1, BYTE,      WB_SRC_MEM, 1'b1};
      I_LHU: {cntrl_i.mem_valid, cntrl_i.mem_size, cntrl_i.wb_src, cntrl_i.load_zero_extend} = {1'b1, HALF_WORD, WB_SRC_MEM, 1'b1};
      default: ;
    endcase

    if (opcode == OPCODE_I_JALR) begin
      // JALR aliases I_LB in opcode_i (both 4h0)
      // clear spurious load decode below
      cntrl_i.mem_valid        = 1'b0;
      cntrl_i.mem_size         = BYTE;
      cntrl_i.load_zero_extend = 1'b0;
      cntrl_i.pc_src_select    = 1'b1;  // PC = rs1 + imm
      cntrl_i.wb_src           = WB_SRC_PC;
      cntrl_i.alu_op           = ADD;
    end
  end

  // S-type
  always_comb begin
    cntrl_s = '0;
    cntrl_s.mem_valid        = 1'b1;
    cntrl_s.mem_write        = 1'b1;
    cntrl_s.alu_src_b_select = 1'b1;
    case (funct3)
      S_SB: cntrl_s.mem_size = BYTE;
      S_SH: cntrl_s.mem_size = HALF_WORD;
      S_SW: cntrl_s.mem_size = WORD;
      default: cntrl_s.mem_size = WORD;
    endcase
  end

  // B-type
  always_comb begin
    cntrl_b = '0;
    cntrl_b.alu_src_a_select = 1'b1;
    cntrl_b.alu_src_b_select = 1'b1;
    cntrl_b.alu_op           = ADD;
  end

  // U-type
  always_comb begin
    cntrl_u = '0;
    cntrl_u.rf_write_enable = 1'b1;
    case (opcode)
      OPCODE_AUIPC: begin
        cntrl_u.alu_src_a_select = 1'b1;
        cntrl_u.alu_src_b_select = 1'b1;
      end
      OPCODE_LUI: begin
        cntrl_u.wb_src = WB_SRC_IMM;
      end
      default: ;
    endcase
  end

  // J-type
  always_comb begin
    cntrl_j = '0;
    cntrl_j.rf_write_enable  = 1'b1;
    cntrl_j.wb_src           = WB_SRC_PC;
    cntrl_j.alu_src_a_select = 1'b1;
    cntrl_j.alu_src_b_select = 1'b1;
    cntrl_j.pc_src_select    = 1'b1;
  end

  // Final selection
  always_comb begin
    cntrl = '0;
    if      (r_type) cntrl = cntrl_r;
    else if (i_type) cntrl = cntrl_i;
    else if (s_type) cntrl = cntrl_s;
    else if (b_type) cntrl = cntrl_b;
    else if (u_type) cntrl = cntrl_u;
    else if (j_type) cntrl = cntrl_j;
    
    else             cntrl = '0;
  end

  assign pc_sel           = cntrl.pc_src_select;
  assign op1_sel          = cntrl.alu_src_a_select;
  assign op2_sel          = cntrl.alu_src_b_select;
  assign alu_op           = cntrl.alu_op;
  assign rf_wr_data_sel   = cntrl.wb_src;
  assign dmem_req         = cntrl.mem_valid;
  assign dmem_wr_en       = cntrl.mem_write;
  assign dmem_size        = cntrl.mem_size;
  assign dmem_zero_extend = cntrl.load_zero_extend;
  assign rf_wr_en         = cntrl.rf_write_enable;


endmodule
