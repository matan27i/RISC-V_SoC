// Branch Control - RTL

import risc_pkg::*;

module branch_control(
  // Operands to compare
  input  logic [XLEN-1:0] opr_a,
  input  logic [XLEN-1:0] opr_b,

  // Branch instruction info
  input  logic            is_b_type,
  input  logic [2:0]      funct3,

  output logic            branch_taken
);

  // Explicitly cast operands to signed for signed comparisons
  logic signed [XLEN-1:0] a_signed, b_signed;
  assign a_signed = $signed(opr_a);
  assign b_signed = $signed(opr_b);

  logic condition_met;

  always_comb begin
    condition_met = 1'b0; // Default to not met

    case (funct3)
      B_BEQ:  condition_met = (opr_a == opr_b);
      B_BNE:  condition_met = (opr_a != opr_b);
      B_BLT:  condition_met = (a_signed < b_signed);
      B_BGE:  condition_met = (a_signed >= b_signed);
      B_BLTU: condition_met = (opr_a < opr_b);
      B_BGEU: condition_met = (opr_a >= opr_b);
      default: condition_met = 1'b0;
    endcase
  end

  assign branch_taken = is_b_type & condition_met;

endmodule