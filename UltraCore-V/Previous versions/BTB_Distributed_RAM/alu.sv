// ALU - RTL
import risc_pkg::*;

module alu(
  input  logic [XLEN-1:0] alu_a,    // ALU input A
  input  logic [XLEN-1:0] alu_b,    // ALU input B
  input  alu_op_t         alu_op,   // ALU operation from risc_pkg

  output logic [XLEN-1:0] alu_res,  // ALU output
  output logic            zero      // Zero flag for branch instructions
);

// Internal signals for signed arithmetic operations
logic signed [XLEN-1:0] signed_a;
logic signed [XLEN-1:0] signed_b;

assign signed_a = $signed(alu_a);
assign signed_b = $signed(alu_b);

// ALU Operation Logic
always_comb begin
  alu_res = '0; // Default to zero to prevent latches
  
  case (alu_op)
    ADD: alu_res = signed_a + signed_b;
    SUB: alu_res = signed_a - signed_b;

    // Logical shifts operate on unsigned data, Arithmetic shift (SRA) on signed
    SLL: alu_res = alu_a << (alu_b[4:0]);
    SRL: alu_res = alu_a >> (alu_b[4:0]);
    SRA: alu_res = signed_a >>> (alu_b[4:0]);

    // Bitwise operations
    OR:  alu_res = alu_a | alu_b;
    AND: alu_res = alu_a & alu_b;
    XOR: alu_res = alu_a ^ alu_b;

    // Comparisons
    SLTU: alu_res = (alu_a < alu_b) ? 32'd1 : '0;         // Unsigned
    SLT:  alu_res = (signed_a < signed_b) ? 32'd1 : '0;   // Signed
    
    default: alu_res = '0;
  endcase
end 

// Generate Zero flag
// High when the result is 0 (Used primarily for BEQ/BNE branch evaluation)
assign zero = (alu_res == '0);

endmodule