// Decode - RTL
import risc_pkg::*;

module decode(
  input  logic [31:0]  instruction,  // 32-bit instruction from fetch stage

  // Decoded fields
  output logic [4:0]   rs1_addr,
  output logic [4:0]   rs2_addr,
  output logic [4:0]   rd_addr,
  output logic [6:0]   opcode,
  output logic [2:0]   funct3,
  output logic [6:0]   funct7,
  output logic         r_type,
  output logic         i_type,
  output logic         s_type,
  output logic         b_type,
  output logic         u_type,
  output logic         j_type,
  output logic [31:0]  immediate     // Final immediate value
);

// Internal signals
logic [31:0] imm_i_type, imm_s_type, imm_b_type, imm_u_type, imm_j_type;
  
// Extract base fields
assign opcode   = instruction[6:0];
assign rd_addr  = instruction[11:7];
assign funct3   = instruction[14:12];
assign rs1_addr = instruction[19:15];
assign rs2_addr = instruction[24:20];
assign funct7   = instruction[31:25];


// Immediate values for different instruction types
assign imm_i_type = { {20{instruction[31]}}, instruction[31:20] };
assign imm_s_type = { {21{instruction[31]}}, instruction[30:25], instruction[11:7] };
assign imm_b_type = { {20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0 };
assign imm_u_type = { instruction[31:12], 12'b0 };
assign imm_j_type = { {12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0 };

// Instruction type decoding
always_comb begin
    // Default assignments to prevent latches
    r_type = 1'b0;
    i_type = 1'b0;
    s_type = 1'b0;
    b_type = 1'b0;
    u_type = 1'b0;
    j_type = 1'b0;
    
    case (opcode)
        OPCODE_R_TYPE: r_type = 1'b1;
        OPCODE_I_LOAD, OPCODE_I_ALU, OPCODE_I_JALR: i_type = 1'b1;
        OPCODE_S_TYPE: s_type = 1'b1;
        OPCODE_B_TYPE: b_type = 1'b1;
        OPCODE_LUI, OPCODE_AUIPC: u_type = 1'b1;
        OPCODE_JAL: j_type = 1'b1;
        default: begin
            r_type = 1'b0;
            i_type = 1'b0;
            s_type = 1'b0;
            b_type = 1'b0;
            u_type = 1'b0;
            j_type = 1'b0;
        end
    endcase
end

// Final immediate selection based on instruction type
assign immediate = i_type ? imm_i_type :
                   s_type ? imm_s_type :
                   b_type ? imm_b_type :
                   u_type ? imm_u_type :
                   j_type ? imm_j_type : '0;
                     

endmodule