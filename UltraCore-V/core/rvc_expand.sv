// RVC Decompressor (C-extension) - RTL
// Combinational expander: maps a 16-bit RV32C instruction to its
// equivalent 32-bit RV32I instruction. Placing this between the fetch
// realign buffer and the existing decode.sv lets the entire decode/EX
// datapath stay unchanged ("expand-then-decode"). `illegal` flags
// unmapped / reserved encodings (feeds the illegal-instruction trap).
//
// Covers the full RV32C integer set across the three quadrants. The
// compressed immediate fields are heavily bit-scrambled per the spec;
// each is reassembled below and fed through the standard I/S/B/U/J
// encoders so the result is a canonical base-ISA instruction.

module rvc_expand (
  input  logic [15:0] c_instr,
  output logic [31:0] instr_out,
  output logic        illegal
);

  // Register fields
  logic [4:0] rd_rs1, rs2;          // full (x0..x31) fields
  logic [4:0] rdp, rs1p, rs2p;      // popular (x8..x15) fields
  assign rd_rs1 = c_instr[11:7];
  assign rs2    = c_instr[6:2];
  assign rdp    = {2'b01, c_instr[9:7]};   // rd'/rs1' in CB/CA  (instr[9:7] + 8)
  assign rs1p   = {2'b01, c_instr[9:7]};   // rs1'  in CL/CS     (instr[9:7] + 8)
  assign rs2p   = {2'b01, c_instr[4:2]};   // rs2'/rd' low field (instr[4:2] + 8)

  logic [1:0] quadrant;
  logic [2:0] funct3;
  assign quadrant = c_instr[1:0];
  assign funct3   = c_instr[15:13];

  // ---- Base-ISA field encoders ----
  function automatic logic [31:0] enc_i(input logic [11:0] imm, input logic [4:0] rs1,
                                        input logic [2:0] f3, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm, rs1, f3, rd, op};
  endfunction
  function automatic logic [31:0] enc_s(input logic [11:0] imm, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3,
                                        input logic [6:0] op);
    return {imm[11:5], rs2, rs1, f3, imm[4:0], op};
  endfunction
  function automatic logic [31:0] enc_r(input logic [6:0] f7, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3,
                                        input logic [4:0] rd, input logic [6:0] op);
    return {f7, rs2, rs1, f3, rd, op};
  endfunction
  function automatic logic [31:0] enc_b(input logic [12:0] imm, input logic [4:0] rs2,
                                        input logic [4:0] rs1, input logic [2:0] f3,
                                        input logic [6:0] op);
    return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], op};
  endfunction
  function automatic logic [31:0] enc_u(input logic [31:0] imm, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm[31:12], rd, op};
  endfunction
  function automatic logic [31:0] enc_j(input logic [20:0] imm, input logic [4:0] rd,
                                        input logic [6:0] op);
    return {imm[20], imm[10:1], imm[11], imm[19:12], rd, op};
  endfunction

  localparam logic [6:0] OP_OPIMM = 7'b0010011, OP_LOAD = 7'b0000011,
                         OP_STORE = 7'b0100011, OP_OP   = 7'b0110011,
                         OP_LUI   = 7'b0110111, OP_JAL  = 7'b1101111,
                         OP_JALR  = 7'b1100111, OP_BRANCH= 7'b1100011,
                         OP_SYSTEM= 7'b1110011;

  // ---- Reassembled immediates ----
  logic [11:0] imm_addi4spn, imm_lw, imm_swsp, imm_lwsp;
  logic [11:0] imm_ci, imm_addi16sp;
  logic [31:0] imm_lui;
  logic [20:0] imm_cj;
  logic [12:0] imm_cb;
  logic [4:0]  shamt;

  assign imm_addi4spn = {2'b0, c_instr[10:7], c_instr[12:11], c_instr[5], c_instr[6], 2'b00};
  assign imm_lw       = {5'b0, c_instr[5], c_instr[12:10], c_instr[6], 2'b00};
  assign imm_lwsp     = {4'b0, c_instr[3:2], c_instr[12], c_instr[6:4], 2'b00};
  assign imm_swsp     = {4'b0, c_instr[8:7], c_instr[12:9], 2'b00};
  assign imm_ci       = {{6{c_instr[12]}}, c_instr[12], c_instr[6:2]};       // sign-ext 6-bit
  assign imm_addi16sp = {{2{c_instr[12]}}, c_instr[12], c_instr[4:3], c_instr[5],
                         c_instr[2], c_instr[6], 4'b0};                       // *16, sign-ext
  assign imm_lui      = {{14{c_instr[12]}}, c_instr[12], c_instr[6:2], 12'b0};// nzimm<<12
  assign imm_cj       = {{9{c_instr[12]}}, c_instr[12], c_instr[8], c_instr[10:9],
                         c_instr[6], c_instr[7], c_instr[2], c_instr[11], c_instr[5:3], 1'b0};
  assign imm_cb       = {{4{c_instr[12]}}, c_instr[12], c_instr[6:5], c_instr[2],
                         c_instr[11:10], c_instr[4:3], 1'b0};
  assign shamt        = {c_instr[12], c_instr[6:2]};

  // ---- Expansion ----
  always_comb begin
    instr_out = 32'h0000_0013;   // default NOP
    illegal   = 1'b0;

    unique case (quadrant)
      // ============ Quadrant 0 ============
      2'b00: begin
        unique case (funct3)
          3'b000: begin // C.ADDI4SPN -> addi rd', x2, imm   (rd' = instr[4:2])
            if (c_instr[12:5] == 8'b0) illegal = 1'b1;   // reserved (imm=0)
            else instr_out = enc_i(imm_addi4spn, 5'd2, 3'b000, rs2p, OP_OPIMM);
          end
          3'b010:  instr_out = enc_i(imm_lw, rs1p, 3'b010, rs2p, OP_LOAD);   // C.LW (rd' = instr[4:2])
          3'b110:  instr_out = enc_s(imm_lw, rs2p, rs1p, 3'b010, OP_STORE);  // C.SW
          default: illegal = 1'b1;
        endcase
      end

      // ============ Quadrant 1 ============
      2'b01: begin
        unique case (funct3)
          3'b000:  instr_out = enc_i(imm_ci, rd_rs1, 3'b000, rd_rs1, OP_OPIMM); // C.ADDI / C.NOP
          3'b001:  instr_out = enc_j({imm_cj}, 5'd1, OP_JAL);                    // C.JAL  -> jal x1
          3'b010:  instr_out = enc_i(imm_ci, 5'd0, 3'b000, rd_rs1, OP_OPIMM);    // C.LI   -> addi rd,x0
          3'b011: begin
            if (rd_rs1 == 5'd2)                                                  // C.ADDI16SP
              instr_out = enc_i(imm_addi16sp, 5'd2, 3'b000, 5'd2, OP_OPIMM);
            else if (rd_rs1 != 5'd0)                                             // C.LUI
              instr_out = enc_u(imm_lui, rd_rs1, OP_LUI);
            else illegal = 1'b1;
          end
          3'b100: begin // CA / shift group
            unique case (c_instr[11:10])
              2'b00: instr_out = enc_i({7'b0000000, shamt}, rs1p, 3'b101, rdp, OP_OPIMM);     // C.SRLI
              2'b01: instr_out = enc_i({7'b0100000, shamt}, rs1p, 3'b101, rdp, OP_OPIMM);     // C.SRAI
              2'b10: instr_out = enc_i(imm_ci, rs1p, 3'b111, rdp, OP_OPIMM);                   // C.ANDI
              2'b11: begin // register-register
                unique case (c_instr[6:5])
                  2'b00: instr_out = enc_r(7'b0100000, rs2p, rs1p, 3'b000, rdp, OP_OP); // C.SUB
                  2'b01: instr_out = enc_r(7'b0000000, rs2p, rs1p, 3'b100, rdp, OP_OP); // C.XOR
                  2'b10: instr_out = enc_r(7'b0000000, rs2p, rs1p, 3'b110, rdp, OP_OP); // C.OR
                  2'b11: instr_out = enc_r(7'b0000000, rs2p, rs1p, 3'b111, rdp, OP_OP); // C.AND
                endcase
              end
            endcase
          end
          3'b101:  instr_out = enc_j({imm_cj}, 5'd0, OP_JAL);                    // C.J -> jal x0
          3'b110:  instr_out = enc_b(imm_cb, 5'd0, rs1p, 3'b000, OP_BRANCH);     // C.BEQZ
          3'b111:  instr_out = enc_b(imm_cb, 5'd0, rs1p, 3'b001, OP_BRANCH);     // C.BNEZ
          default: illegal = 1'b1;
        endcase
      end

      // ============ Quadrant 2 ============
      2'b10: begin
        unique case (funct3)
          3'b000:  instr_out = enc_i({7'b0, shamt}, rd_rs1, 3'b001, rd_rs1, OP_OPIMM); // C.SLLI
          3'b010: begin                                                                // C.LWSP
            if (rd_rs1 == 5'd0) illegal = 1'b1;
            else instr_out = enc_i(imm_lwsp, 5'd2, 3'b010, rd_rs1, OP_LOAD);
          end
          3'b100: begin
            if (c_instr[12] == 1'b0) begin
              if (rs2 == 5'd0) begin
                if (rd_rs1 == 5'd0) illegal = 1'b1;
                else instr_out = enc_i(12'b0, rd_rs1, 3'b000, 5'd0, OP_JALR);  // C.JR -> jalr x0,0(rs1)
              end
              else                                                            // C.MV -> add rd,x0,rs2
                instr_out = enc_r(7'b0, rs2, 5'd0, 3'b000, rd_rs1, OP_OP);
            end
            else begin
              if (rs2 == 5'd0) begin
                if (rd_rs1 == 5'd0)                                           // C.EBREAK
                  instr_out = enc_i(12'b1, 5'd0, 3'b000, 5'd0, OP_SYSTEM);
                else                                                         // C.JALR -> jalr x1,0(rs1)
                  instr_out = enc_i(12'b0, rd_rs1, 3'b000, 5'd1, OP_JALR);
              end
              else                                                           // C.ADD -> add rd,rd,rs2
                instr_out = enc_r(7'b0, rs2, rd_rs1, 3'b000, rd_rs1, OP_OP);
            end
          end
          3'b110:  instr_out = enc_s(imm_swsp, rs2, 5'd2, 3'b010, OP_STORE);   // C.SWSP
          default: illegal = 1'b1;
        endcase
      end

      // Quadrant 3 = not compressed (16-bit input only)
      default: illegal = 1'b1;
    endcase
  end

endmodule
