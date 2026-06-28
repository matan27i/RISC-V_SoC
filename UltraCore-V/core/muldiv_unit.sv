// M-extension Multiply/Divide Unit - RTL
// Iterative, stall-based functional unit for the EX stage. The parent
// pipeline freezes (muldiv_stall = req & ~done) while this unit is busy,
// exactly like a cache miss, so no result-forwarding scoreboard is
// needed - a dependent instruction cannot leave ID until the result is
// registered into EX/MEM.
//
// funct3 select (RV32M, funct7 = 0000001):
//   000 MUL     low 32 bits of rs1 * rs2
//   001 MULH    upper 32 bits, signed   * signed
//   010 MULHSU  upper 32 bits, signed   * unsigned
//   011 MULHU   upper 32 bits, unsigned * unsigned
//   100 DIV     signed   quotient      101 DIVU  unsigned quotient
//   110 REM     signed   remainder     111 REMU  unsigned remainder
//
// Multiply: 33x33 product (DSP-mappable), registered through MUL_LATENCY
// cycles to model a pipelined multiplier. Divide: restoring radix-2,
// one quotient bit per cycle (32 cycles), producing the RISC-V-defined
// results for divide-by-zero and signed overflow (no trap).
//
// Handshake:
//   req     - level: an M-op occupies EX and is valid (held by the parent
//             while muldiv_stall freezes the pipe)
//   advance - level: EX is allowed to retire this cycle (no cache/MMIO
//             stall). The unit leaves DONE only when advance=1, so a
//             completed result is held if another stall is active and the
//             op is never restarted while it lingers in EX.
//   busy/done - status; result is valid while done=1.

module muldiv_unit #(
  parameter int unsigned MUL_LATENCY = 2          // >= 1, models DSP depth
)(
  input  logic        clk,
  input  logic        reset_n,

  input  logic        req,
  input  logic        advance,
  input  logic [2:0]  funct3,
  input  logic [31:0] op_a,
  input  logic [31:0] op_b,

  output logic [31:0] result,
  output logic        busy,
  output logic        done
);

  localparam int unsigned LW = (MUL_LATENCY < 2) ? 1 : $clog2(MUL_LATENCY+1);

  typedef enum logic [1:0] {MD_IDLE, MD_MUL, MD_DIV, MD_DONE} state_t;
  state_t state;

  logic        start;
  assign start = (state == MD_IDLE) && req;

  // Latched request
  logic [2:0]  op_q;
  logic [31:0] a_q, b_q;

  // ---- Multiply (combinational from latched operands) ----
  logic               a_signed, b_signed;
  assign a_signed = (op_q == 3'b001) || (op_q == 3'b010);   // MULH, MULHSU
  assign b_signed = (op_q == 3'b001);                        // MULH
  logic signed [33:0] a_ext, b_ext;
  logic signed [67:0] product;
  assign a_ext   = $signed({a_signed & a_q[31], a_q});
  assign b_ext   = $signed({b_signed & b_q[31], b_q});
  assign product = a_ext * b_ext;
  logic [31:0] mul_result;
  assign mul_result = (op_q == 3'b000) ? product[31:0] : product[63:32];

  logic [LW-1:0] mul_cnt;

  // ---- Divide (restoring, registered state) ----
  logic        div_signed_q;
  assign div_signed_q = (op_q == 3'b100) || (op_q == 3'b110);  // DIV, REM
  logic        quo_neg, rem_neg, div_special;
  logic [31:0] divisor_abs, quo_acc, spec_quo, spec_rem;
  logic [31:0] rem_acc;
  logic [5:0]  div_cnt;

  // One restoring step (combinational)
  logic [32:0] rem_shifted, rem_minus;
  assign rem_shifted = {rem_acc[31:0], quo_acc[31]};
  assign rem_minus   = rem_shifted - {1'b0, divisor_abs};
  logic        step_sub;
  assign step_sub = (rem_shifted >= {1'b0, divisor_abs});

  // Operand magnitudes / signs computed from the *incoming* op at start
  logic        in_signed;
  assign in_signed = (funct3 == 3'b100) || (funct3 == 3'b110);
  logic [31:0] a_abs_in, b_abs_in;
  assign a_abs_in = (in_signed && op_a[31]) ? (~op_a + 32'd1) : op_a;
  assign b_abs_in = (in_signed && op_b[31]) ? (~op_b + 32'd1) : op_b;

  // ---- FSM + datapath (single sequential block) ----
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      state       <= MD_IDLE;
      op_q        <= 3'b0;  a_q <= 32'b0;  b_q <= 32'b0;
      mul_cnt     <= '0;
      rem_acc     <= 32'b0; quo_acc <= 32'b0; div_cnt <= 6'b0;
      divisor_abs <= 32'b0; quo_neg <= 1'b0; rem_neg <= 1'b0;
      div_special <= 1'b0;  spec_quo <= 32'b0; spec_rem <= 32'b0;
    end
    else begin
      case (state)
        MD_IDLE: if (start) begin
          op_q <= funct3;  a_q <= op_a;  b_q <= op_b;
          if (!funct3[2]) begin
            mul_cnt <= LW'(MUL_LATENCY);
            state   <= MD_MUL;
          end
          else begin
            // Divide setup
            quo_neg     <= (funct3 == 3'b100) ? (op_a[31] ^ op_b[31]) : 1'b0;
            rem_neg     <= (funct3 == 3'b110) ? op_a[31] : 1'b0;
            divisor_abs <= b_abs_in;
            quo_acc     <= a_abs_in;     // |dividend| shifts left into the quotient
            rem_acc     <= 32'b0;
            div_cnt     <= 6'd32;
            // Spec corner cases bypass the iteration
            if (op_b == 32'b0) begin
              div_special <= 1'b1;
              spec_quo    <= 32'hFFFF_FFFF;   // quotient = -1
              spec_rem    <= op_a;            // remainder = dividend
            end
            else if (in_signed && (op_a == 32'h8000_0000) && (op_b == 32'hFFFF_FFFF)) begin
              div_special <= 1'b1;
              spec_quo    <= 32'h8000_0000;   // INT_MIN
              spec_rem    <= 32'b0;
            end
            else begin
              div_special <= 1'b0;
            end
            state <= MD_DIV;
          end
        end

        MD_MUL: begin
          if (mul_cnt <= LW'(1)) state <= MD_DONE;
          else                   mul_cnt <= mul_cnt - 1'b1;
        end

        MD_DIV: begin
          if (div_special || div_cnt == 6'd0) begin
            state <= MD_DONE;
          end
          else begin
            rem_acc <= step_sub ? rem_minus[31:0] : rem_shifted[31:0];
            quo_acc <= {quo_acc[30:0], step_sub};
            div_cnt <= div_cnt - 1'b1;
          end
        end

        MD_DONE: if (advance) state <= MD_IDLE;  // retire only when EX can advance

        default: state <= MD_IDLE;
      endcase
    end
  end

  // ---- Result selection ----
  logic [31:0] div_result;
  always_comb begin
    if (div_special)
      div_result = ((op_q == 3'b110) || (op_q == 3'b111)) ? spec_rem : spec_quo;
    else if ((op_q == 3'b110) || (op_q == 3'b111))         // REM / REMU
      div_result = rem_neg ? (~rem_acc + 32'd1) : rem_acc;
    else                                                    // DIV / DIVU
      div_result = quo_neg ? (~quo_acc + 32'd1) : quo_acc;
  end

  assign result = op_q[2] ? div_result : mul_result;
  assign busy   = (state == MD_MUL) || (state == MD_DIV);
  assign done   = (state == MD_DONE);

endmodule
