// Testbench - SIMULATION ONLY
// Checks the RVC decompressor: each 16-bit C instruction must expand to
// the independently-encoded 32-bit RV32I in tb_rvc_exp.mem.
`timescale 1ns/1ps
module tb_rvc_expand;
  localparam int NV = 24;
  logic [15:0] cvec [0:NV-1];
  logic [31:0] evec [0:NV-1];
  int errors = 0;

  logic [15:0] c_instr;
  logic [31:0] instr_out;
  logic        illegal;

  rvc_expand dut(.c_instr(c_instr), .instr_out(instr_out), .illegal(illegal));

  initial begin
    $readmemh("tb_rvc_c.mem", cvec);
    $readmemh("tb_rvc_exp.mem", evec);
    for (int i = 0; i < NV; i++) begin
      c_instr = cvec[i];
      #1;
      if (illegal) begin
        $display("ERROR: vec %0d c=%04h flagged illegal", i, cvec[i]); errors++;
      end
      else if (instr_out !== evec[i]) begin
        $display("ERROR: vec %0d c=%04h -> %08h  exp %08h", i, cvec[i], instr_out, evec[i]); errors++;
      end
      else
        $display("OK   : vec %0d c=%04h -> %08h", i, cvec[i], instr_out);
      #1;
    end

    // a couple of illegal encodings
    c_instr = 16'h0000; #1;   // all-zero = reserved/illegal
    if (!illegal) begin $display("ERROR: c=0000 should be illegal"); errors++; end
    else $display("OK   : c=0000 flagged illegal");

    if (errors == 0) $display("==== ALL TESTS PASSED ==== (%0d expansions)", NV);
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
