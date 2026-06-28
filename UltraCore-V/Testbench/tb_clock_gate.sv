// Testbench - SIMULATION ONLY
// Verifies the clock gate passes edges only while enabled and produces
// no edges (no toggles, no glitches) while disabled.
`timescale 1ns/1ps
module tb_clock_gate;
  int errors = 0;
  logic clk = 0; always #5 clk = ~clk;
  logic enable = 0, test_en = 0, gclk;
  int gedges = 0;

  clock_gate dut(.clk(clk), .enable(enable), .test_en(test_en), .gated_clk(gclk));

  // count rising edges on the gated clock
  always @(posedge gclk) gedges++;

  task automatic check(input bit c, input string m);
    if (c) $display("OK   : %s", m); else begin $display("ERROR: %s", m); errors++; end
  endtask

  initial begin
    // disabled: expect no gated edges
    enable = 0; repeat (10) @(posedge clk);
    check(gedges == 0, "no gated edges while disabled");

    // enabled: expect ~10 edges over 10 cycles
    @(negedge clk) enable = 1;
    gedges = 0;
    repeat (10) @(posedge clk);
    @(negedge clk) enable = 0;
    check(gedges == 10, "gated clock toggles while enabled (10 edges)");

    // disabled again: frozen
    gedges = 0; repeat (10) @(posedge clk);
    check(gedges == 0, "gated clock frozen again when disabled");

    // test_en forces clock on regardless of enable
    @(negedge clk) test_en = 1; gedges = 0;
    repeat (5) @(posedge clk);
    @(negedge clk) test_en = 0;
    check(gedges == 5, "test_en forces the clock on (DFT bypass)");

    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
