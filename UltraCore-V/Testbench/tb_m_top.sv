// Testbench - SIMULATION ONLY
// In-pipeline M-extension test: the RAM program computes MUL/DIV/REM,
// forwards the results through dependent ADDs, and writes the 8-bit
// sum (0x3A) to GPIO. Proves M-op decode, the stall-based functional
// unit, writeback, and forwarding-after-stall on the real core.
`timescale 1ns/1ps
module tb_m_top;
  int errors = 0;
  logic clk = 0; always #5 clk = ~clk;
  logic resetn = 0;
  logic uart_txd; logic uart_rxd = 1; wire [7:0] gpio; logic [31:0] debug_pc; logic bus_timeout;

  soc_top #(.CLK_FREQ_HZ(100_000_000), .BAUD_RATE(1_562_500), .MEM_INIT_FILE("tb_m_app.mem")) u_soc (
    .clk(clk), .resetn(resetn), .uart_txd(uart_txd), .uart_rxd(uart_rxd),
    .gpio(gpio), .debug_pc(debug_pc), .bus_timeout(bus_timeout));

  task automatic check(input bit c, input string m);
    if (c) $display("[%0t ns] OK   : %s", $time, m);
    else begin $display("[%0t ns] ERROR: %s", $time, m); errors++; end
  endtask

  initial begin
    #3_000_000; $display("ERROR: watchdog (pc=%08h gpio=%b)", debug_pc, gpio); $finish;
  end

  initial begin
    repeat (20) @(negedge clk); resetn = 1;
    // wait for the program's GPIO signature
    begin
      int waited = 0;
      while (gpio !== 8'h3A && waited < 200000) begin repeat (100) @(posedge clk); waited += 100; end
    end
    check(gpio === 8'h3A, "GPIO = 0x3A (MUL/DIV/REM + forwarding correct)");
    check(bus_timeout === 1'b0, "no bus timeout");
    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
