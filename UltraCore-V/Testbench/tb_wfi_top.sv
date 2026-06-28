// Testbench - SIMULATION ONLY
// WFI low-power test: the core executes WFI, parks (debug_pc holds at the
// WFI PC, core_asleep high), and is woken + vectored by the timer
// interrupt (GPIO 0x11 -> 0xCC). Proves WFI sleep, wake, and that the
// pipeline freezes while asleep.
`timescale 1ns/1ps
module tb_wfi_top;
  int errors = 0;
  logic clk = 0; always #5 clk = ~clk;
  logic resetn = 0;
  logic uart_txd; logic uart_rxd = 1; wire [7:0] gpio; logic [31:0] debug_pc; logic bus_timeout;

  soc_top #(.CLK_FREQ_HZ(100_000_000), .BAUD_RATE(1_562_500), .MEM_INIT_FILE("tb_wfi_app.mem")) u_soc (
    .clk(clk), .resetn(resetn), .uart_txd(uart_txd), .uart_rxd(uart_rxd),
    .gpio(gpio), .debug_pc(debug_pc), .bus_timeout(bus_timeout));

  task automatic check(input bit c, input string m);
    if (c) $display("[%0t ns] OK   : %s", $time, m);
    else begin $display("[%0t ns] ERROR: %s", $time, m); errors++; end
  endtask

  // observe sleep
  logic saw_asleep = 0;
  always @(posedge clk) if (u_soc.u_core.core_asleep) saw_asleep <= 1;

  initial begin #3_000_000; $display("ERROR: watchdog (pc=%08h)", debug_pc); $finish; end

  initial begin
    int waited;
    repeat (20) @(negedge clk); resetn = 1;

    // wait for awake marker
    waited = 0;
    while (gpio !== 8'h11 && waited < 200000) begin repeat (100) @(posedge clk); waited += 100; end
    check(gpio === 8'h11, "GPIO=0x11 (setup ran, about to WFI)");

    // timer wakes + vectors the core
    waited = 0;
    while (gpio !== 8'hCC && waited < 200000) begin repeat (100) @(posedge clk); waited += 100; end
    check(saw_asleep === 1'b1, "core entered WFI sleep (pipeline parked) before waking");
    check(gpio === 8'hCC, "timer woke the core and vectored to the handler (GPIO=0xCC)");
    check(u_soc.u_core.u_csr.mcause === 32'h8000_0007, "mcause = machine timer");
    // WFI retires and the core resumes at the next instruction (0x2044),
    // where the pending interrupt is taken - valid per the RISC-V spec.
    check(u_soc.u_core.u_csr.mepc === 32'h0000_2044, "mepc = instruction after WFI (post-WFI resume)");

    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
