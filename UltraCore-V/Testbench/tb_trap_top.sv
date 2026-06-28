// Testbench - SIMULATION ONLY
// End-to-end interrupt test for the Zicsr/trap unit on the full SoC.
// The program (tb_trap_app.mem, loaded at 0x2000) sets mtvec/mie/mstatus
// via CSR instructions, marks GPIO=0x11, starts the timer, then spins.
// When the timer fires, the core must vector to the handler (GPIO=0xCC),
// capture mepc=loop PC and mcause=machine-timer, ack+mret, and resume.
//
// This proves interrupts VECTOR the core (asynchronously) instead of the
// old poll-the-STATUS-register approach.

`timescale 1ns/1ps

module tb_trap_top;
  localparam int unsigned TB_BAUD = 1_562_500;  // fast banner (boot ROM runs first)

  int errors = 0;
  logic clk = 1'b0; always #5 clk = ~clk;
  logic resetn = 1'b0;

  logic        uart_txd;
  logic        uart_rxd = 1'b1;
  wire  [7:0]  gpio;
  logic [31:0] debug_pc;
  logic        bus_timeout;

  soc_top #(
    .CLK_FREQ_HZ   (100_000_000),
    .BAUD_RATE     (TB_BAUD),
    .MEM_INIT_FILE ("tb_trap_app.mem")
  ) u_soc (
    .clk(clk), .resetn(resetn),
    .uart_txd(uart_txd), .uart_rxd(uart_rxd),
    .gpio(gpio), .debug_pc(debug_pc), .bus_timeout(bus_timeout)
  );

  task automatic check(input bit c, input string m);
    if (c) $display("[%0t ns] OK   : %s", $time, m);
    else begin $display("[%0t ns] ERROR: %s", $time, m); errors++; end
  endtask

  task automatic wait_gpio(input logic [7:0] val, input int timeout_us, input string m);
    int waited = 0;
    while (gpio !== val && waited < timeout_us*100) begin
      repeat (100) @(posedge clk); waited += 100;
    end
    check(gpio === val, m);
  endtask

  // observe whether the handler vector was ever entered
  logic entered_handler = 1'b0;
  always @(posedge clk) if (debug_pc == 32'h0000_2044) entered_handler <= 1'b1;

  initial begin
    #3_000_000;
    $display("ERROR: watchdog timeout (pc=0x%08h gpio=%b)", debug_pc, gpio);
    $finish;
  end

  initial begin
    repeat (20) @(negedge clk);
    resetn = 1'b1;

    // Pre-interrupt: the RAM program ran its CSR setup and marked GPIO
    wait_gpio(8'h11, 1500, "GPIO=0x11 set by RAM program (CSR setup survived)");
    check(debug_pc >= 32'h0000_2000, "executing from main RAM");

    // The timer interrupt must asynchronously vector to the handler
    wait_gpio(8'hCC, 1500, "GPIO=0xCC written by the interrupt handler");
    check(entered_handler === 1'b1, "core fetched the mtvec handler (PC hit 0x2044)");

    // CSR trap state captured precisely
    check(u_soc.u_core.u_csr.mcause === 32'h8000_0007, "mcause = machine timer interrupt");
    check(u_soc.u_core.u_csr.mepc   === 32'h0000_2040, "mepc = interrupted spin-loop PC (0x2040)");

    // After mret the core resumed in the loop and stays alive / GPIO holds
    repeat (2000) @(posedge clk);
    check(gpio === 8'hCC, "GPIO holds 0xCC after mret (no re-trigger, timer acked)");
    check(debug_pc >= 32'h0000_2000 && debug_pc <= 32'h0000_2044,
          "PC back in the main-loop region after mret");
    check(u_soc.u_core.u_csr.mstatus_mie === 1'b1, "mstatus.MIE re-enabled by mret");
    check(bus_timeout === 1'b0, "no spurious bus timeout");

    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
