// Testbench - SIMULATION ONLY (do not add to synthesis sources)
// Full-system boot test for the UltraCore-V SoC:
//   1. Reset: core boots from the ROM at 0x0, bootloader polls the UART
//      through the AXI bridge and prints "LIVE\r\n" on txd (checked by
//      an 8N1 line decoder).
//   2. Bootloader jumps to main RAM (0x2000); the application configures
//      GPIO DIR/DATA -> pins must show 0xA5 (proves ROM->RAM handoff
//      plus MMIO stores through bridge + crossbar to GPIO).
//   3. Application starts the timer (compare=100) -> the routed
//      irq_timer line must be sticky-high.
//   4. TB sends a 13-bit SEC-DED frame (payload 0x42, one injected bit
//      error) as two UART bytes into rxd. Hardware path: UART RX ->
//      frame packer -> ECC accelerator corrects -> CPU polls STATUS,
//      reads the corrected byte, writes it to GPIO -> pins show 0x42.
//
// The UART baud is overridden to 1.5625 MBd (divider = exactly 4 at
// 100 MHz, zero baud error) purely to keep simulation time short.

`timescale 1ns/1ps

module tb_soc_top;

  // 1.5625 MBd -> 640 ns per bit at the 100 MHz system clock
  localparam int unsigned TB_BAUD = 1_562_500;
  localparam int unsigned BIT_NS  = 640;

  int errors = 0;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic resetn = 1'b0;

  logic        uart_txd;
  logic        uart_rxd = 1'b1;     // Idle high
  wire  [7:0]  gpio;                // TB never drives: observe only
  logic [31:0] debug_pc;
  logic        bus_timeout;

  // Sticky record of any watchdog firing during the run
  logic        bus_timeout_seen = 1'b0;
  always @(posedge clk) if (bus_timeout) bus_timeout_seen <= 1'b1;

  soc_top #(
    .CLK_FREQ_HZ   (100_000_000),
    .BAUD_RATE     (TB_BAUD),
    .MEM_INIT_FILE ("tb_soc_app.mem")
  ) u_soc (
    .clk         (clk),
    .resetn      (resetn),
    .uart_txd    (uart_txd),
    .uart_rxd    (uart_rxd),
    .gpio        (gpio),
    .debug_pc    (debug_pc),
    .bus_timeout (bus_timeout)
  );

  // ----------------------------------------------------------------
  // Reference SEC-DED encoder (contract from ecc_accel.sv header)
  // ----------------------------------------------------------------
  function automatic logic [12:0] ecc_encode(input logic [7:0] d);
    logic c0, c1, c2, c3, p;
    c0 = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6];
    c1 = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6];
    c2 = d[1] ^ d[2] ^ d[3] ^ d[7];
    c3 = d[4] ^ d[5] ^ d[6] ^ d[7];
    p  = ^{c3, c2, c1, c0, d};
    return {p, c3, c2, c1, c0, d};
  endfunction

  // ----------------------------------------------------------------
  // 8N1 line monitor / driver
  // ----------------------------------------------------------------
  task automatic uart_recv_byte(output logic [7:0] b);
    @(negedge uart_txd);             // Start-bit edge
    #(BIT_NS / 2);
    if (uart_txd !== 1'b0) begin
      $display("[%0t] ERROR: start bit not low at mid-bit", $time);
      errors++;
    end
    for (int i = 0; i < 8; i++) begin
      #(BIT_NS);
      b[i] = uart_txd;               // LSB first, center sampling
    end
    #(BIT_NS);
    if (uart_txd !== 1'b1) begin
      $display("[%0t] ERROR: stop bit not high", $time);
      errors++;
    end
  endtask

  task automatic uart_send_byte(input logic [7:0] b);
    uart_rxd = 1'b0;                 // Start bit
    #(BIT_NS);
    for (int i = 0; i < 8; i++) begin
      uart_rxd = b[i];               // LSB first
      #(BIT_NS);
    end
    uart_rxd = 1'b1;                 // Stop bit + idle gap
    #(2 * BIT_NS);
  endtask

  task automatic check(input bit cond, input string msg);
    if (cond)
      $display("[%0t ns] OK   : %s", $time, msg);
    else begin
      $display("[%0t ns] ERROR: %s", $time, msg);
      errors++;
    end
  endtask

  // Bounded wait for an exact GPIO pattern
  task automatic wait_gpio(input logic [7:0] val, input int timeout_us,
                           input string msg);
    int waited = 0;
    while (gpio !== val && waited < timeout_us * 100) begin
      repeat (1000) @(posedge clk);  // 10 us steps
      waited += 1000;
    end
    check(gpio === val, msg);
  endtask

  // ----------------------------------------------------------------
  // Watchdog
  // ----------------------------------------------------------------
  initial begin
    #5_000_000;    // 5 ms
    $display("ERROR: watchdog timeout (pc=0x%08h, gpio=%b)", debug_pc, gpio);
    $finish;
  end

  // ----------------------------------------------------------------
  // Main sequence
  // ----------------------------------------------------------------
  logic [7:0]  banner [0:5];
  logic [7:0]  rx_ch;
  logic [12:0] frame;

  initial begin
    banner[0] = "L"; banner[1] = "I"; banner[2] = "V"; banner[3] = "E";
    banner[4] = 8'h0D; banner[5] = 8'h0A;

    repeat (20) @(negedge clk);
    resetn = 1'b1;

    // ---- Phase 1: boot banner ----
    $display("---- Phase 1: boot ROM banner over UART ----");
    for (int i = 0; i < 6; i++) begin
      uart_recv_byte(rx_ch);
      if (rx_ch === banner[i])
        $display("[%0t ns] OK   : banner char %0d = 0x%02h ('%c')",
                 $time, i, rx_ch, (rx_ch >= 8'h20) ? rx_ch : 8'h2E);
      else begin
        $display("[%0t ns] ERROR: banner char %0d = 0x%02h (expected 0x%02h)",
                 $time, i, rx_ch, banner[i]);
        errors++;
      end
    end

    // ---- Phase 2: jump to RAM + GPIO via AXI ----
    $display("---- Phase 2: application in main RAM drives GPIO ----");
    wait_gpio(8'hA5, 1000, "GPIO pins show 0xA5 written by the RAM application");
    check(debug_pc >= 32'h0000_2000, "PC is executing from main RAM (>= 0x2000)");

    // ---- Phase 3: timer interrupt routed to the core ----
    // compare=100 was passed long ago; the sticky flag must be high
    check(u_soc.irq_timer_w === 1'b1, "timer irq line high and routed to core");
    check(u_soc.irq_ecc_w   === 1'b0, "ecc irq line idle (IRQ_EN not set)");

    // ---- Phase 4: end-to-end ECC receive path ----
    $display("---- Phase 4: UART -> packer -> ECC -> CPU -> GPIO ----");
    frame = ecc_encode(8'h42) ^ 13'h0002;        // Inject single-bit error (D1)
    uart_send_byte(frame[7:0]);                  // byte 0: payload field
    uart_send_byte({3'b000, frame[12:8]});       // byte 1: {P,C3..C0}
    wait_gpio(8'h42, 1000, "corrected ECC payload 0x42 reached the GPIO pins");

    // ---- Phase 5: watchdog transparency ----
    // All peripherals are healthy, so the bus watchdog must never have
    // fired across the entire boot+run sequence above.
    check(bus_timeout_seen === 1'b0, "AXI watchdog stayed transparent (no false timeout)");

    // ---- Summary ----
    if (errors == 0)
      $display("==== ALL TESTS PASSED ====");
    else
      $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end

endmodule
