// Testbench - SIMULATION ONLY (do not add to synthesis sources)
// Self-checking smoke test for the uart and timer IP cores:
//   1. UART loopback: 5 bytes streamed back-to-back through txd -> rxd
//   2. UART error path: bit-banged frames into a second instance,
//      including a broken stop bit (framing error) and recovery
//   3. Timer: interrupt timing, stickiness, clear/ack, enable-hold,
//      second period, and the compare_value = 0 corner case

`timescale 1ns/1ps

module tb_uart_timer;

  localparam int unsigned CLK_FREQ = 100_000_000;
  localparam int unsigned BAUD     = 115_200;
  localparam int unsigned NBYTES   = 5;
  // True 115200 bit period in ns. The DUT divider runs 0.47% fast
  // (8640 ns/bit), so bit-banging at the exact nominal rate also
  // exercises the receiver's baud-mismatch tolerance.
  localparam int unsigned BIT_T    = 8681;

  int errors = 0;

  // 100 MHz clock
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic resetn = 1'b0;

  // ----------------------------------------------------------------
  // UART A: external loopback txd -> rxd
  // ----------------------------------------------------------------
  logic       tx_start = 1'b0;
  logic [7:0] tx_data  = '0;
  logic       tx_busy;
  logic [7:0] rx_data;
  logic       rx_data_valid, rx_frame_error;
  logic       serial_loop;

  uart #(.CLK_FREQ_HZ(CLK_FREQ), .BAUD_RATE(BAUD)) u_uart_a (
    .clk            (clk),
    .resetn         (resetn),
    .tx_start       (tx_start),
    .tx_data        (tx_data),
    .tx_busy        (tx_busy),
    .rx_data        (rx_data),
    .rx_data_valid  (rx_data_valid),
    .rx_frame_error (rx_frame_error),
    .txd            (serial_loop),
    .rxd            (serial_loop)
  );

  // ----------------------------------------------------------------
  // UART B: receiver driven by a bit-banged line (error-path tests)
  // ----------------------------------------------------------------
  logic       b_rxd = 1'b1;
  logic [7:0] b_rx_data;
  logic       b_rx_valid, b_rx_ferr;
  logic       b_txd;

  uart #(.CLK_FREQ_HZ(CLK_FREQ), .BAUD_RATE(BAUD)) u_uart_b (
    .clk            (clk),
    .resetn         (resetn),
    .tx_start       (1'b0),
    .tx_data        (8'h00),
    .tx_busy        (),
    .rx_data        (b_rx_data),
    .rx_data_valid  (b_rx_valid),
    .rx_frame_error (b_rx_ferr),
    .txd            (b_txd),
    .rxd            (b_rxd)
  );

  // ----------------------------------------------------------------
  // Timer under test
  // ----------------------------------------------------------------
  logic        t_en  = 1'b0;
  logic        t_clr = 1'b0;
  logic [31:0] t_cmp = '0;
  logic [31:0] t_cnt;
  logic        t_irq;

  timer u_timer (
    .clk           (clk),
    .resetn        (resetn),
    .enable        (t_en),
    .clear         (t_clr),
    .compare_value (t_cmp),
    .count_value   (t_cnt),
    .interrupt     (t_irq)
  );

  // ----------------------------------------------------------------
  // Loopback scoreboard
  // ----------------------------------------------------------------
  logic [7:0] payload [0:NBYTES-1];
  int rx_idx = 0;

  always @(posedge clk) begin
    if (rx_data_valid) begin
      if (rx_idx < NBYTES && rx_data === payload[rx_idx])
        $display("[%0t ns] loopback OK   : byte %0d = 0x%02h", $time, rx_idx, rx_data);
      else begin
        $display("[%0t ns] loopback ERROR: byte %0d = 0x%02h (expected 0x%02h)",
                 $time, rx_idx, rx_data, (rx_idx < NBYTES) ? payload[rx_idx] : 8'hxx);
        errors++;
      end
      rx_idx++;
    end
    if (rx_frame_error) begin
      $display("[%0t ns] ERROR: unexpected frame error on loopback UART", $time);
      errors++;
    end
  end

  // UART B event counters
  int b_valid_cnt = 0;
  int b_ferr_cnt  = 0;
  logic [7:0] b_last = 8'h00;

  always @(posedge clk) begin
    if (b_rx_valid) begin
      b_valid_cnt++;
      b_last = b_rx_data;
    end
    if (b_rx_ferr) b_ferr_cnt++;
  end

  // ----------------------------------------------------------------
  // Helper tasks
  // ----------------------------------------------------------------
  task automatic send_byte(input logic [7:0] b);
    while (tx_busy) @(negedge clk);
    tx_data  = b;
    tx_start = 1'b1;
    @(negedge clk);
    tx_start = 1'b0;
  endtask

  // Drive one raw frame onto b_rxd; stop_bit = 0 forces a framing error
  task automatic bitbang_frame(input logic [7:0] b, input logic stop_bit);
    b_rxd = 1'b0;                 // start bit
    #(BIT_T);
    for (int i = 0; i < 8; i++) begin
      b_rxd = b[i];               // LSB first
      #(BIT_T);
    end
    b_rxd = stop_bit;             // stop bit
    #(BIT_T);
    b_rxd = 1'b1;                 // back to idle
    #(BIT_T);
  endtask

  task automatic check(input bit cond, input string msg);
    if (cond)
      $display("[%0t ns] OK   : %s", $time, msg);
    else begin
      $display("[%0t ns] ERROR: %s", $time, msg);
      errors++;
    end
  endtask

  // ----------------------------------------------------------------
  // Watchdog
  // ----------------------------------------------------------------
  initial begin
    #10_000_000;  // 10 ms
    $display("ERROR: watchdog timeout");
    $finish;
  end

  // ----------------------------------------------------------------
  // Main sequence
  // ----------------------------------------------------------------
  initial begin
    int cycles;
    logic [31:0] held;

    payload[0] = 8'hA5;
    payload[1] = 8'h3C;
    payload[2] = 8'h00;
    payload[3] = 8'hFF;
    payload[4] = 8'h7E;

    repeat (10) @(negedge clk);
    resetn = 1'b1;
    repeat (5)  @(negedge clk);

    // ---------------- Test 1: UART loopback ----------------
    $display("---- Test 1: UART TX->RX loopback, %0d bytes back-to-back ----", NBYTES);
    for (int i = 0; i < NBYTES; i++) send_byte(payload[i]);
    wait (rx_idx == NBYTES);
    // RX completes at mid-stop; give TX time to finish its stop bit
    #(2 * BIT_T);
    check(!tx_busy, "tx_busy deasserted after final frame");
    check(serial_loop === 1'b1, "txd idles high");

    // ---------------- Test 2: UART error path ----------------
    $display("---- Test 2: framing error detection and recovery ----");
    bitbang_frame(8'h96, 1'b1);   // good frame
    repeat (10) @(negedge clk);
    check(b_valid_cnt == 1 && b_last === 8'h96, "good bit-banged frame received (0x96)");
    check(b_ferr_cnt == 0, "no spurious frame error");

    bitbang_frame(8'h55, 1'b0);   // broken stop bit
    repeat (10) @(negedge clk);
    check(b_ferr_cnt == 1, "broken stop bit flagged as frame error");
    check(b_valid_cnt == 1, "corrupted byte was discarded");

    bitbang_frame(8'hC3, 1'b1);   // receiver must recover
    repeat (10) @(negedge clk);
    check(b_valid_cnt == 2 && b_last === 8'hC3, "receiver recovered after frame error (0xC3)");

    // ---------------- Test 3: Timer ----------------
    $display("---- Test 3: timer compare-match interrupt ----");
    t_cmp = 32'd99;
    t_clr = 1'b1;
    @(negedge clk);
    t_clr = 1'b0;
    t_en  = 1'b1;
    check(t_irq === 1'b0, "interrupt low after clear");

    cycles = 0;
    while (!t_irq) begin
      @(negedge clk);
      cycles++;
    end
    $display("[%0t ns] interrupt after %0d enabled cycles (count_value=%0d)", $time, cycles, t_cnt);
    check(cycles == 100, "match period = compare_value + 1 = 100 cycles");

    repeat (20) @(negedge clk);
    check(t_irq === 1'b1, "interrupt is sticky while not cleared");
    check(t_cnt > 32'd100, "counter free-runs past the match");

    // Enable-hold check
    t_en = 1'b0;
    @(negedge clk);
    held = t_cnt;
    repeat (10) @(negedge clk);
    check(t_cnt === held, "counter holds while enable is low");

    // Acknowledge via clear, then a second period
    t_clr = 1'b1;
    @(negedge clk);
    t_clr = 1'b0;
    @(negedge clk);
    check(t_irq === 1'b0 && t_cnt === 32'd0, "clear acknowledges interrupt and zeroes counter");

    t_en   = 1'b1;
    cycles = 0;
    while (!t_irq) begin
      @(negedge clk);
      cycles++;
    end
    check(cycles == 100, "second period also 100 cycles");

    // Corner case: compare_value = 0 fires on the first enabled cycle
    t_en  = 1'b0;
    t_cmp = 32'd0;
    t_clr = 1'b1;
    @(negedge clk);
    t_clr = 1'b0;
    t_en  = 1'b1;
    @(negedge clk);
    check(t_irq === 1'b1, "compare_value=0 fires on the first enabled cycle");

    // ---------------- Summary ----------------
    if (errors == 0)
      $display("==== ALL TESTS PASSED ====");
    else
      $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end

endmodule
