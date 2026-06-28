// Testbench - SIMULATION ONLY (do not add to synthesis sources)
// Self-checking test for the SEC-DED ECC accelerator:
//   1. Clean frames        : all 256 payloads decode unchanged, no flags
//   2. Single-bit errors   : all 13 frame positions x 64 payloads are
//                            corrected (payload intact, SINGLE flag set)
//   3. Double-bit errors   : all 78 position pairs x 4 payloads raise
//                            DOUBLE (uncorrectable), never SINGLE
//   4. Triple-bit error    : check-bit triple producing the impossible
//                            syndrome 13 is flagged uncorrectable
//   5. IRQ                 : masked while IRQ_EN=0, level-high when
//                            enabled, drops on CLEAR_STATUS
//   6. Backpressure        : frames queued in the FIFO are delivered in
//                            order with per-frame status, none dropped
//   7. AXI protocol checks : SLVERR on the unmapped offset, read-only
//                            registers ignore writes, CTRL readback
// Plus continuous FIFO-protocol monitors (no pop while empty, strobe is
// a single cycle).

`timescale 1ns/1ps

module tb_ecc_accel;

  int errors = 0;

  // 100 MHz clock
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic resetn = 1'b0;

  // ----------------------------------------------------------------
  // DUT hookup
  // ----------------------------------------------------------------
  logic        rx_empty;
  logic        rx_read_en;
  logic [12:0] rx_data;

  logic [3:0]  awaddr  = '0;
  logic        awvalid = 1'b0;
  logic        awready;
  logic [31:0] wdata   = '0;
  logic [3:0]  wstrb   = '0;
  logic        wvalid  = 1'b0;
  logic        wready;
  logic [1:0]  bresp;
  logic        bvalid;
  logic        bready  = 1'b0;
  logic [3:0]  araddr  = '0;
  logic        arvalid = 1'b0;
  logic        arready;
  logic [31:0] rdata;
  logic [1:0]  rresp;
  logic        rvalid;
  logic        rready  = 1'b0;
  logic        irq;

  ecc_accel dut (
    .clk           (clk),
    .resetn        (resetn),
    .rx_empty      (rx_empty),
    .rx_read_en    (rx_read_en),
    .rx_data       (rx_data),
    .s_axi_awaddr  (awaddr),
    .s_axi_awvalid (awvalid),
    .s_axi_awready (awready),
    .s_axi_wdata   (wdata),
    .s_axi_wstrb   (wstrb),
    .s_axi_wvalid  (wvalid),
    .s_axi_wready  (wready),
    .s_axi_bresp   (bresp),
    .s_axi_bvalid  (bvalid),
    .s_axi_bready  (bready),
    .s_axi_araddr  (araddr),
    .s_axi_arvalid (arvalid),
    .s_axi_arready (arready),
    .s_axi_rdata   (rdata),
    .s_axi_rresp   (rresp),
    .s_axi_rvalid  (rvalid),
    .s_axi_rready  (rready),
    .irq           (irq)
  );

  // ----------------------------------------------------------------
  // FWFT FIFO model: head word visible while not empty; rx_read_en pops
  // ----------------------------------------------------------------
  localparam int unsigned FIFO_DEPTH = 64;

  logic [12:0] fifo_mem [0:FIFO_DEPTH-1];
  int unsigned wr_ptr = 0;
  int unsigned rd_ptr = 0;

  assign rx_empty = (wr_ptr == rd_ptr);
  assign rx_data  = fifo_mem[rd_ptr % FIFO_DEPTH];

  always @(posedge clk) begin
    if (rx_read_en && !rx_empty)
      rd_ptr <= rd_ptr + 1;
  end

  // FIFO protocol monitors
  logic rx_read_en_q = 1'b0;
  always @(posedge clk) begin
    rx_read_en_q <= rx_read_en;
    if (rx_read_en && rx_empty) begin
      $display("[%0t] ERROR: rx_read_en asserted while FIFO empty", $time);
      errors++;
    end
    if (rx_read_en && rx_read_en_q) begin
      $display("[%0t] ERROR: rx_read_en held for more than one cycle", $time);
      errors++;
    end
  end

  // ----------------------------------------------------------------
  // Reference encoder (must match the contract in ecc_accel.sv header)
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

  task automatic push_frame(input logic [12:0] frame);
    @(negedge clk);
    fifo_mem[wr_ptr % FIFO_DEPTH] = frame;
    wr_ptr = wr_ptr + 1;
  endtask

  // ----------------------------------------------------------------
  // AXI4-Lite master BFM
  // ----------------------------------------------------------------
  task automatic axi_write(input logic [3:0] a, input logic [31:0] d,
                           output logic [1:0] resp);
    @(negedge clk);
    awaddr = a;  awvalid = 1'b1;
    wdata  = d;  wstrb   = 4'h1;  wvalid = 1'b1;
    bready = 1'b1;
    @(negedge clk);
    while (!(awready && wready)) @(negedge clk);
    @(negedge clk);                 // past the acceptance edge
    awvalid = 1'b0; wvalid = 1'b0;
    while (!bvalid) @(negedge clk);
    resp = bresp;
    @(negedge clk);                 // past the B handshake edge
    bready = 1'b0;
  endtask

  task automatic axi_read(input logic [3:0] a, output logic [31:0] d,
                          output logic [1:0] resp);
    @(negedge clk);
    araddr = a; arvalid = 1'b1; rready = 1'b1;
    @(negedge clk);
    while (!arready) @(negedge clk);
    @(negedge clk);                 // past the acceptance edge
    arvalid = 1'b0;
    while (!rvalid) @(negedge clk);
    d    = rdata;
    resp = rresp;
    @(negedge clk);                 // past the R handshake edge
    rready = 1'b0;
  endtask

  // ----------------------------------------------------------------
  // Test helpers
  // ----------------------------------------------------------------
  logic tb_irq_en = 1'b0;   // shadow of CTRL.IRQ_EN for clear writes

  task automatic check(input bit cond, input string msg);
    if (!cond) begin
      $display("[%0t] ERROR: %s", $time, msg);
      errors++;
    end
  endtask

  // Poll STATUS until DATA_READY, verify payload/flags, then acknowledge
  task automatic receive_check(input logic [7:0] exp_data,
                               input logic exp_single,
                               input logic exp_double,
                               input string msg);
    logic [31:0] st, dat;
    logic [1:0]  resp;
    int guard = 0;
    bit timed_out = 1'b0;
    do begin
      axi_read(4'h4, st, resp);
      guard++;
      if (guard > 200) begin
        $display("[%0t] ERROR: timeout waiting for DATA_READY (%s)", $time, msg);
        errors++;
        timed_out = 1'b1;
      end
    end while (!st[0] && !timed_out);

    if (!timed_out) begin
      axi_read(4'h0, dat, resp);

      // Payload is only meaningful when the frame was correctable
      if (!exp_double && dat[7:0] !== exp_data) begin
        $display("[%0t] ERROR: %s: data=0x%02h expected 0x%02h", $time, msg, dat[7:0], exp_data);
        errors++;
      end
      if (st[1] !== exp_single || st[2] !== exp_double) begin
        $display("[%0t] ERROR: %s: single=%b (exp %b) double=%b (exp %b)",
                 $time, msg, st[1], exp_single, st[2], exp_double);
        errors++;
      end

      // Acknowledge: CLEAR_STATUS=1, preserve IRQ_EN
      axi_write(4'h8, {30'b0, 1'b1, tb_irq_en}, resp);
    end
  endtask

  // ----------------------------------------------------------------
  // Watchdog
  // ----------------------------------------------------------------
  initial begin
    #20_000_000;   // 20 ms
    $display("ERROR: watchdog timeout");
    $finish;
  end

  // ----------------------------------------------------------------
  // Main sequence
  // ----------------------------------------------------------------
  initial begin
    logic [31:0] rd;
    logic [1:0]  resp;
    logic [12:0] frame;
    logic [7:0]  dd;
    string       msg;

    repeat (10) @(negedge clk);
    resetn = 1'b1;
    repeat (5)  @(negedge clk);

    // ------------ Test 1: all 256 clean payloads ------------
    $display("---- Test 1: clean frames, all 256 payloads ----");
    for (int d = 0; d < 256; d++) begin
      push_frame(ecc_encode(d[7:0]));
      $sformat(msg, "clean d=0x%02h", d[7:0]);
      receive_check(d[7:0], 1'b0, 1'b0, msg);
    end
    $display("      done");

    // ------------ Test 2: every single-bit error position ------------
    $display("---- Test 2: single-bit errors, 13 positions x 64 payloads ----");
    for (int d = 0; d < 256; d += 4) begin
      for (int pos = 0; pos < 13; pos++) begin
        push_frame(ecc_encode(d[7:0]) ^ (13'h1 << pos));
        $sformat(msg, "single d=0x%02h pos=%0d", d[7:0], pos);
        receive_check(d[7:0], 1'b1, 1'b0, msg);
      end
    end
    $display("      done");

    // ------------ Test 3: every double-bit error pair ------------
    $display("---- Test 3: double-bit errors, 78 pairs x 4 payloads ----");
    for (int k = 0; k < 4; k++) begin
      case (k)
        0: dd = 8'h00;
        1: dd = 8'hA5;
        2: dd = 8'hFF;
        default: dd = 8'h3C;
      endcase
      for (int i = 0; i < 13; i++) begin
        for (int j = i + 1; j < 13; j++) begin
          push_frame(ecc_encode(dd) ^ (13'h1 << i) ^ (13'h1 << j));
          $sformat(msg, "double d=0x%02h pos=%0d,%0d", dd, i, j);
          receive_check(8'hxx, 1'b0, 1'b1, msg);
        end
      end
    end
    $display("      done");

    // ------------ Test 4: triple error -> impossible syndrome ------------
    // Flipping C0 (pos 1), C2 (pos 4) and C3 (pos 8) gives S = 1^4^8 = 13
    // with odd overall parity: must be reported uncorrectable, payload
    // must NOT be modified by a bogus flip.
    $display("---- Test 4: triple check-bit error (syndrome 13) ----");
    frame = ecc_encode(8'h5A) ^ (13'h1 << 8) ^ (13'h1 << 10) ^ (13'h1 << 11);
    push_frame(frame);
    receive_check(8'hxx, 1'b0, 1'b1, "triple error syndrome=13");

    // ------------ Test 5: interrupt behavior ------------
    $display("---- Test 5: IRQ masking / assertion / clear ----");
    check(irq === 1'b0, "irq low while idle");

    push_frame(ecc_encode(8'h42));
    // Wait for DATA_READY with IRQ disabled
    do axi_read(4'h4, rd, resp); while (!rd[0]);
    repeat (3) @(negedge clk);
    check(irq === 1'b0, "irq stays low while IRQ_EN=0");

    // Enable interrupts (no clear): irq must rise on the pending frame
    tb_irq_en = 1'b1;
    axi_write(4'h8, 32'h0000_0001, resp);
    repeat (3) @(negedge clk);
    check(irq === 1'b1, "irq high once IRQ_EN set with DATA_READY pending");

    // CTRL readback: IRQ_EN=1, CLEAR_STATUS reads as zero
    axi_read(4'h8, rd, resp);
    check(rd === 32'h0000_0001, "CTRL readback: IRQ_EN=1, CLEAR bit reads 0");

    // Read data, then acknowledge -> irq must drop
    axi_read(4'h0, rd, resp);
    check(rd[7:0] === 8'h42, "payload of IRQ test frame");
    axi_write(4'h8, 32'h0000_0003, resp);   // CLEAR_STATUS=1, keep IRQ_EN=1
    repeat (3) @(negedge clk);
    check(irq === 1'b0, "irq cleared by CTRL.CLEAR_STATUS");

    // ------------ Test 6: backpressure and ordering ------------
    $display("---- Test 6: FIFO backpressure, in-order delivery ----");
    push_frame(ecc_encode(8'h10));                    // clean
    push_frame(ecc_encode(8'h11) ^ 13'h0008);         // single error (D3)
    push_frame(ecc_encode(8'h12) ^ 13'h0003);         // double error (D0+D1)
    push_frame(ecc_encode(8'h13));                    // clean
    push_frame(ecc_encode(8'h14) ^ 13'h1000);         // single error (P bit)
    receive_check(8'h10, 1'b0, 1'b0, "queued frame 0 (clean)");
    receive_check(8'h11, 1'b1, 1'b0, "queued frame 1 (single)");
    receive_check(8'hxx, 1'b0, 1'b1, "queued frame 2 (double)");
    receive_check(8'h13, 1'b0, 1'b0, "queued frame 3 (clean)");
    receive_check(8'h14, 1'b1, 1'b0, "queued frame 4 (P-bit single)");
    check(rx_empty === 1'b1, "FIFO drained after ordered readout");

    // ------------ Test 7: AXI error responses / read-only regs ------------
    $display("---- Test 7: AXI responses ----");
    axi_read(4'hC, rd, resp);
    check(resp === 2'b10 && rd === 32'h0, "read of unmapped 0x0C -> SLVERR, zero data");
    axi_write(4'hC, 32'hDEAD_BEEF, resp);
    check(resp === 2'b10, "write to unmapped 0x0C -> SLVERR");

    axi_read(4'h0, rd, resp);                  // remember current DATA value
    axi_write(4'h0, 32'h0000_00FF, resp);
    check(resp === 2'b00, "write to read-only DATA_REG -> OKAY (ignored)");
    axi_read(4'h0, rd, resp);
    check(rd[7:0] === 8'h14, "DATA_REG unchanged by the ignored write");

    // ------------ Summary ------------
    if (errors == 0)
      $display("==== ALL TESTS PASSED ====");
    else
      $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end

endmodule
