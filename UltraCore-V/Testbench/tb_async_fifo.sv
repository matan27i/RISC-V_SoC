// Testbench - SIMULATION ONLY
// Dual-clock test of the async FIFO: writer (~143 MHz) and reader
// (~91 MHz) on asynchronous clocks. N incrementing words are pushed and
// popped; the test checks in-order delivery with no loss/duplication.
// Enables are combinational (= ~full / ~empty) and the index advances
// only on an accepted beat, so the testbench models a correct producer/
// consumer and the FIFO flags gate the flow.
`timescale 1ns/1ps
module tb_async_fifo;
  localparam int DW = 32, AW = 4, N = 256;
  int errors = 0;

  logic wr_clk = 0; always #3.5 wr_clk = ~wr_clk;   // ~142.9 MHz
  logic rd_clk = 0; always #5.5 rd_clk = ~rd_clk;   // ~90.9 MHz
  logic wr_rst_n = 0, rd_rst_n = 0;

  logic full, empty;
  logic [DW-1:0] wr_data, rd_data;
  int wr_idx = 0, rd_idx = 0;

  // Combinational producer/consumer; advance only on accepted beats
  logic wr_en, rd_en;
  assign wr_en   = wr_rst_n && !full  && (wr_idx < N);
  assign wr_data = 32'hA000_0000 | wr_idx;
  assign rd_en   = rd_rst_n && !empty;

  async_fifo #(.DATA_WIDTH(DW), .ADDR_WIDTH(AW)) dut (
    .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(wr_en), .wr_data(wr_data), .full(full),
    .rd_clk(rd_clk), .rd_rst_n(rd_rst_n), .rd_en(rd_en), .rd_data(rd_data), .empty(empty));

  always_ff @(posedge wr_clk) if (wr_en && !full) wr_idx <= wr_idx + 1;

  always_ff @(posedge rd_clk) if (rd_en && !empty) begin
    if (rd_data !== (32'hA000_0000 | rd_idx)) begin
      $display("ERROR: pop %0d got %08h exp %08h", rd_idx, rd_data, 32'hA000_0000 | rd_idx);
      errors++;
    end
    rd_idx <= rd_idx + 1;
  end

  initial begin
    #1_000_000;
    $display("ERROR: timeout (wr_idx=%0d rd_idx=%0d)", wr_idx, rd_idx);
    $display("==== TEST FAILED (timeout) ====");
    $finish;
  end

  initial begin
    repeat (4) @(posedge wr_clk); wr_rst_n = 1;
    repeat (4) @(posedge rd_clk); rd_rst_n = 1;
    while (rd_idx < N) @(posedge rd_clk);
    if (errors == 0) $display("==== ALL TESTS PASSED ==== (%0d words crossed two async clocks clean)", N);
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end
endmodule
