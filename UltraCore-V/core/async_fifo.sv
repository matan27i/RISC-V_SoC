// Asynchronous FIFO - RTL
// Dual-clock FIFO for safe data crossing between two asynchronous clock
// domains (the building block for an AXI clock-domain-crossing bridge).
// Classic Gray-pointer design (Cummings): the write and read pointers
// are Gray-coded so only one bit changes per increment, and each pointer
// is double-flop synchronized into the opposite domain before the
// full/empty comparison. Depth is a power of two (2**ADDR_WIDTH).
//
// Crossing rules honored here:
//   - Only Gray-coded pointers cross domains (one bit at a time).
//   - The RAM is written in wr_clk and read in rd_clk; the data path
//     itself never has a combinational cross-domain path.
//   - full is generated in wr_clk, empty in rd_clk.

module async_fifo #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned ADDR_WIDTH = 4          // depth = 16
)(
  // Write domain
  input  logic                  wr_clk,
  input  logic                  wr_rst_n,
  input  logic                  wr_en,
  input  logic [DATA_WIDTH-1:0] wr_data,
  output logic                  full,

  // Read domain
  input  logic                  rd_clk,
  input  logic                  rd_rst_n,
  input  logic                  rd_en,
  output logic [DATA_WIDTH-1:0] rd_data,
  output logic                  empty
);

  localparam int unsigned DEPTH = 1 << ADDR_WIDTH;

  // Dual-port memory (simple dual-port; no reset -> RAM inference)
  logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  // Binary + Gray pointers (one extra MSB to distinguish full/empty)
  logic [ADDR_WIDTH:0] wr_bin, wr_gray, wr_bin_next, wr_gray_next;
  logic [ADDR_WIDTH:0] rd_bin, rd_gray, rd_bin_next, rd_gray_next;

  // Synchronized opposite-domain Gray pointers
  logic [ADDR_WIDTH:0] wq2_rd_gray;   // read ptr  -> write domain
  logic [ADDR_WIDTH:0] rq2_wr_gray;   // write ptr -> read domain

  function automatic logic [ADDR_WIDTH:0] bin2gray(input logic [ADDR_WIDTH:0] b);
    return b ^ (b >> 1);
  endfunction

  //  Write domain 
  assign wr_bin_next  = wr_bin + (wr_en & ~full);
  assign wr_gray_next = bin2gray(wr_bin_next);

  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_bin  <= '0;
      wr_gray <= '0;
    end
    else begin
      wr_bin  <= wr_bin_next;
      wr_gray <= wr_gray_next;
    end
  end

  always_ff @(posedge wr_clk) begin
    if (wr_en & ~full) mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
  end

  // Full is REGISTERED (breaks the combinational feedback through the
  // pointer increment). Next write Gray equals the synchronized read
  // Gray with the top two bits inverted.
  logic full_next;
  assign full_next = (wr_gray_next == {~wq2_rd_gray[ADDR_WIDTH:ADDR_WIDTH-1],
                                       wq2_rd_gray[ADDR_WIDTH-2:0]});
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) full <= 1'b0;
    else           full <= full_next;
  end

  //  Read domain 
  assign rd_bin_next  = rd_bin + (rd_en & ~empty);
  assign rd_gray_next = bin2gray(rd_bin_next);

  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_bin  <= '0;
      rd_gray <= '0;
    end
    else begin
      rd_bin  <= rd_bin_next;
      rd_gray <= rd_gray_next;
    end
  end

  assign rd_data = mem[rd_bin[ADDR_WIDTH-1:0]];

  // Empty is REGISTERED (breaks the combinational feedback through the
  // pointer increment). Next read Gray equals the synchronized write Gray.
  logic empty_next;
  assign empty_next = (rd_gray_next == rq2_wr_gray);
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) empty <= 1'b1;
    else           empty <= empty_next;
  end

  //  Pointer synchronizers 
  sync_2ff #(.WIDTH(ADDR_WIDTH+1)) u_sync_wr2rd (
    .dst_clk(rd_clk), .dst_rst_n(rd_rst_n), .d_async(wr_gray), .q_sync(rq2_wr_gray));

  sync_2ff #(.WIDTH(ADDR_WIDTH+1)) u_sync_rd2wr (
    .dst_clk(wr_clk), .dst_rst_n(wr_rst_n), .d_async(rd_gray), .q_sync(wq2_rd_gray));

endmodule
