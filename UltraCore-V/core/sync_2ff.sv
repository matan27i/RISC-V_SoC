// Multi-Flop Synchronizer - RTL
// Brings an asynchronous single- or multi-bit *control* signal safely
// into the destination clock domain. For multi-bit buses, only signals
// that are Gray-coded or known-stable may be synchronized this way;
// arbitrary multi-bit data must cross through an async FIFO instead.
//
// STAGES flops (default 2; use 3 for very high-frequency domains). The
// ASYNC_REG attribute asks the tools to place the chain in adjacent
// flops and exclude the first stage from timing (false path).

module sync_2ff #(
  parameter int unsigned WIDTH  = 1,
  parameter int unsigned STAGES = 2
)(
  input  logic              dst_clk,
  input  logic              dst_rst_n,
  input  logic [WIDTH-1:0]  d_async,
  output logic [WIDTH-1:0]  q_sync
);

  (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] sync_chain [STAGES];

  always_ff @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
      for (int i = 0; i < STAGES; i++) sync_chain[i] <= '0;
    end
    else begin
      sync_chain[0] <= d_async;
      for (int i = 1; i < STAGES; i++) sync_chain[i] <= sync_chain[i-1];
    end
  end

  assign q_sync = sync_chain[STAGES-1];

endmodule
