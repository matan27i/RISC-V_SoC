// Reset Synchronizer - RTL
// Per-clock-domain reset that asserts asynchronously (the moment the
// upstream reset drops) and releases synchronously to the local clock,
// removing recovery/removal metastability when one reset feeds several
// asynchronous domains. Instantiate one of these in every clock domain.

module reset_sync #(
  parameter int unsigned STAGES = 2
)(
  input  logic clk,
  input  logic async_rst_n,   // raw, possibly-async active-low reset in
  output logic sync_rst_n     // local active-low reset, sync release
);

  (* ASYNC_REG = "TRUE" *) logic [STAGES-1:0] rst_chain;

  always_ff @(posedge clk or negedge async_rst_n) begin
    if (!async_rst_n)
      rst_chain <= '0;                       // async assert (all zero)
    else
      rst_chain <= {rst_chain[STAGES-2:0], 1'b1};  // sync release
  end

  assign sync_rst_n = rst_chain[STAGES-1];

endmodule
