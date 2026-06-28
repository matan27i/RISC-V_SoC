// Clock Gating Cell - RTL
// Portable integrated-clock-gate (ICG): a negedge-transparent enable
// latch ANDed with the clock. Latching the enable on the low phase
// removes the glitch you would get from a bare AND of a combinational
// enable, so the gated clock has clean edges.
//
//   gated_clk = clk & (enable | test_en), with enable sampled while clk=0
//
// FPGA note: on Xilinx, map this to a BUFGCE primitive for a real
// clock-enable buffer (`BUFGCE u(.I(clk), .CE(enable|test_en), .O(gated_clk))`)
// rather than fabric AND-gating; the latch form here is for portable
// simulation and ASIC ICG inference. `test_en` forces the clock on for
// scan/DFT.

module clock_gate (
  input  logic clk,
  input  logic enable,
  input  logic test_en,
  output logic gated_clk
);
  logic enable_latched;

  // Transparent while clk is low; holds the enable while clk is high
  always_latch begin
    if (!clk)
      enable_latched = enable | test_en;
  end

  assign gated_clk = clk & enable_latched;
endmodule
