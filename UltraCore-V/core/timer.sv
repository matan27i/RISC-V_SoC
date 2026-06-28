// Timer - RTL
// 32-bit free-running up-counter with compare-match interrupt.
// Native register-level interface, no bus wrapper.
//
// Behavior:
//   - count_value increments once per clk cycle while enable is high and
//     holds while enable is low. It wraps naturally at 2^COUNTER_WIDTH - 1.
//   - clear is synchronous and has priority over enable: it zeroes both
//     the counter and the interrupt flag. It is the single mechanism for
//     restarting the timer and for acknowledging the interrupt.
//   - When the counter equals compare_value while enabled, the interrupt
//     flag is set (registered, so it asserts on the following clk edge).
//     The flag is sticky: it stays high until clear or reset, making it
//     usable with both level-sensitive and edge-sensitive interrupt logic.
//   - compare_value is compared combinationally, so run-time changes take
//     effect immediately; if the counter is already past the new value,
//     the next match occurs after wrap-around.
//   - Periodic use: keep enable high and pulse clear on every interrupt;
//     a match then occurs every (compare_value + 1) enabled cycles.

module timer #(
  parameter int unsigned COUNTER_WIDTH = 32
)(
  input  logic                     clk,
  input  logic                     resetn,

  // Native register-level interface
  input  logic                     enable,         // Count while high
  input  logic                     clear,          // Sync clear of counter + interrupt (priority over enable)
  input  logic [COUNTER_WIDTH-1:0] compare_value,  // Match threshold

  output logic [COUNTER_WIDTH-1:0] count_value,    // Current counter value (for SoC register reads)
  output logic                     interrupt       // Sticky compare-match flag
);

  // Combinational match detect
  logic match;
  assign match = (count_value == compare_value);

  // Up-counter
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn)
      count_value <= '0;
    else if (clear)
      count_value <= '0;
    else if (enable)
      count_value <= count_value + 1'b1;
  end

  // Sticky interrupt flag: set on an enabled match, cleared only by
  // clear or reset. Registered so the SoC sees a glitch-free level.
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn)
      interrupt <= 1'b0;
    else if (clear)
      interrupt <= 1'b0;
    else if (enable && match)
      interrupt <= 1'b1;
  end

endmodule
