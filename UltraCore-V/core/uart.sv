// UART Top - RTL
// 8N1 serial port (1 start, 8 data LSB-first, no parity, 1 stop) with a
// shared 16x-oversampling baud-rate generator feeding the TX and RX
// engines. Native register-level interface, no bus wrapper.
//
// Parameterization:
//   CLK_FREQ_HZ - system clock frequency in Hz (default 100 MHz, matches
//                 the 10 ns clock constraint of the UltraCore-V SoC)
//   BAUD_RATE   - target line rate in bits/s (default 115200)
//
// Baud accuracy: the generator divides clk by
//   BAUD_DIV = round(CLK_FREQ_HZ / (16 * BAUD_RATE))
// At 100 MHz / 115200 this yields BAUD_DIV = 54, i.e. an actual rate of
// 115740 baud (+0.47%). Accumulated over the 9.5-bit span from start edge
// to stop-bit sample this shifts the sampling point by under 5% of a bit
// period - far inside the margin of center sampling.
//
// Clocking / CDC:
//   - All control/status signals are synchronous to clk.
//   - rxd is asynchronous; it is double-flop synchronized inside uart_rx.
//
// Usage (native interface, all in the clk domain):
//   TX: when tx_busy is low, present tx_data and assert tx_start for at
//       least one cycle. The byte is latched on acceptance; tx_busy rises
//       one cycle later and falls after the stop bit. Holding tx_start
//       high streams frames back-to-back.
//   RX: rx_data_valid pulses for one clk cycle when a clean frame has
//       landed in rx_data. rx_frame_error pulses instead when the stop
//       bit is low (the corrupted byte is discarded).

module uart #(
  parameter int unsigned CLK_FREQ_HZ = 100_000_000,
  parameter int unsigned BAUD_RATE   = 115_200
)(
  input  logic       clk,
  input  logic       resetn,

  // Native register-level interface - transmit
  input  logic       tx_start,
  input  logic [7:0] tx_data,
  output logic       tx_busy,

  // Native register-level interface - receive
  output logic [7:0] rx_data,
  output logic       rx_data_valid,
  output logic       rx_frame_error,

  // Serial lines (FPGA pads)
  output logic       txd,
  input  logic       rxd
);

  // ----------------------------------------------------------------
  // 16x oversampling tick generator
  //
  // The divider is rounded to the nearest integer to minimize the
  // systematic baud error. Both engines consume the same tick, so TX
  // and RX cannot drift apart.
  // ----------------------------------------------------------------
  localparam int unsigned OVERSAMPLE   = 16;
  localparam int unsigned TICK_RATE_HZ = BAUD_RATE * OVERSAMPLE;
  localparam int unsigned BAUD_DIV     = (CLK_FREQ_HZ + TICK_RATE_HZ / 2) / TICK_RATE_HZ;
  localparam int unsigned BAUD_CNT_W   = (BAUD_DIV <= 1) ? 1 : $clog2(BAUD_DIV);

  // Elaboration-time parameter sanity check (synthesizable; evaluated by
  // Vivado during elaboration, generates no logic)
  generate
    if (CLK_FREQ_HZ < TICK_RATE_HZ) begin : g_baud_param_check
      $error("uart: CLK_FREQ_HZ (%0d) must be >= 16 * BAUD_RATE (%0d) for 16x oversampling",
             CLK_FREQ_HZ, TICK_RATE_HZ);
    end
  endgenerate

  logic [BAUD_CNT_W-1:0] baud_cnt;
  logic                  tick_16x;

  // tick_16x is registered: a clean single-cycle pulse every BAUD_DIV
  // clk cycles, glitch-free across its fanout to both engines
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      baud_cnt <= '0;
      tick_16x <= 1'b0;
    end
    else if (baud_cnt == BAUD_CNT_W'(BAUD_DIV - 1)) begin
      baud_cnt <= '0;
      tick_16x <= 1'b1;
    end
    else begin
      baud_cnt <= baud_cnt + 1'b1;
      tick_16x <= 1'b0;
    end
  end

  // ----------------------------------------------------------------
  // Transmit engine
  // ----------------------------------------------------------------
  uart_tx u_uart_tx (
    .clk      (clk),
    .resetn   (resetn),
    .tick_16x (tick_16x),
    .tx_start (tx_start),
    .tx_data  (tx_data),
    .tx_busy  (tx_busy),
    .tx       (txd)
  );

  // ----------------------------------------------------------------
  // Receive engine
  // ----------------------------------------------------------------
  uart_rx u_uart_rx (
    .clk            (clk),
    .resetn         (resetn),
    .tick_16x       (tick_16x),
    .rx             (rxd),
    .rx_data        (rx_data),
    .rx_data_valid  (rx_data_valid),
    .rx_frame_error (rx_frame_error)
  );

endmodule
