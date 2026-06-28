// UART Receiver - RTL
// 8N1 frame, standard 16x oversampling.
//
// Robustness features:
//   (1) Double-flop synchronizer on the asynchronous rx line
//       (metastability protection, ASYNC_REG hint for Vivado placement).
//   (2) Start-bit qualification: a falling edge arms the receiver, and
//       the line is re-checked at the center of the start bit (8 ticks
//       later). A glitch that does not survive half a bit is rejected.
//   (3) Center sampling: each data bit is sampled at its midpoint
//       (16 ticks after the previous sample), giving maximum margin
//       against baud-rate mismatch between transmitter and receiver.
//   (4) Stop-bit check: a low stop bit raises rx_frame_error and the
//       byte is discarded (rx_data is only updated on clean frames).
//   (5) Edge-based start detection: after a framing error or line
//       break, the receiver waits for the line to return high before
//       re-arming, so a stuck-low line produces a single error.

module uart_rx (
  input  logic       clk,
  input  logic       resetn,

  // Baud timing
  input  logic       tick_16x,        // Single-cycle pulse at 16x the baud rate

  // Serial line
  input  logic       rx,              // Serial data input (asynchronous, idles high)

  // Native register-level interface
  output logic [7:0] rx_data,         // Last correctly received byte
  output logic       rx_data_valid,   // Single-cycle pulse: rx_data was updated
  output logic       rx_frame_error   // Single-cycle pulse: stop bit was low, byte discarded
);

  // 16x oversampling tick counts (counters are zero-based)
  localparam logic [3:0] HALF_BIT_TICKS = 4'd7;   // 8 ticks  = center of start bit
  localparam logic [3:0] FULL_BIT_TICKS = 4'd15;  // 16 ticks = one full bit period
  localparam logic [2:0] LAST_DATA_BIT  = 3'd7;   // 8 data bits (8N1)

  // ----------------------------------------------------------------
  // Input synchronizer (2 flops) + previous-value flop for falling-edge
  // detection. ASYNC_REG keeps the synchronizer pair in adjacent slices.
  // ----------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic rx_meta, rx_sync;
  logic rx_sync_q;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      rx_meta   <= 1'b1;   // Reset to the idle-high line state
      rx_sync   <= 1'b1;
      rx_sync_q <= 1'b1;
    end
    else begin
      rx_meta   <= rx;
      rx_sync   <= rx_meta;
      rx_sync_q <= rx_sync;
    end
  end

  // Falling edge on the synchronized line = candidate start bit
  logic rx_fall;
  assign rx_fall = rx_sync_q & ~rx_sync;

  // ----------------------------------------------------------------
  // Receive FSM
  // ----------------------------------------------------------------
  typedef enum logic [1:0] {
    RX_IDLE  = 2'b00,
    RX_START = 2'b01,
    RX_DATA  = 2'b10,
    RX_STOP  = 2'b11
  } rx_state_t;

  rx_state_t  state;
  logic [3:0] tick_cnt;    // Counts oversampling ticks within the current bit
  logic [2:0] bit_idx;     // Index of the data bit being received
  logic [7:0] shift_reg;   // Receive shift register (LSB arrives first)

  // Sampling-point timeline (ticks measured from the detected start edge):
  //   start-bit re-check : tick  8           (HALF_BIT_TICKS)
  //   data bit k center  : tick 24 + 16*k    (FULL_BIT_TICKS after previous)
  //   stop bit center    : tick 152
  // Returning to IDLE at the stop-bit center (instead of its end) leaves
  // half a bit of slack to re-acquire the next start edge, which tolerates
  // baud-rate mismatch on back-to-back frames.
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      state          <= RX_IDLE;
      tick_cnt       <= '0;
      bit_idx        <= '0;
      shift_reg      <= '0;
      rx_data        <= '0;
      rx_data_valid  <= 1'b0;
      rx_frame_error <= 1'b0;
    end
    else begin
      // Status outputs are single-cycle pulses
      rx_data_valid  <= 1'b0;
      rx_frame_error <= 1'b0;

      case (state)
        RX_IDLE: begin
          if (rx_fall) begin
            tick_cnt <= '0;
            state    <= RX_START;
          end
        end

        RX_START: begin
          if (tick_16x) begin
            if (tick_cnt == HALF_BIT_TICKS) begin
              if (!rx_sync) begin
                // Confirmed start bit: restart the tick counter here so
                // every subsequent sample lands at a bit center
                tick_cnt <= '0;
                bit_idx  <= '0;
                state    <= RX_DATA;
              end
              else
                state <= RX_IDLE;   // Line went back high: noise, reject
            end
            else
              tick_cnt <= tick_cnt + 4'd1;
          end
        end

        RX_DATA: begin
          if (tick_16x) begin
            if (tick_cnt == FULL_BIT_TICKS) begin
              tick_cnt  <= '0;
              shift_reg <= {rx_sync, shift_reg[7:1]};   // LSB first: shift right
              if (bit_idx == LAST_DATA_BIT)
                state <= RX_STOP;
              else
                bit_idx <= bit_idx + 3'd1;
            end
            else
              tick_cnt <= tick_cnt + 4'd1;
          end
        end

        RX_STOP: begin
          if (tick_16x) begin
            if (tick_cnt == FULL_BIT_TICKS) begin
              // Center of the stop bit: must be high for a valid frame
              state <= RX_IDLE;
              if (rx_sync) begin
                rx_data       <= shift_reg;
                rx_data_valid <= 1'b1;
              end
              else
                rx_frame_error <= 1'b1;
            end
            else
              tick_cnt <= tick_cnt + 4'd1;
          end
        end

        default: state <= RX_IDLE;
      endcase
    end
  end

endmodule
