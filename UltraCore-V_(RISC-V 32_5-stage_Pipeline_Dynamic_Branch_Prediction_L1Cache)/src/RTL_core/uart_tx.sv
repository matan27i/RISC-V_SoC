// UART Transmitter - RTL
// 8N1 frame: 1 start bit (low), 8 data bits (LSB first), 1 stop bit (high).
// Bit timing is derived from tick_16x: one bit period = 16 ticks. TX and RX
// therefore share a single clock divider and stay frequency-locked.

module uart_tx (
  input  logic       clk,
  input  logic       resetn,

  // Baud timing
  input  logic       tick_16x,   // Single-cycle pulse at 16x the baud rate

  // Native register-level interface
  input  logic       tx_start,   // Request transmission; sampled only while idle
  input  logic [7:0] tx_data,    // Byte to send; latched when tx_start is accepted
  output logic       tx_busy,    // High while a frame is on the wire

  // Serial line
  output logic       tx          // Serial data output (idles high)
);

  // 16x oversampling: one bit period = 16 ticks (tick counter range 0..15)
  localparam logic [3:0] FULL_BIT_TICKS = 4'd15;
  localparam logic [2:0] LAST_DATA_BIT  = 3'd7;   // 8 data bits (8N1)

  typedef enum logic [1:0] {
    TX_IDLE  = 2'b00,
    TX_START = 2'b01,
    TX_DATA  = 2'b10,
    TX_STOP  = 2'b11
  } tx_state_t;

  tx_state_t  state;
  logic [3:0] tick_cnt;    // Counts oversampling ticks within the current bit
  logic [2:0] bit_idx;     // Index of the data bit currently on the wire
  logic [7:0] shift_reg;   // Transmit shift register, shifts right (LSB first)

  // ----------------------------------------------------------------
  // FSM + datapath in one sequential process. The serial output is
  // registered so the line is glitch-free. Bit boundaries advance only
  // on tick_16x, so every bit lasts exactly 16 tick periods. The start
  // bit can be up to one tick period (1/16 bit) short because tx_start
  // arrives asynchronously to the tick phase; this is harmless since
  // the receiver re-synchronizes on the start-bit falling edge.
  // ----------------------------------------------------------------
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      state     <= TX_IDLE;
      tick_cnt  <= '0;
      bit_idx   <= '0;
      shift_reg <= '0;
      tx        <= 1'b1;            // Line idles high
    end
    else begin
      case (state)
        TX_IDLE: begin
          tx <= 1'b1;
          if (tx_start) begin
            // Accept the request: capture the data byte so the parent
            // is free to change tx_data immediately afterwards
            shift_reg <= tx_data;
            tick_cnt  <= '0;
            state     <= TX_START;
          end
        end

        TX_START: begin
          tx <= 1'b0;               // Start bit
          if (tick_16x) begin
            if (tick_cnt == FULL_BIT_TICKS) begin
              tick_cnt <= '0;
              bit_idx  <= '0;
              state    <= TX_DATA;
            end
            else
              tick_cnt <= tick_cnt + 4'd1;
          end
        end

        TX_DATA: begin
          tx <= shift_reg[0];       // LSB first
          if (tick_16x) begin
            if (tick_cnt == FULL_BIT_TICKS) begin
              tick_cnt  <= '0;
              shift_reg <= shift_reg >> 1;
              if (bit_idx == LAST_DATA_BIT)
                state <= TX_STOP;
              else
                bit_idx <= bit_idx + 3'd1;
            end
            else
              tick_cnt <= tick_cnt + 4'd1;
          end
        end

        TX_STOP: begin
          tx <= 1'b1;               // Stop bit
          if (tick_16x) begin
            if (tick_cnt == FULL_BIT_TICKS)
              state <= TX_IDLE;
            else
              tick_cnt <= tick_cnt + 4'd1;
          end
        end

        default: state <= TX_IDLE;
      endcase
    end
  end

  // Busy from one cycle after tx_start is accepted until the stop bit
  // completes. Holding tx_start high streams frames back-to-back
  // (exactly one stop bit between frames).
  assign tx_busy = (state != TX_IDLE);

endmodule
