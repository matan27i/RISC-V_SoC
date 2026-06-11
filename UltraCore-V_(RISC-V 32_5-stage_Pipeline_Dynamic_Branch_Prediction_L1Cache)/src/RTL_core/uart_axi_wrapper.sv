// UART AXI4-Lite Wrapper - RTL
// Bridges the AXI4-Lite bus to the native register interface of the
// existing UART core (uart.sv, 8N1, fixed-baud by parameter). The UART
// core itself is unchanged.
//
// Register map (byte offsets inside the 16-byte aperture):
//   0x00  TX_DATA  [7:0]  WO  write a byte to transmit. Accepted only
//                             when STATUS.TX_BUSY = 0, silently dropped
//                             otherwise (poll STATUS first). Reads 0.
//   0x04  RX_DATA  [7:0]  RO  last received byte. Reading this register
//                             clears RX_VALID / FRAME_ERR / OVERRUN.
//   0x08  STATUS          RO  [0] TX_BUSY    transmitter busy (covers
//                                            the 1-cycle start window)
//                             [1] RX_VALID   unread byte in RX_DATA
//                             [2] FRAME_ERR  sticky: stop bit was low
//                             [3] OVERRUN    sticky: byte arrived while
//                                            RX_VALID was still set
//   0x0C  -               --  reserved: reads 0, writes ignored (OKAY)
//
// RX stream tap: the raw receive stream (byte + valid + frame-error
// pulses) is exported so the ECC frame packer can listen to the same
// stream. Software reads via RX_DATA and the hardware ECC path are
// independent consumers; a deployment uses one or the other.

module uart_axi_wrapper #(
  parameter int unsigned CLK_FREQ_HZ = 100_000_000,
  parameter int unsigned BAUD_RATE   = 115_200
)(
  input  logic        clk,
  input  logic        resetn,

  // AXI4-Lite target (AxPROT omitted: terminated at the interconnect)
  input  logic [3:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [3:0]  s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  // RX stream tap (for the ECC frame packer)
  output logic [7:0]  rx_stream_data,
  output logic        rx_stream_valid,
  output logic        rx_stream_ferr,

  // Serial lines (FPGA pads)
  output logic        txd,
  input  logic        rxd
);

  localparam logic [1:0] REG_TX     = 2'b00;
  localparam logic [1:0] REG_RX     = 2'b01;
  localparam logic [1:0] REG_STATUS = 2'b10;

  localparam logic [1:0] RESP_OKAY = 2'b00;

  // ----------------------------------------------------------------
  // Native UART core
  // ----------------------------------------------------------------
  logic       tx_start_r;
  logic [7:0] tx_byte_r;
  logic       tx_busy;
  logic [7:0] rx_byte;
  logic       rx_byte_valid;
  logic       rx_frame_error;

  uart #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .BAUD_RATE   (BAUD_RATE)
  ) u_uart (
    .clk            (clk),
    .resetn         (resetn),
    .tx_start       (tx_start_r),
    .tx_data        (tx_byte_r),
    .tx_busy        (tx_busy),
    .rx_data        (rx_byte),
    .rx_data_valid  (rx_byte_valid),
    .rx_frame_error (rx_frame_error),
    .txd            (txd),
    .rxd            (rxd)
  );

  // Raw stream tap for the ECC front-end
  assign rx_stream_data  = rx_byte;
  assign rx_stream_valid = rx_byte_valid;
  assign rx_stream_ferr  = rx_frame_error;

  // tx_busy rises one cycle after tx_start is accepted; OR-ing the
  // pending start pulse closes that status window so software polling
  // STATUS can never double-load the transmitter.
  logic tx_busy_eff;
  assign tx_busy_eff = tx_busy | tx_start_r;

  // ----------------------------------------------------------------
  // RX holding register + sticky status flags
  // ----------------------------------------------------------------
  logic [7:0] rx_hold;
  logic       rx_valid;
  logic       ferr_sticky;
  logic       overrun_sticky;

  // Asserted by the read FSM during the cycle an RX_DATA read is served
  logic rx_read_strobe;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      rx_hold        <= '0;
      rx_valid       <= 1'b0;
      ferr_sticky    <= 1'b0;
      overrun_sticky <= 1'b0;
    end
    else begin
      // Reading RX_DATA acknowledges everything...
      if (rx_read_strobe) begin
        rx_valid       <= 1'b0;
        ferr_sticky    <= 1'b0;
        overrun_sticky <= 1'b0;
      end

      // ...but a byte landing in the same cycle wins (it must not be
      // lost; it simply becomes the next unread byte).
      if (rx_byte_valid) begin
        rx_hold  <= rx_byte;
        rx_valid <= 1'b1;
        if (rx_valid && !rx_read_strobe)
          overrun_sticky <= 1'b1;
      end
      if (rx_frame_error)
        ferr_sticky <= 1'b1;
    end
  end

  // ----------------------------------------------------------------
  // AXI4-Lite write channel (joint AW/W acceptance, registered FSM)
  // ----------------------------------------------------------------
  typedef enum logic [1:0] {
    WR_IDLE = 2'b00,
    WR_XFER = 2'b01,
    WR_RESP = 2'b10
  } wr_state_t;

  wr_state_t wr_state;

  logic [1:0] wr_sel;
  assign wr_sel = s_axi_awaddr[3:2];

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      wr_state      <= WR_IDLE;
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      s_axi_bvalid  <= 1'b0;
      s_axi_bresp   <= RESP_OKAY;
      tx_start_r    <= 1'b0;
      tx_byte_r     <= '0;
    end
    else begin
      tx_start_r <= 1'b0;               // Single-cycle start pulse

      case (wr_state)
        WR_IDLE: begin
          if (s_axi_awvalid && s_axi_wvalid) begin
            s_axi_awready <= 1'b1;
            s_axi_wready  <= 1'b1;
            wr_state      <= WR_XFER;
          end
        end

        WR_XFER: begin
          s_axi_awready <= 1'b0;
          s_axi_wready  <= 1'b0;
          // TX write: accept only when the transmitter is free
          if (wr_sel == REG_TX && s_axi_wstrb[0] && !tx_busy_eff) begin
            tx_byte_r  <= s_axi_wdata[7:0];
            tx_start_r <= 1'b1;
          end
          s_axi_bresp  <= RESP_OKAY;
          s_axi_bvalid <= 1'b1;
          wr_state     <= WR_RESP;
        end

        WR_RESP: begin
          if (s_axi_bready) begin
            s_axi_bvalid <= 1'b0;
            wr_state     <= WR_IDLE;
          end
        end

        default: wr_state <= WR_IDLE;
      endcase
    end
  end

  // ----------------------------------------------------------------
  // AXI4-Lite read channel
  // ----------------------------------------------------------------
  typedef enum logic [1:0] {
    RD_IDLE = 2'b00,
    RD_XFER = 2'b01,
    RD_RESP = 2'b10
  } rd_state_t;

  rd_state_t rd_state;

  logic [1:0] rd_sel;
  assign rd_sel = s_axi_araddr[3:2];

  // RX acknowledge happens in the cycle the read data is sampled
  assign rx_read_strobe = (rd_state == RD_XFER) && (rd_sel == REG_RX);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      rd_state      <= RD_IDLE;
      s_axi_arready <= 1'b0;
      s_axi_rvalid  <= 1'b0;
      s_axi_rresp   <= RESP_OKAY;
      s_axi_rdata   <= '0;
    end
    else begin
      case (rd_state)
        RD_IDLE: begin
          if (s_axi_arvalid) begin
            s_axi_arready <= 1'b1;
            rd_state      <= RD_XFER;
          end
        end

        RD_XFER: begin
          s_axi_arready <= 1'b0;
          case (rd_sel)
            REG_RX:     s_axi_rdata <= {24'b0, rx_hold};
            REG_STATUS: s_axi_rdata <= {28'b0, overrun_sticky, ferr_sticky,
                                        rx_valid, tx_busy_eff};
            default:    s_axi_rdata <= 32'b0;   // TX_DATA / reserved read 0
          endcase
          s_axi_rresp  <= RESP_OKAY;
          s_axi_rvalid <= 1'b1;
          rd_state     <= RD_RESP;
        end

        RD_RESP: begin
          if (s_axi_rready) begin
            s_axi_rvalid <= 1'b0;
            rd_state     <= RD_IDLE;
          end
        end

        default: rd_state <= RD_IDLE;
      endcase
    end
  end

  // Sink for intentionally unused inputs
  wire _unused_inputs;
  assign _unused_inputs = &{1'b0, s_axi_wstrb[3:1],
                            s_axi_wdata[31:8], s_axi_awaddr[1:0], s_axi_araddr[1:0]};

endmodule
