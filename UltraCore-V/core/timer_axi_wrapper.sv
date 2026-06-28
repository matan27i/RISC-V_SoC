// Timer AXI4-Lite Wrapper - RTL
// Bridges the AXI4-Lite bus to the native register interface of the
// existing 32-bit timer core (timer.sv). The timer itself is unchanged:
// enable / clear / compare_value in, count_value / interrupt out.
//
// Register map (byte offsets inside the 16-byte aperture):
//   0x00  CTRL     [0] ENABLE       RW   count while 1
//                  [1] CLEAR        W1   one-cycle pulse: zeroes the
//                                        counter and acknowledges the
//                                        interrupt (reads as 0)
//   0x04  COMPARE  [31:0]           RW   match threshold (byte strobes
//                                        honored)
//   0x08  COUNT    [31:0]           RO   live counter value
//   0x0C  STATUS   [0] IRQ_PENDING  RO   sticky compare-match flag
//                                        (clear via CTRL.CLEAR)
//
// Reserved bits read as zero; writes to read-only offsets are ignored
// (OKAY response). COMPARE resets to all-ones so an enabled-but-
// unconfigured timer does not fire for 2^32 cycles.
//
// The native sticky interrupt is exported unmodified on irq for the
// SoC interrupt fabric.

module timer_axi_wrapper (
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

  // Interrupt to the SoC fabric (sticky until CTRL.CLEAR)
  output logic        irq
);

  localparam logic [1:0] REG_CTRL    = 2'b00;
  localparam logic [1:0] REG_COMPARE = 2'b01;
  localparam logic [1:0] REG_COUNT   = 2'b10;
  localparam logic [1:0] REG_STATUS  = 2'b11;

  localparam logic [1:0] RESP_OKAY = 2'b00;

  
  // Native timer core
  
  logic        enable_r;
  logic        clear_pulse;
  logic [31:0] compare_r;
  logic [31:0] count_value;
  logic        interrupt;

  timer u_timer (
    .clk           (clk),
    .resetn        (resetn),
    .enable        (enable_r),
    .clear         (clear_pulse),
    .compare_value (compare_r),
    .count_value   (count_value),
    .interrupt     (interrupt)
  );

  assign irq = interrupt;

  
  // AXI4-Lite write channel (joint AW/W acceptance, registered FSM)
  
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
      enable_r      <= 1'b0;
      clear_pulse   <= 1'b0;
      compare_r     <= 32'hFFFF_FFFF;   // Safe: no immediate match
    end
    else begin
      clear_pulse <= 1'b0;              // W1 action: single-cycle pulse

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
          case (wr_sel)
            REG_CTRL: begin
              if (s_axi_wstrb[0]) begin
                enable_r    <= s_axi_wdata[0];
                clear_pulse <= s_axi_wdata[1];
              end
            end
            REG_COMPARE: begin
              // Honor byte strobes on the 32-bit compare value
              if (s_axi_wstrb[0]) compare_r[7:0]   <= s_axi_wdata[7:0];
              if (s_axi_wstrb[1]) compare_r[15:8]  <= s_axi_wdata[15:8];
              if (s_axi_wstrb[2]) compare_r[23:16] <= s_axi_wdata[23:16];
              if (s_axi_wstrb[3]) compare_r[31:24] <= s_axi_wdata[31:24];
            end
            default: ;                  // COUNT / STATUS read-only
          endcase
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

  
  // AXI4-Lite read channel
  
  typedef enum logic [1:0] {
    RD_IDLE = 2'b00,
    RD_XFER = 2'b01,
    RD_RESP = 2'b10
  } rd_state_t;

  rd_state_t rd_state;

  logic [1:0] rd_sel;
  assign rd_sel = s_axi_araddr[3:2];

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
            REG_CTRL:    s_axi_rdata <= {31'b0, enable_r};   // CLEAR reads 0
            REG_COMPARE: s_axi_rdata <= compare_r;
            REG_COUNT:   s_axi_rdata <= count_value;
            REG_STATUS:  s_axi_rdata <= {31'b0, interrupt};
            default:     s_axi_rdata <= 32'b0;
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
  assign _unused_inputs = &{1'b0, s_axi_awaddr[1:0], s_axi_araddr[1:0]};

endmodule
