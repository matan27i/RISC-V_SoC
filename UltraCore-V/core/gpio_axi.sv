// GPIO (AXI4-Lite) - RTL
// 8-bit general-purpose I/O with per-pin direction (tri-state) control.
//
// Register map (byte offsets inside the 16-byte aperture):
//   0x00  DATA  R: synchronized pin inputs (actual pad state, all pins)
//               W: output data register (drives pins whose DIR bit = 1)
//   0x04  DIR   RW: per-pin direction, 1 = output enabled, 0 = input (Z)
//   0x08  -     reserved: reads 0, writes ignored (OKAY)
//   0x0C  -     reserved: reads 0, writes ignored (OKAY)
//
// Pad interface: gpio_out / gpio_oe / gpio_in. The actual tri-state
// buffer is instantiated at the SoC top level (the recommended FPGA
// practice - IOBUFs belong at the chip boundary):
//   assign pad      = gpio_oe[i] ? gpio_out[i] : 1'bz;
//   assign gpio_in  = pad;
// Inputs pass through a double-flop synchronizer (pins are asynchronous).
//
// All pins reset to inputs (DIR = 0) so external hardware is never
// driven before software configures the port.

module gpio_axi (
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

  // Pad-side interface (tri-state buffer lives in soc_top)
  output logic [7:0]  gpio_out,
  output logic [7:0]  gpio_oe,
  input  logic [7:0]  gpio_in
);

  localparam logic [1:0] REG_DATA = 2'b00;
  localparam logic [1:0] REG_DIR  = 2'b01;

  localparam logic [1:0] RESP_OKAY = 2'b00;

  
  // Input synchronizer (pins are asynchronous to clk)
  
  (* ASYNC_REG = "TRUE" *) logic [7:0] in_meta, in_sync;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      in_meta <= '0;
      in_sync <= '0;
    end
    else begin
      in_meta <= gpio_in;
      in_sync <= in_meta;
    end
  end

  
  // Output and direction registers
  
  logic [7:0] out_r;
  logic [7:0] dir_r;

  assign gpio_out = out_r;
  assign gpio_oe  = dir_r;

  
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
      out_r         <= '0;
      dir_r         <= '0;       // All pins inputs out of reset
    end
    else begin
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
          if (s_axi_wstrb[0]) begin
            case (wr_sel)
              REG_DATA: out_r <= s_axi_wdata[7:0];
              REG_DIR:  dir_r <= s_axi_wdata[7:0];
              default:  ;                       // Reserved: write ignored
            endcase
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
            REG_DATA: s_axi_rdata <= {24'b0, in_sync};  // Live pad state
            REG_DIR:  s_axi_rdata <= {24'b0, dir_r};
            default:  s_axi_rdata <= 32'b0;             // Reserved
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
