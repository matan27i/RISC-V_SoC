// AXI4-Lite Interconnect (1 Master -> 5 Slaves) - RTL
// Single-master address-decoded interconnect for the UltraCore-V SoC.
// With exactly one master a full crossbar degenerates into a router:
// no arbitration is needed, only decode, routing, and response return.
//
// Memory map (strict, per SoC specification):
//   S0  Boot ROM  0x0000_0000 - 0x0000_1FFF   (8 KB window, data reads)
//   S1  UART      0x4000_0000 - 0x4000_000F
//   S2  Timer     0x4000_0010 - 0x4000_001F
//   S3  ECC accel 0x4000_0020 - 0x4000_002F
//   S4  GPIO      0x4000_0030 - 0x4000_003F
//   --  elsewhere: the interconnect itself responds with DECERR
//       (reads return zero), so a stray pointer can never hang the bus.
//
// Routing scheme:
//   - AW, W and AR valids are routed combinationally by live address
//     decode; the master holds address and payload stable until ready,
//     so the route cannot change mid-handshake.
//   - The response selects (bsel_q / rsel_q) are latched at the AW / AR
//     handshake and route B / R back until their handshakes complete.
//     b_active / r_active guard against any overlap.
//
// Protocol requirement (holds for every slave in this SoC, and for the
// axi_master_bridge): a slave accepts AW and W with a JOINT ready pair
// (both handshakes complete in the same cycle), and the master asserts
// AWVALID and WVALID together. This lets W be routed by the live AW
// decode without a W-routing shadow register.

module axi_crossbar (
  input  logic        clk,
  input  logic        resetn,

  // ---------------- Slave port (from axi_master_bridge) ----------------
  input  logic [31:0] s_axi_awaddr,
  input  logic [2:0]  s_axi_awprot,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [31:0] s_axi_araddr,
  input  logic [2:0]  s_axi_arprot,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  // ---------------- Master port M0: Boot ROM ----------------
  output logic [31:0] m_rom_awaddr,
  output logic        m_rom_awvalid,
  input  logic        m_rom_awready,
  output logic [31:0] m_rom_wdata,
  output logic [3:0]  m_rom_wstrb,
  output logic        m_rom_wvalid,
  input  logic        m_rom_wready,
  input  logic [1:0]  m_rom_bresp,
  input  logic        m_rom_bvalid,
  output logic        m_rom_bready,
  output logic [31:0] m_rom_araddr,
  output logic        m_rom_arvalid,
  input  logic        m_rom_arready,
  input  logic [31:0] m_rom_rdata,
  input  logic [1:0]  m_rom_rresp,
  input  logic        m_rom_rvalid,
  output logic        m_rom_rready,

  // ---------------- Master port M1: UART ----------------
  output logic [31:0] m_uart_awaddr,
  output logic        m_uart_awvalid,
  input  logic        m_uart_awready,
  output logic [31:0] m_uart_wdata,
  output logic [3:0]  m_uart_wstrb,
  output logic        m_uart_wvalid,
  input  logic        m_uart_wready,
  input  logic [1:0]  m_uart_bresp,
  input  logic        m_uart_bvalid,
  output logic        m_uart_bready,
  output logic [31:0] m_uart_araddr,
  output logic        m_uart_arvalid,
  input  logic        m_uart_arready,
  input  logic [31:0] m_uart_rdata,
  input  logic [1:0]  m_uart_rresp,
  input  logic        m_uart_rvalid,
  output logic        m_uart_rready,

  // ---------------- Master port M2: Timer ----------------
  output logic [31:0] m_timer_awaddr,
  output logic        m_timer_awvalid,
  input  logic        m_timer_awready,
  output logic [31:0] m_timer_wdata,
  output logic [3:0]  m_timer_wstrb,
  output logic        m_timer_wvalid,
  input  logic        m_timer_wready,
  input  logic [1:0]  m_timer_bresp,
  input  logic        m_timer_bvalid,
  output logic        m_timer_bready,
  output logic [31:0] m_timer_araddr,
  output logic        m_timer_arvalid,
  input  logic        m_timer_arready,
  input  logic [31:0] m_timer_rdata,
  input  logic [1:0]  m_timer_rresp,
  input  logic        m_timer_rvalid,
  output logic        m_timer_rready,

  // ---------------- Master port M3: ECC accelerator ----------------
  output logic [31:0] m_ecc_awaddr,
  output logic        m_ecc_awvalid,
  input  logic        m_ecc_awready,
  output logic [31:0] m_ecc_wdata,
  output logic [3:0]  m_ecc_wstrb,
  output logic        m_ecc_wvalid,
  input  logic        m_ecc_wready,
  input  logic [1:0]  m_ecc_bresp,
  input  logic        m_ecc_bvalid,
  output logic        m_ecc_bready,
  output logic [31:0] m_ecc_araddr,
  output logic        m_ecc_arvalid,
  input  logic        m_ecc_arready,
  input  logic [31:0] m_ecc_rdata,
  input  logic [1:0]  m_ecc_rresp,
  input  logic        m_ecc_rvalid,
  output logic        m_ecc_rready,

  // ---------------- Master port M4: GPIO ----------------
  output logic [31:0] m_gpio_awaddr,
  output logic        m_gpio_awvalid,
  input  logic        m_gpio_awready,
  output logic [31:0] m_gpio_wdata,
  output logic [3:0]  m_gpio_wstrb,
  output logic        m_gpio_wvalid,
  input  logic        m_gpio_wready,
  input  logic [1:0]  m_gpio_bresp,
  input  logic        m_gpio_bvalid,
  output logic        m_gpio_bready,
  output logic [31:0] m_gpio_araddr,
  output logic        m_gpio_arvalid,
  input  logic        m_gpio_arready,
  input  logic [31:0] m_gpio_rdata,
  input  logic [1:0]  m_gpio_rresp,
  input  logic        m_gpio_rvalid,
  output logic        m_gpio_rready
);

  localparam logic [1:0] RESP_DECERR = 2'b11;

  // Target select encoding
  localparam logic [2:0] SEL_ROM   = 3'd0;
  localparam logic [2:0] SEL_UART  = 3'd1;
  localparam logic [2:0] SEL_TIMER = 3'd2;
  localparam logic [2:0] SEL_ECC   = 3'd3;
  localparam logic [2:0] SEL_GPIO  = 3'd4;
  localparam logic [2:0] SEL_NONE  = 3'd7;

  // ----------------------------------------------------------------
  // Address decode (strict memory map)
  // ----------------------------------------------------------------
  function automatic logic [2:0] decode (input logic [31:0] addr);
    if (addr < 32'h0000_2000)              return SEL_ROM;
    else if (addr[31:4] == 28'h4000_000)   return SEL_UART;
    else if (addr[31:4] == 28'h4000_001)   return SEL_TIMER;
    else if (addr[31:4] == 28'h4000_002)   return SEL_ECC;
    else if (addr[31:4] == 28'h4000_003)   return SEL_GPIO;
    else                                   return SEL_NONE;
  endfunction

  logic [2:0] aw_dec, ar_dec;
  assign aw_dec = decode(s_axi_awaddr);
  assign ar_dec = decode(s_axi_araddr);

  // ----------------------------------------------------------------
  // Transaction state: latched response routes
  // ----------------------------------------------------------------
  logic       b_active, r_active;
  logic [2:0] bsel_q, rsel_q;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      b_active <= 1'b0;
      bsel_q   <= SEL_NONE;
      r_active <= 1'b0;
      rsel_q   <= SEL_NONE;
    end
    else begin
      // Write: route latched at the AW handshake, released at B
      if (!b_active && s_axi_awvalid && s_axi_awready) begin
        b_active <= 1'b1;
        bsel_q   <= aw_dec;
      end
      else if (b_active && s_axi_bvalid && s_axi_bready)
        b_active <= 1'b0;

      // Read: route latched at the AR handshake, released at R
      if (!r_active && s_axi_arvalid && s_axi_arready) begin
        r_active <= 1'b1;
        rsel_q   <= ar_dec;
      end
      else if (r_active && s_axi_rvalid && s_axi_rready)
        r_active <= 1'b0;
    end
  end

  // ----------------------------------------------------------------
  // DECERR responder for unmapped regions (same joint AW/W contract)
  // ----------------------------------------------------------------
  logic err_awready, err_wready, err_bvalid;
  logic err_arready, err_rvalid;

  typedef enum logic [1:0] {
    EW_IDLE = 2'b00,
    EW_XFER = 2'b01,
    EW_RESP = 2'b10
  } ew_state_t;

  ew_state_t ew_state;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      ew_state    <= EW_IDLE;
      err_awready <= 1'b0;
      err_wready  <= 1'b0;
      err_bvalid  <= 1'b0;
    end
    else begin
      case (ew_state)
        EW_IDLE: begin
          if (!b_active && s_axi_awvalid && s_axi_wvalid && (aw_dec == SEL_NONE)) begin
            err_awready <= 1'b1;
            err_wready  <= 1'b1;
            ew_state    <= EW_XFER;
          end
        end
        EW_XFER: begin
          err_awready <= 1'b0;
          err_wready  <= 1'b0;
          err_bvalid  <= 1'b1;
          ew_state    <= EW_RESP;
        end
        EW_RESP: begin
          if (s_axi_bready) begin
            err_bvalid <= 1'b0;
            ew_state   <= EW_IDLE;
          end
        end
        default: ew_state <= EW_IDLE;
      endcase
    end
  end

  typedef enum logic [1:0] {
    ER_IDLE = 2'b00,
    ER_XFER = 2'b01,
    ER_RESP = 2'b10
  } er_state_t;

  er_state_t er_state;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      er_state    <= ER_IDLE;
      err_arready <= 1'b0;
      err_rvalid  <= 1'b0;
    end
    else begin
      case (er_state)
        ER_IDLE: begin
          if (!r_active && s_axi_arvalid && (ar_dec == SEL_NONE)) begin
            err_arready <= 1'b1;
            er_state    <= ER_XFER;
          end
        end
        ER_XFER: begin
          err_arready <= 1'b0;
          err_rvalid  <= 1'b1;
          er_state    <= ER_RESP;
        end
        ER_RESP: begin
          if (s_axi_rready) begin
            err_rvalid <= 1'b0;
            er_state   <= ER_IDLE;
          end
        end
        default: er_state <= ER_IDLE;
      endcase
    end
  end

  // ----------------------------------------------------------------
  // Payload broadcast (address/data/strobe fan out to every slave;
  // only the routed valid selects who acts on them)
  // ----------------------------------------------------------------
  assign m_rom_awaddr   = s_axi_awaddr;
  assign m_uart_awaddr  = s_axi_awaddr;
  assign m_timer_awaddr = s_axi_awaddr;
  assign m_ecc_awaddr   = s_axi_awaddr;
  assign m_gpio_awaddr  = s_axi_awaddr;

  assign m_rom_wdata    = s_axi_wdata;
  assign m_uart_wdata   = s_axi_wdata;
  assign m_timer_wdata  = s_axi_wdata;
  assign m_ecc_wdata    = s_axi_wdata;
  assign m_gpio_wdata   = s_axi_wdata;

  assign m_rom_wstrb    = s_axi_wstrb;
  assign m_uart_wstrb   = s_axi_wstrb;
  assign m_timer_wstrb  = s_axi_wstrb;
  assign m_ecc_wstrb    = s_axi_wstrb;
  assign m_gpio_wstrb   = s_axi_wstrb;

  assign m_rom_araddr   = s_axi_araddr;
  assign m_uart_araddr  = s_axi_araddr;
  assign m_timer_araddr = s_axi_araddr;
  assign m_ecc_araddr   = s_axi_araddr;
  assign m_gpio_araddr  = s_axi_araddr;

  // ----------------------------------------------------------------
  // Valid routing (write address + write data, gated by route decode)
  // ----------------------------------------------------------------
  assign m_rom_awvalid   = s_axi_awvalid && !b_active && (aw_dec == SEL_ROM);
  assign m_uart_awvalid  = s_axi_awvalid && !b_active && (aw_dec == SEL_UART);
  assign m_timer_awvalid = s_axi_awvalid && !b_active && (aw_dec == SEL_TIMER);
  assign m_ecc_awvalid   = s_axi_awvalid && !b_active && (aw_dec == SEL_ECC);
  assign m_gpio_awvalid  = s_axi_awvalid && !b_active && (aw_dec == SEL_GPIO);

  assign m_rom_wvalid    = s_axi_wvalid && !b_active && (aw_dec == SEL_ROM);
  assign m_uart_wvalid   = s_axi_wvalid && !b_active && (aw_dec == SEL_UART);
  assign m_timer_wvalid  = s_axi_wvalid && !b_active && (aw_dec == SEL_TIMER);
  assign m_ecc_wvalid    = s_axi_wvalid && !b_active && (aw_dec == SEL_ECC);
  assign m_gpio_wvalid   = s_axi_wvalid && !b_active && (aw_dec == SEL_GPIO);

  assign m_rom_arvalid   = s_axi_arvalid && !r_active && (ar_dec == SEL_ROM);
  assign m_uart_arvalid  = s_axi_arvalid && !r_active && (ar_dec == SEL_UART);
  assign m_timer_arvalid = s_axi_arvalid && !r_active && (ar_dec == SEL_TIMER);
  assign m_ecc_arvalid   = s_axi_arvalid && !r_active && (ar_dec == SEL_ECC);
  assign m_gpio_arvalid  = s_axi_arvalid && !r_active && (ar_dec == SEL_GPIO);

  // ----------------------------------------------------------------
  // Ready return muxes (AW/W/AR by live decode)
  // ----------------------------------------------------------------
  always_comb begin
    unique case (aw_dec)
      SEL_ROM:   begin s_axi_awready = m_rom_awready;   s_axi_wready = m_rom_wready;   end
      SEL_UART:  begin s_axi_awready = m_uart_awready;  s_axi_wready = m_uart_wready;  end
      SEL_TIMER: begin s_axi_awready = m_timer_awready; s_axi_wready = m_timer_wready; end
      SEL_ECC:   begin s_axi_awready = m_ecc_awready;   s_axi_wready = m_ecc_wready;   end
      SEL_GPIO:  begin s_axi_awready = m_gpio_awready;  s_axi_wready = m_gpio_wready;  end
      default:   begin s_axi_awready = err_awready;     s_axi_wready = err_wready;     end
    endcase
  end

  always_comb begin
    unique case (ar_dec)
      SEL_ROM:   s_axi_arready = m_rom_arready;
      SEL_UART:  s_axi_arready = m_uart_arready;
      SEL_TIMER: s_axi_arready = m_timer_arready;
      SEL_ECC:   s_axi_arready = m_ecc_arready;
      SEL_GPIO:  s_axi_arready = m_gpio_arready;
      default:   s_axi_arready = err_arready;
    endcase
  end

  // ----------------------------------------------------------------
  // B channel return (routed by the latched bsel_q)
  // ----------------------------------------------------------------
  always_comb begin
    unique case (bsel_q)
      SEL_ROM:   begin s_axi_bvalid = m_rom_bvalid;   s_axi_bresp = m_rom_bresp;   end
      SEL_UART:  begin s_axi_bvalid = m_uart_bvalid;  s_axi_bresp = m_uart_bresp;  end
      SEL_TIMER: begin s_axi_bvalid = m_timer_bvalid; s_axi_bresp = m_timer_bresp; end
      SEL_ECC:   begin s_axi_bvalid = m_ecc_bvalid;   s_axi_bresp = m_ecc_bresp;   end
      SEL_GPIO:  begin s_axi_bvalid = m_gpio_bvalid;  s_axi_bresp = m_gpio_bresp;  end
      default:   begin s_axi_bvalid = err_bvalid;     s_axi_bresp = RESP_DECERR;   end
    endcase
  end

  assign m_rom_bready   = s_axi_bready && b_active && (bsel_q == SEL_ROM);
  assign m_uart_bready  = s_axi_bready && b_active && (bsel_q == SEL_UART);
  assign m_timer_bready = s_axi_bready && b_active && (bsel_q == SEL_TIMER);
  assign m_ecc_bready   = s_axi_bready && b_active && (bsel_q == SEL_ECC);
  assign m_gpio_bready  = s_axi_bready && b_active && (bsel_q == SEL_GPIO);

  // ----------------------------------------------------------------
  // R channel return (routed by the latched rsel_q)
  // ----------------------------------------------------------------
  always_comb begin
    unique case (rsel_q)
      SEL_ROM:   begin s_axi_rvalid = m_rom_rvalid;   s_axi_rdata = m_rom_rdata;   s_axi_rresp = m_rom_rresp;   end
      SEL_UART:  begin s_axi_rvalid = m_uart_rvalid;  s_axi_rdata = m_uart_rdata;  s_axi_rresp = m_uart_rresp;  end
      SEL_TIMER: begin s_axi_rvalid = m_timer_rvalid; s_axi_rdata = m_timer_rdata; s_axi_rresp = m_timer_rresp; end
      SEL_ECC:   begin s_axi_rvalid = m_ecc_rvalid;   s_axi_rdata = m_ecc_rdata;   s_axi_rresp = m_ecc_rresp;   end
      SEL_GPIO:  begin s_axi_rvalid = m_gpio_rvalid;  s_axi_rdata = m_gpio_rdata;  s_axi_rresp = m_gpio_rresp;  end
      default:   begin s_axi_rvalid = err_rvalid;     s_axi_rdata = 32'b0;         s_axi_rresp = RESP_DECERR;   end
    endcase
  end

  assign m_rom_rready   = s_axi_rready && r_active && (rsel_q == SEL_ROM);
  assign m_uart_rready  = s_axi_rready && r_active && (rsel_q == SEL_UART);
  assign m_timer_rready = s_axi_rready && r_active && (rsel_q == SEL_TIMER);
  assign m_ecc_rready   = s_axi_rready && r_active && (rsel_q == SEL_ECC);
  assign m_gpio_rready  = s_axi_rready && r_active && (rsel_q == SEL_GPIO);

  // Sink for intentionally unused protection qualifiers
  wire _unused_prot;
  assign _unused_prot = &{1'b0, s_axi_awprot, s_axi_arprot};

endmodule
