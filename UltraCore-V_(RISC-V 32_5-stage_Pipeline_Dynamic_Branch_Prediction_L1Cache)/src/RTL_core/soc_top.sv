// UltraCore-V SoC Top - RTL
// Integrates the 5-stage RV32I pipelined core (with its L1 caches and
// internal main RAM) with the AXI4-Lite peripheral subsystem.
//
//                +--------------------------------------+
//                |  five_stage_pipeline_riscv_core      |
//   irq_timer -->|  (I$/D$, BTB, main RAM @0x2000)      |
//   irq_ecc   -->|                                      |
//                |   rom_fetch port        mmio port    |
//                +-------+--------------------+---------+
//                        |                    |
//                  +-----v-----+      +-------v---------+
//                  | boot_rom  |      | axi_master_     |
//                  | (port A)  |      | bridge          |
//                  +-----+-----+      +-------+---------+
//                        | port B (AXI)       | AXI4-Lite
//                        |             +------v---------+
//                        +------------>|  axi_crossbar  |
//                                      +-+--+--+--+--+--+
//                                        |  |  |  |  |
//                                     ROM  UART TMR ECC GPIO
//
// Memory map:
//   0x0000_0000 - 0x0000_1FFF  Boot ROM   (fetch via I$, data via AXI)
//   0x0000_2000 - 0x0000_3FFF  Main RAM   (cacheable, execution space)
//   0x4000_0000 - 0x4000_000F  UART       (TX, RX, STATUS)
//   0x4000_0010 - 0x4000_001F  Timer      (CTRL, COMPARE, COUNT, STATUS)
//   0x4000_0020 - 0x4000_002F  ECC accel  (DATA, STATUS, CTRL)
//   0x4000_0030 - 0x4000_003F  GPIO       (DATA, DIR)
//
// Boot flow: the core resets to PC = 0x0000_0000 and executes the ROM
// bootloader (prints "LIVE\r\n" on the UART, then jumps to 0x2000).
// The application image is loaded into main RAM from MEM_INIT_FILE;
// it must be linked for base address 0x0000_2000.
//
// ECC receive path: UART RX bytes are paired into 13-bit SEC-DED
// frames by ecc_frame_packer (byte0 = payload, byte1[4:0] = check
// bits) and pulled by the ECC accelerator, whose corrected payload is
// read by software at 0x4000_0020.
//
// Interrupts: the timer compare-match flag and the ECC DATA_READY
// interrupt are routed to the core's irq pins (pending hooks for the
// future trap unit) - both are also observable through the peripherals'
// own STATUS registers, which is how today's RV32I software uses them.

module soc_top #(
  parameter int unsigned CLK_FREQ_HZ   = 100_000_000,
  parameter int unsigned BAUD_RATE     = 115_200,
  parameter              MEM_INIT_FILE = "machine_code.mem"
)(
  input  logic        clk,
  input  logic        resetn,

  // Serial port
  output logic        uart_txd,
  input  logic        uart_rxd,

  // General-purpose I/O pins
  inout  wire  [7:0]  gpio,

  // Debug: architectural PC of the fetch stage
  output logic [31:0] debug_pc
);

  // ----------------------------------------------------------------
  // Core <-> bridge / boot ROM nets
  // ----------------------------------------------------------------
  logic        mmio_req, mmio_wr_en, mmio_zero_extend, mmio_ready, mmio_accept;
  logic [31:0] mmio_addr, mmio_wr_data, mmio_rd_data;
  risc_pkg::mem_size_t mmio_size;

  logic        rom_fetch_req, rom_fetch_data_valid, rom_fetch_ready;
  logic [31:0] rom_fetch_addr, rom_fetch_data;

  logic        irq_timer_w, irq_ecc_w;

  // ----------------------------------------------------------------
  // Bridge <-> crossbar AXI nets
  // ----------------------------------------------------------------
  logic [31:0] xb_awaddr, xb_wdata, xb_araddr, xb_rdata;
  logic [2:0]  xb_awprot, xb_arprot;
  logic [3:0]  xb_wstrb;
  logic [1:0]  xb_bresp, xb_rresp;
  logic        xb_awvalid, xb_awready, xb_wvalid, xb_wready;
  logic        xb_bvalid, xb_bready, xb_arvalid, xb_arready;
  logic        xb_rvalid, xb_rready;

  // Crossbar -> slave nets (per slave)
  logic [31:0] rom_awaddr, rom_wdata, rom_araddr, rom_rdata;
  logic [3:0]  rom_wstrb;
  logic [1:0]  rom_bresp, rom_rresp;
  logic        rom_awvalid, rom_awready, rom_wvalid, rom_wready;
  logic        rom_bvalid, rom_bready, rom_arvalid, rom_arready;
  logic        rom_rvalid, rom_rready;

  logic [31:0] uart_awaddr, uart_wdata, uart_araddr, uart_rdata;
  logic [3:0]  uart_wstrb;
  logic [1:0]  uart_bresp, uart_rresp;
  logic        uart_awvalid, uart_awready, uart_wvalid, uart_wready;
  logic        uart_bvalid, uart_bready, uart_arvalid, uart_arready;
  logic        uart_rvalid, uart_rready;

  logic [31:0] timer_awaddr, timer_wdata, timer_araddr, timer_rdata;
  logic [3:0]  timer_wstrb;
  logic [1:0]  timer_bresp, timer_rresp;
  logic        timer_awvalid, timer_awready, timer_wvalid, timer_wready;
  logic        timer_bvalid, timer_bready, timer_arvalid, timer_arready;
  logic        timer_rvalid, timer_rready;

  logic [31:0] ecc_awaddr, ecc_wdata, ecc_araddr, ecc_rdata;
  logic [3:0]  ecc_wstrb;
  logic [1:0]  ecc_bresp, ecc_rresp;
  logic        ecc_awvalid, ecc_awready, ecc_wvalid, ecc_wready;
  logic        ecc_bvalid, ecc_bready, ecc_arvalid, ecc_arready;
  logic        ecc_rvalid, ecc_rready;

  logic [31:0] gpio_awaddr, gpio_wdata, gpio_araddr, gpio_rdata;
  logic [3:0]  gpio_wstrb;
  logic [1:0]  gpio_bresp, gpio_rresp;
  logic        gpio_awvalid, gpio_awready, gpio_wvalid, gpio_wready;
  logic        gpio_bvalid, gpio_bready, gpio_arvalid, gpio_arready;
  logic        gpio_rvalid, gpio_rready;

  // ----------------------------------------------------------------
  // UART RX stream tap -> ECC frame packer -> ECC accelerator FIFO
  // ----------------------------------------------------------------
  logic [7:0]  rx_stream_data;
  logic        rx_stream_valid, rx_stream_ferr;
  logic        ecc_fifo_empty, ecc_fifo_read_en;
  logic [12:0] ecc_fifo_data;

  // GPIO pad-side nets
  logic [7:0] gpio_out, gpio_oe, gpio_in;

  // ----------------------------------------------------------------
  // RISC-V core (caches + main RAM inside)
  // ----------------------------------------------------------------
  five_stage_pipeline_riscv_core #(
    .RESET_PC      (32'h0000_0000),       // Boot from ROM
    .MEM_INIT_FILE (MEM_INIT_FILE)
  ) u_core (
    .clk                  (clk),
    .reset_n              (resetn),
    .pc_out               (debug_pc),
    .mmio_req             (mmio_req),
    .mmio_wr_en           (mmio_wr_en),
    .mmio_addr            (mmio_addr),
    .mmio_wr_data         (mmio_wr_data),
    .mmio_size            (mmio_size),
    .mmio_zero_extend     (mmio_zero_extend),
    .mmio_rd_data         (mmio_rd_data),
    .mmio_ready           (mmio_ready),
    .mmio_accept          (mmio_accept),
    .rom_fetch_req        (rom_fetch_req),
    .rom_fetch_addr       (rom_fetch_addr),
    .rom_fetch_data       (rom_fetch_data),
    .rom_fetch_data_valid (rom_fetch_data_valid),
    .rom_fetch_ready      (rom_fetch_ready),
    .irq_timer            (irq_timer_w),
    .irq_ecc              (irq_ecc_w)
  );

  // ----------------------------------------------------------------
  // MMIO -> AXI4-Lite master bridge
  // ----------------------------------------------------------------
  axi_master_bridge u_bridge (
    .clk              (clk),
    .resetn           (resetn),
    .mmio_req         (mmio_req),
    .mmio_wr_en       (mmio_wr_en),
    .mmio_addr        (mmio_addr),
    .mmio_wr_data     (mmio_wr_data),
    .mmio_size        (mmio_size),
    .mmio_zero_extend (mmio_zero_extend),
    .mmio_rd_data     (mmio_rd_data),
    .mmio_ready       (mmio_ready),
    .mmio_accept      (mmio_accept),
    .m_axi_awaddr     (xb_awaddr),
    .m_axi_awprot     (xb_awprot),
    .m_axi_awvalid    (xb_awvalid),
    .m_axi_awready    (xb_awready),
    .m_axi_wdata      (xb_wdata),
    .m_axi_wstrb      (xb_wstrb),
    .m_axi_wvalid     (xb_wvalid),
    .m_axi_wready     (xb_wready),
    .m_axi_bresp      (xb_bresp),
    .m_axi_bvalid     (xb_bvalid),
    .m_axi_bready     (xb_bready),
    .m_axi_araddr     (xb_araddr),
    .m_axi_arprot     (xb_arprot),
    .m_axi_arvalid    (xb_arvalid),
    .m_axi_arready    (xb_arready),
    .m_axi_rdata      (xb_rdata),
    .m_axi_rresp      (xb_rresp),
    .m_axi_rvalid     (xb_rvalid),
    .m_axi_rready     (xb_rready)
  );

  // ----------------------------------------------------------------
  // AXI4-Lite interconnect
  // ----------------------------------------------------------------
  axi_crossbar u_xbar (
    .clk             (clk),
    .resetn          (resetn),
    .s_axi_awaddr    (xb_awaddr),
    .s_axi_awprot    (xb_awprot),
    .s_axi_awvalid   (xb_awvalid),
    .s_axi_awready   (xb_awready),
    .s_axi_wdata     (xb_wdata),
    .s_axi_wstrb     (xb_wstrb),
    .s_axi_wvalid    (xb_wvalid),
    .s_axi_wready    (xb_wready),
    .s_axi_bresp     (xb_bresp),
    .s_axi_bvalid    (xb_bvalid),
    .s_axi_bready    (xb_bready),
    .s_axi_araddr    (xb_araddr),
    .s_axi_arprot    (xb_arprot),
    .s_axi_arvalid   (xb_arvalid),
    .s_axi_arready   (xb_arready),
    .s_axi_rdata     (xb_rdata),
    .s_axi_rresp     (xb_rresp),
    .s_axi_rvalid    (xb_rvalid),
    .s_axi_rready    (xb_rready),

    .m_rom_awaddr    (rom_awaddr),
    .m_rom_awvalid   (rom_awvalid),
    .m_rom_awready   (rom_awready),
    .m_rom_wdata     (rom_wdata),
    .m_rom_wstrb     (rom_wstrb),
    .m_rom_wvalid    (rom_wvalid),
    .m_rom_wready    (rom_wready),
    .m_rom_bresp     (rom_bresp),
    .m_rom_bvalid    (rom_bvalid),
    .m_rom_bready    (rom_bready),
    .m_rom_araddr    (rom_araddr),
    .m_rom_arvalid   (rom_arvalid),
    .m_rom_arready   (rom_arready),
    .m_rom_rdata     (rom_rdata),
    .m_rom_rresp     (rom_rresp),
    .m_rom_rvalid    (rom_rvalid),
    .m_rom_rready    (rom_rready),

    .m_uart_awaddr   (uart_awaddr),
    .m_uart_awvalid  (uart_awvalid),
    .m_uart_awready  (uart_awready),
    .m_uart_wdata    (uart_wdata),
    .m_uart_wstrb    (uart_wstrb),
    .m_uart_wvalid   (uart_wvalid),
    .m_uart_wready   (uart_wready),
    .m_uart_bresp    (uart_bresp),
    .m_uart_bvalid   (uart_bvalid),
    .m_uart_bready   (uart_bready),
    .m_uart_araddr   (uart_araddr),
    .m_uart_arvalid  (uart_arvalid),
    .m_uart_arready  (uart_arready),
    .m_uart_rdata    (uart_rdata),
    .m_uart_rresp    (uart_rresp),
    .m_uart_rvalid   (uart_rvalid),
    .m_uart_rready   (uart_rready),

    .m_timer_awaddr  (timer_awaddr),
    .m_timer_awvalid (timer_awvalid),
    .m_timer_awready (timer_awready),
    .m_timer_wdata   (timer_wdata),
    .m_timer_wstrb   (timer_wstrb),
    .m_timer_wvalid  (timer_wvalid),
    .m_timer_wready  (timer_wready),
    .m_timer_bresp   (timer_bresp),
    .m_timer_bvalid  (timer_bvalid),
    .m_timer_bready  (timer_bready),
    .m_timer_araddr  (timer_araddr),
    .m_timer_arvalid (timer_arvalid),
    .m_timer_arready (timer_arready),
    .m_timer_rdata   (timer_rdata),
    .m_timer_rresp   (timer_rresp),
    .m_timer_rvalid  (timer_rvalid),
    .m_timer_rready  (timer_rready),

    .m_ecc_awaddr    (ecc_awaddr),
    .m_ecc_awvalid   (ecc_awvalid),
    .m_ecc_awready   (ecc_awready),
    .m_ecc_wdata     (ecc_wdata),
    .m_ecc_wstrb     (ecc_wstrb),
    .m_ecc_wvalid    (ecc_wvalid),
    .m_ecc_wready    (ecc_wready),
    .m_ecc_bresp     (ecc_bresp),
    .m_ecc_bvalid    (ecc_bvalid),
    .m_ecc_bready    (ecc_bready),
    .m_ecc_araddr    (ecc_araddr),
    .m_ecc_arvalid   (ecc_arvalid),
    .m_ecc_arready   (ecc_arready),
    .m_ecc_rdata     (ecc_rdata),
    .m_ecc_rresp     (ecc_rresp),
    .m_ecc_rvalid    (ecc_rvalid),
    .m_ecc_rready    (ecc_rready),

    .m_gpio_awaddr   (gpio_awaddr),
    .m_gpio_awvalid  (gpio_awvalid),
    .m_gpio_awready  (gpio_awready),
    .m_gpio_wdata    (gpio_wdata),
    .m_gpio_wstrb    (gpio_wstrb),
    .m_gpio_wvalid   (gpio_wvalid),
    .m_gpio_wready   (gpio_wready),
    .m_gpio_bresp    (gpio_bresp),
    .m_gpio_bvalid   (gpio_bvalid),
    .m_gpio_bready   (gpio_bready),
    .m_gpio_araddr   (gpio_araddr),
    .m_gpio_arvalid  (gpio_arvalid),
    .m_gpio_arready  (gpio_arready),
    .m_gpio_rdata    (gpio_rdata),
    .m_gpio_rresp    (gpio_rresp),
    .m_gpio_rvalid   (gpio_rvalid),
    .m_gpio_rready   (gpio_rready)
  );

  // ----------------------------------------------------------------
  // S0: Boot ROM (fetch port to the core, AXI port to the crossbar)
  // ----------------------------------------------------------------
  boot_rom u_boot_rom (
    .clk              (clk),
    .resetn           (resetn),
    .fetch_req        (rom_fetch_req),
    .fetch_addr       (rom_fetch_addr),
    .fetch_data       (rom_fetch_data),
    .fetch_data_valid (rom_fetch_data_valid),
    .fetch_ready      (rom_fetch_ready),
    .s_axi_awaddr     (rom_awaddr[12:0]),
    .s_axi_awvalid    (rom_awvalid),
    .s_axi_awready    (rom_awready),
    .s_axi_wdata      (rom_wdata),
    .s_axi_wstrb      (rom_wstrb),
    .s_axi_wvalid     (rom_wvalid),
    .s_axi_wready     (rom_wready),
    .s_axi_bresp      (rom_bresp),
    .s_axi_bvalid     (rom_bvalid),
    .s_axi_bready     (rom_bready),
    .s_axi_araddr     (rom_araddr[12:0]),
    .s_axi_arvalid    (rom_arvalid),
    .s_axi_arready    (rom_arready),
    .s_axi_rdata      (rom_rdata),
    .s_axi_rresp      (rom_rresp),
    .s_axi_rvalid     (rom_rvalid),
    .s_axi_rready     (rom_rready)
  );

  // ----------------------------------------------------------------
  // S1: UART (with RX stream tap feeding the ECC packer)
  // ----------------------------------------------------------------
  uart_axi_wrapper #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .BAUD_RATE   (BAUD_RATE)
  ) u_uart_wrap (
    .clk             (clk),
    .resetn          (resetn),
    .s_axi_awaddr    (uart_awaddr[3:0]),
    .s_axi_awvalid   (uart_awvalid),
    .s_axi_awready   (uart_awready),
    .s_axi_wdata     (uart_wdata),
    .s_axi_wstrb     (uart_wstrb),
    .s_axi_wvalid    (uart_wvalid),
    .s_axi_wready    (uart_wready),
    .s_axi_bresp     (uart_bresp),
    .s_axi_bvalid    (uart_bvalid),
    .s_axi_bready    (uart_bready),
    .s_axi_araddr    (uart_araddr[3:0]),
    .s_axi_arvalid   (uart_arvalid),
    .s_axi_arready   (uart_arready),
    .s_axi_rdata     (uart_rdata),
    .s_axi_rresp     (uart_rresp),
    .s_axi_rvalid    (uart_rvalid),
    .s_axi_rready    (uart_rready),
    .rx_stream_data  (rx_stream_data),
    .rx_stream_valid (rx_stream_valid),
    .rx_stream_ferr  (rx_stream_ferr),
    .txd             (uart_txd),
    .rxd             (uart_rxd)
  );

  // ----------------------------------------------------------------
  // S2: Timer
  // ----------------------------------------------------------------
  timer_axi_wrapper u_timer_wrap (
    .clk           (clk),
    .resetn        (resetn),
    .s_axi_awaddr  (timer_awaddr[3:0]),
    .s_axi_awvalid (timer_awvalid),
    .s_axi_awready (timer_awready),
    .s_axi_wdata   (timer_wdata),
    .s_axi_wstrb   (timer_wstrb),
    .s_axi_wvalid  (timer_wvalid),
    .s_axi_wready  (timer_wready),
    .s_axi_bresp   (timer_bresp),
    .s_axi_bvalid  (timer_bvalid),
    .s_axi_bready  (timer_bready),
    .s_axi_araddr  (timer_araddr[3:0]),
    .s_axi_arvalid (timer_arvalid),
    .s_axi_arready (timer_arready),
    .s_axi_rdata   (timer_rdata),
    .s_axi_rresp   (timer_rresp),
    .s_axi_rvalid  (timer_rvalid),
    .s_axi_rready  (timer_rready),
    .irq           (irq_timer_w)
  );

  // ----------------------------------------------------------------
  // S3: ECC accelerator (existing IP) + frame packer front-end
  // ----------------------------------------------------------------
  ecc_frame_packer u_packer (
    .clk          (clk),
    .resetn       (resetn),
    .byte_data    (rx_stream_data),
    .byte_valid   (rx_stream_valid),
    .byte_ferr    (rx_stream_ferr),
    .fifo_empty   (ecc_fifo_empty),
    .fifo_read_en (ecc_fifo_read_en),
    .fifo_data    (ecc_fifo_data)
  );

  ecc_accel u_ecc (
    .clk           (clk),
    .resetn        (resetn),
    .rx_empty      (ecc_fifo_empty),
    .rx_read_en    (ecc_fifo_read_en),
    .rx_data       (ecc_fifo_data),
    .s_axi_awaddr  (ecc_awaddr[3:0]),
    .s_axi_awvalid (ecc_awvalid),
    .s_axi_awready (ecc_awready),
    .s_axi_wdata   (ecc_wdata),
    .s_axi_wstrb   (ecc_wstrb),
    .s_axi_wvalid  (ecc_wvalid),
    .s_axi_wready  (ecc_wready),
    .s_axi_bresp   (ecc_bresp),
    .s_axi_bvalid  (ecc_bvalid),
    .s_axi_bready  (ecc_bready),
    .s_axi_araddr  (ecc_araddr[3:0]),
    .s_axi_arvalid (ecc_arvalid),
    .s_axi_arready (ecc_arready),
    .s_axi_rdata   (ecc_rdata),
    .s_axi_rresp   (ecc_rresp),
    .s_axi_rvalid  (ecc_rvalid),
    .s_axi_rready  (ecc_rready),
    .irq           (irq_ecc_w)
  );

  // ----------------------------------------------------------------
  // S4: GPIO with top-level tri-state pads
  // ----------------------------------------------------------------
  gpio_axi u_gpio (
    .clk           (clk),
    .resetn        (resetn),
    .s_axi_awaddr  (gpio_awaddr[3:0]),
    .s_axi_awvalid (gpio_awvalid),
    .s_axi_awready (gpio_awready),
    .s_axi_wdata   (gpio_wdata),
    .s_axi_wstrb   (gpio_wstrb),
    .s_axi_wvalid  (gpio_wvalid),
    .s_axi_wready  (gpio_wready),
    .s_axi_bresp   (gpio_bresp),
    .s_axi_bvalid  (gpio_bvalid),
    .s_axi_bready  (gpio_bready),
    .s_axi_araddr  (gpio_araddr[3:0]),
    .s_axi_arvalid (gpio_arvalid),
    .s_axi_arready (gpio_arready),
    .s_axi_rdata   (gpio_rdata),
    .s_axi_rresp   (gpio_rresp),
    .s_axi_rvalid  (gpio_rvalid),
    .s_axi_rready  (gpio_rready),
    .gpio_out      (gpio_out),
    .gpio_oe       (gpio_oe),
    .gpio_in       (gpio_in)
  );

  // Tri-state pads at the chip boundary (Vivado infers IOBUFs here)
  generate
    for (genvar g = 0; g < 8; g++) begin : g_gpio_pads
      assign gpio[g] = gpio_oe[g] ? gpio_out[g] : 1'bz;
    end
  endgenerate
  assign gpio_in = gpio;

endmodule
