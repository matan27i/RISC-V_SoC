// Testbench - SIMULATION ONLY (do not add to synthesis sources)
// Self-checking test for the AXI4-Lite timeout monitor (TIMEOUT=16):
//   1. Normal read   : healthy slave -> OKAY data, no timeout pulse
//   2. Normal write  : healthy slave -> OKAY response, no timeout pulse
//   3. Read timeout  : hung slave -> wrapper injects RRESP=SLVERR,
//                      rd_timeout_evt pulses, slave AR is isolated
//   4. Write timeout : hung slave -> wrapper injects BRESP=SLVERR,
//                      wr_timeout_evt pulses
//   5. Counter reset : slave answers a few cycles before expiry ->
//                      normal completion, no timeout
//   6. Recovery      : after a timeout, a normal transaction still works

`timescale 1ns/1ps

module tb_axi4lite_timeout_monitor;

  localparam int unsigned TO = 16;

  int errors = 0;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic resetn = 1'b0;

  // ---- master-facing (s_axi) signals, driven by the master BFM ----
  logic [31:0] s_awaddr = '0;  logic s_awvalid = 0; logic s_awready;
  logic [31:0] s_wdata  = '0;  logic [3:0] s_wstrb = 4'hF; logic s_wvalid = 0; logic s_wready;
  logic [1:0]  s_bresp;        logic s_bvalid;     logic s_bready = 0;
  logic [31:0] s_araddr = '0;  logic s_arvalid = 0; logic s_arready;
  logic [31:0] s_rdata;        logic [1:0] s_rresp; logic s_rvalid; logic s_rready = 0;

  // ---- slave-facing (m_axi) signals, driven by the slave model ----
  logic [31:0] m_awaddr;       logic [2:0] m_awprot; logic m_awvalid; logic m_awready;
  logic [31:0] m_wdata;        logic [3:0] m_wstrb;  logic m_wvalid;  logic m_wready;
  logic [1:0]  m_bresp;        logic m_bvalid;       logic m_bready;
  logic [31:0] m_araddr;       logic [2:0] m_arprot; logic m_arvalid; logic m_arready;
  logic [31:0] m_rdata;        logic [1:0] m_rresp;  logic m_rvalid;  logic m_rready;

  logic wr_timeout_evt, rd_timeout_evt;

  // slave behavior controls
  logic       rd_hang = 0, wr_hang = 0;
  int         rd_lat  = 1;          // cycles AR is held before slave accepts

  // ----------------------------------------------------------------
  // DUT
  // ----------------------------------------------------------------
  axi4lite_timeout_monitor #(.ADDR_WIDTH(32), .DATA_WIDTH(32), .TIMEOUT_CYCLES(TO)) dut (
    .clk(clk), .resetn(resetn),
    .s_axi_awaddr(s_awaddr), .s_axi_awprot(3'b000), .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready),
    .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb), .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready),
    .s_axi_bresp(s_bresp), .s_axi_bvalid(s_bvalid), .s_axi_bready(s_bready),
    .s_axi_araddr(s_araddr), .s_axi_arprot(3'b000), .s_axi_arvalid(s_arvalid), .s_axi_arready(s_arready),
    .s_axi_rdata(s_rdata), .s_axi_rresp(s_rresp), .s_axi_rvalid(s_rvalid), .s_axi_rready(s_rready),
    .m_axi_awaddr(m_awaddr), .m_axi_awprot(m_awprot), .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
    .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb), .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
    .m_axi_bresp(m_bresp), .m_axi_bvalid(m_bvalid), .m_axi_bready(m_bready),
    .m_axi_araddr(m_araddr), .m_axi_arprot(m_arprot), .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
    .m_axi_rdata(m_rdata), .m_axi_rresp(m_rresp), .m_axi_rvalid(m_rvalid), .m_axi_rready(m_rready),
    .wr_timeout_evt(wr_timeout_evt), .rd_timeout_evt(rd_timeout_evt)
  );

  // ----------------------------------------------------------------
  // Behavioral downstream slave (healthy unless *_hang asserted)
  // ----------------------------------------------------------------
  localparam logic [31:0] SLAVE_RDATA = 32'hCAFEBABE;

  typedef enum logic [1:0] {RS_IDLE, RS_HS, RS_R} rs_t;
  rs_t  rs;
  int   rlc;

  always_ff @(posedge clk) begin
    if (!resetn) begin
      rs <= RS_IDLE; rlc <= 0;
      m_arready <= 0; m_rvalid <= 0; m_rresp <= 2'b00; m_rdata <= '0;
    end
    else begin
      m_arready <= 0;
      case (rs)
        RS_IDLE: begin
          m_rvalid <= 0;
          if (m_arvalid && !rd_hang) begin
            if (rlc >= rd_lat) begin
              m_arready <= 1;        // accept next cycle
              rlc       <= 0;
              rs        <= RS_HS;
            end
            else rlc <= rlc + 1;
          end
          else rlc <= 0;
        end
        RS_HS: begin
          m_rvalid <= 1; m_rresp <= 2'b00; m_rdata <= SLAVE_RDATA;
          rs <= RS_R;
        end
        RS_R: if (m_rready) begin m_rvalid <= 0; rs <= RS_IDLE; end
        default: rs <= RS_IDLE;
      endcase
    end
  end

  typedef enum logic [1:0] {WS_IDLE, WS_HS, WS_B} ws_t;
  ws_t ws;

  always_ff @(posedge clk) begin
    if (!resetn) begin
      ws <= WS_IDLE;
      m_awready <= 0; m_wready <= 0; m_bvalid <= 0; m_bresp <= 2'b00;
    end
    else begin
      m_awready <= 0; m_wready <= 0;
      case (ws)
        WS_IDLE: begin
          m_bvalid <= 0;
          if (m_awvalid && m_wvalid && !wr_hang) begin
            m_awready <= 1; m_wready <= 1;   // accept both next cycle
            ws <= WS_HS;
          end
        end
        WS_HS: begin m_bvalid <= 1; m_bresp <= 2'b00; ws <= WS_B; end
        WS_B:  if (m_bready) begin m_bvalid <= 0; ws <= WS_IDLE; end
        default: ws <= WS_IDLE;
      endcase
    end
  end

  // ----------------------------------------------------------------
  // Master BFM tasks (drive at negedge, detect handshakes at posedge)
  // ----------------------------------------------------------------
  task automatic do_read(input logic [31:0] addr, output logic [1:0] resp, output logic [31:0] data);
    @(negedge clk);
    s_araddr = addr; s_arvalid = 1'b1; s_rready = 1'b1;
    do @(posedge clk); while (!(s_arvalid && s_arready));
    @(negedge clk) s_arvalid = 1'b0;
    do @(posedge clk); while (!(s_rvalid && s_rready));
    resp = s_rresp; data = s_rdata;
    @(negedge clk) s_rready = 1'b0;
  endtask

  task automatic do_write(input logic [31:0] addr, input logic [31:0] data, output logic [1:0] resp);
    @(negedge clk);
    s_awaddr = addr; s_awvalid = 1'b1;
    s_wdata  = data; s_wvalid  = 1'b1;
    s_bready = 1'b1;
    // hold AW/W until each is accepted
    fork
      begin do @(posedge clk); while (!(s_awvalid && s_awready)); @(negedge clk) s_awvalid = 1'b0; end
      begin do @(posedge clk); while (!(s_wvalid  && s_wready));  @(negedge clk) s_wvalid  = 1'b0; end
    join
    do @(posedge clk); while (!(s_bvalid && s_bready));
    resp = s_bresp;
    @(negedge clk) s_bready = 1'b0;
  endtask

  task automatic check(input bit cond, input string msg);
    if (cond) $display("[%0t ns] OK   : %s", $time, msg);
    else begin $display("[%0t ns] ERROR: %s", $time, msg); errors++; end
  endtask

  // track timeout pulses across a transaction window
  logic rd_evt_seen, wr_evt_seen;
  always_ff @(posedge clk) begin
    if (rd_timeout_evt) rd_evt_seen <= 1'b1;
    if (wr_timeout_evt) wr_evt_seen <= 1'b1;
  end

  initial begin
    #2_000_000;
    $display("ERROR: watchdog timeout");
    $finish;
  end

  logic [1:0]  resp;
  logic [31:0] data;

  initial begin
    repeat (5) @(negedge clk);
    resetn = 1'b1;
    repeat (3) @(negedge clk);

    // ---- Test 1: normal read ----
    $display("---- Test 1: normal read (healthy slave) ----");
    rd_hang = 0; rd_lat = 2; rd_evt_seen = 0;
    do_read(32'h0000_1000, resp, data);
    check(resp == 2'b00,        "read RRESP = OKAY");
    check(data == SLAVE_RDATA,  "read data = slave data (0xCAFEBABE)");
    check(rd_evt_seen == 1'b0,  "no read-timeout pulse on healthy read");

    // ---- Test 2: normal write ----
    $display("---- Test 2: normal write (healthy slave) ----");
    wr_hang = 0; wr_evt_seen = 0;
    do_write(32'h0000_2000, 32'h12345678, resp);
    check(resp == 2'b00,        "write BRESP = OKAY");
    check(wr_evt_seen == 1'b0,  "no write-timeout pulse on healthy write");

    // ---- Test 3: read timeout ----
    $display("---- Test 3: read timeout (hung slave) ----");
    rd_hang = 1; rd_evt_seen = 0;
    do_read(32'h0000_3000, resp, data);
    check(resp == 2'b10,        "read RRESP = SLVERR injected");
    check(rd_evt_seen == 1'b1,  "rd_timeout_evt pulsed");
    check(m_arvalid == 1'b0,    "slave AR isolated after injection");

    // ---- Test 4: write timeout ----
    $display("---- Test 4: write timeout (hung slave) ----");
    rd_hang = 0; wr_hang = 1; wr_evt_seen = 0;
    do_write(32'h0000_4000, 32'hDEADBEEF, resp);
    check(resp == 2'b10,        "write BRESP = SLVERR injected");
    check(wr_evt_seen == 1'b1,  "wr_timeout_evt pulsed");
    check(m_awvalid == 1'b0,    "slave AW isolated after injection");

    // ---- Test 5: counter resets when slave answers just before expiry ----
    $display("---- Test 5: slave answers before expiry (no timeout) ----");
    wr_hang = 0; rd_hang = 0; rd_lat = TO-3; rd_evt_seen = 0;
    do_read(32'h0000_5000, resp, data);
    check(resp == 2'b00,        "late-but-in-time read completes OKAY");
    check(data == SLAVE_RDATA,  "late read returns real slave data");
    check(rd_evt_seen == 1'b0,  "counter reset: no timeout despite long latency");

    // ---- Test 6: recovery after a timeout ----
    $display("---- Test 6: recovery (normal traffic after a timeout) ----");
    rd_lat = 1; rd_evt_seen = 0; wr_evt_seen = 0;
    do_write(32'h0000_6000, 32'hA5A5A5A5, resp);
    check(resp == 2'b00 && !wr_evt_seen, "write works after prior timeout");
    do_read(32'h0000_6004, resp, data);
    check(resp == 2'b00 && data == SLAVE_RDATA && !rd_evt_seen, "read works after prior timeout");

    if (errors == 0) $display("==== ALL TESTS PASSED ====");
    else             $display("==== %0d TEST(S) FAILED ====", errors);
    $finish;
  end

endmodule
