// AXI4-Lite Timeout Monitor (Watchdog) - RTL
// Transparent pass-through wrapper inserted between an AXI4-Lite master
// and an AXI4-Lite slave. Under normal conditions it is combinationally
// invisible. If the downstream slave fails to accept a request within
// TIMEOUT_CYCLES, the monitor closes the master's handshake itself and
// injects a SLVERR response, so a hung slave can never lock up the bus.
//
// ----------------------------------------------------------------------
// Interface naming (standard AXI convention):
//   s_axi_*  - SLAVE port  : the upstream AXI master connects here
//              (this module behaves as a slave toward the master)
//   m_axi_*  - MASTER port : the downstream AXI slave connects here
//              (this module behaves as a master toward the slave)
//
// ----------------------------------------------------------------------
// Operation
//   Pass-through  : all address/data/response signals are wired straight
//                   through (combinational) so the monitor adds zero
//                   latency and full throughput when the slave is healthy.
//   Monitoring    : the counter runs while a request channel is stalled
//                   (VALID=1, READY=0). Reads watch AR; writes watch AW
//                   and W independently (a write needs both accepted).
//                   Any accepted handshake resets the counter immediately.
//   Timeout       : when the counter reaches TIMEOUT_CYCLES with the
//                   slave still not ready, the monitor
//                     (1) isolates the slave (drops the request VALID
//                         on the m_axi side),
//                     (2) asserts READY to the master for exactly one
//                         cycle to close the stuck address/data handshake,
//                     (3) drives the matching response (BVALID/BRESP for
//                         writes, RVALID/RRESP/RDATA for reads) with
//                         SLVERR back to the master,
//                     (4) returns to pass-through, counter cleared.
//
// Each direction uses an independent 3-state FSM:
//   PASS -> ACK (1-cycle READY pulse) -> RESP (hold response until
//   accepted) -> PASS.
//
// Design notes
//   * Synchronous logic clocked by clk; the FSMs/counters use an
//     active-low reset (resetn) asynchronously asserted and
//     synchronously released, matching the UltraCore-V SoC convention.
//     (The standalone version used a synchronous reset; only the
//     sensitivity lists differ.) The channel pass-through is
//     intentionally combinational - registering it would insert a
//     pipeline stage and break AXI4-Lite transparency/back-pressure.
//   * Isolation of an already-stuck slave deasserts the m_axi request
//     VALID without a completed handshake. This is a deliberate, bounded
//     protocol deviation toward a slave that is by definition
//     misbehaving; a healthy slave never reaches the timeout path.
//   * Spec-literal monitoring (VALID && !READY) assumes a well-behaved
//     master that presents AW and W within a bounded window. With the
//     default 256-cycle budget only a genuine slave stall trips it.

module axi4lite_timeout_monitor #(
  parameter int unsigned ADDR_WIDTH     = 32,
  parameter int unsigned DATA_WIDTH     = 32,
  parameter int unsigned TIMEOUT_CYCLES = 256
)(
  input  logic                    clk,
  input  logic                    resetn,

  // ================= Upstream slave port (faces the master) =========
  input  logic [ADDR_WIDTH-1:0]   s_axi_awaddr,
  input  logic [2:0]              s_axi_awprot,
  input  logic                    s_axi_awvalid,
  output logic                    s_axi_awready,
  input  logic [DATA_WIDTH-1:0]   s_axi_wdata,
  input  logic [DATA_WIDTH/8-1:0] s_axi_wstrb,
  input  logic                    s_axi_wvalid,
  output logic                    s_axi_wready,
  output logic [1:0]              s_axi_bresp,
  output logic                    s_axi_bvalid,
  input  logic                    s_axi_bready,
  input  logic [ADDR_WIDTH-1:0]   s_axi_araddr,
  input  logic [2:0]              s_axi_arprot,
  input  logic                    s_axi_arvalid,
  output logic                    s_axi_arready,
  output logic [DATA_WIDTH-1:0]   s_axi_rdata,
  output logic [1:0]              s_axi_rresp,
  output logic                    s_axi_rvalid,
  input  logic                    s_axi_rready,

  // ================= Downstream master port (faces the slave) =======
  output logic [ADDR_WIDTH-1:0]   m_axi_awaddr,
  output logic [2:0]              m_axi_awprot,
  output logic                    m_axi_awvalid,
  input  logic                    m_axi_awready,
  output logic [DATA_WIDTH-1:0]   m_axi_wdata,
  output logic [DATA_WIDTH/8-1:0] m_axi_wstrb,
  output logic                    m_axi_wvalid,
  input  logic                    m_axi_wready,
  input  logic [1:0]              m_axi_bresp,
  input  logic                    m_axi_bvalid,
  output logic                    m_axi_bready,
  output logic [ADDR_WIDTH-1:0]   m_axi_araddr,
  output logic [2:0]              m_axi_arprot,
  output logic                    m_axi_arvalid,
  input  logic                    m_axi_arready,
  input  logic [DATA_WIDTH-1:0]   m_axi_rdata,
  input  logic [1:0]              m_axi_rresp,
  input  logic                    m_axi_rvalid,
  output logic                    m_axi_rready,

  // ================= Status (optional, for SoC integration) =========
  output logic                    wr_timeout_evt,  // 1-cycle pulse on write timeout
  output logic                    rd_timeout_evt   // 1-cycle pulse on read timeout
);

  localparam logic [1:0] RESP_SLVERR = 2'b10;

  // Counter width: hold 0 .. TIMEOUT_CYCLES-1
  localparam int unsigned CW = (TIMEOUT_CYCLES <= 1) ? 1 : $clog2(TIMEOUT_CYCLES);

  // ==================================================================
  //  Read path (AR -> R)
  // ==================================================================
  typedef enum logic [1:0] {
    R_PASS = 2'b00,   // transparent pass-through + monitoring
    R_ACK  = 2'b01,   // 1-cycle ARREADY pulse to master (close handshake)
    R_RESP = 2'b10    // drive injected R (SLVERR) until master accepts
  } rstate_t;

  rstate_t       rstate;
  logic [CW-1:0] rcnt;
  logic          ar_stall;   // master wants a read, slave not accepting

  assign ar_stall = (rstate == R_PASS) && s_axi_arvalid && !m_axi_arready;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      rstate         <= R_PASS;
      rcnt           <= '0;
      rd_timeout_evt <= 1'b0;
    end
    else begin
      rd_timeout_evt <= 1'b0;
      case (rstate)
        R_PASS: begin
          if (ar_stall) begin
            if (rcnt == (TIMEOUT_CYCLES-1)) begin
              rstate         <= R_ACK;
              rcnt           <= '0;
              rd_timeout_evt <= 1'b1;
            end
            else
              rcnt <= rcnt + 1'b1;
          end
          else
            rcnt <= '0;          // progress or idle: reset immediately
        end

        R_ACK:  rstate <= R_RESP; // ARREADY asserted (combinationally) this cycle

        R_RESP: if (s_axi_rready) rstate <= R_PASS;

        default: rstate <= R_PASS;
      endcase
    end
  end

  // Read pass-through / isolation toward the slave
  assign m_axi_araddr  = s_axi_araddr;
  assign m_axi_arprot  = s_axi_arprot;
  assign m_axi_arvalid = s_axi_arvalid && (rstate == R_PASS);
  assign m_axi_rready  = s_axi_rready  && (rstate == R_PASS);

  // ARREADY toward the master
  always_comb begin
    case (rstate)
      R_PASS:  s_axi_arready = m_axi_arready;  // pass-through
      R_ACK:   s_axi_arready = 1'b1;           // one-cycle close
      default: s_axi_arready = 1'b0;
    endcase
  end

  // R channel toward the master (pass-through, or injected SLVERR)
  always_comb begin
    if (rstate == R_RESP) begin
      s_axi_rvalid = 1'b1;
      s_axi_rresp  = RESP_SLVERR;
      s_axi_rdata  = '0;                        // data invalid on error
    end
    else begin
      s_axi_rvalid = m_axi_rvalid && (rstate == R_PASS);
      s_axi_rresp  = m_axi_rresp;
      s_axi_rdata  = m_axi_rdata;
    end
  end

  // ==================================================================
  //  Write path (AW + W -> B)
  // ==================================================================
  typedef enum logic [1:0] {
    W_PASS = 2'b00,
    W_ACK  = 2'b01,   // 1-cycle AWREADY/WREADY pulse for the stuck channel(s)
    W_RESP = 2'b10    // drive injected B (SLVERR) until master accepts
  } wstate_t;

  wstate_t       wstate;
  logic [CW-1:0] wcnt;

  // Per-channel acceptance tracking for the in-flight write
  logic aw_hs, w_hs;
  logic aw_acc, w_acc, progress;
  logic aw_stall, w_stall, w_pending;

  assign aw_acc    = (wstate == W_PASS) && s_axi_awvalid && m_axi_awready;
  assign w_acc     = (wstate == W_PASS) && s_axi_wvalid  && m_axi_wready;
  assign progress  = aw_acc || w_acc;

  assign aw_stall  = s_axi_awvalid && !aw_hs;   // AW presented, not yet accepted
  assign w_stall   = s_axi_wvalid  && !w_hs;    // W presented, not yet accepted
  assign w_pending = (wstate == W_PASS) && (aw_stall || w_stall);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      wstate         <= W_PASS;
      wcnt           <= '0;
      aw_hs          <= 1'b0;
      w_hs           <= 1'b0;
      wr_timeout_evt <= 1'b0;
    end
    else begin
      wr_timeout_evt <= 1'b0;
      case (wstate)
        W_PASS: begin
          // Accumulate channel acceptances for the current transaction
          if (aw_acc) aw_hs <= 1'b1;
          if (w_acc)  w_hs  <= 1'b1;
          // Normal completion: clear once the slave's B is taken
          if (s_axi_bvalid && s_axi_bready) begin
            aw_hs <= 1'b0;
            w_hs  <= 1'b0;
          end
          // Timeout counter: any acceptance (progress) resets it
          if (w_pending && !progress) begin
            if (wcnt == (TIMEOUT_CYCLES-1)) begin
              wstate         <= W_ACK;
              wcnt           <= '0;
              wr_timeout_evt <= 1'b1;
            end
            else
              wcnt <= wcnt + 1'b1;
          end
          else
            wcnt <= '0;
        end

        W_ACK:  wstate <= W_RESP;  // AWREADY/WREADY asserted (comb) this cycle

        W_RESP: if (s_axi_bready) begin
          wstate <= W_PASS;
          aw_hs  <= 1'b0;
          w_hs   <= 1'b0;
        end

        default: wstate <= W_PASS;
      endcase
    end
  end

  // Write pass-through / isolation toward the slave
  assign m_axi_awaddr  = s_axi_awaddr;
  assign m_axi_awprot  = s_axi_awprot;
  assign m_axi_awvalid = s_axi_awvalid && (wstate == W_PASS) && !aw_hs;
  assign m_axi_wdata   = s_axi_wdata;
  assign m_axi_wstrb   = s_axi_wstrb;
  assign m_axi_wvalid  = s_axi_wvalid  && (wstate == W_PASS) && !w_hs;
  assign m_axi_bready  = s_axi_bready  && (wstate == W_PASS);

  // AWREADY / WREADY toward the master
  always_comb begin
    case (wstate)
      W_PASS: begin
        s_axi_awready = m_axi_awready;
        s_axi_wready  = m_axi_wready;
      end
      W_ACK: begin
        // Close only the channel(s) the slave never accepted
        s_axi_awready = ~aw_hs;
        s_axi_wready  = ~w_hs;
      end
      default: begin
        s_axi_awready = 1'b0;
        s_axi_wready  = 1'b0;
      end
    endcase
  end

  // B channel toward the master (pass-through, or injected SLVERR)
  always_comb begin
    if (wstate == W_RESP) begin
      s_axi_bvalid = 1'b1;
      s_axi_bresp  = RESP_SLVERR;
    end
    else begin
      s_axi_bvalid = m_axi_bvalid && (wstate == W_PASS);
      s_axi_bresp  = m_axi_bresp;
    end
  end

endmodule
