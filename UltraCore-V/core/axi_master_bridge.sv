// AXI4-Lite Master Bridge - RTL
// Translates the core's uncached MMIO load/store requests (the region
// the L1 D-cache does not claim: peripherals at and above 0x4000_0000
// plus the boot-ROM data window) into single-beat AXI4-Lite
// transactions.
//
// CPU-side protocol (native, identical in spirit to the D-cache CPU
// port): the MEM stage asserts mmio_req with stable address/data/size
// fields and the pipeline stalls on (mmio_req & ~mmio_ready). Because
// the EX/MEM registers are frozen for the whole stall, every request
// field is guaranteed stable for the duration of the transaction, so
// the bridge can drive the AXI payload signals from them directly.
// mmio_ready is a registered single-cycle done pulse; read data is
// captured into a register before the pulse, so the pipeline latches a
// stable value on the release edge.
//
// Transaction rules implemented:
//   - Writes issue AW and W together; the two handshakes may complete
//     in either order or simultaneously (tracked independently). The
//     transaction finishes when B returns: MMIO stores are strongly
//     ordered - the next instruction cannot proceed before the
//     peripheral has accepted the write. This is intentional for
//     device registers.
//   - Reads issue AR, then wait for R.
//   - Sub-word accesses (lb/lbu/lh/lhu/sb/sh) are supported: write
//     data is lane-replicated with the matching WSTRB, read data is
//     lane-extracted and sign/zero extended, mirroring the D-cache
//     load path semantics.
//   - BRESP/RRESP are accepted but not acted upon: the RV32I core has
//     no bus-fault trap yet. Hook point marked below.
//
// Fmax: every AXI output is registered; the only combinational outputs
// are mmio_rd_data (LUT extract from a local register) and the stall
// term derived from registered mmio_ready.

import risc_pkg::*;

module axi_master_bridge (
  input  logic             clk,
  input  logic             resetn,

  // CPU MMIO port (request fields stable while stalled)
  input  logic             mmio_req,
  input  logic             mmio_wr_en,
  input  logic [XLEN-1:0]  mmio_addr,
  input  logic [31:0]      mmio_wr_data,
  input  mem_size_t        mmio_size,
  input  logic             mmio_zero_extend,
  output logic [31:0]      mmio_rd_data,
  output logic             mmio_ready,        // Done level, held until accepted
  input  logic             mmio_accept,       // Pipeline can consume this cycle

  // AXI4-Lite master
  output logic [31:0]      m_axi_awaddr,
  output logic [2:0]       m_axi_awprot,
  output logic             m_axi_awvalid,
  input  logic             m_axi_awready,
  output logic [31:0]      m_axi_wdata,
  output logic [3:0]       m_axi_wstrb,
  output logic             m_axi_wvalid,
  input  logic             m_axi_wready,
  input  logic [1:0]       m_axi_bresp,
  input  logic             m_axi_bvalid,
  output logic             m_axi_bready,
  output logic [31:0]      m_axi_araddr,
  output logic [2:0]       m_axi_arprot,
  output logic             m_axi_arvalid,
  input  logic             m_axi_arready,
  input  logic [31:0]      m_axi_rdata,
  input  logic [1:0]       m_axi_rresp,
  input  logic             m_axi_rvalid,
  output logic             m_axi_rready
);

  // Data accesses from machine mode; constant per AXI4-Lite practice
  assign m_axi_awprot = 3'b000;
  assign m_axi_arprot = 3'b000;

  
  // Store lane replication and strobe generation
  //
  // AXI4-Lite carries a full 32-bit beat; the byte lanes selected by
  // WSTRB define the written bytes. Replicating the narrow data into
  // every lane makes the strobe decode independent of the data mux.
  
  logic [31:0] wdata_lanes;
  logic [3:0]  wstrb_dec;

  always_comb begin
    unique case (mmio_size)
      BYTE:      wdata_lanes = {4{mmio_wr_data[7:0]}};
      HALF_WORD: wdata_lanes = {2{mmio_wr_data[15:0]}};
      default:   wdata_lanes = mmio_wr_data;            // WORD
    endcase
  end

  always_comb begin
    unique case (mmio_size)
      BYTE:      wstrb_dec = 4'b0001 << mmio_addr[1:0];
      HALF_WORD: wstrb_dec = mmio_addr[1] ? 4'b1100 : 4'b0011;
      default:   wstrb_dec = 4'b1111;                   // WORD
    endcase
  end

  
  // Transaction FSM
  
  typedef enum logic [2:0] {
    MB_IDLE    = 3'd0,
    MB_WRITE   = 3'd1,   // AW/W handshakes in flight
    MB_WR_RESP = 3'd2,   // Waiting for B
    MB_READ    = 3'd3,   // AR handshake in flight
    MB_RD_RESP = 3'd4,   // Waiting for R
    MB_DONE    = 3'd5    // One-cycle ready pulse to the pipeline
  } mb_state_t;

  mb_state_t state;

  logic [31:0] rd_word;    // Captured R-channel data

  // Both write handshakes are complete (or completing this cycle)
  logic aw_settled, w_settled;
  assign aw_settled = ~m_axi_awvalid | m_axi_awready;
  assign w_settled  = ~m_axi_wvalid  | m_axi_wready;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      state         <= MB_IDLE;
      m_axi_awaddr  <= '0;
      m_axi_awvalid <= 1'b0;
      m_axi_wdata   <= '0;
      m_axi_wstrb   <= '0;
      m_axi_wvalid  <= 1'b0;
      m_axi_bready  <= 1'b0;
      m_axi_araddr  <= '0;
      m_axi_arvalid <= 1'b0;
      m_axi_rready  <= 1'b0;
      rd_word       <= '0;
    end
    else begin
      case (state)
        MB_IDLE: begin
          if (mmio_req) begin
            if (mmio_wr_en) begin
              m_axi_awaddr  <= {mmio_addr[31:2], 2'b00};
              m_axi_awvalid <= 1'b1;
              m_axi_wdata   <= wdata_lanes;
              m_axi_wstrb   <= wstrb_dec;
              m_axi_wvalid  <= 1'b1;
              state         <= MB_WRITE;
            end
            else begin
              m_axi_araddr  <= {mmio_addr[31:2], 2'b00};
              m_axi_arvalid <= 1'b1;
              state         <= MB_READ;
            end
          end
        end

        MB_WRITE: begin
          // Retire each channel independently as its ready arrives
          if (m_axi_awvalid && m_axi_awready) m_axi_awvalid <= 1'b0;
          if (m_axi_wvalid  && m_axi_wready)  m_axi_wvalid  <= 1'b0;
          if (aw_settled && w_settled) begin
            m_axi_bready <= 1'b1;
            state        <= MB_WR_RESP;
          end
        end

        MB_WR_RESP: begin
          if (m_axi_bvalid) begin
            // BRESP intentionally ignored (no bus-fault trap in RV32I
            // core yet); a future trap unit hooks in here.
            m_axi_bready <= 1'b0;
            state        <= MB_DONE;
          end
        end

        MB_READ: begin
          if (m_axi_arready) begin
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b1;
            state         <= MB_RD_RESP;
          end
        end

        MB_RD_RESP: begin
          if (m_axi_rvalid) begin
            rd_word      <= m_axi_rdata;   // RRESP ignored, see above
            m_axi_rready <= 1'b0;
            state        <= MB_DONE;
          end
        end

        MB_DONE: begin
          // mmio_ready is held high until the pipeline can actually
          // consume the completion (an I-/D-cache stall may be holding
          // MEM/WB frozen). Leaving DONE early would re-launch the
          // same transaction and repeat device side effects; waiting
          // costs nothing because the pipeline is stalled anyway.
          if (mmio_accept)
            state <= MB_IDLE;
        end

        default: state <= MB_IDLE;
      endcase
    end
  end

  assign mmio_ready = (state == MB_DONE);

  
  // Load lane extraction with sign/zero extension (mirrors the
  // D-cache extract_load semantics so lb/lh/lw behave identically on
  // peripherals and on RAM). Request fields are still frozen during
  // MB_DONE, so this decode is stable when the pipeline samples it.
  
  logic [7:0]  rd_byte;
  logic [15:0] rd_half;

  always_comb begin
    unique case (mmio_addr[1:0])
      2'd0:    rd_byte = rd_word[7:0];
      2'd1:    rd_byte = rd_word[15:8];
      2'd2:    rd_byte = rd_word[23:16];
      default: rd_byte = rd_word[31:24];
    endcase
  end

  assign rd_half = mmio_addr[1] ? rd_word[31:16] : rd_word[15:0];

  always_comb begin
    unique case (mmio_size)
      BYTE:      mmio_rd_data = mmio_zero_extend ? {24'b0, rd_byte}
                                                 : {{24{rd_byte[7]}}, rd_byte};
      HALF_WORD: mmio_rd_data = mmio_zero_extend ? {16'b0, rd_half}
                                                 : {{16{rd_half[15]}}, rd_half};
      default:   mmio_rd_data = rd_word;               // WORD
    endcase
  end

  // Sink for intentionally unused response codes
  wire _unused_resp;
  assign _unused_resp = &{1'b0, m_axi_bresp, m_axi_rresp};

endmodule
