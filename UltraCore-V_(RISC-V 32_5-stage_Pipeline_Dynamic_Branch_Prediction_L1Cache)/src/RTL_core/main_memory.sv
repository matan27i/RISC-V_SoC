// Main Memory Controller - RTL
// Unified backing store with dual-port arbiter for I-Cache and D-Cache
// Configurable latency, burst read/write interface

import risc_pkg::*;

module main_memory #(
  parameter ADDR_WIDTH = 14,       // 16 KB backing store (byte-addressable at IF)
  parameter DATA_WIDTH = 32,       // Word-wide storage (BRAM-friendly)
  parameter LATENCY    = 10,       // Cycles before first burst word
  parameter BURST_LEN  = 4         // Words per burst (= LINE_WORDS)
)(
  input  logic        clk,
  input  logic        reset_n,

  // I-Cache port (read-only)
  input  logic        icache_req,
  input  logic [31:0] icache_addr,       // Line-aligned address
  output logic [31:0] icache_data,       // Burst word output
  output logic        icache_data_valid, // Pulses high per word
  output logic        icache_ready,      // Pulses high when burst complete

  // D-Cache port (read/write)
  input  logic        dcache_req,
  input  logic        dcache_wr_en,      // 1=writeback, 0=fill
  input  logic [31:0] dcache_addr,       // Line-aligned address
  input  logic [31:0] dcache_wr_data,    // Writeback word
  output logic [31:0] dcache_data,       // Burst word output
  output logic        dcache_data_valid, // Pulses high per word (reads only)
  output logic        dcache_ready       // Pulses high when burst complete
);

  // ------------------------------------------------------------------
  // Storage: Unified backing memory (word-addressable, BRAM inferable)
  // 2^ADDR_WIDTH bytes total => 2^(ADDR_WIDTH-2) 32-bit words
  // Single 32-bit read/write per cycle maps to a single BRAM port,
  // avoiding the 4-way byte-access pattern that defeats BRAM inference.
  // ------------------------------------------------------------------
  localparam int WORD_ADDR_WIDTH = ADDR_WIDTH - 2;
  localparam int NUM_WORDS       = 2**WORD_ADDR_WIDTH;

  logic [DATA_WIDTH-1:0] mem [0:NUM_WORDS-1];

  initial begin
    $readmemh("machine_code.mem", mem);
  end

  // ------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------
  typedef enum logic [2:0] {
    MEM_IDLE             = 3'd0,
    MEM_ICACHE_LATENCY   = 3'd1,
    MEM_ICACHE_BURST     = 3'd2,
    MEM_DCACHE_LATENCY   = 3'd3,
    MEM_DCACHE_BURST_RD  = 3'd4,
    MEM_DCACHE_BURST_WR  = 3'd5
  } mem_state_t;

  mem_state_t state, next_state;

  // Latched request info
  logic [31:0]                  latched_addr;
  logic [$clog2(LATENCY+1)-1:0] lat_cnt;
  logic [$clog2(BURST_LEN)-1:0] burst_cnt;

  // Current word address (line-aligned + burst offset), in 32-bit word units
  logic [WORD_ADDR_WIDTH-1:0] cur_word_addr;
  assign cur_word_addr = latched_addr[ADDR_WIDTH-1:2] + burst_cnt;

  // ------------------------------------------------------------------
  // Arbitration: D-Cache has priority
  // ------------------------------------------------------------------
  always_comb begin
    next_state = state;
    case (state)
      MEM_IDLE: begin
        if (dcache_req) begin
          if (dcache_wr_en) next_state = MEM_DCACHE_BURST_WR;
          else              next_state = MEM_DCACHE_LATENCY;
        end
        else if (icache_req) begin
          next_state = MEM_ICACHE_LATENCY;
        end
      end

      MEM_ICACHE_LATENCY:
        if (lat_cnt == LATENCY-1) next_state = MEM_ICACHE_BURST;

      MEM_ICACHE_BURST:
        if (burst_cnt == BURST_LEN-1) next_state = MEM_IDLE;

      MEM_DCACHE_LATENCY:
        if (lat_cnt == LATENCY-1) next_state = MEM_DCACHE_BURST_RD;

      MEM_DCACHE_BURST_RD:
        if (burst_cnt == BURST_LEN-1) next_state = MEM_IDLE;

      MEM_DCACHE_BURST_WR:
        if (burst_cnt == BURST_LEN-1) next_state = MEM_IDLE;

      default: next_state = MEM_IDLE;
    endcase
  end

  // ------------------------------------------------------------------
  // Sequential state update
  // ------------------------------------------------------------------
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      state        <= MEM_IDLE;
      lat_cnt      <= '0;
      burst_cnt    <= '0;
      latched_addr <= '0;
    end
    else begin
      state <= next_state;

      case (state)
        MEM_IDLE: begin
          lat_cnt   <= '0;
          burst_cnt <= '0;
          if (dcache_req)      latched_addr <= dcache_addr;
          else if (icache_req) latched_addr <= icache_addr;
        end

        MEM_ICACHE_LATENCY, MEM_DCACHE_LATENCY:
          lat_cnt <= lat_cnt + 1;

        MEM_ICACHE_BURST, MEM_DCACHE_BURST_RD, MEM_DCACHE_BURST_WR:
          burst_cnt <= burst_cnt + 1;

        default: ;
      endcase

      if (next_state == MEM_IDLE) begin
        lat_cnt   <= '0;
        burst_cnt <= '0;
      end
    end
  end

  // ------------------------------------------------------------------
  // Write Logic (D-Cache writeback) - single 32-bit word per cycle
  // ------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (state == MEM_DCACHE_BURST_WR) begin
      mem[cur_word_addr] <= dcache_wr_data;
    end
  end

  // ------------------------------------------------------------------
  // Read Logic - single 32-bit word read, BRAM-friendly
  // ------------------------------------------------------------------
  logic [31:0] read_word;
  assign read_word = mem[cur_word_addr];

  // ------------------------------------------------------------------
  // Output Drivers
  // ------------------------------------------------------------------
  always_comb begin
    icache_data       = 32'd0;
    icache_data_valid = 1'b0;
    icache_ready      = 1'b0;
    dcache_data       = 32'd0;
    dcache_data_valid = 1'b0;
    dcache_ready      = 1'b0;

    case (state)
      MEM_ICACHE_BURST: begin
        icache_data       = read_word;
        icache_data_valid = 1'b1;
        icache_ready      = (burst_cnt == BURST_LEN-1);
      end

      MEM_DCACHE_BURST_RD: begin
        dcache_data       = read_word;
        dcache_data_valid = 1'b1;
        dcache_ready      = (burst_cnt == BURST_LEN-1);
      end

      MEM_DCACHE_BURST_WR: begin
        dcache_ready = (burst_cnt == BURST_LEN-1);
      end

      default: ;
    endcase
  end

endmodule
