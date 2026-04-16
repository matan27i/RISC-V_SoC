// L1 Instruction Cache - RTL
// Direct-mapped, BRAM-based, 1 KB (64 lines × 4 words)
// Uses lookahead addressing to absorb BRAM 1-cycle read latency

import risc_pkg::*;

module l1_icache #(
  parameter  NUM_LINES   = 64,
  parameter  LINE_WORDS  = 4,
  localparam INDEX_W     = $clog2(NUM_LINES),
  localparam WORD_OFF_W  = $clog2(LINE_WORDS),
  localparam OFFSET_W    = WORD_OFF_W + 2,                  // byte+word offset
  localparam TAG_W       = XLEN - INDEX_W - OFFSET_W,       // 32 - 6 - 4 = 22
  localparam DATA_DEPTH  = NUM_LINES * LINE_WORDS
)(
  input  logic             clk,
  input  logic             reset_n,

  // CPU-side (IF stage) — receives lookahead address
  input  logic             cpu_req,
  input  logic [XLEN-1:0]  cpu_addr,       // = btb_lookup_addr (1-cycle lookahead)
  output logic [31:0]      cpu_data,       // Fetched instruction
  output logic             cpu_ready,      // 1 = hit, 0 = stall

  // Main memory interface
  output logic             mem_req,
  output logic [XLEN-1:0]  mem_addr,       // Line-aligned address
  input  logic [31:0]      mem_data,
  input  logic             mem_data_valid,
  input  logic             mem_ready
);

  // ------------------------------------------------------------------
  // Storage (BRAM-inferred)
  // ------------------------------------------------------------------
  (* ram_style = "block" *)
  logic [TAG_W-1:0] tag_array  [NUM_LINES];

  (* ram_style = "block" *)
  logic [31:0]      data_array [DATA_DEPTH];

  // Valid bits in FFs (resettable)
  logic [NUM_LINES-1:0] valid_r;

  // ------------------------------------------------------------------
  // Address Decomposition (on lookahead address)
  // ------------------------------------------------------------------
  logic [INDEX_W-1:0]    la_index;
  logic [WORD_OFF_W-1:0] la_word_off;
  logic [TAG_W-1:0]      la_tag;

  assign la_word_off = cpu_addr[OFFSET_W-1:2];
  assign la_index    = cpu_addr[OFFSET_W +: INDEX_W];
  assign la_tag      = cpu_addr[OFFSET_W + INDEX_W +: TAG_W];

  // ------------------------------------------------------------------
  // Registered BRAM outputs (1-cycle latency)
  // ------------------------------------------------------------------
  logic [TAG_W-1:0]    rd_tag_reg;        // Registered stored tag
  logic [31:0]         rd_word_reg;       // Registered stored word
  logic [TAG_W-1:0]    la_tag_reg;        // Latched input tag
  logic [INDEX_W-1:0]  la_index_reg;      // Latched input index
  logic                valid_reg;         // Latched valid bit

  // Compute flat address into data_array
  logic [$clog2(DATA_DEPTH)-1:0] data_rd_addr;
  assign data_rd_addr = {la_index, la_word_off};

  always_ff @(posedge clk) begin
    rd_tag_reg   <= tag_array[la_index];
    rd_word_reg  <= data_array[data_rd_addr];
    la_tag_reg   <= la_tag;
    la_index_reg <= la_index;
    valid_reg    <= valid_r[la_index];
  end

  // ------------------------------------------------------------------
  // Hit detection
  // ------------------------------------------------------------------
  logic hit;
  assign hit = valid_reg && (rd_tag_reg == la_tag_reg);

  // ------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------
  cache_state_t state, next_state;

  // Miss tracking
  logic [TAG_W-1:0]              miss_tag;
  logic [INDEX_W-1:0]            miss_index;
  logic [WORD_OFF_W-1:0]         fill_cnt;
  logic                          fill_active;

  always_comb begin
    next_state = state;
    case (state)
      CACHE_IDLE:
        if (cpu_req) next_state = CACHE_COMPARE;

      CACHE_COMPARE: begin
        if (!cpu_req)
          next_state = CACHE_IDLE;
        else if (!hit)
          next_state = CACHE_ALLOCATE;
      end

      CACHE_ALLOCATE:
        if (mem_ready) next_state = CACHE_COMPARE;

      default: next_state = CACHE_IDLE;
    endcase
  end

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      state       <= CACHE_IDLE;
      valid_r     <= '0;
      miss_tag    <= '0;
      miss_index  <= '0;
      fill_cnt    <= '0;
      fill_active <= 1'b0;
    end
    else begin
      state <= next_state;

      // Latch miss info on transition to ALLOCATE
      if (state == CACHE_COMPARE && !hit && cpu_req) begin
        miss_tag    <= la_tag_reg;
        miss_index  <= la_index_reg;
        fill_cnt    <= '0;
        fill_active <= 1'b1;
      end

      // Fill counter advances per valid word from memory
      if (state == CACHE_ALLOCATE && mem_data_valid) begin
        fill_cnt <= fill_cnt + 1;
      end

      // On fill completion: update tag and set valid
      if (state == CACHE_ALLOCATE && mem_ready) begin
        tag_array[miss_index] <= miss_tag;
        valid_r[miss_index]   <= 1'b1;
        fill_active           <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------
  // BRAM Write during ALLOCATE (separate always_ff for clean inference)
  // ------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (state == CACHE_ALLOCATE && mem_data_valid) begin
      data_array[{miss_index, fill_cnt}] <= mem_data;
    end
  end

  // ------------------------------------------------------------------
  // Memory Interface
  // ------------------------------------------------------------------
  assign mem_req  = (state == CACHE_ALLOCATE) && fill_active;
  assign mem_addr = {miss_tag, miss_index, {OFFSET_W{1'b0}}};

  // ------------------------------------------------------------------
  // CPU Outputs
  // ------------------------------------------------------------------
  assign cpu_data  = rd_word_reg;
  assign cpu_ready = (state == CACHE_COMPARE) && hit && cpu_req;

endmodule
