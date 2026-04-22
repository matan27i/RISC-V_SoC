// L1 Data Cache - RTL
// Direct-mapped, BRAM-based, 1 KB (64 lines × 4 words)
// Write-back policy with dirty bits
// No lookahead (address known only in MEM stage)

import risc_pkg::*;

module l1_dcache #(
  parameter  NUM_LINES   = 64,
  parameter  LINE_WORDS  = 4,
  localparam INDEX_W     = $clog2(NUM_LINES),
  localparam WORD_OFF_W  = $clog2(LINE_WORDS),
  localparam BYTE_OFF_W  = 2,
  localparam OFFSET_W    = WORD_OFF_W + BYTE_OFF_W,
  localparam TAG_W       = XLEN - INDEX_W - OFFSET_W,
  localparam DATA_DEPTH  = NUM_LINES * LINE_WORDS
)
(
  input  logic             clk,
  input  logic             reset_n,

  // CPU-side (MEM stage)
  input  logic             cpu_req,
  input  logic             cpu_wr_en,       // 1=store, 0=load
  input  mem_size_t        cpu_data_size,   // BYTE / HALF_WORD / WORD
  input  logic [XLEN-1:0]  cpu_addr,        // = ex_mem_alu_res
  input  logic [31:0]      cpu_wr_data,
  input  logic             cpu_zero_extend,
  output logic [31:0]      cpu_rd_data,
  output logic             cpu_ready,

  // Main memory interface
  output logic             mem_req,
  output logic             mem_wr_en,
  output logic [XLEN-1:0]  mem_addr,
  output logic [31:0]      mem_wr_data,
  input  logic [31:0]      mem_data,
  input  logic             mem_data_valid,
  input  logic             mem_ready
);

  
  // Storage (BRAM-inferred)
  (* ram_style = "distributed" *)
  logic [TAG_W-1:0] tag_array  [NUM_LINES];

  (* ram_style = "distributed" *)
  logic [31:0]      data_array [DATA_DEPTH];

  logic [NUM_LINES-1:0] valid_r;
  logic [NUM_LINES-1:0] dirty_r;

  
  // Address Decomposition
  logic [INDEX_W-1:0]    req_index;
  logic [WORD_OFF_W-1:0] req_word_off;
  logic [BYTE_OFF_W-1:0] req_byte_off;
  logic [TAG_W-1:0]      req_tag;

  assign req_byte_off = cpu_addr[BYTE_OFF_W-1:0];
  assign req_word_off = cpu_addr[OFFSET_W-1:BYTE_OFF_W];
  assign req_index    = cpu_addr[OFFSET_W +: INDEX_W];
  assign req_tag      = cpu_addr[OFFSET_W + INDEX_W +: TAG_W];


  // Latched request (held during multi-cycle access)
  logic [INDEX_W-1:0]    lat_index;
  logic [WORD_OFF_W-1:0] lat_word_off;
  logic [BYTE_OFF_W-1:0] lat_byte_off;
  logic [TAG_W-1:0]      lat_tag;
  logic                  lat_wr_en;
  mem_size_t             lat_size;
  logic [31:0]           lat_wr_data;
  logic                  lat_zero_ext;

  
  // Registered BRAM outputs (1-cycle latency)
  logic [TAG_W-1:0] rd_tag_reg;
  logic [31:0]      rd_word_reg;
  logic             valid_reg;
  logic             dirty_reg;

  logic [$clog2(DATA_DEPTH)-1:0] data_rd_addr;
  assign data_rd_addr = {req_index, req_word_off};

  always_ff @(posedge clk) begin
    rd_tag_reg  <= tag_array[req_index];
    rd_word_reg <= data_array[data_rd_addr];
    valid_reg   <= valid_r[req_index];
    dirty_reg   <= dirty_r[req_index];
  end

  
  // Hit detection (uses latched tag after first cycle)
  logic hit;
  assign hit = valid_reg && (rd_tag_reg == lat_tag);

  
  // State Machine
  cache_state_t state, next_state;

  // Burst counters
  logic [WORD_OFF_W-1:0] fill_cnt;
  logic [WORD_OFF_W-1:0] wb_cnt;

  // Evicted line info (for writeback address)
  logic [TAG_W-1:0]      evict_tag;

  always_comb begin
    next_state = state;
    case (state)
       CACHE_IDLE:
        if (cpu_req) next_state = CACHE_COMPARE;

       CACHE_COMPARE: begin
        if (hit) begin
          // Stay; transition to IDLE only when no new request
          if (!cpu_req) next_state = CACHE_IDLE;
        end

        else begin
          // Miss
          if (valid_reg && dirty_reg) next_state = CACHE_WRITE_BACK;
          else next_state = CACHE_ALLOCATE;
        end
       end

       CACHE_WRITE_BACK:
        if (mem_ready) next_state = CACHE_ALLOCATE;

       CACHE_ALLOCATE:
        if (mem_ready) next_state = CACHE_COMPARE;

       default: next_state = CACHE_IDLE;
    endcase
  end

  
  // Byte-enable merge helper for store hits
  
  function automatic logic [31:0] merge_store (
    input logic [31:0]      orig,
    input logic [31:0]      wr,
    input mem_size_t        sz,
    input logic [BYTE_OFF_W-1:0] bo
  );

    logic [31:0] result;
    begin
      result = orig;
      case (sz)
        BYTE: begin
          case (bo)
            2'd0: result[7:0]   = wr[7:0];
            2'd1: result[15:8]  = wr[7:0];
            2'd2: result[23:16] = wr[7:0];
            2'd3: result[31:24] = wr[7:0];
            default: ;
          endcase
        end

        HALF_WORD: begin
          if (bo[1] == 1'b0) result[15:0]  = wr[15:0];

          else result[31:16] = wr[15:0];
        end

        WORD: result = wr;

        default: ;

      endcase
      return result;
    end
  endfunction


  // Load extraction helper for load hits
  function automatic logic [31:0] extract_load (
    input logic [31:0]      word,
    input mem_size_t        sz,
    input logic [BYTE_OFF_W-1:0] bo,
    input logic             zero_ext
  );
    logic [7:0]  byte_val;
    logic [15:0] half_val;
    logic [31:0] result;
    begin
      case (sz)
        BYTE: begin
          case (bo)
            2'd0: byte_val = word[7:0];
            2'd1: byte_val = word[15:8];
            2'd2: byte_val = word[23:16];
            2'd3: byte_val = word[31:24];
            default: byte_val = word[7:0];
          endcase
          result = zero_ext ? {24'b0, byte_val} : {{24{byte_val[7]}}, byte_val};
        end
        HALF_WORD: begin
          half_val = (bo[1] == 1'b0) ? word[15:0] : word[31:16];
          result = zero_ext ? {16'b0, half_val} : {{16{half_val[15]}}, half_val};
        end
        WORD:    result = word;
        default: result = 32'd0;
      endcase
      return result;
    end
  endfunction


  // Sequential state + side effects
  // FSM + Control Flops (With Asynchronous Reset)
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      state        <= CACHE_IDLE;
      valid_r      <= '0;
      dirty_r      <= '0;
      fill_cnt     <= '0;
      wb_cnt       <= '0;
      lat_index    <= '0;
      lat_word_off <= '0;
      lat_byte_off <= '0;
      lat_tag      <= '0;
      lat_wr_en    <= 1'b0;
      lat_size     <= WORD;
      lat_wr_data  <= '0;
      lat_zero_ext <= 1'b0;
      evict_tag    <= '0;
    end
    else begin
      state <= next_state;
      
      // Latch request when entering COMPARE
      if (state == CACHE_IDLE && cpu_req) begin
        lat_index    <= req_index;
        lat_word_off <= req_word_off;
        lat_byte_off <= req_byte_off;
        lat_tag      <= req_tag;
        lat_wr_en    <= cpu_wr_en;
        lat_size     <= cpu_data_size;
        lat_wr_data  <= cpu_wr_data;
        lat_zero_ext <= cpu_zero_extend;
      end

      // On transition to WRITE_BACK: snapshot evicted tag
      if (state == CACHE_COMPARE && !hit && valid_reg && dirty_reg) begin
        evict_tag <= rd_tag_reg;
        wb_cnt    <= '0;
      end

      // Writeback burst counter
      if (state == CACHE_WRITE_BACK) begin
        wb_cnt <= wb_cnt + 1;
        if (mem_ready) begin
          dirty_r[lat_index] <= 1'b0;
          wb_cnt             <= '0;
        end
      end

      // Fill burst counter & Control updates
      if (state == CACHE_ALLOCATE) begin
        if (mem_data_valid) fill_cnt <= fill_cnt + 1;

        if (mem_ready) begin
          // Mark line valid and clean after fill completes
          valid_r[lat_index]   <= 1'b1;
          dirty_r[lat_index]   <= 1'b0;
          fill_cnt             <= '0;
        end
      end

      // Store hit: mark dirty
      if (state == CACHE_COMPARE && hit && lat_wr_en) begin
        dirty_r[lat_index] <= 1'b1;
      end
    end
  end


  // Memory Arrays Write Block (Synchronous ONLY - No Reset)
  always_ff @(posedge clk) begin
    // Tag array write during ALLOCATE (separate always_ff for clean inference)
    if (state == CACHE_ALLOCATE && mem_ready) begin
        tag_array[lat_index] <= lat_tag;
    end
  end
  



  // BRAM writes (store hit merge + fill write)
  always_ff @(posedge clk) begin
    // Fill write
    if (state == CACHE_ALLOCATE && mem_data_valid) begin
      data_array[{lat_index, fill_cnt}] <= mem_data;
    end

    // Store hit: read-modify-write
    else if (state == CACHE_COMPARE && hit && lat_wr_en) begin
      data_array[{lat_index, lat_word_off}] <= merge_store(rd_word_reg, lat_wr_data, lat_size, lat_byte_off);
    end
  end

  
  // Memory Interface
  assign mem_req = (state == CACHE_ALLOCATE) || (state == CACHE_WRITE_BACK);
  assign mem_wr_en = (state == CACHE_WRITE_BACK);
  assign mem_addr = (state == CACHE_WRITE_BACK) ? {evict_tag, lat_index, {OFFSET_W{1'b0}}} : {lat_tag, lat_index, {OFFSET_W{1'b0}}};

 // Read-ahead counter for the write-back burst
 /*logic [WORD_OFF_W-1:0] wb_rd_cnt;

  always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n) wb_rd_cnt <= '0;

  else 
  if (state == CACHE_COMPARE && !hit && valid_reg && dirty_reg) wb_rd_cnt <= '0; // about to enter WRITE_BACK

   else 
   if (state == CACHE_WRITE_BACK) wb_rd_cnt <= wb_rd_cnt + 1'b1;
 end*/

 // Synchronous BRAM read — inferred cleanly now
 logic [31:0] wb_rd_word;
 always_ff @(posedge clk) begin
   wb_rd_word <= data_array[{lat_index, 2'b00} + wb_cnt]; // Read the current word being written back for write-back merging
 end
 assign mem_wr_data = wb_rd_word;
  

  
  // CPU Outputs
  assign cpu_ready   = (state == CACHE_COMPARE) && hit;
  assign cpu_rd_data = extract_load(rd_word_reg, lat_size, lat_byte_off, lat_zero_ext);


  // Explicit sink for byte-offset bits to prevent warnings (not used since we handle unaligned accesses in the merge/extract functions)
  wire _unused_addr_bits;
  assign _unused_addr_bits = &{1'b0, cpu_addr[1:0]};

endmodule
