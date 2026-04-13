// Branch Target Buffer (BTB) — FPGA BRAM Implementation

import risc_pkg::*;

module btb #(parameter NUM_ENTRIES = 64, localparam INDEX_W = $clog2(NUM_ENTRIES), localparam TAG_W   = XLEN - 2 - INDEX_W,
localparam DATA_W  = TAG_W + XLEN + 2 )
(
  input  logic             clk,
  input  logic             reset_n,
  input  logic [XLEN-1:0]  pc,              // Current Program Counter
  output logic              predict_taken,   // BTB predicts "Taken"
  output logic [XLEN-1:0]  predict_target,  // Predicted target address

  // Write/Update Port (Execute Stage)
  input  logic              update_en,       // Update enable (branch/jump resolved)
  input  logic [XLEN-1:0]  update_pc,       // PC of the resolved branch
  input  logic [XLEN-1:0]  actual_target,   // Actual computed target address
  input  logic              was_taken        // Actual branch outcome
);

  
  //  Storage

  // True dual-port BRAM — Xilinx: ram_style "block"
  (* ram_style = "block" *)
  reg [DATA_W-1:0] btb_bram [NUM_ENTRIES];

  // Valid bits in flip-flops (synchronous reset, too small for BRAM)
  reg [NUM_ENTRIES-1:0] valid_r;

  //  Port A — Fetch Read (synchronous, 1-cycle latency)
  logic [INDEX_W-1:0] rd_index;
  logic [TAG_W-1:0]   rd_tag;

  reg [DATA_W-1:0]    rd_data_reg;
  reg [TAG_W-1:0]     rd_tag_reg;        // Input tag latched for comparison
  reg                  rd_valid_reg;      // Valid bit aligned with BRAM output

  // Unpacked fields from registered BRAM data
  logic [TAG_W-1:0]   rd_stored_tag;
  logic [XLEN-1:0]    rd_stored_target;
  logic [1:0]         rd_stored_state;
  logic               rd_match;

  // Decode PC fields
  assign rd_index = pc[2 +: INDEX_W];
  assign rd_tag   = pc[2 + INDEX_W +: TAG_W];

  // BRAM Port A — synchronous read (no reset → infers BRAM read port)
  always_ff @(posedge clk) begin
    rd_data_reg  <= btb_bram[rd_index];
    rd_tag_reg   <= rd_tag;
    rd_valid_reg <= valid_r[rd_index];
  end

  // Unpack registered BRAM data: {tag, target_addr, state_bits}
  assign rd_stored_tag    = rd_data_reg[DATA_W-1     -: TAG_W];
  assign rd_stored_target = rd_data_reg[XLEN+1       -: XLEN];
  assign rd_stored_state  = rd_data_reg[1:0];

  // Tag comparison
  assign rd_match = (rd_stored_tag == rd_tag_reg) && rd_valid_reg;   //Match = (Stored_Tag == Latched_Tag) && Valid

  // Prediction outputs (valid 1 cycle after pc presented)
  assign predict_taken  = rd_match && rd_stored_state[1];    //   Predict_Taken = Match && State_Bits[1]
  assign predict_target = rd_stored_target;

  //  Port B — Update (2-stage pipelined read-modify-write)

  logic [INDEX_W-1:0] wr_index;
  logic [TAG_W-1:0]   wr_tag;

  // Stage 1 pipeline registers
  reg                  s1_valid;
  reg [INDEX_W-1:0]   s1_index;
  reg [TAG_W-1:0]     s1_tag;
  reg [XLEN-1:0]      s1_actual_target;
  reg                  s1_was_taken;
  reg [DATA_W-1:0]    s1_rd_data;         // BRAM read result for hit detection
  reg                  s1_entry_valid;     // Valid bit snapshot

  // Stage 1 unpacked fields (from BRAM read)
  logic [TAG_W-1:0]   s1_stored_tag;
  logic [1:0]         s1_stored_state;
  logic               s1_match;

  // Computed write-back values
  logic [1:0]         new_state;
  logic [DATA_W-1:0]  wr_data;

  // Port B address multiplexing
  logic [INDEX_W-1:0] portb_addr;
  logic               portb_we;

  // Decode update PC fields
  assign wr_index = update_pc[2 +: INDEX_W];
  assign wr_tag   = update_pc[2 + INDEX_W +: TAG_W];

  // Port B control: write in stage 2, read in stage 1
  assign portb_addr = s1_valid ? s1_index : wr_index;
  assign portb_we   = s1_valid;

  // ---- Stage 1: Register update inputs ----
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      s1_valid <= 1'b0;
    end 
    
    else begin
      s1_valid         <= update_en && !s1_valid;
      s1_index         <= wr_index;
      s1_tag           <= wr_tag;
      s1_actual_target <= actual_target;
      s1_was_taken     <= was_taken;
      s1_entry_valid   <= valid_r[wr_index];
    end
  end

  // ---- BRAM Port B: true dual-port read/write ----
  always_ff @(posedge clk) begin
    if (portb_we)
      btb_bram[portb_addr] <= wr_data;    // Stage 2: write-back
    s1_rd_data <= btb_bram[portb_addr];   // Stage 1: read (read-first mode)
  end

  // ---- Stage 2: Hit detection + state computation ----

  // Unpack stage 1 BRAM read data
  assign s1_stored_tag   = s1_rd_data[DATA_W-1 -: TAG_W];
  assign s1_stored_state = s1_rd_data[1:0];

  // Hit detection (tag comparison on registered data)
  assign s1_match = (s1_stored_tag == s1_tag) && s1_entry_valid;

  // Saturating counter update
  always_comb begin
    if (s1_match) begin
      if (s1_was_taken)
        new_state = (s1_stored_state == 2'b11) ? 2'b11 : s1_stored_state + 2'b01; // HIT: update existing counter
      else
        new_state = (s1_stored_state == 2'b00) ? 2'b00 : s1_stored_state - 2'b01;
    end 
    
    else begin 
      new_state = s1_was_taken ? 2'b10 : 2'b01;  // MISS: initialize to weak state based on first outcome
    end
  end

  // Pack write-back data: {tag, target_addr, state_bits}
  assign wr_data = {s1_tag, s1_actual_target, new_state};

  // ---- Valid register update (with synchronous reset) ----
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)
      valid_r <= '0;
    else if (s1_valid)
      valid_r[s1_index] <= 1'b1;
  end

endmodule
