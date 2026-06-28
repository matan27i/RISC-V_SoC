// Branch Target Buffer (BTB 64 entries) - RTL

import risc_pkg::*;

module btb # (localparam NUM_ENTRIES = 64, localparam INDEX_W = 6, localparam TAG_W   = XLEN - 2 - INDEX_W)
(
  input  logic             clk,
  input  logic             reset_n,

  // Read Port (Fetch Stage)
  input  logic [XLEN-1:0]   pc,              // Current Program Counter
  output logic              predict_taken,   // BTB predicts "Taken"
  output logic [XLEN-1:0]   predict_target,  // Predicted target address

  // Write/Update Port (Execute Stage)
  input  logic             update_en,       // Update enable (branch/jump resolved)
  input  logic [XLEN-1:0]  update_pc,       // PC of the resolved branch
  input  logic [XLEN-1:0]  actual_target,   // Actual computed target address
  input  logic             was_taken        // Actual branch outcome
);
  


  //  Data Structure

  typedef struct packed {
    logic              valid;       // 
    logic [TAG_W-1:0]  tag;         // High-order PC bits for identification
    logic [XLEN-1:0]   target_addr; // Predicted branch/jump 
    logic [1:0]        state_bits;  // counter (FSM)
  } btb_entry_t;

  // BTB memory array
  btb_entry_t btb_mem [NUM_ENTRIES];

  //  Read-Side Signals

  logic [INDEX_W-1:0] rd_index;
  logic [TAG_W-1:0]   rd_tag;
  btb_entry_t         rd_entry;
  logic               rd_match;

  //  Write-Side Signals

  logic [INDEX_W-1:0] wr_index;
  logic [TAG_W-1:0]   wr_tag;
  btb_entry_t         wr_entry;
  logic               wr_match;

  //  Read Logic (Fetch Stage)

  // Extract index and tag from current PC
  assign rd_index = pc[2 +: INDEX_W];
  assign rd_tag   = pc[2 + INDEX_W +: TAG_W];

  // Read entry from memory array
  assign rd_entry = btb_mem[rd_index];

  // Tag comparison with valid check
  // Match = (Stored_Tag == Current_Tag) && Valid
  assign rd_match = (rd_entry.tag == rd_tag) && rd_entry.valid;

  // Prediction outputs
  // Predict_Taken = Match && State_Bits[1]  (MSB = 1 => "Taken")
  assign predict_taken  = rd_match && rd_entry.state_bits[1];
  assign predict_target = rd_entry.target_addr;

  //  Write/Update Logic (Execute Stage)
  //  Sequential update on clock edge

  // Extract index and tag from update PC
  assign wr_index = update_pc[2 +: INDEX_W];
  assign wr_tag   = update_pc[2 + INDEX_W +: TAG_W];

  // Read existing entry at the update index (for hit detection)
  assign wr_entry = btb_mem[wr_index];
  assign wr_match = (wr_entry.tag == wr_tag) && wr_entry.valid;

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      for (int i = 0; i < NUM_ENTRIES; i++) begin
        btb_mem[i].valid       <= 1'b0;
        btb_mem[i].tag         <= '0;
        btb_mem[i].target_addr <= '0;
        btb_mem[i].state_bits  <= 2'b00;
      end
    end 

    else if (update_en) begin
      if (wr_match) begin   // HIT- Entry exists for this branch
        btb_mem[wr_index].target_addr <= actual_target;     // Update target address
      
        if (was_taken) begin                                // Saturating counter update based on actual outcome
          if (wr_entry.state_bits != 2'b11)                 // Increment: saturate at 2'b11 (Strongly Taken)
            btb_mem[wr_index].state_bits <= wr_entry.state_bits + 2'b01;
        end 
        
        else begin
          if (wr_entry.state_bits != 2'b00)                // Decrement: saturate at 2'b00 (Strongly Not Taken)
            btb_mem[wr_index].state_bits <= wr_entry.state_bits - 2'b01;
        end
      end 

      else begin
        
        btb_mem[wr_index].valid       <= 1'b1; // ---- MISS: Allocate new entry ----
        btb_mem[wr_index].tag         <= wr_tag;
        btb_mem[wr_index].target_addr <= actual_target;
        
        btb_mem[wr_index].state_bits  <= was_taken ? 2'b10 : 2'b01; // Initialize to "Weakly Taken" (2'b10) if taken, "Weakly Not Taken" (2'b01) if not taken                           
      end
      
    end
  end

endmodule
