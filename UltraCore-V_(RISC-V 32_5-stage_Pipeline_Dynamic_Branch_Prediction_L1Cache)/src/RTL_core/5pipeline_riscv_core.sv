// Top-Level 5-Stage Pipelined RISC-V Core
import risc_pkg::*;

module \5pipeline_riscv_core #(parameter RESET_PC = 32'h0000)
(
  input  logic             clk,
  input  logic             reset_n,
  output logic [XLEN-1:0]  pc_out
);

  //  Internal Signals
  //  Hazard & Flush Control
  logic hazard_stall;       // Load-use hazard stall (from hazard_detection)
  logic icache_stall;       // I-Cache miss stall
  logic dcache_stall;       // D-Cache miss stall
  logic pipeline_stall;     // Composite: any source of pipeline stall
  logic ex_redirect;        // Branch/jump taken in EX stage
  logic effective_redirect; // Redirect gated by cache stalls

  assign pipeline_stall     = hazard_stall | icache_stall | dcache_stall;
  assign effective_redirect = ex_redirect & ~icache_stall & ~dcache_stall;

  
  //  IF Stage Signals
  logic [XLEN-1:0] pc, next_pc, next_seq_pc;
  logic [31:0]     instruction;
  logic            icache_ready;

  //  Cache ↔ Main Memory interface (I-Cache)
  logic             ic_mem_req;
  logic [XLEN-1:0]  ic_mem_addr;
  logic [31:0]      ic_mem_data;
  logic             ic_mem_data_valid;
  logic             ic_mem_ready;

  //  Cache ↔ Main Memory interface (D-Cache)
  logic             dc_mem_req;
  logic             dc_mem_wr_en;
  logic [XLEN-1:0]  dc_mem_addr;
  logic [31:0]      dc_mem_wr_data;
  logic [31:0]      dc_mem_data;
  logic             dc_mem_data_valid;
  logic             dc_mem_ready;

  //  D-Cache ready handshake
  logic             dcache_ready;

  
  //  ID Stage Signals (Decode + Control + Register File)
  logic [6:0]      opcode;
  logic [2:0]      funct3;
  logic [6:0]      funct7;
  logic [4:0]      rs1_addr, rs2_addr, rd_addr;
  logic [XLEN-1:0] rs1_data, rs2_data;
  logic [31:0]     immediate;
  logic            r_type, i_type, s_type, b_type, u_type, j_type;
  logic            pc_sel;
  logic            op1_sel, op2_sel;
  alu_op_t         alu_op;
  wb_src_t         rf_wr_data_sel;
  logic            rf_wr_en;
  logic            dmem_req;
  logic            dmem_wr_en;
  mem_size_t       dmem_size;
  logic            dmem_zero_extend;

  // WB-to-ID bypass (same-cycle register write + read)
  logic [XLEN-1:0] rs1_data_bypassed, rs2_data_bypassed;


  //  BTB Signals
  // BRAM registered outputs (aligned with current PC via lookahead addressing)
  logic              btb_predict_taken;
  logic [XLEN-1:0]  btb_predict_target;

  // Lookahead address — feeds BRAM 1 cycle early so output aligns with current PC
  logic [XLEN-1:0]  btb_lookup_addr;

  // EX-stage update inputs
  logic              btb_update_en;
  logic              actual_taken;
  logic [XLEN-1:0]  actual_target_addr;
  logic              btb_mispredict;


  //  EX Stage Signals
  logic [XLEN-1:0] alu_a, alu_b, alu_res;
  logic [XLEN-1:0] fwd_rs1_data, fwd_rs2_data;  // Forwarded operands
  logic [1:0]      fwd_a_sel, fwd_b_sel;         // Forwarding mux selects
  logic [XLEN-1:0] ex_mem_fwd_data;              // EX/MEM forwarding value
  logic [XLEN-1:0] ex_pc_plus_4;                 // PC+4 for JAL/JALR link
  logic            ex_branch_taken;              // Branch decision in EX
  logic [XLEN-1:0] ex_redirect_target;           // Branch/jump target

  
  //  MEM Stage Signals
  logic [XLEN-1:0] dmem_rd_data;

  
  //  WB Stage Signals 
  logic [XLEN-1:0] wr_data;

  //  Pipeline Registers
  // IF/ID
  logic [31:0]     if_id_pc;
  logic [31:0]     if_id_instr;
  logic            if_id_btb_predict_taken;
  logic [XLEN-1:0] if_id_btb_predict_target;

  // ID/EX — Control
  logic            id_ex_rf_wr_en;
  wb_src_t         id_ex_rf_wr_data_sel;
  logic            id_ex_dmem_wr_en;
  logic            id_ex_dmem_req;
  mem_size_t       id_ex_dmem_size;
  logic            id_ex_dmem_zero_extend;
  alu_op_t         id_ex_alu_op;
  logic            id_ex_op1_sel;
  logic            id_ex_op2_sel;
  logic            id_ex_pc_sel;
  logic            id_ex_b_type;
  logic [2:0]      id_ex_funct3;

  // ID/EX — Data
  logic [XLEN-1:0] id_ex_pc;
  logic [XLEN-1:0] id_ex_rs1_data;
  logic [XLEN-1:0] id_ex_rs2_data;
  logic [XLEN-1:0] id_ex_immediate;

  // ID/EX — Register addresses (for forwarding)
  logic [4:0]      id_ex_rs1_addr;
  logic [4:0]      id_ex_rs2_addr;
  logic [4:0]      id_ex_rd_addr;

  // ID/EX — BTB prediction (propagated from IF/ID for misprediction detection)
  logic            id_ex_btb_predict_taken;
  logic [XLEN-1:0] id_ex_btb_predict_target;

  // EX/MEM — Control
  logic            ex_mem_rf_wr_en;
  wb_src_t         ex_mem_rf_wr_data_sel;
  logic            ex_mem_dmem_wr_en;
  logic            ex_mem_dmem_req;
  mem_size_t       ex_mem_dmem_size;
  logic            ex_mem_dmem_zero_extend;

  // EX/MEM — Data
  logic [XLEN-1:0] ex_mem_alu_res;
  logic [XLEN-1:0] ex_mem_rs2_data;
  logic [4:0]      ex_mem_rd_addr;
  logic [XLEN-1:0] ex_mem_pc_plus_4;
  logic [XLEN-1:0] ex_mem_immediate;

  // MEM/WB — Control
  logic            mem_wb_rf_wr_en;
  wb_src_t         mem_wb_rf_wr_data_sel;

  // MEM/WB — Data
  logic [XLEN-1:0] mem_wb_alu_res;
  logic [XLEN-1:0] mem_wb_dmem_rd_data;
  logic [4:0]      mem_wb_rd_addr;
  logic [XLEN-1:0] mem_wb_pc_plus_4;
  logic [XLEN-1:0] mem_wb_immediate;

  
  //  IF Stage
  // BTB lookahead address: mirrors PC mux so BRAM output is pre-aligned
  always_comb begin
    if (effective_redirect)
      btb_lookup_addr = ex_redirect_target;
    else if (btb_predict_taken && !pipeline_stall)
      btb_lookup_addr = btb_predict_target;
    else if (!pipeline_stall)
      btb_lookup_addr = next_seq_pc;
    else
      btb_lookup_addr = pc;             // Stall — re-read same entry
  end

  // PC Register (priority: EX correction > BTB prediction > sequential)
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)
      pc <= RESET_PC;

    else if (effective_redirect)
      pc <= ex_redirect_target;         // Highest: misprediction correction

    else if (btb_predict_taken && !pipeline_stall)
      pc <= btb_predict_target;         // BTB speculative redirect

    else if (!pipeline_stall)
      pc <= next_seq_pc;                // Sequential PC+4
  end

  assign next_seq_pc = pc + 32'h4;
  assign pc_out      = pc;

  // L1 Instruction Cache — direct-mapped, BRAM-based, lookahead addressing
  l1_icache u_l1_icache (
    .clk             (clk),
    .reset_n         (reset_n),
    .cpu_req         (reset_n),
    .cpu_addr        (btb_lookup_addr),     // Lookahead address (aligned with current PC)
    .cpu_data        (instruction),
    .cpu_ready       (icache_ready),
    .mem_req         (ic_mem_req),
    .mem_addr        (ic_mem_addr),
    .mem_data        (ic_mem_data),
    .mem_data_valid  (ic_mem_data_valid),
    .mem_ready       (ic_mem_ready)
  );

  assign icache_stall = ~icache_ready;

  // Branch Target Buffer (BTB) — BRAM, 1-cycle read latency
  btb u_btb (
    .clk            (clk),
    .reset_n        (reset_n),
    .pc             (btb_lookup_addr),
    .predict_taken  (btb_predict_taken),
    .predict_target (btb_predict_target),
    .update_en      (btb_update_en),
    .update_pc      (id_ex_pc),
    .actual_target  (actual_target_addr),
    .was_taken      (actual_taken)
  );

  // IF/ID Pipeline Register
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      if_id_pc                 <= RESET_PC;
      if_id_instr              <= 32'h00000013; // NOP
      if_id_btb_predict_taken  <= 1'b0;
      if_id_btb_predict_target <= '0;
    end

    else if (effective_redirect) begin
      if_id_pc                 <= RESET_PC;
      if_id_instr              <= 32'h00000013; // NOP
      if_id_btb_predict_taken  <= 1'b0;
      if_id_btb_predict_target <= '0;
    end

    else if (!pipeline_stall) begin
      if_id_pc                 <= pc;
      if_id_instr              <= instruction;
      if_id_btb_predict_taken  <= btb_predict_taken;
      if_id_btb_predict_target <= btb_predict_target;
    end
    // else: cache stall — hold all IF/ID values
  end

  
  //  ID Stage
  // Decode
  decode u_decode (
    .instruction (if_id_instr),
    .rs1_addr    (rs1_addr),
    .rs2_addr    (rs2_addr),
    .rd_addr     (rd_addr),
    .opcode      (opcode),
    .funct3      (funct3),
    .funct7      (funct7),
    .r_type      (r_type),
    .i_type      (i_type),
    .s_type      (s_type),
    .b_type      (b_type),
    .u_type      (u_type),
    .j_type      (j_type),
    .immediate   (immediate)
  );

  // Control Unit
  control u_control (
    .r_type           (r_type),
    .i_type           (i_type),
    .s_type           (s_type),
    .b_type           (b_type),
    .u_type           (u_type),
    .j_type           (j_type),
    .funct3           (funct3),
    .funct7           (funct7),
    .opcode           (opcode),
    .pc_sel           (pc_sel),
    .op1_sel          (op1_sel),
    .op2_sel          (op2_sel),
    .alu_op           (alu_op),
    .rf_wr_data_sel   (rf_wr_data_sel),
    .dmem_req         (dmem_req),
    .dmem_size        (dmem_size),
    .dmem_wr_en       (dmem_wr_en),
    .dmem_zero_extend (dmem_zero_extend),
    .rf_wr_en         (rf_wr_en)
  );

  // Register File (write port driven from WB stage)
  register_file u_register_file (
    .clk      (clk),
    .reset_n  (reset_n),
    .rs1_addr (rs1_addr),
    .rs2_addr (rs2_addr),
    .rd_addr  (mem_wb_rd_addr),
    .rf_wr_en (mem_wb_rf_wr_en),
    .wr_data  (wr_data),
    .rs1_data (rs1_data),
    .rs2_data (rs2_data)
  );

  // WB-to-ID handle same-cycle write+read conflict
  assign rs1_data_bypassed = (mem_wb_rf_wr_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == rs1_addr))
                             ? wr_data : rs1_data;

  assign rs2_data_bypassed = (mem_wb_rf_wr_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == rs2_addr))
                             ? wr_data : rs2_data;

  // Hazard Detection Unit
  hazard_detection u_hazard_detection (
    .rs1_addr              (rs1_addr),
    .rs2_addr              (rs2_addr),
    .id_ex_rf_wr_data_sel  (id_ex_rf_wr_data_sel),
    .id_ex_rd_addr         (id_ex_rd_addr),
    .stall                 (hazard_stall)
  );

  // ID/EX Pipeline Register
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      id_ex_rf_wr_en           <= 1'b0;
      id_ex_rf_wr_data_sel     <= WB_SRC_ALU;
      id_ex_dmem_wr_en         <= 1'b0;
      id_ex_dmem_req           <= 1'b0;
      id_ex_dmem_size          <= BYTE;
      id_ex_dmem_zero_extend   <= 1'b0;
      id_ex_alu_op             <= ADD;
      id_ex_op1_sel            <= 1'b0;
      id_ex_op2_sel            <= 1'b0;
      id_ex_pc_sel             <= 1'b0;
      id_ex_b_type             <= 1'b0;
      id_ex_funct3             <= 3'b0;
      id_ex_pc                 <= '0;
      id_ex_rs1_data           <= '0;
      id_ex_rs2_data           <= '0;
      id_ex_immediate          <= '0;
      id_ex_rs1_addr           <= 5'b0;
      id_ex_rs2_addr           <= 5'b0;
      id_ex_rd_addr            <= 5'b0;
      id_ex_btb_predict_taken  <= 1'b0;
      id_ex_btb_predict_target <= '0;
    end 
    
    // Bubble: on effective redirect OR hazard-only stall (not cache stall)
    else if (effective_redirect || (hazard_stall && !icache_stall && !dcache_stall)) begin
      id_ex_rf_wr_en           <= 1'b0;
      id_ex_rf_wr_data_sel     <= WB_SRC_ALU;
      id_ex_dmem_wr_en         <= 1'b0;
      id_ex_dmem_req           <= 1'b0;
      id_ex_dmem_size          <= BYTE;
      id_ex_dmem_zero_extend   <= 1'b0;
      id_ex_alu_op             <= ADD;
      id_ex_op1_sel            <= 1'b0;
      id_ex_op2_sel            <= 1'b0;
      id_ex_pc_sel             <= 1'b0;
      id_ex_b_type             <= 1'b0;
      id_ex_funct3             <= 3'b0;
      id_ex_pc                 <= '0;
      id_ex_rs1_data           <= '0;
      id_ex_rs2_data           <= '0;
      id_ex_immediate          <= '0;
      id_ex_rs1_addr           <= 5'b0;
      id_ex_rs2_addr           <= 5'b0;
      id_ex_rd_addr            <= 5'b0;
      id_ex_btb_predict_taken  <= 1'b0;
      id_ex_btb_predict_target <= '0;
    end

    // Normal latch: only when no stall source active
    else if (!pipeline_stall) begin
      id_ex_rf_wr_en           <= rf_wr_en;
      id_ex_rf_wr_data_sel     <= rf_wr_data_sel;
      id_ex_dmem_wr_en         <= dmem_wr_en;
      id_ex_dmem_req           <= dmem_req;
      id_ex_dmem_size          <= dmem_size;
      id_ex_dmem_zero_extend   <= dmem_zero_extend;
      id_ex_alu_op             <= alu_op;
      id_ex_op1_sel            <= op1_sel;
      id_ex_op2_sel            <= op2_sel;
      id_ex_pc_sel             <= pc_sel;
      id_ex_b_type             <= b_type;
      id_ex_funct3             <= funct3;
      id_ex_pc                 <= if_id_pc;
      id_ex_rs1_data           <= rs1_data_bypassed;
      id_ex_rs2_data           <= rs2_data_bypassed;
      id_ex_immediate          <= immediate;
      id_ex_rs1_addr           <= rs1_addr;
      id_ex_rs2_addr           <= rs2_addr;
      id_ex_rd_addr            <= rd_addr;
      id_ex_btb_predict_taken  <= if_id_btb_predict_taken;
      id_ex_btb_predict_target <= if_id_btb_predict_target;
    end
    // else: cache stall — hold all ID/EX registers
  end

  
  //  EX Stage
  // Forwarding Unit
  forwarding_unit u_forwarding_unit (
    .id_ex_rs1_addr (id_ex_rs1_addr),
    .id_ex_rs2_addr (id_ex_rs2_addr),
    .ex_mem_rf_wr_en (ex_mem_rf_wr_en),
    .ex_mem_rd_addr  (ex_mem_rd_addr),
    .mem_wb_rf_wr_en (mem_wb_rf_wr_en),
    .mem_wb_rd_addr  (mem_wb_rd_addr),
    .fwd_a_sel       (fwd_a_sel),
    .fwd_b_sel       (fwd_b_sel)
  );

  // EX/MEM forwarding value: select the correct writeback data type
  always_comb begin
    case (ex_mem_rf_wr_data_sel)
      WB_SRC_ALU: ex_mem_fwd_data = ex_mem_alu_res;
      WB_SRC_IMM: ex_mem_fwd_data = ex_mem_immediate;
      WB_SRC_PC:  ex_mem_fwd_data = ex_mem_pc_plus_4;
      default:    ex_mem_fwd_data = ex_mem_alu_res;
    endcase
  end

  // Forwarding muxes
  always_comb begin
    case (fwd_a_sel)
      2'b01:   fwd_rs1_data = ex_mem_fwd_data;  // Forward from EX/MEM
      2'b10:   fwd_rs1_data = wr_data;           // Forward from MEM/WB
      default: fwd_rs1_data = id_ex_rs1_data;    // No forwarding
    endcase
  end

  always_comb begin
    case (fwd_b_sel)
      2'b01:   fwd_rs2_data = ex_mem_fwd_data;  // Forward from EX/MEM
      2'b10:   fwd_rs2_data = wr_data;           // Forward from MEM/WB
      default: fwd_rs2_data = id_ex_rs2_data;    // No forwarding
    endcase
  end

  // ALU operand muxes (use forwarded data)
  assign alu_a = id_ex_op1_sel ? id_ex_pc       : fwd_rs1_data;
  assign alu_b = id_ex_op2_sel ? id_ex_immediate : fwd_rs2_data;

  // ALU
  alu u_alu (
    .alu_a   (alu_a),
    .alu_b   (alu_b),
    .alu_op  (id_ex_alu_op),
    .alu_res (alu_res),
    .zero    ()
  );

  // Branch Control (resolved in EX using forwarded operands)
  branch_control u_branch_control (
    .opr_a        (fwd_rs1_data),
    .opr_b        (fwd_rs2_data),
    .is_b_type    (id_ex_b_type),
    .funct3       (id_ex_funct3),
    .branch_taken (ex_branch_taken)
  );

  // EX-stage: Actual branch/jump outcome
  assign actual_taken      = ex_branch_taken | id_ex_pc_sel;
  assign actual_target_addr = {alu_res[XLEN-1:1], 1'b0};

  // BTB update enable: fire for any branch or jump instruction in EX
  assign btb_update_en = id_ex_b_type | id_ex_pc_sel;

  
  assign btb_mispredict = (id_ex_btb_predict_taken != actual_taken) || (actual_taken && (id_ex_btb_predict_target != actual_target_addr));

  // Redirect only on misprediction (correct prediction = no pipeline flush)
  assign ex_redirect        = btb_mispredict;
  assign ex_redirect_target = actual_taken ? actual_target_addr : ex_pc_plus_4;

  // PC+4 for JAL/JALR link register writeback (and not-taken correction target)
  assign ex_pc_plus_4 = id_ex_pc + 32'h4;

  // EX/MEM Pipeline Register — freeze on cache stall
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      ex_mem_rf_wr_en         <= 1'b0;
      ex_mem_rf_wr_data_sel   <= WB_SRC_ALU;
      ex_mem_dmem_wr_en       <= 1'b0;
      ex_mem_dmem_req         <= 1'b0;
      ex_mem_dmem_size        <= BYTE;
      ex_mem_dmem_zero_extend <= 1'b0;
      ex_mem_alu_res          <= '0;
      ex_mem_rs2_data         <= '0;
      ex_mem_rd_addr          <= 5'b0;
      ex_mem_pc_plus_4        <= '0;
      ex_mem_immediate        <= '0;
    end

    else if (!icache_stall && !dcache_stall) begin
      ex_mem_rf_wr_en         <= id_ex_rf_wr_en;
      ex_mem_rf_wr_data_sel   <= id_ex_rf_wr_data_sel;
      ex_mem_dmem_wr_en       <= id_ex_dmem_wr_en;
      ex_mem_dmem_req         <= id_ex_dmem_req;
      ex_mem_dmem_size        <= id_ex_dmem_size;
      ex_mem_dmem_zero_extend <= id_ex_dmem_zero_extend;
      ex_mem_alu_res          <= alu_res;
      ex_mem_rs2_data         <= fwd_rs2_data;  // Forwarded store data
      ex_mem_rd_addr          <= id_ex_rd_addr;
      ex_mem_pc_plus_4        <= ex_pc_plus_4;
      ex_mem_immediate        <= id_ex_immediate;
    end
    // else: cache stall — hold
  end

  
  //  MEM Stage
  // L1 Data Cache — direct-mapped, BRAM-based, write-back policy
  l1_dcache u_l1_dcache (
    .clk              (clk),
    .reset_n          (reset_n),
    .cpu_req          (ex_mem_dmem_req),
    .cpu_wr_en        (ex_mem_dmem_wr_en),
    .cpu_data_size    (ex_mem_dmem_size),
    .cpu_addr         (ex_mem_alu_res),
    .cpu_wr_data      (ex_mem_rs2_data),
    .cpu_zero_extend  (ex_mem_dmem_zero_extend),
    .cpu_rd_data      (dmem_rd_data),
    .cpu_ready        (dcache_ready),
    .mem_req          (dc_mem_req),
    .mem_wr_en        (dc_mem_wr_en),
    .mem_addr         (dc_mem_addr),
    .mem_wr_data      (dc_mem_wr_data),
    .mem_data         (dc_mem_data),
    .mem_data_valid   (dc_mem_data_valid),
    .mem_ready        (dc_mem_ready)
  );

  // D-Cache stall: only when a valid memory request is outstanding and not ready
  assign dcache_stall = ex_mem_dmem_req & ~dcache_ready;

  // Main Memory Controller — unified backing store with arbiter
  main_memory u_main_memory (
    .clk                (clk),
    .reset_n            (reset_n),
    .icache_req         (ic_mem_req),
    .icache_addr        (ic_mem_addr),
    .icache_data        (ic_mem_data),
    .icache_data_valid  (ic_mem_data_valid),
    .icache_ready       (ic_mem_ready),
    .dcache_req         (dc_mem_req),
    .dcache_wr_en       (dc_mem_wr_en),
    .dcache_addr        (dc_mem_addr),
    .dcache_wr_data     (dc_mem_wr_data),
    .dcache_data        (dc_mem_data),
    .dcache_data_valid  (dc_mem_data_valid),
    .dcache_ready       (dc_mem_ready)
  );

  // MEM/WB Pipeline Register — freeze on D-Cache stall
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      mem_wb_rf_wr_en       <= 1'b0;
      mem_wb_rf_wr_data_sel <= WB_SRC_ALU;
      mem_wb_alu_res        <= '0;
      mem_wb_dmem_rd_data   <= '0;
      mem_wb_rd_addr        <= 5'b0;
      mem_wb_pc_plus_4      <= '0;
      mem_wb_immediate      <= '0;
    end

    else if (!dcache_stall) begin
      mem_wb_rf_wr_en       <= ex_mem_rf_wr_en;
      mem_wb_rf_wr_data_sel <= ex_mem_rf_wr_data_sel;
      mem_wb_alu_res        <= ex_mem_alu_res;
      mem_wb_dmem_rd_data   <= dmem_rd_data;
      mem_wb_rd_addr        <= ex_mem_rd_addr;
      mem_wb_pc_plus_4      <= ex_mem_pc_plus_4;
      mem_wb_immediate      <= ex_mem_immediate;
    end
    // else: D-cache stall — hold
  end

  
  //  WB Stage
  always_comb begin
    case (mem_wb_rf_wr_data_sel)
      WB_SRC_ALU: wr_data = mem_wb_alu_res;
      WB_SRC_MEM: wr_data = mem_wb_dmem_rd_data;
      WB_SRC_IMM: wr_data = mem_wb_immediate;
      WB_SRC_PC:  wr_data = mem_wb_pc_plus_4;
      default:    wr_data = mem_wb_alu_res;
    endcase
  end

endmodule
