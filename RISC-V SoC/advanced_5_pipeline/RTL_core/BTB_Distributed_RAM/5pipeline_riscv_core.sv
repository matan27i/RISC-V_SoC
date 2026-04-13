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
  logic stall;        // Load-use hazard stall (from hazard_detection)
  logic ex_redirect;  // Branch/jump taken in EX stage

  
  //  IF Stage Signals
  
  logic [XLEN-1:0] pc, next_pc, next_seq_pc;
  logic            imem_req;
  logic [XLEN-1:0] imem_addr;
  logic [31:0]     imem_data;
  logic [31:0]     instruction;

  
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

  // IF-stage outputs (combinational read from BTB)
  logic              btb_predict_taken;
  logic [XLEN-1:0]  btb_predict_target;
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
  // PC Register (priority: EX correction > BTB prediction > sequential)
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)
      pc <= RESET_PC;
    else if (ex_redirect)
      pc <= ex_redirect_target;         // Highest: misprediction correction
    else if (!stall) begin
      if (btb_predict_taken)
        pc <= btb_predict_target;       // BTB speculative redirect
      else
        pc <= next_seq_pc;              // Sequential PC+4
    end
  end

  assign next_seq_pc = pc + 32'h4;
  assign pc_out      = pc;

  // Instruction Memory
  instruction_memory u_instruction_memory (
    .imem_req  (imem_req),
    .imem_addr (imem_addr),
    .imem_data (imem_data)
  );

  // Fetch
  fetch u_fetch (
    .clk         (clk),
    .reset_n     (reset_n),
    .pc          (pc),
    .imem_data   (imem_data),
    .imem_req    (imem_req),
    .imem_addr   (imem_addr),
    .instruction (instruction)
  );

  // Branch Target Buffer (BTB)
  btb u_btb (
    .clk            (clk),
    .reset_n        (reset_n),
    // Read port — IF stage (combinational lookup using current PC)
    .pc             (pc),
    .predict_taken  (btb_predict_taken),
    .predict_target (btb_predict_target),
    // Write/Update port — EX stage (feedback from branch resolution)
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
    
    else if (ex_redirect) begin
      if_id_pc                 <= RESET_PC;
      if_id_instr              <= 32'h00000013; // NOP (flush wrong-path instruction)
      if_id_btb_predict_taken  <= 1'b0;
      if_id_btb_predict_target <= '0;
    end 
    
    else if (!stall) begin
      if_id_pc                 <= pc;
      if_id_instr              <= instruction;
      if_id_btb_predict_taken  <= btb_predict_taken;
      if_id_btb_predict_target <= btb_predict_target;
    end
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

  // WB-to-ID Bypass: handle same-cycle write+read conflict
  // (Register file uses NBA write, so combinational read returns the OLD value)
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
    .stall                 (stall)
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
    
    else if (ex_redirect || stall) begin //insert bubble (NOP) in EX stage
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
    
    else begin                            //propagate values from ID to EX
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

  assign ex_pc_plus_4 = id_ex_pc + 32'h4;

  // EX/MEM Pipeline Register
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
    
    else begin
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
  end

  
  //  MEM Stage
  data_memory u_data_memory (
    .clk              (clk),
    .dmem_req         (ex_mem_dmem_req),
    .dmem_wr_en       (ex_mem_dmem_wr_en),
    .dmem_data_size   (ex_mem_dmem_size),
    .dmem_addr        (ex_mem_alu_res),
    .dmem_wr_data     (ex_mem_rs2_data),
    .dmem_zero_extend (ex_mem_dmem_zero_extend),
    .dmem_rd_data     (dmem_rd_data)
  );

  // MEM/WB Pipeline Register
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
    
    else begin
      mem_wb_rf_wr_en       <= ex_mem_rf_wr_en;
      mem_wb_rf_wr_data_sel <= ex_mem_rf_wr_data_sel;
      mem_wb_alu_res        <= ex_mem_alu_res;
      mem_wb_dmem_rd_data   <= dmem_rd_data;
      mem_wb_rd_addr        <= ex_mem_rd_addr;
      mem_wb_pc_plus_4      <= ex_mem_pc_plus_4;
      mem_wb_immediate      <= ex_mem_immediate;
    end
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
