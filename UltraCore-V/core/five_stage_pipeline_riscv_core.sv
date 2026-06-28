// Top-Level 5-Stage Pipelined RISC-V Core

// Timing optimizations (target: 100 MHz / 10 ns period):
//  (1) Pipelined redirect  — ex_redirect/target registered before
//      reaching IF. Breaks the long EX→IF combinational path that
//      forced BRAM address pins to settle within the ALU+compare
//      critical path. +1 cycle mispredict penalty (2→3).
//  (2) Branch Target Adder — dedicated pc+imm adder in parallel
//      with the ALU. B-type / JAL bypass the 32-bit ALU carry
//      chain on the mispredict path.
//  (3) Pre-registered forwarding selects — fwd_a_sel/fwd_b_sel
//      computed in ID (using id_ex_rd_addr and ex_mem_rd_addr as
//      proxies for next cycle's ex_mem_rd_addr and mem_wb_rd_addr)
//      then registered into ID/EX. Removes 4 address comparators
//      from the EX critical path.
//  (4) max_fanout attributes on high-fanout pipeline regs so
//      Vivado auto-replicates (reduces 11 ns net delay).
//  (5) JALR Target Adder — dedicated rs1+imm adder in parallel
//      with the ALU. Removes the ALU's 10-way output case-mux
//      (~1.5 ns) from the JALR mispredict path. Combined with
//      Opt 2, no branch/jump target now goes through the ALU,
//      so alu_res only feeds the ex_mem_alu_res register
//      (register-to-register).



// SoC integration (UltraCore-V):
//  (a) MMIO bypass — data accesses outside the cacheable RAM window
//      (peripherals at/above 0x4000_0000 and the boot-ROM data window
//      below 0x0000_2000) bypass the write-back D-cache through a
//      native MMIO port served by axi_master_bridge. A dedicated
//      mmio_stall freezes the pipeline for the duration of the bus
//      transaction, giving strongly-ordered, uncached device access.
//  (b) Boot ROM fetch routing — I-cache line fills for addresses below
//      0x0000_2000 are routed to the external boot_rom burst port
//      instead of main_memory. Fill addresses are stable per miss, so
//      the route is a safe combinational decode.
//  (c) Zicsr + trap unit — csr_file holds the M-mode CSRs; irq_timer /
//      irq_ecc map to mip.MTIP / mip.MEIP and, when enabled, vector the
//      core to mtvec (precise: mepc captures the trapped PC). CSR
//      instructions, MRET, ECALL and EBREAK resolve in EX through the
//      existing redirect path.
//  Cacheable RAM window: 0x0000_2000 - 0x0000_3FFF (16 KB backing
//  store, upper half; addresses above it up to 0x3FFF_FFFF alias RAM
//  and are reserved — software must not use them).


import risc_pkg::*;

module five_stage_pipeline_riscv_core #(
  parameter RESET_PC      = 32'h0000,
  parameter MEM_INIT_FILE = "machine_code.mem"
)
(
  input  logic             clk,
  input  logic             reset_n,
  output logic [XLEN-1:0]  pc_out,

  // MMIO master port (uncached; request fields stable while stalled)
  output logic             mmio_req,
  output logic             mmio_wr_en,
  output logic [XLEN-1:0]  mmio_addr,
  output logic [31:0]      mmio_wr_data,
  output mem_size_t        mmio_size,
  output logic             mmio_zero_extend,
  input  logic [31:0]      mmio_rd_data,
  input  logic             mmio_ready,
  // High when the pipeline can consume an MMIO completion this cycle.
  // The bridge holds its DONE state (mmio_ready level) until accepted,
  // so a completion arriving under an I-/D-cache stall is neither lost
  // nor re-issued (re-issuing would repeat device side effects).
  output logic             mmio_accept,

  // Boot ROM fetch-fill port (I-cache line fills for addr < 0x2000)
  output logic             rom_fetch_req,
  output logic [XLEN-1:0]  rom_fetch_addr,
  input  logic [31:0]      rom_fetch_data,
  input  logic             rom_fetch_data_valid,
  input  logic             rom_fetch_ready,

  // Interrupt request lines (synchronous, level). Registered below as
  // pending hooks; trap/CSR logic is future work.
  input  logic             irq_timer,
  input  logic             irq_ecc
);

  //  Internal Signals
  //  Hazard & Flush Control
  logic hazard_stall;       // Load-use hazard stall (from hazard_detection)
  logic icache_stall;       // I-Cache miss stall
  logic dcache_stall;       // D-Cache miss stall
  logic mmio_stall;         // MMIO (AXI) transaction in progress
  logic muldiv_stall;       // M-extension multiply/divide in progress
  logic wfi_stall;          // Core parked on WFI (asleep) until an interrupt
  logic pipeline_stall;     // Composite: any source of pipeline stall
  logic ex_redirect;        // Branch/jump mispredict in EX (combinational)
  logic ex_redirect_qual;   // Gated by cache stalls (still combinational, internal)

  // MMIO decode (MEM stage): peripherals at/above 0x4000_0000 and the
  // boot-ROM data window below 0x2000 bypass the D-cache.
  logic mmio_sel;           // Current MEM-stage access targets MMIO
  logic dc_cpu_req;         // D-cache request, gated off for MMIO

  // Pipelined redirect — the single optimization that breaks the
  // EX→IF combinational path. redir_valid_r is the flush signal
  // seen by IF / IF-ID / ID-EX; redir_target_r is the new PC.
  (* max_fanout = 32 *) logic            redir_valid_r;
  logic [XLEN-1:0]                       redir_target_r;

  assign pipeline_stall   = hazard_stall | icache_stall | dcache_stall | mmio_stall | muldiv_stall | wfi_stall;
  assign ex_redirect_qual = ex_redirect & ~icache_stall & ~dcache_stall & ~mmio_stall & ~muldiv_stall & ~wfi_stall;


  //  IF Stage Signals
  logic [XLEN-1:0] pc, next_seq_pc;
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

  // Pre-computed forwarding selects (Optimization 3)
  logic [1:0]      pre_fwd_a_sel, pre_fwd_b_sel;

  // ID-stage Zicsr / privileged-instruction decode (SYSTEM opcode)
  logic            is_system_id, is_csr_id, is_mret_id, is_ecall_id, is_ebreak_id;
  logic [11:0]     csr_addr_id;
  logic [1:0]      csr_op_id;
  logic            csr_use_imm_id;
  logic            csr_wen_pre_id;

  // ID-stage M-extension decode (R-type, funct7 = 0000001)
  logic            is_muldiv_id;

  // ID-stage WFI decode (SYSTEM funct3=0, funct12=0x105)
  logic            is_wfi_id;


  //  BTB Signals
  // BRAM registered outputs (aligned with current PC via lookahead addressing)
  logic              btb_predict_taken;
  logic [XLEN-1:0]   btb_predict_target;

  // Lookahead address — feeds BRAM 1 cycle early so output aligns with current PC
  logic [XLEN-1:0]   btb_lookup_addr;

  // EX-stage update inputs
  logic              btb_update_en;
  logic              actual_taken;
  logic [XLEN-1:0]   actual_target_addr;
  logic              btb_mispredict;

  // Branch Target Adder — parallel to ALU (Optimization 2)
  logic [XLEN-1:0]   bta_target;

  // JALR Target Adder — parallel to ALU (Optimization 5)
  logic [XLEN-1:0]   jalr_target;


  //  EX Stage Signals
  logic [XLEN-1:0] alu_a, alu_b, alu_res;
  logic [XLEN-1:0] fwd_rs1_data, fwd_rs2_data;  // Forwarded operands
  logic [XLEN-1:0] ex_mem_fwd_data;              // EX/MEM forwarding value
  logic [XLEN-1:0] ex_pc_plus_4;                 // PC+4 for JAL/JALR link
  logic            ex_branch_taken;              // Branch decision in EX
  logic [XLEN-1:0] ex_redirect_target;           // Branch/jump target

  //  EX Stage — Zicsr / trap
  logic [31:0]     csr_rdata;        // old CSR value (combinational read)
  logic [31:0]     csr_src;          // rs1 or zimm source operand
  logic [31:0]     csr_wdata_ex;     // computed new CSR value
  logic [31:0]     mtvec_w, mepc_w;  // trap vector / return address
  logic [31:0]     irq_cause_w;      // pending-interrupt cause code
  logic            irq_req_w;        // an enabled interrupt is pending
  logic            ex_commit;        // EX holds a real, non-flushed, unstalled instr
  logic            take_interrupt, take_exception;
  logic            trap_redirect, mret_redirect, csr_serialize, trap_flush_ex;
  logic            csr_commit_wen;
  logic [31:0]     trap_cause_ex;

  //  EX Stage — M-extension (muldiv_stall declared with the stall set above)
  logic [31:0]     muldiv_result;
  logic            muldiv_req, muldiv_busy, muldiv_done, ex_can_advance;
  logic            muldiv_gclk;        // gated clock for the M-unit

  //  EX Stage — WFI / low power
  logic            irq_wake_w;         // any locally-enabled IRQ pending (ignores MIE)
  logic            core_asleep;        // parked on WFI


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

  // ID/EX — Destination register address (hazard detection + the
  // pre-computed forwarding compares in ID). The rs1/rs2 address
  // registers that once fed the EX-stage forwarding_unit were removed:
  // dead since Optimization 3 inlined the selects into ID.
  (* max_fanout = 32 *) logic [4:0]      id_ex_rd_addr;

  // ID/EX — Pre-registered forwarding selects (Optimization 3)
  logic [1:0]      id_ex_fwd_a_sel;
  logic [1:0]      id_ex_fwd_b_sel;

  // ID/EX — BTB prediction (propagated from IF/ID for misprediction detection)
  logic            id_ex_btb_predict_taken;
  logic [XLEN-1:0] id_ex_btb_predict_target;

  // ID/EX — M-extension
  logic            id_ex_is_muldiv;

  // ID/EX — WFI
  logic            id_ex_is_wfi;

  // ID/EX — Zicsr / trap control (propagated from the ID-stage decode)
  logic            id_ex_valid;        // 1 = a real instruction occupies EX
  logic            id_ex_is_csr;       // CSRRW/S/C (reg or imm)
  logic [11:0]     id_ex_csr_addr;     // CSR index (instr[31:20])
  logic [1:0]      id_ex_csr_op;       // funct3[1:0]: 01=RW 10=RS 11=RC
  logic            id_ex_csr_use_imm;  // funct3[2]: immediate variant
  logic [4:0]      id_ex_csr_zimm;     // 5-bit CSR immediate (rs1 field)
  logic            id_ex_csr_wen;      // CSR write pre-condition (RW, or src!=x0)
  logic            id_ex_is_mret;
  logic            id_ex_is_ecall;
  logic            id_ex_is_ebreak;

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
  (* max_fanout = 32 *) logic [4:0]      ex_mem_rd_addr;
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
  
  // btb_lookup_addr is driven by the REGISTERED redirect (Opt 1).
  // The combinational ex_redirect no longer reaches BRAM pins.
  
  always_comb begin
    if (redir_valid_r)
      btb_lookup_addr = redir_target_r;       // Registered redirect
    else if (btb_predict_taken && !pipeline_stall)
      btb_lookup_addr = btb_predict_target;   // BTB speculative
    else if (!pipeline_stall)
      btb_lookup_addr = next_seq_pc;          // Sequential
    else
      btb_lookup_addr = pc;                   // Stall — hold
  end

  // PC Register — now updated from the REGISTERED redirect
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)
      pc <= RESET_PC;

    else if (redir_valid_r)
      pc <= redir_target_r;                   // Registered mispredict recovery

    else if (btb_predict_taken && !pipeline_stall)
      pc <= btb_predict_target;               // BTB speculative redirect

    else if (!pipeline_stall)
      pc <= next_seq_pc;                      // Sequential PC+4
  end

  assign next_seq_pc = pc + 32'h4;
  assign pc_out      = pc;

  
  // Redirect pipeline register (Optimization 1)
  // Captures ex_redirect/target in EX, applies one cycle later.
  // One-shot: cleared the cycle after it fires.
  // Frozen during cache stalls so the redirect is preserved.
  
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      redir_valid_r  <= 1'b0;
      redir_target_r <= '0;
    end
    else if (icache_stall || dcache_stall || mmio_stall || muldiv_stall || wfi_stall) begin
      // Freeze during cache/MMIO/muldiv/WFI stall
    end
    else if (redir_valid_r) begin
      redir_valid_r  <= 1'b0;                 // Consumed this cycle
    end
    else if (ex_redirect_qual) begin
      redir_valid_r  <= 1'b1;
      redir_target_r <= ex_redirect_target;
    end
  end

  // L1 Instruction Cache — direct-mapped, BRAM-based, lookahead addressing
  l1_icache u_l1_icache (
    .clk             (clk),
    .reset_n         (reset_n),
    .cpu_req         (1'b1),
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

  // IF/ID Pipeline Register — flushed by registered redirect
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      if_id_pc                 <= RESET_PC;
      if_id_instr              <= 32'h00000013; // NOP
      if_id_btb_predict_taken  <= 1'b0;
      if_id_btb_predict_target <= '0;
    end

    else if (redir_valid_r) begin
      // Flush the speculative instruction fetched while redirect was in flight
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
    .funct7      (),                // Unused: control derives funct7_bit5 from instr[30]
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
    .funct7_bit5      (if_id_instr[30]), // For SRLI vs SRAI
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

  // WB-to-ID handle same-cycle register write + read
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

  
  // Zicsr / privileged decode (SYSTEM opcode 0x73)
  //   funct3 != 0  -> CSRRW/S/C, immediate variant if funct3[2]=1
  //   funct3 == 0  -> ECALL(0x000) / EBREAK(0x001) / MRET(0x302) by funct12
  // CSR writes rd with the old value, so it asserts rf_wr_en in ID/EX.
  // RS/RC with rs1=x0 (or immediate zimm=0) must not write the CSR.
  
  assign is_system_id   = (opcode == 7'b1110011);
  assign csr_addr_id    = if_id_instr[31:20];
  assign csr_op_id      = funct3[1:0];
  assign csr_use_imm_id = funct3[2];
  assign is_csr_id      = is_system_id && (funct3 != 3'b000);
  assign is_ecall_id    = is_system_id && (funct3 == 3'b000) && (csr_addr_id == 12'h000);
  assign is_ebreak_id   = is_system_id && (funct3 == 3'b000) && (csr_addr_id == 12'h001);
  assign is_mret_id     = is_system_id && (funct3 == 3'b000) && (csr_addr_id == 12'h302);
  assign csr_wen_pre_id = is_csr_id && ((csr_op_id == 2'b01) || (rs1_addr != 5'b0));

  // M-extension: R-type opcode with funct7 = 0000001
  assign is_muldiv_id = (opcode == 7'b0110011) && (if_id_instr[31:25] == 7'b0000001);

  // WFI: SYSTEM, funct3=000, funct12=0x105
  assign is_wfi_id = is_system_id && (funct3 == 3'b000) && (csr_addr_id == 12'h105);

  
  // Pre-registered forwarding selects (Optimization 3)
  //
  // In cycle N (instr I in ID), the values currently in id_ex_rd_addr
  // and ex_mem_rd_addr will be in ex_mem_rd_addr and mem_wb_rd_addr
  // respectively when I reaches EX in cycle N+1. So comparing I's
  // rs1/rs2 against those now is equivalent to the EX-stage compare
  // that was formerly on the critical path.
  //
  // Encoding: 2'b01 = forward from EX/MEM, 2'b10 = forward from MEM/WB,
  //           2'b00 = use id_ex_rs*_data (no forward).
  
  always_comb begin
    // Default: no forwarding
    pre_fwd_a_sel = 2'b00;

    // Forward from EX/MEM (what id_ex_rd_addr becomes next cycle).
    // WB_SRC_MEM case is blocked by hazard unit, so we don't need to exclude it here.
    if (id_ex_rf_wr_en && (id_ex_rd_addr != 5'b0) && (id_ex_rd_addr == rs1_addr))
      pre_fwd_a_sel = 2'b01;
    else if (ex_mem_rf_wr_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == rs1_addr))
      pre_fwd_a_sel = 2'b10;
  end

  always_comb begin
    pre_fwd_b_sel = 2'b00;

    if (id_ex_rf_wr_en && (id_ex_rd_addr != 5'b0) && (id_ex_rd_addr == rs2_addr))
      pre_fwd_b_sel = 2'b01;
    else if (ex_mem_rf_wr_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == rs2_addr))
      pre_fwd_b_sel = 2'b10;
  end

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
      id_ex_rd_addr            <= 5'b0;
      id_ex_fwd_a_sel          <= 2'b00;
      id_ex_fwd_b_sel          <= 2'b00;
      id_ex_btb_predict_taken  <= 1'b0;
      id_ex_btb_predict_target <= '0;
      id_ex_valid              <= 1'b0;
      id_ex_is_muldiv          <= 1'b0;
      id_ex_is_wfi             <= 1'b0;
      id_ex_is_csr             <= 1'b0;
      id_ex_csr_addr           <= 12'b0;
      id_ex_csr_op             <= 2'b0;
      id_ex_csr_use_imm        <= 1'b0;
      id_ex_csr_zimm           <= 5'b0;
      id_ex_csr_wen            <= 1'b0;
      id_ex_is_mret            <= 1'b0;
      id_ex_is_ecall           <= 1'b0;
      id_ex_is_ebreak          <= 1'b0;
    end

    // Bubble on:
    //  - Registered redirect (flush the speculative instr that was
    //    in ID when the branch resolved in EX)
    //  - Load-use hazard (not during cache/MMIO stall — pipe is frozen)
    else if (redir_valid_r || (hazard_stall && !icache_stall && !dcache_stall && !mmio_stall && !muldiv_stall && !wfi_stall)) begin
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
      id_ex_rd_addr            <= 5'b0;
      id_ex_fwd_a_sel          <= 2'b00;
      id_ex_fwd_b_sel          <= 2'b00;
      id_ex_btb_predict_taken  <= 1'b0;
      id_ex_btb_predict_target <= '0;
      // Bubble is not a real instruction: it must never trigger a trap,
      // CSR write, or mret in EX.
      id_ex_valid              <= 1'b0;
      id_ex_is_muldiv          <= 1'b0;
      id_ex_is_wfi             <= 1'b0;
      id_ex_is_csr             <= 1'b0;
      id_ex_csr_addr           <= 12'b0;
      id_ex_csr_op             <= 2'b0;
      id_ex_csr_use_imm        <= 1'b0;
      id_ex_csr_zimm           <= 5'b0;
      id_ex_csr_wen            <= 1'b0;
      id_ex_is_mret            <= 1'b0;
      id_ex_is_ecall           <= 1'b0;
      id_ex_is_ebreak          <= 1'b0;
    end

    // Normal latch: only when no stall source active
    else if (!pipeline_stall) begin
      // CSR instructions write rd with the old CSR value, so force the
      // RF write enable on (control unit leaves SYSTEM as a NOP).
      id_ex_rf_wr_en           <= rf_wr_en | is_csr_id;
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
      id_ex_rd_addr            <= rd_addr;
      id_ex_fwd_a_sel          <= pre_fwd_a_sel;
      id_ex_fwd_b_sel          <= pre_fwd_b_sel;
      id_ex_btb_predict_taken  <= if_id_btb_predict_taken;
      id_ex_btb_predict_target <= if_id_btb_predict_target;
      id_ex_valid              <= 1'b1;
      id_ex_is_muldiv          <= is_muldiv_id;
      id_ex_is_wfi             <= is_wfi_id;
      id_ex_is_csr             <= is_csr_id;
      id_ex_csr_addr           <= csr_addr_id;
      id_ex_csr_op             <= csr_op_id;
      id_ex_csr_use_imm        <= csr_use_imm_id;
      id_ex_csr_zimm           <= rs1_addr;          // zimm field = rs1 bits
      id_ex_csr_wen            <= csr_wen_pre_id;
      id_ex_is_mret            <= is_mret_id;
      id_ex_is_ecall           <= is_ecall_id;
      id_ex_is_ebreak          <= is_ebreak_id;
    end
    // else: cache stall — hold all ID/EX registers
  end


  //  EX Stage
  
  // Forwarding muxes — NO comparators on the critical path anymore.
  // id_ex_fwd_{a,b}_sel were computed in ID and registered.
  

  // EX/MEM forwarding value: select the correct writeback data type
  always_comb begin
    case (ex_mem_rf_wr_data_sel)
      WB_SRC_ALU: ex_mem_fwd_data = ex_mem_alu_res;
      WB_SRC_IMM: ex_mem_fwd_data = ex_mem_immediate;
      WB_SRC_PC:  ex_mem_fwd_data = ex_mem_pc_plus_4;
      default:    ex_mem_fwd_data = ex_mem_alu_res;
    endcase
  end

  always_comb begin
    case (id_ex_fwd_a_sel)
      2'b01:   fwd_rs1_data = ex_mem_fwd_data;   // Forward from EX/MEM
      2'b10:   fwd_rs1_data = wr_data;           // Forward from MEM/WB
      default: fwd_rs1_data = id_ex_rs1_data;    // No forwarding
    endcase
  end

  always_comb begin
    case (id_ex_fwd_b_sel)
      2'b01:   fwd_rs2_data = ex_mem_fwd_data;
      2'b10:   fwd_rs2_data = wr_data;
      default: fwd_rs2_data = id_ex_rs2_data;
    endcase
  end

  // ALU operand muxes (use forwarded data)
  assign alu_a = id_ex_op1_sel ? id_ex_pc        : fwd_rs1_data;
  assign alu_b = id_ex_op2_sel ? id_ex_immediate : fwd_rs2_data;

  // ALU
  alu u_alu (
    .alu_a   (alu_a),
    .alu_b   (alu_b),
    .alu_op  (id_ex_alu_op),
    .alu_res (alu_res)
  );

  // Branch Control (resolved in EX using forwarded operands)
  branch_control u_branch_control (
    .opr_a        (fwd_rs1_data),
    .opr_b        (fwd_rs2_data),
    .is_b_type    (id_ex_b_type),
    .funct3       (id_ex_funct3),
    .branch_taken (ex_branch_taken)
  );

  
  // Branch Target Adder (Optimization 2) — B-type / JAL
  //
  // target = pc + imm. Both inputs are registered in ID/EX, so this
  // adder runs in parallel with the ALU carry chain.
  //
  // JALR Target Adder (Optimization 5) — JALR
  //
  // target = rs1 + imm. Uses the already-forwarded rs1 value, and
  // sidesteps the ALU's 10-way output case-mux. JALR is identified
  // by pc_sel=1 AND op1_sel=0 (distinguishes it from JAL which has
  // op1_sel=1). After Opts 2 & 5, no branch/jump target goes
  // through alu_res — the ALU is no longer on the EX→IF path.
  
  assign bta_target  = id_ex_pc       + id_ex_immediate;
  assign jalr_target = fwd_rs1_data   + id_ex_immediate;

  assign actual_taken      = ex_branch_taken | id_ex_pc_sel;
  assign actual_target_addr = (id_ex_pc_sel & ~id_ex_op1_sel)
                              ? {jalr_target[XLEN-1:1], 1'b0}  // JALR: dedicated adder
                              : {bta_target[XLEN-1:1],  1'b0}; // B-type / JAL: BTA

  // BTB update enable: fire for any branch or jump instruction in EX
  assign btb_update_en = id_ex_b_type | id_ex_pc_sel;

  assign btb_mispredict = (id_ex_btb_predict_taken != actual_taken)
                          || (actual_taken && (id_ex_btb_predict_target != actual_target_addr));

  
  // Zicsr / Trap logic (EX stage)
  //
  // CSR read is combinational from csr_file; the old value becomes the
  // rd writeback (routed through the alu_res slot below). The new CSR
  // value is computed per the CSR op and committed when the instruction
  // retires. Every CSR instruction also forces a redirect to PC+4
  // (csr_serialize) so it fully serializes - the simplest way to make
  // CSR side effects globally visible before any dependent instruction.
  //
  // Traps reuse the existing redirect path:
  //   - interrupt / ECALL / EBREAK -> redirect to mtvec, squash the
  //     trapped instruction (trap_flush_ex) and everything younger,
  //     and save its PC in mepc (precise: it re-executes after mret).
  //   - mret -> redirect to mepc and restore mstatus.MIE.
  // Priority: trap > mret > csr_serialize > branch.
  
  assign csr_src = id_ex_csr_use_imm ? {27'b0, id_ex_csr_zimm} : fwd_rs1_data;

  always_comb begin
    unique case (id_ex_csr_op)
      2'b01:   csr_wdata_ex = csr_src;                // CSRRW(I): write src
      2'b10:   csr_wdata_ex = csr_rdata |  csr_src;   // CSRRS(I): set bits
      2'b11:   csr_wdata_ex = csr_rdata & ~csr_src;   // CSRRC(I): clear bits
      default: csr_wdata_ex = csr_src;
    endcase
  end

  
  // M-extension functional unit (EX). It freezes the pipeline via
  // muldiv_stall while iterating, so its result simply lands in the
  // EX/MEM ALU-result slot on the completion cycle and forwards
  // normally. ex_can_advance lets it retire only when no other stall
  // is holding the op in EX.
  
  assign muldiv_req     = id_ex_is_muldiv & id_ex_valid & ~redir_valid_r;
  assign ex_can_advance = ~icache_stall & ~dcache_stall & ~mmio_stall;
  assign muldiv_stall   = muldiv_req & ~muldiv_done;

  // Clock-gate the M-unit: its clock runs only while an M-op is in
  // flight (idle the rest of the time, which is almost always).
  clock_gate u_muldiv_cg (
    .clk       (clk),
    .enable    (muldiv_req),
    .test_en   (1'b0),
    .gated_clk (muldiv_gclk)
  );

  muldiv_unit #(.MUL_LATENCY(2)) u_muldiv (
    .clk     (muldiv_gclk),
    .reset_n (reset_n),
    .req     (muldiv_req),
    .advance (ex_can_advance),
    .funct3  (id_ex_funct3),
    .op_a    (fwd_rs1_data),
    .op_b    (fwd_rs2_data),
    .result  (muldiv_result),
    .busy    (muldiv_busy),
    .done    (muldiv_done)
  );

  // EX instruction is real, not being flushed by an in-flight redirect,
  // and not frozen by a stall (incl. an in-flight multiply/divide or WFI).
  assign ex_commit = id_ex_valid & ~redir_valid_r
                     & ~icache_stall & ~dcache_stall & ~mmio_stall & ~muldiv_stall & ~wfi_stall;

  
  // WFI low-power park. When a WFI retires with no interrupt pending,
  // the core sleeps (freezes the pipeline, which clock-gates downstream
  // and is the BUFGCE enable for the whole core in the FPGA flow) until
  // a locally-enabled interrupt arrives. irq_wake ignores mstatus.MIE so
  // a WFI with global interrupts off still wakes and cannot deadlock.
  
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)
      core_asleep <= 1'b0;
    else if (core_asleep)
      core_asleep <= ~irq_wake_w;                 // wake on pending interrupt
    else if (ex_commit & id_ex_is_wfi & ~irq_wake_w)
      core_asleep <= 1'b1;                         // enter sleep on WFI
  end

  assign wfi_stall = core_asleep;

  assign take_interrupt = ex_commit & irq_req_w;
  assign take_exception = ex_commit & (id_ex_is_ecall | id_ex_is_ebreak);
  assign trap_redirect  = take_interrupt | take_exception;
  assign mret_redirect  = ex_commit & id_ex_is_mret;
  assign csr_serialize  = ex_commit & id_ex_is_csr;
  assign trap_flush_ex  = trap_redirect;   // squash the trapped instruction itself

  assign trap_cause_ex  = take_interrupt ? irq_cause_w :
                          id_ex_is_ecall ? 32'd11 :    // ECALL from M-mode
                                           32'd3;      // EBREAK

  // Commit the CSR write unless the instruction is being trapped/flushed
  assign csr_commit_wen = ex_commit & id_ex_csr_wen & ~trap_flush_ex;

  csr_file u_csr (
    .clk         (clk),
    .reset_n     (reset_n),
    .csr_raddr   (id_ex_csr_addr),
    .csr_rdata   (csr_rdata),
    .csr_wen     (csr_commit_wen),
    .csr_waddr   (id_ex_csr_addr),
    .csr_wdata   (csr_wdata_ex),
    .trap_take   (trap_redirect),
    .trap_cause  (trap_cause_ex),
    .trap_epc    (id_ex_pc),
    .mret        (mret_redirect),
    .irq_timer   (irq_timer),
    .irq_ecc     (irq_ecc),
    .mtvec_o     (mtvec_w),
    .mepc_o      (mepc_w),
    .irq_req     (irq_req_w),
    .irq_cause_o (irq_cause_w),
    .irq_wake    (irq_wake_w)
  );

  // Combined redirect: branch mispredict OR any CSR/trap event
  assign ex_redirect = btb_mispredict | trap_redirect | mret_redirect | csr_serialize;

  assign ex_redirect_target = trap_redirect ? mtvec_w :
                              mret_redirect ? mepc_w  :
                              csr_serialize ? (id_ex_pc + 32'h4) :
                              (actual_taken ? actual_target_addr : ex_pc_plus_4);

  // PC+4 for JAL/JALR link register writeback (and not-taken correction target)
  assign ex_pc_plus_4 = id_ex_pc + 32'h4;

  
  // EX/MEM Pipeline Register
  //
  // Squash the speculative instruction that was in EX when the
  // registered redirect fires. Its control signals were set based
  // on the pre-redirect pipeline state, so we must mask its side-
  // effects (RF write, DMEM req/wr) here. The data itself doesn't
  // matter because nothing downstream commits it.
  
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

    else if (!icache_stall && !dcache_stall && !mmio_stall && !muldiv_stall && !wfi_stall) begin
      // Squash side-effects when a registered redirect is in flight
      // (younger flush) or when this very instruction is trapped now.
      ex_mem_rf_wr_en         <= id_ex_rf_wr_en   & ~redir_valid_r & ~trap_flush_ex;
      ex_mem_dmem_req         <= id_ex_dmem_req   & ~redir_valid_r & ~trap_flush_ex;
      ex_mem_dmem_wr_en       <= id_ex_dmem_wr_en & ~redir_valid_r & ~trap_flush_ex;
      ex_mem_rf_wr_data_sel   <= id_ex_rf_wr_data_sel;
      ex_mem_dmem_size        <= id_ex_dmem_size;
      ex_mem_dmem_zero_extend <= id_ex_dmem_zero_extend;
      // Writeback value: M-extension result, else CSR old value, else
      // the ALU result.
      ex_mem_alu_res          <= id_ex_is_muldiv ? muldiv_result :
                                 id_ex_is_csr    ? csr_rdata     : alu_res;
      ex_mem_rs2_data         <= fwd_rs2_data;  // Forwarded store data
      ex_mem_rd_addr          <= id_ex_rd_addr;
      ex_mem_pc_plus_4        <= ex_pc_plus_4;
      ex_mem_immediate        <= id_ex_immediate;
    end
    // else: cache stall — hold
  end


  //  MEM Stage
  
  // MMIO decode (SoC integration): a data access leaves the cacheable
  // RAM window when it targets the peripheral space (>= 0x4000_0000,
  // i.e. addr[31] | addr[30]) or the boot-ROM data window (< 0x2000).
  // Such accesses are steered to the native MMIO port; the D-cache
  // never sees them, so device registers are never cached and stores
  // reach the peripheral before the pipeline resumes.
  
  assign mmio_sel   = ex_mem_dmem_req &
                      ((|ex_mem_alu_res[31:30]) | (ex_mem_alu_res < 32'h0000_2000));
  assign dc_cpu_req = ex_mem_dmem_req & ~mmio_sel;

  // MMIO port: fields come straight from EX/MEM registers, which are
  // frozen while mmio_stall holds the pipeline — stable by design.
  assign mmio_req         = mmio_sel;
  assign mmio_wr_en       = ex_mem_dmem_wr_en;
  assign mmio_addr        = ex_mem_alu_res;
  assign mmio_wr_data     = ex_mem_rs2_data;
  assign mmio_size        = ex_mem_dmem_size;
  assign mmio_zero_extend = ex_mem_dmem_zero_extend;

  assign mmio_stall  = mmio_sel & ~mmio_ready;
  assign mmio_accept = ~icache_stall & ~dcache_stall;

  // L1 Data Cache — direct-mapped, BRAM-based, write-back policy
  l1_dcache u_l1_dcache (
    .clk              (clk),
    .reset_n          (reset_n),
    .cpu_req          (dc_cpu_req),
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

  // D-Cache stall: only for cacheable requests (MMIO is gated off)
  assign dcache_stall = dc_cpu_req & ~dcache_ready;

  
  // Fetch router (SoC integration): I-cache line fills below 0x2000
  // are served by the external boot ROM, everything else by main
  // memory. The fill address is held stable for the whole miss
  // (l1_icache drives it from latched miss_tag/miss_index), so the
  // combinational route cannot change mid-burst.
  
  logic        ic_rom_sel;
  logic [31:0] mm_ic_data;
  logic        mm_ic_data_valid;
  logic        mm_ic_ready;

  assign ic_rom_sel     = (ic_mem_addr < 32'h0000_2000);
  assign rom_fetch_req  = ic_mem_req & ic_rom_sel;
  assign rom_fetch_addr = ic_mem_addr;

  assign ic_mem_data       = ic_rom_sel ? rom_fetch_data       : mm_ic_data;
  assign ic_mem_data_valid = ic_rom_sel ? rom_fetch_data_valid : mm_ic_data_valid;
  assign ic_mem_ready      = ic_rom_sel ? rom_fetch_ready      : mm_ic_ready;

  // Main Memory Controller — unified backing store with arbiter
  main_memory #(
    .MEM_INIT_FILE (MEM_INIT_FILE)
  ) u_main_memory (
    .clk                (clk),
    .reset_n            (reset_n),
    .icache_req         (ic_mem_req & ~ic_rom_sel),
    .icache_addr        (ic_mem_addr),
    .icache_data        (mm_ic_data),
    .icache_data_valid  (mm_ic_data_valid),
    .icache_ready       (mm_ic_ready),
    .dcache_req         (dc_mem_req),
    .dcache_wr_en       (dc_mem_wr_en),
    .dcache_addr        (dc_mem_addr),
    .dcache_wr_data     (dc_mem_wr_data),
    .dcache_data        (dc_mem_data),
    .dcache_data_valid  (dc_mem_data_valid),
    .dcache_ready       (dc_mem_ready)
  );

  // MEM/WB Pipeline Register — freezes under the SAME condition as
  // EX/MEM. (Bug fix: the original gate was !dcache_stall only, so
  // during an I-cache stall MEM/WB kept re-latching the frozen EX/MEM
  // content. That overwrote the WB-stage value one cycle into the
  // stall and corrupted any instruction sitting in EX that forwarded
  // its operand from MEM/WB — a latent bug independent of the SoC
  // integration. Uniform freeze restores the stage alignment the
  // pre-computed forwarding selects rely on.)
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

    else if (!icache_stall && !dcache_stall && !mmio_stall && !muldiv_stall && !wfi_stall) begin
      mem_wb_rf_wr_en       <= ex_mem_rf_wr_en;
      mem_wb_rf_wr_data_sel <= ex_mem_rf_wr_data_sel;
      mem_wb_alu_res        <= ex_mem_alu_res;
      // Load data source: MMIO loads return through the AXI bridge,
      // cacheable loads through the D-cache
      mem_wb_dmem_rd_data   <= mmio_sel ? mmio_rd_data : dmem_rd_data;
      mem_wb_rd_addr        <= ex_mem_rd_addr;
      mem_wb_pc_plus_4      <= ex_mem_pc_plus_4;
      mem_wb_immediate      <= ex_mem_immediate;
    end
    // else: D-cache / MMIO stall — hold
  end

  // Interrupt lines irq_timer / irq_ecc are consumed by u_csr (EX
  // stage) as mip.MTIP / mip.MEIP and vector the core through mtvec.

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
