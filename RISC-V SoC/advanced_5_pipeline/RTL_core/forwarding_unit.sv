// Forwarding Unit - RTL
import risc_pkg::*;

module forwarding_unit (
  // EX stage source register addresses
  input  logic [4:0] id_ex_rs1_addr,
  input  logic [4:0] id_ex_rs2_addr,

  // EX/MEM stage forwarding source
  input  logic       ex_mem_rf_wr_en,
  input  logic [4:0] ex_mem_rd_addr,

  // MEM/WB stage forwarding source
  input  logic       mem_wb_rf_wr_en,
  input  logic [4:0] mem_wb_rd_addr,

  // Forwarding mux selects
  output logic [1:0] fwd_a_sel,
  output logic [1:0] fwd_b_sel
);

  // Operand A (rs1) forwarding
  always_comb begin
    if (ex_mem_rf_wr_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs1_addr))
      fwd_a_sel = 2'b01;  // Forward from EX/MEM (most recent)

    else if (mem_wb_rf_wr_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_ex_rs1_addr))
      fwd_a_sel = 2'b10;  // Forward from MEM/WB

    else
      fwd_a_sel = 2'b00;  // No forwarding
  end

  // Operand B (rs2) forwarding
  always_comb begin
    if (ex_mem_rf_wr_en && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs2_addr))
      fwd_b_sel = 2'b01;  // Forward from EX/MEM (most recent)

    else if (mem_wb_rf_wr_en && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_ex_rs2_addr))
      fwd_b_sel = 2'b10;  // Forward from MEM/WB

    else
      fwd_b_sel = 2'b00;  // No forwarding
  end

endmodule
