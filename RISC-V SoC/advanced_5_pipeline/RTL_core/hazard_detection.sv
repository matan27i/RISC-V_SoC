// Hazard Detection Unit - RTL

import risc_pkg::*;

module hazard_detection (
  // Decode stage source register addresses (from decode in ID)
  input  logic [4:0] rs1_addr,
  input  logic [4:0] rs2_addr,

  // EX stage load detection (from ID/EX pipeline register)
  input  wb_src_t    id_ex_rf_wr_data_sel,
  input  logic [4:0] id_ex_rd_addr,

  output logic       stall
);
  assign stall = (id_ex_rf_wr_data_sel == WB_SRC_MEM) && (id_ex_rd_addr != 5'b0) &&
                 ((id_ex_rd_addr == rs1_addr) || (id_ex_rd_addr == rs2_addr));

endmodule
