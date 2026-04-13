// Register File - RTL
import risc_pkg::*;

module register_file(
  input  logic             clk,
  input  logic             reset_n,
  input  logic [4:0]       rs1_addr,
  input  logic [4:0]       rs2_addr,
  input  logic [4:0]       rd_addr,
  input  logic             rf_wr_en,
  input  logic [XLEN-1:0]  wr_data,

  output logic [XLEN-1:0]  rs1_data,
  output logic [XLEN-1:0]  rs2_data
);

  // Register file: 32 x XLEN-bit registers
  logic [XLEN-1:0] regs [0:31];

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      for (int i = 0; i < 32; i++) begin
        regs[i] <= '0;
      end
    end 
    
    else if (rf_wr_en && (rd_addr != 5'b0)) begin
      // Write to register file (x0 is hardwired to 0, prevent overwriting)
      regs[rd_addr] <= wr_data;
    end
  end

  // Read logic (combinational) + Enforce x0 = 0 directly 
  assign rs1_data = (rs1_addr == 5'b0) ? '0 : regs[rs1_addr];
  assign rs2_data = (rs2_addr == 5'b0) ? '0 : regs[rs2_addr];

endmodule