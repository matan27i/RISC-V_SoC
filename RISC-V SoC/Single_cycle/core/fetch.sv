// Fetch - RTL 
import risc_pkg::*;

module fetch (
  input  logic             clk,
  input  logic             reset_n,
  input  logic [31:0]      imem_data,      // RISC-V instructions are always 32-bit
  input  logic [XLEN-1:0]  pc,             // PC 

  output logic             imem_req,       // Instruction memory request
  output logic [XLEN-1:0]  imem_addr,      // Address scales with architecture size
  output logic [31:0]      instruction     // Decoded instruction is always 32-bit
);
  
   // Route the current PC directly to the instruction memory address.
   assign imem_addr = pc;
   
   // Pass the fetched data directly to the decode stage.
   assign instruction = imem_data;
   
   // Request an instruction from memory as long as the system is active (not in reset).
   assign imem_req = reset_n;

endmodule