// Instruction Memory - RTL (Parameterized)
import risc_pkg::*;

module instruction_memory #(parameter ADDR_WIDTH = 7,parameter DATA_WIDTH = 8)
(
  input  logic             imem_req,              // Read enable
  input  logic [XLEN-1:0]  imem_addr,             // Address bus scales with architecture width
  output logic [31:0]      imem_data              // Output instruction is ALWAYS 32-bit in RISC-V
);


  // Memory Declaration (ROM)
  logic [DATA_WIDTH-1:0] mem [0:(2**ADDR_WIDTH)-1];

  initial begin
	$readmemh("machine_code.mem", mem);
  end
  
  // Extract the relevant address bits to match the memory depth
  logic [ADDR_WIDTH-1:0] internal_addr;
  assign internal_addr = imem_addr[ADDR_WIDTH-1:0];

  // Read logic (Combinational)
  always_comb begin
      if (imem_req)
          // mem[internal_addr] is the LSB, mem[internal_addr+3] is the MSB.
          imem_data = {mem[internal_addr+3], mem[internal_addr+2], mem[internal_addr+1], 
          mem[internal_addr]};
          
      else 
          imem_data = 32'b0;
  end

endmodule