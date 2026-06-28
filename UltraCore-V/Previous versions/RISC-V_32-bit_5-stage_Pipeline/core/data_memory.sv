// Data Memory - RTL
import risc_pkg::*;

module data_memory #(
  parameter ADDR_WIDTH = 6,
  parameter DATA_WIDTH = 8
)(
  input  logic              clk,
  input  logic              dmem_req,
  input  logic              dmem_wr_en,
  input  mem_size_t         dmem_data_size,
  input  logic [31:0]       dmem_addr,
  input  logic [31:0]       dmem_wr_data,
  input  logic              dmem_zero_extend,
  output logic [31:0]       dmem_rd_data
);

  logic [DATA_WIDTH-1:0] mem [0:(2**ADDR_WIDTH)-1];
  logic [ADDR_WIDTH-1:0] addr;
  assign addr = dmem_addr[ADDR_WIDTH-1:0];

  // Write
  always_ff @(posedge clk) begin
    if (dmem_req && dmem_wr_en) begin
      case (dmem_data_size)
        BYTE: begin
          mem[addr] <= dmem_wr_data[7:0];
        end
        HALF_WORD: begin
          mem[addr]   <= dmem_wr_data[7:0];
          mem[addr+1] <= dmem_wr_data[15:8];
        end
        WORD: begin
          mem[addr]   <= dmem_wr_data[7:0];
          mem[addr+1] <= dmem_wr_data[15:8];
          mem[addr+2] <= dmem_wr_data[23:16];
          mem[addr+3] <= dmem_wr_data[31:24];
        end
        default: ;
      endcase
    end
  end

  // Read
  always_comb begin
    dmem_rd_data = 32'd0;
    if (dmem_req && !dmem_wr_en) begin
      case (dmem_data_size)
        BYTE: begin
          dmem_rd_data = dmem_zero_extend
                       ? {24'b0, mem[addr]}
                       : {{24{mem[addr][7]}}, mem[addr]};
        end
        HALF_WORD: begin
          dmem_rd_data = dmem_zero_extend
                       ? {16'b0, mem[addr+1], mem[addr]}
                       : {{16{mem[addr+1][7]}}, mem[addr+1], mem[addr]};
        end
        WORD: begin
          dmem_rd_data = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
        end
        default: dmem_rd_data = 32'd0;
      endcase
    end
  end

endmodule
