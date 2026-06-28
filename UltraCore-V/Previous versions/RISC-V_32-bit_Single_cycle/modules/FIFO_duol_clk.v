module FIFO_duol_clk (
    input clk,           // Combined clock for synchronous design
    input rst,           // Asynchronous reset
    input [7:0] buf_in,  // Data input
    input wr_en,         // Write enable
    input rd_en,         // Read enable
    output reg [7:0] buf_out,    // Data output
    output buf_empty,    // FIFO empty flag
    output buf_full,     // FIFO full flag
    output reg [6:0] fifo_counter // Counter to track occupancy
);

    // Internal memory and pointers
    reg [7:0] buf_mem [63:0]; 
    reg [5:0] rd_ptr, wr_ptr;

    // Status flags logic using continuous assignment
    assign buf_empty = (fifo_counter == 0);
    assign buf_full  = (fifo_counter == 64);

    // Main Control Block (Sequential Logic)
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fifo_counter <= 0;
            rd_ptr       <= 0;
            wr_ptr       <= 0;
            buf_out      <= 0;
        end else begin
            
            // Write Operation
            if (wr_en && !buf_full) begin
                buf_mem[wr_ptr] <= buf_in;
                wr_ptr <= wr_ptr + 1;
            end

            // Read Operation
            if (rd_en && !buf_empty) begin
                buf_out <= buf_mem[rd_ptr];
                rd_ptr <= rd_ptr + 1;
            end

            // Occupancy Counter Management
            // Increments on Write, decrements on Read, stays same if both or neither
            case ({wr_en && !buf_full, rd_en && !buf_empty})
                2'b10: fifo_counter <= fifo_counter + 1;
                2'b01: fifo_counter <= fifo_counter - 1;
                default: fifo_counter <= fifo_counter;
            endcase
            
        end
    end

endmodule