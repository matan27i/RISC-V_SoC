module true_dual_port_64Byte_ram(
    output reg [7:0] q_a, q_b,
    input [7:0] data_a, data_b,
    input [5:0] addr_a, addr_b,
    input we_a, we_b, clk
);

    reg [7:0] ram[63:0];

    // port_a
    always @(posedge clk) begin
        if (we_a) begin
            ram[addr_a] <= data_a;
        end
        // Non-blocking assignment outside the 'if' ensures 
        // read-before-write behavior (outputs old data)
        q_a <= ram[addr_a];         
    end

    // port_b
    always @(posedge clk) begin
        if (we_b) begin
            ram[addr_b] <= data_b;
        end
        // Non-blocking assignment outside the 'if' ensures 
        // read-before-write behavior (outputs old data)
        q_b <= ram[addr_b];         
    end

endmodule